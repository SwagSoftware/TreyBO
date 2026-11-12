#pragma once

#include "tl/physics/include/constraint_solver/pulse_sum_constraint_solver.h"

// Start-of-step relative point velocity (b1 - b2) from the rigid bodies.
inline const phys_vec3 pulse_sum_point::get_vel()
{
	const phys_vec3 b2_vel = m_b2 ? rbint::gtv(m_b2->m_rb, m_b2_r) : m_b2_apx;
	const phys_vec3 b1_vel = rbint::gtv(m_b1->m_rb, m_b1_r);
	return b1_vel - b2_vel;
}

// Joint separation (b1 point - b2 point).
inline const phys_vec3 pulse_sum_point::get_pos()
{
	const phys_vec3 b2_pt = m_b2 ? m_b2->m_rb->m_mat.GetW() + m_b2_r : m_b2_r;
	const phys_vec3 b1_pt = m_b1->m_rb->m_mat.GetW() + m_b1_r;
	return b1_pt - b2_pt;
}

// b1 must have a solver node (no swap branch, unlike pulse_sum_normal::set).
inline void pulse_sum_point::set(rigid_body * const b1, const phys_vec3 & b1_r, rigid_body * const b2, const phys_vec3 & b2_r,
	pulse_sum_cache * const ps_cache, const float delta_t, const bool is_spring,
	const float spring_k, const float damp_k)
{
	PHYS_ASSERT(ps_cache);
	PHYS_ASSERT(b1);
	PHYS_ASSERT(b2);
	PHYS_ASSERT(b1 != b2);
	PHYS_ASSERT_STR(rbint_get_pulse_sum_node(b1) || rbint_get_pulse_sum_node(b2), "rbint::get_pulse_sum_node(b1) || rbint::get_pulse_sum_node(b2)");
	PHYS_ASSERT_STR(rbint_verify_pulse_sum_node(b1) && rbint_verify_pulse_sum_node(b2), "rbint::verify_pulse_sum_node(b1) && rbint::verify_pulse_sum_node(b2)");
	PHYS_ASSERT_STR(rbint_get_pulse_sum_node(b1) != NULL, "rbint::get_pulse_sum_node(b1) != NULL");

	m_b1 = b1->m_node;
	m_b1_r = b1_r;
	if (b2->m_node)
	{
		m_b2 = b2->m_node;
		m_b2_r = b2_r;
	}
	else
	{
		m_b2 = NULL;
		set_object_vel(rbint::gtv(b2, b2_r));
		set_object_col_pt(b2_r + b2->m_mat.GetW());
	}
	m_pulse_sum_cache = ps_cache;
	calc_abs_and_setup(delta_t, is_spring, spring_k, damp_k);
}

// Relative velocity change accumulated in the solver nodes this sub-step.
inline const phys_vec3 pulse_sum_point::get_objective()
{
	phys_vec3 result = m_b1->t_vel + phys_cross(m_b1->a_vel, m_b1_r);
	if (m_b2)
		result = result - (m_b2->t_vel + phys_cross(m_b2->a_vel, m_b2_r));
	return result;
}

// Apply the impulse vector s_: +s_ to b1, -s_ to b2.
inline void pulse_sum_point::apply(const phys_vec3 & s_)
{
	m_b1->t_vel += s_ * m_b1->m_inv_mass;
	m_b1->a_vel += m_b1_apx * s_.x + m_b1_apy * s_.y + m_b1_apz * s_.z;
	if (m_b2)
	{
		m_b2->t_vel -= s_ * m_b2->m_inv_mass;
		m_b2->a_vel -= m_b2_apx * s_.x + m_b2_apy * s_.y + m_b2_apz * s_.z;
	}
}

// Build K = sum over bodies of (inv_mass * I - [r]x I^-1 [r]x), column by column, plus the soft
// spring terms, then invert it by cofactors.
inline void pulse_sum_point::calc_abs_and_setup(const float delta_t, const bool is_spring, const float spring_k, const float damp_k)
{
	const float b1_inv_mass = m_b1->m_inv_mass;
	m_b1_apx = phys_multiply(m_b1->m_world_inv_inertia, phys_cross_i(m_b1_r));
	phys_vec3 kx = phys_cross(m_b1_apx, m_b1_r);
	kx.x = kx.x + b1_inv_mass;
	m_b1_apy = phys_multiply(m_b1->m_world_inv_inertia, phys_cross_j(m_b1_r));
	phys_vec3 ky = phys_cross(m_b1_apy, m_b1_r);
	ky.y = ky.y + b1_inv_mass;
	m_b1_apz = phys_multiply(m_b1->m_world_inv_inertia, phys_cross_k(m_b1_r));
	phys_vec3 kz = phys_cross(m_b1_apz, m_b1_r);
	kz.z = kz.z + b1_inv_mass;

	if (m_b2)
	{
		const float b2_inv_mass = m_b2->m_inv_mass;
		m_b2_apx = phys_multiply(m_b2->m_world_inv_inertia, phys_cross_i(m_b2_r));
		kx = phys_cross(m_b2_apx, m_b2_r) + kx;
		kx.x = b2_inv_mass + kx.x;
		m_b2_apy = phys_multiply(m_b2->m_world_inv_inertia, phys_cross_j(m_b2_r));
		ky = phys_cross(m_b2_apy, m_b2_r) + ky;
		ky.y = ky.y + b2_inv_mass;
		m_b2_apz = phys_multiply(m_b2->m_world_inv_inertia, phys_cross_k(m_b2_r));
		kz = phys_cross(m_b2_apz, m_b2_r) + kz;
		kz.z = kz.z + b2_inv_mass;
	}

	if (is_spring)
	{
		// implicit spring-damper: cfm = 1 / (dt * damp_k + dt^2 * spring_k)
		const phys_vec3 pos = get_pos();
		const float k_dt2 = delta_t * spring_k * delta_t;
		m_cfm = 1.0f / (delta_t * damp_k + k_dt2);
		kx.x = m_cfm + kx.x;
		ky.y = ky.y + m_cfm;
		kz.z = kz.z + m_cfm;
		const float rate = -(k_dt2 * m_cfm) / delta_t;
		m_right_side = pos * rate;
		m_big_dirt = PHYS_ZERO_VEC;
	}
	else
	{
		// rigid joint: close half the gap per step in the vel+pos phase
		const float dt = delta_t > MIN_NERP_DELTA_T ? delta_t : MIN_NERP_DELTA_T;
		const phys_vec3 pos = get_pos();
		m_big_dirt = pos * (-0.5f / dt);
		m_right_side = PHYS_ZERO_VEC;
		m_cfm = 0.0f;
	}

	m_cr23 = phys_cross(ky, kz);
	m_cr31 = phys_cross(kz, kx);
	m_cr12 = phys_cross(kx, ky);
	m_denom = phys_vec3(kx.x, ky.y, kz.z);
	const float oo_det = 1.0f / phys_dot(kz, m_cr12);
	m_cr23 *= oo_det;
	m_cr31 *= oo_det;
	m_cr12 *= oo_det;
}

// Warm start from the cached impulse rates.
inline void pulse_sum_point::SOLVER_solver_prolog(const float delta_t)
{
	m_right_side -= get_vel();
	m_pulse_sum.x = m_pulse_sum_cache[0].m_pulse_sum * delta_t;
	m_pulse_sum.y = m_pulse_sum_cache[1].m_pulse_sum * delta_t;
	m_pulse_sum.z = m_pulse_sum_cache[2].m_pulse_sum * delta_t;
	apply(m_pulse_sum);
}

// Block Gauss-Seidel step on all three rows (no limits).
inline void pulse_sum_point::SOLVER_apply_relaxation(float & error_sq)
{
	const phys_vec3 m_objective = get_objective() + m_pulse_sum * m_cfm - m_right_side;
	const phys_vec3 m_last_pulse_sum = m_pulse_sum;
	const phys_vec3 delta = m_cr23 * m_objective.x + m_cr31 * m_objective.y + m_cr12 * m_objective.z;
	m_pulse_sum -= delta;
	const phys_vec3 s_ = m_pulse_sum - m_last_pulse_sum;
	apply(s_);
	const phys_vec3 error_ = phys_vec3(m_denom.x * s_.x, m_denom.y * s_.y, m_denom.z * s_.z);
	const float error_x_sq = error_.x * error_.x;
	const float error_y_sq = error_.y * error_.y;
	const float error_z_sq = error_.z * error_.z;
	if (error_sq < error_x_sq)
		error_sq = error_x_sq;
	if (error_sq < error_y_sq)
		error_sq = error_y_sq;
	if (error_sq < error_z_sq)
		error_sq = error_z_sq;
}

// Store the velocity-phase impulse rates and turn on the position correction.
inline void pulse_sum_point::SOLVER_solver_intermediate(const float delta_t)
{
	m_pulse_sum_cache[0].m_pulse_sum = m_pulse_sum.x / delta_t;
	m_pulse_sum_cache[1].m_pulse_sum = m_pulse_sum.y / delta_t;
	m_pulse_sum_cache[2].m_pulse_sum = m_pulse_sum.z / delta_t;
	m_right_side = m_big_dirt + m_right_side;
}
