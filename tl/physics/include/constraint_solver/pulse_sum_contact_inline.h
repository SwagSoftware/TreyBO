#pragma once

#include "tl/physics/include/constraint_solver/pulse_sum_constraint_solver.h"
#include "tl/physics/include/rbc_defs/rbc_def_contact.h"

// Start-of-step relative point velocity in (n, f1, f2), from the rigid bodies.
inline const phys_vec3 pulse_sum_contact_point::get_vel(pulse_sum_contact * psc)
{
	const phys_vec3 b2_vel = psc->m_b2 ? rbint::gtv(psc->m_b2->m_rb, m_b2_r) : m_b2_ap_n;
	const phys_vec3 vel = rbint::gtv(psc->m_b1->m_rb, m_b1_r) - b2_vel;
	return phys_vec3(phys_dot(psc->m_ud_n, vel), phys_dot(psc->m_ud_f1, vel), phys_dot(psc->m_ud_f2, vel));
}

// Penetration along the normal (positive = overlapping).
inline const float pulse_sum_contact_point::get_pos(pulse_sum_contact * psc)
{
	const phys_vec3 b2_pt = psc->m_b2 ? psc->m_b2->m_rb->m_mat.GetW() + m_b2_r : m_b2_r;
	const phys_vec3 b1_pt = psc->m_b1->m_rb->m_mat.GetW() + m_b1_r;
	return phys_dot(psc->m_ud_n, b1_pt - b2_pt);
}

// Approach speed along the normal: the larger of this frame's and last frame's.
inline const float pulse_sum_contact_point::get_impact_vel(pulse_sum_contact * psc)
{
	phys_vec3 b2_last_vel;
	if (psc->m_b2)
	{
		const rigid_body * const rb2 = psc->m_b2->m_rb;
		b2_last_vel = rb2->m_last_t_vel + phys_cross(rb2->m_last_a_vel, m_b2_r);
	}
	else
	{
		b2_last_vel = m_b2_ap_n;
	}
	const rigid_body * const rb1 = psc->m_b1->m_rb;
	const phys_vec3 last_vel = rb1->m_last_t_vel + phys_cross(rb1->m_last_a_vel, m_b1_r) - b2_last_vel;

	const phys_vec3 b2_vel = psc->m_b2 ? rbint::gtv(psc->m_b2->m_rb, m_b2_r) : m_b2_ap_n;
	const phys_vec3 vel = rbint::gtv(psc->m_b1->m_rb, m_b1_r) - b2_vel;

	const float impact_vel = phys_dot(psc->m_ud_n, vel);
	const float last_impact_vel = phys_dot(psc->m_ud_n, last_vel);
	// Keeps impact_vel unless it is ordered-less: a NaN keeps it.
	return !(impact_vel < last_impact_vel) ? impact_vel : last_impact_vel;
}

// K in the (n, f1, f2) frame: diagonal, the normal-friction
// couplings xy / xz, and the inverse of the friction block [yy yz; yz zz].
inline void pulse_sum_contact_point::calc_abs(pulse_sum_contact * psc)
{
	const phys_vec3 b1_r_x_n = phys_cross(m_b1_r, psc->m_ud_n);
	m_b1_ap_n = phys_multiply(psc->m_b1->m_world_inv_inertia, b1_r_x_n);
	m_denom_xx = phys_dot(m_b1_ap_n, b1_r_x_n) + psc->m_b1->m_inv_mass;
	const phys_vec3 b1_r_x_f1 = phys_cross(m_b1_r, psc->m_ud_f1);
	m_b1_ap_f1 = phys_multiply(psc->m_b1->m_world_inv_inertia, b1_r_x_f1);
	m_denom_yy = phys_dot(m_b1_ap_f1, b1_r_x_f1) + psc->m_b1->m_inv_mass;
	const phys_vec3 b1_r_x_f2 = phys_cross(m_b1_r, psc->m_ud_f2);
	m_b1_ap_f2 = phys_multiply(psc->m_b1->m_world_inv_inertia, b1_r_x_f2);
	m_denom_zz = phys_dot(m_b1_ap_f2, b1_r_x_f2) + psc->m_b1->m_inv_mass;
	float denom_yz = phys_dot(m_b1_ap_f2, b1_r_x_f1);
	m_denom_xy = phys_dot(m_b1_ap_f1, b1_r_x_n);
	m_denom_xz = phys_dot(b1_r_x_n, m_b1_ap_f2);

	if (psc->m_b2)
	{
		const phys_vec3 b2_r_x_n = phys_cross(m_b2_r, psc->m_ud_n);
		m_b2_ap_n = phys_multiply(psc->m_b2->m_world_inv_inertia, b2_r_x_n);
		m_denom_xx = phys_dot(m_b2_ap_n, b2_r_x_n) + psc->m_b2->m_inv_mass + m_denom_xx;
		const phys_vec3 b2_r_x_f1 = phys_cross(m_b2_r, psc->m_ud_f1);
		m_b2_ap_f1 = phys_multiply(psc->m_b2->m_world_inv_inertia, b2_r_x_f1);
		m_denom_yy = phys_dot(m_b2_ap_f1, b2_r_x_f1) + psc->m_b2->m_inv_mass + m_denom_yy;
		const phys_vec3 b2_r_x_f2 = phys_cross(m_b2_r, psc->m_ud_f2);
		m_b2_ap_f2 = phys_multiply(psc->m_b2->m_world_inv_inertia, b2_r_x_f2);
		m_denom_zz = phys_dot(m_b2_ap_f2, b2_r_x_f2) + psc->m_b2->m_inv_mass + m_denom_zz;
		denom_yz = phys_dot(m_b2_ap_f2, b2_r_x_f1) + denom_yz;
		m_denom_xy = phys_dot(m_b2_ap_f1, b2_r_x_n) + m_denom_xy;
		m_denom_xz = phys_dot(b2_r_x_n, m_b2_ap_f2) + m_denom_xz;
	}

	float det = m_denom_zz * m_denom_yy - denom_yz * denom_yz;
	PHYS_ASSERT(det > 0.0f);
	det = 1.0f / det;
	m_inv_yy = det * m_denom_zz;
	m_inv_yz = -denom_yz * det;
	m_inv_zz = det * m_denom_yy;
}

// Relative velocity change accumulated in the solver nodes, in (n, f1, f2).
inline const phys_vec3 pulse_sum_contact_point::get_objective(pulse_sum_contact * psc)
{
	phys_vec3 vel = psc->m_b1->t_vel + phys_cross(psc->m_b1->a_vel, m_b1_r);
	if (psc->m_b2)
		vel = vel - (psc->m_b2->t_vel + phys_cross(psc->m_b2->a_vel, m_b2_r));
	return phys_vec3(phys_dot(psc->m_ud_n, vel), phys_dot(psc->m_ud_f1, vel), phys_dot(psc->m_ud_f2, vel));
}

// Apply the (n, f1, f2) impulse s_: +s_ to b1, -s_ to b2.
inline void pulse_sum_contact_point::apply(pulse_sum_contact * psc, const phys_vec3 & s_)
{
	const phys_vec3 impulse = psc->m_ud_n * s_.x + psc->m_ud_f1 * s_.y + psc->m_ud_f2 * s_.z;
	pulse_sum_node * const b1 = psc->m_b1;
	b1->t_vel += impulse * b1->m_inv_mass;
	b1->a_vel += m_b1_ap_n * s_.x + m_b1_ap_f1 * s_.y + m_b1_ap_f2 * s_.z;
	if (psc->m_b2)
	{
		pulse_sum_node * const b2 = psc->m_b2;
		b2->t_vel -= impulse * b2->m_inv_mass;
		b2->a_vel -= m_b2_ap_n * s_.x + m_b2_ap_f1 * s_.y + m_b2_ap_f2 * s_.z;
	}
}

// Friction cone: |(f1, f2)| <= -fric_coef * normal.
inline void pulse_sum_contact_point::clamp_f(pulse_sum_contact * psc)
{
	const float m_pulse_sum_fric_limit = -psc->m_fric_coef * m_pulse_sum.x;
	const float pulse_sum_fric_sq = m_pulse_sum.y * m_pulse_sum.y + m_pulse_sum.z * m_pulse_sum.z;
	if (m_pulse_sum_fric_limit * m_pulse_sum_fric_limit < pulse_sum_fric_sq)
	{
		const float pulse_sum_fric = sqrtf(pulse_sum_fric_sq);
		PHYS_ASSERT(pulse_sum_fric > 0.0f);
		PHYS_ASSERT(m_pulse_sum_fric_limit >= 0.0f);
		const float ratio = m_pulse_sum_fric_limit / pulse_sum_fric;
		m_pulse_sum.y = ratio * m_pulse_sum.y;
		m_pulse_sum.z = ratio * m_pulse_sum.z;
	}
}

// Warm start from the cached impulse rates (normal <= 0, friction in the cone).
inline void pulse_sum_contact_point::SOLVER_solver_prolog(pulse_sum_contact * psc, pulse_sum_cache * m_pulse_sum_cache, const float delta_t)
{
	m_right_side -= get_vel(psc);
	m_pulse_sum.x = m_pulse_sum_cache[contact_point_info::PSC_N].m_pulse_sum * delta_t;
	m_pulse_sum.y = m_pulse_sum_cache[contact_point_info::PSC_F1].m_pulse_sum * delta_t;
	m_pulse_sum.z = m_pulse_sum_cache[contact_point_info::PSC_F2].m_pulse_sum * delta_t;
	if (m_pulse_sum.x > 0.0f)
		m_pulse_sum.x = 0.0f;
	clamp_f(psc);
	apply(psc, m_pulse_sum);
}

// Normal first (clamped <= 0), then the friction pair through the inverse friction block with the
// normal change folded in, then the cone clamp.
inline void pulse_sum_contact_point::SOLVER_apply_relaxation(pulse_sum_contact * psc, float & error_sq)
{
	const phys_vec3 m_last_pulse_sum = m_pulse_sum;
	phys_vec3 m_objective = get_objective(psc) - m_right_side;
	m_pulse_sum.x = m_pulse_sum.x - m_objective.x / m_denom_xx;
	if (m_pulse_sum.x > 0.0f)
		m_pulse_sum.x = 0.0f;
	const float dn = m_pulse_sum.x - m_last_pulse_sum.x;
	m_objective.y = m_denom_xy * dn + m_objective.y;
	m_objective.z = dn * m_denom_xz + m_objective.z;
	m_pulse_sum.y = m_pulse_sum.y - (m_inv_yz * m_objective.z + m_inv_yy * m_objective.y);
	m_pulse_sum.z = m_pulse_sum.z - (m_objective.y * m_inv_yz + m_objective.z * m_inv_zz);
	clamp_f(psc);
	const phys_vec3 s_ = m_pulse_sum - m_last_pulse_sum;
	apply(psc, s_);
	const float error_x = m_denom_xx * s_.x;
	const float error_y = m_denom_yy * s_.y;
	const float error_z = m_denom_zz * s_.z;
	const float error_x_sq = error_x * error_x;
	const float error_y_sq = error_y * error_y;
	const float error_z_sq = error_z * error_z;
	if (error_sq < error_x_sq)
		error_sq = error_x_sq;
	if (error_sq < error_y_sq)
		error_sq = error_y_sq;
	if (error_sq < error_z_sq)
		error_sq = error_z_sq;
}

// Penetration recovery (as pulse_sum_normal::setup_vel_uni_standard, normal row only) plus
// restitution: an approaching contact (impact speed above sqrt(min_restitution_impact_vel_sq)) gets a
// separating target of min(impact_vel * restitution_k, max_restitution_v).
inline void pulse_sum_contact_point::setup_vel_uni_restitution(pulse_sum_contact * psc, const float restitution_k,
	const float max_restitution_v, const float delta_t,
	const float max_penalty_restitution_vel,
	const float min_restitution_impact_vel_sq)
{
	const float pos = get_pos(psc);
	const float dt = delta_t > MIN_NERP_DELTA_T ? delta_t : MIN_NERP_DELTA_T;
	const float big_dirt = -pos / dt;
	m_big_dirt = big_dirt;
	if (big_dirt < 0.0f)
		m_big_dirt = big_dirt * 0.3f;
	const float max_big_dirt = -max_penalty_restitution_vel;
	if (max_big_dirt > m_big_dirt)
		m_big_dirt = max_big_dirt;
	if (!(m_big_dirt >= 0.0f)) // NaN counts as negative
	{
		m_right_side = PHYS_ZERO_VEC;
	}
	else
	{
		// Only the normal row gets the target; the friction rows stay at 0.
		m_right_side.x = m_big_dirt;
		m_right_side.y = 0.0f;
		m_right_side.z = 0.0f;
		m_big_dirt = 0.0f;
	}

	if (restitution_k > PHYS_EPS_1E5 && max_restitution_v > PHYS_EPS_1E5 && pos >= 0.0f)
	{
		const float impact_vel = get_impact_vel(psc);
		if (impact_vel > 0.0f)
		{
			const float impact_vel_sq = impact_vel * impact_vel;
			if (min_restitution_impact_vel_sq < impact_vel_sq)
			{
				float restitution_vel = impact_vel * restitution_k;
				if (max_restitution_v < restitution_vel)
					restitution_vel = max_restitution_v;
				if (!(m_big_dirt <= -PHYS_EPS_1E5)) // NaN takes this branch
				{
					m_right_side.x = m_right_side.x - restitution_vel;
				}
				else if (m_big_dirt > -restitution_vel)
				{
					// the bounce separates faster than the penetration recovery would: take it instead
					m_right_side.x = -restitution_vel;
					m_big_dirt = 0.0f;
				}
			}
		}
	}
}

inline void pulse_sum_contact_point::set_object_col_pt(pulse_sum_contact * psc, const phys_vec3 & object_col_pt)
{
	PHYS_ASSERT(psc->m_b2 == NULL);
	m_b2_r = object_col_pt;
}

inline void pulse_sum_contact::set(rigid_body * const b1, rigid_body * const b2, contact_point_info * cpi, const float delta_t)
{
	PHYS_ASSERT(b1);
	PHYS_ASSERT(b2);
	PHYS_ASSERT(b1 != b2);
	PHYS_ASSERT_STR(rbint_get_pulse_sum_node(b1) || rbint_get_pulse_sum_node(b2), "rbint::get_pulse_sum_node(b1) || rbint::get_pulse_sum_node(b2)");
	PHYS_ASSERT_STR(rbint_verify_pulse_sum_node(b1) && rbint_verify_pulse_sum_node(b2), "rbint::verify_pulse_sum_node(b1) && rbint::verify_pulse_sum_node(b2)");
	PHYS_ASSERT_STR_MSG(rbint_get_pulse_sum_node(b1) != NULL, "rbint::get_pulse_sum_node(b1) != NULL",
		"b1 in contact constraint cannot be environment or user rigid body.");

	m_b1 = b1->m_node;
	m_b2 = b2->m_node;
	m_ud_n = cpi->m_normal;
	m_ud_f1 = construct_orth_ud(m_ud_n);
	m_ud_f2 = phys_cross(m_ud_n, m_ud_f1);
	m_fric_coef = cpi->m_fric_coef;
	m_pulse_sum_cache_list = cpi->m_list_pulse_sum_cache_info;
	PHYS_ASSERT(m_list_pscp_count == cpi->m_point_pair_count);

	// impacts slower than 0.3 * |gravity| (per second) do not bounce
	const float min_restitution_impact_vel_sq = AbsSquared(m_b1->m_rb->m_gravity_acc_vec) * 0.09f;
	for (int pp_i = 0; pp_i < cpi->m_point_pair_count; pp_i++)
	{
		pulse_sum_contact_point * const pscp = &m_list_pscp[pp_i];
		pscp->m_b1_r = phys_multiply(b1->m_mat, cpi->m_list_b1_r_loc[pp_i]);
		const phys_vec3 b2_r = phys_multiply(b2->m_mat, cpi->m_list_b2_r_loc[pp_i]);
		if (m_b2)
		{
			pscp->m_b2_r = b2_r;
		}
		else
		{
			pscp->m_b2_ap_n = rbint::gtv(b2, b2_r); // object velocity
			pscp->set_object_col_pt(this, b2_r + b2->m_mat.GetW());
		}
		pscp->calc_abs(this);
		pscp->setup_vel_uni_restitution(this, cpi->m_bounce_coef, cpi->m_max_restitution_vel, delta_t, 50.0f,
			min_restitution_impact_vel_sq);
	}
}

// Store the velocity-phase impulse rates in the manifold caches and turn on the normal position
// correction for the vel+pos phase.
inline void pulse_sum_contact::SOLVER_solver_intermediate(const float delta_t)
{
	contact_point_info::pulse_sum_cache_info * psci = (contact_point_info::pulse_sum_cache_info *)m_pulse_sum_cache_list;
	for (pulse_sum_contact_point * pscp = m_list_pscp; pscp != &m_list_pscp[m_list_pscp_count]; ++pscp, ++psci)
	{
		psci->m_ps_cache_list[contact_point_info::PSC_N].m_pulse_sum = pscp->m_pulse_sum.x / delta_t;
		psci->m_ps_cache_list[contact_point_info::PSC_F1].m_pulse_sum = pscp->m_pulse_sum.y / delta_t;
		psci->m_ps_cache_list[contact_point_info::PSC_F2].m_pulse_sum = pscp->m_pulse_sum.z / delta_t;
		pscp->m_right_side.x = pscp->m_big_dirt + pscp->m_right_side.x;
	}
}
