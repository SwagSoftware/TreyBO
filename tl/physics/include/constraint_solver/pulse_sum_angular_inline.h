#pragma once

#include "tl/physics/include/constraint_solver/pulse_sum_constraint_solver.h"

// Start-of-step relative angular velocity about m_ud, from the rigid bodies.
inline const float pulse_sum_angular::get_vel()
{
	const phys_vec3 & b2_a_vel = m_b2 ? m_b2->m_rb->m_a_vel : m_b2_ap;
	return phys_dot(m_ud, m_b1->m_rb->m_a_vel - b2_a_vel);
}

// Misalignment of the two axes about m_ud.
inline const float pulse_sum_angular::get_pos()
{
	return -phys_dot(m_ud, phys_cross(m_b1_r, m_b2_r));
}

inline void pulse_sum_angular::set(rigid_body * const b1, const phys_vec3 & b1_r, rigid_body * const b2, const phys_vec3 & b2_r,
	const phys_vec3 & ud, pulse_sum_cache * const ps_cache)
{
	PHYS_ASSERT(ps_cache);
	PHYS_ASSERT(b1);
	PHYS_ASSERT(b2);
	PHYS_ASSERT(b1 != b2);
	PHYS_ASSERT_UNIT(ud);
	PHYS_ASSERT_STR(rbint_get_pulse_sum_node(b1) || rbint_get_pulse_sum_node(b2), "rbint::get_pulse_sum_node(b1) || rbint::get_pulse_sum_node(b2)");
	PHYS_ASSERT_STR(rbint_verify_pulse_sum_node(b1) && rbint_verify_pulse_sum_node(b2), "rbint::verify_pulse_sum_node(b1) && rbint::verify_pulse_sum_node(b2)");

	if (b1->m_node)
	{
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
			set_object_vel(b2->m_a_vel);
			set_object_col_pt(b2_r);
		}
		m_ud = ud;
	}
	else
	{
		// b1 is fixed: solve from b2's side with the direction flipped
		m_b1 = b2->m_node;
		m_b1_r = b2_r;
		m_b2 = NULL;
		set_object_vel(b1->m_a_vel);
		set_object_col_pt(b1_r);
		m_ud = -ud;
	}
	m_pulse_sum_cache = ps_cache;
	m_flags = 0;
	calc_abs();
}

inline void pulse_sum_angular::calc_abs()
{
	m_b1_ap = phys_multiply(m_b1->m_world_inv_inertia, m_ud);
	m_denom = phys_dot(m_b1_ap, m_ud);
	if (m_b2)
	{
		m_b2_ap = phys_multiply(m_b2->m_world_inv_inertia, m_ud);
		m_denom = phys_dot(m_b2_ap, m_ud) + m_denom;
	}
	PHYS_ASSERT(m_denom > 0.00001f);
}

// Relative angular velocity change accumulated in the solver nodes this sub-step.
inline const float pulse_sum_angular::get_objective()
{
	phys_vec3 a_vel = m_b1->a_vel;
	if (m_b2)
		a_vel = a_vel - m_b2->a_vel;
	return phys_dot(m_ud, a_vel);
}

// Angular impulse s_ about m_ud: +s_ to b1, -s_ to b2.
inline void pulse_sum_angular::apply(const float & s_)
{
	m_b1->a_vel += m_b1_ap * s_;
	if (m_b2)
		m_b2->a_vel -= m_b2_ap * s_;
}

// No flags, unlike pulse_sum_normal.
inline const float pulse_sum_angular::clamp_pulse_sum(const float ps)
{
	PHYS_ASSERT(m_pulse_sum_min <= m_pulse_sum_max);
	if (m_pulse_sum_min > ps)
		return m_pulse_sum_min;
	if (m_pulse_sum_max < ps)
		return m_pulse_sum_max;
	return ps;
}

inline void pulse_sum_angular::SOLVER_apply_relaxation(float & error_sq)
{
	float m_objective = get_objective() + m_cfm * m_pulse_sum - m_right_side;
	const float m_last_pulse_sum = m_pulse_sum;
	m_objective = m_pulse_sum - m_objective / m_denom;
	m_pulse_sum = clamp_pulse_sum(m_objective);
	PHYS_ASSERT(m_pulse_sum >= (m_pulse_sum_min - .0001f) && m_pulse_sum <= (m_pulse_sum_max + .0001f));
	const float s_ = m_pulse_sum - m_last_pulse_sum;
	apply(s_);
	const float error = (m_pulse_sum - m_last_pulse_sum) * m_denom;
	const float error_sq_ = error * error;
	if (error_sq < error_sq_)
		error_sq = error_sq_;
}

// Warm start from the cached impulse rate.
inline void pulse_sum_angular::SOLVER_solver_prolog(const float delta_t)
{
	m_right_side = m_right_side - get_vel();
	m_pulse_sum = m_pulse_sum_cache->m_pulse_sum * delta_t;
	m_pulse_sum = clamp_pulse_sum(m_pulse_sum);
	apply(m_pulse_sum);
}

inline void pulse_sum_angular::SOLVER_solver_intermediate(const float delta_t)
{
	m_pulse_sum_cache->m_pulse_sum = m_pulse_sum / delta_t;
	m_right_side = m_big_dirt + m_right_side;
}

// As pulse_sum_normal::setup_vel_uni_standard, on the angular misalignment.
inline void pulse_sum_angular::setup_vel_uni_standard(const float delta_t, const float max_penalty_restitution_vel)
{
	const float dt = delta_t > MIN_NERP_DELTA_T ? delta_t : MIN_NERP_DELTA_T;
	const float big_dirt = -get_pos() / dt;
	m_big_dirt = big_dirt;
	if (big_dirt < 0.0f)
		m_big_dirt = big_dirt * 0.3f;
	const float max_big_dirt = -max_penalty_restitution_vel;
	if (max_big_dirt > m_big_dirt)
		m_big_dirt = max_big_dirt;
	if (!(m_big_dirt >= 0.0f)) // NaN counts as negative
	{
		m_right_side = 0.0f;
	}
	else
	{
		m_right_side = m_big_dirt;
		m_big_dirt = 0.0f;
	}
	m_cfm = 0.0f;
}
