#pragma once

#include "tl/physics/include/constraint_solver/pulse_sum_constraint_solver.h"

// Is (side, fwd) inside the friction ellipse (with a relative 0.00001 tolerance)?
// The suspension pulse sum is <= 0 (push), so -ratio * pulse is the positive axis length.
inline const bool pulse_sum_wheel::pulse_chain_within_limits()
{
	// a NaN axis becomes m_side_fric_max
	float a_ = -m_side->m_pulse_limit_ratio * m_suspension.m_pulse_sum;
	if (!(m_side_fric_max > a_))
		a_ = m_side_fric_max;
	float b_ = -m_fwd->m_pulse_limit_ratio * m_suspension.m_pulse_sum;
	if (!(m_side_fric_max > b_))
		b_ = m_side_fric_max;
	const float a2_ = a_ * a_;
	const float b2_ = b_ * b_;
	const float a2_b2_ = b2_ * a2_;
	const float side_sq = m_side->m_pulse_sum * m_side->m_pulse_sum;
	const float fwd_sq = m_fwd->m_pulse_sum * m_fwd->m_pulse_sum;
	return rel_cmp_le(a2_ * fwd_sq + b2_ * side_sq, a2_b2_, PHYS_EPS_1E5);
}

// Scale (side, fwd) back onto the friction ellipse when outside it; sets the fwd row's
// FLAG_PULSE_CHAIN_CLAMPED (read by the wheel epilog as "sliding"). Returns true when the result differs
// from the current pulse sums.
inline const bool pulse_sum_wheel::clamp_pulse_sum_pulse_chain(float * ps1_, float * ps2_)
{
	// a NaN axis becomes m_side_fric_max
	float a_ = -m_side->m_pulse_limit_ratio * m_suspension.m_pulse_sum;
	if (!(m_side_fric_max > a_))
		a_ = m_side_fric_max;
	float t_ = -m_fwd->m_pulse_limit_ratio * m_suspension.m_pulse_sum;
	if (!(m_side_fric_max > t_))
		t_ = m_side_fric_max;
	const float numer_sq_a = a_ * a_;
	const float numer_sq_b = t_ * t_;
	const float p1_ = m_side->m_pulse_sum;
	const float p2_ = m_fwd->m_pulse_sum;
	const float denom_sq = p2_ * p2_ * numer_sq_a + p1_ * p1_ * numer_sq_b;
	const float numer_sq = numer_sq_b * numer_sq_a;
	if (denom_sq >= numer_sq || numer_sq < PHYS_EPS_1E5)
	{
		m_fwd->m_flags |= pulse_sum_normal::FLAG_PULSE_CHAIN_CLAMPED;
		const float scale = !(denom_sq > PHYS_EPS_1E5) ? 0.0f : sqrtf(numer_sq / denom_sq); // NaN -> 0
		*ps1_ = scale * p1_;
		*ps2_ = scale * p2_;
		return true;
	}
	m_fwd->m_flags &= ~pulse_sum_normal::FLAG_PULSE_CHAIN_CLAMPED;
	*ps1_ = m_side->m_pulse_sum;
	*ps2_ = m_fwd->m_pulse_sum;
	return false;
}

inline void pulse_sum_wheel::addp_pulse_chain()
{
	pulse_sum_normal * const side = m_side;
	pulse_sum_normal * const fwd = m_fwd;
	float ps1_;
	float ps2_;
	if (clamp_pulse_sum_pulse_chain(&ps1_, &ps2_))
	{
		float s_ = ps1_ - side->m_pulse_sum;
		side->apply(s_);
		side->m_pulse_sum = ps1_;
		s_ = ps2_ - fwd->m_pulse_sum;
		fwd->apply(s_);
		fwd->m_pulse_sum = ps2_;
	}
	PHYS_ASSERT(pulse_chain_within_limits());
}

inline void pulse_sum_wheel::SOLVER_solver_prolog(const float delta_t)
{
	m_suspension.SOLVER_solver_prolog(delta_t);
	if (m_side)
	{
		m_side->SOLVER_solver_prolog(delta_t);
		if (m_fwd)
		{
			m_fwd->SOLVER_solver_prolog(delta_t);
			addp_pulse_chain();
		}
	}
}

// With a fwd row the two friction rows are relaxed without error, chained, and their
// post-chain change is what counts toward the error.
inline void pulse_sum_wheel::SOLVER_apply_relaxation(float & error_sq)
{
	m_suspension.SOLVER_apply_relaxation(error_sq, true);
	if (!m_side)
		return;
	const float side_last_pulse_sum = m_side->m_pulse_sum;
	m_side->SOLVER_apply_relaxation(error_sq, m_fwd == NULL);
	if (!m_fwd)
		return;
	const float m_fwd_last_pulse_sum = m_fwd->m_pulse_sum;
	m_fwd->SOLVER_apply_relaxation(error_sq, false);
	addp_pulse_chain();
	const float side_error = (m_side->m_pulse_sum - side_last_pulse_sum) * m_side->m_denom;
	const float side_error_sq = side_error * side_error;
	if (error_sq < side_error_sq)
		error_sq = side_error_sq;
	const float fwd_error = (m_fwd->m_pulse_sum - m_fwd_last_pulse_sum) * m_fwd->m_denom;
	const float fwd_error_sq = fwd_error * fwd_error;
	if (error_sq < fwd_error_sq)
		error_sq = fwd_error_sq;
}

inline void pulse_sum_wheel::SOLVER_solver_intermediate(const float delta_t)
{
	m_suspension.SOLVER_solver_intermediate(delta_t);
	if (m_side)
	{
		m_side->SOLVER_solver_intermediate(delta_t);
		if (m_fwd)
			m_fwd->SOLVER_solver_intermediate(delta_t);
	}
}
