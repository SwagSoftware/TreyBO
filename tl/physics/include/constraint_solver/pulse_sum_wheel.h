#pragma once

// A vehicle wheel contact: the suspension row, an optional side-friction row and an optional forward
// (drive / brake) row. With both friction rows present their impulses are kept inside the friction ellipse
// side^2 / a^2 + fwd^2 / b^2 <= 1, a and b scaled by the suspension load ("pulse chain").

#include "tl/base/include/tl_defs.h"
#include "tl/physics/include/phys_assert.h"
#include "tl/physics/include/phys_mem.h"
#include "tl/physics/include/constraint_solver/pulse_sum_types.inc"
#include "tl/physics/include/constraint_solver/pulse_sum_normal.h"

// 16-aligned through m_suspension.
class pulse_sum_wheel : public phys_link_list_base<pulse_sum_wheel>
{
	friend class pulse_sum_constraint_solver;

public:
	// Data members public: rigid_body_constraint_wheel::setup_constraint fills them.
	pulse_sum_normal m_suspension;
	pulse_sum_normal * m_side; // allocated by the wheel constraint itself
	pulse_sum_normal * m_fwd; // create_pulse_sum_wheel_fwd
	float m_side_fric_max; // caps both ellipse axes (the fwd axis too)

	void set_side_fwd_ratios(const float side_ratio, const float fwd_ratio, const float side_fric_max)
	{
		PHYS_ASSERT(m_side && m_fwd);
		m_side->m_pulse_limit_ratio = side_ratio;
		m_fwd->m_pulse_limit_ratio = fwd_ratio;
		m_side_fric_max = side_fric_max;
	}

private:
	const bool pulse_chain_within_limits();
	const bool clamp_pulse_sum_pulse_chain(float * ps1_, float * ps2_);
	void addp_pulse_chain();
	void SOLVER_solver_prolog(const float delta_t);
	void SOLVER_apply_relaxation(float & error_sq);
	void SOLVER_solver_intermediate(const float delta_t);
};
