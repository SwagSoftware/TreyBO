#pragma once

// Retail: codsrc\tl\physics\include\constraint_solver\pulse_sum_wheel.h (assert :34).
// Bodies: pulse_sum_wheel_inline.h; out-of-line copies in the rbc_def_vehicle TU (0xa19430) and the solver TU
// (0xa28e30, 0xa2aea0-0xa2b220).
//
// A vehicle wheel contact: the suspension row, an optional side-friction row and an optional forward
// (drive / brake) row. With both friction rows present their impulses are kept inside the friction ellipse
// side^2 / a^2 + fwd^2 / b^2 <= 1, a and b scaled by the suspension load ("pulse chain").

#include "tl/base/include/tl_defs.h"
#include "tl/physics/include/phys_assert.h"
#include "tl/physics/include/phys_mem.h"
#include "tl/physics/include/constraint_solver/pulse_sum_types.inc"
#include "tl/physics/include/constraint_solver/pulse_sum_normal.h"

// TREYBO: IDB declares this 8-aligned; it is 16-aligned anyway through m_suspension.
class pulse_sum_wheel : public phys_link_list_base<pulse_sum_wheel>
{
    friend class pulse_sum_constraint_solver;

public:
    // TREYBO: data members public; rigid_body_constraint_wheel::setup_constraint (0xa19aa0) fills them.
    pulse_sum_normal m_suspension;
    pulse_sum_normal *m_side; // allocated by the wheel constraint itself
    pulse_sum_normal *m_fwd;  // create_pulse_sum_wheel_fwd
    float m_side_fric_max;    // caps both ellipse axes (retail uses it for the fwd axis too)

    // pulse_sum_wheel.h:34 (0xa19430)
    void set_side_fwd_ratios(const float side_ratio, const float fwd_ratio, const float side_fric_max)
    {
        PHYS_ASSERT(m_side && m_fwd);
        m_side->m_pulse_limit_ratio = side_ratio;
        m_fwd->m_pulse_limit_ratio = fwd_ratio;
        m_side_fric_max = side_fric_max;
    }

private:
    const bool pulse_chain_within_limits();                           // 0xa2aea0
    const bool clamp_pulse_sum_pulse_chain(float *ps1_, float *ps2_); // 0xa2af80
    void addp_pulse_chain();                                          // 0xa2b0c0 (pulse_sum_wheel_inline.h:67)
    void SOLVER_solver_prolog(const float delta_t);                   // 0xa2b220
    void SOLVER_apply_relaxation(float &error_sq);                    // 0xa2b150
    void SOLVER_solver_intermediate(const float delta_t);             // 0xa28e30
};
