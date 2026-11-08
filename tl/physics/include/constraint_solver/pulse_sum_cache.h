#pragma once

// Retail: codsrc\tl\physics\include\constraint_solver\pulse_sum_cache.h. Header only.
// BO1 dropped SM3's m_visit_key: the IDB class is { float m_pulse_sum } (4 bytes). The default
// constructor zeroes it: every phys_free_list<rigid_body_constraint_*>::add (0xa16a30-0xa17250)
// zeroes exactly the cache arrays of the new constraint, even for types whose constructor does
// nothing else (hinge, wheel, distance, angular_actuator, upright, custom_path).

#include "tl/base/include/tl_defs.h"

class pulse_sum_cache
{
public:
    // TREYBO: public (SM3 private + friend pulse_sum_constraint_solver). BO1 constraint code reads
    // m_pulse_sum directly (rigid_body_constraint_point::epilog_vel_constraint 0xa17a10).
    float m_pulse_sum;

    pulse_sum_cache() : m_pulse_sum(0.0f) {}

    const float get_pulse_sum() const { return m_pulse_sum; }
};
