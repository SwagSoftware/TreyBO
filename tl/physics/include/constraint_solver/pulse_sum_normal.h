#pragma once

// Retail: codsrc\tl\physics\include\constraint_solver\pulse_sum_normal.h (asserts :69, :72, :269, :270).
// Bodies: pulse_sum_normal_inline.h. Out-of-line COMDAT copies sit in the rbc_def_generic / rbc_def_vehicle TUs
// (0xa18f10, 0xa193b0, 0xa1a2c0) and in the solver TU (0xa25930-0xa28440).
//
// One scalar row along m_ud between a point on b1 and a point on b2 (or a fixed world point with a known
// velocity when b2 has no solver node). m_pulse_sum is the accumulated impulse of this row for the current
// sub-step; the pulse_sum_cache keeps it between frames as an impulse rate (pulse / delta_t) for warm starting.

#include "tl/base/include/tl_defs.h"
#include "tl/physics/include/phys_math.h"
#include "tl/physics/include/phys_assert.h"
#include "tl/physics/include/phys_mem.h"
#include "tl/physics/include/constraint_solver/pulse_sum_cache.h"
#include "tl/physics/include/constraint_solver/pulse_sum_types.inc"

class rigid_body;

class pulse_sum_normal : public phys_link_list_base<pulse_sum_normal>
{
    friend class pulse_sum_constraint_solver;
    friend class pulse_sum_wheel;

public:
    // TREYBO: names not recovered; values from clamp_pulse_sum 0xa28260 (0x2), set_pulse_sum_limits_parent_ratio
    // 0xa193b0 (0x1) and pulse_sum_wheel::clamp_pulse_sum_pulse_chain 0xa2af80 (0x4, fwd row only).
    // rigid_body_constraint_wheel::epilog_vel_constraint (0xa197e0) reads 0x2 and 0x4 as literals.
    enum flags_e
    {
        FLAG_PULSE_LIMIT_PARENT_RATIO = 0x1, // limits = +-|parent pulse| * m_pulse_limit_ratio, recomputed per clamp
        FLAG_PULSE_SUM_CLAMPED = 0x2,        // the last clamp_pulse_sum hit a limit
        FLAG_PULSE_CHAIN_CLAMPED = 0x4,      // wheel friction ellipse was active on the last chain clamp
    };

    // TREYBO: data members public (SM3 style would be private + friends); rbc_def setup code writes
    // m_pulse_sum_min/max, m_right_side, m_big_dirt, m_cfm and m_denom directly and reads m_flags.
    phys_vec3 m_ud;   // row direction (unit); the row's relative velocity is dot(m_ud, v1 - v2)
    phys_vec3 m_b1_r; // world offset of the row point from b1's position
    phys_vec3 m_b2_r; // world offset from b2's position; the world point itself when m_b2 == NULL
    phys_vec3 m_b1_ap; // b1 world inverse inertia * (r1 x ud): angular velocity change per unit impulse
    phys_vec3 m_b2_ap; // same for b2; the object's point velocity when m_b2 == NULL
    float m_pulse_sum_min;
    float m_pulse_sum_max;
    float m_pulse_sum;
    float m_right_side; // target relative velocity (minus the start velocity after the prolog)
    float m_big_dirt;   // position-correction velocity, added to m_right_side for the vel+pos phase
    float m_cfm;        // constraint force mixing (soft rows): objective += m_cfm * m_pulse_sum
    float m_denom;      // effective inverse mass of the row (+ m_cfm)
    float m_pulse_limit_ratio;
    unsigned int m_flags;
    pulse_sum_normal *m_pulse_parent;
    pulse_sum_node *m_b1;
    pulse_sum_node *m_b2;
    pulse_sum_cache *m_pulse_sum_cache;

    void set(rigid_body *const b1, const phys_vec3 &b1_r, rigid_body *const b2, const phys_vec3 &b2_r,
             const phys_vec3 &ud, pulse_sum_cache *const ps_cache, const phys_vec3 &b1_r_displace); // 0xa26670

    void setup_vel_uni_standard(const float delta_t, const float max_penalty_restitution_vel);                          // 0xa1a2c0
    void setup_vel_uni_standard_pos_adjust(const float delta_t, const float pos_adjust, const float max_penalty_restitution_vel); // 0xa18f10

    // pulse_sum_normal.h:269 / :270 (0xa193b0)
    void set_pulse_sum_limits_parent_ratio(const float limit_ratio, pulse_sum_normal *const parent)
    {
        PHYS_ASSERT(limit_ratio >= 0.0f);
        PHYS_ASSERT(parent);
        m_flags |= FLAG_PULSE_LIMIT_PARENT_RATIO;
        m_pulse_limit_ratio = limit_ratio;
        m_pulse_parent = parent;
        m_pulse_sum_min = 0.0f;
        m_pulse_sum_max = 0.0f;
    }

    // pulse_sum_normal.h:69 (0xa25930)
    void set_object_vel(const phys_vec3 &object_vel)
    {
        PHYS_ASSERT_STR(m_b2 == NULL, "m_b2 == NULL");
        m_b2_ap = object_vel;
    }

    // pulse_sum_normal.h:72 (0xa25980)
    void set_object_col_pt(const phys_vec3 &object_col_pt)
    {
        PHYS_ASSERT_STR(m_b2 == NULL, "m_b2 == NULL");
        m_b2_r = object_col_pt;
    }

    const phys_vec3 get_relative_velocity();            // 0xa26ac0
    const phys_vec3 get_relative_velocity_change_dir(); // 0xa26970
    const float get_unclamped_pulse_sum();              // 0xa26c00

    // TREYBO: private in retail (?get_pos@pulse_sum_normal@@AAE?BMXZ). Public here because the
    // rigid_body_constraint_wheel / _custom_path setup_constraint bodies (0xa19aa0, 0xa2d220) call it and TreyBO
    // does not befriend every constraint class.
    const float get_pos(); // 0xa265c0

private:
    void calc_abs(const phys_vec3 &b1_r_displace); // 0xa26150 (pulse_sum_normal_inline.h:11)
    const float get_vel();                          // 0xa26320
    const float get_last_vel();                     // 0xa26470
    const float get_objective();                    // 0xa28000
    void apply(const float &s_);                    // 0xa28110
    const float clamp_pulse_sum(const float ps);    // 0xa28260
    void SOLVER_apply_relaxation(float &error_sq, const bool add_error); // 0xa28360
    void SOLVER_solver_prolog(const float delta_t);                      // 0xa28440
    void SOLVER_solver_intermediate(const float delta_t); // TREYBO: name; retail inlines it in solve_constraints 0xa2b990
};
