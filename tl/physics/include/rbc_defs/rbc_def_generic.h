#pragma once

// Retail: codsrc\tl\physics\include\rbc_defs\rbc_def_generic.h (no BO1 assert references it).
// Out-of-line part: source/rbc_def_generic.cpp (TU ~0xa179b0-0xa18f08).
// Point, hinge and distance follow SM3 (BO1 layouts); angular_actuator and upright are BO1 layouts.
// TREYBO: data members are public (SM3 private) so host code can tune them as BO1's game code does.

#include "tl/base/include/tl_defs.h"
#include "tl/physics/include/phys_math.h"
#include "tl/physics/include/rigid_body.h"
#include "tl/physics/include/rbc_defs/rbc_def_base.h"
#include "tl/physics/include/constraint_solver/pulse_sum_cache.h"

class rigid_body_constraint_point : public rigid_body_constraint
{
public:
    enum ps_cache_e
    {
        PSC_X,
        PSC_Y,
        PSC_Z,
        NUM_PSC
    };

    phys_vec3 m_b1_r_loc;
    phys_vec3 m_b2_r_loc;
    pulse_sum_cache m_ps_cache_list[NUM_PSC];
    float m_stress; // squared length of the last pulse sum
    float m_spring_k; // BO1: optional soft (spring) point
    float m_damp_k;
    bool m_spring_enabled;

    // inlined in phys_free_list<rigid_body_constraint_point>::add 0xa16a30
    rigid_body_constraint_point() : m_stress(0.0f), m_spring_k(0.0f), m_damp_k(0.0f), m_spring_enabled(false) {}

    void set(const phys_vec3 &b1_r_loc, const phys_vec3 &b2_r_loc); // 0xa179e0

    // TREYBO: setter name not recovered (retail writes the three fields inline).
    void set_spring(const bool spring_enabled, const float spring_k, const float damp_k)
    {
        m_spring_enabled = spring_enabled;
        m_spring_k = spring_k;
        m_damp_k = damp_k;
    }

    const phys_vec3 get_b1_r_loc() const { return m_b1_r_loc; }
    const phys_vec3 get_b2_r_loc() const { return m_b2_r_loc; }
    float get_stress() const { return m_stress; }

    void setup_constraint(pulse_sum_constraint_solver *psys, const float delta_t); // 0xa18370
    void epilog_vel_constraint(const float delta_t);                              // 0xa17a10
};

class rigid_body_constraint_hinge : public rigid_body_constraint
{
public:
    enum flags_e
    {
        HINGE_MIN_LIMIT_ACTIVE = (1 << 0),
        HINGE_MAX_LIMIT_ACTIVE = (1 << 1),
        // TREYBO: names not recovered (BO1-only bits, tested in setup_constraint 0xa18400; set() clears them).
        HINGE_FLAG_CLOSEST_LIMIT_ONLY = (1 << 3), // activate only the limit b1_ref is nearer to, no tolerance band
        HINGE_FLAG_NO_POINT = (1 << 4),           // skip the point constraint (pure axis hinge)
    };

    enum ps_cache_e
    {
        PSC_X,
        PSC_Y,
        PSC_Z,
        PSC_DAMP,
        PSC_A1,
        PSC_A2,
        PSC_MIN_LIMIT,
        PSC_MAX_LIMIT,
        NUM_PSC
    };

    // BO1 IDB order (SM3 had m_flags first).
    phys_vec3 m_b1_r_loc;
    phys_vec3 m_b2_r_loc;
    phys_vec3 m_b1_axis_loc;
    phys_vec3 m_b2_axis_loc;
    phys_vec3 m_b1_a1_loc;
    phys_vec3 m_b1_a2_loc;
    phys_vec3 m_b1_ref_loc;
    phys_vec3 m_b2_ref_min_loc;
    phys_vec3 m_b2_ref_max_loc;
    float m_damp_k;
    unsigned int m_flags;
    pulse_sum_cache m_ps_cache[NUM_PSC];

    // TREYBO: public (SM3 private) so the BO1-only bits can be set.
    void set_flag(const flags_e f, const unsigned int b) { b ? m_flags |= f : m_flags &= ~f; }
    const unsigned int get_flag(const flags_e f) const { return m_flags & f; }

    void set(const phys_vec3 &b1_r_loc, const phys_vec3 &b2_r_loc,
             const phys_vec3 &b1_axis_loc, const phys_vec3 &b2_axis_loc,
             const phys_vec3 &b1_ref_loc, const phys_vec3 &b2_ref_loc,
             const float theta_min, const float theta_max, const float damp_k); // 0xa17e40

    void set_damp_k(const float damp_k) { m_damp_k = damp_k; }

    void setup_constraint(pulse_sum_constraint_solver *psys, const float delta_t); // 0xa18400
};

class rigid_body_constraint_distance : public rigid_body_constraint
{
public:
    enum flags_e
    {
        FLAG_ENABLE = (1 << 0),
        FLAG_B2_R_IS_DAMP_POINT = (1 << 1),
    };

    enum ps_cache_e
    {
        PSC_MAX_DIST,
        PSC_MIN_DIST,
        PSC_DAMP,
        NUM_PSC
    };

    phys_vec3 m_b1_r_loc;
    phys_vec3 m_b2_r_loc;
    float m_min_distance;
    float m_max_distance;
    float m_next_max_distance;
    float m_max_distance_vel;
    float m_damp_coef;
    unsigned int m_flags;
    pulse_sum_cache m_ps_cache_list[NUM_PSC];

    void set_flag(const unsigned int f, const int b) { b ? m_flags |= f : m_flags &= ~f; }
    const int get_flag(const unsigned int f) const { return m_flags & f; }

    void set(const phys_vec3 &b1_r_loc, const phys_vec3 &b2_r_loc,
             const float min_distance, const float max_distance); // 0xa17a50

    void set_next_max_distance(const float next_max_distance) { m_next_max_distance = next_max_distance; }
    void set_next_b2_r_loc(const phys_vec3 &next_b2_r_loc) { m_b2_r_loc = next_b2_r_loc; }

    void set_damp_coef(const float damp_coef) { m_damp_coef = damp_coef; }
    const float get_damp_coef() const { return m_damp_coef; }

    void set_b1_r_is_damp_point() { set_flag(FLAG_B2_R_IS_DAMP_POINT, false); }
    void set_b2_r_is_damp_point() { set_flag(FLAG_B2_R_IS_DAMP_POINT, true); }

    void set_enable(const bool enable) { set_flag(FLAG_ENABLE, enable); }
    const int get_enable() const { return get_flag(FLAG_ENABLE); }

    const phys_vec3 get_b1_r_loc() const { return m_b1_r_loc; }
    const phys_vec3 get_b2_r_loc() const { return m_b2_r_loc; }
    const float get_min_distance() const { return m_min_distance; }
    const float get_max_distance() const { return m_max_distance; }

    void setup_constraint(pulse_sum_constraint_solver *psys, const float delta_t); // 0xa187b0

    // m_max_distance moves to m_next_max_distance over one outer step.
    void outer_prolog_update(const float delta_t); // 0xa17aa0
    void inner_update(const float delta_t);        // 0xa17ac0
    void outer_epilog_update(const float delta_t); // 0xa17ae0
};

class rigid_body_constraint_angular_actuator : public rigid_body_constraint
{
public:
    enum ps_cache_e
    {
        PSC_X,
        PSC_Y,
        PSC_Z,
        NUM_PSC
    };

    phys_mat44 m_target_mat; // b1-relative target orientation of b2
    phys_mat44 m_next_target_mat;
    phys_vec3 m_a_vel; // target angular velocity, b1 local
    float m_power;
    float m_power_scale;
    bool m_enabled;
    pulse_sum_cache m_ps_cache_list[NUM_PSC];

    void set(float power, const phys_mat44 &target_mat); // 0xa17af0

    void set_next_target(const phys_mat44 &next_target_mat)
    {
        m_next_target_mat = next_target_mat;
        PHYS_ASSERT_ORTHONORMAL(m_next_target_mat);
    }
    void set_power(const float power) { m_power = power; }
    void set_power_scale(const float power_scale) { m_power_scale = power_scale; }
    void set_enabled(bool enabled) { m_enabled = enabled; }
    bool is_enabled() const { return m_enabled; }

    void setup_constraint(pulse_sum_constraint_solver *psys, const float delta_t); // 0xa18c40

    void outer_prolog_update(const float delta_t); // 0xa17be0
    void inner_update(const float delta_t);        // 0xa18090
    void outer_epilog_update(const float delta_t); // 0xa17c50
};

// BO1-only: keeps b1's lean axis aligned with b2's up axis (used for bikes / leaning vehicles).
class rigid_body_constraint_upright : public rigid_body_constraint
{
public:
    phys_vec3 m_b1_forward_axis_loc;
    phys_vec3 m_b1_right_axis_loc;
    phys_vec3 m_b1_up_axis_loc;
    phys_vec3 m_b1_lean_axis_loc;
    phys_vec3 m_b2_up_axis_loc; // used as a world direction (b2 is the environment)
    phys_vec3 m_last_t_vel;
    phys_vec3 m_last_a_vel;
    float m_avg_side_force;
    float m_avg_normal_force;
    float m_lean_angle_calc_delta_t;
    float m_lean_angle;
    float m_lean_angle_multiplier;
    float m_max_lean_angle;
    float m_moving_average_total_time;
    bool m_enabled;
    pulse_sum_cache m_ps_cache_list[1];

    const float calc_current_lean_angle() const;                       // 0xa17cb0
    const phys_vec3 calc_b1_lean_axis_loc(const float lean_angle) const; // 0xa17da0
    void update_lean_axis(const phys_vec3 &b1_lean_center, const phys_vec3 &b1_lean_axis_loc); // 0xa18120

    void setup_constraint(pulse_sum_constraint_solver *psys, const float delta_t); // 0xa18e40
    void epilog_vel_constraint(const float delta_t);                              // 0xa180f0
};
