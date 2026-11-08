#pragma once

// Retail: codsrc\tl\physics\include\rbc_defs\rbc_def_vehicle.h (no BO1 assert references it).
// Out-of-line part: source/rbc_def_vehicle.cpp (TU 0xa19490-0xa1a2b5).
// SM3 shape; BO1 IDB layout. BO1 adds m_side_fric_max, m_pitch_stability_factor and splits
// m_turning_radius_ratio into _max_speed / _accel. rb_vehicle_model is in rb_vehicle_model.h.
// TREYBO: data members are public (SM3 private), the internal flag enum too.

#include "tl/base/include/tl_defs.h"
#include "tl/physics/include/phys_math.h"
#include "tl/physics/include/rigid_body.h"
#include "tl/physics/include/rbc_defs/rbc_def_base.h"
#include "tl/physics/include/constraint_solver/pulse_sum_cache.h"

class pulse_sum_normal;

// 0xa19550: move cur toward tgt at rate per second; snaps to tgt within 0.05 or when the step would overshoot.
float lerp_float(const float tgt, float cur, float rate, float delta_t);

// b1 = chassis, b2 = the body the wheel touches this frame (NULL when airborne).
class rigid_body_constraint_wheel : public rigid_body_constraint
{
public:
    enum wheel_state_e
    {
        WHEEL_STATE_ACCELERATING,
        WHEEL_STATE_BRAKING
    };

    enum internal_wheel_flags_e
    {
        WHEEL_FLAG_IS_COLLIDING = (1 << 0),
        WHEEL_FLAG_HARD_LIMIT_ACTIVE = (1 << 1),
        WHEEL_FLAG_IS_SLIDING = (1 << 2),
    };

    enum wheel_flags_e
    {
        WHEEL_FLAG_HAS_TURNING = (1 << 3),
        WHEEL_FLAG_HAS_POWER = (1 << 4),
        WHEEL_FLAG_HAS_POWER_BRAKING = (1 << 5),
        WHEEL_FLAG_HAS_BRAKING = (1 << 6)
    };

    enum ps_cache_e
    {
        PSC_HARD_LIMIT,
        PSC_SUSPENSION,
        PSC_SIDE_FRIC,
        PSC_FWD_FRIC,
        NUM_PSC
    };

    phys_vec3 m_b2_hitp_loc;
    phys_vec3 m_b2_hitn_loc;
    phys_vec3 m_b1_wheel_center_loc;
    phys_vec3 m_b1_suspension_dir_loc;
    phys_vec3 m_b1_wheel_axis_loc;
    float m_wheel_radius;
    float m_fwd_fric_k;
    float m_side_fric_k;
    float m_side_fric_max;
    float m_suspension_stiffness_k;
    float m_suspension_damp_k;
    float m_hard_limit_dist;
    float m_roll_stability_factor;
    float m_pitch_stability_factor;
    float m_turning_radius_ratio_max_speed;
    float m_turning_radius_ratio_accel;
    float m_desired_speed_k;
    float m_acceleration_factor_k;
    float m_braking_factor_k;
    float m_wheel_vel;
    float m_wheel_fwd;
    float m_wheel_pos;
    float m_wheel_displaced_center_dist;
    float m_wheel_normal_force;
    unsigned int m_wheel_state;
    unsigned int m_wheel_flags;
    pulse_sum_cache m_ps_cache_list[NUM_PSC];
    pulse_sum_normal *m_ps_suspension; // this frame's solver nodes, valid until the solver memory is reset
    pulse_sum_normal *m_ps_side_fric;
    pulse_sum_normal *m_ps_fwd_fric;

    void set_wheel_flag(internal_wheel_flags_e f, const unsigned int b) { b ? m_wheel_flags |= f : m_wheel_flags &= ~f; }
    const unsigned int get_wheel_flag(internal_wheel_flags_e f) const { return m_wheel_flags & f; }
    void set_wheel_flag(wheel_flags_e f, const unsigned int b) { b ? m_wheel_flags |= f : m_wheel_flags &= ~f; }
    const unsigned int get_wheel_flag(wheel_flags_e f) const { return m_wheel_flags & f; }

    void set(const phys_vec3 &wheel_center_loc,
             const phys_vec3 &suspension_dir_loc,
             const phys_vec3 &wheel_axis_loc, const float wheel_radius,
             const float fwd_fric_k, const float side_fric_k,
             const float suspension_stiffness_k, const float suspension_damp_k,
             const float hard_limit_dist, const float roll_stability_factor,
             const float pitch_stability_factor, const float side_fric_max); // 0xa195d0

    void set_wheel_radius(const float radius) { m_wheel_radius = radius; }
    void set_side_fric_k(const float side_fric) { m_side_fric_k = side_fric; }
    void set_fwd_fric_k(const float fwd_fric) { m_fwd_fric_k = fwd_fric; }
    void set_suspension_stiffness_k(const float suspension_stiffness) { m_suspension_stiffness_k = suspension_stiffness; }
    void set_suspension_damp_k(const float suspension_damp) { m_suspension_damp_k = suspension_damp; }
    void set_hard_limit_dist(const float hard_limit_dist) { m_hard_limit_dist = hard_limit_dist; }

    void get_wheel_collide_segment(const phys_mat44 &b1_mat, phys_vec3 *const p0, phys_vec3 *const p1) const; // 0xa196b0
    void set_no_collision();                                                                                // 0xa19500
    void set_collision(rigid_body *const rb, const phys_vec3 &hitp_loc, const phys_vec3 &hitn_loc);         // 0xa19510
    const unsigned int get_wheel_is_colliding() const { return get_wheel_flag(WHEEL_FLAG_IS_COLLIDING); }
    const unsigned int get_wheel_hard_limit_active() const { return get_wheel_flag(WHEEL_FLAG_HARD_LIMIT_ACTIVE); }

    void set_wheel_axis_loc(const phys_vec3 &wheel_axis_loc) { m_b1_wheel_axis_loc = wheel_axis_loc; }
    void set_roll_stability_factor(const float roll_stability_factor) { m_roll_stability_factor = roll_stability_factor; }
    void set_pitch_stability_factor(const float pitch_stability_factor) { m_pitch_stability_factor = pitch_stability_factor; } // TREYBO: BO1 field, SM3-style setter

    void set_wheel_state_accelerating(const float desired_speed_k, const float acceleration_factor_k); // 0xa19490
    void get_wheel_state_accelerating(float &desired_speed_k, float &acceleration_factor_k);          // 0xa194c0
    void set_wheel_state_braking(const float braking_factor_k);                                       // 0xa194e0

    rigid_body *get_chassis() { return b1; }
    const rigid_body *get_chassis() const { return b1; }
    const phys_vec3 get_wheel_center_loc() const { return m_b1_wheel_center_loc; }
    const phys_vec3 get_suspension_dir_loc() const { return m_b1_suspension_dir_loc; }
    const phys_vec3 get_wheel_axis_loc() const { return m_b1_wheel_axis_loc; }
    const float get_wheel_radius() const { return m_wheel_radius; }
    const float get_side_fric_k() const { return m_side_fric_k; }
    const float get_fwd_fric_k() const { return m_fwd_fric_k; }
    const float get_suspension_stiffness_k() const { return m_suspension_stiffness_k; }
    const float get_suspension_damp_k() const { return m_suspension_damp_k; }
    const float get_hard_limit_dist() const { return m_hard_limit_dist; }
    const float get_roll_stability_factor() const { return m_roll_stability_factor; }
    const phys_vec3 get_hitp_loc() const { return m_b2_hitp_loc; }
    const phys_vec3 get_hitn_loc() const { return m_b2_hitn_loc; }

    const float get_wheel_vel() const { return m_wheel_vel; }
    const float get_wheel_pos() const { return m_wheel_pos; }
    const phys_vec3 get_wheel_displaced_center_loc() const { return m_b1_wheel_center_loc - m_wheel_displaced_center_dist * m_b1_suspension_dir_loc; }
    const float get_wheel_normal_force() const { return m_wheel_normal_force; }
    const unsigned int get_wheel_is_sliding() const { return get_wheel_flag(WHEEL_FLAG_IS_SLIDING); }

    void setup_constraint(pulse_sum_constraint_solver *psys, const float delta_t); // 0xa19aa0
    void epilog_vel_constraint(const float delta_t);                              // 0xa197e0
};
