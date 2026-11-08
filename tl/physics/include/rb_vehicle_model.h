#pragma once

// Retail: codsrc\tl\physics\include\rb_vehicle_model.h. Header only: BO1 has no out-of-line rb_vehicle_model
// code (CoD drives its wheels from NitrousVehicle), so SM3's set / update_braking_and_acceleration /
// update_steering / get_wheel_matrix are not in the binary and are not declared here.
// Layout is the BO1 IDB layout (0x90): adds m_right_dir_loc / m_forward_dir_loc and m_rbc_upright.

#include "tl/base/include/tl_defs.h"
#include "tl/physics/include/phys_math.h"
#include "tl/physics/include/phys_array_base.inc"

class rigid_body_constraint_wheel;
class rigid_body_constraint_upright;

class rb_vehicle_model
{
public:
    enum rb_vehicle_model_flags_e
    {
        FLAG_IS_POWER_BRAKING = (1 << 0),
        FLAG_IS_BRAKING = (1 << 1),
        FLAG_IS_FORWARD_ACCELERATION = (1 << 2),
        FLAG_IS_REVERSE_ACCELERATION = (1 << 3),
        FLAG_IS_COASTING = (1 << 4)
    };

    // TREYBO: public (SM3 private); the host vehicle code owns these.
    phys_vec3 m_right_dir_loc;
    phys_vec3 m_forward_dir_loc;
    phys_static_array<rigid_body_constraint_wheel *, 4> m_wheels;
    float m_desired_speed_factor;
    float m_acceleration_factor;
    float m_power_braking_factor;
    float m_braking_factor;
    float m_coasting_factor;
    float m_reference_wheel_radius;
    float m_steer_factor;
    float m_steer_factor_offset;
    float m_steer_current_angle;
    float m_steer_max_angle;
    float m_steer_speed;
    phys_vec3 m_steer_front_pt_loc;
    float m_steer_front_back_length;
    unsigned int m_state_flags;
    rigid_body_constraint_upright *m_rbc_upright;

    void set_flag(rb_vehicle_model_flags_e f, const unsigned int b) { b ? m_state_flags |= f : m_state_flags &= ~f; }
    const unsigned int get_flag(rb_vehicle_model_flags_e f) const { return m_state_flags & f; }

    void add_wheel(rigid_body_constraint_wheel *rbc_wheel) { *m_wheels.add() = rbc_wheel; }
    rigid_body_constraint_wheel *get_wheel(const int i) { return m_wheels[i]; }
    const rigid_body_constraint_wheel *get_wheel(const int i) const { return m_wheels[i]; }
    void remove_wheels() { m_wheels.remove_all(); }

    void set_power_braking_factor(const float power_braking_factor) { m_power_braking_factor = power_braking_factor; }
    void set_braking_factor(const float braking_factor) { m_braking_factor = braking_factor; }
    void set_acceleration_factor(const float acceleration_factor) { m_acceleration_factor = acceleration_factor; }
    void set_desired_speed_factor(const float desired_speed_factor) { m_desired_speed_factor = desired_speed_factor; }
    void set_coasting_factor(const float coasting_factor) { m_coasting_factor = coasting_factor; }
    const float get_power_braking_factor() const { return m_power_braking_factor; }
    const float get_braking_factor() const { return m_braking_factor; }
    const float get_acceleration_factor() const { return m_acceleration_factor; }
    const float get_desired_speed_factor() const { return m_desired_speed_factor; }
    const float get_coasting_factor() const { return m_coasting_factor; }
    const phys_vec3 get_steer_front_pt_loc() const { return m_steer_front_pt_loc; }

    void set_power_braking(const unsigned int b) { set_flag(FLAG_IS_POWER_BRAKING, b); }
    void set_braking(const unsigned int b) { set_flag(FLAG_IS_BRAKING, b); }
    void set_forward_acceleration(const unsigned int b) { set_flag(FLAG_IS_FORWARD_ACCELERATION, b); }
    void set_reverse_acceleration(const unsigned int b) { set_flag(FLAG_IS_REVERSE_ACCELERATION, b); }
    void set_coasting(const unsigned int b) { set_flag(FLAG_IS_COASTING, b); }
    const unsigned int get_power_braking() const { return get_flag(FLAG_IS_POWER_BRAKING); }
    const unsigned int get_braking() const { return get_flag(FLAG_IS_BRAKING); }
    const unsigned int get_forward_acceleration() const { return get_flag(FLAG_IS_FORWARD_ACCELERATION); }
    const unsigned int get_reverse_acceleration() const { return get_flag(FLAG_IS_REVERSE_ACCELERATION); }
    const unsigned int get_coasting() const { return get_flag(FLAG_IS_COASTING); }

    void set_steer_factor(const float steer_factor) { m_steer_factor = steer_factor; }
    void set_steer_factor_offset(const float steer_factor_offset) { m_steer_factor_offset = steer_factor_offset; }
    void set_steer_max_angle(const float steer_max_angle) { m_steer_max_angle = steer_max_angle; }
    void set_steer_speed(const float steer_speed) { m_steer_speed = steer_speed; }
    const float get_steer_factor() const { return m_steer_factor; }
    const float get_steer_factor_offset() const { return m_steer_factor_offset; }
    const float get_steer_max_angle() const { return m_steer_max_angle; }
    const float get_steer_speed() const { return m_steer_speed; }
};
