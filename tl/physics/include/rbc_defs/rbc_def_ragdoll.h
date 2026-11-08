#pragma once

// Retail: codsrc\tl\physics\include\rbc_defs\rbc_def_ragdoll.h (no BO1 assert references it).
// Out-of-line part: source/rbc_def_ragdoll.cpp (TU 0xa23a20-0xa249c5).
// SM3 shape; BO1 IDB layout. BO1 stores the joint limits as a plain array + count (SM3: phys_static_array).
// TREYBO: data members are public (SM3 private).

#include "tl/base/include/tl_defs.h"
#include "tl/physics/include/phys_math.h"
#include "tl/physics/include/rigid_body.h"
#include "tl/physics/include/rbc_defs/rbc_def_base.h"
#include "tl/physics/include/constraint_solver/pulse_sum_cache.h"

// A cone limit around a b1-local direction, applied to b2's axis.
class ragdoll_joint_limit_info
{
public:
    phys_vec3 m_b1_ud_loc;
    float m_b1_ud_limit_co_;
    float m_b1_ud_limit_si_;
    float m_b1_ud_active_limit_co_; // limit activates when cos(angle to b2 axis) drops to this

    void set(const phys_vec3 &b1_ud_loc, const float theta_limit); // 0xa23bf0
    void set_b1_ud_loc(const phys_vec3 &b1_ud_loc);                // 0xa23a20
    void set_theta_limit(const float theta_limit);                 // 0xa23cc0
};

class rigid_body_constraint_ragdoll : public rigid_body_constraint
{
public:
    enum
    {
        MAX_JOINT_LIMITS = 2 // retail assert text "m_joint_limits_count < MAX_JOINT_LIMITS"
    };

    // Bits 0..1 are the per-joint-limit active flags (SM3 set/get_joint_limit_active).
    enum flags_e
    {
        FLAG_HAS_HINGE = ((1 << 2) << 0),
        FLAG_HAS_SWIVEL = ((1 << 2) << 1),
        FLAG_MIN_LIMIT_ACTIVE = ((1 << 2) << 2),
        FLAG_MAX_LIMIT_ACTIVE = ((1 << 2) << 3),
        FLAG_HAS_DAMP = ((1 << 2) << 4),
        FLAG_FORCE_LIMITS_ACTIVE = ((1 << 2) << 5),
        FLAG_DISABLE_JOINT_LIMITS = ((1 << 2) << 6),
        FLAG_DAMP_TYPE_IMPLICIT = ((1 << 2) << 7),
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
        PSC_J_LIMIT0,
        PSC_J_LIMIT1,
        NUM_PSC
    };

    phys_vec3 m_b1_r_loc;
    phys_vec3 m_b2_r_loc;
    unsigned int m_flags;
    pulse_sum_cache m_ps_cache_list[NUM_PSC];
    phys_vec3 m_b1_axis_loc;
    phys_vec3 m_b2_axis_loc;
    phys_vec3 m_b1_a1_loc;
    phys_vec3 m_b1_a2_loc;
    phys_vec3 m_b1_ref_loc;
    phys_vec3 m_b2_ref_min_loc;
    phys_vec3 m_b2_ref_max_loc;
    ragdoll_joint_limit_info m_joint_limits[MAX_JOINT_LIMITS];
    int m_joint_limits_count;
    float m_damp_k;

    // inlined in phys_free_list<rigid_body_constraint_ragdoll>::add 0xa17010
    rigid_body_constraint_ragdoll() : m_flags(0), m_joint_limits_count(0) {}

    void set_flag(const unsigned int f, const bool b) { b ? m_flags |= f : m_flags &= ~f; }
    const unsigned int get_flag(const unsigned int f) const { return m_flags & f; }

    void set_joint_limit_active(const unsigned int f, const bool b)
    {
        PHYS_ASSERT_STR(f >= 0 && f < MAX_JOINT_LIMITS, "f >= 0 && f < MAX_JOINT_LIMITS");
        b ? m_flags |= (1 << f) : m_flags &= ~(1 << f);
    }
    const unsigned int get_joint_limit_active(const unsigned int f) const
    {
        PHYS_ASSERT_STR(f >= 0 && f < MAX_JOINT_LIMITS, "f >= 0 && f < MAX_JOINT_LIMITS");
        return m_flags & (1 << f);
    }

    void set(const phys_vec3 &b1_r_loc, const phys_vec3 &b2_r_loc); // 0xa23a70

    void set_snider_style(const phys_vec3 &b1_axis_loc, const phys_vec3 &b1_ref_loc); // 0xa23aa0

    void set_theta_min_max(const phys_vec3 &b2_ref_loc, const float theta_min, const float theta_max); // 0xa23d50

    void set_hinge(const phys_vec3 &b1_axis_loc, const phys_vec3 &b2_axis_loc,
                   const phys_vec3 &b1_ref_loc, const phys_vec3 &b2_ref_loc,
                   const float theta_min, const float theta_max); // 0xa23e30

    void set_swivel(const phys_vec3 &b1_axis_loc, const phys_vec3 &b2_axis_loc,
                    const phys_vec3 &b1_ref_loc, const phys_vec3 &b2_ref_loc,
                    const float theta_min, const float theta_max); // 0xa23ff0

    void add_joint_limit(const phys_vec3 &b1_ud_loc, const float theta_limit); // 0xa24130
    ragdoll_joint_limit_info *get_joint_limit(const int id) { return &m_joint_limits[id]; }
    void disable_joint_limits(const bool b) { set_flag(FLAG_DISABLE_JOINT_LIMITS, b); }

    void set_damp_k(const float damp_k); // 0xa23bc0
    void set_damp_type_motor() { set_flag(FLAG_DAMP_TYPE_IMPLICIT, false); }
    void set_damp_type_implicit() { set_flag(FLAG_DAMP_TYPE_IMPLICIT, true); }

    void set_force_limits_active(const bool b) { set_flag(FLAG_FORCE_LIMITS_ACTIVE, b); }
    const unsigned int get_force_limits_active() const { return get_flag(FLAG_FORCE_LIMITS_ACTIVE); }

    const phys_vec3 &get_b1_r_loc() const { return m_b1_r_loc; }
    const phys_vec3 &get_b2_r_loc() const { return m_b2_r_loc; }
    const phys_vec3 &get_b1_axis_loc() const { return m_b1_axis_loc; }
    const phys_vec3 &get_b2_axis_loc() const { return m_b2_axis_loc; }
    const phys_vec3 &get_b1_ref_loc() const { return m_b1_ref_loc; }

    const float pull_together(); // 0xa24190

    void setup_constraint(pulse_sum_constraint_solver *psys, const float delta_t); // 0xa244a0

    // public in BO1 (mangled QAE); SM3 had it private
    void setup_hinge(pulse_sum_constraint_solver *psys, const phys_vec3 &b1_ref, const phys_vec3 &b2_axis, const float delta_t); // 0xa242c0
};
