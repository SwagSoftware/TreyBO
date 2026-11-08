// Retail: codsrc\tl\physics\source\rbc_def_ragdoll.cpp, TU 0xa23a20-0xa249c5.

#include "tl/physics/include/rbc_defs/rbc_def_ragdoll.h"
#include "tl/physics/include/constraint_solver/pulse_sum_constraint_solver.h"

// TREYBO: retail x86 sin/cos were the CRT double routines (__CIsin / __CIcos); call the double versions.
static inline float rbc_cosf(const float x) { return (float)cos((double)x); }
static inline float rbc_sinf(const float x) { return (float)sin((double)x); }

// ---------------------------------------------------------------------------------------------
// ragdoll_joint_limit_info
// ---------------------------------------------------------------------------------------------

// 0xa23a20
void ragdoll_joint_limit_info::set_b1_ud_loc(const phys_vec3 &b1_ud_loc)
{
    m_b1_ud_loc = phys_Unitize(b1_ud_loc);
    PHYS_ASSERT_UNIT(m_b1_ud_loc);
}

// 0xa23bf0
void ragdoll_joint_limit_info::set(const phys_vec3 &b1_ud_loc, const float theta_limit)
{
    PHYS_ASSERT(theta_limit > 0.0f);
    m_b1_ud_loc = phys_Unitize(b1_ud_loc);
    m_b1_ud_limit_co_ = rbc_cosf(theta_limit);
    m_b1_ud_limit_si_ = rbc_sinf(theta_limit);
    // Retail clamps (theta_limit - 2.5 deg) to <= 0 (fcom / test ah,41h, e.g. 0xa23d22), so any limit wider than
    // 2.5 deg gets an activation cosine of 1 and its row is set up whenever the cone edge is defined.
    float active_theta = theta_limit - PHYS_LIMIT_ACTIVE_ANGLE;
    if (active_theta > 0.0f)
        active_theta = 0.0f;
    m_b1_ud_active_limit_co_ = rbc_cosf(active_theta);
    PHYS_ASSERT_UNIT(m_b1_ud_loc);
}

// 0xa23cc0
void ragdoll_joint_limit_info::set_theta_limit(const float theta_limit)
{
    PHYS_ASSERT(theta_limit > 0.0f);
    m_b1_ud_limit_co_ = rbc_cosf(theta_limit);
    m_b1_ud_limit_si_ = rbc_sinf(theta_limit);
    float active_theta = theta_limit - PHYS_LIMIT_ACTIVE_ANGLE; // see set()
    if (active_theta > 0.0f)
        active_theta = 0.0f;
    m_b1_ud_active_limit_co_ = rbc_cosf(active_theta);
}

// ---------------------------------------------------------------------------------------------
// rigid_body_constraint_ragdoll
// ---------------------------------------------------------------------------------------------

// 0xa23a70
void rigid_body_constraint_ragdoll::set(const phys_vec3 &b1_r_loc, const phys_vec3 &b2_r_loc)
{
    m_b1_r_loc = b1_r_loc;
    m_b2_r_loc = b2_r_loc;
}

// 0xa23aa0
void rigid_body_constraint_ragdoll::set_snider_style(const phys_vec3 &b1_axis_loc, const phys_vec3 &b1_ref_loc)
{
    m_b1_axis_loc = phys_Unitize(b1_axis_loc);
    m_b1_ref_loc = phys_Unitize(b1_ref_loc);
    if (get_flag(FLAG_HAS_HINGE))
    {
        m_b1_a1_loc = construct_orth_ud(m_b1_axis_loc);
        m_b1_a2_loc = phys_cross(m_b1_axis_loc, m_b1_a1_loc);
    }
    PHYS_ASSERT_UNIT(m_b1_axis_loc);
    PHYS_ASSERT_UNIT(m_b1_ref_loc);
}

// 0xa23bc0
void rigid_body_constraint_ragdoll::set_damp_k(const float damp_k)
{
    m_damp_k = damp_k;
    set_flag(FLAG_HAS_DAMP, damp_k > 0.0f);
}

// 0xa23d50. theta_min / theta_max rotate b2_ref_loc about m_b2_axis_loc (right hand), as in the hinge.
void rigid_body_constraint_ragdoll::set_theta_min_max(const phys_vec3 &b2_ref_loc, const float theta_min, const float theta_max)
{
    phys_mat44 rot;
    make_rotate(&rot, m_b2_axis_loc, theta_min, 1000.0f);
    m_b2_ref_min_loc = phys_Unitize(phys_multiply(rot, b2_ref_loc));
    make_rotate(&rot, m_b2_axis_loc, theta_max, 1000.0f);
    m_b2_ref_max_loc = phys_Unitize(phys_multiply(rot, b2_ref_loc));
}

// 0xa23e30
void rigid_body_constraint_ragdoll::set_hinge(const phys_vec3 &b1_axis_loc, const phys_vec3 &b2_axis_loc,
                                              const phys_vec3 &b1_ref_loc, const phys_vec3 &b2_ref_loc,
                                              const float theta_min, const float theta_max)
{
    m_b1_axis_loc = phys_Unitize(b1_axis_loc);
    m_b2_axis_loc = phys_Unitize(b2_axis_loc);
    m_b1_ref_loc = phys_Unitize(b1_ref_loc);
    m_flags |= FLAG_HAS_HINGE;
    m_b1_a1_loc = construct_orth_ud(m_b1_axis_loc);
    m_b1_a2_loc = phys_cross(m_b1_axis_loc, m_b1_a1_loc);

    phys_mat44 rot;
    make_rotate(&rot, m_b2_axis_loc, theta_min, 1000.0f);
    m_b2_ref_min_loc = phys_Unitize(phys_multiply(rot, b2_ref_loc));
    make_rotate(&rot, m_b2_axis_loc, theta_max, 1000.0f);
    m_b2_ref_max_loc = phys_Unitize(phys_multiply(rot, b2_ref_loc));
}

// 0xa23ff0
void rigid_body_constraint_ragdoll::set_swivel(const phys_vec3 &b1_axis_loc, const phys_vec3 &b2_axis_loc,
                                               const phys_vec3 &b1_ref_loc, const phys_vec3 &b2_ref_loc,
                                               const float theta_min, const float theta_max)
{
    m_b1_axis_loc = phys_Unitize(b1_axis_loc);
    m_b2_axis_loc = phys_Unitize(b2_axis_loc);
    m_b1_ref_loc = phys_Unitize(b1_ref_loc);
    m_flags |= FLAG_HAS_SWIVEL;

    phys_mat44 rot;
    make_rotate(&rot, m_b2_axis_loc, theta_min, 1000.0f);
    m_b2_ref_min_loc = phys_Unitize(phys_multiply(rot, b2_ref_loc));
    make_rotate(&rot, m_b2_axis_loc, theta_max, 1000.0f);
    m_b2_ref_max_loc = phys_Unitize(phys_multiply(rot, b2_ref_loc));
}

// 0xa24130
void rigid_body_constraint_ragdoll::add_joint_limit(const phys_vec3 &b1_ud_loc, const float theta_limit)
{
    PHYS_ASSERT(m_joint_limits_count < MAX_JOINT_LIMITS);
    m_joint_limits[m_joint_limits_count].set(b1_ud_loc, theta_limit);
    ++m_joint_limits_count;
}

// 0xa24190: snap b2 so the two joint points coincide; returns the squared correction.
const float rigid_body_constraint_ragdoll::pull_together()
{
    const phys_vec3 b2_pt = phys_full_multiply(b2->m_mat, m_b2_r_loc);
    const phys_vec3 b1_pt = phys_full_multiply(b1->m_mat, m_b1_r_loc);
    const phys_vec3 delta = b1_pt - b2_pt;
    b2->m_mat.GetW() += delta;
    b2->debug_check_vec(b2->m_mat.GetW());
    return AbsSquared(delta);
}

// 0xa242c0. Same limit convention as rigid_body_constraint_hinge::setup_constraint: s = (b1_ref x b2_ref) . b2_axis;
// the min row is live while s >= -sin(2.5 deg) and pushes with -b2_axis, the max row while s <= +sin(2.5 deg).
void rigid_body_constraint_ragdoll::setup_hinge(pulse_sum_constraint_solver *psys, const phys_vec3 &b1_ref, const phys_vec3 &b2_axis, const float delta_t)
{
    const phys_vec3 b2_ref_min = phys_multiply(b2->m_mat, m_b2_ref_min_loc);
    const float si_min = phys_dot(b2_axis, phys_cross(b1_ref, b2_ref_min));
    if (si_min >= -PHYS_LIMIT_ACTIVE_SIN)
    {
        pulse_sum_angular *const psa = psys->create_pulse_sum_angular(b1, b1_ref, b2, b2_ref_min, -b2_axis, &m_ps_cache_list[PSC_MIN_LIMIT]);
        psa->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
        psa->m_pulse_sum_max = 0.0f;
        psa->setup_vel_uni_standard(delta_t, 5.0f);
    }

    const phys_vec3 b2_ref_max = phys_multiply(b2->m_mat, m_b2_ref_max_loc);
    const float si_max = phys_dot(b2_axis, phys_cross(b1_ref, b2_ref_max));
    if (si_max <= PHYS_LIMIT_ACTIVE_SIN)
    {
        pulse_sum_angular *const psa = psys->create_pulse_sum_angular(b1, b1_ref, b2, b2_ref_max, b2_axis, &m_ps_cache_list[PSC_MAX_LIMIT]);
        psa->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
        psa->m_pulse_sum_max = 0.0f;
        psa->setup_vel_uni_standard(delta_t, 5.0f);
    }
}

// 0xa244a0
void rigid_body_constraint_ragdoll::setup_constraint(pulse_sum_constraint_solver *psys, const float delta_t)
{
    {
        const phys_vec3 b2_r = phys_multiply(b2->m_mat, m_b2_r_loc);
        const phys_vec3 b1_r = phys_multiply(b1->m_mat, m_b1_r_loc);
        psys->create_point(b1, b1_r, b2, b2_r, m_ps_cache_list, delta_t, false, 0.0f, 0.0f);
    }

    const phys_vec3 b1_axis = phys_multiply(b1->m_mat, m_b1_axis_loc);
    const phys_vec3 b2_axis = phys_multiply(b2->m_mat, m_b2_axis_loc);

    if (get_flag(FLAG_HAS_DAMP))
    {
        // damp the relative spin about its own axis (b2's axis when there is no relative spin)
        phys_vec3 axis = b2->m_a_vel - b1->m_a_vel;
        float naxis = Abs(axis);
        if (!(naxis > PHYS_EPS_1E4)) // TREYBO FIX(0xa245ab): `test ah,41h; jnz` takes b2's axis for a NaN spin too
        {
            axis = b2_axis;
        }
        else
        {
            naxis = 1.0f / naxis;
            axis = naxis * axis;
        }
        pulse_sum_angular *const psa = psys->create_pulse_sum_angular(b1, PHYS_ZERO_VEC, b2, PHYS_ZERO_VEC, axis, &m_ps_cache_list[PSC_DAMP]);
        if (get_flag(FLAG_DAMP_TYPE_IMPLICIT))
        {
            const float cfm = 1.0f / (m_damp_k * delta_t);
            psa->m_right_side = 0.0f;
            psa->m_big_dirt = 0.0f;
            psa->m_cfm = cfm;
            psa->m_denom = cfm + psa->m_denom;
            psa->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
            psa->m_pulse_sum_max = PHYS_PULSE_SUM_UNBOUNDED;
        }
        else
        {
            psa->m_right_side = 0.0f;
            psa->m_big_dirt = 0.0f;
            psa->m_cfm = 0.0f;
            const float damp = m_damp_k * delta_t;
            psa->m_pulse_sum_min = -damp;
            psa->m_pulse_sum_max = damp;
        }
    }

    if (get_flag(FLAG_HAS_HINGE))
    {
        const phys_vec3 a2 = phys_multiply(b1->m_mat, m_b1_a2_loc);
        const phys_vec3 a1 = phys_multiply(b1->m_mat, m_b1_a1_loc);
        psys->create_hinge(b1, b1_axis, b2, b2_axis, a1, a2, &m_ps_cache_list[PSC_A1], delta_t);
        const phys_vec3 b1_ref = phys_multiply(b1->m_mat, m_b1_ref_loc);
        setup_hinge(psys, b1_ref, b2_axis, delta_t);
    }

    if (get_flag(FLAG_HAS_SWIVEL))
    {
        if (!get_flag(FLAG_DISABLE_JOINT_LIMITS))
        {
            for (int i = 0; i < m_joint_limits_count; i++)
            {
                const ragdoll_joint_limit_info &jl = m_joint_limits[i];
                const phys_vec3 b1_ud = phys_multiply(b1->m_mat, jl.m_b1_ud_loc);
                const float co_ = phys_dot(b1_ud, b2_axis);
                if (jl.m_b1_ud_active_limit_co_ >= co_)
                {
                    // the cone edge nearest b2's axis, and the row axis perpendicular to it
                    const phys_vec3 b1_perp = b2_axis - b1_ud * co_;
                    const float nb1_perp = Abs(b1_perp);
                    if (!(nb1_perp < PHYS_EPS_1E4)) // TREYBO FIX(0xa247fa): `test ah,5; jnp` skips only an ordered "<"
                    {
                        const phys_vec3 b1_limit = b1_perp * (jl.m_b1_ud_limit_si_ / nb1_perp) + jl.m_b1_ud_limit_co_ * b1_ud;
                        const phys_vec3 ud = phys_cross(b1_limit, b1_ud) * (1.0f / jl.m_b1_ud_limit_si_);
                        PHYS_ASSERT_UNIT(ud);
                        pulse_sum_angular *const psa = psys->create_pulse_sum_angular(b1, b1_limit, b2, b2_axis, ud, &m_ps_cache_list[PSC_J_LIMIT0 + i]);
                        psa->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
                        psa->m_pulse_sum_max = 0.0f;
                        psa->setup_vel_uni_standard(delta_t, 5.0f);
                    }
                }
            }
        }
        phys_mat44 rot;
        make_rotate(&rot, b1_axis, b2_axis);
        const phys_vec3 b1_ref = phys_multiply(rot, phys_multiply(b1->m_mat, m_b1_ref_loc));
        setup_hinge(psys, b1_ref, b2_axis, delta_t);
    }
}
