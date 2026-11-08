// Retail: codsrc\tl\physics\source\rbc_def_generic.cpp, TU ~0xa179e0-0xa18f08
// (point, distance, angular_actuator, upright, hinge; the pulse_sum_* inlines after 0xa18f10 are solver code).

#include "tl/physics/include/rbc_defs/rbc_def_generic.h"
#include "tl/physics/include/rigid_body_internal.h"
#include "tl/physics/include/phys_util.h"
#include "tl/physics/include/constraint_solver/pulse_sum_constraint_solver.h"

// ---------------------------------------------------------------------------------------------
// rigid_body_constraint_point
// ---------------------------------------------------------------------------------------------

// 0xa179e0
void rigid_body_constraint_point::set(const phys_vec3 &b1_r_loc, const phys_vec3 &b2_r_loc)
{
    m_b1_r_loc = b1_r_loc;
    m_b2_r_loc = b2_r_loc;
}

// 0xa17a10
// TREYBO FIX(0xa17a10): kisakblack named this epilog_vel_constaint, so the solver never called it; retail
// mangles ?epilog_vel_constraint@rigid_body_constraint_point@@QAEXM@Z.
void rigid_body_constraint_point::epilog_vel_constraint(const float)
{
    m_stress = m_ps_cache_list[PSC_X].m_pulse_sum * m_ps_cache_list[PSC_X].m_pulse_sum
             + m_ps_cache_list[PSC_Y].m_pulse_sum * m_ps_cache_list[PSC_Y].m_pulse_sum
             + m_ps_cache_list[PSC_Z].m_pulse_sum * m_ps_cache_list[PSC_Z].m_pulse_sum;
}

// 0xa18370
void rigid_body_constraint_point::setup_constraint(pulse_sum_constraint_solver *psys, const float delta_t)
{
    const phys_vec3 b2_r = phys_multiply(b2->m_mat, m_b2_r_loc);
    const phys_vec3 b1_r = phys_multiply(b1->m_mat, m_b1_r_loc);
    psys->create_point(b1, b1_r, b2, b2_r, m_ps_cache_list, delta_t, m_spring_enabled, m_spring_k, m_damp_k);
}

// ---------------------------------------------------------------------------------------------
// rigid_body_constraint_distance
// ---------------------------------------------------------------------------------------------

// 0xa17a50
void rigid_body_constraint_distance::set(const phys_vec3 &b1_r_loc, const phys_vec3 &b2_r_loc,
                                         const float min_distance, const float max_distance)
{
    m_b1_r_loc = b1_r_loc;
    m_b2_r_loc = b2_r_loc;
    m_flags = FLAG_ENABLE;
    m_min_distance = min_distance;
    m_max_distance = max_distance;
    m_next_max_distance = max_distance;
    m_max_distance_vel = 0.0f;
    m_damp_coef = 0.0f;
}

// 0xa17aa0
void rigid_body_constraint_distance::outer_prolog_update(const float delta_t)
{
    m_max_distance_vel = (m_next_max_distance - m_max_distance) / delta_t;
}

// 0xa17ac0
void rigid_body_constraint_distance::inner_update(const float delta_t)
{
    m_max_distance = m_max_distance_vel * delta_t + m_max_distance;
}

// 0xa17ae0
void rigid_body_constraint_distance::outer_epilog_update(const float)
{
    m_max_distance = m_next_max_distance;
}

// 0xa187b0. Rope-like: the max limit pulls b2 toward b1 (normal -ud), the optional min limit pushes it away
// (normal +ud). Near the max length an optional damper resists the sideways (perpendicular) relative velocity.
void rigid_body_constraint_distance::setup_constraint(pulse_sum_constraint_solver *psys, const float delta_t)
{
    if (!get_flag(FLAG_ENABLE))
        return;

    PHYS_ASSERT(m_min_distance >= 0.0f);
    PHYS_ASSERT(m_min_distance <= m_max_distance);

    const phys_vec3 b1_r = phys_multiply(b1->m_mat, m_b1_r_loc);
    const phys_vec3 b2_r = phys_multiply(b2->m_mat, m_b2_r_loc);
    const phys_vec3 b1_pt = b1->m_mat.GetW() + b1_r;
    const phys_vec3 b2_pt = b2->m_mat.GetW() + b2_r;
    const phys_vec3 dir = b2_pt - b1_pt;
    const float ndir_sq = AbsSquared(dir);
    // TREYBO FIX(0xa188da): `fcom; test ah,5; jnp -> return` returns only for an ordered "<"; a NaN length goes on.
    if (ndir_sq < 1.0000001e-06f) // phys_sqr(.001f), folded
        return;

    const float oo_ndir = 1.0f / sqrtf(ndir_sq);
    const phys_vec3 ud = dir * oo_ndir;

    pulse_sum_normal *const ps_max = psys->create_pulse_sum_normal();
    ps_max->set(b1, b1_r, b2, b2_r, -ud, &m_ps_cache_list[PSC_MAX_DIST], PHYS_ZERO_VEC);
    ps_max->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
    ps_max->m_pulse_sum_max = 0.0f;
    ps_max->setup_vel_uni_standard_pos_adjust(delta_t, -m_max_distance, 1700.0f);
    ps_max->m_right_side = m_max_distance_vel + ps_max->m_right_side;

    if (m_min_distance > 0.01f)
    {
        pulse_sum_normal *const ps_min = psys->create_pulse_sum_normal();
        ps_min->set(b1, b1_r, b2, b2_r, ud, &m_ps_cache_list[PSC_MIN_DIST], PHYS_ZERO_VEC);
        ps_min->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
        ps_min->m_pulse_sum_max = 0.0f;
        ps_min->setup_vel_uni_standard_pos_adjust(delta_t, m_min_distance, 1700.0f);
    }

    if (m_damp_coef > PHYS_EPS_1E5)
    {
        const float damp_min = m_max_distance - 0.1f;
        if (damp_min * damp_min <= ndir_sq)
        {
            const float damp_max = m_max_distance + 0.1f;
            if (damp_max * damp_max >= ndir_sq)
            {
                phys_vec3 damp_dir = ps_max->get_relative_velocity();
                damp_dir -= phys_dot(damp_dir, ud) * ud;
                float ndamp_dir = Abs(damp_dir);
                if (ndamp_dir < PHYS_EPS_1E4)
                {
                    damp_dir = ps_max->get_relative_velocity_change_dir();
                    damp_dir -= phys_dot(damp_dir, ud) * ud;
                    ndamp_dir = Abs(damp_dir);
                }
                if (ndamp_dir > PHYS_EPS_1E4)
                {
                    damp_dir *= 1.0f / ndamp_dir;
                    pulse_sum_normal *const ps_damp = psys->create_pulse_sum_normal();
                    phys_vec3 r1;
                    phys_vec3 r2;
                    if (get_flag(FLAG_B2_R_IS_DAMP_POINT))
                    {
                        r1 = rbint::sub_pos(b1, b2_pt);
                        r2 = b2_r;
                    }
                    else
                    {
                        r1 = b1_r;
                        r2 = rbint::sub_pos(b2, b1_pt);
                    }
                    ps_damp->set(b1, r1, b2, r2, damp_dir, &m_ps_cache_list[PSC_DAMP], PHYS_ZERO_VEC);
                    const float cfm = 1.0f / (m_damp_coef * delta_t);
                    ps_damp->m_right_side = 0.0f;
                    ps_damp->m_big_dirt = 0.0f;
                    ps_damp->m_cfm = cfm;
                    ps_damp->m_denom = cfm + ps_damp->m_denom;
                    ps_damp->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
                    ps_damp->m_pulse_sum_max = PHYS_PULSE_SUM_UNBOUNDED;
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// rigid_body_constraint_angular_actuator
// ---------------------------------------------------------------------------------------------

// 0xa17af0
void rigid_body_constraint_angular_actuator::set(float power, const phys_mat44 &target_mat)
{
    m_power = power;
    m_power_scale = 1.0f;
    m_target_mat = target_mat;
    m_next_target_mat = target_mat;
    m_a_vel = phys_vec3(0.0f, 0.0f, 0.0f);
    m_enabled = true;
    PHYS_ASSERT_ORTHONORMAL(m_target_mat);
}

// 0xa17be0: target angular velocity that carries m_target_mat to m_next_target_mat over the outer step.
void rigid_body_constraint_angular_actuator::outer_prolog_update(const float delta_t)
{
    m_target_mat.SetW(phys_vec3(0.0f, 0.0f, 0.0f));
    m_next_target_mat.SetW(phys_vec3(0.0f, 0.0f, 0.0f));
    phys_vec3 t_vel;
    nuge::calc_velocities(m_target_mat, m_next_target_mat, delta_t, &t_vel, &m_a_vel);
}

// 0xa18090
void rigid_body_constraint_angular_actuator::inner_update(const float delta_t)
{
    phys_mat44 rot;
    make_rotate(&rot, m_a_vel, delta_t, 1000.0f);
    phys_multiply_mat(m_target_mat, rot, m_target_mat);
}

// 0xa17c50
void rigid_body_constraint_angular_actuator::outer_epilog_update(const float)
{
    m_target_mat = m_next_target_mat;
}

// 0xa18c40: three angular rows drive b2's axes toward b1 * m_target_mat, each capped at m_power * dt * scale.
void rigid_body_constraint_angular_actuator::setup_constraint(pulse_sum_constraint_solver *psys, const float delta_t)
{
    if (!m_enabled)
        return;

    const phys_mat44 &b2_mat = b2->m_mat;
    const float ps_limit = m_power * delta_t * m_power_scale;
    phys_mat44 target;
    phys_multiply_mat(target, b1->m_mat, m_target_mat);
    const phys_vec3 a_vel = phys_multiply(b1->m_mat, m_a_vel);
    const float rate = -1.0f / delta_t;

    pulse_sum_angular *const psa_x = psys->create_pulse_sum_angular(b1, target.GetX(), b2, b2_mat.GetX(), target.GetY(), &m_ps_cache_list[PSC_X]);
    psa_x->m_pulse_sum_min = -ps_limit;
    psa_x->m_pulse_sum_max = ps_limit;
    psa_x->m_big_dirt = psa_x->get_pos() * rate;
    psa_x->m_right_side = 0.0f;
    psa_x->m_cfm = 0.0f;
    psa_x->m_denom = psa_x->m_denom + 0.0f;
    psa_x->m_right_side -= phys_dot(target.GetY(), a_vel);

    pulse_sum_angular *const psa_y = psys->create_pulse_sum_angular(b1, target.GetY(), b2, b2_mat.GetY(), target.GetZ(), &m_ps_cache_list[PSC_Y]);
    psa_y->m_pulse_sum_min = -ps_limit;
    psa_y->m_pulse_sum_max = ps_limit;
    psa_y->m_big_dirt = psa_y->get_pos() * rate;
    psa_y->m_right_side = 0.0f;
    psa_y->m_cfm = 0.0f;
    psa_y->m_denom = psa_y->m_denom + 0.0f;
    psa_y->m_right_side -= phys_dot(target.GetZ(), a_vel);

    pulse_sum_angular *const psa_z = psys->create_pulse_sum_angular(b1, target.GetZ(), b2, b2_mat.GetZ(), target.GetX(), &m_ps_cache_list[PSC_Z]);
    psa_z->m_pulse_sum_min = -ps_limit;
    psa_z->m_pulse_sum_max = ps_limit;
    psa_z->m_big_dirt = psa_z->get_pos() * rate;
    psa_z->m_right_side = 0.0f;
    psa_z->m_cfm = 0.0f;
    psa_z->m_denom = psa_z->m_denom + 0.0f;
    psa_z->m_right_side -= phys_dot(target.GetX(), a_vel);
}

// ---------------------------------------------------------------------------------------------
// rigid_body_constraint_upright
// ---------------------------------------------------------------------------------------------

// 0xa17cb0: lean of b1 around its forward axis, measured from b2's up axis.
// TREYBO: retail x86 atan2 was the CRT double routine (__CIatan2); call the double version.
const float rigid_body_constraint_upright::calc_current_lean_angle() const
{
    const phys_vec3 b2_up_axis = phys_inv_multiply(b1->m_mat, m_b2_up_axis_loc);
    const float up = phys_dot(m_b1_up_axis_loc, b2_up_axis);
    const float right = -phys_dot(b2_up_axis, m_b1_right_axis_loc);
    // TREYBO FIX(0xa17d73): `fcomp; test ah,41h; jnz -> 0` also returns 0 for a NaN length ("<=" called atan2).
    if (!(right * right + up * up > 9.999999e-09f)) // phys_sqr(PHYS_EPS_1E4), folded
        return 0.0f;
    return (float)atan2((double)right, (double)up);
}

// 0xa17da0
// TREYBO: retail sin/cos were the CRT double routines (__CIsin / __CIcos); call the double versions.
const phys_vec3 rigid_body_constraint_upright::calc_b1_lean_axis_loc(const float lean_angle) const
{
    const float si_ = (float)sin((double)lean_angle);
    const float co_ = (float)cos((double)lean_angle);
    return m_b1_up_axis_loc * co_ - m_b1_right_axis_loc * si_;
}

// 0xa180f0
void rigid_body_constraint_upright::epilog_vel_constraint(const float delta_t)
{
    if (m_enabled)
    {
        m_lean_angle = calc_current_lean_angle();
        m_lean_angle_calc_delta_t = m_lean_angle_calc_delta_t + delta_t;
    }
}

// 0xa18120: store the new lean axis and rotate b1 about b1_lean_center so that axis lines up with b2's up
// axis projected off b1's forward axis.
void rigid_body_constraint_upright::update_lean_axis(const phys_vec3 &b1_lean_center, const phys_vec3 &b1_lean_axis_loc)
{
    m_b1_lean_axis_loc = b1_lean_axis_loc;
    if (!m_enabled)
        return;

    const phys_mat44 &b1_mat = b1->m_mat;
    const phys_vec3 b1_forward_axis = phys_multiply(b1_mat, m_b1_forward_axis_loc);
    phys_vec3 b2_up_axis = m_b2_up_axis_loc;
    b2_up_axis -= b1_forward_axis * phys_dot(b1_forward_axis, b2_up_axis);
    float nb2_up_axis = Abs(b2_up_axis);
    if (nb2_up_axis > PHYS_EPS_1E5)
    {
        nb2_up_axis = 1.0f / nb2_up_axis;
        b2_up_axis *= nb2_up_axis;
        const phys_vec3 b1_lean_axis = phys_multiply(b1_mat, m_b1_lean_axis_loc);
        phys_mat44 lean_mat;
        make_rotate(&lean_mat, b1_lean_axis, b2_up_axis);
        lean_mat.SetW(b1_lean_center - phys_multiply(lean_mat, b1_lean_center));
        phys_full_multiply_mat(b1->m_mat, lean_mat, b1->m_mat);
        b1->debug_check_vec(b1->m_mat.GetW());
    }
}

// 0xa18e40
void rigid_body_constraint_upright::setup_constraint(pulse_sum_constraint_solver *psys, const float delta_t)
{
    if (!m_enabled)
        return;

    const phys_vec3 b1_forward_axis = phys_multiply(b1->m_mat, m_b1_forward_axis_loc);
    const phys_vec3 b1_lean_axis = phys_multiply(b1->m_mat, m_b1_lean_axis_loc);
    pulse_sum_angular *const psa = psys->create_pulse_sum_angular(b1, b1_lean_axis, b2, m_b2_up_axis_loc, b1_forward_axis, &m_ps_cache_list[0]);
    psa->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
    psa->m_pulse_sum_max = PHYS_PULSE_SUM_UNBOUNDED;
    const float dt = delta_t > MIN_NERP_DELTA_T ? delta_t : MIN_NERP_DELTA_T;
    psa->m_big_dirt = psa->get_pos() * (-0.5f / dt);
    psa->m_right_side = 0.0f;
    psa->m_cfm = 0.0f;
}

// ---------------------------------------------------------------------------------------------
// rigid_body_constraint_hinge
// ---------------------------------------------------------------------------------------------

// 0xa17e40. theta_min / theta_max rotate b2_ref_loc about b2's axis (right hand) to the two limit references.
void rigid_body_constraint_hinge::set(const phys_vec3 &b1_r_loc, const phys_vec3 &b2_r_loc,
                                      const phys_vec3 &b1_axis_loc, const phys_vec3 &b2_axis_loc,
                                      const phys_vec3 &b1_ref_loc, const phys_vec3 &b2_ref_loc,
                                      const float theta_min, const float theta_max, const float damp_k)
{
    m_flags = 0;
    m_b1_r_loc = b1_r_loc;
    m_b2_r_loc = b2_r_loc;
    m_b1_axis_loc = phys_Unitize(b1_axis_loc);
    m_b2_axis_loc = phys_Unitize(b2_axis_loc);
    m_b1_ref_loc = phys_Unitize(b1_ref_loc);
    m_b1_a1_loc = construct_orth_ud(m_b1_axis_loc);
    m_b1_a2_loc = phys_cross(m_b1_axis_loc, m_b1_a1_loc);

    phys_mat44 rot;
    make_rotate(&rot, m_b2_axis_loc, theta_min, 1000.0f);
    m_b2_ref_min_loc = phys_Unitize(phys_multiply(rot, b2_ref_loc));
    make_rotate(&rot, m_b2_axis_loc, theta_max, 1000.0f);
    m_b2_ref_max_loc = phys_Unitize(phys_multiply(rot, b2_ref_loc));
    m_damp_k = damp_k;

    PHYS_ASSERT_UNIT(m_b1_axis_loc);
    PHYS_ASSERT_UNIT(m_b2_axis_loc);
    PHYS_ASSERT_UNIT(m_b1_ref_loc);
    PHYS_ASSERT_UNIT(m_b1_a1_loc);
    PHYS_ASSERT_UNIT(m_b1_a2_loc);
    PHYS_ASSERT_UNIT(m_b2_ref_min_loc);
    PHYS_ASSERT_UNIT(m_b2_ref_max_loc);
    PHYS_ASSERT_ORTHOGONAL(m_b1_axis_loc, m_b1_ref_loc);
    PHYS_ASSERT_ORTHOGONAL(m_b1_a1_loc, m_b1_a2_loc);
}

// 0xa18400
// Limit sign convention: s = (b1_ref x b2_ref_limit) . b2_axis is the signed sine from b1_ref to the limit
// reference about b2's axis. The min limit is live while s >= -sin(2.5 deg) (b1_ref within 2.5 deg past it, or
// beyond) and pushes with -b2_axis; the max limit is live while s <= +sin(2.5 deg) and pushes with +b2_axis.
void rigid_body_constraint_hinge::setup_constraint(pulse_sum_constraint_solver *psys, const float delta_t)
{
    if (!get_flag(HINGE_FLAG_NO_POINT))
    {
        const phys_vec3 b2_r = phys_multiply(b2->m_mat, m_b2_r_loc);
        const phys_vec3 b1_r = phys_multiply(b1->m_mat, m_b1_r_loc);
        psys->create_point(b1, b1_r, b2, b2_r, m_ps_cache, delta_t, false, 0.0f, 0.0f);
    }

    const phys_mat44 &b1_mat = b1->m_mat;
    const phys_vec3 b1_axis = phys_multiply(b1_mat, m_b1_axis_loc);
    const phys_vec3 b2_axis = phys_multiply(b2->m_mat, m_b2_axis_loc);
    const phys_vec3 a2 = phys_multiply(b1_mat, m_b1_a2_loc);
    const phys_vec3 a1 = phys_multiply(b1_mat, m_b1_a1_loc);
    psys->create_hinge(b1, b1_axis, b2, b2_axis, a1, a2, &m_ps_cache[PSC_A1], delta_t);

    if (m_damp_k > PHYS_EPS_1E5)
    {
        pulse_sum_angular *const psa = psys->create_pulse_sum_angular(b1, PHYS_ZERO_VEC, b2, PHYS_ZERO_VEC, b2_axis, &m_ps_cache[PSC_DAMP]);
        psa->m_right_side = 0.0f;
        psa->m_big_dirt = 0.0f;
        psa->m_cfm = 0.0f;
        const float damp = delta_t * m_damp_k;
        psa->m_pulse_sum_min = -damp;
        psa->m_pulse_sum_max = damp;
    }

    const phys_vec3 b1_ref = phys_multiply(b1->m_mat, m_b1_ref_loc);
    const phys_mat44 &b2_mat = b2->m_mat;
    const phys_vec3 b2_ref_min = phys_multiply(b2_mat, m_b2_ref_min_loc);
    const phys_vec3 b2_ref_max = phys_multiply(b2_mat, m_b2_ref_max_loc);

    if (get_flag(HINGE_FLAG_CLOSEST_LIMIT_ONLY))
    {
        const float co_min = phys_dot(b1_ref, b2_ref_min);
        const float co_max = phys_dot(b1_ref, b2_ref_max);
        if (!(co_max <= co_min)) // TREYBO FIX(0xa185df): `fcompp; test ah,41h; jp` picks the max limit on NaN
        {
            m_flags &= ~(HINGE_MIN_LIMIT_ACTIVE | HINGE_MAX_LIMIT_ACTIVE);
            m_flags |= HINGE_MAX_LIMIT_ACTIVE;
        }
        else
        {
            m_flags |= HINGE_MIN_LIMIT_ACTIVE;
            m_flags &= ~HINGE_MAX_LIMIT_ACTIVE;
        }
    }
    else
    {
        const float si_min = phys_dot(b2_axis, phys_cross(b1_ref, b2_ref_min));
        // TREYBO FIX(0xa18664 / 0xa186c7): `test ah,1; jnz` and `test ah,41h; jp` clear the limit on NaN.
        if (!(si_min >= -PHYS_LIMIT_ACTIVE_SIN))
            m_flags &= ~HINGE_MIN_LIMIT_ACTIVE;
        else
            m_flags |= HINGE_MIN_LIMIT_ACTIVE;
        const float si_max = phys_dot(b2_axis, phys_cross(b1_ref, b2_ref_max));
        if (!(si_max <= PHYS_LIMIT_ACTIVE_SIN))
            m_flags &= ~HINGE_MAX_LIMIT_ACTIVE;
        else
            m_flags |= HINGE_MAX_LIMIT_ACTIVE;
    }

    if (get_flag(HINGE_MIN_LIMIT_ACTIVE))
    {
        pulse_sum_angular *const psa = psys->create_pulse_sum_angular(b1, b1_ref, b2, b2_ref_min, -b2_axis, &m_ps_cache[PSC_MIN_LIMIT]);
        psa->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
        psa->m_pulse_sum_max = 0.0f;
        psa->setup_vel_uni_standard(delta_t, 5.0f);
    }
    if (get_flag(HINGE_MAX_LIMIT_ACTIVE))
    {
        pulse_sum_angular *const psa = psys->create_pulse_sum_angular(b1, b1_ref, b2, b2_ref_max, b2_axis, &m_ps_cache[PSC_MAX_LIMIT]);
        psa->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
        psa->m_pulse_sum_max = 0.0f;
        psa->setup_vel_uni_standard(delta_t, 5.0f);
    }
}
