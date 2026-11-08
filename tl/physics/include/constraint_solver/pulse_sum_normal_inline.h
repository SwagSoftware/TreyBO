#pragma once

// Retail: codsrc\tl\physics\include\constraint_solver\pulse_sum_normal_inline.h (asserts :11, :56, :57, :61, :90,
// :117-123). Functions without asserts are placed here too (their retail header is not recoverable).
// Out-of-line copies: rbc_def_generic TU (0xa18f10), rbc_def_vehicle TU (0xa1a2c0), solver TU (0xa26150-0xa28440).

#include "tl/physics/include/constraint_solver/pulse_sum_constraint_solver.h"

// 0xa26150. Effective inverse mass of the row. b1's angular term uses the displaced point (the vehicle
// roll / pitch stability offset), but the denominator crosses back with the undisplaced m_b1_r (retail).
inline void pulse_sum_normal::calc_abs(const phys_vec3 &b1_r_displace)
{
    const phys_vec3 b1_r = m_b1_r + b1_r_displace;
    m_b1_ap = phys_multiply(m_b1->m_world_inv_inertia, phys_cross(b1_r, m_ud));
    m_denom = phys_dot(m_ud, phys_cross(m_b1_ap, m_b1_r)) + m_b1->m_inv_mass;
    if (m_b2)
    {
        const phys_vec3 b2_r_x_ud = phys_cross(m_b2_r, m_ud);
        m_b2_ap = phys_multiply(m_b2->m_world_inv_inertia, b2_r_x_ud);
        m_denom = phys_dot(m_b2_ap, b2_r_x_ud) + m_b2->m_inv_mass + m_denom;
    }
    PHYS_ASSERT(m_denom > 0.00001f);
}

// 0xa26320. Start-of-step relative velocity along m_ud, from the rigid bodies' own velocities.
inline const float pulse_sum_normal::get_vel()
{
    const phys_vec3 b2_vel = m_b2 ? rbint::gtv(m_b2->m_rb, m_b2_r) : m_b2_ap;
    const phys_vec3 b1_vel = rbint::gtv(m_b1->m_rb, m_b1_r);
    return phys_dot(m_ud, b1_vel - b2_vel);
}

// 0xa26470. As get_vel, with last frame's velocities.
inline const float pulse_sum_normal::get_last_vel()
{
    phys_vec3 b2_vel;
    if (m_b2)
    {
        const rigid_body *const rb2 = m_b2->m_rb;
        b2_vel = rb2->m_last_t_vel + phys_cross(rb2->m_last_a_vel, m_b2_r);
    }
    else
    {
        b2_vel = m_b2_ap;
    }
    const rigid_body *const rb1 = m_b1->m_rb;
    const phys_vec3 b1_vel = rb1->m_last_t_vel + phys_cross(rb1->m_last_a_vel, m_b1_r);
    return phys_dot(m_ud, b1_vel - b2_vel);
}

// 0xa265c0. Separation of the two row points along m_ud.
inline const float pulse_sum_normal::get_pos()
{
    const phys_vec3 b2_pt = m_b2 ? m_b2->m_rb->m_mat.GetW() + m_b2_r : m_b2_r;
    const phys_vec3 b1_pt = m_b1->m_rb->m_mat.GetW() + m_b1_r;
    return phys_dot(m_ud, b1_pt - b2_pt);
}

// 0xa26670
inline void pulse_sum_normal::set(rigid_body *const b1, const phys_vec3 &b1_r, rigid_body *const b2, const phys_vec3 &b2_r,
                                  const phys_vec3 &ud, pulse_sum_cache *const ps_cache, const phys_vec3 &b1_r_displace)
{
    PHYS_ASSERT(ps_cache);
    PHYS_ASSERT(b1);
    PHYS_ASSERT(b2);
    PHYS_ASSERT(b1 != b2);
    PHYS_ASSERT_UNIT(ud);
    PHYS_ASSERT_STR(rbint_get_pulse_sum_node(b1) || rbint_get_pulse_sum_node(b2), "rbint::get_pulse_sum_node(b1) || rbint::get_pulse_sum_node(b2)");
    PHYS_ASSERT_STR(rbint_verify_pulse_sum_node(b1) && rbint_verify_pulse_sum_node(b2), "rbint::verify_pulse_sum_node(b1) && rbint::verify_pulse_sum_node(b2)");

    if (b1->m_node)
    {
        m_b1 = b1->m_node;
        m_b1_r = b1_r;
        if (b2->m_node)
        {
            m_b2 = b2->m_node;
            m_b2_r = b2_r;
        }
        else
        {
            m_b2 = NULL;
            set_object_vel(rbint::gtv(b2, b2_r));
            set_object_col_pt(b2_r + b2->m_mat.GetW());
        }
        m_ud = ud;
    }
    else
    {
        // b1 is fixed (environment / user body): solve the row from b2's side with the direction flipped.
        m_b1 = b2->m_node;
        m_b1_r = b2_r;
        m_b2 = NULL;
        set_object_vel(rbint::gtv(b1, b1_r));
        set_object_col_pt(b1_r + b1->m_mat.GetW());
        m_ud = -ud;
    }
    m_pulse_sum_cache = ps_cache;
    m_flags = 0;
    // retail quirk: the displacement is applied to whichever body ended up as m_b1
    calc_abs(b1_r_displace);
}

// 0xa26970. Relative velocity change per unit impulse (row point of b1 minus row point of b2).
inline const phys_vec3 pulse_sum_normal::get_relative_velocity_change_dir()
{
    phys_vec3 result = m_ud * m_b1->m_inv_mass + phys_cross(m_b1_ap, m_b1_r);
    if (m_b2)
        result = result + (m_ud * m_b2->m_inv_mass + phys_cross(m_b2_ap, m_b2_r));
    return result;
}

// 0xa26ac0. Start-of-step relative point velocity (b1 - b2).
inline const phys_vec3 pulse_sum_normal::get_relative_velocity()
{
    const phys_vec3 b2_vel = m_b2 ? rbint::gtv(m_b2->m_rb, m_b2_r) : m_b2_ap;
    const phys_vec3 b1_vel = rbint::gtv(m_b1->m_rb, m_b1_r);
    return b1_vel - b2_vel;
}

// 0xa28000. Relative velocity change accumulated in the solver nodes this sub-step.
inline const float pulse_sum_normal::get_objective()
{
    phys_vec3 vel = m_b1->t_vel + phys_cross(m_b1->a_vel, m_b1_r);
    if (m_b2)
        vel = vel - (m_b2->t_vel + phys_cross(m_b2->a_vel, m_b2_r));
    return phys_dot(m_ud, vel);
}

// 0xa28110. Apply impulse s_ along m_ud: +s_ to b1, -s_ to b2.
inline void pulse_sum_normal::apply(const float &s_)
{
    m_b1->t_vel += m_ud * (m_b1->m_inv_mass * s_);
    m_b1->a_vel += m_b1_ap * s_;
    if (m_b2)
    {
        m_b2->t_vel -= m_ud * (m_b2->m_inv_mass * s_);
        m_b2->a_vel -= m_b2_ap * s_;
    }
}

// 0xa28260 (pulse_sum_normal_inline.h:56 / :57 / :61)
inline const float pulse_sum_normal::clamp_pulse_sum(const float ps)
{
    if (m_flags & FLAG_PULSE_LIMIT_PARENT_RATIO)
    {
        PHYS_ASSERT(m_pulse_parent);
        PHYS_ASSERT(m_pulse_limit_ratio >= 0.0f);
        const float limit = fabsf(m_pulse_parent->m_pulse_sum) * m_pulse_limit_ratio;
        m_pulse_sum_max = limit;
        m_pulse_sum_min = -limit;
    }
    PHYS_ASSERT(m_pulse_sum_min <= m_pulse_sum_max);
    // TREYBO FIX(0xa28316 / 0xa28335): `test ah,41h; jnz` then `test ah,5; jp` pass a NaN ps through unclamped
    // (flag cleared); kisakblack returned the min.
    if (!(m_pulse_sum_min > ps))
    {
        if (!(m_pulse_sum_max < ps))
        {
            m_flags &= ~FLAG_PULSE_SUM_CLAMPED;
            return ps;
        }
        m_flags |= FLAG_PULSE_SUM_CLAMPED;
        return m_pulse_sum_max;
    }
    m_flags |= FLAG_PULSE_SUM_CLAMPED;
    return m_pulse_sum_min;
}

// 0xa28360 (pulse_sum_normal_inline.h:90). One projected Gauss-Seidel step.
inline void pulse_sum_normal::SOLVER_apply_relaxation(float &error_sq, const bool add_error)
{
    float m_objective = get_objective() + m_cfm * m_pulse_sum - m_right_side;
    const float m_last_pulse_sum = m_pulse_sum;
    m_objective = m_pulse_sum - m_objective / m_denom;
    m_pulse_sum = clamp_pulse_sum(m_objective);
    PHYS_ASSERT(m_pulse_sum >= (m_pulse_sum_min - .0001f) && m_pulse_sum <= (m_pulse_sum_max + .0001f));
    const float s_ = m_pulse_sum - m_last_pulse_sum;
    apply(s_);
    if (add_error)
    {
        const float error = (m_pulse_sum - m_last_pulse_sum) * m_denom;
        const float error_sq_ = error * error;
        if (error_sq < error_sq_)
            error_sq = error_sq_;
    }
}

// 0xa28440 (inlined in solve_constraints 0xa2b990). Warm start from the cached impulse rate.
inline void pulse_sum_normal::SOLVER_solver_prolog(const float delta_t)
{
    m_right_side = m_right_side - get_vel();
    m_pulse_sum = m_pulse_sum_cache->m_pulse_sum * delta_t;
    m_pulse_sum = clamp_pulse_sum(m_pulse_sum);
    apply(m_pulse_sum);
}

// TREYBO: name (inlined in solve_constraints 0xa2b990). Store the velocity-phase impulse rate and turn on the
// position correction for the vel+pos phase.
inline void pulse_sum_normal::SOLVER_solver_intermediate(const float delta_t)
{
    m_pulse_sum_cache->m_pulse_sum = m_pulse_sum / delta_t;
    m_right_side = m_big_dirt + m_right_side;
}

// 0xa1a2c0. Position-correction velocity from the current separation: 30% of the penetration recovery per
// step, capped at max_penalty_restitution_vel. A separated row (non-negative correction) becomes a velocity
// target instead and needs no position phase.
inline void pulse_sum_normal::setup_vel_uni_standard(const float delta_t, const float max_penalty_restitution_vel)
{
    const float dt = delta_t > MIN_NERP_DELTA_T ? delta_t : MIN_NERP_DELTA_T;
    const float big_dirt = -get_pos() / dt;
    m_big_dirt = big_dirt;
    if (big_dirt < 0.0f)
        m_big_dirt = big_dirt * 0.3f;
    const float max_big_dirt = -max_penalty_restitution_vel;
    if (max_big_dirt > m_big_dirt)
        m_big_dirt = max_big_dirt;
    // TREYBO FIX(0xa1a32e): `fcom; test ah,41h; jp` treats a NaN m_big_dirt as negative (right side 0, kept).
    if (!(m_big_dirt >= 0.0f))
    {
        m_right_side = 0.0f;
    }
    else
    {
        m_right_side = m_big_dirt;
        m_big_dirt = 0.0f;
    }
    m_cfm = 0.0f;
}

// 0xa18f10. As setup_vel_uni_standard, with the separation measured against pos_adjust.
inline void pulse_sum_normal::setup_vel_uni_standard_pos_adjust(const float delta_t, const float pos_adjust,
                                                                 const float max_penalty_restitution_vel)
{
    const float dt = delta_t > MIN_NERP_DELTA_T ? delta_t : MIN_NERP_DELTA_T;
    const float big_dirt = -((get_pos() + pos_adjust) / dt);
    m_big_dirt = big_dirt;
    if (big_dirt < 0.0f)
        m_big_dirt = big_dirt * 0.3f;
    const float max_big_dirt = -max_penalty_restitution_vel;
    if (max_big_dirt > m_big_dirt)
        m_big_dirt = max_big_dirt;
    if (!(m_big_dirt >= 0.0f)) // TREYBO FIX(0xa18f81): NaN counts as negative, as above
    {
        m_right_side = 0.0f;
    }
    else
    {
        m_right_side = m_big_dirt;
        m_big_dirt = 0.0f;
    }
    m_cfm = 0.0f;
}

// 0xa26c00. The impulse one more relaxation step would give (no apply).
inline const float pulse_sum_normal::get_unclamped_pulse_sum()
{
    const float p_ = (m_right_side - get_objective() - m_cfm * m_pulse_sum) / m_denom;
    return clamp_pulse_sum(m_pulse_sum + p_);
}
