// Retail: codsrc\tl\physics\source\rbc_def_custom.cpp, TU 0xa2cf20-0xa2d839
// (rbint::mul_L at 0xa2ce80 is the rigid_body_internal.h inline emitted for this TU; defined in rigid_body.cpp).

#include "tl/physics/include/rbc_defs/rbc_def_custom.h"
#include "tl/physics/include/rigid_body_internal.h"
#include "tl/physics/include/constraint_solver/pulse_sum_constraint_solver.h"

// 0xa2cf20
void rigid_body_constraint_custom_orientation::setup_constraint(pulse_sum_constraint_solver *psys, const float delta_t)
{
    const phys_mat44 &b1_mat = b1->m_mat;
    const phys_mat44 &b2_mat = b2->m_mat;

    // pitch / roll resistance: soft rows about b1's y and x axes
    if (m_torque_resistance_pitch_roll > 0.0f)
    {
        const float pitch_roll = m_torque_resistance_pitch_roll * 10.0f;
        pulse_sum_angular *const psa_pitch = psys->create_pulse_sum_angular(b1, b1_mat.GetZ(), b2, b2_mat.GetZ(), b1_mat.GetY(), &m_ps_cache_list[0]);
        psa_pitch->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
        psa_pitch->m_pulse_sum_max = PHYS_PULSE_SUM_UNBOUNDED;
        const float cfm = 1.0f / (pitch_roll * delta_t);
        psa_pitch->m_right_side = 0.0f;
        psa_pitch->m_big_dirt = 0.0f;
        psa_pitch->m_cfm = cfm;
        psa_pitch->m_denom = cfm + psa_pitch->m_denom;

        pulse_sum_angular *const psa_roll = psys->create_pulse_sum_angular(b1, b1_mat.GetZ(), b2, b2_mat.GetZ(), b1_mat.GetX(), &m_ps_cache_list[1]);
        psa_roll->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
        psa_roll->m_pulse_sum_max = PHYS_PULSE_SUM_UNBOUNDED;
        psa_roll->m_right_side = 0.0f;
        psa_roll->m_big_dirt = 0.0f;
        psa_roll->m_cfm = cfm;
        psa_roll->m_denom = cfm + psa_roll->m_denom;
    }

    // yaw resistance: soft row about b1's z axis
    if (0.0f < m_torque_resistance_yaw)
    {
        const float yaw = m_torque_resistance_yaw * 10.0f;
        pulse_sum_angular *const psa_yaw = psys->create_pulse_sum_angular(b1, b1_mat.GetX(), b2, b2_mat.GetX(), b1_mat.GetZ(), &m_ps_cache_list[2]);
        psa_yaw->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
        psa_yaw->m_pulse_sum_max = PHYS_PULSE_SUM_UNBOUNDED;
        const float cfm = 1.0f / (yaw * delta_t);
        psa_yaw->m_right_side = 0.0f;
        psa_yaw->m_big_dirt = 0.0f;
        psa_yaw->m_cfm = cfm;
        psa_yaw->m_denom = cfm + psa_yaw->m_denom;
    }

    // self-righting: drive b1's z axis back to b2's, with strength growing with the tilt (full when upside down)
    if (m_active && 0.0f < m_upright_strength)
    {
        float roll = fabsf(b1_mat.GetY().GetZ());
        float pitch = fabsf(b1_mat.GetX().GetZ());
        if (0.0f > b1_mat.GetZ().GetZ())
        {
            pitch = 1.0f;
            roll = 1.0f;
        }

        const float pitch_limit = m_upright_strength * 30.0f * delta_t * pitch;
        pulse_sum_angular *const psa_pitch = psys->create_pulse_sum_angular(b1, b1_mat.GetZ(), b2, b2_mat.GetZ(), b1_mat.GetY(), &m_ps_cache_list[3]);
        float rate;
        if (m_no_orientation_correction)
        {
            if (!(b1_mat.GetX().GetZ() < 0.0f)) // TREYBO FIX(0xa2d0f0): NaN takes the >= 0 branch
            {
                psa_pitch->m_pulse_sum_min = 0.0f;
                psa_pitch->m_pulse_sum_max = pitch_limit;
            }
            else
            {
                psa_pitch->m_pulse_sum_min = -pitch_limit;
                psa_pitch->m_pulse_sum_max = 0.0f;
            }
            rate = -0.0f / delta_t;
        }
        else
        {
            psa_pitch->m_pulse_sum_min = -pitch_limit;
            psa_pitch->m_pulse_sum_max = pitch_limit;
            rate = -1.0f / delta_t;
        }
        psa_pitch->m_right_side = psa_pitch->get_pos() * rate;
        psa_pitch->m_big_dirt = 0.0f;
        psa_pitch->m_cfm = 0.0f;
        psa_pitch->m_denom = psa_pitch->m_denom + 0.0f;

        const float roll_limit = delta_t * (m_upright_strength * 100.0f) * roll;
        pulse_sum_angular *const psa_roll = psys->create_pulse_sum_angular(b1, b1_mat.GetZ(), b2, b2_mat.GetZ(), b1_mat.GetX(), &m_ps_cache_list[4]);
        if (m_no_orientation_correction)
        {
            const float half_roll_limit = roll_limit * 0.5f;
            if (!(b1_mat.GetY().GetZ() > 0.0f)) // TREYBO FIX(0xa2d1a2): NaN takes the <= 0 branch
            {
                psa_roll->m_pulse_sum_min = 0.0f;
                psa_roll->m_pulse_sum_max = half_roll_limit;
            }
            else
            {
                psa_roll->m_pulse_sum_min = -half_roll_limit;
                psa_roll->m_pulse_sum_max = 0.0f;
            }
            rate = -0.0f / delta_t;
        }
        else
        {
            psa_roll->m_pulse_sum_min = -roll_limit;
            psa_roll->m_pulse_sum_max = roll_limit;
            rate = -1.0f / delta_t;
        }
        psa_roll->m_right_side = psa_roll->get_pos() * rate;
        psa_roll->m_big_dirt = 0.0f;
        psa_roll->m_cfm = 0.0f;
        psa_roll->m_denom = psa_roll->m_denom + 0.0f;
    }
}

// 0xa2d220. Springs b1 onto b2 (the path's user rigid body): x / y position rows, b1's x axis onto the plane
// normal to b2's y axis, and a 30 degree cone keeping b1's z axis near world up (0, 0, 1).
void rigid_body_constraint_custom_path::setup_constraint(pulse_sum_constraint_solver *psys, const float delta_t)
{
    // function-scope statics in retail (writable data 0xe1a058-0xe1a06c)
    static float psn_spring_k = 50.0f;
    static float psn_spring_d = 12.0f;

    const phys_vec3 b2_r_loc(0.0f, 0.0f, 0.0f);
    const phys_vec3 b1_r = phys_multiply(b1->m_mat, b1_r_loc);
    const phys_vec3 b2_r = phys_multiply(b2->m_mat, b2_r_loc);
    PHYS_ASSERT(m_urb);

    {
        const float mass = 1.0f / b1->m_inv_mass;
        const float k_dt2 = psn_spring_k * mass * delta_t * delta_t;
        const float cfm = 1.0f / (mass * psn_spring_d * delta_t + k_dt2);
        const float rate = -(k_dt2 * cfm) / delta_t;

        pulse_sum_normal *const psn_x = psys->create_pulse_sum_normal();
        psn_x->set(b1, b1_r, b2, b2_r, PHYS_X_VEC, &m_list_psc[0], PHYS_ZERO_VEC);
        psn_x->m_right_side = psn_x->get_pos() * rate;
        psn_x->m_big_dirt = 0.0f;
        psn_x->m_cfm = cfm;
        psn_x->m_denom = cfm + psn_x->m_denom;
        psn_x->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
        psn_x->m_pulse_sum_max = PHYS_PULSE_SUM_UNBOUNDED;

        pulse_sum_normal *const psn_y = psys->create_pulse_sum_normal();
        psn_y->set(b1, b1_r, b2, b2_r, PHYS_Y_VEC, &m_list_psc[1], PHYS_ZERO_VEC);
        psn_y->m_right_side = psn_y->get_pos() * rate;
        psn_y->m_big_dirt = 0.0f;
        psn_y->m_cfm = cfm;
        psn_y->m_denom = cfm + psn_y->m_denom;
        psn_y->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
        psn_y->m_pulse_sum_max = PHYS_PULSE_SUM_UNBOUNDED;
    }

    {
        static float psa_spring_k = 100.0f; // retail psa_spring_k_0 / psa_spring_d_0
        static float psa_spring_d = 5.0f;

        const phys_vec3 b1_axis = b1->m_mat.GetX();
        const phys_vec3 &b2_axis = b2->m_mat.GetY();
        phys_vec3 b1_ud = b1_axis - b2_axis * phys_dot(b2_axis, b1_axis);
        float nb1_ud = Abs(b1_ud);
        if (nb1_ud > PHYS_EPS_1E4) // TREYBO FIX(0xa2d484): a NaN length skips the row
        {
            nb1_ud = 1.0f / nb1_ud;
            b1_ud *= nb1_ud;
            const phys_vec3 ud = phys_cross(b2_axis, b1_ud);
            const float L = phys_dot(rbint::mul_L(b1, ud), ud);
            const float k_dt2 = psa_spring_k * L * delta_t * delta_t;
            const float cfm = 1.0f / (k_dt2 + L * psa_spring_d * delta_t);
            pulse_sum_angular *const psa = psys->create_pulse_sum_angular(b1, b1_axis, b2, b1_ud, ud, &m_list_psc[2]);
            psa->m_right_side = psa->get_pos() * (-(k_dt2 * cfm) / delta_t);
            psa->m_big_dirt = 0.0f;
            psa->m_cfm = cfm;
            psa->m_denom = cfm + psa->m_denom;
            psa->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
            psa->m_pulse_sum_max = PHYS_PULSE_SUM_UNBOUNDED;
        }
    }

    {
        static float psa_spring_k = 100.0f; // retail psa_spring_k / psa_spring_d
        static float psa_spring_d = 5.0f;

        const phys_vec3 b1_axis = b1->m_mat.GetZ();
        const phys_vec3 up(0.0f, 0.0f, 1.0f);
        const phys_vec3 perp = up - b1_axis * phys_dot(b1_axis, up);
        const float nperp = Abs(perp);
        if (nperp >= PHYS_EPS_1E4)
        {
            // the cone edge 30 degrees from b1's z axis toward up, and the row axis perpendicular to it
            const phys_vec3 b1_r_ = perp * (0.8660254f / nperp) + b1_axis * 0.5f; // cos(30), sin(30)
            const phys_vec3 ud = phys_cross(b1_r_, b1_axis) * 1.1547005f;      // 1 / sin(60)
            PHYS_ASSERT_UNIT(ud);
            const float L = phys_dot(rbint::mul_L(b1, ud), ud);
            const float k_dt2 = L * psa_spring_k * delta_t * delta_t;
            const float cfm = 1.0f / (k_dt2 + L * psa_spring_d * delta_t);
            pulse_sum_angular *const psa = psys->create_pulse_sum_angular(b1, b1_r_, b2, up, ud, &m_list_psc[3]);
            psa->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
            psa->m_pulse_sum_max = 0.0f;
            psa->m_right_side = psa->get_pos() * (-(k_dt2 * cfm) / delta_t);
            psa->m_big_dirt = 0.0f;
            psa->m_cfm = cfm;
            psa->m_denom = cfm + psa->m_denom;
        }
    }
}
