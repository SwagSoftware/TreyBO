#include "tl/physics/include/rbc_defs/rbc_def_vehicle.h"
#include "tl/physics/include/constraint_solver/pulse_sum_constraint_solver.h"

void rigid_body_constraint_wheel::set_wheel_state_accelerating(const float desired_speed_k, const float acceleration_factor_k)
{
	m_wheel_state = WHEEL_STATE_ACCELERATING;
	m_desired_speed_k = desired_speed_k;
	m_acceleration_factor_k = acceleration_factor_k;
}

void rigid_body_constraint_wheel::get_wheel_state_accelerating(float & desired_speed_k, float & acceleration_factor_k)
{
	desired_speed_k = m_desired_speed_k;
	acceleration_factor_k = m_acceleration_factor_k;
}

void rigid_body_constraint_wheel::set_wheel_state_braking(const float braking_factor_k)
{
	m_wheel_state = WHEEL_STATE_BRAKING;
	m_braking_factor_k = braking_factor_k;
}

void rigid_body_constraint_wheel::set_no_collision()
{
	set_wheel_flag(WHEEL_FLAG_IS_COLLIDING, false);
	b2 = NULL;
}

void rigid_body_constraint_wheel::set_collision(rigid_body * const rb, const phys_vec3 & hitp_loc, const phys_vec3 & hitn_loc)
{
	set_wheel_flag(WHEEL_FLAG_IS_COLLIDING, true);
	b2 = rb;
	m_b2_hitp_loc = hitp_loc;
	m_b2_hitn_loc = hitn_loc;
}

float lerp_float(const float tgt, float cur, float rate, float delta_t)
{
	const float diff = tgt - cur;
	const float step = (diff <= 0.0f ? -rate : rate) * delta_t;
	const float ndiff = fabsf(diff);
	if (ndiff > 0.05f && !(fabsf(step) > ndiff)) // a NaN step returns step + cur
		return step + cur;
	return tgt;
}

void rigid_body_constraint_wheel::set(const phys_vec3 & wheel_center_loc,
	const phys_vec3 & suspension_dir_loc,
	const phys_vec3 & wheel_axis_loc, const float wheel_radius,
	const float fwd_fric_k, const float side_fric_k,
	const float suspension_stiffness_k, const float suspension_damp_k,
	const float hard_limit_dist, const float roll_stability_factor,
	const float pitch_stability_factor, const float side_fric_max)
{
	m_b1_wheel_center_loc = wheel_center_loc;
	m_b1_suspension_dir_loc = suspension_dir_loc;
	m_b1_wheel_axis_loc = wheel_axis_loc;
	m_wheel_state = WHEEL_STATE_BRAKING;
	m_wheel_flags = 0;
	m_wheel_radius = wheel_radius;
	m_fwd_fric_k = fwd_fric_k;
	m_side_fric_k = side_fric_k;
	m_side_fric_max = side_fric_max;
	m_suspension_stiffness_k = suspension_stiffness_k;
	m_suspension_damp_k = suspension_damp_k;
	m_hard_limit_dist = hard_limit_dist;
	m_roll_stability_factor = roll_stability_factor;
	m_pitch_stability_factor = pitch_stability_factor;
	m_braking_factor_k = 0.0f;
	m_turning_radius_ratio_max_speed = 1.0f;
	m_turning_radius_ratio_accel = 1.0f;
	m_wheel_vel = 0.0f;
	m_wheel_pos = 0.0f;
	m_wheel_fwd = 0.0f;
	m_wheel_displaced_center_dist = 0.0f;
	PHYS_ASSERT_UNIT(m_b1_suspension_dir_loc);
	PHYS_ASSERT_UNIT(m_b1_wheel_axis_loc);
	PHYS_ASSERT_ORTHOGONAL(m_b1_suspension_dir_loc, m_b1_wheel_axis_loc);
}

// The suspension travel segment, center -/+ radius along the suspension direction.
void rigid_body_constraint_wheel::get_wheel_collide_segment(const phys_mat44 & b1_mat, phys_vec3 * const p0, phys_vec3 * const p1) const
{
	PHYS_ASSERT_ALIGNED(p0);
	PHYS_ASSERT_ALIGNED(p1);
	const phys_vec3 wheel_center = phys_full_multiply(b1_mat, m_b1_wheel_center_loc);
	const phys_vec3 seg_dir = phys_multiply(b1_mat, m_b1_suspension_dir_loc) * m_wheel_radius;
	*p0 = wheel_center - seg_dir;
	*p1 = seg_dir + wheel_center;
}

void rigid_body_constraint_wheel::epilog_vel_constraint(const float delta_t)
{
	static float velocity_clamp = 0.5f;
	static float lr = 1000.0f; // spin-down rate with hard braking
	static float lr_0 = 1.0f; // free spin-down rate

	const float prev_wheel_displaced_center_dist = m_wheel_displaced_center_dist;
	if (get_wheel_flag(WHEEL_FLAG_IS_COLLIDING))
	{
		const phys_vec3 b1_suspension_dir = phys_multiply(b1->m_mat, m_b1_suspension_dir_loc);
		const phys_vec3 b1_r = phys_multiply(b1->m_mat, m_b1_wheel_center_loc) + b1_suspension_dir * m_wheel_radius;
		const phys_vec3 b2_r = phys_multiply(b2->m_mat, m_b2_hitp_loc);
		const phys_vec3 pen = (b1->m_mat.GetW() + b1_r) - (b2->m_mat.GetW() + b2_r);
		float displaced = phys_dot(pen, b1_suspension_dir);
		if (m_hard_limit_dist < displaced)
			displaced = m_hard_limit_dist;
		m_wheel_displaced_center_dist = displaced;
	}
	else
	{
		m_wheel_displaced_center_dist = 0.0f;
	}

	// the wheel drops back at most 51 units/s
	if (m_wheel_displaced_center_dist < prev_wheel_displaced_center_dist) // NaN skips the drop
	{
		float dropped = prev_wheel_displaced_center_dist - 51.0f * delta_t;
		if (dropped < m_wheel_displaced_center_dist)
			dropped = m_wheel_displaced_center_dist;
		m_wheel_displaced_center_dist = dropped;
	}

	m_wheel_normal_force = m_ps_suspension ? m_ps_cache_list[PSC_SUSPENSION].m_pulse_sum : 0.0f;

	const bool apply_wheel_velocity = velocity_clamp < fabsf(m_wheel_vel * m_wheel_radius);
	if (m_ps_side_fric)
	{
		// pulse_sum_normal::m_flags bits 0x2 / 0x4 are set by the solver.
		if (m_ps_fwd_fric)
		{
			if (m_ps_fwd_fric->m_flags & 0x4)
				m_wheel_flags |= WHEEL_FLAG_IS_SLIDING;
			m_wheel_vel = m_ps_fwd_fric->get_unclamped_pulse_sum() * m_wheel_fwd / m_wheel_radius + m_wheel_vel;
		}
		else if (m_ps_side_fric->m_flags & 0x2)
		{
			m_wheel_flags |= WHEEL_FLAG_IS_SLIDING;
		}
	}
	if (apply_wheel_velocity)
		m_wheel_pos = m_wheel_vel * delta_t + m_wheel_pos;

	if (!get_wheel_flag(WHEEL_FLAG_IS_COLLIDING))
	{
		const float lerp_rate = m_braking_factor_k < 50.0f ? lr_0 : lr; // NaN picks lr
		m_wheel_vel = lerp_float(0.0f, m_wheel_vel, lerp_rate, delta_t);
	}
}

void rigid_body_constraint_wheel::setup_constraint(pulse_sum_constraint_solver * psys, const float delta_t)
{
	set_wheel_flag(WHEEL_FLAG_IS_SLIDING, false);
	m_wheel_fwd = 0.0f;
	m_ps_suspension = NULL;
	m_ps_side_fric = NULL;
	m_ps_fwd_fric = NULL;
	if (!get_wheel_flag(WHEEL_FLAG_IS_COLLIDING))
		return;

	// contact points: b1_r at the wheel bottom, b2_r at the hit point
	const phys_vec3 b1_suspension_dir = phys_multiply(b1->m_mat, m_b1_suspension_dir_loc);
	phys_vec3 b1_r = phys_multiply(b1->m_mat, m_b1_wheel_center_loc) + b1_suspension_dir * m_wheel_radius;
	phys_vec3 b2_r = phys_multiply(b2->m_mat, m_b2_hitp_loc);
	const phys_vec3 pen = (b1->m_mat.GetW() + b1_r) - (b2->m_mat.GetW() + b2_r);

	if (!(m_hard_limit_dist - 3.4f <= phys_dot(pen, b1_suspension_dir))) // NaN clears
		m_wheel_flags &= ~WHEEL_FLAG_HARD_LIMIT_ACTIVE;
	else
		m_wheel_flags |= WHEEL_FLAG_HARD_LIMIT_ACTIVE;

	if (get_wheel_flag(WHEEL_FLAG_HARD_LIMIT_ACTIVE))
	{
		// suspension bottomed out: rigid contact at the top of the travel
		const phys_vec3 hard_limit_ud = -phys_multiply(b2->m_mat, m_b2_hitn_loc);
		pulse_sum_normal * const ps_hard_limit = psys->create_pulse_sum_normal();
		ps_hard_limit->set(b1, b1_r - m_hard_limit_dist * b1_suspension_dir, b2, b2_r, hard_limit_ud,
			&m_ps_cache_list[PSC_HARD_LIMIT], PHYS_ZERO_VEC);
		ps_hard_limit->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
		ps_hard_limit->m_pulse_sum_max = 0.0f;
		ps_hard_limit->setup_vel_uni_standard(delta_t, 170.0f);
	}

	// move both points onto the contact plane
	const phys_vec3 hitn = -phys_multiply(b2->m_mat, m_b2_hitn_loc);
	const float pen_depth = phys_dot(hitn, pen);
	const phys_vec3 pen_vec = hitn * pen_depth;
	b1_r -= pen_vec;
	b2_r -= pen_vec;

	// the suspension and side friction act at a point raised by the roll stability factor
	const phys_vec3 b1_r_displace = -m_roll_stability_factor * b1_suspension_dir;
	pulse_sum_wheel * const ps_wheel = psys->create_pulse_sum_wheel();
	ps_wheel->m_side = NULL;
	ps_wheel->m_fwd = NULL;
	m_ps_suspension = &ps_wheel->m_suspension;
	m_ps_suspension->set(b1, b1_r, b2, b2_r, hitn, &m_ps_cache_list[PSC_SUSPENSION], b1_r_displace);
	m_ps_suspension->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
	m_ps_suspension->m_pulse_sum_max = 0.0f;
	{
		// implicit spring-damper
		pulse_sum_normal * const ps = m_ps_suspension;
		const float k_dt2 = m_suspension_stiffness_k * delta_t * delta_t;
		const float cfm = 1.0f / (k_dt2 + delta_t * m_suspension_damp_k);
		ps->m_right_side = ps->get_pos() * (-(k_dt2 * cfm) / delta_t);
		ps->m_big_dirt = 0.0f;
		ps->m_cfm = cfm;
		ps->m_denom = cfm + ps->m_denom;
	}

	// side friction along the wheel axis tilted onto the contact plane
	const phys_vec3 wheel_axis = phys_multiply(b1->m_mat, m_b1_wheel_axis_loc);
	phys_mat44 rot_mat;
	make_rotate(&rot_mat, b1_suspension_dir, hitn);
	const phys_vec3 fric_ud = phys_multiply(rot_mat, wheel_axis);
	pulse_sum_normal * const ps_side_fric = (pulse_sum_normal *)psys->m_solver_memory_allocator.allocate(
		sizeof(pulse_sum_normal), 16, false, SOLVER_MEMORY_ALLOCATOR_ERROR_MSG);
	ps_wheel->m_side = ps_side_fric;
	m_ps_side_fric = ps_side_fric;
	m_ps_side_fric->set(b1, b1_r, b2, b2_r, fric_ud, &m_ps_cache_list[PSC_SIDE_FRIC], b1_r_displace);
	m_ps_side_fric->m_right_side = 0.0f;
	m_ps_side_fric->m_big_dirt = 0.0f;
	m_ps_side_fric->m_cfm = 0.0f;

	const phys_vec3 fwd = -phys_cross(fric_ud, hitn);
	m_wheel_vel = phys_dot(m_ps_suspension->get_relative_velocity(), fwd) / m_wheel_radius;
	if ((m_wheel_state == WHEEL_STATE_ACCELERATING && m_acceleration_factor_k < PHYS_EPS_1E4) ||
		(m_wheel_state == WHEEL_STATE_BRAKING && m_braking_factor_k < PHYS_EPS_1E4))
	{
		// free rolling: no forward row, side friction bounded by the suspension load
		m_ps_side_fric->set_pulse_sum_limits_parent_ratio(m_side_fric_k, m_ps_suspension);
		return;
	}

	m_ps_side_fric->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
	m_ps_side_fric->m_pulse_sum_max = PHYS_PULSE_SUM_UNBOUNDED;
	m_ps_fwd_fric = psys->create_pulse_sum_wheel_fwd(ps_wheel);
	m_ps_fwd_fric->set(b1, b1_r, b2, b2_r, fwd, &m_ps_cache_list[PSC_FWD_FRIC], -m_pitch_stability_factor * b1_suspension_dir);
	if (m_wheel_state != WHEEL_STATE_ACCELERATING)
	{
		m_ps_fwd_fric->m_right_side = 0.0f;
		m_ps_fwd_fric->m_big_dirt = 0.0f;
		m_ps_fwd_fric->m_cfm = 0.0f;
		const float braking = m_braking_factor_k * delta_t;
		m_ps_fwd_fric->m_pulse_sum_min = -braking;
		m_ps_fwd_fric->m_pulse_sum_max = braking;
	}
	else
	{
		// drive toward the desired rim speed through a soft row
		const float desired_speed = m_desired_speed_k * m_turning_radius_ratio_max_speed;
		float k_ = m_acceleration_factor_k * m_turning_radius_ratio_accel * delta_t / (m_wheel_radius * m_wheel_radius);
		if (k_ <= PHYS_EPS_1E6)
			k_ = PHYS_EPS_1E5;
		const float cfm = 1.0f / k_;
		m_ps_fwd_fric->m_right_side = m_wheel_radius * desired_speed;
		m_ps_fwd_fric->m_big_dirt = 0.0f;
		m_ps_fwd_fric->m_cfm = cfm;
		m_ps_fwd_fric->m_denom = cfm + m_ps_fwd_fric->m_denom;
		if (desired_speed > PHYS_EPS_1E4)
		{
			m_ps_fwd_fric->m_pulse_sum_min = 0.0f;
			m_ps_fwd_fric->m_pulse_sum_max = PHYS_PULSE_SUM_UNBOUNDED;
		}
		else if (!(desired_speed < -PHYS_EPS_1E4)) // NaN -> free range
		{
			m_ps_fwd_fric->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
			m_ps_fwd_fric->m_pulse_sum_max = PHYS_PULSE_SUM_UNBOUNDED;
		}
		else
		{
			m_ps_fwd_fric->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
			m_ps_fwd_fric->m_pulse_sum_max = 0.0f;
		}
	}
	ps_wheel->set_side_fwd_ratios(m_side_fric_k, m_fwd_fric_k, m_side_fric_max);
	m_wheel_fwd = phys_dot(m_ps_fwd_fric->get_relative_velocity_change_dir(), fwd);
}
