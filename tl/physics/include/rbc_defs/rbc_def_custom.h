#pragma once

#include "tl/base/include/tl_defs.h"
#include "tl/physics/include/phys_math.h"
#include "tl/physics/include/rigid_body.h"
#include "tl/physics/include/rbc_defs/rbc_def_base.h"
#include "tl/physics/include/constraint_solver/pulse_sum_cache.h"

// Soft orientation hold of b1 against b2 (pitch/roll and yaw resistance) plus an optional self-righting term.
class rigid_body_constraint_custom_orientation : public rigid_body_constraint
{
public:
	pulse_sum_cache m_ps_cache_list[5];
	bool m_active; // enables the upright terms
	bool m_no_orientation_correction; // upright terms only resist further tilt, never push back
	float m_torque_resistance_pitch_roll;
	float m_torque_resistance_yaw;
	float m_upright_strength;

	rigid_body_constraint_custom_orientation() :
		m_active(false), m_no_orientation_correction(false),
		m_torque_resistance_pitch_roll(50.0f), m_torque_resistance_yaw(10.0f), m_upright_strength(50.0f)
	{
	}

	void setup_constraint(pulse_sum_constraint_solver * psys, const float delta_t);
};

// Spring-holds b1 on a path driven by a user rigid body (b2 / m_urb): x and y position plus two angular terms.
// m_path_mat is written by the host; setup_constraint does not read it.
class rigid_body_constraint_custom_path : public rigid_body_constraint
{
public:
	phys_mat44 m_path_mat;
	phys_vec3 b1_r_loc;
	user_rigid_body * m_urb;
	pulse_sum_cache m_list_psc[4];

	void setup_constraint(pulse_sum_constraint_solver * psys, const float delta_t);
};
