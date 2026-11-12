#pragma once

// One angular row about m_ud. m_b1_r / m_b2_r are not lever arms here but the two body axes whose alignment
// the row measures (get_pos = -dot(m_ud, m_b1_r x m_b2_r)).

#include "tl/base/include/tl_defs.h"
#include "tl/physics/include/phys_math.h"
#include "tl/physics/include/phys_assert.h"
#include "tl/physics/include/phys_mem.h"
#include "tl/physics/include/constraint_solver/pulse_sum_cache.h"
#include "tl/physics/include/constraint_solver/pulse_sum_types.inc"

class rigid_body;

class pulse_sum_angular : public phys_link_list_base<pulse_sum_angular>
{
	friend class pulse_sum_constraint_solver;

public:
	// Data members public (see pulse_sum_normal.h).
	phys_vec3 m_ud;
	phys_vec3 m_b1_r; // b1 axis
	phys_vec3 m_b2_r; // b2 axis (the fixed body's axis when m_b2 == NULL)
	phys_vec3 m_b1_ap; // b1 world inverse inertia * ud
	phys_vec3 m_b2_ap; // same for b2; the object's angular velocity when m_b2 == NULL
	float m_pulse_sum_min;
	float m_pulse_sum_max;
	float m_pulse_sum;
	float m_right_side;
	float m_big_dirt;
	float m_cfm;
	float m_denom;
	unsigned int m_flags; // written 0 by set; never tested
	pulse_sum_node * m_b1;
	pulse_sum_node * m_b2;
	pulse_sum_cache * m_pulse_sum_cache;

	void set(rigid_body * const b1, const phys_vec3 & b1_r, rigid_body * const b2, const phys_vec3 & b2_r,
		const phys_vec3 & ud, pulse_sum_cache * const ps_cache);

	void setup_vel_uni_standard(const float delta_t, const float max_penalty_restitution_vel);

	// Public: the angular_actuator / upright / custom_orientation / custom_path setup_constraint bodies call it.
	const float get_pos();

private:
	void set_object_vel(const phys_vec3 & object_vel)
	{
		PHYS_ASSERT(m_b2 == NULL);
		m_b2_ap = object_vel;
	}

	void set_object_col_pt(const phys_vec3 & object_col_pt)
	{
		PHYS_ASSERT(m_b2 == NULL);
		m_b2_r = object_col_pt;
	}

	const float get_vel();
	void calc_abs();
	const float get_objective();
	void apply(const float & s_);
	const float clamp_pulse_sum(const float ps);
	void SOLVER_apply_relaxation(float & error_sq);

	void SOLVER_solver_prolog(const float delta_t);
	void SOLVER_solver_intermediate(const float delta_t);
};
