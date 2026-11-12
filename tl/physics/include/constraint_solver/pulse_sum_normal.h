#pragma once

// One scalar row along m_ud between a point on b1 and a point on b2 (or a fixed world point with a known
// velocity when b2 has no solver node). m_pulse_sum is the accumulated impulse of this row for the current
// sub-step; the pulse_sum_cache keeps it between frames as an impulse rate (pulse / delta_t) for warm starting.

#include "tl/base/include/tl_defs.h"
#include "tl/physics/include/phys_math.h"
#include "tl/physics/include/phys_assert.h"
#include "tl/physics/include/phys_mem.h"
#include "tl/physics/include/constraint_solver/pulse_sum_cache.h"
#include "tl/physics/include/constraint_solver/pulse_sum_types.inc"

class rigid_body;

class pulse_sum_normal : public phys_link_list_base<pulse_sum_normal>
{
	friend class pulse_sum_constraint_solver;
	friend class pulse_sum_wheel;

public:
	// rigid_body_constraint_wheel::epilog_vel_constraint reads 0x2 and 0x4 as literals.
	enum flags_e
	{
		FLAG_PULSE_LIMIT_PARENT_RATIO = 0x1, // limits = +-|parent pulse| * m_pulse_limit_ratio, recomputed per clamp
		FLAG_PULSE_SUM_CLAMPED = 0x2, // the last clamp_pulse_sum hit a limit
		FLAG_PULSE_CHAIN_CLAMPED = 0x4, // wheel friction ellipse was active on the last chain clamp (fwd row)
	};

	// Data members public: rbc_def setup code writes
	// m_pulse_sum_min/max, m_right_side, m_big_dirt, m_cfm and m_denom directly and reads m_flags.
	phys_vec3 m_ud; // row direction (unit); the row's relative velocity is dot(m_ud, v1 - v2)
	phys_vec3 m_b1_r; // world offset of the row point from b1's position
	phys_vec3 m_b2_r; // world offset from b2's position; the world point itself when m_b2 == NULL
	phys_vec3 m_b1_ap; // b1 world inverse inertia * (r1 x ud): angular velocity change per unit impulse
	phys_vec3 m_b2_ap; // same for b2; the object's point velocity when m_b2 == NULL
	float m_pulse_sum_min;
	float m_pulse_sum_max;
	float m_pulse_sum;
	float m_right_side; // target relative velocity (minus the start velocity after the prolog)
	float m_big_dirt; // position-correction velocity, added to m_right_side for the vel+pos phase
	float m_cfm; // constraint force mixing (soft rows): objective += m_cfm * m_pulse_sum
	float m_denom; // effective inverse mass of the row (+ m_cfm)
	float m_pulse_limit_ratio;
	unsigned int m_flags;
	pulse_sum_normal * m_pulse_parent;
	pulse_sum_node * m_b1;
	pulse_sum_node * m_b2;
	pulse_sum_cache * m_pulse_sum_cache;

	void set(rigid_body * const b1, const phys_vec3 & b1_r, rigid_body * const b2, const phys_vec3 & b2_r,
		const phys_vec3 & ud, pulse_sum_cache * const ps_cache, const phys_vec3 & b1_r_displace);

	void setup_vel_uni_standard(const float delta_t, const float max_penalty_restitution_vel);
	void setup_vel_uni_standard_pos_adjust(const float delta_t, const float pos_adjust, const float max_penalty_restitution_vel);

	void set_pulse_sum_limits_parent_ratio(const float limit_ratio, pulse_sum_normal * const parent)
	{
		PHYS_ASSERT(limit_ratio >= 0.0f);
		PHYS_ASSERT(parent);
		m_flags |= FLAG_PULSE_LIMIT_PARENT_RATIO;
		m_pulse_limit_ratio = limit_ratio;
		m_pulse_parent = parent;
		m_pulse_sum_min = 0.0f;
		m_pulse_sum_max = 0.0f;
	}

	void set_object_vel(const phys_vec3 & object_vel)
	{
		PHYS_ASSERT_STR(m_b2 == NULL, "m_b2 == NULL");
		m_b2_ap = object_vel;
	}

	void set_object_col_pt(const phys_vec3 & object_col_pt)
	{
		PHYS_ASSERT_STR(m_b2 == NULL, "m_b2 == NULL");
		m_b2_r = object_col_pt;
	}

	const phys_vec3 get_relative_velocity();
	const phys_vec3 get_relative_velocity_change_dir();
	const float get_unclamped_pulse_sum();

	// Public: the rigid_body_constraint_wheel / _custom_path setup_constraint bodies call it.
	const float get_pos();

private:
	void calc_abs(const phys_vec3 & b1_r_displace);
	const float get_vel();
	const float get_last_vel();
	const float get_objective();
	void apply(const float & s_);
	const float clamp_pulse_sum(const float ps);
	void SOLVER_apply_relaxation(float & error_sq, const bool add_error);
	void SOLVER_solver_prolog(const float delta_t);
	void SOLVER_solver_intermediate(const float delta_t);
};
