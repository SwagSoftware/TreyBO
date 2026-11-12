#pragma once

// Ball joint: three coupled rows (world x, y, z) solved together through the inverse of the 3x3 effective
// mass matrix K (stored as its scaled cofactor rows m_cr23 / m_cr31 / m_cr12).

#include "tl/base/include/tl_defs.h"
#include "tl/physics/include/phys_math.h"
#include "tl/physics/include/phys_assert.h"
#include "tl/physics/include/phys_mem.h"
#include "tl/physics/include/constraint_solver/pulse_sum_cache.h"
#include "tl/physics/include/constraint_solver/pulse_sum_types.inc"

class rigid_body;

class pulse_sum_point : public phys_link_list_base<pulse_sum_point>
{
	friend class pulse_sum_constraint_solver;

public:
	// Data members public (see pulse_sum_normal.h).
	phys_vec3 m_b1_r;
	phys_vec3 m_b2_r; // the world point when m_b2 == NULL
	phys_vec3 m_b1_apx; // b1 world inverse inertia * (r1 x X): angular velocity change per unit x impulse
	phys_vec3 m_b2_apx; // same for b2; the object's point velocity when m_b2 == NULL
	phys_vec3 m_b1_apy;
	phys_vec3 m_b2_apy;
	phys_vec3 m_b1_apz;
	phys_vec3 m_b2_apz;
	phys_vec3 m_pulse_sum;
	phys_vec3 m_right_side;
	phys_vec3 m_big_dirt;
	phys_vec3 m_cr23; // rows of K^-1 (K symmetric): cross(K.y, K.z) / det, ...
	phys_vec3 m_cr31;
	phys_vec3 m_cr12;
	phys_vec3 m_denom; // diagonal of K
	float m_cfm;
	pulse_sum_node * m_b1;
	pulse_sum_node * m_b2;
	pulse_sum_cache * m_pulse_sum_cache; // three caches: x, y, z

	void set(rigid_body * const b1, const phys_vec3 & b1_r, rigid_body * const b2, const phys_vec3 & b2_r,
		pulse_sum_cache * const ps_cache, const float delta_t, const bool is_spring, const float spring_k,
		const float damp_k);

private:
	void set_object_vel(const phys_vec3 & object_vel)
	{
		PHYS_ASSERT(m_b2 == NULL);
		m_b2_apx = object_vel;
	}

	void set_object_col_pt(const phys_vec3 & object_col_pt)
	{
		PHYS_ASSERT(m_b2 == NULL);
		m_b2_r = object_col_pt;
	}

	const phys_vec3 get_vel();
	const phys_vec3 get_pos();
	const phys_vec3 get_objective();
	void apply(const phys_vec3 & s_);
	void calc_abs_and_setup(const float delta_t, const bool is_spring, const float spring_k, const float damp_k);
	void SOLVER_solver_prolog(const float delta_t);
	void SOLVER_apply_relaxation(float & error_sq);
	void SOLVER_solver_intermediate(const float delta_t);
};
