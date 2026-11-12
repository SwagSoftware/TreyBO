#pragma once

// Rows are taken from the solver allocator without construction; set() or the caller initialises every field
// the solver reads.

#include "tl/physics/include/constraint_solver/pulse_sum_constraint_solver.h"

// The caller sets the row up with pulse_sum_normal::set.
inline pulse_sum_normal * pulse_sum_constraint_solver::create_pulse_sum_normal()
{
	pulse_sum_normal * const psn = (pulse_sum_normal *)m_solver_memory_allocator.allocate((int)sizeof(pulse_sum_normal), 16, false,
		SOLVER_MEMORY_ALLOCATOR_ERROR_MSG);
	m_list_pulse_sum_normal.add(psn);
	return psn;
}

inline void pulse_sum_constraint_solver::create_point(rigid_body * const b1, const phys_vec3 & b1_r, rigid_body * const b2,
	const phys_vec3 & b2_r, pulse_sum_cache * const ps_cache,
	const float delta_t, const bool is_spring, const float spring_k,
	const float damp_k)
{
	pulse_sum_point * const psp = (pulse_sum_point *)m_solver_memory_allocator.allocate((int)sizeof(pulse_sum_point), 16, false,
		SOLVER_MEMORY_ALLOCATOR_ERROR_MSG);
	m_list_pulse_sum_point.add(psp);
	psp->set(b1, b1_r, b2, b2_r, ps_cache, delta_t, is_spring, spring_k, damp_k);
}

inline pulse_sum_angular * pulse_sum_constraint_solver::create_pulse_sum_angular(rigid_body * const b1, const phys_vec3 & b1_r,
	rigid_body * const b2, const phys_vec3 & b2_r,
	const phys_vec3 & ud, pulse_sum_cache * const ps_cache)
{
	pulse_sum_angular * const psa = (pulse_sum_angular *)m_solver_memory_allocator.allocate((int)sizeof(pulse_sum_angular), 16, false,
		SOLVER_MEMORY_ALLOCATOR_ERROR_MSG);
	m_list_pulse_sum_angular.add(psa);
	psa->set(b1, b1_r, b2, b2_r, ud, ps_cache);
	return psa;
}

// Two angular rows about a1 and a2 (both perpendicular to b1's axis) keep the axes parallel;
// half the misalignment is removed per step in the vel+pos phase. ps_cache points at two caches (a1, a2).
inline void pulse_sum_constraint_solver::create_hinge(rigid_body * const b1, const phys_vec3 & b1_axis, rigid_body * const b2,
	const phys_vec3 & b2_axis, const phys_vec3 & a1, const phys_vec3 & a2,
	pulse_sum_cache * const ps_cache, const float delta_t)
{
	pulse_sum_angular * const psa1 = create_pulse_sum_angular(b1, b1_axis, b2, b2_axis, a1, ps_cache);
	psa1->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
	psa1->m_pulse_sum_max = PHYS_PULSE_SUM_UNBOUNDED;
	const float dt1 = delta_t > MIN_NERP_DELTA_T ? delta_t : MIN_NERP_DELTA_T;
	psa1->m_big_dirt = psa1->get_pos() * (-0.5f / dt1);
	psa1->m_right_side = 0.0f;
	psa1->m_cfm = 0.0f;

	pulse_sum_angular * const psa2 = create_pulse_sum_angular(b1, b1_axis, b2, b2_axis, a2, ps_cache + 1);
	psa2->m_pulse_sum_min = -PHYS_PULSE_SUM_UNBOUNDED;
	psa2->m_pulse_sum_max = PHYS_PULSE_SUM_UNBOUNDED;
	const float dt2 = delta_t > MIN_NERP_DELTA_T ? delta_t : MIN_NERP_DELTA_T;
	psa2->m_big_dirt = psa2->get_pos() * (-0.5f / dt2);
	psa2->m_right_side = 0.0f;
	psa2->m_cfm = 0.0f;
}

// The caller sets m_suspension up and allocates m_side / m_fwd.
inline pulse_sum_wheel * pulse_sum_constraint_solver::create_pulse_sum_wheel()
{
	pulse_sum_wheel * const psw = (pulse_sum_wheel *)m_solver_memory_allocator.allocate((int)sizeof(pulse_sum_wheel), 16, false,
		SOLVER_MEMORY_ALLOCATOR_ERROR_MSG);
	m_list_pulse_sum_wheel.add(psw);
	return psw;
}
