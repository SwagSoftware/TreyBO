#include "tl/physics/include/constraint_solver/pulse_sum_constraint_solver.h"
#include "tl/physics/include/rbc_defs/rbc_def_generic.h"
#include "tl/physics/include/rbc_defs/rbc_def_ragdoll.h"
#include "tl/physics/include/rbc_defs/rbc_def_vehicle.h"
#include "tl/physics/include/rbc_defs/rbc_def_custom.h"
#include "tl/physics/include/rbc_defs/rbc_def_contact.h"
#include "tl/physics/include/physics_system_internal.h"

// Next island index for the solver jobs.
volatile int g_list_island_cur;

// ---------------------------------------------------------------------------------------------
// Island constraint walks: one loop per constraint type, always in the rb_inplace_partition_node
// order below; the bodies are each type's own (non-virtual) method.
// ---------------------------------------------------------------------------------------------

template <class T, class F>
static inline void for_each_rbc(T * first, F & f)
{
	for (T * rbc = first; rbc; rbc = static_cast<T *>(rbc->m_next))
		f(rbc);
}

template <class F>
static inline void for_each_island_constraint(rigid_body * const head, F f)
{
	const rb_inplace_partition_node & pn = head->m_partition_node;
	for_each_rbc(pn.m_rbc_point_first, f);
	for_each_rbc(pn.m_rbc_hinge_first, f);
	for_each_rbc(pn.m_rbc_dist_first, f);
	for_each_rbc(pn.m_rbc_ragdoll_first, f);
	for_each_rbc(pn.m_rbc_wheel_first, f);
	for_each_rbc(pn.m_rbc_angular_actuator_first, f);
	for_each_rbc(pn.m_rbc_upright_first, f);
	for_each_rbc(pn.m_rbc_custom_orientation_first, f);
	for_each_rbc(pn.m_rbc_custom_path_first, f);
	for_each_rbc(pn.m_rbc_contact_first, f);
}

template <int N>
static inline void reset_pulse_sum_caches(pulse_sum_cache (&ps_cache_list)[N])
{
	for (int i = 0; i < N; i++)
		ps_cache_list[i].m_pulse_sum = 0.0f;
}

// The caches each constraint type owns (contact caches live in the contact_point_info buffers instead).
static inline void reset_rbc_pulse_sum_caches(rigid_body_constraint_point * rbc)
{
	reset_pulse_sum_caches(rbc->m_ps_cache_list);
}
static inline void reset_rbc_pulse_sum_caches(rigid_body_constraint_hinge * rbc)
{
	reset_pulse_sum_caches(rbc->m_ps_cache);
}
static inline void reset_rbc_pulse_sum_caches(rigid_body_constraint_distance * rbc)
{
	reset_pulse_sum_caches(rbc->m_ps_cache_list);
}
static inline void reset_rbc_pulse_sum_caches(rigid_body_constraint_ragdoll * rbc)
{
	reset_pulse_sum_caches(rbc->m_ps_cache_list);
}
static inline void reset_rbc_pulse_sum_caches(rigid_body_constraint_wheel * rbc)
{
	reset_pulse_sum_caches(rbc->m_ps_cache_list);
}
static inline void reset_rbc_pulse_sum_caches(rigid_body_constraint_angular_actuator * rbc)
{
	reset_pulse_sum_caches(rbc->m_ps_cache_list);
}
static inline void reset_rbc_pulse_sum_caches(rigid_body_constraint_upright * rbc)
{
	reset_pulse_sum_caches(rbc->m_ps_cache_list);
}
static inline void reset_rbc_pulse_sum_caches(rigid_body_constraint_custom_orientation * rbc)
{
	reset_pulse_sum_caches(rbc->m_ps_cache_list);
}
static inline void reset_rbc_pulse_sum_caches(rigid_body_constraint_custom_path * rbc)
{
	reset_pulse_sum_caches(rbc->m_list_psc);
}
static inline void reset_rbc_pulse_sum_caches(rigid_body_constraint_contact *) {}

// ---------------------------------------------------------------------------------------------
// rbint::setup_constraint
// ---------------------------------------------------------------------------------------------

void rbint::setup_constraint(rigid_body * rb, pulse_sum_node * psn)
{
	PHYS_ASSERT(rb);
	PHYS_ASSERT(rb->get_inv_mass() > 0.00001f);
	nuge::tensor_transform_principle(rb->m_inv_inertia, rb->m_mat, &psn->m_world_inv_inertia);
	rb->m_node = psn;
	psn->m_rb = rb;
	psn->m_inv_mass = rb->m_inv_mass;
}

// ---------------------------------------------------------------------------------------------
// Job entry points
// ---------------------------------------------------------------------------------------------

void constraint_solver_process(phys_transient_allocator * transient_buffer, physics_system * psys, float outside_delta_t)
{
	rigid_body ** const list_island = psys->m_list_island;
	const int list_island_count = psys->m_list_island_count;
	if (list_island_count > 0)
	{
		g_list_island_cur = 0;
		constraint_solver_task_input * const input = (constraint_solver_task_input *)transient_buffer->allocate(
			(int)sizeof(constraint_solver_task_input), (int)TL_ALIGNOF(constraint_solver_task_input), false,
			"phys_transient_allocator out of memory.");
		input->m_list_island = list_island;
		input->m_list_island_count = list_island_count;
		input->m_list_island_cur = &g_list_island_cur;
		input->m_psys_max_vel_iters = psys->m_max_vel_iters;
		input->m_psys_max_vel_pos_iters = psys->m_max_vel_pos_iters;
		input->m_outside_delta_t = outside_delta_t;
		// At most 6 jobs; tlRunJobs returns when all are done, so there is nothing left to flush.
		tlRunJobs(phys_jq_constraint_solver_batch_function, input, tl_min(list_island_count, 6));
	}
}

// One solver per job (job_index is unused); islands are taken off the shared counter until it runs past the end.
void phys_jq_constraint_solver_batch_function(void * user_data, int)
{
	const constraint_solver_task_input * const input = (const constraint_solver_task_input *)user_data;
	pulse_sum_constraint_solver solver;
	solver.m_outside_delta_t = input->m_outside_delta_t;
	solver.m_psys_max_vel_pos_iters = input->m_psys_max_vel_pos_iters;
	solver.m_psys_max_vel_iters = input->m_psys_max_vel_iters;
	solver.m_memory_high_water = 0;
	for (int i = tlAtomicExchangeAdd(input->m_list_island_cur, 1); i < input->m_list_island_count;
		i = tlAtomicExchangeAdd(input->m_list_island_cur, 1))
	{
		solver.execute_constraint_solver(input->m_list_island[i]);
	}
}

// ---------------------------------------------------------------------------------------------
// pulse_sum_constraint_solver
// ---------------------------------------------------------------------------------------------

// Gauss-Seidel sweeps over every row until the largest (impulse change * denom)^2 of a sweep is at most
// max_error_sq. The bound is "iters <= max_iters", so up to max_iters + 1 sweeps run (the first sweep always
// runs: error_sq starts at twice the tolerance). A NaN error ends the loop.
void pulse_sum_constraint_solver::solve_iterative(int max_iters, float max_error_sq)
{
	float error_sq = max_error_sq + max_error_sq;
	int iters = 0;
	while (iters <= max_iters && error_sq > max_error_sq)
	{
		iters++;
		error_sq = 0.0f;
		for (pulse_sum_normal * psn = m_list_pulse_sum_normal.get_first(); psn; psn = psn->m_next_link)
			psn->SOLVER_apply_relaxation(error_sq, true);
		for (pulse_sum_point * psp = m_list_pulse_sum_point.get_first(); psp; psp = psp->m_next_link)
			psp->SOLVER_apply_relaxation(error_sq);
		for (pulse_sum_angular * psa = m_list_pulse_sum_angular.get_first(); psa; psa = psa->m_next_link)
			psa->SOLVER_apply_relaxation(error_sq);
		for (pulse_sum_wheel * psw = m_list_pulse_sum_wheel.get_first(); psw; psw = psw->m_next_link)
			psw->SOLVER_apply_relaxation(error_sq);
		for (pulse_sum_contact * psc = m_list_pulse_sum_contact.get_first(); psc; psc = psc->m_next_link)
		{
			pulse_sum_contact_point * const end = &psc->m_list_pscp[psc->m_list_pscp_count];
			for (pulse_sum_contact_point * pscp = psc->m_list_pscp; pscp != end; ++pscp)
				pscp->SOLVER_apply_relaxation(psc, error_sq);
		}
	}
}

// Point the constraint's user body (b1 checked first; only one of the two is replaced) at this solve's
// temp copy, creating the copy on first use, and remember how to undo it.
void pulse_sum_constraint_solver::add_urb(turb_search_tree_t * turb_search_tree, turb_list_t * list_turb,
	urbri_list_t * list_urbri, rigid_body_constraint * rbc)
{
	rigid_body ** rbc_urb;
	if (rbc->b1 && rbc->b1->is_user_rigid_body())
		rbc_urb = &rbc->b1;
	else if (rbc->b2 && rbc->b2->is_user_rigid_body())
		rbc_urb = &rbc->b2;
	else
		return;

	user_rigid_body * const urb = static_cast<user_rigid_body *>(*rbc_urb);
	temp_user_rigid_body * turb = turb_search_tree->find(urb);
	if (!turb)
	{
		turb = (temp_user_rigid_body *)m_solver_memory_allocator.allocate((int)sizeof(temp_user_rigid_body), 16, false,
			SOLVER_MEMORY_ALLOCATOR_ERROR_MSG);
		static_cast<user_rigid_body &>(*turb) = *urb;
		turb_search_tree->add(urb, turb);
		list_turb->add(turb);
	}

	user_rigid_body_restore_info * const urbri = (user_rigid_body_restore_info *)m_solver_memory_allocator.allocate(
		(int)sizeof(user_rigid_body_restore_info), (int)TL_ALIGNOF(user_rigid_body_restore_info), false,
		SOLVER_MEMORY_ALLOCATOR_ERROR_MSG);
	urbri->m_rbc_urb = rbc_urb;
	urbri->m_original_urb = urb;
	*rbc_urb = turb;
	list_urbri->add(urbri);
}

// One sub-step: integrate external forces, run the velocity phase and the velocity+position phase,
// then integrate positions. Rows were built by the setup_constraint calls just before.
void pulse_sum_constraint_solver::solve_constraints(rigid_body * const head)
{
	for (pulse_sum_node * psn = m_list_pulse_sum_node.get_first(); psn; psn = psn->m_next_link)
	{
		rbint::euler_integrate_velocity(psn->m_rb, m_si.m_delta_t);
		psn->t_vel = PHYS_ZERO_VEC;
		psn->a_vel = PHYS_ZERO_VEC;
	}

	// velocity phase: right sides become (target - current) velocity; warm start from the caches
	for (pulse_sum_normal * psn = m_list_pulse_sum_normal.get_first(); psn; psn = psn->m_next_link)
		psn->SOLVER_solver_prolog(m_si.m_delta_t);
	for (pulse_sum_point * psp = m_list_pulse_sum_point.get_first(); psp; psp = psp->m_next_link)
		psp->SOLVER_solver_prolog(m_si.m_delta_t);
	for (pulse_sum_angular * psa = m_list_pulse_sum_angular.get_first(); psa; psa = psa->m_next_link)
		psa->SOLVER_solver_prolog(m_si.m_delta_t);
	for (pulse_sum_wheel * psw = m_list_pulse_sum_wheel.get_first(); psw; psw = psw->m_next_link)
		psw->SOLVER_solver_prolog(m_si.m_delta_t);
	for (pulse_sum_contact * psc = m_list_pulse_sum_contact.get_first(); psc; psc = psc->m_next_link)
	{
		const float delta_t = m_si.m_delta_t;
		contact_point_info::pulse_sum_cache_info * psci = (contact_point_info::pulse_sum_cache_info *)psc->m_pulse_sum_cache_list;
		pulse_sum_contact_point * const end = &psc->m_list_pscp[psc->m_list_pscp_count];
		for (pulse_sum_contact_point * pscp = psc->m_list_pscp; pscp != end; ++pscp, ++psci)
			pscp->SOLVER_solver_prolog(psc, psci->m_ps_cache_list, delta_t);
	}
	solve_iterative(m_si.m_max_vel_iters, m_si.m_max_vel_error_sq);

	// Every constraint cache is cleared; the rows built this sub-step write theirs back below, so a row that
	// was not built (limit inactive, wheel off the ground) starts cold next time.
	for_each_island_constraint(head, [](auto * rbc)
		{ reset_rbc_pulse_sum_caches(rbc); });

	// vel+pos phase: caches <- velocity-phase impulse rates, right sides += position correction
	for (pulse_sum_normal * psn = m_list_pulse_sum_normal.get_first(); psn; psn = psn->m_next_link)
		psn->SOLVER_solver_intermediate(m_si.m_delta_t);
	for (pulse_sum_point * psp = m_list_pulse_sum_point.get_first(); psp; psp = psp->m_next_link)
		psp->SOLVER_solver_intermediate(m_si.m_delta_t);
	for (pulse_sum_angular * psa = m_list_pulse_sum_angular.get_first(); psa; psa = psa->m_next_link)
		psa->SOLVER_solver_intermediate(m_si.m_delta_t);
	for (pulse_sum_wheel * psw = m_list_pulse_sum_wheel.get_first(); psw; psw = psw->m_next_link)
		psw->SOLVER_solver_intermediate(m_si.m_delta_t);
	for (pulse_sum_contact * psc = m_list_pulse_sum_contact.get_first(); psc; psc = psc->m_next_link)
		psc->SOLVER_solver_intermediate(m_si.m_delta_t);
	solve_iterative(m_si.m_max_vel_pos_iters, m_si.m_max_vel_pos_error_sq);

	// The node velocity changes (including the position-correction part) go into the bodies.
	bool group_stable = true;
	for (pulse_sum_node * psn = m_list_pulse_sum_node.get_first(); psn; psn = psn->m_next_link)
	{
		rbint::add_vel(psn->m_rb, psn->t_vel, psn->a_vel);
		rbint::euler_integrate_pos(psn->m_rb, m_si.m_delta_t);
		rbint::update_stability(psn->m_rb, m_si.m_delta_t);
		group_stable = group_stable && psn->m_rb->is_stable() && !psn->m_rb->get_flag(rigid_body::DEBUG_FLAG_IN_COLLISION);
	}
	for (pulse_sum_node * psn = m_list_pulse_sum_node.get_first(); psn; psn = psn->m_next_link)
	{
		rigid_body * const rb = psn->m_rb;
		if (group_stable)
			rb->m_flags |= rigid_body::FLAG_GROUP_STABLE;
		else
			rb->m_flags &= ~rigid_body::FLAG_GROUP_STABLE;
	}
}

// Solve one island over the outside step,
// split into enough sub-steps that none exceeds the smallest m_max_delta_t of its bodies.
void pulse_sum_constraint_solver::execute_constraint_solver(rigid_body * const head)
{
	PHYS_ASSERT(head);

	float max_delta_t = head->m_max_delta_t;
	for (rigid_body * rb = head->m_partition_node.m_next_node; rb; rb = rb->m_partition_node.m_next_node)
	{
		if (rb->m_max_delta_t < max_delta_t)
			max_delta_t = rb->m_max_delta_t;
	}
	int sub_steps = (int)(m_outside_delta_t / max_delta_t);
	if (sub_steps * max_delta_t < m_outside_delta_t)
		sub_steps++;
	const float f_sub_steps = (float)sub_steps;
	PHYS_ASSERT_STR(m_outside_delta_t <= max_delta_t * f_sub_steps, "m_outside_delta_t <= max_delta_t * sub_steps");

	const float outside_delta_t = m_outside_delta_t;
	turb_search_tree_t turb_search_tree;
	turb_list_t list_turb;
	urbri_list_t list_urbri;
	PHYS_ASSERT(m_solver_memory_allocator.is_empty());

	for (rigid_body * rb = head; rb; rb = rb->m_partition_node.m_next_node)
	{
		rbint::solver_prolog(rb, outside_delta_t);
		rb->swap_last_position();
	}

	// user bodies: temp copies live for the whole solve (allocated before the saved state)
	for_each_island_constraint(head, [&](rigid_body_constraint * rbc)
		{ add_urb(&turb_search_tree, &list_turb, &list_urbri, rbc); });

	phys_transient_allocator::allocator_state saved_allocator_state;
	m_solver_memory_allocator.get_state(saved_allocator_state);

	// phys_sys::set_v_tol / set_vp_tol set only the iteration budgets; they are spread over the sub-steps.
	// The tolerances are fixed.
	int max_vel_iters = m_psys_max_vel_iters / sub_steps;
	if (max_vel_iters <= 1)
		max_vel_iters = 1;
	m_si.m_max_vel_iters = max_vel_iters;
	int max_vel_pos_iters = m_psys_max_vel_pos_iters / sub_steps;
	if (max_vel_pos_iters <= 1)
		max_vel_pos_iters = 1;
	m_si.m_max_vel_pos_iters = max_vel_pos_iters;
	m_si.m_max_vel_error_sq = 0.115600005f; // phys_sqr(0.34f); 0.1156f would be one ulp low
	m_si.m_max_vel_pos_error_sq = 2.89f; // phys_sqr(1.7f)
	m_si.m_delta_t = outside_delta_t / f_sub_steps;

	for (int sub_step = 0; sub_step < sub_steps; sub_step++)
	{
		m_solver_memory_allocator.reset_to_state(saved_allocator_state);
		m_list_pulse_sum_node.remove_all();
		m_list_pulse_sum_normal.remove_all();
		m_list_pulse_sum_point.remove_all();
		m_list_pulse_sum_angular.remove_all();
		m_list_pulse_sum_wheel.remove_all();
		m_list_pulse_sum_contact.remove_all();

		for (rigid_body * rb = head; rb; rb = rb->m_partition_node.m_next_node)
		{
			pulse_sum_node * const psn = (pulse_sum_node *)m_solver_memory_allocator.allocate((int)sizeof(pulse_sum_node), 16, false,
				SOLVER_MEMORY_ALLOCATOR_ERROR_MSG);
			m_list_pulse_sum_node.add(psn);
			rbint::setup_constraint(rb, psn);
		}

		for_each_island_constraint(head, [&](auto * rbc)
			{ rbc->setup_constraint(this, m_si.m_delta_t); });

		solve_constraints(head);

		for (temp_user_rigid_body * turb = list_turb.get_first(); turb; turb = turb->m_next_link)
			rbint::substep(turb, m_si.m_delta_t);

		// distance and angular_actuator have inner updates; the other types use the rigid_body_constraint no-op
		for_each_island_constraint(head, [&](auto * rbc)
			{ rbc->inner_update(m_si.m_delta_t); });

		const int total_memory_allocated = (int)m_solver_memory_allocator.m_total_memory_allocated;
		m_memory_high_water = m_memory_high_water > total_memory_allocated ? m_memory_high_water : total_memory_allocated;
	}

	for (user_rigid_body_restore_info * urbri = list_urbri.get_first(); urbri; urbri = urbri->m_next_link)
		*urbri->m_rbc_urb = urbri->m_original_urb;

	// point, wheel and upright have epilogs; the other types use the rigid_body_constraint no-op
	for_each_island_constraint(head, [&](auto * rbc)
		{ rbc->epilog_vel_constraint(outside_delta_t); });

	// contacts: limit the next sweep against the environment, keep this frame's manifolds as next frame's
	// warm-start source (buffer_2) and empty buffer_1 for the next collision pass
	for (rigid_body_constraint_contact * rbc = head->m_partition_node.m_rbc_contact_first; rbc;
		rbc = static_cast<rigid_body_constraint_contact *>(rbc->m_next))
	{
		rbc->update_smallest_lambda();
		rbc->m_list_contact_point_info_buffer_2.m_first = rbc->m_list_contact_point_info_buffer_1.m_first;
		rbc->m_list_contact_point_info_buffer_1.m_first = NULL;
	}

	for (rigid_body * rb = head; rb; rb = rb->m_partition_node.m_next_node)
	{
		rb->m_force_sum = PHYS_ZERO_VEC;
		rb->m_torque_sum = PHYS_ZERO_VEC;
		rb->update_last_position();
	}

	m_solver_memory_allocator.reset();
}
