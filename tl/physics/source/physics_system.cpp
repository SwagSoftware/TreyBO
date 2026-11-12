#include "tl/physics/include/physics_system.h"
#include "tl/physics/include/physics_system_internal.h"
#include "tl/physics/include/phys_assert.h"

phys_assert_info pai_create_rbc_contact(0, 2, true);

// Shared by every create_rbc_X except contact: validate the bodies, take a slot, set b1 / b2.
template <class T>
static T * create(phys_free_list<T> & list, T *, rigid_body * const b1, rigid_body * const b2, const int no_error, const char * error_msg)
{
	g_physics_system->validate_member(b1);
	g_physics_system->validate_member(b2);
	T * const rbc = list.add(no_error, error_msg);
	if (rbc)
	{
		rbc->b1 = b1;
		rbc->b2 = b2;
	}
	return rbc;
}

// One contact constraint per body pair:
// returns the existing one, else creates it and adds it to the pair search tree.
rigid_body_constraint_contact * create(phys_free_list<rigid_body_constraint_contact> & list, rigid_body_constraint_contact *,
	rigid_body * const b1, rigid_body * const b2, int no_error, const char * error_msg)
{
	g_physics_system->validate_member(b1);
	g_physics_system->validate_member(b2);
	const rigid_body_pair_key key(b1, b2);
	rigid_body_constraint_contact * rbc = g_physics_system->m_search_tree_rbc_contact.find(key);
	if (rbc)
		return rbc;
	rbc = list.add(true, error_msg); // no assert here; the pai assert below reports it
	if (rbc)
	{
		rbc->b1 = b1;
		rbc->b2 = b2;
		g_physics_system->m_search_tree_rbc_contact.add(key, rbc);
		return rbc;
	}
	if (no_error)
		return rbc;
	PHYS_ASSERT_INFO(pai_create_rbc_contact, rbc || no_error, error_msg);
	return rbc;
}

// ---------------------------------------------------------------------------------------------
// Per-type functions. The bodies are identical for every type (create dispatches to the contact overload above).
// ---------------------------------------------------------------------------------------------

#define RBC_DEF_TYPE(class_name, short_name)                                                                     \
	class_name * phys_sys::create_##short_name(rigid_body * const b1, rigid_body * const b2, const int no_error) \
	{                                                                                                            \
		return create(g_physics_system->m_list_##short_name, (class_name *)NULL, b1, b2, no_error,               \
			"OUT_OF_MEMORY, " #short_name ", INCREASE phys_mem_info::m_num_" #short_name ".");                   \
	}                                                                                                            \
	void phys_sys::destroy(class_name * const rbc)                                                               \
	{                                                                                                            \
		g_physics_system->m_list_##short_name.remove(rbc);                                                       \
	}                                                                                                            \
	void phys_sys::destroy_all_##short_name()                                                                    \
	{                                                                                                            \
		g_physics_system->m_list_##short_name.remove_all();                                                      \
	}                                                                                                            \
	int phys_sys::get_##short_name##_alloc_count()                                                               \
	{                                                                                                            \
		return g_physics_system->m_list_##short_name.get_count();                                                \
	}
#include "tl/physics/include/rbc_defs/rbc_def_types.inc"

// ---------------------------------------------------------------------------------------------
// Removal helpers for destroy_all_user_rigid_body / destroy_all_constraint(_with_user_rigid_body).
// Each walks one list and removes the matching constraints (remove(T *) validates the slot first).
// ---------------------------------------------------------------------------------------------

template <class T, class pred_t>
static void remove_constraints_if(phys_free_list<T> & list, pred_t pred)
{
	typename phys_free_list<T>::T_internal_base * i = list.m_dummy_head.m_next_T_internal;
	while (i != &list.m_dummy_head)
	{
		T & rbc = static_cast<typename phys_free_list<T>::T_internal *>(i)->m_data;
		i = i->m_next_T_internal;
		if (pred(rbc))
			list.remove(&rbc);
	}
}

template <class pred_t>
static void remove_constraints_if_all_types(pred_t pred)
{
	remove_constraints_if(g_physics_system->m_list_rbc_point, pred);
	remove_constraints_if(g_physics_system->m_list_rbc_hinge, pred);
	remove_constraints_if(g_physics_system->m_list_rbc_dist, pred);
	remove_constraints_if(g_physics_system->m_list_rbc_ragdoll, pred);
	remove_constraints_if(g_physics_system->m_list_rbc_wheel, pred);
	remove_constraints_if(g_physics_system->m_list_rbc_angular_actuator, pred);
	remove_constraints_if(g_physics_system->m_list_rbc_upright, pred);
	remove_constraints_if(g_physics_system->m_list_rbc_custom_orientation, pred);
	remove_constraints_if(g_physics_system->m_list_rbc_custom_path, pred);
	remove_constraints_if(g_physics_system->m_list_rbc_contact, pred);
}

struct rbc_touches_user_rigid_body
{
	bool operator()(const rigid_body_constraint & rbc) const
	{
		return (rbc.b1 && rbc.b1->is_user_rigid_body()) || (rbc.b2 && rbc.b2->is_user_rigid_body());
	}
};

struct rbc_touches_rigid_body
{
	rigid_body * m_rb;
	bool operator()(const rigid_body_constraint & rbc) const
	{
		return (rbc.b1 && rbc.b1 == m_rb) || (rbc.b2 && rbc.b2 == m_rb);
	}
};

struct rbc_joins_rigid_body_and_user_rigid_body
{
	rigid_body * m_rb;
	bool operator()(const rigid_body_constraint & rbc) const
	{
		return rbc.b1 && rbc.b2 &&
			((rbc.b1 == m_rb && rbc.b2->is_user_rigid_body()) || (rbc.b2 == m_rb && rbc.b1->is_user_rigid_body()));
	}
};

// ---------------------------------------------------------------------------------------------
// phys_sys
// ---------------------------------------------------------------------------------------------

environment_rigid_body * phys_sys::get_environment_rigid_body()
{
	return &g_physics_system->m_environment_rigid_body;
}

void phys_sys::set_max_delta_t(const float max_delta_t)
{
	g_physics_system->m_max_delta_t = max_delta_t;
}

float phys_sys::get_max_delta_t()
{
	return g_physics_system->m_max_delta_t;
}

void phys_sys::set_v_tol(const int max_v_iters)
{
	g_physics_system->m_max_vel_iters = max_v_iters;
}

void phys_sys::get_v_tol(int * max_v_iters)
{
	*max_v_iters = g_physics_system->m_max_vel_iters;
}

void phys_sys::set_vp_tol(const int max_vp_iters)
{
	g_physics_system->m_max_vel_pos_iters = max_vp_iters;
}

void phys_sys::get_vp_tol(int * max_vp_iters)
{
	*max_vp_iters = g_physics_system->m_max_vel_pos_iters;
}

void phys_sys::set_collision_callback(phys_collision_callback_t collision_callback)
{
	g_physics_system->m_collision_callback = collision_callback;
}

void phys_sys::phys_frame_advance(float delta_t)
{
	g_physics_system->frame_advance(delta_t);
}

void phys_sys::phys_init()
{
	physics_system::initialize();
}

void phys_sys::phys_shutdown()
{
	physics_system::shutdown();
}

void phys_sys::set_phys_proftimer_callbacks(const phys_proftimer_callbacks & ppc)
{
	g_phys_proftimer_callbacks = ppc;
}

int phys_sys::get_rigid_body_alloc_count()
{
	return g_physics_system->m_list_rigid_body.get_count();
}

int phys_sys::get_user_rigid_body_alloc_count()
{
	return g_physics_system->m_list_user_rigid_body.get_count();
}

user_rigid_body * phys_sys::get_user_rigid_body(const phys_mat44 * const dictactor)
{
	for (user_rigid_body & rb : g_physics_system->m_list_user_rigid_body)
	{
		PHYS_ASSERT(rb.is_user_rigid_body());
		if (rb.m_dictator == dictactor)
			return &rb;
	}
	return NULL;
}

rigid_body_constraint_contact * phys_sys::get_rbc_contact(rigid_body * const b1, rigid_body * const b2)
{
	g_physics_system->validate_member(b1);
	g_physics_system->validate_member(b2);
	return g_physics_system->m_search_tree_rbc_contact.find(rigid_body_pair_key(b1, b2));
}

// set_no_collision clears b2, so the same wheel is re-tested instead of advancing.
void phys_sys::fixup_wheel_constraints(rigid_body * const rb)
{
	phys_free_list<rigid_body_constraint_wheel> & list = g_physics_system->m_list_rbc_wheel;
	phys_free_list<rigid_body_constraint_wheel>::T_internal_base * i = list.m_dummy_head.m_next_T_internal;
	while (i != &list.m_dummy_head)
	{
		rigid_body_constraint_wheel & rbc = static_cast<phys_free_list<rigid_body_constraint_wheel>::T_internal *>(i)->m_data;
		if (rbc.b2 == rb)
			rbc.set_no_collision();
		else
			i = i->m_next_T_internal;
	}
}

template <class T>
static void count_constraint(phys_free_list<T> & list)
{
	for (T & rbc : list)
	{
		if (rbc.b1)
			++rbc.b1->m_constraint_count;
		if (rbc.b2)
			++rbc.b2->m_constraint_count;
	}
}

static int get_point_pair_count(const phys_simple_link_list<contact_point_info> & list)
{
	int count = 0;
	for (const contact_point_info * cpi = list.get_first(); cpi; cpi = cpi->m_next_link)
		count += cpi->m_point_pair_count;
	return count;
}

// The counting half of physics_system::generate_partitions_and_stuff (same order and rules).
void phys_sys::update_constraint_infos()
{
	physics_system * const psys = g_physics_system;
	psys->m_environment_rigid_body.m_constraint_count = 0;
	psys->m_environment_rigid_body.m_contact_count = 0;
	for (user_rigid_body & rb : psys->m_list_user_rigid_body)
	{
		rb.m_constraint_count = 0;
		rb.m_contact_count = 0;
	}
	for (rigid_body & rb : psys->m_list_rigid_body)
	{
		rb.m_constraint_count = 0;
		rb.m_contact_count = 0;
	}
	count_constraint(psys->m_list_rbc_point);
	count_constraint(psys->m_list_rbc_hinge);
	count_constraint(psys->m_list_rbc_dist);
	count_constraint(psys->m_list_rbc_ragdoll);
	for (rigid_body_constraint_wheel & rbc : psys->m_list_rbc_wheel)
	{
		if (rbc.b1)
			++rbc.b1->m_constraint_count;
		if (rbc.b2)
			++rbc.b2->m_constraint_count;
		if (rbc.get_wheel_is_colliding())
		{
			if (rbc.b1)
				++rbc.b1->m_contact_count;
			if (rbc.b2)
				++rbc.b2->m_contact_count;
		}
	}
	count_constraint(psys->m_list_rbc_angular_actuator);
	count_constraint(psys->m_list_rbc_upright);
	count_constraint(psys->m_list_rbc_custom_orientation);
	count_constraint(psys->m_list_rbc_custom_path);
	for (rigid_body_constraint_contact & rbc : psys->m_list_rbc_contact)
	{
		const int count_1 = get_point_pair_count(rbc.m_list_contact_point_info_buffer_1);
		const int count_2 = get_point_pair_count(rbc.m_list_contact_point_info_buffer_2);
		const int point_count = count_2 <= count_1 ? count_1 : count_2;
		if (rbc.b1)
			rbc.b1->m_contact_count += point_count;
		if (rbc.b2)
			rbc.b2->m_contact_count += point_count;
	}
}

rigid_body * phys_sys::create_rigid_body(const int no_error)
{
	return g_physics_system->m_list_rigid_body.add(no_error, "POOL OUT OF MEMORY, rigid_body, INCREASE phys_mem_info::m_num_rigid_body.");
}

user_rigid_body * phys_sys::create_user_rigid_body(const int no_error)
{
	return g_physics_system->m_list_user_rigid_body.add(no_error, "POOL OUT OF MEMORY, user_rigid_body, INCREASE phys_mem_info::m_num_user_rigid_body.");
}

// User bodies no constraint referenced at the last count (m_constraint_count; contacts are not counted).
void phys_sys::destroy_all_unused_user_rigid_body()
{
	phys_free_list<user_rigid_body> & list = g_physics_system->m_list_user_rigid_body;
	phys_free_list<user_rigid_body>::T_internal_base * i = list.m_dummy_head.m_next_T_internal;
	while (i != &list.m_dummy_head)
	{
		user_rigid_body & rb = static_cast<phys_free_list<user_rigid_body>::T_internal *>(i)->m_data;
		i = i->m_next_T_internal;
		if (!rb.m_constraint_count)
			list.remove(&rb);
	}
}

// Every constraint (including those on user bodies) and every rigid body.
void phys_sys::destroy_all_rigid_body()
{
	g_physics_system->m_list_rbc_point.remove_all();
	g_physics_system->m_list_rbc_hinge.remove_all();
	g_physics_system->m_list_rbc_dist.remove_all();
	g_physics_system->m_list_rbc_ragdoll.remove_all();
	g_physics_system->m_list_rbc_wheel.remove_all();
	g_physics_system->m_list_rbc_angular_actuator.remove_all();
	g_physics_system->m_list_rbc_upright.remove_all();
	g_physics_system->m_list_rbc_custom_orientation.remove_all();
	g_physics_system->m_list_rbc_custom_path.remove_all();
	g_physics_system->m_list_rbc_contact.remove_all();
	g_physics_system->m_list_rigid_body.remove_all();
}

void phys_sys::destroy_all_user_rigid_body()
{
	remove_constraints_if_all_types(rbc_touches_user_rigid_body());
	g_physics_system->m_list_user_rigid_body.remove_all();
}

// Every constraint with rb as b1 or b2 (wheels resting on rb were detached by the fixup first).
void phys_sys::destroy_all_constraint(rigid_body * const rb)
{
	fixup_wheel_constraints(rb);
	rbc_touches_rigid_body pred;
	pred.m_rb = rb;
	remove_constraints_if_all_types(pred);
}

// Only the constraints between rb and a user body.
void phys_sys::destroy_all_constraint_with_user_rigid_body(rigid_body * const rb)
{
	fixup_wheel_constraints(rb);
	rbc_joins_rigid_body_and_user_rigid_body pred;
	pred.m_rb = rb;
	remove_constraints_if_all_types(pred);
}

void phys_sys::destroy(rigid_body * const rb)
{
	destroy_all_constraint(rb);
	g_physics_system->m_list_rigid_body.remove(rb);
}

void phys_sys::destroy(user_rigid_body * const rb)
{
	destroy_all_constraint(rb);
	g_physics_system->m_list_user_rigid_body.remove(rb);
}
