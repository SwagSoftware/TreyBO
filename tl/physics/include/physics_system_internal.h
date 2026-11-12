#pragma once

// rbint is declared in rigid_body_internal.h (included below), not here.

#include "tl/base/include/tl_defs.h"
#include "tl/physics/include/phys_assert.h"
#include "tl/physics/include/phys_mem.h"
#include "tl/physics/include/phys_avl_tree.h"
#include "tl/physics/include/phys_transient_allocator.h"
#include "tl/physics/include/rigid_body.h"
#include "tl/physics/include/rigid_body_internal.h"
#include "tl/physics/include/physics_system.h"
#include "tl/physics/include/rbc_defs/rbc_def_base.h"
#include "tl/physics/include/rbc_defs/rbc_def_contact.h"
#include "tl/physics/include/rbc_defs/rbc_def_generic.h"
#include "tl/physics/include/rbc_defs/rbc_def_ragdoll.h"
#include "tl/physics/include/rbc_defs/rbc_def_vehicle.h"
#include "tl/physics/include/rbc_defs/rbc_def_custom.h"

class TL_ALIGN(16) physics_system
{
public:
	// Set by time_step while the collision pass runs; a body created or reset in
	// that window runs its collision prolog immediately (rigid_body::set / user_rigid_body::set test bit 0).
	enum physics_system_flags_e
	{
		FLAG_IN_COLLISION_STEP = (1 << 0),
	};

	int m_flags;
	float m_outside_sub_delta_t; // current sub-step length while FLAG_IN_COLLISION_STEP is set
	phys_collision_callback_t m_collision_callback;
	float m_max_delta_t; // 0.051282052f (1 / 19.5)
	int m_max_vel_iters; // 8
	int m_max_vel_pos_iters; // 4
	environment_rigid_body m_environment_rigid_body;
	phys_inplace_avl_tree<rigid_body_pair_key, rigid_body_constraint_contact, rigid_body_constraint_contact::avl_tree_accessor> m_search_tree_rbc_contact;
	phys_free_list<user_rigid_body> m_list_user_rigid_body;
	phys_free_list<rigid_body> m_list_rigid_body;
#define RBC_DEF_TYPE(class_name, short_name) phys_free_list<class_name> m_list_##short_name;
#include "tl/physics/include/rbc_defs/rbc_def_types.inc"
	phys_transient_allocator m_contact_point_buffer_1; // this sub-step's contact_point_infos
	phys_transient_allocator m_contact_point_buffer_2; // last sub-step's (warm-start source); swapped each time_step
	rigid_body ** m_list_island; // island heads, largest first; valid during constraint_solver_process only
	int m_list_island_count;
	int m_solver_memory_high_water;
	int m_contact_point_buffer_high_water;

	physics_system();

	// rb may be NULL.
	void validate_member(const rigid_body * rb) const
	{
		if (rb)
		{
			if (rb->is_environment_rigid_body())
			{
				PHYS_ASSERT(rb == &m_environment_rigid_body);
			}
			else if (rb->is_user_rigid_body())
			{
				user_rigid_body * const urb = const_cast<user_rigid_body *>(static_cast<const user_rigid_body *>(rb));
				PMM_VALIDATE(phys_free_list<user_rigid_body>::get_T_internal(urb), phys_free_list<user_rigid_body>::get_slot_size(),
					phys_free_list<user_rigid_body>::get_slot_alignment());
			}
			else
			{
				PMM_VALIDATE(phys_free_list<rigid_body>::get_T_internal(const_cast<rigid_body *>(rb)),
					phys_free_list<rigid_body>::get_slot_size(), phys_free_list<rigid_body>::get_slot_alignment());
			}
		}
	}

	void solver_priority_sort(phys_transient_allocator * transient_buffer);
	void generate_partitions_and_stuff(phys_transient_allocator * transient_buffer);
	void time_step(const float outside_delta_t, const bool last_step);
	void frame_advance(const float delta_t);

	static physics_system * create_physics_system();
	static void initialize();
	static void destroy_physics_system(physics_system * psys);
	static void shutdown();
};

extern physics_system * g_physics_system;

// Island partitioning ("in-place partition node", rigid_body::m_partition_node). Every free rigid body starts
// as its own island (head = tail = itself, size 1); each constraint between two free bodies merges their islands
// (smaller into larger). User and environment bodies never belong to an island.
void IPN_partition_process(const rigid_body_constraint & rbc, int * island_count);
void IPN_merge(rigid_body * dest, rigid_body * source);
void IPN_verify_rigid_bodies(rigid_body * rb_partition_head);

// Same algorithm despite the names: sort both halves, then slide the last element of the first half right while
// the second half's elements are better, insertion-sorting each one into the first half.
void rigid_body_island_qsort(rigid_body ** list, const int list_count); // by island size, descending
void merge_sort(rigid_body_constraint_contact ** list, const int list_count); // by m_solver_priority, ascending
