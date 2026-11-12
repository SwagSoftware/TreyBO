#pragma once

// Included at the end of phys_broad_phase.h.

#include "tl/physics/include/collision/phys_broad_phase.h"
#include "tl/physics/include/rbc_defs/rbc_def_vehicle.h"

#include <new>

// bpb_to_remove must be on the list.
inline void broad_phase_memory::list_bpb_remove(broad_phase_base * bpb_to_remove)
{
	broad_phase_base ** bpb_cur = &g_list_bpb;
	broad_phase_base * bpb;
	for (bpb = g_list_bpb; bpb != bpb_to_remove; bpb = bpb->m_list_bpb_next)
	{
		PHYS_ASSERT(bpb);
		bpb_cur = &bpb->m_list_bpb_next;
	}
	*bpb_cur = bpb->m_list_bpb_next;
	--g_list_bpb_count;
}

// Runs every member's prolog, then sets the group's swept volume around all members (and the
// vehicle wheel rays, which it also rebuilds into m_list_wci).
inline void broad_phase_group::collision_prolog()
{
	PHYS_ASSERT(m_list_bpi_head);
	m_list_bpi_head->collision_prolog();

	const broad_phase_info * const head = m_list_bpi_head;
	phys_vec3 aabb1_min = head->m_trace_aabb_min_whace; // start of the step
	phys_vec3 aabb1_max = head->m_trace_aabb_max_whace;
	phys_vec3 aabb2_min = head->m_trace_aabb_min_whace + head->m_trace_translation; // end of the step
	phys_vec3 aabb2_max = head->m_trace_aabb_max_whace + head->m_trace_translation;
	m_env_collision_flags = head->m_env_collision_flags;

	for (broad_phase_info * bpi = static_cast<broad_phase_info *>(head->m_list_bpb_next); bpi; bpi = static_cast<broad_phase_info *>(bpi->m_list_bpb_next))
	{
		bpi->collision_prolog();
		aabb1_min = phys_min(aabb1_min, bpi->m_trace_aabb_min_whace);
		aabb1_max = phys_max(aabb1_max, bpi->m_trace_aabb_max_whace);
		aabb2_min = phys_min(aabb2_min, bpi->m_trace_aabb_min_whace + bpi->m_trace_translation);
		aabb2_max = phys_max(aabb2_max, bpi->m_trace_aabb_max_whace + bpi->m_trace_translation);
		m_env_collision_flags |= bpi->m_env_collision_flags;
	}

	if (m_rbvm)
	{
		const int wheel_count = m_rbvm->m_wheels.get_count();
		m_list_wci = (phys_wheel_collide_info *)G_BPM->g_collision_memory_buffer.mt_allocate(wheel_count * (int)sizeof(phys_wheel_collide_info), 16, 0, "broad phase collision out of memory.");
		// Every wheel is placed with the first wheel's chassis matrix (m_wheels[0] is read even when empty).
		const phys_mat44 & rb_mat = m_rbvm->m_wheels[0]->get_b1()->get_mat();
		for (int i = 0; i < wheel_count; ++i)
		{
			phys_wheel_collide_info * const wci = &m_list_wci[i];
			m_rbvm->m_wheels[i]->get_wheel_collide_segment(rb_mat, &wci->m_ray_pos, &wci->m_ray_dir);
			wci->m_ray_dir -= wci->m_ray_pos;
			wci->m_hit_bpi = NULL;
			wci->m_hit_t = 1.0f;
			aabb1_min = phys_min(aabb1_min, wci->m_ray_pos);
			aabb1_max = phys_max(aabb1_max, wci->m_ray_pos);
			const phys_vec3 ray_end = wci->m_ray_pos + wci->m_ray_dir;
			aabb2_min = phys_min(aabb2_min, ray_end);
			aabb2_max = phys_max(aabb2_max, ray_end);
		}
	}

	phys_vec3 p1, p2, half_dims;
	comp_trace_volume(aabb1_min, aabb1_max, aabb2_min, aabb2_max, &p1, &p2, &half_dims);
	m_trace_aabb_min_whace = p1 - half_dims;
	// The max is built from p2, not p1 as the cluster code does.
	m_trace_aabb_max_whace = half_dims + p2;
	m_trace_translation = p2 - p1;
}

// Puts a bpi or bpg on G_BPM->g_list_bpb (once).
inline void environment_collision_list_add(broad_phase_base * bpb)
{
	PHYS_ASSERT(bpb->is_bpi() || bpb->is_bpg());
	PHYS_ASSERT(!bpb->get_flag(broad_phase_base::FLAG_ON_BPG_LIST));
	if (!bpb->get_flag(broad_phase_base::FLAG_ON_ENV_LIST))
	{
		PHYS_ASSERT(bpb->m_list_bpb_next == NULL);
		broad_phase_memory * const bpm = G_BPM;
		bpb->m_list_bpb_next = bpm->g_list_bpb;
		bpm->g_list_bpb = bpb;
		++bpm->g_list_bpb_count;
		bpb->m_flags |= broad_phase_base::FLAG_ON_ENV_LIST;
	}
}

inline void environment_collision_list_remove(broad_phase_base * bpb)
{
	PHYS_ASSERT(bpb->is_bpi() || bpb->is_bpg());
	PHYS_ASSERT(!bpb->get_flag(broad_phase_base::FLAG_ON_BPG_LIST));
	if (bpb->get_flag(broad_phase_base::FLAG_ON_ENV_LIST))
	{
		G_BPM->list_bpb_remove(bpb);
		bpb->m_list_bpb_next = NULL;
		bpb->m_flags &= ~broad_phase_base::FLAG_ON_ENV_LIST;
	}
}

// Symmetric pair -> index into the triangular table.
inline const int surface_type_info_database_get_index(int surface_type_1, int surface_type_2)
{
	PHYS_ASSERT(surface_type_1 >= 0 && surface_type_1 < G_BPM->m_max_num_surface_types);
	PHYS_ASSERT(surface_type_2 >= 0 && surface_type_2 < G_BPM->m_max_num_surface_types);
	const int index = (surface_type_1 < surface_type_2) ? surface_type_1 + surface_type_2 * (surface_type_2 + 1) / 2
														: surface_type_2 + surface_type_1 * (surface_type_1 + 1) / 2;
	PHYS_ASSERT(index >= 0 && index < G_BPM->m_max_num_surface_type_infos);
	return index;
}

inline void surface_type_info_database_set(int surface_type_1, int surface_type_2, const phys_surface_type_info & pst)
{
	PHYS_ASSERT(G_BPM->g_surface_type_info_database);
	G_BPM->g_surface_type_info_database[surface_type_info_database_get_index(surface_type_1, surface_type_2)] = pst;
}

inline const phys_surface_type_info & surface_type_info_database_get(int surface_type_1, int surface_type_2)
{
	PHYS_ASSERT(G_BPM->g_surface_type_info_database);
	return G_BPM->g_surface_type_info_database[surface_type_info_database_get_index(surface_type_1, surface_type_2)];
}

// For the host query (phys_environment.h). Lock-free: callable from any query job.
// The info is raw transient memory: the caller must set() / set_bpi_env() / collision_prolog() it.
inline broad_phase_info * allocate_bpi_env()
{
	tlAtomicExchangeAdd(&G_BPM->m_bpi_env_count, 1);
	void * const mem = G_BPM->g_collision_memory_buffer.mt_allocate((int)sizeof(broad_phase_info), 16, 0, "broad phase collision out of memory.");
	broad_phase_info * const bpi_env = mem ? new (mem) broad_phase_info : NULL;
	broad_phase_info * first;
	do
	{
		first = tlAtomicLoad(&G_BPM->m_list_bpi_env);
		bpi_env->m_list_bpb_cluster_next = first;
	} while (tlAtomicCompareExchange(&G_BPM->m_list_bpi_env, bpi_env, first) != first);
	return bpi_env;
}
