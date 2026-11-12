#pragma once

// Narrow phase: GJK + contact manifold per broad-phase pair.

#include "tl/physics/include/phys_mem.h"
#include "tl/physics/include/collision/phys_gjk.h"
#include "tl/physics/include/collision/phys_contact_manifold.h"
#include "tl/physics/include/collision/phys_broad_phase.h" // broad_phase_info (must not include this header)

// One broad-phase overlap queued for the narrow phase. Linked through phys_link_list<phys_collision_pair>,
// filled by add_collision_pair / add_collision_pair_mutex.
class phys_collision_pair : public phys_link_list_base<phys_collision_pair>
{
public:
	broad_phase_info * m_bpi1;
	broad_phase_info * m_bpi2;
	float m_hit_time; // swept-AABB entry time in [0, 1]; the GJK sweep starts here
	phys_gjk_cache_info * m_gjk_ci; // may be NULL (cache full)

	phys_collision_pair() :
		m_hit_time(-1.0f) {}
};

inline void setup_gjk_input_from_pcp(phys_gjk_input * pgi, phys_collision_pair * pcp)
{
	PHYS_ASSERT(pcp->m_hit_time >= 0.0f && pcp->m_hit_time <= 1.0f);
	const broad_phase_info * const bpi1 = pcp->m_bpi1;
	const broad_phase_info * const bpi2 = pcp->m_bpi2;
	pgi->m_cg1_translation = bpi1->m_trace_translation;
	pgi->m_cg2_translation = bpi2->m_trace_translation;
	pgi->m_start_time = pcp->m_hit_time;
	pgi->m_end_time = 1.0f;
	pgi->gjk_cg1 = bpi1->m_gjk_geom;
	pgi->gjk_cg2 = bpi2->m_gjk_geom;
	pgi->cg1_to_world_xform = bpi1->m_cg_to_world_xform;
	pgi->cg2_to_world_xform = bpi2->m_cg_to_world_xform;
	pgi->gjk_ci = pcp->m_gjk_ci;
	pgi->cg1_radius = pgi->gjk_cg1->get_geom_radius() + PHYS_CONTACT_SKIN;
	pgi->cg2_radius = pgi->gjk_cg2->get_geom_radius() + PHYS_CONTACT_SKIN;
}

// Swept GJK with a 1.02f (0.03 * 34) separation threshold, then the contact manifold on a hit.
inline void phys_collide_do_gjk_collide_and_contact_manifold(phys_collision_pair * pcp, phys_gjk_info * gjk_info, phys_contact_manifold_process * cman_process)
{
	PHYS_ASSERT(pcp->m_hit_time >= 0.0f && pcp->m_hit_time <= 1.0f);
	phys_gjk_input pgi;
	setup_gjk_input_from_pcp(&pgi, pcp);
	pgi.m_sep_thresh = 1.02f;
	pgi.m_intersection_test_only = false;
	pgi.m_continuous_collision = true;
	if (gjk_info->phys_collide_do_gjk_collide(&pgi))
		cman_process->process(pcp, gjk_info);
}

// phys_collision_multithreaded.cpp
class contact_point_info;
extern phys_collision_pair * volatile g_list_pcp_iterator; // next pair to collide (job input)
extern contact_point_info * volatile g_list_output_cpi; // every job's contact_point_infos, chained

void process_cpi(contact_point_info * cpi);
// Narrow phase of every broad-phase pair (jobs), then the contact constraints (calling thread).
void process_list_do_gjk_collide_and_contact_manifold(phys_link_list<phys_collision_pair> * list_pcd);
// tlJobFunc (tl_system.h).
void phys_gjk_collide_jq_batch_function(void * input, int job_index);
