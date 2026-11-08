#pragma once

// Retail: codsrc\tl\physics\include\collision\phys_collision.h (asserts at lines 11 and 29).
// Inline copies: setup_gjk_input_from_pcp 0xa1cea0, phys_collide_do_gjk_collide_and_contact_manifold 0xa2cbd0.
// TREYBO: only the types/inlines the GJK + manifold path needs, plus the narrow-phase driver declarations of
// source/phys_collision_multithreaded.cpp (end of file). source/phys_collision.cpp's only function,
// phys_contact_manifold_process::process 0xa2e080, is defined in phys_contact_manifold.cpp.

#include "tl/physics/include/phys_mem.h"
#include "tl/physics/include/collision/phys_gjk.h"
#include "tl/physics/include/collision/phys_contact_manifold.h"
#include "tl/physics/include/collision/phys_broad_phase.h" // broad_phase_info (must not include this header)

// One broad-phase overlap queued for the narrow phase. Linked through phys_link_list<phys_collision_pair>
// (add_mt 0xa20040), filled by add_collision_pair / add_collision_pair_mutex (0xa20310 / 0xa20410).
class phys_collision_pair : public phys_link_list_base<phys_collision_pair>
{
public:
    broad_phase_info *m_bpi1;
    broad_phase_info *m_bpi2;
    float m_hit_time;           // swept-AABB entry time in [0, 1]; the GJK sweep starts here
    phys_gjk_cache_info *m_gjk_ci; // may be NULL (cache full)

    // TREYBO: reconstructed retail inline ctor; the -1.0f store is seen in add_collision_pair 0xa20310,
    // add_collision_pair_mutex 0xa20410 and do_initial_tunnel_test 0xa1e670.
    phys_collision_pair() : m_hit_time(-1.0f) {}
};

// 0xa1cea0
inline void setup_gjk_input_from_pcp(phys_gjk_input *pgi, phys_collision_pair *pcp)
{
    PHYS_ASSERT(pcp->m_hit_time >= 0.0f && pcp->m_hit_time <= 1.0f);
    const broad_phase_info *const bpi1 = pcp->m_bpi1;
    const broad_phase_info *const bpi2 = pcp->m_bpi2;
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

// 0xa2cbd0. Swept GJK with a 1.02f (0.03 * 34) separation threshold, then the contact manifold on a hit.
inline void phys_collide_do_gjk_collide_and_contact_manifold(phys_collision_pair *pcp, phys_gjk_info *gjk_info, phys_contact_manifold_process *cman_process)
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

// TREYBO: declarations for source/phys_collision_multithreaded.cpp (retail has no header text for these).
class contact_point_info;
extern phys_collision_pair *volatile g_list_pcp_iterator; // next pair to collide (job input)
extern contact_point_info *volatile g_list_output_cpi;    // every job's contact_point_infos, chained

void process_cpi(contact_point_info *cpi); // 0xa2cc70
// 0xa2cd00. Narrow phase of every broad-phase pair (jobs), then the contact constraints (calling thread).
void process_list_do_gjk_collide_and_contact_manifold(phys_link_list<phys_collision_pair> *list_pcd);
// 0xa2cd80. TREYBO: retail took a jqBatch * (jqModule phys_gjk_collide_jqModule); tlJobFunc here.
void phys_gjk_collide_jq_batch_function(void *input, int job_index);
