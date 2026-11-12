#pragma once

// The world-geometry boundary of the physics library, gathered in one place.
//
// It adds no mechanism: tl/physics reaches the world only through the abstract types below
// (phys_broad_phase.h). This header documents that contract and includes everything a host needs to meet it.
//
// =============================================================================================================
// 1. Set-up (in this order)
// =============================================================================================================
//    phys_memory_manager_init(buffer, size);          // phys_mem.h
//    phys_task_manager_init();                        // collision/phys_task_manager.h
//    phys_sys::phys_init();                           // physics_system.h
//    broad_phase_memory_info bpmi;                    // e.g. 2048 gjk caches, 2048 sap pairs, 35 surface types
//    broad_phase_system_init(bpmi, should_collide);   // should_collide(a, b) filters object/object pairs (may be NULL)
//    G_BPM->g_broad_phase_terrain_query_callback = &my_query;   // REQUIRED once any bpb is on the environment list
//    surface_type_info_database_set(t1, t2, info);    // every surface-type pair the host's set_cpi_params reads
//    phys_set_cpi_params_callback(my_set_cpi_params); // optional (phys_contact_manifold.h); default: surface table
//    phys_sys::set_collision_callback(broad_phase_default_collision_callback);   // or a host callback, see 4.
//    tlSetSystemCallbacks(...) with RunJobs           // optional; without it every job runs serially
//
// =============================================================================================================
// 2. What the host implements: broad_phase_terrain_query_callback::query
// =============================================================================================================
//    void query(const broad_phase_environment_query_input &bpeqi, broad_phase_environement_query_results *bpeqr);
//
//    Called from up to 6 jobs at once (bp_env_jq_batch_function2), once per cluster of moving bodies, while
//    broad_phase_process runs. It must be thread safe.
//
//    It is handed:
//      bpeqi.trace_aabb_min_wace / _max_wace   the cluster's box at the start of the step (world space, padded)
//      bpeqi.trace_translation                 the box's movement over the step: query the swept volume
//      bpeqi.env_collision_flags               union of the cluster members' m_env_collision_flags
//      bpeqr->m_thread_id                      index of the calling job, in [0, 6): per-job scratch slot
//
//    It returns, by calling bpeqr->add(bpi_env) once per world geom that overlaps the swept volume, one
//    FLAG_IS_BPI_ENV broad_phase_info per geom. Pairs are only made where bpi_env->m_env_collision_flags
//    intersects the moving info's m_env_collision_flags.
//
// =============================================================================================================
// 3. How a world geom is returned
// =============================================================================================================
//    a. Key:      bpei_database_id id(host_word1, host_word2);   // unique and stable per geom (e.g. entity, geom ptr)
//    b. Cache:    broad_phase_environment_info *bpei = G_BPM->g_bpei_database.get_bpei_mt(id);
//                 if bpei->m_data: another job (or cluster) built it this pass
//                   -> bpeqr->add((broad_phase_base *)bpei->m_data); done.
//                 else bpei->m_mutex.Lock(); re-test m_data (unlock + add if set meanwhile).
//    c. Geom:     a phys_gjk_geom (collision/phys_gjk.h) in its own local frame, alive until broad_phase_reset_buffer
//                 (e.g. from per-job scratch). Its GJK cache id is bpei->m_gjk_geom_id (stable while the
//                 geom is queried every pass, so warm starts survive between passes).
//    d. Info:     broad_phase_info *bpi_env = allocate_bpi_env();           // phys_broad_phase_inline.h, lock-free
//                 bpi_env->set(rb, rb_to_world, cg_to_world, cg_to_rb, geom, bpei->m_gjk_geom_id,
//                              false, surface_type, user_data, env_collision_flags);
//                   rb           phys_sys::get_environment_rigid_body() for static world, or the entity's
//                                user_rigid_body (phys_sys::get_user_rigid_body(&entity_mat)) for moving brush models
//                   rb_to_world  that body's matrix (identity for the static world)
//                   cg_to_world  geom frame -> world; cg_to_rb geom frame -> rb frame (all three must outlive the pass)
//                   surface_type index into the surface-type table (0 <= t < bpmi.m_max_num_surface_types)
//                 bpi_env->set_bpi_env(aac);    // aac: phys_auto_activate_callback * or NULL (see 5.)
//                 bpi_env->collision_prolog();  // computes its box and translation from rb->m_moved_vec
//                 bpei->m_data = bpi_env; bpei->m_mutex.Unlock();
//                 bpeqr->add(bpi_env);
//    Entries not returned during a pass are dropped at its end (bpei_database_t::update_database).
//    check_terrain_query_params warns when a cluster moves more than 510 units in one step.
//
// =============================================================================================================
// 4. The collision pass
// =============================================================================================================
//    physics_system::time_step calls the collision callback once per sub-step. The minimal callback is
//    broad_phase_default_collision_callback below (a host callback may add effects and debug drawing
//    between the two calls). broad_phase_reset_buffer frees every bpi_env, pair and wheel info of the pass, so
//    host pointers into them die there. contact_point_infos and rigid_body_constraint_contacts are created on the
//    calling thread (process_cpi); the manifold calls g_phys_set_cpi_params for each new contact.
//
// =============================================================================================================
// 5. Optional host pieces
// =============================================================================================================
//    phys_auto_activate_callback   trigger on a world geom: auto_activate(bpi) runs once (under
//                                  G_BPM->g_bp_auto_activate_mutex) when a free body's geom really intersects it;
//                                  no contacts are made against that geom. It may create new bodies: new bpbs on
//                                  the environment list are processed in the same pass.
//    should_collide_callback       object/object filter for the sweep and prune (broad_phase_system_init).
//    broad_phase_collision_pair    explicit pairs (create_broad_phase_collision_pair), tested every pass.
//    surface types                 surface_type_info_database_set / _get (friction, bounce, priority per pair).
//
// =============================================================================================================
// 6. Moving bodies (host-owned, not world geometry)
// =============================================================================================================
//    one geom:   bpi = create_broad_phase_info(); bpi->set(rb, &rb->get_mat(), cg_to_world, cg_to_rb, geom,
//                g_gjk_unique_id_database.get_unique_id(), calc_cg_to_world, surface_type, user_data, flags);
//                environment_collision_list_add(bpi) (world collision) and/or aasap_list_add(bpi) (object collision).
//    several:    bpg = create_broad_phase_group(); bpg->set(); bpg->add_bpi(bpi) per geom; optional bpg->m_rbvm
//                (vehicle wheels) and FLAG_DO_INITIAL_TUNNEL_TEST; then list the group, not its members.
//    teardown:   aasap_list_remove / destroy_broad_phase_info / destroy_broad_phase_group /
//                destroy_broad_phase_info_list (a group's members).
//    A body with a sap node must be on the environment list too: only g_list_bpb members get update_ae_val.

#include "tl/physics/include/collision/phys_broad_phase.h"
#include "tl/physics/include/collision/phys_collision.h"
#include "tl/physics/include/collision/phys_contact_manifold.h"
#include "tl/physics/include/collision/phys_task_manager.h"

// The minimal collision callback (phys_sys::set_collision_callback).
inline void broad_phase_default_collision_callback()
{
	broad_phase_process();
	broad_phase_reset_buffer();
}
