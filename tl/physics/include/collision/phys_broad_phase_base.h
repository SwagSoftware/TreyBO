#pragma once

// Retail: codsrc\tl\physics\include\collision\phys_broad_phase_base.h (asserts :99-:101 get_bpi / get_bpg /
// get_bpi_env, :163/:164 set_bpi_env). Out-of-line copies: get_bpi 0x5b1a70, get_bpg 0x5b1ab0, get_bpi_env 0x838ee0,
// get_aabb 0xa1cdf0, broad_phase_info::set 0x8391f0, set_bpi_env 0x839300, collision_prolog 0x839410.
//
// broad_phase_base is the swept volume the broad phase works on: the AABB at the start of the step (with
// hace, "half active constraint epsilon", padding: m_trace_aabb_*_whace = "with hace"; see phys_aabb_add_hace)
// plus the translation over the step. broad_phase_info is one collision geom of a rigid body (or, with
// FLAG_IS_BPI_ENV, one piece of world geometry the host handed back); broad_phase_group (phys_broad_phase.h)
// clusters the infos of one body.

#include "tl/base/include/tl_defs.h"
#include "tl/physics/include/phys_math.h"
#include "tl/physics/include/phys_assert.h"
#include "tl/physics/include/rigid_body.h"
#include "tl/physics/include/collision/phys_gjk.h"
#include "tl/physics/include/collision/phys_bounding_volume.h"

class broad_phase_info;
class broad_phase_group;
class phys_auto_activate_callback;

class broad_phase_base
{
public:
    // FLAG_IS_IN_CLUSTER / FLAG_ON_ENV_LIST / FLAG_ON_BPG_LIST are retail assert text; TREYBO: the other names are
    // not recovered (kisakblack names, values are retail).
    enum broad_phase_base_flags_e
    {
        FLAG_IS_BPI = (1 << 0),
        FLAG_IS_BPG = (1 << 1),
        FLAG_IS_BPI_ENV = (1 << 2),   // world geometry returned by the host query (broad_phase_info::set_bpi_env)
        FLAG_IS_IN_CLUSTER = (1 << 4), // claimed by an environment-query cluster this pass
        FLAG_ON_ENV_LIST = (1 << 5),   // linked into G_BPM->g_list_bpb (environment_collision_list_add)
        FLAG_ON_BPG_LIST = (1 << 6),   // a member of a broad_phase_group (broad_phase_group::add_bpi)
    };

    phys_vec3 m_trace_aabb_min_whace;
    phys_vec3 m_trace_aabb_max_whace;
    phys_vec3 m_trace_translation;
    unsigned int m_flags;
    broad_phase_base *m_list_bpb_next;         // g_list_bpb, a group's bpi list, or the transient cluster list
    broad_phase_base *m_list_bpb_cluster_next; // members of one environment-query cluster
    void *m_sap_node;   // axis_aligned_sweep_and_prune::sap_node; for a FLAG_IS_BPI_ENV info, its phys_auto_activate_callback
    void *m_user_data;
    unsigned int m_env_collision_flags;
    unsigned int m_my_collision_type_flags;

    const unsigned int get_flag(const unsigned int f) const { return m_flags & f; }
    void set_flag(const unsigned int f, const bool b) { b ? m_flags |= f : m_flags &= ~f; }

    const unsigned int is_bpi() const { return get_flag(FLAG_IS_BPI); }
    const unsigned int is_bpg() const { return get_flag(FLAG_IS_BPG); }
    const unsigned int is_bpi_env() const { return get_flag(FLAG_IS_BPI_ENV); }

    broad_phase_info *get_bpi();     // :99  (0x5b1a70)
    broad_phase_group *get_bpg();    // :100 (0x5b1ab0), defined in phys_broad_phase.h
    broad_phase_info *get_bpi_env(); // :101 (0x838ee0)

    broad_phase_base *get_bpb_cluster_next() const { return m_list_bpb_cluster_next; }

    // 0xa1cdf0. Swept AABB over the whole step: aabb[0] = min, aabb[1] = max.
    void get_aabb(phys_vec3 *const aabb)
    {
        aabb[0] = phys_min(m_trace_aabb_min_whace, m_trace_aabb_min_whace + m_trace_translation);
        aabb[1] = phys_max(m_trace_aabb_max_whace, m_trace_aabb_max_whace + m_trace_translation);
    }
};

void calc_largest_vel_sq(broad_phase_info *bpi); // source/phys_broad_phase.cpp 0xa1d4a0

class broad_phase_info : public broad_phase_base
{
public:
    // TREYBO: names not recovered (values are retail: set 0x8391f0, set_bpi_env 0x839300).
    enum broad_phase_info_flags_e
    {
        FLAG_HAS_AUTO_ACTIVATE_CALLBACK = (1 << 7), // m_sap_node holds a phys_auto_activate_callback
        FLAG_CALC_CG_TO_WORLD_XFORM = (1 << 9),     // collision_prolog rebuilds *m_cg_to_world_xform
    };

    rigid_body *m_rb;
    const phys_mat44 *m_rb_to_world_xform;
    const phys_mat44 *m_cg_to_world_xform; // written by collision_prolog when FLAG_CALC_CG_TO_WORLD_XFORM is set
    const phys_mat44 *m_cg_to_rb_xform;
    const phys_gjk_geom *m_gjk_geom;
    unsigned int m_gjk_geom_id; // GJK cache key; unique per geom (gjk_unique_id_database_t)
    int m_surface_type;

    // 0x8391f0
    void set(rigid_body *rb, const phys_mat44 *rb_to_world_xform, const phys_mat44 *cg_to_world_xform,
             const phys_mat44 *cg_to_rb_xform, const phys_gjk_geom *gjk_geom, unsigned int gjk_geom_id,
             bool calc_cg_to_world_xform, int surface_type, void *user_data, unsigned int env_collision_flags)
    {
        m_flags = 0;
        m_list_bpb_next = NULL;
        m_sap_node = NULL;
        m_flags |= FLAG_IS_BPI;
        m_rb = rb;
        m_rb_to_world_xform = rb_to_world_xform;
        m_cg_to_world_xform = cg_to_world_xform;
        m_cg_to_rb_xform = cg_to_rb_xform;
        m_gjk_geom = gjk_geom;
        m_gjk_geom_id = gjk_geom_id;
        set_flag(FLAG_CALC_CG_TO_WORLD_XFORM, calc_cg_to_world_xform);
        m_surface_type = surface_type;
        m_user_data = user_data;
        m_env_collision_flags = env_collision_flags;
        m_my_collision_type_flags = 0;
    }

    // :163/:164 (0x839300). Turns a set() info into a world-geometry info. Note it clears every flag, FLAG_IS_BPI too.
    void set_bpi_env(phys_auto_activate_callback *auto_activate_callback)
    {
        PHYS_ASSERT_MSG(is_bpi(), "call broad_phase_info::set first.");
        PHYS_ASSERT(m_sap_node == NULL);
        m_flags = 0;
        m_sap_node = auto_activate_callback;
        m_flags |= FLAG_IS_BPI_ENV;
        set_flag(FLAG_HAS_AUTO_ACTIVATE_CALLBACK, auto_activate_callback != NULL);
    }

    // 0x839410 (the rigid_body.h assert text comes from rb->get_flag). Recomputes the swept volume from the body's
    // pending move (m_moved_vec) and, for free bodies, updates rb->m_largest_vel_sq.
    void collision_prolog()
    {
        if (get_flag(FLAG_CALC_CG_TO_WORLD_XFORM))
            phys_full_multiply_mat(*const_cast<phys_mat44 *>(m_cg_to_world_xform), *m_rb_to_world_xform, *m_cg_to_rb_xform); // TREYBO: retail casts away const too
        m_gjk_geom->calc_aabb(*m_cg_to_world_xform, &m_trace_aabb_min_whace, &m_trace_aabb_max_whace);
        phys_aabb_add_hace(&m_trace_aabb_min_whace, &m_trace_aabb_max_whace);
        m_trace_translation = m_rb->m_moved_vec;
        if (!m_rb->get_flag(rigid_body::FLAG_ENVIRONMENT_RIGID_BODY | rigid_body::FLAG_USER_RIGID_BODY))
            calc_largest_vel_sq(this);
    }
};

inline broad_phase_info *broad_phase_base::get_bpi()
{
    PHYS_ASSERT(is_bpi());
    return static_cast<broad_phase_info *>(this);
}

inline broad_phase_info *broad_phase_base::get_bpi_env()
{
    PHYS_ASSERT(is_bpi_env());
    return static_cast<broad_phase_info *>(this);
}
