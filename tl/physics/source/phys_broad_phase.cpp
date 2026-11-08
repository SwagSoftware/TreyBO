// Retail: codsrc\tl\physics\source\phys_broad_phase.cpp ("source/phys_broad_phase.cpp" asserts), TU 0xa1cbd0-0xa218a0
// (comp_trace_volume / get_aabb / setup_gjk_input_from_pcp COMDATs first, phys_wheel_collide_info at 0xa1cf90).
// Optimized x87 build: float subexpressions stayed in 80-bit registers; TreyBO evaluates them in float.

#include "tl/physics/include/collision/phys_broad_phase.h"
#include "tl/physics/include/collision/phys_collision.h"
#include "tl/physics/include/collision/phys_task_manager.h"
#include "tl/physics/include/rbc_defs/rbc_def_vehicle.h"
#include "tl/base/include/tl_system.h"

#include <new>

broad_phase_memory *G_BPM;
axis_aligned_sweep_and_prune *g_axis_aligned_sweep_and_prune;
gjk_unique_id_database_t g_gjk_unique_id_database = { 1 };

// TREYBO FIX(0xc56f20..0xc56f80): kisakblack aggregate-initialised { 0, 2, true } into the hit counters and never
// registered the sites; retail data is (max_hits_total, max_hits_per_frame, warnings only), linked by the ctor.
phys_assert_info pai_check_terrain_query_params(0, 2, true);
phys_assert_info pai_max_num_sap_active_pair(0, 2, true);
phys_assert_info pai_create_sap_active_pair(0, 1, true);
phys_assert_info pai_create_sap_gjk_ci(0, 1, true);

// Job state of broad_phase_process_object_environment_collision.
broad_phase_base **g_bpb_ptr_list;          // prolog jobs: every bpb of the slice
broad_phase_base *volatile g_bpb_list_cur;  // query jobs: next cluster to take
volatile int g_bpb_list_index;
volatile int g_bpb_list_max_index;
tlAtomicMutex g_prolog_task_mutex;
int g_bpb_cluster_sort_axis = -1;
volatile int g_thread_id;

// Cluster cell size: 4 * 34 (BO1 world scale).
static const float MAX_CLUSTER_SIZE = 136.0f; // TREYBO: name not recovered

// ---------------------------------------------------------------------------------------------
// Swept volumes
// ---------------------------------------------------------------------------------------------

// 0xa1cbd0. The smallest box swept from p1 to p2 (half extents half_dims) that covers aabb1 at the start and aabb2
// at the end: the box size is the larger of the two, its centres are clamped into the union.
void comp_trace_volume(const phys_vec3 &aabb1_min, const phys_vec3 &aabb1_max, const phys_vec3 &aabb2_min, const phys_vec3 &aabb2_max,
                       phys_vec3 *p1, phys_vec3 *p2, phys_vec3 *half_dims)
{
    const phys_vec3 total_half_dims = phys_max(aabb1_max - aabb1_min, aabb2_max - aabb2_min) * 0.5f;
    *half_dims = total_half_dims;
    const phys_vec3 cmin = phys_min(aabb1_min, aabb2_min) + total_half_dims;
    const phys_vec3 cmax = phys_max(aabb1_max, aabb2_max) - total_half_dims;
    *p1 = (aabb1_min + aabb1_max) * 0.5f;
    *p1 = phys_min(*p1, cmax);
    *p1 = phys_max(*p1, cmin);
    *p2 = (aabb2_min + aabb2_max) * 0.5f;
    *p2 = phys_min(*p2, cmax);
    *p2 = phys_max(*p2, cmin);
}

// ---------------------------------------------------------------------------------------------
// phys_wheel_collide_info
// ---------------------------------------------------------------------------------------------

// 0xa1cf90. Ray-casts the wheel segment against one world geom; keeps the closest hit.
void phys_wheel_collide_info::collision_process(broad_phase_info *bpi)
{
    const phys_mat44 &cg_to_world_xform = *bpi->m_cg_to_world_xform;
    const phys_vec3 ray_pos_loc = phys_full_inv_multiply(cg_to_world_xform, m_ray_pos);
    const phys_vec3 ray_dir_loc = phys_inv_multiply(cg_to_world_xform, m_ray_dir);
    float hit_t;
    phys_vec3 hitn;
    if (bpi->m_gjk_geom->ray_cast(ray_pos_loc, ray_dir_loc, m_hit_t, &hit_t, &hitn))
    {
        PHYS_ASSERT(hit_t <= m_hit_t);
        m_hit_t = hit_t;
        m_hitn = hitn;
        m_hit_bpi = bpi;
    }
}

// 0xa1d0a0. Hands the closest hit (in the hit body's frame) to the wheel constraint.
void phys_wheel_collide_info::collision_epilog(rigid_body_constraint_wheel *rbc_wheel)
{
    rbc_wheel->set_no_collision();
    if (m_hit_bpi)
    {
        const float nhitn = Abs(m_hitn);
        if (nhitn > PHYS_EPS_1E6)
        {
            m_hitn /= nhitn;
            m_hitn = phys_multiply(*m_hit_bpi->m_cg_to_world_xform, m_hitn);
            const phys_vec3 hitp = m_ray_pos + m_hit_t * m_ray_dir;
            broad_phase_info *const bpi = m_hit_bpi;
            const phys_vec3 b2_hitp_loc = phys_full_inv_multiply(*bpi->m_rb_to_world_xform, hitp);
            const phys_vec3 b2_hitn_loc = phys_inv_multiply(*bpi->m_rb_to_world_xform, m_hitn);
            rbc_wheel->set_collision(bpi->m_rb, b2_hitp_loc, b2_hitn_loc);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// axis_aligned_sweep_and_prune (part 1)
// ---------------------------------------------------------------------------------------------

// 0xa1d1f0. Pushes the node's min/max pair onto the front of each axis list (sorted on the next process()).
void axis_aligned_sweep_and_prune::sap_node::init(broad_phase_base *bpb, axis_element **xlist, axis_element **ylist, axis_element **zlist)
{
    m_bpb = bpb;
    m_updated = 0;
    axis_element **const lists[3] = { xlist, ylist, zlist };
    for (int axis = 0; axis < 3; ++axis)
    {
        axis_element *const ae_min = &m_ae1[axis][0];
        axis_element *const ae_max = &m_ae1[axis][1];
        ae_min->m_node = this;
        ae_min->m_min_max = 0;
        ae_max->m_node = this;
        ae_max->m_min_max = 1;
        ae_min->m_next = ae_max;
        ae_max->m_next = *lists[axis];
        *lists[axis] = ae_min;
    }
}

// 0xa1d270. Overlap in endpoint order on all three axes.
const int axis_aligned_sweep_and_prune::are_overlapping(sap_node *n1, sap_node *n2)
{
    return n1->m_ae1[0][0].m_ae_list_index < n2->m_ae1[0][1].m_ae_list_index
        && n2->m_ae1[0][0].m_ae_list_index < n1->m_ae1[0][1].m_ae_list_index
        && n1->m_ae1[1][0].m_ae_list_index < n2->m_ae1[1][1].m_ae_list_index
        && n2->m_ae1[1][0].m_ae_list_index < n1->m_ae1[1][1].m_ae_list_index
        && n1->m_ae1[2][0].m_ae_list_index < n2->m_ae1[2][1].m_ae_list_index
        && n2->m_ae1[2][0].m_ae_list_index < n1->m_ae1[2][1].m_ae_list_index;
}

// 0xa1d2c0. Unlinks both endpoints of node from one axis list.
void axis_aligned_sweep_and_prune::remove(axis_element **axis_list, sap_node *node)
{
    int count = 0;
    axis_element **ae_i = axis_list;
    for (axis_element *ae = *ae_i; ae && count < 2; ae = *ae_i)
    {
        if (ae->m_node == node)
        {
            *ae_i = ae->m_next;
            ++count;
        }
        else
        {
            ae_i = &ae->m_next;
        }
    }
    PHYS_ASSERT(count == 2);
}

// ---------------------------------------------------------------------------------------------
// Environment query
// ---------------------------------------------------------------------------------------------

// 0xa1d320. The query volume is the cluster's whace volume padded by another PHYS_HACE: the wace volume.
void init_bpeqi(broad_phase_environment_query_input *bpeqi, broad_phase_base *bpb)
{
    bpeqi->trace_aabb_min_wace = bpb->m_trace_aabb_min_whace - phys_vec3(PHYS_HACE);
    bpeqi->trace_aabb_max_wace = bpb->m_trace_aabb_max_whace + phys_vec3(PHYS_HACE);
    bpeqi->trace_translation = bpb->m_trace_translation;
    bpeqi->env_collision_flags = bpb->m_env_collision_flags;
}

// 0xa1d3c0. Rate-limited warning only; the query still runs.
void check_terrain_query_params(broad_phase_base *bpb)
{
    const float MAX_TRANSLATION_SQ = 260100.0f; // phys_sqr(15 * 34)
    const float ntranslation_sq = AbsSquared(bpb->m_trace_translation);
    (void)ntranslation_sq;
    (void)MAX_TRANSLATION_SQ;
    PHYS_ASSERT_INFO(pai_check_terrain_query_params, ntranslation_sq <= MAX_TRANSLATION_SQ, "terrain query translation too large.");
}

// 0xa1d4a0. Upper bound of the speed of any point of the geom: t_vel + a_vel x r, with r towards the geom's support
// point perpendicular to a_vel. Called from broad_phase_info::collision_prolog, possibly from several jobs per body.
// TREYBO FIX(0xa1d4a0): kisakblack's rewrite used 1e-6f (0x358637bd); retail compares against phys_sqr(0.001f)
// (0x358637be), and tests "a_vel_sq >= eps" (NaN takes the translation-only branch).
void calc_largest_vel_sq(broad_phase_info *bpi)
{
    PHYS_ASSERT(!bpi->is_bpi_env());
    rigid_body *const rb = bpi->m_rb;
    PHYS_ASSERT(!rb->get_flag(rigid_body::FLAG_ENVIRONMENT_RIGID_BODY|rigid_body::FLAG_USER_RIGID_BODY));

    const float eps_sq = phys_sqr(0.001f);
    const float a_vel_sq = AbsSquared(rb->m_a_vel);
    const float t_vel_sq = AbsSquared(rb->m_t_vel);
    float lvs;
    if (a_vel_sq >= eps_sq)
    {
        // Direction of the worst point: perpendicular to a_vel, nudged off t_vel when they are parallel.
        phys_vec3 dir = phys_cross(rb->m_t_vel, rb->m_a_vel);
        float ndir_sq = AbsSquared(dir);
        if (ndir_sq < eps_sq)
        {
            dir = phys_cross(rb->m_t_vel + phys_cross(rb->m_a_vel, phys_vec3(1.0f, 0.0f, 0.0f)), rb->m_a_vel);
            ndir_sq = AbsSquared(dir);
            if (ndir_sq < eps_sq)
            {
                dir = phys_cross(rb->m_t_vel + phys_cross(rb->m_a_vel, phys_vec3(0.0f, 2.0f, 0.0f)), rb->m_a_vel);
                ndir_sq = AbsSquared(dir);
            }
        }

        if (eps_sq > ndir_sq)
        {
            lvs = t_vel_sq + a_vel_sq;
        }
        else
        {
            const phys_vec3 dir_loc = phys_inv_multiply(*bpi->m_cg_to_world_xform, dir);
            phys_vec3 support_pt;
            phys_vec3 support_ind;
            bpi->m_gjk_geom->support(dir_loc, &support_pt, &support_ind);
            const phys_vec3 r = phys_full_multiply(*bpi->m_cg_to_world_xform, support_pt) - bpi->m_rb_to_world_xform->GetW();
            lvs = AbsSquared(phys_cross(rb->m_a_vel, r) + rb->m_t_vel);
        }
    }
    else
    {
        lvs = t_vel_sq;
    }

    // Lock-free max (retail CAS on the float bits).
    volatile float *const largest_vel_sq = &rb->m_largest_vel_sq;
    float cur;
    do
    {
        cur = tlAtomicLoad(largest_vel_sq);
        if (cur > lvs)
            break;
    } while (!tlAtomicCompareExchangeBool(largest_vel_sq, lvs, cur));
}

// 0xa1d860. End of the host collision pass: frees this pass's pairs, bpi_envs and wheel infos.
void broad_phase_reset_buffer()
{
    G_BPM->g_collision_memory_buffer.reset();
}

// 0xa1ff00. Prolog of one bpb, then grow info's running bounds by its start volume.
void process_cluster_environment_collision_prolog(broad_phase_base *bpb, broad_phase_base *info)
{
    if (bpb->is_bpi())
        static_cast<broad_phase_info *>(bpb)->collision_prolog();
    else
        bpb->get_bpg()->collision_prolog();
    bpb->m_flags &= ~broad_phase_base::FLAG_IS_IN_CLUSTER;
    bpb->m_list_bpb_cluster_next = NULL;
    info->m_trace_aabb_min_whace = phys_min(info->m_trace_aabb_min_whace, bpb->m_trace_aabb_min_whace);
    info->m_trace_aabb_max_whace = phys_max(info->m_trace_aabb_max_whace, bpb->m_trace_aabb_max_whace);
}

// 0xa1d890 (retail jqBatch function, module bp_env_jq_module1Module). Drains g_bpb_ptr_list, then merges its bounds
// into the shared ones.
// TREYBO FIX(0xa1d890): kisakblack merged with "local < shared ? local : shared"; retail keeps the shared value on
// ties and NaN polarity is "local >= shared ? shared : local" (max: "local <= shared ? shared : local").
void bp_env_jq_batch_function1(void *input, int /*job_index*/)
{
    broad_phase_base bpb; // retail accumulates into a whole local broad_phase_base
    bpb.m_trace_aabb_min_whace = phys_vec3(1.0e30f);
    bpb.m_trace_aabb_max_whace = phys_vec3(-1.0e30f);
    for (int i = tlAtomicExchangeAdd(&g_bpb_list_index, 1); i < g_bpb_list_max_index; i = tlAtomicExchangeAdd(&g_bpb_list_index, 1))
        process_cluster_environment_collision_prolog(g_bpb_ptr_list[i], &bpb);

    broad_phase_prolog_task_input *const bppti = (broad_phase_prolog_task_input *)input;
    g_prolog_task_mutex.Lock();
    phys_vec3 &aabb_min = *bppti->m_aabb_min;
    phys_vec3 &aabb_max = *bppti->m_aabb_max;
    const phys_vec3 &bpb_min = bpb.m_trace_aabb_min_whace;
    const phys_vec3 &bpb_max = bpb.m_trace_aabb_max_whace;
    aabb_min = phys_vec3(bpb_min.x >= aabb_min.x ? aabb_min.x : bpb_min.x,
                         bpb_min.y >= aabb_min.y ? aabb_min.y : bpb_min.y,
                         bpb_min.z >= aabb_min.z ? aabb_min.z : bpb_min.z);
    aabb_max = phys_vec3(bpb_max.x <= aabb_max.x ? aabb_max.x : bpb_max.x,
                         bpb_max.y <= aabb_max.y ? aabb_max.y : bpb_max.y,
                         bpb_max.z <= aabb_max.z ? aabb_max.z : bpb_max.z);
    g_prolog_task_mutex.Unlock();
}

// 0xa1ffb0. Ascending min along g_bpb_cluster_sort_axis.
const int compare_bpb(broad_phase_base *bpb1, broad_phase_base *bpb2)
{
    const float v1 = bpb1->m_trace_aabb_min_whace[g_bpb_cluster_sort_axis];
    return bpb2->m_trace_aabb_min_whace[g_bpb_cluster_sort_axis] > v1;
}

// 0xa1da10. Sort both halves, then walk the second half while it is out of order, sinking each element into place.
void merge_sort_bpb(broad_phase_base **list, int list_count)
{
    if (list_count >= 3)
    {
        const int half = list_count / 2;
        merge_sort_bpb(list, half);
        merge_sort_bpb(&list[half], list_count - half);
        broad_phase_base **const last = &list[list_count];
        for (broad_phase_base **j = &list[half]; j < last; ++j)
        {
            if (!compare_bpb(*j, j[-1]))
                break;
            tl_swap(j[-1], *j);
            for (broad_phase_base **i = j - 1; i > list; --i)
            {
                if (!compare_bpb(*i, i[-1]))
                    break;
                tl_swap(i[-1], *i);
            }
        }
    }
    else if (list_count == 2)
    {
        if (compare_bpb(list[1], list[0]))
            tl_swap(list[0], list[1]);
    }
}

// 0xa1daf0. One slice of g_list_bpb: prolog every bpb (jobs), sort along the widest axis, group neighbours into
// clusters of at most MAX_CLUSTER_SIZE per axis (start and end volumes), then run one host query per cluster (jobs).
void broad_phase_process_object_environment_collision(bpi_environment_collision_info &eci)
{
    PHYS_ASSERT(eci.m_bpb_count > 0);

    phys_transient_allocator transient_buffer;
    broad_phase_base **const bpb_ptr_list = (broad_phase_base **)transient_buffer.allocate(
        eci.m_bpb_count * (int)sizeof(broad_phase_base *), (int)PHYS_ALIGNOF(broad_phase_base *), 0, "phys_transient_allocator out of memory.");
    broad_phase_base **bpb_ptr_cur = bpb_ptr_list;
    for (broad_phase_base *bpb_i = eci.m_bpb_i_start; bpb_i != eci.m_bpb_i_end; bpb_i = bpb_i->m_list_bpb_next)
    {
        PHYS_ASSERT(bpb_ptr_cur - bpb_ptr_list < eci.m_bpb_count);
        *bpb_ptr_cur++ = bpb_i;
    }
    PHYS_ASSERT(bpb_ptr_cur - bpb_ptr_list == eci.m_bpb_count);

    g_bpb_ptr_list = bpb_ptr_list;
    phys_vec3 bp_aabb_min(1.0e30f);
    phys_vec3 bp_aabb_max(-1.0e30f);
    g_bpb_list_index = 0;
    g_bpb_list_max_index = eci.m_bpb_count;
    broad_phase_prolog_task_input bppti;
    bppti.m_aabb_min = &bp_aabb_min;
    bppti.m_aabb_max = &bp_aabb_max;
    phys_task_manager_process(bp_env_jq_batch_function1, &bppti, eci.m_bpb_count);
    phys_task_manager_flush();

    const float dx = bp_aabb_max.x - bp_aabb_min.x;
    const float dy = bp_aabb_max.y - bp_aabb_min.y;
    const float dz = bp_aabb_max.z - bp_aabb_min.z;
    if (dy > dx)
        g_bpb_cluster_sort_axis = (dz <= dy) ? 1 : 2;
    else
        g_bpb_cluster_sort_axis = (dz <= dx) ? 0 : 2;

    merge_sort_bpb(bpb_ptr_list, eci.m_bpb_count);
    G_BPM->g_list_bpb = bpb_ptr_list[0];
    eci.m_bpb_i_start = bpb_ptr_list[0];
    // TREYBO: retail links list[i] -> list[i + 1] up to i = count - 1 (reading one past the array) and then overwrites
    // the last link; same result.
    for (int i = 0; i < eci.m_bpb_count - 1; ++i)
        bpb_ptr_list[i]->m_list_bpb_next = bpb_ptr_list[i + 1];
    bpb_ptr_list[eci.m_bpb_count - 1]->m_list_bpb_next = eci.m_bpb_i_end;

    broad_phase_base *bpb_cluster_list = NULL;
    int bpb_cluster_list_count = 0;
    for (broad_phase_base *bpb_i = eci.m_bpb_i_start; bpb_i != eci.m_bpb_i_end; bpb_i = bpb_i->m_list_bpb_next)
    {
        if (bpb_i->get_flag(broad_phase_base::FLAG_IS_IN_CLUSTER))
            continue;
        PHYS_ASSERT(bpb_i->get_bpb_cluster_next() == NULL);
        bpb_i->m_flags |= broad_phase_base::FLAG_IS_IN_CLUSTER;

        phys_vec3 aabb1_min = bpb_i->m_trace_aabb_min_whace;
        phys_vec3 aabb1_max = bpb_i->m_trace_aabb_max_whace;
        phys_vec3 aabb2_min = bpb_i->m_trace_aabb_min_whace + bpb_i->m_trace_translation;
        phys_vec3 aabb2_max = bpb_i->m_trace_aabb_max_whace + bpb_i->m_trace_translation;
        unsigned int env_collision_flags = bpb_i->m_env_collision_flags;
        broad_phase_base *bpb_cluster_head = bpb_i;
        const float bpb_i_aabb_min = bpb_i->m_trace_aabb_min_whace[g_bpb_cluster_sort_axis];

        for (broad_phase_base *bpb_j = bpb_i->m_list_bpb_next; bpb_j != eci.m_bpb_i_end; bpb_j = bpb_j->m_list_bpb_next)
        {
            if (bpb_j->m_trace_aabb_min_whace[g_bpb_cluster_sort_axis] - bpb_i_aabb_min > MAX_CLUSTER_SIZE)
                break;
            if (bpb_j->get_flag(broad_phase_base::FLAG_IS_IN_CLUSTER))
                continue;
            PHYS_ASSERT(bpb_j->get_bpb_cluster_next() == NULL);

            // TREYBO FIX(0xa1daf0): kisakblack rejected on "extent > 136" (accepting NaN); retail accepts only on
            // "extent <= 136" for all three axes, for the start volume and then the end volume.
            const phys_vec3 new_aabb1_min = phys_min(aabb1_min, bpb_j->m_trace_aabb_min_whace);
            const phys_vec3 new_aabb1_max = phys_max(aabb1_max, bpb_j->m_trace_aabb_max_whace);
            const phys_vec3 dims1 = new_aabb1_max - new_aabb1_min;
            if (dims1.x <= MAX_CLUSTER_SIZE && dims1.y <= MAX_CLUSTER_SIZE && dims1.z <= MAX_CLUSTER_SIZE)
            {
                const phys_vec3 new_aabb2_min = phys_min(aabb2_min, bpb_j->m_trace_aabb_min_whace + bpb_j->m_trace_translation);
                const phys_vec3 new_aabb2_max = phys_max(aabb2_max, bpb_j->m_trace_aabb_max_whace + bpb_j->m_trace_translation);
                const phys_vec3 dims2 = new_aabb2_max - new_aabb2_min;
                if (dims2.x <= MAX_CLUSTER_SIZE && dims2.y <= MAX_CLUSTER_SIZE && dims2.z <= MAX_CLUSTER_SIZE)
                {
                    bpb_j->m_flags |= broad_phase_base::FLAG_IS_IN_CLUSTER;
                    bpb_j->m_list_bpb_cluster_next = bpb_cluster_head;
                    bpb_cluster_head = bpb_j;
                    env_collision_flags |= bpb_j->m_env_collision_flags;
                    aabb1_min = new_aabb1_min;
                    aabb1_max = new_aabb1_max;
                    aabb2_min = new_aabb2_min;
                    aabb2_max = new_aabb2_max;
                }
            }
        }

        // The cluster's own volume: a transient bpb whose cluster list holds the members (no flags set).
        broad_phase_base *const bpb_cluster = new (transient_buffer.allocate((int)sizeof(broad_phase_base), 16, 0, "phys_transient_allocator out of memory.")) broad_phase_base;
        ++bpb_cluster_list_count;
        bpb_cluster->m_list_bpb_next = bpb_cluster_list;
        bpb_cluster_list = bpb_cluster;
        phys_vec3 p1, p2, half_dims;
        comp_trace_volume(aabb1_min, aabb1_max, aabb2_min, aabb2_max, &p1, &p2, &half_dims);
        bpb_cluster->m_trace_aabb_min_whace = p1 - half_dims;
        bpb_cluster->m_trace_aabb_max_whace = half_dims + p1;
        bpb_cluster->m_trace_translation = p2 - p1;
        bpb_cluster->m_list_bpb_cluster_next = bpb_cluster_head;
        bpb_cluster->m_env_collision_flags = env_collision_flags;
    }

    g_bpb_list_cur = bpb_cluster_list;
    g_bpb_list_index = 0;
    g_bpb_list_max_index = bpb_cluster_list_count;
    g_thread_id = 0;
    phys_task_manager_process(bp_env_jq_batch_function2, NULL, bpb_cluster_list_count);
    phys_task_manager_flush();
    transient_buffer.reset();
}

// 0xa1e3f0. Host queries may put new bpbs on the front of g_list_bpb (auto-activation); those are processed as
// a new slice until none are added.
void broad_phase_process_object_environment_collision()
{
    bpi_environment_collision_info eci;
    eci.m_bpb_i_start = G_BPM->g_list_bpb;
    eci.m_bpb_i_end = NULL;
    eci.m_bpb_count = G_BPM->g_list_bpb_count;
    eci.m_bpb_last_count = G_BPM->g_list_bpb_count;
    while (eci.m_bpb_count)
    {
        broad_phase_process_object_environment_collision(eci);
        eci.m_bpb_i_end = eci.m_bpb_i_start;
        const int count = G_BPM->g_list_bpb_count;
        eci.m_bpb_i_start = G_BPM->g_list_bpb;
        eci.m_bpb_count = count - eci.m_bpb_last_count;
        eci.m_bpb_last_count = count;
    }
}

// ---------------------------------------------------------------------------------------------
// axis_aligned_sweep_and_prune (part 2)
// ---------------------------------------------------------------------------------------------

// 0xa1e460. Exchanges two adjacent endpoints. A max moving past a min starts an overlap on this axis.
void axis_aligned_sweep_and_prune::swap(axis_element **a1_ptr, axis_element **a2_ptr)
{
    axis_element *const a1 = *a1_ptr;
    axis_element *const a2 = *a2_ptr;
    PHYS_ASSERT(a1->m_node != a2->m_node);
    PHYS_ASSERT(a1->m_ae_list_index + 1 == a2->m_ae_list_index);
    ++a1->m_ae_list_index;
    --a2->m_ae_list_index;
    if (a1->m_min_max == 1 && a2->m_min_max == 0)
    {
        sap_node *const n2 = a2->m_node;
        if (are_overlapping(a1->m_node, n2))
        {
            if (!m_should_collide_callback || m_should_collide_callback(a1->m_node->m_bpb, n2->m_bpb))
                add_active_pair(a1->m_node, a2->m_node);
        }
    }
    *a1_ptr = a2;
    *a2_ptr = a1;
}

// 0xa1e530. Same scheme as merge_sort_bpb, on m_val, through swap() (keeps the list indices and the pairs current).
// Retail comparisons kept exactly: the outer walk runs while "prev > cur", the sink stops at "prev <= cur".
void axis_aligned_sweep_and_prune::merge_sort(axis_element **list, int list_count)
{
    if (list_count >= 3)
    {
        const int half = list_count / 2;
        merge_sort(list, half);
        merge_sort(&list[half], list_count - half);
        axis_element **const last = &list[list_count];
        for (axis_element **j = &list[half]; j < last; ++j)
        {
            if (!(j[-1]->m_val > (*j)->m_val))
                break;
            swap(j - 1, j);
            for (axis_element **i = j - 1; i > list; --i)
            {
                if (i[-1]->m_val <= (*i)->m_val)
                    break;
                swap(i - 1, i);
            }
        }
    }
    else if (list_count == 2 && list[0]->m_val > list[1]->m_val)
    {
        swap(list, list + 1);
    }
}

// 0xa1e630 / 0xa1e650
void aasap_list_add(broad_phase_base *bpb)
{
    g_axis_aligned_sweep_and_prune->create_sap_node(bpb);
}

void aasap_list_remove(broad_phase_base *bpb)
{
    g_axis_aligned_sweep_and_prune->destroy_sap_node(bpb);
}

// ---------------------------------------------------------------------------------------------
// Object vs environment
// ---------------------------------------------------------------------------------------------

// 0xa1e670. First pass of a new group: move every member only as far as its first time of impact with the
// environment (continuous GJK with a reduced radius), so a fast spawn does not start inside geometry.
void do_initial_tunnel_test(broad_phase_group *bpg, const broad_phase_environement_query_results &bpeqr)
{
    PHYS_ASSERT(bpg->get_flag(broad_phase_group::FLAG_DO_INITIAL_TUNNEL_TEST));
    bpg->m_flags &= ~broad_phase_group::FLAG_DO_INITIAL_TUNNEL_TEST;

    phys_gjk_info gjk_info;
    phys_collision_pair pcp;
    float hit_time = 0.0f;
    for (broad_phase_info *bpi = bpg->m_list_bpi_head; bpi; bpi = static_cast<broad_phase_info *>(bpi->m_list_bpb_next))
    {
        float smallest_lambda = 1.0f;
        // Retail evaluates this in x87 extended precision.
        const float reduced_radius = bpi->m_gjk_geom->get_geom_radius() * 0.9f + PHYS_CONTACT_SKIN;
        const broad_phase_base_list::node *const last_bpi_env_iter = *bpeqr.m_list_bpi_env.m_list_cur;
        for (const broad_phase_base_list::node *bpi_env_iter = bpeqr.m_list_bpi_env.m_list; bpi_env_iter != last_bpi_env_iter; bpi_env_iter = bpi_env_iter->m_next)
        {
            broad_phase_info *const bpi_env = static_cast<broad_phase_info *>(bpi_env_iter->m_bpb);
            if ((bpi_env->m_env_collision_flags & bpi->m_env_collision_flags) && phys_are_potentially_colliding_whace(bpi, bpi_env, &hit_time))
            {
                pcp.m_hit_time = hit_time;
                pcp.m_bpi1 = bpi;
                pcp.m_bpi2 = bpi_env;
                pcp.m_gjk_ci = NULL;
                phys_gjk_input pgi;
                setup_gjk_input_from_pcp(&pgi, &pcp);
                pgi.cg1_radius = reduced_radius;
                pgi.m_sep_thresh = 1.02f;
                pgi.m_intersection_test_only = false;
                pgi.m_continuous_collision = true;
                if (gjk_info.phys_collide_do_gjk_collide(&pgi))
                {
                    // approaching hits only
                    const float approach = phys_dot(gjk_info.cg1_cinfo_loc.m_n, gjk_info.m_cg1_relative_translation_loc);
                    if (approach < 0.0f && smallest_lambda > gjk_info.m_continuous_collision_lambda)
                        smallest_lambda = gjk_info.m_continuous_collision_lambda;
                }
            }
        }
        bpi->m_rb->adjust_col_moved_vec(smallest_lambda);
        bpi->collision_prolog();
    }
}

// 0xa1ebf0. Exact intersection test of two geoms over the step (GJK, intersection only, swept).
const bool bpi_do_gjk_intersect(broad_phase_info *p1, broad_phase_info *p2, float hit_time)
{
    PHYS_ASSERT(hit_time >= 0.0f && hit_time <= 1.0f);
    const unsigned int id1 = p1->m_gjk_geom_id;
    const unsigned int id2 = p2->m_gjk_geom_id;
    PHYS_ASSERT(p1 && p2 && p1 != p2);
    PHYS_ASSERT(id1 != id2);
    phys_collision_pair pcp;
    pcp.m_hit_time = hit_time;
    pcp.m_bpi1 = p1;
    pcp.m_bpi2 = p2;
    pcp.m_gjk_ci = G_BPM->g_phys_gjk_cache_system.get_gjk_cache_info_mutex(id1, id2, G_BPM->g_bp_gjk_cache_mutex, true);
    phys_gjk_input pgi;
    setup_gjk_input_from_pcp(&pgi, &pcp);
    pgi.m_sep_thresh = 1.02f;
    pgi.m_intersection_test_only = true;
    pgi.m_continuous_collision = true;
    phys_gjk_info gjk_info;
    return gjk_info.phys_collide_do_gjk_collide(&pgi);
}

// Auto-activation against one world-geom info (the shared tail of collide_bpi/bpg_environment).
static void bpi_auto_activate(broad_phase_info *bpi, broad_phase_info *bpi_env, phys_auto_activate_callback *aac, const float hit_time) // TREYBO: helper
{
    if (bpi_do_gjk_intersect(bpi, bpi_env, hit_time))
    {
        G_BPM->g_bp_auto_activate_mutex.Lock();
        if (!aac->has_auto_activated())
            aac->auto_activate(bpi);
        G_BPM->g_bp_auto_activate_mutex.Unlock();
    }
}

// 0xa1ed10
void collide_bpi_environment(broad_phase_info *bpi, const broad_phase_environement_query_results &bpeqr)
{
    if (bpeqr.m_env_collision_flags & bpi->m_env_collision_flags)
    {
        const broad_phase_base_list::node *const last_bpi_env_iter = *bpeqr.m_list_bpi_env.m_list_cur;
        for (const broad_phase_base_list::node *bpi_env_iter = bpeqr.m_list_bpi_env.m_list; bpi_env_iter != last_bpi_env_iter; bpi_env_iter = bpi_env_iter->m_next)
        {
            broad_phase_info *const bpi_env = static_cast<broad_phase_info *>(bpi_env_iter->m_bpb);
            float hit_time = 0.0f;
            if ((bpi_env->m_env_collision_flags & bpi->m_env_collision_flags) && phys_are_potentially_colliding_whace(bpi, bpi_env, &hit_time))
            {
                if (!bpi_env->get_flag(broad_phase_info::FLAG_HAS_AUTO_ACTIVATE_CALLBACK))
                {
                    add_collision_pair_mutex(bpi, bpi_env, hit_time, NULL);
                }
                else
                {
                    phys_auto_activate_callback *const aac = (phys_auto_activate_callback *)bpi_env->m_sap_node;
                    if (!aac->has_auto_activated())
                        bpi_auto_activate(bpi, bpi_env, aac, hit_time);
                }
            }
        }
    }
}

// 0xa1edf0. Group test first, then its members (and the vehicle's wheel rays); always ends with the group epilog.
void collide_bpg_environment(broad_phase_group *bpg, const broad_phase_environement_query_results &bpeqr)
{
    float hit_time = 0.0f;
    if (bpeqr.m_env_collision_flags & bpg->m_env_collision_flags)
    {
        if (bpg->get_flag(broad_phase_group::FLAG_DO_INITIAL_TUNNEL_TEST))
            do_initial_tunnel_test(bpg, bpeqr);

        const broad_phase_base_list::node *const last_bpi_env_iter = *bpeqr.m_list_bpi_env.m_list_cur;
        for (const broad_phase_base_list::node *bpi_env_iter = bpeqr.m_list_bpi_env.m_list; bpi_env_iter != last_bpi_env_iter; bpi_env_iter = bpi_env_iter->m_next)
        {
            broad_phase_info *const bpi_env = static_cast<broad_phase_info *>(bpi_env_iter->m_bpb);
            // TREYBO: retail passes the group through the <broad_phase_info, broad_phase_info> instantiation by cast.
            if (!(bpi_env->m_env_collision_flags & bpg->m_env_collision_flags) || !phys_are_potentially_colliding_whace(bpg, bpi_env, &hit_time))
                continue;

            if (!bpi_env->get_flag(broad_phase_info::FLAG_HAS_AUTO_ACTIVATE_CALLBACK))
            {
                for (broad_phase_info *bpi = bpg->m_list_bpi_head; bpi; bpi = static_cast<broad_phase_info *>(bpi->m_list_bpb_next))
                {
                    if ((bpi->m_env_collision_flags & bpi_env->m_env_collision_flags) && phys_are_potentially_colliding_whace(bpi, bpi_env, &hit_time))
                        add_collision_pair_mutex(bpi, bpi_env, hit_time, NULL);
                }
                if (bpg->m_rbvm)
                {
                    PHYS_ASSERT(bpg->m_list_wci);
                    phys_wheel_collide_info *const last_wci = bpg->m_list_wci + bpg->m_rbvm->m_wheels.get_count();
                    for (phys_wheel_collide_info *wci = bpg->m_list_wci; wci != last_wci; ++wci)
                    {
                        // the ray as a point box swept by m_ray_dir
                        if (phys_are_potentially_colliding(wci->m_ray_pos, wci->m_ray_pos, wci->m_ray_dir - bpi_env->m_trace_translation,
                                                           bpi_env->m_trace_aabb_min_whace, bpi_env->m_trace_aabb_max_whace, &hit_time))
                            wci->collision_process(bpi_env);
                    }
                }
            }
            else
            {
                phys_auto_activate_callback *const aac = (phys_auto_activate_callback *)bpi_env->m_sap_node;
                for (broad_phase_info *bpi = bpg->m_list_bpi_head; bpi && !aac->has_auto_activated(); bpi = static_cast<broad_phase_info *>(bpi->m_list_bpb_next))
                {
                    if ((bpi->m_env_collision_flags & bpi_env->m_env_collision_flags) && phys_are_potentially_colliding_whace(bpi, bpi_env, &hit_time))
                        bpi_auto_activate(bpi, bpi_env, aac, hit_time);
                }
            }
        }
    }
    bpg->collision_epilog();
}

// 0xa21720
void process_cluster_environment_collision(broad_phase_base *bpb, const broad_phase_environement_query_results &bpeqr)
{
    PHYS_ASSERT(bpb && bpb->get_bpb_cluster_next());
    for (broad_phase_base *bpb_i = bpb->get_bpb_cluster_next(); bpb_i; bpb_i = bpb_i->get_bpb_cluster_next())
    {
        PHYS_ASSERT_STR(bpb_i->get_flag(broad_phase_base::FLAG_IS_IN_CLUSTER), "bpb->get_flag(broad_phase_base::FLAG_IS_IN_CLUSTER)");
        if (bpb_i->is_bpi())
            collide_bpi_environment(static_cast<broad_phase_info *>(bpb_i), bpeqr);
        else
            collide_bpg_environment(bpb_i->get_bpg(), bpeqr);
    }
}

// 0xa1f070 (retail jqBatch function, module bp_env_jq_module2Module). Takes clusters off g_bpb_list_cur until none
// are left; one host query each. The results' node list is reused across this job's clusters.
void bp_env_jq_batch_function2(void * /*input*/, int /*job_index*/)
{
    broad_phase_environement_query_results bpeqr;
    bpeqr.m_list_bpi_env.m_list = NULL;
    bpeqr.m_list_bpi_env.m_list_cur = &bpeqr.m_list_bpi_env.m_list;
    bpeqr.m_thread_id = tlAtomicExchangeAdd(&g_thread_id, 1);
    for (broad_phase_base *bpb = tlAtomicLoad(&g_bpb_list_cur); bpb; bpb = tlAtomicLoad(&g_bpb_list_cur))
    {
        if (tlAtomicCompareExchange(&g_bpb_list_cur, bpb->m_list_bpb_next, bpb) == bpb)
        {
            check_terrain_query_params(bpb);
            bpeqr.m_list_bpi_env.m_list_cur = &bpeqr.m_list_bpi_env.m_list;
            bpeqr.m_list_bpi_env_count = 0;
            bpeqr.m_env_collision_flags = 0;
            broad_phase_environment_query_input bpeqi;
            init_bpeqi(&bpeqi, bpb);
            G_BPM->g_broad_phase_terrain_query_callback->query(bpeqi, &bpeqr);
            process_cluster_environment_collision(bpb, bpeqr);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Create / destroy
// ---------------------------------------------------------------------------------------------

// 0xa1e850. Uninitialised; the caller runs set().
broad_phase_info *create_broad_phase_info()
{
    return G_BPM->g_list_broad_phase_info.add(false, "phys_free_list error: out of memory.");
}

// 0xa1e8c0
void destroy_broad_phase_info(broad_phase_info *bpi)
{
    environment_collision_list_remove(bpi);
    g_axis_aligned_sweep_and_prune->destroy_sap_node(bpi);
    G_BPM->g_list_broad_phase_info.remove(bpi);
}

// 0xa1e910. Frees a group's member list (the infos must be off every list already).
void destroy_broad_phase_info_list(broad_phase_info *list_bpi)
{
    broad_phase_info *next_bpi;
    for (broad_phase_info *bpi = list_bpi; bpi; bpi = next_bpi)
    {
        PHYS_ASSERT(!bpi->get_flag(broad_phase_base::FLAG_ON_ENV_LIST));
        PHYS_ASSERT(bpi->m_sap_node == NULL);
        next_bpi = static_cast<broad_phase_info *>(bpi->m_list_bpb_next);
        G_BPM->g_list_broad_phase_info.remove(bpi);
    }
}

// 0xa1e9f0
broad_phase_collision_pair *create_broad_phase_collision_pair()
{
    return G_BPM->g_list_broad_phase_collision_pair.add(false, "phys_free_list error: out of memory.");
}

// 0xa1ea60
void destroy_broad_phase_collision_pair(broad_phase_collision_pair *bpcp)
{
    G_BPM->g_list_broad_phase_collision_pair.remove(bpcp);
}

// 0xa1eaa0
void destroy_broad_phase_collision_pair_list(broad_phase_collision_pair *list_bpcp)
{
    broad_phase_collision_pair *next_bpcp;
    for (broad_phase_collision_pair *bpcp = list_bpcp; bpcp; bpcp = next_bpcp)
    {
        next_bpcp = bpcp->m_next_bpcp;
        G_BPM->g_list_broad_phase_collision_pair.remove(bpcp);
    }
}

// 0xa1eb30
broad_phase_group *create_broad_phase_group()
{
    return G_BPM->g_list_broad_phase_group.add(false, "phys_free_list error: out of memory.");
}

// 0xa1eba0. The member infos are not freed (destroy_broad_phase_info_list).
void destroy_broad_phase_group(broad_phase_group *bpg)
{
    environment_collision_list_remove(bpg);
    g_axis_aligned_sweep_and_prune->destroy_sap_node(bpg);
    G_BPM->g_list_broad_phase_group.remove(bpg);
}

// ---------------------------------------------------------------------------------------------
// Process
// ---------------------------------------------------------------------------------------------

// 0xa1f120. The whole broad + narrow phase of one sub-step. The host collision callback calls this and, when
// done with the pass's transient data, broad_phase_reset_buffer.
void broad_phase_process()
{
    G_BPM->m_list_bpi_env = NULL;
    G_BPM->m_bpi_env_count = 0;
    G_BPM->m_bpg_env_count = 0;
    G_BPM->m_bpg_env_bpi_count = 0;
    G_BPM->m_bpi_env_no_database_count = 0;
    tlScratchPadLock();
    PHYS_ASSERT(G_BPM->g_collision_memory_buffer.is_empty());
    G_BPM->g_list_phys_collide_data.remove_all();

    broad_phase_process_object_environment_collision();
    broad_phase_process_collision_pairs();

    for (broad_phase_base *bpb = G_BPM->g_list_bpb; bpb; bpb = bpb->m_list_bpb_next)
    {
        if (bpb->m_sap_node)
            static_cast<axis_aligned_sweep_and_prune::sap_node *>(bpb->m_sap_node)->update_ae_val();
    }
    g_axis_aligned_sweep_and_prune->process();

    // Pairs without a pair-owned cache (everything but sap bpi/bpi) use the shared GJK cache.
    for (phys_collision_pair *pcp = G_BPM->g_list_phys_collide_data.m_first; pcp; pcp = pcp->m_next_link)
    {
        if (!pcp->m_gjk_ci)
            pcp->m_gjk_ci = G_BPM->g_phys_gjk_cache_system.get_gjk_cache_info(pcp->m_bpi1->m_gjk_geom_id, pcp->m_bpi2->m_gjk_geom_id, true);
    }

    const int memory_allocated = (int)G_BPM->g_collision_memory_buffer.m_total_memory_allocated;
    G_BPM->m_memory_high_water = G_BPM->m_memory_high_water > memory_allocated ? G_BPM->m_memory_high_water : memory_allocated;

    process_list_do_gjk_collide_and_contact_manifold(&G_BPM->g_list_phys_collide_data);
    G_BPM->g_list_phys_collide_data.remove_all();
    tlScratchPadUnlock();
    G_BPM->g_phys_gjk_cache_system.update_cache();
    G_BPM->g_bpei_database.update_database();
}

// 0xa1f2c0. After this the host sets G_BPM->g_broad_phase_terrain_query_callback (phys_environment.h).
void broad_phase_system_init(const broad_phase_memory_info &bpmi, bool (*should_collide_callback)(const broad_phase_base *, const broad_phase_base *))
{
    G_BPM = broad_phase_memory::allocate_buffer(bpmi);
    // TREYBO: TL_ALIGNOF instead of retail's 4 (pointers need 8 on x64).
    void *const aasap = PMM_PERM_ALLOCATE((uint)sizeof(axis_aligned_sweep_and_prune), (uint)TL_ALIGNOF(axis_aligned_sweep_and_prune));
    g_axis_aligned_sweep_and_prune = aasap ? new (aasap) axis_aligned_sweep_and_prune : NULL;
    broad_phase_memory *const bpm = G_BPM;
    bpm->g_broad_phase_terrain_query_callback = NULL;
    bpm->g_list_bpb = NULL;
    bpm->g_list_bpb_count = 0;
    bpm->g_list_broad_phase_info.remove_all();
    bpm->g_list_broad_phase_group.remove_all();
    bpm->g_list_broad_phase_collision_pair.remove_all();
    bpm->g_bpei_database.purge_database();
    bpm->m_memory_high_water = 0;
    g_axis_aligned_sweep_and_prune->init_system(bpmi.m_max_num_sap_active_pairs);
    g_axis_aligned_sweep_and_prune->m_should_collide_callback = should_collide_callback;
}

// 0xa1f370. Retail runs the destructor only (both blocks are permanent PMM memory).
void broad_phase_system_shutdown()
{
    G_BPM->~broad_phase_memory();
    G_BPM = NULL;
    g_axis_aligned_sweep_and_prune = NULL;
}

// 0xa1fd40. Hands each wheel its closest hit.
void broad_phase_group::collision_epilog()
{
    if (m_rbvm)
    {
        PHYS_ASSERT(m_list_wci);
        const int wheel_count = m_rbvm->m_wheels.get_count();
        for (int i = 0; i < wheel_count; ++i)
            m_list_wci[i].collision_epilog(m_rbvm->m_wheels[i]);
    }
}

// 0xa1fdf0. Endpoint values = the whole-step AABB.
void axis_aligned_sweep_and_prune::sap_node::update_ae_val()
{
    PHYS_ASSERT(m_updated == 0);
    m_updated = 1;
    phys_vec3 aabb[2];
    m_bpb->get_aabb(aabb);
    m_ae1[0][0].m_val = aabb[0].x;
    m_ae1[0][1].m_val = aabb[1].x;
    m_ae1[1][0].m_val = aabb[0].y;
    m_ae1[1][1].m_val = aabb[1].y;
    m_ae1[2][0].m_val = aabb[0].z;
    m_ae1[2][1].m_val = aabb[1].z;
}

// 0xa1fe80
void axis_aligned_sweep_and_prune::init_system(int max_num_active_pairs)
{
    PHYS_ASSERT(m_active_pair_allocator.get_count() == 0);
    PHYS_ASSERT(m_sap_node_allocator.get_count() == 0);
    m_x_list = NULL;
    m_y_list = NULL;
    m_z_list = NULL;
    m_list_bpi_bpi = NULL;
    m_list_bpi_bpg = NULL;
    m_list_bpg_bpg = NULL;
    m_max_num_active_pairs = max_num_active_pairs;
}

// ---------------------------------------------------------------------------------------------
// Narrow-phase pair lists
// ---------------------------------------------------------------------------------------------

// 0xa20310. Single-threaded append (sap and explicit pairs).
void add_collision_pair(broad_phase_info *bpi1, broad_phase_info *bpi2, float hit_time, phys_gjk_cache_info *gjk_ci)
{
    PHYS_ASSERT(hit_time >= 0.0f && hit_time <= 1.0f);
    // TREYBO: PHYS_ALIGNOF instead of retail's 4 (pointers need 8 on x64).
    phys_collision_pair *const pcp = new (G_BPM->g_collision_memory_buffer.allocate((int)sizeof(phys_collision_pair), (int)PHYS_ALIGNOF(phys_collision_pair), 0,
                                                                                   "phys_transient_allocator out of memory.")) phys_collision_pair;
    pcp->m_hit_time = hit_time;
    pcp->m_bpi1 = bpi1;
    pcp->m_bpi2 = bpi2;
    pcp->m_gjk_ci = gjk_ci;
    G_BPM->g_list_phys_collide_data.add(pcp);
}

// 0xa20410. Append from the environment-query jobs.
void add_collision_pair_mutex(broad_phase_info *bpi1, broad_phase_info *bpi2, float hit_time, phys_gjk_cache_info *gjk_ci)
{
    PHYS_ASSERT(hit_time >= 0.0f && hit_time <= 1.0f);
    phys_collision_pair *const pcp = new (G_BPM->g_collision_memory_buffer.mt_allocate((int)sizeof(phys_collision_pair), (int)PHYS_ALIGNOF(phys_collision_pair), 0,
                                                                                      "broad phase collision out of memory.")) phys_collision_pair;
    pcp->m_hit_time = hit_time;
    pcp->m_bpi1 = bpi1;
    pcp->m_gjk_ci = gjk_ci;
    pcp->m_bpi2 = bpi2;
    G_BPM->g_list_phys_collide_data.add_mt(pcp);
}

// 0xa204e0. New overlap of two nodes; bpi/bpg pairs are stored bpi first.
void axis_aligned_sweep_and_prune::add_active_pair(sap_node *p1, sap_node *p2)
{
    PHYS_ASSERT_INFO(pai_max_num_sap_active_pair, m_active_pair_allocator.get_count() < m_max_num_active_pairs, "max num sap active pairs reached.");
    if (m_active_pair_allocator.get_count() < m_max_num_active_pairs)
    {
        active_pair *const ap = m_active_pair_allocator.allocate();
        if (ap)
        {
            ap->m_p1 = p1;
            ap->m_p2 = p2;
            ap->m_gjk_ci = NULL;
            if (p1->m_bpb->is_bpi())
            {
                if (p2->m_bpb->is_bpi())
                {
                    ap->m_next = m_list_bpi_bpi;
                    m_list_bpi_bpi = ap;
                }
                else
                {
                    ap->m_next = m_list_bpi_bpg;
                    m_list_bpi_bpg = ap;
                }
            }
            else if (p2->m_bpb->is_bpi())
            {
                ap->m_p1 = p2;
                ap->m_p2 = p1;
                ap->m_gjk_ci = NULL;
                ap->m_next = m_list_bpi_bpg;
                m_list_bpi_bpg = ap;
            }
            else
            {
                ap->m_next = m_list_bpg_bpg;
                m_list_bpg_bpg = ap;
            }
        }
        else
        {
            PHYS_ASSERT_INFO(pai_create_sap_active_pair, ap, "list sap active pairs out of memory.");
        }
    }
}

// 0xa20670. Retail does not null-check the node allocation.
void axis_aligned_sweep_and_prune::create_sap_node(broad_phase_base *bpb)
{
    PHYS_ASSERT(bpb->m_sap_node == NULL);
    sap_node *const node = m_sap_node_allocator.allocate();
    node->init(bpb, &m_x_list, &m_y_list, &m_z_list);
    bpb->m_sap_node = node;
}

// Frees a pair and its pair-owned cache entry (never in the cache tree: allocator only, as retail).
static void sap_free_active_pair(phys_simple_allocator<axis_aligned_sweep_and_prune::active_pair> &allocator, axis_aligned_sweep_and_prune::active_pair *ap) // TREYBO: helper
{
    G_BPM->g_phys_gjk_cache_system.m_list_phys_gjk_cache_info_internal.free(static_cast<phys_heap_gjk_cache_system_avl_tree::phys_gjk_cache_info_internal *>(ap->m_gjk_ci));
    allocator.free(ap);
}

// 0xa206e0
void axis_aligned_sweep_and_prune::remove(active_pair **list_ap, sap_node *node)
{
    for (active_pair *ap = *list_ap; ap; ap = *list_ap)
    {
        if (ap->m_p1 == node || ap->m_p2 == node)
        {
            *list_ap = ap->m_next;
            sap_free_active_pair(m_active_pair_allocator, ap);
        }
        else
        {
            list_ap = &ap->m_next;
        }
    }
}

// 0xa20770
void axis_aligned_sweep_and_prune::destroy_sap_node(broad_phase_base *bpb)
{
    sap_node *const node = static_cast<sap_node *>(bpb->m_sap_node);
    if (node)
    {
        remove(&m_list_bpi_bpi, node);
        remove(&m_list_bpi_bpg, node);
        remove(&m_list_bpg_bpg, node);
        remove(&m_x_list, node);
        remove(&m_y_list, node);
        remove(&m_z_list, node);
        m_sap_node_allocator.free(node);
        bpb->m_sap_node = NULL;
    }
}

// 0xa20800. Host-requested pairs.
void broad_phase_process_collision_pairs()
{
    float hit_time = 0.0f;
    for (phys_free_list<broad_phase_collision_pair>::iterator it = G_BPM->g_list_broad_phase_collision_pair.begin(); it != G_BPM->g_list_broad_phase_collision_pair.end(); ++it)
    {
        if (phys_are_potentially_colliding_whace(it->m_bpi1, it->m_bpi2, &hit_time))
            add_collision_pair(it->m_bpi1, it->m_bpi2, hit_time, NULL);
    }
}

// 0xa20ef0. Drops pairs that stopped overlapping; queues the swept-volume hits of the others.
void axis_aligned_sweep_and_prune::process_active_pair_list()
{
    float hit_time;

    active_pair **ap_i = &m_list_bpi_bpi;
    for (active_pair *ap = *ap_i; ap; ap = *ap_i)
    {
        if (!are_overlapping(ap->m_p1, ap->m_p2))
        {
            *ap_i = ap->m_next;
            sap_free_active_pair(m_active_pair_allocator, ap);
            continue;
        }
        ap_i = &ap->m_next;
        broad_phase_info *const bpi1 = ap->m_p1->m_bpb->get_bpi();
        broad_phase_info *const bpi2 = ap->m_p2->m_bpb->get_bpi();
        if (phys_are_potentially_colliding_whace(bpi1, bpi2, &hit_time))
        {
            if (!ap->m_gjk_ci)
            {
                // Pair-owned cache entry: allocator only, dummy key (1, 2), not in the search tree.
                phys_heap_gjk_cache_system_avl_tree::phys_gjk_cache_info_internal *const gjk_ci = G_BPM->g_phys_gjk_cache_system.m_list_phys_gjk_cache_info_internal.allocate();
                if (gjk_ci)
                    gjk_ci->constructor(phys_gjk_geom_id_pair_key(1, 2), false);
                ap->m_gjk_ci = gjk_ci;
            }
            add_collision_pair(bpi1, bpi2, hit_time, ap->m_gjk_ci);
        }
    }

    ap_i = &m_list_bpi_bpg;
    for (active_pair *ap = *ap_i; ap; ap = *ap_i)
    {
        if (!are_overlapping(ap->m_p1, ap->m_p2))
        {
            *ap_i = ap->m_next;
            sap_free_active_pair(m_active_pair_allocator, ap);
            continue;
        }
        ap_i = &ap->m_next;
        broad_phase_info *const bpi = ap->m_p1->m_bpb->get_bpi();
        broad_phase_group *const bpg = ap->m_p2->m_bpb->get_bpg();
        if (phys_are_potentially_colliding_whace(bpi, bpg, &hit_time))
        {
            for (broad_phase_info *bpi_j = bpg->m_list_bpi_head; bpi_j; bpi_j = static_cast<broad_phase_info *>(bpi_j->m_list_bpb_next))
            {
                if (phys_are_potentially_colliding_whace(bpi, bpi_j, &hit_time))
                    add_collision_pair(bpi, bpi_j, hit_time, NULL);
            }
        }
    }

    ap_i = &m_list_bpg_bpg;
    for (active_pair *ap = *ap_i; ap; ap = *ap_i)
    {
        if (!are_overlapping(ap->m_p1, ap->m_p2))
        {
            *ap_i = ap->m_next;
            sap_free_active_pair(m_active_pair_allocator, ap);
            continue;
        }
        ap_i = &ap->m_next;
        broad_phase_group *const bpg1 = ap->m_p1->m_bpb->get_bpg();
        broad_phase_group *const bpg2 = ap->m_p2->m_bpb->get_bpg();
        for (broad_phase_info *bpi_j = bpg1->m_list_bpi_head; bpi_j; bpi_j = static_cast<broad_phase_info *>(bpi_j->m_list_bpb_next))
        {
            if (phys_are_potentially_colliding_whace(bpi_j, bpg2, &hit_time))
            {
                for (broad_phase_info *bpi_k = bpg2->m_list_bpi_head; bpi_k; bpi_k = static_cast<broad_phase_info *>(bpi_k->m_list_bpb_next))
                {
                    if (phys_are_potentially_colliding_whace(bpi_j, bpi_k, &hit_time))
                        add_collision_pair(bpi_j, bpi_k, hit_time, NULL);
                }
            }
        }
    }
}

// 0xa21470. Rebuilds the three endpoint arrays from the lists (every node must have run update_ae_val), sorts
// them incrementally (swap() creates the new pairs), relinks the lists, then updates the pair lists.
void axis_aligned_sweep_and_prune::process()
{
    const int axis_element_count = 2 * m_sap_node_allocator.get_count();
    if (axis_element_count > 0)
    {
        phys_transient_allocator transient_buffer;
        const int list_size = axis_element_count * (int)sizeof(axis_element *);
        // TREYBO: PHYS_ALIGNOF instead of retail's 4.
        axis_element **const xlist = (axis_element **)transient_buffer.allocate(list_size, (int)PHYS_ALIGNOF(axis_element *), 0, "phys_transient_allocator out of memory.");
        axis_element **const ylist = (axis_element **)transient_buffer.allocate(list_size, (int)PHYS_ALIGNOF(axis_element *), 0, "phys_transient_allocator out of memory.");
        axis_element **const zlist = (axis_element **)transient_buffer.allocate(list_size, (int)PHYS_ALIGNOF(axis_element *), 0, "phys_transient_allocator out of memory.");

        axis_element *xcur = m_x_list;
        axis_element *ycur = m_y_list;
        axis_element *zcur = m_z_list;
        for (int i = 0; i < axis_element_count; ++i)
        {
            PHYS_ASSERT(xcur->m_node->m_updated == 1);
            PHYS_ASSERT(ycur->m_node->m_updated == 1);
            PHYS_ASSERT(zcur->m_node->m_updated == 1);
            xlist[i] = xcur;
            xcur->m_ae_list_index = i;
            xcur = xcur->m_next;
            ylist[i] = ycur;
            ycur->m_ae_list_index = i;
            ycur = ycur->m_next;
            zlist[i] = zcur;
            zcur->m_ae_list_index = i;
            zcur = zcur->m_next;
        }
        PHYS_ASSERT(xcur == NULL && ycur == NULL && zcur == NULL);

        merge_sort(xlist, axis_element_count);
        merge_sort(ylist, axis_element_count);
        merge_sort(zlist, axis_element_count);

        m_x_list = xlist[0];
        m_y_list = ylist[0];
        m_z_list = zlist[0];
        // TREYBO: retail links element i to element i + 1 for every i (reading one past the arrays) and then clears
        // the last links; same result.
        for (int i = 0; i < axis_element_count; ++i)
        {
            xlist[i]->m_node->m_updated = 0;
            ylist[i]->m_node->m_updated = 0;
            zlist[i]->m_node->m_updated = 0;
            const bool last = i == axis_element_count - 1;
            xlist[i]->m_next = last ? NULL : xlist[i + 1];
            ylist[i]->m_next = last ? NULL : ylist[i + 1];
            zlist[i]->m_next = last ? NULL : zlist[i + 1];
        }
        transient_buffer.reset();
    }
    process_active_pair_list();
}

// ---------------------------------------------------------------------------------------------
// broad_phase_memory
// ---------------------------------------------------------------------------------------------

// 0xa218a0. Permanent PMM block; the surface-type table is n * (n + 1) / 2 zeroed entries.
broad_phase_memory *broad_phase_memory::allocate_buffer(const broad_phase_memory_info &bpmi)
{
    // TREYBO: TL_ALIGNOF instead of retail's 8.
    void *const mem = PMM_PERM_ALLOCATE((uint)sizeof(broad_phase_memory), (uint)TL_ALIGNOF(broad_phase_memory));
    broad_phase_memory *const bpm = mem ? new (mem) broad_phase_memory : NULL;
    bpm->g_phys_gjk_cache_system.init(bpmi.m_max_num_gjk_ci + bpmi.m_max_num_sap_active_pairs);
    const int max_num_surface_types = bpmi.m_max_num_surface_types;
    bpm->m_max_num_surface_types = max_num_surface_types;
    bpm->m_max_num_surface_type_infos = max_num_surface_types * (max_num_surface_types + 1) / 2;
    if (bpm->m_max_num_surface_type_infos <= 0)
    {
        bpm->g_surface_type_info_database = NULL;
        return bpm;
    }
    bpm->g_surface_type_info_database = (phys_surface_type_info *)PMM_PERM_ALLOCATE((uint)(sizeof(phys_surface_type_info) * bpm->m_max_num_surface_type_infos),
                                                                                  (uint)TL_ALIGNOF(phys_surface_type_info));
    for (int i = 0; i < bpm->m_max_num_surface_type_infos; ++i)
    {
        phys_surface_type_info &psti = bpm->g_surface_type_info_database[i];
        psti.m_friction_coef = 0.0f;
        psti.m_bounce_coef = 0.0f;
        psti.m_solver_priority = 0;
        psti.m_no_overflow_error = false;
    }
    return bpm;
}
