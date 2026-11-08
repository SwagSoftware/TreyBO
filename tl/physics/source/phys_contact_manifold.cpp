// Retail: codsrc\tl\physics\source\phys_contact_manifold.cpp (0xa21950-0xa237d0), plus
// phys_contact_manifold_process::process (0xa2e080, retail TU source/phys_collision.cpp). Optimized x87 TU.

#include "tl/physics/include/collision/phys_contact_manifold.h"
#include "tl/physics/include/collision/phys_collision.h"
#include "tl/physics/include/collision/phys_broad_phase.h"
#include "tl/physics/include/collision/phys_gjk.h"
#include "tl/physics/include/rbc_defs/rbc_def_contact.h"
#include "tl/physics/include/phys_avl_tree.h"
#include "tl/base/include/tl_system.h"

const char *g_contact_manifold_error_msg = "contact_manifold memory overflow: INCREASE phys_contact_manifold_process::ALLOCATOR_MEMORY_SIZE";

phys_set_cpi_params_func g_phys_set_cpi_params = NULL;

void phys_set_cpi_params_callback(phys_set_cpi_params_func func) // TreyBO-new
{
    g_phys_set_cpi_params = func;
}

void displace_contact_p(contact_manifold_mesh_point **mp, const phys_vec2 &d, const phys_mat44 &contact_mat) // 0xa219e0
{
    (*mp)->m_contact_p += d;
    (*mp)->m_p += phys_v2_to_v3_multiply(contact_mat, d);
}

const float phys_contact_manifold::compute_convex_poly_area() // 0xa21a50
{
    float area = 0.0f;
    const contact_manifold_mesh_point *const p0 = m_list_contact_point[0];
    for (contact_manifold_mesh_point **i = m_list_contact_point + 1, **const last_i = m_list_contact_point + m_list_contact_point_count - 1; i != last_i; ++i)
        area = phys_v2_cross(i[0]->m_contact_p - p0->m_contact_p, i[1]->m_contact_p - i[0]->m_contact_p) + area;
    area *= 0.5f;
    return area < 0.0f ? -area : area; // retail compares; -0.0f stays -0.0f
}

const float phys_contact_manifold::compute_convex_poly_perimeter() // 0xa21af0
{
    float perimeter = 0.0f;
    contact_manifold_mesh_point **const last_i = m_list_contact_point + m_list_contact_point_count;
    for (contact_manifold_mesh_point **i = m_list_contact_point, **prev_i = last_i - 1; i != last_i; prev_i = i, ++i)
        perimeter = Abs((*i)->m_contact_p - (*prev_i)->m_contact_p) + perimeter;
    return perimeter;
}

// Quicksort with the first element as pivot. Retail recursed on the left part and looped on the right one.
void feature_qsort(contact_manifold_mesh_point **list, int list_count) // 0xa21b70
{
    if (list_count > 2)
    {
        contact_manifold_mesh_point *const pivot = list[0];
        contact_manifold_mesh_point **const last = list + list_count - 1;
        contact_manifold_mesh_point **mid = list;
        for (contact_manifold_mesh_point **i = list + 1; i <= last; ++i)
        {
            if (pivot->m_contact_p.x < (*i)->m_contact_p.x)
                tl_swap(*++mid, *i);
        }
        list[0] = *mid;
        *mid = pivot;
        feature_qsort(list, (int)(mid - list));
        feature_qsort(mid + 1, (int)(last - mid));
    }
    else if (list_count == 2)
    {
        if (list[0]->m_contact_p.x < list[1]->m_contact_p.x)
            tl_swap(list[0], list[1]);
    }
}

void convex_poly_qsort(contact_manifold_mesh_point **list, int list_count) // 0xa21cb0
{
    if (list_count > 2)
    {
        contact_manifold_mesh_point *const pivot = list[0];
        contact_manifold_mesh_point **const last = list + list_count - 1;
        contact_manifold_mesh_point **mid = list;
        for (contact_manifold_mesh_point **i = list + 1; i <= last; ++i)
        {
            if (phys_v2_le((*i)->m_contact_p, pivot->m_contact_p))
                tl_swap(*++mid, *i);
        }
        list[0] = *mid;
        *mid = pivot;
        convex_poly_qsort(list, (int)(mid - list));
        convex_poly_qsort(mid + 1, (int)(last - mid));
    }
    else if (list_count == 2)
    {
        if (phys_v2_le(list[1]->m_contact_p, list[0]->m_contact_p))
            tl_swap(list[0], list[1]);
    }
}

// Monotone-chain hull of the sorted points into m_list_contact_point (lower chain, then upper chain).
// The chain closes on the first point, which is written twice and not counted.
void phys_contact_manifold::generate_convex_poly_internal() // 0xa21eb0
{
    convex_poly_qsort(m_list_sorted_mesh_point, m_list_mesh_point_count);

    contact_manifold_mesh_point **cp_mp = m_list_contact_point;
    contact_manifold_mesh_point **i_smp = m_list_sorted_mesh_point;
    {
        contact_manifold_mesh_point **const min_cp_mp = cp_mp + 1;
        for (contact_manifold_mesh_point **const last_i_smp = m_list_sorted_mesh_point + m_list_mesh_point_count; i_smp < last_i_smp; ++i_smp)
        {
            contact_manifold_mesh_point *const mp = *i_smp;
            while (cp_mp > min_cp_mp)
            {
                const phys_vec2 e1 = cp_mp[-1]->m_contact_p - cp_mp[-2]->m_contact_p;
                const phys_vec2 e2 = mp->m_contact_p - cp_mp[-1]->m_contact_p;
                if (rht(e1, e2, 0.115600005f, 0.0003045865f)) // phys_sqr(0.01 * 34), sin^2(1 deg)
                    break;
                --cp_mp;
            }
            PHYS_ASSERT_STR_MSG(m_allocator->fast_is_within_buffer_limits(cp_mp, sizeof(contact_manifold_mesh_point *)),
                                "m_allocator->fast_is_within_buffer_limits(cp_mp,sizeof(contact_manifold_mesh_point*))", g_contact_manifold_error_msg);
            *cp_mp++ = mp;
        }
    }
    {
        // i_smp is one past the last sorted point here.
        contact_manifold_mesh_point **const min_cp_mp = cp_mp;
        for (i_smp -= 2; i_smp >= m_list_sorted_mesh_point; --i_smp)
        {
            contact_manifold_mesh_point *const mp = *i_smp;
            while (cp_mp > min_cp_mp)
            {
                const phys_vec2 e1 = cp_mp[-1]->m_contact_p - cp_mp[-2]->m_contact_p;
                const phys_vec2 e2 = mp->m_contact_p - cp_mp[-1]->m_contact_p;
                if (rht(e1, e2, 0.115600005f, 0.0003045865f))
                    break;
                --cp_mp;
            }
            PHYS_ASSERT_STR_MSG(m_allocator->fast_is_within_buffer_limits(cp_mp, sizeof(contact_manifold_mesh_point *)),
                                "m_allocator->fast_is_within_buffer_limits(cp_mp,sizeof(contact_manifold_mesh_point*))", g_contact_manifold_error_msg);
            *cp_mp++ = mp;
        }
    }
    m_list_contact_point_count = (int)(cp_mp - m_list_contact_point) - 1;
    PHYS_ASSERT(m_list_contact_point_count <= m_list_mesh_point_count);
}

// Clips segment p0-p1 against the convex polygon of cman; the clipped segment overwrites cman's first two
// contact points and becomes the intersection list.
void phys_contact_manifold_process::intersect_poly_segment(phys_contact_manifold *cman, const phys_vec2 &p0, const phys_vec2 &p1) // 0xa220b0
{
    const phys_vec2 dir = p1 - p0;
    float t_enter = 0.0f;
    float t_exit = 1.0f;
    contact_manifold_mesh_point **const last_i = cman->m_list_contact_point + cman->m_list_contact_point_count - 1;
    const contact_manifold_mesh_point *prev_mp = *last_i;
    for (contact_manifold_mesh_point **i = cman->m_list_contact_point; i <= last_i; ++i)
    {
        const contact_manifold_mesh_point *const mp = *i;
        const phys_vec2 normal = phys_v2_rotr(prev_mp->m_contact_p - mp->m_contact_p);
        const float numer = phys_v2_dot(prev_mp->m_contact_p - p0, normal);
        const float denom = phys_v2_dot(dir, normal);
        // TREYBO FIX(0xa2215a / 0xa22163): `test ah,41h; jnz` / `test ah,5; jp` send a NaN denom to the division.
        if (!(denom > -PHYS_EPS_1E4 && denom < PHYS_EPS_1E4))
        {
            const float t_ = numer / denom;
            if (denom < 0.0f)
            {
                if (t_enter < t_)
                    t_enter = t_;
            }
            else if (t_exit > t_)
            {
                t_exit = t_;
            }
            if (t_exit < t_enter)
                return;
        }
        else if (numer < 0.0f)
        {
            return;
        }
        prev_mp = mp;
    }
    PHYS_ASSERT(t_enter <= t_exit);
    PHYS_ASSERT(cman->m_list_contact_point_count >= 2);
    m_list_isect_point = cman->m_list_contact_point;
    m_contact_point_count = 2;
    m_list_isect_point[0]->m_contact_p = p0 + t_enter * dir;
    m_list_isect_point[1]->m_contact_p = p0 + t_exit * dir;
}

// Walks the bridge down to the lowest edge pair and stores the edge intersection. False when a walk wraps around.
bool phys_contact_manifold_process::find_bottom(bridge *b, isect_info *left_cman, isect_info *right_cman) // 0xa222f0
{
    contact_manifold_mesh_point **const start_left_i = b->m_left_i;
    contact_manifold_mesh_point **const start_right_i = b->m_right_i;
    contact_manifold_mesh_point **l_i;
    contact_manifold_mesh_point **r_i;
    const phys_vec2 *left_p;  // point after l_i
    const phys_vec2 *right_p; // point before b->m_right_i
    phys_vec2 left_edge;
    phys_vec2 right_edge;
    do
    {
        l_i = b->m_left_i;
        r_i = b->m_right_i;
        left_p = &(*left_cman->next_pv(l_i))->m_contact_p;
        left_edge = *left_p - (*l_i)->m_contact_p;
        for (;;)
        {
            contact_manifold_mesh_point **const prev_right_i = right_cman->prev_pv(b->m_right_i);
            const float det = phys_v2_cross(left_edge, (*prev_right_i)->m_contact_p - *left_p);
            if (!(det < 0.0f)) // TREYBO FIX(0xa2238e): `test ah,5; jp` stops the walk on NaN
                break;
            b->m_right_i = prev_right_i;
            if (prev_right_i == start_right_i)
                return false;
        }

        right_p = &(*right_cman->prev_pv(b->m_right_i))->m_contact_p;
        right_edge = *right_p - (*b->m_right_i)->m_contact_p;
        for (;;)
        {
            contact_manifold_mesh_point **const next_left_i = left_cman->next_pv(b->m_left_i);
            const float det = phys_v2_cross(right_edge, (*next_left_i)->m_contact_p - *right_p);
            if (!(det > 0.0f)) // TREYBO FIX(0xa22413): `test ah,41h; jnz` stops the walk on NaN
                break;
            b->m_left_i = next_left_i;
            if (next_left_i == start_left_i)
                return false;
        }
    } while (l_i != b->m_left_i || r_i != b->m_right_i);

    const float det = phys_v2_cross(left_edge, right_edge);
    if (!(det > PHYS_EPS_1E4)) // TREYBO FIX(0xa22463): `test ah,41h; jnz` takes the corner point on NaN
    {
        b->m_intersection_p = (*b->m_right_i)->m_contact_p;
        return true;
    }
    const float t_ = phys_v2_cross(*right_p - *left_p, right_edge) / det;
    b->m_intersection_p = *left_p + t_ * left_edge;
    return true;
}

// Builds the sorted mesh point list, picks the feature normal and keeps the points lying on that feature
// (re-based to absolute coordinates). Two-point features project the hit normal off the edge instead.
void phys_contact_manifold::comp_feature_normal() // 0xa22500
{
    PHYS_ASSERT(m_list_mesh_point != NULL);
    PHYS_ASSERT(m_list_sorted_mesh_point == NULL);
    m_list_sorted_mesh_point = (contact_manifold_mesh_point **)m_allocator->fast_align_start(PHYS_ALIGNOF(contact_manifold_mesh_point *), g_contact_manifold_error_msg);
    m_allocator->fast_allocate(sizeof(contact_manifold_mesh_point *) * m_list_mesh_point_count, g_contact_manifold_error_msg);

    contact_manifold_mesh_point **const last_mp_i = m_list_sorted_mesh_point + m_list_mesh_point_count;
    {
        contact_manifold_mesh_point *mp = m_list_mesh_point;
        for (contact_manifold_mesh_point **mp_i = m_list_sorted_mesh_point; mp_i < last_mp_i; ++mp_i, ++mp)
            *mp_i = mp;
    }
    m_list_mesh_point = NULL;

    PHYS_ASSERT(m_list_mesh_point_count >= 2);
    if (m_list_mesh_point_count == 2)
    {
        contact_manifold_mesh_point **const smp = m_list_sorted_mesh_point;
        const phys_vec3 edge = smp[1]->m_p - smp[0]->m_p;
        const float nedge_sq = AbsSquared(edge);
        if (!(nedge_sq > 1.1559999e-05f)) // phys_sqr(0.0001 * 34). TREYBO FIX(0xa2267c): NaN -> hitn
        {
            m_feature_normal = m_feature_hitn;
        }
        else
        {
            m_feature_normal = m_feature_hitn - (phys_dot(m_feature_hitn, edge) / nedge_sq) * edge;
            if (AbsSquared(m_feature_normal) <= 1.1559999e-05f)
                m_feature_normal = m_feature_hitn;
        }
        smp[0]->m_p += m_feature_hitp;
        smp[1]->m_p += m_feature_hitp;
    }
    else
    {
        feature_qsort(m_list_sorted_mesh_point, m_list_mesh_point_count);
        m_feature_normal = m_feature_hitn;

        // Try every plane through two candidate points (and the hit point). The list is sorted by sin^2 to the
        // hit normal, descending, so the loops stop at the first point below the current best.
        contact_manifold_mesh_point **const first_mp_i = m_list_sorted_mesh_point + m_close_mesh_point_count;
        float dot_p_fn = 0.116977781f; // sin^2(20 deg)
        for (contact_manifold_mesh_point **mp_i_it = first_mp_i; mp_i_it < last_mp_i - 1; ++mp_i_it)
        {
            const contact_manifold_mesh_point *const mp_i = *mp_i_it;
            if (!(dot_p_fn <= mp_i->m_contact_p.x)) // TREYBO FIX(0xa227cc): NaN breaks
                break;
            for (contact_manifold_mesh_point **mp_j_it = mp_i_it + 1; mp_j_it < last_mp_i; ++mp_j_it)
            {
                const contact_manifold_mesh_point *const mp_j = *mp_j_it;
                if (!(dot_p_fn <= mp_j->m_contact_p.x)) // TREYBO FIX(0xa227f8): NaN breaks
                    break;
                phys_vec3 normal = phys_cross(mp_i->m_p, mp_j->m_p);
                const float nnormal_sq = AbsSquared(normal);
                if (nnormal_sq > 1.1559999e-05f)
                {
                    const float dotp = phys_dot(m_feature_hitn, normal);
                    const float best_cos_sq = phys_sqr(dotp) / nnormal_sq;
                    if (dot_p_fn < best_cos_sq)
                    {
                        if (dotp < 0.0f)
                            normal = -normal;
                        // Reject the plane if any other candidate lies clearly behind it.
                        bool is_valid = true;
                        for (contact_manifold_mesh_point **mp_k_it = first_mp_i; mp_k_it < last_mp_i && is_valid; ++mp_k_it)
                        {
                            const contact_manifold_mesh_point *const mp_k = *mp_k_it;
                            if (!(best_cos_sq <= mp_k->m_contact_p.x)) // TREYBO FIX(0xa2291a): NaN breaks
                                break;
                            if (mp_k != mp_i && mp_k != mp_j)
                            {
                                const float dist = phys_dot(mp_k->m_p, normal);
                                if (dist < 0.0f)
                                {
                                    const float dist_sq = phys_sqr(dist) / nnormal_sq;
                                    if (dist_sq > 0.00115600007f && mp_k->m_contact_p.y * 6.853876e-06f < dist_sq) // phys_sqr(0.001 * 34), sin^2(0.15 deg)
                                        is_valid = false;
                                }
                            }
                        }
                        if (is_valid)
                        {
                            dot_p_fn = best_cos_sq;
                            m_feature_normal = normal;
                        }
                    }
                }
            }
        }

        const float nfeature_normal_sq = AbsSquared(m_feature_normal);
        PHYS_ASSERT(nfeature_normal_sq > 0.0f);
        contact_manifold_mesh_point **cur_mp_i = m_list_sorted_mesh_point;
        for (contact_manifold_mesh_point **mp_i = m_list_sorted_mesh_point; mp_i < last_mp_i; ++mp_i)
        {
            contact_manifold_mesh_point *const mp = *mp_i;
            const float dotp = phys_dot(mp->m_p, m_feature_normal);
            const float orth_dist_sq = phys_sqr(dotp) / nfeature_normal_sq;
            if (phys_sqr(m_feature_distance_eps) >= orth_dist_sq || mp->m_contact_p.y * m_sin_feautre_angular_eps_sq >= orth_dist_sq)
            {
                *cur_mp_i++ = mp;
                // Project onto the feature plane through the hit point.
                mp->m_p += m_feature_hitp - (dotp / nfeature_normal_sq) * m_feature_normal;
            }
        }
        m_list_mesh_point_count = (int)(cur_mp_i - m_list_sorted_mesh_point);
    }
    PHYS_ASSERT(AbsSquared(m_feature_normal) > 0.0f);
}

// Projects the feature points into the contact plane and keeps their convex hull.
void phys_contact_manifold::generate_convex_poly(const phys_mat44 &contact_mat) // 0xa22da0
{
    PHYS_ASSERT(m_list_sorted_mesh_point != NULL);
    PHYS_ASSERT(m_list_contact_point == NULL);
    PHYS_ASSERT(m_list_mesh_point_count > 1);
    m_list_contact_point = (contact_manifold_mesh_point **)m_allocator->fast_align_start(PHYS_ALIGNOF(contact_manifold_mesh_point *), g_contact_manifold_error_msg);
    PHYS_ASSERT_STR_MSG(m_allocator->fast_is_within_buffer_limits(m_list_contact_point, sizeof(contact_manifold_mesh_point *) * m_list_mesh_point_count),
                        "m_allocator->fast_is_within_buffer_limits(m_list_contact_point,sizeof(contact_manifold_mesh_point*)*m_list_mesh_point_count)",
                        g_contact_manifold_error_msg);

    for (contact_manifold_mesh_point **mp_i = m_list_sorted_mesh_point, **const last_mp_i = mp_i + m_list_mesh_point_count; mp_i != last_mp_i; ++mp_i)
        (*mp_i)->m_contact_p = phys_v3_to_v2_inv_multiply(contact_mat, (*mp_i)->m_p);

    generate_convex_poly_internal();

    if (m_list_contact_point_count == 2)
    {
        if (AbsSquared(m_list_contact_point[0]->m_contact_p - m_list_contact_point[1]->m_contact_p) < 0.115600005f) // phys_sqr(0.01 * 34)
            m_list_contact_point_count = 1;
    }
    PHYS_ASSERT(m_list_contact_point_count > 0);
    void *const temp = m_allocator->fast_allocate(sizeof(contact_manifold_mesh_point *) * m_list_contact_point_count, g_contact_manifold_error_msg);
    PHYS_ASSERT(m_list_contact_point == temp);
}

// Intersects the two contact polygons with rotating calipers; the bridges between them give the
// intersection polygon, whose vertices become m_list_isect_point.
void phys_contact_manifold_process::intersect_poly_poly() // 0xa22fd0
{
    PHYS_ASSERT(cman1.get_poly_vert_count() >= 2 && cman2.get_poly_vert_count() >= 2);
    PHYS_ASSERT(cman1.get_poly_vert_count() > 2 || cman2.get_poly_vert_count() > 2);
    m_list_isect_point = NULL;
    m_contact_point_count = 0;

    if (cman1.get_poly_vert_count() == 2)
    {
        intersect_poly_segment(&cman2, cman1.get_poly_vert(0), cman1.get_poly_vert(1));
        return;
    }
    if (cman2.get_poly_vert_count() == 2)
    {
        intersect_poly_segment(&cman1, cman2.get_poly_vert(0), cman2.get_poly_vert(1));
        return;
    }

    isect_info gb_cman1;
    isect_info gb_cman2;
    gb_cman1.init(&cman1);
    gb_cman2.init(&cman2);
    isect_info *left_gb = &gb_cman1;
    isect_info *right_gb = &gb_cman2;

    bridge *const list_bridge = (bridge *)m_allocator.fast_align_start(PHYS_ALIGNOF(bridge), g_contact_manifold_error_msg);
    bridge *b_cur = list_bridge;
    const int max_ctr = gb_cman1.m_cman->m_list_contact_point_count + gb_cman2.m_cman->m_list_contact_point_count;
    for (int ctr = 0; ctr <= max_ctr; ++ctr)
    {
        b_cur->m_left_i = left_gb->m_i;
        b_cur->m_right_i = right_gb->m_i;

        // Advance the caliper on the polygon whose edge turns first.
        phys_vec2 caliper_dir;
        if (!(phys_v2_cross(left_gb->m_edge, right_gb->m_edge) >= 0.0f)) // TREYBO FIX(0xa231f8): `test ah,1; jnz`, NaN -> right
        {
            caliper_dir = right_gb->m_edge;
            right_gb->update();
        }
        else
        {
            caliper_dir = left_gb->m_edge;
            left_gb->update();
        }

        const float cr = phys_v2_cross(caliper_dir, (*right_gb->m_i)->m_contact_p - (*left_gb->m_i)->m_contact_p);
        // TREYBO FIX(0xa23268 / 0xa23275): a NaN cr takes the collinear (displace) branch, as `test ah,41h; jz` then
        // `test ah,5; jp` route it; kisakblack's "<=" skipped it.
        if (!(cr > 0.0f))
        {
            if (!(cr < 0.0f))
            {
                // Collinear vertex: nudge it off the caliper line (0.001 * 34) and keep going.
                phys_vec2 vdisplace = phys_v2_rotr(caliper_dir);
                const float nvdisplace = Abs(vdisplace);
                PHYS_ASSERT(nvdisplace > 0.0001f);
                vdisplace *= 0.034f / nvdisplace;
                displace_contact_p(right_gb->m_i, vdisplace, contact_mat);
            }
            else
            {
                // The polygons cross here: record a bridge and swap sides.
                if (ctr)
                {
                    PHYS_ASSERT_STR_MSG(m_allocator.fast_is_within_buffer_limits(b_cur, sizeof(bridge)),
                                        "m_allocator.fast_is_within_buffer_limits(b_cur,sizeof(bridge))", g_contact_manifold_error_msg);
                    if (!find_bottom(b_cur, left_gb, right_gb))
                        return;
                    ++b_cur;
                }
                tl_swap(left_gb, right_gb);
            }
        }
    }

    const int bridge_count = (int)(b_cur - list_bridge);
    if (bridge_count % 2)
    {
        tlWarning("contact manifold intersect poly poly failed.\n");
        // Fall back to the polygon with the smaller perimeter.
        phys_contact_manifold *cman = &cman1;
        if (!(cman1.compute_convex_poly_perimeter() <= cman2.compute_convex_poly_perimeter())) // TREYBO FIX(0xa233c6): NaN -> cman2
            cman = &cman2;
        m_list_isect_point = cman->m_list_contact_point;
        m_contact_point_count = cman->m_list_contact_point_count;
        return;
    }

    void *const temp_ptr = m_allocator.fast_allocate(sizeof(bridge) * bridge_count, g_contact_manifold_error_msg);
    PHYS_ASSERT(temp_ptr == list_bridge);

    if (bridge_count == 0)
    {
        // No crossing: one polygon contains the other.
        m_list_isect_point = right_gb->m_cman->m_list_contact_point;
        m_contact_point_count = right_gb->m_cman->m_list_contact_point_count;
        return;
    }

    // Each bridge contributes its intersection point (stored in the left vertex) followed by the left polygon's
    // vertices up to the next bridge's right vertex; the sides alternate per bridge.
    m_list_isect_point = (contact_manifold_mesh_point **)m_allocator.fast_align_start(PHYS_ALIGNOF(contact_manifold_mesh_point *), g_contact_manifold_error_msg);
    contact_manifold_mesh_point **ip_i = m_list_isect_point;
    for (int bridge_i = 0; bridge_i < bridge_count; ++bridge_i)
    {
        const bridge *const b = &list_bridge[bridge_i];
        PHYS_ASSERT_STR_MSG(m_allocator.fast_is_within_buffer_limits(ip_i, sizeof(contact_manifold_mesh_point *)),
                            "m_allocator.fast_is_within_buffer_limits(ip_i,sizeof(contact_manifold_mesh_point*))", g_contact_manifold_error_msg);
        *ip_i = *b->m_left_i;
        for (contact_manifold_mesh_point **j = m_list_isect_point; j != ip_i; ++j)
        {
            if (*j == *b->m_left_i)
                tlWarning("contact manifold failure.");
        }
        (*ip_i)->m_contact_p = b->m_intersection_p;
        ++ip_i;

        contact_manifold_mesh_point **const term_left_i = list_bridge[(bridge_i + 1) % bridge_count].m_right_i;
        for (contact_manifold_mesh_point **l_i = left_gb->next_pv(b->m_left_i); l_i != term_left_i; l_i = left_gb->next_pv(l_i))
        {
            PHYS_ASSERT_STR_MSG(m_allocator.fast_is_within_buffer_limits(ip_i, sizeof(contact_manifold_mesh_point *)),
                                "m_allocator.fast_is_within_buffer_limits(ip_i,sizeof(contact_manifold_mesh_point*))", g_contact_manifold_error_msg);
            for (contact_manifold_mesh_point **j = m_list_isect_point; j != ip_i; ++j)
            {
                if (*j == *l_i)
                    tlWarning("contact manifold failure.");
            }
            *ip_i++ = *l_i;
        }
        tl_swap(left_gb, right_gb);
    }
    m_contact_point_count = (int)(ip_i - m_list_isect_point);
    void *const temp_ptr2 = m_allocator.fast_allocate(sizeof(contact_manifold_mesh_point *) * m_contact_point_count, g_contact_manifold_error_msg);
    PHYS_ASSERT_STR(temp_ptr2 == m_list_isect_point, "temp_ptr == m_list_isect_point");
}

bool phys_contact_manifold::rht(const phys_vec2 &e1, const phys_vec2 &e2, const float min_length2, const float min_sin_sq) // 0xa23690
{
    // Strict right-hand turn with both edges long enough and a non-negligible angle.
    // TREYBO FIX(0xa236ad / 0xa236ce / 0xa236f4 / 0xa23710): every test only rejects on an ordered failure, so a
    // NaN passes all four (returns true); kisakblack's ordered tests rejected NaN.
    const float cr = phys_v2_cross(e1, e2);
    if (!(cr <= 0.0f))
    {
        const float ne1_sq = AbsSquared(e1);
        if (!(min_length2 >= ne1_sq))
        {
            const float ne2_sq = AbsSquared(e2);
            if (!(ne2_sq <= min_length2))
            {
                const float sin_sq = cr * cr / (ne1_sq * ne2_sq);
                if (!(min_sin_sq >= sin_sq))
                    return true;
            }
        }
    }
    return false;
}

void phys_contact_manifold_process::isect_info::init(phys_contact_manifold *cman) // 0xa23720
{
    m_cman = cman;
    m_i = cman->m_list_contact_point;
    m_next_i = m_i + 1;
    m_last_i = &m_i[cman->m_list_contact_point_count - 1];
    m_edge = (*m_next_i)->m_contact_p - (*m_i)->m_contact_p;
}

void phys_contact_manifold_process::isect_info::update() // 0xa23780
{
    m_i = m_next_i;
    m_next_i = next_pv(m_i);
    m_edge = (*m_next_i)->m_contact_p - (*m_i)->m_contact_p;
}

// ---------------------------------------------------------------------------------------------
// process (retail: source/phys_collision.cpp, 0xa2e080)
// ---------------------------------------------------------------------------------------------

// Both manifolds must hold a polygon, and not two segments.
static inline bool phys_cman_counts_usable(const int count1, const int count2) // TREYBO: helper for a test retail repeats inline
{
    return count1 >= 2 && count2 >= 2 && !(count1 == 2 && count2 == 2);
}

// Turns the GJK closest points of one collision pair into a contact_point_info: features of both geoms are
// gathered around the (pushed-apart) closest points, intersected in the contact plane and reduced to at most
// five points. Falls back to a single point pair when no usable manifold results.
void phys_contact_manifold_process::process(phys_collision_pair *pcp, phys_gjk_info *gjk_info) // 0xa2e080
{
    cman1.m_list_mesh_point = NULL;
    cman1.m_list_sorted_mesh_point = NULL;
    cman1.m_list_contact_point = NULL;
    cman2.m_list_mesh_point = NULL;
    cman2.m_list_sorted_mesh_point = NULL;
    cman2.m_list_contact_point = NULL;
    m_list_isect_point = NULL;
    m_allocator.reset();

    const phys_mat44 *const cg1_to_world_xform = pcp->m_bpi1->m_cg_to_world_xform;
    const phys_mat44 *const cg1_to_rb1_xform = pcp->m_bpi1->m_cg_to_rb_xform;
    m_cpi = NULL;
    phys_full_inv_multiply_mat(cg1_to_rb2_xform, *pcp->m_bpi2->m_rb_to_world_xform, *cg1_to_world_xform);

    // Closest points in cg1's frame; move them apart along the normal by the penetration threshold (0.01 * 34).
    phys_gjk_collision_info &cinfo = gjk_info->cg1_cinfo_loc;
    const phys_vec3 &n = cinfo.m_n;
    const phys_vec3 p1 = cinfo.m_p1;
    const phys_vec3 p2 = cinfo.m_p2;
    {
        const phys_vec3 displace = n * 0.34f;
        cinfo.m_p1 += displace;
        cinfo.m_p2 -= displace;
    }
    const float dist_p1_p2_n = phys_dot(n, p1 - p2);
    const float penetration_t = tl_clamp(dist_p1_p2_n / -10.200001f, 0.0f, 1.0f); // 0.3 * 34

    const float feature_distance_eps = phys_contact_manifold::get_STD_GET_FEATURE_DISTANCE_EPS(penetration_t);
    const float sin_feautre_angular_eps_sq = phys_contact_manifold::get_STD_GET_FEATURE_SIN_ANGULAR_EPS_SQ(penetration_t);
    cman1.set_get_feature_params(cinfo.m_p1, n, feature_distance_eps, sin_feautre_angular_eps_sq);
    pcp->m_bpi1->m_gjk_geom->get_feature(&cman1);
    cman2.set_get_feature_params(phys_full_inv_multiply(gjk_info->cg2_to_cg1_xform, cinfo.m_p2), phys_inv_multiply(gjk_info->cg2_to_cg1_xform, -n),
                                 feature_distance_eps, sin_feautre_angular_eps_sq);
    pcp->m_bpi2->m_gjk_geom->get_feature(&cman2);

    const phys_vec3 cg1_relative_translation_loc = gjk_info->m_continuous_collision_lambda * gjk_info->m_cg1_relative_translation_loc;

    bool have_manifold = false;
    if (phys_cman_counts_usable(cman1.m_list_mesh_point_count, cman2.m_list_mesh_point_count))
    {
        const float comp_distance_eps = phys_contact_manifold::get_STD_COMP_FEATURE_NORMAL_DISTANCE_EPS(penetration_t);
        const float comp_sin_angular_eps_sq = phys_contact_manifold::get_STD_COMP_FEATURE_NORMAL_SIN_ANGULAR_EPS_SQ(penetration_t);
        cman1.m_feature_distance_eps = comp_distance_eps;
        cman1.m_sin_feautre_angular_eps_sq = comp_sin_angular_eps_sq;
        cman1.comp_feature_normal();
        cman2.m_feature_distance_eps = comp_distance_eps;
        cman2.m_sin_feautre_angular_eps_sq = comp_sin_angular_eps_sq;
        cman2.comp_feature_normal();
        if (phys_cman_counts_usable(cman1.m_list_mesh_point_count, cman2.m_list_mesh_point_count))
        {
            comp_contact_mat(n);
            cman1.generate_convex_poly(contact_mat);
            // cman2 into cg1's frame, at the same time of impact as cg1.
            cman2.xform_and_translate_mesh_points(gjk_info->cg2_to_cg1_xform, -cg1_relative_translation_loc);
            cman2.generate_convex_poly(contact_mat);
            if (phys_cman_counts_usable(cman1.m_list_contact_point_count, cman2.m_list_contact_point_count))
            {
                intersect_poly_poly();
                have_manifold = m_contact_point_count != 0;
            }
        }
    }

    if (!have_manifold)
    {
        m_cpi = contact_point_info::create_cpi(1, false, m_cpi_allocator);
        if (!m_cpi)
            return;
        m_cpi->m_list_b1_r_loc[0] = phys_full_multiply(*cg1_to_rb1_xform, p1);
        m_cpi->m_list_b2_r_loc[0] = phys_full_multiply(cg1_to_rb2_xform, p2);
        m_contact_point_count = 1;
    }
    else
    {
        PHYS_ASSERT(m_contact_point_count > 0);
        PHYS_ASSERT(m_list_isect_point);
        const float d1 = phys_dot(n, cman1.m_feature_normal);
        const float d2 = phys_dot(n, cman2.m_feature_normal);
        PHYS_ASSERT(fabsf(d1) > 0.0000001f);
        PHYS_ASSERT(fabsf(d2) > 0.0000001f);
        // Feature planes: dot(m_feature_normal, x) == n1 / n2.
        const float n1 = phys_dot(cman1.m_feature_normal, p1);
        const float n2 = phys_dot(cman2.m_feature_normal, p2);

        if (m_contact_point_count > 5)
        {
            // Keep the deepest point along the planes' relative tilt, then drop the smallest-area vertices.
            contact_manifold_mesh_point *closest_mp = NULL;
            const phys_vec3 tilt = cman2.m_feature_normal * (1.0f / d2) - cman1.m_feature_normal * (1.0f / d1);
            const phys_vec2 tilt_2d = phys_v3_to_v2_inv_multiply(contact_mat, tilt);
            contact_manifold_mesh_point **last_mp_i = m_list_isect_point + m_contact_point_count;
            {
                float closest_dist = 10000000.0f;
                contact_manifold_mesh_point **prev_prev_mp_i = last_mp_i - 2;
                contact_manifold_mesh_point **cur_mp_i = last_mp_i - 1;
                for (contact_manifold_mesh_point **next_mp_i = m_list_isect_point; next_mp_i != last_mp_i; ++next_mp_i)
                {
                    // m_p.x is reused as the area of the triangle at each vertex.
                    contact_manifold_mesh_point *const mp = *cur_mp_i;
                    const phys_vec2 &pp = (*prev_prev_mp_i)->m_contact_p;
                    mp->m_p.x = fabsf(phys_v2_cross(mp->m_contact_p - pp, (*next_mp_i)->m_contact_p - pp));
                    const float dist = phys_v2_dot(mp->m_contact_p, tilt_2d);
                    if (closest_dist > dist)
                    {
                        closest_dist = dist;
                        closest_mp = mp;
                    }
                    prev_prev_mp_i = cur_mp_i;
                    cur_mp_i = next_mp_i;
                }
            }
            PHYS_ASSERT(closest_mp);

            contact_manifold_mesh_point **const MAX_MP_I = m_list_isect_point + 5;
            while (last_mp_i > MAX_MP_I)
            {
                // Retail never tests the first entry against closest_mp.
                contact_manifold_mesh_point **smallest_area_mp_i = m_list_isect_point;
                for (contact_manifold_mesh_point **cur_mp_i = m_list_isect_point + 1; cur_mp_i != last_mp_i; ++cur_mp_i)
                {
                    if (*cur_mp_i != closest_mp && (*smallest_area_mp_i)->m_p.x > (*cur_mp_i)->m_p.x)
                        smallest_area_mp_i = cur_mp_i;
                }

                --last_mp_i; // now the last entry
                contact_manifold_mesh_point **const prev_mp_i = smallest_area_mp_i > m_list_isect_point ? smallest_area_mp_i - 1 : last_mp_i;
                contact_manifold_mesh_point **const next_mp_i = smallest_area_mp_i < last_mp_i ? smallest_area_mp_i + 1 : m_list_isect_point;
                contact_manifold_mesh_point **const prev_prev_mp_i = prev_mp_i > m_list_isect_point ? prev_mp_i - 1 : last_mp_i;
                contact_manifold_mesh_point **const next_next_mp_i = next_mp_i < last_mp_i ? next_mp_i + 1 : m_list_isect_point;
                {
                    const phys_vec2 &pp = (*prev_prev_mp_i)->m_contact_p;
                    (*prev_mp_i)->m_p.x = fabsf(phys_v2_cross((*prev_mp_i)->m_contact_p - pp, (*next_mp_i)->m_contact_p - pp));
                }
                {
                    const phys_vec2 &pp = (*prev_mp_i)->m_contact_p;
                    (*next_mp_i)->m_p.x = fabsf(phys_v2_cross((*next_mp_i)->m_contact_p - pp, (*next_next_mp_i)->m_contact_p - pp));
                }
                for (contact_manifold_mesh_point **i = smallest_area_mp_i; i < last_mp_i; ++i)
                    i[0] = i[1];
            }
            m_contact_point_count = 5;
        }

        m_cpi = contact_point_info::create_cpi(m_contact_point_count, false, m_cpi_allocator);
        if (!m_cpi)
            return;

        // Lift each 2D point back onto both feature planes (cg2's at its swept position) and store the arms.
        phys_vec3 *b1_r_loc = m_cpi->m_list_b1_r_loc;
        phys_vec3 *b2_r_loc = m_cpi->m_list_b2_r_loc;
        for (contact_manifold_mesh_point **ip_i = m_list_isect_point, **const last_ip_i = ip_i + m_contact_point_count; ip_i != last_ip_i; ++ip_i, ++b1_r_loc, ++b2_r_loc)
        {
            const phys_vec3 ip_3d_ = phys_v2_to_v3_multiply(contact_mat, (*ip_i)->m_contact_p);
            {
                const float t_ = (n1 - phys_dot(ip_3d_, cman1.m_feature_normal)) / d1;
                *b1_r_loc = phys_full_multiply(*cg1_to_rb1_xform, ip_3d_ + t_ * n);
            }
            {
                const phys_vec3 ip_3d_2 = ip_3d_ + cg1_relative_translation_loc;
                const float t_ = (n2 - phys_dot(ip_3d_2, cman2.m_feature_normal)) / d2;
                *b2_r_loc = phys_full_multiply(cg1_to_rb2_xform, ip_3d_2 + t_ * n);
            }
        }
    }

    PHYS_ASSERT(m_cpi);
    m_cpi->m_normal = phys_multiply(*cg1_to_world_xform, -n);
    PHYS_ASSERT_UNIT(m_cpi->m_normal);

    // Fraction of the step's relative motion at which the contact closes (only when approaching, 0.005 * 34).
    const float dot_n_translation = phys_dot(n, gjk_info->m_cg1_relative_translation_loc);
    if (dot_n_translation < -0.17f)
        m_cpi->m_translation_lambda = tl_clamp(-dist_p1_p2_n / dot_n_translation, 0.0f, 1.0f);
    else
        m_cpi->m_translation_lambda = 1.0f;

    if (g_phys_set_cpi_params) // TREYBO: retail called set_cpi_params(m_cpi, pcp) (game code, 0x837f50)
    {
        g_phys_set_cpi_params(m_cpi, pcp);
    }
    else
    {
        // TREYBO FIX(0x837f50): no host callback. create_cpi leaves friction / bounce / max restitution at -1,
        // which fails check_surface_properties and feeds the solver negative friction. Do what retail
        // set_cpi_params does for a body against the world, without the game's per-object multipliers
        // (PhysObjUserData friction / bounce = 1): the surface-type pair table, max restitution 3400,
        // solver priority 0, overflow errors on.
        const phys_surface_type_info &psti = surface_type_info_database_get(pcp->m_bpi1->m_surface_type, pcp->m_bpi2->m_surface_type);
        m_cpi->m_fric_coef = psti.m_friction_coef;
        m_cpi->m_bounce_coef = psti.m_bounce_coef;
        m_cpi->m_max_restitution_vel = 3400.0f;
        m_cpi->check_surface_properties();
        m_cpi->set_solver_priority(0);
        m_cpi->m_flags &= ~contact_point_info::FLAG_NO_OVERFLOW_ERROR;
    }

    m_list_cpi.add(m_cpi);
    rigid_body *const rb1 = pcp->m_bpi1->m_rb;
    rigid_body *const rb2 = pcp->m_bpi2->m_rb;
    m_cpi->m_pcp = pcp;
    m_cpi->m_rbc_contact = avl_tree_find<rigid_body_pair_key, rigid_body_constraint_contact, rigid_body_constraint_contact::avl_tree_accessor>(
        m_rbc_contact_search_tree_root, rigid_body_pair_key(rb1, rb2));

    const contact_point_info *cached_cpi = NULL;
    if (m_cpi->m_rbc_contact)
    {
        // The constraint may store the pair the other way round.
        if (m_cpi->m_rbc_contact->b1 != rb1)
        {
            tl_swap(m_cpi->m_list_b1_r_loc, m_cpi->m_list_b2_r_loc);
            m_cpi->m_normal = -m_cpi->m_normal;
        }
        cached_cpi = m_cpi->m_rbc_contact->m_list_contact_point_info_buffer_2.m_first;
    }
    m_cpi->set_closest_cached_psc(cached_cpi);
}
