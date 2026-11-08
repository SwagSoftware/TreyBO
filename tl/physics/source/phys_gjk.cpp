// Retail: codsrc\tl\physics\source\phys_gjk.cpp (0xa0bf90-0xa0f410). Optimized x87 TU; the dense routines
// (backup1, gjk, gjk_ray_cast, collide) were rebuilt from the instruction stream, keeping retail's float order
// and comparison/NaN behaviour. Retail kept intermediates in x87 extended precision; TreyBO rounds to float.

#include "tl/physics/include/collision/phys_gjk.h"
#include "tl/physics/include/collision/phys_gjk_sep_dir.h"
#include "tl/physics/include/phys_assert.h"
#include "tl/base/include/tl_system.h"

float CONV_THRESH = 0.001f;     // 0xe19878
float SEP_CONV_THRESH = 0.001f; // 0xe1987c
int MAX_CC_RESET_ITERS = 5;     // 0xe19880

// Retail .data, static-initialised then linked into g_list_phys_assert_info by 0xc56fa0..0xc57000.
phys_assert_info pai_gjk_cache_system_max_num_gjk_ci(0, 1, true);
phys_assert_info pai_gjk_cache_system_create_gjk_ci(0, 1, true);
phys_assert_info pai_gjk_comp_v(0, 128, true); // TREYBO: defined in retail but no BO1 code references it
phys_assert_info pai_gjk_initial_support_dir(0, 128, true);

static const int BIT_COUNT[16] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4}; // 0xd614e8

// ---------------------------------------------------------------------------------------------
// phys_gjk_geom
// ---------------------------------------------------------------------------------------------

void phys_gjk_geom::set_simplex(const phys_vec3 *simplex_inds, int w_set, const phys_vec3 &, cached_simplex_info *cache_info) const // 0x440250
{
    phys_vec3 *ind = cache_info->m_indices;
    for (int i = 0; i < 4; i++)
    {
        if (w_set & (1 << i))
            *ind++ = simplex_inds[i];
    }
}

const float phys_gjk_geom::get_geom_radius() const // 0x440310
{
    return 0.0f;
}

const bool phys_gjk_geom::ray_cast(const phys_vec3 &, const phys_vec3 &, float, float *, phys_vec3 *) const // 0x4402d0
{
    return false;
}

const phys_vec3 phys_gjk_geom::support_only(const phys_mat44 &xform, const phys_vec3 &v) const // 0xa0ed70
{
    phys_vec3 vert;
    phys_vec3 ind;
    support(phys_inv_multiply(xform, v), &vert, &ind);
    return phys_full_multiply(xform, vert);
}

// ---------------------------------------------------------------------------------------------
// Simplex cache bridge
// ---------------------------------------------------------------------------------------------

void get_simplex(const phys_gjk_geom *cg1, const phys_gjk_geom *cg2, phys_gjk_cache_info *gjk_ci,
                 phys_vec3 *a_verts, phys_vec3 *a_inds, phys_vec3 *b_verts, phys_vec3 *b_inds, int *vert_count) // 0xa0bf90
{
    PHYS_ASSERT(gjk_ci->is_simplex_valid());
    cg1->get_simplex(gjk_ci->m_support_a, gjk_ci->m_support_count, a_verts, a_inds);
    cg2->get_simplex(gjk_ci->m_support_b, gjk_ci->m_support_count, b_verts, b_inds);
    *vert_count = gjk_ci->m_support_count;
}

void set_simplex(const phys_gjk_geom *cg1, const phys_gjk_geom *cg2, phys_gjk_cache_info *gjk_ci,
                 const phys_vec3 &a_normal, const phys_vec3 &b_normal, const phys_vec3 *a_inds, const phys_vec3 *b_inds, int w_set) // 0xa0c000
{
    PHYS_ASSERT(w_set > 0 && w_set <= 15);
    gjk_ci->set_flag(phys_gjk_cache_info::FLAG_IS_SIMPLEX_VALID, true);
    cg1->set_simplex(a_inds, w_set, a_normal, &gjk_ci->m_support_a);
    cg2->set_simplex(b_inds, w_set, b_normal, &gjk_ci->m_support_b);
    gjk_ci->m_support_count = BIT_COUNT[w_set];
}

// ---------------------------------------------------------------------------------------------
// Johnson sub-algorithm
// ---------------------------------------------------------------------------------------------

// The new vertex w = m_w_verts[new_index] is always kept unless seed_simplex (which also tests the old
// vertices on their own). Candidates, in retail order: w alone, each edge (w, w_i), [seed: each w_i alone,
// and the edge (w_1, w_2)], each triangle (w, w_a, w_b), then the tetrahedron containment test.
// The winning set's weights are written to m_set_list[set].m_lamda, indexed by vertex slot.
int backup1(phys_gjk_info *gjk_info, int new_index, bool seed_simplex) // 0xa0c360
{
    const phys_vec3 &w = gjk_info->m_w_verts[new_index];
    const int new_bit = 1 << new_index;
    int best_w_set = new_bit;
    float new_dist_sq = AbsSquared(w);
    gjk_info->m_set_list[new_bit].m_lamda[new_index] = 1.0f;

    const int old_w_set = gjk_info->m_w_set & ~new_bit;

    float ndifs_sq[3];
    int w_inds[3];
    phys_vec3 difs[3];
    float dotps[3];
    int w_count = 0;

    // Edges (w, w_i).
    for (int i = 0, bit = 1; i < 4; i++, bit <<= 1)
    {
        if (!(old_w_set & bit))
            continue;

        w_inds[w_count] = i;
        difs[w_count] = gjk_info->m_w_verts[i] - w;
        ndifs_sq[w_count] = AbsSquared(difs[w_count]);
        dotps[w_count] = phys_dot(difs[w_count], w);

        // TREYBO FIX(0xa0c3d3): kisakblack used 1e-10f; retail's constant is phys_sqr(0.00001f), one ulp lower.
        if (ndifs_sq[w_count] > phys_sqr(PHYS_EPS_1E5))
        {
            const float t = -dotps[w_count] / ndifs_sq[w_count];
            const float one_minus_t = 1.0f - t;
            if (t > 0.0f && one_minus_t > 0.0f)
            {
                const phys_vec3 ww = w + t * difs[w_count];
                const float nww_sq = AbsSquared(ww);
                if (new_dist_sq > nww_sq)
                {
                    new_dist_sq = nww_sq;
                    best_w_set = bit | new_bit;
                    phys_gjk_info::phys_gjk_set_info &set_info = gjk_info->m_set_list[best_w_set];
                    set_info.m_lamda[i] = t;
                    set_info.m_lamda[new_index] = one_minus_t;
                }
            }
        }

        if (seed_simplex)
        {
            const float nw_sq = AbsSquared(gjk_info->m_w_verts[i]);
            if (new_dist_sq > nw_sq)
            {
                new_dist_sq = nw_sq;
                best_w_set = bit;
                gjk_info->m_set_list[bit].m_lamda[i] = 1.0f;
            }
        }
        w_count++;
    }

    // Seeding from a cached 3-vertex simplex (new_index 0): the edge between the two old vertices.
    if (seed_simplex && w_count == 2)
    {
        const phys_vec3 &w1 = gjk_info->m_w_verts[1];
        const phys_vec3 dif = gjk_info->m_w_verts[2] - w1;
        const float ndif_sq = AbsSquared(dif);
        if (ndif_sq > phys_sqr(PHYS_EPS_1E5))
        {
            const float t = -phys_dot(w1, dif) / ndif_sq;
            const float one_minus_t = 1.0f - t;
            if (t > 0.0f && one_minus_t > 0.0f)
            {
                const phys_vec3 ww = w1 + t * dif;
                const float nww_sq = AbsSquared(ww);
                if (new_dist_sq > nww_sq)
                {
                    new_dist_sq = nww_sq;
                    best_w_set = 6;
                    gjk_info->m_set_list[6].m_lamda[1] = one_minus_t;
                    gjk_info->m_set_list[6].m_lamda[2] = t;
                }
            }
        }
    }

    // Triangles (w, w_a, w_b).
    for (int a = 0; a < w_count - 1; a++)
    {
        const float nda_sq = ndifs_sq[a];
        const float mdota = -dotps[a];
        for (int b = a + 1; b < w_count; b++)
        {
            const float dot_ab = phys_dot(difs[a], difs[b]);
            const float ndb_sq = ndifs_sq[b];
            const float mdotb = -dotps[b];
            const float denom = ndb_sq * nda_sq - dot_ab * dot_ab;
            if (denom > PHYS_EPS_1E5)
            {
                const float ca = (ndb_sq * mdota - mdotb * dot_ab) / denom;
                const float cb = (mdotb * nda_sq - dot_ab * mdota) / denom;
                const float cw = 1.0f - ca - cb;
                if (ca > 0.0f && cb > 0.0f && cw > 0.0f)
                {
                    const phys_vec3 ww = (w + ca * difs[a]) + cb * difs[b];
                    const float nww_sq = AbsSquared(ww);
                    if (new_dist_sq > nww_sq)
                    {
                        new_dist_sq = nww_sq;
                        best_w_set = new_bit | (1 << w_inds[a]) | (1 << w_inds[b]);
                        phys_gjk_info::phys_gjk_set_info &set_info = gjk_info->m_set_list[best_w_set];
                        set_info.m_lamda[w_inds[a]] = ca;
                        set_info.m_lamda[w_inds[b]] = cb;
                        set_info.m_lamda[new_index] = cw;
                    }
                }
            }
        }
    }

    // Tetrahedron: Cramer's rule for -w = c1 * difs[0] + c2 * difs[1] + c0 * difs[2]; the origin is inside
    // when all weights (and 1 - their sum) are non-negative. Only while no positive lower bound is known.
    if (w_count == 3 && gjk_info->m_lower_dist_sq <= 0.0f)
    {
        const phys_vec3 cr01 = phys_cross(difs[0], difs[1]);
        const float denom = phys_dot(cr01, difs[2]);
        if (fabsf(denom) > PHYS_EPS_1E5)
        {
            const float c0 = -phys_dot(w, cr01) / denom;
            if (c0 >= 0.0f)
            {
                const phys_vec3 crw2 = phys_cross(w, difs[2]);
                const float c1 = phys_dot(difs[1], crw2) / denom;
                if (c1 >= 0.0f)
                {
                    const float c2 = -phys_dot(difs[0], crw2) / denom;
                    if (c2 >= 0.0f && 1.0f - c1 - c2 - c0 >= 0.0f)
                        return 15;
                }
            }
        }
    }
    return best_w_set;
}

const int phys_gjk_info::gjk_subalgorithm(int, int new_index) // 0xa0c080
{
    return backup1(this, new_index, false);
}

// ---------------------------------------------------------------------------------------------
// Simplex helpers
// ---------------------------------------------------------------------------------------------

const int phys_gjk_info::seed_simplex(int cached_vert_count) // 0xa0c0a0
{
    PHYS_ASSERT(cached_vert_count > 0);
    PHYS_ASSERT(cached_vert_count < 4);
    m_w_set = 0;
    for (int i = 0, bit = 1; i < cached_vert_count; i++, bit <<= 1)
    {
        m_w_verts[i] = m_a_verts[i] - m_b_verts[i] - m_gjk_origin;
        m_w_set |= bit;
    }
    return backup1(this, 0, true);
}

// Packs the vertices of w_set to the front of the a/b arrays; returns their count.
const int phys_gjk_info::compress_verts(int w_set) // 0xa0e4a0
{
    int count = 0;
    for (int i = 0; i < 4; i++)
    {
        if (w_set & (1 << i))
        {
            m_a_verts[count] = m_a_verts[i];
            m_b_verts[count] = m_b_verts[i];
            m_a_inds[count] = m_a_inds[i];
            m_b_inds[count] = m_b_inds[i];
            count++;
        }
    }
    return count;
}

// Closest point of the simplex w_set to the origin (w_set has 1-3 vertices).
void phys_gjk_info::comp_v(int w_set, phys_vec3 *v) // 0xa0ee10
{
    PHYS_ASSERT(w_set > 0 && w_set < 15);

    phys_vec3 list_w_vert[3];
    int list_w_vert_count = 0;
    for (int i = 0; i < 4; i++)
    {
        if (w_set & (1 << i))
            list_w_vert[list_w_vert_count++] = m_w_verts[i];
    }
    PHYS_ASSERT(list_w_vert_count > 0 && list_w_vert_count < 4);

    if (list_w_vert_count == 1)
    {
        *v = list_w_vert[0];
        return;
    }

    const phys_vec3 e1 = list_w_vert[1] - list_w_vert[0];
    if (list_w_vert_count == 2)
    {
        const float ne1_sq = AbsSquared(e1);
        PHYS_ASSERT(ne1_sq > 0.0f);
        const float t = phys_dot(e1, list_w_vert[0]) / ne1_sq;
        *v = list_w_vert[0] - t * e1;
        return;
    }

    PHYS_ASSERT(list_w_vert_count == 3);
    const phys_vec3 e2 = list_w_vert[2] - list_w_vert[0];
    const phys_vec3 normal = phys_cross(e1, e2);
    const float nnormal_sq = AbsSquared(normal);
    if (nnormal_sq > phys_sqr(0.1f))
    {
        const float t = phys_dot(normal, list_w_vert[0]) / nnormal_sq;
        *v = t * normal;
        return;
    }

    // Degenerate triangle: project onto its longest side.
    const phys_vec3 sides[3] = {e1, list_w_vert[2] - list_w_vert[1], -e2};
    float nside_sq[3];
    nside_sq[0] = AbsSquared(sides[0]);
    nside_sq[1] = AbsSquared(sides[1]);
    nside_sq[2] = AbsSquared(sides[2]);
    int side_i = 0;
    // TREYBO FIX(0xa0f1ec / 0xa0f1fa / 0xa0f211): `test ah,41h; jp` moves to the later side on NaN too.
    if (!(nside_sq[1] <= nside_sq[side_i]))
        side_i = 1;
    if (!(nside_sq[2] <= nside_sq[side_i]))
        side_i = 2;
    PHYS_ASSERT(nside_sq[side_i] > 0.0f);
    const float t = phys_dot(list_w_vert[side_i], sides[side_i]) / nside_sq[side_i];
    *v = list_w_vert[side_i] - t * sides[side_i];
}

void phys_gjk_info::comp_closest_points(int w_set, phys_vec3 *a, phys_vec3 *b) // 0xa0e710
{
    *a = PHYS_ZERO_VEC;
    *b = PHYS_ZERO_VEC;
    float lambda_sum = 0.0f;
    const phys_gjk_set_info &set_info = m_set_list[w_set];
    for (int i = 0; i < 4; i++)
    {
        if (w_set & (1 << i))
        {
            const float lambda = set_info.m_lamda[i];
            *a += m_a_verts[i] * lambda;
            *b += m_b_verts[i] * lambda;
            lambda_sum += lambda;
        }
    }
    PHYS_ASSERT(lambda_sum >= 0.0f);
    const float oo_lambda_sum = 1.0f / lambda_sum;
    *a *= oo_lambda_sum;
    *b *= oo_lambda_sum;
}

// ---------------------------------------------------------------------------------------------
// GJK
// ---------------------------------------------------------------------------------------------

// Seeds the simplex from the separation loop's last simplex or from the cache. Returns 0 (and starts from
// initial_support_dir with an empty simplex) when there is nothing to seed from.
const int phys_gjk_info::init_gjk(const phys_gjk_input *d, const phys_vec3 &initial_support_dir, bool in_separation_loop) // 0xa0f2e0
{
    int vert_count = 0;
    if (in_separation_loop)
    {
        vert_count = compress_verts(m_w_set);
    }
    else if (d->gjk_ci && d->gjk_ci->is_simplex_valid())
    {
        get_simplex(d->gjk_cg1, d->gjk_cg2, d->gjk_ci, m_a_verts, m_a_inds, m_b_verts, m_b_inds, &vert_count);
        for (int i = 0; i < vert_count; i++)
            m_b_verts[i] = phys_full_multiply(cg2_to_cg1_xform, m_b_verts[i]);
    }

    if (vert_count)
    {
        m_w_set = seed_simplex(vert_count);
        comp_v(m_w_set, &m_support_dir);
        m_last_w_set = m_w_set;
        return 1;
    }

    m_support_dir = initial_support_dir;
    m_last_w_set = 0;
    m_w_set = 0;
    return 0;
}

// Static GJK distance between cg1 and cg2 (cg2 already mapped into cg1's frame by cg2_to_cg1_xform),
// measured from m_gjk_origin. m_support_dir ends as the closest point of the Minkowski difference.
phys_gjk_info::gjk_retval_e phys_gjk_info::gjk(const phys_gjk_input *d, const phys_vec3 &initial_support_dir, bool in_separation_loop) // 0xa0cc20
{
    m_lower_dist_sq = -34.0f;
    m_upper_dist_sq = 34.0f;
    m_gjk_iter = init_gjk(d, initial_support_dir, in_separation_loop);
    for (;;)
    {
        m_upper_dist_sq = AbsSquared(m_support_dir);
        if (m_gjk_iter && m_gjk_pen_thresh_sq > m_upper_dist_sq)
            return GJK_PENETRATING;

        int new_index;
        if (m_w_set & 1)
        {
            if (m_w_set & 2)
                new_index = ((m_w_set & 4) | 8) >> 2; // first free slot: 2 or 3
            else
                new_index = 1;
        }
        else
        {
            new_index = 0;
        }

        d->gjk_cg1->support(-m_support_dir, &m_a_verts[new_index], &m_a_inds[new_index]);
        PHYS_ASSERT_VALID_VEC3(m_a_verts[new_index]);
        d->gjk_cg2->support(phys_inv_multiply(cg2_to_cg1_xform, m_support_dir), &m_b_verts[new_index], &m_b_inds[new_index]);
        PHYS_ASSERT_VALID_VEC3(m_b_verts[new_index]);
        m_b_verts[new_index] = phys_full_multiply(cg2_to_cg1_xform, m_b_verts[new_index]);

        const phys_vec3 w = m_a_verts[new_index] - m_b_verts[new_index] - m_gjk_origin;

        const float dotp = phys_dot(m_support_dir, w);
        if (dotp > 0.0f && m_upper_dist_sq > 0.0f)
        {
            const float lower_dist_sq = dotp * dotp / m_upper_dist_sq;
            if (m_lower_dist_sq < lower_dist_sq)
            {
                m_lower_dist_sq = lower_dist_sq;
                if ((m_flags & FLAG_EXIT_ON_SEP_THRESH) && phys_sqr(m_gjk_sep_thresh) < lower_dist_sq)
                    return GJK_SEPARATED;
            }
        }

        if (m_gjk_iter && m_lower_dist_sq > 0.0f)
        {
            if (m_lower_dist_sq > phys_sqr(1.0f - CONV_THRESH) * m_upper_dist_sq)
                return GJK_VALID;
            for (int i = 0, bit = 1; i < 4; i++, bit <<= 1)
            {
                if ((m_last_w_set & bit) && AbsSquared(w - m_w_verts[i]) < phys_sqr(0.001f))
                    return GJK_VALID;
            }
        }

        m_w_verts[new_index] = w;
        m_w_set |= 1 << new_index;
        m_last_w_set = m_w_set;
        m_w_set = gjk_subalgorithm(m_w_set, new_index);
        if (m_w_set == 15)
            return GJK_PENETRATING;
        comp_v(m_w_set, &m_support_dir);
        // TREYBO FIX(0xa0d0aa): kisakblack printed a (non-retail) warning here; retail gjk() exits silently.
        if (++m_gjk_iter >= 30)
            return GJK_VALID;
    }
}

// GJK with conservative advancement along m_cg1_relative_translation_loc: finds the first lambda in
// [m_start_time, m_end_time] at which the pair comes within m_gjk_sep_thresh, then refines the simplex there.
phys_gjk_info::gjk_retval_e phys_gjk_info::gjk_ray_cast(const phys_gjk_input *d, const phys_vec3 &initial_support_dir, bool in_separation_loop) // 0xa0d100
{
    float lambda = d->m_start_time;
    m_continuous_collision_lambda = lambda;
    m_gjk_origin = -lambda * m_cg1_relative_translation_loc;
    m_lower_dist_sq = -34.0f;
    m_upper_dist_sq = 34.0f;
    const int gjk_iter = init_gjk(d, initial_support_dir, in_separation_loop);
    m_flags &= ~(FLAG_EXIT_ON_SEP_THRESH | FLAG_CC_STOP_ADVANCING | FLAG_CC_SEPARATING_SUPPORT);
    m_gjk_iter = gjk_iter;
    if (d->gjk_ci && d->gjk_ci->is_support_dir_valid())
        m_flags |= FLAG_CC_SEP_THRESH_EXIT;
    else
        m_flags &= ~FLAG_CC_SEP_THRESH_EXIT;
    m_cc_reset_iter = 0;

    const float support_dir_moveback = m_geom_radii_sum < 0.051f ? 0.051f : m_geom_radii_sum;
    PHYS_ASSERT(support_dir_moveback < m_gjk_sep_thresh);

    for (;;)
    {
        int new_index;
        for (;;)
        {
            m_upper_dist_sq = AbsSquared(m_support_dir);
            if (m_gjk_iter)
            {
                if ((m_flags & FLAG_CC_SEP_THRESH_EXIT) && phys_sqr(m_gjk_sep_thresh) > m_upper_dist_sq)
                    m_flags |= FLAG_EXIT_ON_SEP_THRESH;
                if (m_gjk_pen_thresh_sq > m_upper_dist_sq)
                {
                    if ((m_flags & FLAG_CONTINUOUS_COLLISION) && m_continuous_collision_lambda != 0.0f && !(m_flags & FLAG_INTERSECTION_TEST_ONLY))
                        tlWarning("m_continuous_collision_lambda problem");
                    return GJK_PENETRATING;
                }
            }

            if (m_w_set & 1)
            {
                if (m_w_set & 2)
                    new_index = ((m_w_set & 4) | 8) >> 2;
                else
                    new_index = 1;
            }
            else
            {
                new_index = 0;
            }

            d->gjk_cg1->support(-m_support_dir, &m_a_verts[new_index], &m_a_inds[new_index]);
            PHYS_ASSERT_VALID_VEC3(m_a_verts[new_index]);
            d->gjk_cg2->support(phys_inv_multiply(cg2_to_cg1_xform, m_support_dir), &m_b_verts[new_index], &m_b_inds[new_index]);
            PHYS_ASSERT_VALID_VEC3(m_b_verts[new_index]);
            m_b_verts[new_index] = phys_full_multiply(cg2_to_cg1_xform, m_b_verts[new_index]);

            const phys_vec3 w = m_a_verts[new_index] - m_b_verts[new_index] - m_gjk_origin;

            const float v_dot_w = phys_dot(m_support_dir, w);
            if (v_dot_w > 0.0f && m_upper_dist_sq > 0.0f)
            {
                m_flags |= FLAG_CC_SEPARATING_SUPPORT;
                const float lower_dist_sq = v_dot_w * v_dot_w / m_upper_dist_sq;
                if (m_lower_dist_sq < lower_dist_sq)
                {
                    m_lower_dist_sq = lower_dist_sq;
                    if ((m_flags & FLAG_EXIT_ON_SEP_THRESH) && phys_sqr(m_gjk_sep_thresh) < lower_dist_sq)
                        return GJK_SEPARATED;
                }
            }

            bool has_converged = false;
            if (m_gjk_iter && m_lower_dist_sq > 0.0f)
            {
                has_converged = m_lower_dist_sq > phys_sqr(1.0f - CONV_THRESH) * m_upper_dist_sq;
                for (int i = 0, bit = 1; i < 4 && !has_converged; i++, bit <<= 1)
                {
                    if ((m_last_w_set & bit) && AbsSquared(w - m_w_verts[i]) < phys_sqr(0.001f))
                        has_converged = true;
                }
            }

            // Conservative advancement: when the separating plane is at least support_dir_moveback away,
            // move lambda to where the sweep would reach it.
            if (!(m_flags & FLAG_CC_STOP_ADVANCING) && v_dot_w > 0.0f &&
                phys_sqr(support_dir_moveback) * m_upper_dist_sq <= phys_sqr(v_dot_w))
            {
                const float nsupport_dir = sqrtf(m_upper_dist_sq);
                PHYS_ASSERT(nsupport_dir > 0.0f);
                const float move = -phys_dot(m_cg1_relative_translation_loc, m_support_dir);
                float ray_end_dist_numer = v_dot_w - nsupport_dir * support_dir_moveback;
                if (move <= 0.0f)
                {
                    // Not approaching: separated for the rest of the sweep once the plane clears the threshold.
                    if (ray_end_dist_numer > 0.0f)
                    {
                        if (phys_sqr(m_gjk_sep_thresh) < m_lower_dist_sq)
                            return GJK_SEPARATED;
                        m_flags |= FLAG_EXIT_ON_SEP_THRESH | FLAG_CC_STOP_ADVANCING;
                    }
                }
                else if (ray_end_dist_numer >= 0.0f)
                {
                    const float sd_dot_origin = phys_dot(m_support_dir, m_gjk_origin);
                    const float new_lambda = (sd_dot_origin + ray_end_dist_numer) / move;
                    if (m_continuous_collision_lambda + PHYS_EPS_1E4 < new_lambda)
                    {
                        if (!(d->m_end_time < new_lambda)) // retail NaN behaviour: a NaN lambda advances
                        {
                            lambda = new_lambda;
                            break;
                        }
                        ray_end_dist_numer = ray_end_dist_numer - move * d->m_end_time + sd_dot_origin;
                        if (m_gjk_sep_thresh * nsupport_dir < ray_end_dist_numer)
                        {
                            PHYS_ASSERT(ray_end_dist_numer > 0.0f);
                            return GJK_SEPARATED;
                        }
                        lambda = d->m_end_time;
                        m_flags |= FLAG_EXIT_ON_SEP_THRESH | FLAG_CC_STOP_ADVANCING;
                        break;
                    }
                }
            }

            if (has_converged)
                return GJK_VALID;

            m_w_verts[new_index] = w;
            m_w_set |= 1 << new_index;
            m_last_w_set = m_w_set;
            m_w_set = gjk_subalgorithm(m_w_set, new_index);
            if (m_w_set == 15)
            {
                if ((m_flags & FLAG_CONTINUOUS_COLLISION) && m_continuous_collision_lambda != 0.0f)
                    tlWarning("m_continuous_collision_lambda problem");
                return GJK_PENETRATING;
            }
            comp_v(m_w_set, &m_support_dir);
            if (++m_gjk_iter >= 30)
            {
                tlWarning("gjk reached the maximum number of iterations.");
                return m_lower_dist_sq <= 0.0f ? GJK_PENETRATING : GJK_VALID;
            }
        }

        // Advance to lambda and restart GJK from the current simplex.
        if (lambda - m_continuous_collision_lambda <= PHYS_EPS_1E4)
            m_flags |= FLAG_EXIT_ON_SEP_THRESH | FLAG_CC_STOP_ADVANCING;
        m_continuous_collision_lambda = lambda;
        m_gjk_origin = -lambda * m_cg1_relative_translation_loc;
        m_cc_reset_iter++;
        m_lower_dist_sq = -34.0f;
        m_upper_dist_sq = 34.0f;
        if (m_cc_reset_iter > 1)
            m_flags |= FLAG_CC_SEP_THRESH_EXIT;
        if (m_cc_reset_iter >= MAX_CC_RESET_ITERS)
            m_flags |= FLAG_EXIT_ON_SEP_THRESH | FLAG_CC_STOP_ADVANCING;
        if (!m_w_set)
            m_w_set = 1 << new_index;
        const int did_init = init_gjk(d, m_support_dir, true);
        PHYS_ASSERT(did_init);
        PHYS_ASSERT(m_w_set != 0);
        PHYS_ASSERT(m_last_w_set == m_w_set);
        m_gjk_iter++;
    }
}

// Separation loop for penetrating pairs: GJK is re-run from an origin pushed 17 units (0.5 * 34) out along
// the separation direction, until the shifted query stops penetrating, which yields a contact normal/depth.
phys_gjk_info::gjk_retval_e phys_gjk_info::collide(const phys_gjk_input *d) // 0xa0db10
{
    m_gjk_origin = PHYS_ZERO_VEC;
    m_flags |= FLAG_EXIT_ON_SEP_THRESH;
    const phys_vec3 initial_support_dir = get_initial_support_dir(d);
    PHYS_ASSERT_VALID_VEC3(initial_support_dir);

    const gjk_retval_e retv = (m_flags & FLAG_CONTINUOUS_COLLISION) ? gjk_ray_cast(d, initial_support_dir, false) : gjk(d, initial_support_dir, false);
    if (retv != GJK_PENETRATING)
    {
        PHYS_ASSERT(retv != GJK_INVALID);
        PHYS_ASSERT(retv == GJK_SEPARATED || retv == GJK_VALID);
        return retv;
    }

    if (m_flags & FLAG_INTERSECTION_TEST_ONLY)
        return GJK_PENETRATING;

    if ((m_flags & FLAG_CONTINUOUS_COLLISION) && m_continuous_collision_lambda != 0.0f)
        tlWarning("m_continuous_collision_lambda problem");
    m_continuous_collision_lambda = 0.0f;

    phys_vec3 support_dir = (d->gjk_ci && d->gjk_ci->is_support_dir_valid()) ? initial_support_dir : gjk_sep_dir::comp_sep_dir(d, this);
    float nsupport_dir_sq = AbsSquared(support_dir);
    if (nsupport_dir_sq < phys_sqr(GJK_MIN_SUPPORT_DIR_LENGTH))
    {
        if (AbsSquared(initial_support_dir) < phys_sqr(GJK_MIN_SUPPORT_DIR_LENGTH))
        {
            PHYS_ASSERT_MSG(0, "initial support dir invalid.");
            return GJK_INVALID;
        }
        support_dir = initial_support_dir;
    }

    m_flags &= ~FLAG_EXIT_ON_SEP_THRESH;
    nsupport_dir_sq = AbsSquared(support_dir);
    for (int iter = 1;; iter++)
    {
        phys_vec3 a_vert;
        phys_vec3 a_ind;
        d->gjk_cg1->support(-support_dir, &a_vert, &a_ind);
        const phys_vec3 w = a_vert - d->gjk_cg2->support_only(cg2_to_cg1_xform, support_dir);
        PHYS_ASSERT_VALID_VEC3(w);

        const float nsupport_dir = sqrtf(nsupport_dir_sq);
        m_gjk_origin = support_dir * -(17.0f / nsupport_dir - phys_dot(w, support_dir) / nsupport_dir_sq);

        if (gjk(d, support_dir, iter > 1) == GJK_PENETRATING)
        {
            tlWarning("gjk has possibly failed.");
            if (iter != 1 || !d->gjk_ci || !d->gjk_ci->is_simplex_valid())
                break;
            d->gjk_ci->invalidate_simplex();
            if (gjk(d, support_dir, false) == GJK_PENETRATING)
                break;
        }

        comp_v(m_w_set, &support_dir);
        nsupport_dir_sq = AbsSquared(support_dir);
        PHYS_ASSERT(AbsSquared(support_dir) >= m_gjk_pen_thresh_sq);
        if (phys_sqr(1.0f - SEP_CONV_THRESH) * m_lower_dist_sq < 289.0f || iter >= 10)
            return GJK_PENETRATING;
    }

    PHYS_ASSERT_MSG(0, "gjk has failed.");
    return GJK_INVALID;
}

const phys_vec3 phys_gjk_info::get_initial_support_dir(const phys_gjk_input *d) // 0xa0ea10
{
    if (d->gjk_ci && d->gjk_ci->is_support_dir_valid())
        return phys_inv_multiply(*d->cg1_to_world_xform, d->gjk_ci->get_support_dir());

    const phys_vec3 cg2_center = phys_full_multiply(cg2_to_cg1_xform, d->gjk_cg2->get_center());
    const phys_vec3 dir = d->gjk_cg1->get_center() - cg2_center;
    if (!(AbsSquared(dir) >= phys_sqr(GJK_MIN_SUPPORT_DIR_LENGTH))) // TREYBO FIX(0xa0eac3): NaN -> PHYS_X_VEC
    {
        PHYS_ASSERT_INFO(pai_gjk_initial_support_dir, 0, "degenerate gjk initial support dir.");
        return PHYS_X_VEC;
    }
    return dir;
}

void phys_gjk_info::gjk_cache_update_invalid(const phys_gjk_input *d) // 0xa0eba0
{
    if (d->gjk_ci)
    {
        d->gjk_ci->set_flag_was_touched();
        d->gjk_ci->invalidate_support_dir();
        d->gjk_ci->invalidate_simplex();
    }
}

void phys_gjk_info::gjk_cache_update_test_only_valid(const phys_gjk_input *d) // 0xa0ebd0
{
    if (d->gjk_ci)
    {
        d->gjk_ci->set_flag_was_touched();
        d->gjk_ci->set_support_dir(phys_multiply(*d->cg1_to_world_xform, m_support_dir));
        d->gjk_ci->invalidate_simplex();
    }
}

void phys_gjk_info::gjk_cache_update_colliding(const phys_gjk_input *d) // 0xa0ec40
{
    if (d->gjk_ci)
    {
        d->gjk_ci->set_flag_was_touched();
        d->gjk_ci->set_support_dir(phys_multiply(*d->cg1_to_world_xform, cg1_cinfo_loc.m_n));
        set_simplex(d->gjk_cg1, d->gjk_cg2, d->gjk_ci, -cg1_cinfo_loc.m_n, phys_inv_multiply(cg2_to_cg1_xform, cg1_cinfo_loc.m_n), m_a_inds, m_b_inds, m_w_set);
    }
}

void phys_gjk_info::gjk_cache_update_test_only_penetrating(const phys_gjk_input *d) // 0xa0ed50
{
    if (d->gjk_ci)
    {
        d->gjk_ci->set_flag_was_touched();
        d->gjk_ci->invalidate_simplex();
    }
}

const bool phys_gjk_info::phys_collide_do_gjk_collide(const phys_gjk_input *d) // 0xa0dfd0
{
    PHYS_ASSERT((d->m_start_time >= 0.0f) && (d->m_start_time <= d->m_end_time) && (d->m_end_time <= 1.0f));
    PHYS_ASSERT_VALID_FLOAT(d->m_start_time); // TREYBO: extra assert (not in retail 0xa0dfd0)

    m_flags = 0;
    phys_full_inv_multiply_mat(cg2_to_cg1_xform, *d->cg1_to_world_xform, *d->cg2_to_world_xform);
    if (d->m_continuous_collision)
    {
        m_cg1_relative_translation_loc = phys_inv_multiply(*d->cg1_to_world_xform, d->m_cg1_translation - d->m_cg2_translation);
        m_flags |= FLAG_CONTINUOUS_COLLISION;
    }

    m_geom_radii_sum = d->cg2_radius + d->cg1_radius;
    m_gjk_sep_thresh = m_geom_radii_sum + d->m_sep_thresh;
    if (d->m_intersection_test_only)
    {
        m_flags |= FLAG_INTERSECTION_TEST_ONLY;
        m_gjk_pen_thresh_sq = phys_sqr(m_gjk_sep_thresh);
    }
    else
    {
        m_gjk_pen_thresh_sq = phys_sqr(0.034f);
    }

    const gjk_retval_e retv = collide(d);
    if (retv == GJK_INVALID)
    {
        PHYS_ASSERT_MSG(0, "phys_gjk failed");
        gjk_cache_update_invalid(d);
        return false;
    }

    if (retv == GJK_SEPARATED)
    {
        gjk_cache_update_test_only_valid(d);
        return false;
    }

    if (m_flags & FLAG_INTERSECTION_TEST_ONLY)
    {
        PHYS_ASSERT(retv == GJK_VALID || retv == GJK_PENETRATING);
        if (m_w_set == 15)
        {
            gjk_cache_update_invalid(d);
        }
        else if (retv == GJK_VALID)
        {
            comp_v(m_w_set, &m_support_dir);
            gjk_cache_update_test_only_valid(d);
        }
        else
        {
            PHYS_ASSERT(retv == GJK_PENETRATING);
            gjk_cache_update_test_only_penetrating(d);
        }
        return true;
    }

    PHYS_ASSERT(retv == GJK_VALID || retv == GJK_PENETRATING);
    if (m_w_set == 15)
    {
        PHYS_ASSERT_MSG(0, "phys_gjk failed");
        gjk_cache_update_invalid(d);
        return false;
    }

    PHYS_ASSERT(m_w_set < 15 && m_w_set > 0);
    comp_closest_points(m_w_set, &cg1_cinfo_loc.m_p1, &cg1_cinfo_loc.m_p2);
    comp_v(m_w_set, &cg1_cinfo_loc.m_n);
    cg1_cinfo_loc.m_n /= Abs(cg1_cinfo_loc.m_n);
    cg1_cinfo_loc.m_p1 -= cg1_cinfo_loc.m_n * d->cg1_radius;
    cg1_cinfo_loc.m_p2 += cg1_cinfo_loc.m_n * d->cg2_radius;
    gjk_cache_update_colliding(d);
    return true;
}
