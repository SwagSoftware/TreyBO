#pragma once

// Retail: codsrc\tl\physics\include\collision\phys_convex_hull.h (header-only class; out-of-line copies
// 0x84e730-0x850100, phys_static_array instantiations 0x854ad0-0x854da0). The CoD feeders (generate_*_chull,
// get_*_chull, create_chull, chull_t) are game code and are not here; see phys_convex_hull.cpp.

#include "tl/physics/include/phys_math.h"
#include "tl/physics/include/phys_array_base.inc"

// Retail has two copies (0xd19c48, 0xd1b7e0): a header constant.
const float CONVEX_HULL_VERT_ADD_DIST_THRESH = 0.034f;

// Incremental (quickhull-style) hull of m_vertex_buffer, growing by the support vertex that adds the most volume.
class phys_convex_hull
{
public:
    struct ch_triangle
    {
        phys_vec3 m_normal;
        phys_vec3 *m_verts[3];

        // Copy assignment is implicit (0x850100: m_normal via phys_vec3::operator=, then the 3 pointers).

        const float get_dist(const phys_vec3 &vert) const { return phys_dot(m_normal, vert - *m_verts[0]); } // 0x84f9c0
    };

    struct ch_edge
    {
        phys_vec3 *m_verts[2];
    };

    phys_static_array<phys_vec3, 6144> m_vertex_buffer; // input points; also receives the off-plane seed vertex
    phys_static_array<phys_vec3 *, 6144> m_intermediate_vertex_list;
    phys_static_array<ch_triangle, 256> m_intermediate_triangle_list;
    phys_static_array<ch_edge, 128> m_intermediate_edge_list;
    phys_static_array<phys_vec3 *, 64> m_convex_hull_vert_list;
    phys_static_array<ch_triangle, 64> m_convex_hull_triangle_list;

    // Constructor / destructor are implicit (0x855250 / 0x8559d0).

    void add_convex_hull_vert(phys_vec3 **vert) // 0x84eb70
    {
        PHYS_ASSERT(vert);
        *m_convex_hull_vert_list.add() = *vert;
        m_intermediate_vertex_list.remove(vert);
    }

    // Removes a shared edge, adds a new one: what is left is the horizon of the removed triangles.
    void add_intermediate_edge(phys_vec3 *v0, phys_vec3 *v1) // 0x84fe40
    {
        PHYS_ASSERT(v0 && v1 && v0 != v1);
        for (phys_static_array<ch_edge, 128>::iterator edge_i = m_intermediate_edge_list.begin(); edge_i != m_intermediate_edge_list.end(); edge_i++)
        {
            ch_edge *const edge = &*edge_i;
            if ((edge->m_verts[0] == v0 && edge->m_verts[1] == v1) || (edge->m_verts[0] == v1 && edge->m_verts[1] == v0))
            {
                m_intermediate_edge_list.remove_slow(edge);
                return;
            }
        }
        ch_edge *const edge = m_intermediate_edge_list.add();
        edge->m_verts[0] = v0;
        edge->m_verts[1] = v1;
    }

    void add_triangle_edges(ch_triangle *tri) // 0x84fde0
    {
        add_intermediate_edge(tri->m_verts[0], tri->m_verts[1]);
        add_intermediate_edge(tri->m_verts[1], tri->m_verts[2]);
        add_intermediate_edge(tri->m_verts[2], tri->m_verts[0]);
    }

    void create_intermediate_triangle(phys_vec3 *v0, phys_vec3 *v1, phys_vec3 *v2) // 0x84ec30
    {
        ch_triangle *const tri = m_intermediate_triangle_list.add();
        tri->m_verts[0] = v0;
        tri->m_verts[1] = v1;
        tri->m_verts[2] = v2;
        const phys_vec3 normal = phys_cross(*v1 - *v0, *v2 - *v0);
        const float nnormal = Abs(normal);
        PHYS_ASSERT(nnormal > 0.00001f);
        tri->m_normal = (1.0f / nnormal) * normal;
    }

    phys_vec3 **support_intermediate_verts(const phys_vec3 &dir) // 0x84ee30
    {
        phys_vec3 **best_vert = 0;
        float best_dotp = -1000000.0f;
        for (phys_static_array<phys_vec3 *, 6144>::iterator vert_i = m_intermediate_vertex_list.begin(); vert_i != m_intermediate_vertex_list.end(); vert_i++)
        {
            const float dotp = phys_dot(dir, **vert_i);
            if (dotp > best_dotp)
            {
                best_vert = &*vert_i;
                best_dotp = dotp;
            }
        }
        PHYS_ASSERT(best_vert);
        return best_vert;
    }

    // Seeds the hull with the largest-area triangle and one extra vertex pushed off its plane.
    void calculate_initial_triangle_vertices() // 0x84f210
    {
        float largest_twice_area_sq = 0.0f;
        phys_vec3 **best_verts[3];
        const int NUM_SPHERE_VERTS = m_intermediate_vertex_list.get_count();
        for (int i = 0; i < NUM_SPHERE_VERTS - 2; ++i)
        {
            phys_vec3 **const z0 = &m_intermediate_vertex_list[i];
            for (int j = i + 1; j < NUM_SPHERE_VERTS - 1; ++j)
            {
                phys_vec3 **const z1 = &m_intermediate_vertex_list[j];
                for (int k = j + 1; k < NUM_SPHERE_VERTS; ++k)
                {
                    phys_vec3 **const z2 = &m_intermediate_vertex_list[k];
                    const float twice_area_sq = AbsSquared(phys_cross(**z1 - **z0, **z2 - **z0));
                    if (twice_area_sq > largest_twice_area_sq)
                    {
                        largest_twice_area_sq = twice_area_sq;
                        best_verts[0] = z0;
                        best_verts[1] = z1;
                        best_verts[2] = z2;
                    }
                }
            }
        }
        PHYS_ASSERT_STR(largest_twice_area_sq > 0.0f, "largest_twice_area_sq>0.0f");

        phys_vec3 nn = phys_cross(**best_verts[1] - **best_verts[0], **best_verts[2] - **best_verts[0]);
        nn = phys_Unitize(nn);
        *m_vertex_buffer.add() = **best_verts[0] + nn;
        *m_intermediate_vertex_list.add() = &m_vertex_buffer[m_vertex_buffer.get_count() - 1];

        // remove() swaps the last slot in, so take the higher slots first to keep the others valid.
        bool swapped = true;
        while (swapped)
        {
            swapped = false;
            for (int k = 0; k < 2; ++k)
            {
                if (best_verts[k] < best_verts[k + 1])
                {
                    phys_vec3 **const t = best_verts[k];
                    best_verts[k] = best_verts[k + 1];
                    best_verts[k + 1] = t;
                    swapped = true;
                }
            }
        }
        add_convex_hull_vert(best_verts[0]);
        add_convex_hull_vert(best_verts[1]);
        add_convex_hull_vert(best_verts[2]);
    }

    void init_convex_hull() // 0x84ef40
    {
        m_intermediate_vertex_list.remove_all();
        for (phys_static_array<phys_vec3, 6144>::iterator vert_i = m_vertex_buffer.begin(); vert_i != m_vertex_buffer.end(); vert_i++)
            *m_intermediate_vertex_list.add() = &*vert_i;
        m_convex_hull_vert_list.remove_all();
        m_intermediate_triangle_list.remove_all();
        m_convex_hull_triangle_list.remove_all();

        calculate_initial_triangle_vertices();
        PHYS_ASSERT(m_convex_hull_vert_list.get_count() == 3);

        // Both windings, so every remaining vertex is outside one of them.
        create_intermediate_triangle(m_convex_hull_vert_list[0], m_convex_hull_vert_list[1], m_convex_hull_vert_list[2]);
        create_intermediate_triangle(m_convex_hull_vert_list[1], m_convex_hull_vert_list[0], m_convex_hull_vert_list[2]);
    }

    const float tetrahedron_volume(const phys_vec3 &a, const phys_vec3 &b, const phys_vec3 &c, const phys_vec3 &d) const // 0x84fa70
    {
        return fabsf(phys_dot(a - d, phys_cross(b - d, c - d))) / 6.0f;
    }

    const float calc_expansion_volume(const phys_vec3 &vert) // 0x84f8b0
    {
        float volume = 0.0f;
        for (phys_static_array<ch_triangle, 256>::iterator tri_i = m_intermediate_triangle_list.begin(); tri_i != m_intermediate_triangle_list.end(); tri_i++)
        {
            ch_triangle &tri = *tri_i;
            const float vert_dist = tri.get_dist(vert);
            if (vert_dist > CONVEX_HULL_VERT_ADD_DIST_THRESH)
                volume += tetrahedron_volume(vert, *tri.m_verts[0], *tri.m_verts[1], *tri.m_verts[2]);
        }
        PHYS_ASSERT(volume > 0.0f);
        return volume;
    }

    // Removes every triangle that sees vert and collects their horizon edges.
    void create_edge_list(const phys_vec3 &vert) // 0x84fc10
    {
        m_intermediate_edge_list.remove_all();
        for (phys_static_array<ch_triangle, 256>::iterator tri_i = m_intermediate_triangle_list.begin(); tri_i != m_intermediate_triangle_list.end();)
        {
            ch_triangle *const tri = &*tri_i;
            if (tri->get_dist(vert) > CONVEX_HULL_VERT_ADD_DIST_THRESH)
            {
                add_triangle_edges(tri);
                m_intermediate_triangle_list.remove_slow(tri); // the last triangle moves into this slot
            }
            else
            {
                tri_i++;
            }
        }
        PHYS_ASSERT(m_intermediate_edge_list.get_count() >= 3);
    }

    void remove_inside_verts() // 0x84ffb0
    {
        for (phys_static_array<phys_vec3 *, 6144>::iterator vert_i = m_intermediate_vertex_list.begin(); vert_i != m_intermediate_vertex_list.end();)
        {
            bool inside = true;
            for (phys_static_array<ch_triangle, 256>::iterator tri_i = m_intermediate_triangle_list.begin(); tri_i != m_intermediate_triangle_list.end() && inside; tri_i++)
            {
                const float vert_dist = (*tri_i).get_dist(**vert_i);
                inside = vert_dist <= CONVEX_HULL_VERT_ADD_DIST_THRESH;
            }
            if (inside)
                m_intermediate_vertex_list.remove(&*vert_i);
            else
                vert_i++;
        }
    }

    // Retail game callers use (128, 0.0f).
    void compute_convex_hull(const int max_verts, const float min_expansion_volume_percent) // 0x84e730
    {
        PHYS_ASSERT(max_verts >= 3);
        init_convex_hull();

        float total_volume = 0.0f;
        while (m_convex_hull_vert_list.get_count() < max_verts
            && m_intermediate_triangle_list.get_count() > 0
            && m_intermediate_vertex_list.get_count() > 0)
        {
            ch_triangle *best_tri = 0;
            phys_vec3 **best_vert = 0;
            float best_volume = 0.0f;
            for (phys_static_array<ch_triangle, 256>::iterator tri_i = m_intermediate_triangle_list.begin(); tri_i != m_intermediate_triangle_list.end();)
            {
                ch_triangle *const tri = &*tri_i;
                phys_vec3 **const support_vert = support_intermediate_verts(tri->m_normal);
                const float support_vert_dist = tri->get_dist(**support_vert);
                if (support_vert_dist > CONVEX_HULL_VERT_ADD_DIST_THRESH)
                {
                    const float volume = calc_expansion_volume(**support_vert);
                    if (volume > best_volume)
                    {
                        best_tri = tri;
                        best_vert = support_vert;
                        best_volume = volume;
                    }
                    tri_i++;
                }
                else
                {
                    // Nothing lies outside this face: it is final.
                    *m_convex_hull_triangle_list.add() = *tri;
                    m_intermediate_triangle_list.remove_slow(tri);
                }
            }
            if (!best_tri)
                break;

            PHYS_ASSERT(best_vert);
            PHYS_ASSERT(best_volume > 0.0f);
            PHYS_ASSERT(best_tri->get_dist(**best_vert) > CONVEX_HULL_VERT_ADD_DIST_THRESH);

            // TREYBO FIX(0x84e9dd): `comiss; jbe -> 100000` also takes the fallback for a NaN total.
            const float increase_percent_ = !(total_volume > PHYS_EPS_1E4) ? 100000.0f : best_volume / total_volume;
            if (min_expansion_volume_percent > increase_percent_)
                break;
            total_volume += best_volume;

            create_edge_list(**best_vert);
            for (phys_static_array<ch_edge, 128>::iterator edge_i = m_intermediate_edge_list.begin(); edge_i != m_intermediate_edge_list.end(); edge_i++)
                create_intermediate_triangle(*best_vert, (*edge_i).m_verts[0], (*edge_i).m_verts[1]);
            add_convex_hull_vert(best_vert);
            remove_inside_verts();
        }

        while (m_intermediate_triangle_list.get_count() > 0)
        {
            ch_triangle *const list_head = m_intermediate_triangle_list.get_list_head();
            *m_convex_hull_triangle_list.add() = *list_head;
            m_intermediate_triangle_list.remove_slow(list_head);
        }
    }
};

// TreyBO-new: engine-neutral entry points replacing the game-side feeders (bodies in phys_convex_hull.cpp).
// hull is ~134 KB (x86) / larger on x64: keep it static or heap-allocated.
void phys_convex_hull_build(phys_convex_hull &hull, const phys_vec3 *points, const int point_count, const int max_verts = 128, const float min_expansion_volume_percent = 0.0f);
// Output sizes: hull.m_convex_hull_vert_list.get_count() verts,
// 3 * hull.m_convex_hull_triangle_list.get_count() indices.
void phys_convex_hull_extract(phys_convex_hull &hull, phys_vec3 *out_verts, ushort *out_inds);
