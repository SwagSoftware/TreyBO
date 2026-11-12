// Convex hull build / extract helpers.

#include "tl/physics/include/collision/phys_convex_hull.h"

// Load the point cloud, then compute.
void phys_convex_hull_build(phys_convex_hull & hull, const phys_vec3 * points, const int point_count, const int max_verts, const float min_expansion_volume_percent)
{
	hull.m_vertex_buffer.remove_all();
	for (int i = 0; i < point_count; ++i)
		*hull.m_vertex_buffer.add() = points[i]; // xyz only
	hull.compute_convex_hull(max_verts, min_expansion_volume_percent);
}

// Copies the hull out as a vertex list and triangle index list; the caller owns the buffers.
void phys_convex_hull_extract(phys_convex_hull & hull, phys_vec3 * out_verts, ushort * out_inds)
{
	int vert_i = 0;
	for (phys_static_array<phys_vec3 *, 64>::iterator vi = hull.m_convex_hull_vert_list.begin(); vi != hull.m_convex_hull_vert_list.end(); vi++)
		out_verts[vert_i++] = **vi;

	int ind_i = 0;
	for (phys_static_array<phys_convex_hull::ch_triangle, 64>::iterator tri_i = hull.m_convex_hull_triangle_list.begin(); tri_i != hull.m_convex_hull_triangle_list.end(); tri_i++)
	{
		const phys_convex_hull::ch_triangle & tri = *tri_i;
		ushort slot_i = 0;
		for (phys_static_array<phys_vec3 *, 64>::iterator vi = hull.m_convex_hull_vert_list.begin(); vi != hull.m_convex_hull_vert_list.end(); vi++)
		{
			if (tri.m_verts[0] == *vi)
				out_inds[ind_i] = slot_i;
			if (tri.m_verts[1] == *vi)
				out_inds[ind_i + 1] = slot_i;
			if (tri.m_verts[2] == *vi)
				out_inds[ind_i + 2] = slot_i;
			++slot_i;
		}
		ind_i += 3;
	}
}
