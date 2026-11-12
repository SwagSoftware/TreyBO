#include "tl/physics/include/collision/phys_gjk_sep_dir.h"
#include "tl/physics/include/collision/phys_gjk.h"
#include "tl/base/include/tl_system.h"

// Triangles (indices into the 12 sample directions) of the icosahedron-like sample hull.
static const int m_list_triangle[60] =
	{
		10,
		1,
		4,
		8,
		11,
		6,
		0,
		10,
		4,
		2,
		8,
		6,
		0,
		2,
		10,
		2,
		0,
		8,
		9,
		1,
		10,
		9,
		2,
		6,
		2,
		9,
		10,
		1,
		7,
		4,
		5,
		11,
		8,
		0,
		5,
		8,
		5,
		7,
		11,
		5,
		0,
		4,
		7,
		5,
		4,
		7,
		3,
		11,
		11,
		3,
		6,
		3,
		9,
		6,
		9,
		3,
		1,
		3,
		7,
		1,
};

static const phys_vec3 m_list_support_dir[12] =
	{
		phys_vec3(-0.515304f, 0.445123f, -0.732344f),
		phys_vec3(-0.222601f, 0.275979f, 0.935032f),
		phys_vec3(0.497603f, 0.665553f, -0.556264f),
		phys_vec3(0.515302f, -0.445124f, 0.732344f),
		phys_vec3(-0.960414f, 0.204353f, 0.189327f),
		phys_vec3(-0.678502f, -0.561016f, -0.474231f),
		phys_vec3(0.960414f, -0.204353f, -0.189327f),
		phys_vec3(-0.497604f, -0.665553f, 0.556264f),
		phys_vec3(0.222601f, -0.275977f, -0.935032f),
		phys_vec3(0.678504f, 0.561015f, 0.474231f),
		phys_vec3(-0.23354f, 0.962414f, 0.138628f),
		phys_vec3(0.23354f, -0.962414f, -0.138628f),
};

const phys_vec3 gjk_sep_dir::comp_sep_dir(const phys_gjk_input * m_pcd, phys_gjk_info * m_gjk_info)
{
	phys_vec3 m_normal_list[20];
	phys_vec3 m_vert_list[12];
	phys_vec3 m_mesh_a_center(0.0f, 0.0f, 0.0f);
	phys_vec3 m_mesh_b_center(0.0f, 0.0f, 0.0f);
	phys_vec3 dir(0.0f, 0.0f, 0.0f);

	for (int i = 0; i < 12; ++i)
	{
		phys_vec3 m_a_vert;
		phys_vec3 a_ind;
		m_pcd->gjk_cg1->support(m_list_support_dir[i], &m_a_vert, &a_ind);
		PHYS_ASSERT_VALID_VEC3(m_a_vert);

		// -dir is taken into cg2's frame (transposed rotation) before calling support.
		phys_vec3 m_b_loc_vert;
		phys_vec3 b_ind;
		m_pcd->gjk_cg2->support(phys_inv_multiply(m_gjk_info->cg2_to_cg1_xform, -m_list_support_dir[i]), &m_b_loc_vert, &b_ind);
		PHYS_ASSERT_VALID_VEC3(m_b_loc_vert);

		const phys_vec3 m_b_vert = phys_full_multiply(m_gjk_info->cg2_to_cg1_xform, m_b_loc_vert);
		m_vert_list[i] = m_a_vert - m_b_vert;
		m_mesh_a_center += m_a_vert;
		m_mesh_b_center += m_b_vert;
		dir += m_vert_list[i];
	}
	m_mesh_a_center *= 1.0f / 12.0f;
	m_mesh_b_center *= 1.0f / 12.0f;
	dir *= 1.0f / 12.0f;

	// Pick the outward face of the sampled Minkowski hull with the largest signed origin distance.
	phys_vec3 * best_normal = 0;
	float best_dist_sq = 0.0f;
	for (int tri = 0; tri < 20; ++tri)
	{
		const phys_vec3 & v0 = m_vert_list[m_list_triangle[tri * 3 + 0]];
		const phys_vec3 & v1 = m_vert_list[m_list_triangle[tri * 3 + 1]];
		const phys_vec3 & v2 = m_vert_list[m_list_triangle[tri * 3 + 2]];

		phys_vec3 & normal = m_normal_list[tri];
		normal = phys_cross(v1 - v0, v2 - v0);
		if (phys_dot(v0 - dir, normal) < 0.0f)
			normal = -normal;

		const float normal_sq = AbsSquared(normal);
		if (normal_sq >= phys_sqr(GJK_MIN_SUPPORT_DIR_LENGTH))
		{
			const float dotp = phys_dot(-v0, normal);
			float dist_sq = phys_sqr(dotp) / normal_sq;
			if (dotp < 0.0f)
				dist_sq = -dist_sq;
			if (!best_normal || best_dist_sq < dist_sq)
			{
				best_dist_sq = dist_sq;
				best_normal = &normal;
			}
		}
	}

	if (best_normal)
	{
		PHYS_ASSERT(AbsSquared(*best_normal) >= phys_sqr(GJK_MIN_SUPPORT_DIR_LENGTH));
		return -*best_normal;
	}

	dir = m_mesh_a_center - m_mesh_b_center;
	if (AbsSquared(dir) >= phys_sqr(GJK_MIN_SUPPORT_DIR_LENGTH))
		return dir;

	tlWarning("bad separation dir.\n");
	return PHYS_X_VEC;
}

const float gjk_sep_dir::dist_seg(const phys_vec3 & v0, const phys_vec3 & v1, const phys_vec3 & mesh_center, phys_vec3 * direction)
{
	const phys_vec3 dif_v1_v0 = v1 - v0;
	const float len_sq = AbsSquared(dif_v1_v0);
	if (len_sq > 0.001f)
	{
		const float t_ = tl_clamp(phys_dot(-v0, dif_v1_v0) / len_sq, 0.0f, 1.0f);
		*direction = (1.0f - t_) * v0 + t_ * v1;
	}
	else
	{
		*direction = v1;
	}

	float dist = -AbsSquared(*direction);
	if (phys_dot(*direction, *direction - mesh_center) < 0.0f)
	{
		*direction = -*direction;
		dist = -dist;
	}
	return dist;
}
