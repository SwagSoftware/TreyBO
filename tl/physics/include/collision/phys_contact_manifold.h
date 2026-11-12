#pragma once

#include "tl/physics/include/phys_math.h"
#include "tl/physics/include/phys_mem.h"

class phys_collision_pair;
class phys_gjk_info;
class contact_point_info;
class rigid_body_constraint_contact;
class phys_transient_allocator;

// Description string of every allocator-overflow assert in this file.
extern const char * g_contact_manifold_error_msg;

// ---------------------------------------------------------------------------------------------
// phys_vec2 helpers (phys_vec2 itself, in old_phys_math.h, has no operators).
// ---------------------------------------------------------------------------------------------

inline const phys_vec2 operator+(const phys_vec2 & v1, const phys_vec2 & v2)
{
	return phys_vec2(v1.x + v2.x, v1.y + v2.y);
}
inline const phys_vec2 operator-(const phys_vec2 & v1, const phys_vec2 & v2)
{
	return phys_vec2(v1.x - v2.x, v1.y - v2.y);
}
inline const phys_vec2 operator*(const float c, const phys_vec2 & v)
{
	return phys_vec2(c * v.x, c * v.y);
}
inline const phys_vec2 operator*(const phys_vec2 & v, const float c)
{
	return phys_vec2(c * v.x, c * v.y);
}
inline void operator+=(phys_vec2 & v1, const phys_vec2 & v2)
{
	v1.x += v2.x;
	v1.y += v2.y;
}
inline void operator*=(phys_vec2 & v, const float d)
{
	v.x *= d;
	v.y *= d;
}

inline const float AbsSquared(const phys_vec2 & v)
{
	return v.x * v.x + v.y * v.y;
}
inline const float Abs(const phys_vec2 & v)
{
	return sqrtf(v.x * v.x + v.y * v.y);
}
inline const float phys_v2_dot(const phys_vec2 & v1, const phys_vec2 & v2)
{
	return v1.x * v2.x + v1.y * v2.y;
}
inline const float phys_v2_cross(const phys_vec2 & v1, const phys_vec2 & v2)
{
	return v1.x * v2.y - v2.x * v1.y;
}
inline const bool phys_v2_le(const phys_vec2 & v1, const phys_vec2 & v2)
{
	return (v1.x < v2.x) || (v1.x == v2.x && v1.y < v2.y);
}
inline const phys_vec2 phys_v2_rotr(const phys_vec2 & v)
{
	return phys_vec2(-v.y, v.x);
}

inline const phys_vec3 phys_v2_to_v3_multiply(const phys_mat44 & m, const phys_vec2 & v)
{
	return v.GetX() * m.GetX() + v.GetY() * m.GetY();
}

inline const phys_vec2 phys_v3_to_v2_inv_multiply(const phys_mat44 & m, const phys_vec3 & v)
{
	return phys_vec2(phys_dot(v, m.GetX()), phys_dot(v, m.GetY()));
}

// ---------------------------------------------------------------------------------------------
// contact_manifold_mesh_point / phys_contact_manifold
// ---------------------------------------------------------------------------------------------

// While gathering features: m_p is relative to the feature hit point, m_contact_p.x holds sin^2 of the angle to
// the hit normal (2.0 for points at the hit point) and m_contact_p.y the squared distance. generate_convex_poly
// then stores the contact-plane coordinates in m_contact_p. process() also uses m_p.x as scratch (triangle area).
struct contact_manifold_mesh_point
{
	phys_vec3 m_p;
	phys_vec2 m_contact_p;
};

class phys_contact_manifold
{
public:
	// Public: host get_feature implementations read m_feature_hitn directly.
	phys_vec3 m_feature_normal;
	phys_vec3 m_feature_hitp;
	phys_vec3 m_feature_hitn;
	float m_feature_distance_eps;
	float m_sin_feautre_angular_eps_sq;
	int m_close_mesh_point_count;
	phys_memory_heap * m_allocator;
	contact_manifold_mesh_point * m_list_mesh_point;
	int m_list_mesh_point_count;
	contact_manifold_mesh_point ** m_list_sorted_mesh_point;
	contact_manifold_mesh_point ** m_list_contact_point;
	int m_list_contact_point_count;

	// Starts a new feature gather: mesh points are allocated contiguously after this.
	void set_get_feature_params(const phys_vec3 & hitp, const phys_vec3 & hitn, const float feature_distance_eps, const float sin_feautre_angular_eps_sq)
	{
		m_feature_hitp = hitp;
		m_feature_hitn = hitn;
		m_feature_distance_eps = feature_distance_eps;
		m_sin_feautre_angular_eps_sq = sin_feautre_angular_eps_sq;
		m_list_mesh_point = (contact_manifold_mesh_point *)m_allocator->fast_align_start(PHYS_ALIGNOF(contact_manifold_mesh_point), g_contact_manifold_error_msg);
		m_list_mesh_point_count = 0;
		m_close_mesh_point_count = 0;
		m_list_sorted_mesh_point = NULL;
		m_list_contact_point = NULL;
	}

	// Called by the geometry get_feature implementations for every candidate vertex.
	void add_feature_point(const phys_vec3 & p)
	{
		const phys_vec3 dist = p - m_feature_hitp;
		const float orth_dist = phys_dot(dist, m_feature_hitn);
		const float orth_dist_sq = orth_dist * orth_dist;
		const float dist_sq = AbsSquared(dist);
		if (m_feature_distance_eps >= orth_dist || dist_sq * m_sin_feautre_angular_eps_sq >= orth_dist_sq)
		{
			contact_manifold_mesh_point * const mp = new (m_allocator->fast_allocate(sizeof(contact_manifold_mesh_point), g_contact_manifold_error_msg)) contact_manifold_mesh_point;
			PHYS_ASSERT_STR((uintptr_t)(mp) % PHYS_ALIGNOF(contact_manifold_mesh_point) == 0,
				"(unsigned int)(mp) % PHYS_ALIGNOF(contact_manifold_mesh_point) == 0"); // uintptr_t for x64
			PHYS_ASSERT(mp == m_list_mesh_point + m_list_mesh_point_count);
			m_list_mesh_point_count++;
			mp->m_p = dist;
			mp->m_contact_p.y = dist_sq;
			// Written as !(a > b) so a NaN dist_sq takes the close-point branch ("<=" would not).
			if (!(dist_sq > phys_sqr(0.00339999981f))) // 0.0001 * 34 (unit scale)
			{
				mp->m_contact_p.x = 2.0f;
				m_close_mesh_point_count++;
			}
			else
			{
				mp->m_contact_p.x = 1.0f - orth_dist_sq / dist_sq;
			}
		}
	}

	// Moves the gathered (sorted) points into another frame; the feature normal is rotated only.
	void xform_and_translate_mesh_points(const phys_mat44 & xform, const phys_vec3 & translation)
	{
		for (contact_manifold_mesh_point **mp_i = m_list_sorted_mesh_point, **const last_mp_i = mp_i + m_list_mesh_point_count; mp_i != last_mp_i; ++mp_i)
			(*mp_i)->m_p = translation + phys_full_multiply(xform, (*mp_i)->m_p);
		m_feature_normal = phys_multiply(xform, m_feature_normal);
	}

	void comp_feature_normal();
	void generate_convex_poly(const phys_mat44 & contact_mat);
	void generate_convex_poly_internal();
	const float compute_convex_poly_area();
	const float compute_convex_poly_perimeter();

	static bool rht(const phys_vec2 & e1, const phys_vec2 & e2, const float min_length2, const float min_sin_sq);

	const phys_vec2 & get_poly_vert(const int i) const
	{
		PHYS_ASSERT(i >= 0 && i < m_list_contact_point_count);
		return m_list_contact_point[i]->m_contact_p;
	}

	const int get_poly_vert_count() const { return m_list_contact_point_count; }

	// Feature tolerances as a function of the penetration depth fraction (0 = touching, 1 = deep).
	// Evaluated in double precision (the SIN spans are folded in double).
	static const float get_STD_COMP_FEATURE_NORMAL_DISTANCE_EPS(const float penetration_t)
	{
		PHYS_ASSERT(penetration_t >= 0.0f && penetration_t <= 1.0f);
		return (float)((double)penetration_t * (double)1.7f + (double)1.7f);
	}

	static const float get_STD_COMP_FEATURE_NORMAL_SIN_ANGULAR_EPS_SQ(const float penetration_t)
	{
		PHYS_ASSERT(penetration_t >= 0.0f && penetration_t <= 1.0f);
		return (float)((double)penetration_t * 0.007589269621803396 + (double)6.853876e-06f);
	}

	static const float get_STD_GET_FEATURE_DISTANCE_EPS(const float penetration_t)
	{
		PHYS_ASSERT(penetration_t >= 0.0f && penetration_t <= 1.0f);
		return (float)((double)penetration_t * 5.1000001430511475 + (double)1.7f);
	}

	static const float get_STD_GET_FEATURE_SIN_ANGULAR_EPS_SQ(const float penetration_t)
	{
		PHYS_ASSERT(penetration_t >= 0.0f && penetration_t <= 1.0f);
		return (float)((double)penetration_t * 0.4698463100939989 + (double)0.03015369f);
	}
};

// ---------------------------------------------------------------------------------------------
// phys_contact_manifold_process: per-thread scratch that turns one GJK result into a contact_point_info.
// ---------------------------------------------------------------------------------------------

// Host hook that fills a new contact's friction / bounce / solver priority (e.g. from per-object data and
// surface types). When NULL, process() applies the surface-type pair table (surface_type_info_database_get)
// with max restitution 3400 and priority 0. create_cpi's -1 defaults are invalid.
typedef void (*phys_set_cpi_params_func)(contact_point_info * cpi, phys_collision_pair * pcp);
extern phys_set_cpi_params_func g_phys_set_cpi_params;
void phys_set_cpi_params_callback(phys_set_cpi_params_func func);

class TL_ALIGN(16) phys_contact_manifold_process
{
public:
	enum
	{
		ALLOCATOR_MEMORY_SIZE = 0x4000,
	};

	// Walks one convex polygon (rotating calipers).
	struct isect_info
	{
		phys_contact_manifold * m_cman;
		contact_manifold_mesh_point ** m_i;
		contact_manifold_mesh_point ** m_next_i;
		contact_manifold_mesh_point ** m_last_i;
		phys_vec2 m_edge;

		contact_manifold_mesh_point ** next_pv(contact_manifold_mesh_point ** i) { return i == m_last_i ? m_cman->m_list_contact_point : i + 1; }
		contact_manifold_mesh_point ** prev_pv(contact_manifold_mesh_point ** i) { return i == m_cman->m_list_contact_point ? m_last_i : i - 1; }

		void init(phys_contact_manifold * cman);
		void update();
	};

	struct bridge
	{
		phys_vec2 m_intersection_p;
		contact_manifold_mesh_point ** m_left_i;
		contact_manifold_mesh_point ** m_right_i;
	};

	phys_mat44 contact_mat; // rows: tangent x, tangent y, contact normal (cg1 frame)
	phys_mat44 cg1_to_rb2_xform;
	phys_transient_allocator * m_cpi_allocator;
	phys_link_list<contact_point_info> m_list_cpi;
	contact_point_info * m_cpi;
	contact_manifold_mesh_point ** m_list_isect_point;
	rigid_body_constraint_contact * m_rbc_contact_search_tree_root;
	phys_contact_manifold cman1;
	phys_contact_manifold cman2;
	int m_contact_point_count;
	phys_memory_heap m_allocator;
	char m_allocator_memory[ALLOCATOR_MEMORY_SIZE];

	phys_contact_manifold_process()
	{
		m_cpi_allocator = NULL;
		m_cpi = NULL;
		// Sentinel: the caller sets the real AVL root before process().
		m_rbc_contact_search_tree_root = (rigid_body_constraint_contact *)(intptr_t)-1; // intptr_t for x64
		m_allocator.set_buffer(m_allocator_memory, ALLOCATOR_MEMORY_SIZE, 1);
		cman1.m_allocator = &m_allocator;
		cman2.m_allocator = &m_allocator;
	}

	void comp_contact_mat(const phys_vec3 & contact_normal)
	{
		const phys_vec3 yrow = (fabsf(contact_normal.GetX()) < .8f) ? phys_vec3(1.0f, 0.0f, 0.0f) - contact_normal.GetX() * contact_normal
																	: phys_vec3(0.0f, 1.0f, 0.0f) - contact_normal.GetY() * contact_normal;
		const float nyrow = Abs(yrow);
		PHYS_ASSERT(nyrow > 0.1f);
		contact_mat.SetY(yrow / nyrow);
		contact_mat.SetX(phys_cross(contact_mat.GetY(), contact_normal));
		contact_mat.SetZ(contact_normal);
	}

	void process(phys_collision_pair * pcp, phys_gjk_info * gjk_info);

	void intersect_poly_segment(phys_contact_manifold * cman, const phys_vec2 & p0, const phys_vec2 & p1);
	void intersect_poly_poly();
	bool find_bottom(bridge * b, isect_info * left_cman, isect_info * right_cman);
};

void displace_contact_p(contact_manifold_mesh_point ** mp, const phys_vec2 & d, const phys_mat44 & contact_mat);
void feature_qsort(contact_manifold_mesh_point ** list, int list_count); // descending m_contact_p.x
void convex_poly_qsort(contact_manifold_mesh_point ** list, int list_count); // phys_v2_le order
