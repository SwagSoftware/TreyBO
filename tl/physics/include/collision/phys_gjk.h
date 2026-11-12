#pragma once

// GJK distance / intersection queries with continuous collision (ray-cast GJK) and per-geom simplex caching.

#include "tl/physics/include/phys_math.h"
#include "tl/physics/include/collision/phys_gjk_cache_system.h"

class phys_contact_manifold;

// Smallest usable support direction length.
const float GJK_MIN_SUPPORT_DIR_LENGTH = 0.0001f;

// Convergence tunables (writable).
extern float CONV_THRESH;
extern float SEP_CONV_THRESH;
extern int MAX_CC_RESET_ITERS;

// Abstract convex geometry for GJK.
// Every geometry is expressed in its own local frame; GJK passes local directions.
class phys_gjk_geom
{
public:
	// Supporting vertex in direction dir (local), plus a geom-defined feature "index" vector used by the simplex cache.
	virtual void support(const phys_vec3 & dir, phys_vec3 * vert, phys_vec3 * ind) const = 0;
	// Rebuilds the simplex vertices (and indices) from cached indices.
	virtual void get_simplex(const cached_simplex_info & cache_info, int count, phys_vec3 * verts, phys_vec3 * inds) const = 0;
	// Stores the indices of the vertices in w_set. normal is the contact normal in the geom's frame.
	virtual void set_simplex(const phys_vec3 * simplex_inds, int w_set, const phys_vec3 & normal, cached_simplex_info * cache_info) const;
	virtual const phys_vec3 get_center() const = 0;
	// Fills cman's mesh points near its feature params (set_get_feature_params), in the geom's frame.
	virtual void get_feature(phys_contact_manifold * cman) const = 0;
	virtual const float get_geom_radius() const; // default 0
	virtual void calc_aabb(const phys_mat44 & xform, phys_vec3 * aabb_min, phys_vec3 * aabb_max) const = 0;
	// Analytic ray cast for geoms that support it (default: false).
	virtual const bool ray_cast(const phys_vec3 & ray_start, const phys_vec3 & ray_dir, float t_input, float * t_output, phys_vec3 * hitn) const;
	virtual bool is_polyhedron() const = 0;

	// Support point for a direction v given in the frame xform maps this geom into, returned in that frame.
	const phys_vec3 support_only(const phys_mat44 & xform, const phys_vec3 & v) const;
	// No virtual dtor: geoms are never deleted through this base.
};

// Closest points / normal of a pair, in cg1's local frame.
class phys_gjk_collision_info
{
public:
	phys_vec3 m_p1; // on cg1
	phys_vec3 m_p2; // on cg2
	phys_vec3 m_n; // unit, from cg2 towards cg1
};

// Input of one GJK query.
class phys_gjk_input
{
public:
	phys_vec3 m_cg1_translation; // world-space sweep of cg1 over the step (continuous collision)
	phys_vec3 m_cg2_translation;
	float m_start_time; // sweep interval in [0, 1]
	float m_end_time;
	const phys_gjk_geom * gjk_cg1;
	const phys_gjk_geom * gjk_cg2;
	const phys_mat44 * cg1_to_world_xform;
	const phys_mat44 * cg2_to_world_xform;
	phys_gjk_cache_info * gjk_ci; // optional warm start
	float cg1_radius; // margin around cg1 (geom radius + contact skin)
	float cg2_radius;
	float m_sep_thresh; // report as colliding while closer than this (beyond the radii)
	bool m_intersection_test_only;
	bool m_continuous_collision;
};

class phys_gjk_info
{
public:
	struct phys_gjk_set_info
	{
		float m_lamda[4]; // barycentric weight of each simplex vertex
		int m_candidate;
	};

	enum gjk_retval_e
	{
		GJK_SEPARATED,
		GJK_VALID,
		GJK_PENETRATING,
		GJK_INVALID,
	};

	// m_flags
	enum gjk_flags_e
	{
		FLAG_EXIT_ON_SEP_THRESH = (1 << 0),
		FLAG_INTERSECTION_TEST_ONLY = (1 << 1),
		FLAG_CC_STOP_ADVANCING = (1 << 2), // conservative advancement finished: just refine the simplex
		FLAG_CONTINUOUS_COLLISION = (1 << 3),
		FLAG_CC_SEP_THRESH_EXIT = (1 << 4), // set FLAG_EXIT_ON_SEP_THRESH once the support dir is within m_gjk_sep_thresh
		FLAG_CC_SEPARATING_SUPPORT = (1 << 5), // a support point with positive v.w was seen
	};

	phys_mat44 cg2_to_cg1_xform;
	phys_vec3 m_cg1_relative_translation_loc; // cg1 sweep relative to cg2, in cg1's frame
	float m_continuous_collision_lambda; // time of impact found by gjk_ray_cast, in [0, 1]
	phys_gjk_collision_info cg1_cinfo_loc;
	phys_vec3 m_gjk_origin; // the point GJK measures distance to (moves with lambda / the separation loop)
	phys_vec3 m_w_verts[4];
	phys_vec3 m_a_verts[4];
	phys_vec3 m_b_verts[4]; // in cg1's frame
	phys_vec3 m_a_inds[4];
	phys_vec3 m_b_inds[4];
	phys_vec3 m_support_dir;
	float m_geom_radii_sum;
	int m_cc_reset_iter;
	int m_flags;
	int m_w_set;
	int m_last_w_set;
	int m_gjk_iter;
	float m_gjk_sep_thresh;
	float m_gjk_pen_thresh_sq;
	float m_upper_dist_sq;
	float m_lower_dist_sq;
	float m_dot_ij[4][4]; // unused
	phys_gjk_set_info m_set_list[16];

	const int compress_verts(int w_set);
	void comp_v(int w_set, phys_vec3 * v);
	void comp_closest_points(int w_set, phys_vec3 * a, phys_vec3 * b);

	const int gjk_subalgorithm(int w_set, int new_index);
	const int seed_simplex(int cached_vert_count);
	const int init_gjk(const phys_gjk_input * d, const phys_vec3 & initial_support_dir, bool in_separation_loop);
	gjk_retval_e gjk(const phys_gjk_input * d, const phys_vec3 & initial_support_dir, bool in_separation_loop);
	gjk_retval_e gjk_ray_cast(const phys_gjk_input * d, const phys_vec3 & initial_support_dir, bool in_separation_loop);
	gjk_retval_e collide(const phys_gjk_input * d);

	const phys_vec3 get_initial_support_dir(const phys_gjk_input * d);

	void gjk_cache_update_invalid(const phys_gjk_input * d);
	void gjk_cache_update_test_only_valid(const phys_gjk_input * d);
	void gjk_cache_update_colliding(const phys_gjk_input * d);
	void gjk_cache_update_test_only_penetrating(const phys_gjk_input * d);

	// Full query: true when the pair is closer than the separation threshold. Unless
	// m_intersection_test_only, cg1_cinfo_loc then holds the closest points (moved out by the radii) and the normal.
	const bool phys_collide_do_gjk_collide(const phys_gjk_input * d);
};

// Johnson's distance sub-algorithm: the smallest sub-simplex of m_w_set | (1 << new_index) closest
// to the origin, with its weights written to m_set_list[result]. Returns 15 when the tetrahedron contains the origin.
int backup1(phys_gjk_info * gjk_info, int new_index, bool seed_simplex);

// Simplex cache bridge.
void get_simplex(const phys_gjk_geom * cg1, const phys_gjk_geom * cg2, phys_gjk_cache_info * gjk_ci,
	phys_vec3 * a_verts, phys_vec3 * a_inds, phys_vec3 * b_verts, phys_vec3 * b_inds, int * vert_count);
void set_simplex(const phys_gjk_geom * cg1, const phys_gjk_geom * cg2, phys_gjk_cache_info * gjk_ci,
	const phys_vec3 & a_normal, const phys_vec3 & b_normal, const phys_vec3 * a_inds, const phys_vec3 * b_inds, int w_set);
