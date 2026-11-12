#pragma once

// The concrete GJK geometries, built from host-filled geometry descriptions through a host allocator.

#include "tl/physics/include/phys_math.h"
#include "tl/physics/include/phys_mem.h"
#include "tl/physics/include/collision/phys_gjk.h"

class phys_contact_manifold;
struct gjk_base_t;

// ---------------------------------------------------------------------------------------------
// Geometry type ids returned by gjk_base_t::get_type().
// ---------------------------------------------------------------------------------------------
enum
{
	GJK_BASE = 0x0,
	GJK_AABB = 0x1,
	GJK_BRUSH = 0x2,
	GJK_PARTITION = 0x3,
	GJK_DOUBLE_SPHERE = 0x4,
	GJK_CYLINDER = 0x5,
	GJK_OBB = 0x6,
	GJK_POLYGON_CYLINDER = 0x7,
};

// ---------------------------------------------------------------------------------------------
// Allocator, implemented by the host. The host's collision-query visitor can derive from this class.
// ---------------------------------------------------------------------------------------------
class gjk_geom_allocator
{
public:
	// Scratch memory for a temporary (query) geom. Used only when is_query() is true. Such geoms carry
	// FLAG_TEMP_ALLOCATION, get no geom id and must never reach destroy(); the allocator owns the memory.
	// alignment == 0 (gjk_double_sphere_t::create) means the allocator's default, which must still be
	// at least 16 because the geoms hold phys_vec3 members.
	virtual void * allocate(const int size, const int alignment, const bool no_error) = 0;

	// When false, create() takes the geom from the persistent
	// phys_simple_allocator pools (PMM) and gives it a unique id from get_unique_id().
	virtual const bool is_query() { return false; }

	// E.g. g_gjk_unique_id_database.get_unique_id(). The id keys the GJK simplex cache (phys_heap_gjk_cache_system_avl_tree), so it must be unique among live
	// persistent geoms and never 0.
	virtual const phys_gjk_geom_id get_unique_id() = 0;
};

// ---------------------------------------------------------------------------------------------
// Neutral geometry descriptions filled by the host.
// ---------------------------------------------------------------------------------------------

// A convex brush. Support, simplex, feature and AABB queries read only the vertex array, so no plane data is
// carried. The host keeps the desc and its vertex array alive for as long as the geom lives.
struct phys_brush_desc
{
	float mins[3];
	int contents;
	float maxs[3];
	unsigned int numsides; // 0: create() builds a gjk_aabb_t over mins/maxs instead
	unsigned int numverts;
	const float (*verts)[3]; // convex hull vertices, geom-local
	const void * user_data; // host brush
};

// Collision-map partition border (0x1C).
struct phys_collision_border
{
	float distEq[3];
	float zBase;
	float zSlope;
	float start;
	float length;
};

// Collision-map partition (0x14 on x86).
struct phys_collision_partition
{
	unsigned char triCount;
	unsigned char borderCount;
	int firstTri;
	int nuinds; // unique vertex index count of this partition
	int fuind; // first unique vertex index (into phys_partition_world_desc::uinds)
	phys_collision_border * borders;
};

// Collision-map AABB tree node (0x20).
struct phys_collision_aabb_tree
{
	float origin[3];
	unsigned short materialIndex;
	unsigned short childCount;
	float halfSize[3];
	union
	{
		int firstChildIndex;
		int partitionIndex;
	} u;
};

// The collision-map arrays gjk_partition_t::create reads.
struct phys_partition_world_desc
{
	const float (*verts)[3];
	unsigned short * uinds;
	const phys_collision_partition * partitions;
};

// ---------------------------------------------------------------------------------------------
// gjk_base_t (0x50 on x86). Vtable: the phys_gjk_geom slots, then comp_aabb_loc, get_type and get_brush.
// get_brush is the user-data hook: it returns the phys_brush_desc (desc->user_data is the host brush).
// ---------------------------------------------------------------------------------------------
struct gjk_base_t : public phys_gjk_geom
{
	phys_vec3 m_aabb_mn_loc;
	phys_vec3 m_aabb_mx_loc;
	unsigned int m_gjk_geom_id;
	const phys_mat44 * m_xform_;
	gjk_base_t * m_next_geom;
	unsigned int m_flags;
	int stype; // host surface type
	int m_contents; // host contents bits

	// m_flags
	enum
	{
		FLAG_TEMP_ALLOCATION = 0x1,
		FLAG_AABB_LOC_VALID = 0x2,
		FLAG_GEOM_ID_VALID = 0x4,
		FLAG_XFORM_VALID = 0x8,
		FLAG_CONTENTS_VALID = 0x10,
	};

	gjk_base_t() { m_flags = 0; }

	// Not virtual; runs from phys_simple_allocator::free.
	~gjk_base_t() { PHYS_ASSERT(!get_flag(FLAG_TEMP_ALLOCATION)); }

	const unsigned int get_flag(const unsigned int flag) const { return m_flags & flag; }

	const unsigned int get_geom_id() const
	{
		PHYS_ASSERT(get_flag(FLAG_GEOM_ID_VALID));
		return m_gjk_geom_id;
	}

	void set_geom_id_new(const unsigned int geom_id)
	{
		m_flags |= FLAG_GEOM_ID_VALID;
		m_gjk_geom_id = geom_id;
	}

	const phys_mat44 * get_xform() const
	{
		PHYS_ASSERT(get_flag(FLAG_XFORM_VALID));
		return m_xform_;
	}

	void set_xform(const phys_mat44 * xform)
	{
		PHYS_ASSERT(xform);
		m_flags |= FLAG_XFORM_VALID;
		m_xform_ = xform;
	}

	const int get_contents() const
	{
		PHYS_ASSERT(get_flag(FLAG_CONTENTS_VALID));
		return m_contents;
	}

	void set_contents(const int contents)
	{
		m_flags |= FLAG_CONTENTS_VALID;
		m_contents = contents;
	}

	const phys_vec3 & get_aabb_mn_loc() const
	{
		PHYS_ASSERT(get_flag(FLAG_AABB_LOC_VALID));
		return m_aabb_mn_loc;
	}

	const phys_vec3 & get_aabb_mx_loc() const
	{
		PHYS_ASSERT(get_flag(FLAG_AABB_LOC_VALID));
		return m_aabb_mx_loc;
	}

	virtual void comp_aabb_loc(); // local AABB, computed once
	virtual const unsigned int get_type() const; // GJK_BASE
	virtual const phys_brush_desc * get_brush() const; // NULL
};

// ---------------------------------------------------------------------------------------------
// Axis-aligned box (in the geom frame).
// ---------------------------------------------------------------------------------------------
struct gjk_aabb_t : public gjk_base_t
{
	phys_vec3 m_center_local;
	phys_vec3 m_dims; // half extents
	const phys_brush_desc * m_brush; // set when gjk_brush_t::create fell back to a box (numsides == 0)

	gjk_aabb_t() :
		m_brush(NULL) {}

	static gjk_aabb_t * create(const phys_vec3 & center, const phys_vec3 & dims, int stype, gjk_geom_allocator * allocator);
	static void destroy(gjk_aabb_t * geom);

	void support(const phys_vec3 & v, phys_vec3 * support_vert, phys_vec3 * support_ind) const override;
	void get_simplex(const cached_simplex_info & cache_info, int index_count, phys_vec3 * simplex_verts, phys_vec3 * simplex_inds) const override;
	const phys_vec3 get_center() const override;
	void get_feature(phys_contact_manifold * cman) const override;
	void calc_aabb(const phys_mat44 & xform, phys_vec3 * aabb_min, phys_vec3 * aabb_max) const override;
	bool is_polyhedron() const override; // true
	const unsigned int get_type() const override; // GJK_AABB
	const phys_brush_desc * get_brush() const override; // m_brush
};

// ---------------------------------------------------------------------------------------------
// Oriented box: m_xform maps the box frame into the geom frame.
// ---------------------------------------------------------------------------------------------
struct gjk_obb_t : public gjk_base_t
{
	phys_vec3 m_dims; // half extents
	phys_mat44 m_xform;

	static gjk_obb_t * create(const phys_mat44 & xform, const phys_vec3 & dims, int stype, gjk_geom_allocator * allocator);
	static void destroy(gjk_obb_t * geom);

	void support(const phys_vec3 & v, phys_vec3 * support_vert, phys_vec3 * support_ind) const override;
	void get_simplex(const cached_simplex_info & cache_info, int index_count, phys_vec3 * simplex_verts, phys_vec3 * simplex_inds) const override;
	const phys_vec3 get_center() const override;
	void get_feature(phys_contact_manifold * cman) const override;
	void calc_aabb(const phys_mat44 & xform, phys_vec3 * aabb_min, phys_vec3 * aabb_max) const override;
	bool is_polyhedron() const override; // true
	const unsigned int get_type() const override; // GJK_OBB
};

// ---------------------------------------------------------------------------------------------
// Convex polyhedron given by its vertices (a brush).
// Simplex index: the vertex number, stored as int bits in ind.x.
// ---------------------------------------------------------------------------------------------
struct gjk_brush_t : public gjk_base_t
{
	int nverts;
	const float (*verts)[3];
	const phys_brush_desc * brush;

	// Returns a gjk_aabb_t over mins/maxs when brush->numsides == 0.
	static gjk_base_t * create(const phys_brush_desc * brush, int stype, gjk_geom_allocator * allocator);
	static void destroy(gjk_brush_t * geom);

	void support(const phys_vec3 & v, phys_vec3 * support_vert, phys_vec3 * support_ind) const override;
	void get_simplex(const cached_simplex_info & cache_info, int index_count, phys_vec3 * simplex_verts, phys_vec3 * simplex_inds) const override;
	const phys_vec3 get_center() const override; // origin
	void get_feature(phys_contact_manifold * cman) const override;
	void calc_aabb(const phys_mat44 & xform, phys_vec3 * aabb_min, phys_vec3 * aabb_max) const override;
	bool is_polyhedron() const override; // true
	const unsigned int get_type() const override; // GJK_BRUSH
	const phys_brush_desc * get_brush() const override; // brush
};

// ---------------------------------------------------------------------------------------------
// One world-geometry partition: the unique vertices of a triangle soup cell (convex by construction in the
// collision map). Simplex index: the position in inds[], stored as int bits in ind.x.
// ---------------------------------------------------------------------------------------------
struct gjk_partition_t : public gjk_base_t
{
	int ninds;
	const float (*verts)[3];
	unsigned short * inds;
	const phys_collision_partition * partition;
	const phys_collision_aabb_tree * tree;

	// The host passes the collision-map arrays in world, and the surface type / contents of the tree's material.
	static gjk_partition_t * create(const phys_collision_aabb_tree * tree, const phys_partition_world_desc & world, int stype, int contents, gjk_geom_allocator * allocator);
	static void destroy(gjk_partition_t * geom);

	void support(const phys_vec3 & v, phys_vec3 * support_vert, phys_vec3 * support_ind) const override;
	void get_simplex(const cached_simplex_info & cache_info, int index_count, phys_vec3 * simplex_verts, phys_vec3 * simplex_inds) const override;
	const phys_vec3 get_center() const override; // origin
	void get_feature(phys_contact_manifold * cman) const override;
	void calc_aabb(const phys_mat44 & xform, phys_vec3 * aabb_min, phys_vec3 * aabb_max) const override;
	bool is_polyhedron() const override; // true
	const unsigned int get_type() const override; // GJK_PARTITION
};

// ---------------------------------------------------------------------------------------------
// Two spheres swept together (a capsule when both radii are 0 and m_geom_radius is the capsule radius, which is
// what create() builds). Simplex index: the sphere number, as int bits in ind.x; the cached simplex keeps the
// contact normal in m_indices[0] and the packed sphere numbers from &m_indices[1].x on.
// ---------------------------------------------------------------------------------------------
struct gjk_double_sphere_t : public gjk_base_t
{
	phys_vec3 m_list_center[2];
	phys_vec3 m_center;
	float m_list_radius[2];
	float m_geom_radius;
	int m_count;

	static gjk_double_sphere_t * create(const phys_vec3 & c0, const phys_vec3 & c1, float r, int stype, gjk_geom_allocator * allocator);
	static void destroy(gjk_double_sphere_t * geom);

	void support(const phys_vec3 & v, phys_vec3 * support_vert, phys_vec3 * support_ind) const override;
	void get_simplex(const cached_simplex_info & cache_info, int index_count, phys_vec3 * simplex_verts, phys_vec3 * simplex_inds) const override;
	void set_simplex(const phys_vec3 * simplex_inds, int w_set, const phys_vec3 & normal, cached_simplex_info * cache_info) const override;
	const phys_vec3 get_center() const override;
	void get_feature(phys_contact_manifold * cman) const override;
	const float get_geom_radius() const override;
	void calc_aabb(const phys_mat44 & xform, phys_vec3 * aabb_min, phys_vec3 * aabb_max) const override;
	bool is_polyhedron() const override;
	const unsigned int get_type() const override; // GJK_DOUBLE_SPHERE
};

// ---------------------------------------------------------------------------------------------
// Cylinder along local axis `direction` (0/1/2) of xform. Simplex index: the unit support direction in the
// cylinder frame (axial component +-1), stored as floats.
// ---------------------------------------------------------------------------------------------
struct gjk_cylinder_t : public gjk_base_t
{
	int direction;
	float halfHeight;
	float radius;
	float m_geom_radius; // always 0 (get_feature asserts it)
	phys_mat44 xform;

	static gjk_cylinder_t * create(int _direction, float _halfHeight, float _radius, const phys_mat44 & _xform, int stype, gjk_geom_allocator * allocator);
	static void destroy(gjk_cylinder_t * geom);

	// Half extents in the cylinder frame.
	const phys_vec3 get_dims() const
	{
		phys_vec3 dims(radius);
		dims[direction] = halfHeight;
		return dims;
	}

	void support(const phys_vec3 & v, phys_vec3 * support_vert, phys_vec3 * support_ind) const override;
	void get_simplex(const cached_simplex_info & cache_info, int index_count, phys_vec3 * simplex_verts, phys_vec3 * simplex_inds) const override;
	const phys_vec3 get_center() const override;
	void get_feature(phys_contact_manifold * cman) const override;
	const float get_geom_radius() const override;
	void calc_aabb(const phys_mat44 & xform_, phys_vec3 * aabb_min, phys_vec3 * aabb_max) const override;
	bool is_polyhedron() const override; // false
	const unsigned int get_type() const override; // GJK_CYLINDER
};

// ---------------------------------------------------------------------------------------------
// Character volume around a vertical (z) axis. m_mode == 0: a 12-sided prism capped by two apex points (head
// and foot). m_mode != 0: a capsule (segment of 2 * (m_half_height - m_capsule_radius), radius added through
// get_geom_radius). Simplex index (int bits in ind.x): 0..11 upper ring, 12..23 lower ring, 24 head apex,
// 25 foot apex; in capsule mode 0 = top, 1 = bottom.
// ---------------------------------------------------------------------------------------------
struct gjk_polygon_cylinder_t : public gjk_base_t
{
	phys_vec3 m_center;
	float m_polygon_cylinder_radius;
	float m_capsule_radius;
	float m_half_height;
	float m_geom_radius;
	float m_head_offset;
	float m_foot_offset;
	int m_mode;

	// Unit ring of the prism: the first quadrant's cos/sin, mirrored into the other three.
	struct poly_verts
	{
		enum
		{
			NUM_VERTS_ = 12,
		};

		float m_co[4];
		float m_si[4];

		poly_verts();

		const int get_num_verts() const { return NUM_VERTS_; }

		void get_co_si(const int i, float * co_, float * si_);
		const int support(const phys_vec3 & v); // ring vertex closest to v's azimuth
	};

	static poly_verts s_poly_verts;

	static gjk_polygon_cylinder_t * create(const float (&mins)[3], const float (&maxs)[3], float radius_adjust, int stype, gjk_geom_allocator * allocator);
	static void destroy(gjk_polygon_cylinder_t * geom);

	// AABB of a disc of the given radius around axis.
	static void calc_disc_aabb(const phys_vec3 & axis, const float radius, phys_vec3 * aabb_min, phys_vec3 * aabb_max);

	void support(const phys_vec3 & v, phys_vec3 * support_vert, phys_vec3 * support_ind) const override;
	void get_simplex(const cached_simplex_info & cache_info, int index_count, phys_vec3 * simplex_verts, phys_vec3 * simplex_inds) const override;
	const phys_vec3 get_center() const override;
	void get_feature(phys_contact_manifold * cman) const override; // empty
	const float get_geom_radius() const override;
	void calc_aabb(const phys_mat44 & xform, phys_vec3 * aabb_min, phys_vec3 * aabb_max) const override;
	bool is_polyhedron() const override; // true
	const unsigned int get_type() const override; // GJK_POLYGON_CYLINDER
};

// Added to m_center.z by setup_gjk_polygon_cylinder. Set by the host (default 0).
extern float phys_polygon_cylinder_adjust_height;

// Fits the polygon cylinder to a character box, shrunk by (radius_adjust - 0.125) on every side.
void setup_gjk_polygon_cylinder(const float * const mins, const float * const maxs, float radius_adjust, gjk_polygon_cylinder_t * gjk_cylinder);

// ---------------------------------------------------------------------------------------------
// gjk_geom_list_t: singly linked list of the geoms of one body, through m_next_geom.
// ---------------------------------------------------------------------------------------------
struct gjk_geom_list_t
{
	gjk_base_t * m_first_geom;
	int m_geom_count;

	// New geoms go to the front.
	void add_geom(gjk_base_t * geom)
	{
		PHYS_ASSERT(geom);
		geom->m_next_geom = m_first_geom;
		m_first_geom = geom;
		++m_geom_count;
	}

	int get_geom_count() const
	{
		PHYS_ASSERT(m_geom_count >= 0);
		return m_geom_count;
	}

	// Union of the geoms' local AABBs (computing each on first use).
	void comp_aabb_loc(phys_vec3 * aabb_mn_loc, phys_vec3 * aabb_mx_loc)
	{
		PHYS_ASSERT(m_first_geom);
		m_first_geom->comp_aabb_loc();
		*aabb_mn_loc = m_first_geom->get_aabb_mn_loc();
		*aabb_mx_loc = m_first_geom->get_aabb_mx_loc();
		for (gjk_base_t * geom = m_first_geom->m_next_geom; geom; geom = geom->m_next_geom)
		{
			geom->comp_aabb_loc();
			*aabb_mn_loc = phys_min(*aabb_mn_loc, geom->get_aabb_mn_loc());
			*aabb_mx_loc = phys_max(*aabb_mx_loc, geom->get_aabb_mx_loc());
		}
	}
};

// ---------------------------------------------------------------------------------------------
// Builders and teardown. The float[3] / float[3][3] arguments follow the host's vector / axis conventions.
// ---------------------------------------------------------------------------------------------

void destroy_gjk_geom(gjk_base_t * geom); // dispatch on get_type()
void destroy_gjk_geom(gjk_geom_list_t * gjk_geom_list); // destroys every geom and empties the list

gjk_base_t * create_aabb_gjk_geom(const float * const mn, const float * const mx, int stype, gjk_geom_allocator * allocator);
gjk_base_t * create_obb_gjk_geom(const float (*const orientation)[3], const float * const offset, const float * const halfLengths, int stype, gjk_geom_allocator * allocator);
gjk_base_t * create_brush_gjk_geom(const phys_brush_desc * brush, int stype, gjk_geom_allocator * allocator);
gjk_base_t * create_capsule_gjk_geom(const float * const center, float radius, float halfHeight, int direction, int stype, gjk_geom_allocator * allocator);
gjk_base_t * create_cylinder_gjk_geom(const float (*const rot)[3], const float * const trans, float radius, float halfHeight, int stype, gjk_geom_allocator * allocator);
