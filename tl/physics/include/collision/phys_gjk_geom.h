#pragma once

// TreyBO-new. Retail: game-side code, src\physics\phys_colgeom.h / phys_colgeom.cpp (TU 0x831070-0x837540; header
// inlines 0x439d30-0x441ec0, 0x5b1b40, 0x821b40, 0x8484c0; pool instantiations 0x8376a0-0x837e10; static data
// initialisers 0xc511d0 / 0xc511f0). These are the concrete GJK geometries. TreyBO moves them into the library and
// replaces the CoD collision-map types (cbrush_t, CollisionPartition, CollisionAabbTree, clipMap_t) and the game's
// gjk_collision_visitor with the neutral PODs and allocator below. Support, simplex, feature and AABB math is retail.

#include "tl/physics/include/phys_math.h"
#include "tl/physics/include/phys_mem.h"
#include "tl/physics/include/collision/phys_gjk.h"

class phys_contact_manifold;
struct gjk_base_t;

// ---------------------------------------------------------------------------------------------
// Geometry type ids returned by gjk_base_t::get_type().
// Retail: an unnamed enum (IDB "$2DE106F947F3758108E2812F8595AD24"), and get_type() returns const unsigned int.
// It stays unnamed (kisakblack's gjk_type_t is not a retail name).
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
// Allocator.
// TREYBO: replaces the game's gjk_collision_visitor (vtable 0xc6c8c8, 7 slots). create() uses only its first two
// slots, kept here in retail order with retail signatures. The query slots (get_local_query_aabb,
// query_create_prolog / _epilog / _prolog_1 / _epilog_1) stay in the engine adapter, whose visitor can derive from
// this class.
// ---------------------------------------------------------------------------------------------
class gjk_geom_allocator
{
public:
    // Slot 0. Scratch memory for a temporary (query) geom. Used only when is_query() is true. Such geoms carry
    // FLAG_TEMP_ALLOCATION, get no geom id and must never reach destroy(); the allocator owns the memory.
    // alignment == 0 (retail gjk_double_sphere_t::create) means the allocator's default, which must still be
    // at least 16 because the geoms hold phys_vec3 members.
    virtual void *allocate(const int size, const int alignment, const bool no_error) = 0;

    // Slot 1. Retail default 0x835320 returns false. When false, create() takes the geom from the persistent
    // phys_simple_allocator pools (PMM) and gives it a unique id from get_unique_id().
    virtual const bool is_query() { return false; }

    // TREYBO: retail called g_gjk_unique_id_database.get_unique_id() (0x831260, asserts in phys_broad_phase.h).
    // The id keys the GJK simplex cache (phys_heap_gjk_cache_system_avl_tree), so it must be unique among live
    // persistent geoms and never 0.
    virtual const phys_gjk_geom_id get_unique_id() = 0;
};

// ---------------------------------------------------------------------------------------------
// Neutral geometry descriptions filled by the host.
// ---------------------------------------------------------------------------------------------

// TREYBO: stands in for the cbrush_t fields that gjk_brush_t::create (0x8325c0) reads. Retail cbrush_t (0x60) is
// mins, contents, maxs, numsides, sides, axial_cflags/sflags, numverts, verts. Retail support, get_simplex,
// get_feature and calc_aabb read only the vertex array; the sides/planes were only read by the game's
// is_walkable (0x844bd0), which TreyBO drops. So no plane data is carried.
// The host keeps the desc and its vertex array alive for as long as the geom lives.
struct phys_brush_desc
{
    float mins[3];
    int contents;
    float maxs[3];
    unsigned int numsides;   // 0: create() builds a gjk_aabb_t over mins/maxs instead (retail)
    unsigned int numverts;
    const float (*verts)[3]; // convex hull vertices, geom-local
    const void *user_data;   // host brush (e.g. the cbrush_t). Retail get_brush() returned that brush.
};

// TREYBO: neutral copy of CoD's CollisionBorder. Pure geometry, same layout and member names (0x1C).
struct phys_collision_border
{
    float distEq[3];
    float zBase;
    float zSlope;
    float start;
    float length;
};

// TREYBO: neutral copy of CoD's CollisionPartition (same layout and member names, 0x14 on x86).
struct phys_collision_partition
{
    unsigned char triCount;
    unsigned char borderCount;
    int firstTri;
    int nuinds; // unique vertex index count of this partition
    int fuind;  // first unique vertex index (into phys_partition_world_desc::uinds)
    phys_collision_border *borders;
};

// TREYBO: neutral copy of CoD's CollisionAabbTree (same layout and member names, 0x20).
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

// TREYBO: the three clipMap_t arrays that gjk_partition_t::create (0x832df0) reads from the game global cm.
struct phys_partition_world_desc
{
    const float (*verts)[3];                    // cm.verts
    unsigned short *uinds;                      // cm.uinds
    const phys_collision_partition *partitions; // cm.partitions
};

// ---------------------------------------------------------------------------------------------
// gjk_base_t (U, 0x50 on x86). Vtable: the 9 phys_gjk_geom slots, then comp_aabb_loc, get_type and get_brush.
// TREYBO: retail also has slots 11/12 is_foot(const phys_vec3 &) / is_walkable(const phys_vec3 &, const phys_vec3 &)
// between get_type and get_brush. Those are player-movement tests (game) and are dropped; get_brush stays as the
// neutral user-data hook and returns the phys_brush_desc (desc->user_data is the host brush). Retail asserts are in
// src/physics/phys_colgeom.h.
// ---------------------------------------------------------------------------------------------
struct gjk_base_t : public phys_gjk_geom
{
    phys_vec3 m_aabb_mn_loc;
    phys_vec3 m_aabb_mx_loc;
    unsigned int m_gjk_geom_id;
    const phys_mat44 *m_xform_;
    gjk_base_t *m_next_geom;
    unsigned int m_flags;
    int stype;      // host surface type. Retail: CoD surface type, asserted < SURF_TYPECOUNT (31) in create().
    int m_contents; // host contents bits

    // m_flags (names from the retail assert strings).
    enum
    {
        FLAG_TEMP_ALLOCATION = 0x1,
        FLAG_AABB_LOC_VALID = 0x2,
        FLAG_GEOM_ID_VALID = 0x4,
        FLAG_XFORM_VALID = 0x8,
        FLAG_CONTENTS_VALID = 0x10,
    };

    gjk_base_t() { m_flags = 0; } // 0x4402e0

    // 0x440320. Not virtual (retail vtable has no dtor slot); runs from phys_simple_allocator::free.
    ~gjk_base_t() { PHYS_ASSERT(!get_flag(FLAG_TEMP_ALLOCATION)); }

    const unsigned int get_flag(const unsigned int flag) const { return m_flags & flag; }

    // 0x839700
    const unsigned int get_geom_id() const
    {
        PHYS_ASSERT(get_flag(FLAG_GEOM_ID_VALID));
        return m_gjk_geom_id;
    }

    // 0x8312d0
    void set_geom_id_new(const unsigned int geom_id)
    {
        m_flags |= FLAG_GEOM_ID_VALID;
        m_gjk_geom_id = geom_id;
    }

    // TREYBO FIX(0x43b1a0): no matrix-contents assert (kisakblack's assert_mat44_initialized is not retail).
    const phys_mat44 *get_xform() const
    {
        PHYS_ASSERT(get_flag(FLAG_XFORM_VALID));
        return m_xform_;
    }

    // 0x836f00
    void set_xform(const phys_mat44 *xform)
    {
        PHYS_ASSERT(xform);
        m_flags |= FLAG_XFORM_VALID;
        m_xform_ = xform;
    }

    // 0x8432b0
    const int get_contents() const
    {
        PHYS_ASSERT(get_flag(FLAG_CONTENTS_VALID));
        return m_contents;
    }

    // 0x832a00
    void set_contents(const int contents)
    {
        m_flags |= FLAG_CONTENTS_VALID;
        m_contents = contents;
    }

    // TREYBO: names not recovered (inlined; asserts phys_colgeom.h:82 / :83 in gjk_geom_list_t::comp_aabb_loc 0x821b40).
    const phys_vec3 &get_aabb_mn_loc() const
    {
        PHYS_ASSERT(get_flag(FLAG_AABB_LOC_VALID));
        return m_aabb_mn_loc;
    }

    const phys_vec3 &get_aabb_mx_loc() const
    {
        PHYS_ASSERT(get_flag(FLAG_AABB_LOC_VALID));
        return m_aabb_mx_loc;
    }

    virtual void comp_aabb_loc();                     // 0x440360: local AABB, computed once
    virtual const unsigned int get_type() const;      // folded with get_brush in retail: returns GJK_BASE (0)
    virtual const phys_brush_desc *get_brush() const; // 0x4403e0: NULL
};

// ---------------------------------------------------------------------------------------------
// Axis-aligned box (in the geom frame).
// ---------------------------------------------------------------------------------------------
struct gjk_aabb_t : public gjk_base_t
{
    phys_vec3 m_center_local;
    phys_vec3 m_dims; // half extents
    const phys_brush_desc *m_brush; // set when gjk_brush_t::create fell back to a box (numsides == 0)

    gjk_aabb_t() : m_brush(NULL) {} // inlined in 0x831070 / 0x8376a0

    static gjk_aabb_t *create(const phys_vec3 &center, const phys_vec3 &dims, int stype, gjk_geom_allocator *allocator); // 0x831070
    static void destroy(gjk_aabb_t *geom); // 0x831be0

    void support(const phys_vec3 &v, phys_vec3 *support_vert, phys_vec3 *support_ind) const override;                           // 0x831330
    void get_simplex(const cached_simplex_info &cache_info, int index_count, phys_vec3 *simplex_verts, phys_vec3 *simplex_inds) const override; // 0x8315b0
    const phys_vec3 get_center() const override;                                                                                // 0x831730
    void get_feature(phys_contact_manifold *cman) const override;                                                               // 0x831760
    void calc_aabb(const phys_mat44 &xform, phys_vec3 *aabb_min, phys_vec3 *aabb_max) const override;                          // 0x831b70
    bool is_polyhedron() const override;            // gjk_obb_t::is_polyhedron 0x832d90 (shared COMDAT): true
    const unsigned int get_type() const override;   // 0x831bd0 (COMDAT-folded): GJK_AABB
    const phys_brush_desc *get_brush() const override; // 0x831bb0: m_brush
};

// ---------------------------------------------------------------------------------------------
// Oriented box: m_xform maps the box frame into the geom frame.
// ---------------------------------------------------------------------------------------------
struct gjk_obb_t : public gjk_base_t
{
    phys_vec3 m_dims; // half extents
    phys_mat44 m_xform;

    static gjk_obb_t *create(const phys_mat44 &xform, const phys_vec3 &dims, int stype, gjk_geom_allocator *allocator); // 0x831c30
    static void destroy(gjk_obb_t *geom); // 0x832570

    void support(const phys_vec3 &v, phys_vec3 *support_vert, phys_vec3 *support_ind) const override;                           // 0x831e00
    void get_simplex(const cached_simplex_info &cache_info, int index_count, phys_vec3 *simplex_verts, phys_vec3 *simplex_inds) const override; // 0x832050
    const phys_vec3 get_center() const override;                                                                                // 0x832190
    void get_feature(phys_contact_manifold *cman) const override;                                                               // 0x8321d0
    void calc_aabb(const phys_mat44 &xform, phys_vec3 *aabb_min, phys_vec3 *aabb_max) const override;                          // 0x8322b0
    bool is_polyhedron() const override;          // 0x832d90: true
    const unsigned int get_type() const override; // 0x832560: GJK_OBB
};

// ---------------------------------------------------------------------------------------------
// Convex polyhedron given by its vertices (a CoD brush in retail).
// Simplex index: the vertex number, stored as int bits in ind.x.
// ---------------------------------------------------------------------------------------------
struct gjk_brush_t : public gjk_base_t
{
    int nverts;
    const float (*verts)[3];
    const phys_brush_desc *brush; // TREYBO: retail const cbrush_t *

    // 0x8325c0. Returns a gjk_aabb_t over mins/maxs when brush->numsides == 0, as retail.
    static gjk_base_t *create(const phys_brush_desc *brush, int stype, gjk_geom_allocator *allocator);
    static void destroy(gjk_brush_t *geom); // 0x832da0

    void support(const phys_vec3 &v, phys_vec3 *support_vert, phys_vec3 *support_ind) const override;                           // 0x832a60
    void get_simplex(const cached_simplex_info &cache_info, int index_count, phys_vec3 *simplex_verts, phys_vec3 *simplex_inds) const override; // 0x832b60
    const phys_vec3 get_center() const override;                                                                                // 0x8331e0: origin
    void get_feature(phys_contact_manifold *cman) const override;                                                               // 0x832c00
    void calc_aabb(const phys_mat44 &xform, phys_vec3 *aabb_min, phys_vec3 *aabb_max) const override;                          // 0x832c70
    bool is_polyhedron() const override;               // 0x832d90: true
    const unsigned int get_type() const override;      // 0x832d80: GJK_BRUSH
    const phys_brush_desc *get_brush() const override; // 0x832d60: brush
};

// ---------------------------------------------------------------------------------------------
// One world-geometry partition: the unique vertices of a triangle soup cell (convex by construction in the CoD
// collision map). Simplex index: the position in inds[], stored as int bits in ind.x.
// ---------------------------------------------------------------------------------------------
struct gjk_partition_t : public gjk_base_t
{
    int ninds;
    const float (*verts)[3];
    unsigned short *inds;
    const phys_collision_partition *partition; // TREYBO: retail const CollisionPartition *
    const phys_collision_aabb_tree *tree;      // TREYBO: retail const CollisionAabbTree *

    // 0x832df0. TREYBO: retail read cm.partitions / cm.uinds / cm.verts and took stype / contents from
    // cm.materials[tree->materialIndex] ((surfaceFlags & 0x3F00000) >> 20, contentFlags). The host passes
    // the arrays in world and the material values directly.
    static gjk_partition_t *create(const phys_collision_aabb_tree *tree, const phys_partition_world_desc &world, int stype, int contents, gjk_geom_allocator *allocator);
    static void destroy(gjk_partition_t *geom); // 0x8333c0

    void support(const phys_vec3 &v, phys_vec3 *support_vert, phys_vec3 *support_ind) const override;                           // 0x833020
    void get_simplex(const cached_simplex_info &cache_info, int index_count, phys_vec3 *simplex_verts, phys_vec3 *simplex_inds) const override; // 0x833140
    const phys_vec3 get_center() const override;                                                                                // gjk_brush_t::get_center 0x8331e0
    void get_feature(phys_contact_manifold *cman) const override;                                                               // 0x833220
    void calc_aabb(const phys_mat44 &xform, phys_vec3 *aabb_min, phys_vec3 *aabb_max) const override;                          // 0x8332a0
    bool is_polyhedron() const override;          // 0x832d90: true
    const unsigned int get_type() const override; // 0x8333b0: GJK_PARTITION
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

    static gjk_double_sphere_t *create(const phys_vec3 &c0, const phys_vec3 &c1, float r, int stype, gjk_geom_allocator *allocator); // 0x833f00
    static void destroy(gjk_double_sphere_t *geom); // 0x834240

    void support(const phys_vec3 &v, phys_vec3 *support_vert, phys_vec3 *support_ind) const override;                           // 0x833470
    void get_simplex(const cached_simplex_info &cache_info, int index_count, phys_vec3 *simplex_verts, phys_vec3 *simplex_inds) const override; // 0x833760
    void set_simplex(const phys_vec3 *simplex_inds, int w_set, const phys_vec3 &normal, cached_simplex_info *cache_info) const override; // 0x833900
    const phys_vec3 get_center() const override;                                                                                // 0x833990
    void get_feature(phys_contact_manifold *cman) const override;                                                               // 0x8339c0
    const float get_geom_radius() const override;                                                                               // 0x833e90
    void calc_aabb(const phys_mat44 &xform, phys_vec3 *aabb_min, phys_vec3 *aabb_max) const override;                          // 0x833b00
    bool is_polyhedron() const override;          // 0x833eb0
    const unsigned int get_type() const override; // 0x97eb60 (COMDAT-folded): GJK_DOUBLE_SPHERE
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

    static gjk_cylinder_t *create(int _direction, float _halfHeight, float _radius, const phys_mat44 &_xform, int stype, gjk_geom_allocator *allocator); // 0x834290
    static void destroy(gjk_cylinder_t *geom); // 0x835350

    // 0x834700. Half extents in the cylinder frame.
    const phys_vec3 get_dims() const
    {
        phys_vec3 dims(radius);
        dims[direction] = halfHeight;
        return dims;
    }

    void support(const phys_vec3 &v, phys_vec3 *support_vert, phys_vec3 *support_ind) const override;                           // 0x834470
    void get_simplex(const cached_simplex_info &cache_info, int index_count, phys_vec3 *simplex_verts, phys_vec3 *simplex_inds) const override; // 0x8347d0
    const phys_vec3 get_center() const override;                                                                                // 0x834910
    void get_feature(phys_contact_manifold *cman) const override;                                                               // 0x834980
    const float get_geom_radius() const override;                                                                               // 0x835330
    void calc_aabb(const phys_mat44 &xform_, phys_vec3 *aabb_min, phys_vec3 *aabb_max) const override;                         // 0x835160
    bool is_polyhedron() const override;          // 0x835320 (COMDAT-folded with gjk_collision_visitor::is_query): false
    const unsigned int get_type() const override; // 0x835310: GJK_CYLINDER
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

        poly_verts(); // 0x837ed0

        // TREYBO: no out-of-line copy; the name comes from the retail assert text at phys_colgeom.h:904.
        const int get_num_verts() const { return NUM_VERTS_; }

        void get_co_si(const int i, float *co_, float *si_);  // 0x440410
        const int support(const phys_vec3 &v);                 // 0x440ab0: ring vertex closest to v's azimuth
    };

    static poly_verts s_poly_verts; // dynamic initializer 0xc511d0

    static gjk_polygon_cylinder_t *create(const float (&mins)[3], const float (&maxs)[3], float radius_adjust, int stype, gjk_geom_allocator *allocator); // 0x835b70
    static void destroy(gjk_polygon_cylinder_t *geom); // 0x835d00

    // 0x441c80. AABB of a disc of the given radius around axis.
    static void calc_disc_aabb(const phys_vec3 &axis, const float radius, phys_vec3 *aabb_min, phys_vec3 *aabb_max);

    void support(const phys_vec3 &v, phys_vec3 *support_vert, phys_vec3 *support_ind) const override;                           // 0x4404c0
    void get_simplex(const cached_simplex_info &cache_info, int index_count, phys_vec3 *simplex_verts, phys_vec3 *simplex_inds) const override; // 0x440ba0
    const phys_vec3 get_center() const override;                                                                                // gjk_aabb_t::get_center 0x831730
    void get_feature(phys_contact_manifold *cman) const override;                                                               // 0x444c00 (COMDAT-folded empty function)
    const float get_geom_radius() const override;                                                                               // 0x441ec0
    void calc_aabb(const phys_mat44 &xform, phys_vec3 *aabb_min, phys_vec3 *aabb_max) const override;                          // 0x4413f0
    bool is_polyhedron() const override;          // 0x832d90: true
    const unsigned int get_type() const override; // 0x441eb0: GJK_POLYGON_CYLINDER
};

// TREYBO: retail added the dvar phys_player_collision_adjust_height to m_center.z in setup_gjk_polygon_cylinder.
// The host copies its value here (default 0).
extern float phys_polygon_cylinder_adjust_height;

// 0x8353a0. Fits the polygon cylinder to a character box, shrunk by (radius_adjust - 0.125) on every side.
void setup_gjk_polygon_cylinder(const float *const mins, const float *const maxs, float radius_adjust, gjk_polygon_cylinder_t *gjk_cylinder);

// ---------------------------------------------------------------------------------------------
// gjk_geom_list_t (U, 0x8): singly linked list of the geoms of one body, through m_next_geom.
// ---------------------------------------------------------------------------------------------
struct gjk_geom_list_t
{
    gjk_base_t *m_first_geom;
    int m_geom_count;

    // 0x5b1b40. New geoms go to the front.
    void add_geom(gjk_base_t *geom)
    {
        PHYS_ASSERT(geom);
        geom->m_next_geom = m_first_geom;
        m_first_geom = geom;
        ++m_geom_count;
    }

    // 0x8484c0
    int get_geom_count() const
    {
        PHYS_ASSERT(m_geom_count >= 0);
        return m_geom_count;
    }

    // 0x821b40. Union of the geoms' local AABBs (computing each on first use).
    void comp_aabb_loc(phys_vec3 *aabb_mn_loc, phys_vec3 *aabb_mx_loc)
    {
        PHYS_ASSERT(m_first_geom);
        m_first_geom->comp_aabb_loc();
        *aabb_mn_loc = m_first_geom->get_aabb_mn_loc();
        *aabb_mx_loc = m_first_geom->get_aabb_mx_loc();
        for (gjk_base_t *geom = m_first_geom->m_next_geom; geom; geom = geom->m_next_geom)
        {
            geom->comp_aabb_loc();
            *aabb_mn_loc = phys_min(*aabb_mn_loc, geom->get_aabb_mn_loc());
            *aabb_mx_loc = phys_max(*aabb_mx_loc, geom->get_aabb_mx_loc());
        }
    }
};

// ---------------------------------------------------------------------------------------------
// Builders and teardown (retail phys_colgeom.cpp free functions). The float[3] / float[3][3] arguments follow the
// CoD vector conventions (Phys_Vec3ToNitrousVec / Phys_AxisToNitrousMat).
// ---------------------------------------------------------------------------------------------

void destroy_gjk_geom(gjk_base_t *geom);              // 0x835d50: dispatch on get_type()
void destroy_gjk_geom(gjk_geom_list_t *gjk_geom_list); // 0x84a340: destroys every geom and empties the list

gjk_base_t *create_aabb_gjk_geom(const float *const mn, const float *const mx, int stype, gjk_geom_allocator *allocator); // 0x835e30
gjk_base_t *create_obb_gjk_geom(const float (*const orientation)[3], const float *const offset, const float *const halfLengths, int stype, gjk_geom_allocator *allocator); // 0x835f80
gjk_base_t *create_brush_gjk_geom(const phys_brush_desc *brush, int stype, gjk_geom_allocator *allocator); // 0x835ff0 (TREYBO: takes the neutral desc)
gjk_base_t *create_capsule_gjk_geom(const float *const center, float radius, float halfHeight, int direction, int stype, gjk_geom_allocator *allocator); // 0x836020
gjk_base_t *create_cylinder_gjk_geom(const float (*const rot)[3], const float *const trans, float radius, float halfHeight, int stype, gjk_geom_allocator *allocator); // 0x836140
