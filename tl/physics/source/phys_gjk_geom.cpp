// Concrete GJK geometries; float operation order below is deliberate.

#include "tl/physics/include/collision/phys_gjk_geom.h"
#include "tl/physics/include/collision/phys_bounding_volume.h"
#include "tl/physics/include/collision/phys_contact_manifold.h"
#include "tl/base/include/tl_system.h"

#include <new>

// Persistent geom pools.
phys_simple_allocator<gjk_aabb_t> aabb_pool;
phys_simple_allocator<gjk_obb_t> obb_pool;
phys_simple_allocator<gjk_brush_t> brush_pool;
phys_simple_allocator<gjk_partition_t> partition_pool;
phys_simple_allocator<gjk_double_sphere_t> double_sphere_pool;
phys_simple_allocator<gjk_cylinder_t> cylinder_pool;
phys_simple_allocator<gjk_polygon_cylinder_t> polygon_cylinder_pool;

// Box corner signs.
const phys_vec3 g_phys_vec3_box_sgn[8] = {
	phys_vec3(1.0f, 1.0f, 1.0f),
	phys_vec3(1.0f, 1.0f, -1.0f),
	phys_vec3(1.0f, -1.0f, 1.0f),
	phys_vec3(1.0f, -1.0f, -1.0f),
	phys_vec3(-1.0f, 1.0f, 1.0f),
	phys_vec3(-1.0f, 1.0f, -1.0f),
	phys_vec3(-1.0f, -1.0f, 1.0f),
	phys_vec3(-1.0f, -1.0f, -1.0f),
};

gjk_polygon_cylinder_t::poly_verts gjk_polygon_cylinder_t::s_poly_verts;

float phys_polygon_cylinder_adjust_height = 0.0f;

// setup_gjk_polygon_cylinder tunables.
static float foot_offset = 2.0f;
static float head_offset = 10.0f;
static int tad = 1;

// ---------------------------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------------------------

// Integer simplex indices are stored as raw int bits in a float slot (ind.x, or the packed
// double-sphere cache). memcpy keeps the bits without type-punning through float loads.
static inline void gjk_set_index(float * const dst, const int ind)
{
	memcpy(dst, &ind, sizeof(int));
}
static inline const int gjk_get_index(const float * const src)
{
	int ind;
	memcpy(&ind, src, sizeof(int));
	return ind;
}

// Vec3Dot order on the float[3] vertex arrays (brush / partition support).
static inline const float gjk_vec3_dot(const float * const a, const float * const b)
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// Shared by every create(). The query path constructs in place only when the allocator returned memory, then
// sets the flag without a NULL check.
template <class T>
static T * gjk_create_obj(phys_simple_allocator<T> & pool, gjk_geom_allocator * allocator, const int alignment)
{
	PHYS_ASSERT(allocator);
	T * obj;
	if (allocator->is_query())
	{
		obj = new (allocator->allocate((int)sizeof(T), alignment, false)) T;
		obj->m_flags |= gjk_base_t::FLAG_TEMP_ALLOCATION;
	}
	else
	{
		obj = pool.allocate();
		obj->set_geom_id_new(allocator->get_unique_id());
	}
	PHYS_ASSERT(obj);
	return obj;
}

// ---------------------------------------------------------------------------------------------
// gjk_base_t
// ---------------------------------------------------------------------------------------------

void gjk_base_t::comp_aabb_loc()
{
	if (!get_flag(FLAG_AABB_LOC_VALID))
	{
		m_flags |= FLAG_AABB_LOC_VALID;
		calc_aabb(PHYS_IDENTITY_MATRIX, &m_aabb_mn_loc, &m_aabb_mx_loc);
	}
}

const unsigned int gjk_base_t::get_type() const
{
	return GJK_BASE;
}

const phys_brush_desc * gjk_base_t::get_brush() const
{
	return NULL;
}

// ---------------------------------------------------------------------------------------------
// gjk_aabb_t
// ---------------------------------------------------------------------------------------------

gjk_aabb_t * gjk_aabb_t::create(const phys_vec3 & center, const phys_vec3 & dims, int stype, gjk_geom_allocator * allocator)
{
	gjk_aabb_t * const obj = gjk_create_obj(aabb_pool, allocator, 16);
	obj->m_center_local = center;
	obj->m_dims = dims;
	obj->stype = stype;
	return obj;
}

void gjk_aabb_t::destroy(gjk_aabb_t * geom)
{
	if (geom)
	{
		PHYS_ASSERT(!geom->get_flag(gjk_base_t::FLAG_TEMP_ALLOCATION));
		aabb_pool.free(geom);
	}
}

// -1 only for v[i] < 0, so a NaN component picks +1.
void gjk_aabb_t::support(const phys_vec3 & v, phys_vec3 * support_vert, phys_vec3 * support_ind) const
{
	const float sz = v[2] < 0.0f ? -1.0f : 1.0f;
	const float sy = v[1] < 0.0f ? -1.0f : 1.0f;
	const float sx = v[0] < 0.0f ? -1.0f : 1.0f;
	*support_ind = phys_vec3(sx, sy, sz);
	*support_vert = phys_vec3(support_ind->x * m_dims.x + m_center_local.x,
		support_ind->y * m_dims.y + m_center_local.y,
		support_ind->z * m_dims.z + m_center_local.z);
}

void gjk_aabb_t::get_simplex(const cached_simplex_info & cache_info, int index_count, phys_vec3 * simplex_verts, phys_vec3 * simplex_inds) const
{
	for (int i = 0; i < index_count; ++i)
	{
		const phys_vec3 & ind = cache_info.m_indices[i];
		simplex_inds[i] = ind;
		simplex_verts[i] = phys_vec3(ind.x * m_dims.x + m_center_local.x,
			ind.y * m_dims.y + m_center_local.y,
			ind.z * m_dims.z + m_center_local.z);
	}
}

const phys_vec3 gjk_aabb_t::get_center() const
{
	return m_center_local;
}

void gjk_aabb_t::get_feature(phys_contact_manifold * cman) const
{
	for (const phys_vec3 *i = g_phys_vec3_box_sgn, *const last_i = i + TL_ARRAY_COUNT(g_phys_vec3_box_sgn); i != last_i; ++i)
		cman->add_feature_point(phys_vec3(i->x * m_dims.x, i->y * m_dims.y, i->z * m_dims.z) + m_center_local);
}

void gjk_aabb_t::calc_aabb(const phys_mat44 & xform, phys_vec3 * aabb_min, phys_vec3 * aabb_max) const
{
	phys_calc_world_aabb(m_center_local, m_dims, xform, aabb_min, aabb_max);
}

bool gjk_aabb_t::is_polyhedron() const
{
	return true;
}

const unsigned int gjk_aabb_t::get_type() const
{
	return GJK_AABB;
}

const phys_brush_desc * gjk_aabb_t::get_brush() const
{
	return m_brush;
}

// ---------------------------------------------------------------------------------------------
// gjk_obb_t
// ---------------------------------------------------------------------------------------------

gjk_obb_t * gjk_obb_t::create(const phys_mat44 & xform, const phys_vec3 & dims, int stype, gjk_geom_allocator * allocator)
{
	gjk_obb_t * const obj = gjk_create_obj(obb_pool, allocator, 16);
	obj->m_dims = dims;
	obj->m_xform = xform;
	obj->stype = stype;
	return obj;
}

void gjk_obb_t::destroy(gjk_obb_t * geom)
{
	if (geom)
	{
		PHYS_ASSERT(!geom->get_flag(gjk_base_t::FLAG_TEMP_ALLOCATION));
		obb_pool.free(geom);
	}
}

// Same sign test as gjk_aabb_t::support (NaN -> +1).
void gjk_obb_t::support(const phys_vec3 & v, phys_vec3 * support_vert, phys_vec3 * support_ind) const
{
	const phys_vec3 local_v = phys_inv_multiply(m_xform, v);
	const float sz = local_v[2] < 0.0f ? -1.0f : 1.0f;
	const float sy = local_v[1] < 0.0f ? -1.0f : 1.0f;
	const float sx = local_v[0] < 0.0f ? -1.0f : 1.0f;
	*support_ind = phys_vec3(sx, sy, sz);
	*support_vert = phys_full_multiply(m_xform, phys_vec3(support_ind->x * m_dims.x, support_ind->y * m_dims.y, support_ind->z * m_dims.z));
}

void gjk_obb_t::get_simplex(const cached_simplex_info & cache_info, int index_count, phys_vec3 * simplex_verts, phys_vec3 * simplex_inds) const
{
	for (int i = 0; i < index_count; ++i)
	{
		const phys_vec3 & ind = cache_info.m_indices[i];
		simplex_inds[i] = ind;
		simplex_verts[i] = phys_full_multiply(m_xform, phys_vec3(ind.x * m_dims.x, ind.y * m_dims.y, ind.z * m_dims.z));
	}
}

const phys_vec3 gjk_obb_t::get_center() const
{
	return m_xform.GetW();
}

void gjk_obb_t::get_feature(phys_contact_manifold * cman) const
{
	for (const phys_vec3 *i = g_phys_vec3_box_sgn, *const last_i = i + TL_ARRAY_COUNT(g_phys_vec3_box_sgn); i != last_i; ++i)
		cman->add_feature_point(phys_full_multiply(m_xform, phys_vec3(i->x * m_dims.x, i->y * m_dims.y, i->z * m_dims.z)));
}

void gjk_obb_t::calc_aabb(const phys_mat44 & xform, phys_vec3 * aabb_min, phys_vec3 * aabb_max) const
{
	phys_mat44 mat;
	phys_full_multiply_mat(mat, xform, m_xform);
	*aabb_min = phys_full_multiply(mat, phys_vec3(m_dims.x * g_phys_vec3_box_sgn[0].x, m_dims.y * g_phys_vec3_box_sgn[0].y, m_dims.z * g_phys_vec3_box_sgn[0].z));
	*aabb_max = *aabb_min;
	for (const phys_vec3 *i = g_phys_vec3_box_sgn + 1, *const last_i = g_phys_vec3_box_sgn + TL_ARRAY_COUNT(g_phys_vec3_box_sgn); i != last_i; ++i)
		phys_aabb_add_point(mat, phys_vec3(m_dims.x * i->x, m_dims.y * i->y, m_dims.z * i->z), aabb_min, aabb_max);
}

bool gjk_obb_t::is_polyhedron() const
{
	return true;
}

const unsigned int gjk_obb_t::get_type() const
{
	return GJK_OBB;
}

// ---------------------------------------------------------------------------------------------
// gjk_brush_t
// ---------------------------------------------------------------------------------------------

gjk_base_t * gjk_brush_t::create(const phys_brush_desc * brush, int stype, gjk_geom_allocator * allocator)
{
	if (brush->numsides)
	{
		gjk_brush_t * const obj = gjk_create_obj(brush_pool, allocator, 16);
		obj->verts = brush->verts;
		obj->nverts = (int)brush->numverts;
		obj->brush = brush;
		obj->stype = stype;
		obj->set_contents(brush->contents);
		return obj;
	}

	// No sides: the brush is just its bounds.
	phys_vec3 aabb_min;
	phys_vec3 aabb_max;
	Phys_Vec3ToNitrousVec(brush->mins, aabb_min);
	Phys_Vec3ToNitrousVec(brush->maxs, aabb_max);
	const phys_vec3 center = 0.5f * (aabb_min + aabb_max);
	const phys_vec3 half_dims = 0.5f * (aabb_max - aabb_min);
	gjk_aabb_t * const obj = gjk_aabb_t::create(center, half_dims, stype, allocator);
	obj->set_contents(brush->contents);
	obj->m_brush = brush;
	return obj;
}

void gjk_brush_t::destroy(gjk_brush_t * geom)
{
	if (geom)
	{
		PHYS_ASSERT(!geom->get_flag(gjk_base_t::FLAG_TEMP_ALLOCATION));
		brush_pool.free(geom);
	}
}

void gjk_brush_t::support(const phys_vec3 & v, phys_vec3 * support_vert, phys_vec3 * support_ind) const
{
	float vec3_v[3];
	Phys_NitrousVecToVec3(v, vec3_v);
	int best_i = 0;
	float best_d = gjk_vec3_dot(verts[0], vec3_v);
	for (int i = 1; i < nverts; ++i)
	{
		const float d = gjk_vec3_dot(verts[i], vec3_v);
		if (d > best_d)
		{
			best_i = i;
			best_d = d;
		}
	}
	Phys_Vec3ToNitrousVec(verts[best_i], *support_vert);
	gjk_set_index(&support_ind->x, best_i);
}

void gjk_brush_t::get_simplex(const cached_simplex_info & cache_info, int index_count, phys_vec3 * simplex_verts, phys_vec3 * simplex_inds) const
{
	for (int i = 0; i < index_count; ++i)
	{
		simplex_inds[i] = cache_info.m_indices[i];
		Phys_Vec3ToNitrousVec(verts[gjk_get_index(&cache_info.m_indices[i].x)], simplex_verts[i]);
	}
}

const phys_vec3 gjk_brush_t::get_center() const
{
	return phys_vec3(0.0f, 0.0f, 0.0f);
}

// Every vertex is offered; phys_contact_manifold::add_feature_point applies the distance / angle eps.
void gjk_brush_t::get_feature(phys_contact_manifold * cman) const
{
	phys_vec3 vec;
	for (int i = 0; i < nverts; ++i)
	{
		Phys_Vec3ToNitrousVec(verts[i], vec);
		cman->add_feature_point(vec);
	}
}

void gjk_brush_t::calc_aabb(const phys_mat44 & xform, phys_vec3 * aabb_min, phys_vec3 * aabb_max) const
{
	phys_vec3 vec;
	Phys_Vec3ToNitrousVec(verts[0], vec);
	*aabb_min = phys_full_multiply(xform, vec);
	*aabb_max = *aabb_min;
	for (int i = 1; i < nverts; ++i)
	{
		Phys_Vec3ToNitrousVec(verts[i], vec);
		phys_aabb_add_point(xform, vec, aabb_min, aabb_max);
	}
}

bool gjk_brush_t::is_polyhedron() const
{
	return true;
}

const unsigned int gjk_brush_t::get_type() const
{
	return GJK_BRUSH;
}

const phys_brush_desc * gjk_brush_t::get_brush() const
{
	return brush;
}

// ---------------------------------------------------------------------------------------------
// gjk_partition_t
// ---------------------------------------------------------------------------------------------

gjk_partition_t * gjk_partition_t::create(const phys_collision_aabb_tree * tree, const phys_partition_world_desc & world, int stype, int contents, gjk_geom_allocator * allocator)
{
	gjk_partition_t * const obj = gjk_create_obj(partition_pool, allocator, 16);
	const phys_collision_partition * const partition = &world.partitions[tree->u.firstChildIndex];
	obj->verts = world.verts;
	obj->inds = &world.uinds[partition->fuind];
	obj->ninds = partition->nuinds;
	obj->partition = partition;
	obj->tree = tree;
	obj->stype = stype;
	obj->set_contents(contents);
	return obj;
}

void gjk_partition_t::destroy(gjk_partition_t * geom)
{
	if (geom)
	{
		PHYS_ASSERT(!geom->get_flag(gjk_base_t::FLAG_TEMP_ALLOCATION));
		partition_pool.free(geom);
	}
}

void gjk_partition_t::support(const phys_vec3 & v, phys_vec3 * support_vert, phys_vec3 * support_ind) const
{
	float vec3_v[3];
	Phys_NitrousVecToVec3(v, vec3_v);
	int best_i = 0;
	float best_d = gjk_vec3_dot(verts[inds[0]], vec3_v);
	for (int i = 1; i < ninds; ++i)
	{
		const float d = gjk_vec3_dot(verts[inds[i]], vec3_v);
		if (d > best_d)
		{
			best_i = i;
			best_d = d;
		}
	}
	Phys_Vec3ToNitrousVec(verts[inds[best_i]], *support_vert);
	gjk_set_index(&support_ind->x, best_i);
}

void gjk_partition_t::get_simplex(const cached_simplex_info & cache_info, int index_count, phys_vec3 * simplex_verts, phys_vec3 * simplex_inds) const
{
	for (int i = 0; i < index_count; ++i)
	{
		simplex_inds[i] = cache_info.m_indices[i];
		Phys_Vec3ToNitrousVec(verts[inds[gjk_get_index(&cache_info.m_indices[i].x)]], simplex_verts[i]);
	}
}

const phys_vec3 gjk_partition_t::get_center() const
{
	return phys_vec3(0.0f, 0.0f, 0.0f);
}

// As gjk_brush_t: every vertex is offered, add_feature_point does the selection.
void gjk_partition_t::get_feature(phys_contact_manifold * cman) const
{
	phys_vec3 vec;
	for (int i = 0; i < ninds; ++i)
	{
		Phys_Vec3ToNitrousVec(verts[inds[i]], vec);
		cman->add_feature_point(vec);
	}
}

void gjk_partition_t::calc_aabb(const phys_mat44 & xform, phys_vec3 * aabb_min, phys_vec3 * aabb_max) const
{
	const unsigned short * inds_i = inds;
	const unsigned short * const last_inds_i = inds_i + ninds;
	phys_vec3 vec;
	Phys_Vec3ToNitrousVec(verts[*inds_i], vec);
	*aabb_min = phys_full_multiply(xform, vec);
	*aabb_max = *aabb_min;
	for (++inds_i; inds_i < last_inds_i; ++inds_i)
	{
		Phys_Vec3ToNitrousVec(verts[*inds_i], vec);
		phys_aabb_add_point(xform, vec, aabb_min, aabb_max);
	}
}

bool gjk_partition_t::is_polyhedron() const
{
	return true;
}

const unsigned int gjk_partition_t::get_type() const
{
	return GJK_PARTITION;
}

// ---------------------------------------------------------------------------------------------
// gjk_double_sphere_t
// ---------------------------------------------------------------------------------------------

gjk_double_sphere_t * gjk_double_sphere_t::create(const phys_vec3 & c0, const phys_vec3 & c1, float r, int stype, gjk_geom_allocator * allocator)
{
	gjk_double_sphere_t * const obj = gjk_create_obj(double_sphere_pool, allocator, 0); // alignment 0 on purpose
	obj->m_center = 0.5f * (c0 + c1);
	obj->m_count = 2;
	obj->m_list_center[0] = c0;
	obj->m_list_center[1] = c1;
	obj->m_list_radius[0] = 0.0f;
	obj->m_list_radius[1] = 0.0f;
	obj->m_geom_radius = r;
	obj->stype = stype;
	return obj;
}

void gjk_double_sphere_t::destroy(gjk_double_sphere_t * geom)
{
	if (geom)
	{
		PHYS_ASSERT_STR(!geom->get_flag(gjk_base_t::FLAG_TEMP_ALLOCATION), "!geom->get_flag( gjk_base_t::FLAG_TEMP_ALLOCATION )");
		double_sphere_pool.free(geom);
	}
}

// Only sphere 1's radius moves its point; sphere 0 is used as its center (both radii are 0 for create()'s capsules).
// Sphere 0 is kept only when dot >= 0 (NaN -> sphere 1).
void gjk_double_sphere_t::support(const phys_vec3 & v, phys_vec3 * support_vert, phys_vec3 * support_ind) const
{
	if (m_count == 1)
	{
		*support_vert = m_list_center[0];
		gjk_set_index(&support_ind->x, 0);
	}
	else
	{
		const float nv = Abs(v);
		PHYS_ASSERT(nv > 0.000001f);
		const phys_vec3 unit_v = (1.0f / nv) * v;
		const phys_vec3 p1 = m_list_center[1] + m_list_radius[1] * unit_v;
		if (phys_dot(v, m_list_center[0] - p1) >= 0.0f)
		{
			*support_vert = m_list_center[0];
			gjk_set_index(&support_ind->x, 0);
		}
		else
		{
			*support_vert = p1;
			gjk_set_index(&support_ind->x, 1);
		}
	}
}

void gjk_double_sphere_t::get_simplex(const cached_simplex_info & cache_info, int index_count, phys_vec3 * simplex_verts, phys_vec3 * simplex_inds) const
{
	const float * const cached_inds = &cache_info.m_indices[1].x; // packed by set_simplex
	for (int i = 0; i < index_count; ++i)
	{
		const int ind = gjk_get_index(cached_inds + i);
		PHYS_ASSERT(ind >= 0 && ind < m_count);
		gjk_set_index(&simplex_inds[i].x, ind);
		const float r = m_list_radius[ind];
		simplex_verts[i] = m_list_center[ind] + r * cache_info.m_indices[0];
	}
}

// Stores the normal plus the sphere numbers of the w_set vertices, packed float by float from m_indices[1].x.
void gjk_double_sphere_t::set_simplex(const phys_vec3 * simplex_inds, int w_set, const phys_vec3 & normal, cached_simplex_info * cache_info) const
{
	cache_info->m_indices[0] = normal;
	float * inds = &cache_info->m_indices[1].x;
	for (int i = 0; i < 4; ++i)
	{
		if (w_set & (1 << i))
		{
			memcpy(inds, &simplex_inds[i].x, sizeof(float));
			++inds;
		}
	}
}

const phys_vec3 gjk_double_sphere_t::get_center() const
{
	return m_center;
}

void gjk_double_sphere_t::get_feature(phys_contact_manifold * cman) const
{
	for (int j = 0; j < m_count; ++j)
	{
		const float r = m_list_radius[j] + m_geom_radius;
		cman->add_feature_point(m_list_center[j] - r * cman->m_feature_hitn);
	}
}

const float gjk_double_sphere_t::get_geom_radius() const
{
	return m_geom_radius;
}

void gjk_double_sphere_t::calc_aabb(const phys_mat44 & xform, phys_vec3 * aabb_min, phys_vec3 * aabb_max) const
{
	phys_aabb_init_sphere(xform, m_list_center[0], m_list_radius[0] + m_geom_radius, aabb_min, aabb_max);
	for (int i = 1; i < m_count; ++i)
		phys_aabb_add_sphere(xform, m_list_center[i], m_list_radius[i] + m_geom_radius, aabb_min, aabb_max);
}

bool gjk_double_sphere_t::is_polyhedron() const
{
	return m_count == 1 || m_list_radius[1] == 0.0f;
}

const unsigned int gjk_double_sphere_t::get_type() const
{
	return GJK_DOUBLE_SPHERE;
}

// ---------------------------------------------------------------------------------------------
// gjk_cylinder_t
// ---------------------------------------------------------------------------------------------

gjk_cylinder_t * gjk_cylinder_t::create(int _direction, float _halfHeight, float _radius, const phys_mat44 & _xform, int stype, gjk_geom_allocator * allocator)
{
	gjk_cylinder_t * const obj = gjk_create_obj(cylinder_pool, allocator, 16);
	obj->direction = _direction;
	obj->halfHeight = _halfHeight;
	obj->radius = _radius;
	obj->m_geom_radius = 0.0f;
	obj->xform = _xform;
	obj->stype = stype;
	return obj;
}

void gjk_cylinder_t::destroy(gjk_cylinder_t * geom)
{
	if (geom)
	{
		PHYS_ASSERT(!geom->get_flag(gjk_base_t::FLAG_TEMP_ALLOCATION));
		cylinder_pool.free(geom);
	}
}

// The radial part of the local direction is normalised (zero when within 0.001 of the axis); the axial part
// becomes +-1. The result, scaled by get_dims(), is the rim point.
// A NaN length gives a zero radial part; a NaN v_dir gives -1.
void gjk_cylinder_t::support(const phys_vec3 & v, phys_vec3 * support_vert, phys_vec3 * support_ind) const
{
	phys_vec3 dir_local = phys_inv_multiply(xform, v);
	const float v_dir = dir_local[direction];
	dir_local[direction] = 0.0f;
	const float len = Abs(dir_local);
	if (len > 0.001f)
		dir_local /= len;
	else
		dir_local = PHYS_ZERO_VEC;
	dir_local[direction] = v_dir >= 0.0f ? 1.0f : -1.0f;
	*support_ind = dir_local;
	const phys_vec3 dims = get_dims();
	*support_vert = phys_full_multiply(xform, phys_vec3(dir_local.x * dims.x, dir_local.y * dims.y, dir_local.z * dims.z));
}

void gjk_cylinder_t::get_simplex(const cached_simplex_info & cache_info, int index_count, phys_vec3 * simplex_verts, phys_vec3 * simplex_inds) const
{
	const phys_vec3 dims = get_dims();
	for (int i = 0; i < index_count; ++i)
	{
		const phys_vec3 & ind = cache_info.m_indices[i];
		simplex_inds[i] = ind;
		simplex_verts[i] = phys_full_multiply(xform, phys_vec3(ind.x * dims.x, ind.y * dims.y, ind.z * dims.z));
	}
}

const phys_vec3 gjk_cylinder_t::get_center() const
{
	return phys_full_multiply(xform, phys_vec3(0.0f, 0.0f, 0.0f));
}

// Feature of the cylinder facing -hitn (hitn = cman->m_feature_hitn, geom frame):
// - |cos| between hitn and the axis >= 0.7071: the cap facing away from hitn, as 4 rim points (90 degrees apart);
// - otherwise: the side line opposite hitn, as its 2 end points.
// A NaN d takes the cap / swizzle paths; -halfHeight only when hitn[direction] > 0 (NaN -> +halfHeight).
void gjk_cylinder_t::get_feature(phys_contact_manifold * cman) const
{
	static float c[4] = {1.0f, 0.0f, -1.0f, 0.0f};
	static float s[4] = {0.0f, 1.0f, 0.0f, -1.0f};

	PHYS_ASSERT(m_geom_radius == 0.0f);

	phys_vec3 axis;
	if (direction == 0)
		axis = PHYS_IDENTITY_MATRIX.GetX();
	else if (direction == 1)
		axis = PHYS_IDENTITY_MATRIX.GetY();
	else
		axis = PHYS_IDENTITY_MATRIX.GetZ();

	const phys_vec3 hitn = phys_inv_multiply(xform, cman->m_feature_hitn);
	phys_vec3 r1 = hitn;
	r1[direction] = 0.0f;
	const float d = phys_dot(axis, hitn);

	if (!(fabsf(d) < 0.7071f))
	{
		phys_vec3 r2;
		if (!(fabsf(d) < 0.99f))
		{
			// hitn is (almost) the axis: any two radial directions will do.
			r1 = phys_vec3(axis[2], axis[0], axis[1]);
			r2 = phys_vec3(axis[1], axis[2], axis[0]);
		}
		else
		{
			r1 /= Abs(r1);
			r2 = phys_cross(r1, axis);
		}

		phys_vec3 p0(0.0f, 0.0f, 0.0f);
		p0[direction] = hitn[direction] > 0.0f ? -halfHeight : halfHeight;
		r1 *= radius;
		r2 *= radius;
		for (int j = 0; j < 4; ++j)
		{
			const phys_vec3 p = (c[j] * r1 + s[j] * r2) + p0;
			cman->add_feature_point(phys_full_multiply(xform, p));
		}
	}
	else
	{
		const float len = Abs(r1);
		PHYS_ASSERT(len > 0.00001f);
		const float sr = radius / len;
		phys_vec3 res = -sr * r1;
		res[direction] = halfHeight;
		cman->add_feature_point(phys_multiply(xform, res) + xform.GetW());
		res[direction] = -halfHeight;
		cman->add_feature_point(phys_multiply(xform, res) + xform.GetW());
	}
}

const float gjk_cylinder_t::get_geom_radius() const
{
	return m_geom_radius;
}

// Bounded by the spheres of radius `radius` around the two cap centres.
void gjk_cylinder_t::calc_aabb(const phys_mat44 & xform_, phys_vec3 * aabb_min, phys_vec3 * aabb_max) const
{
	phys_vec3 top(0.0f, 0.0f, 0.0f);
	phys_vec3 bot(0.0f, 0.0f, 0.0f);
	top[direction] += halfHeight;
	bot[direction] -= halfHeight;
	phys_aabb_init_sphere(xform_, phys_full_multiply(xform, top), radius + m_geom_radius, aabb_min, aabb_max);
	phys_aabb_add_sphere(xform_, phys_full_multiply(xform, bot), radius + m_geom_radius, aabb_min, aabb_max);
}

bool gjk_cylinder_t::is_polyhedron() const
{
	return false;
}

const unsigned int gjk_cylinder_t::get_type() const
{
	return GJK_CYLINDER;
}

// ---------------------------------------------------------------------------------------------
// gjk_polygon_cylinder_t
// ---------------------------------------------------------------------------------------------

// Quadrant-0 angles 0, 30, 60, 90 degrees. 3.1415925f (one ulp below pi) on purpose.
gjk_polygon_cylinder_t::poly_verts::poly_verts()
{
	for (int i = 0; i < 4; ++i)
	{
		const float theta = (float)i * 3.1415925f / 2.0f / 3.0f;
		m_co[i] = (float)::cos((double)theta);
		m_si[i] = (float)::sin((double)theta);
	}
}

// Vertex i of the 12-ring: quadrant i / 3 mirrors the quadrant-0 table (index a + i * b, signs cos / sis).
void gjk_polygon_cylinder_t::poly_verts::get_co_si(const int i, float * co_, float * si_)
{
	static const int a[] = {0, 6, -6, 12};
	static const int b[] = {1, -1, 1, -1};
	static const float cos[] = {1.0f, -1.0f, -1.0f, 1.0f};
	static const float sis[] = {1.0f, 1.0f, -1.0f, -1.0f};

	PHYS_ASSERT(i >= 0 && i < NUM_VERTS_);
	const int quad = i / 3;
	const int ii = a[quad] + i * b[quad];
	*co_ = cos[quad] * m_co[ii];
	*si_ = sis[quad] * m_si[ii];
}

const int gjk_polygon_cylinder_t::poly_verts::support(const phys_vec3 & v)
{
	const float pi = 3.1415927f;
	float at = (float)atan2((double)v.y, (double)v.x);
	if (at < 0.0f)
		at = 2.0f * pi + at;
	const float tempi = (12.0f * at) / (2.0f * pi);
	const int best_i = (int)(tempi + 0.5f) % NUM_VERTS_;
	PHYS_ASSERT(best_i >= 0 && best_i < NUM_VERTS_);
	return best_i;
}

gjk_polygon_cylinder_t * gjk_polygon_cylinder_t::create(const float (&mins)[3], const float (&maxs)[3], float radius_adjust, int stype, gjk_geom_allocator * allocator)
{
	gjk_polygon_cylinder_t * const obj = gjk_create_obj(polygon_cylinder_pool, allocator, 16);
	setup_gjk_polygon_cylinder(mins, maxs, radius_adjust, obj);
	obj->stype = stype;
	return obj;
}

void gjk_polygon_cylinder_t::destroy(gjk_polygon_cylinder_t * geom)
{
	if (geom)
	{
		PHYS_ASSERT(!geom->get_flag(gjk_base_t::FLAG_TEMP_ALLOCATION));
		polygon_cylinder_pool.free(geom);
	}
}

void gjk_polygon_cylinder_t::calc_disc_aabb(const phys_vec3 & axis, const float radius, phys_vec3 * aabb_min, phys_vec3 * aabb_max)
{
	const phys_vec3 axis_sq(axis.x * axis.x, axis.y * axis.y, axis.z * axis.z);
	*aabb_max = radius * phys_vec3(sqrtf(axis_sq.y + axis_sq.z), sqrtf(axis_sq.x + axis_sq.z), sqrtf(axis_sq.x + axis_sq.y));
	*aabb_min = -*aabb_max;
}

// Head side only when v.z >= 0 (NaN -> foot); the ring / cap point is kept when dot >= 0 (NaN -> apex / bottom point).
void gjk_polygon_cylinder_t::support(const phys_vec3 & v, phys_vec3 * support_vert, phys_vec3 * support_ind) const
{
	if (m_mode)
	{
		PHYS_ASSERT(m_half_height >= m_capsule_radius);
		const float h = m_half_height - m_capsule_radius;
		const phys_vec3 p1 = m_center + phys_vec3(0.0f, 0.0f, h);
		const phys_vec3 p2 = m_center - phys_vec3(0.0f, 0.0f, h);
		if (phys_dot(p1 - p2, v) >= 0.0f)
		{
			gjk_set_index(&support_ind->x, 0);
			*support_vert = p1;
		}
		else
		{
			gjk_set_index(&support_ind->x, 1);
			*support_vert = p2;
		}
	}
	else
	{
		const int best_i = s_poly_verts.support(v);
		float disc_z;
		float foot_or_head_z;
		int foot_or_head_ind;
		if (v.z >= 0.0f)
		{
			gjk_set_index(&support_ind->x, best_i);
			disc_z = m_half_height - m_head_offset;
			foot_or_head_z = m_half_height;
			foot_or_head_ind = 2 * poly_verts::NUM_VERTS_;
		}
		else
		{
			gjk_set_index(&support_ind->x, best_i + poly_verts::NUM_VERTS_);
			disc_z = -m_half_height + m_foot_offset;
			foot_or_head_z = -m_half_height;
			foot_or_head_ind = 2 * poly_verts::NUM_VERTS_ + 1;
		}

		float co_;
		float si_;
		s_poly_verts.get_co_si(best_i, &co_, &si_);
		const phys_vec3 p = phys_vec3(m_polygon_cylinder_radius * co_, m_polygon_cylinder_radius * si_, disc_z) + m_center;
		const phys_vec3 foot_or_head = phys_vec3(0.0f, 0.0f, foot_or_head_z) + m_center;
		if (phys_dot(p - foot_or_head, v) >= 0.0f)
		{
			*support_vert = p;
		}
		else
		{
			gjk_set_index(&support_ind->x, foot_or_head_ind);
			*support_vert = foot_or_head;
		}
	}
}

// Only ind.x of each simplex index is copied.
void gjk_polygon_cylinder_t::get_simplex(const cached_simplex_info & cache_info, int index_count, phys_vec3 * simplex_verts, phys_vec3 * simplex_inds) const
{
	if (m_mode)
	{
		PHYS_ASSERT(m_half_height >= m_capsule_radius);
		const float h = m_half_height - m_capsule_radius;
		for (int i = 0; i < index_count; ++i)
		{
			const int ind = gjk_get_index(&cache_info.m_indices[i].x);
			PHYS_ASSERT(ind == 0 || ind == 1);
			gjk_set_index(&simplex_inds[i].x, ind);
			if (ind == 0)
				simplex_verts[i] = m_center + phys_vec3(0.0f, 0.0f, h);
			else
				simplex_verts[i] = m_center - phys_vec3(0.0f, 0.0f, h);
		}
	}
	else
	{
		for (int i = 0; i < index_count; ++i)
		{
			const int ind = gjk_get_index(&cache_info.m_indices[i].x);
			PHYS_ASSERT(ind >= 0);
			gjk_set_index(&simplex_inds[i].x, ind);
			if (ind < poly_verts::NUM_VERTS_)
			{
				float co_;
				float si_;
				s_poly_verts.get_co_si(ind, &co_, &si_);
				simplex_verts[i] = phys_vec3(m_polygon_cylinder_radius * co_, m_polygon_cylinder_radius * si_, m_half_height - m_head_offset) + m_center;
			}
			else if (ind < 2 * s_poly_verts.get_num_verts())
			{
				float co_;
				float si_;
				s_poly_verts.get_co_si(ind - poly_verts::NUM_VERTS_, &co_, &si_);
				simplex_verts[i] = phys_vec3(m_polygon_cylinder_radius * co_, m_polygon_cylinder_radius * si_, -m_half_height + m_foot_offset) + m_center;
			}
			else if (ind == 2 * s_poly_verts.get_num_verts())
			{
				simplex_verts[i] = phys_vec3(0.0f, 0.0f, m_half_height) + m_center;
			}
			else
			{
				PHYS_ASSERT(ind == 2 * s_poly_verts.get_num_verts() + 1);
				simplex_verts[i] = phys_vec3(0.0f, 0.0f, -m_half_height) + m_center;
			}
		}
	}
}

const phys_vec3 gjk_polygon_cylinder_t::get_center() const
{
	return m_center;
}

// Empty: the polygon cylinder is a query-only shape.
void gjk_polygon_cylinder_t::get_feature(phys_contact_manifold *) const {}

const float gjk_polygon_cylinder_t::get_geom_radius() const
{
	if (m_mode)
		return m_geom_radius + m_capsule_radius;
	else
		return m_geom_radius;
}

void gjk_polygon_cylinder_t::calc_aabb(const phys_mat44 & xform, phys_vec3 * aabb_min, phys_vec3 * aabb_max) const
{
	calc_disc_aabb(xform.GetZ(), m_polygon_cylinder_radius, aabb_min, aabb_max);
	const phys_vec3 top = m_half_height * xform.GetZ();
	const phys_vec3 bot = -m_half_height * xform.GetZ();
	*aabb_min += phys_min(top, bot);
	*aabb_max += phys_max(top, bot);

	PHYS_ASSERT(m_head_offset <= m_half_height);
	PHYS_ASSERT(m_foot_offset <= m_half_height);
	PHYS_ASSERT(m_capsule_radius <= m_polygon_cylinder_radius);

	const phys_vec3 center = phys_multiply(xform, m_center) + xform.GetW();
	*aabb_min = *aabb_min + center - phys_vec3(m_geom_radius);
	*aabb_max = *aabb_max + center + phys_vec3(m_geom_radius);
}

bool gjk_polygon_cylinder_t::is_polyhedron() const
{
	return true;
}

const unsigned int gjk_polygon_cylinder_t::get_type() const
{
	return GJK_POLYGON_CYLINDER;
}

// The box is shrunk by (radius_adjust - SURFACE_CLIP_EPSILON) on each side (never past the opposite face), then:
// capsule radius = max(1, half the smaller of the x / z extents), half height = max(1, half the z extent),
// polygon radius = capsule radius / cos(15 deg) (the 12-gon circumscribes the circle).
// The clamps are written "1.0f > x ? 1.0f : x" so a NaN passes through; the head / foot offsets are
// "half > offset ? offset : half" (NaN -> half).
void setup_gjk_polygon_cylinder(const float * const mins, const float * const maxs, float radius_adjust, gjk_polygon_cylinder_t * gjk_cylinder)
{
	const float SURFACE_CLIP_EPSILON = 0.125f;

	phys_vec3 pv_mins;
	phys_vec3 pv_maxs;
	Phys_Vec3ToNitrousVec(mins, pv_mins);
	Phys_Vec3ToNitrousVec(maxs, pv_maxs);
	PHYS_ASSERT(radius_adjust >= SURFACE_CLIP_EPSILON);

	const float dim_adjust = radius_adjust - SURFACE_CLIP_EPSILON;
	const phys_vec3 dim_adjust_vec(dim_adjust);
	const phys_vec3 save_pv_mins = pv_mins;
	const phys_vec3 save_pv_maxs = pv_maxs;
	pv_mins = phys_min(save_pv_mins + dim_adjust_vec, save_pv_maxs);
	pv_maxs = phys_max(save_pv_maxs - dim_adjust_vec, save_pv_mins);

	const phys_vec3 dim = pv_maxs - pv_mins;
	PHYS_ASSERT(dim.GetX() >= 0.0f);
	PHYS_ASSERT(dim.GetY() >= 0.0f);
	PHYS_ASSERT(dim.GetZ() >= 0.0f);

	const float min_xz = dim[0] > dim[2] ? dim[2] : dim[0];
	const float half_min_xz = 0.5f * min_xz;
	const float radius = 1.0f > half_min_xz ? 1.0f : half_min_xz;
	if (radius == 1.0f)
		tlWarning("degenerate capsule.");
	PHYS_ASSERT(radius >= 0.0f);

	const float half_z = 0.5f * dim.z;
	gjk_cylinder->m_half_height = 1.0f > half_z ? 1.0f : half_z;
	if (gjk_cylinder->m_half_height == 1.0f)
		tlWarning("degenerate capsule1.");

	gjk_cylinder->m_capsule_radius = radius;
	if (tad)
	{
		static float radius_scale_factor = 1.0f / (float)::cos((double)(6.2831855f / 12.0f / 2.0f));
		gjk_cylinder->m_polygon_cylinder_radius = radius * radius_scale_factor;
	}
	else
	{
		gjk_cylinder->m_polygon_cylinder_radius = radius;
	}

	gjk_cylinder->m_geom_radius = radius_adjust;
	gjk_cylinder->m_center = phys_vec3(0.0f, 0.0f, (pv_mins.z + pv_maxs.z) * 0.5f + phys_polygon_cylinder_adjust_height);
	gjk_cylinder->m_head_offset = gjk_cylinder->m_half_height > head_offset ? head_offset : gjk_cylinder->m_half_height;
	gjk_cylinder->m_foot_offset = gjk_cylinder->m_half_height > foot_offset ? foot_offset : gjk_cylinder->m_half_height;
	gjk_cylinder->m_mode = 0;
}

// ---------------------------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------------------------

void destroy_gjk_geom(gjk_base_t * geom)
{
	switch (geom->get_type())
	{
	case GJK_AABB:
		gjk_aabb_t::destroy((gjk_aabb_t *)geom);
		break;
	case GJK_BRUSH:
		gjk_brush_t::destroy((gjk_brush_t *)geom);
		break;
	case GJK_PARTITION:
		gjk_partition_t::destroy((gjk_partition_t *)geom);
		break;
	case GJK_DOUBLE_SPHERE:
		gjk_double_sphere_t::destroy((gjk_double_sphere_t *)geom);
		break;
	case GJK_CYLINDER:
		gjk_cylinder_t::destroy((gjk_cylinder_t *)geom);
		break;
	case GJK_OBB:
		gjk_obb_t::destroy((gjk_obb_t *)geom);
		break;
	case GJK_POLYGON_CYLINDER:
		gjk_polygon_cylinder_t::destroy((gjk_polygon_cylinder_t *)geom);
		break;
	default:
		PHYS_ASSERT_STR_MSG(0, "0", "Trying to free up unkownn geom. Leak!!!"); // sic
		break;
	}
}

void destroy_gjk_geom(gjk_geom_list_t * gjk_geom_list)
{
	PHYS_ASSERT(gjk_geom_list->m_geom_count >= 0);
	gjk_base_t * next_geom;
	for (gjk_base_t * geom = gjk_geom_list->m_first_geom; geom; geom = next_geom)
	{
		next_geom = geom->m_next_geom;
		destroy_gjk_geom(geom);
	}
	gjk_geom_list->m_first_geom = NULL;
	gjk_geom_list->m_geom_count = 0;
}

// dims / center are 0.5 * mx + -0.5 * mn and 0.5 * mx + 0.5 * mn; keep that form.
gjk_base_t * create_aabb_gjk_geom(const float * const mn, const float * const mx, int stype, gjk_geom_allocator * allocator)
{
	float dims[3];
	float center[3];
	dims[0] = 0.5f * mx[0] + -0.5f * mn[0];
	dims[1] = 0.5f * mx[1] + -0.5f * mn[1];
	dims[2] = 0.5f * mx[2] + -0.5f * mn[2];
	center[0] = 0.5f * mx[0] + 0.5f * mn[0];
	center[1] = 0.5f * mx[1] + 0.5f * mn[1];
	center[2] = 0.5f * mx[2] + 0.5f * mn[2];
	phys_vec3 dims_;
	phys_vec3 center_;
	Phys_Vec3ToNitrousVec(dims, dims_);
	Phys_Vec3ToNitrousVec(center, center_);
	return gjk_aabb_t::create(center_, dims_, stype, allocator);
}

gjk_base_t * create_obb_gjk_geom(const float (*const orientation)[3], const float * const offset, const float * const halfLengths, int stype, gjk_geom_allocator * allocator)
{
	phys_mat44 xform;
	phys_vec3 dims;
	Phys_Vec3ToNitrousVec(offset, xform.w);
	Phys_AxisToNitrousMat(orientation, xform);
	Phys_Vec3ToNitrousVec(halfLengths, dims);
	return gjk_obb_t::create(xform, dims, stype, allocator);
}

gjk_base_t * create_brush_gjk_geom(const phys_brush_desc * brush, int stype, gjk_geom_allocator * allocator)
{
	return gjk_brush_t::create(brush, stype, allocator);
}

// Capsule of the given radius around the segment center +- halfHeight along axis `direction`.
gjk_base_t * create_capsule_gjk_geom(const float * const center, float radius, float halfHeight, int direction, int stype, gjk_geom_allocator * allocator)
{
	phys_vec3 _center;
	Phys_Vec3ToNitrousVec(center, _center);
	phys_vec3 c0 = _center;
	phys_vec3 c1 = _center;
	c0[direction] -= halfHeight;
	c1[direction] += halfHeight;
	return gjk_double_sphere_t::create(c0, c1, radius, stype, allocator);
}

// Cylinder along rot[0] (direction 0).
gjk_base_t * create_cylinder_gjk_geom(const float (*const rot)[3], const float * const trans, float radius, float halfHeight, int stype, gjk_geom_allocator * allocator)
{
	phys_vec3 _trans;
	phys_mat44 xform;
	Phys_Vec3ToNitrousVec(trans, _trans);
	Phys_AxisToNitrousMat(rot, xform);
	xform.w = _trans;
	return gjk_cylinder_t::create(0, halfHeight, radius, xform, stype, allocator);
}
