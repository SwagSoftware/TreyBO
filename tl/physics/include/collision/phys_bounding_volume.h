#pragma once

#include "tl/physics/include/phys_math.h"

inline void phys_calc_world_aabb(const phys_vec3 & local_center, const phys_vec3 & local_half_aabb_dims, const phys_mat44 & local_to_world_xform, phys_vec3 * aabb_min, phys_vec3 * aabb_max)
{
	phys_mat44 world_to_local_xform;
	phys_transpose(world_to_local_xform, local_to_world_xform);
	const phys_vec3 world_half_dims(phys_dot(phys_AbsValue(world_to_local_xform.GetX()), local_half_aabb_dims),
		phys_dot(phys_AbsValue(world_to_local_xform.GetY()), local_half_aabb_dims),
		phys_dot(phys_AbsValue(world_to_local_xform.GetZ()), local_half_aabb_dims));
	const phys_vec3 world_center = phys_full_multiply(local_to_world_xform, local_center);
	*aabb_min = world_center - world_half_dims;
	*aabb_max = world_center + world_half_dims;
}

inline void phys_calc_local_aabb(const phys_vec3 & aabb_min, const phys_vec3 & aabb_max, const phys_mat44 & local_to_world_xform, phys_vec3 * local_aabb_min, phys_vec3 * local_aabb_max)
{
	const phys_vec3 half_dims = 0.5f * (aabb_max - aabb_min);
	const phys_vec3 local_half_dims(phys_dot(phys_AbsValue(local_to_world_xform.GetX()), half_dims),
		phys_dot(phys_AbsValue(local_to_world_xform.GetY()), half_dims),
		phys_dot(phys_AbsValue(local_to_world_xform.GetZ()), half_dims));
	const phys_vec3 local_center = phys_full_inv_multiply(local_to_world_xform, 0.5f * (aabb_min + aabb_max));
	*local_aabb_min = local_center - local_half_dims;
	*local_aabb_max = local_center + local_half_dims;
}

inline void phys_aabb_add_point(const phys_mat44 & xform, const phys_vec3 & point, phys_vec3 * aabb_min, phys_vec3 * aabb_max)
{
	const phys_vec3 point_abs = phys_full_multiply(xform, point);
	*aabb_min = phys_min(*aabb_min, point_abs);
	*aabb_max = phys_max(*aabb_max, point_abs);
}

inline void phys_aabb_init_sphere(const phys_mat44 & xform, const phys_vec3 & center, const float radius, phys_vec3 * aabb_min, phys_vec3 * aabb_max)
{
	const phys_vec3 center_abs = phys_full_multiply(xform, center);
	const phys_vec3 rvec(radius);
	*aabb_min = center_abs - rvec;
	*aabb_max = center_abs + rvec;
}

inline void phys_aabb_add_sphere(const phys_mat44 & xform, const phys_vec3 & center, const float radius, phys_vec3 * aabb_min, phys_vec3 * aabb_max)
{
	const phys_vec3 center_abs = phys_multiply(xform, center) + xform.GetW();
	const phys_vec3 rvec(radius);
	*aabb_min = phys_min(*aabb_min, center_abs - rvec);
	*aabb_max = phys_max(*aabb_max, center_abs + rvec);
}

// hace = "half active constraint epsilon" (PHYS_HACE): the pad added around trace volumes, so contacts within the
// active-constraint distance are found before bodies actually touch. Volumes carrying this pad are named
// *_whace ("with half active constraint epsilon"); padding twice gives *_wace ("with active constraint epsilon").
inline void phys_aabb_add_hace(phys_vec3 * aabb_min, phys_vec3 * aabb_max)
{
	aabb_min->x -= PHYS_HACE;
	aabb_min->y -= PHYS_HACE;
	aabb_min->z -= PHYS_HACE;
	aabb_max->x += PHYS_HACE;
	aabb_max->y += PHYS_HACE;
	aabb_max->z += PHYS_HACE;
}

// Swept AABB test: box 0 moves by aabb0_translation against static box 1. hit_time is the entry fraction.
// The tmin / tmax updates are written as !(a > b) on purpose: a NaN t must replace the bound, which a "<="
// comparison would not do.
inline const bool phys_are_potentially_colliding(const phys_vec3 & aabb_min0, const phys_vec3 & aabb_max0, const phys_vec3 & aabb0_translation,
	const phys_vec3 & aabb_min1, const phys_vec3 & aabb_max1, float * hit_time)
{
	float tmin = 0.0f;
	float tmax = 1.0f;
	for (int i = 0; i < 3; ++i)
	{
		const float dif_min1_max0 = aabb_min1[i] - aabb_max0[i];
		const float dif_max1_min0 = aabb_max1[i] - aabb_min0[i];
		if (aabb0_translation[i] > PHYS_EPS_1E5)
		{
			const float t0 = dif_min1_max0 / aabb0_translation[i];
			tmin = !(tmin > t0) ? t0 : tmin;
			const float t1 = dif_max1_min0 / aabb0_translation[i];
			tmax = !(t1 > tmax) ? t1 : tmax;
		}
		else if (aabb0_translation[i] < -PHYS_EPS_1E5)
		{
			const float t0 = dif_min1_max0 / aabb0_translation[i];
			tmax = !(t0 > tmax) ? t0 : tmax;
			const float t1 = dif_max1_min0 / aabb0_translation[i];
			tmin = !(tmin > t1) ? t1 : tmin;
		}
		else if (dif_min1_max0 > 0.0f || dif_max1_min0 < 0.0f)
		{
			return false;
		}
		if (tmin > tmax)
			return false;
	}
	PHYS_ASSERT(tmin >= 0.0f && tmin <= 1.0f);
	PHYS_ASSERT_VALID_FLOAT(tmin);
	*hit_time = tmin;
	return true;
}

inline bool phys_are_aabb_overlapping(const phys_vec3 & aabb1_min, const phys_vec3 & aabb1_max, const phys_vec3 & aabb2_min, const phys_vec3 & aabb2_max)
{
	return aabb2_max.x >= aabb1_min.x && aabb2_max.y >= aabb1_min.y && aabb2_max.z >= aabb1_min.z && aabb1_max.x >= aabb2_min.x && aabb1_max.y >= aabb2_min.y && aabb1_max.z >= aabb2_min.z;
}
