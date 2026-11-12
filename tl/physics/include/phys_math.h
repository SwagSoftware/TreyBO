#pragma once

// old_phys_math.h holds the vector/matrix types and the math inlines.

#include "tl/physics/include/old_phys_math.h"

// Real (non-macro) functions: they run in release builds too.
inline void PHYS_ASSERT_UNIT(const phys_vec3 & v)
{
	const float nv = Abs(v);
	PHYS_ASSERT(fabsf(nv - 1.0f) < 0.001f);
}

inline void PHYS_ASSERT_ORTHOGONAL(const phys_vec3 & v1, const phys_vec3 & v2)
{
	const float d = phys_dot(v1, v2);
	PHYS_ASSERT(fabsf(d) < 0.001f);
}

inline void PHYS_ASSERT_ORTHONORMAL(const phys_mat44 & m)
{
	const float nx = Abs(m.GetX());
	const float ny = Abs(m.GetY());
	const float nz = Abs(m.GetZ());
	const float dxy = phys_dot(m.GetX(), m.GetY());
	const float dxz = phys_dot(m.GetX(), m.GetZ());
	const float dyz = phys_dot(m.GetY(), m.GetZ());
	PHYS_ASSERT(fabsf(nx - 1.0f) < 0.001f);
	PHYS_ASSERT(fabsf(ny - 1.0f) < 0.001f);
	PHYS_ASSERT(fabsf(nz - 1.0f) < 0.001f);
	PHYS_ASSERT(fabsf(dxy) < 0.001f);
	PHYS_ASSERT(fabsf(dxz) < 0.001f);
	PHYS_ASSERT(fabsf(dyz) < 0.001f);
}

inline void PHYS_ASSERT_ALIGNED(const phys_vec3 * v)
{
	PHYS_ASSERT_STR((uintptr_t)v % TL_ALIGNOF(phys_vec3) == 0, "uint(v) % PHYS_ALIGNOF(phys_vec3) == 0"); // uintptr_t for x64
}

// NaN checks.
#define PHYS_ASSERT_VALID_VEC3(v) PHYS_ASSERT_MSG((v.GetX() == v.GetX() && v.GetY() == v.GetY() && v.GetZ() == v.GetZ()), "invalid vector")
#define PHYS_ASSERT_VALID_FLOAT(f) PHYS_ASSERT_MSG(f == f, "invalid float number")
