#pragma once

// Retail: codsrc\tl\physics\include\phys_math.h (PHYS_ASSERT_UNIT 0xa17670, _ORTHOGONAL 0xa176f0,
// _ORTHONORMAL 0xa17750, PHYS_ASSERT_ALIGNED inlined, e.g. 0xa1b6c0). Includes old_phys_math.h,
// which holds the vector/matrix types and the math inlines (construct_orth_ud, make_rotate,
// Phys_GetQuaternion, ...), as SM3 and the BO1 assert paths show.

#include "tl/physics/include/old_phys_math.h"

// These are real (non-macro) functions in retail: out-of-line copies exist and they run in release.
inline void PHYS_ASSERT_UNIT(const phys_vec3 &v)
{
    const float nv = Abs(v);
    PHYS_ASSERT(fabsf(nv - 1.0f) < 0.001f);
}

inline void PHYS_ASSERT_ORTHOGONAL(const phys_vec3 &v1, const phys_vec3 &v2)
{
    const float d = phys_dot(v1, v2);
    PHYS_ASSERT(fabsf(d) < 0.001f);
}

inline void PHYS_ASSERT_ORTHONORMAL(const phys_mat44 &m)
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

inline void PHYS_ASSERT_ALIGNED(const phys_vec3 *v)
{
    PHYS_ASSERT_STR((uintptr_t)v % TL_ALIGNOF(phys_vec3) == 0, "uint(v) % PHYS_ALIGNOF(phys_vec3) == 0"); // TREYBO: uintptr_t for x64
}

// TREYBO: names are TreyBO's; retail expands an unknown macro to exactly these assert texts
// ("(v.GetX() == v.GetX() && ...)" / "invalid vector" at 0xa24a58, "f == f" / "invalid float number" at 0x8397d0).
#define PHYS_ASSERT_VALID_VEC3(v) PHYS_ASSERT_MSG((v.GetX() == v.GetX() && v.GetY() == v.GetY() && v.GetZ() == v.GetZ()), "invalid vector")
#define PHYS_ASSERT_VALID_FLOAT(f) PHYS_ASSERT_MSG(f == f, "invalid float number")
