#pragma once

// Retail: codsrc\tl\physics\include\collision\phys_gjk_sep_dir.h
// (bodies in source/phys_gjk_sep_dir.cpp 0xa249d0-0xa251f0).

#include "tl/physics/include/phys_math.h"

class phys_gjk_input;
class phys_gjk_info;

// Static helpers only (sizeof 1 in the IDB).
class gjk_sep_dir
{
public:
    // Initial GJK search direction from a 12-direction Minkowski sample.
    static const phys_vec3 comp_sep_dir(const phys_gjk_input *m_pcd, phys_gjk_info *m_gjk_info);
    // Closest point of segment v0-v1 to the origin, oriented away from mesh_center.
    static const float dist_seg(const phys_vec3 &v0, const phys_vec3 &v1, const phys_vec3 &mesh_center, phys_vec3 *direction);
};
