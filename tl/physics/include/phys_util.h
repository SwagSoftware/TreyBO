#pragma once

// Retail: codsrc\tl\physics\include\phys_util.h (bodies in source/phys_util.cpp 0xa1b6c0-0xa1cbc0;
// tensor_transform_principle inline, out-of-line copy 0xa25d10).

#include "tl/physics/include/phys_math.h"

class rigid_body;

class nuge
{
public:
    static void get_ballistic_info(rigid_body *const list_rigid_body[], const int rbodies_count, phys_vec3 *center_of_mass, phys_vec3 *total_momentum, float *total_mass);
    static void apply_ballistic_target(rigid_body *const list_rigid_body[], const int rbodies_count, const phys_vec3 &target, float *const dist_sq);

    static void calc_velocities(const phys_mat44 &mat0, const phys_mat44 &mat1, const float delta_t, phys_vec3 *t_vel, phys_vec3 *a_vel);
    static void calc_velocities(const phys_mat44 &mat0, const phys_mat44 &mat1, const phys_vec3 &center_offset_loc, const float delta_t, phys_vec3 *t_vel, phys_vec3 *a_vel);

    static void calc_sphere_inertia(const float radius, phys_vec3 *unit_inertia, float *volume);
    static void calc_box_inertia(const phys_vec3 &dim, phys_vec3 *unit_inertia, float *volume);
    static void calc_bound_sphere(const phys_vec3 *vert_list, const int vert_count, float *radius, phys_vec3 *com);
    static void calc_bound_box(const phys_vec3 *vert_list, const int vert_count, phys_vec3 *dim, phys_vec3 *com);

    // tensor = mat^T * diag(diag) * mat, row by row.
    static inline void tensor_transform_principle(const phys_vec3 &diag, const phys_mat44 &mat, phys_mat44 *tensor)
    {
        const phys_vec3 left_xrow = diag.GetX() * mat.GetX();
        const phys_vec3 left_yrow = diag.GetY() * mat.GetY();
        const phys_vec3 left_zrow = diag.GetZ() * mat.GetZ();

        tensor->SetX(mat.GetX().GetX() * left_xrow + mat.GetY().GetX() * left_yrow + mat.GetZ().GetX() * left_zrow);
        tensor->SetY(mat.GetX().GetY() * left_xrow + mat.GetY().GetY() * left_yrow + mat.GetZ().GetY() * left_zrow);
        tensor->SetZ(mat.GetX().GetZ() * left_xrow + mat.GetY().GetZ() * left_yrow + mat.GetZ().GetZ() * left_zrow);
    }
};
