// Retail: codsrc\tl\physics\source\phys_util.cpp (0xa1b6c0-0xa1cbc0). Optimized x87 TU; float
// operation order below follows the retail instruction stream.

#include "tl/physics/include/phys_util.h"
#include "tl/physics/include/rigid_body.h"

void nuge::get_ballistic_info(rigid_body *const list_rigid_body[], const int rbodies_count, phys_vec3 *center_of_mass, phys_vec3 *total_momentum, float *total_mass) // 0xa1b6c0
{
    PHYS_ASSERT_ALIGNED(center_of_mass);
    PHYS_ASSERT_ALIGNED(total_momentum);

    *center_of_mass = PHYS_ZERO_VEC;
    *total_momentum = PHYS_ZERO_VEC;
    *total_mass = 0.0f;
    for (int i = 0; i < rbodies_count; ++i)
    {
        const rigid_body *const rb = list_rigid_body[i];
        if (rb)
        {
            const float mass = 1.0f / rb->m_inv_mass;
            *center_of_mass += mass * rb->m_mat.GetW();
            *total_momentum += rb->m_t_vel * mass;
            *total_mass += mass;
        }
    }
    PHYS_ASSERT(*total_mass > 0.00001f);
    *center_of_mass *= 1.0f / *total_mass;
}

// Newton-solves for a launch velocity in the vertical plane through target that is closest to the
// group's current velocity, then pushes every body by a mass^2-weighted share of the difference.
void nuge::apply_ballistic_target(rigid_body *const list_rigid_body[], const int rbodies_count, const phys_vec3 &target, float *const dist_sq) // 0xa1bab0
{
    phys_vec3 center_of_mass;
    phys_vec3 total_momentum;
    float total_mass;
    get_ballistic_info(list_rigid_body, rbodies_count, &center_of_mass, &total_momentum, &total_mass);

    const phys_vec3 total_velocity = total_momentum / total_mass;
    const phys_vec3 delta_pos = target - center_of_mass;
    *dist_sq = AbsSquared(delta_pos);

    const float delta_x = sqrtf(delta_pos.x * delta_pos.x + delta_pos.z * delta_pos.z);
    if (delta_x < 0.001f)
        return;

    // Full 16-byte copy in retail (copy construction).
    const phys_vec3 gravity = list_rigid_body[0]->m_gravity_acc_vec;
    const float A = 0.5f * gravity.y * delta_x;
    const float B = delta_pos.y / delta_x;
    const phys_vec3 dir = phys_vec3(delta_pos.x, 0.0f, delta_pos.z) / delta_x;

    // Work in (along dir, up, unused) coordinates; s_ is the horizontal launch speed.
    const phys_vec3 vt(phys_dot(dir, total_velocity), total_velocity.y, 0.0f);
    float s_ = 1.0f;
    int iter = 0;
    bool converged = false;
    do
    {
        const phys_vec3 vs(s_, s_ * B - A / s_, 0.0f);
        const phys_vec3 dvs(1.0f, B + A / (s_ * s_), 0.0f);
        const phys_vec3 ddvs(0.0f, -2.0f * A / (s_ * s_ * s_), 0.0f);
        const phys_vec3 dif = vs - vt;
        const float s1_ = s_ - phys_dot(dif, dvs) / (phys_dot(dif, ddvs) + phys_dot(dvs, dvs));
        if (fabsf(s1_ - s_) < 0.001f)
            converged = true;
        ++iter;
        s_ = s1_;
    } while (iter <= 10 && !converged);

    if (fabsf(s_) < 1.0f && fabsf(s_) > 0.001f)
        s_ = s_ < 0.0f ? -1.0f : 1.0f;

    const phys_vec3 velocity = dir * s_ + phys_vec3(0.0f, s_ * B - A / s_, 0.0f);

    float total_square_mass = 0.0f;
    for (int i = 0; i < rbodies_count; ++i)
    {
        if (list_rigid_body[i])
            total_square_mass += 1.0f / phys_sqr(list_rigid_body[i]->m_inv_mass);
    }
    PHYS_ASSERT(total_square_mass > 0.00001f);

    const phys_vec3 delta_vel = (velocity - total_velocity) * (total_mass / total_square_mass);
    for (int i = 0; i < rbodies_count; ++i)
    {
        rigid_body *const rb = list_rigid_body[i];
        if (rb)
            rb->add_force(delta_vel * (1.0f / phys_sqr(rb->m_inv_mass)));
    }
}

void nuge::calc_velocities(const phys_mat44 &mat0, const phys_mat44 &mat1, const float delta_t, phys_vec3 *t_vel, phys_vec3 *a_vel) // 0xa1bfb0
{
    PHYS_ASSERT_ALIGNED(t_vel);
    PHYS_ASSERT_ALIGNED(a_vel);

    const float oo_delta_t = 1.0f / delta_t;

    phys_mat44 vel_mat;
    phys_transpose(vel_mat, mat0);
    phys_multiply_mat(vel_mat, mat1, vel_mat);

    const Phys_UnitQuaternion qvel = Phys_GetQuaternion(vel_mat);
    const float theta = phys_acosf_safe(qvel.GetW());
    const float nqvec = Abs(qvel.GetImag());
    if (nqvec > PHYS_EPS_1E4)
        *a_vel = (oo_delta_t / nqvec * (theta + theta)) * qvel.GetImag();
    else
        *a_vel = PHYS_ZERO_VEC;

    *t_vel = (mat1.GetW() - mat0.GetW()) * oo_delta_t;
}

void nuge::calc_velocities(const phys_mat44 &mat0, const phys_mat44 &mat1, const phys_vec3 &center_offset_loc, const float delta_t, phys_vec3 *t_vel, phys_vec3 *a_vel) // 0xa1c190
{
    PHYS_ASSERT_ALIGNED(t_vel);
    PHYS_ASSERT_ALIGNED(a_vel);

    calc_velocities(mat0, mat1, delta_t, t_vel, a_vel);
    *t_vel += phys_cross(*a_vel, phys_multiply(mat1, center_offset_loc));
}

void nuge::calc_sphere_inertia(const float radius, phys_vec3 *unit_inertia, float *volume) // 0xa1c280
{
    PHYS_ASSERT_ALIGNED(unit_inertia);

    // Retail literal is 3.1415925f (0x40490FDA), one ulp below PHYS_PI_.
    *volume = 3.1415925f * radius * radius * radius * 4.0f / 3.0f;
    const float d = 2.0f * *volume * radius * radius / 5.0f;
    *unit_inertia = phys_vec3(d);
}

void nuge::calc_box_inertia(const phys_vec3 &dim, phys_vec3 *unit_inertia, float *volume) // 0xa1c310
{
    PHYS_ASSERT_ALIGNED(unit_inertia);

    *volume = dim.x * dim.y * dim.z;
    const phys_vec3 dim_sq(dim.x * dim.x, dim.y * dim.y, dim.z * dim.z);
    const float k = dim.x * (1.0f / 12.0f) * dim.y * dim.z;
    const phys_vec3 moments = k * dim_sq;
    *unit_inertia = phys_vec3(moments.y + moments.z, moments.z + moments.x, moments.x + moments.y);
}

void nuge::calc_bound_sphere(const phys_vec3 *vert_list, const int vert_count, float *radius, phys_vec3 *com) // 0xa1c400
{
    PHYS_ASSERT_ALIGNED(vert_list);
    PHYS_ASSERT_ALIGNED(com);

    phys_vec3 dir3_com = PHYS_ZERO_VEC;
    for (int i = 0; i < vert_count; ++i)
        dir3_com += vert_list[i];
    dir3_com /= (float)vert_count;

    *radius = 0.0f;
    for (int i = 0; i < vert_count; ++i)
    {
        const float r = AbsSquared(vert_list[i] - dir3_com);
        if (*radius < r)
            *radius = r;
    }
    *radius = sqrtf(*radius);
    *com = dir3_com;
}

void nuge::calc_bound_box(const phys_vec3 *vert_list, const int vert_count, phys_vec3 *dim, phys_vec3 *com) // 0xa1c780
{
    PHYS_ASSERT_ALIGNED(vert_list);
    PHYS_ASSERT_ALIGNED(dim);
    PHYS_ASSERT_ALIGNED(com);

    phys_vec3 vmin(10000000.0f);
    phys_vec3 vmax(-10000000.0f);
    // TREYBO FIX(0xa1c862 / 0xa1c87d): retail inlines phys_min / phys_max here as x87 compares that keep the bound
    // when v is NaN, unlike the out-of-line copies (NaN -> v), so the components are compared directly.
    for (int i = 0; i < vert_count; ++i)
    {
        const phys_vec3 v = vert_list[i];
        for (int j = 0; j < 3; ++j)
        {
            if (vmin[j] > v[j])
                vmin[j] = v[j];
            if (vmax[j] < v[j])
                vmax[j] = v[j];
        }
    }
    *com = 0.5f * (vmax + vmin);
    *dim = vmax - vmin;
}
