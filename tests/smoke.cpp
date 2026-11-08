// TreyBO smoke test: start the library the way BO1's Phys_Init (0x847310) does, drop bodies under gravity, check
// they fall and stay finite.
//   1. a bare body (no geometry) falls freely for 120 steps of 1/60 s;
//   2. a box body (gjk_aabb_t, broad-phase info on the environment and sweep-and-prune lists, as BO1's
//      create_broad_phase_info(rigid_body *) 0x847dd0) falls onto a static ground box returned by a host
//      terrain query callback (the phys_environment.h contract) and must come to rest on it.
// Units and gravity follow BO1: z up, inches, m_gravity_acc_vec = phys_gravity (-800) * phys_gravity_dir (0,0,1),
// set after rigid_body::set as Phys_CreateBodyFromState (0x849210) does.

#include "tl/base/include/tl_system.h"
#include "tl/physics/include/phys_mem.h"
#include "tl/physics/include/physics_system.h"
#include "tl/physics/include/rigid_body.h"
#include "tl/physics/include/phys_util.h"
#include "tl/physics/include/collision/phys_environment.h"
#include "tl/physics/include/collision/phys_gjk_geom.h"
#include "tl/physics/include/rbc_defs/rbc_def_contact.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <malloc.h>

static void smoke_print(const char *s) { fputs(s, stdout); }
static void smoke_warning(const char *s) { printf("WARNING: %s\n", s); }
static void smoke_critical(const char *s)
{
    printf("CRITICAL: %s\n", s);
    fflush(stdout);
    exit(2);
}
static void *smoke_alloc(uint size, uint align, uint) { return _aligned_malloc(size ? size : 1, align < 16 ? 16 : align); }
static void *smoke_realloc(void *p, uint size, uint align, uint) { return _aligned_realloc(p, size ? size : 1, align < 16 ? 16 : align); }
static void smoke_free(void *p) { _aligned_free(p); }

// Persistent geoms: PMM pools, ids from the broad phase's unique id database (retail create() path).
class smoke_geom_allocator : public gjk_geom_allocator
{
public:
    void *allocate(const int, const int, const bool) override { return NULL; } // never a query allocator
    const phys_gjk_geom_id get_unique_id() override { return g_gjk_unique_id_database.get_unique_id(); }
};

// The static world: one ground box, top face at z = 0.
static gjk_base_t *g_ground_geom;
static phys_mat44 g_ground_cg_to_world;
static phys_mat44 g_ground_cg_to_rb;

enum
{
    SMOKE_ENV_FLAGS = 0x1C7, // BO1 create_broad_phase_info passes 0x1C7 for physics objects
    SMOKE_SURFACE_TYPE = 31, // BO1's default surface type (Phys_Init sets the 31/31 pair)
};

class smoke_terrain_query : public broad_phase_terrain_query_callback
{
public:
    void query(const broad_phase_environment_query_input &, broad_phase_environement_query_results *bpeqr) override
    {
        // A real host tests bpeqi's swept box against its world; the ground is big enough to always overlap.
        broad_phase_environment_info *const bpei = G_BPM->g_bpei_database.get_bpei_mt(bpei_database_id(1, 1));
        if (!bpei->m_data)
        {
            bpei->m_mutex.Lock();
            if (!bpei->m_data)
            {
                environment_rigid_body *const env = phys_sys::get_environment_rigid_body();
                broad_phase_info *const bpi_env = allocate_bpi_env();
                bpi_env->set(env, &env->get_mat(), &g_ground_cg_to_world, &g_ground_cg_to_rb, g_ground_geom,
                             bpei->m_gjk_geom_id, false, SMOKE_SURFACE_TYPE, NULL, SMOKE_ENV_FLAGS);
                bpi_env->set_bpi_env(NULL);
                bpi_env->collision_prolog();
                bpei->m_data = bpi_env;
            }
            bpei->m_mutex.Unlock();
        }
        bpeqr->add((broad_phase_base *)bpei->m_data);
    }
};

static bool is_finite3(const phys_vec3 &v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

// BO1 Phys_CreateBodyFromState: unit box inertia scaled by mass / volume, set(), then gravity and max angular vel.
static rigid_body *create_body(const phys_vec3 &pos, const phys_vec3 &dim, const float mass)
{
    rigid_body *const rb = phys_sys::create_rigid_body();
    if (!rb)
        return NULL;
    phys_vec3 inertia;
    float volume;
    nuge::calc_box_inertia(dim, &inertia, &volume);
    inertia = inertia * (mass / volume);
    phys_mat44 mat = PHYS_IDENTITY_MATRIX;
    mat.GetW() = pos;
    rb->set(mass, inertia, mat, PHYS_ZERO_VEC, PHYS_ZERO_VEC, 3);
    rb->m_gravity_acc_vec = phys_vec3(0.0f, 0.0f, -800.0f); // phys_gravity * phys_gravity_dir
    rb->m_max_avel = 16.0f;
    return rb;
}

// Verbose: the library default surface params, plus a dump of each contact's separation along its normal.
static void smoke_debug_cpi_params(contact_point_info *cpi, phys_collision_pair *pcp)
{
    const phys_surface_type_info &psti = surface_type_info_database_get(pcp->m_bpi1->m_surface_type, pcp->m_bpi2->m_surface_type);
    cpi->m_fric_coef = psti.m_friction_coef;
    cpi->m_bounce_coef = psti.m_bounce_coef;
    cpi->m_max_restitution_vel = 3400.0f;
    const rigid_body *const rb1 = pcp->m_bpi1->m_rb;
    const rigid_body *const rb2 = pcp->m_bpi2->m_rb;
    for (int i = 0; i < cpi->m_point_pair_count; ++i)
    {
        const phys_vec3 p1 = phys_multiply(rb1->m_mat, cpi->m_list_b1_r_loc[i]) + rb1->m_mat.GetW();
        const phys_vec3 p2 = phys_multiply(rb2->m_mat, cpi->m_list_b2_r_loc[i]) + rb2->m_mat.GetW();
        printf("    cpi pt %d: n (%.2f %.2f %.2f) p1 (%.3f %.3f %.3f) p2 (%.3f %.3f %.3f) sep %.4f lambda %.3f\n", i,
               cpi->m_normal.x, cpi->m_normal.y, cpi->m_normal.z, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z,
               phys_dot(cpi->m_normal, p1 - p2), cpi->m_translation_lambda);
    }
}

int main(int argc, char **argv)
{
    tlSystemCallbacks cb = {};
    cb.CriticalError = smoke_critical;
    cb.Warning = smoke_warning;
    cb.DebugPrint = smoke_print;
    cb.MemAlloc = smoke_alloc;
    cb.MemRealloc = smoke_realloc;
    cb.MemFree = smoke_free;
    cb.RunJobs = NULL; // serial
    tlSetSystemCallbacks(&cb);

    // BO1 allocates 0x380000 bytes; pointer-heavy pools roughly double on x64.
    const int mem_size = 0x380000 * (int)(sizeof(void *) / 4);
    void *const mem = malloc(mem_size);
    phys_memory_manager_init(mem, mem_size);
    phys_task_manager_init();
    phys_sys::phys_init();

    broad_phase_memory_info bpmi;
    bpmi.m_max_num_gjk_ci = 2048;
    bpmi.m_max_num_sap_active_pairs = 2048;
    bpmi.m_max_num_surface_types = 35;
    broad_phase_system_init(bpmi, NULL);

    smoke_terrain_query terrain_query;
    G_BPM->g_broad_phase_terrain_query_callback = &terrain_query;
    phys_surface_type_info pst;
    pst.m_friction_coef = 0.4f;
    pst.m_bounce_coef = 0.1f;
    pst.m_solver_priority = 0;
    pst.m_no_overflow_error = true;
    surface_type_info_database_set(SMOKE_SURFACE_TYPE, SMOKE_SURFACE_TYPE, pst);

    phys_sys::set_collision_callback(broad_phase_default_collision_callback);
    const bool verbose = argc > 1 && strcmp(argv[1], "-v") == 0; // -v: per-step output and a contact dump
    if (verbose)
        phys_set_cpi_params_callback(smoke_debug_cpi_params);
    phys_sys::set_max_delta_t(1.0f); // BO1 Phys_Init
    phys_sys::set_v_tol(8);

    smoke_geom_allocator geom_allocator;
    const float ground_mn[3] = {-1000.0f, -1000.0f, -64.0f};
    const float ground_mx[3] = {1000.0f, 1000.0f, 0.0f};
    g_ground_geom = create_aabb_gjk_geom(ground_mn, ground_mx, SMOKE_SURFACE_TYPE, &geom_allocator);
    g_ground_cg_to_world = PHYS_IDENTITY_MATRIX;
    g_ground_cg_to_rb = PHYS_IDENTITY_MATRIX;

    // 1. Free fall, far from the ground (x = 5000 is outside the ground box, no geometry anyway).
    rigid_body *const free_body = create_body(phys_vec3(5000.0f, 0.0f, 100.0f), phys_vec3(8.0f), 10.0f);

    // 2. A 16-unit cube whose bottom starts 50 units above the ground.
    const float half = 8.0f;
    rigid_body *const box_body = create_body(phys_vec3(0.0f, 0.0f, 50.0f + half), phys_vec3(2.0f * half), 10.0f);
    const float box_mn[3] = {-half, -half, -half};
    const float box_mx[3] = {half, half, half};
    gjk_base_t *const box_geom = create_aabb_gjk_geom(box_mn, box_mx, SMOKE_SURFACE_TYPE, &geom_allocator);
    if (!free_body || !box_body || !box_geom)
    {
        printf("FAIL: could not create the bodies\n");
        return 1;
    }
    phys_mat44 box_cg_to_world = PHYS_IDENTITY_MATRIX;
    phys_mat44 box_cg_to_rb = PHYS_IDENTITY_MATRIX;
    broad_phase_info *const box_bpi = create_broad_phase_info();
    box_bpi->set(box_body, &box_body->get_mat(), &box_cg_to_world, &box_cg_to_rb, box_geom, box_geom->get_geom_id(),
                 true, SMOKE_SURFACE_TYPE, NULL, SMOKE_ENV_FLAGS);
    environment_collision_list_add(box_bpi);
    aasap_list_add(box_bpi);

    // 3. A convex brush (cube hull, yawed 30 degrees) dropped onto the box: object/object collision through the
    //    sweep and prune, and the gjk_brush_t support path.
    static const float hull_verts[8][3] = {
        {-half, -half, -half}, {half, -half, -half}, {half, half, -half}, {-half, half, -half},
        {-half, -half, half},  {half, -half, half},  {half, half, half},  {-half, half, half},
    };
    phys_brush_desc hull_desc = {};
    for (int k = 0; k < 3; ++k)
    {
        hull_desc.mins[k] = -half;
        hull_desc.maxs[k] = half;
    }
    hull_desc.numsides = 6;
    hull_desc.numverts = 8;
    hull_desc.verts = hull_verts;
    rigid_body *const hull_body = create_body(phys_vec3(0.0f, 0.0f, 80.0f), phys_vec3(2.0f * half), 10.0f);
    gjk_base_t *const hull_geom = create_brush_gjk_geom(&hull_desc, SMOKE_SURFACE_TYPE, &geom_allocator);
    if (!hull_body || !hull_geom)
    {
        printf("FAIL: could not create the hull body\n");
        return 1;
    }
    {
        const float c = cosf(0.5235988f), s = sinf(0.5235988f);
        phys_mat44 &m = hull_body->dangerous_get_mat();
        m.x = phys_vec3(c, s, 0.0f);
        m.y = phys_vec3(-s, c, 0.0f);
    }
    phys_mat44 hull_cg_to_world = PHYS_IDENTITY_MATRIX;
    phys_mat44 hull_cg_to_rb = PHYS_IDENTITY_MATRIX;
    broad_phase_info *const hull_bpi = create_broad_phase_info();
    hull_bpi->set(hull_body, &hull_body->get_mat(), &hull_cg_to_world, &hull_cg_to_rb, hull_geom, hull_geom->get_geom_id(),
                  true, SMOKE_SURFACE_TYPE, NULL, SMOKE_ENV_FLAGS);
    environment_collision_list_add(hull_bpi);
    aasap_list_add(hull_bpi);

    const float free_z0 = free_body->get_mat().GetW().z;
    for (int i = 0; i < 120; ++i)
    {
        phys_sys::phys_frame_advance(1.0f / 60.0f);
        if (verbose ? (i >= 15 && i < 40) : (i % 20 == 19))
            printf("step %3d: free z = %10.3f  box z = %8.3f (vz %8.3f, contacts %d)  hull z = %8.3f (vz %8.3f, contacts %d)\n",
                   i + 1, free_body->get_mat().GetW().z, box_body->get_mat().GetW().z, box_body->get_t_vel().z,
                   box_body->get_contact_count(), hull_body->get_mat().GetW().z, hull_body->get_t_vel().z,
                   hull_body->get_contact_count());
    }

    const phys_vec3 free_pos = free_body->get_mat().GetW();
    const phys_vec3 box_pos = box_body->get_mat().GetW();
    printf("free body height: %.3f (start %.3f, ideal %.3f)\n", free_pos.z, free_z0, free_z0 - 0.5f * 800.0f * 4.0f);
    // Retail rests bodies on the inflated shapes: setup_gjk_input_from_pcp (0xa1cea0) adds a 0.34 skin to each geom.
    const float rest_z = half + 2.0f * 0.34f;
    printf("box body height:  %.3f (resting height %.3f: half size + 2 x 0.34 contact skin), stable %u\n", box_pos.z,
           rest_z, box_body->is_stable());

    int failed = 0;
    if (!is_finite3(free_pos) || !is_finite3(free_body->get_t_vel()))
    {
        printf("FAIL: free body is not finite\n");
        failed = 1;
    }
    else if (!(free_pos.z < free_z0 - 100.0f))
    {
        printf("FAIL: free body did not fall\n");
        failed = 1;
    }
    if (!is_finite3(box_pos) || !is_finite3(box_body->get_t_vel()))
    {
        printf("FAIL: box body is not finite\n");
        failed = 1;
    }
    else if (!(fabsf(box_pos.z - rest_z) < 0.1f))
    {
        printf("FAIL: box body is not resting on the ground\n");
        failed = 1;
    }
    const phys_vec3 hull_pos = hull_body->get_mat().GetW();
    const float hull_rest_z = rest_z + 2.0f * half + 2.0f * 0.34f;
    printf("hull body height: %.3f (resting on the box at %.3f), stable %u\n", hull_pos.z, hull_rest_z, hull_body->is_stable());
    if (!is_finite3(hull_pos) || !is_finite3(hull_body->get_t_vel()))
    {
        printf("FAIL: hull body is not finite\n");
        failed = 1;
    }
    else if (!(fabsf(hull_pos.z - hull_rest_z) < 0.5f))
    {
        printf("FAIL: hull body is not resting on the box\n");
        failed = 1;
    }

    printf(failed ? "TreyBO smoke: FAIL\n" : "TreyBO smoke: PASS\n");
    return failed;
}
