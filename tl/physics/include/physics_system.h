#pragma once

// Retail: codsrc\tl\physics\include\physics_system.h. Out-of-line part: source/physics_system.cpp
// (phys_sys 0xa14160-0xa16680, 66 functions, plus the create/destroy templates up to 0xa17430).
//
// The public static API of the rigid-body system. Layout follows SM3 physics_system.h; BO1 differences:
//  - phys_init() takes no phys_mem_info (the PMM arena is set up by the host with phys_memory_manager_init);
//    the error strings still name phys_mem_info::m_num_*, but BO1 has no such type.
//  - set/get_v_tol and set/get_vp_tol carry iteration counts only (SM3 also had sub-iterations and error).
//  - no max_slots / available_slots / used_slots; BO1 has get_*_alloc_count instead.
//  - 10 constraint types (rbc_def_types.inc), destroy_all_unused_user_rigid_body, fixup_wheel_constraints,
//    get_rbc_contact.
//  - no_error is an int (mangled H), SM3 a bool.

#include "tl/base/include/tl_defs.h"
#include "tl/physics/include/phys_base.h"
#include "tl/physics/include/phys_math.h"

class rigid_body;
class user_rigid_body;
class environment_rigid_body;

#define RBC_DEF_TYPE(class_name, short_name) class class_name;
#include "tl/physics/include/rbc_defs/rbc_def_types.inc"

typedef void (*phys_collision_callback_t)();

class phys_sys
{
public:
    static rigid_body *create_rigid_body(const int no_error = false);           // 0xa14740
    static user_rigid_body *create_user_rigid_body(const int no_error = false); // 0xa14c40

    // Per constraint type (SM3 expands rbc_def_types.inc here):
    //   create_rbc_X(b1, b2, no_error)  validates both bodies; rbc_contact returns the existing pair constraint if any
    //   destroy(X *)                    no-op on NULL
    //   destroy_all_rbc_X()
    //   get_rbc_X_alloc_count()
#define RBC_DEF_TYPE(class_name, short_name)                                                                     \
    static class_name *create_##short_name(rigid_body *const b1, rigid_body *const b2, const int no_error = false); \
    static void destroy(class_name *const rbc);                                                                  \
    static void destroy_all_##short_name();                                                                      \
    static int get_##short_name##_alloc_count();
#include "tl/physics/include/rbc_defs/rbc_def_types.inc"

    static user_rigid_body *get_user_rigid_body(const phys_mat44 *const dictactor); // 0xa14330
    static environment_rigid_body *get_environment_rigid_body();                     // 0xa14160
    static rigid_body_constraint_contact *get_rbc_contact(rigid_body *const b1, rigid_body *const b2); // 0xa143a0

    static void destroy(rigid_body *const rb);      // 0xa16640 (destroys its constraints first)
    static void destroy(user_rigid_body *const rb); // 0xa16680

    static void destroy_all_rigid_body();      // 0xa14ed0 (all constraints too)
    static void destroy_all_user_rigid_body(); // 0xa14f90 (and every constraint touching one)

    static void destroy_all_constraint(rigid_body *const rb);                     // 0xa156f0
    static void destroy_all_constraint_with_user_rigid_body(rigid_body *const rb); // 0xa15e10
    static void destroy_all_unused_user_rigid_body();                              // 0xa14b90

    // Wheels resting on rb lose their contact (rb is about to go away).
    static void fixup_wheel_constraints(rigid_body *const rb); // 0xa143f0

    // Recounts m_constraint_count / m_contact_count of every body (generate_partitions_and_stuff does the same).
    static void update_constraint_infos(); // 0xa14440

    static int get_rigid_body_alloc_count();      // 0xa14270
    static int get_user_rigid_body_alloc_count(); // 0xa14280

    // Frame steps are split into ceil(delta_t / max_delta_t) equal sub-steps.
    static void set_max_delta_t(const float max_delta_t); // 0xa14170
    static float get_max_delta_t();                       // 0xa14180

    // BO1: solver iteration counts (velocity pass / velocity+position pass).
    static void set_v_tol(const int max_v_iters);   // 0xa14190
    static void get_v_tol(int *max_v_iters);        // 0xa141b0
    static void set_vp_tol(const int max_vp_iters); // 0xa141d0
    static void get_vp_tol(int *max_vp_iters);      // 0xa141f0

    // Called once per sub-step between the collision prolog and the solver; runs the collision pass
    // (BO1's Phys_CollisionCallback: broad_phase_process ... broad_phase_reset_buffer).
    static void set_collision_callback(phys_collision_callback_t collision_callback); // 0xa14210

    static void phys_frame_advance(float delta_t); // 0xa14230

    static void phys_init();     // 0xa14250
    static void phys_shutdown(); // 0xa14260

    // TREYBO: SM3 API (no BO1 symbol); sets g_phys_proftimer_callbacks (phys_base.h).
    static void set_phys_proftimer_callbacks(const phys_proftimer_callbacks &ppc);
};
