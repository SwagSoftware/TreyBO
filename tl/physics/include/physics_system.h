#pragma once

// The public static API of the rigid-body system. The host sets up the PMM arena with
// phys_memory_manager_init before phys_init().

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
	static rigid_body * create_rigid_body(const int no_error = false);
	static user_rigid_body * create_user_rigid_body(const int no_error = false);

	// Per constraint type:
	//   create_rbc_X(b1, b2, no_error)  validates both bodies; rbc_contact returns the existing pair constraint if any
	//   destroy(X *)                    no-op on NULL
	//   destroy_all_rbc_X()
	//   get_rbc_X_alloc_count()
#define RBC_DEF_TYPE(class_name, short_name)                                                                           \
	static class_name * create_##short_name(rigid_body * const b1, rigid_body * const b2, const int no_error = false); \
	static void destroy(class_name * const rbc);                                                                       \
	static void destroy_all_##short_name();                                                                            \
	static int get_##short_name##_alloc_count();
#include "tl/physics/include/rbc_defs/rbc_def_types.inc"

	static user_rigid_body * get_user_rigid_body(const phys_mat44 * const dictactor);
	static environment_rigid_body * get_environment_rigid_body();
	static rigid_body_constraint_contact * get_rbc_contact(rigid_body * const b1, rigid_body * const b2);

	static void destroy(rigid_body * const rb); // destroys its constraints first
	static void destroy(user_rigid_body * const rb);

	static void destroy_all_rigid_body(); // all constraints too
	static void destroy_all_user_rigid_body(); // and every constraint touching one

	static void destroy_all_constraint(rigid_body * const rb);
	static void destroy_all_constraint_with_user_rigid_body(rigid_body * const rb);
	static void destroy_all_unused_user_rigid_body();

	// Wheels resting on rb lose their contact (rb is about to go away).
	static void fixup_wheel_constraints(rigid_body * const rb);

	// Recounts m_constraint_count / m_contact_count of every body (generate_partitions_and_stuff does the same).
	static void update_constraint_infos();

	static int get_rigid_body_alloc_count();
	static int get_user_rigid_body_alloc_count();

	// Frame steps are split into ceil(delta_t / max_delta_t) equal sub-steps.
	static void set_max_delta_t(const float max_delta_t);
	static float get_max_delta_t();

	// Solver iteration counts (velocity pass / velocity+position pass).
	static void set_v_tol(const int max_v_iters);
	static void get_v_tol(int * max_v_iters);
	static void set_vp_tol(const int max_vp_iters);
	static void get_vp_tol(int * max_vp_iters);

	// Called once per sub-step between the collision prolog and the solver; runs the collision pass
	// (broad_phase_process ... broad_phase_reset_buffer).
	static void set_collision_callback(phys_collision_callback_t collision_callback);

	static void phys_frame_advance(float delta_t);

	static void phys_init();
	static void phys_shutdown();

	// Sets g_phys_proftimer_callbacks (phys_base.h).
	static void set_phys_proftimer_callbacks(const phys_proftimer_callbacks & ppc);
};
