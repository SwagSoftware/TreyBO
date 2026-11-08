#pragma once

// Retail: codsrc\tl\physics\include\constraint_solver\pulse_sum_constraint_solver.h (assert :196).
// Inline bodies: pulse_sum_constraint_solver_inline.h and the per-row *_inline.h headers (included at the end).
// Out-of-line part: source/pulse_sum_constraint_solver.cpp (solver TU ~0xa25500-0xa2c990).
//
// Sequential-impulse ("pulse sum") solver, one island at a time. Each body of the island gets a
// pulse_sum_node holding its world inverse inertia and the velocity change accumulated this sub-step.
// Every constraint turns into rows (normal / point / angular / wheel / contact) whose accumulated impulse
// m_pulse_sum is warm started from the constraint's pulse_sum_cache (stored as impulse / delta_t), then:
//   1. velocity phase: projected Gauss-Seidel on the velocity error only;
//   2. caches <- velocity-phase impulses; the position-correction velocity m_big_dirt is added to each target;
//   3. vel+pos phase: more iterations on top of the same accumulated impulses;
//   4. node velocity changes are added to the bodies, positions integrated, stability updated.
// User rigid bodies (kinematic, "dictator" driven) never get a node: constraints that touch one are pointed at a
// solver-local copy (temp_user_rigid_body) that is stepped along its dictator path each sub-step.

#include "tl/base/include/tl_defs.h"
#include "tl/base/include/tl_system.h"
#include "tl/base/include/tl_thread.h"
#include "tl/physics/include/phys_base.h"
#include "tl/physics/include/phys_math.h"
#include "tl/physics/include/phys_util.h"
#include "tl/physics/include/phys_assert.h"
#include "tl/physics/include/phys_mem.h"
#include "tl/physics/include/phys_link_list.h"
#include "tl/physics/include/phys_avl_tree.h"
#include "tl/physics/include/phys_transient_allocator.h"
#include "tl/physics/include/rigid_body.h"
#include "tl/physics/include/rigid_body_internal.h"
#include "tl/physics/include/rbc_defs/rbc_def_base.h"
#include "tl/physics/include/constraint_solver/pulse_sum_cache.h"
#include "tl/physics/include/constraint_solver/pulse_sum_types.inc"
#include "tl/physics/include/constraint_solver/pulse_sum_normal.h"
#include "tl/physics/include/constraint_solver/pulse_sum_point.h"
#include "tl/physics/include/constraint_solver/pulse_sum_angular.h"
#include "tl/physics/include/constraint_solver/pulse_sum_wheel.h"
#include "tl/physics/include/constraint_solver/pulse_sum_contact.h"

class contact_point_info;
class physics_system;

// TREYBO: retail has these as rbint inlines in rigid_body_internal.h (assert text "rbint::get_pulse_sum_node(b1)",
// "rbint::verify_pulse_sum_node(b1)"); here they are free functions next to their only users.
// A body has a solver node exactly when it is neither an environment nor a user body.
inline pulse_sum_node *rbint_get_pulse_sum_node(const rigid_body *rb)
{
    return rb->m_node;
}

inline bool rbint_verify_pulse_sum_node(const rigid_body *rb)
{
    const unsigned int fixed_flags = rigid_body::FLAG_ENVIRONMENT_RIGID_BODY | rigid_body::FLAG_USER_RIGID_BODY;
    return rb->m_node ? (rb->m_flags & fixed_flags) == 0 : (rb->m_flags & fixed_flags) != 0;
}

// Per-body solver state for one sub-step (allocated per sub-step from the solver allocator).
class pulse_sum_node : public phys_link_list_base<pulse_sum_node>
{
public:
    phys_mat44 m_world_inv_inertia; // tensor_transform_principle(m_inv_inertia, m_mat) at setup
    phys_vec3 t_vel;                // linear velocity change accumulated by the rows
    phys_vec3 a_vel;                // angular velocity change accumulated by the rows
    float m_inv_mass;
    rigid_body *m_rb;
};

// Shared input of the solver jobs (allocated from the caller's transient buffer by constraint_solver_process).
// TREYBO: class/struct not recoverable (no mangled use); struct.
struct constraint_solver_task_input
{
    rigid_body **m_list_island;
    int m_list_island_count;
    volatile int *m_list_island_cur; // TREYBO: volatile (retail int *, bumped with InterlockedExchangeAdd)
    int m_psys_psc_visit_counter;      // IDB field; never written or read in BO1 (SM3 cache visit keys)
    int m_psys_next_psc_visit_counter; // IDB field; unused
    int m_psys_max_vel_iters;
    int m_psys_max_vel_pos_iters;
    float m_outside_delta_t;
};

class pulse_sum_constraint_solver
{
public:
    // TREYBO: class/struct not recoverable (no mangled use); struct.
    struct solver_info
    {
        int m_max_vel_iters;
        int m_max_vel_pos_iters;
        float m_max_vel_error_sq;
        float m_max_vel_pos_error_sq;
        float m_delta_t; // sub-step length
    };

    // Solver-local copy of a user rigid body: constraints are re-pointed at it for the solve and restored after.
    class TL_ALIGN(16) temp_user_rigid_body : public user_rigid_body, public phys_link_list_base<temp_user_rigid_body>
    {
    public:
        struct avl_tree_accessor
        {
            static user_rigid_body *&get_avl_key(temp_user_rigid_body *turb) { return turb->m_avl_key; }
            static user_rigid_body *const &get_avl_key(const temp_user_rigid_body *turb) { return turb->m_avl_key; }
            static phys_inplace_avl_tree_node<temp_user_rigid_body> *get_avl_node(temp_user_rigid_body *turb) { return &turb->m_avl_tree_node; }
            static const phys_inplace_avl_tree_node<temp_user_rigid_body> *get_avl_node(const temp_user_rigid_body *turb) { return &turb->m_avl_tree_node; }
        };

        user_rigid_body *m_avl_key; // the original user body
        phys_inplace_avl_tree_node<temp_user_rigid_body> m_avl_tree_node;
    };

    class user_rigid_body_restore_info : public phys_link_list_base<user_rigid_body_restore_info>
    {
    public:
        // TREYBO: rigid_body ** (IDB user_rigid_body **): it points at a constraint's b1 / b2 member, which is a
        // rigid_body *; writing through a user_rigid_body ** would alias the wrong pointer type.
        rigid_body **m_rbc_urb;
        user_rigid_body *m_original_urb;
    };

    // TREYBO: typedef names.
    typedef phys_inplace_avl_tree<user_rigid_body *, temp_user_rigid_body, temp_user_rigid_body::avl_tree_accessor> turb_search_tree_t;
    typedef phys_simple_link_list<temp_user_rigid_body> turb_list_t;
    typedef phys_simple_link_list<user_rigid_body_restore_info> urbri_list_t;

    // TREYBO: data members public; the rbc_def setup code allocates from m_solver_memory_allocator (wheel side
    // row) and appends to m_list_pulse_sum_contact (create_pulse_sum_contact).
    float m_outside_delta_t;
    int m_psys_max_vel_iters;
    int m_psys_max_vel_pos_iters;
    solver_info m_si;
    phys_transient_allocator m_solver_memory_allocator;
    phys_link_list<pulse_sum_node> m_list_pulse_sum_node;
    int m_memory_high_water;
    phys_link_list<pulse_sum_normal> m_list_pulse_sum_normal;
    phys_link_list<pulse_sum_point> m_list_pulse_sum_point;
    phys_link_list<pulse_sum_angular> m_list_pulse_sum_angular;
    phys_link_list<pulse_sum_wheel> m_list_pulse_sum_wheel;
    phys_link_list<pulse_sum_contact> m_list_pulse_sum_contact;

    // 0xa2b8b0: member constructors only. The destructor (0xa29800) is the implicit one (the allocator's
    // "m_first_block == NULL" check).
    pulse_sum_constraint_solver() {}

    // Row builders, called from the rbc_def setup_constraint bodies (out-of-line copies 0xa19050-0xa1a3c0 in
    // the rbc_def_generic / rbc_def_vehicle TUs).
    pulse_sum_normal *create_pulse_sum_normal(); // 0xa19050
    void create_point(rigid_body *const b1, const phys_vec3 &b1_r, rigid_body *const b2, const phys_vec3 &b2_r,
                      pulse_sum_cache *const ps_cache, const float delta_t, const bool is_spring, const float spring_k,
                      const float damp_k); // 0xa190b0
    pulse_sum_angular *create_pulse_sum_angular(rigid_body *const b1, const phys_vec3 &b1_r, rigid_body *const b2,
                                                const phys_vec3 &b2_r, const phys_vec3 &ud,
                                                pulse_sum_cache *const ps_cache); // 0xa19150
    void create_hinge(rigid_body *const b1, const phys_vec3 &b1_axis, rigid_body *const b2, const phys_vec3 &b2_axis,
                      const phys_vec3 &a1, const phys_vec3 &a2, pulse_sum_cache *const ps_cache,
                      const float delta_t); // 0xa191d0
    pulse_sum_wheel *create_pulse_sum_wheel(); // 0xa1a3c0

    // pulse_sum_constraint_solver.h:196 (0xa1a360). The side row must exist first (the wheel constraint
    // allocates it itself).
    pulse_sum_normal *create_pulse_sum_wheel_fwd(pulse_sum_wheel *psw)
    {
        PHYS_ASSERT(psw->m_side);
        psw->m_fwd = (pulse_sum_normal *)m_solver_memory_allocator.allocate((int)sizeof(pulse_sum_normal), 16, false,
                                                                            SOLVER_MEMORY_ALLOCATOR_ERROR_MSG);
        return psw->m_fwd;
    }

    // 0xa25330. Declared here, defined in source/rbc_def_contact.cpp (its asserts name that file).
    pulse_sum_contact *create_pulse_sum_contact(rigid_body *b1, rigid_body *b2, contact_point_info *cpi, float delta_t);

    void add_urb(turb_search_tree_t *turb_search_tree, turb_list_t *list_turb, urbri_list_t *list_urbri,
                 rigid_body_constraint *rbc);            // 0xa27e60
    void solve_iterative(int max_iters, float max_error_sq); // 0xa27d50
    void solve_constraints(rigid_body *const head);          // 0xa2b990
    void execute_constraint_solver(rigid_body *const head);  // 0xa2c1d0
};

// 0xa27cd0. Solve every island of psys (psys->m_list_island) through the job system and wait.
void constraint_solver_process(phys_transient_allocator *transient_buffer, physics_system *psys, float outside_delta_t);

// 0xa27f20 (retail: int phys_jq_constraint_solver_batch_function(jqBatch *)). TREYBO: tlJobFunc signature for
// tlRunJobs; user_data is the constraint_solver_task_input. Each job drains islands until none are left.
void phys_jq_constraint_solver_batch_function(void *user_data, int job_index);

extern volatile int g_list_island_cur; // ?g_list_island_cur@@3HA (TREYBO: volatile)

#include "tl/physics/include/constraint_solver/pulse_sum_normal_inline.h"
#include "tl/physics/include/constraint_solver/pulse_sum_point_inline.h"
#include "tl/physics/include/constraint_solver/pulse_sum_angular_inline.h"
#include "tl/physics/include/constraint_solver/pulse_sum_wheel_inline.h"
#include "tl/physics/include/constraint_solver/pulse_sum_contact_inline.h"
#include "tl/physics/include/constraint_solver/pulse_sum_constraint_solver_inline.h"
