#pragma once

// Retail: codsrc\tl\physics\include\rigid_body_internal.h (assert lines 31-124). Declares rbint, the
// static helper class with the integration / collision-step internals of rigid bodies.
//
// Retail defines these bodies inline in this header (COMDAT copies at the addresses below).
// TREYBO: here they are ordinary declarations. sub_pos and mul_L are defined in source/rigid_body.cpp;
// rbint::setup_constraint belongs to constraint_solver/pulse_sum_constraint_solver_inline.h (its asserts);
// the rest are defined in source/physics_system_internal.cpp (see docs/ledger/rigid_body.md).

#include "tl/physics/include/rigid_body.h"

class pulse_sum_node;

class rbint
{
public:
    static void take_last_step(user_rigid_body *rb, const float delta_t);        // 0xa10160 (rigid_body_internal.h:75)
    static void prolog_frame_advance(user_rigid_body *rb, const float delta_t);  // 0xa126a0 (:37)
    static void collision_prolog(user_rigid_body *rb, const float delta_t);      // 0xa12840 (:46)
    static void take_next_step(user_rigid_body *rb, const float delta_t);        // 0xa12970 (:65)
    static void collision_prolog(rigid_body *const rb, const float delta_t);     // 0xa12a60
    static void substep(user_rigid_body *rb, const float delta_t);               // 0xa29a20

    static const phys_vec3 sub_pos(rigid_body *const b, const phys_vec3 &p);              // 0xa179b0: p - b's position
    static const phys_vec3 gtv(rigid_body *const b, const phys_vec3 &r);                  // 0xa260c0: velocity of point r
    static const phys_vec3 mul_L(const rigid_body *rb, const phys_vec3 &t);               // 0xa2ce80: world inertia * t
    static const phys_vec3 inv_L(const rigid_body *rb, const phys_vec3 &t, const float delta_t); // 0xa25fb0

    static void update_stability(rigid_body *const rb, const float delta_t);         // 0xa26010
    static void solver_prolog(rigid_body *rb, const float delta_t);                  // 0xa29ac0
    static void euler_integrate_velocity(rigid_body *const rb, const float delta_t); // 0xa29c60 (:124)
    static void euler_integrate_pos(rigid_body *const rb, const float delta_t);      // 0xa2a3c0
    static void add_vel(rigid_body *rb, const phys_vec3 &t, const phys_vec3 &a);     // 0xa2a510

    static void setup_constraint(rigid_body *rb, pulse_sum_node *psn); // 0xa29770 (pulse_sum_constraint_solver_inline.h:123)
};
