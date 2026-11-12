#pragma once

// rbint: static helpers with the integration / collision-step internals of rigid bodies.

#include "tl/physics/include/rigid_body.h"

class pulse_sum_node;

class rbint
{
public:
	static void take_last_step(user_rigid_body * rb, const float delta_t);
	static void prolog_frame_advance(user_rigid_body * rb, const float delta_t);
	static void collision_prolog(user_rigid_body * rb, const float delta_t);
	static void take_next_step(user_rigid_body * rb, const float delta_t);
	static void collision_prolog(rigid_body * const rb, const float delta_t);
	static void substep(user_rigid_body * rb, const float delta_t);

	static const phys_vec3 sub_pos(rigid_body * const b, const phys_vec3 & p); // p - b's position
	static const phys_vec3 gtv(rigid_body * const b, const phys_vec3 & r); // velocity of point r
	static const phys_vec3 mul_L(const rigid_body * rb, const phys_vec3 & t); // world inertia * t
	static const phys_vec3 inv_L(const rigid_body * rb, const phys_vec3 & t, const float delta_t);

	static void update_stability(rigid_body * const rb, const float delta_t);
	static void solver_prolog(rigid_body * rb, const float delta_t);
	static void euler_integrate_velocity(rigid_body * const rb, const float delta_t);
	static void euler_integrate_pos(rigid_body * const rb, const float delta_t);
	static void add_vel(rigid_body * rb, const phys_vec3 & t, const phys_vec3 & a);

	static void setup_constraint(rigid_body * rb, pulse_sum_node * psn);
};
