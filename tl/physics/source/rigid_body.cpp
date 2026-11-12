#include "tl/physics/include/rigid_body.h"
#include "tl/physics/include/rigid_body_internal.h"
#include "tl/physics/include/physics_system_internal.h"

void rigid_body::set_inertia(const phys_vec3 & inertia)
{
	PHYS_ASSERT_VALID_VEC3(inertia);
	PHYS_ASSERT(inertia.GetX() > 0.000001f);
	PHYS_ASSERT(inertia.GetY() > 0.000001f);
	PHYS_ASSERT(inertia.GetZ() > 0.000001f);
	m_inv_inertia = phys_Inv(inertia);
}

void rigid_body::set_mass(const float mass)
{
	PHYS_ASSERT_VALID_FLOAT(mass);
	PHYS_ASSERT(mass > 0.000001f);
	m_inv_mass = 1.0f / mass;
}

// Drive the body from an internal copy of dictator.
void user_rigid_body::setPosition(const phys_mat44 * const dictator)
{
	m_dictator = &m_dictator_mat;
	m_dictator_mat = *dictator;
}

void rigid_body::set(const float mass, const phys_vec3 & inertia, const phys_mat44 & mat,
	const phys_vec3 & t_vel, const phys_vec3 & a_vel, const int stable_min_contact_count)
{
	PHYS_ASSERT_VALID_FLOAT(mass);
	PHYS_ASSERT_VALID_VEC3(inertia);
	PHYS_ASSERT_VALID_VEC3(t_vel);
	PHYS_ASSERT_VALID_VEC3(a_vel);

	set_mass(mass);
	set_inertia(inertia);

	m_mat = mat;
	debug_check_vec(m_mat.GetW());
	m_last_position = m_mat.GetW();
	debug_check_vec(m_last_position);
	m_moved_vec = PHYS_ZERO_VEC;
	m_smallest_lambda = 0.0f;

	m_t_vel = t_vel;
	m_a_vel = a_vel;
	debug_check_vec(m_t_vel);
	debug_check_vec(m_a_vel);

	m_stable_min_contact_count = stable_min_contact_count;
	m_force_sum = PHYS_ZERO_VEC;
	m_torque_sum = PHYS_ZERO_VEC;

	m_last_t_vel = m_t_vel;
	m_last_a_vel = m_a_vel;
	debug_check_vec(m_last_t_vel);
	debug_check_vec(m_last_a_vel);

	m_gravity_acc_vec = phys_vec3(0.0f, -9.8f, 0.0f);
	m_flags = 0;
	m_tick = 0;
	m_max_delta_t = 0.033898305f; // 1 / 29.5
	m_userdata = NULL;
	m_max_avel = 1000.0f;
	m_largest_vel_sq = 1000.0f;
	m_stable_energy_time = 0.0f;
	m_t_drag_coef = 0.0f;
	m_a_drag_coef = 0.0f;

	// A body created during the collision phase of a step joins it immediately.
	if (g_physics_system->m_flags & physics_system::FLAG_IN_COLLISION_STEP)
		rbint::collision_prolog(this, g_physics_system->m_outside_sub_delta_t);

	PHYS_ASSERT_ORTHONORMAL(m_mat);
}

void rigid_body::add_force(const phys_vec3 & force, const phys_vec3 & point, const float torque_mult)
{
	PHYS_ASSERT_VALID_VEC3(force);
	PHYS_ASSERT_VALID_VEC3(point);
	PHYS_ASSERT_VALID_FLOAT(torque_mult);

	m_force_sum += force;
	m_torque_sum += phys_cross(point - m_mat.GetW(), force) * torque_mult;
	debug_check_vec(m_force_sum);
	debug_check_vec(m_torque_sum);
}

// dictator == NULL: the body drives itself from m_mat.
void user_rigid_body::set(const phys_mat44 * const dictator)
{
	if (dictator)
	{
		m_mat = *dictator;
		m_dictator = dictator;
	}
	else
	{
		m_dictator = &m_mat;
	}
	debug_check_vec(m_mat.GetW());
	m_last_position = m_mat.GetW();
	debug_check_vec(m_last_position);
	m_moved_vec = PHYS_ZERO_VEC;
	m_flags = 0;
	m_smallest_lambda = 0.0f;
	m_inv_mass = 0.0f;
	m_inv_inertia = PHYS_ZERO_VEC;
	m_t_vel = PHYS_ZERO_VEC;
	m_a_vel = PHYS_ZERO_VEC;
	m_last_t_vel = PHYS_ZERO_VEC;
	m_last_a_vel = PHYS_ZERO_VEC;
	m_gravity_acc_vec = PHYS_ZERO_VEC;
	m_flags |= FLAG_USER_RIGID_BODY;
	m_max_delta_t = 0.033898305f;
	m_node = NULL;
	m_userdata = NULL;

	if (g_physics_system->m_flags & physics_system::FLAG_IN_COLLISION_STEP)
		rbint::collision_prolog(this, g_physics_system->m_outside_sub_delta_t);
}

void environment_rigid_body::set()
{
	m_inv_mass = 0.0f;
	m_flags = 0;
	m_inv_inertia = PHYS_ZERO_VEC;
	SetIdentity(m_mat);
	m_last_position = m_mat.GetW();
	debug_check_vec(m_last_position);
	m_moved_vec = PHYS_ZERO_VEC;
	m_smallest_lambda = 0.0f;
	m_t_vel = PHYS_ZERO_VEC;
	m_a_vel = PHYS_ZERO_VEC;
	m_last_t_vel = PHYS_ZERO_VEC;
	m_last_a_vel = PHYS_ZERO_VEC;
	m_gravity_acc_vec = PHYS_ZERO_VEC;
	m_flags |= FLAG_ENVIRONMENT_RIGID_BODY;
	m_max_delta_t = 0.033898305f;
	m_node = NULL;
	m_userdata = NULL;
}

// ---------------------------------------------------------------------------------------------
// rbint (rigid_body_internal.h)
// ---------------------------------------------------------------------------------------------

const phys_vec3 rbint::sub_pos(rigid_body * const b, const phys_vec3 & p)
{
	return p - b->m_mat.GetW();
}

// World inertia tensor (R * diag(1 / inv_inertia) * R^T) applied to t.
const phys_vec3 rbint::mul_L(const rigid_body * rb, const phys_vec3 & t)
{
	return phys_multiply(rb->m_mat, phys_inv_diag_multiply(rb->m_inv_inertia, phys_inv_multiply(rb->m_mat, t)));
}
