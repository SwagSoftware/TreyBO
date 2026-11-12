#pragma once

// Every write of a position, velocity or force is followed by the debug range check (debug_check_vec).

#include "tl/base/include/tl_defs.h"
#include "tl/physics/include/phys_math.h"
#include "tl/physics/include/phys_assert.h"

class rigid_body;
class pulse_sum_node;
class rbint;
class rigid_body_constraint_point;
class rigid_body_constraint_hinge;
class rigid_body_constraint_distance;
class rigid_body_constraint_ragdoll;
class rigid_body_constraint_wheel;
class rigid_body_constraint_angular_actuator;
class rigid_body_constraint_upright;
class rigid_body_constraint_custom_orientation;
class rigid_body_constraint_custom_path;
class rigid_body_constraint_contact;

// In-place partition node: the first constraint of each type in the body's island, plus the island list
// links. Built by physics_system::generate_partitions_and_stuff.
class rb_inplace_partition_node
{
public:
	rigid_body_constraint_point * m_rbc_point_first;
	rigid_body_constraint_hinge * m_rbc_hinge_first;
	rigid_body_constraint_distance * m_rbc_dist_first;
	rigid_body_constraint_ragdoll * m_rbc_ragdoll_first;
	rigid_body_constraint_wheel * m_rbc_wheel_first;
	rigid_body_constraint_angular_actuator * m_rbc_angular_actuator_first;
	rigid_body_constraint_upright * m_rbc_upright_first;
	rigid_body_constraint_custom_orientation * m_rbc_custom_orientation_first;
	rigid_body_constraint_custom_path * m_rbc_custom_path_first;
	rigid_body_constraint_contact * m_rbc_contact_first;
	rigid_body * m_partition_head;
	rigid_body * m_partition_tail;
	rigid_body * m_next_node;
	int m_partition_size;
};

class rigid_body
{
public:
	friend class rbint;

	void set(const float mass, const phys_vec3 & inertia, const phys_mat44 & mat,
		const phys_vec3 & t_vel, const phys_vec3 & a_vel, const int stable_min_contact_count);

	void set_inertia(const phys_vec3 & inertia);
	void set_mass(const float mass);

	void set_max_avel(const float max_avel)
	{
		PHYS_ASSERT_VALID_FLOAT(max_avel);
		m_max_avel = max_avel;
	}

	enum rigid_body_flags_e
	{
		FLAG_DISABLE_FORCES = (1 << 0),
		FLAG_SPHERE_INERTIA = (1 << 1),
		FLAG_STABLE = (1 << 2),
		FLAG_GROUP_STABLE = (1 << 3),
		FLAG_ENVIRONMENT_RIGID_BODY = (1 << 4),
		FLAG_USER_RIGID_BODY = (1 << 5),

		// Set by the host; never tested here.
		DEBUG_FLAG_IN_COLLISION = (1 << 6),

		// Set by rbint::add_vel, tested by is_dangerous.
		FLAG_DANGEROUS = (1 << 7),

		FLAG_CLIENT_FLAGS_START = (1 << 8), // host-owned flags start here; tl flags must stay below
	};

	void set_flag(const unsigned int f, const int b)
	{
		PHYS_ASSERT_MSG((f & ~(FLAG_CLIENT_FLAGS_START - 1)) == 0, "invalid rigid body flag");
		b ? m_flags |= f : m_flags &= ~f;
	}
	const unsigned int get_flag(const unsigned int f) const
	{
		PHYS_ASSERT_MSG((f & ~(FLAG_CLIENT_FLAGS_START - 1)) == 0, "invalid rigid body flag");
		return m_flags & f;
	}

	const unsigned int is_environment_rigid_body() const { return get_flag(FLAG_ENVIRONMENT_RIGID_BODY); }
	const unsigned int is_user_rigid_body() const { return get_flag(FLAG_USER_RIGID_BODY); }
	const unsigned int is_stable() const { return get_flag(FLAG_STABLE); }
	const unsigned int is_group_stable() const { return get_flag(FLAG_GROUP_STABLE); }
	const unsigned int is_dangerous() const { return get_flag(FLAG_DANGEROUS); }

	const phys_mat44 & get_mat() const { return m_mat; }
	phys_mat44 & dangerous_get_mat() { return m_mat; }

	const phys_vec3 & get_t_vel() const { return m_t_vel; }
	const phys_vec3 & get_a_vel() const { return m_a_vel; }

	void dangerous_set_t_vel(const phys_vec3 & t_vel)
	{
		PHYS_ASSERT_VALID_VEC3(t_vel);
		m_t_vel = t_vel;
		debug_check_vec(m_t_vel);
	}
	void dangerous_set_a_vel(const phys_vec3 & a_vel)
	{
		PHYS_ASSERT_VALID_VEC3(a_vel);
		m_a_vel = a_vel;
		debug_check_vec(m_a_vel);
	}

	const phys_vec3 get_force() const { return m_force_sum; }
	const phys_vec3 get_torque() const { return m_torque_sum; }
	void set_force(const phys_vec3 & force) { m_force_sum = force; }
	void set_torque(const phys_vec3 & torque) { m_torque_sum = torque; }

	void add_force(const phys_vec3 & force)
	{
		PHYS_ASSERT_VALID_VEC3(force);
		m_force_sum += force;
		debug_check_vec(m_force_sum);
	}
	void add_torque(const phys_vec3 & torque)
	{
		PHYS_ASSERT_VALID_VEC3(torque);
		m_torque_sum += torque;
		debug_check_vec(m_torque_sum);
	}
	void add_force(const phys_vec3 & force, const phys_vec3 & point, const float torque_mult = 1.0f);

	const float get_max_delta_t() const { return m_max_delta_t; }
	void set_max_delta_t(const float max_delta_t) { m_max_delta_t = max_delta_t; }

	const float get_inv_mass() const { return m_inv_mass; }
	const phys_vec3 & get_inv_inertia() const { return m_inv_inertia; }

	const float get_stable_energy_time() const { return m_stable_energy_time; }

	const int get_constraint_count() const { return m_constraint_count; }
	const int get_contact_count() const { return m_contact_count; }

	// Collision sweep helpers.

	// Exchange the swept-to position with the last position.
	void swap_last_position()
	{
		const phys_vec3 last_position = m_last_position;
		m_last_position = m_mat.GetW();
		m_mat.GetW() = last_position;
		debug_check_vec(m_last_position);
		debug_check_vec(m_mat.GetW());
	}

	void update_last_position()
	{
		m_last_position += m_moved_vec * m_smallest_lambda;
		debug_check_vec(m_last_position);
	}

	// Advance by lambda of the remaining move, keep the rest.
	void adjust_col_moved_vec(const float lambda)
	{
		m_mat.GetW() = m_mat.GetW() + m_moved_vec * lambda;
		m_moved_vec = (1.0f - lambda) * m_moved_vec;
		debug_check_vec(m_moved_vec);
		debug_check_vec(m_mat.GetW());
	}

	// Runs after every write of a body position, velocity or force sum. Per component in x, y, z order: NaN
	// first, then |x| > PHYS_DEBUG_VEC_LIMIT; the first failure calls the debug callback once.
	void debug_check_vec(const phys_vec3 & v)
	{
		if (!(v.x == v.x) || fabsf(v.x) > PHYS_DEBUG_VEC_LIMIT ||
			!(v.y == v.y) || fabsf(v.y) > PHYS_DEBUG_VEC_LIMIT ||
			!(v.z == v.z) || fabsf(v.z) > PHYS_DEBUG_VEC_LIMIT)
			phys_exec_debug_callback(this);
	}

	// Members are public: constraint, solver, broad-phase and host code read and write them directly.
	phys_vec3 m_last_position;
	phys_vec3 m_moved_vec;
	float m_smallest_lambda;
	phys_mat44 m_mat;
	phys_vec3 m_inv_inertia;
	phys_vec3 m_gravity_acc_vec;
	phys_vec3 m_t_vel;
	phys_vec3 m_a_vel;
	phys_vec3 m_last_t_vel;
	phys_vec3 m_last_a_vel;
	phys_vec3 m_force_sum;
	phys_vec3 m_torque_sum;
	float m_inv_mass;
	float m_max_avel;
	float m_max_delta_t;
	unsigned int m_flags;
	unsigned int m_tick;
	pulse_sum_node * m_node;
	int m_constraint_count;
	int m_contact_count;
	int m_stable_min_contact_count;
	float m_stable_energy_time;
	float m_largest_vel_sq;
	float m_t_drag_coef;
	float m_a_drag_coef;
	void * m_userdata;
	rb_inplace_partition_node m_partition_node;
};

class user_rigid_body : public rigid_body
{
public:
	friend class rbint;

	void set(const phys_mat44 * const dictator);
	void setPosition(const phys_mat44 * const dictator);

	const phys_mat44 * get_dictator() const { return m_dictator; }

	// Public, see rigid_body.
	const phys_mat44 * m_dictator; // the driving matrix: caller-owned, m_dictator_mat, or m_mat
	phys_mat44 m_dictator_mat;
};

class environment_rigid_body : public rigid_body
{
public:
	friend class rbint;

	void set();
};
