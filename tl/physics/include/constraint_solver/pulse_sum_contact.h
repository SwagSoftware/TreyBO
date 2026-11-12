#pragma once

// Contact rows. One pulse_sum_contact per contact manifold (contact_point_info): a shared normal and two
// friction directions, followed in memory by m_list_pscp_count pulse_sum_contact_point rows (see
// pulse_sum_constraint_solver::create_pulse_sum_contact in rbc_def_contact.cpp).
// Sign convention (as the code uses it): relative normal velocity is dot(n, v1 - v2) and a separating pair
// has it negative; get_pos = dot(n, p1 - p2) is positive while penetrating. b1 receives n * m_pulse_sum.x, so
// the normal impulse is kept <= 0 (it can only push the pair apart), and friction (y, z) is clamped to the
// cone |f| <= -m_fric_coef * m_pulse_sum.x.

#include "tl/base/include/tl_defs.h"
#include "tl/physics/include/phys_math.h"
#include "tl/physics/include/phys_assert.h"
#include "tl/physics/include/phys_mem.h"
#include "tl/physics/include/constraint_solver/pulse_sum_cache.h"
#include "tl/physics/include/constraint_solver/pulse_sum_types.inc"

class rigid_body;
class contact_point_info;

class TL_ALIGN(16) pulse_sum_contact_point
{
public:
	phys_vec3 m_b1_r;
	phys_vec3 m_b2_r; // the world point when psc->m_b2 == NULL
	phys_vec3 m_b1_ap_n; // b1 world inverse inertia * (r1 x n)
	phys_vec3 m_b2_ap_n; // same for b2; the object's point velocity when psc->m_b2 == NULL
	phys_vec3 m_b1_ap_f1;
	phys_vec3 m_b2_ap_f1;
	phys_vec3 m_b1_ap_f2;
	phys_vec3 m_b2_ap_f2;
	phys_vec3 m_pulse_sum; // (normal, f1, f2)
	phys_vec3 m_right_side; // target relative velocity in (normal, f1, f2)
	float m_big_dirt; // normal position-correction velocity for the vel+pos phase
	float m_denom_xx; // effective mass matrix K in (n, f1, f2); only the entries the solver needs
	float m_denom_yy;
	float m_denom_zz;
	float m_denom_xy;
	float m_denom_xz;
	float m_inv_yy; // inverse of the 2x2 friction block of K
	float m_inv_yz;
	float m_inv_zz;

	const phys_vec3 get_vel(pulse_sum_contact * psc);
	const float get_pos(pulse_sum_contact * psc);
	const float get_impact_vel(pulse_sum_contact * psc);
	void calc_abs(pulse_sum_contact * psc);
	const phys_vec3 get_objective(pulse_sum_contact * psc);
	void apply(pulse_sum_contact * psc, const phys_vec3 & s_);
	void clamp_f(pulse_sum_contact * psc);
	void SOLVER_solver_prolog(pulse_sum_contact * psc, pulse_sum_cache * m_pulse_sum_cache, const float delta_t);
	void SOLVER_apply_relaxation(pulse_sum_contact * psc, float & error_sq);
	void setup_vel_uni_restitution(pulse_sum_contact * psc, const float restitution_k, const float max_restitution_v,
		const float delta_t, const float max_penalty_restitution_vel,
		const float min_restitution_impact_vel_sq);

	void set_object_col_pt(pulse_sum_contact * psc, const phys_vec3 & object_col_pt);
};

class pulse_sum_contact : public phys_link_list_base<pulse_sum_contact>
{
	friend class pulse_sum_constraint_solver;

public:
	// Data members public: create_pulse_sum_contact (rbc_def_contact.cpp) sets m_list_pscp / _count.
	phys_vec3 m_ud_n; // contact normal (cpi->m_normal)
	phys_vec3 m_ud_f1;
	phys_vec3 m_ud_f2;
	float m_fric_coef;
	pulse_sum_node * m_b1;
	pulse_sum_node * m_b2;
	pulse_sum_contact_point * m_list_pscp;
	void * m_pulse_sum_cache_list; // cpi->m_list_pulse_sum_cache_info: NUM_PSC (3) caches per point
	int m_list_pscp_count;

	void set(rigid_body * const b1, rigid_body * const b2, contact_point_info * cpi, const float delta_t);

private:
	void SOLVER_solver_intermediate(const float delta_t);
};
