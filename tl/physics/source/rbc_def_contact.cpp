#include "tl/physics/include/rbc_defs/rbc_def_contact.h"
#include "tl/physics/include/phys_mem.h"
#include "tl/physics/include/physics_system_internal.h"
#include "tl/physics/include/constraint_solver/pulse_sum_constraint_solver.h"

// Unlimited total, 2 reports per frame, warnings only.
phys_assert_info pai_create_cpi(0, 2, true);
phys_assert_info pai_create_pulse_sum_contact(0, 2, true);

phys_transient_allocator * contact_point_info::get_cpi_allocator()
{
	return &g_physics_system->m_contact_point_buffer_1;
}

void rigid_body_constraint_contact::verify_constraint(rigid_body * const b1_, rigid_body * const b2_)
{
	PHYS_ASSERT((b1 == b1_ && b2 == b2_) || (b1 == b2_ && b2 == b1_));
	g_physics_system->validate_member(b1_);
	g_physics_system->validate_member(b2_);
	PMM_VALIDATE(phys_free_list<rigid_body_constraint_contact>::get_T_internal(this),
		phys_free_list<rigid_body_constraint_contact>::get_slot_size(),
		phys_free_list<rigid_body_constraint_contact>::get_slot_alignment());
}

void rigid_body_constraint_contact::setup_constraint(pulse_sum_constraint_solver * psys, const float delta_t)
{
	m_list_contact_point_info_buffer_2.remove_all();
	for (contact_point_info * cpi = m_list_contact_point_info_buffer_1.get_first(); cpi; cpi = cpi->m_next_link)
	{
		cpi->check_surface_properties();
		psys->create_pulse_sum_contact(b1, b2, cpi, delta_t);
	}
}

rigid_body_constraint_contact::~rigid_body_constraint_contact()
{
	g_physics_system->m_search_tree_rbc_contact.remove(m_avl_key);
}

// A pulse_sum_contact followed by one pulse_sum_contact_point per point pair.
pulse_sum_contact * pulse_sum_constraint_solver::create_pulse_sum_contact(rigid_body * b1, rigid_body * b2, contact_point_info * cpi, float delta_t)
{
	pulse_sum_contact * const psc = (pulse_sum_contact *)m_solver_memory_allocator.allocate(
		(int)(sizeof(pulse_sum_contact_point) * cpi->m_point_pair_count + sizeof(pulse_sum_contact)), 16, true,
		SOLVER_MEMORY_ALLOCATOR_ERROR_MSG);
	if (psc)
	{
		m_list_pulse_sum_contact.add(psc);
		psc->m_list_pscp = (pulse_sum_contact_point *)(psc + 1);
		psc->m_list_pscp_count = cpi->m_point_pair_count;
		psc->set(b1, b2, cpi, delta_t);
		return psc;
	}
	PHYS_ASSERT_INFO_STR(pai_create_pulse_sum_contact, psc || cpi->get_flag(contact_point_info::FLAG_NO_OVERFLOW_ERROR),
		"psc || cpi->get_flag(contact_point_info::FLAG_NO_OVERFLOW_ERROR)", SOLVER_MEMORY_ALLOCATOR_ERROR_MSG);
	return NULL;
}

void contact_point_info::get_closest_psc(const phys_vec3 & normal, const phys_vec3 & b1_r_loc, const phys_vec3 & b2_r_loc,
	float * closest_error, const pulse_sum_cache_info ** closest_psc) const
{
	const phys_vec3 * list_b1_r_loc = m_list_b1_r_loc;
	const phys_vec3 * list_b2_r_loc = m_list_b2_r_loc;
	const float normal_error = AbsSquared(normal - m_normal);
	for (int i = 0; i < m_point_pair_count; i++)
	{
		const float error = AbsSquared(list_b1_r_loc[i] - b1_r_loc) + normal_error + AbsSquared(list_b2_r_loc[i] - b2_r_loc);
		if (*closest_error > error)
		{
			*closest_error = error;
			*closest_psc = &m_list_pulse_sum_cache_info[i];
		}
	}
}

void contact_point_info::set_closest_cached_psc(const contact_point_info * cached_cpi, const phys_vec3 & normal,
	const phys_vec3 & b1_r_loc, const phys_vec3 & b2_r_loc,
	pulse_sum_cache_info * psc)
{
	float closest_error = 10000000.0f;
	const pulse_sum_cache_info * closest_psc = NULL;
	for (const contact_point_info * cpi = cached_cpi; cpi; cpi = cpi->m_next_link)
		cpi->get_closest_psc(normal, b1_r_loc, b2_r_loc, &closest_error, &closest_psc);

	if (closest_psc)
	{
		*psc = *closest_psc;
	}
	else if (psc)
	{
		psc->m_ps_cache_list[PSC_N].m_pulse_sum = 0.0f;
		psc->m_ps_cache_list[PSC_F1].m_pulse_sum = 0.0f;
		psc->m_ps_cache_list[PSC_F2].m_pulse_sum = 0.0f;
	}
}

void contact_point_info::set_closest_cached_psc(const contact_point_info * cached_cpi)
{
	for (int i = 0; i < m_point_pair_count; i++)
		set_closest_cached_psc(cached_cpi, m_normal, m_list_b1_r_loc[i], m_list_b2_r_loc[i], &m_list_pulse_sum_cache_info[i]);
}
