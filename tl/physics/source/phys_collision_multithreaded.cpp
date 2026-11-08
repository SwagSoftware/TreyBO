// Retail: codsrc\tl\physics\source\phys_collision_multithreaded.cpp ("source/phys_collision_multithreaded.cpp" assert
// in process_cpi), TU 0xa2cc70-0xa2ce7b (COMDATs set_buffer / add_cpi_simple / manifold ctor /
// phys_collide_do_gjk_collide_and_contact_manifold just before it). Data: g_list_pcp_iterator 0xa8401b0,
// g_list_output_cpi 0xa8401b4, phys_gjk_collide_jqModule 0xe19ff8.
// TREYBO: job dispatch through phys_task_manager_process / tlRunJobs instead of jqModule / jqBatch.

#include "tl/physics/include/collision/phys_collision.h"
#include "tl/physics/include/collision/phys_task_manager.h"
#include "tl/physics/include/physics_system.h"
#include "tl/physics/include/physics_system_internal.h"
#include "tl/physics/include/rbc_defs/rbc_def_contact.h"

phys_collision_pair *volatile g_list_pcp_iterator;
contact_point_info *volatile g_list_output_cpi;

// Input of phys_gjk_collide_jq_batch_function. TREYBO: name not recovered; retail allocates these 12 bytes from the
// cpi allocator and fills them in reverse order (0xa2cd00).
struct phys_gjk_collide_task_input
{
    contact_point_info *volatile *m_list_output_cpi;
    // physics_system::m_search_tree_rbc_contact root, read-only while the jobs run
    rigid_body_constraint_contact *m_rbc_contact_search_tree_root;
    phys_collision_pair *volatile *m_list_pcp_iterator;
};

// 0xa2cc70. Hands every contact_point_info to its pair constraint, creating the constraint when the manifold
// found none in the search tree.
void process_cpi(contact_point_info *cpi)
{
    contact_point_info *next_cpi;
    for (; cpi; cpi = next_cpi)
    {
        next_cpi = cpi->m_next_link;
        PHYS_ASSERT(cpi->m_pcp);
        rigid_body *const b1 = cpi->m_pcp->m_bpi1->m_rb;
        rigid_body *const b2 = cpi->m_pcp->m_bpi2->m_rb;
        rigid_body_constraint_contact *rbc_contact = cpi->m_rbc_contact;
        if (rbc_contact || (rbc_contact = phys_sys::create_rbc_contact(b1, b2, cpi->get_flag(contact_point_info::FLAG_NO_OVERFLOW_ERROR))) != NULL)
        {
            rbc_contact->m_solver_priority = cpi->get_flag(contact_point_info::FLAG_SOLVER_PRIORITY_MASK);
            rbc_contact->add_cpi_simple(cpi, b1, b2);
        }
    }
}

// 0xa2cd00. Called by broad_phase_process with this pass's pairs.
void process_list_do_gjk_collide_and_contact_manifold(phys_link_list<phys_collision_pair> *list_pcd)
{
    if (list_pcd->m_alloc_count > 0)
    {
        g_list_pcp_iterator = list_pcd->m_first;
        g_list_output_cpi = NULL;
        // TREYBO: PHYS_ALIGNOF instead of retail's 4 (pointers need 8 on x64).
        phys_gjk_collide_task_input *const input = (phys_gjk_collide_task_input *)contact_point_info::get_cpi_allocator()->allocate(
            (int)sizeof(phys_gjk_collide_task_input), (int)PHYS_ALIGNOF(phys_gjk_collide_task_input), 0, "phys_transient_allocator out of memory.");
        input->m_list_pcp_iterator = &g_list_pcp_iterator;
        input->m_rbc_contact_search_tree_root = g_physics_system->m_search_tree_rbc_contact.m_tree_root;
        input->m_list_output_cpi = &g_list_output_cpi;
        phys_task_manager_process(phys_gjk_collide_jq_batch_function, input, list_pcd->m_alloc_count);
        phys_task_manager_flush();
        process_cpi(g_list_output_cpi);
    }
}

// 0xa2cd80 (retail module phys_gjk_collide_jqModule). Each job pops pairs off the shared list until it is empty,
// runs GJK + the contact manifold with its own scratch (about 17 KB of stack), then prepends its contact chain to
// the shared output list.
void phys_gjk_collide_jq_batch_function(void *input_, int /*job_index*/)
{
    phys_gjk_collide_task_input *const input = (phys_gjk_collide_task_input *)input_;
    phys_gjk_info gjk_info;
    phys_contact_manifold_process cman_process;
    cman_process.m_cpi_allocator = contact_point_info::get_cpi_allocator();
    cman_process.m_rbc_contact_search_tree_root = input->m_rbc_contact_search_tree_root;

    phys_collision_pair *volatile *const list_pcp_iterator = input->m_list_pcp_iterator;
    // TREYBO: retail reads the list head twice per step (test, then CAS operand); one load here.
    for (phys_collision_pair *pcp = tlAtomicLoad(list_pcp_iterator); pcp; pcp = tlAtomicLoad(list_pcp_iterator))
    {
        if (tlAtomicCompareExchange(list_pcp_iterator, pcp->m_next_link, pcp) == pcp)
            phys_collide_do_gjk_collide_and_contact_manifold(pcp, &gjk_info, &cman_process);
    }

    // TREYBO: retail phys_link_list inline (assert phys_mem.h:263 "m_last_next_ptr"), name not recovered:
    // splice the whole list in front of a lock-free list head.
    contact_point_info *const first = cman_process.m_list_cpi.m_first;
    if (first)
    {
        contact_point_info *old_first;
        do
            old_first = tlAtomicLoad(input->m_list_output_cpi);
        while (tlAtomicCompareExchange(input->m_list_output_cpi, first, old_first) != old_first);
        PHYS_ASSERT(cman_process.m_list_cpi.m_last_next_ptr);
        *cman_process.m_list_cpi.m_last_next_ptr = old_first;
    }
}
