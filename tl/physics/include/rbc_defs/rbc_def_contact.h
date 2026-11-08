#pragma once

// Retail: codsrc\tl\physics\include\rbc_defs\rbc_def_contact.h (assert lines 78-414).
// Out-of-line part: source/rbc_def_contact.cpp (TU 0xa251f0-0xa2542f; contact_point_info cache matching
// 0xa2d840-0xa2e072).
//
// BO1 contact model (replaces SM3's per-point contact_point_info list): the collision pass allocates one
// contact_point_info per manifold from a transient buffer, followed by its point arrays and pulse-sum caches.
// Each body pair owns one rigid_body_constraint_contact, found through an in-place AVL tree keyed by the pair.

#include "tl/base/include/tl_defs.h"
#include "tl/physics/include/phys_math.h"
#include "tl/physics/include/phys_assert.h"
#include "tl/physics/include/phys_avl_tree.h"
#include "tl/physics/include/phys_link_list.h"
#include "tl/physics/include/phys_transient_allocator.h"
#include "tl/physics/include/rigid_body.h"
#include "tl/physics/include/rbc_defs/rbc_def_base.h"
#include "tl/physics/include/constraint_solver/pulse_sum_cache.h"

class phys_collision_pair;
class rigid_body_constraint_contact;

extern phys_assert_info pai_create_cpi; // defined in source/rbc_def_contact.cpp: (0, 2, true)

class TL_ALIGN(16) contact_point_info
{
public:
    enum ps_cache_e
    {
        PSC_N,
        PSC_F1,
        PSC_F2,
        NUM_PSC
    };

    struct pulse_sum_cache_info
    {
        pulse_sum_cache m_ps_cache_list[NUM_PSC];
    };

    // FLAG_SOLVER_PRIORITY_MASK / FLAG_NO_OVERFLOW_ERROR: retail assert text; values from
    // set_solver_priority 0x8381a0 and create_pulse_sum_contact 0xa25330.
    enum flags_e
    {
        FLAG_SOLVER_PRIORITY_MASK = 0x7,
        FLAG_NO_OVERFLOW_ERROR = 0x8,
        FLAG_HAS_RB2_ENTITY = 0x10, // TREYBO: name not recovered (set_rb2_entity 0x838200)
    };

    phys_vec3 m_normal;
    float m_fric_coef;
    float m_bounce_coef;
    float m_max_restitution_vel;
    int m_flags;
    int m_point_pair_count;
    phys_vec3 *m_list_b1_r_loc; // m_point_pair_count entries, right after this header
    phys_vec3 *m_list_b2_r_loc;
    pulse_sum_cache_info *m_list_pulse_sum_cache_info;
    contact_point_info *m_next_link;
    const void *m_rb2_entity; // host entity of body 2; (const void *)-1 until set
    float m_translation_lambda;
    phys_collision_pair *m_pcp;
    rigid_body_constraint_contact *m_rbc_contact;

    const int get_flag(const int f) const { return m_flags & f; } // retail assert text "cpi->get_flag(...)"

    // rbc_def_contact.h:78 (0x8381a0)
    void set_solver_priority(const unsigned int priority)
    {
        PHYS_ASSERT((priority & FLAG_SOLVER_PRIORITY_MASK) == priority);
        m_flags &= ~FLAG_SOLVER_PRIORITY_MASK;
        m_flags |= priority;
    }

    // 0x838200
    void set_rb2_entity(const void *rb2_entity)
    {
        m_flags |= FLAG_HAS_RB2_ENTITY;
        m_rb2_entity = rb2_entity;
    }

    // rbc_def_contact.h:137-139 (0x838260)
    void check_surface_properties()
    {
        PHYS_ASSERT(m_fric_coef >= 0.0f);
        PHYS_ASSERT(m_bounce_coef <= 1.0f && m_bounce_coef >= 0.0f);
        PHYS_ASSERT(m_max_restitution_vel >= 0.0f);
    }

    static phys_transient_allocator *get_cpi_allocator(); // 0xa251f0

    // rbc_def_contact.h:159 / :173 (0xa2ddf0). One block: header, b1 points, b2 points, then (16-aligned) caches.
    // Surface properties start at -1 (unset); the caller fills normal, points and the rest.
    static contact_point_info *create_cpi(const int point_pair_count, const bool no_error, phys_transient_allocator *allocator)
    {
        PHYS_ASSERT(point_pair_count > 0);
        // retail: (32 * n + 95) & ~15 with the x86 header size 0x50
        const int psc_offset = (int)PHYS_ALIGN(sizeof(contact_point_info) + 2 * sizeof(phys_vec3) * point_pair_count, 16);
        const int size = psc_offset + (int)PHYS_ALIGN(sizeof(pulse_sum_cache_info) * point_pair_count, 16);
        contact_point_info *const cpi = (contact_point_info *)allocator->mt_allocate(size, 16, true, "contact_point_info buffer overflow");
        if (cpi)
        {
            cpi->m_fric_coef = -1.0f;
            cpi->m_bounce_coef = -1.0f;
            cpi->m_max_restitution_vel = -1.0f;
            cpi->m_list_b1_r_loc = (phys_vec3 *)(cpi + 1);
            cpi->m_list_b2_r_loc = cpi->m_list_b1_r_loc + point_pair_count;
            cpi->m_list_pulse_sum_cache_info = (pulse_sum_cache_info *)((char *)cpi + psc_offset);
            cpi->m_point_pair_count = point_pair_count;
            cpi->m_flags = 0;
            cpi->m_rb2_entity = (const void *)(intptr_t)-1;
            return cpi;
        }
        PHYS_ASSERT_INFO(pai_create_cpi, cpi || no_error, "contact_point_info buffer overflow");
        return NULL;
    }

    // Warm start: give each new point the caches of the closest point of last frame's manifolds.
    void get_closest_psc(const phys_vec3 &normal, const phys_vec3 &b1_r_loc, const phys_vec3 &b2_r_loc,
                         float *closest_error, const pulse_sum_cache_info **closest_psc) const; // 0xa2d840
    static void set_closest_cached_psc(const contact_point_info *cached_cpi, const phys_vec3 &normal,
                                       const phys_vec3 &b1_r_loc, const phys_vec3 &b2_r_loc,
                                       pulse_sum_cache_info *psc); // 0xa2dfa0
    void set_closest_cached_psc(const contact_point_info *cached_cpi); // 0xa2e020
};

// Order-independent body pair (lower address first). Key of physics_system::m_search_tree_rbc_contact.
class rigid_body_pair_key
{
public:
    rigid_body *m_b1;
    rigid_body *m_b2;

    rigid_body_pair_key() {} // TREYBO: rigid_body_constraint_contact keeps one uninitialised (retail ctor leaves it)

    // rbc_def_contact.h:232-234 (0xa140c0)
    // TREYBO FIX(0xa140c0): kisakblack stored the swapped pair and then overwrote it with (b1, b2) unconditionally.
    rigid_body_pair_key(rigid_body *const b1, rigid_body *const b2)
    {
        PHYS_ASSERT(b1 != b2);
        PHYS_ASSERT(b1);
        PHYS_ASSERT(b2);
        if ((uintptr_t)b1 < (uintptr_t)b2) // TREYBO: uintptr_t compare (retail unsigned pointer compare)
        {
            m_b1 = b1;
            m_b2 = b2;
        }
        else
        {
            m_b1 = b2;
            m_b2 = b1;
        }
    }

    // As inlined in phys_inplace_avl_tree<rigid_body_pair_key,...>::find 0xa166c0.
    bool operator<(const rigid_body_pair_key &key) const
    {
        if (m_b1 == key.m_b1)
            return (uintptr_t)m_b2 < (uintptr_t)key.m_b2;
        return (uintptr_t)m_b1 < (uintptr_t)key.m_b1;
    }
    bool operator==(const rigid_body_pair_key &key) const { return m_b1 == key.m_b1 && m_b2 == key.m_b2; }
};

class rigid_body_constraint_contact : public rigid_body_constraint
{
public:
    // Accessor for phys_inplace_avl_tree (see docs/ledger/phys_avl_tree.md).
    struct avl_tree_accessor
    {
        static rigid_body_pair_key &get_avl_key(rigid_body_constraint_contact *rbc) { return rbc->m_avl_key; }
        static const rigid_body_pair_key &get_avl_key(const rigid_body_constraint_contact *rbc) { return rbc->m_avl_key; }
        static phys_inplace_avl_tree_node<rigid_body_constraint_contact> *get_avl_node(rigid_body_constraint_contact *rbc) { return &rbc->m_avl_tree_node; }
        static const phys_inplace_avl_tree_node<rigid_body_constraint_contact> *get_avl_node(const rigid_body_constraint_contact *rbc) { return &rbc->m_avl_tree_node; }
    };

    // buffer_1: this frame's manifolds (pushed by add_cpi_simple, consumed by setup_constraint).
    // buffer_2: cleared by setup_constraint.
    phys_simple_link_list<contact_point_info> m_list_contact_point_info_buffer_1;
    phys_simple_link_list<contact_point_info> m_list_contact_point_info_buffer_2;
    unsigned int m_solver_priority;
    phys_inplace_avl_tree_node<rigid_body_constraint_contact> m_avl_tree_node;
    rigid_body_pair_key m_avl_key;

    // inlined in create(phys_free_list<rigid_body_constraint_contact> &, ...) 0xa172d0
    rigid_body_constraint_contact() : m_solver_priority(0) {}
    ~rigid_body_constraint_contact(); // 0xa252c0: unlinks from the search tree

    // rbc_def_contact.h:364-368 (0xa2ca70)
    void add_cpi_simple(contact_point_info *cpi, rigid_body *const b1_, rigid_body *const b2_)
    {
        PHYS_ASSERT(cpi);
        PHYS_ASSERT(cpi->m_list_b1_r_loc);
        PHYS_ASSERT(cpi->m_list_b2_r_loc);
        PHYS_ASSERT(cpi->m_list_pulse_sum_cache_info);
        PHYS_ASSERT(cpi->m_point_pair_count > 0);
        verify_constraint(b1_, b2_);
        m_list_contact_point_info_buffer_1.add(cpi);
    }

    // rbc_def_contact.h:414 (0xa2a700). Against the environment, b1 may only sweep as far as its earliest hit.
    void update_smallest_lambda()
    {
        PHYS_ASSERT(b1->is_environment_rigid_body() == false);
        if (b2->is_environment_rigid_body())
        {
            for (contact_point_info *cpi = m_list_contact_point_info_buffer_1.get_first(); cpi; cpi = cpi->m_next_link)
            {
                // TREYBO FIX(0xa2a74e): `fcompp; test ah,41h; jnz` takes the cpi lambda when it is NaN.
                b1->m_smallest_lambda = !(cpi->m_translation_lambda > b1->m_smallest_lambda) ? cpi->m_translation_lambda : b1->m_smallest_lambda;
            }
        }
    }

    void setup_constraint(pulse_sum_constraint_solver *psys, const float delta_t); // 0xa25270

private:
    void verify_constraint(rigid_body *const b1_, rigid_body *const b2_); // 0xa25200
};
