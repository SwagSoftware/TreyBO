// Retail: codsrc\tl\physics\source\physics_system_internal.cpp, TU 0xa10210-0xa131d0 (plus the rbint
// rigid_body_internal.h inlines it owns: 0xa10160, 0xa126a0-0xa12a60 and the solver-side COMDATs
// 0xa25fb0-0xa260c0, 0xa29a20-0xa2a510).

#include "tl/physics/include/physics_system_internal.h"
#include "tl/physics/include/phys_util.h"
#include "tl/base/include/tl_system.h"
#include "tl/physics/include/constraint_solver/pulse_sum_constraint_solver.h"

physics_system *g_physics_system;

// ---------------------------------------------------------------------------------------------
// rbint (rigid_body_internal.h). TREYBO: retail inlines; the asserts keep their retail text but report this file.
// ---------------------------------------------------------------------------------------------

// 0xa10160 (rigid_body_internal.h:75). The last sub-step lands exactly on the dictator (xyz of the 4 rows,
// which is what phys_mat44's copy assignment copies).
void rbint::take_last_step(user_rigid_body *rb, const float)
{
    PHYS_ASSERT(rb->is_user_rigid_body());
    PHYS_ASSERT(rb->m_dictator);
    rb->m_mat = *rb->m_dictator;
}

// 0xa126a0 (rigid_body_internal.h:37). Velocities that carry m_mat to the dictator over the whole frame.
void rbint::prolog_frame_advance(user_rigid_body *rb, const float delta_t)
{
    PHYS_ASSERT(rb->is_user_rigid_body());
    PHYS_ASSERT(rb->m_dictator);
    nuge::calc_velocities(rb->m_mat, *rb->m_dictator, delta_t, &rb->m_t_vel, &rb->m_a_vel);
    rb->debug_check_vec(rb->m_t_vel);
    rb->debug_check_vec(rb->m_a_vel);
}

// 0xa12840 (rigid_body_internal.h:46). User bodies sweep from where they are; no position swap.
void rbint::collision_prolog(user_rigid_body *rb, const float delta_t)
{
    PHYS_ASSERT(rb->is_user_rigid_body());
    PHYS_ASSERT(rb->m_dictator);
    rb->m_moved_vec = rb->m_t_vel * delta_t;
    rb->debug_check_vec(rb->m_moved_vec);
    rb->m_smallest_lambda = 0.0f;
}

// 0xa12970 (rigid_body_internal.h:65)
void rbint::take_next_step(user_rigid_body *rb, const float delta_t)
{
    PHYS_ASSERT(rb->is_user_rigid_body());
    PHYS_ASSERT(rb->m_dictator);
    rb->m_mat.GetW() = rb->m_mat.GetW() + rb->m_t_vel * delta_t;
    phys_mat44 rv;
    make_rotate(&rv, rb->m_a_vel, delta_t, 1000.0f);
    phys_multiply_mat(rb->m_mat, rv, rb->m_mat);
}

// 0xa12a60. Sweep vector from m_last_position to the predicted position; afterwards m_mat.w holds
// m_last_position and m_last_position the old position (swap_last_position).
void rbint::collision_prolog(rigid_body *const rb, const float delta_t)
{
    rb->m_moved_vec = (rb->m_t_vel * delta_t + rb->m_mat.GetW()) - rb->m_last_position;
    rb->debug_check_vec(rb->m_moved_vec);
    rb->m_smallest_lambda = 1.0f;
    rb->swap_last_position();
    rb->m_largest_vel_sq = 0.0f;
}

// 0xa25fb0: world inverse inertia (solver node) * t * delta_t.
const phys_vec3 rbint::inv_L(const rigid_body *rb, const phys_vec3 &t, const float delta_t)
{
    return phys_multiply(rb->m_node->m_world_inv_inertia, t) * delta_t;
}

// 0xa26010. Sleep test: a body whose largest velocity of the sub-step stays under the threshold accumulates
// m_stable_energy_time; FLAG_STABLE after 1 s, or after 0.5 s once it has m_stable_min_contact_count contacts.
void rbint::update_stability(rigid_body *const rb, const float delta_t)
{
    float max_stable_energy_sq = 141.60999f; // (11.9 = 0.35 * 34)^2
    if (!rb->m_stable_min_contact_count)
        max_stable_energy_sq = 29.593601f; // (5.44 = 0.16 * 34)^2

    // TREYBO FIX(0xa26041): kisakblack tested max >= largest (a NaN m_largest_vel_sq went unstable); retail only
    // leaves the stable path when max < largest.
    if (max_stable_energy_sq < rb->m_largest_vel_sq)
    {
        rb->m_flags &= ~rigid_body::FLAG_STABLE;
        rb->m_stable_energy_time = 0.0f;
    }
    else if (!(rb->m_flags & rigid_body::FLAG_STABLE))
    {
        rb->m_stable_energy_time = rb->m_stable_energy_time + delta_t;
        if (rb->m_stable_energy_time >= 1.0f)
            rb->m_flags |= rigid_body::FLAG_STABLE;
        else if (rb->m_stable_energy_time >= 0.5f && rb->m_contact_count >= rb->m_stable_min_contact_count)
            rb->m_flags |= rigid_body::FLAG_STABLE;
    }
}

// 0xa260c0: velocity of the body point at offset r.
const phys_vec3 rbint::gtv(rigid_body *const b, const phys_vec3 &r)
{
    return b->m_t_vel + phys_cross(b->m_a_vel, r);
}

// 0xa29a20: take_next_step without the asserts (solver copies of user bodies).
void rbint::substep(user_rigid_body *rb, const float delta_t)
{
    rb->m_mat.GetW() = rb->m_mat.GetW() + rb->m_t_vel * delta_t;
    phys_mat44 rv;
    make_rotate(&rv, rb->m_a_vel, delta_t, 1000.0f);
    phys_multiply_mat(rb->m_mat, rv, rb->m_mat);
}

// 0xa29ac0: the accumulated force / torque impulses become rates for this sub-step.
void rbint::solver_prolog(rigid_body *rb, const float delta_t)
{
    const float inv_delta_t = 1.0f / delta_t;
    rb->m_force_sum *= inv_delta_t;
    rb->m_torque_sum *= inv_delta_t;
    rb->debug_check_vec(rb->m_force_sum);
    rb->debug_check_vec(rb->m_torque_sum);
}

// 0xa29c60 (rigid_body_internal.h:124). Float order from the disasm; the three-term sums are ((x + y) + z).
void rbint::euler_integrate_velocity(rigid_body *const rb, const float delta_t)
{
    rb->debug_check_vec(rb->m_t_vel);
    rb->debug_check_vec(rb->m_a_vel);
    rb->m_last_t_vel = rb->m_t_vel;
    rb->m_last_a_vel = rb->m_a_vel;

    rb->m_t_vel += (rb->m_inv_mass * rb->m_force_sum + rb->m_gravity_acc_vec) * delta_t;
    rb->m_a_vel += inv_L(rb, rb->m_torque_sum, delta_t);

    // Gyroscopic term in body space: a_vel += R * I^-1 * ((I w) x w) * delta_t.
    const phys_vec3 a_vel_loc = phys_inv_multiply(rb->m_mat, rb->m_a_vel);
    const phys_vec3 a_mom_loc = phys_inv_diag_multiply(rb->m_inv_inertia, a_vel_loc);
    const float ake_times_two = phys_dot(a_mom_loc, a_vel_loc);
    rb->m_a_vel += phys_multiply(rb->m_mat, phys_diag_multiply(rb->m_inv_inertia, phys_cross(a_mom_loc, a_vel_loc)) * delta_t);

    const float navel_sq = AbsSquared(rb->m_a_vel);
    if (rb->m_max_avel * rb->m_max_avel < navel_sq)
    {
        PHYS_ASSERT(navel_sq > phys_sqr(0.00001f));
        rb->m_a_vel *= rb->m_max_avel / sqrtf(navel_sq);
    }

    // The gyroscopic step must not add rotational energy.
    const phys_vec3 a_vel_loc2 = phys_inv_multiply(rb->m_mat, rb->m_a_vel);
    const phys_vec3 a_mom_loc2 = phys_inv_diag_multiply(rb->m_inv_inertia, a_vel_loc2);
    const float ake_times_two_2 = phys_dot(a_vel_loc2, a_mom_loc2);
    if (ake_times_two < ake_times_two_2 && ake_times_two_2 > PHYS_EPS_1E5)
    {
        const float factor = sqrtf(ake_times_two / ake_times_two_2);
        rb->m_a_vel *= factor;
    }

    // TREYBO FIX(0xa2a1d9): kisakblack clamped with `drag >= 0.9 ? 0.9 : drag` (NaN kept); retail keeps the drag
    // only when it is below 0.9 (NaN clamps to 0.9).
    const float t_drag = rb->m_t_drag_coef * delta_t;
    rb->m_t_vel *= 1.0f - (t_drag < 0.9f ? t_drag : 0.9f);
    const float a_drag = delta_t * rb->m_a_drag_coef;
    rb->m_a_vel *= 1.0f - (a_drag < 0.9f ? a_drag : 0.9f);

    rb->debug_check_vec(rb->m_t_vel);
    rb->debug_check_vec(rb->m_a_vel);
}

// 0xa2a3c0. Re-orthonormalises m_mat every 6th step.
void rbint::euler_integrate_pos(rigid_body *const rb, const float delta_t)
{
    rb->m_mat.GetW() = rb->m_mat.GetW() + rb->m_t_vel * delta_t;
    phys_mat44 rv;
    make_rotate(&rv, rb->m_a_vel, delta_t, 1000.0f);
    phys_multiply_mat(rb->m_mat, rv, rb->m_mat);
    if (++rb->m_tick > 5)
    {
        rb->m_tick = 0;
        orthonormalize(&rb->m_mat);
    }
    rb->debug_check_vec(rb->m_mat.GetW());
}

// 0xa2a510
void rbint::add_vel(rigid_body *rb, const phys_vec3 &t, const phys_vec3 &a)
{
    rb->m_t_vel += t;
    rb->m_a_vel += a;
    rb->debug_check_vec(rb->m_t_vel);
    rb->debug_check_vec(rb->m_a_vel);
    // TREYBO: retail keeps the angular term in x87 precision for the sum (the linear one is stored to float first).
    if (AbsSquared(rb->m_t_vel) + AbsSquared(rb->m_a_vel) > 1.0e10f)
        rb->m_flags |= rigid_body::FLAG_DANGEROUS;
}

// ---------------------------------------------------------------------------------------------
// In-place partition nodes
// ---------------------------------------------------------------------------------------------

// Retail spells it GIPN() in every assert.
static inline rb_inplace_partition_node *GIPN(rigid_body *rb)
{
    return &rb->m_partition_node;
}

// TREYBO: name not recovered (inline, assert line 141). Clears the per-type constraint lists of an island head.
static void IPN_clear_constraint_lists(rigid_body *rb_partition_head)
{
    PHYS_ASSERT(GIPN(rb_partition_head)->m_partition_head == rb_partition_head);
    rb_inplace_partition_node *const ipn = GIPN(rb_partition_head);
    ipn->m_rbc_point_first = NULL;
    ipn->m_rbc_hinge_first = NULL;
    ipn->m_rbc_dist_first = NULL;
    ipn->m_rbc_ragdoll_first = NULL;
    ipn->m_rbc_wheel_first = NULL;
    ipn->m_rbc_angular_actuator_first = NULL;
    ipn->m_rbc_upright_first = NULL;
    ipn->m_rbc_custom_orientation_first = NULL;
    ipn->m_rbc_custom_path_first = NULL;
    ipn->m_rbc_contact_first = NULL;
}

// TREYBO: name not recovered (inline, assert line 163). Pushes rbc onto its island head's list of that type.
template <class T>
static void IPN_add_constraint(rigid_body *rb_partition_head, T &rbc, T *rb_inplace_partition_node::*first)
{
    PHYS_ASSERT(GIPN(rb_partition_head)->m_partition_head == rb_partition_head);
    rbc.m_next = GIPN(rb_partition_head)->*first;
    GIPN(rb_partition_head)->*first = &rbc;
}

template <class T>
static void IPN_verify_constraint_list(T *rbc)
{
    for (; rbc; rbc = static_cast<T *>(rbc->m_next))
    {
        g_physics_system->validate_member(rbc->b1);
        g_physics_system->validate_member(rbc->b2);
    }
}

// 0xa10480
void IPN_verify_rigid_bodies(rigid_body *rb_partition_head)
{
    PHYS_ASSERT(GIPN(rb_partition_head)->m_partition_head == rb_partition_head);
    rb_inplace_partition_node *const ipn = GIPN(rb_partition_head);
    IPN_verify_constraint_list(ipn->m_rbc_point_first);
    IPN_verify_constraint_list(ipn->m_rbc_hinge_first);
    IPN_verify_constraint_list(ipn->m_rbc_dist_first);
    IPN_verify_constraint_list(ipn->m_rbc_ragdoll_first);
    IPN_verify_constraint_list(ipn->m_rbc_wheel_first);
    IPN_verify_constraint_list(ipn->m_rbc_angular_actuator_first);
    IPN_verify_constraint_list(ipn->m_rbc_upright_first);
    IPN_verify_constraint_list(ipn->m_rbc_custom_orientation_first);
    IPN_verify_constraint_list(ipn->m_rbc_custom_path_first);
    IPN_verify_constraint_list(ipn->m_rbc_contact_first);
}

// 0xa10210: append source's island to dest's.
void IPN_merge(rigid_body *dest, rigid_body *source)
{
    PHYS_ASSERT(GIPN(dest)->m_partition_head == dest);
    PHYS_ASSERT(GIPN(source)->m_partition_head == source);
    PHYS_ASSERT(dest != source);
    GIPN(GIPN(dest)->m_partition_tail)->m_next_node = source;
    GIPN(dest)->m_partition_tail = GIPN(source)->m_partition_tail;
    GIPN(dest)->m_partition_size += GIPN(source)->m_partition_size;
    rigid_body *rb = source;
    do
    {
        GIPN(rb)->m_partition_head = GIPN(dest)->m_partition_head;
        rb = GIPN(rb)->m_next_node;
    } while (rb);
    GIPN(source)->m_partition_tail = NULL;
    GIPN(source)->m_partition_size = 0;
}

// 0xa121d0: a constraint between two island bodies joins their islands (the smaller one is appended).
void IPN_partition_process(const rigid_body_constraint &rbc, int *island_count)
{
    if (rbc.b1 && rbc.b2)
    {
        rigid_body *const head1 = GIPN(rbc.b1)->m_partition_head;
        rigid_body *const head2 = GIPN(rbc.b2)->m_partition_head;
        if (head1 && head2 && head1 != head2)
        {
            --*island_count;
            PHYS_ASSERT(*island_count > 0);
            if (GIPN(head1)->m_partition_size < GIPN(head2)->m_partition_size)
                IPN_merge(head2, head1);
            else
                IPN_merge(head1, head2);
        }
    }
}

// TREYBO: name not recovered (inline, assert line 229). b1's island head, else b2's.
static rigid_body *IPN_get_partition_head(const rigid_body_constraint &rbc)
{
    if (rbc.get_b1() && GIPN(rbc.get_b1())->m_partition_head)
        return GIPN(rbc.get_b1())->m_partition_head;
    PHYS_ASSERT(rbc.get_b2() && GIPN(rbc.get_b2())->m_partition_head);
    return GIPN(rbc.get_b2())->m_partition_head;
}

// 0xa102f0
void rigid_body_island_qsort(rigid_body **list, const int list_count)
{
    if (list_count >= 3)
    {
        const int half = list_count / 2;
        rigid_body_island_qsort(list, half);
        rigid_body_island_qsort(&list[half], list_count - half);
        rigid_body **const last = &list[list_count];
        rigid_body **prev = &list[half] - 1;
        for (rigid_body **cur = &list[half]; cur < last; ++cur)
        {
            if (GIPN(*cur)->m_partition_size <= GIPN(*prev)->m_partition_size)
                break;
            rigid_body *const swap_rb = *prev;
            *prev = *cur;
            *cur = swap_rb;
            for (rigid_body **p = prev; p > list; --p)
            {
                if (GIPN(*p)->m_partition_size <= GIPN(p[-1])->m_partition_size)
                    break;
                rigid_body *const swap_rb2 = p[-1];
                p[-1] = *p;
                *p = swap_rb2;
            }
            prev = cur;
        }
    }
    else if (list_count == 2)
    {
        if (GIPN(list[1])->m_partition_size > GIPN(list[0])->m_partition_size)
        {
            rigid_body *const swap_rb = list[0];
            list[0] = list[1];
            list[1] = swap_rb;
        }
    }
}

// 0xa103c0
void merge_sort(rigid_body_constraint_contact **list, const int list_count)
{
    if (list_count >= 3)
    {
        const int half = list_count / 2;
        merge_sort(list, half);
        merge_sort(&list[half], list_count - half);
        rigid_body_constraint_contact **const last = &list[list_count];
        rigid_body_constraint_contact **prev = &list[half] - 1;
        for (rigid_body_constraint_contact **cur = &list[half]; cur < last; ++cur)
        {
            if ((*cur)->m_solver_priority >= (*prev)->m_solver_priority)
                break;
            rigid_body_constraint_contact *const swap_rbc = *prev;
            *prev = *cur;
            *cur = swap_rbc;
            for (rigid_body_constraint_contact **p = prev; p > list; --p)
            {
                if ((*p)->m_solver_priority >= p[-1]->m_solver_priority)
                    break;
                rigid_body_constraint_contact *const swap_rbc2 = p[-1];
                p[-1] = *p;
                *p = swap_rbc2;
            }
            prev = cur;
        }
    }
    else if (list_count == 2)
    {
        if (list[1]->m_solver_priority < list[0]->m_solver_priority)
        {
            rigid_body_constraint_contact *const swap_rbc = list[0];
            list[0] = list[1];
            list[1] = swap_rbc;
        }
    }
}

// ---------------------------------------------------------------------------------------------
// physics_system
// ---------------------------------------------------------------------------------------------

// Points found for this pair by the current collision pass (buffer_1) or the previous one (buffer_2).
static int get_point_pair_count(const phys_simple_link_list<contact_point_info> &list)
{
    int count = 0;
    for (const contact_point_info *cpi = list.get_first(); cpi; cpi = cpi->m_next_link)
        count += cpi->m_point_pair_count;
    return count;
}

template <class T>
static void count_constraint_and_partition(phys_free_list<T> &list, int *island_count)
{
    for (T &rbc : list)
    {
        if (rbc.b1)
            ++rbc.b1->m_constraint_count;
        if (rbc.b2)
            ++rbc.b2->m_constraint_count;
        IPN_partition_process(rbc, island_count);
    }
}

template <class T>
static void add_constraints_to_partitions(phys_free_list<T> &list, T *rb_inplace_partition_node::*first)
{
    for (T &rbc : list)
        IPN_add_constraint(IPN_get_partition_head(rbc), rbc, first);
}

// 0xa10c70. Recount body constraints/contacts, build the islands, give every island head its per-type
// constraint lists, and collect the island heads (largest first) into m_list_island.
void physics_system::generate_partitions_and_stuff(phys_transient_allocator *transient_buffer)
{
    m_environment_rigid_body.m_constraint_count = 0;
    m_environment_rigid_body.m_contact_count = 0;
    GIPN(&m_environment_rigid_body)->m_partition_head = NULL;
    for (user_rigid_body &rb : m_list_user_rigid_body)
    {
        rb.m_constraint_count = 0;
        rb.m_contact_count = 0;
        GIPN(&rb)->m_partition_head = NULL;
    }
    for (rigid_body &rb : m_list_rigid_body)
    {
        rb.m_constraint_count = 0;
        rb.m_contact_count = 0;
        GIPN(&rb)->m_partition_head = &rb;
        GIPN(&rb)->m_partition_tail = &rb;
        GIPN(&rb)->m_next_node = NULL;
        GIPN(&rb)->m_partition_size = 1;
    }

    // Order is retail (not rbc_def_types.inc order).
    m_list_island_count = m_list_rigid_body.get_count();
    count_constraint_and_partition(m_list_rbc_point, &m_list_island_count);
    count_constraint_and_partition(m_list_rbc_hinge, &m_list_island_count);
    count_constraint_and_partition(m_list_rbc_dist, &m_list_island_count);
    count_constraint_and_partition(m_list_rbc_ragdoll, &m_list_island_count);
    for (rigid_body_constraint_wheel &rbc : m_list_rbc_wheel)
    {
        if (rbc.b1)
            ++rbc.b1->m_constraint_count;
        if (rbc.b2)
            ++rbc.b2->m_constraint_count;
        if (rbc.get_wheel_is_colliding())
        {
            if (rbc.b1)
                ++rbc.b1->m_contact_count;
            if (rbc.b2)
                ++rbc.b2->m_contact_count;
        }
        IPN_partition_process(rbc, &m_list_island_count);
    }
    count_constraint_and_partition(m_list_rbc_angular_actuator, &m_list_island_count);
    count_constraint_and_partition(m_list_rbc_upright, &m_list_island_count);
    count_constraint_and_partition(m_list_rbc_custom_orientation, &m_list_island_count);
    count_constraint_and_partition(m_list_rbc_custom_path, &m_list_island_count);
    for (rigid_body_constraint_contact &rbc : m_list_rbc_contact)
    {
        // contacts add to m_contact_count only
        const int count_1 = get_point_pair_count(rbc.m_list_contact_point_info_buffer_1);
        const int count_2 = get_point_pair_count(rbc.m_list_contact_point_info_buffer_2);
        const int point_count = count_2 <= count_1 ? count_1 : count_2;
        if (rbc.b1)
            rbc.b1->m_contact_count += point_count;
        if (rbc.b2)
            rbc.b2->m_contact_count += point_count;
        IPN_partition_process(rbc, &m_list_island_count);
    }

    for (rigid_body &rb : m_list_rigid_body)
    {
        if (GIPN(&rb)->m_partition_size > 0)
            IPN_clear_constraint_lists(&rb);
    }

    add_constraints_to_partitions(m_list_rbc_point, &rb_inplace_partition_node::m_rbc_point_first);
    add_constraints_to_partitions(m_list_rbc_hinge, &rb_inplace_partition_node::m_rbc_hinge_first);
    add_constraints_to_partitions(m_list_rbc_dist, &rb_inplace_partition_node::m_rbc_dist_first);
    add_constraints_to_partitions(m_list_rbc_ragdoll, &rb_inplace_partition_node::m_rbc_ragdoll_first);
    add_constraints_to_partitions(m_list_rbc_wheel, &rb_inplace_partition_node::m_rbc_wheel_first);
    add_constraints_to_partitions(m_list_rbc_angular_actuator, &rb_inplace_partition_node::m_rbc_angular_actuator_first);
    add_constraints_to_partitions(m_list_rbc_upright, &rb_inplace_partition_node::m_rbc_upright_first);
    add_constraints_to_partitions(m_list_rbc_custom_orientation, &rb_inplace_partition_node::m_rbc_custom_orientation_first);
    add_constraints_to_partitions(m_list_rbc_custom_path, &rb_inplace_partition_node::m_rbc_custom_path_first);
    add_constraints_to_partitions(m_list_rbc_contact, &rb_inplace_partition_node::m_rbc_contact_first);

    PHYS_ASSERT(transient_buffer->is_empty());
    if (m_list_island_count <= 0)
    {
        m_list_island = NULL;
    }
    else
    {
        m_list_island = (rigid_body **)transient_buffer->allocate((int)sizeof(rigid_body *) * m_list_island_count,
                                                                  (int)PHYS_ALIGNOF(rigid_body *), false,
                                                                  "phys_transient_allocator out of memory.");
        int list_island_cur = 0;
        for (rigid_body &rb : m_list_rigid_body)
        {
            if (GIPN(&rb)->m_partition_size > 0)
            {
                IPN_verify_rigid_bodies(&rb);
                PHYS_ASSERT(list_island_cur < m_list_island_count);
                m_list_island[list_island_cur++] = &rb;
            }
        }
        PHYS_ASSERT(list_island_cur == m_list_island_count);
        rigid_body_island_qsort(m_list_island, m_list_island_count);
    }
}

// 0xa118c0. One sub-step: collision prolog, the host collision pass, drop pairs without points, solve
// (the solver integrates velocities and positions and updates stability), step the user bodies along
// their dictators, then swap the contact buffers.
void physics_system::time_step(const float outside_delta_t, const bool last_step)
{
    m_flags |= FLAG_IN_COLLISION_STEP;
    m_outside_sub_delta_t = outside_delta_t;
    for (rigid_body &rb : m_list_rigid_body)
        rbint::collision_prolog(&rb, outside_delta_t);
    for (user_rigid_body &rb : m_list_user_rigid_body)
        rbint::collision_prolog(&rb, outside_delta_t);
    if (m_collision_callback)
        m_collision_callback();
    m_flags &= ~FLAG_IN_COLLISION_STEP;

    // Pairs the collision pass found no points for are destroyed (removal runs ~rigid_body_constraint_contact).
    for (phys_free_list<rigid_body_constraint_contact>::T_internal_base *i = m_list_rbc_contact.m_dummy_head.m_next_T_internal;
         i != &m_list_rbc_contact.m_dummy_head;)
    {
        rigid_body_constraint_contact &rbc = static_cast<phys_free_list<rigid_body_constraint_contact>::T_internal *>(i)->m_data;
        const int point_count = get_point_pair_count(rbc.m_list_contact_point_info_buffer_1);
        i = i->m_next_T_internal;
        if (!point_count)
            m_list_rbc_contact.remove(&rbc);
    }

    phys_transient_allocator transient_buffer;
    solver_priority_sort(&transient_buffer);
    generate_partitions_and_stuff(&transient_buffer);
    tlScratchPadLock();
    constraint_solver_process(&transient_buffer, this, outside_delta_t);
    tlScratchPadUnlock();
    transient_buffer.reset();
    m_list_island = NULL;
    m_list_island_count = 0;

    if (last_step)
    {
        for (user_rigid_body &rb : m_list_user_rigid_body)
            rbint::take_last_step(&rb, outside_delta_t);
    }
    else
    {
        for (user_rigid_body &rb : m_list_user_rigid_body)
            rbint::take_next_step(&rb, outside_delta_t);
    }

    const int contact_point_buffer_size = (int)m_contact_point_buffer_1.m_total_memory_allocated;
    m_contact_point_buffer_high_water = m_contact_point_buffer_high_water > contact_point_buffer_size ? m_contact_point_buffer_high_water : contact_point_buffer_size;

    // This sub-step's contact points become next sub-step's warm-start cache.
    m_contact_point_buffer_2.reset();
    {
        phys_transient_allocator temp_buffer = m_contact_point_buffer_2;
        m_contact_point_buffer_2 = m_contact_point_buffer_1;
        m_contact_point_buffer_1 = temp_buffer;
    } // ~temp_buffer asserts it was reset (retail does the same)
}

// 0xa131d0: contact constraints in ascending m_solver_priority (list relinked in place).
void physics_system::solver_priority_sort(phys_transient_allocator *transient_buffer)
{
    if (m_list_rbc_contact.get_count() >= 4)
    {
        PHYS_ASSERT(transient_buffer->is_empty());
        rigid_body_constraint_contact **const list = (rigid_body_constraint_contact **)transient_buffer->allocate(
            (int)sizeof(rigid_body_constraint_contact *) * m_list_rbc_contact.get_count(),
            (int)PHYS_ALIGNOF(rigid_body_constraint_contact *), false, "phys_transient_allocator out of memory.");
        m_list_rbc_contact.ptr_array_read(list, m_list_rbc_contact.get_count());
        merge_sort(list, m_list_rbc_contact.get_count());
        m_list_rbc_contact.ptr_array_write(list, m_list_rbc_contact.get_count());
        transient_buffer->reset();
    }
}

template <class T>
static void outer_prolog_update_all(phys_free_list<T> &list, const float delta_t)
{
    for (T &rbc : list)
        rbc.outer_prolog_update(delta_t);
}

template <class T>
static void outer_epilog_update_all(phys_free_list<T> &list, const float delta_t)
{
    for (T &rbc : list)
        rbc.outer_epilog_update(delta_t);
}

// 0xa11c20. Splits delta_t into the smallest number of equal sub-steps no longer than m_max_delta_t.
void physics_system::frame_advance(const float delta_t)
{
    phys_assert_info_frame_advance_all();

    // TREYBO: retail divides in x87 extended precision and truncates; the double quotient of two floats has the
    // same integer part (a float rounding of the quotient could reach the next integer).
    int sub_count = (int)((double)delta_t / (double)m_max_delta_t);
    if (sub_count * m_max_delta_t < delta_t)
        ++sub_count;
    const float sub_count_f = (float)sub_count;
    PHYS_ASSERT_STR(!(sub_count_f * m_max_delta_t < delta_t), "delta_t <= m_max_delta_t * sub_count");

    for (user_rigid_body &rb : m_list_user_rigid_body)
        rbint::prolog_frame_advance(&rb, delta_t);

    // Retail order; only distance and angular_actuator do anything (the rest are the rbc_def_base no-ops).
    outer_prolog_update_all(m_list_rbc_point, delta_t);
    outer_prolog_update_all(m_list_rbc_hinge, delta_t);
    outer_prolog_update_all(m_list_rbc_dist, delta_t);
    outer_prolog_update_all(m_list_rbc_ragdoll, delta_t);
    outer_prolog_update_all(m_list_rbc_wheel, delta_t);
    outer_prolog_update_all(m_list_rbc_angular_actuator, delta_t);
    outer_prolog_update_all(m_list_rbc_upright, delta_t);
    outer_prolog_update_all(m_list_rbc_custom_orientation, delta_t);
    outer_prolog_update_all(m_list_rbc_custom_path, delta_t);
    outer_prolog_update_all(m_list_rbc_contact, delta_t);

    for (int sub_i = 0; sub_i < sub_count; ++sub_i)
    {
        const float sub_delta_t = delta_t / sub_count_f;
        time_step(sub_delta_t, sub_i + 1 == sub_count);
    }

    outer_epilog_update_all(m_list_rbc_point, delta_t);
    outer_epilog_update_all(m_list_rbc_hinge, delta_t);
    outer_epilog_update_all(m_list_rbc_dist, delta_t);
    outer_epilog_update_all(m_list_rbc_ragdoll, delta_t);
    outer_epilog_update_all(m_list_rbc_wheel, delta_t);
    outer_epilog_update_all(m_list_rbc_angular_actuator, delta_t);
    outer_epilog_update_all(m_list_rbc_upright, delta_t);
    outer_epilog_update_all(m_list_rbc_custom_orientation, delta_t);
    outer_epilog_update_all(m_list_rbc_custom_path, delta_t);
    outer_epilog_update_all(m_list_rbc_contact, delta_t);
}

// 0xa11f30. m_list_island / m_list_island_count are left uninitialised, as in retail (set before every use).
physics_system::physics_system()
{
    m_outside_sub_delta_t = 0.0f;
    m_max_delta_t = 0.051282052f; // 0x3d520d21
    m_flags = 0;
    m_collision_callback = NULL;
    m_max_vel_iters = 8;
    m_max_vel_pos_iters = 4;
    m_environment_rigid_body.set();
    m_solver_memory_high_water = 0;
    m_contact_point_buffer_high_water = 0;
}

// 0xa12120. Permanent PMM allocation (never freed).
physics_system *physics_system::create_physics_system()
{
    void *const mem = PMM_PERM_ALLOCATE((uint)sizeof(physics_system), (uint)PHYS_ALIGNOF(physics_system));
    if (mem)
        return new (mem) physics_system;
    return NULL;
}

// 0xa12190
void physics_system::initialize()
{
    g_physics_system = create_physics_system();
}

// 0xa121a0
void physics_system::destroy_physics_system(physics_system *psys)
{
    psys->~physics_system();
}

// 0xa121b0
void physics_system::shutdown()
{
    g_physics_system->~physics_system();
    g_physics_system = NULL;
}
