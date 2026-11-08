// Retail: codsrc\tl\physics\source\physics_system.cpp, TU ~0xa13200-0xa17430 (phys_sys 0xa14160-0xa16680,
// the create(...) overload 0xa172d0 and the phys_free_list instantiations it pulls in).

#include "tl/physics/include/physics_system.h"
#include "tl/physics/include/physics_system_internal.h"
#include "tl/physics/include/phys_assert.h"

// create(phys_free_list<rigid_body_constraint_contact> &, ...) 0xa172d0: assert "rbc || no_error", line 92.
phys_assert_info pai_create_rbc_contact(0, 2, true);

// Inlined into every create_rbc_X except contact (0xa14760 ...): validate the bodies, take a slot, set b1 / b2.
template <class T>
static T *create(phys_free_list<T> &list, T *, rigid_body *const b1, rigid_body *const b2, const int no_error, const char *error_msg)
{
    g_physics_system->validate_member(b1);
    g_physics_system->validate_member(b2);
    T *const rbc = list.add(no_error, error_msg);
    if (rbc)
    {
        rbc->b1 = b1;
        rbc->b2 = b2;
    }
    return rbc;
}

// 0xa172d0 (plain overload in retail, not a template instance). One contact constraint per body pair:
// returns the existing one, else creates it and adds it to the pair search tree.
rigid_body_constraint_contact *create(phys_free_list<rigid_body_constraint_contact> &list, rigid_body_constraint_contact *,
                                      rigid_body *const b1, rigid_body *const b2, int no_error, const char *error_msg)
{
    g_physics_system->validate_member(b1);
    g_physics_system->validate_member(b2);
    const rigid_body_pair_key key(b1, b2);
    rigid_body_constraint_contact *rbc = g_physics_system->m_search_tree_rbc_contact.find(key);
    if (rbc)
        return rbc;
    rbc = list.add(true, error_msg); // retail inlines add without its own assert; the pai assert below replaces it
    if (rbc)
    {
        rbc->b1 = b1;
        rbc->b2 = b2;
        g_physics_system->m_search_tree_rbc_contact.add(key, rbc);
        return rbc;
    }
    if (no_error)
        return rbc;
    PHYS_ASSERT_INFO(pai_create_rbc_contact, rbc || no_error, error_msg);
    return rbc;
}

// ---------------------------------------------------------------------------------------------
// Per-type functions. The bodies are identical for every type (create dispatches to the contact overload above).
// Addresses (create / destroy / destroy_all / get_alloc_count):
//   contact        0xa14800 / 0xa14e90 / 0xa156d0 / 0xa14320
//   point          0xa14760 / 0xa14830 / 0xa14a70 / 0xa14290
//   hinge          0xa14c60 / 0xa14870 / 0xa14a90 / 0xa142a0
//   dist           0xa14cb0 / 0xa148b0 / 0xa14ab0 / 0xa142b0
//   ragdoll        0xa14d00 / 0xa148f0 / 0xa14ad0 / 0xa142c0
//   wheel          0xa14d50 / 0xa14930 / 0xa14af0 / 0xa142d0
//   custom_orient. 0xa147b0 / 0xa149f0 / 0xa14b50 / 0xa14300
//   custom_path    0xa14e40 / 0xa14a30 / 0xa14b70 / 0xa14310
//   angular_act.   0xa14da0 / 0xa14970 / 0xa14b10 / 0xa142e0
//   upright        0xa14df0 / 0xa149b0 / 0xa14b30 / 0xa142f0
// ---------------------------------------------------------------------------------------------

#define RBC_DEF_TYPE(class_name, short_name)                                                                      \
    class_name *phys_sys::create_##short_name(rigid_body *const b1, rigid_body *const b2, const int no_error)     \
    {                                                                                                            \
        return create(g_physics_system->m_list_##short_name, (class_name *)NULL, b1, b2, no_error,               \
                      "OUT_OF_MEMORY, " #short_name ", INCREASE phys_mem_info::m_num_" #short_name ".");         \
    }                                                                                                            \
    void phys_sys::destroy(class_name *const rbc)                                                                \
    {                                                                                                            \
        g_physics_system->m_list_##short_name.remove(rbc);                                                       \
    }                                                                                                            \
    void phys_sys::destroy_all_##short_name()                                                                    \
    {                                                                                                            \
        g_physics_system->m_list_##short_name.remove_all();                                                      \
    }                                                                                                            \
    int phys_sys::get_##short_name##_alloc_count()                                                               \
    {                                                                                                            \
        return g_physics_system->m_list_##short_name.get_count();                                                \
    }
#include "tl/physics/include/rbc_defs/rbc_def_types.inc"

// ---------------------------------------------------------------------------------------------
// Removal helpers inlined in destroy_all_user_rigid_body / destroy_all_constraint(_with_user_rigid_body).
// Each walks one list and removes the matching constraints (remove(T *) validates the slot first).
// ---------------------------------------------------------------------------------------------

template <class T, class pred_t>
static void remove_constraints_if(phys_free_list<T> &list, pred_t pred)
{
    typename phys_free_list<T>::T_internal_base *i = list.m_dummy_head.m_next_T_internal;
    while (i != &list.m_dummy_head)
    {
        T &rbc = static_cast<typename phys_free_list<T>::T_internal *>(i)->m_data;
        i = i->m_next_T_internal;
        if (pred(rbc))
            list.remove(&rbc);
    }
}

// Retail list order of all three callers.
template <class pred_t>
static void remove_constraints_if_all_types(pred_t pred)
{
    remove_constraints_if(g_physics_system->m_list_rbc_point, pred);
    remove_constraints_if(g_physics_system->m_list_rbc_hinge, pred);
    remove_constraints_if(g_physics_system->m_list_rbc_dist, pred);
    remove_constraints_if(g_physics_system->m_list_rbc_ragdoll, pred);
    remove_constraints_if(g_physics_system->m_list_rbc_wheel, pred);
    remove_constraints_if(g_physics_system->m_list_rbc_angular_actuator, pred);
    remove_constraints_if(g_physics_system->m_list_rbc_upright, pred);
    remove_constraints_if(g_physics_system->m_list_rbc_custom_orientation, pred);
    remove_constraints_if(g_physics_system->m_list_rbc_custom_path, pred);
    remove_constraints_if(g_physics_system->m_list_rbc_contact, pred);
}

struct rbc_touches_user_rigid_body
{
    bool operator()(const rigid_body_constraint &rbc) const
    {
        return (rbc.b1 && rbc.b1->is_user_rigid_body()) || (rbc.b2 && rbc.b2->is_user_rigid_body());
    }
};

struct rbc_touches_rigid_body
{
    rigid_body *m_rb;
    bool operator()(const rigid_body_constraint &rbc) const
    {
        return (rbc.b1 && rbc.b1 == m_rb) || (rbc.b2 && rbc.b2 == m_rb);
    }
};

struct rbc_joins_rigid_body_and_user_rigid_body
{
    rigid_body *m_rb;
    bool operator()(const rigid_body_constraint &rbc) const
    {
        return rbc.b1 && rbc.b2 &&
               ((rbc.b1 == m_rb && rbc.b2->is_user_rigid_body()) || (rbc.b2 == m_rb && rbc.b1->is_user_rigid_body()));
    }
};

// ---------------------------------------------------------------------------------------------
// phys_sys
// ---------------------------------------------------------------------------------------------

// 0xa14160
environment_rigid_body *phys_sys::get_environment_rigid_body()
{
    return &g_physics_system->m_environment_rigid_body;
}

// 0xa14170
void phys_sys::set_max_delta_t(const float max_delta_t)
{
    g_physics_system->m_max_delta_t = max_delta_t;
}

// 0xa14180
float phys_sys::get_max_delta_t()
{
    return g_physics_system->m_max_delta_t;
}

// 0xa14190
void phys_sys::set_v_tol(const int max_v_iters)
{
    g_physics_system->m_max_vel_iters = max_v_iters;
}

// 0xa141b0
void phys_sys::get_v_tol(int *max_v_iters)
{
    *max_v_iters = g_physics_system->m_max_vel_iters;
}

// 0xa141d0
void phys_sys::set_vp_tol(const int max_vp_iters)
{
    g_physics_system->m_max_vel_pos_iters = max_vp_iters;
}

// 0xa141f0
void phys_sys::get_vp_tol(int *max_vp_iters)
{
    *max_vp_iters = g_physics_system->m_max_vel_pos_iters;
}

// 0xa14210
void phys_sys::set_collision_callback(phys_collision_callback_t collision_callback)
{
    g_physics_system->m_collision_callback = collision_callback;
}

// 0xa14230
void phys_sys::phys_frame_advance(float delta_t)
{
    g_physics_system->frame_advance(delta_t);
}

// 0xa14250
void phys_sys::phys_init()
{
    physics_system::initialize();
}

// 0xa14260
void phys_sys::phys_shutdown()
{
    physics_system::shutdown();
}

// TREYBO: SM3 API, no BO1 symbol.
void phys_sys::set_phys_proftimer_callbacks(const phys_proftimer_callbacks &ppc)
{
    g_phys_proftimer_callbacks = ppc;
}

// 0xa14270
int phys_sys::get_rigid_body_alloc_count()
{
    return g_physics_system->m_list_rigid_body.get_count();
}

// 0xa14280
int phys_sys::get_user_rigid_body_alloc_count()
{
    return g_physics_system->m_list_user_rigid_body.get_count();
}

// 0xa14330
user_rigid_body *phys_sys::get_user_rigid_body(const phys_mat44 *const dictactor)
{
    for (user_rigid_body &rb : g_physics_system->m_list_user_rigid_body)
    {
        // TREYBO: retail asserts this through an unidentified rigid_body_internal.h:31 inline.
        PHYS_ASSERT(rb.is_user_rigid_body());
        if (rb.m_dictator == dictactor)
            return &rb;
    }
    return NULL;
}

// 0xa143a0
rigid_body_constraint_contact *phys_sys::get_rbc_contact(rigid_body *const b1, rigid_body *const b2)
{
    g_physics_system->validate_member(b1);
    g_physics_system->validate_member(b2);
    return g_physics_system->m_search_tree_rbc_contact.find(rigid_body_pair_key(b1, b2));
}

// 0xa143f0. set_no_collision clears b2, so retail re-tests the same wheel instead of advancing.
void phys_sys::fixup_wheel_constraints(rigid_body *const rb)
{
    phys_free_list<rigid_body_constraint_wheel> &list = g_physics_system->m_list_rbc_wheel;
    phys_free_list<rigid_body_constraint_wheel>::T_internal_base *i = list.m_dummy_head.m_next_T_internal;
    while (i != &list.m_dummy_head)
    {
        rigid_body_constraint_wheel &rbc = static_cast<phys_free_list<rigid_body_constraint_wheel>::T_internal *>(i)->m_data;
        if (rbc.b2 == rb)
            rbc.set_no_collision();
        else
            i = i->m_next_T_internal;
    }
}

template <class T>
static void count_constraint(phys_free_list<T> &list)
{
    for (T &rbc : list)
    {
        if (rbc.b1)
            ++rbc.b1->m_constraint_count;
        if (rbc.b2)
            ++rbc.b2->m_constraint_count;
    }
}

static int get_point_pair_count(const phys_simple_link_list<contact_point_info> &list)
{
    int count = 0;
    for (const contact_point_info *cpi = list.get_first(); cpi; cpi = cpi->m_next_link)
        count += cpi->m_point_pair_count;
    return count;
}

// 0xa14440. The counting half of physics_system::generate_partitions_and_stuff (same order and rules).
// TREYBO FIX(0xa14440): kisakblack compiled this out behind iassert(0).
void phys_sys::update_constraint_infos()
{
    physics_system *const psys = g_physics_system;
    psys->m_environment_rigid_body.m_constraint_count = 0;
    psys->m_environment_rigid_body.m_contact_count = 0;
    for (user_rigid_body &rb : psys->m_list_user_rigid_body)
    {
        rb.m_constraint_count = 0;
        rb.m_contact_count = 0;
    }
    for (rigid_body &rb : psys->m_list_rigid_body)
    {
        rb.m_constraint_count = 0;
        rb.m_contact_count = 0;
    }
    count_constraint(psys->m_list_rbc_point);
    count_constraint(psys->m_list_rbc_hinge);
    count_constraint(psys->m_list_rbc_dist);
    count_constraint(psys->m_list_rbc_ragdoll);
    for (rigid_body_constraint_wheel &rbc : psys->m_list_rbc_wheel)
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
    }
    count_constraint(psys->m_list_rbc_angular_actuator);
    count_constraint(psys->m_list_rbc_upright);
    count_constraint(psys->m_list_rbc_custom_orientation);
    count_constraint(psys->m_list_rbc_custom_path);
    for (rigid_body_constraint_contact &rbc : psys->m_list_rbc_contact)
    {
        const int count_1 = get_point_pair_count(rbc.m_list_contact_point_info_buffer_1);
        const int count_2 = get_point_pair_count(rbc.m_list_contact_point_info_buffer_2);
        const int point_count = count_2 <= count_1 ? count_1 : count_2;
        if (rbc.b1)
            rbc.b1->m_contact_count += point_count;
        if (rbc.b2)
            rbc.b2->m_contact_count += point_count;
    }
}

// 0xa14740
rigid_body *phys_sys::create_rigid_body(const int no_error)
{
    return g_physics_system->m_list_rigid_body.add(no_error, "POOL OUT OF MEMORY, rigid_body, INCREASE phys_mem_info::m_num_rigid_body.");
}

// 0xa14c40
user_rigid_body *phys_sys::create_user_rigid_body(const int no_error)
{
    return g_physics_system->m_list_user_rigid_body.add(no_error, "POOL OUT OF MEMORY, user_rigid_body, INCREASE phys_mem_info::m_num_user_rigid_body.");
}

// 0xa14b90. User bodies no constraint referenced at the last count (m_constraint_count; contacts are not counted).
// TREYBO FIX(0xa14b90): kisakblack compiled this out behind iassert(0).
void phys_sys::destroy_all_unused_user_rigid_body()
{
    phys_free_list<user_rigid_body> &list = g_physics_system->m_list_user_rigid_body;
    phys_free_list<user_rigid_body>::T_internal_base *i = list.m_dummy_head.m_next_T_internal;
    while (i != &list.m_dummy_head)
    {
        user_rigid_body &rb = static_cast<phys_free_list<user_rigid_body>::T_internal *>(i)->m_data;
        i = i->m_next_T_internal;
        if (!rb.m_constraint_count)
            list.remove(&rb);
    }
}

// 0xa14ed0: every constraint (including those on user bodies) and every rigid body.
void phys_sys::destroy_all_rigid_body()
{
    g_physics_system->m_list_rbc_point.remove_all();
    g_physics_system->m_list_rbc_hinge.remove_all();
    g_physics_system->m_list_rbc_dist.remove_all();
    g_physics_system->m_list_rbc_ragdoll.remove_all();
    g_physics_system->m_list_rbc_wheel.remove_all();
    g_physics_system->m_list_rbc_angular_actuator.remove_all();
    g_physics_system->m_list_rbc_upright.remove_all();
    g_physics_system->m_list_rbc_custom_orientation.remove_all();
    g_physics_system->m_list_rbc_custom_path.remove_all();
    g_physics_system->m_list_rbc_contact.remove_all();
    g_physics_system->m_list_rigid_body.remove_all();
}

// 0xa14f90. TREYBO FIX(0xa14f90): kisakblack compiled this out behind iassert(0).
void phys_sys::destroy_all_user_rigid_body()
{
    remove_constraints_if_all_types(rbc_touches_user_rigid_body());
    g_physics_system->m_list_user_rigid_body.remove_all();
}

// 0xa156f0: every constraint with rb as b1 or b2 (wheels resting on rb were detached by the fixup first).
void phys_sys::destroy_all_constraint(rigid_body *const rb)
{
    fixup_wheel_constraints(rb);
    rbc_touches_rigid_body pred;
    pred.m_rb = rb;
    remove_constraints_if_all_types(pred);
}

// 0xa15e10: only the constraints between rb and a user body.
// TREYBO FIX(0xa15e10): kisakblack compiled this out behind iassert(0).
void phys_sys::destroy_all_constraint_with_user_rigid_body(rigid_body *const rb)
{
    fixup_wheel_constraints(rb);
    rbc_joins_rigid_body_and_user_rigid_body pred;
    pred.m_rb = rb;
    remove_constraints_if_all_types(pred);
}

// 0xa16640
void phys_sys::destroy(rigid_body *const rb)
{
    destroy_all_constraint(rb);
    g_physics_system->m_list_rigid_body.remove(rb);
}

// 0xa16680
void phys_sys::destroy(user_rigid_body *const rb)
{
    destroy_all_constraint(rb);
    g_physics_system->m_list_user_rigid_body.remove(rb);
}
