// Retail: codsrc\tl\physics\include\rbc_defs\rbc_def_types.inc. The per-constraint-type list.
//
// SM3 ships this file pre-expanded inside physics_system (counters, create_rbc_X, destroy, destroy_all_rbc_X,
// max/available/used slots per type). TREYBO: it is an X-macro list here; the includer defines
//     RBC_DEF_TYPE(class_name, short_name)
// and gets one expansion per type, e.g. create_##short_name / destroy_all_##short_name / m_list_##short_name.
//
// Order = BO1 physics_system free-list member order (IDB physics_system +0x9ac..+0x2e60). The first six match
// SM3; BO1 inserts custom_orientation / custom_path before angular_actuator and appends upright (SM3 had 7 types).
// Other BO1 orderings differ: rb_inplace_partition_node and the phys_sys::get_rbc_*_alloc_count functions
// (0xa14290-0xa14320) run point, hinge, dist, ragdoll, wheel, angular_actuator, upright, custom_orientation,
// custom_path, contact. Those are written out by hand.
//
// No include guard: include once per expansion.

#ifndef RBC_DEF_TYPE
#error "define RBC_DEF_TYPE(class_name, short_name) before including rbc_def_types.inc"
#endif

RBC_DEF_TYPE(rigid_body_constraint_contact, rbc_contact)
RBC_DEF_TYPE(rigid_body_constraint_point, rbc_point)
RBC_DEF_TYPE(rigid_body_constraint_hinge, rbc_hinge)
RBC_DEF_TYPE(rigid_body_constraint_distance, rbc_dist)
RBC_DEF_TYPE(rigid_body_constraint_ragdoll, rbc_ragdoll)
RBC_DEF_TYPE(rigid_body_constraint_wheel, rbc_wheel)
RBC_DEF_TYPE(rigid_body_constraint_custom_orientation, rbc_custom_orientation)
RBC_DEF_TYPE(rigid_body_constraint_custom_path, rbc_custom_path)
RBC_DEF_TYPE(rigid_body_constraint_angular_actuator, rbc_angular_actuator)
RBC_DEF_TYPE(rigid_body_constraint_upright, rbc_upright)

#undef RBC_DEF_TYPE
