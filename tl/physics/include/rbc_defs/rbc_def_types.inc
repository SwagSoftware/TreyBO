// The per-constraint-type list. The includer defines
//     RBC_DEF_TYPE(class_name, short_name)
// and gets one expansion per type, e.g. create_##short_name / destroy_all_##short_name / m_list_##short_name.
//
// Order = physics_system free-list member order. rb_inplace_partition_node and the
// phys_sys::get_rbc_*_alloc_count functions use a different order (point, hinge, dist, ragdoll, wheel,
// angular_actuator, upright, custom_orientation, custom_path, contact) and are written out by hand.
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
