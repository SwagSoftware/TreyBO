// Retail: codsrc\tl\physics\source\phys_collision.cpp ("source/phys_collision.cpp" asserts, all in
// phys_contact_manifold_process::process), TU 0xa2d840-0xa2ee1f. Not the game file src/physics/phys_collision.cpp
// (0x837xxx-0x83cxxx: set_cpi_params, entity bpis, gjk_physics_collision_visitor, standard_query).
//
// TREYBO: this TU holds no definitions. Its retail contents live in:
//   rbc_def_contact.cpp        contact_point_info::get_closest_psc / set_closest_cached_psc x2
//                              (0xa2d840 / 0xa2dfa0 / 0xa2e020)
//   their headers              contact_point_info::create_cpi, avl_tree_find<rigid_body_pair_key, ...>,
//                              phys_contact_manifold header inlines (0xa2d990-0xa2df10)
//   phys_contact_manifold.cpp  phys_contact_manifold_process::process (0xa2e080)
// The narrow-phase drivers that call process() (job input: m_cpi_allocator, m_rbc_contact_search_tree_root)
// are in phys_collision_multithreaded.cpp, where retail has them.

#include "tl/physics/include/collision/phys_collision.h"
