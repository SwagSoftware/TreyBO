#pragma once

// The constraint types are not virtual: the solver walks one list per type (rb_inplace_partition_node)
// and calls each type's methods directly. These base versions are the defaults a type inherits.

#include "tl/base/include/tl_defs.h"
#include "tl/physics/include/rigid_body.h"
#include "tl/physics/include/constraint_solver/pulse_sum_cache.h"

class pulse_sum_constraint_solver;

class rigid_body_constraint
{
public:
	// Public: the partition / solver code walks m_next directly.
	rigid_body * b1;
	rigid_body * b2;
	rigid_body_constraint * m_next; // next constraint of the same type in the island

	rigid_body * get_b1() const { return b1; }
	rigid_body * get_b2() const { return b2; }

	void do_collision(const float) {}
	void setup_constraint(pulse_sum_constraint_solver *, const float) { PHYS_ASSERT(0); }
	void epilog_vel_constraint(const float) {}

	void outer_prolog_update(const float) {}
	void inner_update(const float) {}
	void outer_epilog_update(const float) {}
};
