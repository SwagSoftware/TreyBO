#pragma once

// Accumulated impulse rate of one solver row, kept across steps for warm starting. The default constructor
// zeroes it, so a new constraint's cache arrays start at zero.

#include "tl/base/include/tl_defs.h"

class pulse_sum_cache
{
public:
	// Public: constraint code reads m_pulse_sum directly.
	float m_pulse_sum;

	pulse_sum_cache() :
		m_pulse_sum(0.0f) {}

	const float get_pulse_sum() const { return m_pulse_sum; }
};
