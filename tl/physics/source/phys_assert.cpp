#include "tl/physics/include/phys_assert.h"
#include "tl/physics/include/phys_base.h"
#include "tl/base/include/tl_system.h"

#include <cstdio>

phys_debug_callback_t g_phys_debug_callback;
phys_assert_info * g_list_phys_assert_info;

// Profiler hook (see phys_base.h).
phys_proftimer_callbacks g_phys_proftimer_callbacks;

void phys_set_debug_callback(phys_debug_callback_t debug_callback)
{
	g_phys_debug_callback = debug_callback;
}

void phys_exec_debug_callback(void * data)
{
	if (g_phys_debug_callback)
		g_phys_debug_callback(data);
}

void PHYS_WARNING(const char * file, int line, const char * expr, const char * desc)
{
	char strbuf[512];
	snprintf(strbuf, sizeof(strbuf), "%s(%d) - '%s' \"%s\"\n", file, line, expr, desc);
	// Through "%s": an expression such as "a % b" must not be read as a conversion.
	tlWarning("%s", strbuf);
}

phys_assert_info::phys_assert_info(int max_hits_total, int max_hits_per_frame, bool use_warnings_only)
{
	m_max_hits_total = max_hits_total;
	m_hits_total_count = 0;
	m_hits_frame_count = 0;
	m_max_hits_per_frame = max_hits_per_frame;
	m_use_warnings_only = use_warnings_only;
	m_next = g_list_phys_assert_info;
	g_list_phys_assert_info = this;
}

void phys_assert_info::frame_advance()
{
	m_hits_frame_count = 0;
}

void phys_assert_info_frame_advance_all()
{
	for (phys_assert_info * i = g_list_phys_assert_info; i; i = i->m_next)
		i->m_hits_frame_count = 0;
}
