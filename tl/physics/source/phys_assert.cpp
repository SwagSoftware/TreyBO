// Retail: codsrc\tl\physics\source\phys_assert.cpp, TU 0xa192c0-0xa193b0.

#include "tl/physics/include/phys_assert.h"
#include "tl/physics/include/phys_base.h"
#include "tl/base/include/tl_system.h"

#include <cstdio>

phys_debug_callback_t g_phys_debug_callback;
phys_assert_info *g_list_phys_assert_info;

// TREYBO: SM3 profiler hook (see phys_base.h); no BO1 counterpart.
phys_proftimer_callbacks g_phys_proftimer_callbacks;

// 0xa192c0
void phys_set_debug_callback(phys_debug_callback_t debug_callback)
{
    g_phys_debug_callback = debug_callback;
}

// 0xa192d0
void phys_exec_debug_callback(void *data)
{
    if (g_phys_debug_callback) // TREYBO: retail calls through unconditionally
        g_phys_debug_callback(data);
}

// 0xa192e0
void PHYS_WARNING(const char *file, int line, const char *expr, const char *desc)
{
    char strbuf[512];
    snprintf(strbuf, sizeof(strbuf), "%s(%d) - '%s' \"%s\"\n", file, line, expr, desc);
    // TREYBO: retail passes strbuf as the format string; expressions such as "a % b" would then be
    // read as conversions. Print it through "%s".
    tlWarning("%s", strbuf);
}

// 0xa19340
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

// 0xa19380
void phys_assert_info::frame_advance()
{
    m_hits_frame_count = 0;
}

// 0xa19390
void phys_assert_info_frame_advance_all()
{
    for (phys_assert_info *i = g_list_phys_assert_info; i; i = i->m_next)
        i->m_hits_frame_count = 0;
}
