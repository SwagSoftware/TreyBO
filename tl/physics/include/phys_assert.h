#pragma once

// Retail: codsrc\tl\physics\include\phys_assert.h. Source TU phys_assert.cpp, 0xa192c0-0xa193b0.

#include "tl/base/include/tl_defs.h"
#include "tl/base/include/tl_thread.h"

// A rate-limited assert site: hits beyond the totals are counted but not reported, and
// m_use_warnings_only turns the assert into a PHYS_WARNING. Instances self-register in a global list.
class phys_assert_info
{
public:
    volatile int m_hits_total_count;
    volatile int m_hits_frame_count;
    int m_max_hits_total; // 0 = unlimited
    int m_max_hits_per_frame;
    bool m_use_warnings_only;
    phys_assert_info *m_next;

    phys_assert_info(int max_hits_total, int max_hits_per_frame, bool use_warnings_only); // 0xa19340
    void frame_advance();                                                              // 0xa19380
};

void phys_assert_info_frame_advance_all(); // 0xa19390

// 0xa192e0: prints "file(line) - 'expr' \"desc\"" through tlWarning.
void PHYS_WARNING(const char *file, int line, const char *expr, const char *desc);

typedef void (*phys_debug_callback_t)(void *data); // TREYBO: typedef name
void phys_set_debug_callback(phys_debug_callback_t debug_callback); // 0xa192c0
void phys_exec_debug_callback(void *data);                          // 0xa192d0

// The phys_assert_info check, as inlined at every retail site (e.g. contact_point_info::create_cpi 0xa2ddf0,
// phys_heap_gjk_cache_system_avl_tree::get_gjk_cache_info_mutex 0xa20bc0). TREYBO: macro name not recovered.
#if TL_ASSERTS_ENABLED
#define PHYS_ASSERT_INFO_STR(pai, expr, exprstr, desc)                                                         \
    do                                                                                                         \
    {                                                                                                          \
        if (!(expr))                                                                                           \
        {                                                                                                      \
            if (((pai).m_hits_total_count < (pai).m_max_hits_total || !(pai).m_max_hits_total) &&             \
                (pai).m_hits_frame_count < (pai).m_max_hits_per_frame)                                         \
            {                                                                                                  \
                if ((pai).m_use_warnings_only)                                                                 \
                    PHYS_WARNING(__FILE__, __LINE__, exprstr, desc);                                           \
                else if (_tlAssert(__FILE__, __LINE__, exprstr, desc))                                         \
                    TL_DEBUGBREAK();                                                                           \
            }                                                                                                  \
            tlAtomicExchangeAdd(&(pai).m_hits_total_count, 1);                                                 \
            tlAtomicExchangeAdd(&(pai).m_hits_frame_count, 1);                                                 \
        }                                                                                                      \
    } while (0)
#else
#define PHYS_ASSERT_INFO_STR(pai, expr, exprstr, desc) ((void)0)
#endif
#define PHYS_ASSERT_INFO(pai, expr, desc) PHYS_ASSERT_INFO_STR(pai, expr, #expr, desc)

// From kisakblack phys_assert.h (retail placement unknown).
// TREYBO: a const pointer has internal linkage, so the header inlines that pass it do not odr-use a different
// object per TU, and unused copies draw no warnings.
static const char *const SOLVER_MEMORY_ALLOCATOR_ERROR_MSG = "physics constraint solver OUT OF MEMORY.";
