#pragma once

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
	phys_assert_info * m_next;

	phys_assert_info(int max_hits_total, int max_hits_per_frame, bool use_warnings_only);
	void frame_advance();
};

void phys_assert_info_frame_advance_all();

// Prints "file(line) - 'expr' \"desc\"" through tlWarning.
void PHYS_WARNING(const char * file, int line, const char * expr, const char * desc);

typedef void (*phys_debug_callback_t)(void * data);
void phys_set_debug_callback(phys_debug_callback_t debug_callback);
void phys_exec_debug_callback(void * data);

// Assert through a phys_assert_info.
#if TL_ASSERTS_ENABLED
#define PHYS_ASSERT_INFO_STR(pai, expr, exprstr, desc)                                            \
	do                                                                                            \
	{                                                                                             \
		if (!(expr))                                                                              \
		{                                                                                         \
			if (((pai).m_hits_total_count < (pai).m_max_hits_total || !(pai).m_max_hits_total) && \
				(pai).m_hits_frame_count < (pai).m_max_hits_per_frame)                            \
			{                                                                                     \
				if ((pai).m_use_warnings_only)                                                    \
					PHYS_WARNING(__FILE__, __LINE__, exprstr, desc);                              \
				else if (_tlAssert(__FILE__, __LINE__, exprstr, desc))                            \
					TL_DEBUGBREAK();                                                              \
			}                                                                                     \
			tlAtomicExchangeAdd(&(pai).m_hits_total_count, 1);                                    \
			tlAtomicExchangeAdd(&(pai).m_hits_frame_count, 1);                                    \
		}                                                                                         \
	} while (0)
#else
#define PHYS_ASSERT_INFO_STR(pai, expr, exprstr, desc) ((void)0)
#endif
#define PHYS_ASSERT_INFO(pai, expr, desc) PHYS_ASSERT_INFO_STR(pai, expr, #expr, desc)

// A const pointer has internal linkage, so the header inlines that pass it do not odr-use a different
// object per TU, and unused copies draw no warnings.
static const char * const SOLVER_MEMORY_ALLOCATOR_ERROR_MSG = "physics constraint solver OUT OF MEMORY.";
