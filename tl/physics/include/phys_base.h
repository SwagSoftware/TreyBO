#pragma once

// Retail: codsrc\tl\physics\include\phys_base.h. Header only; no out-of-line code in CoDMPServer.exe.
// Layout and bodies from SM3 phys_base.h. BO1 differences:
//  - outer_time is gone: BO1 constraint updates take a plain float (e.g. outer_prolog_update(float) at 0xa17aa0).
//  - phys_proftimer_callbacks has no symbol in BO1 (CoD used cdl_proftimer). TreyBO keeps the SM3 hook as the
//    host profiler interface.

#include "tl/base/include/tl_defs.h"
#include "tl/base/include/tl_system.h"

// BO1 assert text uses PHYS_ALIGNOF ("slot_alignment % PHYS_ALIGNOF(phys_slot_pool::extra_info) == 0").
#define PHYS_ALIGNOF(type) TL_ALIGNOF(type)

inline const size_t PHYS_ALIGN(const size_t p, const int a)
{
    return tl_align(p, a);
}

inline const size_t PHYS_ALIGN(char *p, const int a)
{
    return (size_t)tl_align(p, a);
}

inline const bool rel_cmp_ge(const float v1, const float v2, const float e)
{
    return (v1 >= (1.0f - e) * v2);
}

inline const bool rel_cmp_le(const float v1, const float v2, const float e)
{
    return ((1.0f - e) * v1 <= v2);
}

inline const bool rel_cmp_e(const float v1, const float v2, const float e)
{
    const float k = 1.0f - e;
    return (v1 >= k * v2 && v2 >= k * v1);
}

// Enumerators are SM3 phys_proftimers.inc.
enum phys_proftimer_e
{
    phys_proftimer_phys_col_detect,
    phys_proftimer_phys_misc,
    phys_proftimer_phys_solver,
    phys_proftimer_phys_solver_setup,
    phys_proftimer_phys_solver_iterative,
    phys_proftimer_phys_solver_epilog,
    num_phys_proftimers
};

class phys_proftimer_callbacks
{
public:
    phys_proftimer_callbacks() : proftimer_start(0), proftimer_stop(0) {}
    void (*proftimer_start)(phys_proftimer_e p);
    void (*proftimer_stop)(phys_proftimer_e p);
};

// TREYBO: defined in phys_assert.cpp (no BO1 symbol); set by phys_sys::set_phys_proftimer_callbacks.
extern phys_proftimer_callbacks g_phys_proftimer_callbacks;

inline void PHYS_START_PROF_TIMER(phys_proftimer_e p)
{
    if (g_phys_proftimer_callbacks.proftimer_start)
        (*g_phys_proftimer_callbacks.proftimer_start)(p);
}

inline void PHYS_STOP_PROF_TIMER(phys_proftimer_e p)
{
    if (g_phys_proftimer_callbacks.proftimer_stop)
        (*g_phys_proftimer_callbacks.proftimer_stop)(p);
}
