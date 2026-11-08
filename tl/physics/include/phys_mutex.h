#pragma once

// Retail: codsrc\tl\physics\include\phys_mutex.h (header inlines; out-of-line copies at 0x8387e0-0x838cb0).

#include "tl/base/include/tl_defs.h"
#include "tl/base/include/tl_thread.h"

// Atomic read of a shared word. Retail asserts spell their checks with it ("GetStuff32(&m_token) == 1").
// TREYBO: reads the native width, so pointer fields stay correct on x64.
template <typename T> inline T GetStuff32(const volatile T *p)
{
    return tlAtomicLoad(p);
}

// Non-recursive spin lock: m_token 0 = free, 1 = held.
struct minspec_mutex
{
    volatile uint m_token;

    minspec_mutex() : m_token(0) {} // inlined in phys_memory_manager::phys_memory_manager (0xa0f9b0)

    // 0x838c70
    void Lock()
    {
        while (tlAtomicCompareExchange(&m_token, 1u, 0u))
            ;
        tlMemoryBarrier();
    }

    // 0x838cb0
    void Unlock()
    {
        tlMemoryBarrier();
        PHYS_ASSERT(GetStuff32(&m_token) == 1);
        const bool retv = tlAtomicCompareExchange(&m_token, 0u, 1u) == 1;
        PHYS_ASSERT(retv);
        (void)retv;
    }
};

// Spin read/write lock: m_count 1 = free, 0 = write-held, n > 1 = (n - 1) readers.
struct minspec_read_write_mutex
{
    volatile uint m_count;

    minspec_read_write_mutex() : m_count(1) {} // inlined in the phys_memory_manager and broad_phase_memory ctors

    // 0x8387e0
    void ReadLock()
    {
        uint count;
        do
        {
            do
                count = m_count;
            while (!count);
        } while (tlAtomicCompareExchange(&m_count, count + 1, count) != count);
        tlMemoryBarrier();
    }

    // 0x838840
    void ReadUnlock()
    {
        uint count;
        do
        {
            count = m_count;
            PHYS_ASSERT(count > 1);
        } while (tlAtomicCompareExchange(&m_count, count - 1, count) != count);
    }

    // 0x8388b0
    void WriteLock()
    {
        while (tlAtomicCompareExchange(&m_count, 0u, 1u) != 1)
            ;
        tlMemoryBarrier();
    }

    // 0x838900
    void WriteUnlock()
    {
        tlMemoryBarrier();
        PHYS_ASSERT(GetStuff32(&m_count) == 0);
        const bool retv = tlAtomicCompareExchange(&m_count, 1u, 0u) == 0;
        PHYS_ASSERT(retv);
        (void)retv;
    }
};
