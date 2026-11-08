#pragma once

// Retail: codsrc\tl\base\include\tl_thread.h. The out-of-line bodies (tlAtomicAdd/And/Or,
// tlAtomicMutex::Unlock, tlSharedAtomicMutex::Lock) sit in the tl_system.cpp TU, 0xa093c0-0xa09530.
// The other mutex methods are header inlines (out-of-line copies at 0x82fe90-0x830160, 0x950c70, 0x951ba0).

#include "tl/base/include/tl_defs.h"

#if defined(_MSC_VER)
#include <intrin.h>
#define TL_THREAD_MSVC 1
#else
#define TL_THREAD_MSVC 0
#include <atomic>
#if __cplusplus >= 202002L
#define TL_THREAD_ATOMIC_REF 1
#else
#define TL_THREAD_ATOMIC_REF 0 // GCC/Clang __atomic builtins
#endif
#endif

// TREYBO: host thread helpers (retail calls GetCurrentThreadId / SwitchToThread / Sleep(0) directly).
// Defined in tl_system.cpp so this header does not pull in <Windows.h>. The id is never 0.
uint64_t tlGetCurrentThreadId();
void tlYieldThread(); // SwitchToThread()
void tlSleepZero();   // Sleep(0)

// ---------------------------------------------------------------------------------------------
// Atomic primitives. Every read-modify-write is a full barrier, like the Interlocked* family.
// They work on any 4- or 8-byte trivially copyable type (ints, pointers, tagged pairs).
// ---------------------------------------------------------------------------------------------

template <int N> struct tl_atomic_word;

template <> struct tl_atomic_word<4>
{
#if TL_THREAD_MSVC
    typedef long type;
#else
    typedef int32_t type;
#endif

    static type cas(volatile type *d, const type exch, const type comp)
    {
#if TL_THREAD_MSVC
        return _InterlockedCompareExchange(d, exch, comp);
#elif TL_THREAD_ATOMIC_REF
        type expected = comp;
        std::atomic_ref<type>(*const_cast<type *>(d)).compare_exchange_strong(expected, exch);
        return expected;
#else
        type expected = comp;
        __atomic_compare_exchange_n(const_cast<type *>(d), &expected, exch, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
        return expected;
#endif
    }

    static type add(volatile type *d, const type v)
    {
#if TL_THREAD_MSVC
        return _InterlockedExchangeAdd(d, v);
#elif TL_THREAD_ATOMIC_REF
        return std::atomic_ref<type>(*const_cast<type *>(d)).fetch_add(v);
#else
        return __atomic_fetch_add(const_cast<type *>(d), v, __ATOMIC_SEQ_CST);
#endif
    }

    static type load(const volatile type *d)
    {
#if TL_THREAD_MSVC
        return *d;
#elif TL_THREAD_ATOMIC_REF
        return std::atomic_ref<type>(*const_cast<type *>(d)).load();
#else
        return __atomic_load_n(const_cast<type *>(d), __ATOMIC_SEQ_CST);
#endif
    }

    static void store(volatile type *d, const type v)
    {
#if TL_THREAD_MSVC
        *d = v;
#elif TL_THREAD_ATOMIC_REF
        std::atomic_ref<type>(*const_cast<type *>(d)).store(v);
#else
        __atomic_store_n(const_cast<type *>(d), v, __ATOMIC_SEQ_CST);
#endif
    }
};

template <> struct tl_atomic_word<8>
{
#if TL_THREAD_MSVC
    typedef __int64 type;
#else
    typedef int64_t type;
#endif

    static type cas(volatile type *d, const type exch, const type comp)
    {
#if TL_THREAD_MSVC
        return _InterlockedCompareExchange64(d, exch, comp);
#elif TL_THREAD_ATOMIC_REF
        type expected = comp;
        std::atomic_ref<type>(*const_cast<type *>(d)).compare_exchange_strong(expected, exch);
        return expected;
#else
        type expected = comp;
        __atomic_compare_exchange_n(const_cast<type *>(d), &expected, exch, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
        return expected;
#endif
    }

    static type add(volatile type *d, const type v)
    {
#if TL_THREAD_MSVC
        type old;
        do
            old = load(d);
        while (_InterlockedCompareExchange64(d, old + v, old) != old);
        return old;
#elif TL_THREAD_ATOMIC_REF
        return std::atomic_ref<type>(*const_cast<type *>(d)).fetch_add(v);
#else
        return __atomic_fetch_add(const_cast<type *>(d), v, __ATOMIC_SEQ_CST);
#endif
    }

    static type load(const volatile type *d)
    {
#if TL_THREAD_MSVC && defined(_M_IX86)
        // A plain 64-bit read tears on x86.
        return _InterlockedCompareExchange64(const_cast<volatile type *>(d), 0, 0);
#elif TL_THREAD_MSVC
        return *d;
#elif TL_THREAD_ATOMIC_REF
        return std::atomic_ref<type>(*const_cast<type *>(d)).load();
#else
        return __atomic_load_n(const_cast<type *>(d), __ATOMIC_SEQ_CST);
#endif
    }

    static void store(volatile type *d, const type v)
    {
#if TL_THREAD_MSVC && defined(_M_IX86)
        type old;
        do
            old = load(d);
        while (_InterlockedCompareExchange64(d, v, old) != old);
#elif TL_THREAD_MSVC
        *d = v;
#elif TL_THREAD_ATOMIC_REF
        std::atomic_ref<type>(*const_cast<type *>(d)).store(v);
#else
        __atomic_store_n(const_cast<type *>(d), v, __ATOMIC_SEQ_CST);
#endif
    }
};

template <typename To, typename From> inline To tl_bit_cast(const From &from)
{
    To to;
    memcpy(&to, &from, sizeof(To));
    return to;
}

// Full memory barrier. Retail spells it "LONG Target = 0; InterlockedExchange(&Target, 0);".
inline void tlMemoryBarrier()
{
#if TL_THREAD_MSVC
    volatile long target = 0;
    _InterlockedExchange(&target, 0);
#else
    std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
}

// Returns the previous value of *dest (InterlockedCompareExchange semantics).
template <typename T> inline T tlAtomicCompareExchange(volatile T *dest, const T exch, const T comp)
{
    typedef tl_atomic_word<sizeof(T)> W;
    typedef typename W::type U;
    return tl_bit_cast<T>(W::cas((volatile U *)dest, tl_bit_cast<U>(exch), tl_bit_cast<U>(comp)));
}

// True when the swap happened (compares raw bits, so it works for tagged pairs).
template <typename T> inline bool tlAtomicCompareExchangeBool(volatile T *dest, const T exch, const T comp)
{
    typedef tl_atomic_word<sizeof(T)> W;
    typedef typename W::type U;
    const U c = tl_bit_cast<U>(comp);
    return W::cas((volatile U *)dest, tl_bit_cast<U>(exch), c) == c;
}

// Returns the previous value (InterlockedExchangeAdd semantics). Integers only.
template <typename T> inline T tlAtomicExchangeAdd(volatile T *dest, const T value)
{
    typedef tl_atomic_word<sizeof(T)> W;
    typedef typename W::type U;
    return (T)W::add((volatile U *)dest, (U)value);
}

template <typename T> inline T tlAtomicLoad(const volatile T *src)
{
    typedef tl_atomic_word<sizeof(T)> W;
    typedef typename W::type U;
    return tl_bit_cast<T>(W::load((const volatile U *)src));
}

template <typename T> inline void tlAtomicStore(volatile T *dest, const T value)
{
    typedef tl_atomic_word<sizeof(T)> W;
    typedef typename W::type U;
    W::store((volatile U *)dest, tl_bit_cast<U>(value));
}

// Retail tl atomics (tl_system.cpp TU). Each spins with Sleep(0) on contention and returns the NEW value.
uint tlAtomicAdd(volatile uint *var, uint value);                // 0xa093c0
uint64_t tlAtomicAnd(volatile uint64_t *var, uint64_t value);    // 0xa09410
uint64_t tlAtomicOr(volatile uint64_t *var, uint64_t value);     // 0xa09480

// ---------------------------------------------------------------------------------------------
// TREYBO: x64-safe tagged references for lock-free free lists.
// Retail phys_slot_pool swaps a {void *ptr, uint tag} pair with one 8-byte cmpxchg8b. On x64 that
// pair is 16 bytes. TreyBO packs a 32-bit reference (the pointer itself on x86, an offset from a
// known arena base on x64) and a 32-bit ABA tag into one 64-bit word, so the same lock-free CAS
// works on every target and the retail 8-byte layout is kept. The reference 0 means NULL.
// ---------------------------------------------------------------------------------------------

inline uint64_t tlTagged64Make(const uint32_t ref, const uint32_t tag) { return (uint64_t)ref | ((uint64_t)tag << 32); }
inline uint32_t tlTagged64Ref(const uint64_t v) { return (uint32_t)v; }
inline uint32_t tlTagged64Tag(const uint64_t v) { return (uint32_t)(v >> 32); }
inline bool tlTagged64CompareExchange(volatile uint64_t *dest, const uint64_t exch, const uint64_t comp)
{
    return tlAtomicCompareExchangeBool(dest, exch, comp);
}

// Pointer <-> 32-bit reference relative to base (base 0 on 32-bit targets: the reference IS the pointer).
inline uint32_t tlTaggedRefEncode(const void *base, const void *p)
{
    if (!p)
        return 0;
#if UINTPTR_MAX == 0xFFFFFFFFu
    (void)base;
    return (uint32_t)(uintptr_t)p;
#else
    const uintptr_t off = (uintptr_t)p - (uintptr_t)base;
    TL_ASSERT_MSG(off != 0 && off <= 0xFFFFFFFFu, "tagged reference outside its 4 GB arena");
    return (uint32_t)off;
#endif
}

inline void *tlTaggedRefDecode(const void *base, const uint32_t ref)
{
    if (!ref)
        return NULL;
#if UINTPTR_MAX == 0xFFFFFFFFu
    (void)base;
    return (void *)(uintptr_t)ref;
#else
    return (void *)((uintptr_t)base + ref);
#endif
}

#if TL_THREAD_MSVC && defined(_M_X64)
// Full 16-byte {ptr, tag} CAS for callers that need a real 64-bit pointer. dest must be 16-aligned;
// comparand[0] is the low qword. On failure comparand receives the current value.
inline bool tlAtomicCompareExchange128(volatile int64_t *dest, const int64_t exch_high, const int64_t exch_low, int64_t *comparand)
{
    return _InterlockedCompareExchange128((volatile __int64 *)dest, exch_high, exch_low, (__int64 *)comparand) != 0;
}
#endif

// ---------------------------------------------------------------------------------------------
// tlAtomicMutex: recursive spin mutex owned by a thread id. CAS goes through ThisPtr (always this).
// ---------------------------------------------------------------------------------------------

class TL_ALIGN(8) tlAtomicMutex
{
public:
    mutable volatile uint64_t ThreadId;
    mutable volatile int LockCount;
    tlAtomicMutex *ThisPtr;

    tlAtomicMutex() : ThreadId(0), LockCount(0), ThisPtr(this) {}
    ~tlAtomicMutex()
    {
        ThreadId = 0;
        ThisPtr = NULL;
    }

    void Lock() const;
    bool TryLock() const;
    void Unlock() const; // tl_system.cpp, 0xa094f0
};

// 0x950c70
inline void tlAtomicMutex::Lock() const
{
    const uint64_t CurThread = tlGetCurrentThreadId();
    if (tlAtomicLoad(&ThreadId) == CurThread)
    {
        LockCount = LockCount + 1; // plain ++ on volatile is deprecated in C++20
    }
    else
    {
        while (tlAtomicCompareExchange(&ThisPtr->ThreadId, CurThread, (uint64_t)0) != 0)
            tlYieldThread();
        tlMemoryBarrier();
        LockCount = 1;
    }
}

// 0x951ba0
inline bool tlAtomicMutex::TryLock() const
{
    const uint64_t CurThread = tlGetCurrentThreadId();
    if (tlAtomicLoad(&ThreadId) == CurThread)
    {
        LockCount = LockCount + 1;
        return true;
    }
    if (tlAtomicCompareExchange(&ThisPtr->ThreadId, CurThread, (uint64_t)0) != 0)
        return false;
    tlMemoryBarrier();
    LockCount = 1;
    return true;
}

// ---------------------------------------------------------------------------------------------
// tlSharedAtomicMutex: same as tlAtomicMutex, but every access goes through ThisPtr.
// ---------------------------------------------------------------------------------------------

class TL_ALIGN(8) tlSharedAtomicMutex
{
public:
    mutable volatile uint64_t ThreadId;
    mutable volatile int LockCount;
    tlSharedAtomicMutex *ThisPtr;

    tlSharedAtomicMutex() : ThreadId(0), LockCount(0), ThisPtr(this) {} // TREYBO: ctor not in binary, mirrors tlAtomicMutex
    ~tlSharedAtomicMutex() { ThisPtr = NULL; }                           // TREYBO: dtor not in binary

    void Lock() const; // tl_system.cpp, 0xa09530
    void Unlock() const;
};

// TREYBO: Unlock is not in the binary; it mirrors tlAtomicMutex::Unlock through ThisPtr.
inline void tlSharedAtomicMutex::Unlock() const
{
    const int count = ThisPtr->LockCount;
    ThisPtr->LockCount = count - 1;
    if (count == 1)
    {
        tlMemoryBarrier();
        tlAtomicStore(&ThisPtr->ThreadId, (uint64_t)0);
    }
}

// ---------------------------------------------------------------------------------------------
// tlAtomicReadWriteMutex: WriteThreadId doubles as a short-lived guard for the read count.
// ---------------------------------------------------------------------------------------------

class TL_ALIGN(8) tlAtomicReadWriteMutex
{
public:
    mutable volatile uint64_t WriteThreadId;
    mutable volatile int ReadLockCount;
    mutable volatile int WriteLockCount;
    tlAtomicReadWriteMutex *ThisPtr;

    // inlined in broad_phase_memory::broad_phase_memory (0xa20e20)
    tlAtomicReadWriteMutex() : WriteThreadId(0), ReadLockCount(0), WriteLockCount(0), ThisPtr(this) {}
    ~tlAtomicReadWriteMutex() { ThisPtr = NULL; } // 0xa1f580

    void WriteLock() const;
    void ReadLock() const;
    void WriteUnlock() const;
    void ReadUnlock() const;
};

// 0x82fe90
inline void tlAtomicReadWriteMutex::WriteLock() const
{
    const uint64_t CurThread = tlGetCurrentThreadId();
    if (tlAtomicCompareExchange(&ThisPtr->WriteThreadId, CurThread, CurThread) == CurThread)
    {
        tlAtomicExchangeAdd(&ThisPtr->WriteLockCount, 1);
        return;
    }
    for (;;)
    {
        if (!tlAtomicCompareExchange(&ThisPtr->WriteThreadId, CurThread, (uint64_t)0))
        {
            if (!tlAtomicCompareExchange(&ThisPtr->ReadLockCount, 0, 0))
                break;
            // readers still inside: give the guard back and retry
            while (tlAtomicCompareExchange(&ThisPtr->WriteThreadId, (uint64_t)0, CurThread) != CurThread)
                ;
        }
        tlYieldThread();
    }
    tlAtomicExchangeAdd(&ThisPtr->WriteLockCount, 1);
    tlMemoryBarrier();
}

// 0x830010
inline void tlAtomicReadWriteMutex::ReadLock() const
{
    const uint64_t CurThread = tlGetCurrentThreadId();
    if (tlAtomicCompareExchange(&ThisPtr->WriteThreadId, CurThread, CurThread) == CurThread)
    {
        tlAtomicExchangeAdd(&ThisPtr->ReadLockCount, 1);
    }
    else
    {
        while (tlAtomicCompareExchange(&ThisPtr->WriteThreadId, CurThread, (uint64_t)0))
            tlYieldThread();
        tlAtomicExchangeAdd(&ThisPtr->ReadLockCount, 1);
        while (tlAtomicCompareExchange(&ThisPtr->WriteThreadId, (uint64_t)0, CurThread) != CurThread)
            ;
    }
    tlMemoryBarrier();
}

// 0x830160
inline void tlAtomicReadWriteMutex::WriteUnlock() const
{
    const uint64_t CurThread = tlGetCurrentThreadId();
    if (tlAtomicExchangeAdd(&ThisPtr->WriteLockCount, -1) == 1)
    {
        tlMemoryBarrier();
        while (tlAtomicCompareExchange(&ThisPtr->WriteThreadId, (uint64_t)0, CurThread) != CurThread)
            ;
    }
}

// No out-of-line copy; inlined in phys_heap_gjk_cache_system_avl_tree::get_gjk_cache_info_mutex (0xa20bc0).
inline void tlAtomicReadWriteMutex::ReadUnlock() const
{
    tlAtomicExchangeAdd(&ThisPtr->ReadLockCount, -1);
}
