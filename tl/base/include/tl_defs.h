#pragma once

// Treyarch "tl" base definitions, shared by every tl header. Keep this file free of physics types.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cmath>

typedef unsigned int uint;
typedef unsigned char uchar;
typedef unsigned short ushort;

#if defined(_MSC_VER)
#define TL_FORCEINLINE __forceinline
#define TL_NOINLINE __declspec(noinline)
#define TL_DEBUGBREAK() __debugbreak()
#else
#define TL_FORCEINLINE inline __attribute__((always_inline))
#define TL_NOINLINE __attribute__((noinline))
#define TL_DEBUGBREAK() __builtin_trap()
#endif

#define TL_ALIGN(n) alignas(n)
#define TL_ALIGNOF(type) alignof(type)
#define TL_ARRAY_COUNT(a) (sizeof(a) / sizeof((a)[0]))

template <typename T> inline T tl_min(const T a, const T b)
{
	return a < b ? a : b;
}
template <typename T> inline T tl_max(const T a, const T b)
{
	return a > b ? a : b;
}
template <typename T> inline T tl_clamp(const T v, const T lo, const T hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}
template <typename T> inline void tl_swap(T & a, T & b)
{
	T t = a;
	a = b;
	b = t;
}

inline size_t tl_align(const size_t p, const int a)
{
	return (p + (size_t)(a - 1)) & ~(size_t)(a - 1);
}
template <typename T> inline T * tl_align(T * p, const int a)
{
	return (T *)tl_align((size_t)p, a);
}

// Assert plumbing. _tlAssert reports through the host callbacks (tl_system.h) and returns
// true when the caller should break.
bool _tlAssert(const char * file, int line, const char * expr, const char * desc);

#ifndef TL_ASSERTS_ENABLED
#if defined(_DEBUG) || defined(TREYBO_ASSERTS)
#define TL_ASSERTS_ENABLED 1
#else
#define TL_ASSERTS_ENABLED 0
#endif
#endif

#if TL_ASSERTS_ENABLED
// exprstr is the reported assert text; use the _STR form when the condition
// does not spell the same as the message.
#define TL_ASSERT_STR(expr, exprstr) ((!(expr) && _tlAssert(__FILE__, __LINE__, exprstr, "")) ? TL_DEBUGBREAK() : (void)0)
#define TL_ASSERT_MSG(expr, desc) ((!(expr) && _tlAssert(__FILE__, __LINE__, #expr, desc)) ? TL_DEBUGBREAK() : (void)0)
#else
#define TL_ASSERT_STR(expr, exprstr) ((void)0)
#define TL_ASSERT_MSG(expr, desc) ((void)0)
#endif
#define TL_ASSERT(expr) TL_ASSERT_STR(expr, #expr)

#define PHYS_ASSERT(expr) TL_ASSERT(expr)
#define PHYS_ASSERT_STR(expr, exprstr) TL_ASSERT_STR(expr, exprstr)
#define PHYS_ASSERT_MSG(expr, desc) TL_ASSERT_MSG(expr, desc)

// Assert text and a description together.
#if TL_ASSERTS_ENABLED
#define TL_ASSERT_STR_MSG(expr, exprstr, desc) ((!(expr) && _tlAssert(__FILE__, __LINE__, exprstr, desc)) ? TL_DEBUGBREAK() : (void)0)
#else
#define TL_ASSERT_STR_MSG(expr, exprstr, desc) ((void)0)
#endif
#define PHYS_ASSERT_STR_MSG(expr, exprstr, desc) TL_ASSERT_STR_MSG(expr, exprstr, desc)
