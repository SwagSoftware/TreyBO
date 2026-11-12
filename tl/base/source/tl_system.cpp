#if defined(_MSC_VER) && !defined(_CRT_SECURE_NO_WARNINGS)
#define _CRT_SECURE_NO_WARNINGS // fopen / strncpy
#endif

#include "tl/base/include/tl_system.h"
#include "tl/base/include/tl_thread.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <Psapi.h>
#include <malloc.h>
#if defined(_MSC_VER)
#pragma comment(lib, "psapi.lib")
#endif
#else
#include <atomic>
#include <chrono>
#include <thread>
#endif

tlSystemCallbacks tlCurSystemCallbacks;
char tlHostPrefix[256];
int tlMemAllocCounter;
int tlScratchPadRefCount;
bool tlScratchpadLocked;
void * tlScratchPadPtr;
void * tlStackBegin;
void * tlStackEnd;
float tlPcTicksPerMS = 1.0f;

// ---------------------------------------------------------------------------------------------
// Platform defaults: _aligned_malloc / OutputDebugStringA on Windows, a portable equivalent elsewhere.
// ---------------------------------------------------------------------------------------------

namespace
{
// _aligned_malloc rejects an alignment of 0.
inline uint tl_default_align(const uint Align)
{
	return Align ? Align : 16u;
}

#if !defined(_WIN32)
struct tl_alloc_header
{
	void * base;
	size_t size;
};

void * tl_aligned_malloc(const size_t size, const size_t align)
{
	const size_t a = align < sizeof(void *) ? sizeof(void *) : align;
	char * const base = (char *)malloc(size + a + sizeof(tl_alloc_header));
	if (!base)
		return NULL;
	char * const p = (char *)tl_align((size_t)(base + sizeof(tl_alloc_header)), (int)a);
	tl_alloc_header * const h = (tl_alloc_header *)p - 1;
	h->base = base;
	h->size = size;
	return p;
}

void tl_aligned_free(void * p)
{
	if (p)
		free(((tl_alloc_header *)p - 1)->base);
}

void * tl_aligned_realloc(void * p, const size_t size, const size_t align)
{
	if (!p)
		return tl_aligned_malloc(size, align);
	if (!size)
	{
		tl_aligned_free(p);
		return NULL;
	}
	void * const n = tl_aligned_malloc(size, align);
	if (!n)
		return NULL;
	const size_t old_size = ((tl_alloc_header *)p - 1)->size;
	memcpy(n, p, old_size < size ? old_size : size);
	tl_aligned_free(p);
	return n;
}
#endif

void tl_default_free(void * p)
{
#if defined(_WIN32)
	_aligned_free(p);
#else
	tl_aligned_free(p);
#endif
}

void * tl_default_malloc(const uint size, const uint align)
{
#if defined(_WIN32)
	return _aligned_malloc(size, tl_default_align(align));
#else
	return tl_aligned_malloc(size, tl_default_align(align));
#endif
}

void * tl_default_realloc(void * p, const uint size, const uint align)
{
#if defined(_WIN32)
	return _aligned_realloc(p, size, tl_default_align(align));
#else
	return tl_aligned_realloc(p, size, tl_default_align(align));
#endif
}

void tl_output_debug_string(const char * txt)
{
#if defined(_WIN32)
	OutputDebugStringA(txt);
#else
	fputs(txt, stderr);
#endif
}

// Shared "no CriticalError callback" path of _tlAssert, tlFatalHandler and tlFatal.
void tl_print_fatal(const char * msg)
{
	tl_output_debug_string("TL Fatal Error: ");
	tl_output_debug_string(msg);
	tl_output_debug_string("\n");
}
}

void tlSetSystemCallbacks(const tlSystemCallbacks * Callbacks)
{
	memcpy(&tlCurSystemCallbacks, Callbacks, sizeof(tlCurSystemCallbacks));
}

void tlSetFileServerRootPC(const char * Path)
{
	strncpy(tlHostPrefix, Path, sizeof(tlHostPrefix) - 1);
	tlHostPrefix[sizeof(tlHostPrefix) - 1] = 0;
}

void tlMemFree(void * Ptr)
{
	--tlMemAllocCounter;
	if (tlCurSystemCallbacks.MemFree)
		tlCurSystemCallbacks.MemFree(Ptr);
	else
		tl_default_free(Ptr);
}

void tlReleaseFile(tlFileBuf * File)
{
	if (tlCurSystemCallbacks.ReleaseFile)
	{
		tlCurSystemCallbacks.ReleaseFile(File);
	}
	else
	{
		tlMemFree(File->Buf);
		File->Buf = NULL;
		File->Size = 0;
		File->UserData = 0;
	}
}

uint64_t tlPcGetTick()
{
#if defined(_WIN32)
	LARGE_INTEGER li;
	QueryPerformanceCounter(&li);
	return (uint64_t)li.QuadPart;
#else
	return (uint64_t)std::chrono::steady_clock::now().time_since_epoch().count();
#endif
}

void tlPrint(const char * txt)
{
	tl_output_debug_string(txt);
}

int tlGetVersion()
{
	return 66560;
}

void tlStackRangeInit()
{
	tlStackBegin = NULL;
	tlStackEnd = NULL;
}

// Returns true when the caller should break.
bool tlFatalHandler(const char * Msg)
{
	if (tlCurSystemCallbacks.CriticalError)
	{
		tlCurSystemCallbacks.CriticalError(Msg);
		return false;
	}
	tl_print_fatal(Msg);
	return true;
}

void tlDebugPrint(const char * txt)
{
	if (tlCurSystemCallbacks.DebugPrint)
		tlCurSystemCallbacks.DebugPrint(txt);
	else
		tl_output_debug_string(txt);
}

void tlVPrintf(const char * Format, va_list args)
{
	char Work[512];
	vsnprintf(Work, sizeof(Work), Format, args);
	Work[sizeof(Work) - 1] = 0;
	tlDebugPrint(Work);
}

void tlPrintf(const char * Format, ...)
{
	va_list ap;
	va_start(ap, Format);
	tlVPrintf(Format, ap);
	va_end(ap);
}

bool _tlAssert(const char * file, int line, const char * expr, const char * desc)
{
	char Buf[256];
	snprintf(Buf, sizeof(Buf), "ASSERT in %s(%d):\n\"%s\" - %s", file, line, expr, desc);
	Buf[255] = 0;
	return tlFatalHandler(Buf);
}

void tlFatal(const char * Format, ...)
{
	char Work[512];
	va_list ap;
	va_start(ap, Format);
	vsnprintf(Work, sizeof(Work), Format, ap);
	va_end(ap);
	Work[sizeof(Work) - 1] = 0;
	if (tlCurSystemCallbacks.CriticalError)
	{
		tlCurSystemCallbacks.CriticalError(Work);
	}
	else
	{
		tl_print_fatal(Work);
		TL_DEBUGBREAK();
	}
}

void * tlMemAlloc(uint Size, uint Align, uint Flags)
{
	++tlMemAllocCounter;
	void * const Ptr = tlCurSystemCallbacks.MemAlloc ? tlCurSystemCallbacks.MemAlloc(Size, Align, Flags)
													 : tl_default_malloc(Size, Align);
	if (!(Flags & TL_MEMFLAG_NO_ERROR) && !Ptr && Size)
		tlFatal("Memory allocation failed. %d bytes, %d align", Size, Align);
	return Ptr;
}

// Does not touch tlMemAllocCounter.
void * tlMemRealloc(void * Ptr, uint Size, uint Align, uint Flags)
{
	void * const NewPtr = tlCurSystemCallbacks.MemRealloc ? tlCurSystemCallbacks.MemRealloc(Ptr, Size, Align, Flags)
														  : tl_default_realloc(Ptr, Size, Align);
	if (!(Flags & TL_MEMFLAG_NO_ERROR) && !NewPtr && Size)
		tlFatal("Memory reallocation failed.");
	return NewPtr;
}

// Total physical memory minus this process's peak working set.
uint tlGetFreeMemory()
{
#if defined(_WIN32)
	uint result = 0;
	MEMORYSTATUS stat;
	GlobalMemoryStatus(&stat);
	const SIZE_T dwTotalPhys = stat.dwTotalPhys;
	const HANDLE process = OpenProcess(PROCESS_QUERY_INFORMATION | PROCESS_VM_READ, FALSE, GetCurrentProcessId());
	PROCESS_MEMORY_COUNTERS pmc;
	if (GetProcessMemoryInfo(process, &pmc, sizeof(pmc)))
		result = (uint)(dwTotalPhys - pmc.PeakWorkingSetSize);
	CloseHandle(process);
	return result;
#else
	return 0; // no portable query
#endif
}

void * tlScratchPadInit()
{
	if (tlScratchPadRefCount++)
		return tlScratchPadPtr;
	++tlMemAllocCounter;
	void * const Ptr = tlCurSystemCallbacks.MemAlloc ? tlCurSystemCallbacks.MemAlloc(0x4000, 16, 0)
													 : tl_default_malloc(0x4000, 16);
	if (!Ptr)
		tlFatal("Memory allocation failed. %d bytes, %d align", 0x4000, 16);
	tlScratchPadPtr = Ptr;
	return Ptr;
}

void tlScratchPadReset()
{
	TL_ASSERT_MSG(tlScratchPadRefCount >= 1, "Scratchpad reset underflow.");
	if (!--tlScratchPadRefCount)
	{
		tlMemFree(tlScratchPadPtr);
		tlScratchPadPtr = NULL;
	}
}

void tlWarning(const char * Format, ...)
{
	char Work[512];
	va_list ap;
	va_start(ap, Format);
	vsnprintf(Work, sizeof(Work), Format, ap);
	va_end(ap);
	Work[sizeof(Work) - 1] = 0;
	if (tlCurSystemCallbacks.Warning)
		tlCurSystemCallbacks.Warning(Work);
	else
		tlPrintf("%s", Work);
}

bool tlReadFile(const char * FileName, tlFileBuf * File, uint Align, uint Flags)
{
	if (tlCurSystemCallbacks.ReadFile)
		return tlCurSystemCallbacks.ReadFile(FileName, File, Align, Flags);

	// Prepend the file-server root unless the name already starts with it.
	char Work[512];
	const char * const prefix = strncmp(FileName, tlHostPrefix, strlen(tlHostPrefix)) ? tlHostPrefix : "";
	snprintf(Work, sizeof(Work), "%s%s", prefix, FileName);
#if defined(_WIN32)
	for (char * i = Work; *i; ++i)
	{
		if (*i == '/')
			*i = '\\';
	}
#endif

	FILE * const fp = fopen(Work, "rb");
	if (!fp)
		return false;
	fseek(fp, 0, SEEK_END);
	const long FileSize = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	File->Size = (uint)(FileSize > 0 ? FileSize : 0);
	File->Buf = (uchar *)tlMemAlloc(File->Size, Align, Flags);
	if (File->Buf) // NULL when the allocation failed with TL_MEMFLAG_NO_ERROR
		fread(File->Buf, 1, File->Size, fp);
	fclose(fp);
	return true;
}

// Runs job_count jobs through the host's RunJobs callback (or inline) and returns when all are done.
void tlRunJobs(tlJobFunc func, void * user_data, int job_count)
{
	if (job_count <= 0)
		return;
	if (tlCurSystemCallbacks.RunJobs)
	{
		tlCurSystemCallbacks.RunJobs(func, user_data, job_count);
		return;
	}
	for (int i = 0; i < job_count; ++i)
		func(user_data, i);
}

// ---------------------------------------------------------------------------------------------
// tl_thread.h out-of-line bodies
// ---------------------------------------------------------------------------------------------

uint64_t tlGetCurrentThreadId()
{
#if defined(_WIN32)
	return (uint64_t)GetCurrentThreadId();
#else
	static std::atomic<uint64_t> s_next_id(1);
	static thread_local uint64_t s_id = s_next_id.fetch_add(1);
	return s_id;
#endif
}

void tlYieldThread()
{
#if defined(_WIN32)
	SwitchToThread();
#else
	std::this_thread::yield();
#endif
}

void tlSleepZero()
{
#if defined(_WIN32)
	Sleep(0);
#else
	std::this_thread::yield();
#endif
}

uint tlAtomicAdd(volatile uint * var, uint value)
{
	uint old = tlAtomicLoad(var);
	while (tlAtomicCompareExchange(var, old + value, old) != old)
	{
		tlSleepZero();
		old = tlAtomicLoad(var);
	}
	return old + value;
}

uint64_t tlAtomicAnd(volatile uint64_t * var, uint64_t value)
{
	uint64_t old = tlAtomicLoad(var);
	while (tlAtomicCompareExchange(var, value & old, old) != old)
	{
		tlSleepZero();
		old = tlAtomicLoad(var);
	}
	return value & old;
}

uint64_t tlAtomicOr(volatile uint64_t * var, uint64_t value)
{
	uint64_t old = tlAtomicLoad(var);
	while (tlAtomicCompareExchange(var, value | old, old) != old)
	{
		tlSleepZero();
		old = tlAtomicLoad(var);
	}
	return value | old;
}

// Touches this, not ThisPtr.
void tlAtomicMutex::Unlock() const
{
	const int count = LockCount;
	LockCount = count - 1;
	if (count == 1)
	{
		tlMemoryBarrier();
		tlAtomicStore(&ThreadId, (uint64_t)0);
	}
}

void tlSharedAtomicMutex::Lock() const
{
	const uint64_t CurThread = tlGetCurrentThreadId();
	if (tlAtomicLoad(&ThisPtr->ThreadId) == CurThread)
	{
		ThisPtr->LockCount = ThisPtr->LockCount + 1;
	}
	else
	{
		while (tlAtomicCompareExchange(&ThisPtr->ThreadId, CurThread, (uint64_t)0))
			tlYieldThread();
		tlMemoryBarrier();
		ThisPtr->LockCount = 1;
	}
}
