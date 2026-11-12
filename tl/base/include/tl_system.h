#pragma once

// The host installs tlSystemCallbacks; every tl* service falls back to a portable default when a callback is NULL.

#include "tl/base/include/tl_defs.h"
#include <cstdarg>

struct tlFileBuf
{
	uchar * Buf;
	uint Size;
	uint UserData;
};

// tlMemAlloc / tlMemRealloc flags.
enum
{
	TL_MEMFLAG_NO_ERROR = 2, // no fatal error on failure
};

// Job dispatch. RunJobs must call func(user_data, i) once for every i in [0, job_count), on any threads,
// and return only after every call has finished.
typedef void (*tlJobFunc)(void * user_data, int job_index);
typedef void (*tlRunJobsCallback)(tlJobFunc func, void * user_data, int job_count);

struct tlSystemCallbacks
{
	bool (*ReadFile)(const char *, tlFileBuf *, uint, uint);
	void (*ReleaseFile)(tlFileBuf *);
	void (*CriticalError)(const char *);
	void (*Warning)(const char *);
	void (*DebugPrint)(const char *);
	void * (*MemAlloc)(uint, uint, uint);
	void * (*MemRealloc)(void *, uint, uint, uint);
	void (*MemFree)(void *);
	tlRunJobsCallback RunJobs;
};

extern tlSystemCallbacks tlCurSystemCallbacks;
extern int tlMemAllocCounter;
extern int tlScratchPadRefCount;
extern bool tlScratchpadLocked;
extern void * tlScratchPadPtr;
extern char tlHostPrefix[256];
extern void * tlStackBegin;
extern void * tlStackEnd;
extern float tlPcTicksPerMS;

void tlSetSystemCallbacks(const tlSystemCallbacks * Callbacks);
void tlSetFileServerRootPC(const char * Path);

void * tlMemAlloc(uint Size, uint Align, uint Flags);
void * tlMemRealloc(void * Ptr, uint Size, uint Align, uint Flags);
void tlMemFree(void * Ptr);
uint tlGetFreeMemory();

bool tlReadFile(const char * FileName, tlFileBuf * File, uint Align, uint Flags);
void tlReleaseFile(tlFileBuf * File);

uint64_t tlPcGetTick();
int tlGetVersion();
void tlStackRangeInit();

void tlPrint(const char * txt);
void tlDebugPrint(const char * txt);
void tlVPrintf(const char * Format, va_list args);
void tlPrintf(const char * Format, ...);
void tlWarning(const char * Format, ...);
void tlFatal(const char * Format, ...);
bool tlFatalHandler(const char * Msg);
// _tlAssert is declared in tl_defs.h.

void * tlScratchPadInit();
void tlScratchPadReset();

inline void tlScratchPadLock()
{
	TL_ASSERT_MSG(!tlScratchpadLocked, "Scratchpad is already locked!");
	tlScratchpadLocked = true;
}

inline void tlScratchPadUnlock()
{
	TL_ASSERT_MSG(tlScratchpadLocked, "Scratchpad is already unlocked!");
	tlScratchpadLocked = false;
}

// Runs func(user_data, i) for i in [0, job_count) through the host RunJobs callback and waits.
// Serial fallback when no callback is installed.
void tlRunJobs(tlJobFunc func, void * user_data, int job_count);
