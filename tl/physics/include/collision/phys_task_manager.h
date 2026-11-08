#pragma once

// TreyBO-new header. Retail has no header text for these (no asserts); declarations for
// source/phys_task_manager.cpp (TU 0xa23820-0xa23a15).
//
// Retail queued min(input_count, 6) copies of one jqBatch {module, input} on the IW task manager and
// phys_task_manager_flush waited for them. TreyBO runs the same min(input_count, 6) jobs through tlRunJobs
// (tl_system.h), which returns only when every job is done; flush then only clears the worker count.
// Every job gets the same input and is expected to pull work items itself (atomic counters / lists), as
// every retail batch function does.

#include "tl/base/include/tl_system.h"

enum
{
    PHYS_TASK_MANAGER_MAX_WORKERS = 6, // TREYBO: name not recovered (retail constant in phys_task_manager_process)
};

void phys_task_manager_init();      // 0xa23820
void phys_task_manager_shutdown();  // 0xa238a0
int phys_task_manager_needs_flush(); // 0xa23900
void phys_task_manager_flush();     // 0xa23940
// TREYBO: retail takes a jqModule *; TreyBO takes the job function (tlJobFunc: func(input, job_index)).
void phys_task_manager_process(tlJobFunc func, void *input, int input_count); // 0xa23980
