#pragma once

// phys_task_manager_process runs min(input_count, 6) jobs through tlRunJobs (tl_system.h), which returns
// only when every job is done; flush then only clears the worker count. Every job gets the same input and
// is expected to pull work items itself (atomic counters / lists).

#include "tl/base/include/tl_system.h"

enum
{
	PHYS_TASK_MANAGER_MAX_WORKERS = 6,
};

void phys_task_manager_init();
void phys_task_manager_shutdown();
int phys_task_manager_needs_flush();
void phys_task_manager_flush();
// func is a tlJobFunc: func(input, job_index).
void phys_task_manager_process(tlJobFunc func, void * input, int input_count);
