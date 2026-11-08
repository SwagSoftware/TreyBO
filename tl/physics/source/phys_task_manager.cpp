// Retail: codsrc\tl\physics\source\phys_task_manager.cpp ("source/phys_task_manager.cpp" asserts),
// TU 0xa23820-0xa23a15.
// TREYBO: re-based on tlRunJobs (see collision/phys_task_manager.h).

#include "tl/physics/include/collision/phys_task_manager.h"
#include "tl/physics/include/phys_assert.h"

int g_phys_task_manager_inited;
int g_phys_task_manager_current_worker_count;

// TREYBO: retail keeps one jqBatch {Module, Input, Output} (g_phys_task_manager_batch); TreyBO keeps its two used fields.
static tlJobFunc g_phys_task_manager_batch_func;
static void *g_phys_task_manager_batch_input;

// 0xa23820
void phys_task_manager_init()
{
    PHYS_ASSERT(g_phys_task_manager_inited == 0);
    PHYS_ASSERT(g_phys_task_manager_current_worker_count == 0);
    g_phys_task_manager_inited = 1;
    g_phys_task_manager_batch_func = NULL;
    g_phys_task_manager_batch_input = NULL;
}

// 0xa238a0
void phys_task_manager_shutdown()
{
    PHYS_ASSERT(g_phys_task_manager_inited == 1);
    PHYS_ASSERT(g_phys_task_manager_current_worker_count == 0);
    g_phys_task_manager_inited = 0;
}

// 0xa23900
int phys_task_manager_needs_flush()
{
    PHYS_ASSERT(g_phys_task_manager_inited == 1);
    return g_phys_task_manager_current_worker_count > 0;
}

// 0xa23940
void phys_task_manager_flush()
{
    PHYS_ASSERT(g_phys_task_manager_inited == 1);
    if (g_phys_task_manager_current_worker_count > 0)
    {
        g_phys_task_manager_current_worker_count = 0;
        // TREYBO: retail IW_task_manager_flush(); tlRunJobs already waited in phys_task_manager_process.
    }
}

// 0xa23980. Runs min(input_count, 6) copies of func on input. Every call must be followed by phys_task_manager_flush.
void phys_task_manager_process(tlJobFunc func, void *input, int input_count)
{
    PHYS_ASSERT(g_phys_task_manager_inited == 1);
    PHYS_ASSERT(g_phys_task_manager_current_worker_count == 0);
    if (input_count)
    {
        const int worker_count = input_count > PHYS_TASK_MANAGER_MAX_WORKERS ? PHYS_TASK_MANAGER_MAX_WORKERS : input_count;
        g_phys_task_manager_current_worker_count = worker_count;
        g_phys_task_manager_batch_func = func;
        g_phys_task_manager_batch_input = input;
        // TREYBO: retail IW_task_manager_add_batch x worker_count
        if (worker_count > 0)
            tlRunJobs(g_phys_task_manager_batch_func, g_phys_task_manager_batch_input, worker_count);
    }
}
