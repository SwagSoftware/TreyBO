// Physics job dispatch on top of tlRunJobs (see collision/phys_task_manager.h).

#include "tl/physics/include/collision/phys_task_manager.h"
#include "tl/physics/include/phys_assert.h"

int g_phys_task_manager_inited;
int g_phys_task_manager_current_worker_count;

static tlJobFunc g_phys_task_manager_batch_func;
static void * g_phys_task_manager_batch_input;

void phys_task_manager_init()
{
	PHYS_ASSERT(g_phys_task_manager_inited == 0);
	PHYS_ASSERT(g_phys_task_manager_current_worker_count == 0);
	g_phys_task_manager_inited = 1;
	g_phys_task_manager_batch_func = NULL;
	g_phys_task_manager_batch_input = NULL;
}

void phys_task_manager_shutdown()
{
	PHYS_ASSERT(g_phys_task_manager_inited == 1);
	PHYS_ASSERT(g_phys_task_manager_current_worker_count == 0);
	g_phys_task_manager_inited = 0;
}

int phys_task_manager_needs_flush()
{
	PHYS_ASSERT(g_phys_task_manager_inited == 1);
	return g_phys_task_manager_current_worker_count > 0;
}

void phys_task_manager_flush()
{
	PHYS_ASSERT(g_phys_task_manager_inited == 1);
	if (g_phys_task_manager_current_worker_count > 0)
	{
		g_phys_task_manager_current_worker_count = 0;
		// Nothing to wait for: tlRunJobs already returned with every job done.
	}
}

// Runs min(input_count, 6) copies of func on input. Every call must be followed by phys_task_manager_flush.
void phys_task_manager_process(tlJobFunc func, void * input, int input_count)
{
	PHYS_ASSERT(g_phys_task_manager_inited == 1);
	PHYS_ASSERT(g_phys_task_manager_current_worker_count == 0);
	if (input_count)
	{
		const int worker_count = input_count > PHYS_TASK_MANAGER_MAX_WORKERS ? PHYS_TASK_MANAGER_MAX_WORKERS : input_count;
		g_phys_task_manager_current_worker_count = worker_count;
		g_phys_task_manager_batch_func = func;
		g_phys_task_manager_batch_input = input;
		if (worker_count > 0)
			tlRunJobs(g_phys_task_manager_batch_func, g_phys_task_manager_batch_input, worker_count);
	}
}
