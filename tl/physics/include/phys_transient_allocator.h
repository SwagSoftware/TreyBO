#pragma once

// Frame-scoped bump allocator over a chain of 16 KB PMM slots. Call reset() before destruction.

#include "tl/base/include/tl_defs.h"
#include "tl/base/include/tl_thread.h"
#include "tl/physics/include/phys_mutex.h"
#include "tl/physics/include/phys_mem.h"

void transient_allocator_update_largest_size(int size); // empty

class phys_transient_allocator
{
public:
	// block_size is 0x4000 on x86 and scaled by sizeof(void *) / 4 (0x8000) on x64. A single allocation must fit
	// one block, and the biggest ones are pointer arrays (broad-phase bpb list and sap endpoint lists, island
	// list, contact sort list), which double on x64; with 16 KB blocks x64 would cap at 2048 bpbs / 1024 sap
	// nodes instead of 4096 / 2048. The x64 slot (0x8000 + 16-byte trailer) still fits the 16-bit pool size key.
	enum
	{
		block_size = 0x4000 * (int)(sizeof(void *) / 4),
		block_alignment = 4,
	};

	struct block_header
	{
		uint m_block_size;
		uint m_block_alignment;
		block_header * m_next_block;
	};

	struct allocator_state
	{
		block_header * m_first_block;
		char * m_cur;
		char * m_end;
		uint m_total_memory_allocated;
	};

	block_header * m_first_block;
	char * volatile m_cur;
	char * m_end;
	uint m_total_memory_allocated;
	minspec_read_write_mutex m_mutex;
	void * m_slot_pool; // phys_slot_pool for block_size slots, fetched on first resize

	phys_transient_allocator() :
		m_first_block(NULL), m_cur(NULL), m_end(NULL), m_total_memory_allocated(0), m_slot_pool(NULL) {}

	~phys_transient_allocator()
	{
		PHYS_ASSERT(m_first_block == NULL);
	}

	// Single-threaded.
	void * allocate(const int size, const int alignment, const int no_error, const char * error_msg)
	{
		transient_allocator_update_largest_size(size);
		void * ptr = allocate_in_block(size, alignment);
		if (!ptr)
		{
			resize();
			ptr = allocate_in_block(size, alignment);
			PHYS_ASSERT_MSG(ptr, "transient allocation too large, increase block_size.");
		}
		PHYS_ASSERT_MSG(ptr || no_error, error_msg);
		return ptr;
	}

	// Many threads may allocate at once; only block changes take the write lock.
	void * mt_allocate(const int size, const int alignment, const int no_error, const char * error_msg)
	{
		transient_allocator_update_largest_size(size);
		m_mutex.ReadLock();
		void * ptr = mt_allocate_internal(size, alignment);
		m_mutex.ReadUnlock();
		if (!ptr)
		{
			m_mutex.WriteLock();
			ptr = allocate_in_block(size, alignment);
			if (!ptr)
			{
				resize();
				ptr = allocate_in_block(size, alignment);
				PHYS_ASSERT_MSG(ptr, "transient allocation too large, increase block_size.");
			}
			m_mutex.WriteUnlock();
		}
		PHYS_ASSERT_MSG(ptr || no_error, error_msg);
		return ptr;
	}

	void reset();
	void reset_to_state(const allocator_state & as);

	bool is_empty() const { return m_cur == NULL; }

	void get_state(allocator_state & as) const
	{
		as.m_first_block = m_first_block;
		as.m_cur = m_cur;
		as.m_end = m_end;
		as.m_total_memory_allocated = m_total_memory_allocated;
	}

private:
	void * allocate_in_block(const int size, const int alignment)
	{
		char * const ptr = (char *)(((uintptr_t)m_cur + (alignment - 1)) & ~(uintptr_t)(alignment - 1));
		if (ptr + size <= m_end)
		{
			m_cur = ptr + size;
			return ptr;
		}
		return NULL;
	}

	// Lock-free bump inside the current block.
	void * mt_allocate_internal(const int size, const int alignment)
	{
		char * cur;
		char * ptr;
		do
		{
			cur = tlAtomicLoad(&m_cur);
			ptr = (char *)(((uintptr_t)cur + (alignment - 1)) & ~(uintptr_t)(alignment - 1));
			if (ptr + size > m_end)
				return NULL;
		} while (tlAtomicCompareExchange(&m_cur, ptr + size, cur) != cur);
		return ptr;
	}

	void resize();
};
