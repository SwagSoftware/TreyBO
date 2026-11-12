#pragma once

// Memory model: phys_memory_manager owns one linear arena. phys_slot_pool hands out fixed-size slots from it
// through a lock-free free list; slots are never returned to the arena. PMM_ALLOC / PMM_FREE look the pool
// up by (size, alignment).

#include "tl/base/include/tl_defs.h"
#include "tl/base/include/tl_thread.h"
#include "tl/physics/include/phys_base.h"
#include "tl/physics/include/phys_mutex.h"
#include "tl/physics/include/phys_hash_table.h"
#include "tl/physics/include/phys_array_base.inc"

#include <new>

class phys_slot_pool;
class phys_memory_manager;

extern void * g_phys_memory_buffer;
extern int g_phys_memory_buffer_size;
extern phys_memory_manager * g_phys_memory_manager;

// ---------------------------------------------------------------------------------------------
// tagged_void_pointer_t / phys_slot_pool
// ---------------------------------------------------------------------------------------------

// Head of a lock-free free list, swapped as one 64-bit word. m_ptr is a 32-bit slot reference (see
// tlTaggedRefEncode in tl_thread.h): the pointer itself on x86, an offset from g_phys_memory_manager on x64.
// That keeps an 8-byte cmpxchg on x64, where {ptr, tag} would be 16 bytes.
struct TL_ALIGN(8) tagged_void_pointer_t
{
	uint32_t m_ptr;
	uint32_t m_tag; // bumped on every push and pop (ABA guard)
};

// Alignment key of the PMM pool for T: max(alignof(T), 4). On x64, alignof(T) is 8 for pointer-holding T;
// phys_memory_manager::get_slot_pool rounds every alignment up to PHYS_ALIGNOF(phys_slot_pool::extra_info) (8)
// anyway.
template <class T> inline uint phys_slot_alignment()
{
	return TL_ALIGNOF(T) > 4 ? (uint)TL_ALIGNOF(T) : 4u;
}

class TL_ALIGN(8) phys_slot_pool
{
public:
	// Trailer at the end of every slot: owner pool + allocation owner.
	// m_allocation_owner is PHYS_SLOT_ALLOCATED while handed out and the owning pool while free.
	struct extra_info
	{
		phys_slot_pool * m_slot_pool_owner;
		void * m_allocation_owner;
	};

	volatile tagged_void_pointer_t m_first_free_slot;
	uint m_map_key; // low 16 bits: slot size incl. extra_info, high 16 bits: alignment
	phys_slot_pool * m_hash_next;
	volatile int m_total_slot_count;
	volatile int m_allocated_slot_count;

	static const uint encode_size_alignment(const uint size, const uint alignment);
	void init(const uint slot_size, const uint slot_alignment);

	void * allocate_slot();
	void free_slot(void * slot);
	void validate_slot(void * slot);
	void extra_info_init(void * slot);
	void extra_info_allocate(void * slot);
	void extra_info_free(void * slot);
};

// The allocation owner a handed-out slot carries.
#define PHYS_SLOT_ALLOCATED ((void *)(uintptr_t)0xFEDCBA98u)

// ---------------------------------------------------------------------------------------------
// phys_memory_manager
// ---------------------------------------------------------------------------------------------

class phys_memory_manager
{
public:
	char * m_buffer_start;
	char * m_buffer_end;
	char * volatile m_buffer_cur;
	int m_list_slot_pool_count;
	minspec_read_write_mutex m_slot_pool_map_mutex;
	minspec_hash_table<phys_slot_pool, 64> m_slot_pool_map;
	minspec_mutex m_slot_pool_allocate_mutex;
	phys_slot_pool m_list_preallocated_slot_pools[28];
	int m_list_preallocated_slot_pools_count;

	phys_memory_manager(void * const memory_buffer, const int memory_buffer_size);

	void * allocate(const uint size, const uint alignment);
	phys_slot_pool * allocate_slot_pool();
	phys_slot_pool * get_slot_pool(const uint slot_size, const uint slot_alignment);
};

void phys_memory_manager_init(void * const memory_buffer, const int memory_buffer_size);
void phys_memory_manager_term();
void ppu_pmm_get_linear_buffer(char *** linear_buffer_cur, char ** linear_buffer_end);

void * PMM_PERM_ALLOCATE(const uint size, const uint alignment); // permanent linear allocation
void * PMM_ALLOC(const uint size, const uint alignment);
void PMM_FREE(void * ptr, const uint size, const uint alignment);
void PMM_VALIDATE(void * ptr, const uint size, const uint alignment);
void * GET_PHYS_SLOT_POOL(const uint size, const uint alignment); // returns a phys_slot_pool
void * PSP_ALLOC(void * slot_pool);
void PSP_FREE(void * slot_pool, void * slot);

// ---------------------------------------------------------------------------------------------
// phys_memory_heap
// ---------------------------------------------------------------------------------------------

class phys_memory_heap
{
protected:
	char * m_buffer_start;
	char * m_buffer_end;
	char * m_buffer_cur;
	char * m_user_start;

public:
	phys_memory_heap() :
		m_buffer_start(0), m_buffer_end(0), m_buffer_cur(0), m_user_start(0) {}

	void set_buffer(void * const start, const int size, const int alignment)
	{
		PHYS_ASSERT(m_buffer_start == NULL);
		PHYS_ASSERT(m_buffer_end == NULL);
		PHYS_ASSERT(m_buffer_cur == NULL);
		PHYS_ASSERT(size > 0);
		PHYS_ASSERT(((size_t)start) % alignment == 0);
		m_buffer_start = (char *)start;
		m_buffer_cur = m_buffer_start;
		m_user_start = m_buffer_start;
		m_buffer_end = m_buffer_start + size;
	}

	void allocate_buffer(const int size, const int alignment, phys_memory_heap & allocater)
	{
		set_buffer(allocater.allocate(size, alignment), size, alignment);
	}

	void capture_user_start() { m_user_start = m_buffer_cur; }
	void reset() { m_buffer_cur = m_buffer_start; }
	void reset_to_user_start() { m_buffer_cur = m_user_start; }

	void * allocate_no_error(const int size, const int alignment)
	{
		PHYS_ASSERT(size > 0);
		char * const addr = (char *)PHYS_ALIGN(m_buffer_cur, alignment);
		char * const new_cur = addr + size;
		if (new_cur <= m_buffer_end)
		{
			m_buffer_cur = new_cur;
			return addr;
		}
		return NULL;
	}

	void * allocate(const int size, const int alignment, const char * error_msg = "phys_memory_heap overflow.")
	{
		void * const addr = allocate_no_error(size, alignment);
		PHYS_ASSERT_MSG(addr, error_msg);
		return addr;
	}

	// Unaligned bump.
	void * fast_allocate(const int size, const char * error_msg)
	{
		char * const addr = m_buffer_cur;
		m_buffer_cur = addr + size;
		PHYS_ASSERT_MSG(m_buffer_cur <= m_buffer_end, error_msg);
		return addr;
	}

	// Only used in asserts.
	const bool fast_is_within_buffer_limits(const void * p, const size_t size) const
	{
		return (const char *)p >= m_buffer_start && (const char *)p + size <= m_buffer_end;
	}

	void * fast_align_start(const int alignment, const char * error_msg)
	{
		m_buffer_cur = (char *)PHYS_ALIGN(m_buffer_cur, alignment);
		PHYS_ASSERT_MSG(m_buffer_cur < m_buffer_end, error_msg);
		return m_buffer_cur;
	}
};
typedef phys_memory_heap phys_scratchpad_allocater;

// ---------------------------------------------------------------------------------------------
// Intrusive singly linked lists
// ---------------------------------------------------------------------------------------------

// m_next_link is public: phys_link_list takes &p->m_next_link.
template <class T>
class phys_link_list_base
{
public:
	T * m_next_link;

	void set_next_link(T * const p) { m_next_link = p; }
	T * get_next_link() const { return m_next_link; }
};

// Tail-pointer list. T needs a T *m_next_link member.
template <class T>
class phys_link_list
{
public:
	T * m_first;
	T ** volatile m_last_next_ptr;
	volatile int m_alloc_count;

	phys_link_list() :
		m_first(NULL), m_last_next_ptr(&m_first), m_alloc_count(0) {}

	void remove_all()
	{
		m_first = NULL;
		m_last_next_ptr = &m_first;
		m_alloc_count = 0;
	}

	void add(T * p)
	{
		PHYS_ASSERT(m_last_next_ptr);
		p->m_next_link = NULL;
		m_alloc_count = m_alloc_count + 1;
		*m_last_next_ptr = p;
		m_last_next_ptr = &p->m_next_link;
	}

	// Lock-free append from several threads.
	void add_mt(T * p)
	{
		PHYS_ASSERT(m_last_next_ptr);
		p->m_next_link = NULL;
		tlAtomicExchangeAdd(&m_alloc_count, 1);
		T ** last_next_ptr;
		do
			last_next_ptr = tlAtomicLoad(&m_last_next_ptr);
		while (tlAtomicCompareExchange(&m_last_next_ptr, &p->m_next_link, last_next_ptr) != last_next_ptr);
		*last_next_ptr = p;
	}

	const int get_count() const { return m_alloc_count; }
	T * get_first() const { return m_first; }

	class iterator
	{
		T * m_ptr;

	public:
		iterator(T * ptr) :
			m_ptr(ptr) {}
		void operator++(int) { m_ptr = m_ptr->m_next_link; }
		iterator & operator++() // prefix form for range-for
		{
			m_ptr = m_ptr->m_next_link;
			return *this;
		}
		bool operator!=(const iterator & i) const { return i.m_ptr != m_ptr; }
		T & operator*() { return *m_ptr; }
	};
	iterator begin() { return iterator(m_first); }
	iterator end() { return iterator(NULL); }
};

// ---------------------------------------------------------------------------------------------
// phys_free_list: doubly linked list of PMM slots, each holding one T, plus a small debug pointer table.
// ---------------------------------------------------------------------------------------------

template <class T>
class phys_free_list
{
public:
	static const int PTR_LIST_SIZE = 256;

	struct T_internal_base
	{
		T_internal_base * m_prev_T_internal;
		T_internal_base * m_next_T_internal;
	};

	class T_internal : public T_internal_base
	{
	public:
		T m_data;
		int m_ptr_list_index; // slot in m_ptr_list, or -1
	};

	class iterator
	{
		T_internal_base * m_ptr;

	public:
		iterator(T_internal_base * ptr) :
			m_ptr(ptr) {}
		void operator++(int) { m_ptr = m_ptr->m_next_T_internal; }
		iterator & operator++() // prefix form for range-for
		{
			m_ptr = m_ptr->m_next_T_internal;
			return *this;
		}
		bool operator!=(const iterator & i) const { return i.m_ptr != m_ptr; }
		T & operator*() { return static_cast<T_internal *>(m_ptr)->m_data; }
		T * operator->() { return &static_cast<T_internal *>(m_ptr)->m_data; }
	};

	T_internal_base m_dummy_head; // circular sentinel
	int m_list_count;
	int m_list_count_high_water;
	T * m_ptr_list[PTR_LIST_SIZE]; // debug view of the first PTR_LIST_SIZE live entries
	int m_ptr_list_count;

	phys_free_list()
	{
		m_dummy_head.m_next_T_internal = &m_dummy_head;
		m_dummy_head.m_prev_T_internal = &m_dummy_head;
		m_list_count = 0;
		m_list_count_high_water = 0;
		m_ptr_list_count = 0;
	}

	~phys_free_list()
	{
		remove_all();
		PHYS_ASSERT(m_list_count == 0);
		PHYS_ASSERT(m_dummy_head.m_next_T_internal == &m_dummy_head);
		PHYS_ASSERT(m_dummy_head.m_prev_T_internal == &m_dummy_head);
		m_list_count_high_water = 0;
		m_ptr_list_count = 0;
	}

	static uint get_slot_size() { return (uint)sizeof(T_internal); }
	static uint get_slot_alignment() { return phys_slot_alignment<T_internal>(); }

	// New entries go to the front. T is default-initialised (no zeroing).
	T * add(const int no_error = false, const char * error_msg = "phys_free_list add failed.")
	{
		T_internal * const ptr = (T_internal *)PMM_ALLOC(get_slot_size(), get_slot_alignment());
		if (ptr)
		{
			new ((void *)&ptr->m_data) T; // not T(): that would zero-fill PODs
			ptr->m_prev_T_internal = &m_dummy_head;
			ptr->m_next_T_internal = m_dummy_head.m_next_T_internal;
			m_dummy_head.m_next_T_internal->m_prev_T_internal = ptr;
			m_dummy_head.m_next_T_internal = ptr;
			m_list_count++;
			debug_add(ptr);
			return &ptr->m_data;
		}
		PHYS_ASSERT_MSG(no_error, error_msg);
		return NULL;
	}

	// Runs ~T after unlinking (~rigid_body_constraint_contact unlinks from the search tree).
	void remove(T_internal * data)
	{
		PHYS_ASSERT(data);
		--m_list_count;
		debug_remove(data);
		T_internal_base * const next = data->m_next_T_internal;
		T_internal_base * const prev = data->m_prev_T_internal;
		prev->m_next_T_internal = next;
		next->m_prev_T_internal = prev;
		data->m_data.~T();
		PMM_FREE(data, get_slot_size(), get_slot_alignment());
	}

	void remove(T * data_)
	{
		if (data_)
		{
			T_internal * const data = get_T_internal(data_);
			PMM_VALIDATE(data, get_slot_size(), get_slot_alignment());
			remove(data);
		}
	}

	void remove_all()
	{
		while (m_dummy_head.m_next_T_internal != &m_dummy_head)
			remove(static_cast<T_internal *>(m_dummy_head.m_next_T_internal));
	}

	// Copies the entries, in list order, into ptr_array.
	void ptr_array_read(T ** ptr_array, const int ptr_array_size)
	{
		PHYS_ASSERT(ptr_array_size == m_list_count);
		for (T_internal_base * i = m_dummy_head.m_next_T_internal; i != &m_dummy_head; i = i->m_next_T_internal)
			*ptr_array++ = &static_cast<T_internal *>(i)->m_data;
	}

	// Relinks the list in ptr_array order (used after sorting the array).
	void ptr_array_write(T ** ptr_array, const int ptr_array_size)
	{
		PHYS_ASSERT(ptr_array_size == m_list_count);
		if (ptr_array_size > 0)
		{
			T_internal_base * cur = get_T_internal(ptr_array[0]);
			T ** const last = &ptr_array[ptr_array_size - 1];
			cur->m_prev_T_internal = &m_dummy_head;
			m_dummy_head.m_next_T_internal = cur;
			for (T ** p = ptr_array; p < last; ++p)
			{
				T_internal_base * const next = get_T_internal(p[1]);
				cur->m_next_T_internal = next;
				next->m_prev_T_internal = cur;
				cur = next;
			}
			cur->m_next_T_internal = &m_dummy_head;
			m_dummy_head.m_prev_T_internal = cur;
		}
	}

	const int get_count() const { return m_list_count; }
	const int get_count_high_water() const { return m_list_count_high_water; }

	iterator begin() { return iterator(m_dummy_head.m_next_T_internal); }
	iterator end() { return iterator(&m_dummy_head); }

	static T_internal * get_T_internal(T * data)
	{
		return (T_internal *)((char *)data - offsetof(T_internal, m_data));
	}

private:
	void debug_add(T_internal * T_i)
	{
		m_list_count_high_water = tl_max(m_list_count_high_water, m_list_count);
		if (m_ptr_list_count >= PTR_LIST_SIZE)
		{
			T_i->m_ptr_list_index = -1;
		}
		else
		{
			T_i->m_ptr_list_index = m_ptr_list_count;
			m_ptr_list[m_ptr_list_count++] = &T_i->m_data;
		}
	}

	void debug_remove(T_internal * T_i)
	{
		if (T_i->m_ptr_list_index != -1)
		{
			PHYS_ASSERT(T_i->m_ptr_list_index >= 0 && T_i->m_ptr_list_index < PTR_LIST_SIZE);
			PHYS_ASSERT(m_ptr_list[T_i->m_ptr_list_index] == &T_i->m_data);
			// move the last table entry into the freed slot
			--m_ptr_list_count;
			get_T_internal(m_ptr_list[m_ptr_list_count])->m_ptr_list_index = T_i->m_ptr_list_index;
			m_ptr_list[T_i->m_ptr_list_index] = m_ptr_list[m_ptr_list_count];
		}
	}
};

// ---------------------------------------------------------------------------------------------
// phys_simple_allocator: typed PMM slot allocator with a live count.
// ---------------------------------------------------------------------------------------------

template <class T>
class phys_simple_allocator
{
	int m_count;

public:
	phys_simple_allocator() :
		m_count(0) {}

	T * allocate()
	{
		void * const slot = PMM_ALLOC((uint)sizeof(T), phys_slot_alignment<T>());
		if (!slot)
			return NULL;
		++m_count;
		// default-initialised: the slot is not zeroed
		return new (slot) T;
	}

	void free(T * slot)
	{
		if (slot)
		{
			PMM_VALIDATE(slot, (uint)sizeof(T), phys_slot_alignment<T>());
			--m_count;
			slot->~T();
			PMM_FREE(slot, (uint)sizeof(T), phys_slot_alignment<T>());
		}
	}

	const int get_count() const { return m_count; }
};
