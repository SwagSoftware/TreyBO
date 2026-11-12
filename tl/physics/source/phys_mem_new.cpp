#include "tl/physics/include/phys_mem.h"
#include "tl/physics/include/phys_transient_allocator.h"

void * g_phys_memory_buffer;
int g_phys_memory_buffer_size;
phys_memory_manager * g_phys_memory_manager;

// Slot references in the free lists are relative to the manager (see tagged_void_pointer_t).
static inline uint32_t phys_slot_ref_encode(const void * slot)
{
	return tlTaggedRefEncode(g_phys_memory_manager, slot);
}

static inline char * phys_slot_ref_decode(const uint32_t ref)
{
	return (char *)tlTaggedRefDecode(g_phys_memory_manager, ref);
}

static inline phys_slot_pool::extra_info * phys_slot_extra_info(void * slot, const uint map_key)
{
	return (phys_slot_pool::extra_info *)((char *)slot + (ushort)map_key - sizeof(phys_slot_pool::extra_info));
}

const uint phys_slot_pool::encode_size_alignment(const uint size, const uint alignment)
{
	PHYS_ASSERT(size <= 0xFFFF);
	PHYS_ASSERT(alignment <= 0xFFFF);
	return size | (alignment << 16);
}

void phys_memory_manager_term()
{
	g_phys_memory_buffer = NULL;
	g_phys_memory_buffer_size = 0;
	g_phys_memory_manager = NULL;
}

void phys_slot_pool::init(const uint slot_size, const uint slot_alignment)
{
	m_first_free_slot.m_ptr = 0;
	m_first_free_slot.m_tag = 0;
	m_map_key = encode_size_alignment(slot_size, slot_alignment);
	m_total_slot_count = 0;
	m_allocated_slot_count = 0;
}

void ppu_pmm_get_linear_buffer(char *** linear_buffer_cur, char ** linear_buffer_end)
{
	*linear_buffer_cur = (char **)&g_phys_memory_manager->m_buffer_cur;
	*linear_buffer_end = g_phys_memory_manager->m_buffer_end;
}

void PSP_FREE(void * slot_pool, void * slot)
{
	((phys_slot_pool *)slot_pool)->free_slot(slot);
}

void transient_allocator_update_largest_size(int size)
{
	(void)size;
}

// Lock-free bump allocation from the arena. Never freed.
void * phys_memory_manager::allocate(const uint size, const uint alignment)
{
	for (;;)
	{
		// One read: the aligned pointer and the CAS comparand must come from the same load.
		char * const cur = tlAtomicLoad(&m_buffer_cur);
		char * const ptr = (char *)(((uintptr_t)cur + (alignment - 1)) & ~(uintptr_t)(alignment - 1));
		if (ptr + size > m_buffer_end)
			return NULL;
		if (tlAtomicCompareExchange(&m_buffer_cur, ptr + size, cur) == cur)
			return ptr;
	}
}

// The first 28 pools live inside the manager, the rest come from the arena.
phys_slot_pool * phys_memory_manager::allocate_slot_pool()
{
	m_slot_pool_allocate_mutex.Lock();
	phys_slot_pool * slot_pool;
	if (m_list_preallocated_slot_pools_count >= (int)TL_ARRAY_COUNT(m_list_preallocated_slot_pools))
	{
		slot_pool = (phys_slot_pool *)allocate((uint)sizeof(phys_slot_pool), (uint)PHYS_ALIGNOF(phys_slot_pool));
		++m_list_slot_pool_count;
	}
	else
	{
		++m_list_slot_pool_count;
		slot_pool = &m_list_preallocated_slot_pools[m_list_preallocated_slot_pools_count++];
	}
	m_slot_pool_allocate_mutex.Unlock();
	return slot_pool;
}

// Find or create the pool for (slot_size, slot_alignment). Double-checked under the map mutex.
phys_slot_pool * phys_memory_manager::get_slot_pool(const uint slot_size, const uint slot_alignment)
{
	PHYS_ASSERT(slot_alignment >= 4);
	PHYS_ASSERT(slot_size % slot_alignment == 0);
	// PHYS_ALIGNOF(extra_info) is 4 on x86; on x64 (8) the alignment is rounded up below instead.
	PHYS_ASSERT_STR(slot_alignment % 4 == 0, "slot_alignment % PHYS_ALIGNOF(phys_slot_pool::extra_info) == 0");

#if UINTPTR_MAX == 0xFFFFFFFFu
	const uint pool_alignment = slot_alignment;
	const uint pool_slot_size = slot_size + (uint)sizeof(phys_slot_pool::extra_info);
#else
	// On x64 extra_info is two 8-byte pointers. Round the alignment up to its alignment and the
	// payload up to that alignment so the trailer stays aligned. PMM_FREE / PMM_VALIDATE go through
	// here with the same arguments, so every caller maps to the same pool.
	const uint pool_alignment = tl_max(slot_alignment, (uint)PHYS_ALIGNOF(phys_slot_pool::extra_info));
	const uint pool_slot_size = (uint)tl_align(slot_size, (int)pool_alignment) + (uint)sizeof(phys_slot_pool::extra_info);
#endif

	const uint map_key = phys_slot_pool::encode_size_alignment(pool_slot_size, pool_alignment);

	m_slot_pool_map_mutex.ReadLock();
	phys_slot_pool * slot_pool = m_slot_pool_map.find(map_key);
	m_slot_pool_map_mutex.ReadUnlock();
	if (!slot_pool)
	{
		m_slot_pool_map_mutex.WriteLock();
		slot_pool = m_slot_pool_map.find(map_key);
		if (!slot_pool)
		{
			slot_pool = allocate_slot_pool();
			slot_pool->init(pool_slot_size, pool_alignment);
			m_slot_pool_map.add(map_key, slot_pool);
		}
		m_slot_pool_map_mutex.WriteUnlock();
	}
	return slot_pool;
}

void * PMM_PERM_ALLOCATE(const uint size, const uint alignment)
{
	void * const ptr = g_phys_memory_manager->allocate(size, alignment);
	PHYS_ASSERT_MSG(ptr, "physics memory manager error: out of memory.");
	return ptr;
}

void * PMM_ALLOC(const uint size, const uint alignment)
{
	PHYS_ASSERT(size > 0);
	phys_slot_pool * const slot_pool = g_phys_memory_manager->get_slot_pool(size, alignment);
	return slot_pool->allocate_slot();
}

void PMM_FREE(void * ptr, const uint size, const uint alignment)
{
	phys_slot_pool * const slot_pool = g_phys_memory_manager->get_slot_pool(size, alignment);
	slot_pool->free_slot(ptr);
}

void * PSP_ALLOC(void * slot_pool)
{
	return ((phys_slot_pool *)slot_pool)->allocate_slot();
}

void * GET_PHYS_SLOT_POOL(const uint size, const uint alignment)
{
	PHYS_ASSERT(size > 0);
	return g_phys_memory_manager->get_slot_pool(size, alignment);
}

void PMM_VALIDATE(void * ptr, const uint size, const uint alignment)
{
	phys_slot_pool * const slot_pool = g_phys_memory_manager->get_slot_pool(size, alignment);
	slot_pool->validate_slot(ptr);
}

phys_memory_manager::phys_memory_manager(void * const memory_buffer, const int memory_buffer_size)
{
	m_buffer_start = (char *)memory_buffer;
	m_buffer_cur = (char *)memory_buffer;
	m_buffer_end = (char *)memory_buffer + memory_buffer_size;
	m_list_slot_pool_count = 0;
	m_slot_pool_map.reset();
	m_list_preallocated_slot_pools_count = 0;
}

// The manager object sits at the 8-aligned start of the buffer, the arena is the rest.
void phys_memory_manager_init(void * const memory_buffer, const int memory_buffer_size)
{
	PHYS_ASSERT(g_phys_memory_buffer == NULL);
	PHYS_ASSERT(g_phys_memory_buffer_size == 0);
	PHYS_ASSERT(g_phys_memory_manager == NULL);

	char * const manager = (char *)tl_align((uintptr_t)memory_buffer, (int)PHYS_ALIGNOF(phys_memory_manager));
	g_phys_memory_buffer = memory_buffer;
	g_phys_memory_buffer_size = memory_buffer_size;
	g_phys_memory_manager = (phys_memory_manager *)manager;
	if (manager)
	{
		char * const arena = manager + sizeof(phys_memory_manager);
		new (manager) phys_memory_manager(arena, (int)((char *)memory_buffer + memory_buffer_size - arena));
	}
}

// First use of a slot fresh from the arena (counts it as allocated).
void phys_slot_pool::extra_info_init(void * slot)
{
	if (slot)
	{
		extra_info * const ei = phys_slot_extra_info(slot, m_map_key);
		ei->m_slot_pool_owner = this;
		ei->m_allocation_owner = PHYS_SLOT_ALLOCATED;
		tlAtomicExchangeAdd(&m_total_slot_count, 1);
		tlAtomicExchangeAdd(&m_allocated_slot_count, 1);
		PHYS_ASSERT_MSG(GetStuff32(&m_allocated_slot_count) <= GetStuff32(&m_total_slot_count), "internal error.");
		tlMemoryBarrier();
	}
}

// Slot popped from the free list.
void phys_slot_pool::extra_info_allocate(void * slot)
{
	tlMemoryBarrier();
	phys_slot_pool * const this_ = this;
	PHYS_ASSERT(slot);
	extra_info * const ei = phys_slot_extra_info(slot, m_map_key);
	PHYS_ASSERT_MSG(GetStuff32(&ei->m_slot_pool_owner) == this_, "internal error.");
	const bool retv = tlAtomicCompareExchange((void * volatile *)&ei->m_allocation_owner, PHYS_SLOT_ALLOCATED, (void *)this_) == (void *)this_;
	PHYS_ASSERT_MSG(retv, "internal error.");
	(void)retv;
	tlAtomicExchangeAdd(&m_allocated_slot_count, 1);
	PHYS_ASSERT_MSG(GetStuff32(&m_allocated_slot_count) <= GetStuff32(&m_total_slot_count), "internal error.");
	tlMemoryBarrier();
}

// Slot about to go back on the free list. Fills the payload with 0xFF.
void phys_slot_pool::extra_info_free(void * slot)
{
	tlMemoryBarrier();
	phys_slot_pool * const this_ = this;
	const uint map_key = m_map_key;
	PHYS_ASSERT(slot);
	memset(slot, 0xFF, (ushort)map_key - sizeof(extra_info));
	extra_info * const ei = phys_slot_extra_info(slot, map_key);
	PHYS_ASSERT_MSG(GetStuff32(&ei->m_slot_pool_owner) == this_, "attempting to free a slot that was not allocated from this pool.");
	const bool retv = tlAtomicCompareExchange((void * volatile *)&ei->m_allocation_owner, (void *)this_, PHYS_SLOT_ALLOCATED) == PHYS_SLOT_ALLOCATED;
	PHYS_ASSERT_MSG(retv, "incorrect owner attempting to free a slot, possibly trying to free a slot more than once.");
	(void)retv;
	tlAtomicExchangeAdd(&m_allocated_slot_count, -1);
	PHYS_ASSERT_MSG(GetStuff32(&m_allocated_slot_count) <= GetStuff32(&m_total_slot_count), "internal error.");
	PHYS_ASSERT_MSG(GetStuff32(&m_allocated_slot_count) >= 0, "internal error.");
	tlMemoryBarrier();
}

void phys_slot_pool::validate_slot(void * slot)
{
	tlMemoryBarrier();
	phys_slot_pool * const this_ = this;
	void * const allocation_owner = PHYS_SLOT_ALLOCATED;
	extra_info * const ei = phys_slot_extra_info(slot, m_map_key);
	PHYS_ASSERT_MSG(GetStuff32(&ei->m_slot_pool_owner) == this_, "slot does not belong to this slot_pool.");
	PHYS_ASSERT_MSG(GetStuff32(&ei->m_allocation_owner) == allocation_owner, "slot does not belong to this allocation_owner.");
	(void)this_;
	(void)allocation_owner;
}

// Lock-free push. The free slot's first 4 bytes hold the next reference.
void phys_slot_pool::free_slot(void * slot)
{
	PHYS_ASSERT_MSG(slot, "no support support for freeing NULL slots.");
	extra_info_free(slot);

	tagged_void_pointer_t new_first_slot;
	new_first_slot.m_ptr = phys_slot_ref_encode(slot);
	for (;;)
	{
		const tagged_void_pointer_t first_slot = tlAtomicLoad(&m_first_free_slot);
		*(volatile uint32_t *)slot = first_slot.m_ptr;
		tlMemoryBarrier();
		new_first_slot.m_tag = first_slot.m_tag + 1;
		// 64-bit {ref, tag} CAS on every target
		if (tlAtomicCompareExchangeBool(&m_first_free_slot, new_first_slot, first_slot))
			break;
	}
}

// Lock-free pop, else a fresh slot from the arena (NULL when the arena is full).
void * phys_slot_pool::allocate_slot()
{
	for (;;)
	{
		const tagged_void_pointer_t first_slot = tlAtomicLoad(&m_first_free_slot);
		if (!first_slot.m_ptr)
			break;
		char * const slot = phys_slot_ref_decode(first_slot.m_ptr);
		tlMemoryBarrier();
		tagged_void_pointer_t new_first_slot;
		new_first_slot.m_ptr = *(const volatile uint32_t *)slot; // may be stale; the tag makes the CAS fail then
		new_first_slot.m_tag = first_slot.m_tag + 1;
		if (tlAtomicCompareExchangeBool(&m_first_free_slot, new_first_slot, first_slot))
		{
			extra_info_allocate(slot);
			return slot;
		}
	}

	void * const slot = g_phys_memory_manager->allocate((ushort)m_map_key, m_map_key >> 16);
	extra_info_init(slot);
	return slot;
}
