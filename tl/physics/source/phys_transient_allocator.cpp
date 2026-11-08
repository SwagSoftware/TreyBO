// Retail: codsrc\tl\physics\source\phys_transient_allocator.cpp, TU 0xa0f410-0xa0f4f0.

#include "tl/physics/include/phys_transient_allocator.h"

// 0xa0f410: return every block to the slot pool.
void phys_transient_allocator::reset()
{
    void *const slot_pool = m_slot_pool;
    for (block_header *block = m_first_block; block;)
    {
        block_header *const next_block = block->m_next_block;
        PSP_FREE(slot_pool, block);
        block = next_block;
    }
    m_first_block = NULL;
    m_cur = NULL;
    m_end = NULL;
    m_total_memory_allocated = 0;
}

// 0xa0f450: free the blocks added since get_state() and rewind to it.
void phys_transient_allocator::reset_to_state(const allocator_state &as)
{
    void *const slot_pool = m_slot_pool;
    block_header *block = m_first_block;
    if (block != as.m_first_block)
    {
        block_header *next_block;
        do
        {
            next_block = block->m_next_block;
            PSP_FREE(slot_pool, block);
            block = next_block;
        } while (next_block != as.m_first_block);
    }
    m_first_block = as.m_first_block;
    m_cur = as.m_cur;
    m_end = as.m_end;
    m_total_memory_allocated = as.m_total_memory_allocated;
}

// 0xa0f4a0: push a fresh block. On failure m_cur / m_end stay as they were.
void phys_transient_allocator::resize()
{
    if (!m_slot_pool)
        m_slot_pool = GET_PHYS_SLOT_POOL(block_size, block_alignment);
    char *const slot = (char *)PSP_ALLOC(m_slot_pool);
    if (slot)
    {
        block_header *const block = (block_header *)slot;
        block->m_block_size = block_size;
        block->m_block_alignment = block_alignment;
        block->m_next_block = m_first_block;
        m_first_block = block;
        m_total_memory_allocated += block_size;
        m_cur = slot + sizeof(block_header);
        m_end = slot + block_size;
    }
}
