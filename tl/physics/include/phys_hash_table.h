#pragma once

// Retail: codsrc\tl\physics\include\phys_hash_table.h. Header-only template.
// Retail instantiation: minspec_hash_table<phys_slot_pool,64> (add 0xa0fef0; ctor and find inlined in
// phys_memory_manager::phys_memory_manager 0xa0f9b0 and phys_memory_manager::get_slot_pool 0xa0f700).
//
// Small chained table that re-picks its modulus on every add to minimise collisions.
// T needs: uint m_map_key; T *m_hash_next.

#include "tl/base/include/tl_defs.h"

template <class T, int TABLE_SIZE>
struct minspec_hash_table
{
    T *m_hash_table[TABLE_SIZE];
    uint m_mod;
    uint m_highest_collision;
    uint m_total_collisions;

    minspec_hash_table()
    {
        memset(m_hash_table, 0, sizeof(m_hash_table));
        m_mod = 1;
        m_highest_collision = 0;
        m_total_collisions = 0;
    }

    // TREYBO: name not recovered. Retail phys_memory_manager ctor re-clears the table with this exact sequence.
    void reset()
    {
        memset(m_hash_table, 0, sizeof(m_hash_table));
        m_highest_collision = 0;
        m_total_collisions = 0;
        m_mod = 1;
    }

    T *find(const uint key) const
    {
        for (T *i = m_hash_table[key % m_mod]; i; i = i->m_hash_next)
        {
            if (i->m_map_key == key)
                return i;
        }
        return NULL;
    }

    void add(const uint key, T *entry_to_add)
    {
        PHYS_ASSERT(find(key) == NULL);

        // Pull every entry out, then try every modulus from the entry count up to TABLE_SIZE - 1.
        T *entry_list[TABLE_SIZE];
        uint entry_count = 0;
        entry_list[entry_count++] = entry_to_add;
        for (int i = 0; i < TABLE_SIZE; i++)
        {
            for (T *j = m_hash_table[i]; j; j = j->m_hash_next)
            {
                PHYS_ASSERT(entry_count < (uint)TABLE_SIZE); // TREYBO: extra assert (retail overruns entry_list)
                entry_list[entry_count++] = j;
            }
            m_hash_table[i] = NULL;
        }

        m_mod = entry_count;
        m_highest_collision = 100000;
        m_total_collisions = 100000;
        for (uint mod_i = entry_count; mod_i < (uint)TABLE_SIZE; mod_i++)
        {
            uint collision_counts[TABLE_SIZE];
            if (mod_i)
                memset(collision_counts, 0, mod_i * sizeof(uint));

            uint highest_collision = 0;
            uint total_collisions = 0;
            for (uint i = 0; i < entry_count; i++)
            {
                const uint count = ++collision_counts[entry_list[i]->m_map_key % mod_i];
                if (count > highest_collision)
                    highest_collision = count;
                total_collisions += count;
            }

            if (total_collisions < m_total_collisions)
            {
                m_mod = mod_i;
                m_highest_collision = highest_collision;
                m_total_collisions = total_collisions;
            }
        }

        PHYS_ASSERT(m_mod > 0 && m_mod < TABLE_SIZE);

        for (uint i = 0; i < entry_count; i++)
        {
            T *const entry = entry_list[i];
            const uint hash = entry->m_map_key % m_mod;
            entry->m_hash_next = m_hash_table[hash];
            m_hash_table[hash] = entry;
        }
    }
};
