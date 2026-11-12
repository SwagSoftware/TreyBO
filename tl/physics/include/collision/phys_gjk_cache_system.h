#pragma once

// Per geom-pair GJK warm-start cache.

#include "tl/base/include/tl_thread.h"
#include "tl/physics/include/phys_math.h"
#include "tl/physics/include/phys_mem.h"
#include "tl/physics/include/phys_avl_tree.h"
#include "tl/physics/include/phys_assert.h"

typedef unsigned int phys_gjk_geom_id;

// The assert sites in get_gjk_cache_info(_mutex). Defined in phys_gjk.cpp, values (0, 1, true).
extern phys_assert_info pai_gjk_cache_system_max_num_gjk_ci;
extern phys_assert_info pai_gjk_cache_system_create_gjk_ci;

class phys_gjk_geom_id_pair_key
{
	phys_gjk_geom_id m_id1;
	phys_gjk_geom_id m_id2;

public:
	const phys_gjk_geom_id & get_id1() const { return m_id1; }
	const phys_gjk_geom_id & get_id2() const { return m_id2; }

	phys_gjk_geom_id_pair_key() {}
	phys_gjk_geom_id_pair_key(const phys_gjk_geom_id & id1, const phys_gjk_geom_id & id2)
	{
		PHYS_ASSERT(id1 < id2);
		m_id1 = id1;
		m_id2 = id2;
	}

	const int operator<(const phys_gjk_geom_id_pair_key & k) const { return (m_id1 == k.m_id1) ? (m_id2 < k.m_id2) : (m_id1 < k.m_id1); }
	const int operator<=(const phys_gjk_geom_id_pair_key & k) const { return (m_id1 == k.m_id1) ? (m_id2 <= k.m_id2) : (m_id1 <= k.m_id1); }
	const int operator>(const phys_gjk_geom_id_pair_key & k) const { return (m_id1 == k.m_id1) ? (m_id2 > k.m_id2) : (m_id1 > k.m_id1); }
	const int operator>=(const phys_gjk_geom_id_pair_key & k) const { return (m_id1 == k.m_id1) ? (m_id2 >= k.m_id2) : (m_id1 >= k.m_id1); }
	const int operator==(const phys_gjk_geom_id_pair_key & k) const { return (m_id1 == k.m_id1) && (m_id2 == k.m_id2); }
	const int operator!=(const phys_gjk_geom_id_pair_key & k) const { return (m_id1 != k.m_id1) || (m_id2 != k.m_id2); }
};

// The cached simplex of one geom: one geom-defined "index" vector per simplex vertex (at most 3).
// Written by phys_gjk_geom::set_simplex and read back by phys_gjk_geom::get_simplex.
struct cached_simplex_info
{
	phys_vec3 m_indices[3];
};

// Per geom-pair GJK warm start: last support direction (world space) and the last simplex.
// Members are public so GJK and broad-phase code can name them; prefer the accessors.
class phys_gjk_cache_info
{
public:
	phys_vec3 m_support_dir;
	cached_simplex_info m_support_a;
	cached_simplex_info m_support_b;
	int m_support_count;
	phys_gjk_geom_id_pair_key m_key;
	uint m_flags;

	enum phys_gjk_cache_info_e
	{
		FLAG_WAS_TOUCHED = (1 << 0),
		FLAG_IS_SWAPPED = (1 << 1),
		FLAG_IS_SUPPORT_DIR_VALID = (1 << 2),
		FLAG_IS_SIMPLEX_VALID = (1 << 3),
	};

	void constructor(const phys_gjk_geom_id_pair_key & key, const bool swapped)
	{
		m_key = key;
		m_flags = 0;
		set_flag(FLAG_IS_SWAPPED, swapped);
	}

	const phys_gjk_geom_id_pair_key & get_key() const { return m_key; }

	const uint get_flag(const uint f) const { return m_flags & f; }
	void set_flag(const uint f, const bool b) { b ? m_flags |= f : m_flags &= ~f; }

	// Swaps the whole cached simplex.
	void update_swapped(const bool swapped)
	{
		if (swapped != (get_flag(FLAG_IS_SWAPPED) ? true : false))
		{
			set_flag(FLAG_IS_SWAPPED, swapped);

			if (get_flag(FLAG_IS_SUPPORT_DIR_VALID))
				m_support_dir = -m_support_dir;

			if (get_flag(FLAG_IS_SIMPLEX_VALID))
			{
				const cached_simplex_info temp = m_support_a;
				m_support_a = m_support_b;
				m_support_b = temp;
			}
		}
	}

	const phys_vec3 & get_support_dir() const
	{
		PHYS_ASSERT(get_flag(FLAG_IS_SUPPORT_DIR_VALID));
		return m_support_dir;
	}

	void set_flag_was_touched() { set_flag(FLAG_WAS_TOUCHED, true); }
	const int get_flag_was_touched() const { return get_flag(FLAG_WAS_TOUCHED); }

	void set_support_dir(const phys_vec3 & support_dir)
	{
		set_flag(FLAG_IS_SUPPORT_DIR_VALID, true);
		m_support_dir = support_dir;
	}

	void invalidate_support_dir() { set_flag(FLAG_IS_SUPPORT_DIR_VALID, false); }
	void invalidate_simplex() { set_flag(FLAG_IS_SIMPLEX_VALID, false); }

	const int is_support_dir_valid() const { return get_flag(FLAG_IS_SUPPORT_DIR_VALID); }
	const int is_simplex_valid() const { return get_flag(FLAG_IS_SIMPLEX_VALID); }
};

// PMM slot per pair, an in-place AVL tree keyed by the id pair, and a singly linked list for
// the per-frame sweep. Not thread safe except through get_gjk_cache_info_mutex.
class phys_heap_gjk_cache_system_avl_tree
{
public:
	struct phys_gjk_cache_info_internal : public phys_gjk_cache_info
	{
		struct avl_tree_accessor
		{
			static phys_gjk_geom_id_pair_key & get_avl_key(phys_gjk_cache_info_internal * n) { return n->m_key; }
			static const phys_gjk_geom_id_pair_key & get_avl_key(const phys_gjk_cache_info_internal * n) { return n->m_key; }
			static phys_inplace_avl_tree_node<phys_gjk_cache_info_internal> * get_avl_node(phys_gjk_cache_info_internal * n) { return &n->m_avl_tree_node; }
			static const phys_inplace_avl_tree_node<phys_gjk_cache_info_internal> * get_avl_node(const phys_gjk_cache_info_internal * n) { return &n->m_avl_tree_node; }
		};

		phys_inplace_avl_tree_node<phys_gjk_cache_info_internal> m_avl_tree_node;
		phys_gjk_cache_info_internal * m_next_gjk_ci;
	};

	phys_simple_allocator<phys_gjk_cache_info_internal> m_list_phys_gjk_cache_info_internal;
	phys_inplace_avl_tree<phys_gjk_geom_id_pair_key, phys_gjk_cache_info_internal, phys_gjk_cache_info_internal::avl_tree_accessor> m_search_tree;
	int m_max_num_gjk_ci;
	phys_gjk_cache_info_internal * m_list_head;

	// The default ctor sets only the allocator count and the tree root; the owner calls init() right after
	// construction (broad_phase_memory::allocate_buffer passes m_max_num_gjk_ci + m_max_num_sap_active_pairs).
	void init(const int max_num_gjk_ci)
	{
		m_list_head = NULL;
		m_max_num_gjk_ci = max_num_gjk_ci;
	}

	~phys_heap_gjk_cache_system_avl_tree() { shutdown(); }

	void shutdown()
	{
		while (m_list_head)
		{
			phys_gjk_cache_info_internal * const next = m_list_head->m_next_gjk_ci;
			m_search_tree.remove(m_list_head->m_key);
			m_list_phys_gjk_cache_info_internal.free(m_list_head);
			m_list_head = next;
		}
		PHYS_ASSERT(m_list_phys_gjk_cache_info_internal.get_count() == 0);
	}

	// Finds or creates the entry for (id1, id2) and flips it to the caller's order.
	// Returns NULL when the cache is full or the PMM pool is exhausted.
	phys_gjk_cache_info * get_gjk_cache_info(phys_gjk_geom_id id1, phys_gjk_geom_id id2, const bool /*no_error*/ = false)
	{
		PHYS_ASSERT(id1 != id2);
		bool swapped;
		if (id1 > id2)
		{
			swapped = true;
			const phys_gjk_geom_id temp = id1;
			id1 = id2;
			id2 = temp;
		}
		else
		{
			swapped = false;
		}
		const phys_gjk_geom_id_pair_key key(id1, id2);

		phys_gjk_cache_info_internal * gjk_ci = m_search_tree.find(key);
		if (gjk_ci)
		{
			gjk_ci->update_swapped(swapped);
		}
		else
		{
			PHYS_ASSERT_INFO(pai_gjk_cache_system_max_num_gjk_ci, m_list_phys_gjk_cache_info_internal.get_count() < m_max_num_gjk_ci, "max num gjk_ci reached.");
			if (m_list_phys_gjk_cache_info_internal.get_count() < m_max_num_gjk_ci)
			{
				gjk_ci = m_list_phys_gjk_cache_info_internal.allocate();
				PHYS_ASSERT_INFO(pai_gjk_cache_system_create_gjk_ci, gjk_ci, "gjk_ci memory allocation failed.");
				if (gjk_ci)
				{
					gjk_ci->constructor(key, swapped);
					m_search_tree.add(key, gjk_ci);
					gjk_ci->m_next_gjk_ci = m_list_head;
					m_list_head = gjk_ci;
				}
			}
		}
		return gjk_ci;
	}

	// Lookup under a read lock; creation under the write lock.
	phys_gjk_cache_info * get_gjk_cache_info_mutex(phys_gjk_geom_id id1, phys_gjk_geom_id id2, tlAtomicReadWriteMutex & query_mutex, const bool /*no_error*/ = false)
	{
		PHYS_ASSERT(id1 != id2);
		bool swapped;
		if (id1 > id2)
		{
			swapped = true;
			const phys_gjk_geom_id temp = id1;
			id1 = id2;
			id2 = temp;
		}
		else
		{
			swapped = false;
		}
		const phys_gjk_geom_id_pair_key key(id1, id2);

		query_mutex.ReadLock();
		phys_gjk_cache_info_internal * gjk_ci = m_search_tree.find(key);
		query_mutex.ReadUnlock();
		if (gjk_ci)
		{
			gjk_ci->update_swapped(swapped);
			return gjk_ci;
		}

		PHYS_ASSERT_INFO(pai_gjk_cache_system_max_num_gjk_ci, m_list_phys_gjk_cache_info_internal.get_count() < m_max_num_gjk_ci, "max num gjk_ci reached.");
		if (m_list_phys_gjk_cache_info_internal.get_count() >= m_max_num_gjk_ci)
			return gjk_ci;

		query_mutex.WriteLock();
		PHYS_ASSERT(m_search_tree.find(key) == NULL);
		gjk_ci = m_list_phys_gjk_cache_info_internal.allocate();
		if (gjk_ci)
		{
			gjk_ci->constructor(key, swapped);
			m_search_tree.add(key, gjk_ci);
			gjk_ci->m_next_gjk_ci = m_list_head;
			m_list_head = gjk_ci;
			query_mutex.WriteUnlock();
			return gjk_ci;
		}
		PHYS_ASSERT_INFO(pai_gjk_cache_system_create_gjk_ci, gjk_ci, "gjk_ci memory allocation failed.");
		query_mutex.WriteUnlock();
		return NULL;
	}

	// Drops every entry not touched since the last sweep and clears the touched flag on the rest.
	void update_cache()
	{
		phys_gjk_cache_info_internal ** gjk_ci_i = &m_list_head;
		for (phys_gjk_cache_info_internal * gjk_ci = m_list_head; gjk_ci; gjk_ci = *gjk_ci_i)
		{
			if (gjk_ci->get_flag(phys_gjk_cache_info::FLAG_WAS_TOUCHED))
			{
				gjk_ci_i = &gjk_ci->m_next_gjk_ci;
				gjk_ci->set_flag(phys_gjk_cache_info::FLAG_WAS_TOUCHED, false);
			}
			else
			{
				*gjk_ci_i = gjk_ci->m_next_gjk_ci;
				m_search_tree.remove(gjk_ci->m_key);
				m_list_phys_gjk_cache_info_internal.free(gjk_ci);
			}
		}
	}
};
