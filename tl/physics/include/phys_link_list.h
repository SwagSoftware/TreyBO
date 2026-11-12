#pragma once

// Singly linked lists:
//  - phys_link_list1: list with m_first / m_last; add checks the entry is not already on it.
//  - phys_simple_link_list: push-front list of T with m_next_link.
// phys_link_list_base and phys_link_list are in phys_mem.h.
//
// T needs get_next_link() / set_next_link() (derive from phys_link_list_base<T>) for phys_link_list1,
// and a T *m_next_link member for phys_simple_link_list.

#include "tl/base/include/tl_defs.h"
#include "tl/physics/include/phys_mem.h"

template <class T>
class phys_link_list1
{
	T * m_first;
	T * m_last;
	int m_alloc_count;

public:
	phys_link_list1() :
		m_first(0), m_last(0), m_alloc_count(0) {}

	void add(T * p)
	{
		for (T * i = m_first; i; i = i->get_next_link())
			PHYS_ASSERT(i != p);
		m_alloc_count++;
		if (m_last)
			m_last->set_next_link(p);
		else
			m_first = p;
		m_last = p;
		m_last->set_next_link(0);
	}

	void remove(T * p)
	{
		for (T *i = m_first, *last_i = 0; i; last_i = i, i = i->get_next_link())
		{
			if (p == i)
			{
				m_alloc_count--;
				if (last_i)
					last_i->set_next_link(i->get_next_link());
				else
					m_first = i->get_next_link();
				if (i == m_last)
				{
					m_last = last_i;
					PHYS_ASSERT_STR(!last_i || last_i->get_next_link() == 0, "!last_i || last_i->get_next_link() == NULL");
				}
				return;
			}
		}
		PHYS_ASSERT(0);
	}

	void remove_all()
	{
		m_first = 0;
		m_last = 0;
		m_alloc_count = 0;
	}

	const int get_count() const { return m_alloc_count; }
	T * get_first() const { return m_first; }
	T * get_last() const { return m_last; }

	class iterator
	{
		T * m_ptr;

	public:
		iterator(T * ptr) :
			m_ptr(ptr) {}
		void operator++(int) { m_ptr = m_ptr->get_next_link(); }
		iterator & operator++() // prefix form for range-for
		{
			m_ptr = m_ptr->get_next_link();
			return *this;
		}
		bool operator!=(const iterator & i) const { return i.m_ptr != m_ptr; }
		T & operator*() { return *m_ptr; }
	};
	iterator begin() { return iterator(m_first); }
	iterator end() { return iterator(0); }
};

template <class T>
class phys_simple_link_list
{
public:
	T * m_first;

	phys_simple_link_list() :
		m_first(NULL) {}

	void add(T * p)
	{
		p->m_next_link = m_first;
		m_first = p;
	}

	void remove_all() { m_first = NULL; }
	T * get_first() const { return m_first; }

	class iterator
	{
		T * m_ptr;

	public:
		iterator(T * ptr) :
			m_ptr(ptr) {}
		void operator++(int) { m_ptr = m_ptr->m_next_link; }
		iterator & operator++()
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
