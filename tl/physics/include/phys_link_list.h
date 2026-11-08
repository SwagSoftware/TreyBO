#pragma once

// TreyBO-new header. Holds the singly linked lists whose retail home is not a tl header:
//  - phys_link_list1: retail asserts name codsrc\src\physics\phys_local.h (game side). It is the SM3
//    phys_link_list (m_first / m_last) kept by CoD under a new name, plus an "i != p" check in add.
//    Retail copies: add / remove <PhysObjUserData> 0x854710 / 0x8547c0.
//  - phys_simple_link_list: no assert text anywhere. Push-front list of T with m_next_link.
//    Inlined: ctor in create(phys_free_list<rigid_body_constraint_contact> &, ...) 0xa172d0,
//    add in rigid_body_constraint_contact::add_cpi_simple 0xa2ca70,
//    remove_all in pulse_sum_constraint_solver::execute_constraint_solver 0xa2c1d0.
// phys_link_list_base and the BO1 phys_link_list are in phys_mem.h (their retail header).
//
// T needs get_next_link() / set_next_link() (derive from phys_link_list_base<T>) for phys_link_list1,
// and a T *m_next_link member for phys_simple_link_list.

#include "tl/base/include/tl_defs.h"
#include "tl/physics/include/phys_mem.h"

template <class T>
class phys_link_list1
{
    T *m_first;
    T *m_last;
    int m_alloc_count;

public:
    phys_link_list1() : m_first(0), m_last(0), m_alloc_count(0) {}

    void add(T *p)
    {
        for (T *i = m_first; i; i = i->get_next_link())
            PHYS_ASSERT(i != p);
        m_alloc_count++;
        if (m_last)
            m_last->set_next_link(p);
        else
            m_first = p;
        m_last = p;
        m_last->set_next_link(0);
    }

    void remove(T *p)
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
    T *get_first() const { return m_first; } // TREYBO: accessor
    T *get_last() const { return m_last; }

    class iterator
    {
        T *m_ptr;

    public:
        iterator(T *ptr) : m_ptr(ptr) {}
        void operator++(int) { m_ptr = m_ptr->get_next_link(); }
        iterator &operator++() // TREYBO: prefix form for range-for
        {
            m_ptr = m_ptr->get_next_link();
            return *this;
        }
        bool operator!=(const iterator &i) const { return i.m_ptr != m_ptr; }
        T &operator*() { return *m_ptr; }
    };
    iterator begin() { return iterator(m_first); }
    iterator end() { return iterator(0); }
};

template <class T>
class phys_simple_link_list
{
public:
    T *m_first;

    phys_simple_link_list() : m_first(NULL) {}

    // TREYBO: method names not recovered (bodies are retail inlines).
    void add(T *p)
    {
        p->m_next_link = m_first;
        m_first = p;
    }

    void remove_all() { m_first = NULL; }
    T *get_first() const { return m_first; }

    class iterator
    {
        T *m_ptr;

    public:
        iterator(T *ptr) : m_ptr(ptr) {}
        void operator++(int) { m_ptr = m_ptr->m_next_link; }
        iterator &operator++()
        {
            m_ptr = m_ptr->m_next_link;
            return *this;
        }
        bool operator!=(const iterator &i) const { return i.m_ptr != m_ptr; }
        T &operator*() { return *m_ptr; }
    };
    iterator begin() { return iterator(m_first); }
    iterator end() { return iterator(NULL); }
};
