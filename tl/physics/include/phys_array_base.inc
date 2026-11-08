#pragma once

// Retail: codsrc\tl\physics\include\phys_array_base.inc. Header-only template.
// TREYBO: retail (like SM3) #includes this file inside each array class body; TreyBO only needs
// phys_static_array, so the class lives here whole and this file can be included directly.
//
// Retail instantiations (BO1 assert lines differ from SM3, so the BO1 .inc was edited):
//   operator[]       phys_static_array<geom_plane,128>      0x444de0 (returns T &)
//   add              phys_static_array<geom_plane,128>      0x444d50 (+5 more: none constructs T)
//   back             phys_static_array<phys_vec3 *,6144>    0x854f80 (returns T &)
//   remove           phys_static_array<phys_vec3 *,6144>    0x8549b0
//   remove_slow      phys_static_array<ch_triangle,256>     0x854b60 (BO1 only)
//   get_list_head    phys_static_array<ch_triangle,256>     0x854c20 (BO1 only)
//   call_destructors phys_static_array<float,512>           0x85b7e0
//   ctor / dtor / remove_all inlined in clip_winding        0x8589d0
// is_member is inlined in remove / remove_slow. The rest are SM3 bodies with no BO1 copy.

#include "tl/base/include/tl_defs.h"
#include "tl/base/include/tl_system.h"

template <class T, const int m_slot_array_size>
class phys_static_array
{
private:
    TL_ALIGN(16) char m_buffer[m_slot_array_size * sizeof(T)];
    T *const m_slot_array;
    int m_alloc_count;

public:
    phys_static_array() : m_slot_array((T *)m_buffer), m_alloc_count(0) {} // TREYBO FIX: no buffer memset (kisakblack added one)
    ~phys_static_array() { call_destructors(); }
    void operator=(phys_static_array &) { tlFatal("assignment not allowed"); }

private:
    void call_destructors()
    {
        for (int i = 0; i < m_alloc_count; i++)
            m_slot_array[i].~T();
    }

    void reset_buffer()
    {
        call_destructors();
        m_alloc_count = 0;
    }

public:
    // BO1 add takes int and does not construct the slot (SM3 placement-new'd it).
    T *add(const int no_error = false, const char *error_msg = "phys array add overflow.")
    {
        if (m_alloc_count < m_slot_array_size) // TREYBO FIX(0x444d50): kisakblack hard-coded 512
            return &m_slot_array[m_alloc_count++];
        if (!no_error)
            tlFatal(error_msg);
        return NULL;
    }

    T *add_nc(const int no_error = false, const char *error_msg = "phys array add_nc overflow.")
    {
        if (m_alloc_count >= m_slot_array_size)
        {
            if (!no_error)
                tlFatal(error_msg);
            return NULL;
        }
        return &m_slot_array[m_alloc_count++];
    }

    T *add_block_nc(const int size, const int no_error = false, const char *error_msg = "phys array add_block_nc overflow.")
    {
        if (m_alloc_count + size > m_slot_array_size)
        {
            if (!no_error)
                tlFatal(error_msg);
            return NULL;
        }
        T *const rbc = &m_slot_array[m_alloc_count];
        m_alloc_count += size;
        return rbc;
    }

    // Swap-with-last removal.
    // TREYBO: SM3's PHYS_ASSERT(sizeof(T) == 4) is dropped: it is folded away in BO1 and fails for pointers on x64.
    void remove(T *data)
    {
        PHYS_ASSERT(is_member(data));
        data->~T();
        *data = back();
        m_alloc_count--;
    }

    // BO1: swap-with-last without the destructor call.
    void remove_slow(T *data)
    {
        PHYS_ASSERT(is_member(data));
        *data = m_slot_array[--m_alloc_count];
    }

    void remove_all() { reset_buffer(); }

    bool is_member(T *data) const
    {
        if (((char *)data - (char *)m_slot_array) % sizeof(T) != 0)
            return false;
        if (!(data >= m_slot_array && data < m_slot_array + m_alloc_count))
            return false;
        return true;
    }

    T *find_by_val(const T &data) const
    {
        for (int i = 0; i < m_alloc_count; i++)
        {
            if (m_slot_array[i] == data)
                return m_slot_array + i;
        }
        return NULL;
    }

    void remove_by_val(const T &data)
    {
        T *const i = find_by_val(data);
        remove(i);
    }

    bool is_member_by_val(const T &data) const { return find_by_val(data) != NULL; }

    T &operator[](const int i)
    {
        PHYS_ASSERT(i >= 0 && i < m_alloc_count);
        return m_slot_array[i];
    }

    const T &operator[](const int i) const
    {
        PHYS_ASSERT(i >= 0 && i < m_alloc_count);
        return m_slot_array[i];
    }

    const int get_max_slots() const { return m_slot_array_size; }
    const int get_available_slots() const { return m_slot_array_size - m_alloc_count; }
    const int get_used_slots() const { return m_alloc_count; }
    const int get_count() const { return m_alloc_count; }

    class iterator
    {
        T *m_ptr;

    public:
        iterator(T *ptr) : m_ptr(ptr) {}
        void operator++(int) { m_ptr++; }
        iterator &operator++() // TREYBO: prefix form for range-for
        {
            m_ptr++;
            return *this;
        }
        bool operator!=(const iterator &i) const { return i.m_ptr != m_ptr; }
        T &operator*() { return *m_ptr; }
    };
    iterator begin() { return iterator(m_slot_array); }
    iterator end() { return iterator(m_slot_array + m_alloc_count); }

    void push_back(const T &data) { *add() = data; }

    T &back()
    {
        PHYS_ASSERT(m_alloc_count > 0);
        return m_slot_array[m_alloc_count - 1];
    }

    T *get_list_head()
    {
        PHYS_ASSERT(m_alloc_count > 0);
        return m_slot_array;
    }
};
