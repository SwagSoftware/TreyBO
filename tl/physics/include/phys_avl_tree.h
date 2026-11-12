#pragma once

// In-place AVL tree: the node record lives inside T2. The accessor class (often T2 itself) provides
//   static T1 &get_avl_key(T2 *);                     static const T1 &get_avl_key(const T2 *);
//   static phys_inplace_avl_tree_node<T2> *get_avl_node(T2 *);
//   static const phys_inplace_avl_tree_node<T2> *get_avl_node(const T2 *);
// T1 needs operator< and operator==.

#include "tl/base/include/tl_defs.h"

template <class T>
struct phys_inplace_avl_tree_node
{
	T * m_left;
	T * m_right;
	int m_balance; // right height - left height, in [-1, +1] when balanced
};

// Lookup from an arbitrary root.
template <class T1, class T2, class accessor>
inline T2 * avl_tree_find(T2 * tree_root, const T1 & key)
{
	T2 * cur = tree_root;
	while (cur)
	{
		if (key == accessor::get_avl_key(cur))
			break;
		cur = (key < accessor::get_avl_key(cur)) ? accessor::get_avl_node(cur)->m_left : accessor::get_avl_node(cur)->m_right;
	}
	return cur;
}

template <class T1, class T2, class accessor>
class phys_inplace_avl_tree
{
public:
	struct stack_item
	{
		T2 ** m_node;
		int m_child; // -1 went left, +1 went right
	};

	T2 * m_tree_root;

	phys_inplace_avl_tree() :
		m_tree_root(NULL) {}

	T2 * find(const T1 & key) const
	{
		T2 * cur = m_tree_root;
		while (cur)
		{
			if (key == accessor::get_avl_key(cur))
				break;
			cur = (key < accessor::get_avl_key(cur)) ? accessor::get_avl_node(cur)->m_left : accessor::get_avl_node(cur)->m_right;
		}
		return cur;
	}

	// key must not be in the tree yet.
	void add(const T1 & key, T2 * data)
	{
		stack_item the_stack[32];
		stack_item * cur_item = the_stack;
		cur_item->m_node = &m_tree_root;

		while (*cur_item->m_node)
		{
			T2 * const root = *cur_item->m_node;
			PHYS_ASSERT(cur_item + 1 - the_stack < 32);
			stack_item * const next_item = cur_item + 1;
			if (key < accessor::get_avl_key(root))
			{
				cur_item->m_child = -1;
				next_item->m_node = &accessor::get_avl_node(root)->m_left;
			}
			else
			{
				PHYS_ASSERT_STR(accessor::get_avl_key(root) < key, "key > accessor::get_avl_key(root)");
				cur_item->m_child = +1;
				next_item->m_node = &accessor::get_avl_node(root)->m_right;
			}
			cur_item = next_item;
		}

		*cur_item->m_node = data;
		accessor::get_avl_node(data)->m_left = NULL;
		accessor::get_avl_node(data)->m_right = NULL;
		accessor::get_avl_node(data)->m_balance = 0;
		accessor::get_avl_key(data) = key;

		// Walk back up while the subtree grew.
		do
		{
			if (cur_item <= the_stack)
				break;
			--cur_item;
			T2 *& root = *cur_item->m_node;
			accessor::get_avl_node(root)->m_balance += cur_item->m_child;
			if (accessor::get_avl_node(root)->m_balance == -2)
			{
				PHYS_ASSERT(accessor::get_avl_node(accessor::get_avl_node(root)->m_left)->m_balance == -1 || accessor::get_avl_node(accessor::get_avl_node(root)->m_left)->m_balance == 1);
				if (accessor::get_avl_node(accessor::get_avl_node(root)->m_left)->m_balance == 1)
					rotate_left(accessor::get_avl_node(root)->m_left);
				rotate_right(root);
				PHYS_ASSERT(accessor::get_avl_node(root)->m_balance == 0);
			}
			else if (accessor::get_avl_node(root)->m_balance == 2)
			{
				PHYS_ASSERT(accessor::get_avl_node(accessor::get_avl_node(root)->m_right)->m_balance == -1 || accessor::get_avl_node(accessor::get_avl_node(root)->m_right)->m_balance == 1);
				if (accessor::get_avl_node(accessor::get_avl_node(root)->m_right)->m_balance == -1)
					rotate_right(accessor::get_avl_node(root)->m_right);
				rotate_left(root);
				PHYS_ASSERT(accessor::get_avl_node(root)->m_balance == 0);
			}
		} while (accessor::get_avl_node(*cur_item->m_node)->m_balance != 0);
	}

	// key must be in the tree.
	void remove(const T1 & key)
	{
		stack_item the_stack[32];
		stack_item * cur_item = the_stack;
		cur_item->m_node = &m_tree_root;

		T2 * root;
		for (;;)
		{
			root = *cur_item->m_node;
			PHYS_ASSERT(root);
			PHYS_ASSERT(cur_item + 1 - the_stack < 32);
			stack_item * const next_item = cur_item + 1;
			if (key < accessor::get_avl_key(root))
			{
				cur_item->m_child = -1;
				next_item->m_node = &accessor::get_avl_node(root)->m_left;
			}
			else if (accessor::get_avl_key(root) < key)
			{
				cur_item->m_child = +1;
				next_item->m_node = &accessor::get_avl_node(root)->m_right;
			}
			else
			{
				break;
			}
			cur_item = next_item;
		}
		PHYS_ASSERT(key == accessor::get_avl_key(root));

		T2 ** const node_to_be_removed = cur_item->m_node;
		if (accessor::get_avl_node(*node_to_be_removed)->m_right)
		{
			// Replace with the in-order successor (leftmost node of the right subtree).
			cur_item->m_child = +1;
			++cur_item;
			cur_item->m_node = &accessor::get_avl_node(*node_to_be_removed)->m_right;
			stack_item * const right_item = cur_item;
			while (accessor::get_avl_node(*cur_item->m_node)->m_left)
			{
				PHYS_ASSERT(cur_item + 1 - the_stack < 32);
				stack_item * const next_item = cur_item + 1;
				cur_item->m_child = -1;
				next_item->m_node = &accessor::get_avl_node(*cur_item->m_node)->m_left;
				cur_item = next_item;
			}

			T2 * const replace_node = *cur_item->m_node;
			*cur_item->m_node = accessor::get_avl_node(replace_node)->m_right;
			accessor::get_avl_node(replace_node)->m_left = accessor::get_avl_node(*node_to_be_removed)->m_left;
			accessor::get_avl_node(replace_node)->m_right = accessor::get_avl_node(*node_to_be_removed)->m_right;
			accessor::get_avl_node(replace_node)->m_balance = accessor::get_avl_node(*node_to_be_removed)->m_balance;
			right_item->m_node = &accessor::get_avl_node(replace_node)->m_right;
			*node_to_be_removed = replace_node;
		}
		else
		{
			*node_to_be_removed = accessor::get_avl_node(*node_to_be_removed)->m_left;
		}

		// Walk back up while the subtree shrank.
		do
		{
			if (cur_item <= the_stack)
				break;
			--cur_item;
			T2 *& node = *cur_item->m_node;
			accessor::get_avl_node(node)->m_balance -= cur_item->m_child;
			if (accessor::get_avl_node(node)->m_balance == -2)
			{
				if (accessor::get_avl_node(accessor::get_avl_node(node)->m_left)->m_balance == 1)
					rotate_left(accessor::get_avl_node(node)->m_left);
				rotate_right(node);
				PHYS_ASSERT_STR(accessor::get_avl_node(node)->m_balance == 0 || accessor::get_avl_node(node)->m_balance == +1,
					"accessor::get_avl_node(root)->m_balance == 0 || accessor::get_avl_node(root)->m_balance == +1");
			}
			else if (accessor::get_avl_node(node)->m_balance == 2)
			{
				if (accessor::get_avl_node(accessor::get_avl_node(node)->m_right)->m_balance == -1)
					rotate_right(accessor::get_avl_node(node)->m_right);
				rotate_left(node);
				PHYS_ASSERT_STR(accessor::get_avl_node(node)->m_balance == 0 || accessor::get_avl_node(node)->m_balance == -1,
					"accessor::get_avl_node(root)->m_balance == 0 || accessor::get_avl_node(root)->m_balance == -1");
			}
			// balance 0 means the subtree shrank: keep walking up
		} while (accessor::get_avl_node(*cur_item->m_node)->m_balance == 0);
	}

private:
	void rotate_left(T2 *& root)
	{
		T2 * const save_right = accessor::get_avl_node(root)->m_right;
		accessor::get_avl_node(root)->m_right = accessor::get_avl_node(save_right)->m_left;
		accessor::get_avl_node(save_right)->m_left = root;
		accessor::get_avl_node(root)->m_balance -= (accessor::get_avl_node(save_right)->m_balance <= 0 ? 0 : accessor::get_avl_node(save_right)->m_balance) + 1;
		accessor::get_avl_node(save_right)->m_balance -= (accessor::get_avl_node(root)->m_balance >= 0 ? 0 : -accessor::get_avl_node(root)->m_balance) + 1;
		root = save_right;
	}

	void rotate_right(T2 *& root)
	{
		T2 * const save_left = accessor::get_avl_node(root)->m_left;
		accessor::get_avl_node(root)->m_left = accessor::get_avl_node(save_left)->m_right;
		accessor::get_avl_node(save_left)->m_right = root;
		accessor::get_avl_node(root)->m_balance += (accessor::get_avl_node(save_left)->m_balance >= 0 ? 0 : -accessor::get_avl_node(save_left)->m_balance) + 1;
		accessor::get_avl_node(save_left)->m_balance += (accessor::get_avl_node(root)->m_balance <= 0 ? 0 : accessor::get_avl_node(root)->m_balance) + 1;
		root = save_left;
	}
};
