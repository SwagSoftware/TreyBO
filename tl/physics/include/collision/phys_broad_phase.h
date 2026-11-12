#pragma once

// Pipeline (broad_phase_process, once per sub-step from the host collision callback):
//  1. object vs environment: every bpb on G_BPM->g_list_bpb is swept, the swept volumes are clustered
//     (136-unit cells) and each cluster is handed to the host's broad_phase_terrain_query_callback, which returns
//     world geometry as FLAG_IS_BPI_ENV infos (see phys_environment.h). Pairs go to g_list_phys_collide_data.
//  2. explicit pairs (g_list_broad_phase_collision_pair).
//  3. object vs object: incremental 3-axis sweep and prune (axis_aligned_sweep_and_prune).
//  4. narrow phase: GJK + contact manifold per pair (phys_collision_multithreaded.cpp).
//
// Must not include phys_collision.h (phys_collision.h includes this header).

#include "tl/base/include/tl_defs.h"
#include "tl/base/include/tl_thread.h"
#include "tl/physics/include/phys_math.h"
#include "tl/physics/include/phys_assert.h"
#include "tl/physics/include/phys_mem.h"
#include "tl/physics/include/phys_mutex.h"
#include "tl/physics/include/phys_avl_tree.h"
#include "tl/physics/include/phys_transient_allocator.h"
#include "tl/physics/include/rigid_body.h"
#include "tl/physics/include/rb_vehicle_model.h"
#include "tl/physics/include/collision/phys_gjk.h"
#include "tl/physics/include/collision/phys_gjk_cache_system.h"
#include "tl/physics/include/collision/phys_broad_phase_base.h"

class phys_collision_pair;
class rigid_body_constraint_wheel;
class broad_phase_memory;
class broad_phase_environement_query_results;

// Sizes of the broad-phase pools (typically 2048 / 2048 / 35).
class broad_phase_memory_info
{
public:
	int m_max_num_gjk_ci;
	int m_max_num_sap_active_pairs;
	int m_max_num_surface_types;

	broad_phase_memory_info() :
		m_max_num_gjk_ci(0), m_max_num_sap_active_pairs(0), m_max_num_surface_types(0) {}
};

// Input of bp_env_jq_batch_function1: the running bounds of every swept volume.
class broad_phase_prolog_task_input
{
public:
	phys_vec3 * m_aabb_min;
	phys_vec3 * m_aabb_max;
};

// Friction / bounce / priority of one surface-type pair (G_BPM->g_surface_type_info_database, triangular table).
class phys_surface_type_info
{
public:
	float m_friction_coef;
	float m_bounce_coef;
	unsigned int m_solver_priority;
	bool m_no_overflow_error;
};

// One wheel ray of a vehicle group, rebuilt every broad_phase_group::collision_prolog (transient memory).
class TL_ALIGN(16) phys_wheel_collide_info
{
public:
	phys_vec3 m_ray_pos;
	phys_vec3 m_ray_dir; // whole segment (p1 - p0)
	phys_vec3 m_hitn; // closest hit normal (geom frame until collision_epilog)
	float m_hit_t; // closest hit along m_ray_dir, 1 = none
	broad_phase_info * m_hit_bpi;

	void collision_process(broad_phase_info * bpi);
	void collision_epilog(rigid_body_constraint_wheel * rbc_wheel);
};

// The broad-phase stand-in of a multi-geom body: one swept volume around all of its infos.
class broad_phase_group : public broad_phase_base
{
public:
	enum broad_phase_group_flags_e
	{
		FLAG_DO_INITIAL_TUNNEL_TEST = (1 << 9), // set by the host on creation, cleared after one test
	};

	broad_phase_info * m_list_bpi_head; // linked through m_list_bpb_next
	int m_bpi_count;
	rb_vehicle_model * m_rbvm; // optional: its wheels are ray-tested against the environment
	phys_wheel_collide_info * m_list_wci; // m_rbvm->m_wheels.get_count() entries, transient

	void set()
	{
		m_list_bpi_head = NULL;
		m_bpi_count = 0;
		m_flags = 0;
		m_rbvm = NULL;
		m_list_wci = NULL;
		m_sap_node = NULL;
		m_list_bpb_next = NULL;
		m_user_data = NULL;
		m_flags |= FLAG_IS_BPG;
		m_env_collision_flags = 0;
		m_my_collision_type_flags = 0;
	}

	void add_bpi(broad_phase_info * bpi);
	void collision_prolog();
	void collision_epilog();
};

inline broad_phase_group * broad_phase_base::get_bpg()
{
	PHYS_ASSERT(is_bpg());
	return static_cast<broad_phase_group *>(this);
}

inline void environment_collision_list_add(broad_phase_base * bpb);
inline void environment_collision_list_remove(broad_phase_base * bpb);

inline void broad_phase_group::add_bpi(broad_phase_info * bpi)
{
	PHYS_ASSERT(!bpi->get_flag(broad_phase_info::FLAG_ON_BPG_LIST));
	environment_collision_list_remove(bpi);
	bpi->m_flags |= FLAG_ON_BPG_LIST;
	bpi->m_list_bpb_next = m_list_bpi_head;
	m_list_bpi_head = bpi;
	++m_bpi_count;
	m_env_collision_flags |= bpi->m_env_collision_flags;
	m_my_collision_type_flags |= bpi->m_my_collision_type_flags;
}

// Host-requested pair, tested every pass (create_broad_phase_collision_pair; the caller fills both infos).
class broad_phase_collision_pair
{
public:
	broad_phase_info * m_bpi1;
	broad_phase_info * m_bpi2;
	broad_phase_collision_pair * m_next_bpcp;
};

// ---------------------------------------------------------------------------------------------
// Environment query (the host side is documented in phys_environment.h)
// ---------------------------------------------------------------------------------------------

// One cluster's swept volume, built by init_bpeqi. wace = "with active constraint epsilon": the cluster's whace
// ("with half active constraint epsilon") volume padded by hace once more, i.e. the full epsilon
// (0.51 per side, twice).
class TL_ALIGN(16) broad_phase_environment_query_input
{
public:
	phys_vec3 trace_aabb_min_wace;
	phys_vec3 trace_aabb_max_wace;
	phys_vec3 trace_translation;
	unsigned int env_collision_flags; // union of the cluster members' m_env_collision_flags
};

// Singly linked list of bpb pointers whose nodes survive across clusters of one job: m_list_cur is rewound to
// &m_list per cluster and add() reuses existing nodes before allocating (G_BPM->g_collision_memory_buffer).
class broad_phase_base_list
{
public:
	class node
	{
	public:
		broad_phase_base * m_bpb;
		node * m_next;
	};

	node * m_list;
	node ** m_list_cur; // next slot to fill; the live entries end at *m_list_cur

	void add(broad_phase_base * bpb);
};

// What the host query returns: the FLAG_IS_BPI_ENV infos overlapping the query volume.
class broad_phase_environement_query_results
{
public:
	broad_phase_base_list m_list_bpi_env;
	int m_list_bpi_env_count;
	int m_thread_id; // index of the job running the query, in [0, 6)
	unsigned int m_env_collision_flags; // union of the returned infos' flags

	void add(broad_phase_base * bpb);
};

// Host world-geometry query.
class broad_phase_terrain_query_callback
{
public:
	virtual void query(const broad_phase_environment_query_input & bpeqi, broad_phase_environement_query_results * bpeqr) = 0;
};

// Optional trigger on a world-geometry info. When a free body's geom really intersects it (GJK intersection
// test), auto_activate is called once, under g_bp_auto_activate_mutex. No contact pairs are made against such
// an info.
class phys_auto_activate_callback
{
public:
	virtual bool has_auto_activated() = 0;
	virtual void auto_activate(broad_phase_info * bpi) = 0;
};

// Key of a cached world-geometry info. The host chooses both words (e.g. entity pointer, geom pointer).
class bpei_database_id
{
public:
	unsigned int m_id1;
	unsigned int m_id2;

	bpei_database_id() {}
	bpei_database_id(const unsigned int id1, const unsigned int id2) :
		m_id1(id1), m_id2(id2) {}

	// Lexicographic.
	const int operator<(const bpei_database_id & k) const { return (m_id1 == k.m_id1) ? (m_id2 < k.m_id2) : (m_id1 < k.m_id1); }
	const int operator==(const bpei_database_id & k) const { return (m_id1 == k.m_id1) && (m_id2 == k.m_id2); }
};

// One cached world geom: a persistent GJK geom id, and (per pass) the FLAG_IS_BPI_ENV info built for it.
class broad_phase_environment_info
{
public:
	struct avl_tree_accessor
	{
		static bpei_database_id & get_avl_key(broad_phase_environment_info * n) { return n->m_database_id; }
		static const bpei_database_id & get_avl_key(const broad_phase_environment_info * n) { return n->m_database_id; }
		static phys_inplace_avl_tree_node<broad_phase_environment_info> * get_avl_node(broad_phase_environment_info * n) { return &n->m_avl_tree_node; }
		static const phys_inplace_avl_tree_node<broad_phase_environment_info> * get_avl_node(const broad_phase_environment_info * n) { return &n->m_avl_tree_node; }
	};

	void * m_data; // this pass's broad_phase_info (bpi_env), NULL until the host builds it
	minspec_mutex m_mutex; // held by the host while it builds m_data
	unsigned int m_gjk_geom_id;
	broad_phase_environment_info * m_next_bpei;
	phys_inplace_avl_tree_node<broad_phase_environment_info> m_avl_tree_node;
	bpei_database_id m_database_id;
};

// Unique 32-bit GJK geom ids (lock-free counter). Starts at 1; 0 means "wrapped".
class gjk_unique_id_database_t
{
public:
	volatile unsigned int m_counter;

	const unsigned int get_unique_id()
	{
		unsigned int id;
		do
		{
			id = tlAtomicLoad(&m_counter);
			PHYS_ASSERT_MSG(id != 0, "gjk unique_id wrapped around.");
		} while (tlAtomicCompareExchange(&m_counter, id + 1, id) != id);
		return id;
	}
};

extern gjk_unique_id_database_t g_gjk_unique_id_database; // = { 1 }

// Cache of world-geom infos across passes. update_database (end of broad_phase_process) drops every entry
// whose m_data was not set this pass and clears m_data on the rest.
class bpei_database_t
{
public:
	phys_inplace_avl_tree<bpei_database_id, broad_phase_environment_info, broad_phase_environment_info::avl_tree_accessor> m_bpei_map;
	phys_simple_allocator<broad_phase_environment_info> m_bpei_allocator;
	broad_phase_environment_info * m_bpei_list;
	minspec_read_write_mutex m_mutex;

	bpei_database_t() :
		m_bpei_list(NULL) {}
	~bpei_database_t() { purge_database(); }

	void purge_database()
	{
		for (broad_phase_environment_info *bpei = m_bpei_list, *next_bpei; bpei; bpei = next_bpei)
		{
			next_bpei = bpei->m_next_bpei;
			m_bpei_map.remove(bpei->m_database_id);
			m_bpei_allocator.free(bpei);
		}
		m_bpei_list = NULL;
	}

	void update_database()
	{
		broad_phase_environment_info ** prev_next = &m_bpei_list;
		for (broad_phase_environment_info * bpei = m_bpei_list; bpei; bpei = *prev_next)
		{
			if (bpei->m_data)
			{
				prev_next = &bpei->m_next_bpei;
				bpei->m_data = NULL;
			}
			else
			{
				*prev_next = bpei->m_next_bpei;
				m_bpei_map.remove(bpei->m_database_id);
				m_bpei_allocator.free(bpei);
			}
		}
	}

	// Single-threaded find-or-create.
	broad_phase_environment_info * get_bpei(bpei_database_id database_id)
	{
		broad_phase_environment_info * bpei = m_bpei_map.find(database_id);
		if (!bpei)
		{
			bpei = create_bpei(database_id);
			bpei->m_next_bpei = m_bpei_list;
			m_bpei_list = bpei;
		}
		PHYS_ASSERT(bpei);
		return bpei;
	}

	// Find-or-create from any query job.
	broad_phase_environment_info * get_bpei_mt(bpei_database_id database_id)
	{
		m_mutex.ReadLock();
		broad_phase_environment_info * bpei = m_bpei_map.find(database_id);
		m_mutex.ReadUnlock();
		if (!bpei)
		{
			m_mutex.WriteLock();
			bpei = m_bpei_map.find(database_id);
			if (!bpei)
			{
				bpei = create_bpei(database_id);
				bpei->m_next_bpei = m_bpei_list;
				m_bpei_list = bpei;
			}
			m_mutex.WriteUnlock();
		}
		PHYS_ASSERT(bpei);
		return bpei;
	}

private:
	broad_phase_environment_info * create_bpei(bpei_database_id database_id)
	{
		broad_phase_environment_info * const bpei = m_bpei_allocator.allocate();
		m_bpei_map.add(database_id, bpei);
		bpei->m_data = NULL;
		bpei->m_gjk_geom_id = g_gjk_unique_id_database.get_unique_id();
		return bpei;
	}
};

// One slice of g_list_bpb handled by broad_phase_process_object_environment_collision: [m_bpb_i_start, m_bpb_i_end).
class bpi_environment_collision_info
{
public:
	broad_phase_base * m_bpb_i_start;
	broad_phase_base * m_bpb_i_end;
	int m_bpb_count;
	int m_bpb_last_count;
};

// ---------------------------------------------------------------------------------------------
// Sweep and prune (object vs object)
// ---------------------------------------------------------------------------------------------

// Three sorted endpoint lists; an active pair exists while two nodes overlap on all three axes. Endpoint
// order is maintained by merge_sort, whose swaps create pairs (a max passing a min); overlap ends are
// found in process_active_pair_list.
class axis_aligned_sweep_and_prune
{
public:
	class sap_node;

	class axis_element
	{
	public:
		sap_node * m_node;
		int m_min_max; // 0 = min endpoint, 1 = max endpoint
		float m_val;
		int m_ae_list_index; // position in the sorted list
		axis_element * m_next;
	};

	class sap_node
	{
	public:
		broad_phase_base * m_bpb;
		axis_element m_ae1[3][2]; // [axis][min/max]
		int m_updated; // set by update_ae_val, cleared by process

		void init(broad_phase_base * bpb, axis_element ** xlist, axis_element ** ylist, axis_element ** zlist);
		void update_ae_val();
	};

	class active_pair
	{
	public:
		sap_node * m_p1; // bpi first for a bpi/bpg pair
		sap_node * m_p2;
		active_pair * m_next;
		phys_gjk_cache_info * m_gjk_ci; // bpi/bpi pairs only; owned by the pair, not in the cache tree
	};

	phys_simple_allocator<sap_node> m_sap_node_allocator;
	axis_element * m_x_list;
	axis_element * m_y_list;
	axis_element * m_z_list;
	bool (*m_should_collide_callback)(const broad_phase_base *, const broad_phase_base *); // NULL = always
	phys_simple_allocator<active_pair> m_active_pair_allocator;
	int m_max_num_active_pairs;
	active_pair * m_list_bpi_bpi;
	active_pair * m_list_bpi_bpg;
	active_pair * m_list_bpg_bpg;

	const int are_overlapping(sap_node * n1, sap_node * n2);
	void remove(axis_element ** axis_list, sap_node * node);
	void swap(axis_element ** a1_ptr, axis_element ** a2_ptr);
	void merge_sort(axis_element ** list, int list_count);
	void init_system(int max_num_active_pairs);
	void add_active_pair(sap_node * p1, sap_node * p2);
	void create_sap_node(broad_phase_base * bpb);
	void remove(active_pair ** list_ap, sap_node * node);
	void destroy_sap_node(broad_phase_base * bpb);
	void process_active_pair_list();
	void process();
};

// ---------------------------------------------------------------------------------------------
// broad_phase_memory: every broad-phase global (G_BPM)
// ---------------------------------------------------------------------------------------------

class broad_phase_memory
{
public:
	tlAtomicMutex g_bp_auto_activate_mutex;
	tlAtomicReadWriteMutex g_bp_gjk_cache_mutex;
	phys_heap_gjk_cache_system_avl_tree g_phys_gjk_cache_system;
	phys_free_list<broad_phase_info> g_list_broad_phase_info;
	phys_free_list<broad_phase_group> g_list_broad_phase_group;
	phys_free_list<broad_phase_collision_pair> g_list_broad_phase_collision_pair;
	bpei_database_t g_bpei_database;
	broad_phase_terrain_query_callback * g_broad_phase_terrain_query_callback; // host world query; set after broad_phase_system_init
	broad_phase_base * g_list_bpb; // every bpb that collides with the environment
	int g_list_bpb_count;
	broad_phase_info * volatile m_list_bpi_env; // this pass's world-geom infos (allocate_bpi_env), via m_list_bpb_cluster_next
	volatile int m_bpi_env_count;
	int m_bpg_env_count;
	int m_bpg_env_bpi_count;
	int m_bpi_env_no_database_count;
	int m_memory_high_water;
	phys_link_list<phys_collision_pair> g_list_phys_collide_data; // this pass's narrow-phase pairs
	phys_transient_allocator g_collision_memory_buffer; // pairs, bpi_envs, wheel infos; reset by broad_phase_reset_buffer
	phys_surface_type_info * g_surface_type_info_database;
	int m_max_num_surface_types;
	int m_max_num_surface_type_infos;

	void list_bpb_remove(broad_phase_base * bpb_to_remove);
	static broad_phase_memory * allocate_buffer(const broad_phase_memory_info & bpmi);
};

extern broad_phase_memory * G_BPM;
extern axis_aligned_sweep_and_prune * g_axis_aligned_sweep_and_prune;

extern phys_assert_info pai_check_terrain_query_params; // (0, 2, true)
extern phys_assert_info pai_max_num_sap_active_pair; // (0, 2, true)
extern phys_assert_info pai_create_sap_active_pair; // (0, 1, true)
extern phys_assert_info pai_create_sap_gjk_ci; // (0, 1, true)

inline void broad_phase_base_list::add(broad_phase_base * bpb)
{
	node * cur = *m_list_cur;
	if (!cur)
	{
		cur = (node *)G_BPM->g_collision_memory_buffer.mt_allocate((int)sizeof(node), (int)PHYS_ALIGNOF(node), 0, "broad phase collision out of memory.");
		*m_list_cur = cur;
		cur->m_next = NULL;
	}
	cur->m_bpb = bpb;
	m_list_cur = &cur->m_next;
}

inline void broad_phase_environement_query_results::add(broad_phase_base * bpb)
{
	PHYS_ASSERT(bpb->is_bpi_env());
	broad_phase_info * const bpi_env = bpb->get_bpi_env();
	m_list_bpi_env.add(bpi_env);
	++m_list_bpi_env_count;
	m_env_collision_flags |= bpi_env->m_env_collision_flags;
}

// ---------------------------------------------------------------------------------------------
// source/phys_broad_phase.cpp
// ---------------------------------------------------------------------------------------------

void comp_trace_volume(const phys_vec3 & aabb1_min, const phys_vec3 & aabb1_max, const phys_vec3 & aabb2_min, const phys_vec3 & aabb2_max,
	phys_vec3 * p1, phys_vec3 * p2, phys_vec3 * half_dims);

void init_bpeqi(broad_phase_environment_query_input * bpeqi, broad_phase_base * bpb);
void check_terrain_query_params(broad_phase_base * bpb);
void broad_phase_reset_buffer();
void merge_sort_bpb(broad_phase_base ** list, int list_count);
const int compare_bpb(broad_phase_base * bpb1, broad_phase_base * bpb2);
void broad_phase_process_object_environment_collision(bpi_environment_collision_info & eci);
void broad_phase_process_object_environment_collision();
void process_cluster_environment_collision_prolog(broad_phase_base * bpb, broad_phase_base * info);
void process_cluster_environment_collision(broad_phase_base * bpb, const broad_phase_environement_query_results & bpeqr);
void collide_bpi_environment(broad_phase_info * bpi, const broad_phase_environement_query_results & bpeqr);
void collide_bpg_environment(broad_phase_group * bpg, const broad_phase_environement_query_results & bpeqr);
void do_initial_tunnel_test(broad_phase_group * bpg, const broad_phase_environement_query_results & bpeqr);
const bool bpi_do_gjk_intersect(broad_phase_info * p1, broad_phase_info * p2, float hit_time);

// Job functions (tlJobFunc, tl_system.h), dispatched through phys_task_manager_process.
void bp_env_jq_batch_function1(void * input, int job_index); // input = broad_phase_prolog_task_input
void bp_env_jq_batch_function2(void * input, int job_index); // input unused

void aasap_list_add(broad_phase_base * bpb);
void aasap_list_remove(broad_phase_base * bpb);

broad_phase_info * create_broad_phase_info();
void destroy_broad_phase_info(broad_phase_info * bpi);
void destroy_broad_phase_info_list(broad_phase_info * list_bpi);
broad_phase_collision_pair * create_broad_phase_collision_pair();
void destroy_broad_phase_collision_pair(broad_phase_collision_pair * bpcp);
void destroy_broad_phase_collision_pair_list(broad_phase_collision_pair * list_bpcp);
broad_phase_group * create_broad_phase_group();
void destroy_broad_phase_group(broad_phase_group * bpg);

void add_collision_pair(broad_phase_info * bpi1, broad_phase_info * bpi2, float hit_time, phys_gjk_cache_info * gjk_ci);
void add_collision_pair_mutex(broad_phase_info * bpi1, broad_phase_info * bpi2, float hit_time, phys_gjk_cache_info * gjk_ci);
void broad_phase_process_collision_pairs();

void broad_phase_process();
void broad_phase_system_init(const broad_phase_memory_info & bpmi, bool (*should_collide_callback)(const broad_phase_base *, const broad_phase_base *));
void broad_phase_system_shutdown();

// Swept-volume test of two bpbs (relative translation). Groups are passed through the
// <broad_phase_info, broad_phase_info> form by cast.
template <class T1, class T2>
inline bool phys_are_potentially_colliding_whace(T1 * p1, T2 * p2, float * hit_time)
{
	return phys_are_potentially_colliding(p1->m_trace_aabb_min_whace, p1->m_trace_aabb_max_whace,
		p1->m_trace_translation - p2->m_trace_translation,
		p2->m_trace_aabb_min_whace, p2->m_trace_aabb_max_whace, hit_time);
}

#include "tl/physics/include/collision/phys_broad_phase_inline.h"
