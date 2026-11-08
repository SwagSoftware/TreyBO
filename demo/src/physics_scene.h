#pragma once

// The TreyBO side of the demo: library start-up (README order), the host world (ground + optional walls returned
// through broad_phase_terrain_query_callback, phys_environment.h), Treyarch-logo rigid bodies with convex-hull
// gjk_brush_t geometry, mouse dragging through a user rigid body + point constraint, and statistics.
// Units follow BO1: Z up, inches, gravity 800 units/s^2.

#include "obj_loader.h"

#include <cstdint>
#include <string>
#include <vector>

class rigid_body;
class user_rigid_body;
class rigid_body_constraint_point;
class broad_phase_info;
class broad_phase_group;
struct gjk_base_t;
struct phys_brush_desc;

enum
{
    DEMO_MAX_BODIES = 1000,       // see docs/ledger/_demo.md: x64 transient-block limits
    DEMO_CONTACT_PAIR_GUARD = 3600, // spawning pauses above this many contact pairs (4096-pointer sort list)
};

enum demo_shape_e
{
    DEMO_SHAPE_HULL = 0,     // one convex hull of the whole logo (a disc)
    DEMO_SHAPE_COMPOUND = 1, // a broad_phase_group of convex ring segments
    DEMO_SHAPE_COUNT
};

// One convex piece: its collision vertices (body frame), the brush descriptor gjk_brush_t reads, the geom,
// wireframe edges and outward planes (for mouse picking).
struct demo_hull
{
    std::vector<float> verts; // xyz triplets, referenced by desc.verts
    std::vector<uint16_t> edges;
    std::vector<float> planes; // nx, ny, nz, d (dot(n, p) <= d inside)
    phys_brush_desc *desc = nullptr;
    gjk_base_t *geom = nullptr;
    float mins[3], maxs[3];
};

struct demo_shape
{
    std::vector<demo_hull *> hulls;
};

struct demo_body
{
    rigid_body *rb = nullptr;
    broad_phase_info *bpi = nullptr;   // single-geom bodies
    broad_phase_group *bpg = nullptr;  // compound bodies
    int shape = DEMO_SHAPE_HULL;
    float tint[3] = {1, 1, 1};
    float lowest_z = 0.0f; // lowest collision vertex in the world (update_stats)
};

struct demo_stats
{
    int bodies = 0;
    int stable = 0;       // rigid_body::FLAG_STABLE
    int group_stable = 0; // rigid_body::FLAG_GROUP_STABLE (whole island at rest)
    int contact_pairs = 0;
    int contact_points = 0;
    int nan_bodies = 0;
    int below_ground = 0;
    float min_z = 0, max_z = 0, avg_z = 0;
    float min_vertex_z = 0; // lowest collision-hull vertex of any body (floor penetration when < 0)
    float max_speed = 0;
    float physics_ms = 0;      // last frame's total phys_frame_advance time
    float physics_ms_avg = 0;  // smoothed
    int steps_last_frame = 0;
    long long total_steps = 0;
    int asserts = 0;
    int warnings = 0;
    int bpi_env_count = 0;
    int bpb_count = 0;
};

struct demo_settings
{
    // Spawning
    int spawn_count = 100;
    float spawn_height = 400.0f;
    float spawn_area = 400.0f; // side of the square drop area
    bool rain = false;
    float rain_rate = 20.0f; // bodies per second
    int spawn_shape = DEMO_SHAPE_HULL;
    float spawn_speed = 0.0f;  // random initial speed
    float spawn_spin = 4.0f;   // random initial angular speed (rad/s)

    // Simulation
    bool paused = false;
    bool step_once = false;
    float time_scale = 1.0f;
    float gravity = 800.0f;
    int v_iters = 8;  // phys_sys::set_v_tol
    int vp_iters = 4; // phys_sys::set_vp_tol
    float mass = 10.0f;
    float max_avel = 16.0f;
    float friction = 0.5f;
    float bounce = 0.1f;
    bool walls = true;
    int worker_threads = 0; // tlSystemCallbacks::RunJobs pool size, 0 = serial
    // phys_sys::set_max_delta_t(1 / (60 * substeps)). BO1 uses 1.0 (no split); 2 keeps fast-spinning logos from
    // rotating their rim deeper than the contact skin in one step (fall-through, docs/ledger/_demo.md).
    int substeps = 2;

    // Interaction
    float explode_strength = 900.0f; // units/s at the centre
    float explode_radius = 300.0f;
    bool drag_spring = true;
    float drag_hz = 4.0f;
    float drag_damping = 0.7f;
};

class physics_scene
{
public:
    demo_settings settings;
    demo_stats stats;
    std::vector<demo_body *> bodies;
    demo_shape shapes[DEMO_SHAPE_COUNT];

    // Model space -> body frame (applied to every render vertex as well, so meshes and hulls agree).
    float model_scale = 12.0f;
    float com[3] = {0, 0, 0};  // centre of mass in scaled Z-up model space
    float axes[3][3];          // principal axes (rows) in scaled Z-up model space
    float inertia_unit[3];     // principal moments at density 1
    float volume = 0.0f;
    float bound_radius = 36.0f;
    bool mass_from_mesh = false;
    int hull_phantom_verts = 0; // seed vertices compute_convex_hull left in its output (see _demo.md)
    float arena_half = 480.0f;
    float ground_half = 4000.0f;

    // World pieces (ground + 4 walls): AABBs in world space.
    struct env_piece
    {
        float mn[3], mx[3];
        gjk_base_t *geom = nullptr;
        bool is_wall = false;
    };
    std::vector<env_piece> env_pieces;

    bool init(const obj_mesh &lowq, std::string *error);
    void shutdown();

    void model_to_body(const float in_pos[3], float out_pos[3]) const;
    void model_normal_to_body(const float in_nrm[3], float out_nrm[3]) const;

    // Spawning / scene control
    demo_body *spawn_body(const float pos[3], const float rot[3][3], const float vel[3], const float avel[3], int shape);
    int drop(int count);
    void clear_all();
    void reset();
    void explode();
    void apply_settings(); // pushes gravity, iterations, surface table, threads into the library

    // One render frame: the 60 Hz accumulator, rain, drag target, stats.
    void frame(float real_dt);
    void step_fixed(); // one phys_frame_advance(1/60)

    // Mouse interaction (world-space ray). Returns the closest body hit.
    demo_body *pick(const float origin[3], const float dir[3], float *t_out, float hit_out[3]) const;
    void drag_begin(demo_body *b, const float hit[3], const float drag_plane_normal[3]);
    void drag_move(const float origin[3], const float dir[3]);
    void drag_end();
    demo_body *dragged() const { return m_drag_body; }
    void drag_target(float out[3]) const;

    void update_stats();
    bool can_spawn() const;

private:
    bool build_shapes(const obj_mesh &lowq, std::string *error);
    demo_hull *build_hull(const std::vector<float> &points, int max_verts, std::string *error);
    void destroy_body(demo_body *b);
    void set_worker_threads(int n);

    float m_accum = 0.0f;
    float m_rain_accum = 0.0f;
    uint32_t m_rng = 0x1234567u;
    float frand(); // [0, 1)

    demo_body *m_drag_body = nullptr;
    user_rigid_body *m_drag_urb = nullptr;
    rigid_body_constraint_point *m_drag_rbc = nullptr;
    float m_drag_local[3] = {0, 0, 0};
    float m_drag_plane_n[3] = {0, 0, 1};
    float m_drag_plane_d = 0.0f;
    float m_drag_target[3] = {0, 0, 0};
    void *m_drag_dictator = nullptr; // phys_mat44, 16-aligned storage

    void *m_memory = nullptr;
    int m_threads_active = 0;
    bool m_initialised = false;
};

// Log sink shared by the library callbacks and the demo (stdout, optional file, ImGui ring buffer).
void demo_log(const char *fmt, ...);
void demo_log_set_file(const char *path);
const std::vector<std::string> &demo_log_lines();
