#include "physics_scene.h"

#include "tl/base/include/tl_system.h"
#include "tl/physics/include/phys_mem.h"
#include "tl/physics/include/phys_util.h"
#include "tl/physics/include/physics_system.h"
#include "tl/physics/include/rigid_body.h"
#include "tl/physics/include/collision/phys_environment.h"
#include "tl/physics/include/collision/phys_gjk_geom.h"
#include "tl/physics/include/collision/phys_convex_hull.h"
#include "tl/physics/include/rbc_defs/rbc_def_contact.h"
#include "tl/physics/include/rbc_defs/rbc_def_generic.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <new>
#include <thread>

// ---------------------------------------------------------------------------------------------
// Log
// ---------------------------------------------------------------------------------------------

static std::mutex g_log_mutex;
static std::vector<std::string> g_log_lines;
static FILE *g_log_file = nullptr;

void demo_log_set_file(const char *path)
{
    std::lock_guard<std::mutex> lock(g_log_mutex);
    if (g_log_file)
        fclose(g_log_file);
    g_log_file = path ? fopen(path, "w") : nullptr;
}

void demo_log(const char *fmt, ...)
{
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    buf[sizeof(buf) - 1] = 0;
    size_t n = strlen(buf);
    while (n && (buf[n - 1] == '\n' || buf[n - 1] == '\r'))
        buf[--n] = 0;
    std::lock_guard<std::mutex> lock(g_log_mutex);
    fprintf(stdout, "%s\n", buf);
    fflush(stdout);
    if (g_log_file)
    {
        fprintf(g_log_file, "%s\n", buf);
        fflush(g_log_file);
    }
    g_log_lines.push_back(buf);
    if (g_log_lines.size() > 200)
        g_log_lines.erase(g_log_lines.begin(), g_log_lines.begin() + 50);
}

const std::vector<std::string> &demo_log_lines() { return g_log_lines; }

// ---------------------------------------------------------------------------------------------
// tl host callbacks
// ---------------------------------------------------------------------------------------------

static std::atomic<int> g_assert_count{0};
static std::atomic<int> g_warning_count{0};

static void *demo_aligned_alloc(size_t size, size_t align)
{
    if (align < 16)
        align = 16;
    if (!size)
        size = 1;
#if defined(_WIN32)
    return _aligned_malloc(size, align);
#else
    void *p = nullptr;
    if (posix_memalign(&p, align, (size + align - 1) & ~(align - 1)))
        return nullptr;
    return p;
#endif
}

static void demo_aligned_free(void *p)
{
#if defined(_WIN32)
    _aligned_free(p);
#else
    free(p);
#endif
}

static void cb_critical(const char *msg)
{
    // Asserts and fatals both land here. Keep running (the demo is a test harness), but rate-limit the spam.
    const int n = ++g_assert_count;
    if (n <= 50 || (n % 1000) == 0)
        demo_log("[TreyBO CRITICAL #%d] %s", n, msg);
}
static void cb_warning(const char *msg)
{
    const int n = ++g_warning_count;
    if (n <= 50 || (n % 1000) == 0)
        demo_log("[TreyBO warning #%d] %s", n, msg);
}
static void cb_print(const char *msg) { demo_log("[TreyBO] %s", msg); }
static void *cb_alloc(uint size, uint align, uint) { return demo_aligned_alloc(size, align); }
static void *cb_realloc(void *p, uint size, uint align, uint)
{
#if defined(_WIN32)
    return _aligned_realloc(p, size ? size : 1, align < 16 ? 16 : align);
#else
    // The library does not realloc on the paths the demo uses; a portable fallback without the old size.
    (void)p;
    return demo_aligned_alloc(size, align);
#endif
}
static void cb_free(void *p) { demo_aligned_free(p); }

// tlSystemCallbacks::RunJobs: a small persistent pool. func(user_data, i) runs once per i on any thread; the
// caller helps and returns when every job has finished.
class job_pool
{
public:
    void start(int workers)
    {
        stop();
        m_quit = false;
        for (int i = 0; i < workers; ++i)
            m_threads.emplace_back([this] { worker(); });
    }
    void stop()
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_quit = true;
        }
        m_cv.notify_all();
        for (std::thread &t : m_threads)
            t.join();
        m_threads.clear();
    }
    void run(tlJobFunc func, void *user_data, int job_count)
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_func = func;
            m_user_data = user_data;
            m_count = job_count;
            m_next.store(0);
            m_done.store(0);
            ++m_generation;
        }
        m_cv.notify_all();
        work(func, user_data, job_count);
        std::unique_lock<std::mutex> lock(m_mutex);
        m_done_cv.wait(lock, [&] { return m_done.load() == job_count && m_busy == 0; });
    }

private:
    void work(tlJobFunc func, void *user_data, int count)
    {
        for (int i = m_next.fetch_add(1); i < count; i = m_next.fetch_add(1))
        {
            func(user_data, i);
            if (m_done.fetch_add(1) + 1 == count)
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_done_cv.notify_all();
            }
        }
    }
    void worker()
    {
        uint64_t seen = 0;
        for (;;)
        {
            tlJobFunc func;
            void *user_data;
            int count;
            {
                std::unique_lock<std::mutex> lock(m_mutex);
                m_cv.wait(lock, [&] { return m_quit || m_generation != seen; });
                if (m_quit)
                    return;
                seen = m_generation;
                func = m_func;
                user_data = m_user_data;
                count = m_count;
                ++m_busy;
            }
            work(func, user_data, count);
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                --m_busy;
            }
            m_done_cv.notify_all();
        }
    }

    std::vector<std::thread> m_threads;
    std::mutex m_mutex;
    std::condition_variable m_cv, m_done_cv;
    tlJobFunc m_func = nullptr;
    void *m_user_data = nullptr;
    int m_count = 0;
    std::atomic<int> m_next{0}, m_done{0};
    int m_busy = 0;
    uint64_t m_generation = 0;
    bool m_quit = false;
};

static job_pool g_job_pool;
static void cb_run_jobs(tlJobFunc func, void *user_data, int job_count) { g_job_pool.run(func, user_data, job_count); }

static tlSystemCallbacks make_callbacks(bool threaded)
{
    tlSystemCallbacks cb = {};
    cb.CriticalError = cb_critical;
    cb.Warning = cb_warning;
    cb.DebugPrint = cb_print;
    cb.MemAlloc = cb_alloc;
    cb.MemRealloc = cb_realloc;
    cb.MemFree = cb_free;
    cb.RunJobs = threaded ? cb_run_jobs : nullptr;
    return cb;
}

// ---------------------------------------------------------------------------------------------
// Persistent gjk geom allocator (PMM pools, ids from the broad phase's database, as BO1's create() path)
// ---------------------------------------------------------------------------------------------

class demo_geom_allocator : public gjk_geom_allocator
{
public:
    void *allocate(const int, const int, const bool) override { return nullptr; } // never a query allocator
    const phys_gjk_geom_id get_unique_id() override { return g_gjk_unique_id_database.get_unique_id(); }
};
static demo_geom_allocator g_geom_allocator;

enum
{
    DEMO_ENV_FLAGS = 0x1C7,    // BO1 create_broad_phase_info passes 0x1C7 for physics objects
    DEMO_SURFACE_TYPE = 31,    // BO1's default surface type
    DEMO_NUM_SURFACE_TYPES = 35,
};

static physics_scene *g_scene = nullptr;
static phys_mat44 g_identity_mat; // cg_to_rb of every geom (hulls are built in the body frame)

// ---------------------------------------------------------------------------------------------
// The host world: the phys_environment.h contract
// ---------------------------------------------------------------------------------------------

class demo_terrain_query : public broad_phase_terrain_query_callback
{
public:
    void query(const broad_phase_environment_query_input &bpeqi, broad_phase_environement_query_results *bpeqr) override
    {
        if (!g_scene)
            return;
        // The cluster's swept volume over the step.
        const phys_vec3 end_min = bpeqi.trace_aabb_min_wace + bpeqi.trace_translation;
        const phys_vec3 end_max = bpeqi.trace_aabb_max_wace + bpeqi.trace_translation;
        const phys_vec3 qmin = phys_min(bpeqi.trace_aabb_min_wace, end_min);
        const phys_vec3 qmax = phys_max(bpeqi.trace_aabb_max_wace, end_max);

        for (size_t i = 0; i < g_scene->env_pieces.size(); ++i)
        {
            const physics_scene::env_piece &piece = g_scene->env_pieces[i];
            if (piece.is_wall && !g_scene->settings.walls)
                continue;
            if (qmax.x < piece.mn[0] || qmin.x > piece.mx[0] || qmax.y < piece.mn[1] || qmin.y > piece.mx[1] ||
                qmax.z < piece.mn[2] || qmin.z > piece.mx[2])
                continue;
            add_piece(piece, (unsigned int)i + 1, bpeqr);
        }
    }

private:
    // Retail gjk_physics_collision_visitor::query_create_prolog / _epilog, reduced to static geoms.
    static void add_piece(const physics_scene::env_piece &piece, unsigned int id, broad_phase_environement_query_results *bpeqr)
    {
        broad_phase_environment_info *const bpei = G_BPM->g_bpei_database.get_bpei_mt(bpei_database_id(0x4B504859u /* 'KPHY' */, id));
        if (!bpei->m_data)
        {
            bpei->m_mutex.Lock();
            if (!bpei->m_data)
            {
                environment_rigid_body *const env = phys_sys::get_environment_rigid_body();
                broad_phase_info *const bpi_env = allocate_bpi_env();
                bpi_env->set(env, &env->get_mat(), &g_identity_mat, &g_identity_mat, piece.geom, bpei->m_gjk_geom_id, false,
                             DEMO_SURFACE_TYPE, (void *)&piece, DEMO_ENV_FLAGS);
                bpi_env->set_bpi_env(nullptr);
                bpi_env->collision_prolog();
                bpei->m_data = bpi_env;
            }
            bpei->m_mutex.Unlock();
        }
        bpeqr->add((broad_phase_base *)bpei->m_data);
    }
};
static demo_terrain_query g_terrain_query;

// ---------------------------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------------------------

float physics_scene::frand()
{
    // xorshift32
    m_rng ^= m_rng << 13;
    m_rng ^= m_rng >> 17;
    m_rng ^= m_rng << 5;
    return (float)(m_rng >> 8) * (1.0f / 16777216.0f);
}

static bool finite3(const phys_vec3 &v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

// Jacobi eigen decomposition of a symmetric 3x3: a = V diag(d) V^T, eigenvectors in V's columns.
static void jacobi3(double a[3][3], double d[3], double v[3][3])
{
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            v[i][j] = i == j ? 1.0 : 0.0;
    for (int sweep = 0; sweep < 50; ++sweep)
    {
        const double off = fabs(a[0][1]) + fabs(a[0][2]) + fabs(a[1][2]);
        if (off < 1e-14 * (fabs(a[0][0]) + fabs(a[1][1]) + fabs(a[2][2]) + 1e-30))
            break;
        for (int p = 0; p < 2; ++p)
            for (int q = p + 1; q < 3; ++q)
            {
                if (fabs(a[p][q]) < 1e-300)
                    continue;
                const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                const double t = (theta >= 0 ? 1.0 : -1.0) / (fabs(theta) + sqrt(theta * theta + 1.0));
                const double c = 1.0 / sqrt(t * t + 1.0), s = t * c;
                for (int k = 0; k < 3; ++k)
                {
                    const double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < 3; ++k)
                {
                    const double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < 3; ++k)
                {
                    const double vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
    }
    for (int i = 0; i < 3; ++i)
        d[i] = a[i][i];
}

// OBJ is Y up; BO1 is Z up: (x, y, z) -> (x, -z, y).
static void swap_to_z_up(const float in[3], float out[3])
{
    out[0] = in[0];
    out[1] = -in[2];
    out[2] = in[1];
}

void physics_scene::model_to_body(const float in_pos[3], float out_pos[3]) const
{
    float p[3];
    swap_to_z_up(in_pos, p);
    for (int i = 0; i < 3; ++i)
        p[i] = p[i] * model_scale - com[i];
    for (int r = 0; r < 3; ++r)
        out_pos[r] = axes[r][0] * p[0] + axes[r][1] * p[1] + axes[r][2] * p[2];
}

void physics_scene::model_normal_to_body(const float in_nrm[3], float out_nrm[3]) const
{
    float n[3];
    swap_to_z_up(in_nrm, n);
    for (int r = 0; r < 3; ++r)
        out_nrm[r] = axes[r][0] * n[0] + axes[r][1] * n[1] + axes[r][2] * n[2];
}

// ---------------------------------------------------------------------------------------------
// Shapes: mass properties from the closed lowq mesh, hulls from phys_convex_hull
// ---------------------------------------------------------------------------------------------

static phys_convex_hull *g_hull_builder = nullptr; // ~0.3 MB on x64: heap, as the header asks

demo_hull *physics_scene::build_hull(const std::vector<float> &points, int max_verts, std::string *error)
{
    const int n = (int)(points.size() / 3);
    if (n < 4)
    {
        if (error)
            *error = "too few points for a hull";
        return nullptr;
    }

    // compute_convex_hull seeds with an O(n^3) largest-triangle search, so only extreme points go in:
    // the support points of 256 well-spread directions.
    std::vector<phys_vec3> cand;
    std::vector<int> taken(n, 0);
    const int DIRS = 256;
    for (int k = 0; k < DIRS; ++k)
    {
        const float zz = 1.0f - 2.0f * (k + 0.5f) / DIRS;
        const float r = sqrtf(fmaxf(0.0f, 1.0f - zz * zz));
        const float phi = (float)k * 2.39996323f;
        const float d[3] = {r * cosf(phi), r * sinf(phi), zz};
        int best = 0;
        float best_d = -1e30f;
        for (int i = 0; i < n; ++i)
        {
            const float dd = points[i * 3] * d[0] + points[i * 3 + 1] * d[1] + points[i * 3 + 2] * d[2];
            if (dd > best_d)
            {
                best_d = dd;
                best = i;
            }
        }
        if (!taken[best])
        {
            taken[best] = 1;
            cand.push_back(phys_vec3(points[best * 3], points[best * 3 + 1], points[best * 3 + 2]));
        }
    }
    if (cand.size() < 4)
    {
        if (error)
            *error = "degenerate point set";
        return nullptr;
    }

    if (!g_hull_builder)
        g_hull_builder = new phys_convex_hull;
    phys_convex_hull &hull = *g_hull_builder;
    // m_convex_hull_vert_list / m_convex_hull_triangle_list hold 64 entries; a closed hull of V verts has 2V - 4
    // triangles, so 32 verts is the most that could fit (the retail default of 128 overflows, see _demo.md).
    phys_convex_hull_build(hull, cand.data(), (int)cand.size(), max_verts, 0.0f);
    const int vcount = hull.m_convex_hull_vert_list.get_count();
    const int tcount = hull.m_convex_hull_triangle_list.get_count();
    if (vcount < 4 || tcount < 4)
    {
        if (error)
            *error = "phys_convex_hull returned a degenerate hull";
        return nullptr;
    }
    std::vector<phys_vec3> hv(vcount);
    std::vector<ushort> hi((size_t)tcount * 3);
    phys_convex_hull_extract(hull, hv.data(), hi.data());

    // Drop vertices that are not input points (calculate_initial_triangle_vertices adds a seed 1 unit off the
    // first triangle's plane to m_vertex_buffer; it can end up on the hull).
    std::vector<int> remap(vcount, -1);
    demo_hull *h = new demo_hull;
    for (int i = 0; i < vcount; ++i)
    {
        bool real = false;
        for (const phys_vec3 &c : cand)
            if (c.x == hv[i].x && c.y == hv[i].y && c.z == hv[i].z)
            {
                real = true;
                break;
            }
        if (!real)
        {
            ++hull_phantom_verts;
            continue;
        }
        remap[i] = (int)(h->verts.size() / 3);
        h->verts.push_back(hv[i].x);
        h->verts.push_back(hv[i].y);
        h->verts.push_back(hv[i].z);
    }
    const int kept = (int)(h->verts.size() / 3);
    if (kept < 4)
    {
        delete h;
        if (error)
            *error = "hull lost too many vertices";
        return nullptr;
    }

    float cen[3] = {0, 0, 0};
    for (int i = 0; i < kept; ++i)
        for (int c = 0; c < 3; ++c)
            cen[c] += h->verts[i * 3 + c] / kept;
    for (int c = 0; c < 3; ++c)
    {
        h->mins[c] = 1e30f;
        h->maxs[c] = -1e30f;
    }
    for (int i = 0; i < kept; ++i)
        for (int c = 0; c < 3; ++c)
        {
            h->mins[c] = fminf(h->mins[c], h->verts[i * 3 + c]);
            h->maxs[c] = fmaxf(h->maxs[c], h->verts[i * 3 + c]);
        }

    // Edges and outward planes from the triangles whose corners all survived.
    for (int t = 0; t < tcount; ++t)
    {
        const int a = remap[hi[t * 3]], b = remap[hi[t * 3 + 1]], c = remap[hi[t * 3 + 2]];
        if (a < 0 || b < 0 || c < 0)
            continue;
        const int tri[3] = {a, b, c};
        for (int e = 0; e < 3; ++e)
        {
            int i0 = tri[e], i1 = tri[(e + 1) % 3];
            if (i0 > i1)
                std::swap(i0, i1);
            bool dup = false;
            for (size_t k = 0; k + 1 < h->edges.size(); k += 2)
                if (h->edges[k] == i0 && h->edges[k + 1] == i1)
                {
                    dup = true;
                    break;
                }
            if (!dup)
            {
                h->edges.push_back((uint16_t)i0);
                h->edges.push_back((uint16_t)i1);
            }
        }
        const float *pa = &h->verts[a * 3], *pb = &h->verts[b * 3], *pc = &h->verts[c * 3];
        const float e1[3] = {pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]};
        const float e2[3] = {pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2]};
        float nn[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
        const float len = sqrtf(nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2]);
        if (len < 1e-12f)
            continue;
        for (float &x : nn)
            x /= len;
        float d = nn[0] * pa[0] + nn[1] * pa[1] + nn[2] * pa[2];
        if (nn[0] * cen[0] + nn[1] * cen[1] + nn[2] * cen[2] > d)
        {
            for (float &x : nn)
                x = -x;
            d = -d;
        }
        h->planes.insert(h->planes.end(), {nn[0], nn[1], nn[2], d});
    }

    // The library geom: a convex brush over the hull vertices (support / feature / AABB read only the verts).
    h->desc = new phys_brush_desc;
    memset(h->desc, 0, sizeof(*h->desc));
    memcpy(h->desc->mins, h->mins, sizeof(h->mins));
    memcpy(h->desc->maxs, h->maxs, sizeof(h->maxs));
    h->desc->contents = 1;
    h->desc->numsides = (unsigned int)(h->planes.size() / 4); // non-zero: a real brush, not the AABB fallback
    h->desc->numverts = (unsigned int)kept;
    h->desc->verts = (const float(*)[3])h->verts.data();
    h->desc->user_data = h;
    h->geom = create_brush_gjk_geom(h->desc, DEMO_SURFACE_TYPE, &g_geom_allocator);
    if (!h->geom)
    {
        delete h->desc;
        delete h;
        if (error)
            *error = "create_brush_gjk_geom failed";
        return nullptr;
    }
    return h;
}

bool physics_scene::build_shapes(const obj_mesh &lowq, std::string *error)
{
    const int n = (int)(lowq.positions.size() / 3);

    // Scaled, Z-up positions.
    std::vector<double> p((size_t)n * 3);
    for (int i = 0; i < n; ++i)
    {
        float s[3];
        swap_to_z_up(&lowq.positions[(size_t)i * 3], s);
        for (int c = 0; c < 3; ++c)
            p[(size_t)i * 3 + c] = (double)s[c] * model_scale;
    }
    double mn[3] = {1e30, 1e30, 1e30}, mx[3] = {-1e30, -1e30, -1e30};
    for (int i = 0; i < n; ++i)
        for (int c = 0; c < 3; ++c)
        {
            mn[c] = std::min(mn[c], p[(size_t)i * 3 + c]);
            mx[c] = std::max(mx[c], p[(size_t)i * 3 + c]);
        }

    // Closed-mesh volume, centroid and second moments (signed tetrahedra from the origin).
    double vol = 0.0, cx = 0.0, cy = 0.0, cz = 0.0;
    double C[3][3] = {};
    const std::vector<uint32_t> &tri = lowq.position_indices;
    for (size_t t = 0; t + 2 < tri.size(); t += 3)
    {
        const double *a = &p[(size_t)tri[t] * 3], *b = &p[(size_t)tri[t + 1] * 3], *c = &p[(size_t)tri[t + 2] * 3];
        const double det = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]);
        vol += det / 6.0;
        cx += det * (a[0] + b[0] + c[0]) / 24.0;
        cy += det * (a[1] + b[1] + c[1]) / 24.0;
        cz += det * (a[2] + b[2] + c[2]) / 24.0;
        const double s[3] = {a[0] + b[0] + c[0], a[1] + b[1] + c[1], a[2] + b[2] + c[2]};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                C[i][j] += det / 120.0 * (a[i] * a[j] + b[i] * b[j] + c[i] * c[j] + s[i] * s[j]);
    }
    if (vol < 0.0)
    {
        vol = -vol;
        cx = -cx;
        cy = -cy;
        cz = -cz;
        for (auto &row : C)
            for (double &x : row)
                x = -x;
    }
    const double box_vol = (mx[0] - mn[0]) * (mx[1] - mn[1]) * (mx[2] - mn[2]);
    double inertia_tensor[3][3];
    double centre[3];
    mass_from_mesh = vol > 0.01 * box_vol;
    if (mass_from_mesh)
    {
        centre[0] = cx / vol;
        centre[1] = cy / vol;
        centre[2] = cz / vol;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                C[i][j] -= vol * centre[i] * centre[j];
        const double tr = C[0][0] + C[1][1] + C[2][2];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                inertia_tensor[i][j] = (i == j ? tr : 0.0) - C[i][j];
        volume = (float)vol;
    }
    else
    {
        // Not a closed mesh: BO1's box approximation (nuge::calc_box_inertia) about the AABB centre.
        for (int c = 0; c < 3; ++c)
            centre[c] = 0.5 * (mn[c] + mx[c]);
        TL_ALIGN(16) phys_vec3 unit;
        float bvol;
        nuge::calc_box_inertia(phys_vec3((float)(mx[0] - mn[0]), (float)(mx[1] - mn[1]), (float)(mx[2] - mn[2])), &unit, &bvol);
        memset(inertia_tensor, 0, sizeof(inertia_tensor));
        inertia_tensor[0][0] = unit.x;
        inertia_tensor[1][1] = unit.y;
        inertia_tensor[2][2] = unit.z;
        volume = bvol;
    }

    // Principal axes, sorted ascending so the largest moment (the logo's thin axis) is body Z.
    double d[3], v[3][3];
    jacobi3(inertia_tensor, d, v);
    int order[3] = {0, 1, 2};
    std::sort(order, order + 3, [&](int a, int b) { return d[a] < d[b]; });
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
            axes[r][c] = (float)v[c][order[r]];
        inertia_unit[r] = (float)d[order[r]];
    }
    // Right-handed: z = x cross y.
    const float zx = axes[0][1] * axes[1][2] - axes[0][2] * axes[1][1];
    const float zy = axes[0][2] * axes[1][0] - axes[0][0] * axes[1][2];
    const float zz = axes[0][0] * axes[1][1] - axes[0][1] * axes[1][0];
    if (zx * axes[2][0] + zy * axes[2][1] + zz * axes[2][2] < 0.0f)
        for (float &x : axes[2])
            x = -x;
    for (int c = 0; c < 3; ++c)
        com[c] = (float)centre[c];

    demo_log("logo: %d verts, %zu tris, volume %.1f (%s), centre (%.2f %.2f %.2f), principal moments %.4g %.4g %.4g", n,
             tri.size() / 3, volume, mass_from_mesh ? "closed mesh" : "box fallback", com[0], com[1], com[2], inertia_unit[0],
             inertia_unit[1], inertia_unit[2]);

    // Body-frame points.
    std::vector<float> body_pts((size_t)n * 3);
    bound_radius = 0.0f;
    for (int i = 0; i < n; ++i)
    {
        model_to_body(&lowq.positions[(size_t)i * 3], &body_pts[(size_t)i * 3]);
        const float *q = &body_pts[(size_t)i * 3];
        bound_radius = fmaxf(bound_radius, sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]));
    }

    // Shape 0: one hull.
    demo_hull *whole = build_hull(body_pts, 32, error);
    if (!whole)
        return false;
    shapes[DEMO_SHAPE_HULL].hulls.push_back(whole);
    demo_log("hull: %zu verts, %zu faces, extents (%.1f %.1f %.1f)", whole->verts.size() / 3, whole->planes.size() / 4,
             whole->maxs[0] - whole->mins[0], whole->maxs[1] - whole->mins[1], whole->maxs[2] - whole->mins[2]);

    // Shape 1: ring segments around body Z (the thin axis), slightly overlapping.
    const int SEGMENTS = 8;
    const char *seg_env = getenv("TREYBO_DEMO_SEGMENT_HULL_VERTS"); // diagnosis knob for the hull overflow (_demo.md)
    // 24, not 32: a segment hull with max 32 overflowed the 64-entry triangle list (it emits more than 2V - 4 faces).
    const int SEGMENT_HULL_VERTS = seg_env ? atoi(seg_env) : 24;
    const float margin = 0.05f;
    for (int k = 0; k < SEGMENTS; ++k)
    {
        const float a0 = -3.14159265f + 6.2831853f * k / SEGMENTS - margin;
        const float a1 = -3.14159265f + 6.2831853f * (k + 1) / SEGMENTS + margin;
        std::vector<float> seg;
        for (int i = 0; i < n; ++i)
        {
            const float *q = &body_pts[(size_t)i * 3];
            float a = atan2f(q[1], q[0]);
            bool in = (a >= a0 && a <= a1);
            if (!in && a0 < -3.14159265f)
                in = a >= a0 + 6.2831853f;
            if (!in && a1 > 3.14159265f)
                in = a <= a1 - 6.2831853f;
            if (in)
                seg.insert(seg.end(), q, q + 3);
        }
        std::string seg_err;
        demo_log("compound segment %d: %zu points, max %d hull verts", k, seg.size() / 3, SEGMENT_HULL_VERTS);
        demo_hull *h = build_hull(seg, SEGMENT_HULL_VERTS, &seg_err);
        if (h)
        {
            demo_log("  -> %zu verts, %zu faces", h->verts.size() / 3, h->planes.size() / 4);
            shapes[DEMO_SHAPE_COMPOUND].hulls.push_back(h);
        }
        else
            demo_log("compound segment %d skipped: %s", k, seg_err.c_str());
    }
    if (shapes[DEMO_SHAPE_COMPOUND].hulls.empty())
        shapes[DEMO_SHAPE_COMPOUND].hulls.push_back(whole);
    demo_log("compound: %zu convex segments; hull phantom vertices dropped: %d", shapes[DEMO_SHAPE_COMPOUND].hulls.size(),
             hull_phantom_verts);
    return true;
}

// ---------------------------------------------------------------------------------------------
// Start-up / shutdown
// ---------------------------------------------------------------------------------------------

bool physics_scene::init(const obj_mesh &lowq, std::string *error)
{
    g_scene = this;
    g_identity_mat = PHYS_IDENTITY_MATRIX;

    // README order.
    const tlSystemCallbacks cb = make_callbacks(false);
    tlSetSystemCallbacks(&cb);

    // BO1 allocates 0x380000 bytes for 32-bit; the demo allows ~1000 bodies, contacts and caches on x64.
    const int mem_size = 96 * 1024 * 1024;
    m_memory = demo_aligned_alloc(mem_size, 64);
    if (!m_memory)
    {
        if (error)
            *error = "out of memory";
        return false;
    }
    phys_memory_manager_init(m_memory, mem_size);
    phys_task_manager_init();
    phys_sys::phys_init();

    broad_phase_memory_info bpmi;
    bpmi.m_max_num_gjk_ci = 16384;
    bpmi.m_max_num_sap_active_pairs = 16384;
    bpmi.m_max_num_surface_types = DEMO_NUM_SURFACE_TYPES;
    broad_phase_system_init(bpmi, nullptr);
    phys_sys::set_collision_callback(broad_phase_default_collision_callback);

    // Host pieces after the README sequence: world query, surface table, BO1 Phys_Init tunables.
    G_BPM->g_broad_phase_terrain_query_callback = &g_terrain_query;
    phys_sys::set_max_delta_t(1.0f);

    // World geometry: the ground (top at z = 0) and four arena walls, persistent geoms queried every pass.
    {
        env_piece ground;
        const float gmn[3] = {-ground_half, -ground_half, -64.0f};
        const float gmx[3] = {ground_half, ground_half, 0.0f};
        memcpy(ground.mn, gmn, sizeof(gmn));
        memcpy(ground.mx, gmx, sizeof(gmx));
        env_pieces.push_back(ground);
        const float A = arena_half, T = 16.0f, H = 160.0f;
        const float walls[4][6] = {
            {-A - T, -A - T, 0, A + T, -A + T, H},
            {-A - T, A - T, 0, A + T, A + T, H},
            {-A - T, -A - T, 0, -A + T, A + T, H},
            {A - T, -A - T, 0, A + T, A + T, H},
        };
        for (const auto &w : walls)
        {
            env_piece p;
            memcpy(p.mn, w, sizeof(float) * 3);
            memcpy(p.mx, w + 3, sizeof(float) * 3);
            p.is_wall = true;
            env_pieces.push_back(p);
        }
        for (env_piece &p : env_pieces)
            p.geom = create_aabb_gjk_geom(p.mn, p.mx, DEMO_SURFACE_TYPE, &g_geom_allocator);
    }

    if (!build_shapes(lowq, error))
        return false;

    m_drag_dictator = demo_aligned_alloc(sizeof(phys_mat44), 16);
    new (m_drag_dictator) phys_mat44(PHYS_IDENTITY_MATRIX);

    m_initialised = true;
    apply_settings();
    return true;
}

void physics_scene::shutdown()
{
    if (!m_initialised)
        return;
    clear_all();
    set_worker_threads(0);
    // The library has no full teardown path the demo needs (process exit releases the arena).
    m_initialised = false;
}

void physics_scene::set_worker_threads(int n)
{
    if (n == m_threads_active)
        return;
    g_job_pool.stop();
    if (n > 0)
        g_job_pool.start(n);
    const tlSystemCallbacks cb = make_callbacks(n > 0);
    tlSetSystemCallbacks(&cb);
    m_threads_active = n;
}

void physics_scene::apply_settings()
{
    phys_sys::set_v_tol(settings.v_iters);
    phys_sys::set_vp_tol(settings.vp_iters);
    // 1 = BO1 (Phys_Init sets 1.0: one collision pass per 1/60 frame). More sub-steps = a collision pass each.
    const int sub = settings.substeps < 1 ? 1 : settings.substeps;
    phys_sys::set_max_delta_t(sub == 1 ? 1.0f : 1.0f / (60.0f * (float)sub) + 1e-6f);

    phys_surface_type_info pst;
    pst.m_friction_coef = settings.friction;
    pst.m_bounce_coef = settings.bounce;
    pst.m_solver_priority = 0;
    pst.m_no_overflow_error = true;
    surface_type_info_database_set(DEMO_SURFACE_TYPE, DEMO_SURFACE_TYPE, pst);

    for (demo_body *b : bodies)
    {
        b->rb->m_gravity_acc_vec = phys_vec3(0.0f, 0.0f, -settings.gravity);
        b->rb->set_max_avel(settings.max_avel);
    }
    set_worker_threads(settings.worker_threads);
}

// ---------------------------------------------------------------------------------------------
// Bodies
// ---------------------------------------------------------------------------------------------

bool physics_scene::can_spawn() const
{
    return (int)bodies.size() < DEMO_MAX_BODIES && phys_sys::get_rbc_contact_alloc_count() < DEMO_CONTACT_PAIR_GUARD;
}

demo_body *physics_scene::spawn_body(const float pos[3], const float rot[3][3], const float vel[3], const float avel[3], int shape)
{
    if (!can_spawn())
        return nullptr;
    rigid_body *const rb = phys_sys::create_rigid_body(true);
    if (!rb)
        return nullptr;

    demo_body *const b = new demo_body;
    b->rb = rb;
    b->shape = shape;
    static const float palette[][3] = {
        {0.95f, 0.45f, 0.08f}, {0.98f, 0.62f, 0.12f}, {0.85f, 0.30f, 0.06f}, {0.20f, 0.20f, 0.22f},
        {0.85f, 0.85f, 0.82f}, {0.62f, 0.65f, 0.70f}, {0.95f, 0.78f, 0.30f}, {0.70f, 0.18f, 0.10f},
    };
    const int pi = (int)(frand() * 8.0f) & 7;
    const float shade = 0.85f + 0.3f * frand();
    for (int c = 0; c < 3; ++c)
        b->tint[c] = fminf(1.0f, palette[pi][c] * shade);

    TL_ALIGN(16) phys_mat44 mat;
    mat.x = phys_vec3(rot[0][0], rot[0][1], rot[0][2]); // rows: body axes in world (phys_mat44 row-vector convention)
    mat.y = phys_vec3(rot[1][0], rot[1][1], rot[1][2]);
    mat.z = phys_vec3(rot[2][0], rot[2][1], rot[2][2]);
    mat.w = phys_vec3(pos[0], pos[1], pos[2]);
    mat.fix_w_column();
    const float m = settings.mass;
    const phys_vec3 inertia(inertia_unit[0] * m / volume, inertia_unit[1] * m / volume, inertia_unit[2] * m / volume);
    rb->set(m, inertia, mat, phys_vec3(vel[0], vel[1], vel[2]), phys_vec3(avel[0], avel[1], avel[2]), 3);
    // BO1 Phys_CreateBodyFromState: gravity and max angular velocity after set().
    rb->m_gravity_acc_vec = phys_vec3(0.0f, 0.0f, -settings.gravity);
    rb->set_max_avel(settings.max_avel);
    rb->m_userdata = b;

    // Collision: the geoms are in the body frame, so cg_to_world is the body matrix itself.
    const demo_shape &s = shapes[shape];
    if (s.hulls.size() == 1)
    {
        broad_phase_info *const bpi = create_broad_phase_info();
        bpi->set(rb, &rb->get_mat(), &rb->get_mat(), &g_identity_mat, s.hulls[0]->geom, g_gjk_unique_id_database.get_unique_id(), false,
                 DEMO_SURFACE_TYPE, b, DEMO_ENV_FLAGS);
        environment_collision_list_add(bpi);
        aasap_list_add(bpi);
        b->bpi = bpi;
    }
    else
    {
        broad_phase_group *const bpg = create_broad_phase_group();
        bpg->set();
        bpg->m_user_data = b;
        for (demo_hull *h : s.hulls)
        {
            broad_phase_info *const bpi = create_broad_phase_info();
            bpi->set(rb, &rb->get_mat(), &rb->get_mat(), &g_identity_mat, h->geom, g_gjk_unique_id_database.get_unique_id(), false,
                     DEMO_SURFACE_TYPE, b, DEMO_ENV_FLAGS);
            bpg->add_bpi(bpi);
        }
        environment_collision_list_add(bpg);
        aasap_list_add(bpg);
        b->bpg = bpg;
    }
    bodies.push_back(b);
    return b;
}

void physics_scene::destroy_body(demo_body *b)
{
    if (b == m_drag_body)
        drag_end();
    if (b->bpi)
        destroy_broad_phase_info(b->bpi); // off the environment list and the sweep and prune, then freed
    if (b->bpg)
    {
        broad_phase_info *const members = b->bpg->m_list_bpi_head;
        destroy_broad_phase_group(b->bpg);
        destroy_broad_phase_info_list(members);
    }
    phys_sys::destroy(b->rb); // its constraints first (contacts, the drag point)
    delete b;
}

// r0..r2 uniform in [0, 1).
static void random_rotation(float r0, float r1, float r2, float out[3][3])
{
    // Uniform random unit quaternion (Shoemake).
    const float u1 = r0, u2 = r1 * 6.2831853f, u3 = r2 * 6.2831853f;
    const float a = sqrtf(1.0f - u1), b = sqrtf(u1);
    const float x = a * sinf(u2), y = a * cosf(u2), z = b * sinf(u3), w = b * cosf(u3);
    // Rows: images of the body axes.
    out[0][0] = 1 - 2 * (y * y + z * z); out[0][1] = 2 * (x * y + z * w);     out[0][2] = 2 * (x * z - y * w);
    out[1][0] = 2 * (x * y - z * w);     out[1][1] = 1 - 2 * (x * x + z * z); out[1][2] = 2 * (y * z + x * w);
    out[2][0] = 2 * (x * z + y * w);     out[2][1] = 2 * (y * z - x * w);     out[2][2] = 1 - 2 * (x * x + y * y);
}

int physics_scene::drop(int count)
{
    // A jittered 3D grid over the drop area, starting at the spawn height and stacking upwards, skipping cells
    // that overlap a live body (bodies never start interpenetrating).
    const float spacing = 2.0f * bound_radius + 6.0f;
    const int side = std::max(1, (int)(settings.spawn_area / spacing));
    const float start = -0.5f * (side - 1) * spacing;
    int made = 0;
    for (int layer = 0; made < count && layer < 400; ++layer)
    {
        for (int iy = 0; iy < side && made < count; ++iy)
            for (int ix = 0; ix < side && made < count; ++ix)
            {
                if (!can_spawn())
                    return made;
                const float jitter = 0.15f * spacing;
                const float pos[3] = {start + ix * spacing + (frand() - 0.5f) * jitter, start + iy * spacing + (frand() - 0.5f) * jitter,
                                      settings.spawn_height + layer * spacing};
                bool blocked = false;
                const float min_d2 = spacing * spacing;
                for (demo_body *o : bodies)
                {
                    const phys_vec3 &w = o->rb->get_mat().GetW();
                    const float dx = w.x - pos[0], dy = w.y - pos[1], dz = w.z - pos[2];
                    if (dx * dx + dy * dy + dz * dz < min_d2)
                    {
                        blocked = true;
                        break;
                    }
                }
                if (blocked)
                    continue;
                float rot[3][3];
                const float q0 = frand(), q1 = frand(), q2 = frand();
                random_rotation(q0, q1, q2, rot);
                const float vel[3] = {(frand() - 0.5f) * settings.spawn_speed, (frand() - 0.5f) * settings.spawn_speed,
                                      -(frand()) * settings.spawn_speed};
                const float avel[3] = {(frand() - 0.5f) * settings.spawn_spin, (frand() - 0.5f) * settings.spawn_spin,
                                       (frand() - 0.5f) * settings.spawn_spin};
                if (spawn_body(pos, rot, vel, avel, settings.spawn_shape))
                    ++made;
            }
    }
    return made;
}

void physics_scene::clear_all()
{
    drag_end();
    for (demo_body *b : bodies)
        destroy_body(b);
    bodies.clear();
}

void physics_scene::reset()
{
    clear_all();
    const demo_settings defaults;
    const int threads = settings.worker_threads;
    settings = defaults;
    settings.worker_threads = threads;
    m_accum = 0.0f;
    m_rain_accum = 0.0f;
    apply_settings();
}

void physics_scene::explode()
{
    if (bodies.empty())
        return;
    // Pile centre: mean position of the bodies, on the ground.
    double cx = 0, cy = 0;
    for (demo_body *b : bodies)
    {
        cx += b->rb->get_mat().GetW().x;
        cy += b->rb->get_mat().GetW().y;
    }
    const phys_vec3 centre((float)(cx / bodies.size()), (float)(cy / bodies.size()), -20.0f);
    const float R = settings.explode_radius;
    for (demo_body *b : bodies)
    {
        rigid_body *const rb = b->rb;
        const phys_vec3 d = rb->get_mat().GetW() - centre;
        const float dist = Abs(d);
        if (dist > R || dist < 1e-3f)
            continue;
        const float falloff = 1.0f - dist / R;
        phys_vec3 dir = d * (1.0f / dist);
        dir.z += 0.6f;
        dir = phys_Unitize(dir);
        // rigid_body::add_force accumulates an impulse for the next step (solver_prolog divides it by delta_t
        // and the integrator multiplies it back): mass * delta_v, applied off-centre for some spin.
        const float dv = settings.explode_strength * falloff;
        const phys_vec3 impulse = dir * (dv / rb->get_inv_mass());
        const phys_vec3 offs((frand() - 0.5f) * 8.0f, (frand() - 0.5f) * 8.0f, (frand() - 0.5f) * 8.0f);
        rb->add_force(impulse, rb->get_mat().GetW() + offs, 1.0f);
    }
}

// ---------------------------------------------------------------------------------------------
// Picking and dragging
// ---------------------------------------------------------------------------------------------

demo_body *physics_scene::pick(const float origin[3], const float dir[3], float *t_out, float hit_out[3]) const
{
    demo_body *best = nullptr;
    float best_t = 1e30f;
    const phys_vec3 o(origin[0], origin[1], origin[2]), d(dir[0], dir[1], dir[2]);
    for (demo_body *b : bodies)
    {
        const phys_mat44 &m = b->rb->get_mat();
        // Cheap reject with the bounding sphere.
        const phys_vec3 oc = o - m.GetW();
        const float bproj = phys_dot(oc, d);
        const float c = AbsSquared(oc) - bound_radius * bound_radius;
        if (c > 0.0f && bproj > 0.0f)
            continue;
        if (bproj * bproj - c < 0.0f)
            continue;
        // Ray in the body frame, clipped by each hull's planes.
        const phys_vec3 lo = phys_full_inv_multiply(m, o);
        const phys_vec3 ld = phys_inv_multiply(m, d);
        for (const demo_hull *h : shapes[b->shape].hulls)
        {
            float t0 = 0.0f, t1 = best_t;
            bool miss = false;
            for (size_t k = 0; k + 3 < h->planes.size() && !miss; k += 4)
            {
                const float nx = h->planes[k], ny = h->planes[k + 1], nz = h->planes[k + 2], pd = h->planes[k + 3];
                const float denom = nx * ld.x + ny * ld.y + nz * ld.z;
                const float dist = pd - (nx * lo.x + ny * lo.y + nz * lo.z);
                if (fabsf(denom) < 1e-9f)
                {
                    if (dist < 0.0f)
                        miss = true;
                    continue;
                }
                const float t = dist / denom;
                if (denom < 0.0f)
                    t0 = fmaxf(t0, t);
                else
                    t1 = fminf(t1, t);
                if (t0 > t1)
                    miss = true;
            }
            if (!miss && t0 < best_t)
            {
                best_t = t0;
                best = b;
            }
        }
    }
    if (best)
    {
        *t_out = best_t;
        for (int i = 0; i < 3; ++i)
            hit_out[i] = origin[i] + dir[i] * best_t;
    }
    return best;
}

void physics_scene::drag_begin(demo_body *b, const float hit[3], const float plane_n[3])
{
    drag_end();
    rigid_body *const rb = b->rb;
    phys_mat44 &dictator = *static_cast<phys_mat44 *>(m_drag_dictator);
    dictator = PHYS_IDENTITY_MATRIX;
    dictator.w = phys_vec3(hit[0], hit[1], hit[2]);
    dictator.fix_w_column();

    // BO1-style: a user rigid body driven by a caller-owned matrix, pinned to the body with a point constraint.
    user_rigid_body *const urb = phys_sys::create_user_rigid_body(true);
    if (!urb)
        return;
    urb->set(&dictator);
    rigid_body_constraint_point *const rbc = phys_sys::create_rbc_point(rb, urb, true);
    if (!rbc)
    {
        phys_sys::destroy(urb);
        return;
    }
    const phys_vec3 local = phys_inv_multiply(rb->get_mat(), phys_vec3(hit[0], hit[1], hit[2]) - rb->get_mat().GetW());
    rbc->set(local, PHYS_ZERO_VEC);
    const float w = 6.2831853f * settings.drag_hz;
    const float m = settings.mass;
    rbc->set_spring(settings.drag_spring, m * w * w, 2.0f * settings.drag_damping * m * w);

    m_drag_body = b;
    m_drag_urb = urb;
    m_drag_rbc = rbc;
    m_drag_local[0] = local.x;
    m_drag_local[1] = local.y;
    m_drag_local[2] = local.z;
    memcpy(m_drag_plane_n, plane_n, sizeof(m_drag_plane_n));
    m_drag_plane_d = plane_n[0] * hit[0] + plane_n[1] * hit[1] + plane_n[2] * hit[2];
    memcpy(m_drag_target, hit, sizeof(m_drag_target));
}

void physics_scene::drag_move(const float origin[3], const float dir[3])
{
    if (!m_drag_body)
        return;
    const float denom = m_drag_plane_n[0] * dir[0] + m_drag_plane_n[1] * dir[1] + m_drag_plane_n[2] * dir[2];
    if (fabsf(denom) < 1e-6f)
        return;
    const float t = (m_drag_plane_d - (m_drag_plane_n[0] * origin[0] + m_drag_plane_n[1] * origin[1] + m_drag_plane_n[2] * origin[2])) / denom;
    if (t <= 0.0f || t > 1e5f)
        return;
    for (int i = 0; i < 3; ++i)
        m_drag_target[i] = origin[i] + dir[i] * t;
    m_drag_target[2] = fmaxf(m_drag_target[2], 4.0f); // keep the handle above the floor
}

void physics_scene::drag_target(float out[3]) const { memcpy(out, m_drag_target, sizeof(m_drag_target)); }

void physics_scene::drag_end()
{
    if (m_drag_rbc)
        phys_sys::destroy(m_drag_rbc);
    if (m_drag_urb)
        phys_sys::destroy(m_drag_urb);
    m_drag_rbc = nullptr;
    m_drag_urb = nullptr;
    m_drag_body = nullptr;
}

// ---------------------------------------------------------------------------------------------
// Stepping
// ---------------------------------------------------------------------------------------------

void physics_scene::step_fixed()
{
    const float dt = 1.0f / 60.0f;
    if (m_drag_urb)
    {
        // The dictator moves; prolog_frame_advance turns the move into the user body's velocity for this step.
        phys_mat44 &dictator = *static_cast<phys_mat44 *>(m_drag_dictator);
        dictator.w = phys_vec3(m_drag_target[0], m_drag_target[1], m_drag_target[2]);
        if (m_drag_rbc)
        {
            const float w = 6.2831853f * settings.drag_hz;
            const float m = settings.mass;
            m_drag_rbc->set_spring(settings.drag_spring, m * w * w, 2.0f * settings.drag_damping * m * w);
        }
    }
    const auto t0 = std::chrono::high_resolution_clock::now();
    phys_sys::phys_frame_advance(dt);
    const auto t1 = std::chrono::high_resolution_clock::now();
    stats.physics_ms += (float)std::chrono::duration<double, std::milli>(t1 - t0).count();
    ++stats.steps_last_frame;
    ++stats.total_steps;
}

void physics_scene::frame(float real_dt)
{
    stats.physics_ms = 0.0f;
    stats.steps_last_frame = 0;
    if (real_dt > 0.1f)
        real_dt = 0.1f;

    const float dt = 1.0f / 60.0f;
    if (settings.paused)
    {
        m_accum = 0.0f;
        if (settings.step_once)
        {
            step_fixed();
            settings.step_once = false;
        }
    }
    else
    {
        // Rain: spawn at a steady rate over the area, at the spawn height.
        if (settings.rain)
        {
            m_rain_accum += real_dt * settings.rain_rate * settings.time_scale;
            int guard = 0;
            while (m_rain_accum >= 1.0f && guard++ < 16)
            {
                m_rain_accum -= 1.0f;
                const float pos[3] = {(frand() - 0.5f) * settings.spawn_area, (frand() - 0.5f) * settings.spawn_area,
                                      settings.spawn_height + frand() * 200.0f};
                bool blocked = false;
                const float min_d = 2.0f * bound_radius + 4.0f;
                for (demo_body *o : bodies)
                {
                    const phys_vec3 &w = o->rb->get_mat().GetW();
                    const float dx = w.x - pos[0], dy = w.y - pos[1], dz = w.z - pos[2];
                    if (dx * dx + dy * dy + dz * dz < min_d * min_d)
                    {
                        blocked = true;
                        break;
                    }
                }
                if (blocked)
                    continue;
                float rot[3][3];
                const float q0 = frand(), q1 = frand(), q2 = frand();
                random_rotation(q0, q1, q2, rot);
                const float vel[3] = {0, 0, -settings.spawn_speed * frand()};
                const float avel[3] = {(frand() - 0.5f) * settings.spawn_spin, (frand() - 0.5f) * settings.spawn_spin,
                                       (frand() - 0.5f) * settings.spawn_spin};
                spawn_body(pos, rot, vel, avel, settings.spawn_shape);
            }
            if (m_rain_accum > 4.0f)
                m_rain_accum = 4.0f;
        }

        m_accum += real_dt * settings.time_scale;
        int steps = 0;
        while (m_accum >= dt && steps < 4)
        {
            step_fixed();
            m_accum -= dt;
            ++steps;
        }
        if (m_accum > dt)
            m_accum = 0.0f; // too slow to keep up: drop the backlog instead of spiralling
    }
    stats.physics_ms_avg = stats.physics_ms_avg * 0.95f + stats.physics_ms * 0.05f;
    update_stats();
}

void physics_scene::update_stats()
{
    demo_stats &s = stats;
    s.bodies = (int)bodies.size();
    s.stable = s.group_stable = s.nan_bodies = s.below_ground = 0;
    s.contact_points = 0;
    s.min_z = 1e30f;
    s.max_z = -1e30f;
    s.min_vertex_z = 1e30f;
    s.max_speed = 0.0f;
    double zsum = 0.0;
    for (demo_body *b : bodies)
    {
        const rigid_body *rb = b->rb;
        const phys_vec3 &w = rb->get_mat().GetW();
        if (!finite3(w) || !finite3(rb->get_t_vel()) || !finite3(rb->get_a_vel()))
        {
            ++s.nan_bodies;
            continue;
        }
        if (rb->is_stable())
            ++s.stable;
        if (rb->is_group_stable())
            ++s.group_stable;
        s.contact_points += rb->get_contact_count();
        s.min_z = fminf(s.min_z, w.z);
        // Lowest collision vertex in the world: how far the hull reaches into the floor (z = 0).
        {
            const phys_mat44 &m = rb->get_mat();
            float low = 1e30f;
            for (const demo_hull *h : shapes[b->shape].hulls)
                for (size_t k = 0; k + 2 < h->verts.size(); k += 3)
                    low = fminf(low, w.z + h->verts[k] * m.x.z + h->verts[k + 1] * m.y.z + h->verts[k + 2] * m.z.z);
            b->lowest_z = low;
            s.min_vertex_z = fminf(s.min_vertex_z, low);
        }
        s.max_z = fmaxf(s.max_z, w.z);
        zsum += w.z;
        if (w.z < -10.0f)
            ++s.below_ground;
        s.max_speed = fmaxf(s.max_speed, Abs(rb->get_t_vel()));
    }
    const int valid = s.bodies - s.nan_bodies;
    s.avg_z = valid > 0 ? (float)(zsum / valid) : 0.0f;
    if (valid <= 0)
        s.min_z = s.max_z = s.min_vertex_z = 0.0f;
    s.contact_pairs = phys_sys::get_rbc_contact_alloc_count();
    s.asserts = g_assert_count.load();
    s.warnings = g_warning_count.load();
    s.bpb_count = G_BPM ? G_BPM->g_list_bpb_count : 0;
    s.bpi_env_count = G_BPM ? G_BPM->m_bpi_env_count : 0;
}
