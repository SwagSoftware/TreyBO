// treybo_demo: Treyarch logos raining onto a floor, simulated by TreyBO Physics (the BO1 tl/physics rebuild).
// SDL2 window + Vulkan renderer + Dear ImGui panel. "-autotest" runs the physics headless and logs statistics.

#define SDL_MAIN_HANDLED
#include "vk_loader.h" // Vulkan types before SDL_vulkan.h
#include <SDL.h>
#include <SDL_vulkan.h>

#include "demo_math.h"
#include "obj_loader.h"
#include "physics_scene.h"
#include "renderer.h"

#include "imgui.h"
#include "backends/imgui_impl_sdl2.h"
#include "backends/imgui_impl_vulkan.h"

#include "tl/physics/include/rigid_body.h"
#include "tl/physics/include/physics_system.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#ifndef TREYBO_DEMO_ASSET_DIR
#define TREYBO_DEMO_ASSET_DIR ""
#endif

static const char *const DEMO_TITLE = "TreyBO Physics demo";

static std::string find_asset(const char *name)
{
    std::vector<std::string> dirs;
    if (char *base = SDL_GetBasePath())
    {
        dirs.push_back(std::string(base) + "assets/");
        dirs.push_back(std::string(base) + "../assets/");
        SDL_free(base);
    }
    dirs.push_back(TREYBO_DEMO_ASSET_DIR);
    dirs.push_back("assets/");
    for (const std::string &d : dirs)
    {
        const std::string p = d + name;
        if (FILE *f = fopen(p.c_str(), "rb"))
        {
            fclose(f);
            return p;
        }
    }
    return name;
}

// Render vertices of an OBJ in the body frame (the same transform the hulls and mass properties use).
static void build_render_mesh(const physics_scene &scene, const obj_mesh &obj, std::vector<render_vertex> *verts)
{
    verts->resize(obj.vertices.size());
    for (size_t i = 0; i < obj.vertices.size(); ++i)
    {
        scene.model_to_body(obj.vertices[i].pos, (*verts)[i].pos);
        scene.model_normal_to_body(obj.vertices[i].nrm, (*verts)[i].nrm);
    }
}

static void body_instance(const demo_body *b, render_instance *inst, bool highlight)
{
    const phys_mat44 &m = b->rb->get_mat();
    const phys_vec3 *rows[4] = {&m.x, &m.y, &m.z, &m.w};
    for (int r = 0; r < 4; ++r)
    {
        inst->r[r][0] = rows[r]->x;
        inst->r[r][1] = rows[r]->y;
        inst->r[r][2] = rows[r]->z;
        inst->r[r][3] = r == 3 ? 1.0f : 0.0f;
    }
    for (int c = 0; c < 3; ++c)
        inst->tint[c] = highlight ? fminf(1.0f, b->tint[c] * 0.5f + 0.6f) : b->tint[c];
    inst->tint[3] = 1.0f;
}

// ---------------------------------------------------------------------------------------------
// Autotest: physics only
// ---------------------------------------------------------------------------------------------

static void log_stats(const char *tag, int frame, const physics_scene &s)
{
    const demo_stats &st = s.stats;
    demo_log("%s frame %4d: bodies %4d stable %4d group_stable %4d contact_pairs %5d contact_pts %5d z[min %.2f avg %.2f max %.2f] "
             "lowest_vertex %.2f max_speed %8.2f nan %d below_ground %d asserts %d warnings %d phys %.2f ms",
             tag, frame, st.bodies, st.stable, st.group_stable, st.contact_pairs, st.contact_points, st.min_z, st.avg_z, st.max_z,
             st.min_vertex_z, st.max_speed, st.nan_bodies, st.below_ground, st.asserts, st.warnings, st.physics_ms);
}

static int run_autotest(int argc, char **argv)
{
    int frames = 600, count = 300, threads = 0, explode_at = -1;
    bool compound = false, edge_test = false, trace_all = false, drag_test = false;
    float edge_tilt = 1.5707963f, edge_height = 30.0f, edge_speed = 250.0f, max_avel = -1.0f;
    int substeps = demo_settings().substeps;
    std::string log_path = "autotest.log";
    for (int i = 1; i < argc; ++i)
    {
        if (!strcmp(argv[i], "-frames") && i + 1 < argc)
            frames = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-count") && i + 1 < argc)
            count = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-threads") && i + 1 < argc)
            threads = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-explode_at") && i + 1 < argc)
            explode_at = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-log") && i + 1 < argc)
            log_path = argv[++i];
        else if (!strcmp(argv[i], "-compound"))
            compound = true;
        else if (!strcmp(argv[i], "-edge_test") && i + 3 < argc)
        {
            edge_test = true;
            edge_tilt = (float)atof(argv[++i]);
            edge_height = (float)atof(argv[++i]);
            edge_speed = (float)atof(argv[++i]);
        }
        else if (!strcmp(argv[i], "-trace"))
            trace_all = true;
        else if (!strcmp(argv[i], "-drag_test"))
            drag_test = true;
        else if (!strcmp(argv[i], "-substeps") && i + 1 < argc)
            substeps = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-max_avel") && i + 1 < argc)
            max_avel = (float)atof(argv[++i]);
    }
    demo_log_set_file(log_path.c_str());
    demo_log("treybo_demo autotest: %d bodies, %d frames of 1/60 s, shape %s, threads %d, substeps %d", count, frames,
             compound ? "compound" : "hull", threads, substeps);

    obj_mesh lowq;
    std::string err;
    const std::string path = find_asset("treyarch_symbol_lowq.obj");
    if (!obj_load(path.c_str(), &lowq, &err))
    {
        demo_log("FAIL: %s (%s)", err.c_str(), path.c_str());
        return 2;
    }
    demo_log("loaded %s: %zu positions, %d uvs, %d normals, %d faces", path.c_str(), lowq.positions.size() / 3, lowq.uv_count,
             lowq.normal_count, lowq.face_count);

    physics_scene *scene = new physics_scene;
    scene->settings.worker_threads = threads;
    scene->settings.spawn_shape = compound ? DEMO_SHAPE_COMPOUND : DEMO_SHAPE_HULL;
    scene->settings.substeps = substeps;
    if (max_avel > 0.0f)
        scene->settings.max_avel = max_avel;
    if (!scene->init(lowq, &err))
    {
        demo_log("FAIL: physics init: %s", err.c_str());
        return 2;
    }
    int made = 0;
    if (edge_test)
    {
        // One logo standing on its rim, rolling along +x (the fall-through case in _demo.md).
        const float c = cosf(edge_tilt), s = sinf(edge_tilt);
        const float rot[3][3] = {{1, 0, 0}, {0, c, s}, {0, -s, c}}; // body z axis tilted towards -y
        const float pos[3] = {0, 0, edge_height};
        const float vel[3] = {edge_speed, 0, 0};
        const float avel[3] = {0, edge_speed / 25.0f, 0};
        made = scene->spawn_body(pos, rot, vel, avel, scene->settings.spawn_shape) ? 1 : 0;
    }
    else
        made = scene->drop(count);
    scene->update_stats();
    demo_log("dropped %d bodies (requested %d)", made, count);
    log_stats("start", 0, *scene);

    const auto t0 = std::chrono::high_resolution_clock::now();
    double phys_total = 0.0, phys_max = 0.0;
    int first_nan_frame = -1;
    float drag_start_z = 0.0f, drag_target_end[3] = {0, 0, 0};
    std::vector<std::vector<std::string>> history;
    std::vector<char> reported(DEMO_MAX_BODIES, 0);
    int fallen_reported = 0;
    for (int f = 1; f <= frames; ++f)
    {
        if (f == explode_at)
        {
            scene->explode();
            demo_log("explode at frame %d", f);
        }
        // Drag check (pick from above, lift through the user-body point constraint, release).
        if (drag_test && f == frames / 2)
        {
            const float o[3] = {0.0f, 0.0f, 2000.0f}, d[3] = {0.0f, 0.0f, -1.0f};
            float t, hit[3];
            if (demo_body *b = scene->pick(o, d, &t, hit))
            {
                const float n[3] = {1.0f, 0.0f, 0.0f}; // vertical drag plane through the hit
                scene->drag_begin(b, hit, n);
                drag_start_z = b->rb->get_mat().GetW().z;
                demo_log("drag: picked a body at (%.1f %.1f %.1f), t %.1f", hit[0], hit[1], hit[2], t);
            }
            else
                demo_log("drag: pick missed");
        }
        if (scene->dragged() && f > frames / 2)
        {
            // Raise the handle 2 units per frame up to 300 units.
            float tgt[3];
            scene->drag_target(tgt);
            const float o[3] = {tgt[0] - 500.0f, tgt[1], fminf(tgt[2] + 2.0f, 300.0f)}, d[3] = {1.0f, 0.0f, 0.0f};
            scene->drag_move(o, d);
        }
        if (scene->dragged() && f == frames / 2 + 150)
        {
            scene->drag_target(drag_target_end);
            demo_log("drag: after 150 frames the body is at z %.1f (start %.1f), handle at z %.1f; releasing",
                     scene->dragged()->rb->get_mat().GetW().z, drag_start_z, drag_target_end[2]);
            scene->drag_end();
        }
        scene->stats.physics_ms = 0.0f;
        scene->step_fixed();
        phys_total += scene->stats.physics_ms;
        phys_max = std::max(phys_max, (double)scene->stats.physics_ms);
        scene->update_stats();
        if (scene->stats.nan_bodies && first_nan_frame < 0)
        {
            first_nan_frame = f;
            demo_log("first NaN body at frame %d", f);
        }
        // Fall-through diagnosis: the recent track of the first body whose centre drops below the floor.
        if (history.size() < scene->bodies.size())
            history.resize(scene->bodies.size());
        for (size_t i = 0; i < scene->bodies.size(); ++i)
        {
            const rigid_body *rb = scene->bodies[i]->rb;
            const phys_vec3 &w = rb->get_mat().GetW();
            const phys_vec3 &v = rb->get_t_vel();
            char line[256];
            snprintf(line, sizeof(line), "  frame %d pos (%.2f %.2f %.2f) low %.2f vel (%.1f %.1f %.1f) axis_z (%.2f %.2f %.2f) contacts %d flags %x", f,
                     w.x, w.y, w.z, scene->bodies[i]->lowest_z, v.x, v.y, v.z, rb->get_mat().z.x, rb->get_mat().z.y, rb->get_mat().z.z,
                     rb->get_contact_count(), rb->m_flags);
            history[i].push_back(line);
            if (trace_all)
                demo_log("body %zu%s", i, line);
            if (history[i].size() > 24)
                history[i].erase(history[i].begin());
            if (w.z < 0.0f && fallen_reported < 2 && !reported[i])
            {
                reported[i] = 1;
                ++fallen_reported;
                demo_log("body %zu went below the floor at frame %d; last frames:", i, f);
                for (const std::string &h : history[i])
                    demo_log("%s", h.c_str());
            }
        }
        if (f % 30 == 0 || f == frames)
            log_stats("step", f, *scene);
    }
    const double wall = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t0).count();
    const demo_stats &st = scene->stats;
    demo_log("physics: total %.1f ms, avg %.3f ms/frame, max %.3f ms/frame, wall %.2f s", phys_total, phys_total / frames, phys_max, wall);

    // Verdict: finite, nothing through the floor, the pile is on the ground and has (mostly) come to rest.
    const bool finite_ok = st.nan_bodies == 0;
    const bool floor_ok = st.below_ground == 0 && st.min_vertex_z > -2.0f; // no hull deeper than ~3x the contact skin
    const bool fell = st.max_z < scene->settings.spawn_height; // everything left the spawn layers
    const bool settled = st.max_speed < 60.0f;
    demo_log("verdict: finite %s, floor %s, fell %s, settled %s (max speed %.2f, %d/%d stable)", finite_ok ? "yes" : "NO",
             floor_ok ? "yes" : "NO", fell ? "yes" : "NO", settled ? "yes" : "NO", st.max_speed, st.stable, st.bodies);
    const bool pass = finite_ok && floor_ok && fell;
    demo_log(pass ? "AUTOTEST PASS" : "AUTOTEST FAIL");
    scene->shutdown();
    demo_log_set_file(nullptr);
    return pass ? 0 : 1;
}

// ---------------------------------------------------------------------------------------------
// Interactive
// ---------------------------------------------------------------------------------------------

struct camera
{
    vec3 target = vec3(0, 0, 40);
    float yaw = 0.75f;
    float pitch = 0.42f;
    float dist = 1300.0f;
    vec3 eye() const
    {
        return target + vec3(cosf(pitch) * cosf(yaw), cosf(pitch) * sinf(yaw), sinf(pitch)) * dist;
    }
};

static void make_ray(const mat4 &inv_viewproj, float mx, float my, float w, float h, vec3 *origin, vec3 *dir)
{
    const float nx = 2.0f * mx / w - 1.0f;
    const float ny = 2.0f * my / h - 1.0f; // Vulkan: -1 at the top
    const vec3 p0 = transform_point(inv_viewproj, vec3(nx, ny, 0.0f));
    const vec3 p1 = transform_point(inv_viewproj, vec3(nx, ny, 1.0f));
    *origin = p0;
    *dir = normalize(p1 - p0);
}

int main(int argc, char **argv)
{
    SDL_SetMainReady();
    for (int i = 1; i < argc; ++i)
        if (!strcmp(argv[i], "-autotest"))
            return run_autotest(argc, argv);

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0)
    {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }
    if (SDL_Vulkan_LoadLibrary(nullptr) != 0)
    {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, DEMO_TITLE, (std::string("Vulkan not available: ") + SDL_GetError()).c_str(), nullptr);
        return 1;
    }
    SDL_Window *window = SDL_CreateWindow(DEMO_TITLE, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1600, 900,
                                          SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (!window)
    {
        fprintf(stderr, "SDL_CreateWindow: %s\n", SDL_GetError());
        return 1;
    }

    std::string err;
    renderer *gfx = new renderer;
    if (!gfx->init(window, &err))
    {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, DEMO_TITLE, ("Vulkan init failed: " + err).c_str(), window);
        return 1;
    }

    // Assets and physics.
    obj_mesh lowq, full;
    const std::string lowq_path = find_asset("treyarch_symbol_lowq.obj");
    if (!obj_load(lowq_path.c_str(), &lowq, &err))
    {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, DEMO_TITLE, ("cannot load " + lowq_path + ": " + err).c_str(), window);
        return 1;
    }
    demo_log("loaded %s: %zu positions, %d uvs, %d normals, %d faces", lowq_path.c_str(), lowq.positions.size() / 3, lowq.uv_count,
             lowq.normal_count, lowq.face_count);
    physics_scene *scene = new physics_scene;
    if (!scene->init(lowq, &err))
    {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, DEMO_TITLE, ("physics init failed: " + err).c_str(), window);
        return 1;
    }

    std::vector<render_vertex> rv;
    build_render_mesh(*scene, lowq, &rv);
    const int mesh_lowq = gfx->create_mesh(rv.data(), (uint32_t)rv.size(), lowq.indices.data(), (uint32_t)lowq.indices.size());
    int mesh_full = -1;
    bool full_failed = false;

    // Unit cube [-1, 1]^3 for the walls.
    int mesh_cube;
    {
        std::vector<render_vertex> cv;
        std::vector<uint32_t> ci;
        for (int axis = 0; axis < 3; ++axis)
            for (int s = -1; s <= 1; s += 2)
            {
                const uint32_t base = (uint32_t)cv.size();
                const int u = (axis + 1) % 3, v = (axis + 2) % 3;
                const float corners[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
                for (const auto &c : corners)
                {
                    render_vertex vtx = {};
                    vtx.pos[axis] = (float)s;
                    vtx.pos[u] = c[0];
                    vtx.pos[v] = c[1];
                    vtx.nrm[axis] = (float)s;
                    cv.push_back(vtx);
                }
                const uint32_t q[6] = {0, 1, 2, 0, 2, 3};
                for (uint32_t k : q)
                    ci.push_back(base + k);
            }
        mesh_cube = gfx->create_mesh(cv.data(), (uint32_t)cv.size(), ci.data(), (uint32_t)ci.size());
    }

    // Collision hull wireframes, one line mesh per shape.
    int mesh_hull_lines[DEMO_SHAPE_COUNT];
    for (int s = 0; s < DEMO_SHAPE_COUNT; ++s)
    {
        std::vector<float> pos;
        std::vector<uint32_t> idx;
        for (const demo_hull *h : scene->shapes[s].hulls)
        {
            const uint32_t base = (uint32_t)(pos.size() / 3);
            pos.insert(pos.end(), h->verts.begin(), h->verts.end());
            for (uint16_t e : h->edges)
                idx.push_back(base + e);
        }
        mesh_hull_lines[s] = gfx->create_lines(pos.data(), (uint32_t)(pos.size() / 3), idx.data(), (uint32_t)idx.size());
    }

    // ImGui.
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.IniFilename = nullptr;
    ImGui::StyleColorsDark();
    ImGuiStyle &style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.FrameRounding = 3.0f;
    style.ItemSpacing = ImVec2(6, 4);
    ImGui_ImplSDL2_InitForVulkan(window);
    if (!gfx->init_imgui(&err))
    {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, DEMO_TITLE, err.c_str(), window);
        return 1;
    }

    camera cam;
    bool use_full = false, show_hulls = false, show_walls_mesh = true, vsync = true, blobs = true, show_log = false;
    std::vector<render_instance> instances;
    std::vector<demo_body *> by_shape[DEMO_SHAPE_COUNT];
    bool running = true;
    bool orbiting = false, panning = false;
    float fps = 0.0f;
    auto last = std::chrono::high_resolution_clock::now();
    mat4 viewproj = mat4::identity(), inv_viewproj = mat4::identity();
    int win_w = 1600, win_h = 900;

    // Self-check: "-shot file.bmp [-shot_frame N] [-hulls] [-full]" saves frame N (default 240) and exits.
    std::string shot_path;
    int shot_frame = -1, frame_no = 0;
    float shot_time = 4.0f, run_time = 0.0f; // capture after this many seconds unless -shot_frame is given
    for (int i = 1; i < argc; ++i)
    {
        if (!strcmp(argv[i], "-shot") && i + 1 < argc)
            shot_path = argv[++i];
        else if (!strcmp(argv[i], "-shot_frame") && i + 1 < argc)
            shot_frame = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-hulls"))
            show_hulls = true;
        else if (!strcmp(argv[i], "-full"))
            use_full = true;
        else if (!strcmp(argv[i], "-cam_dist") && i + 1 < argc)
            cam.dist = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "-shot_time") && i + 1 < argc)
            shot_time = (float)atof(argv[++i]);
    }
    if (use_full)
    {
        const std::string full_path = find_asset("treyarch_symbol.obj");
        if (obj_load(full_path.c_str(), &full, &err))
        {
            build_render_mesh(*scene, full, &rv);
            mesh_full = gfx->create_mesh(rv.data(), (uint32_t)rv.size(), full.indices.data(), (uint32_t)full.indices.size());
            full = obj_mesh();
        }
    }

    scene->drop(scene->settings.spawn_count);

    while (running)
    {
        ++frame_no;
        if (!shot_path.empty() && shot_frame < 0 && run_time >= shot_time)
            shot_frame = frame_no;
        if (!shot_path.empty() && frame_no == shot_frame)
            gfx->request_capture(shot_path.c_str());
        if (!shot_path.empty() && shot_frame >= 0 && frame_no > shot_frame && !gfx->capture_pending())
        {
            demo_log("saved %s at frame %d, %.1f s (%d bodies, %d stable, %.0f fps)", shot_path.c_str(), shot_frame, run_time,
                     scene->stats.bodies, scene->stats.stable, fps);
            break;
        }
        SDL_Event ev;
        SDL_GetWindowSize(window, &win_w, &win_h);
        while (SDL_PollEvent(&ev))
        {
            ImGui_ImplSDL2_ProcessEvent(&ev);
            const bool ui_mouse = io.WantCaptureMouse;
            const bool ui_keys = io.WantCaptureKeyboard;
            switch (ev.type)
            {
            case SDL_QUIT:
                running = false;
                break;
            case SDL_WINDOWEVENT:
                if (ev.window.event == SDL_WINDOWEVENT_SIZE_CHANGED || ev.window.event == SDL_WINDOWEVENT_RESIZED)
                    gfx->notify_resize();
                break;
            case SDL_MOUSEBUTTONDOWN:
                if (ui_mouse)
                    break;
                if (ev.button.button == SDL_BUTTON_LEFT && !(SDL_GetModState() & KMOD_ALT))
                {
                    vec3 o, d;
                    make_ray(inv_viewproj, (float)ev.button.x, (float)ev.button.y, (float)win_w, (float)win_h, &o, &d);
                    float t, hit[3];
                    if (demo_body *b = scene->pick(&o.x, &d.x, &t, hit))
                    {
                        const vec3 n = normalize(cam.eye() - cam.target);
                        scene->drag_begin(b, hit, &n.x);
                    }
                    else
                        orbiting = true;
                }
                else if (ev.button.button == SDL_BUTTON_RIGHT || (ev.button.button == SDL_BUTTON_LEFT && (SDL_GetModState() & KMOD_ALT)))
                    orbiting = true;
                else if (ev.button.button == SDL_BUTTON_MIDDLE)
                    panning = true;
                break;
            case SDL_MOUSEBUTTONUP:
                if (ev.button.button == SDL_BUTTON_LEFT)
                {
                    scene->drag_end();
                    orbiting = false;
                }
                if (ev.button.button == SDL_BUTTON_RIGHT)
                    orbiting = false;
                if (ev.button.button == SDL_BUTTON_MIDDLE)
                    panning = false;
                break;
            case SDL_MOUSEMOTION:
                if (scene->dragged())
                {
                    vec3 o, d;
                    make_ray(inv_viewproj, (float)ev.motion.x, (float)ev.motion.y, (float)win_w, (float)win_h, &o, &d);
                    scene->drag_move(&o.x, &d.x);
                }
                else if (orbiting && !(SDL_GetModState() & KMOD_SHIFT))
                {
                    cam.yaw -= ev.motion.xrel * 0.006f;
                    cam.pitch = fminf(1.5f, fmaxf(0.03f, cam.pitch + ev.motion.yrel * 0.006f));
                }
                else if (panning || (orbiting && (SDL_GetModState() & KMOD_SHIFT)))
                {
                    const vec3 f = normalize(cam.target - cam.eye());
                    const vec3 r = normalize(cross(f, vec3(0, 0, 1)));
                    const vec3 u = cross(r, f);
                    const float k = cam.dist * 0.0012f;
                    cam.target += r * (-ev.motion.xrel * k) + u * (ev.motion.yrel * k);
                }
                break;
            case SDL_MOUSEWHEEL:
                if (!ui_mouse)
                    cam.dist = fminf(8000.0f, fmaxf(80.0f, cam.dist * powf(0.88f, (float)ev.wheel.y)));
                break;
            case SDL_KEYDOWN:
                if (ui_keys)
                    break;
                switch (ev.key.keysym.sym)
                {
                case SDLK_ESCAPE: running = false; break;
                case SDLK_SPACE: scene->settings.paused = !scene->settings.paused; break;
                case SDLK_PERIOD: scene->settings.step_once = true; break;
                case SDLK_d: scene->drop(scene->settings.spawn_count); break;
                case SDLK_e: scene->explode(); break;
                case SDLK_c: scene->clear_all(); break;
                case SDLK_r: scene->settings.rain = !scene->settings.rain; break;
                case SDLK_h: show_hulls = !show_hulls; break;
                case SDLK_f: cam = camera(); break;
                default: break;
                }
                break;
            default:
                break;
            }
        }
        if (!running)
            break;

        const auto now = std::chrono::high_resolution_clock::now();
        const float dt = (float)std::chrono::duration<double>(now - last).count();
        last = now;
        run_time += dt;
        fps = fps * 0.95f + (dt > 0 ? 1.0f / dt : 0.0f) * 0.05f;

        scene->frame(dt);

        // ---------------- UI ----------------
        ImGui_ImplVulkan_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(370, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin(DEMO_TITLE, nullptr, ImGuiWindowFlags_NoSavedSettings);
        demo_settings &s = scene->settings;
        bool push = false;
        const demo_stats &st = scene->stats;
        ImGui::Text("%d bodies  %d stable  %d group-stable", st.bodies, st.stable, st.group_stable);
        ImGui::Text("contacts: %d pairs, %d points", st.contact_pairs, st.contact_points);
        ImGui::Text("physics %.2f ms/frame (avg)  %.0f fps", st.physics_ms_avg, fps);
        if (st.nan_bodies || st.asserts || st.warnings)
            ImGui::TextColored(ImVec4(1, 0.5f, 0.3f, 1), "NaN %d  asserts %d  warnings %d", st.nan_bodies, st.asserts, st.warnings);
        if (!scene->can_spawn())
            ImGui::TextColored(ImVec4(1, 0.8f, 0.2f, 1), "spawn cap reached (%d bodies / %d pairs)", DEMO_MAX_BODIES, DEMO_CONTACT_PAIR_GUARD);

        if (ImGui::CollapsingHeader("Spawn", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::SliderInt("count", &s.spawn_count, 1, 500);
            ImGui::SliderFloat("height", &s.spawn_height, 50.0f, 2000.0f, "%.0f");
            ImGui::SliderFloat("area", &s.spawn_area, 100.0f, 2.0f * scene->arena_half - 40.0f, "%.0f");
            if (ImGui::Button("Drop N (D)"))
                scene->drop(s.spawn_count);
            ImGui::SameLine();
            if (ImGui::Button("Clear (C)"))
                scene->clear_all();
            ImGui::SameLine();
            if (ImGui::Button("Reset scene"))
            {
                scene->reset();
                cam = camera();
            }
            ImGui::Checkbox("rain (R)", &s.rain);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(150);
            ImGui::SliderFloat("per s", &s.rain_rate, 1.0f, 120.0f, "%.0f");
            ImGui::RadioButton("hull", &s.spawn_shape, DEMO_SHAPE_HULL);
            ImGui::SameLine();
            ImGui::RadioButton("compound (8 segments)", &s.spawn_shape, DEMO_SHAPE_COMPOUND);
            ImGui::SliderFloat("spin", &s.spawn_spin, 0.0f, 16.0f, "%.1f rad/s");
        }
        if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Checkbox("pause (Space)", &s.paused);
            ImGui::SameLine();
            if (ImGui::Button("step (.)"))
                s.step_once = true;
            ImGui::SliderFloat("time scale", &s.time_scale, 0.05f, 2.0f, "%.2f");
            push |= ImGui::SliderFloat("gravity", &s.gravity, 0.0f, 2000.0f, "%.0f u/s^2");
            push |= ImGui::SliderInt("v iters", &s.v_iters, 1, 32);
            push |= ImGui::SliderInt("vp iters", &s.vp_iters, 0, 32);
            push |= ImGui::SliderInt("sub-steps", &s.substeps, 1, 4);
            push |= ImGui::SliderFloat("friction", &s.friction, 0.0f, 1.5f, "%.2f");
            push |= ImGui::SliderFloat("bounce", &s.bounce, 0.0f, 1.0f, "%.2f");
            push |= ImGui::SliderFloat("max avel", &s.max_avel, 1.0f, 64.0f, "%.0f");
            ImGui::SliderFloat("mass (new)", &s.mass, 1.0f, 100.0f, "%.1f");
            push |= ImGui::SliderInt("job threads", &s.worker_threads, 0, 8);
            push |= ImGui::Checkbox("arena walls", &s.walls);
        }
        if (ImGui::CollapsingHeader("Interact", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (ImGui::Button("Explode (E)"))
                scene->explode();
            ImGui::SameLine();
            ImGui::SetNextItemWidth(110);
            ImGui::SliderFloat("##str", &s.explode_strength, 100.0f, 3000.0f, "%.0f u/s");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(80);
            ImGui::SliderFloat("##rad", &s.explode_radius, 50.0f, 1000.0f, "r %.0f");
            ImGui::Checkbox("soft drag", &s.drag_spring);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(90);
            ImGui::SliderFloat("Hz", &s.drag_hz, 0.5f, 15.0f, "%.1f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(70);
            ImGui::SliderFloat("damp", &s.drag_damping, 0.05f, 2.0f, "%.2f");
        }
        if (ImGui::CollapsingHeader("Render", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (ImGui::Checkbox("full-detail model (126k tris)", &use_full) && use_full && mesh_full < 0 && !full_failed)
            {
                const std::string full_path = find_asset("treyarch_symbol.obj");
                if (obj_load(full_path.c_str(), &full, &err))
                {
                    build_render_mesh(*scene, full, &rv);
                    mesh_full = gfx->create_mesh(rv.data(), (uint32_t)rv.size(), full.indices.data(), (uint32_t)full.indices.size());
                    demo_log("loaded %s: %zu verts, %zu tris", full_path.c_str(), rv.size(), full.indices.size() / 3);
                    full = obj_mesh();
                }
                else
                {
                    full_failed = true;
                    demo_log("cannot load %s: %s", full_path.c_str(), err.c_str());
                }
            }
            ImGui::Checkbox("collision hulls (H)", &show_hulls);
            ImGui::SameLine();
            ImGui::Checkbox("blob shadows", &blobs);
            if (ImGui::Checkbox("vsync", &vsync))
                gfx->set_vsync(vsync);
            ImGui::SameLine();
            ImGui::Checkbox("log", &show_log);
            ImGui::TextDisabled("%s, %dx MSAA", gfx->device_name(), gfx->msaa_samples());
        }
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        ImGui::TextWrapped("LMB: drag a body (or orbit on empty space)  RMB: orbit  MMB / Shift+RMB: pan  wheel: zoom  F: reset view");
        ImGui::PopStyleColor();
        ImGui::End();
        if (show_log)
        {
            ImGui::SetNextWindowSize(ImVec2(700, 260), ImGuiCond_FirstUseEver);
            ImGui::Begin("log", &show_log);
            for (const std::string &l : demo_log_lines())
                ImGui::TextUnformatted(l.c_str());
            if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
                ImGui::SetScrollHereY(1.0f);
            ImGui::End();
        }
        if (push)
            scene->apply_settings();
        ImGui::Render();
        (void)show_walls_mesh;

        // ---------------- Scene ----------------
        const VkExtent2D ext = gfx->extent();
        const float aspect = ext.height ? (float)ext.width / (float)ext.height : 1.0f;
        const mat4 view = look_at(cam.eye(), cam.target, vec3(0, 0, 1));
        const mat4 proj = perspective_vk(0.9f, aspect, 4.0f, 20000.0f);
        viewproj = proj * view;
        invert(viewproj, &inv_viewproj);

        render_frame rf;
        rf.viewproj = viewproj;
        rf.cam_pos = cam.eye();
        rf.ground_half = scene->ground_half;
        rf.blob_radius = scene->bound_radius * 0.9f;
        instances.clear();
        for (auto &v : by_shape)
            v.clear();
        for (demo_body *b : scene->bodies)
            by_shape[b->shape].push_back(b);
        uint32_t first[DEMO_SHAPE_COUNT];
        for (int sh = 0; sh < DEMO_SHAPE_COUNT; ++sh)
        {
            first[sh] = (uint32_t)instances.size();
            for (demo_body *b : by_shape[sh])
            {
                render_instance inst;
                body_instance(b, &inst, b == scene->dragged());
                instances.push_back(inst);
            }
        }
        const uint32_t body_count = (uint32_t)instances.size();
        const int body_mesh = (use_full && mesh_full >= 0) ? mesh_full : mesh_lowq;
        render_draw d;
        d.mesh = body_mesh;
        d.first_instance = 0;
        d.instance_count = body_count;
        rf.meshes.push_back(d);
        if (show_hulls)
            for (int sh = 0; sh < DEMO_SHAPE_COUNT; ++sh)
            {
                render_draw l;
                l.mesh = mesh_hull_lines[sh];
                l.first_instance = first[sh];
                l.instance_count = (uint32_t)by_shape[sh].size();
                rf.lines.push_back(l);
            }
        if (blobs)
        {
            rf.blobs.first_instance = 0;
            rf.blobs.instance_count = body_count;
        }
        if (s.walls)
        {
            const uint32_t wall_first = (uint32_t)instances.size();
            for (const physics_scene::env_piece &p : scene->env_pieces)
            {
                if (!p.is_wall)
                    continue;
                render_instance inst = {};
                for (int c = 0; c < 3; ++c)
                {
                    inst.r[c][c] = 0.5f * (p.mx[c] - p.mn[c]);
                    inst.r[3][c] = 0.5f * (p.mx[c] + p.mn[c]);
                }
                inst.r[3][3] = 1.0f;
                inst.tint[0] = 0.42f;
                inst.tint[1] = 0.45f;
                inst.tint[2] = 0.50f;
                inst.tint[3] = 1.0f;
                instances.push_back(inst);
            }
            render_draw w;
            w.mesh = mesh_cube;
            w.first_instance = wall_first;
            w.instance_count = (uint32_t)instances.size() - wall_first;
            rf.meshes.push_back(w);
        }
        // Drag handle: a small cube at the target.
        if (scene->dragged())
        {
            float t[3];
            scene->drag_target(t);
            render_instance inst = {};
            for (int c = 0; c < 3; ++c)
            {
                inst.r[c][c] = 3.0f;
                inst.r[3][c] = t[c];
            }
            inst.r[3][3] = 1.0f;
            inst.tint[0] = 1.0f;
            inst.tint[1] = 1.0f;
            inst.tint[2] = 0.2f;
            inst.tint[3] = 1.0f;
            render_draw h;
            h.mesh = mesh_cube;
            h.first_instance = (uint32_t)instances.size();
            h.instance_count = 1;
            instances.push_back(inst);
            rf.meshes.push_back(h);
        }
        rf.instances = instances.data();
        rf.instance_count = (uint32_t)instances.size();

        if (!gfx->render(rf, ImGui::GetDrawData()))
        {
            demo_log("renderer: device lost");
            running = false;
        }
    }

    scene->shutdown();
    gfx->shutdown_imgui();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    gfx->shutdown();
    SDL_DestroyWindow(window);
    SDL_Vulkan_UnloadLibrary();
    SDL_Quit();
    return 0;
}
