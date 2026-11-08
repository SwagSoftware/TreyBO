# TreyBO Physics

![TreyBO Physics demo: a few hundred Treyarch logos falling and piling up](docs/media/treybo.gif)

**TreyBO** is a standalone rebuild of Treyarch's `tl/physics` rigid-body library: the engine Black Ops 1 ships, which its code calls "Nitrous". It provides:

- a pulse-sum (sequential impulse, warm-started) constraint solver;
- GJK collision with cached simplices and a contact-manifold builder;
- a sweep-and-prune broad phase;
- 10 constraint types: contact, point, hinge, distance, angular actuator, upright, ragdoll, wheel, custom orientation, custom path.

It builds on Windows (MSVC, x64 and Win32) and is written to be portable.

**Sources, in priority order**

| Source | Role |
|---|---|
| **BO1 retail** (`CoDMPServer.exe` internal build, mangled names and assert strings) | Behaviour master. Every function was checked against it, NaN branch routing included. |
| **kisakblack** (hand decompilation of BO1) | Starting code. Its errors are fixed and tagged `TREYBO FIX(0xADDR)`. |
| **Spider-Man 3 headers** (2006, library v0.5.0) | Layout, names and API style wherever they don't contradict BO1. |

The file tree follows the retail paths recovered from the IDB: `codsrc\tl\physics\{include,source}`, `codsrc\tl\base`.

## Layout

```
tl/base/        tl_defs.h (asserts, base helpers)  tl_system (host callbacks)  tl_thread (atomics/mutexes)
tl/physics/include/
    old_phys_math.h phys_math.h          phys_vec3 / phys_mat44 / quaternion, value-returning const-ref API
    phys_mem.h phys_transient_allocator.h phys_array_base.inc phys_link_list.h phys_avl_tree.h phys_hash_table.h phys_mutex.h
    physics_system.h                     public API: phys_sys::*
    physics_system_internal.h rigid_body.h rigid_body_internal.h rb_vehicle_model.h
    rbc_defs/                            the 10 constraint types
    constraint_solver/                   pulse-sum solver
    collision/                           GJK, GJK cache, contact manifold, broad phase, convex hull,
                                         bounding volumes, GJK primitive shapes, host world contract
tl/physics/source/                       one .cpp per retail translation unit
tests/smoke.cpp                          minimal host: falling body, box on ground, convex brush on box
demo/                                    SDL2 + Vulkan + Dear ImGui demo (the GIF above)
docs/PORTING.md                          the porting rules every file followed
docs/ledger/*.md                         per-file record: retail addresses verified, fixes, deviations
docs/ida_function_index.tsv              every physics function in the IDB (addr, size, mangled, demangled, assert files)
reference/sm3_headers/                   extracted Spider-Man 3 physics headers (reference only, not built)
```

## Building

```
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

| Option | Default | Effect |
|---|---|---|
| `TREYBO_BUILD_TESTS` | ON | builds `treybo_smoke` |
| `TREYBO_BUILD_DEMO` | ON | builds `treybo_demo` |
| `TREYBO_ASSERTS` | OFF | enables `PHYS_ASSERT` in release |

The demo needs no Vulkan SDK: Vulkan is loaded at run time through SDL2, and its shaders are pre-compiled into headers.

## Using it

Link `treybo`. Includes are root-relative, e.g. `#include "tl/physics/include/physics_system.h"`. `tests/smoke.cpp` is a complete minimal host.

```cpp
tlSetSystemCallbacks(&cb);                         // memory, print/warn/fatal, assert, RunJobs (job dispatch; NULL = serial)
phys_memory_manager_init(buffer, size);            // one block the library carves its pools from
phys_task_manager_init();
phys_sys::phys_init();
broad_phase_system_init(bpmi, should_collide);     // should_collide may be NULL
G_BPM->g_broad_phase_terrain_query_callback = &world_query;   // host world collision, see phys_environment.h
phys_sys::set_collision_callback(broad_phase_default_collision_callback);

// per body
rigid_body *rb = phys_sys::create_rigid_body();
rb->set(mass, inertia, mat, t_vel, a_vel, 3);
rb->m_gravity_acc_vec = phys_vec3(0, 0, -800);     // BO1 units, Z up
broad_phase_info *bpi = create_broad_phase_info();
bpi->set(rb, &rb->get_mat(), &cg_to_world, &cg_to_rb, geom, geom->get_geom_id(), true, stype, NULL, 0x1C7);
environment_collision_list_add(bpi);
aasap_list_add(bpi);

// per frame
phys_sys::phys_frame_advance(dt);
```

**Shapes:** `collision/phys_gjk_geom.h` provides AABB, OBB, double sphere (capsule), cylinder, polygon cylinder, convex brush and triangle partition. `phys_convex_hull_build` makes a hull from a point cloud.

**World collision** is the host's job. `collision/phys_environment.h` documents the contract: implement `broad_phase_terrain_query_callback::query` and hand back `phys_gjk_geom`s for the world pieces that overlap the query box.

## Demo

The demo drops hundreds of Treyarch logos into an arena. Each logo is a convex hull (or a compound of 8 ring-sector hulls). You can grab and drag them with the mouse.

**ImGui panel:**
- spawn and rain;
- pause and single step;
- time scale and gravity;
- solver iterations and sub-steps;
- friction and bounce;
- job threads;
- explode;
- full-detail model and collision-hull overlay;
- stats.

**Mouse:** LMB drags a body, RMB orbits, MMB or Shift+RMB pans, the wheel zooms.

**Keys:** Space pause, `.` step, D drop, E explode, C clear, R reset, H hulls, F full-detail.

## Behaviour notes (all match retail)

- **Contact skin:** each shape carries a 0.34 skin, so resting bodies sit about 0.68 above surfaces.
- **`add_force`:** takes an impulse per solve.
- **Sleeping:** the library never puts bodies to sleep. Sleeping was game code in BO1; the stable flags are only reported.
- **Tunnelling:** rotation isn't swept, so very fast spinning bodies can tunnel at one sub-step. The demo defaults to 2 sub-steps.
- **Precision:** retail runs some intermediates at x87 80-bit precision, which TreyBO computes in float. Results agree to float precision, not bit for bit.

## Provenance

TreyBO is derived from Activision/Treyarch code, reconstructed from a decompiled retail binary and leaked headers. It is published for research and preservation. All rights to the original code remain with their owners.
