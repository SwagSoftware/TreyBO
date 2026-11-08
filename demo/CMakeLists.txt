cmake_minimum_required(VERSION 3.16)

# treybo_demo: Treyarch logos falling onto a floor, simulated by TreyBO Physics. SDL2 + Vulkan (loaded at run
# time, no SDK needed) + Dear ImGui. Built from the TreyBO root (TREYBO_BUILD_DEMO) or standalone (-S demo).

if(NOT TARGET treybo)
    project(treybo_demo C CXX)
    set(TREYBO_BUILD_DEMO OFF) # the root would add this directory again
    set(TREYBO_BUILD_TESTS OFF)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_CURRENT_BINARY_DIR}/treybo)
endif()
enable_language(C)

set(TREYBO_LIB_TARGET treybo)

set(DEMO_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(DEMO_TP ${DEMO_DIR}/third_party)

# ---- SDL2: system package on non-Windows when present, else the vendored static copy (no SDL2main) ----
set(TREYBO_DEMO_SDL2_TARGET "")
if(NOT WIN32)
    find_package(SDL2 QUIET)
    if(TARGET SDL2::SDL2)
        set(TREYBO_DEMO_SDL2_TARGET SDL2::SDL2)
    elseif(SDL2_FOUND AND SDL2_LIBRARIES)
        add_library(treybo_demo_sdl2 INTERFACE)
        target_include_directories(treybo_demo_sdl2 INTERFACE ${SDL2_INCLUDE_DIRS})
        target_link_libraries(treybo_demo_sdl2 INTERFACE ${SDL2_LIBRARIES})
        set(TREYBO_DEMO_SDL2_TARGET treybo_demo_sdl2)
    endif()
endif()
if(TREYBO_DEMO_SDL2_TARGET STREQUAL "")
    if(NOT TARGET SDL2-static)
        add_subdirectory(${DEMO_TP}/sdl2 ${CMAKE_CURRENT_BINARY_DIR}/sdl2 EXCLUDE_FROM_ALL)
    endif()
    set(TREYBO_DEMO_SDL2_TARGET SDL2::SDL2-static)
endif()

# ---- Dear ImGui 1.92.8 (core + SDL2 / Vulkan backends) ----
add_library(treybo_demo_imgui STATIC
    ${DEMO_TP}/imgui/imgui.cpp
    ${DEMO_TP}/imgui/imgui_draw.cpp
    ${DEMO_TP}/imgui/imgui_tables.cpp
    ${DEMO_TP}/imgui/imgui_widgets.cpp
    ${DEMO_TP}/imgui/backends/imgui_impl_sdl2.cpp
    ${DEMO_TP}/imgui/backends/imgui_impl_vulkan.cpp)
target_include_directories(treybo_demo_imgui PUBLIC ${DEMO_TP}/imgui ${DEMO_TP}/vulkan-headers/include)
target_compile_definitions(treybo_demo_imgui PUBLIC VK_NO_PROTOTYPES IMGUI_IMPL_VULKAN_NO_PROTOTYPES)
target_compile_features(treybo_demo_imgui PUBLIC cxx_std_17)
target_link_libraries(treybo_demo_imgui PUBLIC ${TREYBO_DEMO_SDL2_TARGET})

# ---- Shaders: checked-in SPIR-V headers; regenerate with the optional target when glslangValidator exists ----
set(DEMO_SHADERS mesh.vert mesh.frag ground.vert ground.frag blob.vert blob.frag lines.vert lines.frag)
find_program(GLSLANG_VALIDATOR glslangValidator)
if(GLSLANG_VALIDATOR)
    set(DEMO_SHADER_OUTPUTS "")
    foreach(sh ${DEMO_SHADERS})
        string(REPLACE "." "_" sh_name ${sh})
        set(out ${DEMO_DIR}/shaders/generated/${sh_name}.h)
        add_custom_command(OUTPUT ${out}
            COMMAND ${GLSLANG_VALIDATOR} -V --target-env vulkan1.0 --vn spv_${sh_name} -o ${out} ${DEMO_DIR}/shaders/${sh}
            DEPENDS ${DEMO_DIR}/shaders/${sh} ${DEMO_DIR}/shaders/common.glsl
            COMMENT "glslang ${sh}")
        list(APPEND DEMO_SHADER_OUTPUTS ${out})
    endforeach()
    # Not part of the default build: run `--target treybo_demo_shaders` after editing a shader.
    add_custom_target(treybo_demo_shaders DEPENDS ${DEMO_SHADER_OUTPUTS})
endif()

# ---- The demo ----
add_executable(treybo_demo WIN32
    src/main.cpp
    src/obj_loader.cpp
    src/obj_loader.h
    src/physics_scene.cpp
    src/physics_scene.h
    src/renderer.cpp
    src/renderer.h
    src/vk_loader.cpp
    src/vk_loader.h
    src/demo_math.h)
target_link_libraries(treybo_demo PRIVATE ${TREYBO_LIB_TARGET} treybo_demo_imgui ${TREYBO_DEMO_SDL2_TARGET})
target_include_directories(treybo_demo PRIVATE ${DEMO_DIR}/src)
# Fallback asset location (the source tree); the exe first looks in assets/ next to itself.
target_compile_definitions(treybo_demo PRIVATE TREYBO_DEMO_ASSET_DIR="${DEMO_DIR}/assets/")
if(MSVC)
    target_compile_options(treybo_demo PRIVATE /W3 /permissive-)
    target_compile_definitions(treybo_demo PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
    # WIN32 subsystem, but a plain main(): SDL_MAIN_HANDLED, no SDL2main.
    set_target_properties(treybo_demo PROPERTIES LINK_FLAGS "/ENTRY:mainCRTStartup")
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT treybo_demo)
else()
    find_package(Threads REQUIRED)
    target_link_libraries(treybo_demo PRIVATE Threads::Threads ${CMAKE_DL_LIBS})
endif()

# Assets next to the executable.
add_custom_command(TARGET treybo_demo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:treybo_demo>/assets
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DEMO_DIR}/assets/treyarch_symbol.obj ${DEMO_DIR}/assets/treyarch_symbol_lowq.obj
            $<TARGET_FILE_DIR:treybo_demo>/assets/)
