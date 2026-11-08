cmake_minimum_required(VERSION 3.16)
project(TreyBO CXX)

# TreyBO Physics: standalone rebuild of Treyarch's tl/physics (BO1) rigid-body library.
# Headers are included by root-relative path, e.g. "tl/physics/include/physics_system.h".

file(GLOB_RECURSE TREYBO_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/tl/*.cpp)
file(GLOB_RECURSE TREYBO_HEADERS CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/tl/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/tl/*.inc)

add_library(treybo STATIC ${TREYBO_SOURCES} ${TREYBO_HEADERS})
target_include_directories(treybo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(treybo PUBLIC cxx_std_17)

# Asserts follow _DEBUG; force them on in release with -DTREYBO_ASSERTS=ON.
option(TREYBO_ASSERTS "Enable PHYS_ASSERT in all configurations" OFF)
if(TREYBO_ASSERTS)
    target_compile_definitions(treybo PUBLIC TREYBO_ASSERTS)
endif()

if(MSVC)
    # Retail is MSVC SSE scalar float math; keep IEEE semantics (no /fp:fast reassociation).
    target_compile_options(treybo PRIVATE /W3 /fp:precise /permissive-)
    if(CMAKE_SIZEOF_VOID_P EQUAL 4)
        target_compile_options(treybo PRIVATE /arch:SSE2)
    endif()
    source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/tl FILES ${TREYBO_SOURCES} ${TREYBO_HEADERS})
else()
    target_compile_options(treybo PRIVATE -Wall -fno-fast-math)
endif()

# Smoke test: Phys_Init-order startup, a free-falling body and a box resting on a host ground box.
option(TREYBO_BUILD_TESTS "Build the TreyBO smoke test" ON)
if(TREYBO_BUILD_TESTS)
    add_executable(treybo_smoke tests/smoke.cpp)
    target_link_libraries(treybo_smoke PRIVATE treybo)
    if(MSVC)
        target_compile_options(treybo_smoke PRIVATE /W3 /fp:precise /permissive-)
    endif()
endif()

option(TREYBO_BUILD_DEMO "Build the treybo_demo sample (SDL2 + Vulkan + Dear ImGui)" ON)
if(TREYBO_BUILD_DEMO AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/demo/CMakeLists.txt)
    add_subdirectory(demo)
endif()
