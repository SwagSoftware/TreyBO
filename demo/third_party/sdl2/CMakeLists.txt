cmake_minimum_required(VERSION 3.5.0 FATAL_ERROR)

message(STATUS "[rtcw_sdl2]")

set(SDL2_DISABLE_INSTALL ON CACHE BOOL "Disable installation of SDL2" FORCE)
set(SDL2_DISABLE_UNINSTALL ON CACHE BOOL "Disable uninstallation of SDL2" FORCE)
set(SDL2_DISABLE_SDL2MAIN ON CACHE BOOL "Disable building/installation of SDL2main" FORCE)
set(SDL_SHARED OFF CACHE BOOL "Build a shared version of the library" FORCE)
set(SDL_STATIC ON CACHE BOOL "Build a static version of the library" FORCE)
set(SDL_TEST OFF CACHE BOOL "Build the SDL2_test library" FORCE)
set(SDL_TESTS OFF CACHE BOOL "Build the test directory" FORCE)
set(SDL_FORCE_STATIC_VCRT OFF CACHE BOOL "Force /MT for static VC runtimes" FORCE)

# TreyBO demo: the rtcw tree defines this in cmake/rtcw.cmake (static CRT when RTCW_ENABLE_STATIC_LINKING). The demo
# keeps the default dynamic CRT, so it is a no-op here.
if(NOT COMMAND rtcw_configure_static_linking)
    function(rtcw_configure_static_linking)
    endfunction()
endif()

add_subdirectory(src)
