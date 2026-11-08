#pragma once

// Retail path: codsrc\tl\physics\include\phys_version.h. No version number survives in CoDMPServer.exe
// (BO1 has no phys_get_version symbol and no version string). The values below are the SM3 ones
// (physics_system.h: phys_get_version() returns (0 << 16) | (5 << 8) | 0). BO1's real number is unknown.

#define PHYS_VERSION_MAJOR 0
#define PHYS_VERSION_MINOR 5
#define PHYS_VERSION_PATCH 0

#define PHYS_VERSION ((PHYS_VERSION_MAJOR << 16) | (PHYS_VERSION_MINOR << 8) | (PHYS_VERSION_PATCH))
