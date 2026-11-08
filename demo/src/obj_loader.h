#pragma once

// Minimal Wavefront OBJ loader: v / vt / vn, faces of any vertex count (fan triangulated), negative indices,
// "v", "v/vt", "v//vn" and "v/vt/vn" corners. Objects, groups, materials and smoothing groups are ignored.

#include <cstdint>
#include <string>
#include <vector>

struct obj_vertex
{
    float pos[3];
    float nrm[3];
    float uv[2];
};

struct obj_mesh
{
    std::vector<float> positions;     // unique "v" positions, xyz triplets (for hulls / mass properties)
    std::vector<obj_vertex> vertices; // render vertices, one per distinct (v, vt, vn) corner
    std::vector<uint32_t> indices;    // triangle list into vertices
    std::vector<uint32_t> position_indices; // triangle list into positions (closed-mesh integrals)
    int uv_count = 0;
    int normal_count = 0;
    int face_count = 0; // faces in the file (before triangulation)
    std::string object_name;
};

// Returns false and fills *error on failure. Faces without normals get flat normals.
bool obj_load(const char *path, obj_mesh *out, std::string *error);
