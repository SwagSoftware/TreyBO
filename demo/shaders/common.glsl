// Shared push constants for every treybo_demo pipeline (112 bytes, under the 128-byte guaranteed minimum).
layout(push_constant) uniform push_constants_t
{
    mat4 viewproj;  // world -> clip (Vulkan clip space: y down, z in [0, 1])
    vec4 light_dir; // xyz: direction the light travels (normalised), w: ambient strength
    vec4 cam_pos;   // xyz: eye position, w: fog distance
    vec4 params;    // x: ground half size, y: blob radius, z: ground grid size, w: unused
} pc;
