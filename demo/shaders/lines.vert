#version 450
#extension GL_GOOGLE_include_directive : require
#include "common.glsl"

layout(location = 0) in vec3 in_pos;
layout(location = 2) in vec4 i_r0;
layout(location = 3) in vec4 i_r1;
layout(location = 4) in vec4 i_r2;
layout(location = 5) in vec4 i_r3;
layout(location = 6) in vec4 i_tint;

void main()
{
    vec3 w = in_pos.x * i_r0.xyz + in_pos.y * i_r1.xyz + in_pos.z * i_r2.xyz + i_r3.xyz;
    gl_Position = pc.viewproj * vec4(w, 1.0);
    gl_Position.z -= 0.0004 * gl_Position.w; // pull towards the eye so the overlay wins against the mesh
}
