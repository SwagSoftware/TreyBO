#version 450
#extension GL_GOOGLE_include_directive : require
#include "common.glsl"

layout(location = 0) in vec3 in_pos;
layout(location = 1) in vec3 in_nrm;
// Per instance: the phys_mat44 rows (x, y, z axes and translation, row-vector convention) and a tint.
layout(location = 2) in vec4 i_r0;
layout(location = 3) in vec4 i_r1;
layout(location = 4) in vec4 i_r2;
layout(location = 5) in vec4 i_r3;
layout(location = 6) in vec4 i_tint;

layout(location = 0) out vec3 v_nrm;
layout(location = 1) out vec3 v_world;
layout(location = 2) out vec4 v_tint;

void main()
{
    vec3 w = in_pos.x * i_r0.xyz + in_pos.y * i_r1.xyz + in_pos.z * i_r2.xyz + i_r3.xyz;
    v_nrm = in_nrm.x * i_r0.xyz + in_nrm.y * i_r1.xyz + in_nrm.z * i_r2.xyz;
    v_world = w;
    v_tint = i_tint;
    gl_Position = pc.viewproj * vec4(w, 1.0);
}
