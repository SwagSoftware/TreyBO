#version 450
#extension GL_GOOGLE_include_directive : require
#include "common.glsl"

layout(location = 0) out vec3 v_world;

void main()
{
    const vec2 corners[6] = vec2[](vec2(-1, -1), vec2(1, -1), vec2(1, 1), vec2(-1, -1), vec2(1, 1), vec2(-1, 1));
    vec3 w = vec3(corners[gl_VertexIndex] * pc.params.x, 0.0);
    v_world = w;
    gl_Position = pc.viewproj * vec4(w, 1.0);
}
