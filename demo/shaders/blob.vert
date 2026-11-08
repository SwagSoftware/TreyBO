#version 450
#extension GL_GOOGLE_include_directive : require
#include "common.glsl"

// Blob shadow under each body: a ground quad at the body's xy, fading with height.
layout(location = 2) in vec4 i_r0;
layout(location = 3) in vec4 i_r1;
layout(location = 4) in vec4 i_r2;
layout(location = 5) in vec4 i_r3;
layout(location = 6) in vec4 i_tint;

layout(location = 0) out vec2 v_uv;
layout(location = 1) out float v_alpha;

void main()
{
    const vec2 corners[6] = vec2[](vec2(-1, -1), vec2(1, -1), vec2(1, 1), vec2(-1, -1), vec2(1, 1), vec2(-1, 1));
    vec2 c = corners[gl_VertexIndex];
    float h = max(i_r3.z, 0.0);
    float r = pc.params.y * (1.0 + h * 0.0015);
    // Shift the blob away from the light a little, as a real shadow would be.
    vec2 offs = -pc.light_dir.xy / max(-pc.light_dir.z, 0.2) * h * 0.25;
    vec3 w = vec3(i_r3.xy + offs + c * r, 0.35);
    v_uv = c;
    v_alpha = 0.55 * clamp(1.0 - h / 700.0, 0.0, 1.0);
    gl_Position = pc.viewproj * vec4(w, 1.0);
}
