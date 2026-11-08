#version 450
#extension GL_GOOGLE_include_directive : require
#include "common.glsl"

layout(location = 0) in vec3 v_world;
layout(location = 0) out vec4 o_color;

void main()
{
    vec2 g = v_world.xy / pc.params.z;
    vec2 cell = floor(g);
    float checker = mod(cell.x + cell.y, 2.0);
    vec2 f = abs(fract(g) - 0.5);
    float line = 1.0 - smoothstep(0.47, 0.5, max(f.x, f.y));
    vec3 base = mix(vec3(0.50, 0.52, 0.55), vec3(0.56, 0.58, 0.61), checker);
    base = mix(base * 0.8, base, line);
    float diff = max(-pc.light_dir.z, 0.0);
    vec3 c = base * (pc.light_dir.w * 0.8 + diff * 0.75);
    float fog = clamp(length(pc.cam_pos.xyz - v_world) / pc.cam_pos.w, 0.0, 1.0);
    o_color = vec4(mix(c, vec3(0.62, 0.68, 0.76), fog * fog), 1.0);
}
