#version 450
#extension GL_GOOGLE_include_directive : require
#include "common.glsl"

layout(location = 0) in vec3 v_nrm;
layout(location = 1) in vec3 v_world;
layout(location = 2) in vec4 v_tint;
layout(location = 0) out vec4 o_color;

void main()
{
    vec3 n = normalize(v_nrm);
    vec3 l = -pc.light_dir.xyz;
    vec3 v = normalize(pc.cam_pos.xyz - v_world);
    float diff = max(dot(n, l), 0.0);
    // Hemisphere ambient (sky above, ground bounce below).
    float hemi = 0.5 + 0.5 * n.z;
    vec3 ambient = pc.light_dir.w * mix(vec3(0.45, 0.42, 0.40), vec3(0.75, 0.82, 0.95), hemi);
    vec3 h = normalize(l + v);
    float spec = pow(max(dot(n, h), 0.0), 48.0) * 0.35;
    // Cheap contact darkening: surfaces close to the ground are in the pile's shadow.
    float ao = mix(0.55, 1.0, clamp(v_world.z / 48.0, 0.0, 1.0));
    vec3 c = v_tint.rgb * (ambient * ao + diff * vec3(1.0, 0.97, 0.9) * mix(0.6, 1.0, ao)) + spec;
    float fog = clamp(length(pc.cam_pos.xyz - v_world) / pc.cam_pos.w, 0.0, 1.0);
    o_color = vec4(mix(c, vec3(0.62, 0.68, 0.76), fog * fog), 1.0);
}
