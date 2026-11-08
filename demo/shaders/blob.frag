#version 450

layout(location = 0) in vec2 v_uv;
layout(location = 1) in float v_alpha;
layout(location = 0) out vec4 o_color;

void main()
{
    float d = length(v_uv);
    float a = v_alpha * (1.0 - smoothstep(0.25, 1.0, d));
    o_color = vec4(0.0, 0.0, 0.0, a);
}
