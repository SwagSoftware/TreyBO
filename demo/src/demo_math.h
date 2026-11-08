#pragma once

// Small float math for the camera and the renderer. Physics math stays in TreyBO (phys_vec3 / phys_mat44).
// mat4 is column-major (GLSL layout): m[col * 4 + row]. World is Z up (BO1 convention).

#include <cmath>
#include <cstring>

struct vec3
{
    float x, y, z;
    vec3() : x(0), y(0), z(0) {}
    vec3(float a, float b, float c) : x(a), y(b), z(c) {}
    vec3 operator+(const vec3 &o) const { return vec3(x + o.x, y + o.y, z + o.z); }
    vec3 operator-(const vec3 &o) const { return vec3(x - o.x, y - o.y, z - o.z); }
    vec3 operator*(float s) const { return vec3(x * s, y * s, z * s); }
    vec3 operator-() const { return vec3(-x, -y, -z); }
    vec3 &operator+=(const vec3 &o) { x += o.x; y += o.y; z += o.z; return *this; }
    float &operator[](int i) { return (&x)[i]; }
    const float &operator[](int i) const { return (&x)[i]; }
};

inline float dot(const vec3 &a, const vec3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline vec3 cross(const vec3 &a, const vec3 &b) { return vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
inline float length(const vec3 &a) { return sqrtf(dot(a, a)); }
inline vec3 normalize(const vec3 &a)
{
    const float l = length(a);
    return l > 1e-20f ? a * (1.0f / l) : vec3(0, 0, 1);
}
inline vec3 vmin(const vec3 &a, const vec3 &b) { return vec3(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z)); }
inline vec3 vmax(const vec3 &a, const vec3 &b) { return vec3(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z)); }

struct mat4
{
    float m[16];
    static mat4 identity()
    {
        mat4 r;
        memset(r.m, 0, sizeof(r.m));
        r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0f;
        return r;
    }
    float &at(int row, int col) { return m[col * 4 + row]; }
    float at(int row, int col) const { return m[col * 4 + row]; }
};

inline mat4 operator*(const mat4 &a, const mat4 &b)
{
    mat4 r;
    for (int c = 0; c < 4; ++c)
        for (int rr = 0; rr < 4; ++rr)
        {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k)
                s += a.at(rr, k) * b.at(k, c);
            r.at(rr, c) = s;
        }
    return r;
}

// Right-handed view looking from eye to target.
inline mat4 look_at(const vec3 &eye, const vec3 &target, const vec3 &up)
{
    const vec3 f = normalize(target - eye);
    const vec3 s = normalize(cross(f, up));
    const vec3 u = cross(s, f);
    mat4 r = mat4::identity();
    r.at(0, 0) = s.x; r.at(0, 1) = s.y; r.at(0, 2) = s.z; r.at(0, 3) = -dot(s, eye);
    r.at(1, 0) = u.x; r.at(1, 1) = u.y; r.at(1, 2) = u.z; r.at(1, 3) = -dot(u, eye);
    r.at(2, 0) = -f.x; r.at(2, 1) = -f.y; r.at(2, 2) = -f.z; r.at(2, 3) = dot(f, eye);
    return r;
}

// Vulkan clip space: y down, depth 0 (near) .. 1 (far).
inline mat4 perspective_vk(float fovy, float aspect, float zn, float zf)
{
    const float t = 1.0f / tanf(fovy * 0.5f);
    mat4 r;
    memset(r.m, 0, sizeof(r.m));
    r.at(0, 0) = t / aspect;
    r.at(1, 1) = -t;
    r.at(2, 2) = zf / (zn - zf);
    r.at(2, 3) = zn * zf / (zn - zf);
    r.at(3, 2) = -1.0f;
    return r;
}

// Inverse of a general 4x4 (used for mouse rays).
inline bool invert(const mat4 &in, mat4 *out)
{
    const float *m = in.m;
    float inv[16];
    inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
    inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
    inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
    inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
    inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
    inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
    inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
    inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
    inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] + m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
    inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] - m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
    inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] + m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
    inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] - m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];
    inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] - m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
    inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] + m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
    inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] - m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
    inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] + m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];
    float det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
    if (fabsf(det) < 1e-30f)
        return false;
    det = 1.0f / det;
    for (int i = 0; i < 16; ++i)
        out->m[i] = inv[i] * det;
    return true;
}

inline vec3 transform_point(const mat4 &m, const vec3 &p, float *w_out = nullptr)
{
    const float x = m.at(0, 0) * p.x + m.at(0, 1) * p.y + m.at(0, 2) * p.z + m.at(0, 3);
    const float y = m.at(1, 0) * p.x + m.at(1, 1) * p.y + m.at(1, 2) * p.z + m.at(1, 3);
    const float z = m.at(2, 0) * p.x + m.at(2, 1) * p.y + m.at(2, 2) * p.z + m.at(2, 3);
    const float w = m.at(3, 0) * p.x + m.at(3, 1) * p.y + m.at(3, 2) * p.z + m.at(3, 3);
    if (w_out)
        *w_out = w;
    return fabsf(w) > 1e-30f ? vec3(x / w, y / w, z / w) : vec3(x, y, z);
}
