#pragma once

// Retail: codsrc\tl\physics\include\old_phys_math.h (header-only; inline copies emitted as COMDATs 0x427xxx-0xa25xxx).
// Layout/API from SM3 old_phys_math.h; bodies are BO1 (retail addresses below).

#include "tl/base/include/tl_defs.h"

inline const float phys_sqr(const float x) { return x * x; }

// Shared constants. Retail inlined all of these as literals; only MIN_NERP_DELTA_T has a retail symbol.
// BO1 works in inches: values written "0.01 * 34" are SM3's metre tolerances scaled by 34.
const float PHYS_EPS_1E4 = 0.0001f; // SM3 "EPSILON"
const float PHYS_EPS_1E5 = 0.00001f;
const float PHYS_EPS_1E6 = 0.000001f;
const float MIN_NERP_DELTA_T = 0.016666668f;        // 1/60: smallest step the position-correction rates divide by
const float PHYS_PULSE_SUM_UNBOUNDED = 10000000.0f; // pulse-sum limit that means "no limit"
const float PHYS_CONTACT_SKIN = 0.34f;              // 0.01 * 34: radius added around every geom for GJK
const float PHYS_HACE = 0.51f;                      // half active-constraint epsilon (see phys_aabb_add_hace)
const float PHYS_DEBUG_VEC_LIMIT = 100000.0f;       // rigid_body::debug_check_vec range
const float PHYS_LIMIT_ACTIVE_SIN = 0.043619f;      // sin(2.5 deg): hinge / ragdoll limit activation band
const float PHYS_LIMIT_ACTIVE_ANGLE = 0.04363323f;  // 2.5 deg in radians

// BO1-only type (no SM3 counterpart). Mangled as class (ABVphys_vec2@@); no out-of-line members in retail.
class phys_vec2
{
public:
    float x, y;

    phys_vec2() {}
    phys_vec2(const float _x, const float _y) : x(_x), y(_y) {}

    float GetX() const { return x; }
    float GetY() const { return y; }
    void SetX(const float v) { x = v; }
    void SetY(const float v) { y = v; }
};

class TL_ALIGN(16) phys_vec3
{
public:
    // TREYBO: SM3 makes these private. BO1 code reads them directly (phys_mat44::fix_w_column 0x8290b0
    // writes x.w, and the debug IDB has no out-of-line GetX), so they are public here.
    float x, y, z, w;

    float GetX() const { return x; }
    float GetY() const { return y; }
    float GetZ() const { return z; }

    void SetX(const float v) { x = v; }
    void SetY(const float v) { y = v; }
    void SetZ(const float v) { z = v; }

    // 0x85b810 (const) / 0x53f7f0
    template <typename T> const float &operator[](const T i) const
    {
        PHYS_ASSERT(i >= 0 && i < 3);
        return (&x)[i];
    }
    template <typename T> float &operator[](const T i)
    {
        PHYS_ASSERT(i >= 0 && i < 3);
        return (&x)[i];
    }

    // The constructors leave w untouched (retail dynamic initializers of PHYS_*_VEC write xyz only).
    // TREYBO FIX(0xc490b0): kisakblack set w = 0.
    phys_vec3() {}
    phys_vec3(const float _x, const float _y, const float _z) : x(_x), y(_y), z(_z) {}
    phys_vec3(const float xyz) : x(xyz), y(xyz), z(xyz) {}

    // Copy construction is the implicit 16-byte copy (w included); assignment copies xyz only (0x43cb60).
    phys_vec3 &operator=(const phys_vec3 &v) { x = v.x; y = v.y; z = v.z; return *this; }
    phys_vec3 &operator+=(const phys_vec3 &v) { x += v.x; y += v.y; z += v.z; return *this; } // 0x9a3be0
    phys_vec3 &operator-=(const phys_vec3 &v) { x -= v.x; y -= v.y; z -= v.z; return *this; } // 0x82f820
    phys_vec3 &operator*=(const float d) { x *= d; y *= d; z *= d; return *this; }            // 0xa0bd30
    phys_vec3 &operator/=(const float d)                                                      // 0xa0bd60
    {
        const float d_inv = 1.0f / d;
        x *= d_inv;
        y *= d_inv;
        z *= d_inv;
        return *this;
    }

    phys_vec3 operator-() const { return phys_vec3(-x, -y, -z); }

    friend inline phys_vec3 operator+(const phys_vec3 &a, const phys_vec3 &b) // 0x4401e0
    { return phys_vec3(a.x + b.x, a.y + b.y, a.z + b.z); }

    friend inline phys_vec3 operator-(const phys_vec3 &a, const phys_vec3 &b) // 0x4a8b90
    { return phys_vec3(a.x - b.x, a.y - b.y, a.z - b.z); }

    friend inline phys_vec3 operator*(const phys_vec3 &a, const float d) // 0x82f880
    { return phys_vec3(d * a.x, d * a.y, d * a.z); }

    friend inline phys_vec3 operator*(const float d, const phys_vec3 &a) // 0x826c30
    { return phys_vec3(d * a.x, d * a.y, d * a.z); }

    friend inline phys_vec3 operator/(const phys_vec3 &a, const float d) // 0x82f8f0
    {
        const float d_inv = 1.0f / d;
        return phys_vec3(a.x * d_inv, a.y * d_inv, a.z * d_inv);
    }

    friend inline float AbsSquared(const phys_vec3 &a) // 0x828770
    { return a.x * a.x + a.y * a.y + a.z * a.z; }

    friend inline float Abs(const phys_vec3 &a) // 0x43cba0
    { return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z); }

    // 0x83fc40. BO1 asserts instead of SM3's "a2 > phys_sqr(0.00001f) ? a / sqrtf(a2) : a" tolerance branch.
    friend inline phys_vec3 phys_Unitize(const phys_vec3 &a)
    {
        const float na = Abs(a);
        PHYS_ASSERT(na > 0.0f);
        const float oo_na = 1.0f / na;
        return phys_vec3(a.x * oo_na, a.y * oo_na, a.z * oo_na);
    }

    friend inline const float phys_dot(const phys_vec3 &a, const phys_vec3 &b) // 0x539c20
    { return a.x * b.x + a.y * b.y + a.z * b.z; }

    friend inline phys_vec3 phys_cross(const phys_vec3 &a, const phys_vec3 &b) // 0x827a60
    {
        return phys_vec3(a.y * b.z - a.z * b.y,
                         a.z * b.x - a.x * b.z,
                         a.x * b.y - a.y * b.x);
    }

    friend inline phys_vec3 phys_cross_i(const phys_vec3 &r) { return phys_vec3(0, r.z, -r.y); }
    friend inline phys_vec3 phys_cross_j(const phys_vec3 &r) { return phys_vec3(-r.z, 0, r.x); }
    friend inline phys_vec3 phys_cross_k(const phys_vec3 &r) { return phys_vec3(r.y, -r.x, 0); }

    friend inline phys_vec3 phys_Inv(const phys_vec3 &a) { return phys_vec3(1.0f / a.x, 1.0f / a.y, 1.0f / a.z); }

    friend inline phys_vec3 phys_diag_multiply(const phys_vec3 &v1, const phys_vec3 &v2)
    { return phys_vec3(v1.x * v2.x, v1.y * v2.y, v1.z * v2.z); }

    friend inline phys_vec3 phys_inv_diag_multiply(const phys_vec3 &v1, const phys_vec3 &v2)
    { return phys_vec3(v2.x / v1.x, v2.y / v1.y, v2.z / v1.z); }

    friend inline void phys_add_x(phys_vec3 &v1, const phys_vec3 &v2) { v1.x += v2.x; }
    friend inline void phys_add_y(phys_vec3 &v1, const phys_vec3 &v2) { v1.y += v2.y; }
    friend inline void phys_add_z(phys_vec3 &v1, const phys_vec3 &v2) { v1.z += v2.z; }
};

// BO1 additions (return by value, retail mangling ?BVphys_vec3@@).
// TREYBO FIX(0x441a25 / 0x441b65): retail is `comiss a, b; jb -> v2` per component, so a NaN on either side picks
// v2; kisakblack's "<" picked v1. Every retail caller (tl and game) calls these out-of-line copies.
inline const phys_vec3 phys_min(const phys_vec3 &v1, const phys_vec3 &v2) // 0x441a00
{
    return phys_vec3(!(v2.x >= v1.x) ? v2.x : v1.x,
                     !(v2.y >= v1.y) ? v2.y : v1.y,
                     !(v2.z >= v1.z) ? v2.z : v1.z);
}

inline const phys_vec3 phys_max(const phys_vec3 &v1, const phys_vec3 &v2) // 0x441b40
{
    return phys_vec3(!(v1.x >= v2.x) ? v2.x : v1.x,
                     !(v1.y >= v2.y) ? v2.y : v1.y,
                     !(v1.z >= v2.z) ? v2.z : v1.z);
}

inline phys_vec3 phys_AbsValue(const phys_vec3 &a) // 0x539c50
{
    return phys_vec3(fabsf(a.x), fabsf(a.y), fabsf(a.z));
}

class TL_ALIGN(16) phys_mat44
{
public:
    // TREYBO: SM3 makes these private; public for the same reason as phys_vec3.
    phys_vec3 x, y, z, w;

    phys_mat44() {}
    phys_mat44(const phys_vec3 &x_, const phys_vec3 &y_, const phys_vec3 &z_, const phys_vec3 &w_) // 0xa0bd90
        : x(x_), y(y_), z(z_), w(w_)
    {
    }

    // Copy assignment is implicit: phys_vec3::operator= per row, so xyz of each row (0x438d10).

    phys_vec3 &GetX() { return x; }
    phys_vec3 &GetY() { return y; }
    phys_vec3 &GetZ() { return z; }
    phys_vec3 &GetW() { return w; }

    const phys_vec3 &GetX() const { return x; }
    const phys_vec3 &GetY() const { return y; }
    const phys_vec3 &GetZ() const { return z; }
    const phys_vec3 &GetW() const { return w; }

    void SetX(const phys_vec3 &v) { GetX() = v; }
    void SetY(const phys_vec3 &v) { GetY() = v; }
    void SetZ(const phys_vec3 &v) { GetZ() = v; }
    void SetW(const phys_vec3 &v) { GetW() = v; }

    // No assert in retail (row 3 is valid); the column index is asserted by phys_vec3::operator[].
    const phys_vec3 &operator[](const int i) const { return reinterpret_cast<const phys_vec3 *>(this)[i]; }
    phys_vec3 &operator[](const int i) { return reinterpret_cast<phys_vec3 *>(this)[i]; }

    void fix_w_column() // 0x8290b0 (BO1 only)
    {
        x.w = 0.0f;
        y.w = 0.0f;
        z.w = 0.0f;
        w.w = 1.0f;
    }
};

inline const phys_vec3 phys_multiply(const phys_mat44 &mat, const phys_vec3 &v) // 0x9a3c40
{
    return phys_vec3(v.x * mat.x.x + v.y * mat.y.x + v.z * mat.z.x,
                     v.x * mat.x.y + v.y * mat.y.y + v.z * mat.z.y,
                     v.x * mat.x.z + v.y * mat.y.z + v.z * mat.z.z);
}

inline const phys_vec3 phys_full_multiply(const phys_mat44 &mat, const phys_vec3 &v) // 0x539ce0
{ return phys_multiply(mat, v) + mat.GetW(); }

inline const phys_vec3 phys_inv_multiply(const phys_mat44 &mat, const phys_vec3 &v) // 0x439ec0
{ return phys_vec3(phys_dot(v, mat.GetX()), phys_dot(v, mat.GetY()), phys_dot(v, mat.GetZ())); }

inline phys_vec3 phys_full_inv_multiply(const phys_mat44 &mat, const phys_vec3 &v) // 0x439e20
{ return phys_inv_multiply(mat, v - mat.GetW()); }

// The aliased paths copy left with the implicit (full 64-byte) copy constructor.
inline void phys_multiply_mat(phys_mat44 &dest, const phys_mat44 &left, const phys_mat44 &right) // 0x8287a0
{
    if (&dest != &left)
    {
        dest.GetX() = phys_multiply(left, right.GetX());
        dest.GetY() = phys_multiply(left, right.GetY());
        dest.GetZ() = phys_multiply(left, right.GetZ());
    }
    else
    {
        const phys_mat44 temp = left;
        phys_multiply_mat(dest, temp, right);
    }
}

inline void phys_full_multiply_mat(phys_mat44 &dest, const phys_mat44 &left, const phys_mat44 &right) // 0x826ca0
{
    if (&dest != &left)
    {
        dest.GetX() = phys_multiply(left, right.GetX());
        dest.GetY() = phys_multiply(left, right.GetY());
        dest.GetZ() = phys_multiply(left, right.GetZ());
        dest.GetW() = phys_full_multiply(left, right.GetW());
    }
    else
    {
        const phys_mat44 temp = left;
        phys_full_multiply_mat(dest, temp, right);
    }
}

inline void phys_inv_multiply_mat(phys_mat44 &dest, const phys_mat44 &left, const phys_mat44 &right) // 0x8541b0
{
    if (&dest != &left)
    {
        dest[0] = phys_inv_multiply(left, right.GetX());
        dest[1] = phys_inv_multiply(left, right.GetY());
        dest[2] = phys_inv_multiply(left, right.GetZ());
    }
    else
    {
        const phys_mat44 temp = left;
        phys_inv_multiply_mat(dest, temp, right);
    }
}

inline void phys_full_inv_multiply_mat(phys_mat44 &dest, const phys_mat44 &left, const phys_mat44 &right) // 0xa0be00
{
    if (&dest != &left)
    {
        dest[0] = phys_inv_multiply(left, right.GetX());
        dest[1] = phys_inv_multiply(left, right.GetY());
        dest[2] = phys_inv_multiply(left, right.GetZ());
        dest[3] = phys_full_inv_multiply(left, right.GetW());
    }
    else
    {
        const phys_mat44 temp = left;
        phys_full_inv_multiply_mat(dest, temp, right);
    }
}

// Static storage is zeroed before these dynamic initializers run, so every w is 0,
// including PHYS_IDENTITY_MATRIX.w.w (retail 0xc49070 / 0xc490b0 / 0xc49170).
const phys_vec3 PHYS_ZERO_VEC(0.0f, 0.0f, 0.0f);
const phys_vec3 PHYS_X_VEC(1.0f, 0.0f, 0.0f);
const phys_vec3 PHYS_Y_VEC(0.0f, 1.0f, 0.0f);
const phys_vec3 PHYS_Z_VEC(0.0f, 0.0f, 1.0f);
const phys_mat44 PHYS_IDENTITY_MATRIX(PHYS_X_VEC, PHYS_Y_VEC, PHYS_Z_VEC, PHYS_ZERO_VEC); // BO1 only

// 0xa124c0. BO1 copies PHYS_IDENTITY_MATRIX (xyz of each row; the w column is left alone).
// SM3 wrote all 16 floats, which would trip the phys_vec3::operator[] assert on m[3][3].
inline void SetIdentity(phys_mat44 &m) { m = PHYS_IDENTITY_MATRIX; }

inline void FixupWRow(phys_mat44 &) {}

inline void orthonormalize(phys_mat44 *mat) // 0xa257c0
{
    const float nx = Abs(mat->GetX());
    mat->GetX() *= 1.0f / nx;
    mat->GetY() -= phys_dot(mat->GetY(), mat->GetX()) * mat->GetX();
    const float ny = Abs(mat->GetY());
    mat->GetY() *= 1.0f / ny;
    mat->GetZ() = phys_cross(mat->GetX(), mat->GetY());
}

inline const phys_vec3 construct_orth_ud(const phys_vec3 &ud) // 0xa174a0
{
    const float nud = Abs(ud);
    PHYS_ASSERT(fabsf(nud - 1.0f) < .0001f);

    phys_vec3 orth_ud = phys_cross_i(ud);
    float north_ud = Abs(orth_ud);
    if (north_ud < PHYS_EPS_1E4)
    {
        orth_ud = phys_cross_j(ud);
        north_ud = Abs(orth_ud);
        if (north_ud < PHYS_EPS_1E4)
        {
            orth_ud = phys_cross_k(ud);
            north_ud = Abs(orth_ud);
            if (north_ud < PHYS_EPS_1E4)
            {
                PHYS_ASSERT_MSG(0, "not possible");
            }
        }
    }
    orth_ud *= 1.0f / north_ud;
    return orth_ud;
}

inline void phys_swap_float(float &l, float &r)
{
    const float t = l;
    l = r;
    r = t;
}

inline void phys_transpose(phys_mat44 &dest, const phys_mat44 &source) // 0x539d90
{
    if (&dest != &source)
    {
        dest[0][0] = source[0][0];
        dest[0][1] = source[1][0];
        dest[0][2] = source[2][0];

        dest[1][0] = source[0][1];
        dest[1][1] = source[1][1];
        dest[1][2] = source[2][1];

        dest[2][0] = source[0][2];
        dest[2][1] = source[1][2];
        dest[2][2] = source[2][2];
    }
    else
    {
        phys_swap_float(dest[0][1], dest[1][0]);
        phys_swap_float(dest[0][2], dest[2][0]);
        phys_swap_float(dest[1][2], dest[2][1]);
    }
}

// 0x84a230 (SM3 name: full_inverse). The aliased path assigns back with operator= (xyz per row).
inline void phys_full_inverse(phys_mat44 &dest, const phys_mat44 &source)
{
    if (&dest != &source)
    {
        phys_transpose(dest, source);
        dest[3] = -phys_multiply(dest, source[3]);
    }
    else
    {
        phys_mat44 temp;
        phys_full_inverse(temp, source);
        dest = temp;
    }
}

// 0x8289e0. BO1 replaced SM3's explicit w column / w row writes (m[0][3]..m[3][3]) with fix_w_column.
inline void make_rotate(phys_mat44 &m, const phys_vec3 &u, const float ca, const float sa)
{
    const float omca = 1.0f - ca;
    const float xx = u[0] * u[0] * omca;
    const float xy = u[0] * u[1] * omca;
    const float xz = u[0] * u[2] * omca;
    const float yy = u[1] * u[1] * omca;
    const float yz = u[1] * u[2] * omca;
    const float zz = u[2] * u[2] * omca;
    const float xsa = u[0] * sa;
    const float ysa = u[1] * sa;
    const float zsa = u[2] * sa;

    m[0][0] = xx + ca;
    m[1][0] = xy - zsa;
    m[2][0] = xz + ysa;
    m[3][0] = 0.0f;

    m[0][1] = xy + zsa;
    m[1][1] = yy + ca;
    m[2][1] = yz - xsa;
    m[3][1] = 0.0f;

    m[0][2] = xz - ysa;
    m[1][2] = yz + xsa;
    m[2][2] = zz + ca;
    m[3][2] = 0.0f;

    m.fix_w_column();
}

const float PHYS_PI_ = 3.1415927f;
const float PHYS_PI_TIMES_2 = PHYS_PI_ * 2.0f;
const float PHYS_PI_OVER_2 = PHYS_PI_ / 2.0f;

// Inlined in nuge::calc_velocities 0xa1bfb0.
// TREYBO: retail x86 acosf/cosf/sinf were the CRT double routines (__CIacos, __libm_sse2_cos/sin)
// rounded to float; call the double versions so x64 gives the same results.
inline float phys_acosf_safe(const float co_)
{
    if (co_ >= 1.0f)
        return 0.0f;
    else if (co_ <= -1.0f)
        return PHYS_PI_;
    else
        return (float)acos((double)co_);
}

// 0x8288c0. BO1 adds the max_rotation_radians clamp. Defaults: theta_factor from SM3;
// TREYBO: max_rotation_radians = 1000.0f inferred (the optimized tl callers all pass 1000.0f).
inline void make_rotate(phys_mat44 *mat, const phys_vec3 &v, const float theta_factor = 1.0f, const float max_rotation_radians = 1000.0f)
{
    float nv = Abs(v);
    if (nv < PHYS_EPS_1E5)
    {
        *mat = PHYS_IDENTITY_MATRIX; // BO1 assigns here; SM3 called SetIdentity
    }
    else
    {
        const phys_vec3 ud = (1.0f / nv) * v;
        nv *= theta_factor;
        if (nv > max_rotation_radians)
            nv = max_rotation_radians;
        make_rotate(*mat, ud, (float)cos((double)nv), (float)sin((double)nv));
    }
}

inline void make_rotate(phys_mat44 *mat, const phys_vec3 &v1, const phys_vec3 &v2) // 0x85ac80
{
    phys_vec3 ud = phys_cross(v1, v2);
    float si_ = AbsSquared(ud);
    if (si_ < PHYS_EPS_1E5)
    {
        *mat = PHYS_IDENTITY_MATRIX;
    }
    else
    {
        si_ = sqrtf(si_);
        ud *= 1.0f / si_;
        const float co_ = phys_dot(v1, v2);
        const float len = sqrtf(co_ * co_ + si_ * si_);
        make_rotate(*mat, ud, co_ / len, si_ / len);
    }
}

class Phys_UnitQuaternion
{
protected:
    float x, y, z, w;

public:
    float GetX() const { return x; }
    float GetY() const { return y; }
    float GetZ() const { return z; }
    float GetW() const { return w; }
    const phys_vec3 GetImag() const { return phys_vec3(x, y, z); }
    float GetReal() const { return w; }

    template <typename T> const float &operator[](const T i) const { return (&x)[i]; }
    template <typename T> float &operator[](const T i) { return (&x)[i]; }

    Phys_UnitQuaternion() {}
    Phys_UnitQuaternion(const float _x, const float _y, const float _z, const float _w) : x(_x), y(_y), z(_z), w(_w) {}
};

// 0xa1b3b0. Both branches use the same convention: imag = (M21-M12, M02-M20, M10-M01) / 4w and the
// trace<=0 path solves the same relation for the largest diagonal; the final negation is applied to both.
inline Phys_UnitQuaternion Phys_GetQuaternion(const phys_mat44 &M)
{
    int i, j, k;
    float tr, s, q[4];
    float a, b, c, d;

    tr = M[0][0] + M[1][1] + M[2][2];
    if (tr > 0.0f)
    {
        s = sqrtf(tr + 1.0f);
        a = s * 0.5f;
        s = 0.5f / s;
        b = (M[2][1] - M[1][2]) * s;
        c = (M[0][2] - M[2][0]) * s;
        d = (M[1][0] - M[0][1]) * s;
    }
    else
    {
        i = 0;
        j = 1;
        k = 2;
        if (M[1][1] > M[0][0])
        {
            i = 1;
            j = 2;
            k = 0;
        }
        if (M[2][2] > M[i][i])
        {
            i = 2;
            j = 0;
            k = 1;
        }

        s = sqrtf(M[i][i] - M[j][j] - M[k][k] + 1.0f);
        q[i + 1] = s * 0.5f;
        if (s != 0.0f)
            s = 0.5f / s;
        q[0] = (M[k][j] - M[j][k]) * s;
        q[j + 1] = (M[j][i] + M[i][j]) * s;
        q[k + 1] = (M[k][i] + M[i][k]) * s;

        a = q[0];
        b = q[1];
        c = q[2];
        d = q[3];
    }
    b = -b;
    c = -c;
    d = -d;
    return Phys_UnitQuaternion(b, c, d, a);
}

// ---------------------------------------------------------------------------------------------
// Host interop. TREYBO: CoD-convention converters (float[3] vectors, float[3][3] axis, float[4][3]
// matrix). Retail defines these game-side; the bodies match retail but they are not tl code.
// ---------------------------------------------------------------------------------------------

inline void Phys_NitrousVecToVec3(const phys_vec3 &inVector, float *const outVector) // 0x43cc00
{
    outVector[0] = inVector.x;
    outVector[1] = inVector.y;
    outVector[2] = inVector.z;
}

inline void Phys_Vec3ToNitrousVec(const float *const inVector, phys_vec3 &outVector) // 0x4277e0
{
    outVector.x = inVector[0];
    outVector.y = inVector[1];
    outVector.z = inVector[2];
}

inline const phys_vec3 Phys_Vec3ToNitrousVec(const float *const inVector) // 0x4399d0
{
    return phys_vec3(inVector[0], inVector[1], inVector[2]);
}

inline void Phys_AxisToNitrousMat(const float (*const axis)[3], phys_mat44 &outMat) // 0x4a8c00 (w row untouched)
{
    Phys_Vec3ToNitrousVec(axis[0], outMat.x);
    Phys_Vec3ToNitrousVec(axis[1], outMat.y);
    Phys_Vec3ToNitrousVec(axis[2], outMat.z);
}

inline void Phys_Mat4ToNitrousMat(const float (*const inMat)[3], phys_mat44 &outMat) // 0x8227f0
{
    Phys_Vec3ToNitrousVec(inMat[0], outMat.x);
    Phys_Vec3ToNitrousVec(inMat[1], outMat.y);
    Phys_Vec3ToNitrousVec(inMat[2], outMat.z);
    Phys_Vec3ToNitrousVec(inMat[3], outMat.w);
}

inline void Phys_NitrousMat44ToVec33(const phys_mat44 &inMat, float (*const outAxis)[3]) // 0x829110
{
    Phys_NitrousVecToVec3(inMat.x, outAxis[0]);
    Phys_NitrousVecToVec3(inMat.y, outAxis[1]);
    Phys_NitrousVecToVec3(inMat.z, outAxis[2]);
}
