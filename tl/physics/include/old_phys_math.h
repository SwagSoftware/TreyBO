#pragma once

#include "tl/base/include/tl_defs.h"

inline const float phys_sqr(const float x)
{
	return x * x;
}

// Shared constants. Units are inches: values written "0.01 * 34" are metre tolerances scaled by 34.
const float PHYS_EPS_1E4 = 0.0001f;
const float PHYS_EPS_1E5 = 0.00001f;
const float PHYS_EPS_1E6 = 0.000001f;
const float MIN_NERP_DELTA_T = 0.016666668f; // 1/60: smallest step the position-correction rates divide by
const float PHYS_PULSE_SUM_UNBOUNDED = 10000000.0f; // pulse-sum limit that means "no limit"
const float PHYS_CONTACT_SKIN = 0.34f; // 0.01 * 34: radius added around every geom for GJK
const float PHYS_HACE = 0.51f; // half active-constraint epsilon (see phys_aabb_add_hace)
const float PHYS_DEBUG_VEC_LIMIT = 100000.0f; // rigid_body::debug_check_vec range
const float PHYS_LIMIT_ACTIVE_SIN = 0.043619f; // sin(2.5 deg): hinge / ragdoll limit activation band
const float PHYS_LIMIT_ACTIVE_ANGLE = 0.04363323f; // 2.5 deg in radians

class phys_vec2
{
public:
	float x, y;

	phys_vec2() {}
	phys_vec2(const float _x, const float _y) :
		x(_x), y(_y) {}

	float GetX() const { return x; }
	float GetY() const { return y; }
	void SetX(const float v) { x = v; }
	void SetY(const float v) { y = v; }
};

class TL_ALIGN(16) phys_vec3
{
public:
	// Public: code reads and writes the components (w included, see phys_mat44::fix_w_column) directly.
	float x, y, z, w;

	float GetX() const { return x; }
	float GetY() const { return y; }
	float GetZ() const { return z; }

	void SetX(const float v) { x = v; }
	void SetY(const float v) { y = v; }
	void SetZ(const float v) { z = v; }

	template <typename T> const float & operator[](const T i) const
	{
		PHYS_ASSERT(i >= 0 && i < 3);
		return (&x)[i];
	}
	template <typename T> float & operator[](const T i)
	{
		PHYS_ASSERT(i >= 0 && i < 3);
		return (&x)[i];
	}

	// The constructors leave w untouched.
	phys_vec3() {}
	phys_vec3(const float _x, const float _y, const float _z) :
		x(_x), y(_y), z(_z) {}
	phys_vec3(const float xyz) :
		x(xyz), y(xyz), z(xyz) {}

	// Copy construction is the implicit 16-byte copy (w included); assignment copies xyz only.
	phys_vec3 & operator=(const phys_vec3 & v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
		return *this;
	}
	phys_vec3 & operator+=(const phys_vec3 & v)
	{
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}
	phys_vec3 & operator-=(const phys_vec3 & v)
	{
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
	}
	phys_vec3 & operator*=(const float d)
	{
		x *= d;
		y *= d;
		z *= d;
		return *this;
	}
	phys_vec3 & operator/=(const float d)
	{
		const float d_inv = 1.0f / d;
		x *= d_inv;
		y *= d_inv;
		z *= d_inv;
		return *this;
	}

	phys_vec3 operator-() const { return phys_vec3(-x, -y, -z); }

	friend inline phys_vec3 operator+(const phys_vec3 & a, const phys_vec3 & b)
	{
		return phys_vec3(a.x + b.x, a.y + b.y, a.z + b.z);
	}

	friend inline phys_vec3 operator-(const phys_vec3 & a, const phys_vec3 & b)
	{
		return phys_vec3(a.x - b.x, a.y - b.y, a.z - b.z);
	}

	friend inline phys_vec3 operator*(const phys_vec3 & a, const float d)
	{
		return phys_vec3(d * a.x, d * a.y, d * a.z);
	}

	friend inline phys_vec3 operator*(const float d, const phys_vec3 & a)
	{
		return phys_vec3(d * a.x, d * a.y, d * a.z);
	}

	friend inline phys_vec3 operator/(const phys_vec3 & a, const float d)
	{
		const float d_inv = 1.0f / d;
		return phys_vec3(a.x * d_inv, a.y * d_inv, a.z * d_inv);
	}

	friend inline float AbsSquared(const phys_vec3 & a)
	{
		return a.x * a.x + a.y * a.y + a.z * a.z;
	}

	friend inline float Abs(const phys_vec3 & a)
	{
		return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
	}

	// Asserts on a zero vector (no tolerance branch).
	friend inline phys_vec3 phys_Unitize(const phys_vec3 & a)
	{
		const float na = Abs(a);
		PHYS_ASSERT(na > 0.0f);
		const float oo_na = 1.0f / na;
		return phys_vec3(a.x * oo_na, a.y * oo_na, a.z * oo_na);
	}

	friend inline const float phys_dot(const phys_vec3 & a, const phys_vec3 & b)
	{
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	friend inline phys_vec3 phys_cross(const phys_vec3 & a, const phys_vec3 & b)
	{
		return phys_vec3(a.y * b.z - a.z * b.y,
			a.z * b.x - a.x * b.z,
			a.x * b.y - a.y * b.x);
	}

	friend inline phys_vec3 phys_cross_i(const phys_vec3 & r) { return phys_vec3(0, r.z, -r.y); }
	friend inline phys_vec3 phys_cross_j(const phys_vec3 & r) { return phys_vec3(-r.z, 0, r.x); }
	friend inline phys_vec3 phys_cross_k(const phys_vec3 & r) { return phys_vec3(r.y, -r.x, 0); }

	friend inline phys_vec3 phys_Inv(const phys_vec3 & a) { return phys_vec3(1.0f / a.x, 1.0f / a.y, 1.0f / a.z); }

	friend inline phys_vec3 phys_diag_multiply(const phys_vec3 & v1, const phys_vec3 & v2)
	{
		return phys_vec3(v1.x * v2.x, v1.y * v2.y, v1.z * v2.z);
	}

	friend inline phys_vec3 phys_inv_diag_multiply(const phys_vec3 & v1, const phys_vec3 & v2)
	{
		return phys_vec3(v2.x / v1.x, v2.y / v1.y, v2.z / v1.z);
	}

	friend inline void phys_add_x(phys_vec3 & v1, const phys_vec3 & v2) { v1.x += v2.x; }
	friend inline void phys_add_y(phys_vec3 & v1, const phys_vec3 & v2) { v1.y += v2.y; }
	friend inline void phys_add_z(phys_vec3 & v1, const phys_vec3 & v2) { v1.z += v2.z; }
};

// Per component; written as !(a >= b) so a NaN on either side picks v2 (a plain "<" would pick v1).
inline const phys_vec3 phys_min(const phys_vec3 & v1, const phys_vec3 & v2)
{
	return phys_vec3(!(v2.x >= v1.x) ? v2.x : v1.x,
		!(v2.y >= v1.y) ? v2.y : v1.y,
		!(v2.z >= v1.z) ? v2.z : v1.z);
}

inline const phys_vec3 phys_max(const phys_vec3 & v1, const phys_vec3 & v2)
{
	return phys_vec3(!(v1.x >= v2.x) ? v2.x : v1.x,
		!(v1.y >= v2.y) ? v2.y : v1.y,
		!(v1.z >= v2.z) ? v2.z : v1.z);
}

inline phys_vec3 phys_AbsValue(const phys_vec3 & a)
{
	return phys_vec3(fabsf(a.x), fabsf(a.y), fabsf(a.z));
}

class TL_ALIGN(16) phys_mat44
{
public:
	// Public, as phys_vec3.
	phys_vec3 x, y, z, w;

	phys_mat44() {}
	phys_mat44(const phys_vec3 & x_, const phys_vec3 & y_, const phys_vec3 & z_, const phys_vec3 & w_) :
		x(x_), y(y_), z(z_), w(w_)
	{
	}

	// Copy assignment is implicit: phys_vec3::operator= per row, so xyz of each row.

	phys_vec3 & GetX() { return x; }
	phys_vec3 & GetY() { return y; }
	phys_vec3 & GetZ() { return z; }
	phys_vec3 & GetW() { return w; }

	const phys_vec3 & GetX() const { return x; }
	const phys_vec3 & GetY() const { return y; }
	const phys_vec3 & GetZ() const { return z; }
	const phys_vec3 & GetW() const { return w; }

	void SetX(const phys_vec3 & v) { GetX() = v; }
	void SetY(const phys_vec3 & v) { GetY() = v; }
	void SetZ(const phys_vec3 & v) { GetZ() = v; }
	void SetW(const phys_vec3 & v) { GetW() = v; }

	// No assert (row 3 is valid); the column index is asserted by phys_vec3::operator[].
	const phys_vec3 & operator[](const int i) const { return reinterpret_cast<const phys_vec3 *>(this)[i]; }
	phys_vec3 & operator[](const int i) { return reinterpret_cast<phys_vec3 *>(this)[i]; }

	void fix_w_column()
	{
		x.w = 0.0f;
		y.w = 0.0f;
		z.w = 0.0f;
		w.w = 1.0f;
	}
};

inline const phys_vec3 phys_multiply(const phys_mat44 & mat, const phys_vec3 & v)
{
	return phys_vec3(v.x * mat.x.x + v.y * mat.y.x + v.z * mat.z.x,
		v.x * mat.x.y + v.y * mat.y.y + v.z * mat.z.y,
		v.x * mat.x.z + v.y * mat.y.z + v.z * mat.z.z);
}

inline const phys_vec3 phys_full_multiply(const phys_mat44 & mat, const phys_vec3 & v)
{
	return phys_multiply(mat, v) + mat.GetW();
}

inline const phys_vec3 phys_inv_multiply(const phys_mat44 & mat, const phys_vec3 & v)
{
	return phys_vec3(phys_dot(v, mat.GetX()), phys_dot(v, mat.GetY()), phys_dot(v, mat.GetZ()));
}

inline phys_vec3 phys_full_inv_multiply(const phys_mat44 & mat, const phys_vec3 & v)
{
	return phys_inv_multiply(mat, v - mat.GetW());
}

// The aliased paths copy left with the implicit (full 64-byte) copy constructor.
inline void phys_multiply_mat(phys_mat44 & dest, const phys_mat44 & left, const phys_mat44 & right)
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

inline void phys_full_multiply_mat(phys_mat44 & dest, const phys_mat44 & left, const phys_mat44 & right)
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

inline void phys_inv_multiply_mat(phys_mat44 & dest, const phys_mat44 & left, const phys_mat44 & right)
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

inline void phys_full_inv_multiply_mat(phys_mat44 & dest, const phys_mat44 & left, const phys_mat44 & right)
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
// including PHYS_IDENTITY_MATRIX.w.w.
const phys_vec3 PHYS_ZERO_VEC(0.0f, 0.0f, 0.0f);
const phys_vec3 PHYS_X_VEC(1.0f, 0.0f, 0.0f);
const phys_vec3 PHYS_Y_VEC(0.0f, 1.0f, 0.0f);
const phys_vec3 PHYS_Z_VEC(0.0f, 0.0f, 1.0f);
const phys_mat44 PHYS_IDENTITY_MATRIX(PHYS_X_VEC, PHYS_Y_VEC, PHYS_Z_VEC, PHYS_ZERO_VEC);

// Copies PHYS_IDENTITY_MATRIX (xyz of each row; the w column is left alone). Writing all 16 floats
// would trip the phys_vec3::operator[] assert on m[3][3].
inline void SetIdentity(phys_mat44 & m)
{
	m = PHYS_IDENTITY_MATRIX;
}

inline void FixupWRow(phys_mat44 &) {}

inline void orthonormalize(phys_mat44 * mat)
{
	const float nx = Abs(mat->GetX());
	mat->GetX() *= 1.0f / nx;
	mat->GetY() -= phys_dot(mat->GetY(), mat->GetX()) * mat->GetX();
	const float ny = Abs(mat->GetY());
	mat->GetY() *= 1.0f / ny;
	mat->GetZ() = phys_cross(mat->GetX(), mat->GetY());
}

inline const phys_vec3 construct_orth_ud(const phys_vec3 & ud)
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

inline void phys_swap_float(float & l, float & r)
{
	const float t = l;
	l = r;
	r = t;
}

inline void phys_transpose(phys_mat44 & dest, const phys_mat44 & source)
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

// The aliased path assigns back with operator= (xyz per row).
inline void phys_full_inverse(phys_mat44 & dest, const phys_mat44 & source)
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

inline void make_rotate(phys_mat44 & m, const phys_vec3 & u, const float ca, const float sa)
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

// acos / cos / sin are evaluated in double and rounded to float, so x86 and x64 give the same results.
inline float phys_acosf_safe(const float co_)
{
	if (co_ >= 1.0f)
		return 0.0f;
	else if (co_ <= -1.0f)
		return PHYS_PI_;
	else
		return (float)acos((double)co_);
}

// Rotation by |v| * theta_factor radians about v, clamped to max_rotation_radians.
inline void make_rotate(phys_mat44 * mat, const phys_vec3 & v, const float theta_factor = 1.0f, const float max_rotation_radians = 1000.0f)
{
	float nv = Abs(v);
	if (nv < PHYS_EPS_1E5)
	{
		*mat = PHYS_IDENTITY_MATRIX;
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

inline void make_rotate(phys_mat44 * mat, const phys_vec3 & v1, const phys_vec3 & v2)
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

	template <typename T> const float & operator[](const T i) const { return (&x)[i]; }
	template <typename T> float & operator[](const T i) { return (&x)[i]; }

	Phys_UnitQuaternion() {}
	Phys_UnitQuaternion(const float _x, const float _y, const float _z, const float _w) :
		x(_x), y(_y), z(_z), w(_w) {}
};

// Both branches use the same convention: imag = (M21-M12, M02-M20, M10-M01) / 4w and the
// trace<=0 path solves the same relation for the largest diagonal; the final negation is applied to both.
inline Phys_UnitQuaternion Phys_GetQuaternion(const phys_mat44 & M)
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
// Host interop: converters for float[3] vectors, float[3][3] axes and float[4][3] matrices.
// ---------------------------------------------------------------------------------------------

inline void Phys_NitrousVecToVec3(const phys_vec3 & inVector, float * const outVector)
{
	outVector[0] = inVector.x;
	outVector[1] = inVector.y;
	outVector[2] = inVector.z;
}

inline void Phys_Vec3ToNitrousVec(const float * const inVector, phys_vec3 & outVector)
{
	outVector.x = inVector[0];
	outVector.y = inVector[1];
	outVector.z = inVector[2];
}

inline const phys_vec3 Phys_Vec3ToNitrousVec(const float * const inVector)
{
	return phys_vec3(inVector[0], inVector[1], inVector[2]);
}

inline void Phys_AxisToNitrousMat(const float (*const axis)[3], phys_mat44 & outMat) // w row untouched
{
	Phys_Vec3ToNitrousVec(axis[0], outMat.x);
	Phys_Vec3ToNitrousVec(axis[1], outMat.y);
	Phys_Vec3ToNitrousVec(axis[2], outMat.z);
}

inline void Phys_Mat4ToNitrousMat(const float (*const inMat)[3], phys_mat44 & outMat)
{
	Phys_Vec3ToNitrousVec(inMat[0], outMat.x);
	Phys_Vec3ToNitrousVec(inMat[1], outMat.y);
	Phys_Vec3ToNitrousVec(inMat[2], outMat.z);
	Phys_Vec3ToNitrousVec(inMat[3], outMat.w);
}

inline void Phys_NitrousMat44ToVec33(const phys_mat44 & inMat, float (*const outAxis)[3])
{
	Phys_NitrousVecToVec3(inMat.x, outAxis[0]);
	Phys_NitrousVecToVec3(inMat.y, outAxis[1]);
	Phys_NitrousVecToVec3(inMat.z, outAxis[2]);
}
