// Tangent - minimal linear algebra for real-time 3D.
// Column-major matrices, matching OpenGL memory layout so Mat4 can be handed
// straight to glUniformMatrix4fv with transpose = GL_FALSE.
#pragma once

#include <cmath>
#include <algorithm>
#include <limits>

namespace tg {

// Geometry is computed in double precision.
//
// Booleans are the reason. Their robustness is dominated by how reliably a
// point can be classified against a surface, and the hard cases are always
// near-degenerate: faces that are almost coplanar, an edge that passes almost
// exactly through a vertex. At float32 a 100mm part resolves to about 1e-4 mm,
// which is not enough headroom for those decisions, and the failure mode is a
// boolean that produces a leaking or non-manifold result.
//
// The GPU still wants float32, so conversion happens at that boundary only:
// Shader::set for uniforms and GpuMesh::upload for vertex data.
using Real = double;

inline constexpr Real kPi      = 3.14159265358979323846f;
inline constexpr Real kTwoPi   = 6.28318530717958647692f;
inline constexpr Real kHalfPi  = 1.57079632679489661923f;
inline constexpr Real kDeg2Rad = kPi / 180.0f;
inline constexpr Real kRad2Deg = 180.0f / kPi;
inline constexpr Real kEps     = 1e-6f;

inline Real radians(Real d) { return d * kDeg2Rad; }
inline Real degrees(Real r) { return r * kRad2Deg; }
inline Real clampf(Real v, Real lo, Real hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline Real lerpf(Real a, Real b, Real t) { return a + (b - a) * t; }
inline Real sign(Real v) { return v < 0.0f ? -1.0f : (v > 0.0f ? 1.0f : 0.0f); }

// Rounds a rough magnitude to the nearest "nice" value: 1, 2 or 5 times a
// power of ten. Used to turn a desired snap distance in millimetres into one a
// person would actually choose -- 0.5, 1, 2, 5, 10 -- rather than 0.734.
inline Real niceStep(Real approx) {
    if (!(approx > 0.0f)) return 0.0f;
    const Real e = std::floor(std::log10(approx));
    const Real base = std::pow(10.0f, e);
    const Real m = approx / base;             // in [1, 10)
    const Real mult = m < 1.5f ? 1.0f : (m < 3.5f ? 2.0f : (m < 7.5f ? 5.0f : 10.0f));
    return base * mult;
}

// ---------------------------------------------------------------- Vec2 -----
struct Vec2 {
    Real x = 0, y = 0;
    constexpr Vec2() = default;
    constexpr Vec2(Real x_, Real y_) : x(x_), y(y_) {}
    explicit constexpr Vec2(Real s) : x(s), y(s) {}
    Real& operator[](int i) { return (&x)[i]; }
    const Real& operator[](int i) const { return (&x)[i]; }
};
inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator-(Vec2 a)         { return {-a.x, -a.y}; }
inline Vec2 operator*(Vec2 a, Real s) { return {a.x * s, a.y * s}; }
inline Vec2 operator*(Real s, Vec2 a) { return a * s; }
inline Vec2 operator/(Vec2 a, Real s) { return {a.x / s, a.y / s}; }
inline Vec2& operator+=(Vec2& a, Vec2 b) { a = a + b; return a; }
inline Vec2& operator-=(Vec2& a, Vec2 b) { a = a - b; return a; }
inline Vec2& operator*=(Vec2& a, Real s) { a = a * s; return a; }
inline Real dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline Real length(Vec2 a) { return std::sqrt(dot(a, a)); }
inline Real lengthSq(Vec2 a) { return dot(a, a); }
inline Vec2 normalize(Vec2 a) { Real l = length(a); return l > kEps ? a / l : Vec2{}; }

// ---------------------------------------------------------------- Vec3 -----
struct Vec3 {
    Real x = 0, y = 0, z = 0;
    constexpr Vec3() = default;
    constexpr Vec3(Real x_, Real y_, Real z_) : x(x_), y(y_), z(z_) {}
    explicit constexpr Vec3(Real s) : x(s), y(s), z(s) {}
    Real& operator[](int i) { return (&x)[i]; }
    const Real& operator[](int i) const { return (&x)[i]; }
};
inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator-(Vec3 a)         { return {-a.x, -a.y, -a.z}; }
inline Vec3 operator*(Vec3 a, Real s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3 operator*(Real s, Vec3 a) { return a * s; }
inline Vec3 operator*(Vec3 a, Vec3 b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
inline Vec3 operator/(Vec3 a, Real s) { return {a.x / s, a.y / s, a.z / s}; }
inline Vec3 operator/(Vec3 a, Vec3 b) { return {a.x / b.x, a.y / b.y, a.z / b.z}; }
inline Vec3& operator+=(Vec3& a, Vec3 b) { a = a + b; return a; }
inline Vec3& operator-=(Vec3& a, Vec3 b) { a = a - b; return a; }
inline Vec3& operator*=(Vec3& a, Real s) { a = a * s; return a; }
inline Vec3& operator*=(Vec3& a, Vec3 b) { a = a * b; return a; }
inline bool operator==(Vec3 a, Vec3 b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
inline bool operator!=(Vec3 a, Vec3 b) { return !(a == b); }

inline Real dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline Real lengthSq(Vec3 a) { return dot(a, a); }
inline Real length(Vec3 a) { return std::sqrt(dot(a, a)); }
inline Real distance(Vec3 a, Vec3 b) { return length(b - a); }
inline Vec3 normalize(Vec3 a) { Real l = length(a); return l > kEps ? a / l : Vec3{}; }
inline Vec3 minv(Vec3 a, Vec3 b) { return {std::min(a.x,b.x), std::min(a.y,b.y), std::min(a.z,b.z)}; }
inline Vec3 maxv(Vec3 a, Vec3 b) { return {std::max(a.x,b.x), std::max(a.y,b.y), std::max(a.z,b.z)}; }
inline Vec3 absv(Vec3 a) { return {std::fabs(a.x), std::fabs(a.y), std::fabs(a.z)}; }
inline Vec3 lerp(Vec3 a, Vec3 b, Real t) { return a + (b - a) * t; }
inline Real maxComponent(Vec3 a) { return std::max(a.x, std::max(a.y, a.z)); }
inline int maxAxis(Vec3 a) { return a.x > a.y ? (a.x > a.z ? 0 : 2) : (a.y > a.z ? 1 : 2); }

// Any unit vector perpendicular to n. Branch-free and numerically safe.
inline Vec3 perpendicular(Vec3 n) {
    Vec3 a = std::fabs(n.x) > 0.9f ? Vec3{0, 1, 0} : Vec3{1, 0, 0};
    return normalize(cross(n, a));
}

// ---------------------------------------------------------------- Vec4 -----
struct Vec4 {
    Real x = 0, y = 0, z = 0, w = 0;
    constexpr Vec4() = default;
    constexpr Vec4(Real x_, Real y_, Real z_, Real w_) : x(x_), y(y_), z(z_), w(w_) {}
    constexpr Vec4(Vec3 v, Real w_) : x(v.x), y(v.y), z(v.z), w(w_) {}
    explicit constexpr Vec4(Real s) : x(s), y(s), z(s), w(s) {}
    Real& operator[](int i) { return (&x)[i]; }
    const Real& operator[](int i) const { return (&x)[i]; }
    Vec3 xyz() const { return {x, y, z}; }
};
inline Vec4 operator+(Vec4 a, Vec4 b) { return {a.x+b.x, a.y+b.y, a.z+b.z, a.w+b.w}; }
inline Vec4 operator-(Vec4 a, Vec4 b) { return {a.x-b.x, a.y-b.y, a.z-b.z, a.w-b.w}; }
inline Vec4 operator*(Vec4 a, Real s) { return {a.x*s, a.y*s, a.z*s, a.w*s}; }
inline Vec4 operator*(Real s, Vec4 a) { return a * s; }
inline Real dot(Vec4 a, Vec4 b) { return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w; }

// ---------------------------------------------------------------- Mat4 -----
// Column-major: col[c] is the c-th basis column. M*v = sum_c col[c] * v[c].
struct Mat4 {
    Vec4 col[4];

    Mat4() : col{{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}} {}
    Mat4(Vec4 c0, Vec4 c1, Vec4 c2, Vec4 c3) : col{c0, c1, c2, c3} {}

    static Mat4 identity() { return Mat4(); }
    static Mat4 zero() { return Mat4({0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}); }

    Vec4& operator[](int c) { return col[c]; }
    const Vec4& operator[](int c) const { return col[c]; }
    const Real* data() const { return &col[0].x; }

    // Column-major float copy for OpenGL, which has no double-precision
    // uniform path worth using here.
    void toFloats(float out[16]) const {
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r)
                out[c * 4 + r] = static_cast<float>(col[c][r]);
    }

    Vec3 translation() const { return col[3].xyz(); }
    // Upper-left 3x3 basis vectors (object axes in world space).
    Vec3 axis(int i) const { return col[i].xyz(); }
};

inline Mat4 operator*(const Mat4& a, const Mat4& b) {
    Mat4 r = Mat4::zero();
    for (int c = 0; c < 4; ++c)
        for (int k = 0; k < 4; ++k)
            r.col[c] = r.col[c] + a.col[k] * b.col[c][k];
    return r;
}
inline Vec4 operator*(const Mat4& m, Vec4 v) {
    return m.col[0]*v.x + m.col[1]*v.y + m.col[2]*v.z + m.col[3]*v.w;
}
// Transform a position (w = 1) and a direction (w = 0).
inline Vec3 transformPoint(const Mat4& m, Vec3 p)  { return (m * Vec4(p, 1.0f)).xyz(); }
inline Vec3 transformVector(const Mat4& m, Vec3 v) { return (m * Vec4(v, 0.0f)).xyz(); }

inline Mat4 translate(Vec3 t) {
    Mat4 m; m.col[3] = Vec4(t, 1.0f); return m;
}
inline Mat4 scaleMat(Vec3 s) {
    Mat4 m;
    m.col[0] = {s.x,0,0,0}; m.col[1] = {0,s.y,0,0}; m.col[2] = {0,0,s.z,0};
    return m;
}
inline Mat4 rotateAxis(Vec3 axis, Real angle) {
    Vec3 a = normalize(axis);
    Real c = std::cos(angle), s = std::sin(angle), t = 1.0f - c;
    Mat4 m;
    m.col[0] = {t*a.x*a.x + c,     t*a.x*a.y + s*a.z, t*a.x*a.z - s*a.y, 0};
    m.col[1] = {t*a.x*a.y - s*a.z, t*a.y*a.y + c,     t*a.y*a.z + s*a.x, 0};
    m.col[2] = {t*a.x*a.z + s*a.y, t*a.y*a.z - s*a.x, t*a.z*a.z + c,     0};
    return m;
}

inline Mat4 transpose(const Mat4& m) {
    Mat4 r;
    for (int c = 0; c < 4; ++c)
        for (int j = 0; j < 4; ++j)
            r.col[c][j] = m.col[j][c];
    return r;
}

// General 4x4 inverse (cofactor expansion). Returns identity if singular.
inline Mat4 inverse(const Mat4& m) {
    const Real* a = m.data();
    Real inv[16];
    inv[0]  =  a[5]*a[10]*a[15] - a[5]*a[11]*a[14] - a[9]*a[6]*a[15]
             + a[9]*a[7]*a[14]  + a[13]*a[6]*a[11] - a[13]*a[7]*a[10];
    inv[4]  = -a[4]*a[10]*a[15] + a[4]*a[11]*a[14] + a[8]*a[6]*a[15]
             - a[8]*a[7]*a[14]  - a[12]*a[6]*a[11] + a[12]*a[7]*a[10];
    inv[8]  =  a[4]*a[9]*a[15]  - a[4]*a[11]*a[13] - a[8]*a[5]*a[15]
             + a[8]*a[7]*a[13]  + a[12]*a[5]*a[11] - a[12]*a[7]*a[9];
    inv[12] = -a[4]*a[9]*a[14]  + a[4]*a[10]*a[13] + a[8]*a[5]*a[14]
             - a[8]*a[6]*a[13]  - a[12]*a[5]*a[10] + a[12]*a[6]*a[9];
    inv[1]  = -a[1]*a[10]*a[15] + a[1]*a[11]*a[14] + a[9]*a[2]*a[15]
             - a[9]*a[3]*a[14]  - a[13]*a[2]*a[11] + a[13]*a[3]*a[10];
    inv[5]  =  a[0]*a[10]*a[15] - a[0]*a[11]*a[14] - a[8]*a[2]*a[15]
             + a[8]*a[3]*a[14]  + a[12]*a[2]*a[11] - a[12]*a[3]*a[10];
    inv[9]  = -a[0]*a[9]*a[15]  + a[0]*a[11]*a[13] + a[8]*a[1]*a[15]
             - a[8]*a[3]*a[13]  - a[12]*a[1]*a[11] + a[12]*a[3]*a[9];
    inv[13] =  a[0]*a[9]*a[14]  - a[0]*a[10]*a[13] - a[8]*a[1]*a[14]
             + a[8]*a[2]*a[13]  + a[12]*a[1]*a[10] - a[12]*a[2]*a[9];
    inv[2]  =  a[1]*a[6]*a[15]  - a[1]*a[7]*a[14]  - a[5]*a[2]*a[15]
             + a[5]*a[3]*a[14]  + a[13]*a[2]*a[7]  - a[13]*a[3]*a[6];
    inv[6]  = -a[0]*a[6]*a[15]  + a[0]*a[7]*a[14]  + a[4]*a[2]*a[15]
             - a[4]*a[3]*a[14]  - a[12]*a[2]*a[7]  + a[12]*a[3]*a[6];
    inv[10] =  a[0]*a[5]*a[15]  - a[0]*a[7]*a[13]  - a[4]*a[1]*a[15]
             + a[4]*a[3]*a[13]  + a[12]*a[1]*a[7]  - a[12]*a[3]*a[5];
    inv[14] = -a[0]*a[5]*a[14]  + a[0]*a[6]*a[13]  + a[4]*a[1]*a[14]
             - a[4]*a[2]*a[13]  - a[12]*a[1]*a[6]  + a[12]*a[2]*a[5];
    inv[3]  = -a[1]*a[6]*a[11]  + a[1]*a[7]*a[10]  + a[5]*a[2]*a[11]
             - a[5]*a[3]*a[10]  - a[9]*a[2]*a[7]   + a[9]*a[3]*a[6];
    inv[7]  =  a[0]*a[6]*a[11]  - a[0]*a[7]*a[10]  - a[4]*a[2]*a[11]
             + a[4]*a[3]*a[10]  + a[8]*a[2]*a[7]   - a[8]*a[3]*a[6];
    inv[11] = -a[0]*a[5]*a[11]  + a[0]*a[7]*a[9]   + a[4]*a[1]*a[11]
             - a[4]*a[3]*a[9]   - a[8]*a[1]*a[7]   + a[8]*a[3]*a[5];
    inv[15] =  a[0]*a[5]*a[10]  - a[0]*a[6]*a[9]   - a[4]*a[1]*a[10]
             + a[4]*a[2]*a[9]   + a[8]*a[1]*a[6]   - a[8]*a[2]*a[5];

    Real det = a[0]*inv[0] + a[1]*inv[4] + a[2]*inv[8] + a[3]*inv[12];
    if (std::fabs(det) < 1e-20f) return Mat4::identity();
    det = 1.0f / det;

    Mat4 r;
    for (int c = 0; c < 4; ++c)
        for (int j = 0; j < 4; ++j)
            r.col[c][j] = inv[c*4 + j] * det;
    return r;
}

// Correct normal transform for non-uniform scale: inverse-transpose of the
// upper-left 3x3, returned as a Mat4 with an identity fourth column.
inline Mat4 normalMatrix(const Mat4& model) {
    Mat4 m = model;
    m.col[3] = {0, 0, 0, 1};
    m.col[0].w = m.col[1].w = m.col[2].w = 0.0f;
    return transpose(inverse(m));
}

// ---- Projections (right-handed, depth mapped to OpenGL's [-1, 1]) ---------
inline Mat4 perspective(Real fovyRadians, Real aspect, Real zNear, Real zFar) {
    Real f = 1.0f / std::tan(fovyRadians * 0.5f);
    Mat4 m = Mat4::zero();
    m.col[0][0] = f / aspect;
    m.col[1][1] = f;
    m.col[2][2] = (zFar + zNear) / (zNear - zFar);
    m.col[2][3] = -1.0f;
    m.col[3][2] = (2.0f * zFar * zNear) / (zNear - zFar);
    return m;
}
inline Mat4 orthoProjection(Real l, Real r, Real b, Real t, Real zNear, Real zFar) {
    Mat4 m;
    m.col[0][0] = 2.0f / (r - l);
    m.col[1][1] = 2.0f / (t - b);
    m.col[2][2] = -2.0f / (zFar - zNear);
    m.col[3] = {-(r + l) / (r - l), -(t + b) / (t - b),
                -(zFar + zNear) / (zFar - zNear), 1.0f};
    return m;
}
inline Mat4 lookAt(Vec3 eye, Vec3 center, Vec3 up) {
    Vec3 f = normalize(center - eye);
    Vec3 s = normalize(cross(f, up));
    // Guard against a degenerate up vector parallel to the view direction.
    if (lengthSq(s) < kEps) s = perpendicular(f);
    Vec3 u = cross(s, f);
    Mat4 m;
    m.col[0] = {s.x, u.x, -f.x, 0};
    m.col[1] = {s.y, u.y, -f.y, 0};
    m.col[2] = {s.z, u.z, -f.z, 0};
    m.col[3] = {-dot(s, eye), -dot(u, eye), dot(f, eye), 1};
    return m;
}

// ---------------------------------------------------------------- Quat -----
struct Quat {
    Real x = 0, y = 0, z = 0, w = 1;
    constexpr Quat() = default;
    constexpr Quat(Real x_, Real y_, Real z_, Real w_) : x(x_), y(y_), z(z_), w(w_) {}

    static Quat fromAxisAngle(Vec3 axis, Real angle) {
        Vec3 a = normalize(axis);
        Real h = angle * 0.5f, s = std::sin(h);
        return {a.x * s, a.y * s, a.z * s, std::cos(h)};
    }
    // Intrinsic X-then-Y-then-Z (matches the XYZ Euler order shown in the UI).
    static Quat fromEuler(Vec3 e) {
        Real cx = std::cos(e.x*0.5f), sx = std::sin(e.x*0.5f);
        Real cy = std::cos(e.y*0.5f), sy = std::sin(e.y*0.5f);
        Real cz = std::cos(e.z*0.5f), sz = std::sin(e.z*0.5f);
        return {sx*cy*cz - cx*sy*sz, cx*sy*cz + sx*cy*sz,
                cx*cy*sz - sx*sy*cz, cx*cy*cz + sx*sy*sz};
    }
};
inline Quat operator*(Quat a, Quat b) {
    return {a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
            a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
            a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
            a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z};
}
inline Quat normalize(Quat q) {
    Real l = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (l < kEps) return Quat();
    return {q.x/l, q.y/l, q.z/l, q.w/l};
}
inline Quat conjugate(Quat q) { return {-q.x, -q.y, -q.z, q.w}; }
inline Vec3 rotate(Quat q, Vec3 v) {
    Vec3 u{q.x, q.y, q.z};
    Vec3 t = cross(u, v) * 2.0f;
    return v + t * q.w + cross(u, t);
}
inline Mat4 toMat4(Quat q) {
    Real xx=q.x*q.x, yy=q.y*q.y, zz=q.z*q.z;
    Real xy=q.x*q.y, xz=q.x*q.z, yz=q.y*q.z;
    Real wx=q.w*q.x, wy=q.w*q.y, wz=q.w*q.z;
    Mat4 m;
    m.col[0] = {1-2*(yy+zz), 2*(xy+wz),   2*(xz-wy),   0};
    m.col[1] = {2*(xy-wz),   1-2*(xx+zz), 2*(yz+wx),   0};
    m.col[2] = {2*(xz+wy),   2*(yz-wx),   1-2*(xx+yy), 0};
    return m;
}
// Recover intrinsic XYZ Euler angles; used to round-trip the inspector fields.
inline Vec3 toEuler(Quat q) {
    Vec3 e;
    Real sinx = 2.0f * (q.w*q.x + q.y*q.z);
    Real cosx = 1.0f - 2.0f * (q.x*q.x + q.y*q.y);
    e.x = std::atan2(sinx, cosx);
    Real siny = 2.0f * (q.w*q.y - q.z*q.x);
    e.y = std::fabs(siny) >= 1.0f ? std::copysign(kHalfPi, siny) : std::asin(siny);
    Real sinz = 2.0f * (q.w*q.z + q.x*q.y);
    Real cosz = 1.0f - 2.0f * (q.y*q.y + q.z*q.z);
    e.z = std::atan2(sinz, cosz);
    return e;
}

// ----------------------------------------------------------- Primitives ----
struct Ray {
    Vec3 origin;
    Vec3 dir;  // expected normalized
    Vec3 at(Real t) const { return origin + dir * t; }
};

struct AABB {
    Vec3 min{ std::numeric_limits<Real>::max(),  std::numeric_limits<Real>::max(),  std::numeric_limits<Real>::max()};
    Vec3 max{-std::numeric_limits<Real>::max(), -std::numeric_limits<Real>::max(), -std::numeric_limits<Real>::max()};

    bool valid() const { return min.x <= max.x && min.y <= max.y && min.z <= max.z; }
    void expand(Vec3 p) { min = minv(min, p); max = maxv(max, p); }
    void expand(const AABB& b) { if (b.valid()) { min = minv(min, b.min); max = maxv(max, b.max); } }
    Vec3 center() const { return valid() ? (min + max) * 0.5f : Vec3{}; }
    Vec3 size() const { return valid() ? max - min : Vec3{}; }
    Real radius() const { return valid() ? length(size()) * 0.5f : 0.0f; }
};

// Slab test. Returns nearest positive hit distance, or false if no hit.
inline bool rayAABB(const Ray& r, const AABB& b, Real& tHit) {
    Real t0 = 0.0f, t1 = std::numeric_limits<Real>::max();
    for (int i = 0; i < 3; ++i) {
        if (std::fabs(r.dir[i]) < 1e-8f) {
            if (r.origin[i] < b.min[i] || r.origin[i] > b.max[i]) return false;
            continue;
        }
        Real invD = 1.0f / r.dir[i];
        Real tn = (b.min[i] - r.origin[i]) * invD;
        Real tf = (b.max[i] - r.origin[i]) * invD;
        if (tn > tf) std::swap(tn, tf);
        t0 = std::max(t0, tn);
        t1 = std::min(t1, tf);
        if (t0 > t1) return false;
    }
    tHit = t0;
    return true;
}

// Moller-Trumbore. Culls nothing; back faces hit too (needed for picking).
inline bool rayTriangle(const Ray& r, Vec3 a, Vec3 b, Vec3 c, Real& tHit) {
    Vec3 e1 = b - a, e2 = c - a;
    Vec3 p = cross(r.dir, e2);
    Real det = dot(e1, p);
    if (std::fabs(det) < 1e-12f) return false;
    Real invDet = 1.0f / det;
    Vec3 tv = r.origin - a;
    Real u = dot(tv, p) * invDet;
    if (u < -1e-6f || u > 1.0f + 1e-6f) return false;
    Vec3 q = cross(tv, e1);
    Real v = dot(r.dir, q) * invDet;
    if (v < -1e-6f || u + v > 1.0f + 1e-6f) return false;
    Real t = dot(e2, q) * invDet;
    if (t < 1e-6f) return false;
    tHit = t;
    return true;
}

} // namespace tg
