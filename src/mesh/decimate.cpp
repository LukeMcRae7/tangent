#include "mesh/decimate.h"

#include "core/bvh.h"
#include "mesh/element_id.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <limits>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <vector>

namespace tg {
namespace {

constexpr uint32_t kNone = 0xffffffffu;

// ---------------------------------------------------------------------------
// Geometry helpers.

// A symmetric 4x4 matrix, kept as its ten distinct entries:
//   | a b c d |
//   | b e f g |
//   | c f h i |
//   | d g i j |
// Its value at a point is the weighted sum of squared distances from that point
// to every plane it has absorbed.
struct Quadric {
    double a = 0, b = 0, c = 0, d = 0, e = 0, f = 0, g = 0, h = 0, i = 0, j = 0;

    void addPlane(Vec3 n, double offset, double w) {   // the plane n.x + offset = 0
        a += w * n.x * n.x;  b += w * n.x * n.y;  c += w * n.x * n.z;  d += w * n.x * offset;
        e += w * n.y * n.y;  f += w * n.y * n.z;  g += w * n.y * offset;
        h += w * n.z * n.z;  i += w * n.z * offset;
        j += w * offset * offset;
    }

    Quadric& operator+=(const Quadric& q) {
        a += q.a; b += q.b; c += q.c; d += q.d; e += q.e;
        f += q.f; g += q.g; h += q.h; i += q.i; j += q.j;
        return *this;
    }

    double at(Vec3 v) const {
        const double r = a * v.x * v.x + 2 * b * v.x * v.y + 2 * c * v.x * v.z + 2 * d * v.x +
                         e * v.y * v.y + 2 * f * v.y * v.z + 2 * g * v.y +
                         h * v.z * v.z + 2 * i * v.z + j;
        return r > 0 ? r : 0;           // rounding can take a true zero below it
    }

    // The point where the value is least, when there is one. On a flat patch
    // every plane is the same plane and the least value is a whole plane of
    // points, not one; on a crease it is a line. Neither has an answer here, and
    // the caller falls back to the ends and the middle of the edge.
    bool minimiser(Vec3& out) const {
        const double det = a * (e * h - f * f) - b * (b * h - c * f) + c * (b * f - c * e);
        const double scale = std::fabs(a) + std::fabs(e) + std::fabs(h);
        if (!(scale > 0) || std::fabs(det) <= 1e-10 * scale * scale * scale) return false;
        const double inv = 1.0 / det;
        const double m00 = (e * h - f * f) * inv, m01 = (c * f - b * h) * inv,
                     m02 = (b * f - c * e) * inv, m11 = (a * h - c * c) * inv,
                     m12 = (b * c - a * f) * inv, m22 = (a * e - b * b) * inv;
        out.x = -(m00 * d + m01 * g + m02 * i);
        out.y = -(m01 * d + m11 * g + m12 * i);
        out.z = -(m02 * d + m12 * g + m22 * i);
        return std::isfinite(out.x) && std::isfinite(out.y) && std::isfinite(out.z);
    }
};

uint64_t edgeKey(uint32_t u, uint32_t v) {
    const uint64_t lo = std::min(u, v), hi = std::max(u, v);
    return (lo << 32) | hi;
}

// Twice the area, and the unit normal when there is one.
Real doubleArea(Vec3 p, Vec3 q, Vec3 r, Vec3* normal = nullptr) {
    const Vec3 n = cross(q - p, r - p);
    const Real len = length(n);
    if (normal) *normal = len > 0 ? n * (Real(1) / len) : Vec3{};
    return len;
}

// 1 for an equilateral triangle, towards 0 for a sliver.
Real shape(Vec3 p, Vec3 q, Vec3 r) {
    const Real sum = lengthSq(q - p) + lengthSq(r - q) + lengthSq(p - r);
    return sum > 0 ? Real(2) * std::sqrt(Real(3)) * doubleArea(p, q, r) / sum : 0;
}

// The point of triangle (a, b, c) nearest to `p`, as barycentric weights on b
// and c. Ericson, Real-Time Collision Detection, 5.1.5.
void closestOnTriangle(Vec3 p, Vec3 a, Vec3 b, Vec3 c, Real& u, Real& v) {
    const Vec3 ab = b - a, ac = c - a, ap = p - a;
    const Real d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) { u = 0; v = 0; return; }
    const Vec3 bp = p - b;
    const Real d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) { u = 1; v = 0; return; }
    const Real vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) { u = d1 / (d1 - d3); v = 0; return; }
    const Vec3 cp = p - c;
    const Real d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) { u = 0; v = 1; return; }
    const Real vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) { u = 0; v = d2 / (d2 - d6); return; }
    const Real va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        const Real w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        u = 1 - w; v = w; return;
    }
    const Real sum = va + vb + vc;
    if (std::fabs(sum) < 1e-300) { u = v = Real(1) / 3; return; }
    u = vb / sum;
    v = vc / sum;
}

// Where a triangle whose corners lie on a sphere is furthest from it: the
// circumcentre, kept inside the triangle. Returned as barycentric weights.
void sagPoint(Vec3 a, Vec3 b, Vec3 c, Real& u, Real& v) {
    const Vec3 ab = b - a, ac = c - a;
    const Vec3 n = cross(ab, ac);
    const Real nn = lengthSq(n);
    if (!(nn > 0)) { u = v = Real(1) / 3; return; }
    const Vec3 cc = a + (cross(n, ab) * lengthSq(ac) + cross(ac, n) * lengthSq(ab)) * (Real(1) / (2 * nn));
    closestOnTriangle(cc, a, b, c, u, v);
}

// Whether segment p-q passes through triangle (a, b, c), strictly: touching
// its boundary within `eps` does not count.
bool segmentCrossesTriangle(Vec3 p, Vec3 q, Vec3 a, Vec3 b, Vec3 c, Real eps) {
    const Vec3 d = q - p, e1 = b - a, e2 = c - a;
    const Vec3 h = cross(d, e2);
    const Real det = dot(e1, h);
    const Real scale = std::sqrt(lengthSq(d) * lengthSq(e1) * lengthSq(e2)) + 1e-300;
    if (std::fabs(det) <= 1e-12 * scale) return false;      // parallel: see the coplanar case
    const Real inv = 1 / det;
    const Vec3 s = p - a;
    const Real u = dot(s, h) * inv;
    if (u <= eps || u >= 1 - eps) return false;
    const Vec3 qq = cross(s, e1);
    const Real v = dot(d, qq) * inv;
    if (v <= eps || u + v >= 1 - eps) return false;
    const Real t = dot(e2, qq) * inv;
    return t > eps && t < 1 - eps;
}

// Whether two triangles in a common plane overlap with positive area, tested by
// their edges crossing properly or one's centre lying inside the other.
bool coplanarOverlap(const Vec3 A[3], const Vec3 B[3], Vec3 normal, Real eps) {
    // Drop the axis the plane faces most nearly along.
    const Real ax = std::fabs(normal.x), ay = std::fabs(normal.y), az = std::fabs(normal.z);
    auto flat = [&](Vec3 v) -> std::array<Real, 2> {
        if (ax >= ay && ax >= az) return {v.y, v.z};
        if (ay >= az) return {v.x, v.z};
        return {v.x, v.y};
    };
    std::array<Real, 2> a[3], b[3];
    for (int k = 0; k < 3; ++k) { a[k] = flat(A[k]); b[k] = flat(B[k]); }
    auto orient = [](std::array<Real, 2> p, std::array<Real, 2> q, std::array<Real, 2> r) {
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
    };
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            const auto p1 = a[i], p2 = a[(i + 1) % 3], q1 = b[j], q2 = b[(j + 1) % 3];
            const Real d1 = orient(q1, q2, p1), d2 = orient(q1, q2, p2);
            const Real d3 = orient(p1, p2, q1), d4 = orient(p1, p2, q2);
            if (((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps)) &&
                ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps)))
                return true;
        }
    auto inside = [&](std::array<Real, 2> p, const std::array<Real, 2> t[3]) {
        const Real w = orient(t[0], t[1], t[2]);
        const Real s0 = orient(t[0], t[1], p), s1 = orient(t[1], t[2], p), s2 = orient(t[2], t[0], p);
        return w > 0 ? (s0 > eps && s1 > eps && s2 > eps) : (s0 < -eps && s1 < -eps && s2 < -eps);
    };
    const std::array<Real, 2> ca{(a[0][0] + a[1][0] + a[2][0]) / 3, (a[0][1] + a[1][1] + a[2][1]) / 3};
    const std::array<Real, 2> cb{(b[0][0] + b[1][0] + b[2][0]) / 3, (b[0][1] + b[1][1] + b[2][1]) / 3};
    return inside(ca, b) || inside(cb, a);
}

// Whether two triangles that share no vertex intersect. Non-coplanar triangles
// meet along a segment whose ends lie on edges of one or the other, so edges
// against faces finds every case; coplanar ones are tested in the plane.
bool trianglesIntersect(const Vec3 A[3], const Vec3 B[3], Real eps) {
    auto side = [](const Vec3 T[3], Vec3 p, Vec3& n) {
        n = cross(T[1] - T[0], T[2] - T[0]);
        return dot(n, p - T[0]);
    };
    Vec3 nb, na;
    Real db[3], da[3];
    for (int k = 0; k < 3; ++k) db[k] = side(B, A[k], nb);
    const Real lb = length(nb) + 1e-300;
    for (int k = 0; k < 3; ++k) db[k] /= lb;
    if ((db[0] > eps && db[1] > eps && db[2] > eps) || (db[0] < -eps && db[1] < -eps && db[2] < -eps))
        return false;
    for (int k = 0; k < 3; ++k) da[k] = side(A, B[k], na);
    const Real la = length(na) + 1e-300;
    for (int k = 0; k < 3; ++k) da[k] /= la;
    if ((da[0] > eps && da[1] > eps && da[2] > eps) || (da[0] < -eps && da[1] < -eps && da[2] < -eps))
        return false;

    const bool coplanar = std::fabs(db[0]) <= eps && std::fabs(db[1]) <= eps && std::fabs(db[2]) <= eps;
    if (coplanar) return coplanarOverlap(A, B, nb * (1 / lb), eps * eps);

    const Real rel = 1e-9;
    for (int k = 0; k < 3; ++k) {
        if (segmentCrossesTriangle(A[k], A[(k + 1) % 3], B[0], B[1], B[2], rel)) return true;
        if (segmentCrossesTriangle(B[k], B[(k + 1) % 3], A[0], A[1], A[2], rel)) return true;
    }
    return false;
}

// Euler characteristic of a triangle soup: vertices used, minus edges, plus
// faces. A collapse that keeps topology keeps this; one that pinched a handle
// shut or opened a hole would not.
long euler(size_t vertices, const std::vector<std::array<uint32_t, 3>>& tris) {
    std::vector<uint64_t> edges;
    edges.reserve(tris.size() * 3);
    for (const auto& t : tris)
        for (int k = 0; k < 3; ++k) edges.push_back(edgeKey(t[k], t[(k + 1) % 3]));
    std::sort(edges.begin(), edges.end());
    const size_t e = static_cast<size_t>(std::unique(edges.begin(), edges.end()) - edges.begin());
    return static_cast<long>(vertices) - static_cast<long>(e) + static_cast<long>(tris.size());
}

// ---------------------------------------------------------------------------
// Where one surface is furthest from another.
//
// Where along a flat triangle laid over a curved, faceted surface the distance
// peaks is not anywhere in particular. On a reduced sphere it sat on an edge
// 44% of the way along; on an organic mesh one reduced triangle had three
// separate peaks; and twice the worst point sat just beside a vertex that was
// already near the limit. A fixed lattice misses the first by about a per cent
// of the tolerance whatever its spacing -- the sag between samples shrinks with
// the square of the spacing, and the edges a tolerance allows grow in the same
// proportion -- a climb from only the best sample misses the second, and a
// search that never starts at a corner misses the third.
//
// So samples find where peaks may be -- including the corners -- and every
// start past `climbAbove` is climbed from: golden section along an edge, a
// shrinking pattern inside a triangle. It stops early once anything passes
// `stopAbove`, which is what a collapse check wants and what a measurement sets
// out of reach.
struct PeakSearch {
    const TriangleBvh* tree = nullptr;
    Real climbAbove = 0, stopAbove = 0, radius = 0;

    // When set, a distance is divided by the factor of the triangle it was
    // measured to, so that one limit can be held tighter in some places than
    // others. See the passes in reduceMesh.
    const std::vector<Real>* scale = nullptr;

    // Where the largest distance of the last edge() or face() was found.
    Vec3 peakAt{};

    // The triangle the last query landed on. Queries come in runs along one
    // edge or across one face, so it is nearly always a good first guess, and
    // the search starts with a tight bound instead of `radius`. It never
    // changes an answer, only how fast one is found -- and it is the one piece
    // of state a search changes, which is why each thread has its own.
    uint32_t hint = kNone;

    Real distance(Vec3 p) {
        uint32_t found = kNone;
        const Real d = tree->nearestPoint(p, radius, &found, hint);
        if (found != kNone) hint = found;
        if (scale && found != kNone && d < radius) return d / (*scale)[found];
        return d;
    }

    // As distance(), and remembers `p` if it is the largest seen so far.
    Real probe(Vec3 p, Real& best) {
        const Real d = distance(p);
        if (d > best) { best = d; peakAt = p; }
        return d;
    }

    // The largest distance on segment p-q. `dp` and `dq` are the ends' own
    // distances, which the caller already has.
    Real edge(Vec3 p, Vec3 q, Real dp, Real dq) {
        const Real ts[7] = {0, 1.0 / 6, 2.0 / 6, 3.0 / 6, 4.0 / 6, 5.0 / 6, 1};
        Real vals[7];
        vals[0] = dp;
        vals[6] = dq;
        Real best = dp;
        peakAt = p;
        if (dq > best) { best = dq; peakAt = q; }
        if (best > stopAbove) return best;
        for (int k = 1; k < 6; ++k) {
            vals[k] = probe(p + (q - p) * ts[k], best);
            if (vals[k] > stopAbove) return vals[k];
        }
        constexpr Real kPhi = 0.6180339887498949;
        for (int k = 0; k < 7; ++k) {
            if (vals[k] < climbAbove) continue;
            Real lo = std::max<Real>(0, ts[k] - 1.0 / 6), hi = std::min<Real>(1, ts[k] + 1.0 / 6);
            Real x1 = hi - (hi - lo) * kPhi, x2 = lo + (hi - lo) * kPhi;
            Real f1 = probe(p + (q - p) * x1, best), f2 = probe(p + (q - p) * x2, best);
            for (int it = 0; it < 9; ++it) {
                if (best > stopAbove) return best;
                if (f1 > f2) {
                    hi = x2; x2 = x1; f2 = f1;
                    x1 = hi - (hi - lo) * kPhi;
                    f1 = probe(p + (q - p) * x1, best);
                } else {
                    lo = x1; x1 = x2; f1 = f2;
                    x2 = lo + (hi - lo) * kPhi;
                    f2 = probe(p + (q - p) * x2, best);
                }
            }
            if (best > stopAbove) return best;
        }
        return best;
    }

    // The largest distance inside triangle a-b-c, whose corners' own distances
    // are da, db, dc.
    Real face(Vec3 a, Vec3 b, Vec3 c, Real da, Real db, Real dc) {
        // Barycentric (u, v) with w = 1 - u - v; the point is a*w + b*u + c*v.
        auto at = [&](Real u, Real v) { return a * (1 - u - v) + b * u + c * v; };
        struct Start { Real u, v, d; };
        Start starts[10];
        int n = 0;
        Real best = da;
        peakAt = a;
        if (db > best) { best = db; peakAt = b; }
        if (dc > best) { best = dc; peakAt = c; }
        if (best > stopAbove) return best;
        starts[n++] = {0, 0, da};
        starts[n++] = {1, 0, db};
        starts[n++] = {0, 1, dc};

        // The interior of a degree-five lattice.
        const Real lattice[6][2] = {{0.2, 0.2}, {0.4, 0.2}, {0.2, 0.4}, {0.6, 0.2}, {0.4, 0.4}, {0.2, 0.6}};
        for (const auto& l : lattice) {
            const Real d = probe(at(l[0], l[1]), best);
            if (d > stopAbove) return d;
            starts[n++] = {l[0], l[1], d};
        }
        {
            Real u, v;
            sagPoint(a, b, c, u, v);
            const Real d = probe(at(u, v), best);
            if (d > stopAbove) return d;
            starts[n++] = {u, v, d};
        }

        const Real dirs[6][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, -1}, {-1, 1}};
        for (int s = 0; s < n; ++s) {
            if (starts[s].d < climbAbove) continue;
            Real bu = starts[s].u, bv = starts[s].v, local = starts[s].d;
            Real step = 0.1;
            int queries = 0;
            while (step > 1.0 / 160 && queries < 36) {
                bool moved = false;
                for (const auto& dir : dirs) {
                    const Real u = bu + dir[0] * step, v = bv + dir[1] * step;
                    if (u < 0 || v < 0 || u + v > 1) continue;
                    const Real d = probe(at(u, v), best);
                    ++queries;
                    if (d > stopAbove) return d;
                    if (d > local) { local = d; bu = u; bv = v; moved = true; }
                }
                if (!moved) step *= 0.5;
            }
        }
        return best;
    }
};

// ---------------------------------------------------------------------------
// The reduced mesh's triangles, filed by where they are.
//
// A collapse can keep every point within the tolerance of where it was and
// still make the surface pass through itself: on a plate thinner than twice the
// tolerance, one side may move toward the other as far as it likes. A mesh that
// passes through itself is no use to a boolean, which is most of what a
// converted mesh is for, so new triangles are tested against the ones already
// near them. A uniform grid, because triangles are added and removed on every
// collapse and a grid does both in constant time.
class TriangleGrid {
public:
    void reset(Real cell) {
        cell_ = cell;
        cells_.clear();
    }

    void insert(uint32_t id, Vec3 a, Vec3 b, Vec3 c) { visit(a, b, c, [&](std::vector<uint32_t>& v) { v.push_back(id); }); }

    void remove(uint32_t id, Vec3 a, Vec3 b, Vec3 c) {
        visit(a, b, c, [&](std::vector<uint32_t>& v) {
            const auto it = std::find(v.begin(), v.end(), id);
            if (it != v.end()) { *it = v.back(); v.pop_back(); }
        });
    }

    // Every triangle filed in a cell that the box around (a, b, c) touches.
    void near(Vec3 a, Vec3 b, Vec3 c, Real pad, std::vector<uint32_t>& out) const {
        out.clear();
        const Vec3 lo{std::min({a.x, b.x, c.x}) - pad, std::min({a.y, b.y, c.y}) - pad, std::min({a.z, b.z, c.z}) - pad};
        const Vec3 hi{std::max({a.x, b.x, c.x}) + pad, std::max({a.y, b.y, c.y}) + pad, std::max({a.z, b.z, c.z}) + pad};
        const auto i0 = index(lo), i1 = index(hi);
        for (int64_t x = i0[0]; x <= i1[0]; ++x)
            for (int64_t y = i0[1]; y <= i1[1]; ++y)
                for (int64_t z = i0[2]; z <= i1[2]; ++z) {
                    const auto it = cells_.find(key(x, y, z));
                    if (it != cells_.end()) out.insert(out.end(), it->second.begin(), it->second.end());
                }
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
    }

private:
    std::array<int64_t, 3> index(Vec3 p) const {
        return {static_cast<int64_t>(std::floor(p.x / cell_)), static_cast<int64_t>(std::floor(p.y / cell_)),
                static_cast<int64_t>(std::floor(p.z / cell_))};
    }
    static uint64_t key(int64_t x, int64_t y, int64_t z) {
        return static_cast<uint64_t>(x) * 0x9E3779B97F4A7C15ull ^ static_cast<uint64_t>(y) * 0xC2B2AE3D27D4EB4Full ^
               static_cast<uint64_t>(z) * 0x165667B19E3779F9ull;
    }
    template <typename F> void visit(Vec3 a, Vec3 b, Vec3 c, F f) {
        const Vec3 lo{std::min({a.x, b.x, c.x}), std::min({a.y, b.y, c.y}), std::min({a.z, b.z, c.z})};
        const Vec3 hi{std::max({a.x, b.x, c.x}), std::max({a.y, b.y, c.y}), std::max({a.z, b.z, c.z})};
        const auto i0 = index(lo), i1 = index(hi);
        for (int64_t x = i0[0]; x <= i1[0]; ++x)
            for (int64_t y = i0[1]; y <= i1[1]; ++y)
                for (int64_t z = i0[2]; z <= i1[2]; ++z) f(cells_[key(x, y, z)]);
    }

    Real cell_ = 1;
    std::unordered_map<uint64_t, std::vector<uint32_t>> cells_;
};

// ---------------------------------------------------------------------------
// A few worker threads, woken for each batch of work.
//
// Kept for the life of one reduction rather than spawned per batch: there are
// hundreds of batches, each a few milliseconds of work, and starting threads
// for each would cost a noticeable share of it.
class Workers {
public:
    explicit Workers(size_t extra) {
        for (size_t t = 0; t < extra; ++t) threads_.emplace_back([this] { loop(); });
    }
    ~Workers() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            quit_ = true;
        }
        wake_.notify_all();
        for (std::thread& t : threads_) t.join();
    }

    size_t size() const { return threads_.size() + 1; }

    // Calls job(i) for every i in [0, n), across the workers and this thread,
    // and returns when all are done. `job` must be safe to call concurrently
    // for different i.
    void run(size_t n, const std::function<void(size_t)>& job) {
        if (n == 0) return;
        if (threads_.empty() || n == 1) {
            for (size_t i = 0; i < n; ++i) job(i);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            job_ = &job;
            count_ = n;
            next_.store(0);
            pending_ = threads_.size();
            ++generation_;
        }
        wake_.notify_all();
        drain();
        std::unique_lock<std::mutex> lock(mutex_);
        done_.wait(lock, [this] { return pending_ == 0; });
        job_ = nullptr;
    }

private:
    void drain() {
        for (;;) {
            const size_t i = next_.fetch_add(1);
            if (i >= count_) return;
            (*job_)(i);
        }
    }

    void loop() {
        uint64_t seen = 0;
        for (;;) {
            {
                std::unique_lock<std::mutex> lock(mutex_);
                wake_.wait(lock, [&] { return quit_ || generation_ != seen; });
                if (quit_) return;
                seen = generation_;
            }
            drain();
            {
                std::lock_guard<std::mutex> lock(mutex_);
                --pending_;
            }
            done_.notify_all();
        }
    }

    std::vector<std::thread> threads_;
    std::mutex mutex_;
    std::condition_variable wake_, done_;
    const std::function<void(size_t)>* job_ = nullptr;
    size_t count_ = 0;
    std::atomic<size_t> next_{0};
    size_t pending_ = 0;
    uint64_t generation_ = 0;
    bool quit_ = false;
};

size_t workerCount() {
    const size_t hw = std::thread::hardware_concurrency();
    return std::max<size_t>(1, std::min<size_t>(hw == 0 ? 1 : hw, 8));
}

// ---------------------------------------------------------------------------

class Simplifier {
public:
    Simplifier(const Mesh& in, const ReduceOptions& opt) : opt_(opt) { load(in); }

    // What verification found, for the next pass to tighten around.
    struct Findings {
        std::vector<Real> originalWorst;   // per original triangle, its furthest point from the result
        std::vector<std::pair<Real, Vec3>> reducedPeaks;   // per result triangle, its furthest point and where
    };

    // One reduction, with the checks held to `inner` times the tolerance and,
    // around each original triangle, a further `factors[t]`.
    ReduceResult run(Mesh& out, ElementId salt, Real inner, const std::vector<Real>& factors,
                     Workers& workers, Findings& findings);

    const std::vector<Vec3>& originalPositions() const { return p0_; }
    const std::vector<uint32_t>& originalTriangles() const { return t0_; }
    const TriangleBvh& originalTree() const { return original_; }

private:
    struct Entry {
        double cost;
        uint32_t a, b;
        uint32_t stampA, stampB;
        bool operator>(const Entry& o) const {
            if (cost != o.cost) return cost > o.cost;
            if (a != o.a) return a > o.a;       // ties broken by index, so the same
            return b > o.b;                     // mesh always reduces the same way
        }
    };

    // Everything one collapse check needs of its own, so that checks in a batch
    // can run at once. Filled by evaluate(), read by commit().
    struct Trial {
        Entry entry{};
        bool ok = false;
        uint32_t keep = 0, gone = 0;
        Vec3 at{};
        Real atDistance = 0;
        std::vector<uint32_t> shared, ringOld, ringNewIds, gathered, assignment;
        std::vector<std::array<uint32_t, 3>> ringNew;
        std::vector<Vec3> ringNormals;     // parallel to ringNew
        std::vector<std::pair<double, Vec3>> options;
        std::vector<uint32_t> na, nb, both, opposite, nearby;
        PeakSearch search;

        // The space the collapse changes: every triangle it removes or makes.
        // At commit, a trial whose box meets one already committed in the same
        // batch has its crossing test run again against the mesh as it now is.
        Vec3 lo{}, hi{};
    };

    // ---- state ----
    ReduceOptions opt_;
    Real tol_ = 0, checkTol_ = 0, checkTol2_ = 0, areaEps_ = 0;

    // The original, which is what the tolerance is measured against. Never
    // modified: the tree over it holds pointers into these arrays.
    std::vector<Vec3> p0_;
    std::vector<uint32_t> t0_;
    TriangleBvh original_;
    bool originalClosed_ = false;
    long originalEuler_ = 0;
    size_t verticesUsed_ = 0;

    // The mesh as it is being reduced.
    std::vector<Vec3> p_;
    std::vector<std::array<uint32_t, 3>> t_;
    std::vector<char> triAlive_, vertAlive_, boundary_;
    std::vector<std::vector<uint32_t>> vertTris_;
    std::vector<Quadric> q_;
    std::vector<uint32_t> stamp_, mark_;
    std::vector<Real> vertDistance_;    // each vertex's distance to the original
    uint32_t epoch_ = 0;
    size_t liveTris_ = 0;

    // Points on the original surface, each owned by the reduced triangle it is
    // known to be within tolerance of.
    std::vector<Vec3> samples_;

    // Which way the original faces at each sample, or zero where that is not
    // well defined. See the ownership check in tryCollapse.
    std::vector<Vec3> sampleNormals_;
    std::vector<std::vector<uint32_t>> triSamples_;

    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue_;
    std::vector<std::pair<double, Vec3>> pushScratch_;

    TriangleGrid grid_;
    Real touchEps_ = 0;
    std::vector<uint32_t> ringScratch_, aroundScratch_;

    // ---- setup ----
    void load(const Mesh& in) {
        p0_.reserve(in.verts.size());
        for (const MeshVertex& v : in.verts) p0_.push_back(v.position);

        std::vector<Index> loop, corners;
        for (size_t f = 0; f < in.faces.size(); ++f) {
            in.faceVertices(static_cast<Index>(f), loop);
            if (loop.size() == 3) {
                t0_.insert(t0_.end(), {static_cast<uint32_t>(loop[0]), static_cast<uint32_t>(loop[1]),
                                       static_cast<uint32_t>(loop[2])});
                continue;
            }
            in.triangulateFacePublic(static_cast<Index>(f), corners);
            for (size_t k = 0; k + 2 < corners.size(); k += 3)
                t0_.insert(t0_.end(), {static_cast<uint32_t>(loop[static_cast<size_t>(corners[k])]),
                                       static_cast<uint32_t>(loop[static_cast<size_t>(corners[k + 1])]),
                                       static_cast<uint32_t>(loop[static_cast<size_t>(corners[k + 2])])});
        }
    }

    bool prepare(Real inner, std::string& why);

    // ---- queries; all safe to run concurrently with each other ----
    void neighbours(uint32_t v, std::vector<uint32_t>& out) const {
        out.clear();
        for (uint32_t t : vertTris_[v])
            for (int k = 0; k < 3; ++k)
                if (t_[t][k] != v) out.push_back(t_[t][k]);
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
    }

    bool linkCondition(uint32_t a, uint32_t b, Trial& t) const;
    void candidates(uint32_t a, uint32_t b, std::vector<std::pair<double, Vec3>>& out) const;
    void evaluate(Trial& t) const;
    bool tryCollapse(Trial& t, Vec3 at) const;

    // ---- changes; only ever from one thread ----
    void push(uint32_t a, uint32_t b);

    // Edges a commit has made worth another look, gathered across a batch and
    // costed in parallel once it is done rather than one at a time after each
    // commit. The queue is not read while a batch commits, so nothing can see
    // them late.
    std::vector<uint64_t> repush_;
    void flushRepush(Workers& workers);

    // Whether the trial's new triangles would pass through any triangle already
    // near them. Asked at commit rather than in the parallel checks, because two
    // collapses in one batch can be far apart along the surface and still close
    // in space -- either face of a thin plate -- and each would pass against the
    // mesh as it was before the other.
    bool crossesSurface(Trial& t) const;
    bool commit(const Trial& t);

    // ---- the finished result ----
    std::vector<uint64_t> originalBoundary_;     // sorted keys of edges with one triangle
    bool originalBoundaryEdge(uint32_t u, uint32_t v) const {
        return std::binary_search(originalBoundary_.begin(), originalBoundary_.end(), edgeKey(u, v));
    }
    Real measure(const std::vector<Vec3>& positions,
                 const std::vector<std::array<uint32_t, 3>>& live, Workers& workers,
                 Findings& findings) const;

    const std::vector<Real>* factors_ = nullptr;
    std::vector<Real> sampleLimit2_;     // per sample: its squared limit
};

bool Simplifier::prepare(Real inner, std::string& why) {
    const auto factorOf = [&](uint32_t tri) { return factors_ ? (*factors_)[tri] : Real(1); };
    const size_t nt = t0_.size() / 3;
    if (nt == 0) { why = "there are no triangles to reduce"; return false; }
    if (!(opt_.toleranceMm > 0) || !std::isfinite(opt_.toleranceMm)) {
        why = "the tolerance has to be a distance greater than zero";
        return false;
    }

    tol_ = opt_.toleranceMm;
    checkTol_ = tol_ * inner;
    checkTol2_ = checkTol_ * checkTol_;

    Vec3 lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
    for (uint32_t v : t0_) {
        const Vec3& p = p0_[v];
        lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
        hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
    }
    const Real diag = length(hi - lo);
    // A triangle smaller than this has no direction worth trusting. Relative to
    // the part, because a ring's worth of float noise on a 200mm part is not
    // the same size as on a 2mm one.
    areaEps_ = diag * diag * 1e-14;

    original_.build(p0_, t0_);

    p_ = p0_;
    t_.resize(nt);
    for (size_t t = 0; t < nt; ++t) t_[t] = {t0_[t * 3], t0_[t * 3 + 1], t0_[t * 3 + 2]};
    triAlive_.assign(nt, 1);
    liveTris_ = nt;

    const size_t nv = p0_.size();
    vertTris_.assign(nv, {});
    for (size_t t = 0; t < nt; ++t)
        for (int k = 0; k < 3; ++k) vertTris_[t_[t][k]].push_back(static_cast<uint32_t>(t));
    vertAlive_.assign(nv, 0);
    for (size_t v = 0; v < nv; ++v) vertAlive_[v] = vertTris_[v].empty() ? 0 : 1;
    verticesUsed_ = static_cast<size_t>(std::count(vertAlive_.begin(), vertAlive_.end(), 1));
    stamp_.assign(nv, 0);
    mark_.assign(nv, 0);
    boundary_.assign(nv, 0);
    vertDistance_.assign(nv, 0);        // every vertex starts on the original

    // Edges, and the triangles either side of each.
    struct EdgeUse { uint32_t t0 = kNone, t1 = kNone; int count = 0; };
    std::unordered_map<uint64_t, EdgeUse> edges;
    edges.reserve(nt * 2);
    for (size_t t = 0; t < nt; ++t)
        for (int k = 0; k < 3; ++k) {
            EdgeUse& u = edges[edgeKey(t_[t][k], t_[t][(k + 1) % 3])];
            (u.count == 0 ? u.t0 : u.t1) = static_cast<uint32_t>(t);
            ++u.count;
        }
    // Visited in key order, not hash order, so that everything built from them
    // -- samples, quadrics, the first queue -- is the same on every platform.
    std::vector<uint64_t> keys;
    keys.reserve(edges.size());
    for (const auto& kv : edges) keys.push_back(kv.first);
    std::sort(keys.begin(), keys.end());

    std::vector<Vec3> normal(nt);
    std::vector<Real> area(nt);
    for (size_t t = 0; t < nt; ++t)
        area[t] = doubleArea(p_[t_[t][0]], p_[t_[t][1]], p_[t_[t][2]], &normal[t]) * Real(0.5);

    // The planes each vertex starts with: its own triangles, weighted by area
    // so a sliver does not outvote the face it sits in.
    q_.assign(nv, Quadric{});
    for (size_t t = 0; t < nt; ++t) {
        if (area[t] * 2 <= areaEps_) continue;
        const Vec3 n = normal[t];
        const double off = -dot(n, p_[t_[t][0]]);
        for (int k = 0; k < 3; ++k) q_[t_[t][k]].addPlane(n, off, area[t]);
    }

    // And along boundaries and sharp edges, a plane standing up from the face
    // through the edge. It makes sliding off the edge expensive, which keeps
    // those collapses to the end of the queue and keeps new vertices on the
    // edge when they can be.
    const Real creaseCos = std::cos(opt_.creaseAngleDeg * kPi / 180.0);
    constexpr double kFeatureWeight = 100.0;
    originalClosed_ = true;
    originalBoundary_.clear();
    for (uint64_t key : keys) {
        const EdgeUse& use = edges[key];
        const auto u = static_cast<uint32_t>(key >> 32), v = static_cast<uint32_t>(key & 0xffffffffu);
        const Vec3 dir = p_[v] - p_[u];
        const double w = kFeatureWeight * lengthSq(dir);
        auto stand = [&](uint32_t t) {
            Vec3 n = cross(dir, normal[t]);
            const Real len = length(n);
            if (len <= 0) return;
            n = n * (Real(1) / len);
            const double off = -dot(n, p_[u]);
            q_[u].addPlane(n, off, w);
            q_[v].addPlane(n, off, w);
        };
        if (use.count == 1) {
            originalClosed_ = false;
            originalBoundary_.push_back(key);
            boundary_[u] = boundary_[v] = 1;
            stand(use.t0);
        } else if (use.count == 2 && dot(normal[use.t0], normal[use.t1]) < creaseCos) {
            stand(use.t0);
            stand(use.t1);
        }
    }

    // The points the original is held at: every vertex and edge midpoint, and
    // inside each triangle its centre and the interior of a degree-four
    // lattice -- a point between a triangle's corners can sit further from the
    // reduced surface than any corner does, where the reduced surface folds
    // across it. Each starts on a triangle it lies on.
    samples_.clear();
    sampleNormals_.clear();
    samples_.reserve(verticesUsed_ + keys.size() + nt * 4);
    sampleNormals_.reserve(samples_.capacity());
    triSamples_.assign(nt, {});
    sampleLimit2_.clear();
    auto add = [&](Vec3 p, Vec3 n, uint32_t t, Real factor) {
        triSamples_[t].push_back(static_cast<uint32_t>(samples_.size()));
        samples_.push_back(p);
        sampleNormals_.push_back(n);
        const Real limit = checkTol_ * factor;
        sampleLimit2_.push_back(limit * limit);
    };
    // A vertex or an edge faces the average of its faces -- unless they point
    // so nearly apart that the average means nothing, as at the knife edge of a
    // very thin wedge, where no direction is asked of it.
    auto blend = [](Vec3 sum, int count) {
        const Real len = length(sum);
        return count > 0 && len > Real(0.3) * count ? sum * (Real(1) / len) : Vec3{};
    };
    for (size_t v = 0; v < nv; ++v) {
        if (!vertAlive_[v]) continue;
        Vec3 sum{};
        int count = 0;
        Real factor = 1;
        for (uint32_t t : vertTris_[v]) {
            factor = std::min(factor, factorOf(t));
            if (area[t] * 2 > areaEps_) { sum = sum + normal[t]; ++count; }
        }
        add(p0_[v], blend(sum, count), vertTris_[v].front(), factor);
    }
    for (uint64_t key : keys) {
        const EdgeUse& use = edges[key];
        Vec3 sum{};
        int count = 0;
        Real factor = 1;
        for (uint32_t t : {use.t0, use.t1}) {
            if (t == kNone) continue;
            factor = std::min(factor, factorOf(t));
            if (area[t] * 2 > areaEps_) { sum = sum + normal[t]; ++count; }
        }
        add((p0_[key >> 32] + p0_[key & 0xffffffffu]) * Real(0.5), blend(sum, count), use.t0, factor);
    }
    for (size_t t = 0; t < nt; ++t) {
        const Vec3 a = p0_[t_[t][0]], b = p0_[t_[t][1]], c = p0_[t_[t][2]];
        const auto owner = static_cast<uint32_t>(t);
        const Vec3 n = area[t] * 2 > areaEps_ ? normal[t] : Vec3{};
        const Real f = factorOf(owner);
        add((a + b + c) * (Real(1) / Real(3)), n, owner, f);
        add((a * 2 + b + c) * Real(0.25), n, owner, f);
        add((a + b * 2 + c) * Real(0.25), n, owner, f);
        add((a + b + c * 2) * Real(0.25), n, owner, f);
    }

    originalEuler_ = euler(verticesUsed_, t_);

    // Cells about twice the original's mean edge: a few triangles to a cell at
    // the start, and still a modest count per triangle as they grow.
    {
        Real sum = 0;
        for (uint64_t key : keys) sum += length(p0_[key >> 32] - p0_[key & 0xffffffffu]);
        const Real mean = keys.empty() ? diag : sum / static_cast<Real>(keys.size());
        grid_.reset(std::max(mean * 2, diag * 1e-4));
    }
    for (size_t t = 0; t < nt; ++t)
        grid_.insert(static_cast<uint32_t>(t), p_[t_[t][0]], p_[t_[t][1]], p_[t_[t][2]]);
    touchEps_ = diag * 1e-10;

    for (uint64_t key : keys)
        push(static_cast<uint32_t>(key >> 32), static_cast<uint32_t>(key & 0xffffffffu));
    return true;
}

// Whether collapsing a-b keeps the surface a surface.
//
// The vertices both ends are joined to must be exactly the ones opposite the
// edge: one for an edge on a boundary, two inside. A third shared neighbour
// means the collapse would weld two separate parts of the surface together at a
// point. And an interior edge between two boundary vertices would, collapsed,
// pinch the surface to a single point where two boundaries meet.
bool Simplifier::linkCondition(uint32_t a, uint32_t b, Trial& t) const {
    t.shared.clear();
    for (uint32_t tri : vertTris_[a])
        if (t_[tri][0] == b || t_[tri][1] == b || t_[tri][2] == b) t.shared.push_back(tri);
    if (t.shared.empty() || t.shared.size() > 2) return false;
    if (t.shared.size() == 2 && boundary_[a] && boundary_[b]) return false;

    t.opposite.clear();
    for (uint32_t tri : t.shared)
        for (int k = 0; k < 3; ++k)
            if (t_[tri][k] != a && t_[tri][k] != b) t.opposite.push_back(t_[tri][k]);
    std::sort(t.opposite.begin(), t.opposite.end());

    neighbours(a, t.na);
    neighbours(b, t.nb);
    t.both.clear();
    std::set_intersection(t.na.begin(), t.na.end(), t.nb.begin(), t.nb.end(), std::back_inserter(t.both));
    return t.both == t.opposite;
}

// Where a collapse might put the surviving vertex, cheapest first.
void Simplifier::candidates(uint32_t a, uint32_t b,
                            std::vector<std::pair<double, Vec3>>& out) const {
    out.clear();
    Quadric q = q_[a];
    q += q_[b];

    // A vertex on a boundary may not leave it for the interior: that would
    // shrink the hole or the edge it is on. So when only one end is on a
    // boundary, the only place the pair may meet is that end.
    if (boundary_[a] != boundary_[b]) {
        const Vec3 at = boundary_[a] ? p_[a] : p_[b];
        out.push_back({q.at(at), at});
        return;
    }

    Vec3 best;
    if (q.minimiser(best)) {
        // Only if it lies among the triangles it would join. A minimiser from a
        // nearly singular quadric can be anywhere, and a point beyond the ring
        // is a fold waiting to happen even when the tolerance would allow it.
        Vec3 lo = p_[a], hi = p_[a];
        for (uint32_t v : {a, b})
            for (uint32_t t : vertTris_[v])
                for (int k = 0; k < 3; ++k) {
                    const Vec3& p = p_[t_[t][k]];
                    lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
                    hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
                }
        const bool inside = best.x >= lo.x - tol_ && best.x <= hi.x + tol_ &&
                            best.y >= lo.y - tol_ && best.y <= hi.y + tol_ &&
                            best.z >= lo.z - tol_ && best.z <= hi.z + tol_;
        if (inside) out.push_back({q.at(best), best});
    }
    const Vec3 mid = (p_[a] + p_[b]) * Real(0.5);
    out.push_back({q.at(mid), mid});
    out.push_back({q.at(p_[a]), p_[a]});
    out.push_back({q.at(p_[b]), p_[b]});
    std::stable_sort(out.begin(), out.end(),
                     [](const auto& x, const auto& y) { return x.first < y.first; });
}

void Simplifier::flushRepush(Workers& workers) {
    std::sort(repush_.begin(), repush_.end());
    repush_.erase(std::unique(repush_.begin(), repush_.end()), repush_.end());
    struct Costed { double cost = 0; bool ok = false; };
    std::vector<Costed> costs(repush_.size());
    constexpr size_t kChunk = 256;
    const size_t chunks = (repush_.size() + kChunk - 1) / kChunk;
    workers.run(chunks, [&](size_t c) {
        std::vector<std::pair<double, Vec3>> scratch;
        for (size_t i = c * kChunk; i < std::min(repush_.size(), (c + 1) * kChunk); ++i) {
            const auto a = static_cast<uint32_t>(repush_[i] >> 32), b = static_cast<uint32_t>(repush_[i] & 0xffffffffu);
            if (!vertAlive_[a] || !vertAlive_[b]) continue;
            candidates(a, b, scratch);
            if (scratch.empty()) continue;
            costs[i] = {scratch.front().first + lengthSq(p_[a] - p_[b]) * tol_ * tol_ * 1e-12, true};
        }
    });
    for (size_t i = 0; i < repush_.size(); ++i) {
        if (!costs[i].ok) continue;
        const auto a = static_cast<uint32_t>(repush_[i] >> 32), b = static_cast<uint32_t>(repush_[i] & 0xffffffffu);
        queue_.push({costs[i].cost, a, b, stamp_[a], stamp_[b]});
    }
    repush_.clear();
}

void Simplifier::push(uint32_t a, uint32_t b) {
    if (a == b || !vertAlive_[a] || !vertAlive_[b]) return;
    if (a > b) std::swap(a, b);
    candidates(a, b, pushScratch_);
    if (pushScratch_.empty()) return;
    // Plus a length term far too small to outweigh any real movement of the
    // surface, which only decides between collapses that cost the same.
    //
    // On a flat face every collapse costs exactly nothing, and the order among
    // them used to fall to vertex index -- so one face of a box was reduced to a
    // few huge triangles before the next was touched, each of those triangles
    // came to own thousands of the original's sample points, and every later
    // collapse rechecked all of them: 187 million distance tests for a
    // 3,072-triangle box. Shortest first coarsens evenly instead, triangles stay
    // alike in size, and the samples stay spread across them.
    const double lengthTerm = lengthSq(p_[a] - p_[b]) * tol_ * tol_ * 1e-12;
    queue_.push({pushScratch_.front().first + lengthTerm, a, b, stamp_[a], stamp_[b]});
}

void Simplifier::evaluate(Trial& t) const {
    t.ok = false;
    const uint32_t a = t.entry.a, b = t.entry.b;
    if (!linkCondition(a, b, t)) return;

    // The survivor is the boundary end if there is one, so a boundary keeps
    // its vertex; otherwise the lower index, so the result is repeatable.
    t.keep = a;
    t.gone = b;
    if (boundary_[t.gone] && !boundary_[t.keep]) std::swap(t.keep, t.gone);

    candidates(a, b, t.options);
    for (const auto& option : t.options) {
        if (!tryCollapse(t, option.second)) continue;
        t.at = option.second;
        if (crossesSurface(t)) continue;
        t.ok = true;
        t.lo = t.hi = t.at;
        auto grow = [&](Vec3 p) {
            t.lo = {std::min(t.lo.x, p.x), std::min(t.lo.y, p.y), std::min(t.lo.z, p.z)};
            t.hi = {std::max(t.hi.x, p.x), std::max(t.hi.y, p.y), std::max(t.hi.z, p.z)};
        };
        for (uint32_t tri : t.ringOld)
            for (uint32_t v : t_[tri]) grow(p_[v]);
        return;
    }
    // None passed. The edge is left out of the queue until something near it
    // changes and puts it back.
}

// Every check a collapse has to pass, with `keep` moved to `at` and `gone`
// merged into it. Changes nothing; on success the trial describes the change.
bool Simplifier::tryCollapse(Trial& t, Vec3 at) const {
    const uint32_t keep = t.keep, gone = t.gone;

    t.ringOld.clear();
    for (uint32_t v : {keep, gone})
        for (uint32_t tri : vertTris_[v]) t.ringOld.push_back(tri);
    std::sort(t.ringOld.begin(), t.ringOld.end());
    t.ringOld.erase(std::unique(t.ringOld.begin(), t.ringOld.end()), t.ringOld.end());

    t.ringNew.clear();
    t.ringNewIds.clear();
    for (uint32_t tri : t.ringOld) {
        if (std::find(t.shared.begin(), t.shared.end(), tri) != t.shared.end()) continue;
        std::array<uint32_t, 3> nt = t_[tri];
        for (uint32_t& v : nt)
            if (v == gone) v = keep;
        t.ringNew.push_back(nt);
        t.ringNewIds.push_back(tri);
    }
    if (t.ringNew.empty()) return false;

    auto pos = [&](uint32_t v) { return v == keep ? at : p_[v]; };

    // Shape: nothing collapses to nothing, nothing turns over, and nothing
    // already reasonable is made into a sliver.
    constexpr Real kMinTurnCos = 0.2;
    t.ringNormals.resize(t.ringNew.size());
    for (size_t k = 0; k < t.ringNew.size(); ++k) {
        const auto& tri = t.ringNew[k];
        const auto& old = t_[t.ringNewIds[k]];
        Vec3 nNew, nOld;
        const Real aNew = doubleArea(pos(tri[0]), pos(tri[1]), pos(tri[2]), &nNew);
        t.ringNormals[k] = nNew;
        const Real aOld = doubleArea(p_[old[0]], p_[old[1]], p_[old[2]], &nOld);
        if (aNew <= areaEps_) return false;
        if (aOld > areaEps_ && dot(nNew, nOld) < kMinTurnCos) return false;
        const Real sNew = shape(pos(tri[0]), pos(tri[1]), pos(tri[2]));
        const Real sOld = shape(p_[old[0]], p_[old[1]], p_[old[2]]);
        if (sNew < 0.02 && sNew < sOld * 0.5) return false;
    }

    // No two triangles on the same three vertices. Any duplicate would contain
    // `keep`, and every triangle containing `keep` afterwards is in the ring.
    for (size_t i = 0; i < t.ringNew.size(); ++i) {
        std::array<uint32_t, 3> x = t.ringNew[i];
        std::sort(x.begin(), x.end());
        for (size_t j = i + 1; j < t.ringNew.size(); ++j) {
            std::array<uint32_t, 3> y = t.ringNew[j];
            std::sort(y.begin(), y.end());
            if (x == y) return false;
        }
    }

    // The old surface stays near the new one -- checked first, because it is
    // the cheaper of the two by several times and a collapse it refuses never
    // pays for the other. Every original point owned by a triangle that changed
    // must be within tolerance of one of the triangles replacing them. Nearer
    // triangles elsewhere may exist; this is a bound, not the distance, which is
    // what makes it safe.
    //
    // And the triangle that covers a point has to face the way the original did
    // there. Distance alone could not tell the top of a thin plate from its
    // bottom: at a tolerance near the plate's thickness every point of either
    // side is close to both, and a 0.4mm plate reduced at 0.3mm came back as a
    // four-triangle tetrahedron turned inside out, every point of it honestly
    // within tolerance. Asking the cover to face the same way -- within ninety
    // degrees -- keeps a thin part a two-sided shell, which is what a boolean
    // needs it to be.
    t.gathered.clear();
    for (uint32_t tri : t.ringOld)
        t.gathered.insert(t.gathered.end(), triSamples_[tri].begin(), triSamples_[tri].end());
    t.assignment.resize(t.gathered.size());
    for (size_t s = 0; s < t.gathered.size(); ++s) {
        const Vec3 x = samples_[t.gathered[s]];
        const Vec3 facing = sampleNormals_[t.gathered[s]];
        const bool oriented = lengthSq(facing) > 0;
        const Real limit2 = sampleLimit2_[t.gathered[s]];
        Real best = limit2 * 1.000002;
        uint32_t owner = kNone;
        for (size_t k = 0; k < t.ringNew.size(); ++k) {
            if (oriented && dot(t.ringNormals[k], facing) <= 0) continue;
            const auto& tri = t.ringNew[k];
            const Real d2 = TriangleBvh::pointTriangleDistance2(x, pos(tri[0]), pos(tri[1]), pos(tri[2]));
            if (d2 < best) { best = d2; owner = t.ringNewIds[k]; }
        }
        if (owner == kNone || best > limit2) return false;
        t.assignment[s] = owner;
    }

    // The new surface stays near the old one: the moved vertex, every edge that
    // moved, and every triangle that changed, each searched for its peak.
    t.atDistance = t.search.distance(at);
    if (t.atDistance > checkTol_) return false;
    auto dist = [&](uint32_t v) { return v == keep ? t.atDistance : vertDistance_[v]; };
    // Each moved edge once. Collected as the vertices `keep` is joined to
    // rather than as edges leaving it: on a closed surface every such edge
    // leaves `keep` in exactly one triangle, but on a boundary an edge has only
    // one triangle, and there it may just as well arrive -- and would never be
    // checked at all.
    t.na.clear();
    for (const auto& tri : t.ringNew)
        for (uint32_t v : tri)
            if (v != keep) t.na.push_back(v);
    std::sort(t.na.begin(), t.na.end());
    t.na.erase(std::unique(t.na.begin(), t.na.end()), t.na.end());
    for (uint32_t v : t.na)
        if (t.search.edge(at, p_[v], t.atDistance, vertDistance_[v]) > checkTol_) return false;
    for (const auto& tri : t.ringNew)
        if (t.search.face(pos(tri[0]), pos(tri[1]), pos(tri[2]), dist(tri[0]), dist(tri[1]),
                          dist(tri[2])) > checkTol_)
            return false;
    return true;
}

bool Simplifier::crossesSurface(Trial& t) const {
    const uint32_t keep = t.keep, gone = t.gone;
    auto pos = [&](uint32_t v) { return v == keep ? t.at : p_[v]; };
    for (const auto& nt : t.ringNew) {
        const Vec3 A[3] = {pos(nt[0]), pos(nt[1]), pos(nt[2])};
        grid_.near(A[0], A[1], A[2], touchEps_, t.nearby);
        for (uint32_t other : t.nearby) {
            if (!triAlive_[other]) continue;
            if (std::binary_search(t.ringOld.begin(), t.ringOld.end(), other)) continue;
            const auto& ot = t_[other];
            int shared = 0;
            for (uint32_t v : ot) {
                if (v == gone) return true;             // cannot happen: all of gone's are in the ring
                if (v == nt[0] || v == nt[1] || v == nt[2]) ++shared;
            }
            if (shared >= 2) continue;                  // joined along an edge; folds are the turn check's
            Vec3 B[3] = {p_[ot[0]], p_[ot[1]], p_[ot[2]]};
            Vec3 a3[3] = {A[0], A[1], A[2]};
            if (shared == 1) {
                // Joined at a corner. Pull both a hair toward their centres so
                // the corner they share stops touching, and anything left
                // touching is a real crossing.
                const Vec3 ca = (a3[0] + a3[1] + a3[2]) * (Real(1) / 3);
                const Vec3 cb = (B[0] + B[1] + B[2]) * (Real(1) / 3);
                for (int k = 0; k < 3; ++k) {
                    a3[k] = ca + (a3[k] - ca) * (1 - 1e-6);
                    B[k] = cb + (B[k] - cb) * (1 - 1e-6);
                }
            }
            if (trianglesIntersect(a3, B, touchEps_)) return true;
        }
    }
    return false;
}

bool Simplifier::commit(const Trial& t) {

    const uint32_t keep = t.keep, gone = t.gone;
    for (uint32_t tri : t.ringOld) grid_.remove(tri, p_[t_[tri][0]], p_[t_[tri][1]], p_[t_[tri][2]]);
    for (uint32_t tri : t.shared) {
        triAlive_[tri] = 0;
        --liveTris_;
        for (uint32_t v : t_[tri]) {
            if (v == keep || v == gone) continue;
            auto& list = vertTris_[v];
            list.erase(std::remove(list.begin(), list.end(), tri), list.end());
        }
    }
    for (size_t k = 0; k < t.ringNewIds.size(); ++k) t_[t.ringNewIds[k]] = t.ringNew[k];

    p_[keep] = t.at;
    for (uint32_t tri : t.ringNewIds) grid_.insert(tri, p_[t_[tri][0]], p_[t_[tri][1]], p_[t_[tri][2]]);
    vertDistance_[keep] = t.atDistance;
    q_[keep] += q_[gone];
    boundary_[keep] = boundary_[keep] || boundary_[gone];
    vertAlive_[gone] = 0;
    vertTris_[gone].clear();
    vertTris_[keep] = t.ringNewIds;

    for (uint32_t tri : t.ringOld) triSamples_[tri].clear();
    for (size_t s = 0; s < t.gathered.size(); ++s)
        triSamples_[t.assignment[s]].push_back(t.gathered[s]);

    // Everything whose surroundings just changed goes back in the queue: the
    // surviving vertex and its neighbours, and every edge they have. Their old
    // entries are stale by stamp, and an edge refused before may be allowed
    // now. Gathered here and pushed once the batch is done.
    neighbours(keep, ringScratch_);
    ringScratch_.push_back(keep);
    for (uint32_t v : ringScratch_) ++stamp_[v];
    for (uint32_t v : ringScratch_) {
        neighbours(v, aroundScratch_);
        for (uint32_t w : aroundScratch_) repush_.push_back(edgeKey(v, w));
    }
    return true;
}

Real Simplifier::measure(const std::vector<Vec3>& positions,
                         const std::vector<std::array<uint32_t, 3>>& live, Workers& workers,
                         Findings& findings) const {
    // The verification pass. Both surfaces are searched, in both directions,
    // with the same peak searches the checks use but climbing from anything
    // past a third of the tolerance and never stopping early. It runs across
    // the workers; every chunk reads only the two trees and the two meshes, so
    // the largest value is the same however the threads are scheduled.
    std::vector<uint32_t> flat;
    flat.reserve(live.size() * 3);
    for (const auto& tri : live) flat.insert(flat.end(), tri.begin(), tri.end());
    TriangleBvh reduced;
    reduced.build(positions, flat);

    const Real radius = tol_ * 2;
    const Real climb = tol_ / 3;
    const Real never = std::numeric_limits<Real>::infinity();

    // Per-vertex distances first: each surface's corners to the other.
    std::vector<Real> reducedCorner(positions.size(), 0), originalCorner(p0_.size(), 0);
    constexpr size_t kChunk = 512;
    auto chunks = [&](size_t n, const std::function<void(size_t, size_t)>& body) {
        const size_t count = (n + kChunk - 1) / kChunk;
        workers.run(count, [&](size_t c) { body(c * kChunk, std::min(n, (c + 1) * kChunk)); });
    };
    chunks(positions.size(), [&](size_t b, size_t e) {
        PeakSearch s{&original_, climb, never, radius};
        for (size_t i = b; i < e; ++i) reducedCorner[i] = s.distance(positions[i]);
    });
    chunks(p0_.size(), [&](size_t b, size_t e) {
        PeakSearch s{&reduced, climb, never, radius};
        for (size_t i = b; i < e; ++i) originalCorner[i] = s.distance(p0_[i]);
    });

    const size_t reducedChunks = (live.size() + kChunk - 1) / kChunk;
    const size_t originalTris = t0_.size() / 3;
    const size_t originalChunks = (originalTris + kChunk - 1) / kChunk;
    std::vector<Real> worst(reducedChunks + originalChunks, 0);
    findings.originalWorst.assign(originalTris, 0);
    findings.reducedPeaks.assign(live.size(), {0, Vec3{}});

    workers.run(reducedChunks, [&](size_t c) {
        PeakSearch s{&original_, climb, never, radius};
        Real w = 0;
        for (size_t i = c * kChunk; i < std::min(live.size(), (c + 1) * kChunk); ++i) {
            const auto& tri = live[i];
            const Vec3 a = positions[tri[0]], b = positions[tri[1]], cc = positions[tri[2]];
            const Real da = reducedCorner[tri[0]], db = reducedCorner[tri[1]], dc = reducedCorner[tri[2]];
            auto& peak = findings.reducedPeaks[i];
            auto note = [&](Real d) { if (d > peak.first) { peak.first = d; peak.second = s.peakAt; } };
            note(s.edge(a, b, da, db));
            note(s.edge(b, cc, db, dc));
            note(s.edge(cc, a, dc, da));
            note(s.face(a, b, cc, da, db, dc));
            w = std::max(w, peak.first);
        }
        worst[c] = w;
    });
    workers.run(originalChunks, [&](size_t c) {
        PeakSearch s{&reduced, climb, never, radius};
        Real w = 0;
        for (size_t i = c * kChunk; i < std::min(originalTris, (c + 1) * kChunk); ++i) {
            const uint32_t ia = t0_[i * 3], ib = t0_[i * 3 + 1], ic = t0_[i * 3 + 2];
            const Vec3 a = p0_[ia], b = p0_[ib], cc = p0_[ic];
            const Real da = originalCorner[ia], db = originalCorner[ib], dc = originalCorner[ic];
            // Each edge from the one of its two triangles that runs it from the
            // lower vertex to the higher -- on a closed surface exactly one does
            // -- and a boundary edge, which has only one triangle, from that.
            Real mine = std::max({da, db, dc});
            auto edgeOnce = [&](uint32_t u, uint32_t v, Vec3 pu, Vec3 pv, Real du, Real dv) {
                if (u < v || originalBoundaryEdge(u, v)) mine = std::max(mine, s.edge(pu, pv, du, dv));
            };
            edgeOnce(ia, ib, a, b, da, db);
            edgeOnce(ib, ic, b, cc, db, dc);
            edgeOnce(ic, ia, cc, a, dc, da);
            mine = std::max(mine, s.face(a, b, cc, da, db, dc));
            findings.originalWorst[i] = mine;
            w = std::max(w, mine);
        }
        worst[reducedChunks + c] = w;
    });
    return *std::max_element(worst.begin(), worst.end());
}

ReduceResult Simplifier::run(Mesh& out, ElementId salt, Real inner, const std::vector<Real>& factors,
                             Workers& workers, Findings& findings) {
    factors_ = &factors;
    ReduceResult r;
    r.trianglesBefore = t0_.size() / 3;

    std::string why;
    if (!prepare(inner, why)) { r.error = why; return r; }
    r.verticesBefore = verticesUsed_;

    // ---- reduce, in batches ----
    //
    // A batch is the cheapest collapses in the queue whose closed
    // neighbourhoods -- both ends and every vertex joined to either -- do not
    // overlap. A commit changes only triangles and vertices inside its own
    // neighbourhood, and a check reads only inside its own, so the checks in a
    // batch cannot see each other and run at once; the commits then go in cost
    // order. The batch size depends only on the mesh, never on how many threads
    // there are, so a given mesh reduces to the same result on every machine.
    std::vector<Trial> trials;
    std::vector<Entry> deferred;
    std::vector<std::pair<Vec3, Vec3>> committedBoxes;
    std::vector<uint32_t> hood;
    while (!queue_.empty()) {
        if (opt_.targetTriangles > 0 && liveTris_ <= opt_.targetTriangles) break;

        const size_t batch = std::clamp<size_t>(liveTris_ / 512, 1, 48);
        size_t selected = 0, scanned = 0;
        ++epoch_;
        deferred.clear();
        while (!queue_.empty() && selected < batch && scanned < batch * 8) {
            const Entry e = queue_.top();
            queue_.pop();
            if (!vertAlive_[e.a] || !vertAlive_[e.b]) continue;
            if (stamp_[e.a] != e.stampA || stamp_[e.b] != e.stampB) continue;
            ++scanned;

            hood.clear();
            for (uint32_t v : {e.a, e.b})
                for (uint32_t tri : vertTris_[v])
                    for (int k = 0; k < 3; ++k) hood.push_back(t_[tri][k]);
            bool clash = false;
            for (uint32_t v : hood)
                if (mark_[v] == epoch_) { clash = true; break; }
            if (clash) { deferred.push_back(e); continue; }
            for (uint32_t v : hood) mark_[v] = epoch_;

            if (trials.size() <= selected) trials.emplace_back();
            Trial& t = trials[selected++];
            t.entry = e;
            t.search = PeakSearch{&original_, checkTol_ * 0.5, checkTol_, checkTol_ * 1.5};
            t.search.scale = factors_;
        }

        workers.run(selected, [&](size_t i) { evaluate(trials[i]); });

        // Commit in cost order. A trial's crossing test was run against the mesh
        // before the batch; if an earlier commit in this batch changed anything
        // in the same space, it is run again against the mesh as it is now.
        committedBoxes.clear();
        for (size_t i = 0; i < selected; ++i) {
            if (opt_.targetTriangles > 0 && liveTris_ <= opt_.targetTriangles) break;
            Trial& t = trials[i];
            if (!t.ok) continue;
            const Vec3 pad{touchEps_, touchEps_, touchEps_};
            bool overlaps = false;
            for (const auto& box : committedBoxes)
                if (t.lo.x - pad.x <= box.second.x && t.hi.x + pad.x >= box.first.x &&
                    t.lo.y - pad.y <= box.second.y && t.hi.y + pad.y >= box.first.y &&
                    t.lo.z - pad.z <= box.second.z && t.hi.z + pad.z >= box.first.z) { overlaps = true; break; }
            if (overlaps && crossesSurface(t)) {
                // Refused only because of a neighbour committed a moment ago;
                // back in the queue, to be looked at again against the new mesh.
                deferred.push_back(t.entry);
                continue;
            }
            if (commit(t)) committedBoxes.push_back({t.lo, t.hi});
        }
        for (const Entry& e : deferred) queue_.push(e);
        flushRepush(workers);
    }

    // ---- rebuild ----
    std::vector<uint32_t> remap(p_.size(), kNone);
    std::vector<Vec3> positions;
    std::vector<std::array<uint32_t, 3>> live;
    live.reserve(liveTris_);
    for (size_t t = 0; t < t_.size(); ++t)
        if (triAlive_[t]) live.push_back(t_[t]);
    for (auto& tri : live)
        for (uint32_t& v : tri) {
            if (remap[v] == kNone) {
                remap[v] = static_cast<uint32_t>(positions.size());
                positions.push_back(p_[v]);
            }
            v = remap[v];
        }

    // Topology is the one thing that must come out exactly as it went in. Every
    // collapse was checked to keep it, and this is the check that they did.
    if (euler(positions.size(), live) != originalEuler_) {
        r.error = "the reduction changed the shape's topology, and was not kept";
        return r;
    }

    // Closed in, closed out.
    if (originalClosed_) {
        std::unordered_map<uint64_t, int> uses;
        uses.reserve(live.size() * 2);
        for (const auto& tri : live)
            for (int k = 0; k < 3; ++k) ++uses[edgeKey(tri[k], tri[(k + 1) % 3])];
        for (const auto& kv : uses)
            if (kv.second != 2) {
                r.error = "the reduction opened the surface, and was not kept";
                return r;
            }
    }

    std::vector<uint32_t> sizes(live.size(), 3), indices;
    indices.reserve(live.size() * 3);
    for (const auto& tri : live) indices.insert(indices.end(), tri.begin(), tri.end());

    Mesh::Names names;
    names.vertices.resize(positions.size());
    names.faces.resize(live.size());
    for (size_t i = 0; i < positions.size(); ++i)
        names.vertices[i] = nameId(salt, IdRole::Vertex, static_cast<ElementId>(i));
    for (size_t i = 0; i < live.size(); ++i)
        names.faces[i] = nameId(salt, IdRole::Face, static_cast<ElementId>(i));

    Mesh built;
    if (!built.build(positions, sizes, indices, &names)) {
        r.error = "the reduced triangles would not rebuild into a surface";
        return r;
    }

    r.deviationMm = measure(positions, live, workers, findings);
    out = std::move(built);
    r.ok = true;
    r.trianglesAfter = live.size();
    r.verticesAfter = positions.size();
    r.reachedTarget = opt_.targetTriangles > 0 && live.size() <= opt_.targetTriangles;
    r.withinTolerance = r.deviationMm <= tol_;
    return r;
}

} // namespace

namespace {

// 1 for every triangle, except `factor` for those beside an edge that turns by
// more than `creaseDeg`.
void sharpCreaseFactors(const std::vector<Vec3>& p, const std::vector<uint32_t>& t, Real creaseDeg,
                        Real factor, std::vector<Real>& out) {
    const size_t nt = t.size() / 3;
    out.assign(nt, 1);
    std::vector<Vec3> normal(nt);
    for (size_t i = 0; i < nt; ++i) doubleArea(p[t[i * 3]], p[t[i * 3 + 1]], p[t[i * 3 + 2]], &normal[i]);

    std::vector<std::pair<uint64_t, uint32_t>> uses;
    uses.reserve(nt * 3);
    for (size_t i = 0; i < nt; ++i)
        for (int k = 0; k < 3; ++k)
            uses.push_back({edgeKey(t[i * 3 + k], t[i * 3 + (k + 1) % 3]), static_cast<uint32_t>(i)});
    std::sort(uses.begin(), uses.end());

    const Real cosLimit = std::cos(creaseDeg * kPi / 180.0);
    for (size_t i = 0; i + 1 < uses.size(); ++i) {
        if (uses[i].first != uses[i + 1].first) continue;
        const uint32_t x = uses[i].second, y = uses[i + 1].second;
        if (lengthSq(normal[x]) > 0 && lengthSq(normal[y]) > 0 && dot(normal[x], normal[y]) < cosLimit) {
            out[x] = factor;
            out[y] = factor;
        }
    }
}

} // namespace

ReduceResult reduceMesh(const Mesh& in, const ReduceOptions& options, Mesh& out,
                        ElementId salt) {
    const auto started = std::chrono::steady_clock::now();
    Workers workers(workerCount() - 1);
    auto finish = [&](ReduceResult r) {
        r.milliseconds = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - started).count();
        return r;
    };

    // The checks start held two per cent inside the tolerance, and ten per cent
    // further inside it across sharp creases -- the triangles either side of an
    // edge that turns by more than the crease angle.
    //
    // Both numbers are measured rather than derived. The searches a collapse
    // runs find the surface's peaks to within a per cent or two, and the
    // verification finds the rest; across a sharp crease the peaks are narrow
    // enough that the searches miss by more. Held a per cent inside, a 62,000-
    // triangle organic mesh needed a second full pass at three tolerances out of
    // five, each for a handful of triangles a fraction of a micron over, every
    // one of them beside a sharp crease or within two per cent of the limit.
    // Held like this, it needs one at all five, and gives up about one and a
    // half per cent of its reduction to do it.
    //
    // If the finished result still measures over the tolerance somewhere, the
    // reduction runs again from the start with its checks held tighter *there* -- around each
    // original triangle the verification found too far from the result, or
    // under the result's own furthest points, and the triangles touching those
    // -- by the proportion it was over and a little more. Everywhere else keeps
    // the tolerance it had. Tightening everywhere instead cost a fifth of the
    // reduction on a mesh whose only problem was one sharp spike.
    //
    // A third pass, if it comes to that, also holds everything a little tighter,
    // so that a problem too spread out for the local factors still converges.
    Real inner = options.firstPassMargin;
    std::vector<Real> factors;
    ReduceResult last;
    Mesh attempt;
    constexpr int kPasses = 3;
    for (int pass = 1; pass <= kPasses; ++pass) {
        Simplifier s(in, options);
        if (factors.empty()) sharpCreaseFactors(s.originalPositions(), s.originalTriangles(),
                                                options.creaseAngleDeg, options.sharpCreaseMargin, factors);
        Simplifier::Findings findings;
        ReduceResult r = s.run(attempt, salt, inner, factors, workers, findings);
        r.passes = pass;
        if (!r.ok) return finish(r);
        last = r;
        out = std::move(attempt);
        attempt = Mesh();
        if (r.withinTolerance || pass == kPasses) break;

        const Real tol = options.toleranceMm;
        const size_t nt = factors.size();
        std::vector<Real> tighten(nt, 1);
        auto demand = [&](uint32_t tri, Real measured) {
            if (tri >= nt || measured <= tol * 0.985) return;
            tighten[tri] = std::min(tighten[tri], Real(0.97) * tol / std::max(measured, tol));
        };
        for (size_t i = 0; i < nt; ++i) demand(static_cast<uint32_t>(i), findings.originalWorst[i]);
        for (const auto& peak : findings.reducedPeaks) {
            if (peak.first <= tol * 0.985) continue;
            uint32_t tri = kNone;
            s.originalTree().nearestPoint(peak.second, tol * 4, &tri);
            demand(tri, peak.first);
        }
        // Spread to every triangle sharing a corner with one that needs it.
        const auto& t0 = s.originalTriangles();
        std::vector<Real> corner(s.originalPositions().size(), 1);
        for (size_t i = 0; i < nt; ++i)
            for (int k = 0; k < 3; ++k) corner[t0[i * 3 + k]] = std::min(corner[t0[i * 3 + k]], tighten[i]);
        for (size_t i = 0; i < nt; ++i) {
            Real f = tighten[i];
            for (int k = 0; k < 3; ++k) f = std::min(f, corner[t0[i * 3 + k]]);
            factors[i] *= f;
        }
        if (pass + 1 == kPasses) inner *= 0.95;
    }
    return finish(last);
}

} // namespace tg
