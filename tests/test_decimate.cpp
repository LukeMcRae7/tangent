// Mesh reduction.
//
// Every claim in decimate.h is checked here against something the simplifier
// did not compute itself:
//
//   the tolerance, by sampling both surfaces far more densely than the
//   simplifier does, at random, and measuring through a tree that test_bvh
//   holds against brute force;
//
//   topology, by the Euler characteristic and by counting edge uses;
//
//   features, by finding the original corners and sharp edges in the result;
//
//   and that flat things become as small as they can: a finely divided box has
//   to come back as twelve triangles, not merely fewer.
#include "mesh/decimate.h"

#include "core/bvh.h"
#include "mesh/health.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}

// ---------------------------------------------------------------------------
// Building test meshes.

struct Soup {
    std::vector<Vec3> p;
    std::vector<uint32_t> t;
    uint32_t vertex(Vec3 v) { p.push_back(v); return static_cast<uint32_t>(p.size() - 1); }
    void tri(uint32_t a, uint32_t b, uint32_t c) { t.insert(t.end(), {a, b, c}); }
    bool build(Mesh& m) const {
        std::vector<uint32_t> sizes(t.size() / 3, 3);
        return m.build(p, sizes, t);
    }
};

// A box whose every face is an n x n grid of quads, each split in two. Shared
// edge and corner vertices are welded, so it is one closed surface.
static Mesh gridBox(Real w, Real d, Real h, int n) {
    Soup s;
    std::map<std::array<long, 3>, uint32_t> at;
    auto vert = [&](Vec3 v) {
        const std::array<long, 3> key{std::lround(v.x * 1e6), std::lround(v.y * 1e6), std::lround(v.z * 1e6)};
        auto it = at.find(key);
        if (it != at.end()) return it->second;
        const uint32_t id = s.vertex(v);
        at.emplace(key, id);
        return id;
    };
    // Each face: an origin corner and two spanning vectors, wound so the
    // normal (u x v) points out.
    const Real x = w / 2, y = d / 2, z = h / 2;
    struct Face { Vec3 o, u, v; };
    const Face faces[6] = {
        {{-x, -y, -z}, {0, 2 * y, 0}, {2 * x, 0, 0}},   // -z
        {{-x, -y, z}, {2 * x, 0, 0}, {0, 2 * y, 0}},    // +z
        {{-x, -y, -z}, {2 * x, 0, 0}, {0, 0, 2 * z}},   // -y
        {{-x, y, -z}, {0, 0, 2 * z}, {2 * x, 0, 0}},    // +y
        {{-x, -y, -z}, {0, 0, 2 * z}, {0, 2 * y, 0}},   // -x
        {{x, -y, -z}, {0, 2 * y, 0}, {0, 0, 2 * z}},    // +x
    };
    for (const Face& f : faces)
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j) {
                auto P = [&](int a, int b) { return f.o + f.u * (Real(a) / n) + f.v * (Real(b) / n); };
                const uint32_t a = vert(P(i, j)), b = vert(P(i + 1, j)), c = vert(P(i + 1, j + 1)),
                               e = vert(P(i, j + 1));
                s.tri(a, b, c);
                s.tri(a, c, e);
            }
    Mesh m;
    s.build(m);
    return m;
}

static Mesh uvSphere(Real r, int rings, int segs) {
    Soup s;
    const uint32_t north = s.vertex({0, 0, r});
    for (int i = 1; i < rings; ++i) {
        const Real th = kPi * i / rings;
        for (int j = 0; j < segs; ++j) {
            const Real ph = 2 * kPi * j / segs;
            s.vertex({r * std::sin(th) * std::cos(ph), r * std::sin(th) * std::sin(ph), r * std::cos(th)});
        }
    }
    const uint32_t south = s.vertex({0, 0, -r});
    auto ringAt = [&](int i, int j) { return static_cast<uint32_t>(1 + (i - 1) * segs + (j % segs)); };
    for (int j = 0; j < segs; ++j) s.tri(north, ringAt(1, j), ringAt(1, j + 1));
    for (int i = 1; i < rings - 1; ++i)
        for (int j = 0; j < segs; ++j) {
            s.tri(ringAt(i, j), ringAt(i + 1, j), ringAt(i + 1, j + 1));
            s.tri(ringAt(i, j), ringAt(i + 1, j + 1), ringAt(i, j + 1));
        }
    for (int j = 0; j < segs; ++j) s.tri(ringAt(rings - 1, j), south, ringAt(rings - 1, j + 1));
    Mesh m;
    s.build(m);
    return m;
}

static Mesh torus(Real R, Real r, int n, int k) {
    Soup s;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < k; ++j) {
            const Real u = 2 * kPi * i / n, v = 2 * kPi * j / k;
            s.vertex({(R + r * std::cos(v)) * std::cos(u), (R + r * std::cos(v)) * std::sin(u), r * std::sin(v)});
        }
    auto V = [&](int i, int j) { return static_cast<uint32_t>((i % n) * k + (j % k)); };
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < k; ++j) {
            s.tri(V(i, j), V(i + 1, j), V(i + 1, j + 1));
            s.tri(V(i, j), V(i + 1, j + 1), V(i, j + 1));
        }
    Mesh m;
    s.build(m);
    return m;
}

// Something like a scan or a sculpt: a sphere with low bumps all over and a
// scattering of sharp spikes, the kind of surface where the distance between a
// reduced mesh and the original peaks in narrow, awkward places. Built for the
// test because the mesh that exposed those places cannot live in the repository.
static Mesh spikyBlob(Real R, int rings, int segs) {
    Soup s;
    auto radius = [&](Real th, Real ph) {
        Real r = R * (1 + 0.06 * std::sin(5 * th) * std::cos(7 * ph) + 0.03 * std::cos(11 * th + 3 * ph));
        // Spikes: narrow cones of extra radius around a few directions.
        const Vec3 dir{std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th)};
        const Vec3 tips[6] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {-0.577, 0.577, 0.577}, {0.6, -0.8, 0}, {0, -0.6, -0.8}};
        for (const Vec3& tip : tips) {
            const Real c = dot(dir, normalize(tip));
            if (c > 0.985) r += R * 0.25 * (c - 0.985) / 0.015;
        }
        return r;
    };
    const uint32_t north = s.vertex({0, 0, radius(0, 0)});
    for (int i = 1; i < rings; ++i) {
        const Real th = kPi * i / rings;
        for (int j = 0; j < segs; ++j) {
            const Real ph = 2 * kPi * j / segs;
            const Real r = radius(th, ph);
            s.vertex({r * std::sin(th) * std::cos(ph), r * std::sin(th) * std::sin(ph), r * std::cos(th)});
        }
    }
    const uint32_t south = s.vertex({0, 0, -radius(kPi, 0)});
    auto ringAt = [&](int i, int j) { return static_cast<uint32_t>(1 + (i - 1) * segs + (j % segs)); };
    for (int j = 0; j < segs; ++j) s.tri(north, ringAt(1, j), ringAt(1, j + 1));
    for (int i = 1; i < rings - 1; ++i)
        for (int j = 0; j < segs; ++j) {
            s.tri(ringAt(i, j), ringAt(i + 1, j), ringAt(i + 1, j + 1));
            s.tri(ringAt(i, j), ringAt(i + 1, j + 1), ringAt(i, j + 1));
        }
    for (int j = 0; j < segs; ++j) s.tri(ringAt(rings - 1, j), south, ringAt(rings - 1, j + 1));
    Mesh m;
    s.build(m);
    return m;
}

// A bumpy open cap: a disc of rings with a wavy height, its rim a boundary.
static Mesh bumpyDisc(Real R, int rings, int segs) {
    Soup s;
    const uint32_t centre = s.vertex({0, 0, 1.0});
    for (int i = 1; i <= rings; ++i) {
        const Real rr = R * i / rings;
        for (int j = 0; j < segs; ++j) {
            const Real a = 2 * kPi * j / segs;
            const Real z = std::cos(rr * 0.3) * 1.0 + 0.2 * std::sin(3 * a);
            s.vertex({rr * std::cos(a), rr * std::sin(a), z});
        }
    }
    auto V = [&](int i, int j) { return static_cast<uint32_t>(1 + (i - 1) * segs + (j % segs)); };
    for (int j = 0; j < segs; ++j) s.tri(centre, V(1, j), V(1, j + 1));
    for (int i = 1; i < rings; ++i)
        for (int j = 0; j < segs; ++j) {
            s.tri(V(i, j), V(i + 1, j), V(i + 1, j + 1));
            s.tri(V(i, j), V(i + 1, j + 1), V(i, j + 1));
        }
    Mesh m;
    s.build(m);
    return m;
}

// ---------------------------------------------------------------------------
// Measuring.

static void triangles(const Mesh& m, std::vector<Vec3>& pos, std::vector<uint32_t>& tri) {
    pos.clear();
    tri.clear();
    for (const MeshVertex& v : m.verts) pos.push_back(v.position);
    std::vector<Index> loop;
    for (size_t f = 0; f < m.faces.size(); ++f) {
        m.faceVertices(static_cast<Index>(f), loop);
        for (size_t i = 1; i + 1 < loop.size(); ++i)
            tri.insert(tri.end(), {static_cast<uint32_t>(loop[0]), static_cast<uint32_t>(loop[i]),
                                   static_cast<uint32_t>(loop[i + 1])});
    }
}

// The largest distance from a random point on one surface to the other, both
// ways, `perTriangle` points per triangle plus every vertex and edge midpoint.
static Real denseDeviation(const Mesh& a, const Mesh& b, int perTriangle, unsigned seed) {
    std::vector<Vec3> pa, pb;
    std::vector<uint32_t> ta, tb;
    triangles(a, pa, ta);
    triangles(b, pb, tb);
    TriangleBvh ba, bb;
    ba.build(pa, ta);
    bb.build(pb, tb);

    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> u(0.0, 1.0);
    Real worst = 0;
    auto sweep = [&](const std::vector<Vec3>& p, const std::vector<uint32_t>& t, const TriangleBvh& other) {
        for (size_t i = 0; i + 2 < t.size(); i += 3) {
            const Vec3 A = p[t[i]], B = p[t[i + 1]], C = p[t[i + 2]];
            worst = std::max(worst, other.nearestPoint(A, 1e9));
            worst = std::max(worst, other.nearestPoint((A + B) * 0.5, 1e9));
            for (int k = 0; k < perTriangle; ++k) {
                Real s1 = u(rng), s2 = u(rng);
                if (s1 + s2 > 1) { s1 = 1 - s1; s2 = 1 - s2; }
                worst = std::max(worst, other.nearestPoint(A + (B - A) * s1 + (C - A) * s2, 1e9));
            }
        }
    };
    sweep(pa, ta, bb);
    sweep(pb, tb, ba);
    return worst;
}

static long eulerOf(const Mesh& m, int* boundaryEdges = nullptr) {
    std::vector<Vec3> p;
    std::vector<uint32_t> t;
    triangles(m, p, t);
    std::unordered_map<uint64_t, int> uses;
    std::vector<char> used(p.size(), 0);
    for (size_t i = 0; i + 2 < t.size(); i += 3)
        for (int k = 0; k < 3; ++k) {
            const uint64_t u = t[i + k], v = t[i + (k + 1) % 3];
            ++uses[(std::min(u, v) << 32) | std::max(u, v)];
            used[t[i + k]] = 1;
        }
    if (boundaryEdges) {
        *boundaryEdges = 0;
        for (const auto& [k, c] : uses) if (c == 1) ++*boundaryEdges;
    }
    const long V = static_cast<long>(std::count(used.begin(), used.end(), 1));
    return V - static_cast<long>(uses.size()) + static_cast<long>(t.size() / 3);
}

static bool hasVertexNear(const Mesh& m, Vec3 p, Real within) {
    for (const MeshVertex& v : m.verts) if (length(v.position - p) <= within) return true;
    return false;
}

// ---------------------------------------------------------------------------

int main() {
    std::printf("decimate\n");

    // --- flat faces reduce all the way ------------------------------------
    std::printf("--- a finely divided box comes back as a box ---\n");
    {
        const Mesh in = gridBox(40, 30, 20, 16);       // 6 * 16 * 16 * 2 = 3072 triangles
        Mesh out;
        ReduceOptions o;
        o.toleranceMm = 0.01;
        const ReduceResult r = reduceMesh(in, o, out, 1);
        std::printf("  %zu -> %zu triangles, deviation %.2e mm, %.0f ms\n", r.trianglesBefore,
                    r.trianglesAfter, r.deviationMm, r.milliseconds);
        check(r.ok, "reduces: " + r.error);
        check(r.trianglesBefore == 3072, "from 3072 triangles");
        check(r.trianglesAfter == 12, "to twelve, the fewest a box can have");
        check(r.deviationMm < 1e-9, "without moving the surface at all");
        check(std::fabs(checkHealth(out, false).volume - 24000.0) < 1e-6, "keeping the volume exactly");
        check(eulerOf(out) == 2 && checkHealth(out, false).watertight, "still one closed surface");
        const Vec3 corners[8] = {{-20, -15, -10}, {20, -15, -10}, {-20, 15, -10}, {20, 15, -10},
                                 {-20, -15, 10},  {20, -15, 10},  {-20, 15, 10},  {20, 15, 10}};
        int found = 0;
        for (const Vec3& c : corners) found += hasVertexNear(out, c, 1e-9) ? 1 : 0;
        check(found == 8, "with all eight corners exactly where they were");
    }

    // --- curved, closed, genus zero ---------------------------------------
    std::printf("--- a sphere, held to a tolerance ---\n");
    {
        const Mesh in = uvSphere(20, 90, 180);        // 32,040 triangles
        for (Real tol : {0.2, 0.05, 0.01}) {
            Mesh out;
            ReduceOptions o;
            o.toleranceMm = tol;
            const ReduceResult r = reduceMesh(in, o, out, 2);
            const Real dense = r.ok ? denseDeviation(in, out, 24, 7) : -1;
            std::printf("  tol %.2f: %zu -> %zu triangles, measured %.4f, dense %.4f mm, %.0f ms\n", tol,
                        r.trianglesBefore, r.trianglesAfter, r.deviationMm, dense, r.milliseconds);
            check(r.ok, "reduces: " + r.error);
            check(r.deviationMm <= tol && r.withinTolerance, "its own measurement is within tolerance");
            check(dense <= tol, "and so is dense random sampling");
            check(eulerOf(out) == 2 && checkHealth(out, false).watertight, "still a closed sphere");
            // How much smaller depends on how the tolerance compares with the
            // input's own facets: at 0.01mm on a sphere faceted at 0.7mm it is
            // near the input's resolution, and a third is about what exists.
            const size_t factor = tol >= 0.2 ? 40 : (tol >= 0.05 ? 10 : 2);
            check(r.trianglesAfter * factor < r.trianglesBefore, "and substantially smaller");
            check(checkHealth(out, false).volume > 0, "facing outward");
        }
    }

    // --- an organic surface, with spikes ------------------------------------
    std::printf("--- a bumpy, spiky blob ---\n");
    {
        const Mesh in = spikyBlob(15, 100, 200);      // 39,600 triangles
        const MeshHealth before = checkHealth(in, false);
        for (Real tol : {0.02, 0.1}) {
            Mesh out;
            ReduceOptions o;
            o.toleranceMm = tol;
            const ReduceResult r = reduceMesh(in, o, out, 20);
            const Real dense = r.ok ? denseDeviation(in, out, 8, 21) : -1;
            const MeshHealth h = checkHealth(out, true);
            std::printf("  tol %.2f: %zu -> %zu triangles, %d pass%s, measured %.4f, dense %.4f mm, "
                        "self-intersections %d, %.0f ms\n", tol, r.trianglesBefore, r.trianglesAfter,
                        r.passes, r.passes == 1 ? "" : "es", r.deviationMm, dense, h.selfIntersections,
                        r.milliseconds);
            check(r.ok, "reduces: " + r.error);
            check(r.withinTolerance && r.deviationMm <= tol, "verified within tolerance");
            check(dense <= tol, "and dense sampling agrees");
            check(eulerOf(out) == 2 && h.watertight, "still one closed surface");
            check(h.selfIntersections == 0, "that does not pass through itself");
            check(h.volume > 0 && std::fabs(h.volume - before.volume) < before.volume * 0.02,
                  "facing out, with its volume barely moved");
            check(r.trianglesAfter * 3 < r.trianglesBefore, "and much smaller");
        }
    }

    // --- the second and third passes ---------------------------------------
    // With the first pass held no tighter than the tolerance itself, the same
    // blob comes back over it in places, and the passes that tighten around
    // those places have to bring it in. Pinned to the counts this mesh needs,
    // so that both the local pass and the third, everywhere-a-little pass stay
    // exercised; a change to the algorithm that moves them should be looked at,
    // not waved through.
    std::printf("--- verification sends it back, and it comes back right ---\n");
    {
        const Mesh in = spikyBlob(15, 100, 200);
        for (const auto& [tol, expectPasses] : {std::pair<Real, int>{0.1, 2}, {0.2, 3}}) {
            Mesh out;
            ReduceOptions o;
            o.toleranceMm = tol;
            o.firstPassMargin = 1.0;
            o.sharpCreaseMargin = 1.0;
            const ReduceResult r = reduceMesh(in, o, out, 20);
            const Real dense = r.ok ? denseDeviation(in, out, 8, 23) : -1;
            std::printf("  tol %.2f with no margin: %d passes, measured %.4f, dense %.4f mm\n", tol,
                        r.passes, r.deviationMm, dense);
            check(r.ok && r.passes == expectPasses, "needs the passes it is expected to");
            check(r.withinTolerance && dense <= tol, "and ends within tolerance");
            check(eulerOf(out) == 2 && checkHealth(out, true).selfIntersections == 0, "still whole");
        }
    }

    // --- a handle survives ------------------------------------------------
    std::printf("--- a torus keeps its hole ---\n");
    {
        const Mesh in = torus(20, 4, 160, 48);
        Mesh out;
        ReduceOptions o;
        o.toleranceMm = 0.3;
        const ReduceResult r = reduceMesh(in, o, out, 3);
        const Real dense = r.ok ? denseDeviation(in, out, 6, 11) : -1;
        std::printf("  %zu -> %zu triangles, measured %.4f, dense %.4f mm\n", r.trianglesBefore,
                    r.trianglesAfter, r.deviationMm, dense);
        check(r.ok && r.withinTolerance, "reduces within tolerance: " + r.error);
        check(eulerOf(out) == 0, "genus one: Euler characteristic zero");
        check(dense <= o.toleranceMm, "within tolerance");
        // Pushed as far as it will go, the hole still has to be there.
        Mesh far;
        ReduceOptions big;
        big.toleranceMm = 3.0;
        const ReduceResult rf = reduceMesh(in, big, far, 4);
        std::printf("  at 3mm: %zu triangles\n", rf.trianglesAfter);
        check(rf.ok && eulerOf(far) == 0, "and still has it at a tolerance nearly the tube's radius");
    }

    // --- an open surface keeps its edge ------------------------------------
    std::printf("--- an open surface keeps its boundary ---\n");
    {
        const Mesh in = bumpyDisc(30, 60, 120);
        int bIn = 0, bOut = 0;
        const long eIn = eulerOf(in, &bIn);
        Mesh out;
        ReduceOptions o;
        o.toleranceMm = 0.05;
        const ReduceResult r = reduceMesh(in, o, out, 5);
        const long eOut = eulerOf(out, &bOut);
        const Real dense = r.ok ? denseDeviation(in, out, 6, 13) : -1;
        std::printf("  %zu -> %zu triangles, boundary edges %d -> %d, dense %.4f mm\n",
                    r.trianglesBefore, r.trianglesAfter, bIn, bOut, dense);
        check(r.ok && r.withinTolerance, "reduces within tolerance: " + r.error);
        check(eIn == eOut && eIn == 1, "still a disc");
        check(bOut > 0, "still open");
        check(dense <= o.toleranceMm, "within tolerance, rim included");
    }

    // --- a wall thinner than it is long -----------------------------------
    std::printf("--- a thin wall is not folded onto itself ---\n");
    {
        const Mesh in = gridBox(50, 50, 0.4, 24);
        Mesh out;
        ReduceOptions o;
        o.toleranceMm = 0.05;
        const ReduceResult r = reduceMesh(in, o, out, 6);
        const MeshHealth h = checkHealth(out, true);
        std::printf("  %zu -> %zu triangles, volume %.4f, self-intersections %d\n", r.trianglesBefore,
                    r.trianglesAfter, h.volume, h.selfIntersections);
        check(r.ok, "reduces: " + r.error);
        check(std::fabs(h.volume - 1000.0) < 1e-6, "keeps its volume");
        check(h.selfIntersections == 0, "does not pass through itself");
        check(r.trianglesAfter == 12, "and is still just a box");

        // And with a tolerance larger than the wall: allowed to lose detail,
        // never allowed to break.
        Mesh coarse;
        ReduceOptions loose;
        loose.toleranceMm = 0.3;
        const ReduceResult rc = reduceMesh(gridBox(50, 50, 0.4, 24), loose, coarse, 7);
        const MeshHealth hc = checkHealth(coarse, true);
        check(rc.ok && eulerOf(coarse) == 2 && hc.watertight, "a loose tolerance still gives a closed surface");
        check(hc.selfIntersections == 0 && hc.volume > 0, "that does not pass through itself");
    }

    // --- a triangle budget -------------------------------------------------
    std::printf("--- a target count, and saying when it is not met ---\n");
    {
        const Mesh in = uvSphere(20, 90, 180);
        Mesh out;
        ReduceOptions o;
        o.toleranceMm = 2.0;
        o.targetTriangles = 1000;
        const ReduceResult r = reduceMesh(in, o, out, 8);
        std::printf("  target 1000 at 2mm: %zu triangles, reached=%d, measured %.3f mm\n",
                    r.trianglesAfter, (int)r.reachedTarget, r.deviationMm);
        check(r.ok && r.reachedTarget && r.trianglesAfter <= 1000 && r.trianglesAfter > 900,
              "stops at the target");
        check(r.deviationMm <= 2.0, "and says how far that moved the surface");

        Mesh tight;
        ReduceOptions t;
        t.toleranceMm = 0.001;
        t.targetTriangles = 1000;
        const ReduceResult rt = reduceMesh(in, t, tight, 9);
        std::printf("  target 1000 at 0.001mm: %zu triangles, reached=%d\n", rt.trianglesAfter,
                    (int)rt.reachedTarget);
        check(rt.ok && !rt.reachedTarget && rt.trianglesAfter > 1000,
              "a tolerance too tight for the target stops short and says so");
    }

    // --- the same every time -----------------------------------------------
    {
        const Mesh in = torus(20, 4, 80, 24);
        Mesh a, b;
        ReduceOptions o;
        o.toleranceMm = 0.2;
        reduceMesh(in, o, a, 10);
        reduceMesh(in, o, b, 10);
        bool same = a.verts.size() == b.verts.size() && a.faces.size() == b.faces.size();
        for (size_t i = 0; same && i < a.verts.size(); ++i)
            same = a.verts[i].position.x == b.verts[i].position.x &&
                   a.verts[i].position.y == b.verts[i].position.y &&
                   a.verts[i].position.z == b.verts[i].position.z;
        check(same, "the same mesh reduces to the same result");
    }

    // --- refusals ----------------------------------------------------------
    {
        Mesh out;
        ReduceOptions zero;
        zero.toleranceMm = 0;
        check(!reduceMesh(gridBox(10, 10, 10, 2), zero, out, 11).ok, "a zero tolerance is refused");
        Mesh empty;
        check(!reduceMesh(empty, ReduceOptions{}, out, 12).ok, "an empty mesh is refused");
    }

    // --- what it costs ------------------------------------------------------
    std::printf("--- a large mesh ---\n");
    {
        const Mesh in = uvSphere(30, 180, 360);       // 128,880 triangles
        Mesh out;
        ReduceOptions o;
        o.toleranceMm = 0.05;
        const ReduceResult r = reduceMesh(in, o, out, 13);
        std::printf("  %zu -> %zu triangles in %.0f ms, measured %.4f mm\n", r.trianglesBefore,
                    r.trianglesAfter, r.milliseconds, r.deviationMm);
        check(r.ok && eulerOf(out) == 2, "reduces and stays closed");
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
