// Splitting an exact body, and the whole path a large mesh takes to get there.
//
// Split used to call the mesh implementation whatever the body was, so an
// exact body -- including every mesh converted to one -- could not be split at
// all. The checks are arithmetic: the pieces' volumes add up to the whole, each
// piece is closed, and a plane that misses is refused rather than producing an
// empty half.
#include "geom/operations.h"
#include "mesh/decimate.h"
#include "mesh/import_mesh.h"
#include "mesh/health.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near1(Real got, Real want, Real rel = 1e-6) {
    return std::fabs(got - want) <= std::fabs(want) * rel + 1e-9;
}
static Real volume(const Body& b) { return b.health(false).volume; }
static double msSince(std::chrono::steady_clock::time_point t) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
}

static Body box(Real w, Real d, Real h, Vec3 at = {}) {
    PrimitiveSpec s;
    s.kind = PrimitiveKind::Box;
    s.box = {w, d, h};
    Body b;
    makePrimitive(s, b, Backend::Brep);
    if (length(at) > 0) b.transform(translate(at));
    return b;
}

// A sphere with bumps and spikes, as a mesh -- the stand-in for a scan.
static Body blob() {
    const int rings = 80, segs = 160;
    const Real R = 15;
    std::vector<Vec3> p;
    std::vector<uint32_t> sizes, idx;
    auto radius = [&](Real th, Real ph) {
        Real r = R * (1 + 0.06 * std::sin(5 * th) * std::cos(7 * ph));
        const Vec3 dir{std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th)};
        for (Vec3 tip : {Vec3{1, 0, 0}, Vec3{0, 0, 1}, Vec3{0, -0.6, -0.8}}) {
            const Real c = dot(dir, normalize(tip));
            if (c > 0.985) r += R * 0.25 * (c - 0.985) / 0.015;
        }
        return r;
    };
    p.push_back({0, 0, radius(0, 0)});
    for (int i = 1; i < rings; ++i)
        for (int j = 0; j < segs; ++j) {
            const Real th = kPi * i / rings, ph = 2 * kPi * j / segs, r = radius(th, ph);
            p.push_back({r * std::sin(th) * std::cos(ph), r * std::sin(th) * std::sin(ph), r * std::cos(th)});
        }
    p.push_back({0, 0, -radius(kPi, 0)});
    const uint32_t south = static_cast<uint32_t>(p.size() - 1);
    auto V = [&](int i, int j) { return static_cast<uint32_t>(1 + (i - 1) * segs + (j % segs)); };
    auto tri = [&](uint32_t a, uint32_t b, uint32_t c) { sizes.push_back(3); idx.insert(idx.end(), {a, b, c}); };
    for (int j = 0; j < segs; ++j) tri(0, V(1, j), V(1, j + 1));
    for (int i = 1; i < rings - 1; ++i)
        for (int j = 0; j < segs; ++j) { tri(V(i, j), V(i + 1, j), V(i + 1, j + 1)); tri(V(i, j), V(i + 1, j + 1), V(i, j + 1)); }
    for (int j = 0; j < segs; ++j) tri(V(rings - 1, j), south, V(rings - 1, j + 1));
    Mesh m;
    m.build(p, sizes, idx);
    return Body(std::move(m));
}

int main() {
    if (!brep::available()) {
        std::printf("exact kernel not built; splitting an exact body needs one\n");
        return 0;
    }
    std::printf("split\n");

    std::printf("--- a box, cut in two ---\n");
    {
        const Body b = box(40, 30, 20);
        Body a, c;
        check(splitByPlane(b, {10, 0, 0}, {1, 0, 0}, a, c), "splits along x = 10");
        check(!a.isMesh() && !c.isMesh(), "into exact pieces");
        check(near1(volume(a) + volume(c), 24000), "whose volumes add up to the whole");
        check(near1(std::min(volume(a), volume(c)), 10 * 30 * 20), "one of them 10mm thick");
        check(brep::closedShell(a.brep()) && brep::closedShell(c.brep()), "each one closed");
        check(a.faceCount() == 6 && c.faceCount() == 6, "each one a box");
    }

    std::printf("--- a tilted plane ---\n");
    {
        const Body b = box(40, 30, 20);
        Body a, c;
        check(splitByPlane(b, {0, 0, 0}, normalize(Vec3{1, 1, 1}), a, c), "splits diagonally");
        check(near1(volume(a) + volume(c), 24000), "and nothing is lost");
        check(near1(volume(a), volume(c), 1e-6), "into two equal halves, by symmetry");
    }

    std::printf("--- curved bodies cut where their corners are ---\n");
    {
        // A cylinder cut through its axis: one half keeps the seam's vertices,
        // and the other can be left with every vertex on the cut -- only its
        // curved face tells which side it is on. A sphere cut at its equator,
        // and a torus cut through its middle, where the edges lie in the plane
        // too. Each half has to survive being told apart.
        PrimitiveSpec cyl;
        cyl.kind = PrimitiveKind::Cylinder;
        cyl.cylinder = {10, 20, 48};
        PrimitiveSpec sph;
        sph.kind = PrimitiveKind::Sphere;
        sph.sphere = {10, 32, 16};
        PrimitiveSpec tor;
        tor.kind = PrimitiveKind::Torus;
        tor.torus = {20, 5, 48, 24};
        struct Case { const char* name; PrimitiveSpec spec; Vec3 n; };
        for (const Case& c : {Case{"cylinder through its axis", cyl, {1, 0, 0}},
                              Case{"cylinder through its axis, the other way", cyl, {0, 1, 0}},
                              Case{"sphere at its equator", sph, {0, 0, 1}},
                              Case{"torus through its middle", tor, {0, 0, 1}}}) {
            Body b;
            makePrimitive(c.spec, b, Backend::Brep);
            Body x, y;
            const bool ok = splitByPlane(b, {0, 0, 0}, c.n, x, y);
            check(ok, std::string(c.name) + ": splits");
            if (!ok) continue;
            check(near1(volume(x) + volume(y), volume(b), 1e-6), std::string(c.name) + ": nothing lost");
            check(near1(volume(x), volume(y), 1e-4), std::string(c.name) + ": two equal halves");
        }
    }

    std::printf("--- two cylinders, each cut through its axis ---\n");
    {
        // Each side then holds two half-cylinders, either of which may have
        // every vertex on the cut. A side that kept one and quietly dropped the
        // other would not come back empty, so nothing but the arithmetic below
        // would notice.
        PrimitiveSpec cyl;
        cyl.kind = PrimitiveKind::Cylinder;
        cyl.cylinder = {5, 20, 48};
        Body one, two, both;
        makePrimitive(cyl, one, Backend::Brep);
        makePrimitive(cyl, two, Backend::Brep);
        two.transform(translate({30, 0, 0}));
        check(booleanOp(one, two, BooleanOp::Union, both, 3, false, nullptr), "the pair builds");
        for (Vec3 n : {Vec3{0, 1, 0}, Vec3{0, -1, 0}}) {
            Body x, y;
            check(splitByPlane(both, {0, 0, 0}, n, x, y), "splits through both axes");
            check(near1(volume(x) + volume(y), volume(both), 1e-6), "and no half goes missing");
            std::vector<Body> px, py;
            check(splitBodies(x, px) == 2 && splitBodies(y, py) == 2, "two halves on each side");
        }
    }

    std::printf("--- a plane that misses ---\n");
    {
        const Body b = box(10, 10, 10);
        Body a, c;
        check(!splitByPlane(b, {50, 0, 0}, {1, 0, 0}, a, c), "is refused");
        check(!splitByPlane(b, {5, 0, 0}, {1, 0, 0}, a, c), "and so is one that only touches a face");
    }

    std::printf("--- a U, cut across both arms ---\n");
    {
        // A U-shaped part: a base with two uprights. A plane across the uprights
        // leaves two separate solids on the far side, which splitBodies then
        // separates.
        Body base = box(40, 10, 10, {0, 0, 0});
        Body left = box(10, 10, 30, {-15, 0, 20});
        Body right = box(10, 10, 30, {15, 0, 20});
        Body u1, u;
        check(booleanOp(base, left, BooleanOp::Union, u1, 1, false, nullptr) &&
              booleanOp(u1, right, BooleanOp::Union, u, 2, false, nullptr), "the U builds");
        Body low, high;
        check(splitByPlane(u, {0, 0, 20}, {0, 0, 1}, high, low), "cuts across both arms");
        std::vector<Body> pieces;
        const size_t n = splitBodies(high, pieces);
        check(n == 2, "leaving the two arm ends as two solids");
        if (n == 2) {
            check(near1(volume(pieces[0]), 10 * 10 * 15) && near1(volume(pieces[1]), 10 * 10 * 15),
                  "each the part of an upright above the cut");
            check(brep::closedShell(pieces[0].brep()) && brep::closedShell(pieces[1].brep()),
                  "each closed");
        }
        std::vector<Body> one;
        check(splitBodies(low, one) == 1, "while the side with the base stays one piece");
    }

    // --- the path a scan takes ----------------------------------------------
    // Reduce, convert, then the things a converted mesh is for: a boolean, and a
    // split. Timed, and each step checked to have made a closed solid.
    std::printf("--- a scanned-looking mesh, all the way to a split ---\n");
    {
        Body mesh = blob();
        const Real meshVolume = volume(mesh);
        auto t0 = std::chrono::steady_clock::now();
        ReduceOptions o;
        o.toleranceMm = 0.05;
        o.targetTriangles = kSolidifyFaceLimit;
        ReduceResult rr;
        check(reduceBody(mesh, o, 7, rr), "reduces: " + rr.error);
        const double tReduce = msSince(t0);

        t0 = std::chrono::steady_clock::now();
        Body solid = mesh;
        const SolidifyResult sr = toSolid(solid, 8);
        const double tConvert = msSince(t0);
        check(sr.ok, "converts: " + sr.error);
        check(!solid.isMesh() && brep::closedShell(solid.brep()), "to a closed solid");
        check(near1(volume(solid), meshVolume, 0.01), "the size it was");

        t0 = std::chrono::steady_clock::now();
        Body drilled;
        PrimitiveSpec cyl;
        cyl.kind = PrimitiveKind::Cylinder;
        cyl.cylinder = {3, 60, 48};
        Body tool;
        makePrimitive(cyl, tool, Backend::Brep);
        std::string why;
        const bool bored = booleanOp(solid, tool, BooleanOp::Difference, drilled, 9, false, &why);
        const double tBoolean = msSince(t0);
        check(bored, "a hole bores through it: " + why);
        if (bored) {
            check(brep::closedShell(drilled.brep()), "and it is still closed");
            check(volume(drilled) < volume(solid) && volume(drilled) > volume(solid) * 0.8,
                  "and lighter by about a hole's worth");
        }

        t0 = std::chrono::steady_clock::now();
        Body top, bottom;
        const bool cut = splitByPlane(bored ? drilled : solid, {0, 0, 2}, {0, 0, 1}, top, bottom);
        const double tSplit = msSince(t0);
        check(cut, "it splits");
        if (cut) {
            const Real whole = volume(bored ? drilled : solid);
            check(near1(volume(top) + volume(bottom), whole, 1e-6), "into pieces that add up");
            check(brep::closedShell(top.brep()) && brep::closedShell(bottom.brep()), "each closed");
        }
        std::printf("  %zu -> %zu triangles (%.0f ms), %d faces as a solid (%.0f ms), "
                    "boolean %.0f ms, split %.0f ms\n", rr.trianglesBefore, rr.trianglesAfter,
                    tReduce, sr.facesAfter, tConvert, tBoolean, tSplit);
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
