// Booleans, checked against volumes that can be worked out by hand, and
// against the contract that a result is either a valid solid or a refusal.
#include "mesh/boolean.h"
#include "mesh/health.h"
#include "mesh/primitives.h"

#include <cstdio>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near(double a, double b, double eps = 1e-6) { return std::fabs(a - b) < eps; }

static Mesh boxAt(Vec3 centre, Real size = 20.0) {
    Mesh m;
    BoxParams p; p.width = p.depth = p.height = size;
    makeBox(m, p);
    for (MeshVertex& v : m.verts) v.position += centre;
    return m;
}

static double volumeOf(const Mesh& m) { return checkHealth(m, false).volume; }

static void expectSolid(const Mesh& m, const char* what) {
    const MeshHealth h = checkHealth(m);
    check(h.watertight, std::string(what) + ": watertight");
    check(h.volume > 0.0, std::string(what) + ": positive volume");
    check(h.degenerateFaces == 0, std::string(what) + ": no degenerate faces");
    check(h.selfIntersections == 0, std::string(what) + ": no self-intersections");
    std::string err;
    check(m.validate(&err), std::string(what) + ": " + err);
}

int main() {
    // Two 20mm cubes overlapping over a 10mm slab: the three results have
    // volumes that can be computed exactly.
    //   A: x in [-10, 10]      B: x in [0, 20]      overlap 10 x 20 x 20 = 4000
    const Mesh a = boxAt({0, 0, 0});
    const Mesh b = boxAt({10, 0, 0});
    check(near(volumeOf(a), 8000.0, 1e-3), "input A is 8000");
    check(near(volumeOf(b), 8000.0, 1e-3), "input B is 8000");

    {
        Mesh out;
        check(meshBoolean(a, b, BooleanOp::Union, out), "union succeeds");
        expectSolid(out, "union");
        check(near(volumeOf(out), 12000.0, 1e-3), "union volume is 8000+8000-4000");
        std::printf("[bool] union        %.3f  (expect 12000)  %d faces\n",
                    volumeOf(out), out.faceCount());
    }
    {
        Mesh out;
        check(meshBoolean(a, b, BooleanOp::Intersection, out), "intersection succeeds");
        expectSolid(out, "intersection");
        check(near(volumeOf(out), 4000.0, 1e-3), "intersection is the 10x20x20 slab");
        const AABB box = out.bounds();
        check(near(box.min.x, 0.0, 1e-6) && near(box.max.x, 10.0, 1e-6),
              "and sits exactly where the two overlap");
        std::printf("[bool] intersection %.3f  (expect 4000)   x in [%.3f, %.3f]\n",
                    volumeOf(out), box.min.x, box.max.x);
    }
    {
        Mesh out;
        check(meshBoolean(a, b, BooleanOp::Difference, out), "difference succeeds");
        expectSolid(out, "difference");
        check(near(volumeOf(out), 4000.0, 1e-3), "A minus B leaves 4000");
        const AABB box = out.bounds();
        check(near(box.max.x, 0.0, 1e-6), "cut exactly at the boundary plane");
        std::printf("[bool] difference   %.3f  (expect 4000)   x in [%.3f, %.3f]\n",
                    volumeOf(out), box.min.x, box.max.x);
    }

    // Difference is not symmetric.
    {
        Mesh ab, ba;
        check(meshBoolean(a, b, BooleanOp::Difference, ab), "A-B");
        check(meshBoolean(b, a, BooleanOp::Difference, ba), "B-A");
        check(near(volumeOf(ab), 4000.0, 1e-3) && near(volumeOf(ba), 4000.0, 1e-3),
              "both halves are 4000 here");
        check(near(ab.bounds().max.x, 0.0, 1e-6), "A-B keeps the left part");
        check(near(ba.bounds().min.x, 10.0, 1e-6), "B-A keeps the right part");
        std::printf("[bool] difference is not symmetric: A-B x<=%.1f, B-A x>=%.1f\n",
                    ab.bounds().max.x, ba.bounds().min.x);
    }

    // ---- Disjoint solids ---------------------------------------------------
    {
        const Mesh far = boxAt({100, 0, 0});
        Mesh out;
        check(meshBoolean(a, far, BooleanOp::Union, out), "union of disjoint solids");
        check(near(volumeOf(out), 16000.0, 1e-3), "volumes simply add");
        check(checkHealth(out).shells == 2, "reported as two shells");

        // Nothing in common: there is no solid to return.
        Mesh empty;
        check(!meshBoolean(a, far, BooleanOp::Intersection, empty),
              "intersection of disjoint solids is refused, not empty geometry");
        std::printf("[bool] disjoint: union %.0f in 2 shells, intersection refused\n",
                    volumeOf(out));
    }

    // ---- Containment --------------------------------------------------------
    {
        const Mesh big = boxAt({0, 0, 0}, 40.0);
        const Mesh small = boxAt({0, 0, 0}, 10.0);

        Mesh u, i, d;
        check(meshBoolean(big, small, BooleanOp::Union, u), "union with contained solid");
        check(near(volumeOf(u), 64000.0, 1e-3), "union is just the larger solid");

        check(meshBoolean(big, small, BooleanOp::Intersection, i), "intersection");
        check(near(volumeOf(i), 1000.0, 1e-3), "intersection is the smaller solid");

        // A cavity: the result is a shell with a void inside it, which is a
        // perfectly good solid and prints as a hollow box.
        check(meshBoolean(big, small, BooleanOp::Difference, d), "difference makes a cavity");
        check(near(volumeOf(d), 63000.0, 1e-3), "64000 minus the 1000 void");
        check(checkHealth(d).watertight, "cavity result is still closed");
        std::printf("[bool] cavity: %.0f mm3 with a void, watertight\n", volumeOf(d));

        // Removing everything leaves nothing to return.
        Mesh nothing;
        check(!meshBoolean(small, big, BooleanOp::Difference, nothing),
              "subtracting a solid that swallows the other is refused");
    }

    // ---- Curved surfaces ----------------------------------------------------
    {
        Mesh cyl;
        CylinderParams cp; cp.radius = 6.0; cp.height = 40.0; cp.segments = 24;
        makeCylinder(cyl, cp);

        Mesh drilled;
        check(meshBoolean(a, cyl, BooleanOp::Difference, drilled), "drill a hole");
        expectSolid(drilled, "drilled box");

        // Bore through a 20mm cube: 8000 minus a 20mm length of the cylinder.
        // The facetted cylinder's area is (n/2)r^2 sin(2pi/n), slightly under pi r^2.
        const double n = 24.0;
        const double area = 0.5 * n * 36.0 * std::sin(2.0 * 3.14159265358979 / n);
        check(near(volumeOf(drilled), 8000.0 - area * 20.0, 1e-2),
              "volume matches the facetted bore exactly");
        std::printf("[bool] drilled box  %.3f  (expect %.3f)\n",
                    volumeOf(drilled), 8000.0 - area * 20.0);
    }

    // Tangency is the same degenerate family as coincident planes: a tool that
    // touches a wall exactly, rather than crossing it or clearing it, gives the
    // classification nothing to decide on. Asserted so the boundary is known
    // rather than discovered mid-model.
    {
        Mesh cyl, out;
        CylinderParams cp; cp.radius = 6.0; cp.height = 40.0; cp.segments = 24;
        makeCylinder(cyl, cp);
        Mesh clear_ = cyl, tangent = cyl;
        for (MeshVertex& v : clear_.verts)  v.position += Vec3{3, 3, 0};   // 3+6 = 9 < 10
        for (MeshVertex& v : tangent.verts) v.position += Vec3{4, 4, 0};   // 4+6 = 10 exactly

        check(meshBoolean(a, clear_, BooleanOp::Difference, out),
              "a bore clear of the walls works");
        expectSolid(out, "offset bore");

        Mesh untouched = boxAt({0, 0, 0}, 5.0);
        const double before = volumeOf(untouched);
        const bool ok = meshBoolean(a, tangent, BooleanOp::Difference, untouched);
        if (!ok) check(near(volumeOf(untouched), before, 1e-9),
                       "a refused tangent cut leaves the output alone");
        std::printf("[bool] bore clear of walls: OK;  exactly tangent: %s\n",
                    ok ? "also OK" : "refused (known limit)");
    }

    // ---- Open input is refused ----------------------------------------------
    {
        Mesh plane, out;
        makePlane(plane);
        check(!meshBoolean(a, plane, BooleanOp::Union, out),
              "an open surface has no inside, so it is refused");
        check(!meshBoolean(plane, a, BooleanOp::Difference, out), "either way round");
        std::printf("[bool] open input refused\n");
    }

    // ---- Chained booleans ---------------------------------------------------
    {
        // First cut takes the [0,10]^3 corner: 1000.
        Mesh step1;
        check(meshBoolean(a, boxAt({10, 10, 10}), BooleanOp::Difference, step1), "cut 1");
        check(near(volumeOf(step1), 7000.0, 1e-3), "first cut leaves 7000");

        // Second cut takes an 8^3 bite from the opposite corner, on planes that
        // do not coincide with anything the first cut created.
        Mesh step2;
        check(meshBoolean(step1, boxAt({-12, -12, 12}), BooleanOp::Difference, step2),
              "cut 2 on independent planes");
        expectSolid(step2, "twice-cut box");
        check(near(volumeOf(step2), 8000.0 - 1000.0 - 512.0, 1e-3),
              "1000 then 512 removed");
        std::printf("[bool] chained cuts %.3f  (expect 6488), %d faces\n",
                    volumeOf(step2), step2.faceCount());
    }

    // ---- The known limitation, pinned down ----------------------------------
    //
    // A cut whose plane lands on top of one an earlier cut already created is
    // near-degenerate for a BSP, and the classification goes wrong: the result
    // comes back with two faces walking the same directed edge. Measured, the
    // threshold is around 1e-3 of the model size -- coincident and 1e-6 offsets
    // both fail, 1e-3 succeeds.
    //
    // Fixing it properly needs exact predicates or an intersection-curve
    // formulation, neither of which is implemented. What IS guaranteed, and is
    // what this asserts, is that the failure is a refusal: the operation
    // returns false and the caller's mesh is untouched. It never returns
    // geometry that looks fine and is not.
    {
        Mesh step1;
        check(meshBoolean(a, boxAt({10, 10, 10}), BooleanOp::Difference, step1), "cut 1");

        Mesh out = boxAt({0, 0, 0}, 5.0);          // recognisable sentinel
        const double before = volumeOf(out);
        const bool ok = meshBoolean(step1, boxAt({-10, -10, 10}), BooleanOp::Difference, out);

        if (ok) {
            // If a future change makes this work, it must still be a solid.
            expectSolid(out, "coincident-plane cut");
            std::printf("[bool] coincident-plane cut now SUCCEEDS: %.3f\n", volumeOf(out));
        } else {
            check(near(volumeOf(out), before, 1e-9), "a refused boolean leaves `out` alone");
            std::printf("[bool] coincident-plane cut refused, output untouched (known limit)\n");
        }
    }

    // ---- Touching parts become one face ------------------------------------
    // A boolean cuts every face it touches into triangles and leaves a fan of
    // them where one flat surface used to be. None of those edges are on the
    // model, only in the data, and picking one selects a sliver of what the
    // user sees as a face -- or a piece that disappears under an adjoining
    // part. Coplanar neighbours are merged back into one face afterwards.
    {
        Mesh lo, hi, out;
        BoxParams pl{20, 20, 20};
        BoxParams ph{20, 20, 40};
        makeBox(lo, pl);
        makeBox(hi, ph);
        for (auto& v : hi.verts) v.position += Vec3{20, 0, 10};

        check(meshBoolean(lo, hi, BooleanOp::Union, out), "union of two boxes side by side");
        check(checkHealth(out).solid(), "the L is a solid");

        int seams = 0;
        for (Index he = 0; he < out.halfedgeCount(); ++he) {
            const Index tw = out.halfedges[he].twin;
            if (he > tw) continue;
            const Index a = out.halfedges[he].face, b = out.halfedges[tw].face;
            if (a == kInvalid || b == kInvalid) continue;
            if (dot(out.faceNormal(a), out.faceNormal(b)) > 0.9999619) ++seams;
        }
        check(seams == 0, "no edge is left between two faces in the same plane");

        // An L-shaped solid has eight faces and no more: six walls and two
        // ends. Anything above that is seams that were not merged.
        check(out.faceCount() == 8,
              "the L comes out as eight faces, not a triangle fan (" +
                  std::to_string(out.faceCount()) + ")");
        std::printf("[bool] two touching boxes merge into %d faces, %d seams\n",
                    out.faceCount(), seams);
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
