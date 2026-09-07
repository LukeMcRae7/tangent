// Booleans, checked against volumes that can be worked out by hand, and
// against the contract that a result is either a valid solid or a refusal.
#include "mesh/boolean.h"
#include "mesh/health.h"
#include "mesh/operations.h"
#include "mesh/primitives.h"

#include <cstdio>
#include <random>
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

    // ---- A body can be cut more than once -----------------------------------
    // It could not before. The boolean's own output was not something it could
    // consume: forty triangles going in came back as three and a half thousand,
    // and the rebuild refused them. Face-preserving classification is what
    // makes the operation composable.
    {
        Mesh body;
        makeBox(body);
        BoxParams barSpec{6, 6, 30};
        Mesh bar;
        makeBox(bar, barSpec);

        const Vec3 places[3] = {{-5, 0, 0}, {5, 0, 0}, {0, 5, 0}};
        double volumes[3] = {0, 0, 0};
        for (int i = 0; i < 3; ++i) {
            Mesh tool = bar, next;
            for (auto& v : tool.verts) v.position += places[i];
            check(meshBoolean(body, tool, BooleanOp::Difference, next),
                  "cut " + std::to_string(i + 1) + " succeeds");
            if (next.faceCount() == 0) break;
            body = std::move(next);
            check(checkHealth(body).solid(), "cut " + std::to_string(i + 1) + " stays solid");
            volumes[i] = volumeOf(body);
        }
        // The first cut is a clean 6x6 bar through a 20mm cube.
        check(std::fabs(volumes[0] - 7280.0) < 1e-6,
              "one bar through a cube leaves 7280 (" + std::to_string(volumes[0]) + ")");
        check(volumes[1] < volumes[0] && volumes[2] < volumes[1],
              "each further cut removes more");
        std::printf("[bool] three cuts on one body: %.1f -> %.1f -> %.1f mm3\n",
                    volumes[0], volumes[1], volumes[2]);
    }

    // ---- Divisions represent real edges -------------------------------------
    // A subtracted sphere used to leave the cube's flat walls covered in
    // zigzags: 329 edges between faces that were already in the same plane,
    // none of them anywhere the surface turned.
    {
        Mesh cube, sphere, out;
        makeBox(cube);
        SphereParams sp;
        sp.radius = 7;
        sp.segments = 24;
        sp.rings = 12;
        makeSphere(sphere, sp);
        for (auto& v : sphere.verts) v.position += Vec3{0, 0, 10};

        check(meshBoolean(cube, sphere, BooleanOp::Difference, out), "dish a sphere into a cube");
        check(checkHealth(out).solid(), "the dished cube is solid");

        int seams = 0;
        for (Index he = 0; he < out.halfedgeCount(); ++he) {
            const Index tw = out.halfedges[he].twin;
            if (he > tw) continue;
            const Index a = out.halfedges[he].face, b = out.halfedges[tw].face;
            if (a == kInvalid || b == kInvalid) continue;
            if (dot(out.faceNormal(a), out.faceNormal(b)) > 0.9999619) ++seams;
        }
        // Two: the pair of cuts that let the dished face be held as two simple
        // polygons, since this mesh cannot hold a face with a hole in it.
        check(seams <= 2, "at most two edges lie inside a flat region (" +
                              std::to_string(seams) + ")");
        std::printf("[bool] cube less a sphere: %d faces, %d seams\n", out.faceCount(), seams);

        // ---- Downstream operations on boolean result -----------------------
        // Fillet the bottom edges of the dished cube
        {
            std::vector<Index> bottomEdges;
            for (Index h = 0; h < out.halfedgeCount(); ++h) {
                if (h > out.halfedges[h].twin) continue;
                const Vec3 p1 = out.verts[out.fromVertex(h)].position;
                const Vec3 p2 = out.verts[out.halfedges[h].vertex].position;
                if (std::fabs(p1.z - (-10)) < 1e-4 && std::fabs(p2.z - (-10)) < 1e-4)
                    bottomEdges.push_back(h);
            }
            FilletSpec bspec;
            bspec.segments = 4;
            for (Index e : bottomEdges) bspec.edges.push_back({e, 2.0});
            Mesh bFilleted = out;
            check(filletEdges(bFilleted, bspec), "fillet bottom edges after boolean");
            expectSolid(bFilleted, "filleted dished cube");
        }

        // Extrude a side face of the dished cube
        {
            Index sideFace = kInvalid;
            for (Index f = 0; f < out.faceCount(); ++f) {
                if (out.faceNormal(f).x > 0.99) { sideFace = f; break; }
            }
            check(sideFace != kInvalid, "found side face");
            Mesh extruded = out;
            check(extrudeFaces(extruded, {sideFace}, 5.0), "extrude side face after boolean");
            expectSolid(extruded, "extruded dished cube");
        }
    }

    // ---- Multiple overlapping curved booleans in sequence -------------------
    {
        Mesh cube, sphere;
        makeBox(cube);
        SphereParams sp;
        sp.radius = 7;
        sp.segments = 24;
        sp.rings = 12;
        makeSphere(sphere, sp);
        for (auto& v : sphere.verts) v.position += Vec3{0, 0, 10};

        Mesh cut1;
        check(meshBoolean(cube, sphere, BooleanOp::Difference, cut1), "curved cut 1");
        expectSolid(cut1, "cut 1");

        // Second overlapping sphere cut on the curved geometry
        Mesh sphere2 = sphere;
        for (auto& v : sphere2.verts) v.position += Vec3{5, 0, 0};
        Mesh cut2;
        check(meshBoolean(cut1, sphere2, BooleanOp::Difference, cut2), "overlapping curved cut 2");
        expectSolid(cut2, "cut 2");
        check(volumeOf(cut2) < volumeOf(cut1), "second cut removes volume");

        // Third overlapping sphere cut
        Mesh sphere3 = sphere;
        for (auto& v : sphere3.verts) v.position += Vec3{0, 5, 0};
        Mesh cut3;
        check(meshBoolean(cut2, sphere3, BooleanOp::Difference, cut3), "overlapping curved cut 3");
        expectSolid(cut3, "cut 3");
        check(volumeOf(cut3) < volumeOf(cut2), "third cut removes volume");
        std::printf("[bool] 3 overlapping curved cuts: %.1f -> %.1f -> %.1f mm3\n",
                    volumeOf(cut1), volumeOf(cut2), volumeOf(cut3));
    }

    // ---- A face with more than one hole in it ----------------------------
    //
    // A boolean shreds every face it cuts and mergeCoplanarFaces puts the
    // pieces back. It used to manage a face with one hole and give up entirely
    // on a face with two, so the second hole left the whole fan of fragments on
    // the model as visible lines across a flat surface. Since a face here holds
    // one boundary loop, n holes must come out as n+1 pieces -- and that is
    // what these check, because "fewer faces" alone would also be satisfied by
    // merging something away that should have stayed.
    {
        auto plateWithBores = [](int n, Mesh& out) {
            Mesh cur;
            makeBox(cur, {60.0, 60.0, 10.0});
            const int side = n <= 1 ? 1 : (n <= 4 ? 2 : 3);
            int made = 0;
            for (int i = 0; i < side; ++i)
                for (int j = 0; j < side && made < n; ++j) {
                    Mesh drill;
                    makeCylinder(drill, {3.0, 30.0, 16});
                    const Real step = 48.0 / side;
                    const Vec3 at{-24.0 + step * (i + 0.5), -24.0 + step * (j + 0.5), 0.0};
                    for (MeshVertex& v : drill.verts) v.position += at;
                    Mesh next;
                    if (!meshBoolean(cur, drill, BooleanOp::Difference, next, 300 + made))
                        return false;
                    cur = std::move(next);
                    ++made;
                }
            out = std::move(cur);
            return true;
        };

        auto piecesOnTop = [](const Mesh& m) {
            int n = 0;
            for (Index f = 0; f < m.faceCount(); ++f)
                if (m.faceNormal(f).z > 0.99 && std::fabs(m.faceCentroid(f).z - 5.0) < 1e-3) ++n;
            return n;
        };

        // A 16-sided bore of radius 3 removes this much from a 10mm plate.
        const double bore = 0.5 * 16.0 * 9.0 * std::sin(kTwoPi / 16.0) * 10.0;

        // Nine is here for a second reason. It only succeeds because classify
        // stops splitting a piece once it is clear of the other solid's bounding
        // box: without that, a bore through a plate cuts every face it touches
        // edge to edge into hundreds of slabs, and by the fifth bore one of
        // those arrangements is degenerate enough that the rebuild refuses it.
        for (int n : {1, 2, 4, 9}) {
            Mesh plate;
            const std::string what = std::to_string(n) + " bore" + (n == 1 ? "" : "s");
            check(plateWithBores(n, plate), what + ": every cut succeeds");
            if (plate.empty()) continue;
            expectSolid(plate, "bored plate");
            check(near(volumeOf(plate), 60.0 * 60.0 * 10.0 - n * bore, 1e-3),
                  what + ": volume is exact");
            check(piecesOnTop(plate) == n + 1,
                  what + ": the top comes back as " + std::to_string(n + 1) + " pieces");
            std::printf("[bool] %-8s -> %3d faces, top plane in %d piece%s (ideal %d)\n",
                        what.c_str(), plate.faceCount(), piecesOnTop(plate),
                        piecesOnTop(plate) == 1 ? "" : "s", n + 1);
        }
    }

    // A bridge cut that runs back along a boundary edge leaves a corner of zero
    // angle. The area stays positive and the mesh still validates, so nothing
    // notices until the next boolean has to classify against that sliver and
    // refuses the whole operation. No merged face may contain one.
    {
        Mesh cur;
        makeBox(cur, {20.0, 20.0, 20.0});
        std::mt19937 rng(7);
        std::uniform_real_distribution<double> U(-9.0, 9.0);

        int cuts = 0, spikes = 0;
        std::vector<Index> fv;
        for (int i = 0; i < 40; ++i) {
            Mesh cutter;
            makeBox(cutter, {6.0, 6.0, 40.0});
            const Vec3 at{U(rng), U(rng), 0.0};
            for (MeshVertex& v : cutter.verts) v.position += at;
            Mesh next;
            if (!meshBoolean(cur, cutter, BooleanOp::Difference, next, 500 + i)) continue;
            cur = std::move(next);
            ++cuts;

            for (Index f = 0; f < cur.faceCount(); ++f) {
                cur.faceVertices(f, fv);
                const size_t k = fv.size();
                for (size_t c = 0; c < k; ++c) {
                    const Vec3 a = cur.verts[fv[(c + k - 1) % k]].position;
                    const Vec3 b = cur.verts[fv[c]].position;
                    const Vec3 d = cur.verts[fv[(c + 1) % k]].position;
                    const Vec3 u = a - b, v = d - b;
                    const Real lu = length(u), lv = length(v);
                    if (lu < 1e-12 || lv < 1e-12) { ++spikes; continue; }
                    if (dot(u, v) / (lu * lv) > 1.0 - 1e-9) ++spikes;
                }
            }
        }
        check(cuts > 20, "most of a run of random cuts succeeds");
        check(spikes == 0, "no merged face doubles back on itself");
        std::printf("[bool] %d successive random cuts, %d zero-angle corners\n", cuts, spikes);
    }

    // ---- Nothing is drawn across a flat face ------------------------------
    //
    // A face here holds one boundary loop, so a region with a hole in it has to
    // be cut into pieces and the cuts are real edges in the topology. They are
    // not on the model, though: the surface does not turn at them, and a bore
    // through a plate came back with two lines running from it to the rim. They
    // are dropped from the wireframe and from picking instead of from the mesh.
    //
    // The invariant is the one the user sees: no drawn edge has coplanar faces
    // on both sides that meet along more than one edge.
    auto linesAcrossFlatFaces = [](const Mesh& m) {
        RenderMesh rm;
        m.buildRenderMesh(rm);
        // Positions are per-corner copies, so match drawn segments back to
        // edges by their endpoints.
        int across = 0;
        for (Index h = 0; h < m.halfedgeCount(); ++h) {
            if (h > m.halfedges[h].twin) continue;
            if (!m.isBridgeEdge(h)) continue;
            const Vec3 a = m.verts[m.fromVertex(h)].position;
            const Vec3 b = m.verts[m.halfedges[h].vertex].position;
            for (size_t i = 0; i < rm.edgeLines.size(); i += 2) {
                const Vec3 p = rm.positions[rm.edgeLines[i]];
                const Vec3 q = rm.positions[rm.edgeLines[i + 1]];
                if ((lengthSq(p - a) < 1e-12 && lengthSq(q - b) < 1e-12) ||
                    (lengthSq(p - b) < 1e-12 && lengthSq(q - a) < 1e-12)) { ++across; break; }
            }
        }
        return across;
    };

    {
        // The reported case: a cylindrical pocket in the top of a box.
        Mesh box;
        makeBox(box, {36.0, 60.0, 27.0});
        Mesh drill;
        makeCylinder(drill, {10.0, 20.0, 24});
        for (MeshVertex& v : drill.verts) v.position += Vec3{0, 0, 15.5};
        Mesh pocketed;
        check(meshBoolean(box, drill, BooleanOp::Difference, pocketed, 7), "pocket cut");

        int bridges = 0;
        for (Index h = 0; h < pocketed.halfedgeCount(); ++h)
            if (h < pocketed.halfedges[h].twin && pocketed.isBridgeEdge(h)) ++bridges;
        check(bridges == 2, "the bored face is bridged by two cuts");
        check(linesAcrossFlatFaces(pocketed) == 0, "and neither is drawn");

        RenderMesh rm;
        pocketed.buildRenderMesh(rm);
        check(static_cast<int>(rm.edgeLines.size() / 2) == pocketed.halfedgeCount() / 2 - 2,
              "exactly the two cuts are left out of the wireframe");
        std::printf("[bool] bored face: %d bridge cuts, %d drawn across a flat face\n",
                    bridges, linesAcrossFlatFaces(pocketed));
    }

    {
        // Four slots: four holes in one face, so four bridges.
        Mesh cur;
        makeBox(cur, {20.0, 20.0, 20.0});
        for (int i = 0; i < 4; ++i) {
            Mesh cutter;
            makeBox(cutter, {5.0, 5.0, 30.0});
            const Vec3 at{(i % 2) ? 6.0 : -6.0, (i / 2) ? 6.0 : -6.0, 0.0};
            for (MeshVertex& v : cutter.verts) v.position += at;
            Mesh next;
            if (!meshBoolean(cur, cutter, BooleanOp::Difference, next, 100 + i)) break;
            cur = std::move(next);
        }
        check(linesAcrossFlatFaces(cur) == 0, "four slots draw nothing across the faces");
    }

    {
        // The other half of the rule: a section line an extrude left behind is
        // one shared edge, not two, and stays drawn.
        Mesh box;
        makeBox(box, {20.0, 20.0, 20.0});
        Index top = kInvalid;
        for (Index f = 0; f < box.faceCount(); ++f)
            if (dot(box.faceNormal(f), Vec3{0, 0, 1}) > 0.99) top = f;
        check(extrudeFaces(box, {top}, 6.0), "raise the top");

        int hidden = 0;
        for (Index h = 0; h < box.halfedgeCount(); ++h)
            if (h < box.halfedges[h].twin && box.isBridgeEdge(h)) ++hidden;
        check(hidden == 0, "an extrude's section lines are not bridges");

        RenderMesh rm;
        box.buildRenderMesh(rm);
        check(static_cast<int>(rm.edgeLines.size() / 2) == box.halfedgeCount() / 2,
              "and every edge of an extruded body is still drawn");

        Mesh plain;
        makeBox(plain, {20.0, 20.0, 20.0});
        RenderMesh pr;
        plain.buildRenderMesh(pr);
        check(pr.edgeLines.size() / 2 == 12, "a plain box still draws twelve edges");
        std::printf("[bool] extruded body: %d bridges, all %zu edges drawn\n",
                    hidden, rm.edgeLines.size() / 2);
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
