// Mesh editing operations. Every case checks the result is still a closed
// manifold with the right topology and the right enclosed volume -- winding
// and connectivity errors show up as a negative or wrong volume long before
// they are visible on screen.
#include "mesh/operations.h"
#include "mesh/primitives.h"

#include <cstdio>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near(double a, double b, double eps = 1e-6) { return std::fabs(a - b) < eps; }

static double volumeOf(const Mesh& m) {
    RenderMesh rm;
    m.buildRenderMesh(rm);
    double s6 = 0.0;
    for (size_t i = 0; i < rm.triangles.size(); i += 3)
        s6 += dot(rm.positions[rm.triangles[i]],
                  cross(rm.positions[rm.triangles[i + 1]], rm.positions[rm.triangles[i + 2]]));
    return s6 / 6.0;
}

static int boundaryEdges(const Mesh& m) {
    int n = 0;
    for (Index h = 0; h < m.halfedgeCount(); ++h)
        if (m.halfedges[h].face == kInvalid) ++n;
    return n;
}

// V - E + F for a closed surface: 2 for a sphere-like solid.
static int euler(const Mesh& m) {
    return m.vertexCount() - m.halfedgeCount() / 2 + m.faceCount();
}

static void expectSolid(const Mesh& m, const char* what, int genus = 0) {
    std::string err;
    check(m.validate(&err), std::string(what) + ": validate: " + err);
    check(boundaryEdges(m) == 0, std::string(what) + ": should be closed");
    check(euler(m) == 2 - 2 * genus, std::string(what) + ": wrong Euler characteristic");
    check(volumeOf(m) > 0.0, std::string(what) + ": inside out");
}

// Which face of a box points most strongly along `dir`.
static Index faceFacing(const Mesh& m, Vec3 dir) {
    Index best = kInvalid;
    float bestDot = -2.0f;
    for (Index f = 0; f < m.faceCount(); ++f) {
        const float d = dot(m.faceNormal(f), dir);
        if (d > bestDot) { bestDot = d; best = f; }
    }
    return best;
}

int main() {
    // ---- Extrude -----------------------------------------------------------
    {
        Mesh m;
        makeBox(m);                                   // 20mm cube, volume 8000
        const Index top = faceFacing(m, {0, 0, 1});
        std::vector<Index> moved;

        check(extrudeFaces(m, {top}, 10.0f, &moved), "extrude succeeds");
        expectSolid(m, "extruded box");
        check(std::fabs(volumeOf(m) - 12000.0) < 1e-2,
              "extruding 20x20 by 10mm adds 4000mm^3");
        check(moved.size() == 1, "reports the moved face");
        check(std::fabs(dot(m.faceNormal(moved[0]), Vec3{0, 0, 1}) - 1.0f) < 1e-4f,
              "moved face still points up");
        check(std::fabs(m.faceCentroid(moved[0]).z - 20.0f) < 1e-3f,
              "moved face sits 10mm higher");
        // Pushing a box's whole top out gives a taller box, and a taller box
        // has six faces. The new side walls are in the same planes as the walls
        // they grew out of, and are merged into them -- the seam between them is
        // in the data, not on the model.
        check(m.faceCount() == 6, "extruding a whole face leaves a taller box, "
                                  "not one with seams down its sides");
        std::printf("[extrude] volume %.1f, %d faces\n", volumeOf(m), m.faceCount());
    }

    // Extruding inward removes volume.
    {
        Mesh m;
        makeBox(m);
        const Index top = faceFacing(m, {0, 0, 1});
        check(extrudeFaces(m, {top}, -5.0f, nullptr), "negative extrude succeeds");
        expectSolid(m, "inset-extruded box");
        check(std::fabs(volumeOf(m) - 6000.0) < 1e-2, "cutting 5mm removes 2000mm^3");
        std::printf("[extrude] negative distance -> volume %.1f\n", volumeOf(m));
    }

    // Extruding two adjacent faces walls only the region boundary, not the
    // shared edge between them.
    {
        Mesh m;
        makeBox(m);
        const Index top = faceFacing(m, {0, 0, 1});
        const Index side = faceFacing(m, {1, 0, 0});
        check(extrudeFaces(m, {top, side}, 4.0f, nullptr), "region extrude succeeds");
        expectSolid(m, "region-extruded box");
        std::printf("[extrude] two-face region -> %d faces, volume %.1f\n",
                    m.faceCount(), volumeOf(m));
    }

    // Extruding an open surface leaves it open.
    {
        Mesh m;
        makePlane(m);
        check(extrudeFaces(m, {0}, 10.0f, nullptr), "plane extrude succeeds");
        std::string err;
        check(m.validate(&err), std::string("extruded plane: ") + err);
        check(m.faceCount() == 5, "one cap plus four walls");
        check(boundaryEdges(m) == 4, "far side stays open");
        std::printf("[extrude] plane -> %d faces, %d boundary edges\n",
                    m.faceCount(), boundaryEdges(m));
    }

    // ---- Inset -------------------------------------------------------------
    {
        Mesh m;
        makeBox(m);
        const Index top = faceFacing(m, {0, 0, 1});
        std::vector<Index> inner;

        check(insetFaces(m, {top}, 4.0f, &inner), "inset succeeds");
        expectSolid(m, "inset box");
        // Inset is purely in-plane, so the solid's volume must not change.
        check(std::fabs(volumeOf(m) - 8000.0) < 1e-2, "inset does not change volume");
        check(m.faceCount() == 6 - 1 + 1 + 4, "inner face plus four rim quads");
        check(inner.size() == 1, "reports the inner face");

        // A 20mm square inset by 4mm on every side leaves 12mm.
        const float area = m.faceArea(inner[0]);
        check(std::fabs(area - 144.0f) < 1e-2f, "inner face is 12x12mm");
        std::printf("[inset] inner area %.2f mm^2 (expect 144)\n", area);
    }

    // An inset that would turn the face inside out must be refused outright.
    {
        Mesh m;
        makeBox(m);
        const Mesh before = m;
        const Index top = faceFacing(m, {0, 0, 1});
        check(!insetFaces(m, {top}, 15.0f, nullptr), "over-inset is rejected");
        check(m.faceCount() == before.faceCount(), "mesh untouched after rejection");
        check(std::fabs(volumeOf(m) - 8000.0) < 1e-3, "geometry untouched after rejection");
        std::printf("[inset] over-inset rejected, mesh intact\n");
    }

    // ---- Move faces --------------------------------------------------------
    {
        Mesh m;
        makeBox(m);
        const Index top = faceFacing(m, {0, 0, 1});
        check(moveFaces(m, {top}, {0, 0, 6.0f}), "move face succeeds");
        expectSolid(m, "moved-face box");
        check(std::fabs(volumeOf(m) - (20.0 * 20.0 * 26.0)) < 1e-2,
              "moving the top face 6mm makes a 20x20x26 box");
        check(m.faceCount() == 6, "moving a face changes no topology");
        std::printf("[move] volume %.1f (expect 10400)\n", volumeOf(m));
    }

    // ---- Bevel -------------------------------------------------------------
    {
        Mesh m;
        makeBox(m);
        const float maxW = maxBevelWidth(m);
        check(maxW > 9.0f && maxW < 11.0f, "a 20mm cube takes about a 10mm bevel");

        check(bevelAllEdges(m, 3.0f, 1), "chamfer succeeds");
        expectSolid(m, "chamfered box");

        // Truncating a cube gives 6 octagons + 12 edge quads + 8 corner tris.
        check(m.faceCount() == 6 + 12 + 8, "chamfer produces 26 faces");
        check(m.vertexCount() == 24, "each original corner becomes three vertices");

        // Chamfering only removes material.
        const double v = volumeOf(m);
        check(v < 8000.0 && v > 7000.0, "chamfer trims a little volume");
        std::printf("[bevel] chamfer: %d faces, %d verts, volume %.1f, max width %.2f\n",
                    m.faceCount(), m.vertexCount(), v, maxW);
    }

    // Rounded bevel. With a real fillet the arc is tangent to both faces and
    // bulges outward past the chamfer chord, so more segments keep MORE
    // material, converging on the analytic rounded cube from below.
    {
        Mesh flat, round2, round4, round8;
        makeBox(flat); makeBox(round2); makeBox(round4); makeBox(round8);
        const Real w = 3.0;
        check(bevelAllEdges(flat, w, 1), "chamfer");
        check(bevelAllEdges(round2, w, 2), "2-segment fillet");
        check(bevelAllEdges(round4, w, 4), "4-segment fillet");
        check(bevelAllEdges(round8, w, 8), "8-segment fillet");
        expectSolid(round2, "2-segment fillet");
        expectSolid(round4, "4-segment fillet");
        expectSolid(round8, "8-segment fillet");

        const double vf = volumeOf(flat), v2 = volumeOf(round2);
        const double v4 = volumeOf(round4), v8 = volumeOf(round8);

        // A 20mm cube with every edge rounded to radius r decomposes exactly
        // into a core box, six slabs, twelve quarter-cylinders and one sphere.
        const double PI = 3.14159265358979;
        const double a = 20.0, r = w, core = a - 2 * r;
        const double exact = core * core * core
                           + 6 * core * core * r
                           + 12 * core * (PI * r * r / 4.0)
                           + (4.0 / 3.0) * PI * r * r * r;

        check(v2 > vf && v4 > v2 && v8 > v4, "more segments keep more material");
        check(v8 < exact, "and stay inscribed in the true rounded cube");
        check((exact - v8) / exact < 0.01, "8 segments is within 1% of exact");
        std::printf("[bevel] fillet volumes: 1seg %.1f  2seg %.1f  4seg %.1f  8seg %.1f"
                    "  (exact %.1f)\n", vf, v2, v4, v8, exact);

        // Uniformity. Every edge of a cube is equivalent and every segment
        // subtends the same angle, so all the strip quads are congruent and
        // their areas must match. The old implementation chamfered the chamfer
        // with a decaying width, which made each ring a different size -- the
        // non-uniformity this pins down.
        // Strips run the length of an edge, so they have one long side; the
        // corner patches are small in both directions. Selecting on that keeps
        // the two apart without depending on face ordering.
        Real lo = 1e30, hi = 0.0;
        int strips = 0;
        std::vector<Index> fv;
        for (Index f = 0; f < round4.faceCount(); ++f) {
            if (round4.faceDegree(f) != 4) continue;
            round4.faceVertices(f, fv);
            Real longest = 0.0;
            for (size_t i = 0; i < fv.size(); ++i)
                longest = std::max(longest,
                    length(round4.verts[fv[(i + 1) % fv.size()]].position -
                           round4.verts[fv[i]].position));
            if (longest < 5.0) continue;         // a corner-patch quad
            const Real area = round4.faceArea(f);
            if (area > 100.0) continue;          // the six inset faces are ~196
            lo = std::min(lo, area);
            hi = std::max(hi, area);
            ++strips;
        }
        check(strips == 12 * 4, "twelve edges times four segments");
        check(hi / lo < 1.001, "every segment is the same size");
        std::printf("[bevel] segment areas: %d strips, %.4f to %.4f (ratio %.5f)\n",
                    strips, lo, hi, hi / lo);

        // The shape itself, checked exactly. A box filleted at radius r is the
        // set of points exactly r from an inner core box inset by r -- that is
        // what "a ball of radius r rolled over it" means. So every vertex of
        // the result, on a flat, on a cylinder or on a corner sphere alike,
        // must sit exactly r from that core. This catches a corner patch that
        // is flat, dented or bulging, which a volume total can hide.
        {
            const Real h = a / 2 - r;
            Real worst = 0.0;
            for (Index v = 0; v < round8.vertexCount(); ++v) {
                const Vec3 p = round8.verts[v].position;
                const Vec3 nearest{clampf(p.x, -h, h), clampf(p.y, -h, h),
                                   clampf(p.z, -h, h)};
                worst = std::max(worst, std::fabs(length(p - nearest) - r));
            }
            check(worst < 1e-9, "every vertex lies exactly r from the core box");
            std::printf("[bevel] rolling-ball check: worst deviation %.3e mm\n", worst);
        }
    }

    // Over-wide bevel must fail without damaging the mesh.
    {
        Mesh m;
        makeBox(m);
        check(!bevelAllEdges(m, 50.0f, 1), "over-wide bevel is rejected");
        check(m.faceCount() == 6, "mesh untouched after rejection");
        check(std::fabs(volumeOf(m) - 8000.0) < 1e-3, "geometry untouched");
        std::printf("[bevel] over-wide rejected, mesh intact\n");
    }

    // Bevelling a shape with non-square faces and higher-valence vertices.
    {
        Mesh m;
        CylinderParams cp; cp.segments = 12;
        makeCylinder(m, cp);
        check(bevelAllEdges(m, 1.0f, 1), "chamfer a cylinder");
        expectSolid(m, "chamfered cylinder");
        std::printf("[bevel] cylinder -> %d faces\n", m.faceCount());
    }

    // ---- Beveling a subset of edges -----------------------------------------
    {
        Mesh one;
        makeBox(one);
        // Any single edge. Faces either side pull back; the rest stay sharp.
        check(bevelEdges(one, {0}, 3.0, 1), "bevel a single edge");
        expectSolid(one, "single-edge chamfer");

        const double v = volumeOf(one);
        // Chamfering one edge of a box removes exactly a triangular prism:
        // legs of `width` along a 20mm edge. Nothing is taken from the ends,
        // because the faces meeting there are square to the cut.
        check(near(v, 8000.0 - 0.5 * 3.0 * 3.0 * 20.0, 1e-6),
              "removes exactly one triangular prism");

        Mesh all;
        makeBox(all);
        check(bevelAllEdges(all, 3.0, 1), "bevel every edge");
        check(volumeOf(one) > volumeOf(all),
              "one edge removes far less than all twelve");
        std::printf("[bevel] single edge %.1f vs all edges %.1f (from 8000)\n",
                    v, volumeOf(all));
    }

    // A single edge rounded with segments is still a solid, and rounding keeps
    // more material than the flat chamfer here too.
    {
        Mesh flat, round;
        makeBox(flat); makeBox(round);
        check(bevelEdges(flat, {0}, 2.5, 1), "single-edge chamfer");
        check(bevelEdges(round, {0}, 2.5, 6), "single-edge fillet");
        expectSolid(round, "single-edge fillet");
        check(volumeOf(round) > volumeOf(flat), "the arc bulges outward");
        std::printf("[bevel] single edge: chamfer %.2f, 6-segment fillet %.2f\n",
                    volumeOf(flat), volumeOf(round));
    }

    // Two edges sharing a vertex: the corner between them has to be closed by
    // a patch, which is the case a naive per-edge bevel gets wrong.
    {
        Mesh m;
        makeBox(m);
        // Find two edges of the top face that meet at a corner.
        const Index top = faceFacing(m, {0, 0, 1});
        const Index h0 = m.faces[top].halfedge;
        const Index h1 = m.halfedges[h0].next;
        check(bevelEdges(m, {h0, h1}, 2.0, 3), "bevel two edges at a shared corner");
        expectSolid(m, "two adjacent edges filleted");
        std::printf("[bevel] adjacent edges: %d faces, volume %.1f\n",
                    m.faceCount(), volumeOf(m));
    }

    // Filleting part of a convex solid must only ever remove material. A
    // corner blend that bulges is still watertight and still manifold, so
    // nothing else here would catch it -- but it puts surface outside the
    // shape the user drew.
    {
        const Real w = 4.0;
        for (int count : {1, 2, 4}) {
            Mesh m;
            makeBox(m);
            const Index top = faceFacing(m, {0, 0, 1});
            std::vector<Index> edges;
            Index h = m.faces[top].halfedge;
            for (int i = 0; i < count; ++i) {
                edges.push_back(std::min(h, m.halfedges[h].twin));
                h = m.halfedges[h].next;
            }
            check(bevelEdges(m, edges, w, 8), "partial fillet succeeds");
            expectSolid(m, "partial fillet");

            int outside = 0;
            for (Index v = 0; v < m.vertexCount(); ++v) {
                const Vec3 p = m.verts[v].position;
                if (std::fabs(p.x) > 10.0 + 1e-9 || std::fabs(p.y) > 10.0 + 1e-9 ||
                    std::fabs(p.z) > 10.0 + 1e-9) ++outside;
            }
            check(outside == 0, "no vertex escapes the original box");
            check(volumeOf(m) < 8000.0, "material was removed, not added");
        }
        std::printf("[bevel] partial fillets stay inside the solid\n");
    }

    // The radius has to be right along the whole edge, not just near the
    // corners. A strip is ruled between its two end sections, so an end
    // section that is skewed -- its two sides at different positions along the
    // edge -- deforms the fillet everywhere, which a containment check misses.
    {
        const Real w = 4.0;
        Mesh m;
        makeBox(m);
        const Index top = faceFacing(m, {0, 0, 1});
        std::vector<Index> edges;
        Index h = m.faces[top].halfedge;
        for (int i = 0; i < 4; ++i) {
            edges.push_back(std::min(h, m.halfedges[h].twin));
            h = m.halfedges[h].next;
        }
        check(bevelEdges(m, edges, w, 8), "rim fillet");

        // Along the +X top edge the fillet is a quarter cylinder about
        // (10-w, y, 10-w). Away from the corners its vertices must sit on it.
        Real best = 0.0;
        int sampled = 0;
        for (Index v = 0; v < m.vertexCount(); ++v) {
            const Vec3 p = m.verts[v].position;
            if (p.x < 10.0 - w - 1e-6 || p.z < 10.0 - w - 1e-6) continue;
            // The strip's cross-sections sit exactly at the setback, so the
            // band has to include that position rather than stop short of it.
            if (std::fabs(p.y) > 10.0 - w + 1e-6) continue;      // beyond is corner
            const Real r = std::sqrt((p.x - (10 - w)) * (p.x - (10 - w)) +
                                     (p.z - (10 - w)) * (p.z - (10 - w)));
            best = std::max(best, r);
            ++sampled;
        }
        check(sampled > 0, "found fillet vertices along the edge");
        check(near(best, w, 1e-9), "the fillet reaches its full radius");
        std::printf("[bevel] rim fillet radius along the edge: %.6f (asked %.1f)\n",
                    best, w);
    }

    // An out-of-range edge is rejected rather than silently ignored.
    {
        Mesh m;
        makeBox(m);
        check(!bevelEdges(m, {9999}, 1.0, 1), "bad edge index is refused");
        check(m.faceCount() == 6, "mesh untouched");
        check(!bevelEdges(m, {}, 1.0, 1), "an empty selection is refused");
    }

    // Bevel refuses open surfaces: a boundary edge has no second face to
    // chamfer against.
    {
        Mesh m;
        makePlane(m);
        check(!bevelAllEdges(m, 1.0f, 1), "bevel rejects an open surface");
    }

    // ---- Operations compose ------------------------------------------------
    {
        Mesh m;
        makeBox(m);
        const Index top = faceFacing(m, {0, 0, 1});
        std::vector<Index> inner;
        check(insetFaces(m, {top}, 4.0f, &inner), "inset");
        check(extrudeFaces(m, inner, 12.0f, &inner), "then extrude the inset face");
        expectSolid(m, "inset then extruded");
        // 8000 for the cube plus a 12x12 boss 12mm tall.
        check(std::fabs(volumeOf(m) - (8000.0 + 144.0 * 12.0)) < 1e-1,
              "boss volume adds up");
        std::printf("[compose] inset+extrude volume %.1f (expect %.1f)\n",
                    volumeOf(m), 8000.0 + 144.0 * 12.0);
    }

    // ---- An extrude is a solid combined with the body ----------------------
    // Lifting a face and stitching walls onto it is only the same thing while
    // the sweep meets nothing.
    {
        // Pushed inward, it must be the face moving back, not a shell left
        // inside the solid.
        Mesh m;
        makeBox(m);
        const Index top = faceFacing(m, {0, 0, 1});
        check(extrudeFaces(m, {top}, -6.0, nullptr, 7), "extrude inward");
        expectSolid(m, "inward extrude");
        check(std::fabs(volumeOf(m) - 5600.0) < 1e-6,
              "a 20x20 face pushed back 6mm leaves 5600 (" +
                  std::to_string(volumeOf(m)) + ")");
        check(m.faceCount() == 6, "and the body is still a box, not a box plus a shell");
        check(std::fabs(m.bounds().max.z - 4.0) < 1e-9, "the face is where it was put");
        std::printf("[extrude] inward: %d faces, %.1f mm3\n", m.faceCount(), volumeOf(m));
    }
    {
        // Driven clean through the far side there is nothing left to be, and
        // the old path answered with a solid turned inside out.
        Mesh m;
        BoxParams slab{40, 20, 10};
        makeBox(m, slab);
        const Index top = faceFacing(m, {0, 0, 1});
        const int before = m.faceCount();
        const double volBefore = volumeOf(m);
        check(!extrudeFaces(m, {top}, -25.0, nullptr, 9),
              "pushing a face through the far side is refused");
        check(m.faceCount() == before && std::fabs(volumeOf(m) - volBefore) < 1e-9,
              "and the body is left exactly as it was");
        std::printf("[extrude] through the far side: refused, body untouched\n");
    }
    {
        // Zero length: nothing built, but the face is reported so a drag has
        // something to work from. Repeating it changes nothing.
        Mesh m;
        makeBox(m);
        const Index top = faceFacing(m, {0, 0, 1});
        std::vector<Index> reported;
        const bool ok = extrudeFaces(m, {top}, 0.0, &reported, 11);
        check(ok, "a zero-length extrude succeeds");
        check(m.faceCount() == 6, "and builds nothing");
        check(reported.size() == 1 && reported[0] == top, "but reports the face to drag");
        const bool again = extrudeFaces(m, {top}, 0.0, &reported, 12);
        check(again && m.faceCount() == 6, "repeating it still builds nothing");
        expectSolid(m, "after zero-length extrudes");
        std::printf("[extrude] zero length: %d faces, reports the picked face\n",
                    m.faceCount());
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
