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
        // A cube with one face pushed out is still six-sided prism-wise: 4 new
        // side walls replace nothing, so faces go 6 -> 10.
        check(m.faceCount() == 10, "extrude adds four side walls");
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

    // Rounded bevel. Each extra segment chamfers the previous chamfer, so the
    // edge gets rounder and the surface moves further inward -- unlike a true
    // fillet, whose arc bulges outward past the chamfer chord and therefore
    // keeps *more* material. Asserting the real behaviour rather than the
    // behaviour a proper fillet would have.
    {
        Mesh flat, round2, round4;
        makeBox(flat); makeBox(round2); makeBox(round4);
        check(bevelAllEdges(flat, 3.0f, 1), "chamfer");
        check(bevelAllEdges(round2, 3.0f, 2), "2-segment rounding");
        check(bevelAllEdges(round4, 3.0f, 4), "4-segment rounding");
        expectSolid(round2, "2-segment rounding");
        expectSolid(round4, "4-segment rounding");

        const double vf = volumeOf(flat), v2 = volumeOf(round2), v4 = volumeOf(round4);
        check(v2 < vf, "each pass cuts further in than the flat chamfer");
        check(v4 < v2, "and further again with more passes");
        // Converging, not running away: the extra passes take ever less.
        check((v2 - v4) < (vf - v2), "successive passes remove diminishing amounts");
        check(v4 > 6000.0, "converges rather than eating the solid");

        check(round2.faceCount() > flat.faceCount(), "rounding adds faces");
        check(round4.faceCount() > round2.faceCount(), "and more with more passes");
        std::printf("[bevel] rounding: chamfer %.1f (%d f), 2-seg %.1f (%d f), "
                    "4-seg %.1f (%d f)\n",
                    vf, flat.faceCount(), v2, round2.faceCount(),
                    v4, round4.faceCount());
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

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
