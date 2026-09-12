// The seam itself: Body's handles and topology, independent of what is behind
// it.
//
// These are the tests a second backend has to pass unchanged. Nothing here
// looks at a mesh, and nothing here assumes a handle is an array index or that
// the numbering is dense -- because the whole point of the seam is that a B-rep
// backend gets to choose its own numbering without any of this changing.
#include "geom/body.h"
#include "geom/operations.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <string>
#include <vector>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near(Real a, Real b, Real eps = 1e-9) { return std::fabs(a - b) < eps; }

// Which backend the contract below is being run against. The contract itself
// never mentions it: that is the point of the seam, and a section that had to
// know would be a section that does not belong here.
static Backend gBackend = Backend::Mesh;

static Body box(Real w = 20, Real d = 20, Real h = 20) {
    PrimitiveSpec spec;
    spec.kind = PrimitiveKind::Box;
    spec.box.width = w; spec.box.depth = d; spec.box.height = h;
    Body b;
    makePrimitive(spec, b, gBackend);
    return b;
}

// Everything a Body promises, for any backend. Run once per backend that the
// build has, and identical either way -- if a line here ever needs an "unless
// it is a mesh", the seam has sprung a leak.
static void seamContract() {
    std::printf("--- Body: enumeration and handles ---\n");
    {
        const Body b = box();
        check(!b.empty(), "a box is not empty");
        check(b.faceCount() == 6, "six faces");
        check(b.vertexCount() == 8, "eight vertices");
        check(b.edgeCount() == 12, "twelve edges");

        std::vector<FaceId> faces;
        std::vector<EdgeId> edges;
        std::vector<VertexId> verts;
        b.allFaces(faces);
        b.allEdges(edges);
        b.allVertices(verts);
        check(faces.size() == 6, "allFaces agrees with faceCount");
        check(edges.size() == 12, "allEdges agrees with edgeCount");
        check(verts.size() == 8, "allVertices agrees with vertexCount");

        // Handles are distinct. Nothing here requires them to be 0..n-1.
        check(std::set<FaceId>(faces.begin(), faces.end()).size() == 6, "face handles are distinct");
        check(std::set<EdgeId>(edges.begin(), edges.end()).size() == 12, "edge handles are distinct");
        check(std::set<VertexId>(verts.begin(), verts.end()).size() == 8, "vertex handles are distinct");
    }

    std::printf("--- Body: topology ---\n");
    {
        const Body b = box();
        std::vector<FaceId> faces;
        b.allFaces(faces);

        std::vector<EdgeId> fe;
        std::vector<VertexId> fv;
        std::set<EdgeId> viaFaces;
        for (FaceId f : faces) {
            b.faceEdges(f, fe);
            b.faceVertices(f, fv);
            check(fe.size() == 4, "a box face has four edges");
            check(fv.size() == 4, "a box face has four vertices");
            check(b.faceDegree(f) == 4, "faceDegree agrees");
            viaFaces.insert(fe.begin(), fe.end());
        }
        check(viaFaces.size() == 12, "the faces between them name every edge once");

        // faceEdges must hand back canonical handles: walking from either face
        // of an edge has to name the same edge, or a selection made on one side
        // would not match the same selection made on the other.
        std::vector<EdgeId> edges;
        b.allEdges(edges);
        for (EdgeId e : edges) {
            FaceId f0 = kNoFace, f1 = kNoFace;
            b.edgeFaces(e, f0, f1);
            check(f0 != kNoFace && f1 != kNoFace, "a closed box has no boundary edge");

            bool inF0 = false, inF1 = false;
            b.faceEdges(f0, fe); inF0 = std::find(fe.begin(), fe.end(), e) != fe.end();
            b.faceEdges(f1, fe); inF1 = std::find(fe.begin(), fe.end(), e) != fe.end();
            check(inF0 && inF1, "both of an edge's faces list that same edge handle");
        }
    }

    std::printf("--- Body: geometry through handles ---\n");
    {
        const Body b = box(20, 20, 20);
        std::vector<FaceId> faces;
        b.allFaces(faces);

        Real area = 0.0;
        for (FaceId f : faces) area += b.faceArea(f);
        check(near(area, 6.0 * 400.0, 1e-6), "the six faces total 2400 mm2");

        const AABB bb = b.bounds();
        check(near(bb.size().x, 20.0, 1e-9) && near(bb.size().z, 20.0, 1e-9), "bounds are 20mm");

        std::vector<EdgeId> edges;
        b.allEdges(edges);
        Real total = 0.0;
        for (EdgeId e : edges) {
            Vec3 p, q;
            b.edgePositions(e, p, q);
            total += length(q - p);
            check(near(length(b.edgeDirection(e)), 1.0, 1e-9), "edgeDirection is normalised");
        }
        check(near(total, 12.0 * 20.0, 1e-6), "the twelve edges total 240mm");
    }

    std::printf("--- Body: vertex fan ---\n");
    {
        const Body b = box();
        std::vector<VertexId> verts;
        b.allVertices(verts);
        std::vector<EdgeId> ve;
        for (VertexId v : verts) {
            b.vertexEdges(v, ve);
            check(ve.size() == 3, "three edges meet at a box corner");
            // and each one really does touch this vertex
            for (EdgeId e : ve) {
                VertexId a = kInvalid, c = kInvalid;
                b.edgeEnds(e, a, c);
                check(a == v || c == v, "an edge from the fan touches the vertex");
            }
        }
    }

    std::printf("--- Body: names round-trip ---\n");
    {
        const Body b = box();
        std::vector<FaceId> faces;
        std::vector<EdgeId> edges;
        std::vector<VertexId> verts;
        b.allFaces(faces); b.allEdges(edges); b.allVertices(verts);

        for (FaceId f : faces)
            check(b.findFace(b.faceName(f)) == f, "a face's name finds it again");
        for (EdgeId e : edges)
            check(b.findEdge(b.edgeName(e)) == e, "an edge's name finds it again, canonically");
        for (VertexId v : verts)
            check(b.findVertex(b.vertexName(v)) == v, "a vertex's name finds it again");

        std::set<ElementId> names;
        for (FaceId f : faces) names.insert(b.faceName(f));
        check(names.size() == 6, "the six faces have six distinct names");
    }

    std::printf("--- Body: display and validity ---\n");
    {
        const Body b = box();
        RenderMesh rm;
        b.tessellate(rm);
        check(rm.triangles.size() == 12 * 3, "a box tessellates to twelve triangles");
        check(rm.edgeLines.size() == 12 * 2, "and draws twelve edges");
        check(rm.triangleFace.size() == rm.triangles.size() / 3,
              "every triangle knows the face it came from");

        std::string err;
        check(b.validate(&err), "a box validates: " + err);
        const MeshHealth h = b.health();
        check(h.solid(), "and is a solid");
    }

}

int main() {
    std::printf("== mesh backend ==\n");
    gBackend = Backend::Mesh;
    seamContract();

    if (brep::available()) {
        std::printf("\n== B-rep backend ==\n");
        gBackend = Backend::Brep;
        seamContract();
        gBackend = Backend::Mesh;
    } else {
        std::printf("\n== B-rep backend: not built (TANGENT_BREP=OFF) ==\n");
    }

    // The claim the primitive naming makes: the same part of the same primitive
    // gets the same name whichever backend built it. That is what would let a
    // body change backends without every stored reference going stale.
    if (brep::available()) {
        std::printf("\n--- Body: names agree across backends ---\n");
        PrimitiveSpec spec;
        spec.kind = PrimitiveKind::Box;
        spec.box.width = 20; spec.box.depth = 20; spec.box.height = 20;
        Body meshBox, brepBox;
        check(makePrimitive(spec, meshBox, Backend::Mesh), "mesh box built");
        check(makePrimitive(spec, brepBox, Backend::Brep), "B-rep box built");

        std::vector<FaceId> mf, bf;
        meshBox.allFaces(mf);
        brepBox.allFaces(bf);
        std::set<ElementId> mNames, bNames;
        for (FaceId f : mf) mNames.insert(meshBox.faceName(f));
        for (FaceId f : bf) bNames.insert(brepBox.faceName(f));
        check(mNames == bNames, "a box has the same six face names on either backend");

        // And they name the same faces: the one called X faces the same way.
        for (FaceId f : mf) {
            const FaceId g = brepBox.findFace(meshBox.faceName(f));
            check(g != kInvalid, "each mesh face name resolves on the B-rep box");
            if (g == kInvalid) continue;
            check(length(brepBox.faceNormal(g) - meshBox.faceNormal(f)) < 1e-9,
                  "and points the same way");
        }

        // A cylinder can only agree about the parts both backends have. Its
        // caps are one face either way; its wall is one face here and a ring of
        // facets there, so no name could be shared and none is claimed.
        PrimitiveSpec cyl;
        cyl.kind = PrimitiveKind::Cylinder;
        cyl.cylinder.radius = 8; cyl.cylinder.height = 20; cyl.cylinder.segments = 32;
        Body meshCyl, brepCyl;
        check(makePrimitive(cyl, meshCyl, Backend::Mesh), "mesh cylinder built");
        check(makePrimitive(cyl, brepCyl, Backend::Brep), "B-rep cylinder built");
        check(brepCyl.faceCount() == 3, "a B-rep cylinder is a wall and two caps");

        int capsFound = 0;
        std::vector<FaceId> cf;
        meshCyl.allFaces(cf);
        for (FaceId f : cf) {
            if (std::fabs(meshCyl.faceNormal(f).z) < 0.99) continue;   // the facets
            if (brepCyl.findFace(meshCyl.faceName(f)) != kInvalid) ++capsFound;
        }
        check(capsFound == 2, "both cylinder caps keep their names across backends");

        RenderMesh rm;
        brepCyl.tessellate(rm);
        check(rm.triangles.size() > 60, "and it tessellates to a smooth wall");
        std::printf("  B-rep cylinder: %d faces, %zu triangles\n",
                    brepCyl.faceCount(), rm.triangles.size() / 3);
    }

    if (brep::available()) {
        std::printf("\n--- Body: what a thing actually is ---\n");
        PrimitiveSpec spec;
        spec.kind = PrimitiveKind::Cylinder;
        spec.cylinder.radius = 8;
        spec.cylinder.height = 20;
        spec.cylinder.segments = 32;
        Body cyl, meshCyl;
        check(makePrimitive(spec, cyl, Backend::Brep), "exact cylinder");
        check(makePrimitive(spec, meshCyl, Backend::Mesh), "mesh cylinder");

        std::vector<FaceId> faces;
        cyl.allFaces(faces);
        int walls = 0, caps = 0;
        for (FaceId f : faces) {
            if (cyl.faceKind(f) == SurfaceKind::Cylinder) ++walls;
            if (cyl.faceKind(f) == SurfaceKind::Plane) ++caps;
        }
        check(walls == 1 && caps == 2, "a cylinder is a wall and two flat caps");

        Vec3 point, axis;
        Real radius = 0;
        FaceId wall = kNoFace;
        for (FaceId f : faces) if (cyl.faceKind(f) == SurfaceKind::Cylinder) wall = f;
        check(cyl.faceCylinder(wall, point, axis, radius), "the wall knows it is a cylinder");
        check(near(radius, 8.0, 1e-9), "and its radius is 8, not the width of a facet");
        check(near(std::fabs(axis.z), 1.0, 1e-9), "with the axis it was built on");

        // The rim: a circle, with a centre to snap to and a length along it.
        std::vector<EdgeId> edges;
        cyl.allEdges(edges);
        EdgeId rim = kInvalid;
        for (EdgeId e : edges) if (cyl.edgeKind(e) == CurveKind::Circle) rim = e;
        check(rim != kInvalid, "the rim is a circle");
        Vec3 centre;
        check(cyl.edgeCircle(rim, centre, axis, radius), "and reports where it is");
        check(near(radius, 8.0, 1e-9), "at radius 8");
        check(near(std::hypot(centre.x, centre.y), 0.0, 1e-9), "centred on the axis");
        check(near(cyl.edgeLength(rim), 2.0 * kPi * 8.0, 1e-6),
              "50.27mm around, not the zero its chord would give");
        check(near(length(cyl.edgeMidpoint(rim) - centre), 8.0, 1e-6),
              "and its midpoint is out on the arc, not in the middle of the body");

        // What a rim is made of when something has to *draw* it. Asking for the
        // two ends gave a chord across the hole -- and for a closed circle the
        // two ends are the same point, so it drew nothing at all.
        std::vector<Vec3> poly;
        cyl.edgePolyline(rim, 0.01, poly);
        check(poly.size() > 2, "the rim draws as a chain of points, not one chord");
        Real offCircle = 0.0;
        for (const Vec3& p : poly)
            offCircle = std::max(offCircle, std::fabs(std::hypot(p.x, p.y) - 8.0));
        check(offCircle < 1e-6, "every one of them on the circle");

        // The mesh answers honestly: a polygon and a straight line, which is
        // all it has.
        std::vector<EdgeId> me;
        meshCyl.allEdges(me);
        check(meshCyl.edgeKind(me.front()) == CurveKind::Line, "every mesh edge is a line");
        check(!meshCyl.edgeCircle(me.front(), centre, axis, radius),
              "and none of them is a circle, because none of them is");
        meshCyl.edgePolyline(me.front(), 0.01, poly);
        check(poly.size() == 2, "a mesh edge draws as its two ends, which is all a line is");
        std::printf("  exact: %d faces, rim %.4f mm around; mesh: %d faces\n",
                    cyl.faceCount(), cyl.edgeLength(rim), meshCyl.faceCount());
    }

    // A hole is where the two representations differ most, and where drawing
    // one as if it were the other shows up as lines across the opening.
    std::printf("\n--- Body: a bored face, and what may be drawn of it ---\n");
    {
        PrimitiveSpec plate;
        plate.kind = PrimitiveKind::Box;
        plate.box.width = plate.box.depth = 60; plate.box.height = 10;
        PrimitiveSpec drill;
        drill.kind = PrimitiveKind::Cylinder;
        drill.cylinder.radius = 15;
        drill.cylinder.height = 40;
        drill.cylinder.segments = 24;

        for (Backend backend : {Backend::Mesh, Backend::Brep}) {
            if (backend == Backend::Brep && !brep::available()) continue;
            Body body, tool, bored;
            check(makePrimitive(plate, body, backend), "plate built");
            check(makePrimitive(drill, tool, backend), "drill built");
            tool.transform(translate({0, 0, -10}));
            std::string why;
            check(booleanOp(body, tool, BooleanOp::Difference, bored, 7, false, &why),
                  std::string("bored it: ") + why);

            std::vector<FaceId> faces;
            bored.allFaces(faces);
            FaceId top = kNoFace;
            for (FaceId f : faces)
                if (dot(bored.faceNormal(f), Vec3{0, 0, 1}) > 0.99 &&
                    (top == kNoFace || bored.faceArea(f) > bored.faceArea(top))) top = f;
            check(top != kNoFace, "found a face on top of it");

            std::vector<EdgeId> fe;
            bored.faceEdges(top, fe);
            int bridges = 0;
            for (EdgeId e : fe) if (bored.isBridgeEdge(e)) ++bridges;

            if (backend == Backend::Brep) {
                // One face holds the hole, so nothing had to be invented to
                // reach it -- which is why the outline can be drawn whole.
                check(bridges == 0, "an exact bored face has no bridge edges");
            } else {
                // The mesh needs them, and they are the lines that used to be
                // drawn across the opening. What matters is that they can be
                // told apart from the part's own edges.
                check(bridges > 0, "a mesh bored face needs bridge edges");
            }
            std::printf("  %s: top face has %zu edges, %d of them bridges\n",
                        backend == Backend::Brep ? "exact" : "mesh ", fe.size(), bridges);
        }
    }

    std::printf("\n--- Body: a handle that is no longer ours ---\n");
    {
        // Something always ends up holding a handle from before an edit: a
        // selection made a moment ago, a tool that cached one, a panel drawing
        // last frame's highlight. Both backends have to answer the same way,
        // and the answer has to be an answer rather than a crash.
        for (int pass = 0; pass < 2; ++pass) {
            const Backend backend = pass == 0 ? Backend::Mesh : Backend::Brep;
            if (backend == Backend::Brep && !brep::available()) continue;
            gBackend = backend;
            const Body b = box();
            const char* which = pass == 0 ? "mesh" : "exact";

            // Only handles that are stale for every kind. faceCount() + 7 is
            // not one of them: a mesh numbers edges by half-edge, so a cube has
            // twenty-four of them and 13 is a perfectly good edge.
            for (Index stale : {-1, 9999}) {
                check(!b.hasFace(stale), std::string(which) + ": a stale face is not ours");
                check(length(b.faceNormal(stale)) < 1e-9, std::string(which) + ": its normal is nothing");
                check(near(b.faceArea(stale), 0.0), std::string(which) + ": its area is nothing");
                check(b.faceDegree(stale) == 0, std::string(which) + ": it has no corners");
                std::vector<EdgeId> fe;
                b.faceEdges(stale, fe);
                check(fe.empty(), std::string(which) + ": and no edges");

                check(!b.hasVertex(stale), std::string(which) + ": a stale vertex is not ours");
                check(length(b.vertexPosition(stale)) < 1e-9,
                      std::string(which) + ": it is nowhere");

                check(!b.hasEdge(stale), std::string(which) + ": a stale edge is not ours");
                VertexId ea = 0, eb = 0;
                b.edgeEnds(stale, ea, eb);
                check(ea == kInvalid && eb == kInvalid, std::string(which) + ": it has no ends");
                FaceId fa = 0, fb = 0;
                b.edgeFaces(stale, fa, fb);
                check(fa == kNoFace && fb == kNoFace, std::string(which) + ": and no faces");
            }

            // Moving one does nothing rather than writing past the end.
            Body edit = box();
            const Real volumeBefore = edit.health(false).volume;
            edit.moveVertex(4242, {5, 5, 5});
            edit.setVertexPosition(-3, {1, 1, 1});
            check(near(edit.health(false).volume, volumeBefore, 1e-9),
                  std::string(which) + ": moving a vertex that is not there changes nothing");
        }
        gBackend = Backend::Mesh;
        std::printf("  both backends answer a stale handle rather than taking the process with them\n");
    }

    // Operations are still mesh-only; Stage 2 moves them across one at a time,
    // and this section joins the contract above as it does.
    std::printf("\n--- Body: operations go through the seam ---\n");
    {
        Body b = box();
        // Find the top face without reaching past the seam for it.
        std::vector<FaceId> faces;
        b.allFaces(faces);
        FaceId top = kNoFace;
        for (FaceId f : faces)
            if (dot(b.faceNormal(f), Vec3{0, 0, 1}) > 0.99) top = f;
        check(top != kNoFace, "found the top face");

        std::vector<FaceId> moved;
        check(extrudeFaces(b, {top}, 10.0, &moved), "extrude through the seam");
        check(moved.size() == 1, "it reports the moved face");
        check(near(b.health(false).volume, 12000.0, 1e-6), "and the volume is right");

        Body cutter = box(6, 6, 40);
        Body result;
        check(booleanOp(b, cutter, BooleanOp::Difference, result, 1), "boolean through the seam");
        check(!result.empty() && result.health().solid(), "the result is a solid");

        std::vector<EdgeId> edges;
        result.allEdges(edges);
        check(!edges.empty(), "the result has edges to round");

        // A fillet, and the reason when one is refused.
        Body tooBig = box();
        std::vector<EdgeId> be;
        tooBig.allEdges(be);
        FilletSpec spec;
        spec.segments = 4;
        spec.edges.push_back({be.front(), 50.0});
        std::string why;
        check(!filletEdges(tooBig, spec, &why), "an impossible radius is refused");
        check(!why.empty(), "and it says why: " + why);
    }

    std::printf("--- Body: split ---\n");
    {
        Body b = box();
        std::vector<Body> pieces;
        check(splitBodies(b, pieces) == 1, "one body splits into itself");
        check(pieces.size() == 1 && !pieces.front().empty(), "and the piece is usable");

        Body lo, hi;
        check(splitByPlane(b, {0, 0, 0}, {0, 0, 1}, lo, hi), "split a box in half");
        check(near(lo.health(false).volume + hi.health(false).volume, 8000.0, 1e-6),
              "the halves add back up");
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
