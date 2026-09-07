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

static Body box(Real w = 20, Real d = 20, Real h = 20) {
    PrimitiveSpec spec;
    spec.kind = PrimitiveKind::Box;
    spec.box.width = w; spec.box.depth = d; spec.box.height = h;
    Body b;
    makePrimitive(spec, b);
    return b;
}

int main() {
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

    std::printf("--- Body: operations go through the seam ---\n");
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
