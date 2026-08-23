// Stable names for mesh elements.
//
// The property under test is the one the feature history depends on: an
// element a user picked is still findable after the steps before it change.
// Everything else here supports that -- names have to exist on every element,
// be unique, survive an operation that does not destroy the element, and come
// out the same every time the same chain is run.
//
// The failure this guards against is not a crash. Raise a cylinder's segment
// count under a rim fillet and index-based references still resolve, to
// different edges, and the model comes back valid and wrong.
#include "mesh/health.h"
#include "mesh/operations.h"
#include "mesh/primitives.h"

#include <cstdio>
#include <string>
#include <unordered_set>
#include <vector>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}

static Index faceFacing(const Mesh& m, Vec3 dir) {
    Index best = kInvalid;
    Real bestDot = -2.0;
    for (Index f = 0; f < m.faceCount(); ++f) {
        const Real d = dot(m.faceNormal(f), dir);
        if (d > bestDot) { bestDot = d; best = f; }
    }
    return best;
}

// Every element named, no name used twice.
static void expectNamed(const Mesh& m, const std::string& what) {
    int unnamed = 0, dup = 0;
    std::unordered_set<ElementId> seen;
    for (const MeshVertex& v : m.verts) {
        if (v.id == kNoId) ++unnamed;
        else if (!seen.insert(v.id).second) ++dup;
    }
    seen.clear();
    for (const MeshFace& f : m.faces) {
        if (f.id == kNoId) ++unnamed;
        else if (!seen.insert(f.id).second) ++dup;
    }
    seen.clear();
    for (Index h = 0; h < m.halfedgeCount(); ++h) {
        if (h > m.halfedges[h].twin) continue;
        const ElementId e = m.edgeId(h);
        if (e == kNoId) ++unnamed;
        else if (!seen.insert(e).second) ++dup;
    }
    check(unnamed == 0, what + ": " + std::to_string(unnamed) + " unnamed elements");
    check(dup == 0, what + ": " + std::to_string(dup) + " duplicate names");
    check(m.named(), what + ": Mesh::named disagrees");
}

// The names of every element, in index order.
static std::vector<ElementId> namesOf(const Mesh& m) {
    std::vector<ElementId> out;
    for (const MeshVertex& v : m.verts) out.push_back(v.id);
    for (const MeshFace& f : m.faces) out.push_back(f.id);
    for (Index h = 0; h < m.halfedgeCount(); ++h)
        if (h <= m.halfedges[h].twin) out.push_back(m.edgeId(h));
    return out;
}

int main() {
    // ---- Every primitive names everything it makes -------------------------
    {
        Mesh m;
        makeBox(m);             expectNamed(m, "box");
        makeCylinder(m);        expectNamed(m, "cylinder");
        makeSphere(m);          expectNamed(m, "sphere");
        makeCone(m);            expectNamed(m, "cone");
        makeTorus(m);           expectNamed(m, "torus");
        makePlane(m);           expectNamed(m, "plane");
        std::printf("[names] every primitive is fully named\n");
    }

    // ---- A dimension change renames nothing --------------------------------
    // This is the everyday edit: drag a box's width. The topology is untouched,
    // so every name must be untouched, or a downstream feature would lose what
    // it was acting on for no reason at all.
    {
        Mesh a, b;
        makeBox(a);
        BoxParams big{30, 40, 50};
        makeBox(b, big);
        check(namesOf(a) == namesOf(b), "a box keeps its names when resized");

        Mesh c, d;
        CylinderParams p1, p2;
        p2.radius = 4; p2.height = 55;
        makeCylinder(c, p1);
        makeCylinder(d, p2);
        check(namesOf(c) == namesOf(d), "a cylinder keeps its names when resized");
        std::printf("[names] resizing renames nothing\n");
    }

    // ---- Re-running a chain reproduces the names ---------------------------
    // Names are derived, never allocated from a counter, so evaluating the same
    // chain twice has to give the same answer. Without this a stored reference
    // would go stale every time the model was rebuilt.
    {
        auto buildChain = [] {
            Mesh m;
            makeBox(m);
            const Index top = faceFacing(m, {0, 0, 1});
            extrudeFaces(m, {top}, 8.0, nullptr, 111);
            const Index lifted = faceFacing(m, {0, 0, 1});
            insetFaces(m, {lifted}, 3.0, nullptr, 222);
            std::vector<Index> edges;
            for (Index h = 0; h < m.halfedgeCount(); ++h)
                if (h < m.halfedges[h].twin) edges.push_back(h);
            FilletSpec spec;
            spec.segments = 4;
            spec.salt = 333;
            for (Index e : edges) spec.edges.push_back({e, 0.8});
            filletEdges(m, spec);
            return m;
        };
        const Mesh first = buildChain();
        const Mesh again = buildChain();
        expectNamed(first, "extrude + inset + fillet");
        check(namesOf(first) == namesOf(again), "re-running a chain reproduces every name");
        check(checkHealth(first).solid(), "the chain still produces a solid");
        std::printf("[names] chain of %d faces reproduces exactly\n", first.faceCount());
    }

    // ---- An operation leaves alone what it did not touch --------------------
    {
        Mesh m;
        makeBox(m);
        const Index top = faceFacing(m, {0, 0, 1});
        const Index bottom = faceFacing(m, {0, 0, -1});
        const ElementId topName = m.faces[top].id;
        const ElementId bottomName = m.faces[bottom].id;

        check(extrudeFaces(m, {top}, 8.0, nullptr, 111), "extrude");
        expectNamed(m, "after extrude");

        // The lifted face is the same face and keeps its name; so does every
        // face the operation never looked at.
        check(m.findFace(topName) != kInvalid, "the extruded face keeps its name");
        check(m.findFace(bottomName) != kInvalid, "an untouched face keeps its name");
        const Index lifted = m.findFace(topName);
        check(std::fabs(m.faceCentroid(lifted).z - 18.0) < 1e-9,
              "and that name now finds it in its new position");
        std::printf("[names] extrude keeps the names of what it moved and what it did not\n");
    }

    // ---- An edge is named by its endpoints ---------------------------------
    // Which is why an edge a fillet does not touch keeps its name through the
    // fillet: both its endpoints survive, so the derived name is the same.
    {
        Mesh m;
        makeBox(m);
        const Vec3 tFL{-10, -10, 10}, tFR{10, -10, 10};
        Index target = kInvalid, far = kInvalid;
        for (Index h = 0; h < m.halfedgeCount(); ++h) {
            const Vec3 a = m.verts[m.fromVertex(h)].position;
            const Vec3 b = m.verts[m.halfedges[h].vertex].position;
            if (lengthSq(a - tFL) < 1e-9 && lengthSq(b - tFR) < 1e-9) target = h;
            // The bottom edge diagonally opposite, which the fillet cannot reach.
            if (a.z < -9 && b.z < -9 && a.y > 9 && b.y > 9) far = h;
        }
        check(target != kInvalid && far != kInvalid, "found both edges");
        const ElementId farName = m.edgeId(far);

        FilletSpec spec;
        spec.segments = 5;
        spec.salt = 42;
        spec.edges.push_back({target, 3.0});
        check(filletEdges(m, spec), "fillet one edge");
        expectNamed(m, "after fillet");
        check(m.findEdge(farName) != kInvalid,
              "an edge the fillet never reached keeps its name");
        std::printf("[names] a fillet leaves distant edges named as they were\n");
    }

    // ---- The failure this exists to prevent --------------------------------
    // A rim fillet stored as edge indices, then the cylinder's segment count
    // raised. The indices still resolve; they resolve to the wrong edges. Names
    // do not, because the edges they named no longer exist.
    {
        Mesh coarse, fine;
        CylinderParams c16, c24;
        c16.segments = 16;
        c24.segments = 24;
        makeCylinder(coarse, c16);
        makeCylinder(fine, c24);

        const AABB b = coarse.bounds();
        std::vector<ElementId> rim;
        for (Index h = 0; h < coarse.halfedgeCount(); ++h) {
            if (h > coarse.halfedges[h].twin) continue;
            const Vec3 p = coarse.verts[coarse.fromVertex(h)].position;
            const Vec3 q = coarse.verts[coarse.halfedges[h].vertex].position;
            if (std::fabs(p.z - b.max.z) < 1e-9 && std::fabs(q.z - b.max.z) < 1e-9)
                rim.push_back(coarse.edgeId(h));
        }
        check(rim.size() == 16, "sixteen rim edges to start with");

        int resolved = 0;
        for (ElementId id : rim) if (fine.findEdge(id) != kInvalid) ++resolved;
        check(resolved == 0,
              "none of the coarse rim edges are claimed to exist on the fine cylinder");
        std::printf("[names] %zu rim edges, %d of them wrongly resolve after a "
                    "segment change (indices would resolve all 16)\n",
                    rim.size(), resolved);

        // The other half of the bargain. Refusing to resolve is only useful if
        // something else can still express what the user meant, and what they
        // meant was "the rim", not sixteen particular edges. The cap face is
        // named for what it is, so it survives the change and its boundary is
        // the rim at whatever segment count the cylinder now has.
        Index cap = kInvalid;
        for (Index f = 0; f < coarse.faceCount(); ++f)
            if (dot(coarse.faceNormal(f), Vec3{0, 0, 1}) > 0.99) cap = f;
        check(cap != kInvalid, "found the top cap");
        const ElementId capName = coarse.faces[cap].id;

        const Index capOnFine = fine.findFace(capName);
        check(capOnFine != kInvalid, "the top cap keeps its name across a segment change");
        if (capOnFine != kInvalid) {
            check(fine.faceDegree(capOnFine) == 24,
                  "and it now has twenty-four edges, which is the rim the user meant");
            std::printf("[names] the cap survives: %d edges before, %d after\n",
                        coarse.faceDegree(cap), fine.faceDegree(capOnFine));
        }
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
