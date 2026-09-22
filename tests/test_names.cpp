// Stable names for mesh elements.
//
// The property under test is the one the feature history depends on: an
// element a user picked is still findable after the steps before it change.
// Everything else here supports that -- names have to exist on every element,
// be unique, survive an operation that does not destroy the element, and come
// out the same every time the same primitive is built.
//
// These are the names a mesh primitive carries. Operations on meshes went when
// modelling moved to the exact kernel, and the names an operation hands out are
// tested with that kernel (test_brep, test_chain).
//
// The failure this guards against is not a crash. Raise a cylinder's segment
// count under a rim fillet and index-based references still resolve, to
// different edges, and the model comes back valid and wrong.
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
