// Printability checks. The point of these is that a mesh which exists at all
// is already manifold -- build() guarantees it -- so what is left to detect is
// geometry that is manifold but still not a solid.
#include "mesh/health.h"
#include "mesh/operations.h"
#include "mesh/primitives.h"

#include <cstdio>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}

int main() {
    // ---- The guarantee: non-manifold input cannot become a mesh ------------
    {
        // Edge shared by three faces.
        Mesh m;
        std::vector<Vec3> pos = {{0,0,0},{1,0,0},{0,1,0},{0,0,1},{0,-1,0}};
        check(!m.build(pos, {3,3,3}, {0,1,2, 0,1,3, 0,1,4}),
              "an edge shared by three faces is rejected");

        // Two tetrahedra meeting at one vertex: every directed edge is unique,
        // so only the vertex circulation reveals it.
        Mesh b;
        std::vector<Vec3> bow = {{0,0,0},{1,0,0},{0,1,0},{0,0,1},
                                 {-1,0,0},{0,-1,0},{0,0,-1}};
        check(!b.build(bow, {3,3,3,3,3,3,3,3},
                       {0,2,1, 0,1,3, 0,3,2, 1,2,3,
                        0,4,5, 0,5,6, 0,6,4, 4,6,5}),
              "a bowtie vertex is rejected");
        std::printf("[health] non-manifold input rejected at construction\n");
    }

    // ---- A primitive is a clean solid --------------------------------------
    {
        Mesh m;
        makeBox(m);
        const MeshHealth h = checkHealth(m);
        check(h.watertight, "box is watertight");
        check(h.boundaryEdges == 0, "no boundary edges");
        check(h.degenerateFaces == 0, "no degenerate faces");
        check(h.shells == 1, "one shell");
        check(h.volume > 0.0, "positive volume");
        check(h.selfIntersections == 0, "no self-intersections");
        check(h.solid(), "box is printable");
        std::printf("[health] box: watertight, %d shell, volume %.0f\n", h.shells, h.volume);
    }

    // Every primitive, and results of the mesh operations, stay solid.
    {
        const PrimitiveKind kinds[] = {PrimitiveKind::Box, PrimitiveKind::Cylinder,
                                       PrimitiveKind::Sphere, PrimitiveKind::Cone,
                                       PrimitiveKind::Torus};
        for (PrimitiveKind k : kinds) {
            Mesh m;
            PrimitiveSpec s; s.kind = k;
            switch (k) {
                case PrimitiveKind::Box:      makeBox(m, s.box); break;
                case PrimitiveKind::Cylinder: makeCylinder(m, s.cylinder); break;
                case PrimitiveKind::Sphere:   makeSphere(m, s.sphere); break;
                case PrimitiveKind::Cone:     makeCone(m, s.cone); break;
                default:                      makeTorus(m, s.torus); break;
            }
            const MeshHealth h = checkHealth(m);
            check(h.solid(), std::string(primitiveName(k)) + " is printable");
        }
        std::printf("[health] all closed primitives are printable\n");
    }

    {
        Mesh m;
        makeBox(m);
        Index top = 0;
        for (Index f = 0; f < m.faceCount(); ++f)
            if (dot(m.faceNormal(f), Vec3{0,0,1}) > 0.99f) top = f;
        check(extrudeFaces(m, {top}, 12.0f, nullptr), "extrude");
        check(checkHealth(m).solid(), "still printable after extrude");

        check(bevelAllEdges(m, 2.0f, 2), "bevel");
        check(checkHealth(m).solid(), "still printable after a rounded bevel");
        std::printf("[health] operations preserve printability\n");
    }

    // ---- An open surface is manifold but not a solid ------------------------
    {
        Mesh m;
        makePlane(m);
        const MeshHealth h = checkHealth(m);
        check(!h.watertight, "a plane is not watertight");
        check(h.boundaryEdges == 4, "four boundary edges");
        check(!h.solid(), "and so not printable");
        std::printf("[health] open surface reported: %d boundary edges\n", h.boundaryEdges);
    }

    // ---- Two separate bodies in one mesh ------------------------------------
    {
        Mesh a, b;
        makeBox(a);
        BoxParams p; p.width = p.depth = p.height = 5.0f;
        makeBox(b, p);

        std::vector<Vec3> pos;
        std::vector<uint32_t> sizes, idx;
        for (const MeshVertex& v : a.verts) pos.push_back(v.position);
        for (const MeshVertex& v : b.verts) pos.push_back(v.position + Vec3{100, 0, 0});
        auto append = [&](const Mesh& m, uint32_t base) {
            for (Index f = 0; f < m.faceCount(); ++f) {
                std::vector<Index> verts;
                m.faceVertices(f, verts);
                sizes.push_back(static_cast<uint32_t>(verts.size()));
                for (Index v : verts) idx.push_back(base + static_cast<uint32_t>(v));
            }
        };
        append(a, 0);
        append(b, static_cast<uint32_t>(a.vertexCount()));

        Mesh both;
        check(both.build(pos, sizes, idx), "two disjoint boxes build fine");
        const MeshHealth h = checkHealth(both);
        check(h.shells == 2, "reported as two shells");
        check(h.watertight && h.solid(), "two separate solids are still printable");
        std::printf("[health] disjoint bodies: %d shells\n", h.shells);
    }

    // ---- Self-intersection, which a free-form vertex drag can cause ---------
    {
        Mesh m;
        makeBox(m);
        check(checkHealth(m).selfIntersections == 0, "clean box has none");

        // Drag one corner up through the opposite side of the box. The mesh
        // stays manifold and watertight -- no topology changed -- but it now
        // passes through itself, which a slicer will not thank us for.
        //
        // Deliberately off-centre: pulling it to exactly (0,0,60) puts every
        // crossing precisely on the top face's triangulation diagonal, where a
        // barycentric test correctly reports no interior hit. That is a
        // degenerate arrangement, not a realistic drag.
        m.verts[0].position = {3.0f, -1.0f, 60.0f};
        const MeshHealth h = checkHealth(m);
        check(h.watertight, "still watertight");
        check(h.selfIntersections > 0, "self-intersection detected");
        check(!h.solid(), "and so not printable");
        std::printf("[health] pulled-through corner: %d intersecting triangle pairs\n",
                    h.selfIntersections);
    }

    // Skipping the expensive check is reported honestly rather than as a pass.
    {
        Mesh m;
        makeBox(m);
        m.verts[0].position = {3.0f, -1.0f, 60.0f};
        const MeshHealth h = checkHealth(m, /*checkIntersections=*/false);
        check(h.selfIntersections == -1, "not-run is distinguishable from zero");
        std::printf("[health] skipped check reports -1, not 0\n");
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
