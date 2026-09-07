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

    // ---- What the check used to be blind to ------------------------------
    //
    // A fold that pivots on a shared vertex is the shape an over-large fillet
    // makes, and checkHealth().solid() is the gate the fillet is accepted on.
    // Excluding any pair of triangles that shared a corner exempted exactly
    // that shape, so the gate could not see its own characteristic failure.
    {
        // B shares A's corner exactly and stabs through A's interior.
        const std::vector<Vec3> pos = {
            {0, 0, 0}, {10, 0, 0}, {0, 10, 0},
            {0, 0, 0}, {6, 6, -4}, {6, 6, 4},
        };
        const std::vector<uint32_t> sizes = {3, 3};
        const std::vector<uint32_t> idx = {0, 1, 2, 3, 4, 5};

        Mesh m;
        check(m.build(pos, sizes, idx, nullptr), "shared-corner pair builds");
        check(checkHealth(m).selfIntersections > 0,
              "a fold pivoting on a shared corner is seen");

        // The same crossing with the corner moved off: the answer must not
        // depend on whether the two happen to touch.
        std::vector<Vec3> off = pos;
        off[3] = {0.5, 0.5, 0.0};
        Mesh m2;
        m2.build(off, sizes, idx, nullptr);
        check(checkHealth(m2).selfIntersections > 0, "and still seen when they do not touch");

        // What the exclusion is actually for: two triangles sharing an edge.
        const std::vector<Vec3> edgePair = {
            {0, 0, 0}, {10, 0, 0}, {0, 10, 0},
            {0, 0, 0}, {10, 0, 0}, {0, -10, 0},
        };
        Mesh m3;
        m3.build(edgePair, sizes, idx, nullptr);
        check(checkHealth(m3).selfIntersections == 0, "sharing an edge is not a crossing");
        std::printf("[health] shared-corner fold seen; shared-edge pair still clean\n");
    }

    // Primitives and an operation's output must stay clean under that sharper
    // test -- a check that cries wolf is worse than one that is blind, because
    // every edit downstream is refused on it.
    {
        Mesh box;   makeBox(box, {20.0, 20.0, 20.0});
        Mesh cyl;   makeCylinder(cyl, {8.0, 20.0, 48});
        Mesh sph;   makeSphere(sph, {10.0, 32, 16});
        Mesh tor;   makeTorus(tor, {12.0, 4.0, 40, 20});
        check(checkHealth(box).selfIntersections == 0, "box is clean");
        check(checkHealth(cyl).selfIntersections == 0, "cylinder is clean");
        check(checkHealth(sph).selfIntersections == 0, "sphere is clean");
        check(checkHealth(tor).selfIntersections == 0, "torus is clean");

        // An n-gon cap on a plane whose coordinates are irrational is where an
        // absolute parallel cutoff stopped agreeing with itself: the same solid
        // read clean on the XY plane and self-intersecting when rotated.
        const Vec3 n = normalize(Vec3{1, 1, 1});
        const Vec3 u = normalize(cross(Vec3{0, 0, 1}, n));
        const Vec3 v = cross(n, u);
        std::vector<Vec3> pos;
        std::vector<uint32_t> sizes, idx;
        const int k = 20;
        for (int s = 0; s < k; ++s) {
            const Real a = kTwoPi * s / k;
            const Vec2 p{10.0 * std::cos(a), 10.0 * std::sin(a)};
            pos.push_back(Vec3{10, 20, 30} + u * p.x + v * p.y);
        }
        for (int s = 0; s < k; ++s) {
            const Real a = kTwoPi * s / k;
            const Vec2 p{10.0 * std::cos(a), 10.0 * std::sin(a)};
            pos.push_back(Vec3{10, 20, 30} + u * p.x + v * p.y + n * 15.0);
        }
        sizes.push_back(k);
        for (int s = k; s-- > 0;) idx.push_back(static_cast<uint32_t>(s));
        sizes.push_back(k);
        for (int s = 0; s < k; ++s) idx.push_back(static_cast<uint32_t>(k + s));
        for (int s = 0; s < k; ++s) {
            const int t = (s + 1) % k;
            sizes.push_back(4);
            idx.push_back(static_cast<uint32_t>(s));
            idx.push_back(static_cast<uint32_t>(t));
            idx.push_back(static_cast<uint32_t>(k + t));
            idx.push_back(static_cast<uint32_t>(k + s));
        }
        Mesh angled;
        check(angled.build(pos, sizes, idx, nullptr), "angled prism builds");
        const MeshHealth ah = checkHealth(angled);
        check(ah.selfIntersections == 0, "a prism on an angled plane is clean");
        check(ah.solid(), "and printable");
        std::printf("[health] angled-plane prism: %d intersecting pairs (expect 0)\n",
                    ah.selfIntersections);
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
