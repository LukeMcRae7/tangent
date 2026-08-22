#include "mesh/primitives.h"
#include <cstdio>
#include <string>
using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { printf("  FAIL: %s\n", what.c_str()); ++failures; }
}

static double meshVolume(const Mesh& m) {
    RenderMesh rm; m.buildRenderMesh(rm);
    double s6 = 0.0;
    for (size_t i = 0; i < rm.triangles.size(); i += 3)
        s6 += dot(rm.positions[rm.triangles[i]],
                  cross(rm.positions[rm.triangles[i+1]], rm.positions[rm.triangles[i+2]]));
    return s6 / 6.0;
}

// Closed genus-g surface: V - E + F = 2 - 2g
static void inspect(const char* name, const Mesh& m, int expectGenus, bool closed) {
    printf("[%s]\n", name);
    std::string err;
    check(m.validate(&err), "validate: " + err);

    int boundaryHe = 0;
    for (Index h = 0; h < m.halfedgeCount(); ++h)
        if (m.halfedges[h].face == kInvalid) ++boundaryHe;
    if (closed) check(boundaryHe == 0, "expected closed surface, found boundary edges");

    const int V = m.vertexCount(), F = m.faceCount(), E = m.halfedgeCount() / 2;
    const int chi = V - E + F;
    printf("  V=%d E=%d F=%d  chi=%d\n", V, E, F, chi);
    if (closed) check(chi == 2 - 2 * expectGenus, "wrong Euler characteristic");

    // Orientation via the divergence theorem: a closed mesh wound outward
    // encloses positive signed volume. Unlike a "normal points away from the
    // centroid" test this is also correct for non-star-shaped solids (torus).
    RenderMesh vol;
    m.buildRenderMesh(vol);
    double signed6 = 0.0;
    for (size_t i = 0; i < vol.triangles.size(); i += 3) {
        const Vec3& a = vol.positions[vol.triangles[i + 0]];
        const Vec3& b = vol.positions[vol.triangles[i + 1]];
        const Vec3& c = vol.positions[vol.triangles[i + 2]];
        signed6 += dot(a, cross(b, c));
    }
    const double volume = signed6 / 6.0;
    if (closed) {
        printf("  signed volume = %.3f\n", volume);
        check(volume > 0.0, "mesh is wound inside-out (negative signed volume)");
    }

    // Face normals must agree with the winding of their own triangles.
    int disagree = 0;
    for (size_t i = 0; i < vol.triangles.size(); i += 3) {
        const Vec3& a = vol.positions[vol.triangles[i + 0]];
        const Vec3& b = vol.positions[vol.triangles[i + 1]];
        const Vec3& c = vol.positions[vol.triangles[i + 2]];
        const Vec3 tn = cross(b - a, c - a);
        if (lengthSq(tn) < 1e-12f) continue;
        if (dot(normalize(tn), m.faceNormal(vol.triangleFace[i / 3])) < 0.5f) ++disagree;
    }
    check(disagree == 0, "triangle winding disagrees with face normal: " + std::to_string(disagree));

    RenderMesh rm;
    m.buildRenderMesh(rm);
    check(!rm.triangles.empty(), "no triangles produced");
    check(rm.triangles.size() / 3 == rm.triangleFace.size(), "triangleFace out of sync");
    check(rm.positions.size() == rm.normals.size(), "normal count mismatch");
    for (uint32_t i : rm.triangles) check(i < rm.positions.size(), "triangle index out of range");
    for (uint32_t i : rm.edgeLines) check(i < rm.positions.size(), "edge index out of range");
    for (const Vec3& n : rm.normals) check(std::fabs(length(n) - 1.0f) < 1e-3f, "normal not unit");
    printf("  render: %zu verts, %zu tris, %zu edges\n",
           rm.positions.size(), rm.triangles.size() / 3, rm.edgeLines.size() / 2);
}

int main() {
    Mesh m;
    check(makeBox(m), "makeBox");           inspect("box", m, 0, true);
    check(std::fabs(meshVolume(m) - 8000.0) < 1e-3, "box volume != 8000");
    check(makeCylinder(m), "makeCylinder"); inspect("cylinder", m, 0, true);
    check(makeSphere(m), "makeSphere");     inspect("sphere", m, 0, true);
    { const double PI = 3.14159265358979;
      const double got = meshVolume(m), ideal = 4.0 / 3.0 * PI * 1000.0;
      printf("[sphere] volume=%.3f ideal=%.3f (inscribed, so slightly under)\n", got, ideal);
      check(got < ideal && got > ideal * 0.97, "sphere volume outside inscribed bounds"); }
    check(makeCone(m), "makeCone(apex)");   inspect("cone-apex", m, 0, true);
    ConeParams cp; cp.topRadius = 5.0f;
    check(makeCone(m, cp), "makeCone(frustum)"); inspect("cone-frustum", m, 0, true);
    check(makeTorus(m), "makeTorus");       inspect("torus", m, 1, true);
    { // The faceted torus is inscribed in the true one, so its volume falls
      // short by a factor of (n/2pi)*sin(2pi/n) per sweep -- exactly, not
      // approximately. Predicting it lets us assert to 0.05% instead of
      // hiding a real winding bug behind a loose tolerance.
      const double PI = 3.14159265358979;
      const TorusParams tp{};
      auto inscribe = [&](int n) { return (n / (2.0 * PI)) * std::sin(2.0 * PI / n); };
      const double want = 2.0 * PI * PI * tp.majorRadius * tp.minorRadius * tp.minorRadius
                          * inscribe(tp.minorSegments) * inscribe(tp.majorSegments);
      const double got = meshVolume(m);
      printf("[torus] volume=%.3f predicted=%.3f  (rel %.5f)\n", got, want, std::fabs(got-want)/want);
      check(std::fabs(got - want) / want < 5e-4, "torus volume does not match the exact faceted prediction"); }
    check(makePlane(m), "makePlane");       inspect("plane", m, 0, false);

    // Concave polygon: ear clipping must not emit triangles outside the shape.
    // An L-shaped hexagon has area 3 when built from unit steps.
    {
        Mesh L;
        std::vector<Vec3> pos = {{0,0,0},{2,0,0},{2,1,0},{1,1,0},{1,2,0},{0,2,0}};
        check(L.build(pos, {6}, {0,1,2,3,4,5}), "build L-shape");
        RenderMesh rm; L.buildRenderMesh(rm);
        check(rm.triangles.size() == 4 * 3, "L-shape should clip to 4 triangles");
        float area = 0.0f;
        for (size_t i = 0; i < rm.triangles.size(); i += 3)
            area += length(cross(rm.positions[rm.triangles[i+1]] - rm.positions[rm.triangles[i]],
                                 rm.positions[rm.triangles[i+2]] - rm.positions[rm.triangles[i]])) * 0.5f;
        printf("[L-shape] area=%.4f (expect 3.0)\n", area);
        check(std::fabs(area - 3.0f) < 1e-4f, "concave triangulation area wrong");
    }

    // Non-manifold input must be rejected, not silently mis-linked.
    {
        Mesh bad;
        std::vector<Vec3> pos = {{0,0,0},{1,0,0},{0,1,0},{0,0,1}};
        // Three faces sharing the directed edge 0->1.
        check(!bad.build(pos, {3,3}, {0,1,2, 0,1,3}), "non-manifold input accepted");
        printf("[non-manifold] correctly rejected\n");
    }

    printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
