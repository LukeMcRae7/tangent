// Where does the time actually go on a heavy model?
#include "mesh/health.h"
#include "mesh/operations.h"
#include "mesh/primitives.h"
#include "scene/scene.h"

#include <chrono>
#include <cstdio>
#include <functional>

using namespace tg;
using Clock = std::chrono::steady_clock;

static double ms(std::function<void()> fn, int reps = 1) {
    const auto t0 = Clock::now();
    for (int i = 0; i < reps; ++i) fn();
    const auto t1 = Clock::now();
    return std::chrono::duration<double, std::milli>(t1 - t0).count() / reps;
}

int main() {
    for (int seg : {128, 320}) {
        SphereParams sp;
        sp.segments = seg;
        sp.rings = seg / 2;
        Mesh m;
        makeSphere(m, sp);

        RenderMesh rm;
        m.buildRenderMesh(rm);
        const size_t tris = rm.triangles.size() / 3;
        std::printf("\n=== sphere %d x %d : %d faces, %zu tris ===\n",
                    seg, seg / 2, m.faceCount(), tris);

        std::printf("  build (soup -> halfedge)   %8.2f ms\n", ms([&] {
            Mesh t; std::vector<Vec3> pos; std::vector<uint32_t> sz, ix;
            for (const MeshVertex& v : m.verts) pos.push_back(v.position);
            for (Index f = 0; f < m.faceCount(); ++f) {
                std::vector<Index> vs; m.faceVertices(f, vs);
                sz.push_back(static_cast<uint32_t>(vs.size()));
                for (Index v : vs) ix.push_back(static_cast<uint32_t>(v));
            }
            t.build(pos, sz, ix);
        }));

        std::printf("  buildRenderMesh            %8.2f ms   <- runs per frame while dragging\n",
                    ms([&] { RenderMesh r; m.buildRenderMesh(r); }));

        std::printf("  checkHealth (no self-int)  %8.2f ms\n",
                    ms([&] { checkHealth(m, false); }));
        std::printf("  checkHealth (full)         %8.2f ms   <- runs on every mesh change\n",
                    ms([&] { checkHealth(m, true); }));

        Scene s;
        PrimitiveSpec spec; spec.kind = PrimitiveKind::Sphere; spec.sphere = sp;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Sphere, spec);
        std::printf("  raycast (one pick)         %8.2f ms\n", ms([&] {
            s.raycast(Ray{{0, 0, 500}, {0, 0, -1}});
        }, 5));

        std::printf("  evaluate chain (1 feature) %8.2f ms\n",
                    ms([&] { s.reevaluate(id); }));

        // A chain with real operations on it, as a model accumulates history.
        Feature ext;
        ext.kind = FeatureKind::Extrude;
        ext.faces = {0};
        ext.distance = 1.0f;
        for (int i = 0; i < 4; ++i) s.addFeature(id, ext);
        std::printf("  evaluate chain, full       %8.2f ms\n",
                    ms([&] { s.reevaluate(id); }));
        // What a slider drag on the last feature actually costs now.
        const size_t last = s.find(id)->features.size() - 1;
        std::printf("  evaluate from last feature %8.2f ms   <- slider drag\n",
                    ms([&] { s.reevaluateFrom(id, last); }));
        std::printf("  add one more feature       %8.2f ms\n", ms([&] {
            Feature b2; b2.kind = FeatureKind::Extrude; b2.faces = {0}; b2.distance = 0.2f;
            s.addFeature(id, b2);
        }));
    }
    return 0;
}
