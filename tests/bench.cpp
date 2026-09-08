// Where does the time actually go on a heavy model?
#include "mesh/health.h"
#include "mesh/boolean.h"
#include "mesh/operations.h"
#include "mesh/primitives.h"
#include "app/camera.h"
#include "geom/body.h"
#include "render/lod.h"
#include "geom/operations.h"
#include "scene/scene.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <functional>

using namespace tg;
using Clock = std::chrono::steady_clock;

static double ms(std::function<void()> fn, int reps = 1) {
    const auto t0 = Clock::now();
    for (int i = 0; i < reps; ++i) fn();
    const auto t1 = Clock::now();
    return std::chrono::duration<double, std::milli>(t1 - t0).count() / reps;
}

// The same part on both kernels, through the seam, so the numbers are
// comparable: a plate with a bolt circle, then every rim rounded at once.
static void benchBackends() {
    if (!brep::available()) {
        std::printf("\n=== B-rep backend not built; skipping the comparison ===\n");
        return;
    }

    for (Backend backend : {Backend::Mesh, Backend::Brep}) {
        const char* what = backend == Backend::Mesh ? "mesh (32 segments)" : "exact";

        PrimitiveSpec plateSpec;
        plateSpec.kind = PrimitiveKind::Box;
        plateSpec.box = {100.0, 100.0, 10.0};

        PrimitiveSpec boreSpec;
        boreSpec.kind = PrimitiveKind::Cylinder;
        boreSpec.cylinder.radius = 3.3;
        boreSpec.cylinder.height = 40.0;
        boreSpec.cylinder.segments = 32;

        Body body;
        int cut = 0;
        const double buildMs = ms([&] {
            makePrimitive(plateSpec, body, backend);
            cut = 0;
            for (int i = 0; i < 8; ++i) {
                const double a = 2.0 * 3.14159265358979 * i / 8.0;
                Body tool;
                if (!makePrimitive(boreSpec, tool, backend)) break;
                tool.transform(translate({35.0 * std::cos(a), 35.0 * std::sin(a), 0}));
                Body out;
                if (!booleanOp(body, tool, BooleanOp::Difference, out,
                               static_cast<ElementId>(200 + i), false, nullptr)) break;
                body = std::move(out);
                ++cut;
            }
        });

        // Every rim, in one operation.
        std::vector<EdgeId> rims;
        std::vector<EdgeId> edges;
        body.allEdges(edges);
        for (EdgeId e : edges) {
            Vec3 p, q;
            body.edgePositions(e, p, q);
            if (std::fabs(p.z - 5.0) < 1e-6 && std::fabs(q.z - 5.0) < 1e-6 &&
                std::hypot(p.x, p.y) < 49.0)
                rims.push_back(e);
        }
        FilletSpec spec;
        spec.salt = 4242;
        spec.segments = 4;
        for (EdgeId e : rims) spec.edges.push_back({e, 1.0});

        Body rounded = body;
        std::string why;
        bool filletOk = false;
        const double filletMs = ms([&] {
            rounded = body;
            filletOk = filletEdges(rounded, spec, &why);
        });

        RenderMesh rm;
        const double tessMs = ms([&] { rm.clear(); rounded.tessellate(rm); });

        std::printf("\n=== bolt circle, %s ===\n", what);
        std::printf("  8 cuts                   %8.2f ms   (%d of 8)\n", buildMs, cut);
        std::printf("  fillet %2zu rims at once   %8.2f ms   %s\n", rims.size(), filletMs,
                    filletOk ? "ok" : ("refused: " + why).c_str());
        std::printf("  tessellate               %8.2f ms   %zu triangles\n",
                    tessMs, rm.triangles.size() / 3);
        std::printf("  faces                    %8d\n", body.faceCount());
        std::printf("  volume                   %8.1f mm3\n", body.health(false).volume);
    }
}

// What the level-of-detail pass costs when it has nothing to do, which is most
// frames, and what a re-tessellation costs when it does.
static void benchLod() {
    if (!brep::available()) return;

    Scene s;
    for (int i = 0; i < 50; ++i) {
        PrimitiveSpec spec;
        spec.kind = PrimitiveKind::Cylinder;
        spec.cylinder.radius = 5;
        spec.cylinder.height = 10;
        s.addPrimitive(PrimitiveKind::Cylinder, spec,
                       {static_cast<Real>(i % 10) * 20, static_cast<Real>(i / 10) * 20, 0});
    }

    Camera cam;
    cam.viewportW = 1600;
    cam.viewportH = 900;
    cam.distance = 300.0f;
    cam.target = {0, 0, 0};
    cam.snapToGoal();

    LodPolicy p;
    p.budgetPerFrame = 64;
    while (refreshTessellation(s, cam, p) > 0) {}      // settle

    std::printf("\n=== level of detail, 50 exact bodies ===\n");
    std::printf("  steady frame (nothing to do)  %8.3f ms\n",
                ms([&] { refreshTessellation(s, cam, p); }, 20));

    // A body that has to be re-tessellated because the view moved.
    SceneObject* one = s.objects().front().get();
    const Real settled = one->renderDeviation;
    std::printf("  one body re-tessellated        %8.3f ms\n", ms([&] {
        TessellationQuality q;
        q.deviationMm = settled * 0.25;
        one->body.tessellate(one->render, q);
    }));
    std::printf("  the same tolerance again       %8.3f ms   <- the cache\n", ms([&] {
        TessellationQuality q;
        q.deviationMm = settled * 0.25;
        one->body.tessellate(one->render, q);
    }, 20));
}

int main() {
    benchBackends();
    benchLod();

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
        // Faces are held by name, not index, so the reference has to be taken
        // from the mesh as it stands. (This file is EXCLUDE_FROM_ALL, so it had
        // gone on compiling against the older brace-initialised form.)
        Feature ext;
        ext.kind = FeatureKind::Extrude;
        ext.faces = nameFaces(s.find(id)->body, {0});
        ext.distance = 1.0f;
        for (int i = 0; i < 4; ++i) s.addFeature(id, ext);
        std::printf("  evaluate chain, full       %8.2f ms\n",
                    ms([&] { s.reevaluate(id); }));
        // What a slider drag on the last feature actually costs now.
        const size_t last = s.find(id)->features.size() - 1;
        std::printf("  evaluate from last feature %8.2f ms   <- slider drag\n",
                    ms([&] { s.reevaluateFrom(id, last); }));
        std::printf("  add one more feature       %8.2f ms\n", ms([&] {
            Feature b2; b2.kind = FeatureKind::Extrude; b2.distance = 0.2f;
            b2.faces = nameFaces(s.find(id)->body, {0});
            s.addFeature(id, b2);
        }));
    }

    // ---- Booleans ---------------------------------------------------------
    //
    // A cut is the slowest thing the interface does, and its cost is set by how
    // many pieces the split leaves for the classifier to work through. The
    // segment count is the knob that drives it: every facet of the cutter
    // contributes a plane, and every plane cuts the whole face it crosses.
    {
        std::printf("\nboolean, a cylinder bored through a 40mm plate\n");
        for (int seg : {16, 32, 64, 96}) {
            Mesh box;
            makeBox(box, {40.0, 40.0, 40.0});
            Mesh drill;
            makeCylinder(drill, {10.0, 60.0, seg});
            Mesh out;
            // Timed first: the arguments to one printf are evaluated in no
            // particular order, so out.faceCount() can be read before the call.
            const double t = ms([&] { meshBoolean(box, drill, BooleanOp::Difference, out, 9); });
            std::printf("  %3d segments               %8.2f ms   -> %d faces\n",
                        seg, t, out.faceCount());
        }

        std::printf("\nboolean, successive cuts into one body\n");
        Mesh cur;
        makeBox(cur, {20.0, 20.0, 20.0});
        for (int i = 0; i < 4; ++i) {
            Mesh cutter;
            makeBox(cutter, {5.0, 5.0, 30.0});
            const Vec3 at{(i % 2) ? 6.0 : -6.0, (i / 2) ? 6.0 : -6.0, 0.0};
            for (MeshVertex& v : cutter.verts) v.position += at;
            Mesh out;
            bool ok = false;
            const double t = ms([&] { ok = meshBoolean(cur, cutter, BooleanOp::Difference, out, 100 + i); });
            if (!ok) { std::printf("  cut %d                       refused\n", i); break; }
            cur = std::move(out);
            std::printf("  cut %d                      %8.2f ms   -> %d faces\n", i, t, cur.faceCount());
        }
    }
    return 0;
}
