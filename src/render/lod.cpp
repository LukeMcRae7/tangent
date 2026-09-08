#include "render/lod.h"

#include <algorithm>
#include <cmath>

namespace tg {

Real targetDeviation(const SceneObject& obj, const Camera& camera, const LodPolicy& p) {
    // How big a pixel is out where the body is. Measured at its centre rather
    // than at the origin: two identical parts at different distances should not
    // be drawn at the same tolerance.
    const AABB world = obj.worldBounds();
    const Vec3 centre = (world.min + world.max) * 0.5;
    const Real pixel = static_cast<Real>(camera.pixelWorldSize(centre));

    // Tessellation happens in the body's own space, so an object scaled up
    // needs a proportionally tighter tolerance to look equally smooth.
    const Vec3 s = obj.transform.scale;
    const Real scale = std::max({std::fabs(s.x), std::fabs(s.y), std::fabs(s.z), Real(1e-6)});

    const Real want = pixel * p.pixelTolerance / scale;
    return std::clamp(want, p.minDeviationMm, p.maxDeviationMm);
}

bool needsRetessellation(Real current, Real target, const LodPolicy& p) {
    if (current <= 0.0) return true;                    // never measured
    if (target < current / p.finerFactor) return true;  // the view got closer
    if (target > current * p.coarserFactor) return true;// ...or much further away
    return false;
}

int refreshTessellation(Scene& scene, const Camera& camera, const LodPolicy& p) {
    // Worst first: the body furthest from the tolerance it should have is the
    // one the user is most likely to be looking at the facets of.
    struct Want { SceneObject* obj; Real target; Real ratio; };
    std::vector<Want> wants;

    for (const auto& obj : scene.objects()) {
        SceneObject* o = obj.get();
        if (!o->visible || o->body.empty()) continue;
        if (o->body.isMesh()) continue;                 // fixed at birth; nothing to do

        const Real target = targetDeviation(*o, camera, p);
        if (!needsRetessellation(o->renderDeviation, target, p)) continue;

        const Real ratio = o->renderDeviation > 0.0
                               ? std::max(o->renderDeviation / target, target / o->renderDeviation)
                               : 1e9;
        wants.push_back({o, target, ratio});
    }
    if (wants.empty()) return 0;

    std::sort(wants.begin(), wants.end(),
              [](const Want& a, const Want& b) { return a.ratio > b.ratio; });

    int done = 0;
    for (const Want& w : wants) {
        if (done >= p.budgetPerFrame) break;
        TessellationQuality q;
        q.deviationMm = w.target;
        // Left at the screen's coarse default: it is what keeps a zoom
        // affordable, and the chord tolerance is doing the work here.
        w.obj->body.tessellate(w.obj->render, q);
        w.obj->renderDeviation = w.target;
        w.obj->markMeshChanged();      // the renderer re-uploads what changed
        ++done;
    }
    return done;
}

} // namespace tg
