// How finely a body is drawn, and when that changes.
//
// Headless: no GL context, no window. The camera is set up by hand and the
// policy is asked what it would do, which is the part worth testing -- the
// tessellation itself is covered by test_brep.
#include "render/lod.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}

static Camera viewAt(float distance) {
    Camera c;
    c.viewportW = 1600;
    c.viewportH = 900;
    c.distance = distance;
    c.target = {0, 0, 0};
    c.snapToGoal();
    return c;
}

static ObjectId addCylinder(Scene& s, Backend backend, Vec3 at = {}) {
    PrimitiveSpec spec;
    spec.kind = PrimitiveKind::Cylinder;
    spec.cylinder.radius = 10;
    spec.cylinder.height = 20;
    spec.cylinder.segments = 32;
    const Backend was = s.defaultBackend();
    s.setDefaultBackend(backend);
    const ObjectId id = s.addPrimitive(PrimitiveKind::Cylinder, spec, at);
    s.setDefaultBackend(was);
    return id;
}

int main() {
    std::printf("--- the tolerance follows the view ---\n");
    {
        Scene s;
        const ObjectId id = addCylinder(s, brep::available() ? Backend::Brep : Backend::Mesh);
        const SceneObject* o = s.find(id);

        const Real far_ = targetDeviation(*o, viewAt(400.0f));
        const Real near_ = targetDeviation(*o, viewAt(50.0f));
        check(near_ < far_, "closer asks for a finer tolerance");
        check(std::fabs(far_ / near_ - 8.0) < 0.5,
              "and it scales with distance: 8x further, 8x coarser");
        std::printf("  400mm away: %.4f mm   50mm away: %.4f mm\n", far_, near_);

        // Whatever the zoom, within limits nobody can see past.
        const LodPolicy p;
        check(targetDeviation(*o, viewAt(0.5f)) >= p.minDeviationMm,
              "there is a floor: past it the triangles are finer than the printer");
        check(targetDeviation(*o, viewAt(100000.0f)) <= p.maxDeviationMm, "and a ceiling");
    }

    std::printf("--- an object scaled up is drawn finer ---\n");
    {
        Scene s;
        const ObjectId id = addCylinder(s, brep::available() ? Backend::Brep : Backend::Mesh);
        SceneObject* o = s.find(id);
        const Camera cam = viewAt(400.0f);
        const Real plain = targetDeviation(*o, cam);
        o->transform.scale = {4, 4, 4};
        const Real scaled = targetDeviation(*o, cam);
        // Tessellation happens in the body's own space, so four times the size
        // on screen needs a quarter of the tolerance to look as smooth.
        check(scaled < plain, "a scaled-up body asks for a finer tolerance");
        std::printf("  1x: %.4f mm   4x: %.4f mm\n", plain, scaled);
    }

    std::printf("--- hysteresis: a slow zoom does not re-mesh every frame ---\n");
    {
        const LodPolicy p;
        check(needsRetessellation(0.0, 0.05, p), "a body never measured is redrawn");
        check(!needsRetessellation(0.05, 0.045, p), "a 10% change is not worth the work");
        check(!needsRetessellation(0.05, 0.06, p), "nor is a small step the other way");
        check(needsRetessellation(0.05, 0.02, p), "but zooming right in is");
        check(needsRetessellation(0.05, 0.30, p), "and so is pulling right out");
    }

    if (!brep::available()) {
        std::printf("\nB-rep backend not built; the rest needs one\n");
        std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
        return failures ? 1 : 0;
    }

    std::printf("--- the work is bounded, and spent worst-first ---\n");
    {
        Scene s;
        for (int i = 0; i < 5; ++i)
            addCylinder(s, Backend::Brep, {static_cast<Real>(i) * 40, 0, 0});

        LodPolicy p;
        p.budgetPerFrame = 2;
        const Camera cam = viewAt(80.0f);

        check(refreshTessellation(s, cam, p) == 2, "two bodies a frame, not five");
        check(refreshTessellation(s, cam, p) == 2, "two more on the next frame");
        check(refreshTessellation(s, cam, p) == 1, "and the last one after that");
        check(refreshTessellation(s, cam, p) == 0, "then there is nothing left to do");
    }

    std::printf("--- zooming in actually makes the hole rounder ---\n");
    {
        Scene s;
        const ObjectId id = addCylinder(s, Backend::Brep);
        SceneObject* o = s.find(id);

        LodPolicy p;
        p.budgetPerFrame = 8;
        refreshTessellation(s, viewAt(600.0f), p);
        const size_t coarse = o->render.triangles.size();
        const Real coarseDev = o->renderDeviation;

        refreshTessellation(s, viewAt(40.0f), p);
        const size_t fine = o->render.triangles.size();

        check(fine > coarse, "closer means more triangles");
        check(o->renderDeviation < coarseDev, "at a finer tolerance");
        std::printf("  600mm away: %zu triangles at %.4f mm; 40mm away: %zu at %.4f mm\n",
                    coarse / 3, coarseDev, fine / 3, o->renderDeviation);

        // And back out again, so a big scene does not stay expensive after one
        // close look at one part of it.
        refreshTessellation(s, viewAt(600.0f), p);
        check(o->render.triangles.size() < fine, "and pulling out gives the triangles back");
    }

    std::printf("--- a mesh body is left alone ---\n");
    {
        Scene s;
        const ObjectId id = addCylinder(s, Backend::Mesh);
        SceneObject* o = s.find(id);
        const size_t before = o->render.triangles.size();
        const uint32_t versionBefore = o->meshVersion;

        check(refreshTessellation(s, viewAt(20.0f)) == 0,
              "no work is spent on a body whose resolution was fixed when it was made");
        check(o->render.triangles.size() == before, "and nothing about it changed");
        check(o->meshVersion == versionBefore, "not even a re-upload");
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
