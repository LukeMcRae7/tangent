// Snapping to the geometry rather than to whatever was nearby.
//
// Headless: the camera is set up by hand and asked what it would snap to at a
// given pixel. What matters here is which candidate wins, since that is the
// difference between a hole's centre being reachable and being permanently
// shadowed by an edge midpoint two pixels closer.
#include "app/snap.h"
#include "geom/operations.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near(Real a, Real b, Real eps = 1e-4) { return std::fabs(a - b) < eps; }

static Camera lookingDown(float distance = 120.0f) {
    Camera c;
    c.viewportW = 1600;
    c.viewportH = 900;
    c.distance = distance;
    c.target = {0, 0, 0};
    c.yaw = 0.0f;
    c.pitch = 1.5707f;           // straight down the Z axis, from above
    c.snapToGoal();
    return c;
}

// Where a world point lands on screen, so a test can aim at it.
static Vec2 pixelOf(const Camera& c, Vec3 world) {
    Vec2 sp{};
    c.projectToPixel(world, sp);
    return sp;
}

int main() {
    if (!brep::available()) {
        std::printf("exact kernel not built; snapping to curves needs one\n");
        return 0;
    }

    Scene s;
    // A plate with a hole in it: the case the whole feature exists for.
    PrimitiveSpec plate;
    plate.kind = PrimitiveKind::Box;
    plate.box = {80, 80, 10};
    const ObjectId id = s.addPrimitive(PrimitiveKind::Box, plate);

    PrimitiveSpec bore;
    bore.kind = PrimitiveKind::Cylinder;
    bore.cylinder.radius = 9;
    bore.cylinder.height = 40;
    Body tool;
    check(makePrimitive(bore, tool, Backend::Brep), "bore tool");
    tool.transform(translate({20, 0, 0}));

    Feature cut;
    cut.kind = FeatureKind::Boolean;
    cut.booleanOp = BooleanOp::Difference;
    cut.bakedBody = std::move(tool);
    check(s.addFeature(id, cut), "the hole is cut");

    const Camera cam = lookingDown();

    std::printf("--- the centre of a hole ---\n");
    {
        // Aim a few pixels off the true centre: a person does not click exactly.
        const Vec3 trueCentre{20, 0, 5};
        const Vec2 at = pixelOf(cam, trueCentre) + Vec2{4.0f, 3.0f};
        const SnapHit h = findSnap(s, cam, at);
        check(h.kind == SnapKind::CircleCentre,
              std::string("snaps to the centre, not to ") + snapKindName(h.kind));
        check(near(h.point.x, 20.0, 1e-6) && near(h.point.y, 0.0, 1e-6),
              "at exactly the middle of the hole");
        check(near(h.radius, 9.0, 1e-6), "and it knows the radius, so a tool can offer a diameter");
        std::printf("  centre at %.3f, %.3f  r %.3f\n", h.point.x, h.point.y, h.radius);
    }

    std::printf("--- the quadrants around it ---\n");
    {
        const Vec3 quadrant{29, 0, 5};       // the +X side of the rim
        const SnapHit h = findSnap(s, cam, pixelOf(cam, quadrant) + Vec2{2.0f, 0.0f});
        check(h.kind == SnapKind::ArcQuadrant,
              std::string("the rim offers a quadrant, got ") + snapKindName(h.kind));
        check(near(length(Vec3{h.point.x - 20, h.point.y, 0}), 9.0, 1e-6),
              "and it sits on the circle");
        std::printf("  %s at %.3f, %.3f\n", snapKindName(h.kind), h.point.x, h.point.y);
    }

    std::printf("--- a corner beats everything near it ---\n");
    {
        const Vec3 corner{40, 40, 5};
        const SnapHit h = findSnap(s, cam, pixelOf(cam, corner) + Vec2{3.0f, -2.0f});
        check(h.kind == SnapKind::Vertex, std::string("a corner wins, got ") + snapKindName(h.kind));
        check(near(h.point.x, 40.0, 1e-6) && near(h.point.y, 40.0, 1e-6), "at the corner");
    }

    std::printf("--- nothing near the cursor is not a snap ---\n");
    {
        const SnapHit h = findSnap(s, cam, Vec2{20.0f, 20.0f});   // a corner of the viewport
        check(!h.valid(), "empty space snaps to nothing");
    }

    std::printf("--- the radius is a radius, not an aiming aid ---\n");
    {
        SnapConfig tight;
        tight.radiusPx = 3.0;
        const Vec3 trueCentre{20, 0, 5};
        check(findSnap(s, cam, pixelOf(cam, trueCentre) + Vec2{2.0f, 0.0f}, tight).valid(),
              "2 pixels away is inside a 3 pixel radius");
        check(!findSnap(s, cam, pixelOf(cam, trueCentre) + Vec2{9.0f, 0.0f}, tight).valid(),
              "9 pixels away is not");
    }

    std::printf("--- what can be turned off ---\n");
    {
        SnapConfig noCircles;
        noCircles.circles = false;
        const Vec3 trueCentre{20, 0, 5};
        const SnapHit h = findSnap(s, cam, pixelOf(cam, trueCentre), noCircles);
        check(h.kind != SnapKind::CircleCentre, "circles off means no centre");
    }

    std::printf("--- looking through a hole, the near rim wins ---\n");
    {
        // Both rims of a through hole are the same point on screen. Without a
        // tie-break on depth the answer is whichever the kernel enumerated
        // first, which puts a sketch point ten millimetres below the face being
        // drawn on -- and nothing about the picture says so.
        const SnapHit h = findSnap(s, cam, pixelOf(cam, Vec3{20, 0, 5}) + Vec2{3.0f, 0.0f});
        check(h.kind == SnapKind::CircleCentre, "still the centre");
        check(near(h.point.z, 5.0, 1e-6),
              "and the near rim's, not the one on the far side of the plate");
        std::printf("  centre at z = %.3f (near face is z = 5)\n", h.point.z);
    }

    std::printf("--- a mesh body still snaps to what it has ---\n");
    {
        Scene m;
        m.setDefaultBackend(Backend::Mesh);
        const ObjectId mid = m.addPrimitive(PrimitiveKind::Box);
        (void)mid;
        const Camera c2 = lookingDown(60.0f);
        const SnapHit h = findSnap(m, c2, pixelOf(c2, Vec3{10, 10, 10}) + Vec2{2.0f, 2.0f});
        check(h.kind == SnapKind::Vertex, "a mesh corner is still a corner");
        // and nothing claims to be round, because nothing is
        SnapConfig only;
        only.vertices = false;
        only.midpoints = false;
        only.faceCentres = false;
        check(!findSnap(m, c2, pixelOf(c2, Vec3{0, 0, 10}), only).valid(),
              "a mesh has no circles to offer");
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
