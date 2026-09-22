// Snapping to the geometry rather than to whatever was nearby.
//
// Headless: the camera is set up by hand and asked what it would snap to at a
// given pixel. What matters here is which candidate wins, since that is the
// difference between a hole's centre being reachable and being permanently
// shadowed by an edge midpoint two pixels closer.
#include "app/plane_snap.h"
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

    // -----------------------------------------------------------------------
    // Snapping on a plane: where the point goes when it is not on anything.
    // -----------------------------------------------------------------------
    PlaneFrame plane;
    plane.origin = {0, 0, 5};          // the plate's top face
    plane.u = {1, 0, 0};
    plane.v = {0, 1, 0};
    plane.normal = {0, 0, 1};

    const Camera top = lookingDown(120.0f);
    auto snapAt = [&](Vec2 uv, const PlaneSnapConfig& cfg = {}) {
        const Vec3 world = plane.toWorld(uv);
        return snapOnPlane(s, top, plane, pixelOf(top, world), uv, cfg);
    };

    std::printf("--- the grid the user can see is the grid it snaps to ---\n");
    {
        const GridLevels g = gridLevelsAt(top, plane.origin);
        check(g.main > 0.0 && near(g.major, g.main * 10.0) && near(g.fine, g.main / 10.0),
              "levels are a decade apart, as the shader draws them");
        // Every level is a power of ten of the 1mm base, so every drawn line is
        // at a round number. That is what makes "prefer a whole number" and
        // "snap to a line you can see" the same rule.
        const Real decades = std::log10(g.main);
        check(near(decades, std::round(decades), 1e-9), "levels are powers of ten");
        std::printf("  at this zoom: fine %g, main %g, major %g mm\n", g.fine, g.main, g.major);
    }

    std::printf("--- a coarse line beats a fine one nearby ---\n");
    {
        const GridLevels g = gridLevelsAt(top, plane.origin);
        // Sitting just off a major line, with a fine line much closer. The
        // major is the rounder number and it is what a person means.
        const Real off = g.fine * 0.35;
        const PlaneSnap hit = snapAt({g.major + off, g.major - off});
        check(near(hit.uv.x, g.major) && near(hit.uv.y, g.major),
              "pulled to the round number, not to the nearest line");
        check(hit.kind == SnapKind::GridPoint, "and says so");
    }

    std::printf("--- between drawn lines is a step, not a snap ---\n");
    {
        const GridLevels g = gridLevelsAt(top, plane.origin);
        const Vec2 between{g.main * 3 + g.fine * 3, g.main * 3 + g.fine * 4};
        const PlaneSnap hit = snapAt(between);
        check(hit.kind == SnapKind::None, "no indicator for landing between lines");
        check(near(std::fmod(hit.uv.x + 1e-9, g.fine), 0.0, g.fine * 1e-3) ||
              near(std::fmod(hit.uv.x + 1e-9, g.fine), g.fine, g.fine * 1e-3),
              "but the value still advances in whole fine steps");
    }

    std::printf("--- in line with the hole ---\n");
    {
        // The bore is at x = 20. Standing well away from it in y, but level
        // with it in x, is an alignment: the point belongs at x = 20 exactly.
        PlaneSnapConfig cfg;
        cfg.grid = false;                     // isolate the alignment
        const PlaneSnap hit = snapAt({20.4, -30.0}, cfg);
        check(hit.kind == SnapKind::Alignment, "reported as an alignment");
        check(near(hit.uv.x, 20.0), "pulled onto the hole's line");
        check(near(hit.uv.y, -30.0), "and left alone on the other axis");
        check(hit.refCount == 1, "one reference");
        check(hit.refs[0].alongU, "which fixes u");
        check(near(hit.refs[0].from.x, 20.0), "and is the hole");
    }

    std::printf("--- where two alignments cross ---\n");
    {
        // Level with the hole in x and with the plate's own corner in y.
        PlaneSnapConfig cfg;
        cfg.grid = false;
        const PlaneSnap hit = snapAt({20.3, 39.6}, cfg);
        check(hit.kind == SnapKind::Intersection, "reported as a crossing");
        check(near(hit.uv.x, 20.0) && near(hit.uv.y, 40.0), "landed on the crossing");
        check(hit.refCount == 2, "two references, so two lines to draw");
        check(hit.refs[0].alongU != hit.refs[1].alongU, "one per axis");
    }

    std::printf("--- a feature under the cursor beats any inference ---\n");
    {
        const Vec3 centre{20, 0, 5};
        const PlaneSnap hit = snapOnPlane(s, top, plane, pixelOf(top, centre) + Vec2{2, 2},
                                          plane.toUV(centre) + Vec2{0.4, 0.4});
        check(hit.kind == SnapKind::CircleCentre, "the hole's centre wins");
        check(hit.refCount == 0, "and needs no reference line");
        check(hit.radius > 0.0, "it knows its size");
    }

    std::printf("--- a caller's own point can be lined up with ---\n");
    {
        PlaneSnapConfig cfg;
        cfg.grid = false;
        std::vector<SnapPoint> mine;
        const Vec3 start = plane.toWorld({-33.0, 12.0});
        mine.push_back({start, plane.toUV(start), SnapKind::Vertex, kNoObject, 0.0});

        const PlaneSnap hit = snapAt({5.0, 12.3}, cfg);
        check(hit.kind == SnapKind::None, "nothing there without it");

        const Vec3 world = plane.toWorld({5.0, 12.3});
        const PlaneSnap withIt = snapOnPlane(s, top, plane, pixelOf(top, world),
                                             {5.0, 12.3}, cfg, mine);
        check(withIt.kind == SnapKind::Alignment, "the start point is a reference too");
        check(near(withIt.uv.y, 12.0), "level with it");
        check(!withIt.refs[0].alongU, "along v");
    }

    std::printf("--- an alignment tolerance is tighter than a hit ---\n");
    {
        // An alignment reaches across the whole screen, so it has to be tight
        // or it catches everything. Well outside it, nothing is claimed.
        PlaneSnapConfig cfg;
        cfg.grid = false;
        const Real px = static_cast<Real>(top.pixelWorldSize(plane.origin));
        const PlaneSnap hit = snapAt({20.0 + px * 20.0, -30.0}, cfg);
        check(hit.kind == SnapKind::None, "twenty pixels off the line is not on it");
    }

    std::printf("--- a crossing survives a nearer reference on one axis ---\n");
    {
        // The corner at (40, 40) is the only thing that can hold v here. If a
        // reference were assigned to one axis before the other had looked, the
        // hole's centre -- nearer in u -- would take u, and the corner, having
        // lost u to it, would never be tried on v. The crossing has to survive
        // that: both axes are searched over everything.
        PlaneSnapConfig cfg;
        cfg.grid = false;
        const PlaneSnap hit = snapAt({20.2, 39.7}, cfg);
        check(hit.kind == SnapKind::Intersection, "still a crossing");
        check(near(hit.uv.x, 20.0) && near(hit.uv.y, 40.0), "on the hole's line and the corner's");
        check(hit.refs[0].alongU && !hit.refs[1].alongU, "u from one, v from the other");
    }

    std::printf("--- one reference cannot hold both axes ---\n");
    {
        // Sitting almost on the hole's centre, but far enough away on screen
        // that it is not a hit. It must not be reported as a crossing with
        // itself: it holds the axis it is more convincingly on, and the other
        // is free.
        PlaneSnapConfig cfg;
        cfg.grid = false;
        cfg.points.radiusPx = 1.0;              // refuse the direct hit outright
        cfg.points.vertices = false;            // leave the hole's centre as the
        cfg.points.midpoints = false;           // only thing in reach, so there
        cfg.points.faceCentres = false;         // is nothing to cross with
        const PlaneSnap hit = snapAt({20.1, 0.3}, cfg);
        check(hit.kind == SnapKind::Alignment, "an alignment, not a crossing");
        check(hit.refCount == 1, "from one reference");
        check(near(hit.uv.x, 20.0), "held on the axis it was nearer on");
        check(near(hit.uv.y, 0.3), "and left free on the other");
    }

    std::printf("--- with nothing in the scene, the grid still holds ---\n");
    {
        Scene empty;
        const GridLevels g = gridLevelsAt(top, plane.origin);
        const Vec2 uv{g.main * 2 + g.fine * 0.2, g.main * 2 - g.fine * 0.2};
        const Vec3 world = plane.toWorld(uv);
        const PlaneSnap hit = snapOnPlane(empty, top, plane, pixelOf(top, world), uv);
        check(hit.kind == SnapKind::GridPoint, "a grid crossing, with no bodies at all");
        check(near(hit.uv.x, g.main * 2) && near(hit.uv.y, g.main * 2), "on the crossing");
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
