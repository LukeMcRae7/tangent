// The direction a drag is measured along.
//
// Headless: a camera is set up by hand, a world point is projected to find the
// pixel that sits over it, and the axis is asked what value that pixel means.
// The properties here are what separate aiming from discovering, so they are
// tested as properties rather than as one worked example.
#include "app/drag_axis.h"
#include "geom/operations.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near(Real a, Real b, Real eps = 1e-3) { return std::fabs(a - b) < eps; }

static Camera lookingDown(float distance = 200.0f) {
    Camera c;
    c.viewportW = 1600;
    c.viewportH = 900;
    c.distance = distance;
    c.target = {0, 0, 0};
    c.yaw = 0.0f;
    c.pitch = 1.5707f;
    c.snapToGoal();
    return c;
}
static Vec2 pixelOf(const Camera& c, Vec3 world) {
    Vec2 p{};
    c.projectToPixel(world, p);
    return p;
}

int main() {
    std::printf("--- the value is the cursor projected onto the axis ---\n");
    {
        const Camera cam = lookingDown();
        DragAxis axis;
        axis.origin = {0, 0, 0};
        axis.direction = {1, 0, 0};
        axis.valid = true;

        for (Real want : {2.0, 7.5, 25.0}) {
            const Real got = axis.valueAt(cam, pixelOf(cam, Vec3{want, 0, 0}));
            check(near(got, want, 1e-2), "a cursor over " + std::to_string(want) +
                                         "mm along the axis reads " + std::to_string(got));
        }
        // Behind the start there is nothing to measure. A signed projection
        // flips sign near the edge of the screen, where a perspective ray is
        // most oblique, and the arrow used to turn round and point backwards.
        check(near(axis.valueAt(cam, pixelOf(cam, Vec3{-6, 0, 0})), 0.0, 1e-9),
              "behind the start reads as the start, not as a negative");
        check(axis.offsetPx(cam, pixelOf(cam, Vec3{-6, 0, 0})) < 0.0,
              "though the offset on screen still knows which side it is on");

        DragAxis offset = axis;
        offset.baseValue = 1.5;
        check(near(offset.valueAt(cam, pixelOf(cam, Vec3{-6, 0, 0})), 1.5, 1e-9),
              "and with a base value, the start is the floor");
        check(near(offset.valueAt(cam, pixelOf(cam, Vec3{4, 0, 0})), 5.5, 1e-2),
              "with the travel measured from it");
        std::printf("  25mm along reads %.4f\n", axis.valueAt(cam, pixelOf(cam, Vec3{25, 0, 0})));
    }

    std::printf("--- moving across the axis changes nothing ---\n");
    {
        // The property distance-from-a-point cannot have, and the reason this
        // exists: sliding along an edge used to grow the fillet as fast as
        // pulling away from it did.
        const Camera cam = lookingDown();
        DragAxis axis;
        axis.origin = {0, 0, 0};
        axis.direction = {1, 0, 0};
        axis.valid = true;

        // Under an orthographic camera this is exact: every ray is parallel,
        // so a cursor anywhere on the line across the axis means one value.
        Camera ortho = cam;
        ortho.orthographic = true;
        const Real flat = axis.valueAt(ortho, pixelOf(ortho, Vec3{10, 0, 0}));
        for (Real off : {5.0, 20.0, -35.0}) {
            check(near(axis.valueAt(ortho, pixelOf(ortho, Vec3{10, off, 0})), flat, 1e-6),
                  "orthographic: across the axis reads exactly the same");
        }

        // Under perspective it drifts a little, because the ray through a pixel
        // 35mm off to the side really does pass the axis at a slightly
        // different place. A few percent over a third of the viewport, against
        // the 260% that measuring distance from a point would give.
        const Real straight = axis.valueAt(cam, pixelOf(cam, Vec3{10, 0, 0}));
        for (Real off : {5.0, 20.0, -35.0}) {
            const Real got = axis.valueAt(cam, pixelOf(cam, Vec3{10, off, 0}));
            check(std::fabs(got - straight) < straight * 0.05,
                  "perspective: across the axis stays within a few percent");
            const Real asDistance = length(Vec3{10, off, 0});
            check(std::fabs(asDistance - straight) > std::fabs(got - straight) * 5.0,
                  "where distance from the origin would have moved far more");
        }
        std::printf("  10mm along, 35mm across: %.4f perspective, %.4f ortho "
                    "(distance would say %.4f)\n",
                    axis.valueAt(cam, pixelOf(cam, Vec3{10, -35, 0})),
                    axis.valueAt(ortho, pixelOf(ortho, Vec3{10, -35, 0})),
                    static_cast<double>(length(Vec3{10, -35, 0})));
    }

    std::printf("--- an axis pointing at the eye is not measurable ---\n");
    {
        const Camera cam = lookingDown();
        DragAxis across;
        across.origin = {0, 0, 0};
        across.direction = {1, 0, 0};
        across.valid = true;
        check(across.facingCamera(cam), "an axis across the view is fine");

        DragAxis atEye;
        atEye.origin = {0, 0, 0};
        atEye.direction = normalize(cam.eye());
        atEye.valid = true;
        check(!atEye.facingCamera(cam), "one pointing at the eye is not");

        DragAxis oblique;
        oblique.origin = {0, 0, 0};
        oblique.direction = normalize(Vec3{1, 0, 1});
        oblique.valid = true;
        check(oblique.facingCamera(cam), "45 degrees off is still usable");
    }

    std::printf("--- a fillet grows out of the corner it rounds ---\n");
    {
        PrimitiveSpec spec;
        spec.kind = PrimitiveKind::Box;
        spec.box = {20, 20, 20};
        Body cube;
        check(makePrimitive(spec, cube, brep::available() ? Backend::Brep : Backend::Mesh),
              "a cube");

        // The edge along X at the top front: y = -10, z = +10.
        EdgeId target = kInvalid;
        std::vector<EdgeId> edges;
        cube.allEdges(edges);
        for (EdgeId e : edges) {
            Vec3 a, b;
            cube.edgePositions(e, a, b);
            if (near(a.z, 10.0, 1e-6) && near(b.z, 10.0, 1e-6) &&
                near(a.y, -10.0, 1e-6) && near(b.y, -10.0, 1e-6))
                target = e;
        }
        check(target != kInvalid, "found the top front edge");

        const Mat4 identity = translate({0, 0, 0});
        const DragAxis axis = filletAxis(cube, identity, {target}, Vec3{0, -10, 10});
        check(axis.valid, "it has an axis");

        // Out of the corner: the bisector of the top face and the front face,
        // which points up and forward at 45 degrees. Not along the edge.
        check(near(axis.direction.x, 0.0, 1e-6), "square to the edge");
        check(axis.direction.y < -0.6 && axis.direction.z > 0.6,
              "and out of the corner, not into the solid");
        check(near(length(axis.direction), 1.0, 1e-9), "normalised");
        check(near(axis.origin.y, -10.0, 1e-6) && near(axis.origin.z, 10.0, 1e-6),
              "anchored on the edge");
        std::printf("  direction %.3f, %.3f, %.3f\n",
                    axis.direction.x, axis.direction.y, axis.direction.z);

        // Anchored under the pointer, not at the middle of the edge.
        const DragAxis atEnd = filletAxis(cube, identity, {target}, Vec3{8, -10, 10});
        check(near(atEnd.origin.x, 8.0, 1e-6), "the guide follows the cursor along the edge");

        // The direction is a statement about the boundary, so it needs no
        // special case per selection: the four edges around a face sum that
        // face four times and its four sides, which cancel in pairs.
        FaceId top = kNoFace;
        std::vector<FaceId> faces;
        cube.allFaces(faces);
        for (FaceId f : faces)
            if (dot(cube.faceNormal(f), Vec3{0, 0, 1}) > 0.99) top = f;
        check(top != kNoFace, "found the top face");

        std::vector<EdgeId> ring;
        cube.faceEdges(top, ring);
        check(ring.size() == 4, "which has four edges");
        const DragAxis wholeFace = filletAxis(cube, identity, ring, Vec3{0, 0, 10});
        check(wholeFace.valid, "the ring has an axis");
        check(near(wholeFace.direction.z, 1.0, 1e-6),
              "and it points straight up out of the face, not at 45 degrees");

        // Two edges on opposite sides of that face: their side normals cancel
        // and the face's own is what is left, which is still straight up.
        const DragAxis twoOfThem = filletAxis(cube, identity, {ring[0], ring[2]},
                                              Vec3{0, 0, 10});
        check(near(twoOfThem.direction.z, 1.0, 1e-6), "two opposite edges, still up");

        // Two edges meeting at a corner: one side cancels nothing, and the
        // answer leans that way, which is what a person would expect.
        const DragAxis corner = filletAxis(cube, identity, {ring[0], ring[1]},
                                           Vec3{0, 0, 10});
        check(corner.direction.z > 0.6 && corner.direction.z < 0.98,
              "two adjacent edges lean off the face normal");
        std::printf("  one edge %.2f,%.2f,%.2f   face ring %.2f,%.2f,%.2f\n",
                    axis.direction.x, axis.direction.y, axis.direction.z,
                    wholeFace.direction.x, wholeFace.direction.y, wholeFace.direction.z);
    }

    std::printf("--- a bounded drag lays its whole range on one track ---\n");
    {
        // The complaint this answers: the same gesture has to feel the same
        // whether the range is two millimetres on a thin wall or two hundred on
        // a plate, and whether the camera is close or far. So the range is
        // mapped onto a track of one fixed length in pixels.
        const Camera cam = lookingDown(200.0f);
        DragAxis axis;
        axis.origin = {0, 0, 0};
        axis.direction = {1, 0, 0};
        axis.baseValue = 0.05;
        axis.spanValue = 1.992;          // the shelled box's real limit
        axis.valid = true;

        Vec2 at{};
        check(cam.projectToPixel(axis.origin, at), "the anchor is on screen");

        check(near(axis.valueAt(cam, at), 0.05, 1e-6), "at the anchor, the minimum");
        check(near(axis.valueAt(cam, at + Vec2{DragAxis::kTrackPx, 0}), 1.992, 1e-6),
              "a track's length along, the maximum");
        check(near(axis.valueAt(cam, at + Vec2{DragAxis::kTrackPx * 0.5f, 0}),
                   0.05 + (1.992 - 0.05) * 0.5, 1e-6),
              "and halfway, halfway");

        check(near(axis.valueAt(cam, at + Vec2{DragAxis::kTrackPx * 3.0f, 0}), 1.992, 1e-6),
              "past the end it stops at the maximum");
        check(near(axis.valueAt(cam, at - Vec2{400.0f, 0}), 0.05, 1e-6),
              "and behind the start it stops at the minimum -- never below it");

        // The same drag, on a part a hundred times the size and a camera a
        // hundred times further away, is the same movement of the hand.
        const Camera far_ = lookingDown(20000.0f);
        DragAxis big = axis;
        big.spanValue = 199.2;
        Vec2 bigAt{};
        check(far_.projectToPixel(big.origin, bigAt), "and on screen there too");
        const Real half = big.valueAt(far_, bigAt + Vec2{DragAxis::kTrackPx * 0.5f, 0});
        check(near(half, 0.05 + (199.2 - 0.05) * 0.5, 1e-4),
              "half a track is half the range, whatever the scale");
        std::printf("  2mm range and 200mm range: half a track gives %.3f and %.3f\n",
                    axis.valueAt(cam, at + Vec2{DragAxis::kTrackPx * 0.5f, 0}), half);
    }

    std::printf("--- the ticks are a readable distance apart, always ---\n");
    {
        const Camera cam = lookingDown(120.0f);
        for (Real reach : {0.4, 1.94, 12.0, 250.0}) {
            const Real step = DragAxis::stepFor(cam, {0, 0, 0}, reach);
            const Real spacingPx = step / reach * DragAxis::kTrackPx;
            check(spacingPx >= 9.0 && spacingPx <= 32.0,
                  "ticks " + std::to_string(spacingPx) + "px apart for a range of " +
                  std::to_string(reach));
            const Real m = step / std::pow(10.0, std::floor(std::log10(step)));
            check(near(m, 1.0, 1e-3) || near(m, 2.5, 1e-3) || near(m, 5.0, 1e-3),
                  "and the step is still a number a person would choose");
        }
        std::printf("  a 1.94mm range steps by %.3f, a 250mm range by %.1f\n",
                    DragAxis::stepFor(cam, {0, 0, 0}, 1.94),
                    DragAxis::stepFor(cam, {0, 0, 0}, 250.0));
    }

    std::printf("--- the step follows the zoom and the travel ---\n");
    {
        // Two things decide it. Zoomed in, the step gets finer because a
        // millimetre is worth more pixels; and a gesture with a long travel
        // takes a coarser step so the road is not a hundred identical ticks.
        const Vec3 at{0, 0, 0};
        const Camera close = lookingDown(40.0f);
        const Camera far_ = lookingDown(600.0f);

        // A bounded drag's step comes from its range and nothing else. The
        // zoom used to decide it, which is why the same operation felt
        // different depending on where the camera happened to be.
        check(near(DragAxis::stepFor(close, at, 2.0), DragAxis::stepFor(far_, at, 2.0), 1e-9),
              "the same range gives the same step at any zoom");

        const Real shortTravel = DragAxis::stepFor(close, at, 2.0);
        const Real longTravel = DragAxis::stepFor(close, at, 400.0);
        check(longTravel > shortTravel, "a longer range gives a coarser step");

        // An unbounded drag has no range to divide, so there the zoom is all
        // there is to go on.
        check(DragAxis::stepFor(close, at, 0.0) < DragAxis::stepFor(far_, at, 0.0),
              "unbounded, closer still gives a finer step");

        // And every one of them is a number a person would choose.
        for (Real reach : {1.0, 7.0, 40.0, 250.0}) {
            for (float d : {30.0f, 120.0f, 800.0f}) {
                const Real s2 = DragAxis::stepFor(lookingDown(d), at, reach);
                const Real m = s2 / std::pow(10.0, std::floor(std::log10(s2)));
                check(near(m, 1.0, 1e-3) || near(m, 2.5, 1e-3) || near(m, 5.0, 1e-3),
                      "the step is 1, 2.5 or 5 times a power of ten");
            }
        }

        // The travel is divided into a readable number of stops rather than
        // two or two hundred -- at every zoom, including one far enough out
        // that ten pixels is worth more than the whole gesture.
        for (Real reach : {1.0, 7.0, 40.0, 250.0}) {
            for (float d : {20.0f, 120.0f, 2000.0f}) {
                const Real s2 = DragAxis::stepFor(lookingDown(d), at, reach);
                const Real stops = reach / s2;
                check(stops >= 3.0 && stops <= 60.0,
                      "the travel is a countable number of steps at " +
                      std::to_string(d) + "mm out: " + std::to_string(stops));
            }
        }
        std::printf("  2mm range steps by %.3f at any zoom; 400mm range by %.1f\n",
                    shortTravel, longTravel);
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
