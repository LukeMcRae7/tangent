#include "app/drag_axis.h"

#include "core/palette.h"

#include <algorithm>
#include <cmath>

namespace tg {

namespace {

// How square to the view an axis has to be before measuring along it is
// trustworthy. At 12 degrees a pixel of cursor movement is already worth five
// times what it is worth head-on, and it gets worse fast.
constexpr Real kMinViewAngleCos = 0.975;   // ~12 degrees off perpendicular

}  // namespace

bool DragAxis::facingCamera(const Camera& camera) const {
    if (!valid) return false;
    const Vec3 toEye = normalize(camera.eye() - origin);
    return std::fabs(dot(toEye, direction)) < kMinViewAngleCos;
}

Real DragAxis::valueAt(const Camera& camera, Vec2 mousePx) const {
    if (!valid) return baseValue;
    // Forward only. Behind the start there is nothing to measure, and letting
    // the number go negative is what made the arrow invert near the edge of
    // the screen, where an oblique ray puts the nearest point on the wrong
    // side of the anchor.
    return baseValue + std::max(Real(0), rawOffset(camera, mousePx));
}

Real DragAxis::rawOffset(const Camera& camera, Vec2 mousePx) const {
    if (!valid) return 0.0;
    const Ray ray = camera.rayThroughPixel(static_cast<float>(mousePx.x),
                                           static_cast<float>(mousePx.y));

    // Closest point between the axis line and the cursor's ray. The usual
    // two-line formulation, guarded for the parallel case.
    const Vec3 w0 = origin - ray.origin;
    const Real a = dot(direction, direction);          // 1
    const Real b = dot(direction, ray.dir);
    const Real c = dot(ray.dir, ray.dir);              // 1
    const Real d = dot(direction, w0);
    const Real e = dot(ray.dir, w0);
    const Real denom = a * c - b * b;

    if (std::fabs(denom) > 1e-9) return (b * e - c * d) / denom;

    // Looking straight down the axis: the lines are parallel and there is no
    // nearest point. Fall back to how far the cursor is from the origin on
    // screen, in the axis's own units, which at least keeps moving.
    Vec2 originPx{};
    if (!camera.projectToPixel(origin, originPx)) return 0.0;
    return length(mousePx - originPx) * static_cast<Real>(camera.pixelWorldSize(origin));
}

namespace {

// A line of a given thickness in pixels, drawn as parallel strands.
//
// Not glLineWidth: a core profile is only required to support a width of one,
// and several drivers give exactly that. Strands cost a few more vertices in a
// batch that already holds hundreds.
void thickLine(Renderer& renderer, const Camera& camera, Vec3 a, Vec3 b, Vec4 color,
               Real widthPx) {
    Vec3 along = b - a;
    if (lengthSq(along) < 1e-18) return;
    along = normalize(along);

    Vec3 across = cross(along, normalize(camera.eye() - a));
    if (lengthSq(across) < 1e-12) across = perpendicular(along);
    across = normalize(across);

    const Real px = static_cast<Real>(camera.pixelWorldSize(a));
    const int strands = std::max(1, static_cast<int>(std::lround(widthPx)));
    for (int i = 0; i < strands; ++i) {
        const Real offset = (static_cast<Real>(i) - (strands - 1) * 0.5) * px * 0.8;
        renderer.addLine(a + across * offset, b + across * offset, color);
    }
}

}  // namespace

Real DragAxis::stepFor(const Camera& camera, Vec3 at, Real reach) {
    const Real px = static_cast<Real>(camera.pixelWorldSize(at));
    // Ten pixels is the finest worth offering -- below that the hand cannot
    // pick one tick over its neighbour. A thirtieth of the travel is the
    // coarsest -- above that the whole range is a handful of stops. Whichever
    // is larger, rounded to a number a person would choose.
    const Real byZoom = px * 10.0;
    const Real byReach = reach > 0.0 ? reach / 30.0 : 0.0;
    Real step = niceStep(std::max(byZoom, byReach));

    // And never coarser than a quarter of the travel. Zoomed far enough out,
    // ten pixels is worth more than the whole gesture, and the step would be
    // the only stop on the road -- which is a slider with one position.
    if (reach > 0.0 && step > reach * 0.25) step = niceStepBelow(reach * 0.25);
    return step;
}

void DragAxis::drawGuide(Renderer& renderer, const Camera& camera, Real value,
                         Real step, Real limit) const {
    if (!valid) return;

    // Three weights. The ticks sit behind, quiet enough to read as a scale
    // rather than as content; the track is the road; the arrow is the thing
    // being read, and it is drawn last, filled, over everything.
    const Vec4 ticks = toVec4(palette::kBrand, 0.20f);
    const Vec4 track = toVec4(palette::kBrand, 0.38f);
    const Vec4 arrow = toVec4(palette::kBrand, 1.0f);

    const Real px = static_cast<Real>(camera.pixelWorldSize(origin));

    // How much of the travel to draw. Ahead to the limit when the limit is
    // close enough to show at a readable size, and a fixed length of road when
    // it is further -- so the guide is about the same size on screen whatever
    // the part is and wherever the camera is.
    constexpr Real kShownPx = 180.0;
    const Real toLimit = limit > 0.0 ? limit - baseValue : 0.0;
    const bool limitInView = limit > 0.0 && toLimit <= px * kShownPx * 1.25;
    const Real ahead = limitInView ? std::max(toLimit, px * 24.0) : px * kShownPx;

    Vec3 across = cross(direction, normalize(camera.eye() - origin));
    if (lengthSq(across) < 1e-12) across = perpendicular(direction);
    across = normalize(across);

    // Ticks, from the start forward. Nothing behind it: the gesture cannot go
    // there, so a scale there would be describing travel that does not exist.
    if (step > 0.0) {
        const Real first = std::ceil(baseValue / step) * step;
        int drawn = 0;
        for (Real at = first; at <= baseValue + ahead && drawn < 80; at += step, ++drawn) {
            const Vec3 p = origin + direction * (at - baseValue);
            const bool major = std::fabs(std::fmod(at / step, 4.0)) < 1e-6;
            const Real len = px * (major ? 5.5 : 3.0);
            renderer.addLine(p - across * len, p + across * len, ticks);
        }
    }

    // The road.
    const Vec3 tip = origin + direction * ahead;
    thickLine(renderer, camera, origin, tip, track, 2.0);

    // A bar at the end when the end is the limit: past here the shape will not
    // take it, and that should be a place rather than a surprise.
    if (limitInView) {
        const Real cap = px * 8.0;
        thickLine(renderer, camera, tip - across * cap, tip + across * cap, track, 3.0);
    }

    // The arrow. Filled triangles rather than a bundle of parallel lines: at
    // four strands a shallow angle shows them separately and it reads as a
    // frayed rope rather than an arrow. In the front layer, so the ticks
    // cannot stripe through it and the model cannot hide it.
    const Real travelled = std::max(Real(0), value - baseValue);
    const Vec3 head = origin + direction * travelled;

    const Real barb = px * 14.0;
    const Real shaftHalf = px * 2.6;
    const Real shaftEnd = std::max(Real(0), travelled - barb * 0.82);

    if (shaftEnd > 1e-9) {
        const Vec3 a = origin + across * shaftHalf;
        const Vec3 b = origin - across * shaftHalf;
        const Vec3 c = origin + direction * shaftEnd - across * shaftHalf;
        const Vec3 d = origin + direction * shaftEnd + across * shaftHalf;
        renderer.addFrontTriangle(a, b, c, arrow);
        renderer.addFrontTriangle(a, c, d, arrow);
    }

    // The head sits at the value even when the shaft has no room, so there is
    // always something exactly where the number is.
    const Vec3 back = head - direction * barb;
    renderer.addFrontTriangle(head, back + across * (barb * 0.40),
                              back - across * (barb * 0.40), arrow);
}

DragAxis filletAxis(const Body& body, const Mat4& model,
                    const std::vector<EdgeId>& edges, Vec3 nearPoint) {
    DragAxis axis;
    if (edges.empty()) return axis;

    const Mat4 nrm = normalMatrix(model);
    Vec3 outward{};
    Vec3 anyAlong{};
    Vec3 closest{};
    Real closestDist = 1e30;

    for (EdgeId e : edges) {
        if (!body.hasEdge(e)) continue;

        FaceId f0 = kNoFace, f1 = kNoFace;
        body.edgeFaces(e, f0, f1);
        if (f0 != kNoFace) outward += normalize(transformVector(nrm, body.faceNormal(f0)));
        if (f1 != kNoFace) outward += normalize(transformVector(nrm, body.faceNormal(f1)));

        // The point on the selection nearest the cursor, so the guide is
        // anchored to the part of it being looked at.
        Vec3 aL, bL;
        body.edgePositions(e, aL, bL);
        const Vec3 aW = transformPoint(model, aL);
        const Vec3 bW = transformPoint(model, bL);
        const Vec3 along = bW - aW;
        if (lengthSq(along) > 1e-12) anyAlong = along;
        const Real len2 = lengthSq(along);
        const Real t = len2 > 1e-12 ? clampf(dot(nearPoint - aW, along) / len2, 0.0, 1.0) : 0.0;
        const Vec3 at = aW + along * t;
        const Real d = length(at - nearPoint);
        if (d < closestDist) { closestDist = d; closest = at; }
    }

    if (closestDist > 1e29) return axis;
    axis.origin = closest;

    if (lengthSq(outward) < 1e-9) {
        // Normals that cancel exactly -- a selection that wraps a whole body,
        // say. Anything square to an edge beats nothing, and the guide on
        // screen will show what was chosen.
        outward = lengthSq(anyAlong) > 1e-12 ? perpendicular(normalize(anyAlong))
                                             : Vec3{0, 0, 1};
    }
    axis.direction = normalize(outward);
    axis.valid = true;
    return axis;
}

} // namespace tg
