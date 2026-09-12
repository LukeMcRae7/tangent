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

    // Three weights. The ticks sit behind everything and are quiet enough that
    // the arrow covers them; the track is the road; the arrow is the thing
    // being read, and it is the one the logo is made of.
    const Vec4 ticks = toVec4(palette::kBrand, 0.22f);
    const Vec4 track = toVec4(palette::kBrand, 0.40f);
    const Vec4 arrow = toVec4(palette::kBrand, 1.0f);

    const Vec3 anchor = origin + direction * startValue;
    const Real px = static_cast<Real>(camera.pixelWorldSize(anchor));

    // How much of the travel to draw. Ahead to the limit when the limit is
    // close enough to show at a readable size, and a fixed length of road when
    // it is further than that -- so the guide is about the same size on screen
    // whatever the part is and however far away it is.
    constexpr Real kShownPx = 180.0;
    const Real toLimit = limit > 0.0 ? limit - startValue : 0.0;
    const bool limitInView = limit > 0.0 && toLimit <= px * kShownPx * 1.25;
    const Real ahead = limitInView ? std::max(toLimit, px * 20.0) : px * kShownPx;
    const Real behind = std::min(startValue, px * 45.0);   // room to pull back

    Vec3 across = cross(direction, normalize(camera.eye() - anchor));
    if (lengthSq(across) < 1e-12) across = perpendicular(direction);
    across = normalize(across);

    // Ticks first, so everything else covers them.
    if (step > 0.0) {
        const Real from = startValue - behind;
        const Real to = startValue + ahead;
        const Real first = std::ceil(from / step) * step;
        int drawn = 0;
        for (Real at = first; at <= to && drawn < 80; at += step, ++drawn) {
            if (at < 0.0) continue;
            const Vec3 p = origin + direction * at;
            // Every fourth is taller, so counting them is possible without
            // reading a number.
            const bool major = std::fabs(std::fmod(at / step, 4.0)) < 1e-6;
            const Real len = px * (major ? 5.5 : 3.0);
            renderer.addLine(p - across * len, p + across * len, ticks);
        }
    }

    // The road.
    const Vec3 tail = anchor - direction * behind;
    const Vec3 tip = anchor + direction * ahead;
    thickLine(renderer, camera, tail, tip, track, 2.0);

    // A bar at the end when the end is the limit: past here the shape will not
    // take it, and that should be a place rather than a surprise.
    if (limitInView) {
        const Real cap = px * 8.0;
        thickLine(renderer, camera, tip - across * cap, tip + across * cap, track, 3.0);
    }

    // The arrow: from where the gesture began to where it is now. This is the
    // part that moves, and the only part that moves.
    const Real travelled = value - startValue;
    if (std::fabs(travelled) > px * 2.0) {
        const Vec3 head = origin + direction * value;
        const Vec3 forward = travelled > 0.0 ? direction : -direction;
        thickLine(renderer, camera, anchor, head, arrow, 4.0);

        const Real barb = px * 13.0;
        thickLine(renderer, camera, head, head - forward * barb + across * (barb * 0.42),
                  arrow, 4.0);
        thickLine(renderer, camera, head, head - forward * barb - across * (barb * 0.42),
                  arrow, 4.0);
    } else {
        // Too short to be an arrow yet: a bar, so there is still something at
        // the value.
        const Vec3 head = origin + direction * value;
        const Real mark = px * 9.0;
        thickLine(renderer, camera, head - across * mark, head + across * mark, arrow, 4.0);
    }
}

DragAxis filletAxis(const Body& body, const Mat4& model, EdgeId edge, Vec3 nearPoint) {
    DragAxis axis;
    if (!body.hasEdge(edge)) return axis;

    // Anchored where the cursor is nearest the edge rather than at its middle:
    // on a long edge the guide should appear under the pointer, not a hand's
    // width away from it.
    Vec3 aL, bL;
    body.edgePositions(edge, aL, bL);
    const Vec3 aW = transformPoint(model, aL);
    const Vec3 bW = transformPoint(model, bL);
    const Vec3 along = bW - aW;
    const Real len2 = lengthSq(along);
    const Real t = len2 > 1e-12 ? clampf(dot(nearPoint - aW, along) / len2, 0.0, 1.0) : 0.0;
    axis.origin = aW + along * t;

    // Outward along the bisector of the two faces. A fillet eats into both of
    // them, so the direction that gives the user room is the one pointing out
    // of the corner between them.
    FaceId f0 = kNoFace, f1 = kNoFace;
    body.edgeFaces(edge, f0, f1);
    Vec3 outward{};
    const Mat4 nrm = normalMatrix(model);
    if (f0 != kNoFace) outward += normalize(transformVector(nrm, body.faceNormal(f0)));
    if (f1 != kNoFace) outward += normalize(transformVector(nrm, body.faceNormal(f1)));

    if (lengthSq(outward) < 1e-12) {
        // A boundary edge, or two faces exactly opposed. Anything square to the
        // edge is better than nothing, and the guide will show what was chosen.
        outward = perpendicular(normalize(along));
    }
    axis.direction = normalize(outward);
    axis.valid = true;
    return axis;
}

} // namespace tg
