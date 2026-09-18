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

Real DragAxis::offsetPx(const Camera& camera, Vec2 mousePx) const {
    if (!valid) return 0.0;

    Vec2 originPx{};
    if (!camera.projectToPixel(origin, originPx)) return 0.0;

    // The axis as it appears on screen, taken from a sample far enough along it
    // to be measurable -- a one-pixel step would be mostly rounding.
    const Real px = static_cast<Real>(camera.pixelWorldSize(origin));
    Vec2 aheadPx{};
    if (camera.projectToPixel(origin + direction * (px * 60.0), aheadPx)) {
        const Vec2 along = aheadPx - originPx;
        if (lengthSq(along) > 4.0) {
            const Vec2 unit = along / length(along);
            return dot(mousePx - originPx, unit);
        }
    }

    // Nearly end-on, where the axis has no length on screen to measure along.
    // Distance from the anchor is cruder but it still moves in one direction
    // and cannot flip, which is what matters.
    return length(mousePx - originPx);
}

Real DragAxis::valueAt(const Camera& camera, Vec2 mousePx) const {
    if (!valid) return baseValue;

    const Real along = offsetPx(camera, mousePx);

    // Both ways from the start, and no end in either: a track's length of
    // travel is worth `spanValue`, and going further goes further.
    if (signedRange) return (along / kTrackPx) * spanValue;

    // Bounded: the track carries the whole range, so the same movement of the
    // hand always covers it, whatever the part and wherever the camera.
    if (spanValue > baseValue) {
        const Real t = std::clamp(along / kTrackPx, Real(0), Real(1));
        return baseValue + t * (spanValue - baseValue);
    }

    // Unbounded: a pixel is worth what a pixel is worth out there.
    const Real px = static_cast<Real>(camera.pixelWorldSize(origin));
    return baseValue + std::max(Real(0), along) * px;
}

Real DragAxis::stepFor(const Camera& camera, Vec3 at, Real reach) {
    // A bounded drag lays its whole range along a track of a fixed length, so
    // the step is decided by that length and not by the zoom: twelve pixels is
    // the finest spacing a hand can reliably pick one tick from its neighbour,
    // and twelve pixels is the same distance whatever the camera is doing.
    if (reach > 0.0) {
        // Rounded *up*, so the spacing never falls under the nine pixels the
        // eye needs to separate one tick from its neighbour. Rounding to the
        // nearest lands below it about half the time -- a range of 250mm wants
        // 11.25 and gets 10, which is eight pixels apart.
        Real step = niceStepAbove(reach * 9.0 / kTrackPx);
        // And never so coarse that the range is a handful of stops.
        if (step > reach * 0.25) step = niceStepBelow(reach * 0.25);
        return step;
    }

    // Unbounded: fall back to the zoom, where ten pixels is the finest worth
    // offering.
    return niceStep(static_cast<Real>(camera.pixelWorldSize(at)) * 10.0);
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
