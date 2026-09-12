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

// A line of a given thickness in pixels, drawn as parallel strands.
//
// Not glLineWidth: a core profile is only required to support a width of one,
// and several drivers give exactly that. Good enough for the track and the
// ticks, which are quiet; the arrow is filled triangles, because at four
// strands a shallow angle shows them separately.
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

void DragAxis::drawGuide(Renderer& renderer, const Camera& camera, Real value,
                         Real step, Real limit) const {
    if (!valid) return;

    // Three weights: ticks behind, quiet enough to read as a scale; the track;
    // and the arrow, filled and in front, which is the part being read.
    const Vec4 ticks = toVec4(palette::kBrand, 0.20f);
    const Vec4 track = toVec4(palette::kBrand, 0.38f);
    const Vec4 arrow = toVec4(palette::kBrand, 1.0f);

    const Real px = static_cast<Real>(camera.pixelWorldSize(origin));
    const bool bounded = spanValue > baseValue;

    // The track is the same length on screen every time. That is the whole
    // point of it: the gesture is the same size for a 2mm wall and a 200mm
    // plate, and the ticks along it are spaced the same in both.
    const Real trackWorld = px * kTrackPx;
    (void)limit;   // the span is the limit; the parameter is kept for callers
                   // whose range is not known until the drag is under way.

    // Where a value sits along the track.
    auto place = [&](Real v) {
        if (!bounded) return std::max(Real(0), v - baseValue);
        const Real t = std::clamp((v - baseValue) / (spanValue - baseValue), Real(0), Real(1));
        return t * trackWorld;
    };

    Vec3 across = cross(direction, normalize(camera.eye() - origin));
    if (lengthSq(across) < 1e-12) across = perpendicular(direction);
    across = normalize(across);

    // Ticks, from the start forward. Nothing behind it: the gesture cannot go
    // there, so a scale there would describe travel that does not exist.
    if (step > 0.0) {
        const Real last = bounded ? spanValue : baseValue + trackWorld;
        const Real first = std::ceil(baseValue / step) * step;
        int drawn = 0;
        for (Real at = first; at <= last + 1e-9 && drawn < 90; at += step, ++drawn) {
            const Vec3 p = origin + direction * place(at);
            const bool major = std::fabs(std::fmod(at / step, 4.0)) < 1e-6;
            const Real len = px * (major ? 5.5 : 3.0);
            renderer.addLine(p - across * len, p + across * len, ticks);
        }
    }

    // The road, and a bar at the end of it. With a limit the end of the track
    // *is* the limit, so the end of the travel is a place on screen rather
    // than something found by pushing into it.
    const Vec3 tip = origin + direction * trackWorld;
    thickLine(renderer, camera, origin, tip, track, 2.0);
    if (bounded) {
        const Real cap = px * 8.0;
        thickLine(renderer, camera, tip - across * cap, tip + across * cap, track, 3.0);
    }

    // The arrow: filled, in the front layer, from the start to the value.
    const Vec3 head = origin + direction * place(value);
    const Real barb = px * 14.0;
    const Real shaftHalf = px * 2.6;
    const Real travelled = length(head - origin);
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
