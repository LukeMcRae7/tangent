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

void DragAxis::drawGuide(Renderer& renderer, const Camera& camera, Real value,
                         Real step, Real limit) const {
    if (!valid) return;

    const Vec4 track = toVec4(palette::kBrand, 0.45f);
    const Vec4 lit   = toVec4(palette::kBrand, 1.0f);

    const Real px = static_cast<Real>(camera.pixelWorldSize(origin));

    // The track runs to the limit when there is one, so the end of the travel
    // is a place on screen rather than something discovered by pushing into it.
    const Real reach = limit > 0.0 ? limit : std::max(px * 170.0, std::fabs(value) * 1.4);
    const Real behind = px * 26.0;

    Vec3 across = cross(direction, normalize(camera.eye() - origin));
    if (lengthSq(across) < 1e-12) across = perpendicular(direction);
    across = normalize(across);

    const Vec3 tip = origin + direction * reach;
    thickLine(renderer, camera, origin - direction * behind, tip, track, 2.0);

    // A cap at the end when the end means something.
    if (limit > 0.0) {
        const Real cap = px * 7.0;
        thickLine(renderer, camera, tip - across * cap, tip + across * cap, track, 2.0);
    } else {
        const Real head = px * 9.0;
        thickLine(renderer, camera, tip, tip - direction * head + across * (head * 0.45), lit, 2.0);
        thickLine(renderer, camera, tip, tip - direction * head - across * (head * 0.45), lit, 2.0);
    }

    // Ticks at the snap increment: how far the mouse travels for one step,
    // shown rather than discovered.
    if (step > 0.0) {
        const int most = 60;
        int drawn = 0;
        for (Real at = step; at <= reach && drawn < most; at += step, ++drawn) {
            const Vec3 p = origin + direction * at;
            const Real len = px * ((drawn + 1) % 5 == 0 ? 4.5 : 2.2);
            renderer.addLine(p - across * len, p + across * len, track);
        }
    }

    // The part that moves. Everything above is fixed for the whole gesture, so
    // this is the only thing the eye has to follow: the travelled length, and a
    // bar across the track at the value itself.
    const Real travelled = clampf(value, 0.0, reach);
    if (travelled > 1e-9) {
        thickLine(renderer, camera, origin, origin + direction * travelled, lit, 3.0);
    }
    const Vec3 at = origin + direction * travelled;
    const Real mark = px * 11.0;
    thickLine(renderer, camera, at - across * mark, at + across * mark, lit, 3.0);
    // A short stem either side of the bar, so it reads as a slider rather than
    // as another tick.
    const Real stem = px * 4.0;
    thickLine(renderer, camera, at + across * mark, at + across * mark - direction * stem, lit, 2.0);
    thickLine(renderer, camera, at - across * mark, at - across * mark - direction * stem, lit, 2.0);
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
