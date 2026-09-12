// Tangent - the direction a drag is measured along, and the line that says so.
//
// A value dragged with the mouse has to answer two questions before it feels
// like a tool rather than a guess: which way do I move to make it bigger, and
// how much does a given movement change it. Distance from a point answers
// neither. It grows in every direction at once, so the user finds the
// direction by experiment, and moving *along* an edge changes the radius as
// much as moving away from it does.
//
// So every drag names an axis: an anchor in the world and a direction that
// increases the value. The value is the cursor projected onto that axis, which
// means movement across it changes nothing and movement along it changes
// exactly what the line shows. And because the axis is a real thing in the
// world, it can be drawn -- which is the whole point. The user should not have
// to discover the direction.
#pragma once

#include "app/camera.h"
#include "render/renderer.h"

namespace tg {

struct DragAxis {
    Vec3 origin;         // where the value is zero
    Vec3 direction;      // unit; the way that increases it

    // The value at the origin, which is where the gesture starts and the
    // smallest it can go. The origin sits under the cursor at the moment the
    // operation began, so the guide appears in the hand rather than out on the
    // edge -- and the value is how far the pointer has been pulled from there.
    //
    // Nothing goes behind it. Projecting a ray onto a line gives a signed
    // answer, and near the edge of the screen -- where a perspective ray is
    // most oblique -- that sign flips and the arrow turns to point the other
    // way. There is nothing behind the start to point at.
    Real baseValue = 0.0;

    bool valid = false;

    // Where the cursor sits along the axis, in millimetres from the origin.
    // Signed, so a drag the other way gives a negative value and a caller that
    // only wants positive ones can clamp and say why.
    //
    // Computed as the point on the axis nearest the cursor's ray, which is the
    // right answer from any camera angle. When the axis points nearly at the
    // eye that point is ill-conditioned -- a pixel of movement swings it
    // wildly -- so there is a screen-space fallback below.
    Real valueAt(const Camera& camera, Vec2 mousePx) const;

    // How far along the axis the cursor is, signed and unclamped. valueAt is
    // this, clamped forward and offset by baseValue.
    Real rawOffset(const Camera& camera, Vec2 mousePx) const;

    // True when the axis is square enough to the view to measure along. Near
    // false the caller should say so rather than let the number jump about.
    bool facingCamera(const Camera& camera) const;

    // The guide. It does not move: the anchor and the direction are fixed when
    // the gesture starts, and only the marker inside travels. A line that slid
    // about under the cursor would be a second thing to track rather than a
    // thing to aim along.
    //
    // `limit` is how far the value can go -- the largest fillet that will
    // build, say -- so the track has an end and approaching it is visible
    // rather than felt. Zero means unbounded.
    void drawGuide(Renderer& renderer, const Camera& camera, Real value, Real step,
                   Real limit = 0.0) const;

    // The increment for a gesture that can travel `reach` millimetres, at this
    // zoom. Tied to both: a step fine enough to be worth having on screen, and
    // coarse enough that the travel is not a hundred indistinguishable ticks.
    // Always a number a person would choose -- 0.25, 0.5, 1, 2.5.
    static Real stepFor(const Camera& camera, Vec3 at, Real reach);
};

// The axis a fillet grows along: away from the edge, along the bisector of the
// two faces that meet there. Pulling the cursor off the edge into open space
// increases the radius; sliding along the edge does nothing, which is what
// distance-from-the-edge got wrong.
// The axis a fillet grows along, for a whole selection rather than one edge.
//
// The direction is the sum of the outward normals of every face that meets the
// selected edges, which gives the right answer in each case without any of
// them being special: one edge of a cube sums its two faces and comes out at 45
// degrees; the four edges around a face sum that face four times and its four
// sides, which cancel in pairs, leaving the face's own normal; two
// perpendicular faces meeting at an edge come out at 45 again. It is a
// statement about the boundary, and the boundary is what a fillet eats into.
DragAxis filletAxis(const Body& body, const Mat4& model,
                    const std::vector<EdgeId>& edges, Vec3 nearPoint);

} // namespace tg
