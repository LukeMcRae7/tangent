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

    // True when the axis is square enough to the view to measure along. Near
    // false the caller should say so rather than let the number jump about.
    bool facingCamera(const Camera& camera) const;

    // The guide: a line through the origin along the axis, an arrow the way
    // that increases, and ticks at the snap increment so the amount of
    // movement a step costs is visible rather than inferred.
    void drawGuide(Renderer& renderer, const Camera& camera, Real value, Real step) const;
};

// The axis a fillet grows along: away from the edge, along the bisector of the
// two faces that meet there. Pulling the cursor off the edge into open space
// increases the radius; sliding along the edge does nothing, which is what
// distance-from-the-edge got wrong.
DragAxis filletAxis(const Body& body, const Mat4& model, EdgeId edge, Vec3 nearPoint);

} // namespace tg
