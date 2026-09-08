// Tangent - snapping to the geometry, rather than to whatever was nearby.
//
// The mesh kernel could only offer two things to snap to: a vertex, and the
// grid. Everything a person actually wants -- the centre of a hole, the middle
// of an edge, the quadrant of an arc -- either did not exist as a point or
// existed only as an accident of how many segments the cylinder had.
//
// On an exact body those are facts about the geometry, so they can be asked for
// rather than guessed at. This is the largest single ease-of-use gap against
// Fusion and it is the part of the migration a user feels immediately.
//
// Cost is kept to what the cursor is actually near: objects whose projected
// bounds do not contain the cursor are rejected before any of their elements
// are looked at, which is what keeps this affordable on a scene rather than on
// a part.
#pragma once

#include "app/camera.h"
#include "scene/scene.h"

namespace tg {

enum class SnapKind {
    None,
    Vertex,        // a corner
    CircleCentre,  // the middle of a hole or a rounded corner
    ArcQuadrant,   // the four points around a circle: where a tangent lands
    EdgeMidpoint,
    FaceCentre,
};

const char* snapKindName(SnapKind k);

struct SnapHit {
    SnapKind kind = SnapKind::None;
    Vec3     point;                  // world space
    ObjectId object = kNoObject;
    Real     screenDistancePx = 0.0;

    // What it was snapped to, for the two callers that care: a circle knows
    // its radius, so a tool can offer the diameter as a dimension.
    Real     radius = 0.0;

    bool valid() const { return kind != SnapKind::None; }
};

struct SnapConfig {
    // How close the cursor has to be, in pixels. Generous enough to catch
    // without aiming, tight enough that two features 20 pixels apart are still
    // distinguishable.
    Real radiusPx = 14.0;

    bool vertices    = true;
    bool circles     = true;   // centres and quadrants
    bool midpoints   = true;
    bool faceCentres = true;

    // Snapping to something behind what you are looking at is worse than not
    // snapping: an element is only offered if it is within this much of the
    // nearest surface along the ray, in world units, or nothing was hit.
    Real depthToleranceMm = 1.0;
};

// The best thing to snap to under the cursor, or a hit of kind None.
//
// Ranked by kind before distance. A hole's centre a few pixels further away
// than an edge midpoint is still what was meant -- ranking purely by distance
// makes the useful snaps unreachable next to the common ones.
SnapHit findSnap(const Scene& scene, const Camera& camera, Vec2 mousePx,
                 const SnapConfig& config = {});

} // namespace tg
