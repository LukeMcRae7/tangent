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

#include <cmath>
#include <vector>

namespace tg {

enum class SnapKind {
    None,

    // On a feature: the cursor is sitting on something that exists.
    Vertex,        // a corner
    CircleCentre,  // the middle of a hole or a rounded corner
    ArcQuadrant,   // the four points around a circle: where a tangent lands
    EdgeMidpoint,
    FaceCentre,

    // Off a feature, but lined up with one. These are what a person is doing
    // when they say "level with that hole" -- the point they want has no
    // geometry on it at all, and is exactly as real as the ones that do.
    Alignment,     // in line with one reference, along one axis of the plane
    Intersection,  // where the lines from two references cross

    // Nothing to line up with, so the grid the user can see. Only reported at
    // the levels that are actually drawn -- landing between two visible lines
    // is not a snap, it is just a small step.
    GridPoint,     // on a crossing
    GridLine,      // on one line, free along the other
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

// A sketch plane: where it is and which way its two axes run. Snapping on a
// plane is a different question from snapping in space -- "in line with" only
// means anything once there are two axes to be in line along.
struct PlaneFrame {
    Vec3 origin{};
    Vec3 u{1, 0, 0};
    Vec3 v{0, 1, 0};
    Vec3 normal{0, 0, 1};

    Vec2 toUV(Vec3 world) const {
        const Vec3 rel = world - origin;
        return {dot(rel, u), dot(rel, v)};
    }
    Vec3 toWorld(Vec2 uv) const { return origin + u * uv.x + v * uv.y; }
    Real distanceTo(Vec3 world) const { return std::fabs(dot(world - origin, normal)); }
};

// One thing worth snapping to, found by where it is rather than by what is
// under the cursor.
struct SnapPoint {
    Vec3     point{};              // world
    Vec2     uv{};                 // in the plane's coordinates
    SnapKind kind = SnapKind::None;
    ObjectId object = kNoObject;
    Real     radius = 0.0;
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

// Everything on or near `plane` that could serve as a reference for an
// alignment, gathered from the band of the plane the cursor shares.
//
// An alignment reference is by definition far from the cursor -- that is the
// whole use of it -- so this cannot cull the way findSnap does. What it culls
// on instead is the band: only points that share the cursor's u or its v to
// within `tol` can line up with it, and an object whose extent misses both
// bands is rejected before any of its elements are looked at.
//
// Quadrants are never offered here. There are four on every circle and a part
// has many circles, so they line up with everything and mean nothing; they earn
// their place as targets, not as references.
// Works entirely in the plane's own coordinates: which way the camera happens
// to be looking has no bearing on whether two points line up.
size_t collectSnapPoints(const Scene& scene, const PlaneFrame& plane,
                         Vec2 aroundUV, Real tol, std::vector<SnapPoint>& out,
                         size_t limit = 64, const SnapConfig& config = {});

} // namespace tg
