// Tangent - how finely a body is drawn, and when that has to change.
//
// An exact body has no resolution of its own: a cylinder is a cylinder, and how
// round it looks is decided when it is turned into triangles. Zoom in far
// enough on a mesh and you eventually see the facets it was born with; zoom in
// on an exact body and the only question is whether anything re-tessellated it.
//
// That is the whole of this file: pick a chord tolerance from how big a pixel
// is where the body sits, decide whether the tolerance the body was last drawn
// at is still good enough, and do a bounded amount of the work per frame.
//
// Bounded matters. Re-tessellating is tens of milliseconds on a heavy part (the
// Stage 0 measurements: 60ms for 206 faces), so doing every body that wants it
// in one frame turns a smooth zoom into a series of stalls. Doing a couple per
// frame spreads the same work across the gesture, where it is invisible.
#pragma once

#include "app/camera.h"
#include "scene/scene.h"

namespace tg {

struct LodPolicy {
    // How far a chord may sit from the surface, in pixels. Half a pixel is
    // below what a display can show, and going finer costs triangles for
    // something nobody can see.
    Real pixelTolerance = 0.5;

    // Re-tessellate when what the view wants is this much finer than what the
    // body has, or this much coarser. The gap between them is hysteresis: a
    // slow zoom would otherwise re-mesh on every frame, each time for a
    // difference too small to see. Coarser is the looser threshold because
    // being too fine only costs triangles, while being too coarse is visible.
    Real finerFactor = 1.6;
    Real coarserFactor = 4.0;

    // Bodies re-tessellated per frame. One is enough to keep up with a zoom on
    // an ordinary scene and cheap enough not to be felt on a heavy one.
    int budgetPerFrame = 2;

    // Nothing finer than this, whatever the zoom: past it the triangles are
    // smaller than the printer's resolution and the only thing that grows is
    // the frame time.
    Real minDeviationMm = 0.002;
    Real maxDeviationMm = 2.0;
};

// The chord tolerance this body should be drawn at, given where it is and how
// the camera is looking at it. In the body's own space, so an object scaled up
// is tessellated finer -- its details are bigger on screen.
Real targetDeviation(const SceneObject& obj, const Camera& camera, const LodPolicy& p = {});

// Is what it has good enough for what the view wants?
bool needsRetessellation(Real current, Real target, const LodPolicy& p = {});

// Brings up to `budgetPerFrame` visible bodies to the tolerance the view wants,
// worst offender first. Returns how many were redrawn.
//
// Mesh bodies are skipped: their resolution was decided when they were made,
// and pretending otherwise would spend the budget achieving nothing.
int refreshTessellation(Scene& scene, const Camera& camera, const LodPolicy& p = {});

} // namespace tg
