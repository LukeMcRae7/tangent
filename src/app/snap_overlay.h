// Tangent - showing what the cursor has caught.
//
// A snap nobody can see is indistinguishable from the tool being imprecise --
// and, worse, from a snap to the wrong thing. So every snap says two things:
// what kind of point it is, by its shape, and where it was inferred from, by a
// dotted line back to the reference.
//
// The shapes are the ones CAD has used for thirty years, because a person
// coming from Fusion or AutoCAD already reads them:
//
//   square      a corner                    circle      the centre of something round
//   diamond     a quadrant of an arc        triangle    the middle of an edge
//   bar         in line with a reference    cross       where two references meet
//   plus        a grid line
//
// Everything is built from the eye's own axes and sized in pixels, so a glyph
// is the same shape and the same size whatever the plane is doing -- the ring
// this replaced collapsed to a line whenever the sketch plane turned edge-on.
#pragma once

#include "app/camera.h"
#include "app/plane_snap.h"
#include "render/renderer.h"

namespace tg {

struct SnapOverlayStyle {
    Real glyphPx   = 6.5;    // half-width of a feature glyph
    Real strokePx  = 2.0;
    Real dashPx    = 5.0;
    Real gapPx     = 4.5;
    Real dashWidthPx = 1.5;

    // Reference lines are reported, not drawn attention to: they must not
    // compete with the thing being built, and they must not read through an
    // arrow or a profile edge laid over them.
    Real dashAlpha = 0.5;
};

void drawSnapIndicator(Renderer& renderer, const Camera& camera,
                       const PlaneSnap& snap, const SnapOverlayStyle& style = {});

} // namespace tg
