// Tangent - the view cube.
//
// Twenty-six ways to look at the model, in the place every CAD package puts
// them. A cube that turns with the view answers two questions at once -- which
// way am I looking, and how do I get to the view I want -- and it answers the
// second by being clicked rather than by being read.
//
// The zones are the cube's own anatomy: six faces, twelve edges, eight corners.
// A face is a named view; an edge is the 45 degrees between two of them; a
// corner is the isometric. Which one the cursor is over is found by casting a
// ray into the cube and seeing which bands of it the hit lands in, so the
// regions are exactly the ones drawn and there is no separate hit geometry to
// disagree with what is on screen.
#pragma once

#include "ui/panels.h"

namespace tg {

struct ViewCubeStyle {
    float sizePx   = 104.0f;
    float marginPx = 14.0f;

    // How much of each face belongs to its edges and corners rather than to the
    // face itself. A third is the usual split and is what makes the corner of a
    // view cube big enough to hit without aiming.
    float bandFraction = 0.30f;
};

// Draws the cube in the top-right of the viewport rect and applies whatever is
// done with it.
//
// Nothing is returned for the caller to guard on: the widget is a real ImGui
// window, so hovering it already sets WantCaptureMouse, and the viewport's
// picking is behind that test like everything else.
void drawViewCube(UiContext& ctx, float x, float y, float w, float h,
                  const ViewCubeStyle& style = {});

// Which way the cursor would send the camera, or a zero vector if it is not
// over the cube. Exposed for tests: the interesting part of this widget is the
// mapping from a pixel to one of twenty-six directions, and that can be checked
// without a window.
Vec3 viewCubeZoneAt(const Camera& camera, Vec2 offsetFromCentrePx, float sizePx,
                    float bandFraction = 0.30f);

// The name of a zone, as the cube labels it: "Front", "Front Top Right".
std::string viewCubeZoneName(Vec3 zone);

} // namespace tg
