// Tangent - the arrow a drag is measured along, drawn on the screen.
//
// The guide used to be built out of world-space strands and triangles, and it
// never quite sat under the cursor: the track was a fixed length on screen
// but was drawn as a length in the world, so wherever the axis ran into or
// out of the picture the perspective shortened it and the arrow's head lagged
// or led the pointer. A value that is measured on the screen has to be drawn
// on the screen. So this projects the origin and the direction once, and
// draws the whole thing -- track, ticks, arrow, handle, readout -- in pixels,
// where the head lands exactly where the value says it is.
#pragma once

#include "app/camera.h"
#include "app/drag_axis.h"

#include "imgui.h"

namespace tg::ui {

// `viewportOrigin` is where the viewport's pixel (0, 0) sits in the window.
// `limit` is the furthest the value can go, or zero; `label` is the value in
// words, drawn beside the head, or null for none.
void drawDragGuide(const DragAxis& axis, const Camera& camera, ImVec2 viewportOrigin,
                   Real value, Real step, Real limit, const char* label);

} // namespace tg::ui
