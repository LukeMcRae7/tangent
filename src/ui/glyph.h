// Tangent - the line icons.
//
// Every icon in the interface is drawn, at the moment it is needed, from a
// handful of strokes in a 24-unit box: no texture, no font, nothing to ship
// and nothing that can be the wrong size on a different display. They are
// the same stroke weight everywhere, so a toolbar, a menu and an outliner row
// read as one family rather than three.
//
// The baked renders in ui/icons.h stay for the one place a picture of the
// actual geometry beats a symbol of it: choosing which shape to start from.
#pragma once

#include "imgui.h"

namespace tg {

enum class Glyph {
    // Files
    New, Open, Save, Settings, Import, Export,
    // Shapes and starts
    Box, Cylinder, Sphere, Cone, Torus, Plane, Sketch,
    // Transforms and combining
    Move, Rotate, Scale, Boolean, Union, Difference, Intersect, NewBody,
    // Modelling
    Extrude, PushPull, Fillet, Chamfer, Shell, Inset, Divide, Merge, Pattern,
    Mirror, Split, Convert, Reduce, RotateFace, ScaleFace,
    // Inspecting
    Measure, Alert, Check, Eye, EyeOff, Grid, Wire, Frame, Camera, Cube,
    // Chrome
    ChevronDown, ChevronRight, ChevronUp, Close, Minimize, Maximize, Restore,
    Undo, Redo, Plus, Dot, Clock, Lock, Trash,
    // Outliner
    Body, Mesh,
    // Sketching
    Select, Line, Rect, Circle, Arc, Dimension,
    Count
};

// Draws a glyph centred on `centre`, `sizePx` across. Stroke weight scales
// with the size; pass one explicitly to override.
void drawGlyph(ImDrawList* dl, Glyph g, ImVec2 centre, float sizePx, ImU32 colour,
               float strokePx = 0.0f);

// A glyph as a layout item, the way ImGui::Text is: takes its space and moves
// the cursor on. Vertically centred on the current frame.
void glyphItem(Glyph g, float sizePx, ImU32 colour);

// A square button showing a glyph. `active` draws it as the chosen one of a
// set; `tint` overrides the glyph colour (zero keeps the text colour).
bool glyphButton(const char* id, Glyph g, float sizePx, const char* tooltip,
                 bool active = false, bool enabled = true, ImU32 tint = 0);

} // namespace tg
