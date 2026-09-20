// Tangent - the line icons.
//
// Every icon in the interface comes from Tabler Icons (MIT, tabler.io/icons),
// bundled as assets/fonts/tabler-icons.ttf and drawn through ImGui's font
// atlas. One family, one 2-unit stroke on a 24-unit grid, so a toolbar, a menu
// and an outliner row read as one set rather than three -- and since the atlas
// rasterises each size on demand, an icon is as sharp at 12px as at 24.
//
// The baked renders in ui/icons.h stay for the one place a picture of the
// actual geometry beats a symbol of it: choosing which shape to start from.
#pragma once

#include "imgui.h"

#include <string>

namespace tg {

enum class Glyph {
    // Files
    New, Open, Save, Settings, Import, Export,
    // Shapes and starts
    Box, Cylinder, Sphere, Cone, Torus, Plane, Sketch,
    // Transforms and combining
    Move, Rotate, Scale, Boolean, Union, Difference, Intersect, Overlap, NewBody,
    // Modelling
    Extrude, PushPull, Revolve, Fillet, Chamfer, Shell, Inset, Divide, Merge,
    Hole, Counterbore, Countersink,
    Pattern, PatternRow, PatternRing, Mirror, Split, Convert, Reduce, RotateFace, ScaleFace,
    // Inspecting
    Measure, Alert, Check, Eye, EyeOff, Grid, Wire, Backface, Bounds,
    FrameSelected, FrameAll, Orthographic, Orbit,
    // Chrome
    ChevronDown, ChevronRight, ChevronUp, Close, Minimize, Maximize, Restore,
    Undo, Redo, Plus, Duplicate, SelectAll, Dot, Clock, Lock, Trash,
    // Outliner
    Body, Mesh,
    // Sketching
    Select, Line, Rect, Circle, Arc, Dimension,
    Count
};

// Loads the icon font from `fontsDir`. Until it has been, or if it could not
// be, every glyph draws nothing and keeps its space, so layout does not move.
bool loadGlyphFont(const std::string& fontsDir);

// The Tabler codepoint a glyph is drawn with. Exposed for the test that
// checks every one of them is actually in the bundled font.
ImWchar glyphCodepoint(Glyph g);

// Draws a glyph centred on `centre`, filling a `sizePx` square the way a
// Tabler icon fills its 24-unit box.
void drawGlyph(ImDrawList* dl, Glyph g, ImVec2 centre, float sizePx, ImU32 colour);

// A glyph as a layout item, the way ImGui::Text is: takes its space and moves
// the cursor on. Vertically centred on the current frame.
void glyphItem(Glyph g, float sizePx, ImU32 colour);

// A square button showing a glyph. `active` draws it as the chosen one of a
// set; `tint` overrides the glyph colour (zero keeps the text colour).
bool glyphButton(const char* id, Glyph g, float sizePx, const char* tooltip,
                 bool active = false, bool enabled = true, ImU32 tint = 0);

} // namespace tg
