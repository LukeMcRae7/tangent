// Tangent - the panels the gestures run in, and the guides they draw.
//
// Every modal operation -- fillet, face move, pattern, divide, reduce -- gets
// the same panel: its name at the top, one row per number, the choices as a
// row of tiles, Finish and Cancel at the foot. What differs between them is
// only which rows there are. The panel and the arrow in the viewport are two
// views of the same value: pull the arrow and the bar fills, pull the bar and
// the arrow follows.
#include "app/application.h"

#include "mesh/import_mesh.h"
#include "ui/command_panel.h"
#include "ui/drag_guide.h"
#include "ui/theme.h"
#include "ui/widgets.h"

#include "core/palette.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace tg {
namespace {

// What a pulled bar means to a tool that reads its value as typed text: the
// number, written the way the keyboard would have written it. Six figures,
// so a bar pulled to 123.45 does not come back as 123.5.
std::string typedFrom(double v) {
    char b[48];
    std::snprintf(b, sizeof b, "%.6g", v);
    return b;
}

// A bar has two lives. While the gesture runs, what it sets is held the way a
// typed number is, so the pointer stops driving the value and the tool's own
// update reads it. Once the operation is applied and the panel is only
// adjusting it, there is no update running to read a typed value: the bar
// writes the value itself, and the panel's signature check sees it change and
// applies the operation again.
template <typename Value, typename Update>
void applyBar(const ui::NumberEdit& e, bool active, std::string& typed, Value setValue,
              Update update) {
    if (e.dragged) {
        if (active) {
            typed = typedFrom(e.value);
            update();
        } else {
            typed.clear();
            setValue(e.value);
        }
    } else if (e.clicked) {
        // Clicking the bar hands the value back to the pointer.
        typed.clear();
    }
}

const char* objectName(const Scene& scene, ObjectId id) {
    const SceneObject* o = scene.find(id);
    return o ? o->name.c_str() : nullptr;
}

} // namespace

// The fillet's own dialog.
//
// The gesture already says what the radius is -- the arrow, and the number by
// it -- but a gesture cannot say how many edges were caught, or how to commit
// without a keyboard. An operation with parameters gets a panel.
void Application::drawFilletPanel() {
    const bool settled = settledIs(Settled::Fillet);
    if (!filletTool_.active && !settled) return;

    auto signature = [&] {
        char b[96];
        std::snprintf(b, sizeof b, "%d|%.9g|%.9g", (int)filletTool_.chamfer,
                      filletTool_.currentRadius, filletTool_.endRadius);
        return std::string(b);
    };
    const std::string was = settled ? signature() : std::string();

    if (!ui::beginCommand("##fillet", filletTool_.chamfer ? "Chamfer" : "Fillet",
                          filletTool_.chamfer ? Glyph::Chamfer : Glyph::Fillet,
                          objectName(scene_, filletTool_.objectId)))
        return;

    // The bar runs from the smallest fillet the selection takes to the largest
    // that has been verified to build. Until the search has found that, the
    // most the body could possibly hold stands in, and the arrow's cap says
    // where the shape actually gives up.
    const double lo = std::max(filletTool_.axis.valid ? filletTool_.axis.baseValue : 0.05, 0.05);
    const double hi = filletTool_.maxRadius > lo ? filletTool_.maxRadius
                    : filletTool_.axis.valid    ? std::max(filletTool_.axis.spanValue, lo * 2.0)
                                                : lo * 2.0;
    const ui::NumberEdit r = ui::commandNumber(
        filletTool_.chamfer ? "Distance" : "Radius", filletTool_.currentRadius, "mm",
        !filletTool_.typedValue.empty(), !filletTool_.typedValue.empty(),
        filletTool_.typedValue.c_str(), lo, hi);
    applyBar(r, filletTool_.active, filletTool_.typedValue,
             [&](double v) {
                 filletTool_.currentRadius = std::clamp(v, lo, hi);
             },
             [&] { updateFillet(true); });

    char edges[64];
    std::snprintf(edges, sizeof edges, "%zu edge%s", filletTool_.edges.size(),
                  filletTool_.edges.size() == 1 ? "" : "s");
    ui::commandValue("Selection", edges);

    // While the limit is still being found there is no honest number to show
    // for it, and a maximum that grows as you read it is worse than none.
    if (!filletTool_.search.active && filletTool_.maxRadius > 0.0) {
        char limit[48];
        std::snprintf(limit, sizeof limit, "%.2f mm", filletTool_.maxRadius);
        ui::commandValue("Largest", limit);
    }

    // A flat cut or a round, and whether the round holds its size along the
    // edge. Both change what gets built, so both re-plan and re-preview.
    {
        static const ui::Choice kCut[2] = {
            {Glyph::Fillet,  "Round", "R", "A rounded edge  (R)"},
            {Glyph::Chamfer, "Flat",  "C", "A flat cut, the same from both faces  (C)"},
        };
        const int pick = ui::commandChoices("Edge", kCut, 2, filletTool_.chamfer ? 1 : 0);
        if (pick == 0 && filletTool_.chamfer) {
            filletTool_.chamfer = false;
            filletTool_.requestedRadius = -1.0;
            filletTool_.previewValid = false;
        } else if (pick == 1 && !filletTool_.chamfer) {
            filletTool_.chamfer = true;
            filletTool_.endRadius = -1.0;         // a flat cut does not taper
            filletTool_.requestedRadius = -1.0;
            filletTool_.previewValid = false;
        }
    }

    if (!filletTool_.chamfer) {
        const bool tapering = filletTool_.endRadius > 0.0;
        ui::commandRow("Taper");
        if (ui::pillButton(tapering ? "Even" : "Taper to...", tapering)) {
            filletTool_.endRadius = tapering ? -1.0
                                             : std::max(filletTool_.currentRadius * 0.25, 0.1);
            filletTool_.requestedRadius = -1.0;
            filletTool_.previewValid = false;
        }
        if (tapering) {
            ui::commandRow("Ends at");
            ImGui::SetNextItemWidth(-1.0f);
            if (ImGui::DragScalar("##end", ImGuiDataType_Double, &filletTool_.endRadius,
                                  0.05f, nullptr, nullptr, "%.2f mm")) {
                filletTool_.endRadius = std::max(filletTool_.endRadius, Real(0.05));
                filletTool_.requestedRadius = -1.0;
                filletTool_.previewValid = false;
            }
        }
    }

    if (settled) ui::commandApplied(filletTool_.chamfer ? "Chamfer" : "Fillet");
    ui::commandHint(filletTool_.chamfer
        ? "Pull along the arrow, drag the bar, or type a distance."
        : "Pull along the arrow, drag the bar, or type a radius.");

    const int footer = settled ? ui::commandFooter("Done", true, nullptr)
                               : ui::commandFooter("Finish");
    ui::endCommand();

    if (settled) {
        if (footer > 0)              dismissSettled();
        else if (signature() != was) recommitSettled();
        return;
    }
    if (footer > 0)      commitFillet();
    else if (footer < 0) abortFillet();
}

void Application::drawFacePanel() {
    const bool settled = settledIs(Settled::Face);
    if (!faceTool_.active && !settled) return;

    auto signature = [&] {
        char b[128];
        std::snprintf(b, sizeof b, "%d|%.9g|%.9g|%.9g|%.9g|%d|%d|%d", (int)faceTool_.op,
                      faceTool_.value, faceTool_.direction.x,
                      faceTool_.direction.y, faceTool_.direction.z, (int)faceTool_.choice.op,
                      (int)faceTool_.choice.automatic, (int)faceTool_.combineWithMeet);
        std::string sig = b;
        for (const ReachedBody& r : faceTool_.reach.bodies())
            sig += "|" + std::to_string(r.id) + (r.included ? "+" : "-");
        return sig;
    };
    const std::string was = settled ? signature() : std::string();

    const bool rotate  = faceTool_.op == FaceOp::Rotate;
    const bool scale   = faceTool_.op == FaceOp::Scale;
    const bool extrude = faceTool_.op == FaceOp::Extrude;

    if (!ui::beginCommand("##faceop",
                          rotate ? "Rotate Face" : scale ? "Scale Face"
                                 : extrude ? "Extrude" : "Push / Pull",
                          rotate ? Glyph::RotateFace : scale ? Glyph::ScaleFace
                                 : extrude ? Glyph::Extrude : Glyph::PushPull,
                          objectName(scene_, faceTool_.objectId)))
        return;

    // Either way from nothing, and the bar fills from zero. The range is what
    // each operation can sensibly be asked for: a move or an extrude as far
    // again as the body is thick in each direction, a rotation to a right
    // angle each way (the kernel gives out well before that on most shapes,
    // and the travel stops where it did), and a scale from nearly nothing to
    // four times the size. Wherever the kernel has already refused, the
    // travel stops there instead.
    double lo = 0.0, hi = 0.0;
    if (rotate) {
        lo = -90.0; hi = 90.0;
    } else if (scale) {
        lo = -95.0; hi = 300.0;
    } else {
        const double span = faceTool_.axis.valid && faceTool_.axis.spanValue > 0.0
                                ? faceTool_.axis.spanValue : 10.0;
        lo = -span * 2.0; hi = span * 2.0;
    }
    lo = std::max(lo, static_cast<double>(faceTool_.reachedMin));
    hi = std::min(hi, static_cast<double>(faceTool_.reachedMax));
    // The bar has to hold what has been asked for, however it got there.
    lo = std::min(lo, static_cast<double>(faceTool_.value));
    hi = std::max(hi, static_cast<double>(faceTool_.value));
    const ui::NumberEdit v = ui::commandNumber(
        rotate ? "Angle" : scale ? "Change" : "Distance", faceTool_.value,
        rotate ? "\xC2\xB0" : scale ? "%" : "mm",
        !faceTool_.typedValue.empty(), !faceTool_.typedValue.empty(),
        faceTool_.typedValue.c_str(), lo, hi, /*signedRange=*/true);
    applyBar(v, faceTool_.active, faceTool_.typedValue,
             [&](double x) { faceTool_.value = clampf(x, faceTool_.reachedMin, faceTool_.reachedMax); },
             [&] { updateFaceMove(true); });

    // The multiple the percentage comes to, since that is the number a person
    // thinks in when they say "half again as big".
    if (scale) {
        char mult[32];
        std::snprintf(mult, sizeof mult, "%.3g x", 1.0 + faceTool_.value / 100.0);
        ui::commandValue("Size", mult);
    }

    char sel[64];
    std::snprintf(sel, sizeof sel, "%zu face%s", faceTool_.faces.size(),
                  faceTool_.faces.size() == 1 ? "" : "s");
    ui::commandValue("Selection", sel);

    // Which way it goes. The face's own normal unless an axis key says
    // otherwise -- for a rotate, which way round it turns. A scale goes every
    // way at once, so there is nothing to point.
    if (!scale) {
        static const ui::Choice kAlong[4] = {
            {Glyph::Count, "Normal", nullptr, "Straight out of the face"},
            {Glyph::Count, "X", "X", "Along the world X axis"},
            {Glyph::Count, "Y", "Y", "Along the world Y axis"},
            {Glyph::Count, "Z", "Z", "Along the world Z axis"},
        };
        static const ui::Choice kPivot[4] = {
            {Glyph::Count, "Edge", nullptr, "About the edge nearest the pointer"},
            {Glyph::Count, "X", "X", "About the world X axis"},
            {Glyph::Count, "Y", "Y", "About the world Y axis"},
            {Glyph::Count, "Z", "Z", "About the world Z axis"},
        };
        const int pick = ui::commandChoices(rotate ? "Pivot" : "Along", rotate ? kPivot : kAlong, 4,
                                            faceTool_.lockedAxis + 1, /*compact=*/true);
        if (pick >= 0 && pick - 1 != faceTool_.lockedAxis) setFaceAxis(pick - 1);
    }

    // An extrusion's operation, and the bodies it reaches -- the same two
    // questions the create tool and a sketch ask. See app/extrude_ops.h.
    if (extrude) {
        if (drawExtrudeChoice(faceTool_.choice)) {
            faceTool_.previewValid = false;
            if (faceTool_.active) refreshFaceReach();
        }
        const ObjectId toggled = drawReachedBodies(scene_, faceTool_.reach, faceTool_.choice.op,
                                                   faceTool_.objectId);
        if (toggled != kNoObject) {
            faceTool_.reach.toggle(toggled);
            faceTool_.previewValid = false;
        }
    }

    // What the number is doing to the body, said plainly. Not a choice: moving
    // a face out adds material and moving it in takes some away.
    if (!rotate && !scale && !extrude) {
        ui::commandRow("Result");
        ImGui::AlignTextToFramePadding();
        const bool cutting = faceTool_.value < 0.0;
        if (std::fabs(faceTool_.value) < 1e-6)
            ImGui::TextColored(ui::im(palette::kTextDim), "unchanged");
        else
            ImGui::TextColored(cutting ? ui::im(palette::kBrand) : ui::im(palette::kValid),
                               "%s", cutting ? "takes material away" : "adds material");
    }

    // Only when it has actually run into something. Until the material meets
    // another body there is no decision to make.
    if (!rotate && !extrude && faceTool_.meets != kNoObject) {
        const SceneObject* other = scene_.find(faceTool_.meets);
        char meets[96];
        std::snprintf(meets, sizeof meets, "Meets %s", other ? other->name.c_str() : "another body");
        static const ui::Choice kMeet[3] = {
            {Glyph::Overlap,    "Leave", nullptr, "Two bodies that overlap, left as they are"},
            {Glyph::Union,      "Join",  nullptr, "Join them into one"},
            {Glyph::Difference, "Cut",   nullptr, "Cut this one out of the other"},
        };
        const int on = !faceTool_.combineWithMeet ? 0
                     : faceTool_.meetOp == BooleanOp::Union ? 1 : 2;
        const int pick = ui::commandChoices(meets, kMeet, 3, on);
        if (pick == 0) faceTool_.combineWithMeet = false;
        if (pick == 1) { faceTool_.combineWithMeet = true; faceTool_.meetOp = BooleanOp::Union; }
        if (pick == 2) { faceTool_.combineWithMeet = true; faceTool_.meetOp = BooleanOp::Difference; }
    }

    ui::commandHint(scale
        ? "Pull out from the middle of the face to grow it, in to shrink it. The faces around it slant to follow."
        : rotate
        ? "Pull either way across the pivot, or type an angle. X / Y / Z choose which way it turns."
        : extrude
        ? "Grows off the face and keeps its outline. Out joins, in cuts, until you pick. Click a body to leave it out."
        : "Moves the face; the body follows. X / Y / Z move it along a world axis instead of its own.");

    if (settled)
        ui::commandApplied(rotate ? "Rotation" : scale ? "Scale" : extrude ? "Extrude" : "Move");

    const int footer = settled ? ui::commandFooter("Done", true, nullptr)
                               : ui::commandFooter("Finish");
    ui::endCommand();
    if (settled) {
        if (footer > 0)              dismissSettled();
        else if (signature() != was) recommitSettled();
        return;
    }
    if (footer > 0)      commitFaceMove();
    else if (footer < 0) abortFaceMove();
}

// Combine: the target, the tools, which way, and whether the tools stay.
void Application::drawCombinePanel() {
    CombineToolState& ct = combineTool_;
    const bool settled = settledIs(Settled::Combine);
    if (!ct.active && !settled) return;

    auto signature = [&] {
        std::string sig = std::to_string(static_cast<int>(ct.op)) + (ct.keepTools ? "k" : "u");
        for (ObjectId id : ct.tools) sig += "|" + std::to_string(id);
        return sig;
    };
    const std::string was = settled ? signature() : std::string();

    if (!ui::beginCommand("##combine", "Combine", Glyph::Boolean,
                          ct.targetName.empty() ? nullptr : ct.targetName.c_str()))
        return;

    {
        static const ui::Choice kOps[3] = {
            {Glyph::Union,      "Join",      "J", "The tools become part of the target  (J)"},
            {Glyph::Difference, "Cut",       "D", "The tools are taken out of the target  (D)"},
            {Glyph::Intersect,  "Intersect", "I", "Only what the target shares with every tool  (I)"},
        };
        const int on = ct.op == BooleanOp::Union ? 0 : ct.op == BooleanOp::Difference ? 1 : 2;
        const int pick = ui::commandChoices("Operation", kOps, 3, on);
        if (pick >= 0) {
            ct.op = pick == 0 ? BooleanOp::Union : pick == 1 ? BooleanOp::Difference : BooleanOp::Intersection;
            ct.previewKey.clear();
        }
    }

    // The target: the body that is kept. Any of the bodies can be it.
    ui::commandRow("Target");
    if (ct.target == kNoObject) {
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(ui::im(palette::kTextDim), "click the body to keep");
    } else if (settled) {
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(ct.targetName.c_str());
    } else {
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::BeginCombo("##target", ct.targetName.c_str())) {
            ObjectId chosen = kNoObject;
            ImGui::Selectable(ct.targetName.c_str(), true);
            for (ObjectId id : ct.tools)
                if (const SceneObject* o = scene_.find(id)) {
                    ImGui::PushID(static_cast<int>(id));
                    if (ImGui::Selectable(o->name.c_str(), false)) chosen = id;
                    ImGui::PopID();
                }
            ImGui::EndCombo();
            if (chosen != kNoObject) setCombineTarget(chosen);
        }
        ui::hoverTip("The body that is kept. Pick another to swap it with a tool.");
    }

    // The tools, lit. Clicking one takes it out; clicking a body in the view
    // puts it in.
    ui::commandRow("Tools");
    if (ct.tools.empty()) {
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(ui::im(palette::kTextDim), "click bodies in the view to add them");
    } else {
        const float right = ImGui::GetWindowContentRegionMax().x;
        ObjectId dropped = kNoObject;
        for (size_t i = 0; i < ct.tools.size(); ++i) {
            const SceneObject* o = scene_.find(ct.tools[i]);
            const std::string name = o ? o->name : i < ct.toolNames.size() ? ct.toolNames[i] : "a body";
            const float w = ImGui::CalcTextSize(name.c_str()).x + 24.0f;
            if (i) {
                ImGui::SameLine(0.0f, 4.0f);
                if (ImGui::GetCursorPosX() + w > right) {
                    ImGui::NewLine();
                    ImGui::SetCursorPosX(ui::commandLabelWidth());
                }
            }
            ImGui::PushID(static_cast<int>(ct.tools[i]));
            if (ui::pillButton(name.c_str(), true)) dropped = ct.tools[i];
            ui::hoverTip("A tool. Click to take it out.");
            ImGui::PopID();
        }
        if (dropped != kNoObject) {
            if (settled) {
                ct.tools.erase(std::find(ct.tools.begin(), ct.tools.end(), dropped));
            } else {
                toggleCombineBody(dropped);
            }
        }
    }

    ui::commandRow("Keep tools");
    if (ui::pillButton(ct.keepTools ? "Kept" : "Used up", ct.keepTools)) ct.keepTools = !ct.keepTools;
    ui::hoverTip(ct.keepTools ? "The tools stay in the scene after combining. Click to use them up."
                              : "The tools are used up. Click to keep them in the scene as well.");

    // What it comes to, in a line.
    if (ct.target != kNoObject && !ct.tools.empty()) {
        char text[192];
        const size_t n = ct.tools.size();
        const char* tools = n == 1 ? "1 tool" : "the tools";
        if (ct.op == BooleanOp::Union)
            std::snprintf(text, sizeof text, "%s, with %s joined to it", ct.targetName.c_str(), tools);
        else if (ct.op == BooleanOp::Difference)
            std::snprintf(text, sizeof text, "%s, with %s cut out of it", ct.targetName.c_str(), tools);
        else
            std::snprintf(text, sizeof text, "only what %s shares with %s", ct.targetName.c_str(),
                          n == 1 ? "the tool" : "every tool");
        ui::commandRow("Result");
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(ct.previewError.empty() ? ui::im(palette::kValid) : ui::im(palette::kBrand), "%s",
                           ct.previewError.empty() ? text : ct.previewError.c_str());
    }

    if (settled) ui::commandApplied("Combine");
    ui::commandHint(settled ? "Change the operation, drop a tool, or keep the tools, and it is made again."
                            : "Click bodies in the view to add or remove tools.");

    const int footer = settled ? ui::commandFooter("Done", true, nullptr)
                               : ui::commandFooter("Finish", ct.target != kNoObject && !ct.tools.empty());
    ui::endCommand();
    if (settled) {
        if (footer > 0)              dismissSettled();
        else if (signature() != was) recommitSettled();
        return;
    }
    if (footer > 0)      finishCombine();
    else if (footer < 0) abortCombine();
}

void Application::drawPatternPanel() {
    const bool settled = settledIs(Settled::Pattern);
    if (!patternTool_.active && !settled) return;

    // What the panel is showing, so that a control moved while the operation is
    // already applied re-applies it.
    auto signature = [&] {
        char b[96];
        std::snprintf(b, sizeof b, "%d|%d|%d|%d|%.9g", (int)patternTool_.mode,
                      patternTool_.count, patternTool_.axisIndex,
                      (int)patternTool_.useTool, patternTool_.dragged());
        return std::string(b);
    };
    const std::string was = settled ? signature() : std::string();

    const bool mirror = patternTool_.mode == PatternMode::Mirror;
    const bool ring = patternTool_.mode == PatternMode::Circular;
    if (!ui::beginCommand("##pattern", mirror ? "Mirror" : "Pattern",
                          mirror ? Glyph::Mirror : Glyph::Pattern,
                          objectName(scene_, patternTool_.objectId)))
        return;

    // How the copies are laid out. Three answers to one question.
    {
        static const ui::Choice kLayout[3] = {
            {Glyph::PatternRow,  "Row",    "L", "Copies along a direction  (L)"},
            {Glyph::PatternRing, "Ring",   "C", "Copies around an axis  (C)"},
            {Glyph::Mirror,      "Mirror", "M", "Reflected across a plane  (M)"},
        };
        const int on = patternTool_.mode == PatternMode::Linear ? 0 : ring ? 1 : 2;
        const int pick = ui::commandChoices("Layout", kLayout, 3, on);
        if (pick >= 0 && pick != on) {
            const PatternMode want = pick == 0 ? PatternMode::Linear
                                   : pick == 1 ? PatternMode::Circular : PatternMode::Mirror;
            if (want == PatternMode::Circular) patternTool_.axisIndex = 2;
            setPatternMode(want);
        }
    }

    // The axis, or the plane's normal. Same three keys the transform tools use.
    {
        static const ui::Choice kAxis[3] = {
            {Glyph::Count, "X", nullptr, nullptr}, {Glyph::Count, "Y", nullptr, nullptr},
            {Glyph::Count, "Z", nullptr, nullptr}};
        const int pick = ui::commandChoices(mirror ? "Plane" : "Axis", kAxis, 3,
                                            patternTool_.axisIndex, /*compact=*/true);
        if (pick >= 0 && pick != patternTool_.axisIndex) {
            patternTool_.axisIndex = pick;
            setPatternMode(patternTool_.mode);
        }
    }

    // The same limits updatePattern holds the value to: a spacing is at least
    // a twentieth of a millimetre and runs to twice the body's extent, a turn
    // is a full circle either way, and a mirror plane sits within the body's
    // extent either side of its centre.
    const double span = patternTool_.axis.valid && patternTool_.axis.spanValue > 0.0 && !ring
                            ? patternTool_.axis.spanValue : 20.0;
    auto setPatternValue = [&](double x) {
        if (patternTool_.mode == PatternMode::Linear) x = std::max(x, 0.05);
        if (ring) {
            if (std::fabs(x) < 0.5) x = x < 0.0 ? -0.5 : 0.5;
            x = std::clamp(x, -360.0, 360.0);
        }
        patternTool_.setDragged(x);
        patternTool_.previewValid = false;
    };
    if (mirror) {
        const double reach = std::max(span, std::fabs(patternTool_.offset));
        const ui::NumberEdit v = ui::commandNumber("Plane at", patternTool_.offset, "mm",
                                                   !patternTool_.typedValue.empty(),
                                                   !patternTool_.typedValue.empty(),
                                                   patternTool_.typedValue.c_str(),
                                                   -reach, reach, /*signedRange=*/true);
        applyBar(v, patternTool_.active, patternTool_.typedValue, setPatternValue,
                 [&] { updatePattern(true); });
    } else {
        const double lo = ring ? -360.0 : 0.05;
        const double hi = ring ? 360.0 : std::max(span * 2.0, patternTool_.step);
        const ui::NumberEdit v = ui::commandNumber(ring ? "Turn" : "Spacing", patternTool_.dragged(),
                                                   ring ? "\xC2\xB0" : "mm",
                                                   !patternTool_.typedValue.empty(),
                                                   !patternTool_.typedValue.empty(),
                                                   patternTool_.typedValue.c_str(),
                                                   lo, hi, /*signedRange=*/ring);
        applyBar(v, patternTool_.active, patternTool_.typedValue, setPatternValue,
                 [&] { updatePattern(true); });

        ui::commandRow("Copies");
        // A full turn is what a ring pattern is nearly always for, and working
        // out 360 over the count by hand is not modelling.
        const float turnW = ring ? ImGui::CalcTextSize("Full turn").x + 30.0f : 0.0f;
        ImGui::SetNextItemWidth(-1.0f - turnW);
        int n = patternTool_.count;
        if (ImGui::DragInt("##count", &n, 0.1f, 2, 256, "%d")) {
            patternTool_.count = n < 2 ? 2 : (n > 256 ? 256 : n);
            patternTool_.previewValid = false;
        }
        if (ring) {
            ImGui::SameLine();
            if (ui::pillButton("Full turn", false)) {
                patternTool_.stepAngle = radians(360.0 / std::max(2, patternTool_.count));
                patternTool_.typedValue.clear();
                patternTool_.previewValid = false;   // the next frame rebuilds
            }
        }
    }

    // What is being repeated. A boolean at the end of the chain leaves a tool
    // behind that can be repeated instead of the whole body.
    if (patternTool_.toolAvailable) {
        static const ui::Choice kRepeat[2] = {
            {Glyph::Count, "The body", nullptr, "Copies of the whole body, fused where they meet"},
            {Glyph::Count, "The cut",  nullptr, "The last cut, made again at each copy"},
        };
        const int pick = ui::commandChoices("Repeat", kRepeat, 2, patternTool_.useTool ? 1 : 0, true);
        if (pick >= 0 && (pick == 1) != patternTool_.useTool) {
            patternTool_.useTool = pick == 1;
            patternTool_.previewValid = false;
            // A plane that was right for one of these is a no-op for the other.
            if (mirror) setPatternMode(PatternMode::Mirror);
        }
    }

    if (settled) ui::commandApplied(mirror ? "Mirror" : "Pattern");
    ui::commandHint(mirror
        ? (patternTool_.useTool
               ? "The last cut is reflected across the plane and made again."
               : "The body is reflected across the plane, and the two halves fuse.")
        : (patternTool_.useTool
               ? "The last cut is repeated. Its first copy is where it already is."
               : "The body is repeated, and the copies fuse where they meet."));

    const int footer = settled ? ui::commandFooter("Done", true, nullptr)
                               : ui::commandFooter("Finish");
    ui::endCommand();
    if (settled) {
        if (footer > 0)              dismissSettled();
        else if (signature() != was) recommitSettled();
        return;
    }
    if (footer > 0)      commitPattern();
    else if (footer < 0) abortPattern();
}

void Application::drawDividePanel() {
    const bool settled = settledIs(Settled::Divide);
    if (!divideTool_.active && !settled) return;

    auto signature = [&] {
        char b[48];
        std::snprintf(b, sizeof b, "%.9g", divideTool_.t);
        return std::string(b);
    };
    const std::string was = settled ? signature() : std::string();

    if (!ui::beginCommand("##divide", "Divide", Glyph::Divide,
                          objectName(scene_, divideTool_.objectId)))
        return;

    // Never at either end of the edge: a cut through a corner divides nothing,
    // and updateDivide holds the value inside the same two percent.
    const Real len = length(divideTool_.dir);
    const ui::NumberEdit v = ui::commandNumber("Along", divideTool_.t * len, "mm",
                                               !divideTool_.typedValue.empty(),
                                               !divideTool_.typedValue.empty(),
                                               divideTool_.typedValue.c_str(),
                                               len * 0.02, len * 0.98);
    applyBar(v, divideTool_.active, divideTool_.typedValue,
             [&](double x) {
                 if (len > 1e-9) divideTool_.t = clampf(x / len, 0.02, 0.98);
             },
             [&] { updateDivide(true); });

    char of[48];
    std::snprintf(of, sizeof of, "%.2f mm", len);
    ui::commandValue("Edge", of);

    if (settled) ui::commandApplied("Divide");
    ui::commandHint("The cut runs square across the edge you chose and slides along it. "
                    "The body stays whole.");

    const int footer = settled ? ui::commandFooter("Done", true, nullptr)
                               : ui::commandFooter("Finish");
    ui::endCommand();
    if (settled) {
        if (footer > 0)              dismissSettled();
        else if (signature() != was) recommitSettled();
        return;
    }
    if (footer > 0)      commitDivide();
    else if (footer < 0) abortDivide();
}

void Application::drawReducePanel() {
    if (!reduceTool_.active) return;
    if (!ui::beginCommand("##reduce", "Reduce Mesh", Glyph::Reduce,
                          objectName(scene_, reduceTool_.objectId)))
        return;

    const Real tolBefore = reduceTool_.tolerance;
    const int targetBefore = reduceTool_.target;
    const bool loosenBefore = reduceTool_.loosen;

    // A reduction takes seconds on a large mesh, so the bar only shows its
    // number while it is being pulled and asks for the preview once it is let
    // go. A thousandth of a millimetre is the finest anything prints to; half
    // a millimetre is coarser than any print.
    const ui::NumberEdit v = ui::commandNumber("Within", reduceTool_.tolerance, "mm",
                                               !reduceTool_.typedValue.empty(),
                                               !reduceTool_.typedValue.empty(),
                                               reduceTool_.typedValue.c_str(), 0.001, 0.5);
    bool holdPreview = false;
    if (v.dragged) {
        reduceTool_.tolerance = std::max(0.001, v.value);
        reduceTool_.typedValue.clear();
        holdPreview = true;
    } else if (v.clicked) {
        reduceTool_.typedValue.clear();
    }

    // The tolerances people actually use, one click each.
    {
        static const ui::Choice kPresets[5] = {
            {Glyph::Count, "0.01", nullptr, nullptr}, {Glyph::Count, "0.02", nullptr, nullptr},
            {Glyph::Count, "0.05", nullptr, nullptr}, {Glyph::Count, "0.1", nullptr, nullptr},
            {Glyph::Count, "0.25", nullptr, nullptr}};
        static const Real kValues[5] = {0.01, 0.02, 0.05, 0.1, 0.25};
        int on = -1;
        for (int i = 0; i < 5; ++i)
            if (std::fabs(reduceTool_.tolerance - kValues[i]) < 1e-12) on = i;
        const int pick = ui::commandChoices("", kPresets, 5, on, /*compact=*/true);
        if (pick >= 0) {
            reduceTool_.tolerance = kValues[pick];
            reduceTool_.typedValue.clear();
        }
    }

    ui::commandRow("Stop at");
    {
        // Few enough triangles to convert, however far the tolerance has to
        // loosen to get there -- which is only as far as the last few collapses
        // need. What it came to is shown under "Moved".
        const bool fit = reduceTool_.target == kSolidifyFaceLimit && reduceTool_.loosen;
        if (ui::pillButton("Fit for conversion", fit)) {
            reduceTool_.target = fit ? 0 : kSolidifyFaceLimit;
            reduceTool_.loosen = !fit;
        }
        ui::hoverTip("Reduce until there are few enough triangles to convert to a solid.\n"
                     "The tolerance is loosened only as far as that needs.");
    }

    // What it gives.
    const ReduceJob* shown = reduceTool_.shown.get();
    const bool stale = !reduceTool_.ready();
    char text[160];
    if (reduceTool_.preview.busy()) {
        ui::commandValue("Triangles", "reducing...");
    } else if (shown && !shown->result.ok) {
        ui::commandValue("Triangles", shown->result.error.c_str());
    } else if (shown) {
        std::snprintf(text, sizeof text, "%zu  ->  %zu", shown->result.trianglesBefore,
                      shown->result.trianglesAfter);
        ui::commandValue("Triangles", text);
    }
    if (shown && shown->result.ok) {
        std::snprintf(text, sizeof text, "%.3g mm, measured%s", static_cast<double>(shown->result.deviationMm),
                      shown->result.withinTolerance ? "" : " -- over");
        ui::commandValue("Moved", text);
        if (shown->result.toleranceUsedMm > shown->tolerance * 1.0001) {
            std::snprintf(text, sizeof text, "loosened to %.3g mm", static_cast<double>(shown->result.toleranceUsedMm));
            ui::commandValue("To fit", text);
        }
        if (shown->solidFaces <= kSolidifyFaceLimit)
            std::snprintf(text, sizeof text, "%d faces: will convert", shown->solidFaces);
        else
            std::snprintf(text, sizeof text, "%d faces: too many to convert", shown->solidFaces);
        ui::commandValue("As a solid", text);
    }

    ui::commandHint("Flat faces reduce to almost nothing; curved ones as far as the tolerance "
                    "allows. Edges, corners and holes stay within the tolerance, and the result "
                    "is measured before it is shown.");

    const int footer = ui::commandFooter(stale ? "Finish  (wait)" : "Finish", !stale);
    ui::endCommand();

    const bool changed = reduceTool_.tolerance != tolBefore || reduceTool_.target != targetBefore ||
                         reduceTool_.loosen != loosenBefore;
    if (v.released || (changed && !holdPreview)) requestReducePreview();
    if (footer > 0)      commitReduce();
    else if (footer < 0) abortReduce();
}

// The live value sits next to the cursor rather than only in a panel. During
// a drag the eye is on the geometry, and a number at the edge of the window is
// somewhere the user is not looking. The gestures with an arrow put the number
// on the arrow's head instead; this is for the transform, which has none.
void Application::drawTransformReadout() {
    if (!tool_.active()) return;
    const std::string text = tool_.statusText();
    if (text.empty()) return;

    // If the cursor has never entered the window ImGui reports a sentinel
    // position, and the box would be drawn off-screen. Fall back to the
    // viewport centre so the value is never simply missing.
    ImVec2 at(viewRect_.x + viewRect_.w * 0.5f, viewRect_.y + viewRect_.h * 0.5f);
    if (ImGui::IsMousePosValid()) {
        const ImVec2 m = ImGui::GetIO().MousePos;
        at = ImVec2(m.x + 20.0f, m.y - 34.0f);
    }
    drawReadout(text, at.x, at.y, /*emphasise=*/true);
}

// The arrows: one per gesture that pulls a value along a line, drawn on the
// screen where the value is measured. See ui/drag_guide.h for why on the
// screen and not in the world.
void Application::drawDragGuides() {
    const ImVec2 origin(ImGui::GetMainViewport()->Pos.x + viewRect_.x,
                        ImGui::GetMainViewport()->Pos.y + viewRect_.y);
    char label[64];

    if (filletTool_.active && filletTool_.axis.valid) {
        const SceneObject* o = scene_.find(filletTool_.objectId);
        const Real step = o ? DragAxis::stepFor(camera_, filletTool_.axis.origin, filletTool_.maxRadius)
                            : 0.0;
        std::snprintf(label, sizeof label, "%s %.2f mm", filletTool_.chamfer ? "Chamfer" : "Fillet",
                      filletTool_.currentRadius);
        ui::drawDragGuide(filletTool_.axis, camera_, origin, filletTool_.currentRadius, step,
                          filletTool_.maxRadius, label);
    }

    if (faceTool_.active && faceTool_.axis.valid) {
        const Real step = faceTool_.op == FaceOp::Rotate ? 5.0
                        : DragAxis::stepFor(camera_, faceTool_.axis.origin, faceTool_.axis.spanValue);
        if (faceTool_.op == FaceOp::Rotate)      std::snprintf(label, sizeof label, "%.1f\xC2\xB0", faceTool_.value);
        else if (faceTool_.op == FaceOp::Scale)  std::snprintf(label, sizeof label, "%+.1f %%", faceTool_.value);
        else                                     std::snprintf(label, sizeof label, "%.2f mm", faceTool_.value);
        ui::drawDragGuide(faceTool_.axis, camera_, origin, faceTool_.value, step, 0.0, label);
    }
    if (patternTool_.active && patternTool_.axis.valid) {
        const Real step = DragAxis::stepFor(camera_, patternTool_.axis.origin, patternTool_.axis.spanValue);
        std::snprintf(label, sizeof label, patternTool_.mode == PatternMode::Circular ? "%.1f\xC2\xB0" : "%.2f mm",
                      patternTool_.dragged());
        ui::drawDragGuide(patternTool_.axis, camera_, origin, patternTool_.dragged(), step,
                          patternTool_.axis.spanValue, label);
    }
    if (divideTool_.active && divideTool_.axis.valid) {
        const Real step = DragAxis::stepFor(camera_, divideTool_.axis.origin, divideTool_.axis.spanValue);
        std::snprintf(label, sizeof label, "%.2f mm", divideTool_.t * length(divideTool_.dir));
        ui::drawDragGuide(divideTool_.axis, camera_, origin, divideTool_.t * length(divideTool_.dir),
                          step, divideTool_.axis.spanValue, label);
    }
}

} // namespace tg
