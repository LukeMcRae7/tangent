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
#include <tuple>

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
        // An axis that lies in the face sweeps nothing, and one square to it
        // turns the face in its own plane, which is no turn at all. Those are
        // shown dimmed rather than offered and then refused.
        Vec3 normal{};
        if (const SceneObject* fo = scene_.find(faceTool_.objectId)) {
            const Mat4 nm = normalMatrix(fo->modelMatrix());
            for (FaceId f : faceTool_.faces)
                if (faceTool_.before.hasFace(f))
                    normal += normalize(transformVector(nm, faceTool_.before.faceNormal(f)));
        }
        if (lengthSq(normal) > 1e-12) normal = normalize(normal);
        auto axisWorks = [&](int axis) {
            if (lengthSq(normal) < 1e-12) return true;
            Vec3 a{};
            (&a.x)[axis] = 1.0;
            const Real d = std::fabs(dot(a, normal));
            return rotate ? d < 0.999 : d > 0.02;
        };
        ui::Choice kAlong[4] = {
            {Glyph::Count, "Normal", nullptr, "Straight out of the face"},
            {Glyph::Count, "X", "X", "Along the world X axis"},
            {Glyph::Count, "Y", "Y", "Along the world Y axis"},
            {Glyph::Count, "Z", "Z", "Along the world Z axis"},
        };
        ui::Choice kPivot[4] = {
            {Glyph::Count, "Edge", nullptr, "About the edge nearest the pointer"},
            {Glyph::Count, "X", "X", "About the world X axis"},
            {Glyph::Count, "Y", "Y", "About the world Y axis"},
            {Glyph::Count, "Z", "Z", "About the world Z axis"},
        };
        for (int axis = 0; axis < 3; ++axis) {
            const bool works = axisWorks(axis);
            kAlong[axis + 1].enabled = works;
            kPivot[axis + 1].enabled = works;
            if (works) continue;
            kAlong[axis + 1].tip = "That axis lies along the face: it would sweep nothing";
            kPivot[axis + 1].tip = "That axis is square to the face: it would turn it in its own plane";
        }
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

    // Only when it has actually run into something, and only for a push or
    // pull: that is the one this choice is acted on for. A scale that runs
    // into another body leaves it alone, and offering to join or cut there
    // would be offering something that does not happen.
    if (faceTool_.op == FaceOp::Move && faceTool_.meets != kNoObject) {
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
        std::snprintf(b, sizeof b, "%d|%d|%d|%d|%.9g|%.9g,%.9g,%.9g", (int)patternTool_.mode,
                      patternTool_.count, patternTool_.axisIndex,
                      (int)patternTool_.useTool, patternTool_.dragged(), patternTool_.origin.x,
                      patternTool_.origin.y, patternTool_.origin.z);
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

    // Where the axis stands. A ring turns about the body's middle unless it is
    // told otherwise, and a bolt circle hardly ever goes round the middle of
    // the part: it goes round a hole. The two numbers are the ones across the
    // axis, in the body's own space.
    if (ring) {
        const int u = (patternTool_.axisIndex + 1) % 3, w = (patternTool_.axisIndex + 2) % 3;
        static const char* kNames[3] = {"x", "y", "z"};
        char label[32];
        std::snprintf(label, sizeof label, "Axis at %s, %s", kNames[u], kNames[w]);
        ui::commandRow(label);
        double at[2] = {patternTool_.origin[u], patternTool_.origin[w]};
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::InputScalarN("##axisat", ImGuiDataType_Double, at, 2, nullptr, nullptr, "%.2f");
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            patternTool_.origin[u] = at[0];
            patternTool_.origin[w] = at[1];
            setPatternMode(patternTool_.mode);
        }
        // A face is the usual way to say where: the middle of the hole the
        // copies go round.
        const std::vector<FaceId> faces = scene_.selectedFaces(patternTool_.objectId);
        if (!faces.empty()) {
            ImGui::SameLine(0.0f, 6.0f);
            if (ui::pillButton("From the face", false)) {
                if (const SceneObject* o = scene_.find(patternTool_.objectId)) {
                    std::vector<VertexId> fv;
                    o->body.faceVertices(faces.front(), fv);
                    Vec3 centre{};
                    for (VertexId vid : fv) centre += o->body.vertexPosition(vid);
                    if (!fv.empty()) {
                        centre *= 1.0 / static_cast<Real>(fv.size());
                        patternTool_.origin = centre;
                        setPatternMode(patternTool_.mode);
                    }
                }
            }
            ui::hoverTip("Put the axis through the middle of the selected face");
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

// A draft: how far the walls lean, and off what.
//
// Print-first: the part comes off the bed, so the pull starts as Z and the
// widest place as the bottom. Leaning a wall three degrees is what keeps it
// from printing out over nothing, and it is the same operation a moulded part
// needs to leave its tool.
void Application::drawDraftPanel() {
    const bool settled = settledIs(Settled::Draft);
    if (!settled && !draftTool_.pending) return;
    if (!scene_.find(draftTool_.objectId)) { draftTool_.reset(); return; }

    const auto was = std::make_tuple(draftTool_.angle, draftTool_.axis,
                                     static_cast<int>(draftTool_.widest));

    if (!ui::beginCommand("##draft", "Draft", Glyph::Draft,
                          objectName(scene_, draftTool_.objectId)))
        return;

    {
        double deg = draftTool_.angle * kRad2Deg;
        const ui::NumberEdit v = ui::commandNumber("Angle", deg, "\xC2\xB0",
                                                   !draftTool_.typedValue.empty(),
                                                   !draftTool_.typedValue.empty(),
                                                   draftTool_.typedValue.c_str(), -45.0, 45.0,
                                                   /*signedRange=*/true);
        applyBar(v, /*active=*/false, draftTool_.typedValue,
                 [&](double x) { draftTool_.angle = std::clamp(x, -60.0, 60.0) * kDeg2Rad; },
                 [] {});
    }

    // Which way the part comes off. A wall square to an axis cannot lean about
    // it -- there is no line where it meets the neutral plane -- so that axis
    // is dimmed with the reason rather than offered and then refused.
    {
        ui::Choice kAxis[3] = {
            {Glyph::Count, "X", nullptr, "Pulled along X"},
            {Glyph::Count, "Y", nullptr, "Pulled along Y"},
            {Glyph::Count, "Z", nullptr, "Pulled along Z: up, off the bed"},
        };
        const SceneObject* o = scene_.find(draftTool_.objectId);
        for (int a = 0; a < 3; ++a) {
            Vec3 dir{};
            (&dir.x)[a] = 1.0;
            const bool ok = o && std::all_of(draftTool_.faces.begin(), draftTool_.faces.end(),
                                             [&](FaceId f) {
                                                 const Vec3 n = draftTool_.before.faceNormal(f);
                                                 return length(n) > 1e-9 &&
                                                        std::fabs(dot(normalize(n), dir)) < 0.999;
                                             });
            if (!ok) {
                kAxis[a].enabled = false;
                kAxis[a].tip = "A wall square to this axis has nothing to lean about";
            }
        }
        const int pick = ui::commandChoices("Pulled along", kAxis, 3, draftTool_.axis,
                                            /*compact=*/true);
        if (pick >= 0 && pick != draftTool_.axis) draftTool_.axis = pick;
    }

    // Where the wall is the size it was drawn. It leans the same way all the
    // way along; this only says where it pivots, so a part can be kept to size
    // at the end that matters -- the bed it stands on, or the face something
    // else has to fit against.
    {
        using Widest = DraftToolState::Widest;
        static const ui::Choice kAt[3] = {
            {Glyph::Count, "Bottom", nullptr, "Keeps its size where it stands: the bed"},
            {Glyph::Count, "Top",    nullptr, "Keeps its size at the far end"},
            {Glyph::Count, "Middle", nullptr, "Keeps its size half way along"},
        };
        const int on = static_cast<int>(draftTool_.widest);
        const int pick = ui::commandChoices("Same size at", kAt, 3, on, /*compact=*/true);
        if (pick >= 0 && pick != on) draftTool_.widest = static_cast<Widest>(pick);
    }

    char sel[64];
    std::snprintf(sel, sizeof sel, "%zu face%s", draftTool_.faces.size(),
                  draftTool_.faces.size() == 1 ? "" : "s");
    ui::commandValue("Selection", sel);

    // What the lean comes to on this part, which is the number that decides
    // whether it prints: how far the top of the wall has come in.
    if (const SceneObject* o = scene_.find(draftTool_.objectId)) {
        const AABB b = draftTool_.before.bounds();
        if (b.valid()) {
            const Vec3 size = b.size();
            const Real height = (&size.x)[std::clamp(draftTool_.axis, 0, 2)];
            const Real reach = std::fabs(std::tan(draftTool_.angle)) * height;
            char in[80];
            std::snprintf(in, sizeof in, "%.2f mm over %.1f mm", static_cast<double>(reach),
                          static_cast<double>(height));
            ui::commandValue("Leans", in);
        }
        (void)o;
    }

    if (settled) ui::commandApplied("Draft");
    else         ui::commandRefused(draftTool_.refusal.c_str());
    ui::commandHint("Each wall keeps the size it was drawn where it crosses that plane, and "
                    "narrows along the pull from there. A negative angle leans it the other way.");

    const int footer = settled ? ui::commandFooter("Done", true, nullptr)
                               : ui::commandFooter("Try again", true, "Cancel");
    ui::endCommand();

    if (std::make_tuple(draftTool_.angle, draftTool_.axis,
                        static_cast<int>(draftTool_.widest)) != was) {
        if (settled) recommitSettled();
        else         { draftTool_.active = true; commitDraft(); }
        return;
    }
    if (footer > 0 && settled)  dismissSettled();
    else if (footer > 0)        { draftTool_.active = true; commitDraft(); }
    else if (footer < 0)        draftTool_.reset();
}

// A hole: which screw it is for, how freely that screw passes, what its head
// sits in, and how deep.
//
// Almost none of that is a number to type. A hole in a printed part is chosen
// from a short list -- M3 clearance, counterbored -- and the millimetres come
// from the table, including the two tenths a printed hole has to be cut over
// size to come out the size it was drawn. The panel says what it arrived at,
// so the number is never a mystery, and typing one directly is still there for
// the hole that is not for a screw at all.
void Application::drawHolePanel() {
    const bool settled = settledIs(Settled::Hole);
    if (!settled && !holeTool_.pending && !holeTool_.placing) return;
    if (!scene_.find(holeTool_.objectId)) { holeTool_.reset(); return; }

    auto signature = [&] {
        const HoleCut c = holeCutNow();
        char b[160];
        std::snprintf(b, sizeof b, "%d|%d|%d|%d|%.9g|%.9g|%.9g|%d", holeTool_.fastener,
                      static_cast<int>(holeTool_.fit), static_cast<int>(c.kind),
                      static_cast<int>(c.through), c.depth, c.headDepth, c.diameter,
                      static_cast<int>(c.drillPoint));
        return std::string(b);
    };
    const std::string was = signature();

    if (!ui::beginCommand("##hole", "Hole", Glyph::Hole,
                          objectName(scene_, holeTool_.objectId)))
        return;

    // The sizes, as a row of names rather than diameters: M3 is the thing
    // being chosen, 3.4 is what it happens to measure.
    {
        ui::commandRow("Size");
        for (int i = 0; i < fastenerCount(); ++i) {
            if (i) ImGui::SameLine(0.0f, 3.0f);
            if (ui::pillButton(fastenerAt(i).name, holeTool_.fastener == i) &&
                holeTool_.fastener != i)
                holeTool_.fastener = i;
        }
        ImGui::SameLine(0.0f, 3.0f);
        if (ui::pillButton("Custom", holeTool_.fastener < 0) && holeTool_.fastener >= 0) {
            holeTool_.cut = holeCutNow();      // start from where the table left it
            holeTool_.fastener = -1;
        }
    }

    // How freely the screw passes, or that it cuts its own thread. A tapped
    // hole has no head to sit in, so those go dim rather than away.
    if (holeTool_.fastener >= 0) {
        static const ui::Choice kFits[4] = {
            {Glyph::Count, "Close",  nullptr, "Located by the hole (ISO 273 fine)"},
            {Glyph::Count, "Normal", nullptr, "The everyday clearance (ISO 273 medium)"},
            {Glyph::Count, "Loose",  nullptr, "Room to move (ISO 273 coarse)"},
            {Glyph::Count, "Tapped", nullptr, "No clearance: the screw cuts its own thread"},
        };
        const int on = static_cast<int>(holeTool_.fit);
        const int pick = ui::commandChoices("Fit", kFits, 4, on, /*compact=*/true);
        if (pick >= 0 && pick != on) {
            holeTool_.fit = static_cast<HoleFit>(pick);
            if (holeTool_.fit == HoleFit::Tapped) holeTool_.cut.kind = HoleKind::Simple;
        }
    } else {
        const ui::NumberEdit d = ui::commandNumber("Diameter", holeTool_.cut.diameter, "mm",
                                                   !holeTool_.typedValue.empty(),
                                                   !holeTool_.typedValue.empty(),
                                                   holeTool_.typedValue.c_str(), 0.5, 50.0);
        applyBar(d, /*active=*/false, holeTool_.typedValue,
                 [&](double x) { holeTool_.cut.diameter = std::max(x, 0.1); }, [] {});
    }

    // What the mouth looks like. A tapped hole is not counterbored here --
    // there is no head to sink -- and saying so beats offering it.
    {
        ui::Choice kHead[3] = {
            {Glyph::Hole,        "Plain",       nullptr, "One diameter all the way"},
            {Glyph::Counterbore, "Counterbore", nullptr, "A flat pocket for a cap head"},
            {Glyph::Countersink, "Countersink", nullptr, "A cone for a flat head"},
        };
        const bool tapped = holeTool_.fastener >= 0 && holeTool_.fit == HoleFit::Tapped;
        if (tapped) {
            kHead[1].enabled = kHead[2].enabled = false;
            kHead[1].tip = kHead[2].tip = "A tapped hole has no head to sit in";
        }
        const int on = static_cast<int>(holeTool_.cut.kind);
        const int pick = ui::commandChoices("Head", kHead, 3, on);
        if (pick >= 0 && pick != on) holeTool_.cut.kind = static_cast<HoleKind>(pick);
    }

    // The counterbore's depth is the one head number worth setting: how far
    // below the surface the screw ends up.
    if (holeCutNow().kind == HoleKind::Counterbore) {
        const ui::NumberEdit v = ui::commandNumber("Pocket", holeTool_.cut.headDepth, "mm", false,
                                                   false, nullptr, 0.5, 30.0);
        if (v.dragged) holeTool_.cut.headDepth = std::max(v.value, 0.2);
    }

    // How deep. Through is the common case and is a button rather than a
    // number nobody can pick: a depth that happens to be longer than the part
    // is not the same thing as through.
    {
        ui::commandRow("Depth");
        if (ui::pillButton("Through", holeTool_.cut.through) && !holeTool_.cut.through)
            holeTool_.cut.through = true;
        ImGui::SameLine(0.0f, 3.0f);
        if (ui::pillButton("To a depth", !holeTool_.cut.through) && holeTool_.cut.through)
            holeTool_.cut.through = false;
    }
    if (!holeTool_.cut.through) {
        double most = 100.0;
        if (const SceneObject* o = scene_.find(holeTool_.objectId))
            if (o->localBounds.valid()) most = length(o->localBounds.size());
        const ui::NumberEdit v = ui::commandNumber("Deep", holeTool_.cut.depth, "mm", false, false,
                                                   nullptr, 0.5, most);
        if (v.dragged) holeTool_.cut.depth = std::max(v.value, 0.2);
        ui::commandRow("Bottom");
        if (ui::pillButton("Drill point", holeTool_.cut.drillPoint) && !holeTool_.cut.drillPoint)
            holeTool_.cut.drillPoint = true;
        ui::hoverTip("A cone at the bottom, as a drill leaves and as a printer wants: "
                     "a flat ceiling over a hole has nothing to print onto");
        ImGui::SameLine(0.0f, 3.0f);
        if (ui::pillButton("Flat", !holeTool_.cut.drillPoint) && holeTool_.cut.drillPoint)
            holeTool_.cut.drillPoint = false;
    }

    // What it comes to, and why it is not the number in the standard: a
    // printed hole is cut over size to come out the size it was drawn.
    {
        const HoleCut c = holeCutNow();
        char at[96];
        if (holeTool_.fastener >= 0 && printedAllowance(holeTool_.fit) > 0.0)
            std::snprintf(at, sizeof at, "%.2f mm  (%.2f + %.2f for printing)", c.diameter,
                          c.diameter - printedAllowance(holeTool_.fit),
                          printedAllowance(holeTool_.fit));
        else
            std::snprintf(at, sizeof at, "%.2f mm", c.diameter);
        ui::commandValue("Cut at", at);
    }

    if (holeTool_.placing)   ui::commandHint("Point at the face it goes into, and click to drill it.");
    else if (settled)      { ui::commandApplied("Hole"); ui::commandHint("Change the size or the depth and it is drilled again."); }
    else                     ui::commandRefused(holeTool_.refusal.c_str());

    const int footer = holeTool_.placing ? ui::commandFooter(nullptr, false, "Cancel")
                     : settled           ? ui::commandFooter("Done", true, nullptr)
                                         : ui::commandFooter("Try again", true, "Cancel");
    ui::endCommand();

    if (signature() != was && !holeTool_.placing) {
        if (settled) recommitSettled();
        else         { holeTool_.active = true; commitHole(); }
        return;
    }
    if (footer > 0 && settled)  dismissSettled();
    else if (footer > 0)        { holeTool_.active = true; commitHole(); }
    else if (footer < 0)        abortHole();
}

// Inset, and Shell. Neither has a gesture behind it: the operation is one
// number, made as soon as it is asked for and adjusted here until Done. That
// is the difference between choosing a wall thickness and guessing one.
void Application::drawInsetPanel() {
    const bool settled = settledIs(Settled::Inset);
    if (!settled && !insetTool_.pending) return;
    // The body it was opened on may be gone -- deleted, or undone away.
    if (!scene_.find(insetTool_.objectId)) { insetTool_.reset(); return; }
    const double was = insetTool_.amount;

    if (!ui::beginCommand("##inset", "Inset Face", Glyph::Inset,
                          objectName(scene_, insetTool_.objectId)))
        return;

    // How far in it can go: an inset runs in from every edge of the face, so
    // half the face's smallest side is the most that leaves anything.
    double most = 10.0;
    if (const SceneObject* o = scene_.find(insetTool_.objectId)) {
        const AABB b = o->localBounds;
        if (b.valid()) most = std::max(0.1, std::min({b.size().x, b.size().y, b.size().z}) * 0.5);
    }
    const ui::NumberEdit v = ui::commandNumber("Distance", insetTool_.amount, "mm",
                                               !insetTool_.typedValue.empty(),
                                               !insetTool_.typedValue.empty(),
                                               insetTool_.typedValue.c_str(), 0.05,
                                               std::max(most, insetTool_.amount));
    applyBar(v, /*active=*/false, insetTool_.typedValue,
             [&](double x) { insetTool_.amount = std::max(x, 0.01); }, [] {});

    char sel[64];
    std::snprintf(sel, sizeof sel, "%zu face%s", insetTool_.faces.size(),
                  insetTool_.faces.size() == 1 ? "" : "s");
    ui::commandValue("Selection", sel);

    if (settled) ui::commandApplied("Inset");
    else         ui::commandRefused(insetTool_.refusal.c_str());
    ui::commandHint("A ring inside the face, the same distance in from every edge of it. "
                    "What is left inside is a face of its own, to push or pull.");

    const int footer = settled ? ui::commandFooter("Done", true, nullptr)
                               : ui::commandFooter("Try again", true, "Cancel");
    ui::endCommand();
    if (insetTool_.amount != was) {
        // A number that was refused is tried again as soon as it changes;
        // one that worked re-applies what is already there.
        if (settled) recommitSettled();
        else         { insetTool_.active = true; commitInset(); }
        return;
    }
    if (footer > 0 && settled)       dismissSettled();
    else if (footer > 0)             { insetTool_.active = true; commitInset(); }
    else if (footer < 0)             insetTool_.reset();
}

void Application::drawShellPanel() {
    const bool settled = settledIs(Settled::Shell);
    if (!settled && !shellTool_.pending) return;
    if (!scene_.find(shellTool_.objectId)) { shellTool_.reset(); return; }
    const double was = shellTool_.amount;

    if (!ui::beginCommand("##shell", "Shell", Glyph::Shell,
                          objectName(scene_, shellTool_.objectId)))
        return;

    // A wall cannot be thicker than half the thinnest way through the body.
    double most = 20.0;
    if (const SceneObject* o = scene_.find(shellTool_.objectId)) {
        const AABB b = o->localBounds;
        if (b.valid()) most = std::max(0.1, std::min({b.size().x, b.size().y, b.size().z}) * 0.5);
    }
    const ui::NumberEdit v = ui::commandNumber("Wall", shellTool_.amount, "mm",
                                               !shellTool_.typedValue.empty(),
                                               !shellTool_.typedValue.empty(),
                                               shellTool_.typedValue.c_str(), 0.05,
                                               std::max(most, shellTool_.amount));
    applyBar(v, /*active=*/false, shellTool_.typedValue,
             [&](double x) { shellTool_.amount = std::max(x, 0.01); }, [] {});

    // What it opens. Chosen before it started, so this says what was taken
    // rather than offering a choice that is no longer there to make.
    ui::commandRow("Open");
    ImGui::AlignTextToFramePadding();
    if (shellTool_.faces.empty())
        ImGui::TextColored(ui::im(palette::kTextDim), "nothing: a sealed cavity");
    else
        ImGui::Text("%zu face%s", shellTool_.faces.size(), shellTool_.faces.size() == 1 ? "" : "s");

    // What it comes to at the nozzle, which is the reason the number matters
    // on a printed part: a wall is a whole number of lines or it is not the
    // wall you asked for.
    {
        const PrintProfile profile;
        char walls[64];
        std::snprintf(walls, sizeof walls, "%.1f lines of %.2f mm",
                      shellTool_.amount / profile.nozzleMm, static_cast<double>(profile.nozzleMm));
        ui::commandValue("Prints as", walls);
        if (shellTool_.amount < profile.minWallMm) {
            ui::commandRow("");
            ImGui::AlignTextToFramePadding();
            ImGui::TextColored(ui::im(palette::kBrand), "thinner than %.2f mm prints badly",
                               static_cast<double>(profile.minWallMm));
        }
    }

    if (settled) ui::commandApplied("Shell");
    else         ui::commandRefused(shellTool_.refusal.c_str());
    ui::commandHint(shellTool_.faces.empty()
                        ? "Hollowed out and closed. Select a face before shelling to leave it open."
                        : "Hollowed out, with the faces that were selected left open.");

    const int footer = settled ? ui::commandFooter("Done", true, nullptr)
                               : ui::commandFooter("Try again", true, "Cancel");
    ui::endCommand();
    if (shellTool_.amount != was) {
        if (settled) recommitSettled();
        else         { shellTool_.active = true; commitShell(); }
        return;
    }
    if (footer > 0 && settled)       dismissSettled();
    else if (footer > 0)             { shellTool_.active = true; commitShell(); }
    else if (footer < 0)             shellTool_.reset();
}

// Split: what the cut is made with, and where it sits. Every answer it used to
// pick for itself is a choice here, and the ones this body cannot offer are
// shown dimmed rather than left out.
void Application::drawSplitPanel() {
    const bool settled = settledIs(Settled::Split);
    if (!settled && !splitTool_.pending) return;
    if (!scene_.find(splitTool_.objectId)) { splitTool_.reset(); return; }
    const auto was = std::make_pair(static_cast<int>(splitTool_.by), splitTool_.offset);

    if (!ui::beginCommand("##split", "Split Body", Glyph::Split,
                          objectName(scene_, splitTool_.objectId)))
        return;

    using By = SplitToolState::By;
    ui::Choice kBy[6] = {
        {Glyph::Count, "Face",   nullptr, "The plane of the selected face"},
        {Glyph::Count, "Tool",   nullptr, "The plane of the other selected body"},
        {Glyph::Count, "X",      nullptr, "A plane square to X"},
        {Glyph::Count, "Y",      nullptr, "A plane square to Y"},
        {Glyph::Count, "Z",      nullptr, "A plane square to Z"},
        {Glyph::Count, "Pieces", nullptr, "Take the loose pieces apart, cutting nothing"},
    };
    kBy[0].enabled = splitTool_.face != kInvalid;
    kBy[1].enabled = splitTool_.toolObject != kNoObject;
    kBy[5].enabled = splitTool_.canPieces;
    if (!kBy[0].enabled) kBy[0].tip = "No face is selected on this body";
    if (!kBy[1].enabled) kBy[1].tip = "Select a second body to cut with its plane";
    if (!kBy[5].enabled) kBy[5].tip = "This body is all one piece";

    const int on = static_cast<int>(splitTool_.by);
    const int pick = ui::commandChoices("Cut by", kBy, 6, on, /*compact=*/true);
    if (pick >= 0 && pick != on) {
        splitTool_.by = static_cast<By>(pick);
        // A plane square to an axis starts through the middle of the body;
        // one taken from a face or a tool starts on it.
        const SceneObject* o = scene_.find(splitTool_.objectId);
        const AABB b = o ? splitTool_.before.bounds() : AABB{};
        if (splitTool_.by == By::X || splitTool_.by == By::Y || splitTool_.by == By::Z) {
            const int axis = static_cast<int>(splitTool_.by) - 2;
            const Vec3 centre = b.valid() ? b.center() : Vec3{};
            splitTool_.offset = centre[axis];
        } else {
            splitTool_.offset = 0.0;
        }
        splitTool_.typedValue.clear();
    }

    // Where the plane sits: a coordinate for an axis plane, and how far off
    // the face or the tool for those.
    if (splitTool_.by != By::Pieces) {
        const AABB b = splitTool_.before.bounds();
        double lo = -50.0, hi = 50.0;
        if (b.valid()) {
            const int axis = splitTool_.by == By::X ? 0 : splitTool_.by == By::Y ? 1 : 2;
            if (splitTool_.by == By::X || splitTool_.by == By::Y || splitTool_.by == By::Z) {
                lo = (&b.min.x)[axis];
                hi = (&b.max.x)[axis];
            } else {
                const Real reach = length(b.size());
                lo = -reach;
                hi = reach;
            }
        }
        const bool onAxis = splitTool_.by == By::X || splitTool_.by == By::Y || splitTool_.by == By::Z;
        const ui::NumberEdit v = ui::commandNumber(onAxis ? "Plane at" : "Offset", splitTool_.offset,
                                                   "mm", !splitTool_.typedValue.empty(),
                                                   !splitTool_.typedValue.empty(),
                                                   splitTool_.typedValue.c_str(),
                                                   std::min(lo, splitTool_.offset),
                                                   std::max(hi, splitTool_.offset),
                                                   /*signedRange=*/true);
        applyBar(v, /*active=*/false, splitTool_.typedValue,
                 [&](double x) { splitTool_.offset = x; }, [] {});
    }

    char result[64];
    std::snprintf(result, sizeof result, "%d bodies", splitTool_.pieces);
    ui::commandValue("Result", splitTool_.pieces >= 2 ? result : "nothing cut");

    if (settled) ui::commandApplied("Split");
    else         ui::commandRefused(splitTool_.refusal.c_str());
    ui::commandHint(splitTool_.by == By::Pieces
                        ? "The body was already in separate pieces; each is its own body now."
                        : "The pieces keep the geometry, not the steps that made it: a split is "
                          "where a history ends.");

    const int footer = settled ? ui::commandFooter("Done", true, nullptr)
                               : ui::commandFooter("Try again", true, "Cancel");
    ui::endCommand();
    if (was != std::make_pair(static_cast<int>(splitTool_.by), splitTool_.offset)) {
        if (settled) recommitSettled();
        else         { splitTool_.active = true; commitSplit(); }
        return;
    }
    if (footer > 0 && settled)      dismissSettled();
    else if (footer > 0)            { splitTool_.active = true; commitSplit(); }
    else if (footer < 0)            splitTool_.reset();
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
