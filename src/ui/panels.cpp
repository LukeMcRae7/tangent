#include "ui/panels.h"

#include "geom/fasteners.h"
#include "ui/command_panel.h"
#include "ui/glyph.h"
#include "ui/theme.h"
#include "ui/widgets.h"

#include "core/palette.h"
#include "geom/brep.h"

#include "imgui.h"
#include "imgui_internal.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace tg {
namespace {

using ui::im;
using ui::u32;

constexpr const char* kDegree = "\xC2\xB0";

// Which picture stands for a shape, and for a step of a history.
Glyph glyphFor(PrimitiveKind kind) {
    switch (kind) {
        case PrimitiveKind::Cylinder: return Glyph::Cylinder;
        case PrimitiveKind::Sphere:   return Glyph::Sphere;
        case PrimitiveKind::Cone:     return Glyph::Cone;
        case PrimitiveKind::Torus:    return Glyph::Torus;
        case PrimitiveKind::Plane:    return Glyph::Plane;
        default:                      return Glyph::Box;
    }
}

Glyph glyphFor(const Feature& f) {
    switch (f.kind) {
        case FeatureKind::Primitive:      return glyphFor(f.primitive.kind);
        case FeatureKind::Extrude:        return f.mergeFlush ? Glyph::PushPull : Glyph::Extrude;
        case FeatureKind::ExtrudeProfile: return Glyph::Extrude;
        case FeatureKind::RevolveProfile: return Glyph::Revolve;
        case FeatureKind::Hole:           return Glyph::Hole;
        case FeatureKind::Draft:          return Glyph::Draft;
        case FeatureKind::Bevel:          return f.chamfer ? Glyph::Chamfer : Glyph::Fillet;
        case FeatureKind::Shell:          return Glyph::Shell;
        case FeatureKind::FaceRotate:     return Glyph::RotateFace;
        case FeatureKind::FaceScale:      return Glyph::ScaleFace;
        case FeatureKind::Divide:         return Glyph::Divide;
        case FeatureKind::Merge:          return Glyph::Merge;
        case FeatureKind::Pattern:
            return f.patternMode == PatternMode::Mirror   ? Glyph::Mirror
                 : f.patternMode == PatternMode::Circular ? Glyph::PatternRing
                                                          : Glyph::Pattern;
        case FeatureKind::Inset:          return Glyph::Inset;
        case FeatureKind::Boolean:
            return f.booleanOp == BooleanOp::Union        ? Glyph::Union
                 : f.booleanOp == BooleanOp::Intersection ? Glyph::Intersect
                                                          : Glyph::Difference;
        case FeatureKind::Sketch:         return Glyph::Sketch;
        case FeatureKind::BaseMesh:       return Glyph::Mesh;
        case FeatureKind::Reduce:         return Glyph::Reduce;
        case FeatureKind::VertexEdit:     return Glyph::Move;
        case FeatureKind::Move:           return Glyph::Move;
        case FeatureKind::Rotate:         return Glyph::Rotate;
        case FeatureKind::Scale:          return Glyph::Scale;
    }
    return Glyph::Box;
}

// What an object is, for the outliner's three lists.
enum class ObjectKind { Body, Sketch, Mesh };

ObjectKind kindOf(const SceneObject& o) {
    if (!o.body.empty() && o.body.isMesh()) return ObjectKind::Mesh;
    if (o.body.empty() && !o.features.empty() && o.features.front().kind == FeatureKind::Sketch)
        return ObjectKind::Sketch;
    return ObjectKind::Body;
}

Glyph glyphFor(ObjectKind k) {
    switch (k) {
        case ObjectKind::Sketch: return Glyph::Sketch;
        case ObjectKind::Mesh:   return Glyph::Mesh;
        default:                 return Glyph::Body;
    }
}

// Parametric controls for whichever primitive the object was created from.
// Editing any of these regenerates the body, which is the first real piece of
// the parametric workflow.
bool drawPrimitiveParams(SceneObject& obj) {
    using ui::labelledInt;
    using ui::labelledNumber;
    bool changed = false;
    switch (obj.spec.kind) {
        case PrimitiveKind::Box:
            changed |= labelledNumber("Width",  obj.spec.box.width,  0.1f, 0.01f, 10000.0f);
            changed |= labelledNumber("Depth",  obj.spec.box.depth,  0.1f, 0.01f, 10000.0f);
            changed |= labelledNumber("Height", obj.spec.box.height, 0.1f, 0.01f, 10000.0f);
            break;
        case PrimitiveKind::Cylinder:
            changed |= labelledNumber("Radius", obj.spec.cylinder.radius, 0.1f, 0.01f, 10000.0f);
            changed |= labelledNumber("Height", obj.spec.cylinder.height, 0.1f, 0.01f, 10000.0f);
            changed |= labelledInt   ("Sides",  obj.spec.cylinder.segments, 3, 512);
            break;
        case PrimitiveKind::Sphere:
            changed |= labelledNumber("Radius",   obj.spec.sphere.radius, 0.1f, 0.01f, 10000.0f);
            changed |= labelledInt   ("Segments", obj.spec.sphere.segments, 3, 512);
            changed |= labelledInt   ("Rings",    obj.spec.sphere.rings, 2, 256);
            break;
        case PrimitiveKind::Cone:
            changed |= labelledNumber("Base R",  obj.spec.cone.bottomRadius, 0.1f, 0.01f, 10000.0f);
            changed |= labelledNumber("Top R",   obj.spec.cone.topRadius, 0.1f, 0.0f, 10000.0f);
            changed |= labelledNumber("Height",  obj.spec.cone.height, 0.1f, 0.01f, 10000.0f);
            changed |= labelledInt   ("Sides",   obj.spec.cone.segments, 3, 512);
            break;
        case PrimitiveKind::Torus:
            changed |= labelledNumber("Major R", obj.spec.torus.majorRadius, 0.1f, 0.02f, 10000.0f);
            changed |= labelledNumber("Minor R", obj.spec.torus.minorRadius, 0.1f, 0.01f, 10000.0f);
            changed |= labelledInt   ("Major",   obj.spec.torus.majorSegments, 3, 512);
            changed |= labelledInt   ("Minor",   obj.spec.torus.minorSegments, 3, 256);
            // The generator rejects a minor radius that would self-intersect,
            // so clamp here instead of letting the rebuild silently no-op.
            if (obj.spec.torus.minorRadius >= obj.spec.torus.majorRadius)
                obj.spec.torus.minorRadius = obj.spec.torus.majorRadius * 0.98f;
            break;
        case PrimitiveKind::Plane:
            changed |= labelledNumber("Width", obj.spec.plane.width, 0.1f, 0.01f, 10000.0f);
            changed |= labelledNumber("Depth", obj.spec.plane.depth, 0.1f, 0.01f, 10000.0f);
            break;
        case PrimitiveKind::Custom:
            ImGui::TextColored(im(palette::kTextDim), "Edited mesh: no parameters");
            break;
    }
    return changed;
}

// ---- outliner rows ----------------------------------------------------------

// A collapsible heading: "Bodies  2".
bool sectionHeader(const char* name, size_t count) {
    ImGui::PushID(name);
    ImGuiStorage* store = ImGui::GetStateStorage();
    const ImGuiID key = ImGui::GetID("open");
    bool open = store->GetBool(key, true);

    const float h = ImGui::GetTextLineHeight() + 8.0f;
    const float w = ImGui::GetContentRegionAvail().x;
    const ImVec2 at = ImGui::GetCursorScreenPos();
    if (ImGui::InvisibleButton("##hdr", ImVec2(w, h))) { open = !open; store->SetBool(key, open); }
    const bool hovered = ImGui::IsItemHovered();

    ImDrawList* dl = ImGui::GetWindowDrawList();
    drawGlyph(dl, open ? Glyph::ChevronDown : Glyph::ChevronRight, ImVec2(at.x + 7.0f, at.y + h * 0.5f),
              14.0f, u32(hovered ? palette::kText : palette::kTextDim));
    pushFont(FontWeight::SemiBold, uiFonts().size * 0.95f);
    dl->AddText(ImVec2(at.x + 18.0f, at.y + (h - ImGui::GetTextLineHeight()) * 0.5f),
                u32(palette::kText), name);
    const float nameW = ImGui::CalcTextSize(name).x;
    ImGui::PopFont();
    if (count > 0) {
        char n[16];
        std::snprintf(n, sizeof n, "%zu", count);
        pushFont(FontWeight::Regular, uiFonts().size * 0.82f);
        dl->AddText(ImVec2(at.x + 18.0f + nameW + 8.0f, at.y + (h - ImGui::GetTextLineHeight()) * 0.5f + 1.0f),
                    u32(palette::kTextFaint), n);
        ImGui::PopFont();
    }
    ImGui::PopID();
    return open;
}

void objectRow(UiContext& ctx, Scene& scene, SceneObject& obj, Glyph glyph) {
    ImGui::PushID(static_cast<int>(obj.id));
    const float h = 26.0f;
    const float eyeW = 22.0f;
    const float w = ImGui::GetContentRegionAvail().x;
    const ImVec2 at = ImGui::GetCursorScreenPos();

    const bool clicked = ImGui::InvisibleButton("##row", ImVec2(std::max(10.0f, w - eyeW - 4.0f), h));
    const bool hovered = ImGui::IsItemHovered();
    if (clicked) {
        // The application does it, the same way it does a Ctrl+click on the
        // body in the view, so the two cannot come to mean different things.
        ctx.actions.pickObject = obj.id;
        ctx.actions.pickObjectAdditive = ImGui::GetIO().KeyShift || ImGui::GetIO().KeyCtrl;
    }
    if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) ctx.actions.frameSelected = true;

    const bool selected = scene.isSelected(obj.id);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 lo(at.x, at.y), hi(at.x + w, at.y + h);
    if (selected) {
        dl->AddRectFilled(lo, hi, u32(palette::kRaised), 5.0f);
        dl->AddRectFilled(ImVec2(lo.x, lo.y + 5.0f), ImVec2(lo.x + 2.5f, hi.y - 5.0f), u32(palette::kBrand), 2.0f);
    } else if (hovered) {
        dl->AddRectFilled(lo, hi, u32(palette::kHover, 0.5f), 5.0f);
    }

    const float alpha = obj.visible ? 1.0f : 0.45f;
    drawGlyph(dl, glyph, ImVec2(at.x + 18.0f, at.y + h * 0.5f), 16.0f, u32(palette::kBrand, alpha));
    pushFont(selected ? FontWeight::Medium : FontWeight::Regular);
    dl->AddText(ImVec2(at.x + 34.0f, at.y + (h - ImGui::GetTextLineHeight()) * 0.5f),
                u32(palette::kText, alpha), obj.name.c_str());
    ImGui::PopFont();

    ImGui::SameLine(w - eyeW + 2.0f);
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + (h - (15.0f + 6.0f)) * 0.5f);
    bool visible = obj.visible;
    if (ui::eyeToggle("vis", visible, 15.0f)) obj.visible = visible;

    // The sketches it holds, under it: each with its own eye, and a
    // double-click to go back into it. A sketch is a thing in the model that
    // outlives being extruded, so it belongs here and not only in the history.
    if (glyph != Glyph::Sketch) {
        for (Feature& f : obj.features) {
            if (f.kind != FeatureKind::Sketch) continue;
            ImGui::PushID(static_cast<int>(f.uid));
            const ImVec2 sat = ImGui::GetCursorScreenPos();
            const float sh = 22.0f;
            const bool sclicked = ImGui::InvisibleButton("##sk", ImVec2(std::max(10.0f, w - eyeW - 4.0f), sh));
            const bool shover = ImGui::IsItemHovered();
            if (shover) dl->AddRectFilled(ImVec2(sat.x, sat.y), ImVec2(sat.x + w, sat.y + sh),
                                          u32(palette::kHover, 0.5f), 5.0f);
            if (sclicked && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                ctx.actions.editSketchObject = obj.id;
                ctx.actions.editSketchUid = f.uid;
            }
            if (shover)
                ImGui::SetTooltip("Double-click to edit it%s",
                                  f.sketchFreedoms == 0 ? "  (fully constrained)" : "");
            const float salpha = f.sketchShown ? 0.85f : 0.4f;
            drawGlyph(dl, Glyph::Sketch, ImVec2(sat.x + 36.0f, sat.y + sh * 0.5f), 15.0f,
                      u32(palette::kBrand, salpha));
            char label[64];
            std::snprintf(label, sizeof label, "Sketch  %zu entit%s", f.sketch.entities.size(),
                          f.sketch.entities.size() == 1 ? "y" : "ies");
            pushFont(FontWeight::Regular, uiFonts().size * 0.92f);
            dl->AddText(ImVec2(sat.x + 50.0f, sat.y + (sh - ImGui::GetTextLineHeight()) * 0.5f),
                        u32(palette::kTextDim, f.sketchShown ? 1.0f : 0.6f), label);
            ImGui::PopFont();
            ImGui::SameLine(w - eyeW + 2.0f);
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + (sh - (13.0f + 6.0f)) * 0.5f);
            bool shown = f.sketchShown;
            // Display only, so the chain is not re-run: nothing it builds
            // depends on whether the drawing is on the screen.
            if (ui::eyeToggle("shown", shown, 13.0f)) f.sketchShown = shown;
            ImGui::PopID();
        }
    }
    ImGui::PopID();
}

// ---- history ------------------------------------------------------------------

// The editable parameters of one step, under its row.
void featureDetails(UiContext& ctx, SceneObject& obj, Feature& f, bool& changed) {
    using ui::labelledInt;
    using ui::labelledNumber;
    const ImVec4 dim = im(palette::kTextDim);
    switch (f.kind) {
    case FeatureKind::Primitive:
        ImGui::TextColored(dim, "Its dimensions are under Shape, above");
        break;
    case FeatureKind::Merge:
        ImGui::TextColored(dim, "Drops every division that does not define the shape");
        break;
    case FeatureKind::FaceScale: {
        Real pct = (f.scale - 1.0) * 100.0;
        if (labelledNumber("Change", pct, 0.5f, -95.0f, 1000.0f, "%.1f %%")) {
            f.scale = 1.0 + pct / 100.0;
            changed = true;
        }
        ImGui::TextColored(dim, "%.3g x its size", f.scale);
        break;
    }
    case FeatureKind::FaceRotate: {
        Real deg = degrees(f.angle);
        if (labelledNumber("Angle", deg, 0.2f, -89.0f, 89.0f, "%.1f deg")) {
            f.angle = radians(deg);
            changed = true;
        }
        ImGui::TextColored(dim, "about %.2f, %.2f, %.2f", f.axisPoint.x, f.axisPoint.y, f.axisPoint.z);
        break;
    }
    case FeatureKind::Reduce: {
        // Applied when a field is finished with, not on every frame of a
        // drag: a reduction takes seconds on a large mesh.
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(dim, "Within");
        ImGui::SameLine(ui::labelColumn());
        ImGui::SetNextItemWidth(-1.0f);
        double tol = f.reduceTolerance;
        ImGui::InputDouble("##rtol", &tol, 0.0, 0.0, "%.3f mm");
        if (ImGui::IsItemDeactivatedAfterEdit() && tol > 0.0 && tol != f.reduceTolerance) {
            f.reduceTolerance = tol;
            changed = true;
        }
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(dim, "Stop at");
        ImGui::SameLine(ui::labelColumn());
        ImGui::SetNextItemWidth(-1.0f);
        int target = f.reduceTarget;
        ImGui::InputInt("##rtarget", &target, 0, 0);
        if (ImGui::IsItemDeactivatedAfterEdit() && target >= 0 && target != f.reduceTarget) {
            f.reduceTarget = target;
            changed = true;
        }
        bool loosen = f.reduceLoosen;
        if (f.reduceTarget > 0 && ImGui::Checkbox("Loosen to reach it", &loosen)) {
            f.reduceLoosen = loosen;
            changed = true;
        }
        ImGui::TextColored(dim, "%s", f.reduceTarget == 0
                                          ? "0: as few triangles as the tolerance allows"
                                          : f.reduceLoosen
                                          ? "the tolerance loosens only as far as the count needs"
                                          : "triangles, or the tolerance, whichever comes first");
        break;
    }
    case FeatureKind::Sketch: {
        // The drawing itself is edited where it was drawn, on its plane.
        if (ui::quietButton("Edit Sketch")) {
            ctx.actions.editSketchObject = obj.id;
            ctx.actions.editSketchUid = f.uid;
        }
        // The numbers that size it. Changing one re-solves the sketch and
        // re-runs everything built from it -- the reason a sketch is kept in
        // the history rather than consumed.
        bool hasDimensions = false;
        for (SketchConstraint& k : f.sketch.constraints) {
            if (!isDimension(k.rule)) continue;
            hasDimensions = true;
            ImGui::PushID(static_cast<int>(k.id));
            char label[32];
            if (k.rule == SketchRule::Angle) {
                std::snprintf(label, sizeof label, "Angle #%u", k.id);
                Real deg = degrees(k.value);
                if (labelledNumber(label, deg, 0.2f, 0.0f, 360.0f, "%.1f deg")) {
                    k.value = radians(deg);
                    changed = true;
                }
            } else {
                std::snprintf(label, sizeof label, "%s #%u",
                              k.rule == SketchRule::Radius ? "Radius" : "Distance", k.id);
                changed |= labelledNumber(label, k.value, 0.1f, 0.001f, 100000.0f);
            }
            ImGui::PopID();
        }
        if (!hasDimensions) ImGui::TextColored(dim, "No dimensions: its size is what was drawn");
        ImGui::TextColored(f.sketchFreedoms == 0 ? im(palette::kValid) : im(palette::kInfo), "%s",
                           f.sketchFreedoms == 0 ? "fully constrained"
                                                 : "not fully constrained: some of it can still move");
        break;
    }
    case FeatureKind::Draft: {
        double deg = f.angle * kRad2Deg;
        if (labelledNumber("Angle", deg, 0.1f, -80.0f, 80.0f)) {
            f.angle = deg * kDeg2Rad;
            changed = true;
        }
        // Which way the part is pulled, and where it is widest. Both are
        // geometry rather than a choice of words, so the row says them.
        const Vec3 d = f.axisDir;
        ImGui::TextColored(dim, "along %s, same size at %.2f", std::fabs(d.x) > 0.9 ? "X"
                                                          : std::fabs(d.y) > 0.9 ? "Y" : "Z",
                           dot(f.axisPoint, normalize(d)));
        ImGui::TextColored(dim, "%s", f.faces.describe("face").c_str());
        break;
    }
    case FeatureKind::Hole: {
        // What it is, and what that came to. The size and the fit are the
        // choice; the diameter is what the table made of it, and typing over
        // it is what "no fastener" means.
        if (f.holeFastener >= 0) {
            char what[64];
            std::snprintf(what, sizeof what, "%s %s", fastenerAt(f.holeFastener).name,
                          holeFitName(f.holeFit));
            ui::commandValue("Size", what);
        }
        double dia = f.hole.diameter;
        if (labelledNumber("Diameter", dia, 0.05f, 0.1f, 200.0f)) {
            f.hole.diameter = dia;
            f.holeFastener = -1;          // typed over: it is that size now
            changed = true;
        }
        if (!f.hole.through) {
            double deep = f.hole.depth;
            if (labelledNumber("Depth", deep, 0.1f, 0.2f, 1000.0f)) {
                f.hole.depth = deep;
                changed = true;
            }
        }
        if (f.hole.kind == HoleKind::Counterbore || f.hole.kind == HoleKind::Countersink) {
            double head = f.hole.headDiameter;
            if (labelledNumber(f.hole.kind == HoleKind::Counterbore ? "Pocket" : "Head", head,
                               0.05f, 0.1f, 400.0f)) {
                f.hole.headDiameter = head;
                f.holeFastener = -1;
                changed = true;
            }
        }
        if (f.hole.kind == HoleKind::Counterbore) {
            double deep = f.hole.headDepth;
            if (labelledNumber("Pocket depth", deep, 0.05f, 0.1f, 400.0f)) {
                f.hole.headDepth = deep;
                changed = true;
            }
        }
        ImGui::TextColored(dim, "%s, %s", holeKindName(f.hole.kind),
                           f.hole.through ? "through" : "to a depth");
        break;
    }
    case FeatureKind::RevolveProfile: {
        double deg = f.revolveAngle * kRad2Deg;
        if (labelledNumber("Angle", deg, 1.0f, 1.0f, 360.0f)) {
            f.revolveAngle = clampf(deg, 1.0, 360.0) * kDeg2Rad;
            changed = true;
        }
        static const char* const kOps[] = {"Join", "Cut", "Intersect"};
        static const ExtrudeOp kOf[] = {ExtrudeOp::Join, ExtrudeOp::Cut, ExtrudeOp::Intersect};
        const ExtrudeOp shown = f.extrudeOp == ExtrudeOp::Auto ? ExtrudeOp::Join : f.extrudeOp;
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(dim, "Operation");
        ImGui::SameLine(ui::labelColumn());
        for (int i = 0; i < 3; ++i) {
            if (i) ImGui::SameLine(0.0f, 3.0f);
            if (ui::pillButton(kOps[i], shown == kOf[i]) && f.extrudeOp != kOf[i]) {
                f.extrudeOp = kOf[i];
                changed = true;
            }
        }
        // The axis is picked by pointing at the drawing, not typed here: two
        // numbers and a direction in the sketch's own frame are not something
        // anyone can read off and check. What the step can say is where it is.
        ImGui::TextColored(dim, "turns about the line through (%.2f, %.2f) in the sketch",
                           f.revolveAxisAt.x, f.revolveAxisAt.y);
        break;
    }
    case FeatureKind::ExtrudeProfile:
    case FeatureKind::Extrude: {
        changed |= labelledNumber("Distance", f.distance, 0.1f, -10000.0f, 10000.0f);
        // Three ways to combine, and no "Auto" to pick: a step written before
        // that was dropped shows the one its direction made it.
        static const char* const kOps[] = {"Join", "Cut", "Intersect"};
        static const ExtrudeOp kOf[] = {ExtrudeOp::Join, ExtrudeOp::Cut, ExtrudeOp::Intersect};
        const ExtrudeOp shown = f.extrudeOp == ExtrudeOp::Auto
                                    ? (f.distance < 0.0 ? ExtrudeOp::Cut : ExtrudeOp::Join)
                                    : f.extrudeOp;
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(dim, "Operation");
        ImGui::SameLine(ui::labelColumn());
        for (int i = 0; i < 3; ++i) {
            if (i) ImGui::SameLine(0.0f, 3.0f);
            if (ui::pillButton(kOps[i], shown == kOf[i]) && f.extrudeOp != kOf[i]) {
                f.extrudeOp = kOf[i];
                changed = true;
            }
        }
        if (f.kind == FeatureKind::Extrude) ImGui::TextColored(dim, "%s", f.faces.describe("face").c_str());
        break;
    }
    case FeatureKind::Pattern: {
        static const char* const kLayouts[] = {"Row", "Ring", "Mirror"};
        int layout = static_cast<int>(f.patternMode);
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(dim, "Layout");
        ImGui::SameLine(ui::labelColumn());
        for (int i = 0; i < 3; ++i) {
            if (i) ImGui::SameLine(0.0f, 3.0f);
            if (ui::pillButton(kLayouts[i], layout == i) && layout != i) {
                f.patternMode = static_cast<PatternMode>(i);
                changed = true;
            }
        }
        if (f.patternMode != PatternMode::Mirror) {
            int n = f.patternCount;
            if (labelledInt("Copies", n, 2, 256)) { f.patternCount = n; changed = true; }
            if (f.patternMode == PatternMode::Linear) {
                changed |= labelledNumber("Spacing", f.distance, 0.1f, 0.05f, 10000.0f, "%.2f mm");
            } else {
                Real deg = degrees(f.angle);
                if (labelledNumber("Turn", deg, 0.25f, -360.0f, 360.0f, "%.1f deg")) {
                    f.angle = radians(deg);
                    changed = true;
                }
                ImGui::SameLine();
                if (ui::pillButton("Full turn", false)) {
                    f.angle = radians(360.0 / (f.patternCount < 2 ? 2 : f.patternCount));
                    changed = true;
                }
            }
        }
        // Which way it goes, as the axis it was built on rather than as
        // three numbers: a pattern is nearly always along one of them.
        static const char* const kAxes[] = {"X", "Y", "Z"};
        int axis = std::fabs(f.axisDir.x) >= std::fabs(f.axisDir.y) &&
                           std::fabs(f.axisDir.x) >= std::fabs(f.axisDir.z) ? 0
                 : std::fabs(f.axisDir.y) >= std::fabs(f.axisDir.z)         ? 1 : 2;
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(dim, "%s", f.patternMode == PatternMode::Mirror ? "Plane" : "Axis");
        ImGui::SameLine(ui::labelColumn());
        for (int i = 0; i < 3; ++i) {
            if (i) ImGui::SameLine(0.0f, 3.0f);
            if (ui::pillButton(kAxes[i], axis == i) && axis != i) {
                f.axisDir = Vec3{i == 0 ? 1.0 : 0.0, i == 1 ? 1.0 : 0.0, i == 2 ? 1.0 : 0.0};
                changed = true;
            }
        }
        ImGui::TextColored(dim, "%s", f.bakedBody.empty() ? "Repeats the body" : "Repeats the cut it replaced");
        break;
    }
    case FeatureKind::Divide:
        ImGui::TextColored(dim, "Cuts at %.2f, %.2f, %.2f", f.axisPoint.x, f.axisPoint.y, f.axisPoint.z);
        ImGui::TextColored(dim, "The body stays whole; the faces divide");
        break;
    case FeatureKind::Inset:
        changed |= labelledNumber("Amount", f.amount, 0.05f, 0.01f, 10000.0f);
        break;
    case FeatureKind::Shell:
        changed |= labelledNumber("Wall", f.thickness, 0.05f, 0.01f, 10000.0f);
        ImGui::TextColored(dim, "%s", f.faces.empty() ? "sealed: no face left open"
                                                      : f.faces.describe("face").c_str());
        break;
    case FeatureKind::Bevel: {
        // A segment count is a mesh idea: an exact fillet is a surface rather
        // than an approximation of one. Shown only where it still does something.
        const bool isMesh = obj.body.isMesh();
        if (labelledNumber(f.chamfer ? "Distance" : "Radius", f.width, 0.05f, 0.01f, 10000.0f)) {
            // Editing the feature radius restates every edge's, which is
            // what a user dragging one number expects.
            f.radii.assign(f.edges.count(), f.width);
            changed = true;
        }
        if (isMesh) changed |= labelledInt("Segments", f.segments, 1, 32);
        ImGui::TextColored(dim, "%s", f.edges.describe("edge").c_str());
        break;
    }
    case FeatureKind::Boolean: {
        // The tool body is baked into the feature, so its shape is not
        // editable here -- but which way it combines is.
        static const char* const kOps[] = {"Join", "Cut", "Intersect"};
        int op = static_cast<int>(f.booleanOp);
        if (op < 0 || op > 2) op = 0;
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(dim, "Operation");
        ImGui::SameLine(ui::labelColumn());
        for (int i = 0; i < 3; ++i) {
            if (i) ImGui::SameLine(0.0f, 3.0f);
            if (ui::pillButton(kOps[i], op == i) && op != i) {
                f.booleanOp = static_cast<BooleanOp>(i);
                changed = true;
            }
        }
        ImGui::TextColored(dim, "Tool body: %d faces", f.bakedBody.faceCount());
        break;
    }
    case FeatureKind::BaseMesh:
        ImGui::TextColored(dim, "Imported geometry, %d faces", f.bakedBody.faceCount());
        ImGui::TextColored(dim, "Not parametric: convert it to a solid to edit it");
        break;
    case FeatureKind::VertexEdit:
        ImGui::TextColored(dim, "Free-form edit of %zu vertices", f.verts.size());
        break;
    case FeatureKind::Move:
        changed |= labelledNumber("X", f.moveBy.x, 0.1f, -1e6, 1e6);
        changed |= labelledNumber("Y", f.moveBy.y, 0.1f, -1e6, 1e6);
        changed |= labelledNumber("Z", f.moveBy.z, 0.1f, -1e6, 1e6);
        ImGui::TextColored(dim, "Where the object went, in the world");
        break;
    case FeatureKind::Rotate: {
        // Shown as the three angles the Transform fields use.
        Vec3 e = toEuler(f.turnBy);
        e = {degrees(e.x), degrees(e.y), degrees(e.z)};
        bool turned = labelledNumber("X", e.x, 0.5f, -360.0, 360.0, "%.1f deg");
        turned |= labelledNumber("Y", e.y, 0.5f, -360.0, 360.0, "%.1f deg");
        turned |= labelledNumber("Z", e.z, 0.5f, -360.0, 360.0, "%.1f deg");
        if (turned) {
            f.turnBy = normalize(Quat::fromEuler({radians(e.x), radians(e.y), radians(e.z)}));
            changed = true;
        }
        ImGui::TextColored(dim, "about %.2f, %.2f, %.2f", f.turnAbout.x, f.turnAbout.y, f.turnAbout.z);
        break;
    }
    case FeatureKind::Scale:
        changed |= labelledNumber("X", f.scaleBy.x, 0.01f, 0.001, 1000.0, "%.3f x");
        changed |= labelledNumber("Y", f.scaleBy.y, 0.01f, 0.001, 1000.0, "%.3f x");
        changed |= labelledNumber("Z", f.scaleBy.z, 0.01f, 0.001, 1000.0, "%.3f x");
        if (length(f.scaleAbout) < 1e-9)
            ImGui::TextColored(dim, "Along the body's own axes, about its origin");
        else
            ImGui::TextColored(dim, "Along its own axes, about %.2f, %.2f, %.2f", f.scaleAbout.x,
                               f.scaleAbout.y, f.scaleAbout.z);
        break;
    }
}

void drawHistorySection(UiContext& ctx, SceneObject& obj) {
    // What the chain was, for the undo entry an edit here would make. Taken
    // only when an edit could begin -- the pointer is over this panel, or a
    // field in it is being dragged -- because a step can hold a whole imported
    // mesh or a sketch of thousands of curves, and copying that every frame
    // was a millisecond a frame of nothing.
    const bool couldEdit = ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows |
                                                  ImGuiHoveredFlags_AllowWhenBlockedByActiveItem) ||
                           ImGui::IsAnyItemActive();
    std::vector<Feature> before;
    if (couldEdit) before = obj.features;
    bool changed = false;
    ImGuiStorage* store = ImGui::GetStateStorage();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    // The last step is where the model stands: it is what the viewport shows
    // and what the next operation builds on.
    size_t current = obj.features.empty() ? 0 : obj.features.size() - 1;
    for (size_t i = obj.features.size(); i-- > 0;) {
        if (obj.features[i].enabled) { current = i; break; }
    }

    for (size_t i = 0; i < obj.features.size(); ++i) {
        Feature& f = obj.features[i];
        ImGui::PushID(static_cast<int>(f.uid ? f.uid : i + 1));
        const ImGuiID openKey = ImGui::GetID("open");
        bool open = store->GetBool(openKey, false);

        const float h = 26.0f;
        const float w = ImGui::GetContentRegionAvail().x;
        const float rightW = 46.0f;      // the enable dot and the close
        const ImVec2 at = ImGui::GetCursorScreenPos();
        const bool clicked = ImGui::InvisibleButton("##row", ImVec2(std::max(10.0f, w - rightW), h));
        const bool hovered = ImGui::IsItemHovered();
        if (clicked) { open = !open; store->SetBool(openKey, open); }

        const bool isCurrent = i == current;
        const ImVec2 lo(at.x, at.y), hi(at.x + w, at.y + h);
        if (isCurrent)     dl->AddRectFilled(lo, hi, u32(palette::kRaised), 5.0f);
        else if (hovered)  dl->AddRectFilled(lo, hi, u32(palette::kHover, 0.5f), 5.0f);

        const float alpha = f.enabled ? 1.0f : 0.45f;
        drawGlyph(dl, open ? Glyph::ChevronDown : Glyph::ChevronRight, ImVec2(at.x + 8.0f, at.y + h * 0.5f),
                  13.0f, u32(palette::kTextFaint));
        drawGlyph(dl, glyphFor(f), ImVec2(at.x + 24.0f, at.y + h * 0.5f), 15.0f,
                  u32(f.errored ? palette::kBrand : palette::kTextDim, alpha));

        // The name, and what is special about it beside the name. Clipped
        // short of the row's own controls: a long summary ends under them
        // rather than running through them.
        const float textRight = hi.x - rightW - 4.0f;
        const std::string summary = f.summary();
        dl->PushClipRect(ImVec2(at.x, at.y), ImVec2(textRight, hi.y), true);
        pushFont(isCurrent ? FontWeight::Medium : FontWeight::Regular);
        dl->AddText(ImVec2(at.x + 40.0f, at.y + (h - ImGui::GetTextLineHeight()) * 0.5f),
                    u32(palette::kText, alpha), summary.c_str());
        const float textW = ImGui::CalcTextSize(summary.c_str()).x;
        ImGui::PopFont();
        dl->PopClipRect();

        const float tagX = at.x + 40.0f + textW + 8.0f;
        auto rowTag = [&](const char* text, Rgb col) {
            pushFont(FontWeight::Medium, uiFonts().size * 0.8f);
            const float need = ImGui::CalcTextSize(text).x + 12.0f;
            ImGui::PopFont();
            if (tagX + need > textRight) return;      // no room: the row says it by its look
            ImGui::SetCursorScreenPos(ImVec2(tagX, at.y + (h - ImGui::GetFrameHeight()) * 0.5f));
            ui::tag(text, col, false);
        };
        if (f.errored)       rowTag("failed", palette::kBrand);
        else if (!f.enabled) rowTag("off", palette::kTextDim);
        else if (isCurrent)  rowTag("current", palette::kTextDim);
        if (hovered) {
            if (f.errored) ImGui::SetTooltip("%s", f.error.c_str());
            else if (textW > textRight - (at.x + 40.0f)) ImGui::SetTooltip("%s", summary.c_str());
        }

        // The enable dot, then the close. Both live at the right of the row
        // and show up when it is being looked at.
        if (hovered || isCurrent || !f.enabled || ImGui::IsMouseHoveringRect(ImVec2(hi.x - rightW, lo.y), hi)) {
            ImGui::SetCursorScreenPos(ImVec2(hi.x - rightW + 2.0f, at.y + 3.0f));
            const bool tog = ImGui::InvisibleButton("##on", ImVec2(20.0f, 20.0f));
            const ImVec2 c(hi.x - rightW + 12.0f, at.y + h * 0.5f);
            if (f.enabled) dl->AddCircleFilled(c, 4.0f, u32(palette::kValid));
            else           dl->AddCircle(c, 4.0f, u32(palette::kTextDim), 0, 1.4f);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", f.enabled ? "On. Click to skip this step." : "Skipped. Click to run it.");
            if (tog) { f.enabled = !f.enabled; changed = true; }

            // The base primitive is what the chain starts from, so it cannot
            // be removed without leaving the rest with nothing to act on.
            if (f.kind != FeatureKind::Primitive) {
                ImGui::SetCursorScreenPos(ImVec2(hi.x - 22.0f, at.y + (h - 19.0f) * 0.5f));
                if (ui::closeButton("del", 14.0f)) {
                    obj.features.erase(obj.features.begin() + static_cast<long>(i));
                    changed = true;
                    ImGui::PopID();
                    break;
                }
                ui::hoverTip("Remove this step");
            }
        }
        ImGui::SetCursorScreenPos(ImVec2(at.x, at.y + h + 2.0f));

        if (open) {
            ImGui::Indent(14.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 4.0f));
            featureDetails(ctx, obj, f, changed);
            ImGui::PopStyleVar();
            ImGui::Unindent(14.0f);
            ImGui::Dummy(ImVec2(0, 4));
        }
        ImGui::PopID();
    }

    if (changed) {
        ctx.actions.featuresEdited = obj.id;
        ctx.actions.featuresBefore = std::move(before);
    }
}

} // namespace

// -----------------------------------------------------------------------------
void drawAddMenuItems(UiContext& ctx) {
    struct Entry { Glyph glyph; const char* label; PrimitiveKind kind; };
    static const Entry kEntries[] = {
        {Glyph::Box,      "Box",      PrimitiveKind::Box},
        {Glyph::Cylinder, "Cylinder", PrimitiveKind::Cylinder},
        {Glyph::Sphere,   "Sphere",   PrimitiveKind::Sphere},
        {Glyph::Cone,     "Cone",     PrimitiveKind::Cone},
        {Glyph::Torus,    "Torus",    PrimitiveKind::Torus},
        {Glyph::Plane,    "Plane",    PrimitiveKind::Plane},
    };
    for (const Entry& e : kEntries) {
        if (ui::menuEntry(e.glyph, e.label)) {
            ctx.actions.addRequested = true;
            ctx.actions.addKind = e.kind;
        }
    }
    ui::menuGap();
    if (ui::menuEntry(Glyph::Sketch, "Sketch", "Shift+S", brep::available())) ctx.actions.sketch = true;
    ui::menuNote(brep::available() ? "lines, circles and arcs, kept and sized"
                                   : "needs the exact kernel");
}

// -----------------------------------------------------------------------------
void drawOutliner(UiContext& ctx) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8.0f, 10.0f));
    if (!ImGui::Begin("Outliner##v2", nullptr, ImGuiWindowFlags_NoCollapse)) {
        ImGui::End();
        ImGui::PopStyleVar();
        return;
    }
    ImGui::PopStyleVar();
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f, 2.0f));

    Scene& scene = *ctx.scene;
    if (scene.objectCount() == 0) {
        ImGui::Dummy(ImVec2(0, 6));
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 8.0f);
        ImGui::TextColored(im(palette::kTextDim), "Nothing here yet");
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 8.0f);
        pushFont(FontWeight::Regular, uiFonts().size * 0.88f);
        ImGui::TextColored(im(palette::kTextFaint), "Shift+A adds a shape");
        ImGui::PopFont();
    }

    struct Section { const char* name; ObjectKind kind; };
    static const Section kSections[3] = {
        {"Bodies", ObjectKind::Body}, {"Sketches", ObjectKind::Sketch}, {"Meshes", ObjectKind::Mesh}};
    for (const Section& s : kSections) {
        if (scene.objectCount() == 0) break;
        std::vector<SceneObject*> members;
        for (const auto& obj : scene.objects())
            if (kindOf(*obj) == s.kind) members.push_back(obj.get());
        // A heading over nothing is a promise of something that is not there.
        // The section appears when the first one of its kind does.
        if (members.empty()) continue;
        if (!sectionHeader(s.name, members.size())) continue;
        for (SceneObject* obj : members) objectRow(ctx, scene, *obj, glyphFor(s.kind));
        ImGui::Dummy(ImVec2(0, 6));
    }

    ImGui::PopStyleVar();
    ImGui::End();
}

// -----------------------------------------------------------------------------
void drawInspector(UiContext& ctx) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 10.0f));
    if (!ImGui::Begin("Inspector##v2", nullptr, ImGuiWindowFlags_NoCollapse)) {
        ImGui::End();
        ImGui::PopStyleVar();
        return;
    }
    ImGui::PopStyleVar();

    Scene& scene = *ctx.scene;
    SceneObject* obj = scene.find(scene.contextObject());
    if (!obj) {
        ImGui::Dummy(ImVec2(0, 6));
        ImGui::TextColored(im(palette::kTextDim), "Nothing selected");
        pushFont(FontWeight::Regular, uiFonts().size * 0.88f);
        ImGui::TextColored(im(palette::kTextFaint), "Click a body, or a row on the left");
        ImGui::PopFont();
        ImGui::End();
        return;
    }

    // Snapshot before any widget runs, so a changed value can be paired with
    // what it replaced and pushed onto the undo stack.
    const Transform transformBefore = obj->transform;
    const PrimitiveSpec specBefore = obj->spec;

    // ---- the name, with what it is beside it -------------------------------
    {
        const float box = ImGui::GetFrameHeight();
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const ImVec2 at = ImGui::GetCursorScreenPos();
        dl->AddRectFilled(at, ImVec2(at.x + box, at.y + box), u32(palette::kBrand), 6.0f);
        drawGlyph(dl, glyphFor(kindOf(*obj)), ImVec2(at.x + box * 0.5f, at.y + box * 0.5f), box * 0.7f,
                  IM_COL32(255, 255, 255, 255));
        ImGui::Dummy(ImVec2(box, box));
        ImGui::SameLine(0.0f, 8.0f);
        char nameBuf[128];
        std::snprintf(nameBuf, sizeof nameBuf, "%s", obj->name.c_str());
        ImGui::SetNextItemWidth(-1.0f);
        pushFont(FontWeight::SemiBold);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, im(palette::kField));
        if (ImGui::InputText("##name", nameBuf, sizeof nameBuf)) obj->name = nameBuf;
        ImGui::PopStyleColor();
        ImGui::PopFont();
    }

    // The body of the panel scrolls; the numbers at the foot do not.
    const float footerH = 46.0f;
    ImGui::BeginChild("##inspectorbody", ImVec2(0.0f, -footerH), ImGuiChildFlags_None,
                      ImGuiWindowFlags_NoBackground);

    // ---- transform -----------------------------------------------------------
    {
        // The title, with a way back to where it started small at its right.
        ImGui::Dummy(ImVec2(0, 4));
        pushFont(FontWeight::SemiBold, uiFonts().size);
        ImGui::TextColored(im(palette::kText), "Transform");
        ImGui::PopFont();
        pushFont(FontWeight::Regular, uiFonts().size * 0.82f);
        const float w = ImGui::CalcTextSize("Reset").x + 12.0f;
        ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - w);
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
        ImGui::PushStyleColor(ImGuiCol_Text, im(palette::kTextDim));
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(6.0f, 1.0f));
        if (ImGui::Button("Reset")) ctx.actions.resetTransform = obj->id;
        ImGui::PopStyleVar();
        ImGui::PopStyleColor(2);
        ImGui::PopFont();
        ui::hoverTip("Back to the origin, unturned, at full size");
        ImGui::Dummy(ImVec2(0, 1));
    }
    ui::fieldHeader("Position", "mm");
    ui::axisFields("pos", obj->transform.position, 0.1f, "%.1f");

    // Euler angles are a display convention only; the transform stores a
    // quaternion, so the conversion round-trips through it on every edit.
    ui::fieldHeader("Rotation", kDegree);
    Vec3 euler = toEuler(obj->transform.rotation);
    euler = {degrees(euler.x), degrees(euler.y), degrees(euler.z)};
    if (ui::axisFields("rot", euler, 0.5f, "%.1f")) {
        obj->transform.rotation = normalize(Quat::fromEuler(
            {radians(euler.x), radians(euler.y), radians(euler.z)}));
    }
    // How much the history has scaled the body, each Scale step multiplied
    // in. A drag is shown through the transform and becomes a Scale step when
    // it is let go of: the shape itself changes, which is too much work to do
    // on every frame of a drag.
    ui::fieldHeader("Scale", "x");
    {
        const Vec3 made = scaleOf(obj->features);
        const Vec3& live = obj->transform.scale;
        Vec3 shown{made.x * live.x, made.y * live.y, made.z * live.z};
        if (ui::axisFields("scale", shown, 0.01f, "%.2f")) {
            for (int i = 0; i < 3; ++i)
                obj->transform.scale[i] = std::max(shown[i], Real(1e-4)) / made[i];
        }
    }

    ui::fieldHeader("Bounds", "mm");
    {
        const AABB b = obj->localBounds;
        Vec3 size = b.valid() ? b.size() : Vec3{};
        size = {size.x * obj->transform.scale.x, size.y * obj->transform.scale.y,
                size.z * obj->transform.scale.z};
        ui::axisFields("bounds", size, 0.0f, "%.1f", /*readOnly=*/true);
    }

    // Any transform field that moved becomes one undo entry per drag.
    const Transform& tNow = obj->transform;
    if (tNow.position != transformBefore.position ||
        tNow.scale != transformBefore.scale ||
        tNow.rotation.x != transformBefore.rotation.x ||
        tNow.rotation.y != transformBefore.rotation.y ||
        tNow.rotation.z != transformBefore.rotation.z ||
        tNow.rotation.w != transformBefore.rotation.w) {
        ctx.actions.transformEdited = obj->id;
        ctx.actions.transformBefore = transformBefore;
    }

    // ---- shape -----------------------------------------------------------------
    const bool sketched = !obj->features.empty() && obj->features.front().kind == FeatureKind::Sketch;
    const bool imported = !obj->features.empty() && obj->features.front().kind == FeatureKind::BaseMesh;
    if (!sketched && !imported) {
        ui::sectionTitle("Shape");
        if (drawPrimitiveParams(*obj)) {
            ctx.actions.rebuildObject = obj->id;
            ctx.actions.specBefore = specBefore;
        }
    }

    // ---- history -----------------------------------------------------------------
    ui::sectionTitle("History");
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f, 2.0f));
    drawHistorySection(ctx, *obj);
    ImGui::PopStyleVar();
    ImGui::Dummy(ImVec2(0, 8));
    ImGui::EndChild();

    // ---- the numbers, at the foot ------------------------------------------------
    {
        const ImVec2 at = ImGui::GetCursorScreenPos();
        const float w = ImGui::GetContentRegionAvail().x;
        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->AddLine(ImVec2(at.x, at.y), ImVec2(at.x + w, at.y), u32(palette::kBorder));

        struct Stat { const char* label; char value[32]; };
        Stat stats[3] = {{"Volume", ""}, {"Vertices", ""}, {"Faces", ""}};
        if (obj->healthVersion == obj->geometryVersion)
            std::snprintf(stats[0].value, sizeof stats[0].value, "%.1f cm\xC2\xB3", obj->health.volume / 1000.0);
        else
            std::snprintf(stats[0].value, sizeof stats[0].value, "...");
        std::snprintf(stats[1].value, sizeof stats[1].value, "%d", obj->body.vertexCount());
        std::snprintf(stats[2].value, sizeof stats[2].value, "%d", obj->body.faceCount());

        const float col = w / 3.0f;
        for (int i = 0; i < 3; ++i) {
            const float x = at.x + col * static_cast<float>(i);
            pushFont(FontWeight::Regular, uiFonts().size * 0.78f);
            dl->AddText(ImVec2(x, at.y + 8.0f), u32(palette::kTextDim), stats[i].label);
            ImGui::PopFont();
            pushFont(FontWeight::SemiBold, uiFonts().size * 0.95f);
            dl->AddText(ImVec2(x, at.y + 22.0f), u32(palette::kText), stats[i].value);
            ImGui::PopFont();
        }
        ImGui::Dummy(ImVec2(w, footerH));
    }

    ImGui::End();
}

// -----------------------------------------------------------------------------
void drawViewportOverlays(UiContext& ctx, float x, float y, float w, float h) {
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const ImVec2 origin = ImGui::GetMainViewport()->Pos;
    const float x0 = origin.x + x, y0 = origin.y + y;
    float nextY = y0 + 14.0f;

    // What the tool is doing, at the top left, where the eye starts.
    if (!ctx.toolStatus.empty()) {
        pushFont(FontWeight::Medium, uiFonts().size * 0.9f);
        const float maxW = std::max(200.0f, w * 0.6f);
        const ImVec2 ts = ImGui::CalcTextSize(ctx.toolStatus.c_str(), nullptr, false, maxW);
        const ImVec2 lo(x0 + 14.0f, nextY);
        const ImVec2 hi(lo.x + ts.x + 24.0f, lo.y + ts.y + 12.0f);
        dl->AddRectFilled(lo, hi, u32(palette::kCommand, 0.92f), 6.0f);
        dl->AddRectFilled(ImVec2(lo.x, lo.y + 6.0f), ImVec2(lo.x + 3.0f, hi.y - 6.0f), u32(palette::kBrand), 2.0f);
        dl->AddText(nullptr, 0.0f, ImVec2(lo.x + 14.0f, lo.y + 6.0f), u32(palette::kText),
                    ctx.toolStatus.c_str(), nullptr, maxW);
        ImGui::PopFont();
        nextY = hi.y + 8.0f;
    }

    // A notice: something refused, or something done that ought to be said.
    // It fades, because a message that stays is a message that stops being read.
    if (!ctx.notice.empty()) {
        const float alpha = std::clamp(1.0f - (ctx.noticeAge - 3.0f) / 0.8f, 0.0f, 1.0f);
        if (alpha > 0.0f) {
            pushFont(FontWeight::Medium, uiFonts().size * 0.92f);
            const float maxW = std::max(240.0f, w * 0.5f);
            const ImVec2 ts = ImGui::CalcTextSize(ctx.notice.c_str(), nullptr, false, maxW);
            const float bw = ts.x + 30.0f, bh = ts.y + 14.0f;
            const ImVec2 lo(x0 + (w - bw) * 0.5f, nextY);
            const ImVec2 hi(lo.x + bw, lo.y + bh);
            dl->AddRectFilled(lo, hi, u32(palette::kCommand, 0.96f * alpha), 8.0f);
            dl->AddRect(lo, hi, u32(palette::kBrand, 0.6f * alpha), 8.0f);
            dl->AddRectFilled(ImVec2(lo.x, lo.y + 7.0f), ImVec2(lo.x + 3.0f, hi.y - 7.0f),
                              u32(palette::kBrand, alpha), 2.0f);
            dl->AddText(nullptr, 0.0f, ImVec2(lo.x + 16.0f, lo.y + 7.0f), u32(palette::kText, alpha),
                        ctx.notice.c_str(), nullptr, maxW);
            ImGui::PopFont();
        }
    }

    // The scene's numbers, small, at the bottom right.
    {
        const Scene& scene = *ctx.scene;
        char line[160];
        int n = std::snprintf(line, sizeof line, "%zu object%s", scene.objectCount(),
                              scene.objectCount() == 1 ? "" : "s");
        if (!scene.elementSelection().empty()) {
            const ElementRef& e = scene.elementSelection().front();
            n += std::snprintf(line + n, sizeof line - n, "   %zu %s%s selected",
                               scene.elementSelection().size(), elementKindName(e.kind),
                               scene.elementSelection().size() == 1 ? "" : "s");
        } else if (!scene.selection().empty()) {
            n += std::snprintf(line + n, sizeof line - n, "   %zu selected", scene.selection().size());
        }
        n += std::snprintf(line + n, sizeof line - n, "   %zu tris", ctx.stats.triangles);
        const char* solid = nullptr;
        Rgb solidCol = palette::kTextFaint;
        if (const SceneObject* o = scene.find(scene.contextObject())) {
            if (o->healthVersion != o->geometryVersion) solid = "checking";
            else if (o->health.solid()) { solid = "solid"; solidCol = palette::kValid; }
            else { solid = "not solid"; solidCol = palette::kBrand; }
        }
        pushFont(FontWeight::Regular, uiFonts().size * 0.82f);
        const ImVec2 ts = ImGui::CalcTextSize(line);
        float rx = x0 + w - 16.0f;
        const float ry = y0 + h - ts.y - 12.0f;
        if (solid) {
            const ImVec2 ss = ImGui::CalcTextSize(solid);
            rx -= ss.x;
            dl->AddText(ImVec2(rx, ry), u32(solidCol, 0.95f), solid);
            rx -= 14.0f;
        }
        dl->AddText(ImVec2(rx - ts.x, ry), u32(palette::kTextFaint), line);
        ImGui::PopFont();
    }
}

// -----------------------------------------------------------------------------
void drawMeasurePanel(UiContext& ctx) {
    if (!ctx.measuring) return;

    // The same box every other operation runs in. A measurement is a reading
    // rather than a change, so it has no commit -- but a panel that looks
    // different for no reason is a panel the user has to learn twice.
    if (!ui::beginCommand("##measure", "Measure", Glyph::Measure)) return;

    const MeasureResult& m = ctx.measurement;
    auto value = [](const char* label, const char* fmt, double v, bool lead) {
        ui::commandRow(label);
        ImGui::AlignTextToFramePadding();
        if (lead) {
            pushFont(FontWeight::SemiBold);
            ImGui::TextColored(im(palette::kBrand), fmt, v);
            ImGui::PopFont();
        } else {
            ImGui::Text(fmt, v);
        }
    };

    if (!m.valid) {
        ui::commandHint("Click a vertex, edge or face. Click a second to measure between them.");
    } else {
        if (m.hasDiameter) {
            value("Diameter",  "%.4f mm", m.diameter, true);
            value("Radius",    "%.4f mm", m.diameter * 0.5, false);
            ui::commandRow("Centre");
            ImGui::AlignTextToFramePadding();
            ImGui::Text("%.3f, %.3f, %.3f", m.centre.x, m.centre.y, m.centre.z);
        }
        if (m.hasLength)
            value(m.hasDiameter ? "Around" : "Length", "%.4f mm", m.length, true);
        if (m.hasArea) {
            value("Area",      "%.4f mm\xC2\xB2", m.area, true);
            value("Perimeter", "%.4f mm",  m.perimeter, false);
        }
        if (ctx.measurePicks == 2 || m.hasLength) {
            if (ctx.measurePicks == 2) value("Distance", "%.4f mm", m.distance, true);
            value("dX", "%.4f mm", m.delta.x, false);
            value("dY", "%.4f mm", m.delta.y, false);
            value("dZ", "%.4f mm", m.delta.z, false);
        }
        if (m.hasAngle) value("Angle", "%.3f\xC2\xB0", m.angleDeg, true);
        ui::commandHint("Esc clears the picks. D leaves the tool.");
    }
    if (ui::commandFooter(nullptr, true, "Done") < 0) ctx.actions.toggleMeasure = true;
    ui::endCommand();
}

} // namespace tg
