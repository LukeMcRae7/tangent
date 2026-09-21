#include "ui/glyph.h"

#include "core/palette.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>

namespace tg {
namespace {

ImFont* iconFont = nullptr;

ImU32 faded(ImU32 c, float alpha) {
    const float a = static_cast<float>((c >> IM_COL32_A_SHIFT) & 0xFF) / 255.0f * alpha;
    return (c & ~IM_COL32_A_MASK) | (static_cast<ImU32>(a * 255.0f + 0.5f) << IM_COL32_A_SHIFT);
}

} // namespace

bool loadGlyphFont(const std::string& fontsDir) {
    iconFont = nullptr;
    const std::string path = fontsDir + "/tabler-icons.ttf";
    std::error_code ec;
    if (!std::filesystem::exists(path, ec)) {
        std::fprintf(stderr, "[ui] no icon font at %s; icons will be blank\n", path.c_str());
        return false;
    }
    ImFontConfig cfg;
    cfg.OversampleH = 2;
    cfg.OversampleV = 2;
    cfg.PixelSnapH  = false;
    std::snprintf(cfg.Name, sizeof cfg.Name, "Tabler Icons");
    iconFont = ImGui::GetIO().Fonts->AddFontFromFileTTF(path.c_str(), 0.0f, &cfg);
    if (!iconFont) std::fprintf(stderr, "[ui] icon font rejected by ImGui: %s\n", path.c_str());
    return iconFont != nullptr;
}

// Each glyph's Tabler icon, by name. Look one up at tabler.io/icons; the
// codepoint is in the webfont's tabler-icons.css as `.ti-<name>:before`.
ImWchar glyphCodepoint(Glyph g) {
    switch (g) {
    // ---- files ----------------------------------------------------------
    case Glyph::New:           return 0xeaa0;   // file-plus
    case Glyph::Open:          return 0xfaf7;   // folder-open
    case Glyph::Save:          return 0xeb62;   // device-floppy
    case Glyph::Settings:      return 0xeb20;   // settings
    case Glyph::Import:        return 0xedea;   // file-import
    case Glyph::Export:        return 0xede9;   // file-export

    // ---- shapes ---------------------------------------------------------
    case Glyph::Box:
    case Glyph::Body:          return 0xfa97;   // cube
    case Glyph::Cylinder:      return 0xf54c;   // cylinder
    case Glyph::Sphere:        return 0xfab8;   // sphere
    case Glyph::Cone:          return 0xefdd;   // cone
    case Glyph::Torus:         return 0xeadd;   // lifebuoy: the ring everyone knows
    case Glyph::Plane:         return 0xeebd;   // perspective: a flat plate seen from above
    case Glyph::Sketch:        return 0xeb04;   // pencil

    // ---- transforms and combining --------------------------------------
    case Glyph::Move:          return 0xf22f;   // arrows-move
    case Glyph::Rotate:        return 0xeb15;   // rotate-clockwise
    case Glyph::Scale:         return 0xeecf;   // resize
    case Glyph::Boolean:       return 0xf4c3;   // circles-relation
    case Glyph::Union:         return 0xeacb;   // layers-union
    case Glyph::Difference:    return 0xeaca;   // layers-subtract
    case Glyph::Intersect:     return 0xeff8;   // layers-intersect-2: the shared part marked
    case Glyph::Overlap:       return 0xeac9;   // layers-intersect: both whole, overlapping
    case Glyph::NewBody:       return 0xfa96;   // cube-plus

    // ---- modelling ------------------------------------------------------
    case Glyph::Extrude:       return 0xea10;   // arrow-bar-up
    case Glyph::PushPull:      return 0xf22e;   // arrows-move-vertical
    case Glyph::Fillet:        return 0xfd63;   // border-corner-rounded
    case Glyph::Chamfer:       return 0xff4c;   // join-bevel: the corner cut flat
    case Glyph::Shell:         return 0xee0c;   // box-model: a hollow tray
    case Glyph::Inset:         return 0xef23;   // box-model-2: a face inside a face
    case Glyph::Revolve:       return 0xef85;   // rotate-360
    case Glyph::Hole:          return 0xefb1;   // circle-dot
    case Glyph::Counterbore:   return 0xece5;   // circles
    case Glyph::Countersink:   return 0xefdd;   // cone
    case Glyph::Draft:         return 0xef20;   // angle
    case Glyph::DeleteFace:    return 0xeb8b;   // eraser
    case Glyph::Offset:        return 0xee0b;   // box-margin
    case Glyph::Help:          return 0xeabf;   // help
    case Glyph::Divide:        return 0xebdb;   // slice
    case Glyph::Merge:         return 0xedaf;   // arrows-join
    case Glyph::Pattern:       return 0xedba;   // layout-grid
    case Glyph::PatternRow:    return 0xea95;   // dots
    case Glyph::PatternRing:   return 0xed28;   // circle-dotted
    case Glyph::Mirror:        return 0xeaa7;   // flip-horizontal
    case Glyph::Split:         return 0xedb5;   // arrows-split
    case Glyph::Convert:       return 0xf38e;   // transform
    case Glyph::Reduce:        return 0xfc9b;   // triangle-minus
    case Glyph::RotateFace:    return 0xec15;   // rotate-rectangle
    case Glyph::ScaleFace:     return 0xea28;   // arrows-maximize
    case Glyph::Mesh:          return 0x10201;  // mesh

    // ---- inspecting -----------------------------------------------------
    case Glyph::Measure:       return 0xf291;   // ruler-measure
    case Glyph::Alert:         return 0xea06;   // alert-triangle
    case Glyph::Check:         return 0xea5e;   // check
    case Glyph::Eye:           return 0xea9a;   // eye
    case Glyph::EyeOff:        return 0xecf0;   // eye-off
    case Glyph::Grid:          return 0xfca5;   // grid-4x4
    case Glyph::Wire:          return 0xecd7;   // cube-3d-sphere: a cube of edges only
    case Glyph::Backface:      return 0xfa95;   // cube-off
    case Glyph::Bounds:        return 0xf7a0;   // border-corners: a bounding box
    case Glyph::FrameSelected: return 0xf02a;   // focus-centered
    case Glyph::FrameAll:      return 0xfcb0;   // zoom-scan
    case Glyph::Orthographic:  return 0xf176;   // perspective-off
    case Glyph::Orbit:         return 0xed84;   // view-360

    // ---- chrome ---------------------------------------------------------
    case Glyph::ChevronDown:   return 0xea5f;   // chevron-down
    case Glyph::ChevronRight:  return 0xea61;   // chevron-right
    case Glyph::ChevronUp:     return 0xea62;   // chevron-up
    case Glyph::Close:         return 0xeb55;   // x
    case Glyph::Minimize:      return 0xeaf2;   // minus
    case Glyph::Maximize:      return 0xeb2c;   // square
    case Glyph::Restore:       return 0xeef6;   // squares
    case Glyph::Undo:          return 0xeb77;   // arrow-back-up
    case Glyph::Redo:          return 0xeb78;   // arrow-forward-up
    case Glyph::Plus:          return 0xeb0b;   // plus
    case Glyph::Duplicate:     return 0xea7a;   // copy
    case Glyph::SelectAll:     return 0xf9f7;   // select-all
    case Glyph::Dot:           return 0xeb0c;   // point
    case Glyph::Clock:         return 0xea70;   // clock
    case Glyph::Lock:          return 0xeae2;   // lock
    case Glyph::Trash:         return 0xeb41;   // trash

    // ---- sketching ------------------------------------------------------
    case Glyph::Select:        return 0xf265;   // pointer
    case Glyph::Line:          return 0xec40;   // line
    case Glyph::Rect:          return 0xed37;   // rectangle
    case Glyph::Circle:        return 0xefb1;   // circle-dot: centre, then rim
    case Glyph::Arc:           return 0xf565;   // vector-spline
    case Glyph::Dimension:     return 0xef37;   // arrow-autofit-width

    case Glyph::Count:         break;
    }
    return 0;
}

void drawGlyph(ImDrawList* dl, Glyph g, ImVec2 centre, float sizePx, ImU32 colour) {
    if (!dl || !iconFont || sizePx <= 0.0f) return;
    const ImWchar c = glyphCodepoint(g);
    if (!c) return;
    // The em box is the icon's 24-unit box (ascent 900 + descent 100 of a
    // 1000-unit em), so centring the box centres the icon as it was drawn.
    // Snapped to whole pixels: a stroke that straddles two goes soft.
    const ImVec2 at(std::floor(centre.x - sizePx * 0.5f + 0.5f),
                    std::floor(centre.y - sizePx * 0.5f + 0.5f));
    iconFont->RenderChar(dl, sizePx, at, colour, c);
}

void glyphItem(Glyph g, float sizePx, ImU32 colour) {
    const float h = ImGui::GetFrameHeight();
    const ImVec2 at = ImGui::GetCursorScreenPos();
    ImGui::Dummy(ImVec2(sizePx, std::max(h, sizePx)));
    drawGlyph(ImGui::GetWindowDrawList(), g,
              ImVec2(at.x + sizePx * 0.5f, at.y + std::max(h, sizePx) * 0.5f), sizePx, colour);
}

bool glyphButton(const char* id, Glyph g, float sizePx, const char* tooltip, bool active,
                 bool enabled, ImU32 tint) {
    const ImGuiStyle& st = ImGui::GetStyle();
    const ImVec2 size(sizePx + st.FramePadding.x * 2.0f, sizePx + st.FramePadding.y * 2.0f);

    ImGui::PushID(id);
    ImGui::BeginDisabled(!enabled);
    const ImVec2 at = ImGui::GetCursorScreenPos();
    const bool clicked = ImGui::InvisibleButton("##g", size);
    const bool hovered = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);
    const bool held = ImGui::IsItemActive();
    ImGui::EndDisabled();

    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 lo = at, hi = ImVec2(at.x + size.x, at.y + size.y);
    if (active) {
        dl->AddRectFilled(lo, hi, ImGui::GetColorU32(ImVec4(palette::kBrand.r, palette::kBrand.g,
                                                            palette::kBrand.b, 1.0f)), st.FrameRounding);
    } else if (held && enabled) {
        dl->AddRectFilled(lo, hi, ImGui::GetColorU32(ImGuiCol_ButtonActive), st.FrameRounding);
    } else if (hovered && enabled) {
        dl->AddRectFilled(lo, hi, ImGui::GetColorU32(ImGuiCol_ButtonHovered), st.FrameRounding);
    }

    ImU32 col = tint ? tint : ImGui::GetColorU32(ImGuiCol_Text);
    if (active) col = IM_COL32(255, 255, 255, 255);
    if (!enabled) col = faded(col, 0.35f);
    drawGlyph(dl, g, ImVec2(at.x + size.x * 0.5f, at.y + size.y * 0.5f), sizePx, col);

    // Not while a menu is open: a button that opens one is still hovered while
    // it is open, and its tooltip lands exactly on top of what it opened.
    if (tooltip && hovered &&
        !ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId | ImGuiPopupFlags_AnyPopupLevel))
        ImGui::SetTooltip("%s", tooltip);
    ImGui::PopID();
    return clicked && enabled;
}

} // namespace tg
