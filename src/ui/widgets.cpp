#include "ui/widgets.h"

#include "ui/theme.h"

#include "imgui_internal.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace tg::ui {
namespace {

constexpr float kLabelColumn = 84.0f;

const Rgb kAxisColours[3] = {palette::kAxisX, palette::kAxisY, palette::kAxisZ};
const char* kAxisLetters[3] = {"X", "Y", "Z"};

} // namespace

float labelColumn() { return kLabelColumn; }

// ---- text -----------------------------------------------------------------
void sectionTitle(const char* text) {
    ImGui::Dummy(ImVec2(0, 4));
    pushFont(FontWeight::SemiBold, uiFonts().size);
    ImGui::TextColored(im(palette::kText), "%s", text);
    ImGui::PopFont();
    ImGui::Dummy(ImVec2(0, 1));
}

void captionText(const char* text, ImU32 colour) {
    pushFont(FontWeight::Medium, uiFonts().size * 0.78f);
    // Spaced out by hand: ImGui has no letter-spacing, and a caption in small
    // capitals wants a little air between the letters to read as a label
    // rather than as a word that has been shrunk.
    const ImU32 col = colour ? colour : u32(palette::kTextDim);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 at = ImGui::GetCursorScreenPos();
    float width = 0.0f;
    const float space = 1.2f;
    for (const char* p = text; *p; ++p) {
        char ch[2] = {*p, 0};
        const ImVec2 s = ImGui::CalcTextSize(ch);
        dl->AddText(ImVec2(at.x + width, at.y), col, ch);
        width += s.x + space;
    }
    ImGui::Dummy(ImVec2(width, ImGui::GetTextLineHeight()));
    ImGui::PopFont();
}

void tag(const char* text, Rgb colour, bool filled) {
    pushFont(FontWeight::Medium, uiFonts().size * 0.8f);
    const ImVec2 ts = ImGui::CalcTextSize(text);
    const ImVec2 pad(6.0f, 2.0f);
    const ImVec2 at = ImGui::GetCursorScreenPos();
    const float h = ImGui::GetTextLineHeight() + pad.y * 2.0f;
    const ImVec2 lo(at.x, at.y + (ImGui::GetFrameHeight() - h) * 0.5f);
    const ImVec2 hi(lo.x + ts.x + pad.x * 2.0f, lo.y + h);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    if (filled) {
        dl->AddRectFilled(lo, hi, u32(colour, 0.9f), 4.0f);
        dl->AddText(ImVec2(lo.x + pad.x, lo.y + pad.y), IM_COL32(255, 255, 255, 255), text);
    } else {
        dl->AddRectFilled(lo, hi, u32(colour, 0.16f), 4.0f);
        dl->AddText(ImVec2(lo.x + pad.x, lo.y + pad.y), u32(colour), text);
    }
    ImGui::Dummy(ImVec2(hi.x - lo.x, ImGui::GetFrameHeight()));
    ImGui::PopFont();
}

// ---- buttons --------------------------------------------------------------
bool primaryButton(const char* label, ImVec2 size, bool enabled) {
    ImGui::PushStyleColor(ImGuiCol_Button, im(palette::kBrand));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, im(palette::kBrandHover));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, im(palette::kBrandDown));
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 1, 1, 1));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(14.0f, 6.0f));
    pushFont(FontWeight::SemiBold);
    ImGui::BeginDisabled(!enabled);
    const bool clicked = ImGui::Button(label, size);
    ImGui::EndDisabled();
    ImGui::PopFont();
    ImGui::PopStyleVar();
    ImGui::PopStyleColor(4);
    return clicked;
}

bool quietButton(const char* label, ImVec2 size, bool enabled) {
    ImGui::PushStyleColor(ImGuiCol_Button, im(palette::kRaised));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, im(palette::kHover));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, im(palette::kActive));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(14.0f, 6.0f));
    pushFont(FontWeight::Medium);
    ImGui::BeginDisabled(!enabled);
    const bool clicked = ImGui::Button(label, size);
    ImGui::EndDisabled();
    ImGui::PopFont();
    ImGui::PopStyleVar();
    ImGui::PopStyleColor(3);
    return clicked;
}

bool pillButton(const char* label, bool on, ImVec2 size, bool enabled) {
    ImGui::PushStyleColor(ImGuiCol_Button, on ? im(palette::kBrand) : im(palette::kRaised));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, on ? im(palette::kBrandHover) : im(palette::kHover));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, on ? im(palette::kBrandDown) : im(palette::kActive));
    ImGui::PushStyleColor(ImGuiCol_Text, on ? ImVec4(1, 1, 1, 1) : im(palette::kText));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 999.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(11.0f, 4.0f));
    ImGui::BeginDisabled(!enabled);
    const bool clicked = ImGui::Button(label, size);
    ImGui::EndDisabled();
    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(4);
    return clicked;
}

bool eyeToggle(const char* id, bool& visible, float sizePx) {
    ImGui::PushID(id);
    const ImVec2 size(sizePx + 6.0f, sizePx + 6.0f);
    const ImVec2 at = ImGui::GetCursorScreenPos();
    const bool clicked = ImGui::InvisibleButton("##eye", size);
    const bool hovered = ImGui::IsItemHovered();
    if (clicked) visible = !visible;
    const ImU32 col = visible ? u32(palette::kText, hovered ? 1.0f : 0.85f)
                              : u32(palette::kTextDim, hovered ? 1.0f : 0.8f);
    drawGlyph(ImGui::GetWindowDrawList(), visible ? Glyph::Eye : Glyph::EyeOff,
              ImVec2(at.x + size.x * 0.5f, at.y + size.y * 0.5f), sizePx, col, 1.3f);
    if (hovered) ImGui::SetTooltip("%s", visible ? "Shown. Click to hide." : "Hidden. Click to show.");
    ImGui::PopID();
    return clicked;
}

bool closeButton(const char* id, float sizePx) {
    ImGui::PushID(id);
    const ImVec2 size(sizePx + 6.0f, sizePx + 6.0f);
    const ImVec2 at = ImGui::GetCursorScreenPos();
    const bool clicked = ImGui::InvisibleButton("##x", size);
    const bool hovered = ImGui::IsItemHovered();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    if (hovered)
        dl->AddRectFilled(at, ImVec2(at.x + size.x, at.y + size.y), u32(palette::kBrand, 0.2f), 4.0f);
    drawGlyph(dl, Glyph::Close, ImVec2(at.x + size.x * 0.5f, at.y + size.y * 0.5f), sizePx,
              hovered ? u32(palette::kBrand) : u32(palette::kTextDim), 1.5f);
    ImGui::PopID();
    return clicked;
}

// ---- fields ---------------------------------------------------------------
void fieldHeader(const char* label, const char* unit) {
    ImGui::Dummy(ImVec2(0, 2));
    pushFont(FontWeight::Medium, uiFonts().size * 0.92f);
    ImGui::TextColored(im(palette::kTextDim), "%s", label);
    ImGui::PopFont();
    if (unit && *unit) {
        char text[32];
        std::snprintf(text, sizeof text, "[%s]", unit);
        pushFont(FontWeight::Regular, uiFonts().size * 0.8f);
        const float w = ImGui::CalcTextSize(text).x;
        ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - w);
        ImGui::TextColored(im(palette::kTextFaint), "%s", text);
        ImGui::PopFont();
    }
}

bool axisFields(const char* id, Vec3& v, float speed, const char* fmt, bool readOnly) {
    ImGui::PushID(id);
    const ImGuiStyle& st = ImGui::GetStyle();
    const float avail = ImGui::GetContentRegionAvail().x;
    const float gap = 6.0f;
    const float each = (avail - gap * 2.0f) / 3.0f;
    const float letterW = 12.0f;

    bool changed = false;
    for (int i = 0; i < 3; ++i) {
        if (i) ImGui::SameLine(0.0f, gap);
        ImGui::PushID(i);
        ImGui::BeginGroup();
        // The letter, coloured; then the field, plain. Colour on the letter
        // rather than on the box keeps the number itself clean to read.
        pushFont(FontWeight::SemiBold, uiFonts().size * 0.85f);
        const ImVec2 at = ImGui::GetCursorScreenPos();
        ImGui::GetWindowDrawList()->AddText(
            ImVec2(at.x, at.y + (ImGui::GetFrameHeight() - ImGui::GetTextLineHeight()) * 0.5f),
            u32(kAxisColours[i]), kAxisLetters[i]);
        ImGui::PopFont();
        ImGui::Dummy(ImVec2(letterW, ImGui::GetFrameHeight()));
        ImGui::SameLine(0.0f, 0.0f);
        ImGui::SetNextItemWidth(each - letterW);
        if (readOnly) {
            char text[32];
            std::snprintf(text, sizeof text, fmt, (&v.x)[i]);
            ImGui::PushStyleColor(ImGuiCol_FrameBg, im(palette::kField, 0.55f));
            ImGui::PushStyleColor(ImGuiCol_Text, im(palette::kTextDim));
            ImGui::BeginDisabled(true);
            ImGui::InputText("##v", text, sizeof text, ImGuiInputTextFlags_ReadOnly);
            ImGui::EndDisabled();
            ImGui::PopStyleColor(2);
        } else {
            changed |= ImGui::DragScalar("##v", ImGuiDataType_Double, &(&v.x)[i], speed,
                                         nullptr, nullptr, fmt);
        }
        ImGui::EndGroup();
        ImGui::PopID();
    }
    (void)st;
    ImGui::PopID();
    return changed;
}

bool labelledNumber(const char* label, Real& v, float speed, Real lo, Real hi, const char* fmt) {
    ImGui::PushID(label);
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(im(palette::kTextDim), "%s", label);
    ImGui::SameLine(kLabelColumn);
    ImGui::SetNextItemWidth(-1.0f);
    const bool changed = ImGui::DragScalarN("##v", ImGuiDataType_Double, &v, 1, speed, &lo, &hi, fmt);
    ImGui::PopID();
    return changed;
}

bool labelledInt(const char* label, int& v, int lo, int hi) {
    ImGui::PushID(label);
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(im(palette::kTextDim), "%s", label);
    ImGui::SameLine(kLabelColumn);
    ImGui::SetNextItemWidth(-1.0f);
    bool changed = ImGui::DragInt("##v", &v, 0.25f, lo, hi);
    if (changed) v = v < lo ? lo : (v > hi ? hi : v);
    ImGui::PopID();
    return changed;
}

// ---- menus ----------------------------------------------------------------
bool beginMenuPopup(const char* id, ImGuiWindowFlags extra) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8.0f, 8.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 2.0f));
    ImGui::PushStyleColor(ImGuiCol_Border, im(palette::kBorderStrong));
    // Opaque: a menu with the model showing through it is a menu that is
    // harder to read for no reason.
    ImGui::PushStyleColor(ImGuiCol_PopupBg, im(palette::kPanel));
    const bool open = ImGui::BeginPopup(id, extra);
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar(2);
    return open;
}

void menuHeader(const char* text) {
    ImGui::Dummy(ImVec2(0, 2));
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 8.0f);
    captionText(text, u32(palette::kTextFaint));
    ImGui::Dummy(ImVec2(0, 2));
}

bool menuEntry(Glyph g, const char* label, const char* shortcut, bool enabled, bool selected) {
    ImGui::PushID(label);
    const float h = ImGui::GetFrameHeight() + 2.0f;
    const float w = std::max(ImGui::GetContentRegionAvail().x, 200.0f);
    const ImVec2 at = ImGui::GetCursorScreenPos();
    ImGui::BeginDisabled(!enabled);
    const bool clicked = ImGui::InvisibleButton("##m", ImVec2(w, h));
    const bool hovered = ImGui::IsItemHovered();
    ImGui::EndDisabled();

    ImDrawList* dl = ImGui::GetWindowDrawList();
    if (hovered && enabled)
        dl->AddRectFilled(at, ImVec2(at.x + w, at.y + h), u32(palette::kHover), 5.0f);
    const float alpha = enabled ? 1.0f : 0.38f;
    const ImU32 textCol = u32(selected ? palette::kBrand : palette::kText, alpha);
    if (g != Glyph::Count)
        drawGlyph(dl, g, ImVec2(at.x + 8.0f + 9.0f, at.y + h * 0.5f), 16.0f,
                  u32(selected ? palette::kBrand : palette::kTextDim, alpha), 1.4f);
    dl->AddText(ImVec2(at.x + 34.0f, at.y + (h - ImGui::GetTextLineHeight()) * 0.5f), textCol, label);
    if (shortcut && *shortcut) {
        pushFont(FontWeight::Regular, uiFonts().size * 0.86f);
        const ImVec2 ts = ImGui::CalcTextSize(shortcut);
        dl->AddText(ImVec2(at.x + w - ts.x - 10.0f, at.y + (h - ts.y) * 0.5f),
                    u32(palette::kTextFaint, alpha), shortcut);
        ImGui::PopFont();
    }
    if (clicked && enabled) ImGui::CloseCurrentPopup();
    ImGui::PopID();
    return clicked && enabled;
}

bool menuToggle(Glyph g, const char* label, bool* value, const char* shortcut) {
    ImGui::PushID(label);
    const float h = ImGui::GetFrameHeight() + 2.0f;
    const float w = std::max(ImGui::GetContentRegionAvail().x, 200.0f);
    const ImVec2 at = ImGui::GetCursorScreenPos();
    const bool clicked = ImGui::InvisibleButton("##t", ImVec2(w, h));
    const bool hovered = ImGui::IsItemHovered();
    if (clicked) *value = !*value;

    ImDrawList* dl = ImGui::GetWindowDrawList();
    if (hovered) dl->AddRectFilled(at, ImVec2(at.x + w, at.y + h), u32(palette::kHover), 5.0f);
    if (g != Glyph::Count)
        drawGlyph(dl, g, ImVec2(at.x + 17.0f, at.y + h * 0.5f), 16.0f, u32(palette::kTextDim), 1.4f);
    dl->AddText(ImVec2(at.x + 34.0f, at.y + (h - ImGui::GetTextLineHeight()) * 0.5f),
                u32(palette::kText), label);

    // A small switch at the right, which says on or off more plainly than a
    // tick mark that is sometimes there.
    float sx = at.x + w - 10.0f - 26.0f;
    if (shortcut && *shortcut) {
        pushFont(FontWeight::Regular, uiFonts().size * 0.86f);
        const ImVec2 ts = ImGui::CalcTextSize(shortcut);
        dl->AddText(ImVec2(sx - ts.x - 10.0f, at.y + (h - ts.y) * 0.5f), u32(palette::kTextFaint), shortcut);
        ImGui::PopFont();
    }
    const float sy = at.y + h * 0.5f;
    dl->AddRectFilled(ImVec2(sx, sy - 7.0f), ImVec2(sx + 26.0f, sy + 7.0f),
                      *value ? u32(palette::kBrand) : u32(palette::kActive), 7.0f);
    dl->AddCircleFilled(ImVec2(*value ? sx + 19.0f : sx + 7.0f, sy), 5.0f, IM_COL32(255, 255, 255, 235));
    ImGui::PopID();
    return clicked;
}

void menuNote(const char* text) {
    pushFont(FontWeight::Regular, uiFonts().size * 0.86f);
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 34.0f);
    ImGui::TextColored(im(palette::kTextFaint), "%s", text);
    ImGui::PopFont();
}

void menuGap() {
    ImGui::Dummy(ImVec2(0, 3));
    const ImVec2 at = ImGui::GetCursorScreenPos();
    const float w = ImGui::GetContentRegionAvail().x;
    ImGui::GetWindowDrawList()->AddLine(ImVec2(at.x + 8.0f, at.y), ImVec2(at.x + w - 8.0f, at.y),
                                        u32(palette::kBorderStrong));
    ImGui::Dummy(ImVec2(0, 4));
}

// ---- dialogs --------------------------------------------------------------
bool beginCard(const char* id, const char* title, float width) {
    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->GetCenter(), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(width, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(18.0f, 16.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_Border, im(palette::kBorderStrong));
    ImGui::PushStyleColor(ImGuiCol_PopupBg, im(palette::kCommand, 0.98f));
    const bool open = ImGui::BeginPopupModal(id, nullptr,
                                             ImGuiWindowFlags_AlwaysAutoResize |
                                             ImGuiWindowFlags_NoTitleBar |
                                             ImGuiWindowFlags_NoMove);
    if (!open) {
        ImGui::PopStyleColor(2);
        ImGui::PopStyleVar(2);
        return false;
    }
    pushFont(FontWeight::SemiBold, uiFonts().size * 1.1f);
    ImGui::TextUnformatted(title);
    ImGui::PopFont();
    ImGui::Dummy(ImVec2(0, 6));
    return true;
}

void endCard() {
    ImGui::EndPopup();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar(2);
}

void hoverTip(const char* text) {
    if (!text || !*text) return;
    if (!ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) return;
    if (ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId | ImGuiPopupFlags_AnyPopupLevel)) return;
    ImGui::SetTooltip("%s", text);
}

} // namespace tg::ui
