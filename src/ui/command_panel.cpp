#include "ui/command_panel.h"

#include "core/palette.h"
#include "ui/theme.h"
#include "ui/widgets.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace tg::ui {
namespace {

constexpr float kLabelColumn = 88.0f;
constexpr float kPanelWidth  = 392.0f;
constexpr float kBottomGap   = 22.0f;

struct Anchor { float x = 0, y = 0, w = 0, h = 0; bool set = false; };
Anchor g_anchor;

// Which number bar is being pulled, and whether it has moved since the press,
// so a press that never moved can count as a click instead. The range is
// latched at the press: a caller that sizes its range from the value would
// otherwise move the goalposts under the pointer on every frame.
ImGuiID g_dragId = 0;
bool    g_dragMoved = false;
double  g_dragLo = 0.0, g_dragHi = 0.0;

} // namespace

float commandLabelWidth() { return kLabelColumn; }

void setCommandAnchor(float x, float y, float w, float h) {
    g_anchor = {x, y, w, h, true};
}

bool beginCommand(const char* id, const char* title, Glyph glyph, const char* context) {
    // Bottom-centre of the viewport, pinned by the panel's bottom edge so it
    // grows upward as rows are added and never runs off the screen.
    ImGuiViewport* vp = ImGui::GetMainViewport();
    float cx = vp->WorkPos.x + vp->WorkSize.x * 0.5f;
    float by = vp->WorkPos.y + vp->WorkSize.y - kBottomGap;
    if (g_anchor.set) {
        cx = vp->Pos.x + g_anchor.x + g_anchor.w * 0.5f;
        by = vp->Pos.y + g_anchor.y + g_anchor.h - kBottomGap;
    }
    ImGui::SetNextWindowPos(ImVec2(cx, by), ImGuiCond_Always, ImVec2(0.5f, 1.0f));
    ImGui::SetNextWindowSize(ImVec2(kPanelWidth, 0.0f), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.97f);

    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoDocking |
        ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav |
        ImGuiWindowFlags_AlwaysAutoResize;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(14.0f, 12.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 8.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 10.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, im(palette::kCommand));
    ImGui::PushStyleColor(ImGuiCol_Border, im(palette::kBorderStrong, 0.9f));

    if (!ImGui::Begin(id, nullptr, flags)) {
        ImGui::End();
        ImGui::PopStyleColor(2);
        ImGui::PopStyleVar(4);
        return false;
    }

    // The breadcrumb: the operation's picture, what it acts on, and its name.
    const float line = ImGui::GetTextLineHeight();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 at = ImGui::GetCursorScreenPos();
    const float box = line + 6.0f;
    dl->AddRectFilled(at, ImVec2(at.x + box, at.y + box), u32(palette::kBrand), 5.0f);
    drawGlyph(dl, glyph, ImVec2(at.x + box * 0.5f, at.y + box * 0.5f), line * 0.95f,
              IM_COL32(255, 255, 255, 255));
    ImGui::Dummy(ImVec2(box, box));
    ImGui::SameLine(0.0f, 9.0f);
    ImGui::AlignTextToFramePadding();
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() - (ImGui::GetFrameHeight() - box) * 0.5f);
    pushFont(FontWeight::SemiBold);
    if (context && *context) {
        ImGui::TextColored(im(palette::kText), "%s", context);
        ImGui::SameLine(0.0f, 6.0f);
        ImGui::TextColored(im(palette::kTextFaint), "/");
        ImGui::SameLine(0.0f, 6.0f);
        ImGui::PopFont();
        pushFont(FontWeight::Regular);
        ImGui::TextColored(im(palette::kTextDim), "%s", title);
    } else {
        ImGui::TextColored(im(palette::kText), "%s", title);
    }
    ImGui::PopFont();
    ImGui::Dummy(ImVec2(0, 4));
    return true;
}

void endCommand() {
    ImGui::End();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar(4);
}

void commandRow(const char* label) {
    ImGui::AlignTextToFramePadding();
    pushFont(FontWeight::Medium);
    ImGui::TextColored(im(palette::kTextDim), "%s", label);
    ImGui::PopFont();
    ImGui::SameLine(kLabelColumn);
}

void commandValue(const char* label, const char* value) {
    commandRow(label);
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(value);
}

NumberEdit commandNumber(const char* label, double value, const char* unit,
                         bool fixed, bool editing, const char* buffer,
                         double lo, double hi, bool signedRange) {
    NumberEdit out;
    out.value = value;
    ImGui::PushID(label);
    commandRow(label);

    char text[64];
    if (editing) std::snprintf(text, sizeof text, "%s_", buffer ? buffer : "");
    else         std::snprintf(text, sizeof text, "%.2f %s", value, unit ? unit : "");

    // A bar the width of the rest of the row, so the numbers line up down the
    // panel however long their labels are.
    const float w = ImGui::GetContentRegionAvail().x;
    const ImVec2 size(w, ImGui::GetFrameHeight() + 2.0f);
    const ImVec2 at = ImGui::GetCursorScreenPos();

    ImGui::InvisibleButton("##field", size);
    const ImGuiID id = ImGui::GetItemID();
    const bool hot = ImGui::IsItemHovered();

    // Pulling the bar. A press that moves is a drag and sets the value; a
    // press that does not is a click and hands the value back to the pointer.
    if (ImGui::IsItemActivated()) {
        g_dragId = id;
        g_dragMoved = false;
        g_dragLo = lo;
        g_dragHi = hi;
    }
    if (g_dragId == id) { lo = g_dragLo; hi = g_dragHi; }
    const bool ranged = hi > lo;

    if (ImGui::IsItemActive() && g_dragId == id && ranged) {
        const ImVec2 d = ImGui::GetIO().MouseDelta;
        if (d.x != 0.0f || d.y != 0.0f || g_dragMoved) {
            g_dragMoved = true;
            const float mx = ImGui::GetIO().MousePos.x;
            double t = (mx - at.x) / std::max(1.0f, size.x);
            t = std::clamp(t, 0.0, 1.0);
            double v = lo + t * (hi - lo);
            // To a number a person would type, at a resolution the bar can
            // show -- and never outside the range, whichever way it rounds.
            const double step = niceStep((hi - lo) / 160.0);
            if (step > 0.0) v = std::round(v / step) * step;
            v = std::clamp(v, lo, hi);
            if (std::fabs(v) < 1e-12) v = 0.0;
            out.dragged = true;
            out.dragging = true;
            out.value = v;
        }
    }
    if (ImGui::IsItemDeactivated() && g_dragId == id) {
        if (!g_dragMoved) out.clicked = true;
        else              out.released = true;
        g_dragId = 0;
    }

    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 lo2 = at, hi2 = ImVec2(at.x + size.x, at.y + size.y);
    const float r = 5.0f;
    dl->AddRectFilled(lo2, hi2, u32(palette::kField), r);

    // The fill: how much of the range is in use. From zero for a value that
    // can go either way, so zero is a place and not an edge -- wherever zero
    // falls in the range, which need not be its middle.
    if (ranged) {
        const double shown = out.dragged ? out.value : value;
        const double t = std::clamp((shown - lo) / (hi - lo), 0.0, 1.0);
        const ImU32 fill = u32(mix(palette::kRaised, palette::kBrand, fixed || editing ? 0.75f : 0.42f));
        if (signedRange) {
            const double tz = std::clamp((0.0 - lo) / (hi - lo), 0.0, 1.0);
            const float mid = at.x + static_cast<float>(tz) * size.x;
            const float x = at.x + static_cast<float>(t) * size.x;
            dl->AddRectFilled(ImVec2(std::min(mid, x), at.y), ImVec2(std::max(mid, x), hi2.y), fill, r);
            dl->AddLine(ImVec2(mid, at.y + 3.0f), ImVec2(mid, hi2.y - 3.0f), u32(palette::kTextFaint));
        } else {
            dl->AddRectFilled(lo2, ImVec2(at.x + static_cast<float>(t) * size.x, hi2.y), fill, r);
        }
    }
    if (fixed || editing || hot)
        dl->AddRect(lo2, hi2, u32(palette::kBrand, editing ? 1.0f : (fixed ? 0.8f : 0.4f)), r);

    const ImVec2 ts = ImGui::CalcTextSize(text);
    dl->AddText(ImVec2(hi2.x - ts.x - 9.0f, at.y + (size.y - ts.y) * 0.5f),
                fixed || editing ? u32(palette::kBrand) : u32(palette::kText), text);
    if (hot && !ImGui::IsItemActive())
        ImGui::SetTooltip("%s", ranged ? "Drag to set it, click to hand it back to the mouse, or type a number"
                                       : "Click to hand it back to the mouse, or type a number");

    ImGui::PopID();
    return out;
}

int commandChoices(const char* label, const Choice* choices, int count, int active, bool compact) {
    if (count <= 0) return -1;
    int clicked = -1;
    ImGui::PushID(label);
    if (compact) {
        commandRow(label);
        for (int i = 0; i < count; ++i) {
            if (i) ImGui::SameLine(0.0f, 4.0f);
            ImGui::PushID(i);
            if (pillButton(choices[i].label, i == active, ImVec2(0, 0), choices[i].enabled))
                clicked = i;
            hoverTip(choices[i].tip);
            ImGui::PopID();
        }
        ImGui::PopID();
        return clicked;
    }

    // The big row: each choice its own tile, the picture above the word, the
    // key beside it. Equal widths across the whole panel.
    if (label && *label) {
        pushFont(FontWeight::Medium);
        ImGui::TextColored(im(palette::kTextDim), "%s", label);
        ImGui::PopFont();
    }
    const ImGuiStyle& st = ImGui::GetStyle();
    const float gap = 8.0f;
    const float avail = ImGui::GetContentRegionAvail().x;
    const float each = (avail - gap * static_cast<float>(count - 1)) / static_cast<float>(count);
    const float tileH = 60.0f;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    for (int i = 0; i < count; ++i) {
        if (i) ImGui::SameLine(0.0f, gap);
        ImGui::PushID(i);
        const bool live = choices[i].enabled;
        const ImVec2 at = ImGui::GetCursorScreenPos();
        const bool pressed = ImGui::InvisibleButton("##tile", ImVec2(each, tileH)) && live;
        const bool hovered = ImGui::IsItemHovered();
        const bool on = i == active;
        if (pressed) clicked = i;

        const ImVec2 hi(at.x + each, at.y + tileH);
        dl->AddRectFilled(at, hi, on ? u32(palette::kBrand)
                                     : hovered && live ? u32(palette::kHover) : u32(palette::kRaised), 7.0f);
        const ImU32 fg = on      ? IM_COL32(255, 255, 255, 255)
                       : live    ? u32(palette::kText)
                                 : u32(palette::kTextFaint);
        drawGlyph(dl, choices[i].glyph, ImVec2(at.x + each * 0.5f, at.y + 22.0f), 24.0f, fg);

        pushFont(FontWeight::Medium, uiFonts().size * 0.92f);
        const ImVec2 ls = ImGui::CalcTextSize(choices[i].label);
        float keyW = 0.0f;
        ImVec2 ks(0, 0);
        if (choices[i].key && *choices[i].key) {
            ks = ImGui::CalcTextSize(choices[i].key);
            keyW = ks.x + 7.0f;
        }
        const float x0 = at.x + (each - ls.x - keyW) * 0.5f;
        const float y0 = at.y + tileH - ls.y - 8.0f;
        dl->AddText(ImVec2(x0, y0), fg, choices[i].label);
        if (keyW > 0.0f)
            dl->AddText(ImVec2(x0 + ls.x + 7.0f, y0),
                        on ? IM_COL32(255, 255, 255, 190)
                           : live ? u32(palette::kTextDim) : u32(palette::kTextFaint),
                        choices[i].key);
        ImGui::PopFont();
        hoverTip(choices[i].tip);
        ImGui::PopID();
    }
    (void)st;
    ImGui::PopID();
    return clicked;
}

void commandHint(const char* text) {
    if (!text || !*text) return;
    ImGui::Dummy(ImVec2(0, 1));
    pushFont(FontWeight::Regular, uiFonts().size * 0.9f);
    ImGui::PushStyleColor(ImGuiCol_Text, im(palette::kTextDim));
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextUnformatted(text);
    ImGui::PopTextWrapPos();
    ImGui::PopStyleColor();
    ImGui::PopFont();
}

void commandApplied(const char* what) {
    ImGui::Dummy(ImVec2(0, 1));
    pushFont(FontWeight::Medium, uiFonts().size * 0.9f);
    ImGui::PushStyleColor(ImGuiCol_Text, im(palette::kValid));
    ImGui::PushTextWrapPos(0.0f);
    ImGui::Text("%s applied. Change anything here to adjust it.", what);
    ImGui::PopTextWrapPos();
    ImGui::PopStyleColor();
    ImGui::PopFont();
}

void commandRefused(const char* why) {
    ImGui::Dummy(ImVec2(0, 1));
    pushFont(FontWeight::Medium, uiFonts().size * 0.9f);
    ImGui::PushStyleColor(ImGuiCol_Text, im(palette::kBrand));
    ImGui::PushTextWrapPos(0.0f);
    ImGui::Text("Not made: %s. Change it and it is tried again.", why && *why ? why : "the kernel refused");
    ImGui::PopTextWrapPos();
    ImGui::PopStyleColor();
    ImGui::PopFont();
}

int commandFooter(const char* commitLabel, bool commitEnabled, const char* cancelLabel) {
    ImGui::Dummy(ImVec2(0, 4));
    int result = 0;
    const ImGuiStyle& st = ImGui::GetStyle();
    const float pad = 14.0f;
    const float commitW = commitLabel ? ImGui::CalcTextSize(commitLabel).x + pad * 2.0f : 0.0f;
    const float cancelW = cancelLabel ? ImGui::CalcTextSize(cancelLabel).x + pad * 2.0f : 0.0f;
    const float total = commitW + cancelW + (commitLabel && cancelLabel ? st.ItemSpacing.x : 0.0f);
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.0f, ImGui::GetContentRegionAvail().x - total));
    if (commitLabel) {
        if (primaryButton(commitLabel, ImVec2(commitW, 0.0f), commitEnabled)) result = 1;
        if (cancelLabel) ImGui::SameLine();
    }
    if (cancelLabel) {
        if (quietButton(cancelLabel, ImVec2(cancelW, 0.0f))) result = -1;
    }
    return result;
}

} // namespace tg::ui
