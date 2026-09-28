#include "ui/command_panel.h"
#include "core/units.h"

#include "core/palette.h"
#include "ui/theme.h"
#include "ui/view_cube.h"
#include "ui/widgets.h"

#include "imgui_internal.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace tg::ui {
namespace {

constexpr float kLabelColumn = 86.0f;
constexpr float kPanelWidth  = 316.0f;
constexpr float kMargin      = 14.0f;   // from the viewport's edges, as the status line

struct Anchor { float x = 0, y = 0, w = 0, h = 0; bool set = false; };
Anchor g_anchor;
float  g_topInset = kMargin;

// Stepping back. The panel fades once the pointer has rested on its empty
// parts for a moment, or the view has been orbited or dragged for as long: a
// pointer merely crossing the gap between two buttons, or a click, must not
// make it flicker.
constexpr float kRecedeAfter = 0.14f;   // seconds
constexpr float kRecededAlpha = 0.5f;   // what is drawn on it
constexpr float kRecededBg    = 0.28f;  // the panel itself, so the part shows
bool  g_gesture = false, g_tracking = false;
float g_recedeFor = 0.0f;               // how long it has been asked to
float g_fade = 0.0f;                    // 0 in front, 1 all the way back

// Whether the pointer is over the panel this frame, and over something in it
// that can be used. Gathered as the panels are drawn -- which is before the
// viewport reads them, in the same frame.
bool   g_hovered = false, g_hot = false;
bool   g_hoveredLast = false, g_hotLast = false;
ImVec4 g_rect{0, 0, 0, 0}, g_rectNow{0, 0, 0, 0};

// Which number bar is being pulled, and whether it has moved since the press,
// so a press that never moved can count as a click instead. The range is
// latched at the press: a caller that sizes its range from the value would
// otherwise move the goalposts under the pointer on every frame.
ImGuiID g_dragId = 0;
bool    g_dragMoved = false;
double  g_dragLo = 0.0, g_dragHi = 0.0;

// The bar being typed into, when one has been double-clicked: every bar takes
// a number, whether or not its tool also hears digits typed at the view.
ImGuiID g_typeId = 0;
bool    g_typeFocus = false;
int     g_typeFrame = 0;         // last frame it was drawn; a panel that closes lets it go
char    g_typeBuf[64] = "";

// What the panel would have said, and where the mark that says it goes.
//
// A dialog that explains itself in three lines of grey text is a dialog nobody
// reads and a viewport nobody can see past. The words are still here -- they
// are how an operation says what it does to someone who has not met it -- but
// they live behind the ? in the corner and come out on hover.
std::string g_help;
ImVec2      g_helpAt{0, 0};
constexpr float kHelpSize = 17.0f;

} // namespace

float commandLabelWidth() { return kLabelColumn; }

void setCommandAnchor(float x, float y, float w, float h) {
    g_anchor = {x, y, w, h, true};
    // A new frame of panels. Last frame's answers stand until this one's are
    // in, for anyone who asks in between.
    g_hoveredLast = g_hovered;
    g_hotLast = g_hot;
    g_hovered = g_hot = false;
    g_rect = g_rectNow;
    g_rectNow = ImVec4(0, 0, 0, 0);
}

void setCommandTopInset(float y) { g_topInset = std::max(y, kMargin); }

void setCommandRecede(bool gesture, bool tracking) {
    g_gesture = gesture;
    g_tracking = tracking;
}

bool commandPassesPointer() { return g_hovered && !g_hot; }
bool commandPassesClicks()  { return commandPassesPointer() && g_fade > 0.5f; }
ImVec4 commandPanelRect()   { return g_rect; }

bool beginCommand(const char* id, const char* title, Glyph glyph, const char* context) {
    // Top-left of the viewport, under the status line, growing downward as
    // rows are added -- and no further than the viewport's foot, past which
    // it scrolls rather than running off the screen.
    ImGuiViewport* vp = ImGui::GetMainViewport();
    float x = vp->WorkPos.x + kMargin, y = vp->WorkPos.y + kMargin;
    float maxH = vp->WorkSize.y - kMargin * 2.0f;
    if (g_anchor.set) {
        x = vp->Pos.x + g_anchor.x + kMargin;
        y = vp->Pos.y + g_anchor.y + g_topInset;
        maxH = g_anchor.h - g_topInset - kMargin;
    }
    // Narrower in a narrow view, down to what the rows can still be laid out in.
    const float width = g_anchor.set
        ? std::clamp(g_anchor.w - kMargin * 2.0f, 260.0f, kPanelWidth) : kPanelWidth;
    // In a view too narrow for both corners to be used, under the view cube
    // and the projection named beneath it rather than over them.
    if (g_anchor.set) {
        const ViewCubeStyle cube;
        if (kMargin + width + 12.0f > g_anchor.w - cube.marginPx - cube.sizePx) {
            const float below = cube.marginPx + cube.sizePx + 34.0f;
            if (g_topInset < below) {
                y += below - g_topInset;
                maxH -= below - g_topInset;
            }
        }
    }
    ImGui::SetNextWindowPos(ImVec2(x, y), ImGuiCond_Always);
    ImGui::SetNextWindowSizeConstraints(ImVec2(width, 0.0f),
                                        ImVec2(width, std::max(maxH, 120.0f)));

    // How far back it stands. Asked for by the viewport's gesture, or by the
    // pointer resting on the panel's empty parts while a tool follows it --
    // both only after a moment, so neither a click nor a pass across a gap
    // shows. Coming forward again is at once in the asking and quick in the
    // fading: a panel reached for must be there.
    const float dt = ImGui::GetIO().DeltaTime;
    const bool resting = g_hoveredLast && !g_hotLast && g_tracking;
    if (g_gesture || resting) g_recedeFor += dt;
    else                      g_recedeFor = 0.0f;
    const float goal = g_recedeFor >= kRecedeAfter ? 1.0f : 0.0f;
    const float rate = goal > g_fade ? 10.0f : 16.0f;
    g_fade += (goal - g_fade) * std::min(1.0f, dt * rate);
    if (std::fabs(goal - g_fade) < 0.002f) g_fade = goal;
    const float alpha = 1.0f + (kRecededAlpha - 1.0f) * g_fade;
    const float bg = 0.97f + (kRecededBg - 0.97f) * g_fade;
    ImGui::SetNextWindowBgAlpha(bg / alpha);

    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoDocking |
        ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav |
        ImGuiWindowFlags_AlwaysAutoResize;

    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, alpha);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 10.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(6.0f, 6.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 10.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, im(palette::kCommand));
    ImGui::PushStyleColor(ImGuiCol_Border, im(palette::kBorderStrong, 0.9f));

    if (!ImGui::Begin(id, nullptr, flags)) {
        ImGui::End();
        ImGui::PopStyleColor(2);
        ImGui::PopStyleVar(5);
        return false;
    }

    // The breadcrumb: the operation's picture, what it acts on, and its name.
    const float line = ImGui::GetTextLineHeight();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 at = ImGui::GetCursorScreenPos();
    const float box = line + 4.0f;
    // Where the ? goes, taken now while the row is still the full width of the
    // panel. What it says is collected as the rows are built and drawn at the
    // end, back up here.
    g_help.clear();
    g_helpAt = ImVec2(at.x + ImGui::GetContentRegionAvail().x - kHelpSize,
                      at.y + (box - kHelpSize) * 0.5f);
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
    ImGui::Dummy(ImVec2(0, 2));
    return true;
}

void endCommand() {
    if (!g_help.empty()) {
        // Back up to the header, and back down again: the mark belongs beside
        // the title, but what it says is not known until the rows have said
        // whether they were refused, what they applied, and what they are for.
        const ImVec2 resume = ImGui::GetCursorScreenPos();
        ImGui::SetCursorScreenPos(g_helpAt);
        ImGui::InvisibleButton("##help", ImVec2(kHelpSize, kHelpSize));
        const bool hot = ImGui::IsItemHovered();
        ImDrawList* dl = ImGui::GetWindowDrawList();
        drawGlyph(dl, Glyph::Help, ImVec2(g_helpAt.x + kHelpSize * 0.5f, g_helpAt.y + kHelpSize * 0.5f),
                  kHelpSize, u32(hot ? palette::kText : palette::kTextFaint));
        if (hot) {
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 10.0f));
            ImGui::BeginTooltip();
            ImGui::PushTextWrapPos(ImGui::GetFontSize() * 21.0f);
            ImGui::TextUnformatted(g_help.c_str());
            ImGui::PopTextWrapPos();
            ImGui::EndTooltip();
            ImGui::PopStyleVar();
        }
        // Back where the rows left off, with an item of no size: moving the
        // cursor alone tells ImGui nothing about how big the window is, and it
        // says so, every frame.
        ImGui::SetCursorScreenPos(resume);
        ImGui::Dummy(ImVec2(0, 0));
        g_help.clear();
    }

    // Is the pointer here, and on something that does anything? Only items
    // in the hovered window can be hovered, so an item hovered by now is one
    // of this panel's. A press on its empty parts makes ImGui hold the window
    // as if to drag it -- it cannot move, so that hold is not the panel's use
    // of the pointer, and does not count.
    const ImGuiContext& g = *GImGui;
    ImGuiWindow* self = ImGui::GetCurrentWindow();
    if (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows |
                               ImGuiHoveredFlags_AllowWhenBlockedByActiveItem)) {
        g_hovered = true;
        // A panel taller than the view scrolls, and the wheel is how.
        if (g.HoveredId != 0 || ImGui::GetScrollMaxY() > 0.0f) g_hot = true;
    }
    if (g.ActiveId != 0 && g.ActiveIdWindow == self && g.ActiveId != self->MoveId) g_hot = true;
    if (g_dragId != 0) g_hot = true;
    const ImVec2 at = ImGui::GetWindowPos(), size = ImGui::GetWindowSize();
    g_rectNow = ImVec4(at.x, at.y, at.x + size.x, at.y + size.y);

    ImGui::End();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar(5);
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

namespace {
NumberEdit commandNumberShown(const char* label, double value, const char* unit,
                              bool fixed, bool editing, const char* buffer,
                              double lo, double hi, bool signedRange, int places, bool isLength);
}

NumberEdit commandNumber(const char* label, double value, const char* unit,
                         bool fixed, bool editing, const char* buffer,
                         double lo, double hi, bool signedRange) {
    // A length is kept in millimetres and shown in the unit chosen: the bar,
    // its ends and what it hands back are converted here, once, for every
    // operation's panel.
    if (unit && std::strcmp(unit, "mm") == 0) {
        NumberEdit e = commandNumberShown(label, units::toShown(value), units::suffix(), fixed, editing, buffer,
                                          units::toShown(lo), units::toShown(hi), signedRange, units::decimals(),
                                          /*isLength=*/true);
        e.value = units::fromShown(e.value);
        return e;
    }
    return commandNumberShown(label, value, unit, fixed, editing, buffer, lo, hi, signedRange, 2, false);
}

namespace {
// What was typed into a bar, in the unit the bar shows. A length may carry a
// unit of its own ("12mm", "0.5in"); anything else is the number alone, with
// its sign or unit mark ("45", "45Â°", "20%") allowed after it.
bool parseTyped(const char* text, bool isLength, double& out) {
    if (isLength) {
        Real mm = 0.0;
        if (!units::parse(text, mm)) return false;
        out = units::toShown(mm);
        return true;
    }
    char* end = nullptr;
    const double v = std::strtod(text, &end);
    if (end == text) return false;
    out = v;
    return std::isfinite(v);
}

NumberEdit commandNumberShown(const char* label, double value, const char* unit,
                              bool fixed, bool editing, const char* buffer,
                              double lo, double hi, bool signedRange, int places, bool isLength) {
    NumberEdit out;
    out.value = value;
    ImGui::PushID(label);
    commandRow(label);

    // Typed into: a field in the bar's place until Enter, a click away or Esc.
    // What was typed comes back as a pull that has ended, which every caller
    // already takes as a value to set.
    const ImGuiID fieldId = ImGui::GetID("##field");
    if (g_typeId == fieldId && g_typeFrame + 1 < ImGui::GetFrameCount() && !g_typeFocus) g_typeId = 0;
    if (g_typeId == fieldId) {
        g_typeFrame = ImGui::GetFrameCount();
        const bool escaped = ImGui::IsKeyPressed(ImGuiKey_Escape, false);
        const ImVec2 size(ImGui::GetContentRegionAvail().x, ImGui::GetFrameHeight());
        const ImVec2 at = ImGui::GetCursorScreenPos();
        if (g_typeFocus) { ImGui::SetKeyboardFocusHere(); g_typeFocus = false; }
        ImGui::SetNextItemWidth(size.x);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, im(palette::kField));
        const bool entered = ImGui::InputText("##typed", g_typeBuf, sizeof g_typeBuf,
                                              ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll);
        ImGui::PopStyleColor();
        ImGui::GetWindowDrawList()->AddRect(at, ImVec2(at.x + size.x, at.y + size.y), u32(palette::kBrand), 5.0f);
        const bool leftAfterEdit = ImGui::IsItemDeactivatedAfterEdit();
        const bool left = ImGui::IsItemDeactivated() || (!ImGui::IsItemActive() && !g_typeFocus &&
                                                         ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
                                                         !ImGui::IsItemHovered());
        if (escaped) {
            g_typeId = 0;
        } else if (entered || leftAfterEdit) {
            double v = 0.0;
            if (parseTyped(g_typeBuf, isLength, v)) {
                out.dragged = true;
                out.released = true;
                out.value = std::fabs(v) < 1e-12 ? 0.0 : v;
            }
            g_typeId = 0;
        } else if (left) {
            g_typeId = 0;           // Esc, or a click away with nothing changed
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Type a number and press Enter. Esc leaves it as it was.");
        ImGui::PopID();
        return out;
    }

    char text[64];
    if (editing) std::snprintf(text, sizeof text, "%s_", buffer ? buffer : "");
    else         std::snprintf(text, sizeof text, "%.*f %s", places, value, unit ? unit : "");

    // A bar the width of the rest of the row, so the numbers line up down the
    // panel however long their labels are.
    const float w = ImGui::GetContentRegionAvail().x;
    const ImVec2 size(w, ImGui::GetFrameHeight());
    const ImVec2 at = ImGui::GetCursorScreenPos();

    ImGui::InvisibleButton("##field", size);
    const ImGuiID id = ImGui::GetItemID();
    const bool hot = ImGui::IsItemHovered();

    // A double-click types into it. The first click of the two has already
    // been reported as a click, which only hands the value to the pointer
    // until the typed one arrives.
    if (hot && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
        g_typeId = id;
        g_typeFocus = true;
        g_dragId = 0;
        std::snprintf(g_typeBuf, sizeof g_typeBuf, "%.*f", places, value);
        ImGui::ClearActiveID();
        ImGui::PopID();
        return out;
    }

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
        ImGui::SetTooltip("%s", ranged ? "Drag to set it, or double-click to type a number"
                                       : "Double-click to type a number");

    ImGui::PopID();
    return out;
}
} // namespace

int commandChoices(const char* label, const Choice* choices, int count, int active, bool compact) {
    if (count <= 0) return -1;
    int clicked = -1;
    ImGui::PushID(label);
    if (compact) {
        commandRow(label);
        for (int i = 0; i < count; ++i) {
            if (i) commandNextPill(choices[i].label);
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
    // The key goes in the tile's corner, small, as on a keycap: beside the
    // word it made the word fight for the width, and the word is what is read.
    const float gap = 6.0f;
    const float avail = ImGui::GetContentRegionAvail().x;
    const float each = (avail - gap * static_cast<float>(count - 1)) / static_cast<float>(count);
    const float tileH = 48.0f;
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
        drawGlyph(dl, choices[i].glyph, ImVec2(at.x + each * 0.5f, at.y + 17.0f), 19.0f, fg);

        pushFont(FontWeight::Medium, uiFonts().size * 0.86f);
        const ImVec2 ls = ImGui::CalcTextSize(choices[i].label);
        dl->AddText(ImVec2(at.x + std::max(4.0f, (each - ls.x) * 0.5f), hi.y - ls.y - 6.0f), fg,
                    choices[i].label);
        ImGui::PopFont();
        if (choices[i].key && *choices[i].key) {
            pushFont(FontWeight::Medium, uiFonts().size * 0.72f);
            const ImVec2 ks = ImGui::CalcTextSize(choices[i].key);
            dl->AddText(ImVec2(hi.x - ks.x - 6.0f, at.y + 4.0f),
                        on ? IM_COL32(255, 255, 255, 170)
                           : live ? u32(palette::kTextFaint) : u32(palette::kTextFaint, 0.6f),
                        choices[i].key);
            ImGui::PopFont();
        }
        hoverTip(choices[i].tip);
        ImGui::PopID();
    }
    ImGui::PopID();
    return clicked;
}

void commandNextPill(const char* label, float gap) {
    ImGui::SameLine(0.0f, gap);
    // What pillButton will make of it: the words and its padding either side.
    const float w = ImGui::CalcTextSize(label).x + 22.0f;
    if (ImGui::GetCursorPosX() + w > ImGui::GetWindowContentRegionMax().x) {
        ImGui::NewLine();
        ImGui::SetCursorPosX(kLabelColumn);
    }
}

void commandHint(const char* text) {
    if (!text || !*text) return;
    if (!g_help.empty()) g_help += "\n\n";
    g_help += text;
}

void commandApplied(const char* what) {
    // Two words and a dot of colour. That it can still be adjusted is what the
    // ? says; that it has been made is what has to be seen at a glance.
    ImGui::Dummy(ImVec2(0, 1));
    pushFont(FontWeight::Medium, uiFonts().size * 0.9f);
    ImGui::PushStyleColor(ImGuiCol_Text, im(palette::kValid));
    ImGui::Text("%s applied", what);
    ImGui::PopStyleColor();
    ImGui::PopFont();
}

void commandRefused(const char* why) {
    ImGui::Dummy(ImVec2(0, 1));
    pushFont(FontWeight::Medium, uiFonts().size * 0.9f);
    ImGui::PushStyleColor(ImGuiCol_Text, im(palette::kBrand));
    ImGui::PushTextWrapPos(0.0f);
    ImGui::Text("Not made: %s", why && *why ? why : "the kernel refused");
    ImGui::PopTextWrapPos();
    ImGui::PopStyleColor();
    ImGui::PopFont();
}

int commandFooter(const char* commitLabel, bool commitEnabled, const char* cancelLabel) {
    ImGui::Dummy(ImVec2(0, 2));
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

// ---------------------------------------------------------------------------
// Key hints

namespace {
struct KeyHintRow {
    std::string keys;
    std::string what;
};
std::vector<KeyHintRow> g_keyHints;

enum class MouseButton { None, Left, Right, Middle, Wheel };

MouseButton mouseToken(const std::string& t) {
    if (t == "LMB") return MouseButton::Left;
    if (t == "RMB") return MouseButton::Right;
    if (t == "MMB") return MouseButton::Middle;
    if (t == "Wheel") return MouseButton::Wheel;
    return MouseButton::None;
}

std::vector<std::string> splitWords(const std::string& s) {
    std::vector<std::string> out;
    size_t i = 0;
    while (i < s.size()) {
        while (i < s.size() && s[i] == ' ') ++i;
        size_t j = i;
        while (j < s.size() && s[j] != ' ') ++j;
        if (j > i) out.push_back(s.substr(i, j - i));
        i = j;
    }
    return out;
}

// A mouse, 12 by 17, with one part of it lit.
void drawMouse(ImDrawList* dl, ImVec2 at, float h, MouseButton lit, ImU32 line, ImU32 on) {
    const float w = h * 0.7f;
    const ImVec2 lo = at, hi(at.x + w, at.y + h);
    const float r = w * 0.5f;
    const float split = at.y + h * 0.42f;
    const float mid = at.x + w * 0.5f;
    if (lit == MouseButton::Left)
        dl->AddRectFilled(lo, ImVec2(mid, split), on, r, ImDrawFlags_RoundCornersTopLeft);
    if (lit == MouseButton::Right)
        dl->AddRectFilled(ImVec2(mid, lo.y), ImVec2(hi.x, split), on, r, ImDrawFlags_RoundCornersTopRight);
    dl->AddRect(lo, hi, line, r, 0, 1.3f);
    dl->AddLine(ImVec2(lo.x, split), ImVec2(hi.x, split), line, 1.2f);
    dl->AddLine(ImVec2(mid, lo.y), ImVec2(mid, split), line, 1.2f);
    if (lit == MouseButton::Middle || lit == MouseButton::Wheel) {
        const float ww = w * 0.22f;
        dl->AddRectFilled(ImVec2(mid - ww, lo.y + h * 0.1f), ImVec2(mid + ww, split - h * 0.06f), on, ww);
    }
}
} // namespace

void keyHint(const char* keys, const char* what) {
    if (!keys || !*keys) return;
    g_keyHints.push_back({keys, what ? what : ""});
}

void clearKeyHints() { g_keyHints.clear(); }

float drawKeyHints(ImDrawList* dl, float right, float bottom) {
    if (g_keyHints.empty()) return 0.0f;

    pushFont(FontWeight::Medium, uiFonts().size * 0.78f);
    const float textH = ImGui::GetTextLineHeight();
    const float capH = std::round(textH + 7.0f);
    const float capPad = 6.0f, capGap = 3.0f, rowGap = 5.0f;
    const float mouseH = capH - 1.0f, mouseW = mouseH * 0.7f;

    // Widths first, so the caps line up in a column and the words after them.
    struct Piece { std::string text; MouseButton mouse; bool cap; float w; };
    std::vector<std::vector<Piece>> rows;
    float keysW = 0.0f;
    for (const KeyHintRow& row : g_keyHints) {
        std::vector<Piece> pieces;
        float w = 0.0f;
        for (const std::string& word : splitWords(row.keys)) {
            Piece p{word, mouseToken(word), word != "+" && word != "/", 0.0f};
            if (p.mouse != MouseButton::None) p.w = mouseW + 4.0f;
            else if (p.cap) p.w = std::max(capH, ImGui::CalcTextSize(word.c_str()).x + capPad * 2.0f);
            else            p.w = ImGui::CalcTextSize(word.c_str()).x + 2.0f;
            if (!pieces.empty()) w += capGap;
            w += p.w;
            pieces.push_back(std::move(p));
        }
        keysW = std::max(keysW, w);
        rows.push_back(std::move(pieces));
    }
    ImGui::PopFont();
    pushFont(FontWeight::Regular, uiFonts().size * 0.82f);
    float whatW = 0.0f;
    for (const KeyHintRow& row : g_keyHints) whatW = std::max(whatW, ImGui::CalcTextSize(row.what.c_str()).x);
    ImGui::PopFont();

    const float pad = 10.0f, gapCol = 10.0f;
    const float boxW = pad * 2.0f + keysW + gapCol + whatW;
    const float boxH = pad * 2.0f + capH * static_cast<float>(rows.size()) +
                       rowGap * static_cast<float>(rows.size() - 1);
    const ImVec2 lo(right - boxW, bottom - boxH), hi(right, bottom);
    dl->AddRectFilled(lo, hi, u32(palette::kCommand, 0.78f), 8.0f);
    dl->AddRect(lo, hi, u32(palette::kBorder, 0.7f), 8.0f);

    const ImU32 capFill = u32(palette::kRaised, 0.95f);
    const ImU32 capLine = u32(palette::kBorderStrong);
    const ImU32 capText = u32(palette::kText);
    const ImU32 dim = u32(palette::kTextDim);
    float y = lo.y + pad;
    for (size_t i = 0; i < rows.size(); ++i) {
        float x = lo.x + pad;
        pushFont(FontWeight::Medium, uiFonts().size * 0.78f);
        for (size_t k = 0; k < rows[i].size(); ++k) {
            const Piece& p = rows[i][k];
            if (k) x += capGap;
            if (p.mouse != MouseButton::None) {
                drawMouse(dl, ImVec2(x + 2.0f, y + (capH - mouseH) * 0.5f), mouseH, p.mouse, capText,
                          u32(palette::kBrand));
            } else if (p.cap) {
                dl->AddRectFilled(ImVec2(x, y), ImVec2(x + p.w, y + capH), capFill, 4.0f);
                dl->AddRect(ImVec2(x, y), ImVec2(x + p.w, y + capH), capLine, 4.0f);
                // A cap's bottom lip, as a key has.
                dl->AddLine(ImVec2(x + 3.0f, y + capH - 0.5f), ImVec2(x + p.w - 3.0f, y + capH - 0.5f),
                            u32(palette::kBorderStrong, 0.9f), 1.5f);
                const ImVec2 ts = ImGui::CalcTextSize(p.text.c_str());
                dl->AddText(ImVec2(x + (p.w - ts.x) * 0.5f, y + (capH - ts.y) * 0.5f - 0.5f), capText,
                            p.text.c_str());
            } else {
                const ImVec2 ts = ImGui::CalcTextSize(p.text.c_str());
                dl->AddText(ImVec2(x + 1.0f, y + (capH - ts.y) * 0.5f), dim, p.text.c_str());
            }
            x += p.w;
        }
        ImGui::PopFont();
        pushFont(FontWeight::Regular, uiFonts().size * 0.82f);
        const ImVec2 ts = ImGui::CalcTextSize(g_keyHints[i].what.c_str());
        dl->AddText(ImVec2(lo.x + pad + keysW + gapCol, y + (capH - ts.y) * 0.5f), u32(palette::kText, 0.92f),
                    g_keyHints[i].what.c_str());
        ImGui::PopFont();
        y += capH + rowGap;
    }
    g_keyHints.clear();
    return boxH;
}

} // namespace tg::ui
