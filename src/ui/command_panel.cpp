#include "ui/command_panel.h"

#include "core/palette.h"

namespace tg::ui {
namespace {

constexpr float kLabelColumn = 92.0f;
constexpr float kPanelWidth  = 268.0f;

ImVec4 im(Rgb c, float a = 1.0f) {
    return ImVec4(static_cast<float>(c.r), static_cast<float>(c.g),
                  static_cast<float>(c.b), a);
}

} // namespace

float commandLabelWidth() { return kLabelColumn; }

bool beginCommand(const char* id, const char* title, Icon icon, float x, float y) {
    ImGui::SetNextWindowPos(ImVec2(x, y), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(kPanelWidth, 0.0f), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.94f);

    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoDocking |
        ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav |
        ImGuiWindowFlags_AlwaysAutoResize;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 10.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 7.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);
    // A quiet border. The accent belongs to the one control that finishes the
    // operation; spending it on the outline of the box makes the box shout and
    // the button ordinary.
    ImGui::PushStyleColor(ImGuiCol_Border, im(palette::kBorder, 0.9f));

    if (!ImGui::Begin(id, nullptr, flags)) {
        ImGui::End();
        ImGui::PopStyleColor();
        ImGui::PopStyleVar(3);
        return false;
    }

    // The title, with the operation's own picture beside it.
    const float line = ImGui::GetTextLineHeight();
    if (iconsReady() && icon != Icon::Count) {
        iconImage(icon, line);
        ImGui::SameLine(0.0f, 8.0f);
    }
    ImGui::TextColored(im(palette::kText), "%s", title);
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    return true;
}

void endCommand() {
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(3);
}

void commandRow(const char* label) {
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(im(palette::kTextDim), "%s", label);
    ImGui::SameLine(kLabelColumn);
}

void commandValue(const char* label, const char* value) {
    commandRow(label);
    ImGui::TextUnformatted(value);
}

bool commandNumber(const char* label, double value, const char* unit,
                   bool fixed, bool editing, const char* buffer) {
    ImGui::PushID(label);
    commandRow(label);

    char text[64];
    if (editing) std::snprintf(text, sizeof text, "%s_", buffer ? buffer : "");
    else         std::snprintf(text, sizeof text, "%.2f %s", value, unit ? unit : "");

    // A box the width of the rest of the row, so the numbers line up down the
    // panel however long their labels are.
    const float w = ImGui::GetContentRegionAvail().x;
    const ImVec2 size(w, ImGui::GetFrameHeight());
    const ImVec2 at = ImGui::GetCursorScreenPos();

    const bool clicked = ImGui::InvisibleButton("##field", size);
    const bool hot = ImGui::IsItemHovered();

    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 lo = at, hi = ImVec2(at.x + size.x, at.y + size.y);
    dl->AddRectFilled(lo, hi, ImGui::GetColorU32(ImGuiCol_FrameBg), 3.0f);
    if (fixed || editing || hot)
        dl->AddRect(lo, hi,
                    ImGui::GetColorU32(im(palette::kBrand, editing ? 1.0f : (fixed ? 0.8f : 0.35f))),
                    3.0f);

    const ImVec2 ts = ImGui::CalcTextSize(text);
    dl->AddText(ImVec2(hi.x - ts.x - 8.0f, at.y + (size.y - ts.y) * 0.5f),
                ImGui::GetColorU32(fixed || editing ? im(palette::kBrand) : im(palette::kText)),
                text);

    ImGui::PopID();
    return clicked;
}

void commandHint(const char* text) {
    ImGui::Spacing();
    ImGui::PushStyleColor(ImGuiCol_Text, im(palette::kTextDim));
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextUnformatted(text);
    ImGui::PopTextWrapPos();
    ImGui::PopStyleColor();
}

int commandFooter(const char* commitLabel, bool commitEnabled, const char* cancelLabel) {
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    int result = 0;
    const float avail = ImGui::GetContentRegionAvail().x;
    const float cancelW = ImGui::CalcTextSize(cancelLabel).x + ImGui::GetStyle().FramePadding.x * 2.0f;

    ImGui::PushStyleColor(ImGuiCol_Button, im(palette::kBrand, 0.85f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, im(palette::kBrand, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, im(palette::kBrand, 0.7f));
    ImGui::BeginDisabled(!commitEnabled);
    if (commitLabel) {
        if (ImGui::Button(commitLabel, ImVec2(avail - cancelW - ImGui::GetStyle().ItemSpacing.x, 0.0f)))
            result = 1;
        ImGui::SameLine();
    }
    ImGui::EndDisabled();
    ImGui::PopStyleColor(3);

    if (!commitLabel) ImGui::SetCursorPosX(ImGui::GetCursorPosX() + avail - cancelW);
    if (ImGui::Button(cancelLabel, ImVec2(commitLabel ? cancelW : cancelW, 0.0f))) result = -1;
    return result;
}

} // namespace tg::ui
