// Tangent - the application's side of preferences: reading them in, putting
// them into effect, and the dialog they are changed in.
//
// Every change takes effect the moment it is made and is saved at once: there
// is no Apply and no Cancel to remember, and a crash a minute later loses
// nothing that was chosen.
#include "app/application.h"

#include "core/palette.h"
#include "core/units.h"
#include "ui/theme.h"
#include "ui/widgets.h"

#include <imgui.h>

#include <algorithm>
#include <cstdio>

namespace tg {

void Application::applyPreferences() {
    units::setCurrent(prefs_.units);
    camera_.invertOrbitX = prefs_.invertOrbitX;
    camera_.invertOrbitY = prefs_.invertOrbitY;
    applyAppTheme(prefs_.lightTheme);
    ui_.recentFiles = prefs_.recentFiles;
}

void Application::savePrefs() {
    ui_.recentFiles = prefs_.recentFiles;
    if (unattended_) return;          // a demo leaves a person's settings alone
    savePreferences(prefs_);
}

void Application::applyAppTheme(bool light) {
    applyTheme(light);
    // The view's colours were copied out of the palette when it was made.
    view_.background = toVec3(palette::kViewport);
    view_.objectColor = toVec3(palette::kSurface);
    view_.edgeColor = toVec4(palette::kEdge, palette::isLight() ? 0.75f : 0.85f);
}

PrintProfile Application::printProfile() const {
    PrintProfile p;
    p.nozzleMm = prefs_.nozzleMm;
    p.minWallMm = prefs_.minWallMm;
    return p;
}

bool Application::snapNow() const {
    // Snapping is the default and Ctrl lets go of it -- or, with it turned
    // off in the preferences, the other way about.
    return prefs_.snap != ImGui::GetIO().KeyCtrl;
}

void Application::drawPreferences() {
    if (prefsOpen_ && !ImGui::IsPopupOpen("##prefs")) ImGui::OpenPopup("##prefs");
    if (!ui::beginCard("##prefs", "Preferences", std::min(520.0f, ImGui::GetMainViewport()->Size.x - 32.0f)))
        return;

    bool changed = false;
    const ImVec4 dim = ui::im(palette::kTextDim);
    auto label = [&](const char* text) {
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(dim, "%s", text);
        ImGui::SameLine(ui::labelColumn() + 20.0f);
    };
    auto choice = [&](const char* text, bool on) {
        const bool clicked = ui::pillButton(text, on);
        ImGui::SameLine(0.0f, 3.0f);
        return clicked && !on;
    };
    auto endRow = [] { ImGui::NewLine(); };
    auto section = [](const char* text) {
        ImGui::Dummy(ImVec2(0, 6));
        ui::sectionTitle(text);
    };

    section("Appearance");
    label("Theme");
    if (choice("Dark", !prefs_.lightTheme)) { prefs_.lightTheme = false; changed = true; }
    if (choice("Light", prefs_.lightTheme)) { prefs_.lightTheme = true; changed = true; }
    endRow();
    label("Units");
    for (units::Length u : {units::Length::Millimetre, units::Length::Centimetre, units::Length::Inch})
        if (choice(units::nameOf(u), prefs_.units == u)) { prefs_.units = u; changed = true; }
    endRow();
    ui::captionText("Lengths are shown and typed in these. Anything can be typed with its own unit too: 12mm, 1.5cm, 0.5in.");

    section("Navigation");
    label("Orbit");
    if (ui::pillButton("Invert left-right", prefs_.invertOrbitX)) { prefs_.invertOrbitX = !prefs_.invertOrbitX; changed = true; }
    ImGui::SameLine(0.0f, 3.0f);
    if (ui::pillButton("Invert up-down", prefs_.invertOrbitY)) { prefs_.invertOrbitY = !prefs_.invertOrbitY; changed = true; }
    endRow();
    label("Projection");
    if (choice("Orthographic", prefs_.orthographic)) {
        prefs_.orthographic = true;
        camera_.setOrthographic(true);
        changed = true;
    }
    if (choice("Perspective", !prefs_.orthographic)) {
        prefs_.orthographic = false;
        camera_.setOrthographic(false);
        changed = true;
    }
    endRow();

    section("Modelling");
    label("Snapping");
    if (choice("On, Ctrl lets go", prefs_.snap)) { prefs_.snap = true; changed = true; }
    if (choice("Off, Ctrl snaps", !prefs_.snap)) { prefs_.snap = false; changed = true; }
    endRow();

    section("Printer");
    {
        Real nozzle = prefs_.nozzleMm, wall = prefs_.minWallMm;
        if (ui::labelledNumber("Nozzle", nozzle, 0.01f, 0.1, 2.0, "%.2f mm")) { prefs_.nozzleMm = nozzle; changed = true; }
        if (ui::labelledNumber("Thinnest wall", wall, 0.01f, 0.1, 10.0, "%.2f mm")) { prefs_.minWallMm = wall; changed = true; }
        ui::captionText("Walls thinner than this are drawn red on the part. Two lines of the nozzle is usual.");
    }

    section("Files");
    label("Autosave");
    for (int m : {0, 1, 2, 5, 10}) {
        char text[16];
        if (m == 0) std::snprintf(text, sizeof text, "Off");
        else        std::snprintf(text, sizeof text, "%d min", m);
        ImGui::PushID(m);
        if (choice(text, prefs_.autosaveMinutes == m)) { prefs_.autosaveMinutes = m; changed = true; }
        ImGui::PopID();
    }
    endRow();
    label("Recent files");
    if (ui::quietButton("Clear the list", ImVec2(0, 0), !prefs_.recentFiles.empty())) {
        prefs_.recentFiles.clear();
        changed = true;
    }
    endRow();

    section("Keyboard");
    ImGui::TextColored(dim, "Every command and its key: Ctrl+K, then type.");

    ImGui::Dummy(ImVec2(0, 10));
    const float w = 90.0f;
    ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - w);
    if (ui::primaryButton("Done", ImVec2(w, 0)) || ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
        prefsOpen_ = false;
        ImGui::CloseCurrentPopup();
    }
    ui::endCard();

    if (changed) {
        const units::Length before = units::current();
        applyPreferences();
        savePrefs();
        // The print check measured against the old printer: done again.
        for (const auto& o : scene_.objects()) o->printVersion = 0;
        if (before != units::current()) highlightKey_ = 0;
    }
}

} // namespace tg
