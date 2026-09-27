// Tangent - the application's side of preferences: reading them in, putting
// them into effect, and the dialog they are changed in.
//
// Every change takes effect the moment it is made and is saved at once: there
// is no Apply and no Cancel to remember, and a crash a minute later loses
// nothing that was chosen.
#include "app/application.h"

#include "core/palette.h"
#include "core/crashlog.h"
#include "app/crash.h"
#include "core/units.h"
#include "scene/serialize.h"
#include "ui/theme.h"
#include "ui/widgets.h"

#include <imgui.h>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <ctime>

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

// ---------------------------------------------------------------------------
// Autosave and recovery
// ---------------------------------------------------------------------------

namespace {
std::string recoveryPath() {
    const std::string dir = configDirectory();
    return dir.empty() ? std::string() : dir + "/recovery.tangent";
}
std::string recoveryNote() {
    const std::string dir = configDirectory();
    return dir.empty() ? std::string() : dir + "/recovery.note";
}
} // namespace

void Application::stepAutosave() {
    if (unattended_ || prefs_.autosaveMinutes <= 0) return;
    if (!dirty() || undo_.revision() == autosavedRevision_) {
        autosaveClock_ = 0.0f;
        return;
    }
    autosaveClock_ += lastDt_;
    if (autosaveClock_ < static_cast<float>(prefs_.autosaveMinutes) * 60.0f) return;
    // At a quiet moment: not in the middle of a drag or an operation, when
    // what is on screen is not yet what the project holds.
    if (ImGui::IsMouseDown(ImGuiMouseButton_Left) || commandCornerTaken()) return;
    autosaveClock_ = 0.0f;
    if (writeRecovery()) autosavedRevision_ = undo_.revision();
}

bool Application::writeRecovery() {
    const std::string path = recoveryPath();
    if (path.empty() || !saveProject(scene_, path).ok) return false;
    // What it was, and when, for the offer to recover it.
    if (FILE* f = std::fopen(recoveryNote().c_str(), "w")) {
        const std::time_t now = std::time(nullptr);
        char when[64];
        std::strftime(when, sizeof when, "%H:%M on %e %b", std::localtime(&now));
        std::fprintf(f, "%s\n%s\n", projectPath_.c_str(), when);
        std::fclose(f);
    }
    return true;
}

namespace {
// Called in a forked copy of the crashed process: the work, saved aside.
crash::Saved emergencySave(void* context) {
    return static_cast<Application*>(context)->saveAsideForCrash();
}
std::string crashDirectory() {
    const std::string dir = configDirectory();
    return dir.empty() ? std::string() : dir + "/crashes";
}
} // namespace

crash::Saved Application::saveAsideForCrash() {
    if (!dirty()) return crash::Saved::Nothing;
    return writeRecovery() ? crash::Saved::Done : crash::Saved::Failed;
}

void Application::installCrashHandler() {
    // Not for a demo or a test, which would leave a report -- and work to
    // recover -- in the person's own settings; except the crash test itself.
    const std::string dir = crashDirectory();
    if (dir.empty()) return;
    if (!unattended_ || offerRecoveryDemo_) crashReport_ = crash::pendingReport(dir);
    if (unattended_ && !crashTest_) return;
#if defined(TANGENT_BUILD)
    const char* build = TANGENT_BUILD;
#else
    const char* build = "unknown";
#endif
    crash::install(dir, build, &emergencySave, this);
    crashlog::note("started, build %s", build);
}

void Application::clearRecovery() {
    if (unattended_) return;
    std::error_code ec;
    std::filesystem::remove(recoveryPath(), ec);
    std::filesystem::remove(recoveryNote(), ec);
    autosavedRevision_ = static_cast<size_t>(-1);
}

void Application::checkRecovery() {
    installCrashHandler();
    if (unattended_ && !offerRecoveryDemo_) return;
    std::error_code ec;
    if (!std::filesystem::exists(recoveryPath(), ec)) return;
    recoveryOffered_ = true;
    if (FILE* f = std::fopen(recoveryNote().c_str(), "r")) {
        char line[1024];
        if (std::fgets(line, sizeof line, f)) {
            recoveryFrom_ = line;
            while (!recoveryFrom_.empty() && (recoveryFrom_.back() == '\n' || recoveryFrom_.back() == '\r')) recoveryFrom_.pop_back();
        }
        if (std::fgets(line, sizeof line, f)) {
            recoveryWhen_ = line;
            while (!recoveryWhen_.empty() && (recoveryWhen_.back() == '\n' || recoveryWhen_.back() == '\r')) recoveryWhen_.pop_back();
        }
        std::fclose(f);
    }
}

void Application::drawRecoveryPrompt() {
    // A crash with nothing unsaved: said, with where the report is.
    if (!recoveryOffered_ && !crashReport_.empty()) {
        ImGui::OpenPopup("##crashed");
        if (ui::beginCard("##crashed", "Tangent quit unexpectedly", 440.0f)) {
            ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + 404.0f);
            ImGui::TextColored(ui::im(palette::kTextDim), "Nothing unsaved was lost. What happened was written to:");
            ImGui::PopTextWrapPos();
            ImGui::TextUnformatted(crashReport_.c_str());
            ImGui::Dummy(ImVec2(0, 10));
            if (ui::quietButton("Copy the path", ImVec2(130, 0))) ImGui::SetClipboardText(crashReport_.c_str());
            ImGui::SameLine();
            if (ui::primaryButton("OK", ImVec2(90, 0)) || ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
                crash::acknowledge(crashDirectory());
                crashReport_.clear();
                ImGui::CloseCurrentPopup();
            }
            ui::endCard();
        }
        return;
    }
    if (!recoveryOffered_) return;
    ImGui::OpenPopup("##recover");
    if (!ui::beginCard("##recover", "Recover unsaved work?", 420.0f)) return;
    const std::string name = recoveryFrom_.empty() ? std::string("An unsaved project")
                                                   : std::filesystem::path(recoveryFrom_).filename().string();
    ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + 384.0f);
    ImGui::TextColored(ui::im(palette::kTextDim),
                       "Tangent %s with changes that were not saved. %s was kept aside%s%s.",
                       crashReport_.empty() ? "closed" : "quit unexpectedly", name.c_str(),
                       recoveryWhen_.empty() ? "" : " at ", recoveryWhen_.c_str());
    if (!crashReport_.empty())
        ImGui::TextColored(ui::im(palette::kTextFaint), "What happened was written to %s", crashReport_.c_str());
    ImGui::PopTextWrapPos();
    ImGui::Dummy(ImVec2(0, 10));
    if (ui::primaryButton("Recover", ImVec2(110, 0))) {
        // Replaces the scene only if it reads; the file is kept until the
        // work is saved or the next clean exit.
        const ProjectResult r = loadProject(scene_, recoveryPath());
        if (r.ok) {
            undo_.clear();
            projectPath_ = recoveryFrom_;
            // Not saved: it is work to keep, and saying it is saved would lose it.
            savedRevision_ = undo_.revision() + 1;
            camera_.frame(scene_.bounds());
            setNotice("Recovered: save it to keep it");
        } else {
            setNotice("The kept work could not be read: " + r.error);
        }
        recoveryOffered_ = false;
        crash::acknowledge(crashDirectory());
        crashReport_.clear();
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ui::quietButton("Discard", ImVec2(110, 0))) {
        clearRecovery();
        recoveryOffered_ = false;
        crash::acknowledge(crashDirectory());
        crashReport_.clear();
        ImGui::CloseCurrentPopup();
    }
    ui::endCard();
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
