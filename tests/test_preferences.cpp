// Preferences: every setting written is the setting read back, and a person
// who has never started Tangent gets the defaults a first start should have.
//
// The failure this guards against is quiet: a field added to Preferences but
// left out of the reader or the writer. The app goes on working, and the choice
// is simply forgotten at the next start.
#include "app/preferences.h"
#include "temp_path.h"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}

int main() {
    // A config directory of its own, so the test never touches a real one.
    const std::string dir = tempPath("prefs");
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
#ifdef _WIN32
    _putenv_s("XDG_CONFIG_HOME", dir.c_str());
#else
    setenv("XDG_CONFIG_HOME", dir.c_str(), 1);
#endif

    // Nothing saved yet: the defaults a first start opens with.
    {
        Preferences p;
        check(!loadPreferences(p), "no file yet reads as nothing saved");
        check(!p.orthographic, "opens in perspective");
        check(!p.showPrintIssues, "print problems start hidden");
        check(p.showGrid && p.showEdges && p.showKeyHints, "grid, edges and key hints start shown");
        check(!layoutPath().empty() && layoutPath().rfind(dir, 0) == 0, "the layout is kept in the config directory");
    }

    // Every field away from its default, saved, and read back.
    Preferences out;
    out.units = units::Length::Inch;
    out.lightTheme = true;
    out.invertOrbitX = true;
    out.invertOrbitY = true;
    out.orthographic = true;
    out.showPrintIssues = true;
    out.showGrid = false;
    out.showEdges = false;
    out.showSelectionBox = true;
    out.backfaceCulling = false;
    out.showKeyHints = false;
    out.snap = false;
    out.nozzleMm = 0.6;
    out.minWallMm = 1.2;
    out.autosaveMinutes = 5;
    rememberRecent(out, dir + "/a.tangent");
    rememberRecent(out, dir + "/b.tangent");
    check(savePreferences(out), "saves");

    Preferences in;
    check(loadPreferences(in), "reads back");
    check(in.units == units::Length::Inch, "units");
    check(in.lightTheme, "theme");
    check(in.invertOrbitX && in.invertOrbitY, "orbit inversion");
    check(in.orthographic, "projection");
    check(in.showPrintIssues, "print problems");
    check(!in.showGrid, "grid");
    check(!in.showEdges, "edges");
    check(in.showSelectionBox, "selection box");
    check(!in.backfaceCulling, "backface culling");
    check(!in.showKeyHints, "key hints");
    check(!in.snap, "snapping");
    check(in.nozzleMm == 0.6 && in.minWallMm == 1.2, "printer");
    check(in.autosaveMinutes == 5, "autosave");
    check(in.recentFiles.size() == 2 && in.recentFiles[0].find("b.tangent") != std::string::npos,
          "recent files, most recent first");

    std::filesystem::remove_all(dir);
    if (failures) { std::printf("%d failure(s)\n", failures); return 1; }
    std::printf("preferences: all passed\n");
    return 0;
}
