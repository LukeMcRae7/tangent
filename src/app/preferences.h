// Tangent - what a person has chosen once and does not want to choose again.
//
// Kept in one small text file of key = value lines, in the user's config
// directory ($XDG_CONFIG_HOME/tangent, or ~/.config/tangent), so it survives a
// rebuild, a new checkout and a new project. Not read by an unattended run --
// a demo, a smoke test -- which has to behave the same on every machine.
#pragma once

#include "core/units.h"

#include <string>
#include <vector>

namespace tg {

struct Preferences {
    // Appearance
    units::Length units = units::Length::Millimetre;
    bool lightTheme = false;

    // Navigation
    bool invertOrbitX = false;
    bool invertOrbitY = false;
    // Perspective to start with: a first look at a part is a look at the
    // thing. Numpad 5, the view cube and the View menu change it, and whichever
    // was last asked for is what the next start opens to.
    bool orthographic = false;

    // The view's toggles (Inspect menu), as last left.
    bool showPrintIssues = false;
    bool showGrid = true;
    bool showEdges = true;
    bool showSelectionBox = false;
    bool backfaceCulling = true;
    bool showKeyHints = true;

    // Modelling: snapping is on unless Ctrl is held; turned off here, Ctrl
    // turns it on instead.
    bool snap = true;

    // Printing: what the print check measures walls against.
    double nozzleMm = 0.4;
    double minWallMm = 0.8;

    // Files: minutes between autosaves of changed work; 0 turns it off.
    int autosaveMinutes = 2;
    std::vector<std::string> recentFiles;      // most recent first
};

// The directory preferences, autosaves and recovery files live in, created if
// it is not there. Empty when there is nowhere to put them.
std::string configDirectory();

// Where the panels' layout is kept: in the config directory, so it is the same
// whichever directory Tangent was started from. Empty when there is nowhere.
std::string layoutPath();

bool loadPreferences(Preferences& out);
bool savePreferences(const Preferences& prefs);

// Puts `path` at the front of the recent files, once, keeping ten.
void rememberRecent(Preferences& prefs, const std::string& path);

} // namespace tg
