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
    bool orthographic = true;

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

bool loadPreferences(Preferences& out);
bool savePreferences(const Preferences& prefs);

// Puts `path` at the front of the recent files, once, keeping ten.
void rememberRecent(Preferences& prefs, const std::string& path);

} // namespace tg
