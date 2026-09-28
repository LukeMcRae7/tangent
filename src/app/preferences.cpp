#include "app/preferences.h"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace tg {

std::string configDirectory() {
    namespace fs = std::filesystem;
    fs::path dir;
    if (const char* x = std::getenv("XDG_CONFIG_HOME"); x && *x) dir = fs::path(x) / "tangent";
    else if (const char* a = std::getenv("APPDATA"); a && *a) dir = fs::path(a) / "Tangent";
    else if (const char* h = std::getenv("HOME"); h && *h) dir = fs::path(h) / ".config" / "tangent";
    else return {};
    std::error_code ec;
    fs::create_directories(dir, ec);
    return ec ? std::string() : dir.string();
}

std::string layoutPath() {
    const std::string dir = configDirectory();
    return dir.empty() ? std::string() : dir + "/layout.ini";
}

namespace {

std::string prefsPath() {
    const std::string dir = configDirectory();
    return dir.empty() ? std::string() : dir + "/preferences.ini";
}

bool truthy(const std::string& v) { return v == "1" || v == "true" || v == "yes" || v == "on"; }

} // namespace

bool loadPreferences(Preferences& out) {
    const std::string path = prefsPath();
    if (path.empty()) return false;
    std::ifstream in(path);
    if (!in) return false;
    Preferences p;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        const size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        auto trim = [](std::string s) {
            while (!s.empty() && (s.back() == ' ' || s.back() == '\r')) s.pop_back();
            while (!s.empty() && s.front() == ' ') s.erase(s.begin());
            return s;
        };
        const std::string key = trim(line.substr(0, eq)), value = trim(line.substr(eq + 1));
        if (key == "units") {
            p.units = value == "cm" ? units::Length::Centimetre
                    : value == "in" ? units::Length::Inch : units::Length::Millimetre;
        }
        else if (key == "theme")          p.lightTheme = value == "light";
        else if (key == "invert_orbit_x") p.invertOrbitX = truthy(value);
        else if (key == "invert_orbit_y") p.invertOrbitY = truthy(value);
        else if (key == "orthographic")   p.orthographic = truthy(value);
        else if (key == "snap")           p.snap = truthy(value);
        else if (key == "print_problems") p.showPrintIssues = truthy(value);
        else if (key == "grid")           p.showGrid = truthy(value);
        else if (key == "edges")          p.showEdges = truthy(value);
        else if (key == "selection_box")  p.showSelectionBox = truthy(value);
        else if (key == "backface_cull")  p.backfaceCulling = truthy(value);
        else if (key == "key_hints")      p.showKeyHints = truthy(value);
        else if (key == "nozzle_mm")      p.nozzleMm = std::clamp(std::atof(value.c_str()), 0.05, 5.0);
        else if (key == "min_wall_mm")    p.minWallMm = std::clamp(std::atof(value.c_str()), 0.05, 20.0);
        else if (key == "autosave_minutes") p.autosaveMinutes = std::clamp(std::atoi(value.c_str()), 0, 120);
        else if (key == "recent" && !value.empty() && p.recentFiles.size() < 10) p.recentFiles.push_back(value);
    }
    out = p;
    return true;
}

bool savePreferences(const Preferences& p) {
    const std::string path = prefsPath();
    if (path.empty()) return false;
    std::ostringstream o;
    o << "# Tangent preferences. Edited by Tangent; safe to edit by hand while it is closed.\n";
    o << "units = " << units::suffixOf(p.units) << "\n";
    o << "theme = " << (p.lightTheme ? "light" : "dark") << "\n";
    o << "invert_orbit_x = " << (p.invertOrbitX ? 1 : 0) << "\n";
    o << "invert_orbit_y = " << (p.invertOrbitY ? 1 : 0) << "\n";
    o << "orthographic = " << (p.orthographic ? 1 : 0) << "\n";
    o << "snap = " << (p.snap ? 1 : 0) << "\n";
    o << "print_problems = " << (p.showPrintIssues ? 1 : 0) << "\n";
    o << "grid = " << (p.showGrid ? 1 : 0) << "\n";
    o << "edges = " << (p.showEdges ? 1 : 0) << "\n";
    o << "selection_box = " << (p.showSelectionBox ? 1 : 0) << "\n";
    o << "backface_cull = " << (p.backfaceCulling ? 1 : 0) << "\n";
    o << "key_hints = " << (p.showKeyHints ? 1 : 0) << "\n";
    o << "nozzle_mm = " << p.nozzleMm << "\n";
    o << "min_wall_mm = " << p.minWallMm << "\n";
    o << "autosave_minutes = " << p.autosaveMinutes << "\n";
    for (const std::string& r : p.recentFiles) o << "recent = " << r << "\n";
    // Written beside and moved over, so a crash mid-write leaves the old file.
    const std::string tmp = path + ".tmp";
    {
        std::ofstream out(tmp, std::ios::trunc);
        if (!out) return false;
        out << o.str();
        if (!out) return false;
    }
    std::error_code ec;
    std::filesystem::rename(tmp, path, ec);
    return !ec;
}

void rememberRecent(Preferences& prefs, const std::string& path) {
    if (path.empty()) return;
    std::string abs = path;
    std::error_code ec;
    const auto a = std::filesystem::absolute(path, ec);
    if (!ec) abs = a.lexically_normal().string();
    prefs.recentFiles.erase(std::remove(prefs.recentFiles.begin(), prefs.recentFiles.end(), abs),
                            prefs.recentFiles.end());
    prefs.recentFiles.insert(prefs.recentFiles.begin(), abs);
    if (prefs.recentFiles.size() > 10) prefs.recentFiles.resize(10);
}

} // namespace tg
