#include "core/units.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace tg::units {

namespace {
Length g_current = Length::Millimetre;

Real perUnit(Length u) {
    switch (u) {
        case Length::Millimetre: return 1.0;
        case Length::Centimetre: return 10.0;
        case Length::Inch:       return 25.4;
    }
    return 1.0;
}

std::string format(Real v, int places) {
    // No "-0.00": a value that rounds to nothing is shown as nothing.
    const Real step = std::pow(10.0, -places);
    if (std::fabs(v) < step * 0.5) v = 0.0;
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.*f", places, static_cast<double>(v));
    return buf;
}
} // namespace

Length current() { return g_current; }
void setCurrent(Length u) { g_current = u; }

const char* suffixOf(Length u) {
    switch (u) {
        case Length::Millimetre: return "mm";
        case Length::Centimetre: return "cm";
        case Length::Inch:       return "in";
    }
    return "mm";
}
const char* suffix() { return suffixOf(g_current); }

const char* nameOf(Length u) {
    switch (u) {
        case Length::Millimetre: return "Millimetres";
        case Length::Centimetre: return "Centimetres";
        case Length::Inch:       return "Inches";
    }
    return "Millimetres";
}

Real toShown(Real mm) { return mm / perUnit(g_current); }
Real fromShown(Real shown) { return shown * perUnit(g_current); }

int decimals() { return g_current == Length::Millimetre ? 2 : 3; }

std::string number(Real mm, int places) { return format(toShown(mm), places < 0 ? decimals() : places); }

std::string length(Real mm, int places) { return number(mm, places) + " " + suffix(); }

std::string area(Real mm2) {
    const Real k = perUnit(g_current);
    return format(mm2 / (k * k), g_current == Length::Millimetre ? 1 : 3) + " " + suffix() + "\xC2\xB2";
}

std::string volume(Real mm3) {
    // Material is reckoned in cubic centimetres whether lengths are in
    // millimetres or centimetres -- it is the number on a slicer's estimate --
    // and in cubic inches for inches.
    if (g_current == Length::Inch) return format(mm3 / (25.4 * 25.4 * 25.4), 2) + " in\xC2\xB3";
    return format(mm3 / 1000.0, g_current == Length::Millimetre ? 1 : 3) + " cm\xC2\xB3";
}

bool parse(const std::string& text, Real& mm) {
    const char* s = text.c_str();
    while (std::isspace(static_cast<unsigned char>(*s))) ++s;
    char* end = nullptr;
    const double v = std::strtod(s, &end);
    if (end == s) return false;
    std::string rest(end);
    // Lower case, no spaces: "1.5 CM" reads as 1.5cm.
    std::string unit;
    for (char c : rest)
        if (!std::isspace(static_cast<unsigned char>(c))) unit += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    Real k = perUnit(g_current);
    if (unit.empty())                                      k = perUnit(g_current);
    else if (unit == "mm")                                 k = 1.0;
    else if (unit == "cm")                                 k = 10.0;
    else if (unit == "m")                                  k = 1000.0;
    else if (unit == "in" || unit == "\"" || unit == "inch" || unit == "inches") k = 25.4;
    else                                                   return false;
    mm = static_cast<Real>(v) * k;
    return true;
}

} // namespace tg::units
