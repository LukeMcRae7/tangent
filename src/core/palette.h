// Tangent - the single source of truth for colour.
//
// Change kBrand and the whole application follows: UI accents, selection
// highlights, viewport outlines. Nothing else in the codebase should hardcode
// a colour at all: the values below are the dark theme's, and applyLight()
// swaps every one of them for the light theme's, so a colour read from here
// is right in either.
//
// The greys are stepped from one near-black ground so the interface reads as
// a single dark surface with the model lit in the middle of it. Only three
// things carry saturation: the brand, the axes, and the green that says a body
// is sound.
#pragma once

#include "core/math.h"

#include <cstdint>

namespace tg {

struct Rgb {
    float r = 0, g = 0, b = 0;
};

constexpr Rgb hex(uint32_t v) {
    return Rgb{static_cast<float>((v >> 16) & 0xFF) / 255.0f,
               static_cast<float>((v >>  8) & 0xFF) / 255.0f,
               static_cast<float>((v      ) & 0xFF) / 255.0f};
}

inline Vec3 toVec3(Rgb c) { return {c.r, c.g, c.b}; }
inline Vec4 toVec4(Rgb c, float a) { return {c.r, c.g, c.b, a}; }

// Blend toward another colour; used for hover and pressed states so they stay
// in step with the brand automatically.
constexpr Rgb mix(Rgb a, Rgb b, float t) {
    return Rgb{a.r + (b.r - a.r) * t,
               a.g + (b.g - a.g) * t,
               a.b + (b.b - a.b) * t};
}

namespace palette {

// ---- The one knob -----------------------------------------------------
inline Rgb kBrand      = hex(0xF34425);
inline Rgb kBrandHover = hex(0xF75A3E);
inline Rgb kBrandDown  = hex(0xD8391D);

// ---- Surfaces ---------------------------------------------------------
// From the ground up: the window behind everything, the bars and panels on
// it, then the fields and buttons that sit on those.
inline Rgb kBackground  = hex(0x1A1A1B);   // the window itself
inline Rgb kTopBar      = hex(0x161617);   // the bar along the top
inline Rgb kPanel       = hex(0x1E1E1F);   // outliner, inspector
inline Rgb kCommand     = hex(0x151516);   // the floating operation panel
inline Rgb kField       = hex(0x242426);   // a value you can edit
inline Rgb kRaised      = hex(0x2A2A2C);   // a button at rest
inline Rgb kHover       = hex(0x333335);
inline Rgb kActive      = hex(0x3C3C3F);
inline Rgb kBorder      = hex(0x2B2B2D);
inline Rgb kBorderStrong = hex(0x38383B);
inline Rgb kMenuBar     = kTopBar;

// ---- Text -------------------------------------------------------------
inline Rgb kText        = hex(0xF2F1EF);
inline Rgb kTextDim     = hex(0x8F8E8C);
inline Rgb kTextFaint   = hex(0x5C5B5A);

// ---- Viewport ---------------------------------------------------------
inline Rgb kViewport    = hex(0x1E1E1F);
// Deliberately neutral, and deliberately not the light anchor: the shaded
// surface needs room to brighten under the key light, so it starts mid-light
// and reaches near-white only where the light actually falls.
inline Rgb kSurface     = hex(0xBDBDBE);
inline Rgb kEdge        = hex(0x0F0F10);
// Muted on purpose. The brand is itself a warm red, so saturated axes would
// compete with the selection colour; ground reference must never outrank the
// thing the user has selected.
inline Rgb kGridLine    = hex(0x4D4D52);   // the ground grid's own lines
inline Rgb kGridAxisX   = hex(0x9C3C2D);
inline Rgb kGridAxisY   = hex(0x4F8C3A);

// The axes as letters: X, Y and Z in the inspector and on the gizmos. Brighter
// than the grid lines because they are read rather than seen.
inline Rgb kAxisX       = hex(0xE5484D);
inline Rgb kAxisY       = hex(0x7AC142);
inline Rgb kAxisZ       = hex(0x4F8EF7);

// Selection reuses the brand directly, so a selected outliner row and a
// selected object in the viewport are visibly the same colour.
inline Rgb kSelection   = kBrand;

// The one other saturated colour: a model that is ready to print. Kept well
// away from the brand hue so "solid" and "selected" never read as the same
// signal.
inline Rgb kValid       = hex(0x5FB350);

// Something to look at before printing, and the cool blue of geometry that is
// still free to move in a sketch.
inline Rgb kWarn        = hex(0xE0A13A);
inline Rgb kInfo        = hex(0x4F8EF7);

// ---- Themes -----------------------------------------------------------
inline bool g_light = false;
inline bool isLight() { return g_light; }

inline void applyDark() {
    g_light = false;
    kBackground = hex(0x1A1A1B); kTopBar = hex(0x161617); kPanel = hex(0x1E1E1F); kCommand = hex(0x151516);
    kField = hex(0x242426); kRaised = hex(0x2A2A2C); kHover = hex(0x333335); kActive = hex(0x3C3C3F);
    kBorder = hex(0x2B2B2D); kBorderStrong = hex(0x38383B); kMenuBar = kTopBar;
    kText = hex(0xF2F1EF); kTextDim = hex(0x8F8E8C); kTextFaint = hex(0x5C5B5A);
    kViewport = hex(0x1E1E1F); kSurface = hex(0xBDBDBE); kEdge = hex(0x0F0F10); kGridLine = hex(0x4D4D52);
    kGridAxisX = hex(0x9C3C2D); kGridAxisY = hex(0x4F8C3A);
    kValid = hex(0x5FB350); kWarn = hex(0xE0A13A);
}

// Warm paper greys, stepped the other way: the panels lightest, the fields
// and buttons a shade down into them. The model is a touch darker than on the
// dark ground so its lit faces still stand off the background.
inline void applyLight() {
    g_light = true;
    kBackground = hex(0xECECE9); kTopBar = hex(0xF7F7F5); kPanel = hex(0xF4F4F1); kCommand = hex(0xFCFCFB);
    kField = hex(0xE7E7E3); kRaised = hex(0xE3E3DF); kHover = hex(0xD9D9D5); kActive = hex(0xCDCDC8);
    kBorder = hex(0xDCDCD7); kBorderStrong = hex(0xC8C8C3); kMenuBar = kTopBar;
    kText = hex(0x1C1C1E); kTextDim = hex(0x6A6A67); kTextFaint = hex(0x9E9E9A);
    kViewport = hex(0xE8E8E5); kSurface = hex(0xB4B4B7); kEdge = hex(0x2A2A2D); kGridLine = hex(0xBDBDB8);
    kGridAxisX = hex(0xD0705F); kGridAxisY = hex(0x74AE62);
    kValid = hex(0x3E9A33); kWarn = hex(0xC7861C);
}

} // namespace palette
} // namespace tg
