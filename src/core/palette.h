// Tangent - the single source of truth for colour.
//
// Change kBrand and the whole application follows: UI accents, selection
// highlights, viewport outlines. Nothing else in the codebase should hardcode
// a brand colour.
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
inline constexpr Rgb kBrand      = hex(0xF34425);
inline constexpr Rgb kBrandHover = hex(0xF75A3E);
inline constexpr Rgb kBrandDown  = hex(0xD8391D);

// ---- Surfaces ---------------------------------------------------------
// From the ground up: the window behind everything, the bars and panels on
// it, then the fields and buttons that sit on those.
inline constexpr Rgb kBackground  = hex(0x1A1A1B);   // the window itself
inline constexpr Rgb kTopBar      = hex(0x161617);   // the bar along the top
inline constexpr Rgb kPanel       = hex(0x1E1E1F);   // outliner, inspector
inline constexpr Rgb kCommand     = hex(0x151516);   // the floating operation panel
inline constexpr Rgb kField       = hex(0x242426);   // a value you can edit
inline constexpr Rgb kRaised      = hex(0x2A2A2C);   // a button at rest
inline constexpr Rgb kHover       = hex(0x333335);
inline constexpr Rgb kActive      = hex(0x3C3C3F);
inline constexpr Rgb kBorder      = hex(0x2B2B2D);
inline constexpr Rgb kBorderStrong = hex(0x38383B);
inline constexpr Rgb kMenuBar     = kTopBar;

// ---- Text -------------------------------------------------------------
inline constexpr Rgb kText        = hex(0xF2F1EF);
inline constexpr Rgb kTextDim     = hex(0x8F8E8C);
inline constexpr Rgb kTextFaint   = hex(0x5C5B5A);

// ---- Viewport ---------------------------------------------------------
inline constexpr Rgb kViewport    = hex(0x1E1E1F);
// Deliberately neutral, and deliberately not the light anchor: the shaded
// surface needs room to brighten under the key light, so it starts mid-light
// and reaches near-white only where the light actually falls.
inline constexpr Rgb kSurface     = hex(0xBDBDBE);
inline constexpr Rgb kEdge        = hex(0x0F0F10);
// Muted on purpose. The brand is itself a warm red, so saturated axes would
// compete with the selection colour; ground reference must never outrank the
// thing the user has selected.
inline constexpr Rgb kGridAxisX   = hex(0x9C3C2D);
inline constexpr Rgb kGridAxisY   = hex(0x4F8C3A);

// The axes as letters: X, Y and Z in the inspector and on the gizmos. Brighter
// than the grid lines because they are read rather than seen.
inline constexpr Rgb kAxisX       = hex(0xE5484D);
inline constexpr Rgb kAxisY       = hex(0x7AC142);
inline constexpr Rgb kAxisZ       = hex(0x4F8EF7);

// Selection reuses the brand directly, so a selected outliner row and a
// selected object in the viewport are visibly the same colour.
inline constexpr Rgb kSelection   = kBrand;

// The one other saturated colour: a model that is ready to print. Kept well
// away from the brand hue so "solid" and "selected" never read as the same
// signal.
inline constexpr Rgb kValid       = hex(0x5FB350);

// Something to look at before printing, and the cool blue of geometry that is
// still free to move in a sketch.
inline constexpr Rgb kWarn        = hex(0xE0A13A);
inline constexpr Rgb kInfo        = hex(0x4F8EF7);

} // namespace palette
} // namespace tg
