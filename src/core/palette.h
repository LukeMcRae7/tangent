// Tangent - the single source of truth for colour.
//
// Change kBrand and the whole application follows: UI accents, selection
// highlights, viewport outlines. Nothing else in the codebase should hardcode
// a brand colour.
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
inline constexpr Rgb kBrand = hex(0xF34425);

// ---- Surfaces ---------------------------------------------------------
// Neutral greys stepped from kBackground, which is the same tone the logo
// sits on. Every step keeps the same hue so the accent is the only colour in
// the interface with any saturation to it.
inline constexpr Rgb kBackground  = hex(0x1D1D1E);
inline constexpr Rgb kPanel       = hex(0x222223);
inline constexpr Rgb kRaised      = hex(0x2A2A2B);
inline constexpr Rgb kHover       = hex(0x343436);
inline constexpr Rgb kActive      = hex(0x3E3E40);
inline constexpr Rgb kBorder      = hex(0x313133);
inline constexpr Rgb kMenuBar     = hex(0x161617);

// ---- Text -------------------------------------------------------------
inline constexpr Rgb kText        = hex(0xF5F5F5);
inline constexpr Rgb kTextDim     = hex(0x86868A);

// ---- Viewport ---------------------------------------------------------
inline constexpr Rgb kViewport    = hex(0x1D1D1E);
// Deliberately neutral, and deliberately not the light anchor: the shaded
// surface needs room to brighten under the key light, so it starts mid-light
// and reaches near-white only where the light actually falls.
inline constexpr Rgb kSurface     = hex(0xBDBDBE);
inline constexpr Rgb kEdge        = hex(0x0F0F10);
// Muted on purpose. The brand is itself a warm red, so saturated axes would
// compete with the selection colour; ground reference must never outrank the
// thing the user has selected.
inline constexpr Rgb kGridAxisX   = hex(0x8E3A2A);
inline constexpr Rgb kGridAxisY   = hex(0x5E8A3C);

// Selection reuses the brand directly, so a selected outliner row and a
// selected object in the viewport are visibly the same colour.
inline constexpr Rgb kSelection   = kBrand;

// The one other saturated colour: a model that is ready to print. Kept well
// away from the brand hue so "solid" and "selected" never read as the same
// signal.
inline constexpr Rgb kValid       = hex(0x5FB350);

} // namespace palette
} // namespace tg
