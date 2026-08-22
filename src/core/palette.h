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
inline constexpr Rgb kBrand = hex(0xFF4B33);

// ---- Surfaces ---------------------------------------------------------
// Warm-leaning neutrals. A blue-cast grey next to a warm coral accent reads
// cold and corporate; nudging the greys warm keeps the interface friendly
// without tinting anything enough to disturb colour judgement.
inline constexpr Rgb kBackground  = hex(0x1A1A1D);
inline constexpr Rgb kPanel       = hex(0x1F1F22);
inline constexpr Rgb kRaised      = hex(0x27272B);
inline constexpr Rgb kHover       = hex(0x313136);
inline constexpr Rgb kActive      = hex(0x3B3B41);
inline constexpr Rgb kBorder      = hex(0x2E2E33);
inline constexpr Rgb kMenuBar     = hex(0x161619);

// ---- Text -------------------------------------------------------------
inline constexpr Rgb kText        = hex(0xC9C7C4);
inline constexpr Rgb kTextDim     = hex(0x7A7772);

// ---- Viewport ---------------------------------------------------------
inline constexpr Rgb kViewport    = hex(0x1C1C1F);
// Deliberately near-neutral: the shaded surface is what the user reads form
// from, so it must not carry the brand hue.
inline constexpr Rgb kSurface     = hex(0xB8B6B2);
inline constexpr Rgb kEdge        = hex(0x0E0E10);
// Muted on purpose. The brand is itself a warm red, so saturated axes would
// compete with the selection colour; ground reference must never outrank the
// thing the user has selected.
inline constexpr Rgb kGridAxisX   = hex(0x9E4238);
inline constexpr Rgb kGridAxisY   = hex(0x5E8A3C);

// Selection reuses the brand directly, so a selected outliner row and a
// selected object in the viewport are visibly the same colour.
inline constexpr Rgb kSelection   = kBrand;

} // namespace palette
} // namespace tg
