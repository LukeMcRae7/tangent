// Tangent - small shapes drawn in the viewport but measured on screen.
//
// An overlay is not geometry: a snap glyph, a drag handle and a reference tick
// all have to keep their size and their shape whatever the model is doing, or
// they collapse to a line the moment the plane they were built on turns
// edge-on. So they are drawn from the eye's own axes and sized in pixels.
#pragma once

#include "app/camera.h"
#include "render/renderer.h"

namespace tg::overlay {

// The eye's axes at a point, each scaled to one pixel.
struct ScreenFrame {
    Vec3 right{}, up{};
};

ScreenFrame frameAt(const Camera& camera, Vec3 at);

// `pts` are in pixels, relative to `at`, and the shape closes on itself.
void outline(Renderer& r, const Camera& c, Vec3 at, const ScreenFrame& f,
             const Vec2* pts, int n, Vec4 colour, Real widthPx);
void filled(Renderer& r, Vec3 at, const ScreenFrame& f,
            const Vec2* pts, int n, Vec4 colour);

void ring(Renderer& r, const Camera& c, Vec3 at, const ScreenFrame& f,
          Real radiusPx, Vec4 colour, Real widthPx);
void disc(Renderer& r, Vec3 at, const ScreenFrame& f, Real radiusPx, Vec4 colour);

// A square centred on `at`, filled and outlined: the shape a corner handle has
// in every tool that has one.
void square(Renderer& r, const Camera& c, Vec3 at, const ScreenFrame& f,
            Real halfPx, Vec4 fill, Vec4 edge, Real widthPx);

} // namespace tg::overlay
