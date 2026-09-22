// Tangent - a small rasteriser, for baking icons out of real geometry.
//
// Not the renderer. The renderer needs a GL context, a window and a driver; a
// build step should need a compiler. This draws flat-shaded triangles with a
// depth buffer and then the model's own edges over the top, which is all an
// icon of a solid is, and it produces the same bytes on every machine.
#pragma once

#include "core/math.h"
#include "geom/body.h"

#include <cstdint>
#include <string>
#include <vector>

namespace tg::icons {

struct Rgba {
    uint8_t r = 0, g = 0, b = 0, a = 0;
};

// An image with premultiplied-by-nothing straight alpha, top row first.
struct Image {
    int width = 0, height = 0;
    std::vector<Rgba> pixels;

    Rgba& at(int x, int y) { return pixels[static_cast<size_t>(y) * width + x]; }
    const Rgba& at(int x, int y) const { return pixels[static_cast<size_t>(y) * width + x]; }
};

struct Style {
    // The body, and the part of it the operation is about. An icon says what
    // the operation does by colouring what it touches.
    Rgba solid{212, 214, 216, 255};
    Rgba accent{243, 68, 37, 255};
    Rgba outline{29, 29, 30, 255};

    // Where the light comes from, in view space: up, right and towards the eye.
    Vec3 light{0.35, 0.45, 0.82};
    Real ambient = 0.42;

    Real outlineWidthPx = 2.2;   // at the supersampled size
    Real margin = 0.10;          // fraction of the frame left empty
};

// Everything the rasteriser needs about one icon's geometry: the body, and
// which of its faces are the point of it.
struct Subject {
    const Body* body = nullptr;
    std::vector<FaceId> accentFaces;

    // Looking from the front, above and to the right -- the angle every CAD
    // icon set uses, because it shows three faces of a box at once.
    Vec3 eye{1.0, -1.25, 0.85};
};

// Renders at `size` * `supersample` and boxes it down, which is what keeps a
// 32-pixel icon from being a mess of stair steps.
Image render(const Subject& subject, int size, const Style& style = {},
             int supersample = 4);

// A PNG, written with stored (uncompressed) deflate blocks so that nothing
// outside the standard library is needed to produce one.
bool writePng(const Image& image, const std::string& path);

} // namespace tg::icons
