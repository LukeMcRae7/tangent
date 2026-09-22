#include "app/overlay_shapes.h"

#include <cmath>

namespace tg::overlay {

ScreenFrame frameAt(const Camera& camera, Vec3 at) {
    const Real px = static_cast<Real>(camera.pixelWorldSize(at));
    return {camera.right() * px, camera.up() * px};
}

static Vec3 place(Vec3 at, const ScreenFrame& f, Vec2 p) {
    return at + f.right * p.x + f.up * p.y;
}

void outline(Renderer& r, const Camera& c, Vec3 at, const ScreenFrame& f,
             const Vec2* pts, int n, Vec4 colour, Real widthPx) {
    for (int i = 0; i < n; ++i)
        r.addFrontLine(c, place(at, f, pts[i]), place(at, f, pts[(i + 1) % n]),
                       colour, widthPx);
}

void filled(Renderer& r, Vec3 at, const ScreenFrame& f,
            const Vec2* pts, int n, Vec4 colour) {
    for (int i = 1; i + 1 < n; ++i)
        r.addFrontTriangle(place(at, f, pts[0]), place(at, f, pts[i]),
                           place(at, f, pts[i + 1]), colour);
}

void ring(Renderer& r, const Camera& c, Vec3 at, const ScreenFrame& f,
          Real radiusPx, Vec4 colour, Real widthPx) {
    constexpr int kSides = 20;
    Vec3 prev{};
    for (int i = 0; i <= kSides; ++i) {
        const Real a = kTwoPi * i / kSides;
        const Vec3 p = place(at, f, {radiusPx * std::cos(a), radiusPx * std::sin(a)});
        if (i > 0) r.addFrontLine(c, prev, p, colour, widthPx);
        prev = p;
    }
}

void disc(Renderer& r, Vec3 at, const ScreenFrame& f, Real radiusPx, Vec4 colour) {
    constexpr int kSides = 14;
    for (int i = 0; i < kSides; ++i) {
        const Real a0 = kTwoPi * i / kSides, a1 = kTwoPi * (i + 1) / kSides;
        r.addFrontTriangle(at,
                           place(at, f, {radiusPx * std::cos(a0), radiusPx * std::sin(a0)}),
                           place(at, f, {radiusPx * std::cos(a1), radiusPx * std::sin(a1)}),
                           colour);
    }
}

void square(Renderer& r, const Camera& c, Vec3 at, const ScreenFrame& f,
            Real halfPx, Vec4 fill, Vec4 edge, Real widthPx) {
    const Vec2 pts[4] = {{-halfPx, -halfPx}, {halfPx, -halfPx},
                         {halfPx, halfPx},   {-halfPx, halfPx}};
    filled(r, at, f, pts, 4, fill);
    outline(r, c, at, f, pts, 4, edge, widthPx);
}

} // namespace tg::overlay
