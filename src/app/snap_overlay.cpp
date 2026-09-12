#include "app/snap_overlay.h"

#include "core/palette.h"

#include <cmath>

namespace tg {

namespace {

// The eye's own axes at a point, scaled to one pixel. Every glyph is drawn in
// these, which is what keeps it the same size and shape whatever the geometry
// underneath is doing.
struct ScreenFrame {
    Vec3 right{}, up{};
};

ScreenFrame frameAt(const Camera& camera, Vec3 at) {
    const Real px = static_cast<Real>(camera.pixelWorldSize(at));
    return {camera.right() * px, camera.up() * px};
}

void polygon(Renderer& r, const Camera& c, Vec3 at, const ScreenFrame& f,
             const Vec2* pts, int n, Vec4 col, Real widthPx) {
    for (int i = 0; i < n; ++i) {
        const Vec2 a = pts[i];
        const Vec2 b = pts[(i + 1) % n];
        r.addFrontLine(c, at + f.right * a.x + f.up * a.y,
                       at + f.right * b.x + f.up * b.y, col, widthPx);
    }
}

void ring(Renderer& r, const Camera& c, Vec3 at, const ScreenFrame& f,
          Real radiusPx, Vec4 col, Real widthPx) {
    constexpr int kSides = 20;
    Vec3 prev{};
    for (int i = 0; i <= kSides; ++i) {
        const Real a = kTwoPi * i / kSides;
        const Vec3 p = at + f.right * (radiusPx * std::cos(a)) +
                            f.up * (radiusPx * std::sin(a));
        if (i > 0) r.addFrontLine(c, prev, p, col, widthPx);
        prev = p;
    }
}

void dot(Renderer& r, Vec3 at, const ScreenFrame& f, Real radiusPx, Vec4 col) {
    constexpr int kSides = 12;
    for (int i = 0; i < kSides; ++i) {
        const Real a0 = kTwoPi * i / kSides, a1 = kTwoPi * (i + 1) / kSides;
        r.addFrontTriangle(at,
                           at + f.right * (radiusPx * std::cos(a0)) + f.up * (radiusPx * std::sin(a0)),
                           at + f.right * (radiusPx * std::cos(a1)) + f.up * (radiusPx * std::sin(a1)),
                           col);
    }
}

} // namespace

void drawSnapIndicator(Renderer& renderer, const Camera& camera,
                       const PlaneSnap& snap, const SnapOverlayStyle& style) {
    if (!snap.valid()) return;

    const Vec3 at = snap.point;
    const ScreenFrame f = frameAt(camera, at);
    const Real g = style.glyphPx;
    const Real w = style.strokePx;

    const Vec4 feature = toVec4(palette::kBrand, 1.0f);
    const Vec4 quiet{0.72f, 0.78f, 0.86f, 1.0f};
    const Vec4 dashCol = toVec4(palette::kBrand,
                                static_cast<float>(style.dashAlpha));

    // Where it was inferred from, first, so the glyph is laid over the lines
    // rather than under them.
    for (int i = 0; i < snap.refCount; ++i) {
        if (snap.refs[i].kind == SnapKind::None) continue;
        renderer.addFrontDashes(camera, snap.refs[i].from, at, dashCol,
                                style.dashWidthPx, style.dashPx, style.gapPx);

        // A dot on the reference itself: the line has two ends, and only one of
        // them is where the point is going. Deliberately not a glyph -- the
        // shapes mean "the point landed here", and the far end of a dotted line
        // is the one place that is certainly not true.
        const ScreenFrame rf = frameAt(camera, snap.refs[i].from);
        dot(renderer, snap.refs[i].from, rf, 2.6, dashCol);
    }

    switch (snap.kind) {
        case SnapKind::Vertex: {
            const Vec2 sq[4] = {{-g, -g}, {g, -g}, {g, g}, {-g, g}};
            polygon(renderer, camera, at, f, sq, 4, feature, w);
            break;
        }
        case SnapKind::CircleCentre:
            ring(renderer, camera, at, f, g, feature, w);
            break;
        case SnapKind::FaceCentre:
            ring(renderer, camera, at, f, g, feature, w);
            dot(renderer, at, f, g * 0.32, feature);
            break;
        case SnapKind::ArcQuadrant: {
            const Vec2 di[4] = {{0, -g * 1.15}, {g * 1.15, 0}, {0, g * 1.15}, {-g * 1.15, 0}};
            polygon(renderer, camera, at, f, di, 4, feature, w);
            break;
        }
        case SnapKind::EdgeMidpoint: {
            const Vec2 tri[3] = {{-g, -g * 0.72}, {g, -g * 0.72}, {0, g}};
            polygon(renderer, camera, at, f, tri, 3, feature, w);
            break;
        }
        case SnapKind::Alignment: {
            // A bar across the line being held, which is the one thing an
            // alignment has to say: movement along it is still free, movement
            // across it is not.
            Vec3 along = snap.refs[0].from - at;
            if (lengthSq(along) < 1e-12) break;
            along = normalize(along);
            Vec3 across = cross(along, normalize(camera.eye() - at));
            if (lengthSq(across) < 1e-12) break;
            across = normalize(across) * (static_cast<Real>(camera.pixelWorldSize(at)) * g * 1.3);
            renderer.addFrontLine(camera, at - across, at + across, feature, w + 0.6);
            break;
        }
        case SnapKind::Intersection: {
            const Real d = g * 0.85;
            renderer.addFrontLine(camera, at - f.right * d - f.up * d,
                                  at + f.right * d + f.up * d, feature, w);
            renderer.addFrontLine(camera, at - f.right * d + f.up * d,
                                  at + f.right * d - f.up * d, feature, w);
            break;
        }
        case SnapKind::GridPoint:
        case SnapKind::GridLine: {
            // Quieter than a feature, because it is a weaker claim: the grid is
            // what there is when there is nothing better.
            const Real d = g * 0.95;
            renderer.addFrontLine(camera, at - f.right * d, at + f.right * d, quiet, 2.0);
            renderer.addFrontLine(camera, at - f.up * d, at + f.up * d, quiet, 2.0);
            break;
        }
        case SnapKind::None:
            break;
    }
}

} // namespace tg
