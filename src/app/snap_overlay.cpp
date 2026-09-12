#include "app/snap_overlay.h"

#include "app/overlay_shapes.h"
#include "core/palette.h"

#include <cmath>

namespace tg {

namespace {

using overlay::ScreenFrame;
using overlay::frameAt;

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
        overlay::disc(renderer, snap.refs[i].from, rf, 2.6, dashCol);
    }

    switch (snap.kind) {
        case SnapKind::Vertex: {
            const Vec2 sq[4] = {{-g, -g}, {g, -g}, {g, g}, {-g, g}};
            overlay::outline(renderer, camera, at, f, sq, 4, feature, w);
            break;
        }
        case SnapKind::CircleCentre:
            overlay::ring(renderer, camera, at, f, g, feature, w);
            break;
        case SnapKind::FaceCentre:
            overlay::ring(renderer, camera, at, f, g, feature, w);
            overlay::disc(renderer, at, f, g * 0.32, feature);
            break;
        case SnapKind::ArcQuadrant: {
            const Vec2 di[4] = {{0, -g * 1.15}, {g * 1.15, 0}, {0, g * 1.15}, {-g * 1.15, 0}};
            overlay::outline(renderer, camera, at, f, di, 4, feature, w);
            break;
        }
        case SnapKind::EdgeMidpoint: {
            const Vec2 tri[3] = {{-g, -g * 0.72}, {g, -g * 0.72}, {0, g}};
            overlay::outline(renderer, camera, at, f, tri, 3, feature, w);
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
