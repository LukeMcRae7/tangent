#include "app/snap.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tg {

const char* snapKindName(SnapKind k) {
    switch (k) {
        case SnapKind::None:         return "";
        case SnapKind::Vertex:       return "corner";
        case SnapKind::CircleCentre: return "centre";
        case SnapKind::ArcQuadrant:  return "quadrant";
        case SnapKind::EdgeMidpoint: return "midpoint";
        case SnapKind::FaceCentre:   return "face centre";
    }
    return "";
}

namespace {

// Lower is better. A centre being preferred to a midpoint is the whole reason
// this is not just "nearest wins": on a rounded corner the two are millimetres
// apart, and the centre is what someone is aiming at.
int rank(SnapKind k) {
    switch (k) {
        case SnapKind::Vertex:       return 0;
        case SnapKind::CircleCentre: return 1;
        case SnapKind::ArcQuadrant:  return 2;
        case SnapKind::EdgeMidpoint: return 3;
        case SnapKind::FaceCentre:   return 4;
        case SnapKind::None:         return 5;
    }
    return 5;
}

struct Best {
    SnapHit hit;
    Real depth = 0.0;

    // Kind first, then distance on screen, then which is nearer the eye.
    //
    // The last one matters more than it sounds: looking down at a through hole,
    // both rims are the same point on screen, and without it the answer is
    // whichever the kernel happened to enumerate first -- which is the far one
    // as often as not, and a sketch point ten millimetres below the face being
    // drawn on. Distances within a pixel and a half of each other are treated
    // as the same distance, so a candidate does not win on jitter.
    bool better(SnapKind kind, Real distPx, Real atDepth) const {
        if (!hit.valid()) return true;
        const int a = rank(kind), b = rank(hit.kind);
        if (a != b) return a < b;
        if (std::fabs(distPx - hit.screenDistancePx) > 1.5) return distPx < hit.screenDistancePx;
        return atDepth < depth;
    }
};

} // namespace

SnapHit findSnap(const Scene& scene, const Camera& camera, Vec2 mousePx,
                 const SnapConfig& config) {
    Best best;

    // How deep the thing under the cursor is, so a snap behind it can be
    // rejected. A miss leaves this at infinity and everything is eligible,
    // which is right: there is nothing in front to be behind.
    const Ray ray = camera.rayThroughPixel(mousePx.x, mousePx.y);
    const RayHit under = scene.raycast(ray);
    const Real surfaceDepth = under.hit() ? static_cast<Real>(under.t)
                                          : std::numeric_limits<Real>::infinity();

    std::vector<VertexId> verts;
    std::vector<EdgeId> edges;
    std::vector<FaceId> faces;

    for (const auto& obj : scene.objects()) {
        const SceneObject* o = obj.get();
        if (!o->visible || o->body.empty()) continue;

        // Reject the whole object before looking at any of its elements. This
        // is what keeps the cost proportional to what the cursor is near
        // rather than to how much is in the scene.
        const AABB wb = o->worldBounds();
        Vec2 lo{1e30, 1e30}, hi{-1e30, -1e30};
        bool anyOnScreen = false;
        for (int corner = 0; corner < 8; ++corner) {
            const Vec3 p{corner & 1 ? wb.max.x : wb.min.x,
                         corner & 2 ? wb.max.y : wb.min.y,
                         corner & 4 ? wb.max.z : wb.min.z};
            Vec2 sp{};
            if (!camera.projectToPixel(p, sp)) continue;
            anyOnScreen = true;
            lo = {std::min(lo.x, sp.x), std::min(lo.y, sp.y)};
            hi = {std::max(hi.x, sp.x), std::max(hi.y, sp.y)};
        }
        if (!anyOnScreen) continue;
        const Real pad = config.radiusPx;
        if (mousePx.x < lo.x - pad || mousePx.x > hi.x + pad ||
            mousePx.y < lo.y - pad || mousePx.y > hi.y + pad)
            continue;

        const Mat4 model = o->modelMatrix();
        const Body& body = o->body;

        // One test for every candidate: near enough on screen, and not behind
        // the surface the cursor is over.
        auto offer = [&](Vec3 local, SnapKind kind, Real radius) {
            const Vec3 world = transformPoint(model, local);
            Vec2 sp{};
            if (!camera.projectToPixel(world, sp)) return;
            const Real d = length(sp - mousePx);
            if (d > config.radiusPx) return;
            const Real depth = dot(world - ray.origin, ray.dir);
            if (depth > surfaceDepth + config.depthToleranceMm) return;
            if (!best.better(kind, d, depth)) return;
            best.hit = {kind, world, o->id, d, radius};
            best.depth = depth;
        };

        if (config.vertices) {
            body.allVertices(verts);
            std::vector<EdgeId> ve;
            for (VertexId v : verts) {
                // A closed edge -- a full circle -- begins and ends at one
                // vertex, and that vertex is where the parameterisation was cut
                // open, not a corner of the part. Offering it would put a
                // "corner" on the rim of every hole, at whatever angle the
                // kernel happened to start counting from. The same objection as
                // the seam line the wireframe no longer draws.
                body.vertexEdges(v, ve);
                bool seamOnly = false;
                for (EdgeId e : ve) {
                    VertexId a2 = kInvalid, b2 = kInvalid;
                    body.edgeEnds(e, a2, b2);
                    if (a2 == v && b2 == v) { seamOnly = true; break; }
                }
                if (seamOnly) continue;
                offer(body.vertexPosition(v), SnapKind::Vertex, 0.0);
            }
        }

        if (config.circles || config.midpoints) {
            body.allEdges(edges);
            for (EdgeId e : edges) {
                Vec3 centre, axis;
                Real radius = 0.0;
                if (config.circles && body.edgeCircle(e, centre, axis, radius)) {
                    offer(centre, SnapKind::CircleCentre, radius);

                    // The four points around it. Where a tangent touches, and
                    // where a diameter can be measured across.
                    Vec3 u = normalize(body.edgeMidpoint(e) - centre);
                    if (lengthSq(u) > 0.5) {
                        const Vec3 v = cross(axis, u);
                        for (int k = 0; k < 4; ++k) {
                            const Real a = static_cast<Real>(kHalfPi) * k;
                            offer(centre + u * (radius * std::cos(a)) + v * (radius * std::sin(a)),
                                  SnapKind::ArcQuadrant, radius);
                        }
                    }
                }
                if (config.midpoints) offer(body.edgeMidpoint(e), SnapKind::EdgeMidpoint, 0.0);
            }
        }

        if (config.faceCentres) {
            body.allFaces(faces);
            for (FaceId f : faces) {
                Vec3 point, axis;
                Real radius = 0.0;
                if (body.faceCylinder(f, point, axis, radius)) continue;  // its rims serve better
                offer(body.faceCentroid(f), SnapKind::FaceCentre, 0.0);
            }
        }
    }

    return best.hit;
}

} // namespace tg
