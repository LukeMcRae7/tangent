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
        case SnapKind::Alignment:    return "in line";
        case SnapKind::Intersection: return "crossing";
        case SnapKind::GridPoint:    return "grid";
        case SnapKind::GridLine:     return "grid line";
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

        // Never returned by findSnap -- it deals only in points that exist.
        // Ranked anyway so the comparison is total.
        case SnapKind::Intersection: return 5;
        case SnapKind::Alignment:    return 6;
        case SnapKind::GridPoint:    return 7;
        case SnapKind::GridLine:     return 8;
        case SnapKind::None:         return 9;
    }
    return 9;
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

// Every point on a body that the snapper recognises, in the body's own space.
//
// A template rather than a callback so it costs nothing to pass through: this
// runs over every vertex and edge of everything near the cursor, every frame.
// It exists so that the two ways of asking -- what is under the cursor, and
// what is lined up with it -- can never disagree about what a feature is.
//
// `offer` returns false to stop. A caller that has found all it needs must be
// able to say so: a body can carry a hundred thousand vertices, and walking
// them to discard every one is the difference between this being affordable and
// not.
template <class F>
void visitSnapPoints(const Body& body, const SnapConfig& config, F&& offer) {
    std::vector<VertexId> verts;
    std::vector<EdgeId> edges;
    std::vector<FaceId> faces;

    if (config.vertices) {
        body.allVertices(verts);
        std::vector<EdgeId> ve;
        for (VertexId v : verts) {
            // A closed edge -- a full circle -- begins and ends at one vertex,
            // and that vertex is where the parameterisation was cut open, not a
            // corner of the part. Offering it would put a "corner" on the rim of
            // every hole, at whatever angle the kernel happened to start
            // counting from. The same objection as the seam line the wireframe
            // no longer draws.
            body.vertexEdges(v, ve);
            bool seamOnly = false;
            for (EdgeId e : ve) {
                VertexId a2 = kInvalid, b2 = kInvalid;
                body.edgeEnds(e, a2, b2);
                if (a2 == v && b2 == v) { seamOnly = true; break; }
            }
            if (seamOnly) continue;
            if (!offer(body.vertexPosition(v), SnapKind::Vertex, Real(0))) return;
        }
    }

    if (config.circles || config.midpoints) {
        body.allEdges(edges);
        for (EdgeId e : edges) {
            VertexId ea = kInvalid, eb = kInvalid;
            body.edgeEnds(e, ea, eb);
            const bool closed = ea != kInvalid && ea == eb;

            Vec3 centre, axis;
            Real radius = 0.0;
            if (config.circles && body.edgeCircle(e, centre, axis, radius)) {
                if (!offer(centre, SnapKind::CircleCentre, radius)) return;

                // The four points around it. Where a tangent touches, and where
                // a diameter can be measured across.
                Vec3 u = normalize(body.edgeMidpoint(e) - centre);
                if (lengthSq(u) > 0.5) {
                    const Vec3 w = cross(axis, u);
                    for (int k = 0; k < 4; ++k) {
                        const Real a = static_cast<Real>(kHalfPi) * k;
                        if (!offer(centre + u * (radius * std::cos(a)) + w * (radius * std::sin(a)),
                                   SnapKind::ArcQuadrant, radius))
                            return;
                    }
                }
            }
            // A closed edge has no middle. What edgeMidpoint returns for one
            // is the point halfway along its parameter range, which is to say
            // wherever the kernel happened to start counting -- the same
            // objection as the seam vertex above, and just as misleading: it
            // put a "midpoint" on the rim of every hole, at an angle with no
            // meaning, and it sat a millimetre from nothing. The points a
            // circle really has are its centre and its quadrants, and both are
            // already offered.
            if (config.midpoints && !closed &&
                !offer(body.edgeMidpoint(e), SnapKind::EdgeMidpoint, Real(0)))
                return;
        }
    }

    if (config.faceCentres) {
        body.allFaces(faces);
        for (FaceId f : faces) {
            Vec3 point, axis;
            Real radius = 0.0;
            if (body.faceCylinder(f, point, axis, radius)) continue;  // its rims serve better
            if (!offer(body.faceCentroid(f), SnapKind::FaceCentre, Real(0))) return;
        }
    }
}

// The object's extent on screen, padded. False when none of it projects.
bool screenBounds(const SceneObject& o, const Camera& camera, Vec2& lo, Vec2& hi) {
    const AABB wb = o.worldBounds();
    lo = {1e30, 1e30};
    hi = {-1e30, -1e30};
    bool any = false;
    for (int corner = 0; corner < 8; ++corner) {
        const Vec3 p{corner & 1 ? wb.max.x : wb.min.x,
                     corner & 2 ? wb.max.y : wb.min.y,
                     corner & 4 ? wb.max.z : wb.min.z};
        Vec2 sp{};
        if (!camera.projectToPixel(p, sp)) continue;
        any = true;
        lo = {std::min(lo.x, sp.x), std::min(lo.y, sp.y)};
        hi = {std::max(hi.x, sp.x), std::max(hi.y, sp.y)};
    }
    return any;
}

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

    for (const auto& obj : scene.objects()) {
        const SceneObject* o = obj.get();
        if (!o->visible || o->body.empty()) continue;

        // Reject the whole object before looking at any of its elements. This
        // is what keeps the cost proportional to what the cursor is near
        // rather than to how much is in the scene.
        Vec2 lo{}, hi{};
        if (!screenBounds(*o, camera, lo, hi)) continue;
        const Real pad = config.radiusPx;
        if (mousePx.x < lo.x - pad || mousePx.x > hi.x + pad ||
            mousePx.y < lo.y - pad || mousePx.y > hi.y + pad)
            continue;

        const Mat4 model = o->modelMatrix();

        // One test for every candidate: near enough on screen, and not behind
        // the surface the cursor is over.
        visitSnapPoints(o->body, config, [&](Vec3 local, SnapKind kind, Real radius) {
            const Vec3 world = transformPoint(model, local);
            Vec2 sp{};
            if (!camera.projectToPixel(world, sp)) return true;
            const Real d = length(sp - mousePx);
            if (d > config.radiusPx) return true;
            const Real depth = dot(world - ray.origin, ray.dir);
            if (depth > surfaceDepth + config.depthToleranceMm) return true;
            if (!best.better(kind, d, depth)) return true;
            best.hit = {kind, world, o->id, d, radius};
            best.depth = depth;
            return true;                 // the best of all of them, not the first
        });
    }

    return best.hit;
}

size_t collectSnapPoints(const Scene& scene, const PlaneFrame& plane,
                         Vec2 aroundUV, Real tol, std::vector<SnapPoint>& out,
                         size_t limit, const SnapConfig& config) {
    out.clear();
    if (limit == 0 || tol <= 0.0) return 0;

    for (const auto& obj : scene.objects()) {
        const SceneObject* o = obj.get();
        if (!o->visible || o->body.empty()) continue;
        if (out.size() >= limit) break;

        // The band test, done against the object's corners in plane
        // coordinates. An object entirely off to one side and entirely above
        // cannot line up with the cursor either way, and is dropped here rather
        // than one element at a time.
        const AABB wb = o->worldBounds();
        Real uLo = 1e30, uHi = -1e30, vLo = 1e30, vHi = -1e30;
        for (int corner = 0; corner < 8; ++corner) {
            const Vec2 c = plane.toUV({corner & 1 ? wb.max.x : wb.min.x,
                                       corner & 2 ? wb.max.y : wb.min.y,
                                       corner & 4 ? wb.max.z : wb.min.z});
            uLo = std::min(uLo, c.x); uHi = std::max(uHi, c.x);
            vLo = std::min(vLo, c.y); vHi = std::max(vHi, c.y);
        }
        const bool inU = aroundUV.x > uLo - tol && aroundUV.x < uHi + tol;
        const bool inV = aroundUV.y > vLo - tol && aroundUV.y < vHi + tol;
        if (!inU && !inV) continue;

        const Mat4 model = o->modelMatrix();
        // On the plane means too close to the plane to tell on screen. Half the
        // band tolerance is a few pixels, which is the same rule the sketch
        // tools have always used for "is this point on my plane".
        const Real planeTol = std::max(tol * 0.5, Real(1e-4));

        visitSnapPoints(o->body, config, [&](Vec3 local, SnapKind kind, Real radius) {
            if (kind == SnapKind::ArcQuadrant) return true;

            const Vec3 world = transformPoint(model, local);
            if (plane.distanceTo(world) > planeTol) return true;

            const Vec2 uv = plane.toUV(world);
            if (std::fabs(uv.x - aroundUV.x) > tol && std::fabs(uv.y - aroundUV.y) > tol) return true;

            out.push_back({world, uv, kind, o->id, radius});
            return out.size() < limit;
        });
    }
    return out.size();
}

} // namespace tg
