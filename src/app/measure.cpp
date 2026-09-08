#include "app/measure.h"
#include "core/palette.h"

#include <algorithm>
#include <cstdio>

namespace tg {
namespace {

// An entity reduced to world-space geometry: one point for a vertex, two for
// an edge, the whole loop for a face.
struct Entity {
    ElementKind kind = ElementKind::None;
    std::vector<Vec3> pts;
    Vec3 normal;
    bool ok = false;

    // What the thing is, where the body can say. A mesh always answers "a
    // straight line" and "a polygon", so these stay false and every reading
    // below falls back to what it always was.
    bool round = false;        // a circular edge, or a cylindrical face
    Real radius = 0.0;
    Vec3 centre;
    Real curveLength = 0.0;    // along the curve, not across the chord
};

Entity resolve(const Scene& scene, const ElementRef& ref) {
    Entity e;
    const SceneObject* obj = scene.find(ref.object);
    if (!obj) return e;

    const Mat4 model = obj->modelMatrix();
    const Body& body = obj->body;

    switch (ref.kind) {
    case ElementKind::Vertex:
        if (!body.hasVertex(ref.index)) return e;
        e.pts.push_back(transformPoint(model, body.vertexPosition(ref.index)));
        break;
    case ElementKind::Edge: {
        if (!body.hasEdge(ref.index)) return e;
        Vec3 a, b;
        body.edgePositions(ref.index, a, b);
        e.pts.push_back(transformPoint(model, a));
        e.pts.push_back(transformPoint(model, b));

        Vec3 centre, axis;
        Real radius = 0.0;
        if (body.edgeCircle(ref.index, centre, axis, radius)) {
            e.round = true;
            e.radius = radius;
            e.centre = transformPoint(model, centre);
            // A full circle's two ends are the same point, so the line the
            // viewport draws has to be the diameter instead of a zero-length
            // nothing at the seam.
            if (length(e.pts[1] - e.pts[0]) < 1e-9) {
                const Vec3 out = normalize(transformVector(model, body.edgeMidpoint(ref.index) - centre));
                e.pts[0] = e.centre - out * radius;
                e.pts[1] = e.centre + out * radius;
            }
        }
        e.curveLength = body.edgeLength(ref.index);
        break;
    }
    case ElementKind::Face: {
        if (!body.hasFace(ref.index)) return e;

        Vec3 point, axis;
        Real radius = 0.0;
        if (body.faceCylinder(ref.index, point, axis, radius)) {
            // A bore's wall. Its diameter is what the part was drilled at --
            // not something to be recovered from the vertices around it.
            e.round = true;
            e.radius = radius;
            e.centre = transformPoint(model, point);
        }

        std::vector<VertexId> verts;
        body.faceVertices(ref.index, verts);
        for (VertexId v : verts)
            e.pts.push_back(transformPoint(model, body.vertexPosition(v)));
        e.normal = normalize(transformVector(normalMatrix(model), body.faceNormal(ref.index)));
        break;
    }
    case ElementKind::None:
        return e;
    }
    e.kind = ref.kind;
    e.ok = !e.pts.empty();
    return e;
}

Vec3 closestOnSegment(Vec3 p, Vec3 a, Vec3 b) {
    const Vec3 ab = b - a;
    const Real len2 = lengthSq(ab);
    if (len2 < 1e-18) return a;
    return a + ab * clampf(dot(p - a, ab) / len2, 0.0, 1.0);
}

// Closest points on two segments. The clamped-parameter form: solve the
// unconstrained system, clamp, then re-solve the other parameter so the result
// stays on both segments.
void closestSegments(Vec3 p1, Vec3 q1, Vec3 p2, Vec3 q2, Vec3& c1, Vec3& c2) {
    const Vec3 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    const Real a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);

    if (a < 1e-18 && e < 1e-18) { c1 = p1; c2 = p2; return; }
    if (a < 1e-18) { c1 = p1; c2 = closestOnSegment(p1, p2, q2); return; }
    if (e < 1e-18) { c2 = p2; c1 = closestOnSegment(p2, p1, q1); return; }

    const Real b = dot(d1, d2), c = dot(d1, r);
    const Real denom = a * e - b * b;
    Real s = denom > 1e-18 ? clampf((b * f - c * e) / denom, 0.0, 1.0) : 0.0;
    Real t = (b * s + f) / e;

    if (t < 0.0)      { t = 0.0; s = clampf(-c / a, 0.0, 1.0); }
    else if (t > 1.0) { t = 1.0; s = clampf((b - c) / a, 0.0, 1.0); }

    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
}

bool insidePolygon(Vec3 p, const std::vector<Vec3>& poly, Vec3 n) {
    // Winding-consistent side test: p is inside if it stays on the same side of
    // every edge, using the face normal to define the sense.
    for (size_t i = 0; i < poly.size(); ++i) {
        const Vec3& a = poly[i];
        const Vec3& b = poly[(i + 1) % poly.size()];
        if (dot(cross(b - a, p - a), n) < -1e-9) return false;
    }
    return true;
}

Vec3 closestOnPolygon(Vec3 p, const std::vector<Vec3>& poly, Vec3 n) {
    // Project onto the plane; if the projection lands inside, that is the
    // answer, otherwise the nearest point lies on the boundary.
    const Vec3 projected = p - n * dot(p - poly[0], n);
    if (insidePolygon(projected, poly, n)) return projected;

    Vec3 best = poly[0];
    Real bestD = std::numeric_limits<Real>::max();
    for (size_t i = 0; i < poly.size(); ++i) {
        const Vec3 c = closestOnSegment(p, poly[i], poly[(i + 1) % poly.size()]);
        const Real d = lengthSq(c - p);
        if (d < bestD) { bestD = d; best = c; }
    }
    return best;
}

// Minimum distance between two resolved entities, and where it is attained.
void minimumDistance(const Entity& A, const Entity& B, Vec3& from, Vec3& to) {
    Real best = std::numeric_limits<Real>::max();
    auto consider = [&](Vec3 a, Vec3 b) {
        const Real d = lengthSq(b - a);
        if (d < best) { best = d; from = a; to = b; }
    };

    const bool aPoly = A.kind == ElementKind::Face;
    const bool bPoly = B.kind == ElementKind::Face;

    // Every vertex of one against the whole of the other. For two parallel
    // faces this is what finds the interior-to-interior distance, which is the
    // wall-thickness case.
    for (const Vec3& p : A.pts) {
        if (bPoly)                          consider(p, closestOnPolygon(p, B.pts, B.normal));
        else if (B.pts.size() == 2)         consider(p, closestOnSegment(p, B.pts[0], B.pts[1]));
        else                                consider(p, B.pts[0]);
    }
    for (const Vec3& p : B.pts) {
        if (aPoly)                          consider(closestOnPolygon(p, A.pts, A.normal), p);
        else if (A.pts.size() == 2)         consider(closestOnSegment(p, A.pts[0], A.pts[1]), p);
        else                                consider(A.pts[0], p);
    }

    // Edge against edge, which catches the crossing case that vertex tests
    // miss (two skew edges nearest somewhere along their spans).
    auto edgesOf = [](const Entity& e) {
        std::vector<std::pair<Vec3, Vec3>> out;
        if (e.pts.size() == 2) out.emplace_back(e.pts[0], e.pts[1]);
        else if (e.pts.size() > 2)
            for (size_t i = 0; i < e.pts.size(); ++i)
                out.emplace_back(e.pts[i], e.pts[(i + 1) % e.pts.size()]);
        return out;
    };
    for (const auto& ea : edgesOf(A))
        for (const auto& eb : edgesOf(B)) {
            Vec3 c1, c2;
            closestSegments(ea.first, ea.second, eb.first, eb.second, c1, c2);
            consider(c1, c2);
        }
}

// Direction an entity defines, if it has one: an edge's axis, a face's normal.
bool directionOf(const Entity& e, Vec3& dir) {
    if (e.kind == ElementKind::Edge && e.pts.size() == 2) {
        const Vec3 d = e.pts[1] - e.pts[0];
        if (lengthSq(d) < 1e-18) return false;
        dir = normalize(d);
        return true;
    }
    if (e.kind == ElementKind::Face) { dir = e.normal; return true; }
    return false;
}

} // namespace

void MeasureTool::pick(const ElementRef& e) {
    if (!e.valid()) { picks_.clear(); return; }

    auto it = std::find(picks_.begin(), picks_.end(), e);
    if (it != picks_.end()) { picks_.erase(it); return; }

    // A third pick begins a new measurement from that entity.
    if (picks_.size() >= 2) picks_.clear();
    picks_.push_back(e);
}

MeasureResult MeasureTool::compute(const Scene& scene) const {
    MeasureResult r;
    if (picks_.empty()) return r;

    const Entity A = resolve(scene, picks_[0]);
    if (!A.ok) return r;

    char buf[192];

    // ---- One entity: its own dimensions ----------------------------------
    if (picks_.size() == 1) {
        switch (A.kind) {
        case ElementKind::Vertex:
            r.from = r.to = A.pts[0];
            std::snprintf(buf, sizeof(buf), "%.3f, %.3f, %.3f mm",
                          A.pts[0].x, A.pts[0].y, A.pts[0].z);
            break;
        case ElementKind::Edge:
            r.from = A.pts[0];
            r.to = A.pts[1];
            r.hasLength = true;
            // Along the curve. For a straight edge this is the same number it
            // always was; for an arc it is the one a person means.
            r.length = A.curveLength > 0.0 ? A.curveLength : length(A.pts[1] - A.pts[0]);
            r.delta = A.pts[1] - A.pts[0];
            r.distance = r.length;
            if (A.round) {
                r.hasDiameter = true;
                r.diameter = A.radius * 2.0;
                r.centre = A.centre;
                std::snprintf(buf, sizeof(buf), "\u2300 %.3f mm  (%.3f mm around)",
                              r.diameter, r.length);
            } else {
                std::snprintf(buf, sizeof(buf), "%.3f mm", r.length);
            }
            break;
        case ElementKind::Face: {
            Vec3 c{};
            for (const Vec3& p : A.pts) c += p;
            c = c / static_cast<Real>(A.pts.size());
            r.from = r.to = c;

            // Fan from the first corner: exact for a planar polygon.
            Real area = 0.0, perim = 0.0;
            for (size_t i = 1; i + 1 < A.pts.size(); ++i)
                area += length(cross(A.pts[i] - A.pts[0], A.pts[i + 1] - A.pts[0])) * 0.5;
            for (size_t i = 0; i < A.pts.size(); ++i)
                perim += length(A.pts[(i + 1) % A.pts.size()] - A.pts[i]);
            r.hasArea = true;
            r.area = area;
            r.perimeter = perim;
            if (A.round) {
                r.hasDiameter = true;
                r.diameter = A.radius * 2.0;
                r.centre = A.centre;
                r.from = r.to = A.centre;
                std::snprintf(buf, sizeof(buf), "\u2300 %.3f mm", r.diameter);
            } else {
                std::snprintf(buf, sizeof(buf), "%.3f mm2", area);
            }
            break;
        }
        case ElementKind::None:
            return r;
        }
        r.summary = buf;
        r.valid = true;
        return r;
    }

    // ---- Two entities: distance, and angle where both have a direction ----
    const Entity B = resolve(scene, picks_[1]);
    if (!B.ok) return r;

    minimumDistance(A, B, r.from, r.to);
    r.delta = r.to - r.from;
    r.distance = length(r.delta);

    Vec3 da, db;
    if (directionOf(A, da) && directionOf(B, db)) {
        // Reported as the acute angle: which way a normal happens to point
        // should not turn a 5-degree reading into 175.
        const Real c = clampf(std::fabs(dot(da, db)), 0.0, 1.0);
        r.hasAngle = true;
        r.angleDeg = degrees(std::acos(c));
    }

    if (r.hasAngle)
        std::snprintf(buf, sizeof(buf), "%.3f mm   %.2f deg", r.distance, r.angleDeg);
    else
        std::snprintf(buf, sizeof(buf), "%.3f mm", r.distance);
    r.summary = buf;
    r.valid = true;
    return r;
}

void MeasureTool::drawOverlay(Renderer& renderer, const Camera& camera,
                              const MeasureResult& result) const {
    if (!result.valid) return;

    const Vec4 accent = toVec4(palette::kBrand, 1.0);
    const Real tick = camera.pixelWorldSize(result.from) * 5.0;

    // A single point still gets a marker, so picking one vertex shows where.
    if (lengthSq(result.to - result.from) < 1e-18) {
        const Vec3 p = result.from;
        renderer.addLine(p - Vec3{tick, 0, 0}, p + Vec3{tick, 0, 0}, accent);
        renderer.addLine(p - Vec3{0, tick, 0}, p + Vec3{0, tick, 0}, accent);
        renderer.addLine(p - Vec3{0, 0, tick}, p + Vec3{0, 0, tick}, accent);
        return;
    }

    renderer.addLine(result.from, result.to, accent);

    // End ticks perpendicular to the span, so the extent is unambiguous
    // against the geometry behind it.
    const Vec3 axis = normalize(result.to - result.from);
    const Vec3 side = perpendicular(axis);
    const Vec3 up = cross(axis, side);
    for (const Vec3& p : {result.from, result.to}) {
        const Real t = camera.pixelWorldSize(p) * 5.0;
        renderer.addLine(p - side * t, p + side * t, accent);
        renderer.addLine(p - up * t, p + up * t, accent);
    }
}

} // namespace tg
