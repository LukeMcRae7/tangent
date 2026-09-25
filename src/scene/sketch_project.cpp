#include "scene/sketch_project.h"

#include <algorithm>
#include <cmath>

namespace tg {

namespace {

Vec2 onPlane(const SketchPlane& pl, Vec3 p) {
    const Vec3 d = p - pl.origin;
    return {dot(d, pl.xAxis) / lengthSq(pl.xAxis), dot(d, pl.yAxis) / lengthSq(pl.yAxis)};
}

// What an edge leaves on the plane.
struct Shape {
    enum class Kind { Line, Circle, Arc, Poly } kind = Kind::Poly;
    Vec2 a, b, centre;         // Line a -> b; Arc centre, a = start, b = end (counter-clockwise)
    Real radius = 0;
    std::vector<Vec2> poly;
};

Real angleOf(Vec2 v) { return std::atan2(v.y, v.x); }
Real turnFrom(Real from, Real to) {
    Real d = to - from;
    while (d < 0) d += 2.0 * kPi;
    while (d >= 2.0 * kPi) d -= 2.0 * kPi;
    return d;
}

bool shapeOf(const Sketch& sketch, const Body& body, const Mat4& toSketch, EdgeId edge, Shape& out,
             std::string* why) {
    const SketchPlane& pl = sketch.plane;
    const Vec3 n = pl.normal();
    auto refuse = [&](const char* w) {
        if (why) *why = w;
        return false;
    };
    const CurveKind kind = body.edgeKind(edge);
    Vec3 a3, b3;
    body.edgePositions(edge, a3, b3);
    a3 = transformPoint(toSketch, a3);
    b3 = transformPoint(toSketch, b3);

    if (kind == CurveKind::Line) {
        out.kind = Shape::Kind::Line;
        out.a = onPlane(pl, a3);
        out.b = onPlane(pl, b3);
        if (lengthSq(out.b - out.a) < 1e-12) return refuse("that edge stands straight out of the plane");
        return true;
    }
    Vec3 c3, axis;
    Real r = 0;
    if (kind == CurveKind::Circle && body.edgeCircle(edge, c3, axis, r)) {
        c3 = transformPoint(toSketch, c3);
        axis = transformVector(toSketch, axis);
        if (length(axis) > 1e-12 && std::fabs(dot(normalize(axis), n)) > 1.0 - 1e-9) {
            out.centre = onPlane(pl, c3);
            out.radius = r;
            if (lengthSq(b3 - a3) < 1e-12) {
                out.kind = Shape::Kind::Circle;
                return true;
            }
            // An arc runs counter-clockwise from its start in a sketch: the
            // ends are put the way round that passes through the edge's middle.
            out.kind = Shape::Kind::Arc;
            const Vec2 s = onPlane(pl, a3), e = onPlane(pl, b3);
            const Vec2 m = onPlane(pl, transformPoint(toSketch, body.edgeMidpoint(edge)));
            const Real as = angleOf(s - out.centre), ae = angleOf(e - out.centre), am = angleOf(m - out.centre);
            const bool forward = turnFrom(as, am) < turnFrom(as, ae);
            out.a = forward ? s : e;
            out.b = forward ? e : s;
            return true;
        }
    }
    // Anything else: its shape, as lines.
    out.kind = Shape::Kind::Poly;
    std::vector<Vec3> pts;
    body.edgePolyline(edge, 0.05, pts);
    for (const Vec3& p : pts) {
        const Vec2 q = onPlane(pl, transformPoint(toSketch, p));
        if (out.poly.empty() || lengthSq(q - out.poly.back()) > 1e-12) out.poly.push_back(q);
    }
    if (out.poly.size() < 2) return refuse("that edge stands straight out of the plane");
    return true;
}

bool hasFix(const Sketch& s, SketchId point) {
    return std::any_of(s.constraints.begin(), s.constraints.end(),
                       [&](const SketchConstraint& k) { return k.rule == SketchRule::Fix && k.first == point; });
}

// A point at `at` -- the one already there if there is one, so ends that meet
// are one point -- held where it is.
SketchId fixedPoint(Sketch& s, Vec2 at) {
    SketchId id = kNoSketchId;
    for (const SketchPoint& p : s.points)
        if (lengthSq(p.at - at) < 1e-12) { id = p.id; break; }
    if (id == kNoSketchId) id = s.addPoint(at);
    if (!hasFix(s, id)) s.constrain(SketchRule::Fix, id, kNoSketchId, at.x, at.y);
    return id;
}

// Puts a point, and the Fix that holds it, at `at`.
void movePoint(Sketch& s, SketchId id, Vec2 at) {
    if (SketchPoint* p = s.point(id)) p->at = at;
    for (SketchConstraint& k : s.constraints)
        if (k.rule == SketchRule::Fix && k.first == id) { k.value = at.x; k.value2 = at.y; }
}

void setRadius(Sketch& s, SketchId entity, Real r) {
    if (SketchEntity* e = s.entity(entity)) e->radius = r;
    for (SketchConstraint& k : s.constraints)
        if (k.rule == SketchRule::Radius && k.first == entity) k.value = r;
}

} // namespace

bool projectEdge(Sketch& sketch, const Body& body, const Mat4& toSketch, EdgeId edge, bool link,
                 std::vector<SketchId>* added, std::string* why) {
    if (body.empty() || body.isMesh()) {
        if (why) *why = "only a solid's edges can be projected";
        return false;
    }
    Shape sh;
    if (!shapeOf(sketch, body, toSketch, edge, sh, why)) return false;
    const uint64_t source = link ? body.edgeName(edge) : 0;
    auto made = [&](SketchId id) {
        if (SketchEntity* e = sketch.entity(id)) e->source = sh.kind == Shape::Kind::Poly ? 0 : source;
        if (added) added->push_back(id);
    };
    switch (sh.kind) {
    case Shape::Kind::Line:
        made(sketch.addLine(fixedPoint(sketch, sh.a), fixedPoint(sketch, sh.b)));
        break;
    case Shape::Kind::Circle: {
        const SketchId c = sketch.addCircle(fixedPoint(sketch, sh.centre), sh.radius);
        sketch.constrain(SketchRule::Radius, c, kNoSketchId, sh.radius);
        made(c);
        break;
    }
    case Shape::Kind::Arc: {
        const SketchId c = sketch.addArc(fixedPoint(sketch, sh.centre), fixedPoint(sketch, sh.a),
                                         fixedPoint(sketch, sh.b));
        sketch.constrain(SketchRule::Radius, c, kNoSketchId, sh.radius);
        made(c);
        break;
    }
    case Shape::Kind::Poly:
        for (size_t i = 0; i + 1 < sh.poly.size(); ++i)
            made(sketch.addLine(fixedPoint(sketch, sh.poly[i]), fixedPoint(sketch, sh.poly[i + 1])));
        break;
    }
    return true;
}

bool projectFace(Sketch& sketch, const Body& body, const Mat4& toSketch, FaceId face, bool link,
                 std::vector<SketchId>* added, std::string* why) {
    std::vector<EdgeId> es;
    body.faceEdges(face, es);
    size_t made = 0;
    std::string last;
    for (EdgeId e : es) {
        std::string w;
        if (projectEdge(sketch, body, toSketch, e, link, added, &w)) ++made;
        else last = w;
    }
    if (made == 0) {
        if (why) *why = last.empty() ? "that face has no edges to project" : last;
        return false;
    }
    return true;
}

bool refreshProjections(Sketch& sketch, const Body& body, std::string* why) {
    for (SketchEntity& e : sketch.entities) {
        if (e.source == 0) continue;
        const EdgeId edge = body.empty() || body.isMesh() ? kInvalid : body.findEdge(e.source);
        if (edge == kInvalid) {
            if (why) *why = "an edge it was projected from is gone";
            return false;
        }
        Shape sh;
        if (!shapeOf(sketch, body, Mat4::identity(), edge, sh, why)) return false;
        const SketchEntity copy = e;
        switch (copy.curve) {
        case SketchCurve::Line:
            if (sh.kind != Shape::Kind::Line) break;
            movePoint(sketch, copy.a, sh.a);
            movePoint(sketch, copy.b, sh.b);
            continue;
        case SketchCurve::Circle:
            if (sh.kind != Shape::Kind::Circle) break;
            movePoint(sketch, copy.a, sh.centre);
            setRadius(sketch, copy.id, sh.radius);
            continue;
        case SketchCurve::Arc:
            if (sh.kind != Shape::Kind::Arc) break;
            movePoint(sketch, copy.a, sh.centre);
            movePoint(sketch, copy.b, sh.a);
            movePoint(sketch, copy.c, sh.b);
            setRadius(sketch, copy.id, sh.radius);
            continue;
        case SketchCurve::Bezier:
            break;
        }
        if (why) *why = "an edge it was projected from has become another kind of curve";
        return false;
    }
    return true;
}

} // namespace tg
