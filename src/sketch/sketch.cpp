#include "sketch/sketch.h"

#include "planegcs/GCS.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <unordered_map>
#include <unordered_set>

namespace tg {

bool isDimension(SketchRule rule) {
    return rule == SketchRule::Distance || rule == SketchRule::Radius || rule == SketchRule::Angle;
}

namespace {

const char* ruleName(SketchRule rule) {
    switch (rule) {
        case SketchRule::Coincident:    return "coincident";
        case SketchRule::Horizontal:    return "horizontal";
        case SketchRule::Vertical:      return "vertical";
        case SketchRule::Parallel:      return "parallel";
        case SketchRule::Perpendicular: return "perpendicular";
        case SketchRule::Tangent:       return "tangent";
        case SketchRule::Equal:         return "equal";
        case SketchRule::Concentric:    return "concentric";
        case SketchRule::Fix:           return "fixed";
        case SketchRule::Distance:      return "distance";
        case SketchRule::Radius:        return "radius";
        case SketchRule::Angle:         return "angle";
    }
    return "constraint";
}

template <class T>
auto findById(std::vector<T>& v, SketchId id) {
    auto it = std::find_if(v.begin(), v.end(), [id](const T& x) { return x.id == id; });
    return it == v.end() ? nullptr : &*it;
}
template <class T>
auto findById(const std::vector<T>& v, SketchId id) {
    auto it = std::find_if(v.begin(), v.end(), [id](const T& x) { return x.id == id; });
    return it == v.end() ? nullptr : &*it;
}

} // namespace

// ---- Building ---------------------------------------------------------------

SketchId Sketch::addPoint(Vec2 at) {
    const SketchId id = nextId++;
    points.push_back({id, at});
    return id;
}

SketchId Sketch::addLine(SketchId from, SketchId to) {
    SketchEntity e;
    e.id = nextId++;
    e.curve = SketchCurve::Line;
    e.a = from;
    e.b = to;
    entities.push_back(e);
    return e.id;
}

SketchId Sketch::addCircle(SketchId centre, Real radius) {
    SketchEntity e;
    e.id = nextId++;
    e.curve = SketchCurve::Circle;
    e.a = centre;
    e.radius = radius;
    entities.push_back(e);
    return e.id;
}

SketchId Sketch::addArc(SketchId centre, SketchId start, SketchId end) {
    SketchEntity e;
    e.id = nextId++;
    e.curve = SketchCurve::Arc;
    e.a = centre;
    e.b = start;
    e.c = end;
    const SketchPoint* c = point(centre);
    const SketchPoint* s = point(start);
    e.radius = (c && s) ? length(s->at - c->at) : 0.0;
    entities.push_back(e);
    return e.id;
}

SketchId Sketch::addBezier(SketchId a, SketchId b, SketchId c, SketchId d) {
    SketchEntity e;
    e.id = nextId++;
    e.curve = SketchCurve::Bezier;
    e.a = a;
    e.b = b;
    e.c = c;
    e.d = d;
    entities.push_back(e);
    return e.id;
}

SketchId Sketch::constrain(SketchRule rule, SketchId first, SketchId second, Real value,
                           Real value2) {
    SketchConstraint k;
    k.id = nextId++;
    k.rule = rule;
    k.first = first;
    k.second = second;
    k.value = value;
    k.value2 = value2;
    constraints.push_back(k);
    return k.id;
}

SketchPoint*            Sketch::point(SketchId id)            { return findById(points, id); }
const SketchPoint*      Sketch::point(SketchId id) const      { return findById(points, id); }
SketchEntity*           Sketch::entity(SketchId id)           { return findById(entities, id); }
const SketchEntity*     Sketch::entity(SketchId id) const     { return findById(entities, id); }
SketchConstraint*       Sketch::constraint(SketchId id)       { return findById(constraints, id); }
const SketchConstraint* Sketch::constraint(SketchId id) const { return findById(constraints, id); }

Sketch::Rectangle Sketch::addRectangle(Vec2 min, Real width, Real height) {
    const SketchId p0 = addPoint(min);
    const SketchId p1 = addPoint({min.x + width, min.y});
    const SketchId p2 = addPoint({min.x + width, min.y + height});
    const SketchId p3 = addPoint({min.x, min.y + height});
    Rectangle r{};
    r.bottom = addLine(p0, p1);
    r.right = addLine(p1, p2);
    r.top = addLine(p2, p3);
    r.left = addLine(p3, p0);
    constrain(SketchRule::Horizontal, r.bottom);
    constrain(SketchRule::Horizontal, r.top);
    constrain(SketchRule::Vertical, r.left);
    constrain(SketchRule::Vertical, r.right);
    constrain(SketchRule::Fix, p0, kNoSketchId, min.x, min.y);
    r.width = constrain(SketchRule::Distance, p0, p1, width);
    r.height = constrain(SketchRule::Distance, p0, p3, height);
    return r;
}

// ---- Solving ----------------------------------------------------------------

struct SketchSolver::Impl {
    Sketch& sketch;

    // The solver holds pointers to these doubles, so they live in a deque:
    // growing it never moves what is already there.
    std::deque<double> store;
    std::vector<double*> unknowns;
    GCS::System sys;

    // planegcs geometry is a set of pointers into `store`, so copies are cheap
    // and all refer to the same values.
    std::unordered_map<SketchId, GCS::Point>  pts;
    std::unordered_map<SketchId, GCS::Line>   lines;
    std::unordered_map<SketchId, GCS::Circle> circles;
    std::unordered_map<SketchId, GCS::Arc>    arcs;

    // Where each dimension's value lives, so it can change without the sketch
    // being described to the solver again.
    std::unordered_map<SketchId, double*> dimensions;

    std::string buildError;
    int freedoms = 0;
    std::vector<SketchId> conflicting, redundant;

    explicit Impl(Sketch& s) : sketch(s) { build(); }

    double* unknown(double v) {
        store.push_back(v);
        unknowns.push_back(&store.back());
        return &store.back();
    }
    double* fixed(double v) {
        store.push_back(v);
        return &store.back();
    }

    GCS::Point*  P(SketchId id) { auto it = pts.find(id);     return it == pts.end() ? nullptr : &it->second; }
    GCS::Line*   L(SketchId id) { auto it = lines.find(id);   return it == lines.end() ? nullptr : &it->second; }
    GCS::Circle* C(SketchId id) { auto it = circles.find(id); return it == circles.end() ? nullptr : &it->second; }
    GCS::Arc*    A(SketchId id) { auto it = arcs.find(id);    return it == arcs.end() ? nullptr : &it->second; }

    // The centre point of a circle or an arc, whichever `id` is.
    GCS::Point* centreOf(SketchId id) {
        if (GCS::Circle* c = C(id)) return &c->center;
        if (GCS::Arc* a = A(id)) return &a->center;
        return nullptr;
    }

    void build();
    bool addConstraint(const SketchConstraint& k);
    void readBack();
};

void SketchSolver::Impl::build() {
    for (const SketchPoint& p : sketch.points) {
        GCS::Point g;
        g.x = unknown(p.at.x);
        g.y = unknown(p.at.y);
        pts[p.id] = g;
    }

    for (const SketchEntity& e : sketch.entities) {
        switch (e.curve) {
        case SketchCurve::Line: {
            GCS::Point* a = P(e.a);
            GCS::Point* b = P(e.b);
            if (!a || !b) { buildError = "a line is missing one of its ends"; return; }
            GCS::Line l;
            l.p1 = *a;
            l.p2 = *b;
            lines[e.id] = l;
            break;
        }
        case SketchCurve::Circle: {
            GCS::Point* c = P(e.a);
            if (!c) { buildError = "a circle is missing its centre"; return; }
            GCS::Circle circle;
            circle.center = *c;
            circle.rad = unknown(e.radius);
            circles[e.id] = circle;
            break;
        }
        case SketchCurve::Arc: {
            GCS::Point* c = P(e.a);
            GCS::Point* s = P(e.b);
            GCS::Point* t = P(e.c);
            if (!c || !s || !t) { buildError = "an arc is missing a centre or an end"; return; }
            GCS::Arc arc;
            arc.center = *c;
            arc.start = *s;
            arc.end = *t;
            double a0 = std::atan2(*s->y - *c->y, *s->x - *c->x);
            double a1 = std::atan2(*t->y - *c->y, *t->x - *c->x);
            if (a1 <= a0) a1 += kTwoPi;   // counter-clockwise from start to end
            const double r = std::hypot(*s->x - *c->x, *s->y - *c->y);
            arc.rad = unknown(r);
            arc.startAngle = unknown(a0);
            arc.endAngle = unknown(a1);
            // Ties the end points to the centre, the radius and the angles.
            // Tagged 0: it belongs to the arc rather than to anything a person
            // said, so a diagnosis never names it.
            sys.addConstraintArcRules(arc, 0);
            arcs[e.id] = arc;
            break;
        }
        case SketchCurve::Bezier:
            // Its control points are ordinary points and are constrained as
            // such; the curve adds no rules of its own.
            if (!P(e.a) || !P(e.b) || !P(e.c) || !P(e.d)) {
                buildError = "a curve is missing a control point";
                return;
            }
            break;
        }
    }

    for (const SketchConstraint& k : sketch.constraints) {
        if (!addConstraint(k)) {
            buildError = std::string("a ") + ruleName(k.rule) +
                         " constraint refers to geometry it cannot apply to";
            return;
        }
    }

    sys.declareUnknowns(unknowns);
    sys.initSolution();

    // What is free and what disagrees is a property of how the sketch is put
    // together, not of the numbers in it, so it is found once here rather than
    // on every solve.
    freedoms = sys.dofsNumber();
    GCS::VEC_I bad, again;
    sys.getConflicting(bad);
    sys.getRedundant(again);
    auto keep = [](const GCS::VEC_I& tags, std::vector<SketchId>& out) {
        for (int t : tags)
            if (t > 0) out.push_back(static_cast<SketchId>(t));
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
    };
    keep(bad, conflicting);
    keep(again, redundant);
}

bool SketchSolver::Impl::addConstraint(const SketchConstraint& k) {
    const int tag = static_cast<int>(k.id);
    switch (k.rule) {
    case SketchRule::Coincident: {
        GCS::Point* p = P(k.first);
        GCS::Point* q = P(k.second);
        if (!p || !q) return false;
        sys.addConstraintP2PCoincident(*p, *q, tag);
        return true;
    }
    case SketchRule::Horizontal:
        if (GCS::Line* l = L(k.first)) { sys.addConstraintHorizontal(*l, tag); return true; }
        return false;
    case SketchRule::Vertical:
        if (GCS::Line* l = L(k.first)) { sys.addConstraintVertical(*l, tag); return true; }
        return false;
    case SketchRule::Parallel: {
        GCS::Line* l1 = L(k.first);
        GCS::Line* l2 = L(k.second);
        if (!l1 || !l2) return false;
        sys.addConstraintParallel(*l1, *l2, tag);
        return true;
    }
    case SketchRule::Perpendicular: {
        GCS::Line* l1 = L(k.first);
        GCS::Line* l2 = L(k.second);
        if (!l1 || !l2) return false;
        sys.addConstraintPerpendicular(*l1, *l2, tag);
        return true;
    }
    case SketchRule::Tangent: {
        // Put a line first and a circle before an arc: the solver has an
        // overload for each ordering it understands and nothing else.
        SketchId x = k.first, y = k.second;
        if (!L(x) && L(y)) std::swap(x, y);
        if (A(x) && C(y)) std::swap(x, y);
        if (GCS::Line* l = L(x)) {
            if (GCS::Circle* c = C(y)) { sys.addConstraintTangent(*l, *c, tag); return true; }
            if (GCS::Arc* a = A(y))    { sys.addConstraintTangent(*l, *a, tag); return true; }
            return false;
        }
        if (GCS::Circle* c1 = C(x)) {
            if (GCS::Circle* c2 = C(y)) { sys.addConstraintTangent(*c1, *c2, tag); return true; }
            if (GCS::Arc* a2 = A(y))    { sys.addConstraintTangent(*c1, *a2, tag); return true; }
            return false;
        }
        if (GCS::Arc* a1 = A(x)) {
            if (GCS::Arc* a2 = A(y)) { sys.addConstraintTangent(*a1, *a2, tag); return true; }
        }
        return false;
    }
    case SketchRule::Equal: {
        if (GCS::Line* l1 = L(k.first)) {
            GCS::Line* l2 = L(k.second);
            if (!l2) return false;
            sys.addConstraintEqualLength(*l1, *l2, tag);
            return true;
        }
        SketchId x = k.first, y = k.second;
        if (A(x) && C(y)) std::swap(x, y);
        if (GCS::Circle* c1 = C(x)) {
            if (GCS::Circle* c2 = C(y)) { sys.addConstraintEqualRadius(*c1, *c2, tag); return true; }
            if (GCS::Arc* a2 = A(y))    { sys.addConstraintEqualRadius(*c1, *a2, tag); return true; }
            return false;
        }
        if (GCS::Arc* a1 = A(x)) {
            if (GCS::Arc* a2 = A(y)) { sys.addConstraintEqualRadius(*a1, *a2, tag); return true; }
        }
        return false;
    }
    case SketchRule::Concentric: {
        GCS::Point* c1 = centreOf(k.first);
        GCS::Point* c2 = centreOf(k.second);
        if (!c1 || !c2) return false;
        sys.addConstraintP2PCoincident(*c1, *c2, tag);
        return true;
    }
    case SketchRule::Fix: {
        GCS::Point* p = P(k.first);
        if (!p) return false;
        // Two rules under one tag, so a diagnosis names the one fix.
        sys.addConstraintCoordinateX(*p, fixed(k.value), tag);
        sys.addConstraintCoordinateY(*p, fixed(k.value2), tag);
        return true;
    }
    case SketchRule::Distance: {
        GCS::Point* p = P(k.first);
        GCS::Point* q = P(k.second);
        if (!p || !q) return false;
        double* v = fixed(k.value);
        sys.addConstraintP2PDistance(*p, *q, v, tag);
        dimensions[k.id] = v;
        return true;
    }
    case SketchRule::Radius: {
        double* v = fixed(k.value);
        if (GCS::Circle* c = C(k.first))   sys.addConstraintCircleRadius(*c, v, tag);
        else if (GCS::Arc* a = A(k.first)) sys.addConstraintArcRadius(*a, v, tag);
        else return false;
        dimensions[k.id] = v;
        return true;
    }
    case SketchRule::Angle: {
        GCS::Line* l1 = L(k.first);
        GCS::Line* l2 = L(k.second);
        if (!l1 || !l2) return false;
        double* v = fixed(k.value);
        sys.addConstraintL2LAngle(*l1, *l2, v, tag);
        dimensions[k.id] = v;
        return true;
    }
    }
    return false;
}

void SketchSolver::Impl::readBack() {
    for (SketchPoint& p : sketch.points) {
        if (GCS::Point* g = P(p.id)) p.at = {*g->x, *g->y};
    }
    for (SketchEntity& e : sketch.entities) {
        if (GCS::Circle* c = C(e.id)) e.radius = *c->rad;
        else if (GCS::Arc* a = A(e.id)) e.radius = *a->rad;
    }
}

SketchSolver::SketchSolver(Sketch& sketch) : impl_(std::make_unique<Impl>(sketch)) {}
SketchSolver::~SketchSolver() = default;

SketchSolve SketchSolver::solve() {
    Impl& m = *impl_;
    SketchSolve out;
    out.freedoms = m.freedoms;
    out.conflicting = m.conflicting;
    out.redundant = m.redundant;

    if (!m.buildError.empty()) {
        out.reason = m.buildError;
        return out;
    }
    if (!out.conflicting.empty()) {
        // No solution exists, so nothing is moved toward one: the geometry
        // stays where it was drawn, and the constraints to look at are named.
        out.reason = "these constraints cannot all hold:";
        for (SketchId id : out.conflicting) {
            const SketchConstraint* k = m.sketch.constraint(id);
            out.reason += std::string(" ") + (k ? ruleName(k->rule) : "constraint") + " #" +
                          std::to_string(id);
        }
        return out;
    }

    if (m.sys.solve(true, GCS::DogLeg) != GCS::Success) {
        out.reason = "the sketch does not settle; a dimension may be out of reach";
        return out;
    }
    m.sys.applySolution();
    m.readBack();
    out.solved = true;
    return out;
}

bool SketchSolver::setDimension(SketchId constraint, Real value) {
    auto it = impl_->dimensions.find(constraint);
    if (it == impl_->dimensions.end()) return false;
    *it->second = value;
    if (SketchConstraint* k = impl_->sketch.constraint(constraint)) k->value = value;
    return true;
}

SketchSolve solveSketch(Sketch& sketch) {
    SketchSolver solver(sketch);
    return solver.solve();
}

// ---- Profiles ---------------------------------------------------------------

namespace {

// Points joined by a Coincident constraint are the same corner for the purpose
// of deciding what closes, the same as points that were shared from the start.
struct Corners {
    std::unordered_map<SketchId, SketchId> parent;
    SketchId find(SketchId x) {
        auto it = parent.find(x);
        if (it == parent.end()) {
            parent.emplace(x, x);
            return x;
        }
        if (it->second == x) return x;
        const SketchId root = find(it->second);
        parent[x] = root;
        return root;
    }
    void join(SketchId a, SketchId b) {
        const SketchId ra = find(a), rb = find(b);
        if (ra != rb) parent[ra] = rb;
    }
};

// Where an entity starts and ends; false for a curve that closes on itself.
bool endsOf(const SketchEntity& e, SketchId& s, SketchId& t) {
    switch (e.curve) {
        case SketchCurve::Line:   s = e.a; t = e.b; return true;
        case SketchCurve::Arc:    s = e.b; t = e.c; return true;
        case SketchCurve::Bezier: s = e.a; t = e.d; return true;
        case SketchCurve::Circle: return false;
    }
    return false;
}

// Points along an entity in the direction it is traversed. Exact at the ends
// and sampled between them: enough to tell which loop lies inside which, and to
// measure an area, neither of which needs to be exact to the micron.
void sampleInto(const Sketch& sk, const SketchEntity& e, bool reversed, std::vector<Vec2>& out) {
    auto at = [&](SketchId id) {
        const SketchPoint* p = sk.point(id);
        return p ? p->at : Vec2{};
    };
    constexpr int kSteps = 48;
    std::vector<Vec2> pts;
    switch (e.curve) {
    case SketchCurve::Line:
        pts = {at(e.a), at(e.b)};
        break;
    case SketchCurve::Circle: {
        const Vec2 c = at(e.a);
        for (int i = 0; i <= kSteps; ++i) {
            const Real t = kTwoPi * i / kSteps;
            pts.push_back({c.x + e.radius * std::cos(t), c.y + e.radius * std::sin(t)});
        }
        break;
    }
    case SketchCurve::Arc: {
        const Vec2 c = at(e.a), s = at(e.b), t = at(e.c);
        Real a0 = std::atan2(s.y - c.y, s.x - c.x);
        Real a1 = std::atan2(t.y - c.y, t.x - c.x);
        if (a1 <= a0) a1 += kTwoPi;
        const Real r = length(s - c);
        pts.push_back(s);
        for (int i = 1; i < kSteps; ++i) {
            const Real ang = a0 + (a1 - a0) * i / kSteps;
            pts.push_back({c.x + r * std::cos(ang), c.y + r * std::sin(ang)});
        }
        pts.push_back(t);
        break;
    }
    case SketchCurve::Bezier: {
        const Vec2 p0 = at(e.a), p1 = at(e.b), p2 = at(e.c), p3 = at(e.d);
        for (int i = 0; i <= kSteps; ++i) {
            const Real u = static_cast<Real>(i) / kSteps, v = 1.0 - u;
            pts.push_back(p0 * (v * v * v) + p1 * (3 * v * v * u) + p2 * (3 * v * u * u) +
                          p3 * (u * u * u));
        }
        break;
    }
    }
    if (reversed) std::reverse(pts.begin(), pts.end());
    // The first point of every entity after the first is the last point of the
    // one before it; keeping both would count that corner twice.
    if (!out.empty() && !pts.empty()) pts.erase(pts.begin());
    out.insert(out.end(), pts.begin(), pts.end());
}

Real shoelace(const std::vector<Vec2>& poly) {
    Real twice = 0;
    for (size_t i = 0, n = poly.size(); i < n; ++i) {
        const Vec2& a = poly[i];
        const Vec2& b = poly[(i + 1) % n];
        twice += a.x * b.y - b.x * a.y;
    }
    return twice * 0.5;
}

bool inside(Vec2 p, const std::vector<Vec2>& poly) {
    bool in = false;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const Vec2& a = poly[i];
        const Vec2& b = poly[j];
        if ((a.y > p.y) != (b.y > p.y) &&
            p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)
            in = !in;
    }
    return in;
}

} // namespace

std::vector<SketchProfile> sketchProfiles(const Sketch& sk) {
    Corners corners;
    for (const SketchConstraint& k : sk.constraints)
        if (k.rule == SketchRule::Coincident) corners.join(k.first, k.second);

    struct Edge {
        SketchId entity;
        SketchId s, t;   // corners, after joining
    };
    std::vector<Edge> edges;
    std::vector<SketchLoop> loops;

    for (const SketchEntity& e : sk.entities) {
        if (e.construction) continue;
        SketchId s = kNoSketchId, t = kNoSketchId;
        if (!endsOf(e, s, t)) {
            SketchLoop loop;            // a circle closes on its own
            loop.entities = {e.id};
            loop.reversed = {false};
            loops.push_back(loop);
            continue;
        }
        edges.push_back({e.id, corners.find(s), corners.find(t)});
    }

    std::unordered_map<SketchId, std::vector<size_t>> at;   // corner -> edges meeting there
    for (size_t i = 0; i < edges.size(); ++i) {
        at[edges[i].s].push_back(i);
        at[edges[i].t].push_back(i);
    }

    std::vector<bool> seen(edges.size(), false);
    for (size_t first = 0; first < edges.size(); ++first) {
        if (seen[first]) continue;

        // The whole connected piece, before deciding anything about it.
        std::vector<size_t> piece;
        std::vector<SketchId> frontier{edges[first].s};
        std::unordered_set<SketchId> visited{edges[first].s};
        while (!frontier.empty()) {
            const SketchId corner = frontier.back();
            frontier.pop_back();
            for (size_t ei : at[corner]) {
                if (seen[ei]) continue;
                seen[ei] = true;
                piece.push_back(ei);
                for (SketchId next : {edges[ei].s, edges[ei].t})
                    if (visited.insert(next).second) frontier.push_back(next);
            }
        }

        // It closes into one loop only if every corner on it joins exactly two
        // entities. An open end or a branch is left out whole rather than having
        // a loop guessed out of it.
        bool simple = true;
        for (size_t ei : piece)
            if (at[edges[ei].s].size() != 2 || at[edges[ei].t].size() != 2) simple = false;
        if (!simple || piece.empty()) continue;

        SketchLoop loop;
        size_t cur = piece.front();
        SketchId from = edges[cur].s;
        for (size_t step = 0; step < piece.size(); ++step) {
            const Edge& e = edges[cur];
            const bool rev = e.s != from;
            loop.entities.push_back(e.entity);
            loop.reversed.push_back(rev);
            const SketchId next = rev ? e.s : e.t;
            const std::vector<size_t>& here = at[next];
            from = next;
            cur = here[0] == cur ? here[1] : here[0];
        }
        loops.push_back(loop);
    }

    // Measure every loop once.
    std::vector<std::vector<Vec2>> polys(loops.size());
    for (size_t i = 0; i < loops.size(); ++i) {
        for (size_t k = 0; k < loops[i].entities.size(); ++k)
            if (const SketchEntity* e = sk.entity(loops[i].entities[k]))
                sampleInto(sk, *e, loops[i].reversed[k], polys[i]);
        loops[i].signedArea = shoelace(polys[i]);
    }

    // A loop with no area bounds nothing. Dropped here, before it can be
    // mistaken for a container of anything.
    std::vector<size_t> live;
    for (size_t i = 0; i < loops.size(); ++i)
        if (polys[i].size() >= 3 && std::fabs(loops[i].signedArea) > 1e-12) live.push_back(i);

    // The nearest loop around each one, which is the region it is a hole in.
    std::vector<int> parent(loops.size(), -1);
    for (size_t i : live) {
        for (size_t j : live) {
            if (i == j) continue;
            const Real ai = std::fabs(loops[i].signedArea);
            const Real aj = std::fabs(loops[j].signedArea);
            if (aj <= ai || !inside(polys[i].front(), polys[j])) continue;
            if (parent[i] < 0 || aj < std::fabs(loops[static_cast<size_t>(parent[i])].signedArea))
                parent[i] = static_cast<int>(j);
        }
    }

    std::vector<SketchProfile> profiles;
    // Every loop bounds a region: what lies inside it, less what lies inside
    // the loops directly within it. A circle inside a rectangle is a hole in
    // the plate *and* the disc that fills that hole, and which of the two is
    // swept is the user's choice -- a pocket and a boss are drawn the same way.
    for (size_t i : live) {
        SketchProfile p;
        p.outer = loops[i];
        p.key = *std::min_element(p.outer.entities.begin(), p.outer.entities.end());
        Real area = std::fabs(loops[i].signedArea);
        for (size_t j : live) {
            if (parent[j] != static_cast<int>(i)) continue;
            p.holes.push_back(loops[j]);
            area -= std::fabs(loops[j].signedArea);
        }
        p.area = area;
        profiles.push_back(std::move(p));
    }
    std::sort(profiles.begin(), profiles.end(),
              [](const SketchProfile& a, const SketchProfile& b) { return a.key < b.key; });
    return profiles;
}

std::vector<Vec2> sketchEntityPoints(const Sketch& sketch, const SketchEntity& entity) {
    std::vector<Vec2> out;
    sampleInto(sketch, entity, false, out);
    return out;
}

std::vector<Vec2> sketchLoopPoints(const Sketch& sketch, const SketchLoop& loop) {
    std::vector<Vec2> out;
    for (size_t k = 0; k < loop.entities.size(); ++k)
        if (const SketchEntity* e = sketch.entity(loop.entities[k]))
            sampleInto(sketch, *e, k < loop.reversed.size() && loop.reversed[k], out);
    // The loop comes back to where it started; a polygon says that by closing.
    if (out.size() > 1 && lengthSq(out.front() - out.back()) < 1e-18) out.pop_back();
    return out;
}

bool sketchProfileContains(const Sketch& sketch, const SketchProfile& profile, Vec2 at) {
    if (!inside(at, sketchLoopPoints(sketch, profile.outer))) return false;
    for (const SketchLoop& hole : profile.holes)
        if (inside(at, sketchLoopPoints(sketch, hole))) return false;
    return true;
}

} // namespace tg
