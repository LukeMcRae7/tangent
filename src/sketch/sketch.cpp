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

const char* sketchRuleName(SketchRule rule) {
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
        case SketchRule::Smooth:        return "smooth";
    }
    return "constraint";
}

namespace {

// The name this file has always used internally.
const char* ruleName(SketchRule rule) { return sketchRuleName(rule); }

// Ids are handed out in increasing order and everything is appended, so each
// list is sorted by id and a lookup is a binary search: an imported drawing has
// tens of thousands of points, and every sample of every curve looks up its
// control points. A list put together some other way still finds what is in
// it, by looking through the whole of it.
template <class V>
auto findById(V& v, SketchId id) -> decltype(&v[0]) {
    auto it = std::lower_bound(v.begin(), v.end(), id,
                               [](const auto& x, SketchId want) { return x.id < want; });
    if (it != v.end() && it->id == id) return &*it;
    if (id == kNoSketchId) return nullptr;
    auto any = std::find_if(v.begin(), v.end(), [id](const auto& x) { return x.id == id; });
    return any == v.end() ? nullptr : &*any;
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
    std::vector<SketchId> freePoints, freeEntities;

    // The pull a drag applies, and what it is pulling. Its target lives in
    // `store` like everything else, but is not among the unknowns: the solver
    // reads it and never moves it.
    GCS::Point dragTarget;
    double* dragRadius = nullptr;
    bool dragActive = false;

    // What some constraint reaches. Only these go to the solver as unknowns:
    // everything else is free by definition and nothing can move it, and
    // handing the solver an imported drawing's fifteen thousand coordinates
    // for the sake of a dozen level lines made its diagnosis take a second.
    std::unordered_set<const double*> used;
    size_t undeclared = 0;
    // A drag of something no constraint reaches moves it directly.
    GCS::Point* directPoint = nullptr;
    double* directRadius = nullptr;

    void use(const GCS::Point& p) { used.insert(p.x); used.insert(p.y); }
    void use(const GCS::Line& l) { use(l.p1); use(l.p2); }
    void use(const GCS::Circle& c) { use(c.center); used.insert(c.rad); }
    void use(const GCS::Arc& a) {
        use(a.center); use(a.start); use(a.end);
        used.insert(a.rad); used.insert(a.startAngle); used.insert(a.endAngle);
    }
    // Whichever of these `id` is.
    void useEntity(SketchId id) {
        if (GCS::Line* l = L(id)) use(*l);
        if (GCS::Circle* c = C(id)) use(*c);
        if (GCS::Arc* a = A(id)) use(*a);
    }

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
            use(arc);
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

    std::vector<double*> declared;
    declared.reserve(used.size());
    for (double* u : unknowns)
        if (used.count(u)) declared.push_back(u);
    undeclared = unknowns.size() - declared.size();
    sys.declareUnknowns(declared);
    sys.initSolution();

    // What is free and what disagrees is a property of how the sketch is put
    // together, not of the numbers in it, so it is found once here rather than
    // on every solve.
    freedoms = sys.dofsNumber() + static_cast<int>(undeclared);
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

    // Which parameters the solver could not pin down. Everything is free when
    // nothing constrains anything, which is the state a sketch starts in and
    // the one planegcs reports as an empty diagnosis rather than as a list.
    std::unordered_set<const double*> loose;
    if (sys.isEmptyDiagnoseMatrix()) {
        for (const double* p : declared) loose.insert(p);
    } else {
        GCS::VEC_pD dependent;
        sys.getDependentParams(dependent);
        loose.insert(dependent.begin(), dependent.end());
    }
    for (const double* p : unknowns)
        if (!used.count(p)) loose.insert(p);

    for (const SketchPoint& p : sketch.points) {
        const GCS::Point* g = P(p.id);
        if (g && (loose.count(g->x) || loose.count(g->y))) freePoints.push_back(p.id);
    }
    for (const SketchEntity& e : sketch.entities) {
        bool free = false;
        for (SketchId p : {e.a, e.b, e.c, e.d})
            if (p != kNoSketchId &&
                std::find(freePoints.begin(), freePoints.end(), p) != freePoints.end())
                free = true;
        if (const GCS::Circle* c = C(e.id)) free = free || loose.count(c->rad) > 0;
        if (const GCS::Arc* a = A(e.id))
            free = free || loose.count(a->rad) || loose.count(a->startAngle) ||
                   loose.count(a->endAngle);
        if (free) freeEntities.push_back(e.id);
    }
    // Sorted, so a sketch of thousands of points can be asked about each one
    // every frame without searching the whole list each time.
    std::sort(freePoints.begin(), freePoints.end());
    std::sort(freeEntities.begin(), freeEntities.end());
}

bool SketchSolver::Impl::addConstraint(const SketchConstraint& k) {
    const int tag = static_cast<int>(k.id);
    // What it names, point or curve, is reached by it.
    for (SketchId named : {k.first, k.second}) {
        if (named == kNoSketchId) continue;
        if (GCS::Point* p = P(named)) use(*p);
        useEntity(named);
    }
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
    case SketchRule::Smooth: {
        // The first curve's last handle and the second's first, mirrored
        // about the point they share: the tangent runs straight through it.
        const SketchEntity* e1 = sketch.entity(k.first);
        const SketchEntity* e2 = sketch.entity(k.second);
        if (!e1 || !e2 || e1->curve != SketchCurve::Bezier || e2->curve != SketchCurve::Bezier) return false;
        GCS::Point* in = P(e1->c);
        GCS::Point* out = P(e2->b);
        GCS::Point* at = P(e2->a);
        if (!in || !out || !at || e1->d != e2->a) return false;
        use(*in);
        use(*out);
        use(*at);
        sys.addConstraintP2PSymmetric(*in, *out, *at, tag);
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
    out.freePoints = m.freePoints;
    out.freeEntities = m.freeEntities;

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

bool SketchSolver::dragging() const { return impl_->dragActive; }

bool SketchSolver::beginDrag(SketchId point) {
    Impl& m = *impl_;
    if (!m.buildError.empty() || m.dragActive) return false;
    GCS::Point* p = m.P(point);
    if (!p) return false;
    if (!m.used.count(p->x)) {
        m.directPoint = p;
        m.dragActive = true;
        return true;
    }
    // One pair of doubles for the life of the solver: the target is written
    // over on every drag rather than allocated again.
    if (!m.dragTarget.x) {
        m.dragTarget.x = m.fixed(*p->x);
        m.dragTarget.y = m.fixed(*p->y);
    } else {
        *m.dragTarget.x = *p->x;
        *m.dragTarget.y = *p->y;
    }
    m.sys.addConstraintP2PCoincident(*p, m.dragTarget, GCS::DefaultTemporaryConstraint);
    m.sys.initSolution();
    m.dragActive = true;
    return true;
}

bool SketchSolver::beginRadiusDrag(SketchId entity) {
    Impl& m = *impl_;
    if (!m.buildError.empty() || m.dragActive) return false;
    GCS::Circle* c = m.C(entity);
    GCS::Arc* a = m.A(entity);
    if (!c && !a) return false;
    const double now = c ? *c->rad : *a->rad;
    double* rad = c ? c->rad : a->rad;
    if (!m.used.count(rad)) {
        m.directRadius = rad;
        m.dragActive = true;
        return true;
    }
    if (!m.dragRadius) m.dragRadius = m.fixed(now);
    else               *m.dragRadius = now;
    if (c) m.sys.addConstraintCircleRadius(*c, m.dragRadius, GCS::DefaultTemporaryConstraint);
    else   m.sys.addConstraintArcRadius(*a, m.dragRadius, GCS::DefaultTemporaryConstraint);
    m.sys.initSolution();
    m.dragActive = true;
    return true;
}

bool SketchSolver::dragTo(Vec2 at) {
    Impl& m = *impl_;
    if (m.dragActive && m.directPoint) {
        *m.directPoint->x = at.x;
        *m.directPoint->y = at.y;
        m.readBack();
        return true;
    }
    if (!m.dragActive || !m.dragTarget.x) return false;
    *m.dragTarget.x = at.x;
    *m.dragTarget.y = at.y;
    // Coarse: a drag is judged by eye, and is followed by a full solve when it
    // is let go of.
    if (m.sys.solve(false, GCS::DogLeg) != GCS::Success) return false;
    m.sys.applySolution();
    m.readBack();
    return true;
}

bool SketchSolver::dragRadiusTo(Real radius) {
    Impl& m = *impl_;
    if (m.dragActive && m.directRadius) {
        *m.directRadius = std::max(radius, 1e-6);
        m.readBack();
        return true;
    }
    if (!m.dragActive || !m.dragRadius) return false;
    *m.dragRadius = std::max(radius, 1e-6);
    if (m.sys.solve(false, GCS::DogLeg) != GCS::Success) return false;
    m.sys.applySolution();
    m.readBack();
    return true;
}

void SketchSolver::endDrag() {
    Impl& m = *impl_;
    if (!m.dragActive) return;
    if (m.directPoint || m.directRadius) {
        m.directPoint = nullptr;
        m.directRadius = nullptr;
        m.dragActive = false;
        return;
    }
    m.sys.clearByTag(GCS::DefaultTemporaryConstraint);
    m.sys.initSolution();
    m.dragActive = false;
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
void sampleInto(const Sketch& sk, const SketchEntity& e, bool reversed, std::vector<Vec2>& out,
                int steps = 48) {
    auto at = [&](SketchId id) {
        const SketchPoint* p = sk.point(id);
        return p ? p->at : Vec2{};
    };
    const int kSteps = std::max(steps, 1);
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

struct Box {
    Vec2 lo{1e300, 1e300}, hi{-1e300, -1e300};
    bool has(Vec2 p) const { return p.x >= lo.x && p.x <= hi.x && p.y >= lo.y && p.y <= hi.y; }
};

// A point strictly inside a loop, for asking whether the loop lies inside
// another. Not a point on it: two loops that touch -- the lobes of a figure of
// eight, a letter's serif meeting the next -- share points, and a shared point
// is on the other loop's boundary, where inside and out is a coin toss. Found
// across the loop's middle height, between the first two places it is crossed.
Vec2 interiorPoint(const std::vector<Vec2>& poly) {
    if (poly.size() < 3) return poly.empty() ? Vec2{} : poly.front();
    Real lo = 1e300, hi = -1e300;
    for (Vec2 p : poly) { lo = std::min(lo, p.y); hi = std::max(hi, p.y); }
    // A little off the exact middle, so the line does not run through a
    // vertex of a symmetric shape.
    for (Real f : {0.5 + 1.0 / 7919.0, 0.37 + 1.0 / 104729.0, 0.63 + 1.0 / 7727.0}) {
        const Real y = lo + (hi - lo) * f;
        std::vector<Real> xs;
        for (size_t i = 0, n = poly.size(); i < n; ++i) {
            const Vec2 a = poly[i], b = poly[(i + 1) % n];
            if ((a.y > y) == (b.y > y)) continue;
            xs.push_back(a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y));
        }
        std::sort(xs.begin(), xs.end());
        // The widest span inside, the safest place for the point.
        Real best = -1;
        Vec2 at{};
        for (size_t i = 0; i + 1 < xs.size(); i += 2)
            if (xs[i + 1] - xs[i] > best) { best = xs[i + 1] - xs[i]; at = {(xs[i] + xs[i + 1]) * 0.5, y}; }
        if (best > 0) return at;
    }
    return poly.front();
}

Box boxOf(const std::vector<Vec2>& poly) {
    Box b;
    for (Vec2 p : poly) {
        b.lo = {std::min(b.lo.x, p.x), std::min(b.lo.y, p.y)};
        b.hi = {std::max(b.hi.x, p.x), std::max(b.hi.y, p.y)};
    }
    return b;
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
    // A loop's box is looked at before the loop itself: most pairs of letters
    // in a drawing are nowhere near each other.
    std::vector<Box> boxes(loops.size());
    std::vector<Vec2> probe(loops.size());
    for (size_t i : live) {
        boxes[i] = boxOf(polys[i]);
        probe[i] = interiorPoint(polys[i]);
    }
    std::vector<int> parent(loops.size(), -1);
    for (size_t i : live) {
        for (size_t j : live) {
            if (i == j) continue;
            const Real ai = std::fabs(loops[i].signedArea);
            const Real aj = std::fabs(loops[j].signedArea);
            if (aj <= ai || !boxes[j].has(probe[i]) || !inside(probe[i], polys[j])) continue;
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

std::vector<SketchId> sketchFilledProfiles(const Sketch& sketch,
                                           const std::vector<SketchProfile>& profiles) {
    std::vector<std::vector<Vec2>> outer(profiles.size());
    std::vector<Real> area(profiles.size());
    std::vector<Box> boxes(profiles.size());
    for (size_t i = 0; i < profiles.size(); ++i) {
        outer[i] = sketchLoopPoints(sketch, profiles[i].outer);
        area[i] = std::fabs(profiles[i].outer.signedArea);
        boxes[i] = boxOf(outer[i]);
    }
    std::vector<Vec2> probe(profiles.size());
    for (size_t i = 0; i < profiles.size(); ++i) probe[i] = interiorPoint(outer[i]);
    std::vector<SketchId> out;
    for (size_t i = 0; i < profiles.size(); ++i) {
        if (outer[i].empty()) continue;
        int around = 0;
        for (size_t j = 0; j < profiles.size(); ++j)
            if (j != i && area[j] > area[i] && boxes[j].has(probe[i]) && inside(probe[i], outer[j]))
                ++around;
        if (around % 2 == 0) out.push_back(profiles[i].key);
    }
    return out;
}

std::vector<Vec2> sketchEntityPoints(const Sketch& sketch, const SketchEntity& entity, int steps) {
    std::vector<Vec2> out;
    sampleInto(sketch, entity, false, out, steps);
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

// ---- Paths ------------------------------------------------------------------

namespace {

// Where an entity starts and ends, as it is stored. A circle has neither.
bool entityEnds(const Sketch& sketch, const SketchEntity& e, Vec2& from, Vec2& to) {
    SketchId a = kNoSketchId, b = kNoSketchId;
    switch (e.curve) {
        case SketchCurve::Line:   a = e.a; b = e.b; break;
        case SketchCurve::Arc:    a = e.b; b = e.c; break;
        case SketchCurve::Bezier: a = e.a; b = e.d; break;
        case SketchCurve::Circle: return false;
    }
    const SketchPoint* p = sketch.point(a);
    const SketchPoint* q = sketch.point(b);
    if (!p || !q) return false;
    from = p->at;
    to = q->at;
    return true;
}

// Two ends are one joint when they sit on each other, whether or not they are
// the same point: an imported drawing joins its curves by position.
bool sameEnd(Vec2 a, Vec2 b) { return lengthSq(a - b) < 1e-12; }

} // namespace

bool sketchPathOf(const Sketch& sketch, const std::vector<SketchId>& entities, SketchPath& out,
                  std::string* reason) {
    out = SketchPath{};
    auto refuse = [&](const char* why) {
        if (reason) *reason = why;
        return false;
    };
    if (entities.empty()) return refuse("the path is empty");

    struct Piece { SketchId id; Vec2 from, to; };
    std::vector<Piece> pieces;
    for (SketchId id : entities) {
        const SketchEntity* e = sketch.entity(id);
        if (!e) return refuse("a curve of the path is no longer in its sketch");
        if (e->construction) return refuse("construction geometry is not a path");
        if (e->curve == SketchCurve::Circle) {
            if (entities.size() != 1) return refuse("a circle is a path on its own, not part of one");
            out.entities = {id};
            out.reversed = {false};
            out.closed = true;
            return true;
        }
        Piece p{id, {}, {}};
        if (!entityEnds(sketch, *e, p.from, p.to)) return refuse("a curve of the path has no ends");
        pieces.push_back(p);
    }

    // How many ends meet at each end: two is a joint, one is an end of the
    // path, three or more is a branch that no single sweep can follow.
    auto meeting = [&](Vec2 at) {
        int n = 0;
        for (const Piece& p : pieces) n += sameEnd(p.from, at) + sameEnd(p.to, at);
        return n;
    };
    size_t start = 0;
    bool startReversed = false;
    int loose = 0;
    for (size_t i = 0; i < pieces.size(); ++i) {
        const int f = meeting(pieces[i].from), t = meeting(pieces[i].to);
        if (f > 2 || t > 2) return refuse("the path branches: three curves meet at one point");
        if (f == 1 && loose++ == 0) { start = i; startReversed = false; }
        if (t == 1 && loose++ == 0) { start = i; startReversed = true; }
    }
    if (loose != 0 && loose != 2) return refuse("the curves of the path do not all join end to end");

    // Walked from one end -- or, round a loop, from the first curve given.
    std::vector<bool> used(pieces.size(), false);
    size_t at = start;
    bool rev = startReversed;
    for (;;) {
        used[at] = true;
        out.entities.push_back(pieces[at].id);
        out.reversed.push_back(rev);
        const Vec2 end = rev ? pieces[at].from : pieces[at].to;
        bool found = false;
        for (size_t j = 0; j < pieces.size() && !found; ++j) {
            if (used[j]) continue;
            if (sameEnd(pieces[j].from, end))    { at = j; rev = false; found = true; }
            else if (sameEnd(pieces[j].to, end)) { at = j; rev = true;  found = true; }
        }
        if (!found) break;
    }
    if (out.entities.size() != pieces.size())
        return refuse("the curves of the path do not all join end to end");
    out.closed = loose == 0;
    return true;
}

bool sketchPathThrough(const Sketch& sketch, SketchId entity, SketchPath& out, std::string* reason) {
    const SketchEntity* first = sketch.entity(entity);
    if (!first) {
        if (reason) *reason = "that is not a curve of the sketch";
        return false;
    }
    if (first->curve == SketchCurve::Circle || first->construction)
        return sketchPathOf(sketch, {entity}, out, reason);

    // Out from the one clicked, through every joint two curves share, and no
    // further than a point where a third meets them.
    std::vector<SketchId> run{entity};
    std::vector<Vec2> open;
    {
        Vec2 a, b;
        if (entityEnds(sketch, *first, a, b)) { open.push_back(a); open.push_back(b); }
    }
    while (!open.empty()) {
        const Vec2 at = open.back();
        open.pop_back();
        std::vector<const SketchEntity*> here;
        for (const SketchEntity& e : sketch.entities) {
            if (e.construction || e.curve == SketchCurve::Circle) continue;
            Vec2 a, b;
            if (!entityEnds(sketch, e, a, b)) continue;
            if (sameEnd(a, at) || sameEnd(b, at)) here.push_back(&e);
        }
        if (here.size() != 2) continue;
        for (const SketchEntity* e : here) {
            if (std::find(run.begin(), run.end(), e->id) != run.end()) continue;
            run.push_back(e->id);
            Vec2 a, b;
            entityEnds(sketch, *e, a, b);
            open.push_back(sameEnd(a, at) ? b : a);
        }
    }
    return sketchPathOf(sketch, run, out, reason);
}

std::vector<Vec2> sketchPathPoints(const Sketch& sketch, const SketchPath& path, int steps) {
    std::vector<Vec2> out;
    for (size_t k = 0; k < path.entities.size(); ++k)
        if (const SketchEntity* e = sketch.entity(path.entities[k]))
            sampleInto(sketch, *e, k < path.reversed.size() && path.reversed[k], out, steps);
    return out;
}

} // namespace tg
