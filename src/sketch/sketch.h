// Tangent - a sketch: constrained 2D geometry on a plane, kept in the history.
//
// The create tool has always drawn a profile and then swept it into a prism and
// forgotten it. A sketch is the part it forgot: the lines, circles, arcs and
// curves themselves, what has been said about how they relate, and the numbers
// that size them -- held in the chain so a dimension can be changed later and
// everything built from it follows.
//
// Deliberately small. Lines, circles, arcs and cubic Beziers, because that is
// what a printed part and an imported SVG are made of. Not a drafting package.
//
// Everything here is addressed by SketchId, never by position in a vector. A
// constraint names the entities it relates, a solid built from the sketch
// names its faces after the entities they were swept from, and a diagnosis
// names the constraints that disagree -- and all of that has to survive an edit
// that adds or removes something elsewhere in the sketch.
#pragma once

#include "core/math.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace tg {

using SketchId = uint32_t;
inline constexpr SketchId kNoSketchId = 0;

// Where a sketch lies. Right-handed: the normal is x cross y, so a loop that
// runs counter-clockwise in the sketch's own coordinates bounds a face whose
// normal points along the plane's.
struct SketchPlane {
    Vec3 origin{0, 0, 0};
    Vec3 xAxis{1, 0, 0};
    Vec3 yAxis{0, 1, 0};

    Vec3 normal() const { return normalize(cross(xAxis, yAxis)); }
    Vec3 toWorld(Vec2 p) const { return origin + xAxis * p.x + yAxis * p.y; }
};

// A point that geometry is built on. Two lines meeting at a corner name the
// same point, which makes them coincident without a constraint to say so.
struct SketchPoint {
    SketchId id = kNoSketchId;
    Vec2 at;
};

// New kinds go on the end: the value is what a project file stores.
enum class SketchCurve : uint8_t { Line, Circle, Arc, Bezier };

struct SketchEntity {
    SketchId id = kNoSketchId;
    SketchCurve curve = SketchCurve::Line;

    // Construction geometry constrains the rest and never becomes an edge of
    // a solid -- a centreline, a reference circle for a bolt pattern.
    bool construction = false;

    // The points it is built on:
    //   Line    a -> b
    //   Circle  a = centre
    //   Arc     a = centre, b = start, c = end, counter-clockwise from b to c
    //   Bezier  a, b, c, d = control points; a and d lie on the curve
    SketchId a = kNoSketchId, b = kNoSketchId, c = kNoSketchId, d = kNoSketchId;

    Real radius = 0;   // Circle and Arc; kept in step with the points by the solver
};

// New rules go on the end, for the same reason as SketchCurve.
enum class SketchRule : uint8_t {
    Coincident,     // point, point
    Horizontal,     // line
    Vertical,       // line
    Parallel,       // line, line
    Perpendicular,  // line, line
    Tangent,        // line, arc or circle -- or two arcs or circles
    Equal,          // two lines (length) or two arcs or circles (radius)
    Concentric,     // two arcs or circles
    Fix,            // point, held at (value, value2)
    Distance,       // point, point, value
    Radius,         // arc or circle, value
    Angle,          // line, line, value in radians
};

// A rule that sizes something, as opposed to one that only relates things.
// These are the numbers a person edits.
bool isDimension(SketchRule rule);

// What to call a rule when naming it to a person: "distance", "radius".
const char* sketchRuleName(SketchRule rule);

struct SketchConstraint {
    SketchId id = kNoSketchId;
    SketchRule rule = SketchRule::Coincident;
    SketchId first = kNoSketchId, second = kNoSketchId;
    Real value = 0, value2 = 0;
};

struct Sketch {
    SketchPlane plane;
    std::vector<SketchPoint> points;
    std::vector<SketchEntity> entities;
    std::vector<SketchConstraint> constraints;

    // Ids are never reused, so a constraint or a face name that referred to
    // something deleted cannot quietly come to mean whatever was added next.
    SketchId nextId = 1;

    bool empty() const { return entities.empty(); }

    SketchId addPoint(Vec2 at);
    SketchId addLine(SketchId from, SketchId to);
    SketchId addCircle(SketchId centre, Real radius);
    // Counter-clockwise from start to end, about centre. The radius is taken
    // from the start point; the solver keeps the end on the same circle.
    SketchId addArc(SketchId centre, SketchId start, SketchId end);
    SketchId addBezier(SketchId a, SketchId b, SketchId c, SketchId d);
    SketchId constrain(SketchRule rule, SketchId first, SketchId second = kNoSketchId,
                       Real value = 0, Real value2 = 0);

    SketchPoint*            point(SketchId id);
    const SketchPoint*      point(SketchId id) const;
    SketchEntity*           entity(SketchId id);
    const SketchEntity*     entity(SketchId id) const;
    SketchConstraint*       constraint(SketchId id);
    const SketchConstraint* constraint(SketchId id) const;

    // A rectangle as four lines on four shared corners, with the lines held
    // horizontal and vertical and the corner at `min` fixed. What is left free
    // is the size; returns the ids of the width and height dimensions.
    // The shape almost every test and most parts start from.
    struct Rectangle { SketchId bottom, right, top, left, width, height; };
    Rectangle addRectangle(Vec2 min, Real width, Real height);
};

// ---- Solving ----------------------------------------------------------------

struct SketchSolve {
    // It converged and no two constraints disagree. A sketch with freedoms
    // left is still solved: it has a definite shape, just not a fully
    // determined one.
    bool solved = false;

    // How much is still free. Zero is fully constrained.
    int freedoms = 0;

    // Constraints that cannot all hold at once, and ones that restate what
    // others already say. Named, because "over-constrained" is not something
    // a person can act on and "the width and the second width disagree" is.
    std::vector<SketchId> conflicting;
    std::vector<SketchId> redundant;

    // What still has freedom left in it. A sketch is drawn in two colours from
    // this: geometry a later dimension could still move, and geometry that is
    // pinned down. Everything is free until something says otherwise, so an
    // undimensioned sketch lists all of it. Both sorted by id.
    std::vector<SketchId> freePoints;
    std::vector<SketchId> freeEntities;

    std::string reason;   // for a person, when !solved
};

// Solves a sketch in place and keeps what it built, so that solving again --
// after a dimension changes, or on every frame of a drag -- does not pay to
// describe the sketch to the solver a second time. That is what keeps a sketch
// solvable inside a frame, which is a requirement rather than a hope.
//
// The sketch must outlive the solver and must not gain or lose geometry while
// the solver exists; build a new one after a structural edit.
class SketchSolver {
public:
    explicit SketchSolver(Sketch& sketch);
    ~SketchSolver();
    SketchSolver(const SketchSolver&) = delete;
    SketchSolver& operator=(const SketchSolver&) = delete;

    SketchSolve solve();

    // Changes a dimension's value and leaves the rest of the system as it is.
    // False when the id is not a dimension of this sketch.
    bool setDimension(SketchId constraint, Real value);

    // ---- Dragging -----------------------------------------------------------
    //
    // A drag is a pull toward where the pointer is, not an instruction: the
    // constraints hold, and what is free moves as far as they allow. A point on
    // a line held horizontal slides along it; a fully constrained sketch does
    // not move at all.
    //
    // It works by adding one temporary pull on the solver's *secondary* system,
    // which planegcs minimises subject to the real constraints rather than
    // alongside them -- so a drag can never make a sketch conflict, and the
    // diagnosis a person sees is unaffected by where they happen to be pulling.
    //
    // The system is built once, at beginDrag, and every dragTo after that
    // re-solves the same system: that is what keeps a drag inside a frame.
    bool beginDrag(SketchId point);

    // The same, for how big something is: dragging the rim of a circle or an arc.
    bool beginRadiusDrag(SketchId entity);

    // True when the sketch moved. False leaves it exactly as it was, which is
    // what a fully constrained sketch does.
    bool dragTo(Vec2 at);
    bool dragRadiusTo(Real radius);

    void endDrag();
    bool dragging() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Builds a solver, solves once, and discards it: what re-evaluating a history
// needs, where the sketch is solved once per evaluation.
SketchSolve solveSketch(Sketch& sketch);

// ---- Profiles ---------------------------------------------------------------

// One closed chain of entities, in order around it.
struct SketchLoop {
    std::vector<SketchId> entities;
    std::vector<bool>     reversed;   // traversed from its end back to its start
    Real signedArea = 0;              // positive when counter-clockwise in the plane
};

// A region that can be swept: an outer loop and the loops cut out of it.
struct SketchProfile {
    // The smallest entity id on the outer loop. A feature names the profile it
    // extrudes by this, so that adding a line elsewhere in the sketch does not
    // point it at a different region the way an index would.
    SketchId key = kNoSketchId;
    SketchLoop outer;
    std::vector<SketchLoop> holes;
    Real area = 0;                    // of the region, holes taken away
};

// Every closed region the sketch's non-construction geometry bounds, ordered by
// key. Geometry that does not close -- an open chain, a branch -- is left out
// rather than guessed at. A loop drawn inside another is a hole in it, and is
// also a region of its own: the disc that fills the hole.
std::vector<SketchProfile> sketchProfiles(const Sketch& sketch);

// The keys of the regions that are filled the way SVG and every drawing program
// fill: those inside an even number of other loops. A letter O is its ring and
// not its counter; a plate with a bore is the plate and not the disc; a single
// region is itself.
std::vector<SketchId> sketchFilledProfiles(const Sketch& sketch,
                                           const std::vector<SketchProfile>& profiles);

// Points along one entity from its start to its end, exact at both: enough to
// draw it and to tell whether the pointer is on it.
// `steps` is how many pieces a curve is cut into -- fewer for one that is small
// on screen; a line is always one.
std::vector<Vec2> sketchEntityPoints(const Sketch& sketch, const SketchEntity& entity, int steps = 48);

// A loop as one closed polygon, in the order it runs, without repeating its
// first point at the end.
std::vector<Vec2> sketchLoopPoints(const Sketch& sketch, const SketchLoop& loop);

// Whether a point lies inside a region: within its outer loop and outside every
// hole in it.
bool sketchProfileContains(const Sketch& sketch, const SketchProfile& profile, Vec2 at);

} // namespace tg
