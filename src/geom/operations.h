// Tangent - modelling operations, over a Body rather than a mesh.
//
// Every operation the feature history can run appears here exactly once, taking
// and returning Body. Modelling is the exact kernel's job: a mesh body -- an
// imported STL, or a primitive in a build without the kernel -- is refused by
// every operation that edits part of a shape, with a reason that says to
// convert it first. What a mesh can still have done to it is whole-object: be
// placed, measured, reduced, separated into pieces, exported.
//
// Two things are deliberate. Operations take handles, never raw indices dressed
// up as handles -- see the note in body.h. And every one that can refuse takes a
// `reason` out-parameter, because "it did not work" is not something an
// interface can put in front of a person.
#pragma once

#include "geom/body.h"
#include "geom/op_types.h"
#include "mesh/primitives.h"

#include <string>
#include <vector>

namespace tg {

// Which kernel a body is made of. Chosen per body rather than per project: a
// mesh imported from an STL and a parametric bracket can sit in the same scene,
// and the operations below say plainly when one of them cannot do something.
enum class Backend { Mesh, Brep };

// ---- Construction ---------------------------------------------------------
// Returns false, leaving `out` untouched, for degenerate parameters -- and for
// a Brep body when the project was built without OpenCASCADE, or when the kind
// has no exact form (a Plane is a surface, not a solid). Refusing is deliberate:
// quietly handing back a mesh body would put a body in the scene that cannot do
// what the caller asked for.
bool makePrimitive(const PrimitiveSpec& spec, Body& out, Backend backend = Backend::Mesh);

// ---- Modelling ------------------------------------------------------------
// Pushes faces along the region's area-weighted normal. `newFaces` reports
// where the moved faces ended up, in the order they were given.
//
// `mergeFlush` is the difference between push-pull and extrude: see
// brep::extrudeFaces.
//
// `along` is the direction the sweep runs, or a zero vector for each face's own
// normal. Moving a face along a world axis rather than its normal is a thing
// people want, and the only difference underneath is the vector swept.
bool extrudeFaces(Body& body, const std::vector<FaceId>& faces, Real distance,
                  std::vector<FaceId>* newFaces = nullptr, ElementId salt = 0,
                  ExtrudeOp op = ExtrudeOp::Auto, std::string* reason = nullptr,
                  bool mergeFlush = true, Vec3 along = Vec3{});

// A solid from a closed profile on a plane, swept between two heights along the
// plane's normal -- what the create tool draws.
//
// `arcs` is parallel to `points`: a non-zero entry means that span is a
// circular arc of that sagitta rather than a straight line, which is how a
// rounded corner stays an arc instead of becoming the polyline a mesh has to
// settle for.
bool makeProfileSolid(const std::vector<Vec3>& points, const std::vector<Real>& arcs,
                      Vec3 planeNormal, Real z0, Real z1, Body& out,
                      ElementId salt = 0, std::string* reason = nullptr);

// Tilts faces about one of their own edges, the solid staying one solid.
bool rotateFaces(Body& body, const std::vector<FaceId>& faces, Real angleRad,
                 Vec3 hingePoint, Vec3 hingeDir, ElementId salt = 0,
                 std::string* reason = nullptr);

// Grows or shrinks a face within its own plane; the faces around it follow.
bool scaleFaces(Body& body, const std::vector<FaceId>& faces, Real factor,
                ElementId salt = 0, std::string* reason = nullptr);

// Drops every division that does not define the shape.
bool mergeDivisions(Body& body, ElementId salt = 0, std::string* reason = nullptr);

// Adds a line across the body where a plane crosses it, splitting the faces it
// passes through without splitting the body.
bool divideBody(Body& body, Vec3 planePoint, Vec3 planeNormal,
                ElementId salt = 0, std::string* reason = nullptr);

bool insetFaces(Body& body, const std::vector<FaceId>& faces, Real amount,
                std::vector<FaceId>* newFaces = nullptr, ElementId salt = 0,
                std::string* reason = nullptr);

// Hollows the body out to a wall `thickness` thick, opening the faces given: a
// box shelled with its top face open is a tray. The single most asked-for
// operation for printing, because it is what turns a solid model into one that
// does not cost a spool of filament.
bool shellBody(Body& body, const std::vector<FaceId>& openFaces, Real thickness,
               ElementId salt = 0, std::string* reason = nullptr);

// One edge of a fillet, with its own radius. Fusion attaches a radius per edge
// within a single fillet feature rather than one radius for the whole
// selection, and so do we: rounding two edges to different radii in one go is
// a different solid from rounding them in sequence, because the corner where
// they meet is blended once instead of twice.
struct FilletEdge {
    EdgeId edge   = kInvalid;
    Real   radius = 1.0;

    // Where the radius ends up at the far end of the edge, for a round that
    // tapers along its length. Negative means it does not taper and `radius`
    // holds all the way.
    Real   endRadius = -1.0;
};

struct FilletSpec {
    std::vector<FilletEdge> edges;

    // Identifies the operation when naming what it creates, so that two
    // fillets in a chain do not hand their new faces the same names. A
    // feature passes its own identity here; see element_id.h.
    ElementId salt = 0;

    // A flat cut rather than a round. The same edges, the same distance, a
    // different surface -- and a different thing to want: a chamfer on a
    // bottom edge fights elephant's foot, and one around a hole lets a screw
    // head sit down into it.
    bool chamfer = false;
};

// Rounds edges. `reason` gets a short phrase on refusal; see the note above.
bool filletEdges(Body& body, const FilletSpec& spec, std::string* reason = nullptr);

// How a pattern lays its copies out.
enum class PatternMode : uint32_t {
    Linear,     // `step` mm along `dir`, `count` times
    Circular,   // `stepAngle` radians about the axis, `count` times
    Mirror,     // one reflection across the plane through `origin` with `dir`
};

const char* patternModeName(PatternMode m);

// Repeating something, either a tool or the body itself.
//
// With a tool, this is a boolean applied `count` times at `count` placements:
// cut one hole and pattern it into a bolt circle, or raise one boss and pattern
// it into a row. The first placement is the identity, so the original is the
// pattern's own first copy rather than something the pattern sits on top of.
//
// Without a tool, the body is combined with moved copies of itself -- which is
// what mirror usually means: model the half that is interesting, reflect it,
// and let the two halves fuse where they meet.
struct PatternSpec {
    PatternMode mode = PatternMode::Linear;
    int   count = 2;            // including the original; 2 for a mirror
    Vec3  origin{0, 0, 0};      // a point on the axis, or on the mirror plane
    Vec3  dir{1, 0, 0};         // the direction, the axis, or the plane normal
    Real  step = 10.0;          // mm between copies, Linear
    Real  stepAngle = 0.0;      // radians between copies, Circular

    // How each copy joins what is already there. Union adds, Difference takes
    // away, and which one is wanted is a property of what is being repeated,
    // not of the layout.
    BooleanOp op = BooleanOp::Union;
};

// Applies `spec` to `body`, repeating `tool` if it is not empty and the body
// itself if it is. All or nothing: on a refusal `body` is untouched.
bool patternBody(Body& body, const Body& tool, const PatternSpec& spec,
                 ElementId salt, std::string* reason);

// The placement of the i'th copy, exposed so the editor can draw where the
// copies are going to land before committing to building them.
Mat4 patternPlacement(const PatternSpec& spec, int i);

// Reduces a mesh body to fewer triangles within a tolerance. See
// mesh/decimate.h for what is and is not promised. Mesh bodies only: an exact
// body has no triangles to reduce.
struct ReduceOptions;
struct ReduceResult;
bool reduceBody(Body& body, const ReduceOptions& options, ElementId salt, ReduceResult& result);

// Both bodies must be exact. `trustBNames` is kept for callers that built the
// second operand from the first's own faces; the exact kernel names its output
// from the operation's history and does not need it.
bool booleanOp(const Body& a, const Body& b, BooleanOp op, Body& out,
               ElementId salt = 0, bool trustBNames = false,
               std::string* reason = nullptr);

// ---- Analysis -------------------------------------------------------------
// Splits into connected bodies, largest first. A body that is already one piece
// yields itself, so a caller can always use the result. Whole-object, so a mesh
// can have it done too: an STL often holds several parts.
size_t splitBodies(const Body& body, std::vector<Body>& out);

// Cuts an exact body in two with a plane. Either side may hold more than one
// solid, where the plane passes through the body more than once. Fails, leaving
// `a` and `b` unspecified, when the plane misses the body or only grazes it --
// and for a mesh, which has to be converted first.
bool splitByPlane(const Body& body, Vec3 planePoint, Vec3 planeNormal,
                  Body& a, Body& b);

} // namespace tg
