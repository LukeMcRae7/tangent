// Tangent - the B-rep backend, as seen from the seam.
//
// Body needs about thirty questions answered about a body. This header is those
// questions, asked of an opaque BrepShape, so that body.h never includes an
// OpenCASCADE header and nothing above src/geom can accidentally depend on one.
//
// There are two implementations. brep_occt.cpp answers them with OpenCASCADE;
// brep_stub.cpp answers them by refusing, and is compiled when the project is
// built without OCCT. That is not a nicety: a project may be built either way,
// and the alternative -- #ifdef inside body.h -- would give Body two different
// layouts depending on which translation unit you asked, which is the kind of
// bug that takes a week to find.
//
// The seam's rule holds here too: FaceId, EdgeId and VertexId are opaque. The
// OCCT implementation numbers them by its own indexed maps, which have nothing
// to do with the mesh backend's array indices, and no caller can tell.
#pragma once

#include "core/math.h"
#include "mesh/element_id.h"
#include "geom/op_types.h"
#include "mesh/halfedge.h"
#include "mesh/primitives.h"
#include "mesh/health.h"

#include <memory>
#include <string>
#include <vector>

namespace tg {

using FaceId   = Index;
using EdgeId   = Index;
using VertexId = Index;

// A shape and the names attached to it. Defined only in brep_occt.cpp: everyone
// else holds it through a pointer and asks the functions below.
struct BrepShape;

// Declared in sketch/sketch.h; held by reference here so this header does not
// grow a dependency on the sketch for the many files that never sweep one.
struct Sketch;
struct SketchProfile;
using SketchId = uint32_t;

using BrepRef = std::shared_ptr<const BrepShape>;

// How closely triangles have to follow the surface they stand for.
//
// The two limits do different jobs and both bind. `deviationMm` is how far a
// chord may sit from the surface, which is what a printer cares about.
// `angleRad` is how far the surface may turn between one triangle and the next,
// which is what stops a big shallow cylinder being drawn as a hexagon even
// though every chord is within tolerance. Stage 0 measured the cost: opening
// the angle from OCCT's default to 1 radian took a 206-face part from 179ms to
// 60, which is why the screen's default is coarse and an export's is not.
struct TessellationQuality {
    Real deviationMm = 0.0;      // 0: chosen from the size of the body
    Real angleRad = 0.0;         // 0: the screen's coarse default
    Real creaseAngleDeg = 35.0;  // the mesh backend's shading threshold only

    // Meshing writes its result into the shape, where it is shared with every
    // Body that copied it and reused on the next call -- which is what makes a
    // redraw cheap, and why asking for a *coarser* mesh than the one already
    // there does nothing at all.
    //
    // An export wants an answer of its own: at 0.2mm it should get 0.2mm, and
    // it must not leave the screen showing the coarse result afterwards. Set
    // this and the work happens on a copy.
    bool independent = false;
};

// What a face and an edge are made of. A mesh can only ever answer Plane and
// Line, which is honest: that is all it has.
enum class SurfaceKind { Plane, Cylinder, Cone, Sphere, Torus, Freeform };
enum class CurveKind   { Line, Circle, Ellipse, Freeform };

namespace brep {

// Was the project built with OpenCASCADE? Everything below returns an empty or
// refusing answer when this is false, so a caller can be written once.
bool available();

// ---- Lifetime --------------------------------------------------------------
// A deep copy, for the one case that needs one: mutating a body that other
// Bodies share. Returns null when OCCT is absent.
BrepRef clone(const BrepShape& s);

// A copy that shares nothing with the original.
//
// `clone` shares the underlying shape, which is almost always what is wanted:
// it is immutable and the triangulation hanging off it is a cache every copy
// would otherwise rebuild. Almost always. Meshing writes that cache *into* the
// shape, so a shape being tessellated on one thread cannot also be read by an
// algorithm on another -- and a preview built in the background is exactly
// that. This hands back a shape with its own topology to work on.
BrepRef detach(const BrepShape& s);

// ---- Enumeration -----------------------------------------------------------
bool empty(const BrepShape& s);
int  faceCount(const BrepShape& s);
int  edgeCount(const BrepShape& s);
int  vertexCount(const BrepShape& s);

bool hasFace(const BrepShape& s, FaceId f);
bool hasEdge(const BrepShape& s, EdgeId e);
bool hasVertex(const BrepShape& s, VertexId v);

void allFaces(const BrepShape& s, std::vector<FaceId>& out);
void allEdges(const BrepShape& s, std::vector<EdgeId>& out);
void allVertices(const BrepShape& s, std::vector<VertexId>& out);

// ---- Topology --------------------------------------------------------------
void faceEdges(const BrepShape& s, FaceId f, std::vector<EdgeId>& out);
void faceVertices(const BrepShape& s, FaceId f, std::vector<VertexId>& out);
int  faceDegree(const BrepShape& s, FaceId f);

void edgeEnds(const BrepShape& s, EdgeId e, VertexId& a, VertexId& b);
void edgeFaces(const BrepShape& s, EdgeId e, FaceId& a, FaceId& b);
void vertexEdges(const BrepShape& s, VertexId v, std::vector<EdgeId>& out);

// ---- Geometry --------------------------------------------------------------
// Exact, not sampled: a face's normal comes from its surface and an edge's
// direction from its curve, which is the whole point of the backend.
Vec3 faceNormal(const BrepShape& s, FaceId f);
Vec3 faceCentroid(const BrepShape& s, FaceId f);
// A point that is actually on the face, taken at the middle of its parameter
// range. The centroid of a curved face need not be on it at all -- the centre
// of mass of a whole cylinder's wall is on its axis -- so anything that has to
// stand something *on* a face wants this rather than the centroid.
Vec3 facePoint(const BrepShape& s, FaceId f);
Real faceArea(const BrepShape& s, FaceId f);
AABB faceBounds(const BrepShape& s, FaceId f);
Vec3 vertexPosition(const BrepShape& s, VertexId v);
AABB bounds(const BrepShape& s);
void edgePositions(const BrepShape& s, EdgeId e, Vec3& a, Vec3& b);
Vec3 edgeDirection(const BrepShape& s, EdgeId e);

// ---- What a thing actually is ----------------------------------------------
//
// A mesh can only answer "a polygon" and "a straight line", so nothing above
// the seam has ever been able to ask. These are what a hole's diameter, a
// snap to an arc's centre, and an angle between two faces are made of, and
// they are the reason those become exact rather than sampled.
SurfaceKind faceKind(const BrepShape& s, FaceId f);
CurveKind   edgeKind(const BrepShape& s, EdgeId e);

// True when the edge is a circle or an arc of one, filling in where it is.
// `radius` is the circle's, not the arc's chord.
bool edgeCircle(const BrepShape& s, EdgeId e, Vec3& centre, Vec3& axis, Real& radius);

// True when the face is a cylinder, filling in its axis and radius -- which is
// how a bore reports its diameter rather than the width of a facet.
bool faceCylinder(const BrepShape& s, FaceId f, Vec3& point, Vec3& axis, Real& radius);

// Along the curve, not across the chord: a semicircular edge of radius 10 is
// 31.4mm long and its midpoint is out on the arc, not inside the body.
Real edgeLength(const BrepShape& s, EdgeId e);
Vec3 edgeMidpoint(const BrepShape& s, EdgeId e);

// The edge as a chain of points along the curve, close enough that no chord
// sits further than `deviationMm` from it. Pass 0 to let the backend choose.
//
// Anything that *draws* an edge needs this rather than the two ends: the line
// between a rim's ends runs across the hole instead of around it.
void edgePolyline(const BrepShape& s, EdgeId e, Real deviationMm,
                  std::vector<Vec3>& out);

// ---- Names -----------------------------------------------------------------
ElementId faceName(const BrepShape& s, FaceId f);
ElementId edgeName(const BrepShape& s, EdgeId e);
ElementId vertexName(const BrepShape& s, VertexId v);

FaceId   findFace(const BrepShape& s, ElementId id);
EdgeId   findEdge(const BrepShape& s, ElementId id);
VertexId findVertex(const BrepShape& s, ElementId id);

// ---- Construction ----------------------------------------------------------
// A primitive as an exact solid. Returns null for a kind this backend cannot
// build, and for every kind when OCCT is absent.
//
// Faces are named by the part they play -- the top cap, the wall -- using the
// same formula as the mesh generators in src/mesh/primitives.cpp, so a box has
// the same six face names whichever backend built it. That is what would let a
// chain move between backends without its references going stale.
BrepRef primitive(const PrimitiveSpec& spec);

// ---- Modelling -------------------------------------------------------------
// Both carry names across the operation by asking it what became of what --
// Modified, Generated, IsDeleted -- rather than by matching geometry
// afterwards. `salt` identifies the feature, so re-running the same chain names
// the same things; see element_id.h.
//
// Both return null on refusal and put a short phrase in `reason`. Neither
// touches its input: a refused edit cannot half-apply.
BrepRef booleanOp(const BrepShape& a, const BrepShape& b, BooleanOp op,
                  ElementId salt, std::string* reason);

// One radius per edge, parallel to `edges`. Rounding two edges to different
// radii in one operation is a different solid from rounding them in sequence,
// which is why they go together.
//
// `endRadii`, when given, runs alongside `radii` and makes each round taper
// from one to the other along its edge. `chamfer` cuts a flat instead, at the
// same distance -- the same edges and the same number, a different surface.
BrepRef filletEdges(const BrepShape& s, const std::vector<EdgeId>& edges,
                    const std::vector<Real>& radii, ElementId salt, std::string* reason,
                    const std::vector<Real>* endRadii = nullptr, bool chamfer = false);

// Pushes faces along their own normals and joins the result to the body, or
// cuts it out when the distance is negative. Built as a prism and combined
// through the same boolean path, so the names come out of the same mechanism
// rather than a second one written for the occasion.
//
// `newFaces` reports the name of each moved face's new position, in the order
// the faces were given.
//
// `mergeFlush` decides which of the two operations this is. A prism swept off a
// face has walls flush with the walls it slides along -- the same plane, in two
// pieces. Merging them moves the face and lets the body absorb it, which is
// push and pull. Leaving them draws the boss's own boundary, which is what
// makes it a thing that can be selected and edited afterwards, and that is
// extrude.
//
// `intersect` keeps only what the body and the sweep share instead: pushed into
// the body, the slab under the face.
BrepRef extrudeFaces(const BrepRef& s, const std::vector<FaceId>& faces, Real distance,
                     ElementId salt, std::vector<ElementId>* newFaces, std::string* reason,
                     bool mergeFlush = true, Vec3 along = Vec3{}, bool intersect = false);

// The solid the faces sweep through, on its own, named as extrudeFaces names
// its prisms. What an extrusion does to bodies other than the one it came from
// is done with this.
BrepRef sweptFaces(const BrepRef& s, const std::vector<FaceId>& faces, Real distance, Vec3 along,
                   ElementId salt, std::string* reason);

// Whether two shapes touch or overlap, to within `tol`.
bool touches(const BrepShape& a, const BrepShape& b, Real tol);

// Splits each face into an inner face and the ring around it, the inner one
// offset inward by `amount`. What a pocket or a boss is drawn from.
//
// Only a flat face can be inset: on a curved one the offset is not a wire in
// the same surface and the answer would be an approximation. Refused with a
// reason rather than approximated.
// Tilts faces about one of their own edges.
//
// The direct-modelling answer to "rotate this face": the face turns and the
// ones around it stretch to follow, which is a different thing from rotating a
// body -- the solid stays one solid and only the chosen face moves. OpenCASCADE
// spells it as a draft angle, and a draft is exactly a face rotated about the
// line where it meets a neutral plane.
//
// `hingePoint` and `hingeDir` name that line; it should be an edge of the face,
// or the operation has nothing to pivot on.
// Tips faces away from a pull direction: the draft a moulded part needs to come
// out of its tool, and the one a printed part needs to stand up without a wall
// leaning out over nothing.
//
// Every face named turns about the line where it meets the neutral plane --
// the plane through `neutralPoint` square to `pull` -- so the part is widest
// where it meets that plane and narrows along the pull. A positive angle
// narrows; a negative one widens.
//
// Faces square to the pull cannot be drafted -- there is no line to turn about
// -- and are refused rather than quietly left alone.
BrepRef draftFaces(const BrepRef& s, const std::vector<FaceId>& faces, Real angleRad,
                   Vec3 neutralPoint, Vec3 pull, ElementId salt, std::string* reason);

BrepRef rotateFaces(const BrepRef& s, const std::vector<FaceId>& faces, Real angleRad,
                    Vec3 hingePoint, Vec3 hingeDir, ElementId salt, std::string* reason);

// Drops every division that does not define the shape.
//
// Two coplanar faces sharing an edge are one face with a line drawn on it, and
// the line may be there for a reason -- a divide puts one there on purpose --
// or may be left over from an operation that had no way to avoid it. This
// removes all of them, which is a thing worth being able to ask for outright
// and not a thing any other operation should do behind your back.
BrepRef mergeDivisions(const BrepRef& s, ElementId salt, std::string* reason);

// Grows or shrinks a face within its own plane, the faces around it stretching
// to follow.
//
// A plane scaled about a point in itself is the same plane, so there is nothing
// to do to the face: what moves is its boundary, and what moves that is every
// face meeting it. Each of those tips about the edge furthest from the scaled
// face, by the angle that carries their shared edge the right distance -- which
// is the same draft the rotation uses, one per neighbour, in one operation.
//
// `factor` is a multiple: 1 is unchanged, 1.5 is half again as large, 0.5 is
// half. Proportional, so a boundary twice as far from the middle moves twice as
// far, which is what scaling means and is not the same as offsetting.
BrepRef scaleFaces(const BrepRef& s, const std::vector<FaceId>& faces, Real factor,
                   ElementId salt, std::string* reason);

// Cuts a line across the body where a plane crosses it, without cutting the
// body in two.
//
// Blender calls it a loop cut and the reason to want one is the same here as
// there: a face is one face until something divides it, and you cannot push
// half of a face. The solid comes back whole, with the faces the plane crossed
// each split along it.
BrepRef divideBody(const BrepRef& s, Vec3 planePoint, Vec3 planeNormal,
                   ElementId salt, std::string* reason);

// Takes faces off the body and closes the gap: the boss goes and the face it
// stood on grows back over it, the hole fills in, the fillet becomes the corner
// it rounded. This is what makes a STEP file somebody else made editable --
// there is no history to delete a step from, only the shape.
//
// Refused, with a reason, when the faces around the gap cannot be grown to
// meet: not everything that can be pointed at can be taken away.
BrepRef removeFaces(const BrepRef& s, const std::vector<FaceId>& faces, ElementId salt,
                    std::string* reason);

BrepRef insetFaces(const BrepRef& s, const std::vector<FaceId>& faces, Real amount,
                   ElementId salt, std::vector<ElementId>* newFaces, std::string* reason);

// Hollows the solid out, leaving a wall `thickness` thick, and opens the faces
// named: a box shelled with its top open is a tray, and with nothing open it is
// a sealed void that a printer will want a drain hole in.
//
// `thickness` is the wall and is measured inward, so it must be positive.
// Refused with a reason when the wall does not fit -- two sides that would meet
// in the middle, a thickness wider than the part -- rather than handing back a
// solid that self-intersects.
BrepRef shell(const BrepRef& s, const std::vector<FaceId>& openFaces, Real thickness,
              ElementId salt, std::string* reason);

// Grows or shrinks the whole body by moving every face along its own normal:
// a part made 0.2 mm bigger all round to fit the pocket it has to sit in, or
// a copy of a shape shrunk to be the void inside something else.
//
// The corners stay corners -- the faces are extended to meet, not rounded off
// -- so a 20 mm cube offset by 1 is a 22 mm cube and not a cube with a 1 mm
// round on every edge. Refused, with a reason, when shrinking takes the body
// past nothing.
BrepRef offsetBody(const BrepRef& s, Real distance, ElementId salt, std::string* reason);

// Cuts a thread on a round face: the helical groove a tap or a die would leave,
// taken out of the body.
//
// Not a cosmetic thread. On a printed part the helix has to actually be there,
// because there is nothing to cut it with afterwards. `pitch` is the rise per
// turn and `height` how far the groove reaches into the material; `external`
// says which kind is being asked for, and a face that is the other kind is
// refused rather than threaded inside out.
BrepRef threadFace(const BrepRef& s, FaceId face, Real pitch, Real height, bool external,
                   ElementId salt, std::string* reason);

// Drills a hole into the solid: a cylinder from `at`, going `into` the
// material, with a counterbore or a countersink at its mouth when the cut asks
// for one. `at` and `into` are in the body's own space.
//
// Through means through: the tool is made long enough to leave the body on
// both sides, so a hole across a curved wall does not stop half way. A blind
// hole ends in the drill's own cone unless the cut says otherwise -- a flat
// ceiling over a hole is the one overhang a slicer cannot help with.
//
// The wall is named for the hole, so a fillet or a chamfer put on its rim
// stays on it when the hole moves or changes size. Refused, with a reason,
// when the tool takes nothing away: a hole that misses the material is a
// mistake to be told about, not a step that quietly does nothing.
BrepRef drillHole(const BrepRef& s, Vec3 at, Vec3 into, const HoleCut& cut, ElementId salt,
                  std::string* reason);

// A solid swept from one region of a sketch, between two offsets along the
// sketch plane's normal.
//
// Built from the sketch's own curves rather than from points and sagittas: a
// circle is one circle and an arc has its true centre, so a bore swept from a
// sketch is one cylindrical face rather than four. A hole in the region is a
// hole in the solid.
//
// Each wall is named for the sketch entity it was swept from, and the two caps
// for which end they are. Changing a dimension moves faces without renaming
// them, which is the whole reason for a sketch being in the history -- a fillet
// on the edge a line swept stays on that edge when the line gets longer.
BrepRef sketchSolid(const Sketch& sketch, const SketchProfile& profile, Real from, Real to,
                    ElementId salt, std::string* reason);

// Every region of `profiles` whose key is in `keys`, swept at once: what
// extruding a whole imported drawing needs, where one region at a time would
// be hundreds of booleans. Regions picked inside one another -- a letter and
// the disc in its counter -- are merged into one face before sweeping, so the
// solids that come out never touch and stand together as one compound. One
// region comes out exactly as sketchSolid makes it, names and all; region k
// of several has its caps named 2k and 2k + 1.
BrepRef sketchSolids(const Sketch& sketch, const std::vector<SketchProfile>& profiles,
                     const std::vector<SketchId>& keys, Real from, Real to, ElementId salt,
                     std::string* reason);

// The same regions turned about an axis instead of pushed along the normal:
// a revolve. The axis is a point and a direction in the sketch's own
// coordinates -- what a user picks by pointing at a line in the drawing -- and
// `angle` is in radians, up to a full turn.
//
// A profile that crosses the axis is refused: it would turn through itself.
// Touching the axis is not crossing it, since that is how a half-disc becomes
// a sphere. Faces are named as a sweep names them: the ends of a part turn are
// its caps, and every other face is named for the sketch entity that swept it.
BrepRef revolveSketch(const Sketch& sketch, const std::vector<SketchProfile>& profiles,
                      const std::vector<SketchId>& keys, Vec2 axisAt, Vec2 axisDir, Real angle,
                      ElementId salt, std::string* reason);

// A solid from a closed outline on a plane, swept between two heights along the
// plane's normal. The create tool's profiles arrive this way.
//
// `arcs` is optional and parallel to `points`: a non-zero entry k means the
// span from point k to point k+1 is a circular arc bulging by that sagitta,
// rather than a straight line. That is what keeps a rounded corner an actual
// arc instead of the polyline the mesh backend had to settle for.
BrepRef prism(const std::vector<Vec3>& points, const std::vector<Real>& arcs,
              Vec3 planeNormal, Real z0, Real z1, ElementId salt, std::string* reason);

// ---- Persistence -----------------------------------------------------------
// The shape in OpenCASCADE's own text form, and the face names beside it --
// which are ours, not OCCT's business, and are what makes the file a parametric
// model rather than a lump of geometry.
bool encode(const BrepShape& s, std::string& shapeOut, std::vector<ElementId>& namesOut);
BrepRef decode(const std::string& shapeText, const std::vector<ElementId>& names);

// Every face answering to this name. A name stands for a set: a boolean
// routinely splits one face into several, and a feature that referred to the
// face has to go on referring to all of it.
void findFaces(const BrepShape& s, ElementId id, std::vector<FaceId>& out);

// ---- Display and validity --------------------------------------------------
// Not free, unlike the mesh backend -- see the Stage 0 measurements -- so
// callers should cache the result and re-tessellate on a geometry change or a
// large zoom change, not per frame.
void tessellate(const BrepShape& s, RenderMesh& out, TessellationQuality q);
// Every edge bounds exactly two faces. Linear in the edges, and the question
// "is this closed" rather than "is this valid" -- see health().
bool closedShell(const BrepShape& s);

bool validate(const BrepShape& s, std::string* err);
MeshHealth health(const BrepShape& s, bool checkIntersections);

// ---- Mutation --------------------------------------------------------------
// Returns a new shape; the input is untouched, as everywhere else in the
// kernel, so a refused edit cannot half-apply.
//
// Any matrix that keeps the solid a solid: a placement stays exact, surfaces
// and all; a scale that differs between axes is honoured too, at the cost of
// the surfaces it stretches becoming general ones. A reflection is refused --
// see mirrored() -- and so is anything that flattens the shape.
BrepRef transformed(const BrepShape& s, const Mat4& m);

// Reflects across the plane through `point` with `normal`. Separate from
// transformed() because a reflection is not a rigid placement: it turns the
// shape inside out, and every face's orientation has to be flipped back or the
// result is a solid describing the void around itself. OCCT knows how to do
// that from a mirror transform; it cannot know it from a matrix.
BrepRef mirrored(const BrepShape& s, Vec3 point, Vec3 normal);

// Each solid of a shape that holds more than one, as its own shape, keeping the
// names its faces had. Largest first, so a caller keeping one piece in place
// keeps the main one. A shape with a single solid yields that solid.
size_t separateSolids(const BrepShape& s, std::vector<BrepRef>& out);

// Cuts a body in two along a plane, in one pass: the plane goes in as a single
// face and every solid that comes out is sorted by which side its centre is on.
// `above` is the side `normal` points to. Either side may hold several solids.
// False, with `reason`, when the plane does not cut through.
bool splitByPlane(const BrepShape& s, Vec3 point, Vec3 normal, ElementId salt,
                  BrepRef& above, BrepRef& below, std::string* reason);

// Cuts pins and sockets across the face two pieces were split on.
//
// `above` and `below` are what splitByPlane made, and the plane is the one it
// cut on. The pins are spread along the longest way across the cut face, each
// one somewhere the material is on both sides of the cut, so a shape whose
// section is a ring or two islands does not get a pin in the air.
//
// The socket is cut `clearance` wider than the pin: a printed hole comes out
// undersize and a printed pin oversize, and a pin that has to be forced is a
// pin that splits the part it is going into.
//
// False, with a reason, when there is no room for one: a cut face thinner than
// the pin, or nowhere on it that has material behind it on both sides.
bool pinAcross(BrepRef& above, BrepRef& below, Vec3 point, Vec3 normal, const SplitPins& pins,
               ElementId salt, std::string* reason);

// ---------------------------------------------------------------------------
// STEP.
//
// The one format in this program that carries the model rather than a picture
// of it. An STL is triangles and loses everything that made the body exact; a
// STEP file holds the surfaces themselves, so a cylinder that goes out comes
// back a cylinder and not a sixty-four-sided prism.
//
// What it does not carry is the feature chain. A body written here and read
// back is a body, not a history -- which is why this is import and export
// rather than save and open.

// Writes `shapes` as one STEP file. Units are millimetres, matching the rest
// of the program. Returns false and sets `reason` on failure.
bool writeStep(const std::vector<const BrepShape*>& shapes, const std::string& path,
               std::string* reason);

// Reads every solid in a STEP file. Each becomes one BrepRef with freshly
// minted names -- the file has no idea what Tangent called anything, and
// pretending otherwise would attach operations to the wrong faces.
//
// `salt` distinguishes one import from another so two imports of the same file
// do not name their faces identically.
bool readStep(const std::string& path, ElementId salt, std::vector<BrepRef>& out,
              std::string* reason);

// Sews a triangle soup into a solid, then merges the coplanar faces.
//
// `tris` is three indices into `points` per triangle, wound outward. The merge
// is the point: twelve triangles that were a box come back as six faces, so a
// part that was CAD before somebody exported it to STL gets its flat surfaces
// returned. Curvature does not come back -- the file threw that away -- so a
// cylinder returns as however many narrow planar strips it left as.
// `edgeEnds` is two point indices per shared edge, and `triEdges` three edge
// indices per triangle, in the same order as that triangle's own corners.
//
// Passing the connectivity rather than letting the kernel find it is not a
// micro-optimisation. BRepBuilderAPI_Sewing works out which faces touch by
// searching on position, and on a 62,000-triangle mesh that search took
// thirty-seven seconds -- to rediscover something the half-edge mesh had known
// exactly since it was built. Given the edges, each one is made once and shared
// by the two faces that meet along it, and there is nothing to search for.
// A closed mesh already cut into the faces it will become.
//
// Each region is a connected patch of coplanar triangles, described by its
// boundary loops -- one outer, any number of holes -- in terms of edges shared
// with the regions around it. Handing the kernel this rather than the
// triangles is the difference between it building seven thousand faces and it
// building thirty thousand and then being asked to merge them back: the merge
// was most of the time, and the half-edge mesh can do it in one linear pass
// because it already knows which triangles are neighbours.
struct PlanarRegions {
    std::vector<Vec3>     points;
    std::vector<uint32_t> edgeEnds;          // 2 point indices per shared edge

    struct Loop {
        std::vector<uint32_t> from;           // the point each step starts at
        std::vector<uint32_t> edges;          // the edge each step runs along
    };
    struct Region {
        Vec3 normal;                          // outward
        Vec3 point;                           // any point on the plane
        std::vector<Loop> loops;              // wound counter-clockwise about
                                              // `normal` for the outside, and
                                              // clockwise for each hole
    };
    std::vector<Region> regions;

    // What the mesh enclosed. The solid is checked against it rather than
    // put through the kernel's general validity check -- see the conversion.
    Real volume = 0.0;
};

BrepRef solidFromPlanarRegions(const PlanarRegions& in, ElementId salt, std::string* reason);

BrepRef solidFromTriangles(const std::vector<Vec3>& points,
                           const std::vector<uint32_t>& tris,
                           const std::vector<uint32_t>& edgeEnds,
                           const std::vector<uint32_t>& triEdges,
                           ElementId salt, std::string* reason);

} // namespace brep
} // namespace tg
