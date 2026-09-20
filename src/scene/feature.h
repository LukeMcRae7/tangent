// Tangent - parametric feature history.
//
// An object's mesh is not authored, it is *evaluated*: a base primitive
// followed by an ordered list of operations. Editing any parameter re-runs the
// chain, which is what makes the model parametric rather than a frozen result.
//
// Operations name what they act on by ElementId rather than by index -- see
// element_id.h and ElementRefs below. A name is derived from what made the
// element and carries across an edit: the kernel is asked what each face became
// and the names follow, so editing a dimension near the root re-applies
// everything after it against the right geometry.
//
// What that does not yet buy is inserting or reordering a step. The names
// would survive it; nothing offers to do it. When that arrives it is the real
// test of this machinery, because a feature inserted at step four hands every
// later step geometry it has never seen.
//
// Where a name does not resolve, the answer is honest failure: the feature is
// marked errored and skipped, the chain continues, and the timeline says which
// step gave up rather than silently producing wrong geometry.
#pragma once

#include "geom/body.h"
#include "geom/operations.h"
#include "sketch/sketch.h"

#include <string>
#include <vector>

namespace tg {

enum class FeatureKind {
    Primitive,   // chain root: a parametric shape
    BaseMesh,    // chain root: geometry that has no parameters (a split body)
    Extrude,
    Inset,
    Bevel,
    VertexEdit,
    Boolean,     // combine with a baked copy of another body
    Shell,       // hollow it out, opening the faces named
    FaceRotate,  // tip a face about one of its own edges
    Divide,      // cut a line across the body without cutting it in two
    Merge,       // drop every division that does not define the shape
    FaceScale,   // grow or shrink a face in its own plane
    Pattern,     // repeat a tool, or the body, in a row, around an axis, or mirrored
    Reduce,      // fewer triangles for a mesh, within a tolerance
    Sketch,      // constrained 2D geometry on a plane; changes no body itself
    ExtrudeProfile, // a region of a sketch swept into a solid
    Move,        // puts the object somewhere else; its shape is untouched
    Rotate,      // turns the object about a point; its shape is untouched
    Scale,       // stretches the body along its own axes: a change of shape

    // New kinds go on the end and nowhere else. The value is what is written
    // to a file, so inserting one in the middle renumbers every kind after it
    // and quietly turns a divide in an old project into something else.
};

const char* featureKindName(FeatureKind k);

// What a feature acts on, expressed so that it still means the same thing after
// the steps before it change.
//
// `Explicit` is a list of stable names -- what a user picking individual faces
// or edges produces. It resolves exactly, and refuses rather than guesses when
// an element it named no longer exists.
//
// `FaceBoundary` is every edge around one face. It exists because a list of
// names cannot express "the rim": raise a cylinder from sixteen segments to
// twenty-four and the sixteen edges the user clicked are genuinely gone, but
// the cap they surrounded is still there and still has a rim. Naming the cap
// and taking its boundary re-selects the whole rim at its new segment count,
// which is what the user meant.
struct ElementRefs {
    enum class Kind : uint32_t { Explicit, FaceBoundary, All };

    Kind kind = Kind::Explicit;
    std::vector<ElementId> ids;    // Explicit
    ElementId face = kNoId;        // FaceBoundary

    // How many elements are named outright. A rim or a whole body is not a
    // count, so callers wanting to show something use describe().
    size_t count() const { return kind == Kind::Explicit ? ids.size() : 0; }

    // Short phrase for the timeline: "3 edges", "a face's rim", "every edge".
    std::string describe(const char* noun) const;

    bool empty() const {
        return kind == Kind::Explicit ? ids.empty()
             : kind == Kind::FaceBoundary ? face == kNoId
             : false;
    }

    // Resolves to handles into `body`. Returns false if anything named
    // here has gone; a feature that cannot find what it acts on is errored, not
    // quietly re-pointed at whatever now sits at those numbers.
    bool resolveEdges(const Body& body, std::vector<EdgeId>& out) const;

    // Resolves to face indices.
    bool resolveFaces(const Body& body, std::vector<FaceId>& out) const;
};

// Records a selection made on `body` as something that will still mean the
// same thing later.
ElementRefs nameFaces(const Body& body, const std::vector<FaceId>& faces);

// As above, and additionally: if the chosen edges are exactly the boundary of
// one face, that is recorded instead of the list. It is what the user meant --
// they picked a rim, not sixteen edges that happen to be there today -- and it
// is the only form that survives the face being retessellated under them.
// `allowBoundary` lets a set of edges that happens to be exactly some face's
// rim be stored as that rim instead, which survives an edit that renumbers or
// re-splits the face. It costs the order: a rim resolves in whatever order the
// body lists it, and a caller pairing a parallel array -- a radius per edge --
// then has its radii on other edges. Such a caller passes false, or passes
// true only when every entry is the same and the pairing cannot matter.
ElementRefs nameEdges(const Body& body, const std::vector<EdgeId>& edges,
                      bool allowBoundary = true);

std::vector<ElementId> nameVertices(const Body& body, const std::vector<VertexId>& verts);

struct Feature {
    FeatureKind kind = FeatureKind::Primitive;
    bool        enabled = true;

    // Identifies this feature for as long as it exists, independent of where it
    // sits in the chain. It salts the names of everything the feature creates,
    // so moving a feature does not rename its output and two features of the
    // same kind do not collide. Assigned when the feature is created.
    ElementId uid = 0;

    // Primitive: the base shape the chain starts from, and which kernel builds
    // it. The backend belongs to the feature that creates the body rather than
    // to the object, because it is a property of the geometry: a chain that
    // starts exact stays exact, and one that starts from an imported mesh
    // cannot become exact by wishing.
    PrimitiveSpec primitive;
    Backend       backend = Backend::Mesh;

    // Extrude / Inset: which faces, by name as of this point in the chain.
    ElementRefs faces;
    Real distance = 5.0;    // Extrude, signed
    ExtrudeOp extrudeOp = ExtrudeOp::Auto;

    // Whether the walls the sweep left flush with the walls it slid along are
    // merged into them. True is push and pull -- the face moves and the body
    // absorbs it. False is extrude -- the boss keeps its own outline, which is
    // what makes it something you can point at afterwards.
    bool mergeFlush = true;
    Real amount   = 2.0;    // Inset

    // FaceRotate: how far to tip, and the edge to tip about. Divide: a point on
    // the cutting plane, and its normal.
    //
    // Held in the body's own space as plain geometry rather than as a named
    // edge, which means an edit further up the chain that moves that edge does
    // not carry the hinge with it. Names would be better and are what the rest
    // of the chain uses; this is the honest version of what is built, not a
    // claim that it follows.
    Real angle = 0.0;                    // radians
    Real scale = 1.0;                    // FaceScale: a multiple, 1 is unchanged
    Vec3 axisPoint{0, 0, 0};
    Vec3 axisDir{0, 0, 1};

    // Shell: the wall left behind, measured inward. `faces` holds the faces to
    // open, and may be empty -- that is a sealed cavity, which is a thing
    // someone may want and a thing a printer will want a drain hole in.
    Real thickness = 2.0;

    // Bevel / fillet. `edges` empty means every edge of the body; otherwise
    // just those, named by half-edge as numbered at this point in the chain.
    //
    // `radii` is parallel to `edges` and gives each one its own radius, as a
    // Fusion fillet does. Left empty, `width` applies to all of them -- which
    // is also what a file written before per-edge radii existed will load as.
    //
    // One feature holding many edges is not a convenience. Filleting two edges
    // that meet in a single operation blends their shared corner once, against
    // the original faces; filleting them one after another asks the second to
    // cut into the first one's surface, which is a harder problem and one we
    // currently refuse. So the editor extends this list rather than appending
    // another fillet whenever it can.
    ElementRefs edges;
    std::vector<Real>  radii;
    Real width    = 1.0;

    // A flat cut rather than a round, at the same distance.
    bool chamfer = false;

    // Where the radius ends up at the far end of each edge, for a round that
    // tapers. Negative means it does not.
    Real endWidth = -1.0;

    int  segments = 1;

    // Radius for the i'th resolved edge, falling back to the feature-wide
    // width. Only an explicit selection carries per-edge radii; a rim selected
    // as a face's boundary is one radius by construction.
    Real radiusFor(size_t i) const {
        return i < radii.size() && radii[i] > 0.0 ? radii[i] : width;
    }

    // Boolean: how to combine, and the other body baked into this object's
    // local space. Baked rather than referenced because a live reference would
    // need the other object to stay alive and re-evaluate first -- a dependency
    // graph rather than a list, which is a bigger change than this milestone.
    BooleanOp booleanOp = BooleanOp::Difference;

    // Boolean's tool body, BaseMesh's geometry, or Pattern's tool.
    Body bakedBody;

    // Pattern: how the copies are laid out and how many there are. The rest of
    // the layout reuses fields that already mean the same thing -- `axisPoint`
    // and `axisDir` for the axis or the mirror plane, `distance` for the gap
    // between copies, `angle` for the turn between them, and `booleanOp` for
    // whether each copy adds or takes away.
    //
    // With `bakedBody` set the pattern repeats that tool, which is how a hole
    // becomes a bolt circle. Empty, it repeats the body itself, which is how
    // half a symmetric part becomes the whole of it.
    PatternMode patternMode = PatternMode::Linear;
    int         patternCount = 2;

    // Reduce: how far the surface may move, and optionally how few triangles
    // to stop at (0: as few as the tolerance allows). The reduction is
    // deterministic, so re-evaluating the chain names the same faces again and
    // anything after it still finds what it refers to.
    Real reduceTolerance = 0.05;
    int  reduceTarget = 0;
    bool reduceLoosen = false;       // loosen past the tolerance to reach the target

    // The pattern this feature describes, assembled from the fields above.
    PatternSpec pattern() const;

    // Sketch: the geometry, what has been said about it, and its plane. Solved
    // each time the chain is evaluated, with the solved positions written back
    // here -- so what follows, and anyone looking at it, sees the shape its
    // constraints describe rather than the one it happened to be drawn as.
    Sketch sketch;

    // ExtrudeProfile: which sketch, and which regions of it. The sketch is
    // named by its feature's uid and each region by its key, neither of which
    // changes when a step is added before it or a line is added elsewhere in
    // the sketch. `distance` and `extrudeOp` say how far, and what to do with
    // them. Several regions are one step, swept together and combined with the
    // body once: an imported drawing is hundreds of regions, and a step each
    // was hundreds of booleans.
    ElementId sketchUid = 0;
    std::vector<SketchId> profileKeys;

    // Sketch: whether its geometry is drawn in the viewport. A sketch stays in
    // the outliner once something has been built from it, but showing every
    // sketch of a finished part would bury the part in its own scaffolding, so
    // extruding one turns its drawing off. Display only: it changes nothing
    // about what the chain builds.
    bool sketchShown = true;

    // Move and Rotate: where the object went, in the world. They are history
    // like everything else -- they can be edited, turned off and removed -- but
    // what they change is where the object stands rather than what it is, so
    // the chain leaves the body alone and the scene composes them onto the
    // placement the object was created at (see SceneObject::base). Turning a
    // move off puts the object back; the faces and edges after it keep their
    // names, since nothing about them changed.
    Vec3 moveBy{0, 0, 0};
    Quat turnBy{};
    Vec3 turnAbout{0, 0, 0};

    // Scale: how much along each of the body's own axes, and the point in its
    // own space that stays where it is. A real change of shape, made by the
    // kernel -- not a display transform -- so that everything which bakes
    // this body into something else, a boolean above all, gets the shape that
    // is on the screen.
    Vec3 scaleBy{1, 1, 1};
    Vec3 scaleAbout{0, 0, 0};

    // Boolean: what the tool body was called when it was combined, so the
    // history can say "Cut  Cylinder" rather than counting its faces.
    std::string toolName;

    // VertexEdit: a free-form drag, recorded as explicit offsets. Not
    // parametric in any meaningful sense, but it has to live in the chain so
    // that re-evaluating an earlier feature does not discard it.
    std::vector<ElementId> verts;
    std::vector<Vec3>      offsets;

    // Set by evaluation; not part of the definition.
    bool        errored = false;
    std::string error;
    int         sketchFreedoms = 0;   // Sketch: what the last solve left free

    // Short description for the timeline, e.g. "Extrude  12.0 mm".
    std::string summary() const;

    // What to call this step in a sentence. Not always featureKindName: a
    // bevel is a chamfer when it cuts flat and a fillet when it rounds, and a
    // message that calls it neither leaves the user hunting the timeline for a
    // word that is not there.
    const char* displayKind() const;
};

// Runs the chain, leaving the result in `out`. Marks failing features and keeps
// going, so one bad step does not destroy the rest of the model. Returns false
// only if nothing at all could be produced.
bool evaluateFeatures(std::vector<Feature>& features, Body& out);

// Whether a step places the object rather than shaping it.
inline bool isPlacement(FeatureKind k) { return k == FeatureKind::Move || k == FeatureKind::Rotate; }

// Re-runs the chain from `from` onward, reusing `cache[from - 1]` as the
// starting point. `cache[i]` holds the body as it stood after feature i.
//
// This is what keeps editing responsive on a heavy model: adding a bevel to a
// 100k-triangle part, or dragging the distance slider on the last feature,
// should cost that one operation rather than rebuilding the primitive and
// every step since. Falls back to a full evaluation if the cache cannot
// supply the requested starting point.
bool evaluateFrom(std::vector<Feature>& features, size_t from,
                  std::vector<Body>& cache, Body& out);

} // namespace tg
