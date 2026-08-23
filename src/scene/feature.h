// Tangent - parametric feature history.
//
// An object's mesh is not authored, it is *evaluated*: a base primitive
// followed by an ordered list of operations. Editing any parameter re-runs the
// chain, which is what makes the model parametric rather than a frozen result.
//
// Known limitation, stated plainly because it will bite: operations name the
// faces they act on by index. Face numbering is stable while the features
// before them are unchanged, so editing a primitive's dimensions re-applies
// later operations correctly. But inserting, removing or reordering a feature
// renumbers everything downstream, and an index-based reference cannot follow
// that. This is the topological naming problem, and solving it properly needs
// persistent identifiers that survive a remesh -- not attempted here. What is
// implemented instead is honest failure: a feature whose references no longer
// resolve is marked errored and skipped, the chain continues, and the timeline
// shows which step gave up rather than silently producing wrong geometry.
#pragma once

#include "mesh/boolean.h"
#include "mesh/primitives.h"

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

    // Resolves to half-edge indices in `mesh`. Returns false if anything named
    // here has gone; a feature that cannot find what it acts on is errored, not
    // quietly re-pointed at whatever now sits at those numbers.
    bool resolveEdges(const Mesh& mesh, std::vector<Index>& out) const;

    // Resolves to face indices.
    bool resolveFaces(const Mesh& mesh, std::vector<Index>& out) const;
};

// Records a selection made on `mesh` as something that will still mean the
// same thing later.
ElementRefs nameFaces(const Mesh& mesh, const std::vector<Index>& faces);

// As above, and additionally: if the chosen edges are exactly the boundary of
// one face, that is recorded instead of the list. It is what the user meant --
// they picked a rim, not sixteen edges that happen to be there today -- and it
// is the only form that survives the face being retessellated under them.
ElementRefs nameEdges(const Mesh& mesh, const std::vector<Index>& edges);

std::vector<ElementId> nameVertices(const Mesh& mesh, const std::vector<Index>& verts);

struct Feature {
    FeatureKind kind = FeatureKind::Primitive;
    bool        enabled = true;

    // Identifies this feature for as long as it exists, independent of where it
    // sits in the chain. It salts the names of everything the feature creates,
    // so moving a feature does not rename its output and two features of the
    // same kind do not collide. Assigned when the feature is created.
    ElementId uid = 0;

    // Primitive: the base shape the chain starts from.
    PrimitiveSpec primitive;

    // Extrude / Inset: which faces, by name as of this point in the chain.
    ElementRefs faces;
    Real distance = 5.0;    // Extrude, signed
    Real amount   = 2.0;    // Inset

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

    // Boolean's tool body, or BaseMesh's geometry.
    Mesh bakedMesh;

    // VertexEdit: a free-form drag, recorded as explicit offsets. Not
    // parametric in any meaningful sense, but it has to live in the chain so
    // that re-evaluating an earlier feature does not discard it.
    std::vector<ElementId> verts;
    std::vector<Vec3>      offsets;

    // Set by evaluation; not part of the definition.
    bool        errored = false;
    std::string error;

    // Short description for the timeline, e.g. "Extrude  12.0 mm".
    std::string summary() const;
};

// Runs the chain, leaving the result in `out`. Marks failing features and keeps
// going, so one bad step does not destroy the rest of the model. Returns false
// only if nothing at all could be produced.
bool evaluateFeatures(std::vector<Feature>& features, Mesh& out);

// Re-runs the chain from `from` onward, reusing `cache[from - 1]` as the
// starting point. `cache[i]` holds the mesh as it stood after feature i.
//
// This is what keeps editing responsive on a heavy model: adding a bevel to a
// 100k-triangle part, or dragging the distance slider on the last feature,
// should cost that one operation rather than rebuilding the primitive and
// every step since. Falls back to a full evaluation if the cache cannot
// supply the requested starting point.
bool evaluateFrom(std::vector<Feature>& features, size_t from,
                  std::vector<Mesh>& cache, Mesh& out);

} // namespace tg
