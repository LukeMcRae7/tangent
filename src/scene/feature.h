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

struct Feature {
    FeatureKind kind = FeatureKind::Primitive;
    bool        enabled = true;

    // Primitive: the base shape the chain starts from.
    PrimitiveSpec primitive;

    // Extrude / Inset: which faces, as numbered at this point in the chain.
    std::vector<Index> faces;
    Real distance = 5.0;    // Extrude, signed
    Real amount   = 2.0;    // Inset

    // Bevel.
    Real width    = 1.0;
    int  segments = 1;

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
    std::vector<Index> verts;
    std::vector<Vec3>  offsets;

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
