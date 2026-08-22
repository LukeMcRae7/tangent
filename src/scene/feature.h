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

#include "mesh/primitives.h"

#include <string>
#include <vector>

namespace tg {

enum class FeatureKind { Primitive, Extrude, Inset, Bevel, VertexEdit };

const char* featureKindName(FeatureKind k);

struct Feature {
    FeatureKind kind = FeatureKind::Primitive;
    bool        enabled = true;

    // Primitive: the base shape the chain starts from.
    PrimitiveSpec primitive;

    // Extrude / Inset: which faces, as numbered at this point in the chain.
    std::vector<Index> faces;
    float distance = 5.0f;   // Extrude, signed
    float amount   = 2.0f;   // Inset

    // Bevel.
    float width    = 1.0f;
    int   segments = 1;

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
