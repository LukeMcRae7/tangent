// Tangent - is this mesh actually printable?
//
// Manifoldness is not checked here: Mesh::build rejects non-manifold input
// outright, and every operation rebuilds through it, so a mesh that exists at
// all is already manifold and consistently wound. That is the deliberate
// difference from a general-purpose modeller, where non-manifold geometry is
// permitted and you discover it in the slicer.
//
// What remains possible, and is checked here, is geometry that is manifold but
// still not a solid: an open surface, zero-area faces, an inside-out shell, or
// a mesh that passes through itself after a free-form vertex drag.
#pragma once

#include "mesh/halfedge.h"

namespace tg {

struct MeshHealth {
    bool   watertight      = false;  // no boundary edges
    int    boundaryEdges   = 0;
    int    degenerateFaces = 0;      // area below tolerance
    int    shells          = 0;      // connected components
    double volume          = 0.0;    // signed; negative means inside out
    int    selfIntersections = 0;    // -1 if the check was not run

    // A solid a slicer will accept without complaint.
    bool solid() const {
        return watertight && degenerateFaces == 0 && volume > 0.0 &&
               selfIntersections <= 0;
    }
};

// `checkIntersections` is the expensive part -- it is broad-phased through a
// uniform grid but still quadratic in the worst case, so callers that run this
// every frame should leave it off.
MeshHealth checkHealth(const Mesh& mesh, bool checkIntersections = true);

} // namespace tg
