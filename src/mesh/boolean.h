// Tangent - constructive solid geometry.
//
// Approach: BSP-based CSG. Each solid becomes a set of polygons; one solid's
// polygons are clipped against a BSP tree built from the other's, classified
// as inside or outside, and the halves kept or dropped according to the
// operation.
//
// Not the only option -- computing the exact intersection curve and
// retriangulating along it produces tidier output with fewer slivers -- but
// that needs exact predicates to be robust, and a BSP is far easier to get
// *correct*, which matters more here than tidy. Double precision buys the
// headroom the classification needs.
//
// The result is welded and rebuilt through Mesh::build, so it is manifold or
// the operation fails. A boolean never returns broken geometry.
//
// Known limitation: a cut whose plane lands on one an earlier operation already
// created is near-degenerate for a BSP, and classification goes wrong there --
// the result comes back with two faces on the same directed edge and is
// refused. Measured, the threshold is around 1e-3 of the model size. Fixing it
// needs exact predicates or an intersection-curve formulation. Until then the
// failure is at least honest: the operation returns false rather than handing
// back geometry that looks right and is not.
#pragma once

#include "mesh/halfedge.h"

namespace tg {

enum class BooleanOp { Union, Difference, Intersection };

const char* booleanOpName(BooleanOp op);

// Both inputs must be closed solids. `out` may alias neither input.
//
// Returns false, leaving `out` untouched, if either input is not closed, if the
// operation produces nothing (subtracting a solid from inside itself), or if
// the result cannot be rebuilt as a manifold mesh.
bool meshBoolean(const Mesh& a, const Mesh& b, BooleanOp op, Mesh& out);

} // namespace tg
