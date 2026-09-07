// Tangent - constructive solid geometry.
//
// Approach: each solid becomes a set of polygons; a face the other solid
// reaches is split against the planes of the triangles near it, and each piece
// is kept or dropped according to which side of the other solid it is on. That
// side is decided by summing the solid angle the other solid subtends at a
// point just off the piece -- a full sphere from inside, nothing from outside.
// There is no BSP; the tree that used to drive this made the answer depend on
// how it had been built, and could not be asked twice about the same body.
//
// Not the only option -- computing the exact intersection curve and
// retriangulating along it produces tidier output with fewer slivers -- but
// that needs exact predicates to be robust, and this is far easier to get
// *correct*, which matters more here than tidy. Double precision buys the
// headroom the classification needs.
//
// The result is welded and rebuilt through Mesh::build, so it is manifold or
// the operation fails. A boolean never returns broken geometry.
//
// Splitting against infinite planes cuts a face far beyond where the two solids
// actually meet, so the output arrives in far more pieces than the shape needs
// -- thousands, for a finely faceted cutter. mergeCoplanarFaces puts each flat
// region back together afterwards and reaches the minimum this mesh can
// represent, so that is a cost in time rather than in the result. Two things
// keep it bounded: a piece already clear of the other solid's bounding box is
// not split again, since no further cut could change what it classifies as, and
// a probe outside that box is answered without a sum at all.
//
// Known limitation: a cut landing very close to geometry an earlier operation
// created is near-degenerate, and the rebuild refuses it -- the result comes
// back with two faces on one directed edge, or a vertex two sheets meet at.
// It is rare (one refusal in a thousand successive random cuts) and it is
// honest: the operation returns false rather than handing back geometry that
// looks right and is not. Fixing the last of it needs exact predicates or an
// intersection-curve formulation.
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
// `salt` identifies the operation when naming the result. Every face and vertex
// of the output gets a stable name derived from the face it was cut from, so a
// feature that acts on the result can still find what it acted on. Without one
// the output is nameless, and a stored reference matches the first face in the
// mesh rather than the intended one.
// `trustBNames` says the second operand's names are already distinct from the
// first's, so they pass through as they are. A caller that builds the second
// solid out of the first's own faces -- an extrude sweeping a face it is about
// to replace -- needs this, so the face comes out of the operation still called
// what the user picked. Left false, the second operand's names are re-derived,
// since two solids from the same generator name their faces identically.
bool meshBoolean(const Mesh& a, const Mesh& b, BooleanOp op, Mesh& out,
                 ElementId salt = 0, bool trustBNames = false);

bool debugPointInsideMesh(const Mesh& m, Vec3 p);

} // namespace tg
