// Tangent - mesh editing operations.
//
// These work by extracting the polygon soup, rewriting it, and rebuilding
// connectivity. Editing half-edge links in place is faster but far easier to
// get subtly wrong, and rebuilding re-runs the manifold checks in Mesh::build,
// so a malformed result is rejected rather than silently corrupting the mesh.
//
// Every operation is transactional: on failure the mesh is left untouched.
#pragma once

#include "mesh/halfedge.h"

namespace tg {

// Pushes a set of faces along the region's area-weighted average normal,
// walling in the sides. The moved faces are reported in `newFaces` (in the
// same order as `faces`) so a caller can keep them selected.
//
// A negative distance cuts inward. Extruding an open surface (a plane) leaves
// the far side open, which is what removing the original face implies.
bool extrudeFaces(Mesh& mesh, const std::vector<Index>& faces, Real distance,
                  std::vector<Index>* newFaces = nullptr);

// Shrinks each face toward its own interior by `amount`, measured
// perpendicular to every edge, and fills the gap with a rim of quads.
// Operates per face rather than per region.
bool insetFaces(Mesh& mesh, const std::vector<Index>& faces, Real amount,
                std::vector<Index>* newFaces = nullptr);

// Translates the vertices belonging to a face set. Topology is unchanged, so
// this is cheap: no rebuild is required.
bool moveFaces(Mesh& mesh, const std::vector<Index>& faces, Vec3 delta);

// Replaces every edge with a flat chamfer of the given width, and every vertex
// with a face closing the corner.
//
// `segments` > 1 rounds the edge further by chamfering the chamfer. Note that
// this is an approximation, not an exact fillet: a true fillet arc is tangent
// to both original faces and bulges *outward* past the chamfer chord, whereas
// successive chamfers cut inward from it. The result is a rounded edge that
// removes slightly more material than a fillet of the same nominal radius, and
// `width` is the first cut's width rather than an exact radius.
//
// Fails without modifying the mesh if the width is too large for the geometry
// -- that is, if any face would invert.
bool bevelAllEdges(Mesh& mesh, Real width, int segments = 1);

// Largest bevel width the mesh can take before a face collapses. Useful for
// clamping a UI slider to a range that always produces valid geometry.
Real maxBevelWidth(const Mesh& mesh);

} // namespace tg
