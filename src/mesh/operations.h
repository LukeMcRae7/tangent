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

enum class ExtrudeOp : uint32_t {
    Auto = 0,      // Union if self-intersecting/bridging; Difference if cutting inward; Direct if into empty space
    Join = 1,      // Always Union with body
    Cut = 2,       // Always Difference (cut) from body
    Intersect = 3, // Always Intersection with body
    NewBody = 4    // Separate body from sweep
};

const char* extrudeOpName(ExtrudeOp op);

// Pushes a set of faces along the region's area-weighted average normal,
// walling in the sides. The moved faces are reported in `newFaces` (in the
// same order as `faces`) so a caller can keep them selected.
//
// A negative distance cuts inward. Extruding an open surface (a plane) leaves
// the far side open, which is what removing the original face implies.
// `salt` identifies the operation when naming what it creates; a feature
// passes its own identity. See element_id.h.
bool extrudeFaces(Mesh& mesh, const std::vector<Index>& faces, Real distance,
                  std::vector<Index>* newFaces = nullptr, ElementId salt = 0,
                  ExtrudeOp op = ExtrudeOp::Auto);

// Shrinks each face toward its own interior by `amount`, measured
// perpendicular to every edge, and fills the gap with a rim of quads.
// Operates per face rather than per region.
bool insetFaces(Mesh& mesh, const std::vector<Index>& faces, Real amount,
                std::vector<Index>* newFaces = nullptr, ElementId salt = 0);

// Translates the vertices belonging to a face set. Topology is unchanged, so
// this is cheap: no rebuild is required.
bool moveFaces(Mesh& mesh, const std::vector<Index>& faces, Vec3 delta);

// One edge of a fillet, with its own radius. Fusion attaches a radius per edge
// within a single fillet feature rather than one radius for the whole
// selection, and so do we: rounding two edges to different radii in one go is
// a different solid from rounding them in sequence, because the corner where
// they meet is blended once instead of twice.
struct FilletEdge {
    Index edge   = kInvalid;   // either half-edge
    Real  radius = 1.0;
};

struct FilletSpec {
    std::vector<FilletEdge> edges;

    // Identifies the operation when naming what it creates, so that two
    // fillets in a chain do not hand their new faces the same names. A
    // feature passes its own identity here; see element_id.h.
    ElementId salt = 0;

    // 1 gives a flat chamfer. Above that the section follows a true circular
    // arc, tangent to both faces, swept in equal angular steps -- so the
    // segments are uniform and the surface is an actual fillet rather than a
    // progressively cut corner.
    int segments = 1;
};

// Rounds the given edges, pulling each adjacent face back to where a ball of
// the edge's radius touches it and bridging the gap with the ball's surface.
//
// The section is solved from the two face planes rather than assumed to be a
// quarter circle, which is what makes it correct for any dihedral angle, for
// concave edges as much as convex ones, and for the shallow angles between the
// facets of a curved surface.
//
// At a vertex the same ball settles into the corner, tangent to every face a
// filleted edge runs along, and the point where it touches each of those faces
// is where that face's boundary turns. Solving for that ball is what sets the
// corner back by the right amount -- which is the fillet radius only when the
// faces happen to meet at right angles, and much less on a curved surface.
//
// Fails without modifying the mesh if a radius is too large for the geometry
// -- that is, if any face would invert.
//
// `reason` gets a short phrase saying which of the two dozen ways this can fail
// actually happened, so a caller has something to show besides "it didn't
// work". Cleared on success. Without it the only trace is the TANGENT_BEVEL_DEBUG
// stream, and the interface is left guessing -- which is how "radius too large"
// came to be printed for failures that have nothing to do with the radius.
bool filletEdges(Mesh& mesh, const FilletSpec& spec, std::string* reason = nullptr);

// One radius for the whole selection.
bool bevelEdges(Mesh& mesh, const std::vector<Index>& edges, Real width,
                int segments = 1);

// Every edge at once. This is the whole-part rounding pass.
bool bevelAllEdges(Mesh& mesh, Real width, int segments = 1);

// Merges neighbouring faces that lie in the same plane, so a surface that is
// geometrically flat is one face rather than several with seams across it.
//
// Operations leave these behind routinely. Raise part of a solid and the wall
// of the raised part is coplanar with the wall it grew out of, but they are two
// faces with an edge between them -- an edge that is not on the model, only in
// the data. Picking one of them selects half of what the user sees as a face.
//
// Only pairs sharing exactly one edge are merged, and only when the result is a
// simple polygon. That is what keeps it safe: merging along two shared edges
// would pinch the face, and merging a ring closed would need a face with a
// hole, which this mesh cannot represent. Both cases are left alone, so a ring
// of coplanar faces merges as far as it can and stops.
//
// Returns the number of merges performed.
int mergeCoplanarFaces(Mesh& mesh, Real toleranceDegrees = 0.5);

// Splits a mesh into its connected bodies, in descending order of face count.
// Returns the number produced; a mesh that is already one piece yields itself,
// so a caller can always use the result.
//
// Booleans routinely produce several bodies -- subtracting a bar across the
// middle of a block leaves two -- and a slicer treats those as separate parts,
// so being able to pull them apart matters.
size_t splitShells(const Mesh& mesh, std::vector<Mesh>& out);

// Largest bevel width the mesh can take before a face collapses. Useful for
// clamping a UI slider to a range that always produces valid geometry.
Real maxBevelWidth(const Mesh& mesh);

// Extends a set of edges along smooth tangent curves or collinear segments
// (G1 / tangent continuity), matching Fusion 360's tangent chain selection.
std::vector<Index> extendTangentChain(const Mesh& mesh, const std::vector<Index>& edges);

// Splits a solid mesh with a cutting plane into two watertight solid bodies.
// Returns true if the cut produced two valid non-empty bodies.
bool splitBodyByPlane(const Mesh& mesh, Vec3 planePoint, Vec3 planeNormal,
                      Mesh& body1, Mesh& body2);

} // namespace tg
