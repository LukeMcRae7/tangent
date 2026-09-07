// Tangent - half-edge polygon mesh.
//
// Index-based half-edge connectivity supporting n-gon faces. Faces stay as
// polygons (not triangles) because the modelling operations in later
// milestones -- extrude face, inset, bevel, boolean -- are all defined on
// polygonal faces, and triangulating early would destroy the face identity
// the user selects and manipulates. Triangulation happens only at render time.
//
// Boundary edges are represented explicitly: every half-edge has a twin, and
// twins along an open boundary carry face == kInvalid. This keeps circulators
// total (never null) at the cost of some extra half-edges.
#pragma once

#include "core/math.h"
#include "mesh/element_id.h"

#include <cstdint>
#include <string>
#include <vector>

namespace tg {

using Index = int32_t;
inline constexpr Index kInvalid = -1;

struct HalfEdge {
    Index vertex = kInvalid;  // vertex this half-edge points TO
    Index face   = kInvalid;  // face to its left; kInvalid on a boundary loop
    Index next   = kInvalid;  // next half-edge around the same face
    Index prev   = kInvalid;  // previous half-edge around the same face
    Index twin   = kInvalid;  // opposite half-edge
};

struct MeshVertex {
    Vec3  position;
    Index halfedge = kInvalid;  // one half-edge originating at this vertex
    ElementId id   = kNoId;     // stable across re-evaluation; see element_id.h
};

// One boundary loop per face, and no inner ones.
//
// A drilled face genuinely has an inner boundary, so this is a real
// restriction, not an oversight: a plate with a hole through it cannot be one
// face here. The cost is that such a region has to be cut into pieces --
// mergeCoplanarFaces bridges each hole to the boundary around it with two cuts,
// so n holes come back as n+1 faces joined along 2n edges the surface does not
// really turn at.
//
// Kept deliberately. Inner loops would touch build's twin pairing and its
// manifold checks, triangulation, picking, and every operation that walks a
// face, and the whole of what they would buy is those 2n edges: n+1 faces for a
// region with n holes is already the minimum this representation allows, and it
// is reached in every case measured. The restriction is worth revisiting if
// faces with many holes become common -- a plate on a bolt circle, say -- and
// not before.
struct MeshFace {
    Index halfedge = kInvalid;  // one half-edge bounding this face
    ElementId id   = kNoId;
};

// Flat render buffers produced from the half-edge mesh.
struct RenderMesh {
    std::vector<Vec3>     positions;
    std::vector<Vec3>     normals;
    std::vector<uint32_t> triangles;   // 3 indices per triangle
    std::vector<uint32_t> edgeLines;   // 2 indices per wireframe edge

    // Maps each triangle back to the polygon face it came from, so a ray hit
    // on a triangle resolves to the face the user actually selected.
    std::vector<Index>    triangleFace;

    void clear() {
        positions.clear(); normals.clear(); triangles.clear();
        edgeLines.clear(); triangleFace.clear();
    }
};

class Mesh {
public:
    std::vector<MeshVertex> verts;
    std::vector<HalfEdge>   halfedges;
    std::vector<MeshFace>   faces;

    // Parallel to `halfedges`; an edge's two half-edges hold the same name.
    // Kept beside the half-edges rather than inside them: traversal is the
    // hot path and touches this only when a name is actually wanted.
    std::vector<ElementId>  edgeIds;

    void clear() { verts.clear(); halfedges.clear(); faces.clear(); edgeIds.clear(); }

    // ---- Names -----------------------------------------------------------
    ElementId vertexId(Index v) const { return verts[v].id; }
    ElementId faceId(Index f)   const { return faces[f].id; }
    ElementId edgeId(Index he)  const {
        return he >= 0 && he < static_cast<Index>(edgeIds.size()) ? edgeIds[he] : kNoId;
    }

    // Index for a name, or kInvalid. Linear: callers resolving a whole feature
    // should build a map instead of calling this in a loop.
    Index findVertex(ElementId id) const;
    Index findFace(ElementId id) const;
    Index findEdge(ElementId id) const;   // returns a half-edge

    // Ear-clips one face into triangles, as corner indices local to the face.
    // Public because a boolean works on triangles: the split routine it uses
    // assumes a plane cuts a polygon in two, which is only true if the polygon
    // is convex, and this mesh's faces need not be.
    void triangulateFacePublic(Index f, std::vector<Index>& out) const {
        triangulateFace(f, out);
    }

    // True when every element carries a name and no name is used twice.
    bool named() const;

    // Derives every edge name from its endpoints. Called by build once the
    // connectivity exists; public so an operation that renames vertices in
    // place can bring the edges back into step without a rebuild.
    void nameEdges();
    bool empty() const { return faces.empty(); }

    Index vertexCount() const { return static_cast<Index>(verts.size()); }
    Index faceCount()   const { return static_cast<Index>(faces.size()); }
    Index halfedgeCount() const { return static_cast<Index>(halfedges.size()); }

    // ---- Construction ----------------------------------------------------
    // Builds connectivity from a polygon soup. `faceSizes[i]` gives the vertex
    // count of face i, and `faceIndices` holds those indices back to back.
    // Returns false (leaving the mesh cleared) if the input is non-manifold,
    // which we reject rather than silently producing broken connectivity.
    // Stable names to attach to what is being built, if the caller has them.
    //
    // Only vertices and faces are named here. Edges are not, because they do
    // not need to be: a manifold mesh has at most one edge between any pair of
    // vertices, so the pair names the edge. Deriving it costs nothing, is
    // order-independent, and -- the point of the exercise -- means an edge that
    // survives an operation keeps its name without the operation having to know
    // it did. See element_id.h.
    struct Names {
        std::vector<ElementId> vertices;   // parallel to positions
        std::vector<ElementId> faces;      // parallel to faceSizes
    };

    bool build(const std::vector<Vec3>& positions,
               const std::vector<uint32_t>& faceSizes,
               const std::vector<uint32_t>& faceIndices,
               const Names* names = nullptr);

    // ---- Queries ---------------------------------------------------------
    Vec3 faceNormal(Index f) const;     // Newell's method; robust for n-gons
    Vec3 faceCentroid(Index f) const;
    Real faceArea(Index f) const;
    int  faceDegree(Index f) const;
    void faceVertices(Index f, std::vector<Index>& out) const;

    bool isBoundaryEdge(Index he) const {
        return halfedges[he].face == kInvalid ||
               halfedges[halfedges[he].twin].face == kInvalid;
    }

    // Is this edge only there because a face cannot have a hole?
    //
    // A region with a hole in it has to be cut into pieces here (see MeshFace),
    // and the cuts are edges the surface does not turn at -- the two lines a
    // drilled face comes back with, running from the bore to the rim. Nothing
    // on the model corresponds to them, and they should be neither drawn nor
    // offered to a click.
    //
    // Two coplanar faces sharing *more than one* edge is what says so. One
    // shared edge is the section line an extrude leaves, which is deliberate and
    // stays; a bridged hole always leaves its two halves joined along both of
    // its cuts. mergeCoplanarFaces has already absorbed every coplanar pair that
    // could be merged by the time anyone asks, so what is left sharing one edge
    // is there on purpose.
    bool isBridgeEdge(Index he, Real toleranceDegrees = 0.5) const;

    // The faces `f` was split into because a face cannot have a hole -- itself,
    // plus everything reachable from it across bridge edges only. One face for
    // anything that was not split that way.
    //
    // Crossing *only* bridge edges is the whole point. A split the user made is
    // a face they meant to have, and it stays its own face: raise part of a wall
    // and the section line divides it into two pieces that are picked, moved and
    // extruded separately. A split this representation forced is not a face at
    // all, and picking any of its pieces should reach the surface the user sees.
    // The two are told apart by how many edges the pieces share -- see above.
    void coplanarFaceGroup(Index f, std::vector<Index>& out,
                           Real toleranceDegrees = 0.5) const;
    // Vertex the half-edge starts from.
    Index fromVertex(Index he) const { return halfedges[halfedges[he].prev].vertex; }

    AABB bounds() const;
    AABB faceBounds(Index f) const {
        AABB box;
        std::vector<Index> fv;
        faceVertices(f, fv);
        for (Index v : fv) box.expand(verts[v].position);
        return box;
    }

    // ---- Output ----------------------------------------------------------
    // Triangulates for display. Corner normals are averaged across adjacent
    // faces only when the dihedral angle is under `creaseAngleDeg`, which
    // gives sharp box edges and smooth cylinder walls without per-primitive
    // shading flags.
    void buildRenderMesh(RenderMesh& out, Real creaseAngleDeg = 35.0) const;

    // Structural self-check used by tests and asserts; `err` gets the reason.
    bool validate(std::string* err = nullptr) const;

private:
    // Ear-clips one polygon face, appending triples of *corner* indices that
    // are local to the face (0 .. degree-1) rather than mesh vertex ids.
    void triangulateFace(Index f, std::vector<Index>& out) const;
};

} // namespace tg
