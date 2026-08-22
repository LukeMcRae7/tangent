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
};

struct MeshFace {
    Index halfedge = kInvalid;  // one half-edge bounding this face
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

    void clear() { verts.clear(); halfedges.clear(); faces.clear(); }
    bool empty() const { return faces.empty(); }

    Index vertexCount() const { return static_cast<Index>(verts.size()); }
    Index faceCount()   const { return static_cast<Index>(faces.size()); }
    Index halfedgeCount() const { return static_cast<Index>(halfedges.size()); }

    // ---- Construction ----------------------------------------------------
    // Builds connectivity from a polygon soup. `faceSizes[i]` gives the vertex
    // count of face i, and `faceIndices` holds those indices back to back.
    // Returns false (leaving the mesh cleared) if the input is non-manifold,
    // which we reject rather than silently producing broken connectivity.
    bool build(const std::vector<Vec3>& positions,
               const std::vector<uint32_t>& faceSizes,
               const std::vector<uint32_t>& faceIndices);

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
    // Vertex the half-edge starts from.
    Index fromVertex(Index he) const { return halfedges[halfedges[he].prev].vertex; }

    AABB bounds() const;

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
