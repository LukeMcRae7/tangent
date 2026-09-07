// Tangent - a solid body, whatever it is made of.
//
// This is the seam the rest of the application is written against. Above it
// nothing knows whether a body is a half-edge mesh or an exact boundary
// representation; below it, one backend or the other answers.
//
// The rule that makes the seam real is that **handles are opaque**. A FaceId,
// EdgeId or VertexId comes out of a Body and goes back into a Body, and no
// caller may do arithmetic on one, compare it to a count, or assume the
// numbering is dense or stable across an edit. The mesh backend happens to
// return array indices, and an edge happens to be the lower of its two
// half-edges -- but nothing outside src/geom is allowed to know that, which is
// what lets a second backend choose its own numbering later without touching a
// line of the application.
//
// Two things deliberately stay on this side of the seam. Names (ElementId) are
// the parametric history's currency and mean the same thing for any backend.
// So does RenderMesh: every backend has to end up as triangles to be drawn, and
// carrying the face and edge each one came from is what makes picking work.
#pragma once

#include "mesh/halfedge.h"
#include "mesh/health.h"

#include <string>
#include <vector>

namespace tg {

// Opaque handles into one Body. See the note above: never do arithmetic on
// these, and never keep one across an edit -- keep the ElementId instead.
using FaceId   = Index;
using EdgeId   = Index;
using VertexId = Index;

inline constexpr FaceId kNoFace = kInvalid;

class Body {
public:
    Body() = default;
    explicit Body(Mesh m) : mesh_(std::move(m)) {}

    bool empty() const { return mesh_.empty(); }

    // ---- Enumeration -------------------------------------------------------
    // Counts are for reporting to the user, not for iterating: iterate with the
    // three collectors below, which hand back handles.
    int faceCount()   const { return mesh_.faceCount(); }
    int vertexCount() const { return mesh_.vertexCount(); }
    int edgeCount()   const;

    // Is this handle still one of ours? Selections are held as handles between
    // edits, and an edit renumbers, so anything that kept one has to ask.
    bool hasFace(FaceId f) const { return f >= 0 && f < mesh_.faceCount(); }
    bool hasVertex(VertexId v) const { return v >= 0 && v < mesh_.vertexCount(); }
    bool hasEdge(EdgeId e) const;

    void allFaces(std::vector<FaceId>& out) const;
    void allEdges(std::vector<EdgeId>& out) const;
    void allVertices(std::vector<VertexId>& out) const;

    // ---- Topology ----------------------------------------------------------
    // faceEdges hands back canonical edge handles, so no caller ever has to
    // work out which of a pair of half-edges names the edge.
    void faceEdges(FaceId f, std::vector<EdgeId>& out) const;
    void faceVertices(FaceId f, std::vector<VertexId>& out) const;
    int  faceDegree(FaceId f) const { return mesh_.faceDegree(f); }

    void edgeEnds(EdgeId e, VertexId& a, VertexId& b) const;
    // kNoFace on the far side of a boundary edge.
    void edgeFaces(EdgeId e, FaceId& a, FaceId& b) const;

    // Every edge that touches this vertex.
    void vertexEdges(VertexId v, std::vector<EdgeId>& out) const;

    // ---- Geometry ----------------------------------------------------------
    Vec3 faceNormal(FaceId f)   const { return mesh_.faceNormal(f); }
    Vec3 faceCentroid(FaceId f) const { return mesh_.faceCentroid(f); }
    Real faceArea(FaceId f)     const { return mesh_.faceArea(f); }
    AABB faceBounds(FaceId f)   const { return mesh_.faceBounds(f); }
    Vec3 vertexPosition(VertexId v) const { return mesh_.verts[v].position; }
    AABB bounds() const { return mesh_.bounds(); }

    // Both ends of an edge in one call, which is what nearly every caller wants.
    void edgePositions(EdgeId e, Vec3& a, Vec3& b) const;
    Vec3 edgeDirection(EdgeId e) const;   // normalised, from the first end

    // ---- Names -------------------------------------------------------------
    ElementId faceName(FaceId f)     const { return mesh_.faceId(f); }
    ElementId edgeName(EdgeId e)     const { return mesh_.edgeId(e); }
    ElementId vertexName(VertexId v) const { return mesh_.vertexId(v); }

    FaceId   findFace(ElementId id)   const { return mesh_.findFace(id); }
    EdgeId   findEdge(ElementId id)   const;
    VertexId findVertex(ElementId id) const { return mesh_.findVertex(id); }

    // ---- Display and validity ---------------------------------------------
    void tessellate(RenderMesh& out, Real creaseAngleDeg = 35.0) const {
        mesh_.buildRenderMesh(out, creaseAngleDeg);
    }
    bool validate(std::string* err = nullptr) const { return mesh_.validate(err); }
    MeshHealth health(bool checkIntersections = true) const {
        return checkHealth(mesh_, checkIntersections);
    }

    // ---- Representation artefacts -----------------------------------------
    // An edge that exists only because this representation cannot hold a face
    // with a hole. A B-rep body has none and will answer false to all of them.
    bool isBridgeEdge(EdgeId e) const { return mesh_.isBridgeEdge(e); }
    void coplanarFaceGroup(FaceId f, std::vector<FaceId>& out) const {
        mesh_.coplanarFaceGroup(f, out);
    }

    // ---- Mutation ----------------------------------------------------------
    // Moving a vertex changes no topology, which is why it can be done in place
    // without a rebuild. It is also the one operation here that a B-rep backend
    // cannot honour in general -- a vertex there is where surfaces meet, not a
    // free point -- so VertexEdit stays a mesh-only feature and the backend will
    // refuse rather than approximate.
    void moveVertex(VertexId v, Vec3 delta) { mesh_.verts[v].position += delta; }
    void setVertexPosition(VertexId v, Vec3 p) { mesh_.verts[v].position = p; }

    // Place the whole body somewhere else. Exact on either backend -- a rigid
    // transform of a plane is a plane, of a cylinder a cylinder -- which is why
    // it belongs here rather than being done by walking vertices.
    void transform(const Mat4& m);

    // ---- Backend access ----------------------------------------------------
    // The operations in src/geom/operations.h reach through here. Nothing else
    // should: this is the one place the seam is deliberately open, and it is
    // where the second backend will be dispatched from.
    bool isMesh() const { return true; }
    const Mesh& mesh() const { return mesh_; }
    Mesh&       mesh()       { return mesh_; }

private:
    Mesh mesh_;
};

} // namespace tg
