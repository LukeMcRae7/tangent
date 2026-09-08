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

#include "geom/brep.h"
#include "mesh/halfedge.h"
#include "mesh/health.h"

#include <string>
#include <vector>

namespace tg {

// FaceId, EdgeId and VertexId are declared in brep.h, where the backend that
// numbers them differently also has to see them.

inline constexpr FaceId kNoFace = kInvalid;

class Body {
public:
    Body() = default;
    explicit Body(Mesh m) : mesh_(std::move(m)) {}

    // A B-rep body. The shape is shared rather than copied: a feature chain
    // keeps one body per step, and a TopoDS_Shape is a handle to geometry that
    // may be large. Anything that mutates detaches first.
    explicit Body(BrepRef s) : brep_(std::move(s)) {}

    bool empty() const { return brep_ ? brep::empty(*brep_) : mesh_.empty(); }

    // ---- Enumeration -------------------------------------------------------
    // Counts are for reporting to the user, not for iterating: iterate with the
    // three collectors below, which hand back handles.
    int faceCount()   const { return brep_ ? brep::faceCount(*brep_) : mesh_.faceCount(); }
    int vertexCount() const { return brep_ ? brep::vertexCount(*brep_) : mesh_.vertexCount(); }
    int edgeCount()   const;

    // Is this handle still one of ours? Selections are held as handles between
    // edits, and an edit renumbers, so anything that kept one has to ask.
    bool hasFace(FaceId f) const {
        return brep_ ? brep::hasFace(*brep_, f) : (f >= 0 && f < mesh_.faceCount());
    }
    bool hasVertex(VertexId v) const {
        return brep_ ? brep::hasVertex(*brep_, v) : (v >= 0 && v < mesh_.vertexCount());
    }
    bool hasEdge(EdgeId e) const;

    void allFaces(std::vector<FaceId>& out) const;
    void allEdges(std::vector<EdgeId>& out) const;
    void allVertices(std::vector<VertexId>& out) const;

    // ---- Topology ----------------------------------------------------------
    // faceEdges hands back canonical edge handles, so no caller ever has to
    // work out which of a pair of half-edges names the edge.
    void faceEdges(FaceId f, std::vector<EdgeId>& out) const;
    void faceVertices(FaceId f, std::vector<VertexId>& out) const;
    int  faceDegree(FaceId f) const {
        return brep_ ? brep::faceDegree(*brep_, f) : mesh_.faceDegree(f);
    }

    void edgeEnds(EdgeId e, VertexId& a, VertexId& b) const;
    // kNoFace on the far side of a boundary edge.
    void edgeFaces(EdgeId e, FaceId& a, FaceId& b) const;

    // Every edge that touches this vertex.
    void vertexEdges(VertexId v, std::vector<EdgeId>& out) const;

    // ---- Geometry ----------------------------------------------------------
    Vec3 faceNormal(FaceId f)   const {
        return brep_ ? brep::faceNormal(*brep_, f) : mesh_.faceNormal(f);
    }
    Vec3 faceCentroid(FaceId f) const {
        return brep_ ? brep::faceCentroid(*brep_, f) : mesh_.faceCentroid(f);
    }
    Real faceArea(FaceId f)     const {
        return brep_ ? brep::faceArea(*brep_, f) : mesh_.faceArea(f);
    }
    AABB faceBounds(FaceId f)   const {
        return brep_ ? brep::faceBounds(*brep_, f) : mesh_.faceBounds(f);
    }
    Vec3 vertexPosition(VertexId v) const {
        return brep_ ? brep::vertexPosition(*brep_, v) : mesh_.verts[v].position;
    }
    AABB bounds() const { return brep_ ? brep::bounds(*brep_) : mesh_.bounds(); }

    // Both ends of an edge in one call, which is what nearly every caller wants.
    void edgePositions(EdgeId e, Vec3& a, Vec3& b) const;
    Vec3 edgeDirection(EdgeId e) const;   // normalised, from the first end

    // ---- Names -------------------------------------------------------------
    ElementId faceName(FaceId f)     const {
        return brep_ ? brep::faceName(*brep_, f) : mesh_.faceId(f);
    }
    ElementId edgeName(EdgeId e)     const {
        return brep_ ? brep::edgeName(*brep_, e) : mesh_.edgeId(e);
    }
    ElementId vertexName(VertexId v) const {
        return brep_ ? brep::vertexName(*brep_, v) : mesh_.vertexId(v);
    }

    FaceId   findFace(ElementId id)   const {
        return brep_ ? brep::findFace(*brep_, id) : mesh_.findFace(id);
    }
    EdgeId   findEdge(ElementId id)   const;

    // Every face answering to a name. A name can stand for more than one: an
    // operation splits a face and each piece keeps the name, so a feature that
    // referred to the face goes on referring to all of it. findFace returns the
    // first, for the callers that only need one.
    void findFaces(ElementId id, std::vector<FaceId>& out) const;
    VertexId findVertex(ElementId id) const {
        return brep_ ? brep::findVertex(*brep_, id) : mesh_.findVertex(id);
    }

    // ---- Display and validity ---------------------------------------------
    // `deviationMm` is the chord tolerance a curved backend should meet; zero
    // lets it choose one from the size of the body. The mesh backend ignores it
    // -- its curves were decided when the primitive was made -- and the crease
    // angle is the mirror of that: a B-rep knows where its edges are and does
    // not have to guess from the angle between two facets.
    void tessellate(RenderMesh& out, Real creaseAngleDeg = 35.0,
                    Real deviationMm = 0.0) const {
        if (brep_) brep::tessellate(*brep_, out, deviationMm);
        else       mesh_.buildRenderMesh(out, creaseAngleDeg);
    }
    bool validate(std::string* err = nullptr) const {
        return brep_ ? brep::validate(*brep_, err) : mesh_.validate(err);
    }
    MeshHealth health(bool checkIntersections = true) const {
        return brep_ ? brep::health(*brep_, checkIntersections)
                     : checkHealth(mesh_, checkIntersections);
    }

    // ---- Representation artefacts -----------------------------------------
    // An edge that exists only because this representation cannot hold a face
    // with a hole. A B-rep body has none and will answer false to all of them.
    bool isBridgeEdge(EdgeId e) const {
        return brep_ ? false : mesh_.isBridgeEdge(e);
    }
    void coplanarFaceGroup(FaceId f, std::vector<FaceId>& out) const;

    // ---- Mutation ----------------------------------------------------------
    // Moving a vertex changes no topology, which is why it can be done in place
    // without a rebuild. It is also the one operation here that a B-rep backend
    // cannot honour in general -- a vertex there is where surfaces meet, not a
    // free point -- so VertexEdit stays a mesh-only feature and the backend will
    // refuse rather than approximate.
    // Both are no-ops on a B-rep body, and callers are expected to have asked
    // canMoveVertices() first: a feature that silently did nothing would be
    // worse than one that refuses with a reason.
    void moveVertex(VertexId v, Vec3 delta) {
        if (!brep_) mesh_.verts[v].position += delta;
    }
    void setVertexPosition(VertexId v, Vec3 p) {
        if (!brep_) mesh_.verts[v].position = p;
    }
    bool canMoveVertices() const { return !brep_; }

    // Place the whole body somewhere else. Exact on either backend -- a rigid
    // transform of a plane is a plane, of a cylinder a cylinder -- which is why
    // it belongs here rather than being done by walking vertices.
    void transform(const Mat4& m);

    // ---- Backend access ----------------------------------------------------
    // The operations in src/geom/operations.h reach through here. Nothing else
    // should: this is the one place the seam is deliberately open, and it is
    // where the second backend will be dispatched from.
    bool isMesh() const { return !brep_; }

    // Valid only on a mesh body; ask isMesh() first. An empty Mesh comes back
    // for a B-rep body rather than anything undefined.
    const Mesh& mesh() const { return mesh_; }
    Mesh&       mesh()       { return mesh_; }

    // Valid only on a B-rep body. The reference is to shared state: anything
    // that changes it must build a new shape rather than write through this.
    const BrepShape& brep() const { return *brep_; }
    const BrepRef&   brepRef() const { return brep_; }

private:
    Mesh    mesh_;
    BrepRef brep_;   // non-null means this body is a B-rep, and mesh_ is unused
};

} // namespace tg
