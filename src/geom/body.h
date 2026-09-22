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

#include <mutex>
#include <string>
#include <vector>

namespace tg {

// The one lock over everything that writes into a shape, or reads one that
// might be written: meshing writes its triangulation into the shape itself, and
// every copy of a body points at the same shape. Held for the length of a mesh
// or a copy, by whichever thread is doing it.
std::mutex& meshingLock();

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
        if (brep_) return brep::faceDegree(*brep_, f);
        return hasFace(f) ? mesh_.faceDegree(f) : 0;
    }

    void edgeEnds(EdgeId e, VertexId& a, VertexId& b) const;
    // kNoFace on the far side of a boundary edge.
    void edgeFaces(EdgeId e, FaceId& a, FaceId& b) const;

    // Every edge that touches this vertex.
    void vertexEdges(VertexId v, std::vector<EdgeId>& out) const;

    // ---- Geometry ----------------------------------------------------------
    // Every one of these checks the handle first.
    //
    // A handle does not survive an edit, and something always ends up holding
    // one that did not -- a selection made before an operation, a tool that
    // cached one, a panel drawing last frame's highlight. The B-rep backend has
    // always answered those safely; the mesh backend indexed straight into a
    // vector and took the process down with it. Two backends behind one
    // interface have to fail the same way, and the safe way is the one that
    // leaves a user's work on screen.
    Vec3 faceNormal(FaceId f)   const {
        if (brep_) return brep::faceNormal(*brep_, f);
        return hasFace(f) ? mesh_.faceNormal(f) : Vec3{};
    }
    Vec3 faceCentroid(FaceId f) const {
        if (brep_) return brep::faceCentroid(*brep_, f);
        return hasFace(f) ? mesh_.faceCentroid(f) : Vec3{};
    }
    // A point on the face itself. For a flat face that is its centroid; for a
    // curved one the centroid can be somewhere else entirely -- the centre of
    // mass of a cylinder's wall is on its axis -- and anything standing an
    // arrow or a label on the face wants a point that is on it.
    Vec3 facePoint(FaceId f) const {
        if (brep_) return brep::facePoint(*brep_, f);
        return hasFace(f) ? mesh_.faceCentroid(f) : Vec3{};
    }
    Real faceArea(FaceId f)     const {
        if (brep_) return brep::faceArea(*brep_, f);
        return hasFace(f) ? mesh_.faceArea(f) : Real(0);
    }
    AABB faceBounds(FaceId f)   const {
        if (brep_) return brep::faceBounds(*brep_, f);
        return hasFace(f) ? mesh_.faceBounds(f) : AABB{};
    }
    Vec3 vertexPosition(VertexId v) const {
        if (brep_) return brep::vertexPosition(*brep_, v);
        return hasVertex(v) ? mesh_.verts[v].position : Vec3{};
    }
    AABB bounds() const { return brep_ ? brep::bounds(*brep_) : mesh_.bounds(); }

    // Both ends of an edge in one call, which is what nearly every caller wants.
    void edgePositions(EdgeId e, Vec3& a, Vec3& b) const;
    Vec3 edgeDirection(EdgeId e) const;   // normalised, from the first end

    // ---- What a thing actually is -----------------------------------------
    // A mesh answers "a polygon" and "a straight line" to all of these, which
    // is honest: that is all it has. An exact body answers with the surface and
    // the curve it was built from, and that is what makes a hole's diameter a
    // diameter rather than the width of a facet.
    SurfaceKind faceKind(FaceId f) const {
        return brep_ ? brep::faceKind(*brep_, f) : SurfaceKind::Plane;
    }
    CurveKind edgeKind(EdgeId e) const {
        return brep_ ? brep::edgeKind(*brep_, e) : CurveKind::Line;
    }
    bool edgeCircle(EdgeId e, Vec3& centre, Vec3& axis, Real& radius) const {
        return brep_ ? brep::edgeCircle(*brep_, e, centre, axis, radius) : false;
    }
    bool faceCylinder(FaceId f, Vec3& point, Vec3& axis, Real& radius) const {
        return brep_ ? brep::faceCylinder(*brep_, f, point, axis, radius) : false;
    }

    // Along the curve, not across the chord: a semicircular edge of radius 10
    // is 31.4mm long, and a full circle's chord is zero.
    Real edgeLength(EdgeId e) const;
    Vec3 edgeMidpoint(EdgeId e) const;

    // The edge as a chain of points along the curve, no chord further than
    // `deviationMm` from it; 0 lets the backend choose. Anything that draws an
    // edge wants this and not the two ends -- the straight line between a
    // circular rim's ends runs across the hole rather than around it.
    //
    // A mesh edge comes back as its two ends, which is the whole of a line.
    void edgePolyline(EdgeId e, Real deviationMm, std::vector<Vec3>& out) const;

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
    // How closely the triangles have to follow the surface. The defaults are
    // the screen's answer; an export sets its own, because the number a printer
    // cares about and the number a frame budget can afford are not the same
    // number. See TessellationQuality in brep.h.
    // Meshing writes its triangulation into the shape, which every copy of this
    // body shares, so it takes the lock that guards that state -- see
    // meshingLock(). Uncontended it costs nothing; it only ever waits when a
    // worker is taking its own copy of the same shape at that moment.
    void tessellate(RenderMesh& out, TessellationQuality q = {}) const {
        if (!brep_) { mesh_.buildRenderMesh(out, q.creaseAngleDeg); return; }
        std::lock_guard<std::mutex> hold(meshingLock());
        brep::tessellate(*brep_, out, q);
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
        if (!brep_ && hasVertex(v)) mesh_.verts[v].position += delta;
    }
    void setVertexPosition(VertexId v, Vec3 p) {
        if (!brep_ && hasVertex(v)) mesh_.verts[v].position = p;
    }
    bool canMoveVertices() const { return !brep_; }

    // Place the whole body somewhere else, or stretch it. Exact on either
    // backend -- see brep::transformed for what a stretch costs an exact body.
    // False, leaving the body as it was, for a matrix that would turn it
    // inside out or flatten it.
    bool transform(const Mat4& m);

    // Scales about `pivot`, along the body's own axes. Every factor must be
    // positive: a negative one is a mirror, and mirror() is how to ask.
    bool scale(Vec3 factors, Vec3 pivot = Vec3{});

    // Reflects across a plane. Not transform(reflectionMatrix) -- see
    // brep::mirrored for why that is not the same thing.
    bool mirror(Vec3 planePoint, Vec3 planeNormal);

    // ---- Backend access ----------------------------------------------------
    // The operations in src/geom/operations.h reach through here. Nothing else
    // should: this is the one place the seam is deliberately open, and it is
    // where the second backend will be dispatched from.
    bool isMesh() const { return !brep_; }

    // A body that shares nothing with this one.
    //
    // Copying a Body is cheap because an exact body is a handle to an immutable
    // shape, and every copy points at the same one. That is right until two
    // threads are involved: meshing writes its triangulation into the shape, so
    // a shape being drawn on one thread must not be read by an algorithm on
    // another. Anything handed to a worker gets one of these.
    Body detached() const {
        if (!brep_) return *this;          // a mesh is already a value
        // Reading the shape while another thread meshes it is the race this
        // exists to prevent, so the copy waits for any meshing to finish.
        std::lock_guard<std::mutex> hold(meshingLock());
        return Body(brep::detach(*brep_));
    }

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
