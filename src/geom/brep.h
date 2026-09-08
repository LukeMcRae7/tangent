// Tangent - the B-rep backend, as seen from the seam.
//
// Body needs about thirty questions answered about a body. This header is those
// questions, asked of an opaque BrepShape, so that body.h never includes an
// OpenCASCADE header and nothing above src/geom can accidentally depend on one.
//
// There are two implementations. brep_occt.cpp answers them with OpenCASCADE;
// brep_stub.cpp answers them by refusing, and is compiled when the project is
// built without OCCT. That is not a nicety: a project may be built either way,
// and the alternative -- #ifdef inside body.h -- would give Body two different
// layouts depending on which translation unit you asked, which is the kind of
// bug that takes a week to find.
//
// The seam's rule holds here too: FaceId, EdgeId and VertexId are opaque. The
// OCCT implementation numbers them by its own indexed maps, which have nothing
// to do with the mesh backend's array indices, and no caller can tell.
#pragma once

#include "core/math.h"
#include "mesh/element_id.h"
#include "mesh/boolean.h"
#include "mesh/halfedge.h"
#include "mesh/primitives.h"
#include "mesh/health.h"

#include <memory>
#include <string>
#include <vector>

namespace tg {

using FaceId   = Index;
using EdgeId   = Index;
using VertexId = Index;

// A shape and the names attached to it. Defined only in brep_occt.cpp: everyone
// else holds it through a pointer and asks the functions below.
struct BrepShape;

using BrepRef = std::shared_ptr<const BrepShape>;

namespace brep {

// Was the project built with OpenCASCADE? Everything below returns an empty or
// refusing answer when this is false, so a caller can be written once.
bool available();

// ---- Lifetime --------------------------------------------------------------
// A deep copy, for the one case that needs one: mutating a body that other
// Bodies share. Returns null when OCCT is absent.
BrepRef clone(const BrepShape& s);

// ---- Enumeration -----------------------------------------------------------
bool empty(const BrepShape& s);
int  faceCount(const BrepShape& s);
int  edgeCount(const BrepShape& s);
int  vertexCount(const BrepShape& s);

bool hasFace(const BrepShape& s, FaceId f);
bool hasEdge(const BrepShape& s, EdgeId e);
bool hasVertex(const BrepShape& s, VertexId v);

void allFaces(const BrepShape& s, std::vector<FaceId>& out);
void allEdges(const BrepShape& s, std::vector<EdgeId>& out);
void allVertices(const BrepShape& s, std::vector<VertexId>& out);

// ---- Topology --------------------------------------------------------------
void faceEdges(const BrepShape& s, FaceId f, std::vector<EdgeId>& out);
void faceVertices(const BrepShape& s, FaceId f, std::vector<VertexId>& out);
int  faceDegree(const BrepShape& s, FaceId f);

void edgeEnds(const BrepShape& s, EdgeId e, VertexId& a, VertexId& b);
void edgeFaces(const BrepShape& s, EdgeId e, FaceId& a, FaceId& b);
void vertexEdges(const BrepShape& s, VertexId v, std::vector<EdgeId>& out);

// ---- Geometry --------------------------------------------------------------
// Exact, not sampled: a face's normal comes from its surface and an edge's
// direction from its curve, which is the whole point of the backend.
Vec3 faceNormal(const BrepShape& s, FaceId f);
Vec3 faceCentroid(const BrepShape& s, FaceId f);
Real faceArea(const BrepShape& s, FaceId f);
AABB faceBounds(const BrepShape& s, FaceId f);
Vec3 vertexPosition(const BrepShape& s, VertexId v);
AABB bounds(const BrepShape& s);
void edgePositions(const BrepShape& s, EdgeId e, Vec3& a, Vec3& b);
Vec3 edgeDirection(const BrepShape& s, EdgeId e);

// ---- Names -----------------------------------------------------------------
ElementId faceName(const BrepShape& s, FaceId f);
ElementId edgeName(const BrepShape& s, EdgeId e);
ElementId vertexName(const BrepShape& s, VertexId v);

FaceId   findFace(const BrepShape& s, ElementId id);
EdgeId   findEdge(const BrepShape& s, ElementId id);
VertexId findVertex(const BrepShape& s, ElementId id);

// ---- Construction ----------------------------------------------------------
// A primitive as an exact solid. Returns null for a kind this backend cannot
// build, and for every kind when OCCT is absent.
//
// Faces are named by the part they play -- the top cap, the wall -- using the
// same formula as the mesh generators in src/mesh/primitives.cpp, so a box has
// the same six face names whichever backend built it. That is what would let a
// chain move between backends without its references going stale.
BrepRef primitive(const PrimitiveSpec& spec);

// ---- Modelling -------------------------------------------------------------
// Both carry names across the operation by asking it what became of what --
// Modified, Generated, IsDeleted -- rather than by matching geometry
// afterwards. `salt` identifies the feature, so re-running the same chain names
// the same things; see element_id.h.
//
// Both return null on refusal and put a short phrase in `reason`. Neither
// touches its input: a refused edit cannot half-apply.
BrepRef booleanOp(const BrepShape& a, const BrepShape& b, BooleanOp op,
                  ElementId salt, std::string* reason);

// One radius per edge, parallel to `edges`. Rounding two edges to different
// radii in one operation is a different solid from rounding them in sequence,
// which is why they go together.
BrepRef filletEdges(const BrepShape& s, const std::vector<EdgeId>& edges,
                    const std::vector<Real>& radii, ElementId salt, std::string* reason);

// Every face answering to this name. A name stands for a set: a boolean
// routinely splits one face into several, and a feature that referred to the
// face has to go on referring to all of it.
void findFaces(const BrepShape& s, ElementId id, std::vector<FaceId>& out);

// ---- Display and validity --------------------------------------------------
// `deviation` is the chord tolerance in millimetres; zero asks the backend to
// pick one from the size of the body. Unlike the mesh backend, this is not free
// -- see the Stage 0 measurements -- so callers should cache the result and
// re-tessellate on a geometry change or a large zoom change, not per frame.
void tessellate(const BrepShape& s, RenderMesh& out, Real deviation);
bool validate(const BrepShape& s, std::string* err);
MeshHealth health(const BrepShape& s, bool checkIntersections);

// ---- Mutation --------------------------------------------------------------
// Returns a new shape; the input is untouched, as everywhere else in the
// kernel, so a refused edit cannot half-apply.
BrepRef transformed(const BrepShape& s, const Mat4& m);

} // namespace brep
} // namespace tg
