// Tangent - the B-rep backend when the project is built without OpenCASCADE.
//
// Every answer here is the empty one. Nothing constructs a BrepShape in this
// build, so none of these can actually be reached through a body; they exist so
// that body.cpp compiles and links identically either way, and so that Body has
// one layout across the whole program rather than one per translation unit.
#include "geom/brep.h"

namespace tg {

// Never defined in this build: no one can make one, so no one can ask about one.
struct BrepShape {};

namespace brep {

bool available() { return false; }

BrepRef clone(const BrepShape&) { return {}; }

BrepRef primitive(const PrimitiveSpec&) { return {}; }

BrepRef booleanOp(const BrepShape&, const BrepShape&, BooleanOp, ElementId, std::string* reason) {
    if (reason) *reason = "built without OpenCASCADE";
    return {};
}
BrepRef filletEdges(const BrepShape&, const std::vector<EdgeId>&, const std::vector<Real>&,
                    ElementId, std::string* reason) {
    if (reason) *reason = "built without OpenCASCADE";
    return {};
}
BrepRef extrudeFaces(const BrepRef&, const std::vector<FaceId>&, Real, ElementId,
                     std::vector<ElementId>*, std::string* reason) {
    if (reason) *reason = "built without OpenCASCADE";
    return {};
}
BrepRef prism(const std::vector<Vec3>&, const std::vector<Real>&, Vec3, Real, Real,
              ElementId, std::string* reason) {
    if (reason) *reason = "built without OpenCASCADE";
    return {};
}
bool encode(const BrepShape&, std::string&, std::vector<ElementId>&) { return false; }
BrepRef decode(const std::string&, const std::vector<ElementId>&) { return {}; }
void findFaces(const BrepShape&, ElementId, std::vector<FaceId>& out) { out.clear(); }

bool empty(const BrepShape&) { return true; }
int  faceCount(const BrepShape&) { return 0; }
int  edgeCount(const BrepShape&) { return 0; }
int  vertexCount(const BrepShape&) { return 0; }

bool hasFace(const BrepShape&, FaceId) { return false; }
bool hasEdge(const BrepShape&, EdgeId) { return false; }
bool hasVertex(const BrepShape&, VertexId) { return false; }

void allFaces(const BrepShape&, std::vector<FaceId>& out) { out.clear(); }
void allEdges(const BrepShape&, std::vector<EdgeId>& out) { out.clear(); }
void allVertices(const BrepShape&, std::vector<VertexId>& out) { out.clear(); }

void faceEdges(const BrepShape&, FaceId, std::vector<EdgeId>& out) { out.clear(); }
void faceVertices(const BrepShape&, FaceId, std::vector<VertexId>& out) { out.clear(); }
int  faceDegree(const BrepShape&, FaceId) { return 0; }

void edgeEnds(const BrepShape&, EdgeId, VertexId& a, VertexId& b) { a = kInvalid; b = kInvalid; }
void edgeFaces(const BrepShape&, EdgeId, FaceId& a, FaceId& b) { a = kInvalid; b = kInvalid; }
void vertexEdges(const BrepShape&, VertexId, std::vector<EdgeId>& out) { out.clear(); }

Vec3 faceNormal(const BrepShape&, FaceId) { return {0, 0, 0}; }
Vec3 faceCentroid(const BrepShape&, FaceId) { return {0, 0, 0}; }
Real faceArea(const BrepShape&, FaceId) { return 0.0; }
AABB faceBounds(const BrepShape&, FaceId) { return {}; }
Vec3 vertexPosition(const BrepShape&, VertexId) { return {0, 0, 0}; }
AABB bounds(const BrepShape&) { return {}; }
void edgePositions(const BrepShape&, EdgeId, Vec3& a, Vec3& b) { a = {0, 0, 0}; b = {0, 0, 0}; }
Vec3 edgeDirection(const BrepShape&, EdgeId) { return {0, 0, 0}; }

ElementId faceName(const BrepShape&, FaceId) { return kNoId; }
ElementId edgeName(const BrepShape&, EdgeId) { return kNoId; }
ElementId vertexName(const BrepShape&, VertexId) { return kNoId; }

FaceId   findFace(const BrepShape&, ElementId) { return kInvalid; }
EdgeId   findEdge(const BrepShape&, ElementId) { return kInvalid; }
VertexId findVertex(const BrepShape&, ElementId) { return kInvalid; }

void tessellate(const BrepShape&, RenderMesh& out, Real) { out.clear(); }
bool validate(const BrepShape&, std::string* err) {
    if (err) *err = "built without OpenCASCADE";
    return false;
}
MeshHealth health(const BrepShape&, bool) { return {}; }

BrepRef transformed(const BrepShape&, const Mat4&) { return {}; }

} // namespace brep
} // namespace tg
