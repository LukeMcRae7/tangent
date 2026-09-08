// Tangent - the B-rep backend, on OpenCASCADE.
//
// A BrepShape is a TopoDS_Shape plus the two things the seam needs and OCCT
// does not provide: stable handles, and stable names.
//
// Handles come from indexed maps built once when the shape is adopted. They are
// dense and start at zero, which is a coincidence of this implementation and
// nothing above src/geom may rely on it -- an OCCT face index has nothing to do
// with the mesh backend's array index for the same face.
//
// Names come from provenance. A face carries the name it was given or
// inherited; an edge and a vertex are named from the faces that meet at them,
// derived on demand rather than carried, which is the same rule the mesh
// backend uses (see element_id.h) and which gives the right answer for the
// elements a boolean invents.
#include "geom/brep.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepAlgoAPI_BooleanOperation.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeShape.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <GProp_GProps.hxx>
#include <IMeshTools_Parameters.hxx>
#include <Poly_Triangulation.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Trsf.hxx>

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <unordered_map>

namespace tg {

struct BrepShape {
    TopoDS_Shape shape;

    // Handles. Index i in these maps is handle i - 1, so that handles start at
    // zero like every other handle in the program.
    TopTools_IndexedMapOfShape faces, edges, verts;
    TopTools_IndexedDataMapOfShapeListOfShape edgeFaces, vertEdges;

    // Face names, in handle order. Everything else is derived from them.
    std::vector<ElementId> faceNames;

    // Reverse lookups, built with the maps.
    std::unordered_map<ElementId, FaceId>   byFace;
    std::unordered_map<ElementId, EdgeId>   byEdge;
    std::unordered_map<ElementId, VertexId> byVert;
    std::vector<ElementId> edgeNames, vertNames;
};

namespace {

const TopoDS_Face& faceAt(const BrepShape& s, FaceId f) {
    return TopoDS::Face(s.faces(f + 1));
}
const TopoDS_Edge& edgeAt(const BrepShape& s, EdgeId e) {
    return TopoDS::Edge(s.edges(e + 1));
}
const TopoDS_Vertex& vertAt(const BrepShape& s, VertexId v) {
    return TopoDS::Vertex(s.verts(v + 1));
}

Vec3 toVec3(const gp_Pnt& p) {
    return {static_cast<Real>(p.X()), static_cast<Real>(p.Y()), static_cast<Real>(p.Z())};
}

bool validFace(const BrepShape& s, FaceId f)   { return f >= 0 && f < s.faces.Extent(); }
bool validEdge(const BrepShape& s, EdgeId e)   { return e >= 0 && e < s.edges.Extent(); }
bool validVert(const BrepShape& s, VertexId v) { return v >= 0 && v < s.verts.Extent(); }

// An edge is named by the two faces that meet at it, a vertex by the three that
// do. Sorted, so the name does not depend on the order OCCT lists them in.
ElementId derivedName(const std::vector<ElementId>& parents, IdRole role) {
    if (parents.empty()) return kNoId;
    std::vector<ElementId> ps = parents;
    std::sort(ps.begin(), ps.end());
    ElementId id = nameId(0, role, ps[0], ps.size() > 1 ? ps[1] : 0, ps.size());
    for (size_t i = 2; i < ps.size(); ++i) id = nameId(0, role, id, ps[i], i);
    return id;
}

} // namespace

// Built here rather than in the header so that the maps and the names are
// always in step with the shape: there is no way to have one without the other.
BrepRef makeBrep(const TopoDS_Shape& shape, const std::vector<ElementId>& faceNames) {
    auto s = std::make_shared<BrepShape>();
    s->shape = shape;
    TopExp::MapShapes(shape, TopAbs_FACE, s->faces);
    TopExp::MapShapes(shape, TopAbs_EDGE, s->edges);
    TopExp::MapShapes(shape, TopAbs_VERTEX, s->verts);
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, s->edgeFaces);
    TopExp::MapShapesAndAncestors(shape, TopAbs_VERTEX, TopAbs_EDGE, s->vertEdges);

    s->faceNames = faceNames;
    s->faceNames.resize(static_cast<size_t>(s->faces.Extent()), kNoId);

    for (int i = 0; i < s->faces.Extent(); ++i)
        if (s->faceNames[static_cast<size_t>(i)] != kNoId)
            s->byFace.emplace(s->faceNames[static_cast<size_t>(i)], i);

    // Edges, from the faces that meet at them.
    s->edgeNames.assign(static_cast<size_t>(s->edges.Extent()), kNoId);
    for (int i = 0; i < s->edges.Extent(); ++i) {
        std::vector<ElementId> parents;
        if (s->edgeFaces.Contains(s->edges(i + 1))) {
            for (TopTools_ListOfShape::Iterator it(s->edgeFaces.FindFromKey(s->edges(i + 1)));
                 it.More(); it.Next()) {
                const int fi = s->faces.FindIndex(it.Value());
                if (fi > 0) parents.push_back(s->faceNames[static_cast<size_t>(fi - 1)]);
            }
        }
        const ElementId id = derivedName(parents, IdRole::Edge);
        s->edgeNames[static_cast<size_t>(i)] = id;
        if (id != kNoId) s->byEdge.emplace(id, i);
    }

    // Vertices, from the edges that meet at them, which are themselves named
    // from faces -- so a vertex name too is a fact about the surfaces, not
    // about any numbering.
    s->vertNames.assign(static_cast<size_t>(s->verts.Extent()), kNoId);
    for (int i = 0; i < s->verts.Extent(); ++i) {
        std::vector<ElementId> parents;
        if (s->vertEdges.Contains(s->verts(i + 1))) {
            for (TopTools_ListOfShape::Iterator it(s->vertEdges.FindFromKey(s->verts(i + 1)));
                 it.More(); it.Next()) {
                const int ei = s->edges.FindIndex(it.Value());
                if (ei > 0) parents.push_back(s->edgeNames[static_cast<size_t>(ei - 1)]);
            }
        }
        const ElementId id = derivedName(parents, IdRole::Vertex);
        s->vertNames[static_cast<size_t>(i)] = id;
        if (id != kNoId) s->byVert.emplace(id, i);
    }
    return s;
}

const TopoDS_Shape& brepShapeOf(const BrepShape& s) { return s.shape; }

namespace {

// The mesh generators' naming formula, so that the same part of the same
// primitive gets the same name whichever backend built it. See the note in
// SoupBuilder::commit: `topo` is whatever decides how many elements there are,
// and it is folded into the salt so that a reference to an element that no
// longer exists fails loudly instead of landing on a stranger.
ElementId primitiveSalt(PrimitiveKind kind, uint64_t topo) {
    return nameId(static_cast<ElementId>(kind) + 1, IdRole::Vertex, topo);
}
ElementId ordinalFaceName(PrimitiveKind kind, uint64_t topo, uint64_t f) {
    return nameId(primitiveSalt(kind, topo), IdRole::Face, f);
}
ElementId structuralFaceName(PrimitiveKind kind, IdRole role, uint64_t k) {
    const ElementId base = static_cast<ElementId>(kind) + 1;
    return nameId(base, IdRole::Face, nameId(0xCA9E, role, k));
}

// Which of a box's six faces this is, in the order makeBox generates them:
// -Z, +Z, -Y, +X, +Y, -X. Matching that order is what makes the names match.
int boxFaceOrdinal(Vec3 n) {
    if (n.z < -0.9) return 0;
    if (n.z >  0.9) return 1;
    if (n.y < -0.9) return 2;
    if (n.x >  0.9) return 3;
    if (n.y >  0.9) return 4;
    if (n.x < -0.9) return 5;
    return -1;
}

Vec3 outwardNormal(const TopoDS_Face& face) {
    Standard_Real u0 = 0, u1 = 0, v0 = 0, v1 = 0;
    BRepTools::UVBounds(face, u0, u1, v0, v1);
    BRepAdaptor_Surface surf(face);
    gp_Pnt at;
    gp_Vec du, dv;
    surf.D1((u0 + u1) * 0.5, (v0 + v1) * 0.5, at, du, dv);
    gp_Vec n = du.Crossed(dv);
    if (n.SquareMagnitude() < 1e-24) return {0, 0, 0};
    n.Normalize();
    if (face.Orientation() == TopAbs_REVERSED) n.Reverse();
    return {static_cast<Real>(n.X()), static_cast<Real>(n.Y()), static_cast<Real>(n.Z())};
}

// Names for a solid whose faces are told apart by their surface and their
// normal, which is every primitive here.
std::vector<ElementId> namePrimitiveFaces(const TopoDS_Shape& shape, PrimitiveKind kind,
                                          uint64_t topo) {
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(shape, TopAbs_FACE, faces);
    std::vector<ElementId> names(static_cast<size_t>(faces.Extent()), kNoId);

    for (int i = 1; i <= faces.Extent(); ++i) {
        const TopoDS_Face& face = TopoDS::Face(faces(i));
        const Vec3 n = outwardNormal(face);
        const GeomAbs_SurfaceType type = BRepAdaptor_Surface(face).GetType();
        ElementId id = kNoId;

        switch (kind) {
            case PrimitiveKind::Box: {
                const int ord = boxFaceOrdinal(n);
                id = ord >= 0 ? ordinalFaceName(kind, topo, static_cast<uint64_t>(ord))
                              : ordinalFaceName(kind, topo, static_cast<uint64_t>(i));
                break;
            }
            case PrimitiveKind::Cylinder:
            case PrimitiveKind::Cone:
                // The caps carry the same structural names the mesh generators
                // give them, so "the top cap" resolves on either backend. The
                // wall has no mesh counterpart -- there it is a ring of facets,
                // here it is one face -- so it is named for what it is.
                if (type == GeomAbs_Plane)
                    id = structuralFaceName(kind, IdRole::Cap, n.z > 0 ? 1 : 0);
                else
                    id = structuralFaceName(kind, IdRole::Side, 0);
                break;
            case PrimitiveKind::Sphere:
                id = structuralFaceName(kind, IdRole::Side, 0);
                break;
            case PrimitiveKind::Torus:
                id = structuralFaceName(kind, IdRole::Ring, 0);
                break;
            default:
                id = ordinalFaceName(kind, topo, static_cast<uint64_t>(i));
                break;
        }
        names[static_cast<size_t>(i - 1)] = id;
    }

    // A shape whose faces cannot be told apart -- a sphere is one face, but a
    // cone with two identical-looking planes would not be -- must not hand two
    // faces the same name. Fall back to the ordinal for any that collide.
    for (size_t i = 0; i < names.size(); ++i)
        for (size_t j = i + 1; j < names.size(); ++j)
            if (names[i] == names[j])
                names[j] = ordinalFaceName(kind, topo, static_cast<uint64_t>(j + 1));
    return names;
}

} // namespace

namespace brep {

bool available() { return true; }

BrepRef primitive(const PrimitiveSpec& spec) {
    TopoDS_Shape shape;
    uint64_t topo = 0;
    try {
        switch (spec.kind) {
            case PrimitiveKind::Box: {
                const BoxParams& p = spec.box;
                if (p.width <= 0 || p.depth <= 0 || p.height <= 0) return {};
                shape = BRepPrimAPI_MakeBox(
                    gp_Pnt(-p.width / 2, -p.depth / 2, -p.height / 2),
                    p.width, p.depth, p.height).Shape();
                break;
            }
            case PrimitiveKind::Cylinder: {
                const CylinderParams& p = spec.cylinder;
                if (p.radius <= 0 || p.height <= 0) return {};
                shape = BRepPrimAPI_MakeCylinder(
                    gp_Ax2(gp_Pnt(0, 0, -p.height / 2), gp_Dir(0, 0, 1)),
                    p.radius, p.height).Shape();
                break;
            }
            case PrimitiveKind::Sphere: {
                const SphereParams& p = spec.sphere;
                if (p.radius <= 0) return {};
                shape = BRepPrimAPI_MakeSphere(p.radius).Shape();
                break;
            }
            case PrimitiveKind::Cone: {
                const ConeParams& p = spec.cone;
                if (p.height <= 0 || (p.bottomRadius <= 0 && p.topRadius <= 0)) return {};
                shape = BRepPrimAPI_MakeCone(
                    gp_Ax2(gp_Pnt(0, 0, -p.height / 2), gp_Dir(0, 0, 1)),
                    p.bottomRadius, p.topRadius, p.height).Shape();
                break;
            }
            case PrimitiveKind::Torus: {
                const TorusParams& p = spec.torus;
                if (p.majorRadius <= 0 || p.minorRadius <= 0) return {};
                shape = BRepPrimAPI_MakeTorus(p.majorRadius, p.minorRadius).Shape();
                break;
            }
            // A plane is a surface, not a solid, and the operations this
            // backend exists for are all solid operations. The mesh backend
            // keeps it.
            case PrimitiveKind::Plane:
            case PrimitiveKind::Custom:
                return {};
        }
    } catch (const Standard_Failure&) {
        return {};
    }
    if (shape.IsNull()) return {};
    return makeBrep(shape, namePrimitiveFaces(shape, spec.kind, topo));
}

BrepRef clone(const BrepShape& s) { return makeBrep(s.shape, s.faceNames); }

bool empty(const BrepShape& s) { return s.shape.IsNull() || s.faces.Extent() == 0; }
int  faceCount(const BrepShape& s)   { return s.faces.Extent(); }
int  edgeCount(const BrepShape& s)   { return s.edges.Extent(); }
int  vertexCount(const BrepShape& s) { return s.verts.Extent(); }

bool hasFace(const BrepShape& s, FaceId f)   { return validFace(s, f); }
bool hasEdge(const BrepShape& s, EdgeId e)   { return validEdge(s, e); }
bool hasVertex(const BrepShape& s, VertexId v) { return validVert(s, v); }

void allFaces(const BrepShape& s, std::vector<FaceId>& out) {
    out.clear();
    out.reserve(static_cast<size_t>(s.faces.Extent()));
    for (int i = 0; i < s.faces.Extent(); ++i) out.push_back(i);
}
void allEdges(const BrepShape& s, std::vector<EdgeId>& out) {
    out.clear();
    out.reserve(static_cast<size_t>(s.edges.Extent()));
    for (int i = 0; i < s.edges.Extent(); ++i) out.push_back(i);
}
void allVertices(const BrepShape& s, std::vector<VertexId>& out) {
    out.clear();
    out.reserve(static_cast<size_t>(s.verts.Extent()));
    for (int i = 0; i < s.verts.Extent(); ++i) out.push_back(i);
}

void faceEdges(const BrepShape& s, FaceId f, std::vector<EdgeId>& out) {
    out.clear();
    if (!validFace(s, f)) return;
    for (TopExp_Explorer e(faceAt(s, f), TopAbs_EDGE); e.More(); e.Next()) {
        const int i = s.edges.FindIndex(e.Current());
        if (i <= 0) continue;
        const EdgeId id = i - 1;
        if (std::find(out.begin(), out.end(), id) == out.end()) out.push_back(id);
    }
}

void faceVertices(const BrepShape& s, FaceId f, std::vector<VertexId>& out) {
    out.clear();
    if (!validFace(s, f)) return;
    // Around the outer wire, in connection order. A plain explorer would give
    // the vertices in the order the edges were stored, which is not the order
    // they are joined in -- a perimeter measured from that comes out along the
    // diagonals, 96.6mm around a 20mm square instead of 80.
    const TopoDS_Wire outer = BRepTools::OuterWire(faceAt(s, f));
    if (!outer.IsNull()) {
        for (BRepTools_WireExplorer w(outer, faceAt(s, f)); w.More(); w.Next()) {
            const int i = s.verts.FindIndex(w.CurrentVertex());
            if (i <= 0) continue;
            const VertexId id = i - 1;
            if (std::find(out.begin(), out.end(), id) == out.end()) out.push_back(id);
        }
        if (!out.empty()) return;
    }
    // A face with no usable outer wire -- a full sphere, whose boundary is a
    // seam rather than a loop -- still has to answer with something.
    for (TopExp_Explorer v(faceAt(s, f), TopAbs_VERTEX); v.More(); v.Next()) {
        const int i = s.verts.FindIndex(v.Current());
        if (i <= 0) continue;
        const VertexId id = i - 1;
        if (std::find(out.begin(), out.end(), id) == out.end()) out.push_back(id);
    }
}

int faceDegree(const BrepShape& s, FaceId f) {
    std::vector<EdgeId> es;
    faceEdges(s, f, es);
    return static_cast<int>(es.size());
}

void edgeEnds(const BrepShape& s, EdgeId e, VertexId& a, VertexId& b) {
    a = b = kInvalid;
    if (!validEdge(s, e)) return;
    const TopoDS_Edge& ed = edgeAt(s, e);
    const int ia = s.verts.FindIndex(TopExp::FirstVertex(ed));
    const int ib = s.verts.FindIndex(TopExp::LastVertex(ed));
    if (ia > 0) a = ia - 1;
    if (ib > 0) b = ib - 1;
}

void edgeFaces(const BrepShape& s, EdgeId e, FaceId& a, FaceId& b) {
    a = b = kInvalid;
    if (!validEdge(s, e)) return;
    if (!s.edgeFaces.Contains(s.edges(e + 1))) return;
    for (TopTools_ListOfShape::Iterator it(s.edgeFaces.FindFromKey(s.edges(e + 1)));
         it.More(); it.Next()) {
        const int fi = s.faces.FindIndex(it.Value());
        if (fi <= 0) continue;
        if (a == kInvalid) a = fi - 1;
        else if (b == kInvalid && fi - 1 != a) b = fi - 1;
    }
}

void vertexEdges(const BrepShape& s, VertexId v, std::vector<EdgeId>& out) {
    out.clear();
    if (!validVert(s, v)) return;
    if (!s.vertEdges.Contains(s.verts(v + 1))) return;
    for (TopTools_ListOfShape::Iterator it(s.vertEdges.FindFromKey(s.verts(v + 1)));
         it.More(); it.Next()) {
        const int ei = s.edges.FindIndex(it.Value());
        if (ei <= 0) continue;
        const EdgeId id = ei - 1;
        if (std::find(out.begin(), out.end(), id) == out.end()) out.push_back(id);
    }
}

Vec3 faceNormal(const BrepShape& s, FaceId f) {
    if (!validFace(s, f)) return {0, 0, 0};
    const TopoDS_Face& face = faceAt(s, f);
    Standard_Real u0 = 0, u1 = 0, v0 = 0, v1 = 0;
    BRepTools::UVBounds(face, u0, u1, v0, v1);
    BRepAdaptor_Surface surf(face);
    gp_Pnt p;
    gp_Vec du, dv;
    surf.D1((u0 + u1) * 0.5, (v0 + v1) * 0.5, p, du, dv);
    gp_Vec n = du.Crossed(dv);
    if (n.SquareMagnitude() < 1e-24) return {0, 0, 0};
    n.Normalize();
    // The surface normal points out of the solid only if the face agrees with
    // its surface's own sense.
    if (face.Orientation() == TopAbs_REVERSED) n.Reverse();
    return {static_cast<Real>(n.X()), static_cast<Real>(n.Y()), static_cast<Real>(n.Z())};
}

Vec3 faceCentroid(const BrepShape& s, FaceId f) {
    if (!validFace(s, f)) return {0, 0, 0};
    GProp_GProps props;
    BRepGProp::SurfaceProperties(faceAt(s, f), props);
    return toVec3(props.CentreOfMass());
}

Real faceArea(const BrepShape& s, FaceId f) {
    if (!validFace(s, f)) return 0.0;
    GProp_GProps props;
    BRepGProp::SurfaceProperties(faceAt(s, f), props);
    return static_cast<Real>(props.Mass());
}

AABB faceBounds(const BrepShape& s, FaceId f) {
    AABB out;
    if (!validFace(s, f)) return out;
    Bnd_Box bb;
    BRepBndLib::Add(faceAt(s, f), bb);
    // OCCT enlarges a box by the shape's tolerance. Bounds here mean the
    // geometry's own extent, the same as on the mesh backend, so the slack
    // comes back off -- a caller that wants some can add its own.
    bb.SetGap(0.0);
    if (bb.IsVoid()) return out;
    Standard_Real xa, ya, za, xb, yb, zb;
    bb.Get(xa, ya, za, xb, yb, zb);
    out.expand({static_cast<Real>(xa), static_cast<Real>(ya), static_cast<Real>(za)});
    out.expand({static_cast<Real>(xb), static_cast<Real>(yb), static_cast<Real>(zb)});
    return out;
}

Vec3 vertexPosition(const BrepShape& s, VertexId v) {
    if (!validVert(s, v)) return {0, 0, 0};
    return toVec3(BRep_Tool::Pnt(vertAt(s, v)));
}

AABB bounds(const BrepShape& s) {
    AABB out;
    if (s.shape.IsNull()) return out;
    Bnd_Box bb;
    BRepBndLib::Add(s.shape, bb);
    bb.SetGap(0.0);
    if (bb.IsVoid()) return out;
    Standard_Real xa, ya, za, xb, yb, zb;
    bb.Get(xa, ya, za, xb, yb, zb);
    out.expand({static_cast<Real>(xa), static_cast<Real>(ya), static_cast<Real>(za)});
    out.expand({static_cast<Real>(xb), static_cast<Real>(yb), static_cast<Real>(zb)});
    return out;
}

void edgePositions(const BrepShape& s, EdgeId e, Vec3& a, Vec3& b) {
    a = b = Vec3{0, 0, 0};
    if (!validEdge(s, e)) return;
    const TopoDS_Edge& ed = edgeAt(s, e);
    a = toVec3(BRep_Tool::Pnt(TopExp::FirstVertex(ed)));
    b = toVec3(BRep_Tool::Pnt(TopExp::LastVertex(ed)));
}

Vec3 edgeDirection(const BrepShape& s, EdgeId e) {
    if (!validEdge(s, e)) return {0, 0, 0};
    // The tangent at the start, which is exact for a line and honest for a
    // curve -- unlike the chord between the ends, which is neither.
    BRepAdaptor_Curve c(edgeAt(s, e));
    gp_Pnt p;
    gp_Vec d;
    c.D1(c.FirstParameter(), p, d);
    if (d.SquareMagnitude() < 1e-24) {
        Vec3 a, b;
        edgePositions(s, e, a, b);
        return normalize(b - a);
    }
    d.Normalize();
    return {static_cast<Real>(d.X()), static_cast<Real>(d.Y()), static_cast<Real>(d.Z())};
}

ElementId faceName(const BrepShape& s, FaceId f) {
    return validFace(s, f) ? s.faceNames[static_cast<size_t>(f)] : kNoId;
}
ElementId edgeName(const BrepShape& s, EdgeId e) {
    return validEdge(s, e) ? s.edgeNames[static_cast<size_t>(e)] : kNoId;
}
ElementId vertexName(const BrepShape& s, VertexId v) {
    return validVert(s, v) ? s.vertNames[static_cast<size_t>(v)] : kNoId;
}

FaceId findFace(const BrepShape& s, ElementId id) {
    const auto it = s.byFace.find(id);
    return it == s.byFace.end() ? kInvalid : it->second;
}
EdgeId findEdge(const BrepShape& s, ElementId id) {
    const auto it = s.byEdge.find(id);
    return it == s.byEdge.end() ? kInvalid : it->second;
}
VertexId findVertex(const BrepShape& s, ElementId id) {
    const auto it = s.byVert.find(id);
    return it == s.byVert.end() ? kInvalid : it->second;
}

// ---- Carrying names across an operation ------------------------------------
//
// The mechanism the Stage 0 spike established, and the reason the migration has
// a history to stand on. Three questions per element:
//
//   IsDeleted(X)   X is gone
//   Modified(X)    these are what X became
//   Generated(X)   these appeared because of X
//
// Faces are carried. Edges and vertices are not: they are named from the faces
// that meet at them, which makeBrep derives, so an edge a boolean invented gets
// the right name without anyone having to invent one for it.
namespace {

struct NameSource {
    const BrepShape* shape;
};

std::vector<ElementId> propagateNames(BRepBuilderAPI_MakeShape& op,
                                      const std::vector<NameSource>& inputs,
                                      const TopoDS_Shape& after,
                                      ElementId salt) {
    TopTools_IndexedMapOfShape newFaces;
    TopExp::MapShapes(after, TopAbs_FACE, newFaces);
    std::vector<ElementId> names(static_cast<size_t>(newFaces.Extent()), kNoId);

    auto slotOf = [&](const TopoDS_Shape& f) -> int {
        const int i = newFaces.FindIndex(f);
        return i > 0 ? i - 1 : -1;
    };

    // What survived, and what it became. A face split into several keeps its
    // name on every piece: a feature that referred to the face has to go on
    // referring to all of it, which is the decision already taken on the mesh
    // side for a bored face.
    for (const NameSource& in : inputs) {
        if (!in.shape) continue;
        for (int i = 0; i < in.shape->faces.Extent(); ++i) {
            const TopoDS_Shape& f = in.shape->faces(i + 1);
            const ElementId name = in.shape->faceNames[static_cast<size_t>(i)];
            if (name == kNoId || op.IsDeleted(f)) continue;

            bool any = false;
            for (TopTools_ListOfShape::Iterator it(op.Modified(f)); it.More(); it.Next()) {
                const int at = slotOf(it.Value());
                if (at >= 0) { names[static_cast<size_t>(at)] = name; any = true; }
            }
            if (any) continue;
            const int at = slotOf(f);          // untouched, and still there
            if (at >= 0) names[static_cast<size_t>(at)] = name;
        }
    }

    // What the operation made. A wall opened by a cut is generated from the
    // tool's face; a fillet surface is generated from the edge it rounds, which
    // is why edges are asked as well even though they are not carried.
    for (const NameSource& in : inputs) {
        if (!in.shape) continue;
        auto adopt = [&](const TopoDS_Shape& from, ElementId parent, IdRole role) {
            if (parent == kNoId) return;
            for (TopTools_ListOfShape::Iterator it(op.Generated(from)); it.More(); it.Next()) {
                const int at = slotOf(it.Value());
                if (at >= 0 && names[static_cast<size_t>(at)] == kNoId)
                    names[static_cast<size_t>(at)] = nameId(salt, role, parent);
            }
        };
        for (int i = 0; i < in.shape->faces.Extent(); ++i)
            adopt(in.shape->faces(i + 1), in.shape->faceNames[static_cast<size_t>(i)],
                  IdRole::Wall);
        for (int i = 0; i < in.shape->edges.Extent(); ++i)
            adopt(in.shape->edges(i + 1), in.shape->edgeNames[static_cast<size_t>(i)],
                  IdRole::Patch);
    }

    // Anything left has no provenance to name it from -- rare, and worth being
    // able to count. Named from the operation and a deterministic ordinal, by
    // position, so at least a re-run of the same chain agrees with itself.
    std::vector<std::pair<int, gp_Pnt>> leftovers;
    for (int i = 0; i < newFaces.Extent(); ++i) {
        if (names[static_cast<size_t>(i)] != kNoId) continue;
        GProp_GProps props;
        BRepGProp::SurfaceProperties(newFaces(i + 1), props);
        leftovers.push_back({i, props.CentreOfMass()});
    }
    std::sort(leftovers.begin(), leftovers.end(), [](const auto& a, const auto& b) {
        if (std::fabs(a.second.X() - b.second.X()) > 1e-9) return a.second.X() < b.second.X();
        if (std::fabs(a.second.Y() - b.second.Y()) > 1e-9) return a.second.Y() < b.second.Y();
        return a.second.Z() < b.second.Z();
    });
    for (size_t k = 0; k < leftovers.size(); ++k)
        names[static_cast<size_t>(leftovers[k].first)] =
            nameId(salt, IdRole::Split, static_cast<ElementId>(k) + 1);

    return names;
}

// A result worth handing back: not null, has faces, and passes OCCT's own
// check. The alternative is geometry that looks right and is not, which this
// project has already decided it will not ship.
bool acceptable(const TopoDS_Shape& shape, std::string* reason) {
    if (shape.IsNull()) {
        if (reason) *reason = "no shape came out of it";
        return false;
    }
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(shape, TopAbs_FACE, faces);
    if (faces.Extent() == 0) {
        if (reason) *reason = "the result has no faces";
        return false;
    }
    try {
        if (!BRepCheck_Analyzer(shape).IsValid()) {
            if (reason) *reason = "the result is not a valid solid";
            return false;
        }
    } catch (const Standard_Failure& e) {
        if (reason) *reason = e.GetMessageString() ? e.GetMessageString() : "the check threw";
        return false;
    }
    return true;
}

} // namespace

void tessellate(const BrepShape& s, RenderMesh& out, Real deviation) {
    out.clear();
    if (s.shape.IsNull()) return;

    // A chord tolerance tied to the body, when the caller does not name one.
    // Stage 0 measured what these settings cost: the angular limit, not the
    // chord tolerance, is what drives the triangle count on cylinders and
    // fillets, and opening it from the default to 1 radian took a 206-face part
    // from 179 ms to 60.
    Real dev = deviation;
    if (dev <= 0.0) {
        const AABB b = bounds(s);
        const Real diag = length(b.size());
        dev = std::max(diag / 2000.0, Real(1e-4));
    }

    IMeshTools_Parameters p;
    p.Deflection = dev;
    p.Angle = 1.0;
    p.MinSize = dev * 0.1;
    p.InParallel = Standard_True;
    p.ControlSurfaceDeflection = Standard_False;
    p.Relative = Standard_False;

    try {
        // Meshing writes the triangulation into the shape, which is shared with
        // every Body that copied this one -- deliberately, since it is a cache
        // of exactly the thing they would all recompute. It is not thread-safe,
        // and tessellation is called from the frame loop, which is single
        // threaded.
        BRepMesh_IncrementalMesh mesher(const_cast<TopoDS_Shape&>(s.shape), p);
        (void)mesher;
    } catch (const Standard_Failure&) {
        return;
    }

    for (int fi = 0; fi < s.faces.Extent(); ++fi) {
        const TopoDS_Face& face = faceAt(s, fi);
        TopLoc_Location loc;
        const Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) continue;

        const gp_Trsf& trsf = loc.Transformation();
        const bool flipped = face.Orientation() == TopAbs_REVERSED;
        const uint32_t base = static_cast<uint32_t>(out.positions.size());

        // Normals from the surface at each node, not from the triangle it
        // happens to sit in. This is the shading difference the backend is for:
        // a cylinder is smooth because it is a cylinder, and the crease angle
        // the mesh backend has to guess with does not appear.
        BRepAdaptor_Surface surf(face);
        const bool haveUV = tri->HasUVNodes();
        for (int i = 1; i <= tri->NbNodes(); ++i) {
            gp_Pnt pnt = tri->Node(i);
            pnt.Transform(trsf);
            out.positions.push_back(toVec3(pnt));

            Vec3 n{0, 0, 1};
            if (haveUV) {
                const gp_Pnt2d uv = tri->UVNode(i);
                gp_Pnt at;
                gp_Vec du, dv;
                surf.D1(uv.X(), uv.Y(), at, du, dv);
                gp_Vec sn = du.Crossed(dv);
                if (sn.SquareMagnitude() > 1e-24) {
                    sn.Normalize();
                    if (flipped) sn.Reverse();
                    sn.Transform(trsf);
                    n = {static_cast<Real>(sn.X()), static_cast<Real>(sn.Y()),
                         static_cast<Real>(sn.Z())};
                }
            }
            out.normals.push_back(n);
        }

        for (int i = 1; i <= tri->NbTriangles(); ++i) {
            int a = 0, b = 0, c = 0;
            tri->Triangle(i).Get(a, b, c);
            if (flipped) std::swap(b, c);
            out.triangles.push_back(base + static_cast<uint32_t>(a - 1));
            out.triangles.push_back(base + static_cast<uint32_t>(b - 1));
            out.triangles.push_back(base + static_cast<uint32_t>(c - 1));
            out.triangleFace.push_back(fi);
        }
    }

    // Wireframe from the model's own edges. The mesh backend has to decide
    // which facet boundaries are real edges and hide the rest; here every edge
    // is real, which is why isBridgeEdge answers false and Stage 4 can delete
    // the apparatus that guessed.
    for (int ei = 0; ei < s.edges.Extent(); ++ei) {
        const TopoDS_Edge& edge = edgeAt(s, ei);
        if (BRep_Tool::Degenerated(edge)) continue;
        BRepAdaptor_Curve curve(edge);
        GCPnts_QuasiUniformDeflection sampler(curve, dev);
        if (!sampler.IsDone() || sampler.NbPoints() < 2) continue;

        uint32_t prev = 0;
        for (int i = 1; i <= sampler.NbPoints(); ++i) {
            const uint32_t at = static_cast<uint32_t>(out.positions.size());
            out.positions.push_back(toVec3(sampler.Value(i)));
            out.normals.push_back({0, 0, 1});   // unused: lines are not shaded
            if (i > 1) {
                out.edgeLines.push_back(prev);
                out.edgeLines.push_back(at);
            }
            prev = at;
        }
    }
}

bool validate(const BrepShape& s, std::string* err) {
    if (s.shape.IsNull()) {
        if (err) *err = "null shape";
        return false;
    }
    try {
        const BRepCheck_Analyzer check(s.shape);
        if (!check.IsValid()) {
            if (err) *err = "invalid B-rep";
            return false;
        }
    } catch (const Standard_Failure& e) {
        if (err) *err = e.GetMessageString() ? e.GetMessageString() : "check threw";
        return false;
    }
    return true;
}

MeshHealth health(const BrepShape& s, bool) {
    MeshHealth h;
    if (s.shape.IsNull()) return h;
    try {
        GProp_GProps props;
        BRepGProp::VolumeProperties(s.shape, props);
        h.volume = props.Mass();

        int shells = 0;
        for (TopExp_Explorer e(s.shape, TopAbs_SHELL); e.More(); e.Next()) ++shells;
        h.shells = shells;

        // A B-rep solid is closed by construction, so the question a mesh has
        // to answer by counting boundary edges is answered by whether it checks
        // out at all. Self-intersection is not run: it is the expensive mesh
        // test, and -1 is how MeshHealth says "not asked".
        h.watertight = validate(s, nullptr);
        h.boundaryEdges = 0;
        h.degenerateFaces = 0;
        h.selfIntersections = -1;
    } catch (const Standard_Failure&) {
        h.watertight = false;
    }
    return h;
}

BrepRef booleanOp(const BrepShape& a, const BrepShape& b, BooleanOp op,
                  ElementId salt, std::string* reason) {
    if (reason) reason->clear();
    if (a.shape.IsNull() || b.shape.IsNull()) {
        if (reason) *reason = "one of the bodies is empty";
        return {};
    }
    try {
        std::unique_ptr<BRepAlgoAPI_BooleanOperation> algo;
        switch (op) {
            case BooleanOp::Union:        algo = std::make_unique<BRepAlgoAPI_Fuse>(a.shape, b.shape); break;
            case BooleanOp::Difference:   algo = std::make_unique<BRepAlgoAPI_Cut>(a.shape, b.shape); break;
            case BooleanOp::Intersection: algo = std::make_unique<BRepAlgoAPI_Common>(a.shape, b.shape); break;
        }
        if (!algo) return {};
        if (!algo->IsDone() || algo->HasErrors()) {
            if (reason) {
                std::ostringstream os;
                algo->DumpErrors(os);
                *reason = os.str().empty() ? "the boolean did not complete" : os.str();
            }
            return {};
        }
        const TopoDS_Shape result = algo->Shape();
        if (!acceptable(result, reason)) return {};
        return makeBrep(result, propagateNames(*algo, {{&a}, {&b}}, result, salt));
    } catch (const Standard_Failure& e) {
        if (reason) *reason = e.GetMessageString() ? e.GetMessageString() : "the boolean threw";
        return {};
    }
}

BrepRef filletEdges(const BrepShape& s, const std::vector<EdgeId>& edges,
                    const std::vector<Real>& radii, ElementId salt, std::string* reason) {
    if (reason) reason->clear();
    if (edges.empty()) {
        if (reason) *reason = "no edges to round";
        return {};
    }
    try {
        BRepFilletAPI_MakeFillet fil(s.shape);
        int added = 0;
        for (size_t i = 0; i < edges.size(); ++i) {
            if (!validEdge(s, edges[i])) continue;
            const Real r = i < radii.size() ? radii[i] : (radii.empty() ? Real(1.0) : radii.back());
            if (r <= 0.0) continue;
            const TopoDS_Edge& e = edgeAt(s, edges[i]);
            if (BRep_Tool::Degenerated(e)) continue;
            fil.Add(r, e);
            ++added;
        }
        if (added == 0) {
            if (reason) *reason = "none of those edges can be rounded";
            return {};
        }

        fil.Build();
        if (!fil.IsDone()) {
            // Say which part of it failed, rather than "it did not work". A
            // refusal the interface cannot explain is the thing this project
            // decided it will not ship.
            if (reason) {
                std::ostringstream os;
                os << "the fillet could not be built";
                if (fil.NbFaultyContours() > 0)
                    os << " (" << fil.NbFaultyContours() << " of "
                       << fil.NbContours() << " edge chains failed)";
                else if (fil.NbFaultyVertices() > 0)
                    os << " (" << fil.NbFaultyVertices() << " corners failed)";
                *reason = os.str();
            }
            return {};
        }
        const TopoDS_Shape result = fil.Shape();
        if (!acceptable(result, reason)) return {};
        return makeBrep(result, propagateNames(fil, {{&s}}, result, salt));
    } catch (const Standard_Failure& e) {
        if (reason) *reason = e.GetMessageString() ? e.GetMessageString() : "the fillet threw";
        return {};
    }
}

BrepRef extrudeFaces(const BrepRef& s, const std::vector<FaceId>& faces, Real distance,
                     ElementId salt, std::vector<ElementId>* newFaces, std::string* reason) {
    if (newFaces) newFaces->clear();
    if (reason) reason->clear();
    if (!s || faces.empty()) {
        if (reason) *reason = "nothing to extrude";
        return {};
    }
    if (std::fabs(distance) < 1e-9) {
        if (reason) *reason = "the distance is zero";
        return {};
    }

    // Hold the faces by name, not by handle: each step rebuilds the shape and
    // renumbers everything, which is the whole reason names exist.
    std::vector<ElementId> targets;
    for (FaceId f : faces) {
        const ElementId id = faceName(*s, f);
        if (id != kNoId) targets.push_back(id);
    }
    if (targets.empty()) {
        if (reason) *reason = "those faces have no names to follow";
        return {};
    }

    BrepRef current = s;
    for (size_t i = 0; i < targets.size(); ++i) {
        std::vector<FaceId> at;
        findFaces(*current, targets[i], at);
        if (at.empty()) {
            if (reason) *reason = "a face to extrude no longer exists";
            return {};
        }

        // One prism per piece: a face split by an earlier step is still one
        // face as far as the feature is concerned.
        BrepRef step = current;
        for (FaceId f : at) {
            std::vector<FaceId> live;
            findFaces(*step, targets[i], live);
            if (live.empty()) break;
            const FaceId use = f < static_cast<FaceId>(step->faces.Extent()) ? live.front() : live.front();

            const Vec3 n = faceNormal(*step, use);
            if (length(n) < 0.5) {
                if (reason) *reason = "a face has no direction to be pushed along";
                return {};
            }
            const gp_Vec sweep(n.x * distance, n.y * distance, n.z * distance);

            TopoDS_Shape solid;
            try {
                BRepPrimAPI_MakePrism prism(faceAt(*step, use), sweep);
                prism.Build();
                if (!prism.IsDone()) {
                    if (reason) *reason = "the face could not be swept";
                    return {};
                }
                solid = prism.Shape();
            } catch (const Standard_Failure& e) {
                if (reason) *reason = e.GetMessageString() ? e.GetMessageString() : "the sweep threw";
                return {};
            }

            // Name the swept solid from the face it came from, so the boolean
            // that follows has something to carry.
            TopTools_IndexedMapOfShape pf;
            TopExp::MapShapes(solid, TopAbs_FACE, pf);
            std::vector<ElementId> prismNames(static_cast<size_t>(pf.Extent()), kNoId);
            const Vec3 startCentre = faceCentroid(*step, use);
            for (int k = 0; k < pf.Extent(); ++k) {
                const TopoDS_Face& face = TopoDS::Face(pf(k + 1));
                GProp_GProps props;
                BRepGProp::SurfaceProperties(face, props);
                const Vec3 c = toVec3(props.CentreOfMass());
                const Real along = dot(c - startCentre, n);
                // The cap at the far end carries the *original* face's name,
                // because that is what it is: the face the user selected, moved.
                // A feature that referred to it before the extrude has to go on
                // referring to it after, which is the whole job of the history.
                // The near cap vanishes into the body and gets a derived name.
                if (along > std::fabs(distance) * 0.9)
                    prismNames[static_cast<size_t>(k)] = targets[i];
                else if (std::fabs(along) < std::fabs(distance) * 0.1)
                    prismNames[static_cast<size_t>(k)] = nameId(salt, IdRole::Cap, targets[i]);
                else
                    prismNames[static_cast<size_t>(k)] =
                        nameId(salt, IdRole::Wall, targets[i], static_cast<ElementId>(k));
            }
            BrepRef tool = makeBrep(solid, prismNames);

            BrepRef combined = booleanOp(*step, *tool,
                                         distance > 0 ? BooleanOp::Union : BooleanOp::Difference,
                                         nameId(salt, IdRole::Split, targets[i]), reason);
            if (!combined) return {};
            step = combined;
            break;   // the prism spans every piece of that face already
        }
        current = step;
        if (newFaces) newFaces->push_back(targets[i]);
    }
    return current;
}

BrepRef prism(const std::vector<Vec3>& points, const std::vector<Real>& arcs,
              Vec3 planeNormal, Real z0, Real z1, ElementId salt, std::string* reason) {
    if (reason) reason->clear();
    if (points.size() < 3) {
        if (reason) *reason = "the profile has too few points";
        return {};
    }
    if (std::fabs(z1 - z0) < 1e-9) {
        if (reason) *reason = "the profile has no depth";
        return {};
    }

    const Vec3 n = normalize(planeNormal);
    try {
        BRepBuilderAPI_MakeWire wire;
        const size_t count = points.size();
        for (size_t i = 0; i < count; ++i) {
            const Vec3 a = points[i] + n * z0;
            const Vec3 b = points[(i + 1) % count] + n * z0;
            if (length(b - a) < 1e-9) continue;

            const Real bulge = i < arcs.size() ? arcs[i] : Real(0);
            if (std::fabs(bulge) > 1e-9) {
                // Three-point arc: the sagitta is measured from the chord's
                // midpoint, towards the inside of the corner.
                const Vec3 mid = (a + b) * 0.5;
                const Vec3 chord = b - a;
                Vec3 side = cross(n, chord);
                if (length(side) < 1e-12) continue;
                side = normalize(side);
                const Vec3 through = mid + side * bulge;
                wire.Add(BRepBuilderAPI_MakeEdge(
                    GC_MakeArcOfCircle(gp_Pnt(a.x, a.y, a.z), gp_Pnt(through.x, through.y, through.z),
                                       gp_Pnt(b.x, b.y, b.z)).Value()).Edge());
            } else {
                wire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(a.x, a.y, a.z),
                                                 gp_Pnt(b.x, b.y, b.z)).Edge());
            }
        }
        wire.Build();
        if (!wire.IsDone()) {
            if (reason) *reason = "the profile does not close";
            return {};
        }

        BRepBuilderAPI_MakeFace face(wire.Wire(), Standard_True);
        face.Build();
        if (!face.IsDone()) {
            if (reason) *reason = "the profile does not bound a face";
            return {};
        }

        const Real depth = z1 - z0;
        const gp_Vec sweep(n.x * depth, n.y * depth, n.z * depth);
        BRepPrimAPI_MakePrism solid(face.Face(), sweep);
        solid.Build();
        if (!solid.IsDone() || !acceptable(solid.Shape(), reason)) {
            if (reason && reason->empty()) *reason = "the profile could not be swept into a solid";
            return {};
        }

        // Named by role, the way a primitive is: the two caps and the wall the
        // profile swept out. The wall is several faces -- one per span of the
        // profile -- so each takes the ordinal of the span it came from.
        const TopoDS_Shape shape = solid.Shape();
        TopTools_IndexedMapOfShape fs;
        TopExp::MapShapes(shape, TopAbs_FACE, fs);
        std::vector<ElementId> names(static_cast<size_t>(fs.Extent()), kNoId);
        std::vector<std::pair<Real, int>> byAngle;
        for (int i = 0; i < fs.Extent(); ++i) {
            const TopoDS_Face& f = TopoDS::Face(fs(i + 1));
            const Vec3 fn = outwardNormal(f);
            const Real along = dot(fn, n);
            if (along > 0.9)       names[static_cast<size_t>(i)] = nameId(salt, IdRole::Cap, 1);
            else if (along < -0.9) names[static_cast<size_t>(i)] = nameId(salt, IdRole::Cap, 0);
            else {
                GProp_GProps props;
                BRepGProp::SurfaceProperties(f, props);
                const Vec3 c = toVec3(props.CentreOfMass());
                byAngle.push_back({std::atan2(c.y, c.x), i});
            }
        }
        // The walls in a fixed order around the profile, so that the same
        // profile always names them the same way.
        std::sort(byAngle.begin(), byAngle.end());
        for (size_t k = 0; k < byAngle.size(); ++k)
            names[static_cast<size_t>(byAngle[k].second)] =
                nameId(salt, IdRole::Side, static_cast<ElementId>(k));

        return makeBrep(shape, names);
    } catch (const Standard_Failure& e) {
        if (reason) *reason = e.GetMessageString() ? e.GetMessageString() : "the profile threw";
        return {};
    }
}

bool encode(const BrepShape& s, std::string& shapeOut, std::vector<ElementId>& namesOut) {
    if (s.shape.IsNull()) return false;
    try {
        std::ostringstream os;
        BRepTools::Write(s.shape, os);
        shapeOut = os.str();
    } catch (const Standard_Failure&) {
        return false;
    }
    namesOut = s.faceNames;
    return !shapeOut.empty();
}

BrepRef decode(const std::string& shapeText, const std::vector<ElementId>& names) {
    if (shapeText.empty()) return {};
    try {
        TopoDS_Shape shape;
        BRep_Builder builder;
        std::istringstream is(shapeText);
        BRepTools::Read(shape, is, builder);
        if (shape.IsNull()) return {};
        // The names are matched to faces by index, which is safe because OCCT
        // reads its own text back in the order it wrote it. If a file ever
        // arrives with the wrong count, makeBrep pads with kNoId rather than
        // reading past the end, and the faces without names simply cannot be
        // referred to -- which is a visible failure, not a silent one.
        return makeBrep(shape, names);
    } catch (const Standard_Failure&) {
        return {};
    }
}

void findFaces(const BrepShape& s, ElementId id, std::vector<FaceId>& out) {
    out.clear();
    if (id == kNoId) return;
    for (int i = 0; i < s.faces.Extent(); ++i)
        if (s.faceNames[static_cast<size_t>(i)] == id) out.push_back(i);
}

BrepRef transformed(const BrepShape& s, const Mat4& m) {
    gp_Trsf t;
    // A rigid placement: the seam promises this is exact, so only the rotation
    // and the translation are taken. A scale would make a cylinder into
    // something that is not one, and belongs in an operation, not here.
    t.SetValues(m.col[0].x, m.col[1].x, m.col[2].x, m.col[3].x,
                m.col[0].y, m.col[1].y, m.col[2].y, m.col[3].y,
                m.col[0].z, m.col[1].z, m.col[2].z, m.col[3].z);
    try {
        BRepBuilderAPI_Transform xf(s.shape, t, Standard_True);
        if (!xf.IsDone()) return {};
        return makeBrep(xf.Shape(), s.faceNames);
    } catch (const Standard_Failure&) {
        return {};
    }
}

} // namespace brep
} // namespace tg
