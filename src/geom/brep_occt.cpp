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
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepTools.hxx>
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
    // Around the outer wire, in order: a caller drawing an outline needs the
    // sequence, not the set.
    const TopoDS_Wire outer = BRepTools::OuterWire(faceAt(s, f));
    for (TopExp_Explorer v(outer.IsNull() ? TopoDS_Shape(faceAt(s, f)) : TopoDS_Shape(outer),
                           TopAbs_VERTEX); v.More(); v.Next()) {
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
