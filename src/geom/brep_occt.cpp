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
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepTools_History.hxx>
#include <BRepFeat_SplitShape.hxx>
#include <BRepOffsetAPI_DraftAngle.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRepOffsetAPI_MakeOffsetShape.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
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

BrepRef detach(const BrepShape& s) {
    if (s.shape.IsNull()) return {};
    try {
        // Without the triangulation: the copy exists to be computed on, not
        // drawn, and carrying a mesh it will not use is the one cost worth
        // avoiding here.
        BRepBuilderAPI_Copy copier(s.shape, Standard_False);
        if (!copier.IsDone()) return clone(s);
        const TopoDS_Shape out = copier.Shape();

        // The copy is isomorphic to the original, so mapping its faces walks
        // them in the same order and the names line up by index. tessellate
        // already relies on this for the same reason.
        TopTools_IndexedMapOfShape fs;
        TopExp::MapShapes(out, TopAbs_FACE, fs);
        if (fs.Extent() != s.faces.Extent()) return clone(s);
        return makeBrep(out, s.faceNames);
    } catch (const Standard_Failure&) {
        return clone(s);
    }
}

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

SurfaceKind faceKind(const BrepShape& s, FaceId f) {
    if (!validFace(s, f)) return SurfaceKind::Freeform;
    switch (BRepAdaptor_Surface(faceAt(s, f)).GetType()) {
        case GeomAbs_Plane:    return SurfaceKind::Plane;
        case GeomAbs_Cylinder: return SurfaceKind::Cylinder;
        case GeomAbs_Cone:     return SurfaceKind::Cone;
        case GeomAbs_Sphere:   return SurfaceKind::Sphere;
        case GeomAbs_Torus:    return SurfaceKind::Torus;
        default:               return SurfaceKind::Freeform;
    }
}

CurveKind edgeKind(const BrepShape& s, EdgeId e) {
    if (!validEdge(s, e)) return CurveKind::Freeform;
    switch (BRepAdaptor_Curve(edgeAt(s, e)).GetType()) {
        case GeomAbs_Line:    return CurveKind::Line;
        case GeomAbs_Circle:  return CurveKind::Circle;
        case GeomAbs_Ellipse: return CurveKind::Ellipse;
        default:              return CurveKind::Freeform;
    }
}

bool edgeCircle(const BrepShape& s, EdgeId e, Vec3& centre, Vec3& axis, Real& radius) {
    if (!validEdge(s, e)) return false;
    BRepAdaptor_Curve c(edgeAt(s, e));
    if (c.GetType() != GeomAbs_Circle) return false;
    const gp_Circ circ = c.Circle();
    centre = toVec3(circ.Location());
    const gp_Dir d = circ.Axis().Direction();
    axis = {static_cast<Real>(d.X()), static_cast<Real>(d.Y()), static_cast<Real>(d.Z())};
    radius = static_cast<Real>(circ.Radius());
    return true;
}

bool faceCylinder(const BrepShape& s, FaceId f, Vec3& point, Vec3& axis, Real& radius) {
    if (!validFace(s, f)) return false;
    BRepAdaptor_Surface surf(faceAt(s, f));
    if (surf.GetType() != GeomAbs_Cylinder) return false;
    const gp_Cylinder cyl = surf.Cylinder();
    point = toVec3(cyl.Location());
    const gp_Dir d = cyl.Axis().Direction();
    axis = {static_cast<Real>(d.X()), static_cast<Real>(d.Y()), static_cast<Real>(d.Z())};
    radius = static_cast<Real>(cyl.Radius());
    return true;
}

Real edgeLength(const BrepShape& s, EdgeId e) {
    if (!validEdge(s, e)) return 0.0;
    // Along the curve. The chord between the ends is not the same number, and
    // for a full circle it is zero.
    GProp_GProps props;
    BRepGProp::LinearProperties(edgeAt(s, e), props);
    return static_cast<Real>(props.Mass());
}

Vec3 edgeMidpoint(const BrepShape& s, EdgeId e) {
    if (!validEdge(s, e)) return {0, 0, 0};
    BRepAdaptor_Curve c(edgeAt(s, e));
    return toVec3(c.Value((c.FirstParameter() + c.LastParameter()) * 0.5));
}

void edgePolyline(const BrepShape& s, EdgeId e, Real deviationMm,
                  std::vector<Vec3>& out) {
    out.clear();
    if (!validEdge(s, e)) return;
    const TopoDS_Edge& ed = edgeAt(s, e);
    if (BRep_Tool::Degenerated(ed)) return;

    // A line is already the whole of itself, and every selected edge asks this
    // once a frame -- no reason to run a deflection solve to rediscover two ends.
    if (edgeKind(s, e) == CurveKind::Line) {
        Vec3 a, b;
        edgePositions(s, e, a, b);
        out.push_back(a);
        out.push_back(b);
        return;
    }

    Real dev = deviationMm;
    if (!(dev > 0.0)) {
        // A hundredth of the curve's own length, so the choice scales with the
        // edge rather than with whatever units the part happens to be in.
        const Real len = edgeLength(s, e);
        dev = len > 0.0 ? len * 0.01 : 0.01;
    }

    BRepAdaptor_Curve c(ed);
    GCPnts_QuasiUniformDeflection sampler(c, dev);
    if (!sampler.IsDone() || sampler.NbPoints() < 2) {
        // Refusing would leave the caller with nothing to draw; the chord is
        // wrong but visible, and a curve this sampler cannot walk is a bug
        // worth seeing rather than a silently missing highlight.
        Vec3 a, b;
        edgePositions(s, e, a, b);
        out.push_back(a);
        out.push_back(b);
        return;
    }
    out.reserve(static_cast<size_t>(sampler.NbPoints()));
    for (int i = 1; i <= sampler.NbPoints(); ++i) out.push_back(toVec3(sampler.Value(i)));
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
// What to tell the user when the kernel throws.
//
// OpenCASCADE's exception text is written for whoever is debugging OpenCASCADE:
// "NCollection_Sequence::Value" and "BRepAlgoAPI::Build() failed" name the line
// it gave up on, not anything the person at the screen did or could do
// differently. Putting that in front of them was worse than saying nothing --
// it looks like an answer and is not one.
//
// So the operation says what it could not do, in its own words, and the raw
// text goes to the log where it is useful.
std::string kernelReason(const Standard_Failure& e, const char* said) {
    const char* raw = e.GetMessageString();
    std::fprintf(stderr, "[kernel] %s (%s)\n", said, raw && *raw ? raw : "no detail");
    return said;
}

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
        if (reason) *reason = kernelReason(e, "the body could not be checked");
        return false;
    }
    return true;
}

} // namespace

void tessellate(const BrepShape& s, RenderMesh& out, TessellationQuality q) {
    out.clear();
    if (s.shape.IsNull()) return;

    // A chord tolerance tied to the body, when the caller does not name one.
    // Stage 0 measured what these settings cost: the angular limit, not the
    // chord tolerance, is what drives the triangle count on cylinders and
    // fillets, and opening it from the default to 1 radian took a 206-face part
    // from 179 ms to 60.
    Real dev = q.deviationMm;
    if (dev <= 0.0) {
        const AABB b = bounds(s);
        const Real diag = length(b.size());
        dev = std::max(diag / 2000.0, Real(1e-4));
    }

    IMeshTools_Parameters p;
    p.Deflection = dev;
    // Without this, a request for a *coarser* mesh than the one already on the
    // shape is silently ignored -- an export at 0.2mm quietly writes the
    // screen's 0.017mm, and pulling the camera back never gives the triangles
    // up. Asking for the same tolerance twice still costs nothing, which is
    // what keeps a redraw cheap.
    p.AllowQualityDecrease = Standard_True;
    // The screen's default is coarse because it is paid every zoom step; an
    // export names its own and means it, so the chord tolerance is what binds
    // rather than the angle.
    p.Angle = q.angleRad > 0.0 ? q.angleRad : 1.0;
    p.MinSize = dev * 0.1;
    p.InParallel = Standard_True;
    p.ControlSurfaceDeflection = Standard_False;
    p.Relative = Standard_False;

    // Meshing writes the triangulation into the shape, which is shared with
    // every Body that copied this one -- deliberately, since it is a cache of
    // exactly the thing they would all recompute. It is not thread-safe, and
    // tessellation is called from the frame loop, which is single threaded.
    //
    // A caller that wants an answer of its own gets a copy to write into, so
    // the screen keeps the mesh it had.
    TopoDS_Shape target = s.shape;
    TopTools_IndexedMapOfShape targetFaces;
    try {
        if (q.independent) {
            BRepBuilderAPI_Copy copier(s.shape, Standard_False);
            if (copier.IsDone()) target = copier.Shape();
        }
        BRepMesh_IncrementalMesh mesher(target, p);
        (void)mesher;
    } catch (const Standard_Failure&) {
        return;
    }
    TopExp::MapShapes(target, TopAbs_FACE, targetFaces);
    if (targetFaces.Extent() != s.faces.Extent()) return;

    for (int fi = 0; fi < s.faces.Extent(); ++fi) {
        const TopoDS_Face& face = TopoDS::Face(targetFaces(fi + 1));
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

            // A triangulation carries a few triangles of no area, where a
            // surface closes on itself or comes to a point. They draw as
            // nothing at best and as a stray speck at worst, and a ray that
            // hits one would resolve to a face the user cannot see, so they do
            // not go into the render mesh at all.
            const Vec3 pa = out.positions[base + static_cast<size_t>(a - 1)];
            const Vec3 pb = out.positions[base + static_cast<size_t>(b - 1)];
            const Vec3 pc = out.positions[base + static_cast<size_t>(c - 1)];
            if (length(cross(pb - pa, pc - pa)) < 1e-12) continue;

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

        // A seam is where a closed surface is cut open so it can be given a
        // rectangular parameter space -- a cylinder has one running down its
        // side, a hole has one across it. It is a fact about the parameterisation
        // and not about the part, and drawing it puts a line across a face that
        // does not turn there. The same objection the mesh backend's bridge
        // edges answered, arriving from the other direction.
        bool seam = false;
        if (s.edgeFaces.Contains(s.edges(ei + 1))) {
            for (TopTools_ListOfShape::Iterator it(s.edgeFaces.FindFromKey(s.edges(ei + 1)));
                 it.More() && !seam; it.Next())
                seam = BRep_Tool::IsClosed(edge, TopoDS::Face(it.Value()));
        }
        if (seam) continue;
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
        if (err) *err = kernelReason(e, "the body could not be checked");
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

        // Solids, not shells. A hollowed part has two shells -- its outside and
        // the wall of its cavity -- and reporting that as two bodies tells the
        // user they have something they do not.
        int solids = 0;
        for (TopExp_Explorer e(s.shape, TopAbs_SOLID); e.More(); e.Next()) ++solids;
        h.shells = solids;

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
        if (reason) *reason = kernelReason(e, "the two bodies could not be combined");
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
        if (reason) *reason = kernelReason(e, "that radius will not round these edges");
        return {};
    }
}

// Merges faces that a combine left split but that lie on one surface, and works
// out what the survivor should be called.
//
// Pushing a face out unions a prism onto the body, and the prism's walls are
// flush with the walls they slide along: the same plane, in two pieces, with a
// seam drawn down it where nothing intersects. That is the oldest complaint
// against this tool, and on a push it compounds -- push twice and the side is
// in three pieces.
//
// The naming is the only reason this is not a one-liner. Two faces going into
// one leaves two names for one thing, and a feature that referred to either has
// to keep resolving. The rule is that the face that already existed wins: it is
// the one the user has been pointing at and the one the history refers to, and
// the newly generated strip is the part with no past.
BrepRef unifyFlush(const BrepRef& made, const BrepRef& before, ElementId salt) {
    if (!made || made->shape.IsNull()) return made;

    TopoDS_Shape merged;
    Handle(BRepTools_History) history;
    try {
        ShapeUpgrade_UnifySameDomain unify(made->shape, Standard_True, Standard_True,
                                           Standard_False);

        // Only the seams this operation left.
        //
        // UnifySameDomain merges every coplanar pair in the shape, which is far
        // more than was asked for: a line the user cut on purpose divides two
        // coplanar faces too, and merging it destroys their work to tidy up
        // after ours. An edge between two faces that both existed before this
        // operation is one of those, and is kept.
        std::unordered_map<ElementId, bool> existed;
        auto wasThereBefore = [&](ElementId id) {
            if (id == kNoId) return false;
            auto it = existed.find(id);
            if (it != existed.end()) return it->second;
            std::vector<FaceId> at;
            findFaces(*before, id, at);
            const bool yes = !at.empty();
            existed.emplace(id, yes);
            return yes;
        };

        TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
        TopExp::MapShapesAndAncestors(made->shape, TopAbs_EDGE, TopAbs_FACE, edgeFaces);
        for (int i = 1; i <= edgeFaces.Extent(); ++i) {
            const TopTools_ListOfShape& adj = edgeFaces(i);
            if (adj.Extent() != 2) continue;
            bool bothOld = true;
            for (TopTools_ListIteratorOfListOfShape it(adj); it.More() && bothOld; it.Next()) {
                const int at = made->faces.FindIndex(it.Value());
                bothOld = at > 0 &&
                          wasThereBefore(made->faceNames[static_cast<size_t>(at - 1)]);
            }
            if (bothOld) unify.KeepShape(edgeFaces.FindKey(i));
        }

        unify.Build();
        merged = unify.Shape();
        history = unify.History();
    } catch (const Standard_Failure&) {
        return made;                 // more faces than it needs is only a blemish
    }
    if (merged.IsNull() || history.IsNull() || !acceptable(merged, nullptr)) return made;

    TopTools_IndexedMapOfShape out;
    TopExp::MapShapes(merged, TopAbs_FACE, out);
    if (out.Extent() >= made->faces.Extent()) return made;   // nothing was merged

    std::vector<ElementId> names(static_cast<size_t>(out.Extent()), kNoId);
    std::vector<bool> settled(static_cast<size_t>(out.Extent()), false);

    for (int i = 1; i <= made->faces.Extent(); ++i) {
        const ElementId id = made->faceNames[static_cast<size_t>(i - 1)];
        if (id == kNoId) continue;

        // Where this face ended up: itself if it survived untouched, or the
        // face it was merged into.
        std::vector<TopoDS_Shape> landed;
        const TopTools_ListOfShape& mods = history->Modified(made->faces(i));
        if (mods.IsEmpty()) landed.push_back(made->faces(i));
        else for (TopTools_ListIteratorOfListOfShape it(mods); it.More(); it.Next())
            landed.push_back(it.Value());

        // Did this name exist before the operation? If so it is the one to keep.
        std::vector<FaceId> was;
        findFaces(*before, id, was);
        const bool existed = !was.empty();

        for (const TopoDS_Shape& f : landed) {
            const int at = out.FindIndex(f);
            if (at <= 0) continue;
            const size_t k = static_cast<size_t>(at - 1);
            if (settled[k]) continue;
            names[k] = id;
            if (existed) settled[k] = true;       // nothing may take it from here
        }
    }

    // Anything the merge invented outright still needs a name of its own.
    for (size_t k = 0; k < names.size(); ++k)
        if (names[k] == kNoId)
            names[k] = nameId(salt, IdRole::Patch, static_cast<ElementId>(k));

    return makeBrep(merged, names);
}

BrepRef extrudeFaces(const BrepRef& s, const std::vector<FaceId>& faces, Real distance,
                     ElementId salt, std::vector<ElementId>* newFaces, std::string* reason,
                     bool mergeFlush, Vec3 along) {
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
            // Along the face's own normal unless the caller named a
            // direction. A sweep square to the normal moves the face's plane
            // nowhere, so there is nothing to build and it says so.
            Vec3 push = n;
            if (lengthSq(along) > 1e-12) {
                push = normalize(along);
                if (std::fabs(dot(push, n)) < 1e-3) {
                    if (reason) *reason = "that direction runs along the face, not into it";
                    return {};
                }
            }
            const gp_Vec sweep(push.x * distance, push.y * distance, push.z * distance);

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
                if (reason) *reason = kernelReason(e, "the face could not be swept that far");
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
            step = mergeFlush
                       ? unifyFlush(combined, step, nameId(salt, IdRole::Patch, targets[i]))
                       : combined;
            break;   // the prism spans every piece of that face already
        }
        current = step;
        if (newFaces) newFaces->push_back(targets[i]);
    }
    return current;
}

BrepRef rotateFaces(const BrepRef& s, const std::vector<FaceId>& faces, Real angleRad,
                    Vec3 hingePoint, Vec3 hingeDir, ElementId salt, std::string* reason) {
    if (reason) reason->clear();
    if (!s || s->shape.IsNull() || faces.empty()) {
        if (reason) *reason = "nothing to rotate";
        return {};
    }
    if (std::fabs(angleRad) < 1e-9) {
        if (reason) *reason = "the angle is zero";
        return {};
    }
    if (lengthSq(hingeDir) < 1e-18) {
        if (reason) *reason = "there is no edge to pivot about";
        return {};
    }

    try {
        BRepOffsetAPI_DraftAngle draft(s->shape);
        const gp_Dir along(hingeDir.x, hingeDir.y, hingeDir.z);
        bool any = false;

        for (FaceId f : faces) {
            if (!validFace(*s, f)) {
                if (reason) *reason = "a face to rotate no longer exists";
                return {};
            }
            const TopoDS_Face face = TopoDS::Face(s->faces(static_cast<int>(f) + 1));
            const Vec3 n = faceNormal(*s, f);
            if (lengthSq(n) < 1e-18) continue;

            // The neutral plane is the one that cuts this face along the hinge:
            // it contains the hinge line and stands perpendicular to the face,
            // so the two planes meet in exactly that line and nowhere else.
            const Vec3 across = cross(hingeDir, n);
            if (lengthSq(across) < 1e-12) {
                if (reason) *reason = "the pivot lies flat in the face";
                return {};
            }
            const gp_Pln neutral(gp_Pnt(hingePoint.x, hingePoint.y, hingePoint.z),
                                 gp_Dir(across.x, across.y, across.z));

            // The angle a draft takes is the one between the face and the pull
            // direction, not the one the face turns through. Pulling along the
            // face's own normal therefore asks for a face at `angle` to its
            // normal -- twelve degrees requested, seventy-eight delivered.
            //
            // A direction lying in the face and square to the hinge starts at
            // zero, so asking for `angle` gets exactly `angle` of turn.
            const Vec3 inPlane = normalize(cross(n, hingeDir));
            draft.Add(face, gp_Dir(inPlane.x, inPlane.y, inPlane.z),
                      static_cast<Standard_Real>(angleRad), neutral);
            if (!draft.AddDone()) {
                if (reason) *reason = "that face will not take a rotation about this edge";
                return {};
            }
            any = true;
        }
        if (!any) {
            if (reason) *reason = "no face could be rotated";
            return {};
        }

        draft.Build();
        if (!draft.IsDone()) {
            if (reason) *reason = "the rotation could not be built";
            return {};
        }
        const TopoDS_Shape result = draft.Shape();
        if (!acceptable(result, reason)) return {};
        return makeBrep(result, propagateNames(draft, {{s.get()}}, result, salt));
    } catch (const Standard_Failure& e) {
        if (reason) *reason = kernelReason(e, "that face will not turn about this edge");
        return {};
    }
}

BrepRef scaleFaces(const BrepRef& s, const std::vector<FaceId>& faces, Real factor,
                   ElementId salt, std::string* reason) {
    if (reason) reason->clear();
    if (!s || s->shape.IsNull() || faces.empty()) {
        if (reason) *reason = "nothing to scale";
        return {};
    }
    if (std::fabs(factor - 1.0) < 1e-9) {
        if (reason) *reason = "that is the size it already is";
        return {};
    }
    if (factor <= 0.0) {
        if (reason) *reason = "a face cannot be scaled to nothing";
        return {};
    }

    try {
        BRepOffsetAPI_DraftAngle draft(s->shape);
        int tilted = 0;

        for (FaceId f : faces) {
            if (!validFace(*s, f)) {
                if (reason) *reason = "a face to scale no longer exists";
                return {};
            }
            const Vec3 n = faceNormal(*s, f);
            const Vec3 centre = faceCentroid(*s, f);
            if (lengthSq(n) < 1e-18) continue;

            // How far the body reaches back from this face. Every neighbour
            // pivots about its own far end, and that is where the plane holding
            // them still has to sit.
            Real deepest = 0.0;
            {
                TopTools_IndexedMapOfShape vs;
                TopExp::MapShapes(s->shape, TopAbs_VERTEX, vs);
                for (int i = 1; i <= vs.Extent(); ++i) {
                    const gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(vs(i)));
                    deepest = std::max(deepest, -dot(Vec3{p.X(), p.Y(), p.Z()} - centre, n));
                }
            }
            if (deepest < 1e-6) {
                if (reason) *reason = "there is no depth behind that face to taper";
                return {};
            }

            std::vector<EdgeId> es;
            faceEdges(*s, f, es);
            for (EdgeId e : es) {
                FaceId a = kInvalid, b = kInvalid;
                edgeFaces(*s, e, a, b);
                const FaceId side = a == f ? b : a;
                if (side == kInvalid || !validFace(*s, side)) continue;

                const Vec3 sn = faceNormal(*s, side);
                // The pull has to lie in the neighbour's own plane, or the
                // angle is measured from somewhere the face is not: see
                // rotateFaces, where the same mistake cost sixty-six degrees.
                Vec3 pull = n - sn * dot(n, sn);
                if (lengthSq(pull) < 1e-12) continue;      // parallel to the face
                pull = normalize(pull);

                // How far this edge has to travel: proportional to how far out
                // it already is, which is what makes it a scale.
                const Vec3 mid = edgeMidpoint(*s, e);
                Vec3 out = mid - centre;
                out = out - n * dot(out, n);               // in the face's plane
                const Real reach = length(out);
                if (reach < 1e-9) continue;
                const Real travel = (factor - 1.0) * reach;

                // Outward is whichever way the neighbour faces. Negated
                // because a positive draft leans a face inward, so growing a
                // face is the negative angle: measured on a box, 1.5 came back
                // as 0.5 until this was the other way round.
                const Real sense = dot(out, sn) >= 0.0 ? 1.0 : -1.0;
                const Real angle = -std::atan2(travel * sense, deepest);
                if (std::fabs(angle) < 1e-9) continue;

                const gp_Pln neutral(gp_Pnt(centre.x - n.x * deepest,
                                            centre.y - n.y * deepest,
                                            centre.z - n.z * deepest),
                                     gp_Dir(n.x, n.y, n.z));
                draft.Add(TopoDS::Face(s->faces(static_cast<int>(side) + 1)),
                          gp_Dir(pull.x, pull.y, pull.z),
                          static_cast<Standard_Real>(angle), neutral);
                if (!draft.AddDone()) {
                    if (reason) *reason = "a face beside it will not take the taper";
                    return {};
                }
                ++tilted;
            }
        }

        if (tilted == 0) {
            if (reason) *reason = "nothing beside that face could be tapered";
            return {};
        }

        draft.Build();
        if (!draft.IsDone()) {
            if (reason) *reason = "the faces around it would not follow";
            return {};
        }
        const TopoDS_Shape result = draft.Shape();
        if (!acceptable(result, reason)) return {};
        return makeBrep(result, propagateNames(draft, {{s.get()}}, result, salt));
    } catch (const Standard_Failure& e) {
        if (reason) *reason = kernelReason(e, "that face will not scale");
        return {};
    }
}

BrepRef mergeDivisions(const BrepRef& s, ElementId salt, std::string* reason) {
    if (reason) reason->clear();
    if (!s || s->shape.IsNull()) {
        if (reason) *reason = "there is nothing to merge";
        return {};
    }

    try {
        ShapeUpgrade_UnifySameDomain unify(s->shape, Standard_True, Standard_True,
                                           Standard_False);
        unify.Build();
        const TopoDS_Shape merged = unify.Shape();
        Handle(BRepTools_History) history = unify.History();
        if (merged.IsNull() || history.IsNull() || !acceptable(merged, reason)) {
            if (reason && reason->empty()) *reason = "the merge left nothing valid";
            return {};
        }

        TopTools_IndexedMapOfShape out;
        TopExp::MapShapes(merged, TopAbs_FACE, out);
        if (out.Extent() == s->faces.Extent()) {
            if (reason) *reason = "there are no divisions to drop";
            return {};
        }

        // Where two faces become one, the bigger of them gives the survivor its
        // name: it is the piece a person would have been pointing at, and the
        // one anything earlier in the history is most likely to have meant.
        std::vector<ElementId> names(static_cast<size_t>(out.Extent()), kNoId);
        std::vector<Real> claim(static_cast<size_t>(out.Extent()), -1.0);
        for (int i = 1; i <= s->faces.Extent(); ++i) {
            const ElementId id = s->faceNames[static_cast<size_t>(i - 1)];
            if (id == kNoId) continue;

            GProp_GProps props;
            BRepGProp::SurfaceProperties(s->faces(i), props);
            const Real area = props.Mass();

            std::vector<TopoDS_Shape> landed;
            const TopTools_ListOfShape& mods = history->Modified(s->faces(i));
            if (mods.IsEmpty()) landed.push_back(s->faces(i));
            else for (TopTools_ListIteratorOfListOfShape it(mods); it.More(); it.Next())
                landed.push_back(it.Value());

            for (const TopoDS_Shape& f : landed) {
                const int at = out.FindIndex(f);
                if (at <= 0) continue;
                const size_t k = static_cast<size_t>(at - 1);
                if (area > claim[k]) { claim[k] = area; names[k] = id; }
            }
        }
        for (size_t k = 0; k < names.size(); ++k)
            if (names[k] == kNoId)
                names[k] = nameId(salt, IdRole::Patch, static_cast<ElementId>(k));

        return makeBrep(merged, names);
    } catch (const Standard_Failure& e) {
        if (reason) *reason = kernelReason(e, "those faces could not be merged");
        return {};
    }
}

BrepRef divideBody(const BrepRef& s, Vec3 planePoint, Vec3 planeNormal,
                   ElementId salt, std::string* reason) {
    if (reason) reason->clear();
    if (!s || s->shape.IsNull()) {
        if (reason) *reason = "there is no body to divide";
        return {};
    }
    if (lengthSq(planeNormal) < 1e-18) {
        if (reason) *reason = "the cut has no direction";
        return {};
    }

    try {
        const gp_Pln plane(gp_Pnt(planePoint.x, planePoint.y, planePoint.z),
                           gp_Dir(planeNormal.x, planeNormal.y, planeNormal.z));

        // Where the plane crosses the body, as edges that know which face they
        // came from. Without the pcurves the splitter has nothing to imprint
        // the edge onto.
        BRepAlgoAPI_Section section(s->shape, plane, Standard_False);
        section.ComputePCurveOn1(Standard_True);
        section.Approximation(Standard_True);
        section.Build();
        if (!section.IsDone()) {
            if (reason) *reason = "the plane could not be crossed with the body";
            return {};
        }

        // Imprint, rather than cut: BRepFeat_SplitShape adds the edges to the
        // faces they lie on and hands the solid back whole. A boolean split
        // would hand back two solids, which is a different operation and not
        // the one anybody means by a loop cut.
        BRepFeat_SplitShape splitter(s->shape);
        int added = 0;
        for (TopExp_Explorer e(section.Shape(), TopAbs_EDGE); e.More(); e.Next()) {
            TopoDS_Shape host;
            if (!section.HasAncestorFaceOn1(e.Current(), host)) continue;
            if (host.ShapeType() != TopAbs_FACE) continue;
            splitter.Add(TopoDS::Edge(e.Current()), TopoDS::Face(host));
            ++added;
        }
        if (added == 0) {
            if (reason) *reason = "the plane does not cross the body";
            return {};
        }

        splitter.Build();
        if (!splitter.IsDone()) {
            if (reason) *reason = "the faces could not be divided";
            return {};
        }
        const TopoDS_Shape result = splitter.Shape();
        if (!acceptable(result, reason)) return {};

        std::vector<ElementId> names = propagateNames(splitter, {{s.get()}}, result, salt);

        // Both halves of a divided face come out carrying its name, which would
        // make them one face again to everything downstream -- and the point of
        // dividing is to be able to take hold of one half. Which side of the
        // plane each piece sits gives them their own names, and does it the
        // same way every time the feature is rebuilt.
        TopTools_IndexedMapOfShape faceMap;
        TopExp::MapShapes(result, TopAbs_FACE, faceMap);
        std::unordered_map<ElementId, int> seen;
        for (int i = 1; i <= faceMap.Extent(); ++i) ++seen[names[static_cast<size_t>(i - 1)]];
        for (int i = 1; i <= faceMap.Extent(); ++i) {
            const ElementId id = names[static_cast<size_t>(i - 1)];
            if (id == kNoId || seen[id] < 2) continue;
            GProp_GProps props;
            BRepGProp::SurfaceProperties(faceMap(i), props);
            const gp_Pnt c = props.CentreOfMass();
            const Vec3 mid{c.X(), c.Y(), c.Z()};
            const Real side = dot(mid - planePoint, normalize(planeNormal));
            names[static_cast<size_t>(i - 1)] = nameId(salt, IdRole::Split, id,
                                                       side >= 0.0 ? 0 : 1);
        }
        return makeBrep(result, names);
    } catch (const Standard_Failure& e) {
        if (reason) *reason = kernelReason(e, "the body could not be divided there");
        return {};
    }
}

BrepRef insetFaces(const BrepRef& s, const std::vector<FaceId>& faces, Real amount,
                   ElementId salt, std::vector<ElementId>* newFaces, std::string* reason) {
    if (newFaces) newFaces->clear();
    if (reason) reason->clear();
    if (!s || faces.empty()) {
        if (reason) *reason = "nothing to inset";
        return {};
    }
    if (amount <= 0.0) {
        if (reason) *reason = "the amount has to be positive";
        return {};
    }

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
    for (ElementId target : targets) {
        std::vector<FaceId> at;
        findFaces(*current, target, at);
        if (at.empty()) {
            if (reason) *reason = "a face to inset no longer exists";
            return {};
        }
        const TopoDS_Face& face = faceAt(*current, at.front());

        if (BRepAdaptor_Surface(face).GetType() != GeomAbs_Plane) {
            if (reason) *reason = "only a flat face can be inset";
            return {};
        }

        try {
            // The outline, moved inward within the face's own plane. Offsetting
            // the wire rather than the face keeps the result a wire that still
            // lies in that plane, which is what the splitter needs.
            const TopoDS_Wire outer = BRepTools::OuterWire(face);
            if (outer.IsNull()) {
                if (reason) *reason = "that face has no outline to offset";
                return {};
            }

            BRepOffsetAPI_MakeOffset offset(face, GeomAbs_Arc);
            offset.Perform(-amount);
            if (!offset.IsDone()) {
                if (reason) *reason = "the outline cannot be moved in that far";
                return {};
            }
            TopoDS_Shape inner = offset.Shape();
            if (inner.IsNull()) {
                if (reason) *reason = "the inset leaves nothing of the face";
                return {};
            }

            // Split the face along it: the solid comes back whole, with that
            // one face now two.
            BRepFeat_SplitShape splitter(current->shape);
            bool added = false;
            for (TopExp_Explorer w(inner, TopAbs_WIRE); w.More(); w.Next()) {
                splitter.Add(TopoDS::Wire(w.Current()), face);
                added = true;
            }
            if (!added) {
                if (reason) *reason = "the inset outline is not a closed loop";
                return {};
            }
            splitter.Build();
            if (!splitter.IsDone() || !acceptable(splitter.Shape(), reason)) {
                if (reason && reason->empty()) *reason = "the face could not be split";
                return {};
            }

            const TopoDS_Shape result = splitter.Shape();
            std::vector<ElementId> names = propagateNames(splitter, {{current.get()}}, result, salt);

            // The face was one and is now two, both carrying its name. The
            // inner one is what the user will act on next, so it keeps the
            // name alone and the ring around it takes a derived one -- the
            // opposite of the boolean's rule, and for the same reason: a name
            // should land on the thing a person would point at.
            TopTools_IndexedMapOfShape newFaceMap;
            TopExp::MapShapes(result, TopAbs_FACE, newFaceMap);
            std::vector<int> pieces;
            for (int i = 0; i < newFaceMap.Extent(); ++i)
                if (names[static_cast<size_t>(i)] == target) pieces.push_back(i);
            if (pieces.size() == 2) {
                // The ring is the piece with a hole in it -- two wires, an
                // outline and the inset outline inside it. Not the larger of
                // the two: a 5mm inset on a 40mm face leaves 900mm2 inside and
                // 700 around, and the answer would come out backwards.
                auto wireCount = [](const TopoDS_Shape& f) {
                    int n = 0;
                    for (TopExp_Explorer w(f, TopAbs_WIRE); w.More(); w.Next()) ++n;
                    return n;
                };
                const int w0 = wireCount(newFaceMap(pieces[0] + 1));
                const int w1 = wireCount(newFaceMap(pieces[1] + 1));
                if (w0 != w1) {
                    const int ring = w0 > w1 ? pieces[0] : pieces[1];
                    names[static_cast<size_t>(ring)] = nameId(salt, IdRole::Ring, target);
                }
            }

            current = makeBrep(result, names);
            if (newFaces) newFaces->push_back(target);
        } catch (const Standard_Failure& e) {
            if (reason) *reason = kernelReason(e, "the face could not be inset that far");
            return {};
        }
    }
    return current;
}

// Why an offset might have handed back a solid it did not hollow.
//
// The commonest reason by far is an open face that is only part of a flat
// region: divide the bottom of a box and ask to open one half, and the offset
// reports success and changes nothing. "The wall is too thick" was the guess
// made here before, and it sends the user to make the wall thinner, which will
// never help. If a face next to an opened one lies in the same plane and was
// not opened, that is worth saying outright.
std::string whyNotHollowed(const BrepShape& s, const std::vector<FaceId>& open) {
    for (FaceId f : open) {
        if (!validFace(s, f)) continue;
        const Vec3 n = faceNormal(s, f);
        const Vec3 at = faceCentroid(s, f);

        std::vector<EdgeId> es;
        faceEdges(s, f, es);
        for (EdgeId e : es) {
            FaceId a = kInvalid, b = kInvalid;
            edgeFaces(s, e, a, b);
            const FaceId other = a == f ? b : a;
            if (other == kInvalid || !validFace(s, other)) continue;
            if (std::find(open.begin(), open.end(), other) != open.end()) continue;

            const Vec3 on = faceNormal(s, other);
            if (dot(n, on) < 0.9999) continue;                       // not flat with it
            if (std::fabs(dot(faceCentroid(s, other) - at, n)) > 1e-6) continue;

            return "a face that was divided has only part of it open: open the "
                   "rest of it too";
        }
    }
    return "the wall is too thick to leave a cavity, or this shape defeated the "
           "offset";
}

BrepRef shell(const BrepRef& s, const std::vector<FaceId>& openFaces, Real thickness,
              ElementId salt, std::string* reason) {
    if (reason) reason->clear();
    if (!s || s->shape.IsNull()) {
        if (reason) *reason = "there is no body to shell";
        return {};
    }
    if (!(thickness > 0.0)) {
        if (reason) *reason = "the wall has to have a thickness";
        return {};
    }
    for (FaceId f : openFaces) {
        if (!validFace(*s, f)) {
            if (reason) *reason = "a face to open no longer exists";
            return {};
        }
    }

    // A sealed cavity is a different construction, not the same one with an
    // empty list. MakeThickSolidByJoin with no faces to open does not hollow
    // anything: it offsets the solid inward and hands back a smaller solid --
    // a 20mm cube shelled 2mm comes back as a 16mm cube, valid, watertight,
    // smaller than it was, and completely wrong. Nothing downstream could tell:
    // the volume went down, so even a guard against no-ops is satisfied.
    //
    // The cavity has to be built and subtracted instead.
    if (openFaces.empty()) {
        try {
            BRepOffsetAPI_MakeOffsetShape inward;
            inward.PerformByJoin(s->shape, -thickness, 1e-3);
            inward.Build();
            if (!inward.IsDone() || inward.Shape().IsNull()) {
                if (reason) *reason = "the wall does not fit: try a thinner one";
                return {};
            }

            // Name the cavity from the faces it was offset from, so the inside
            // of the top is recognisably the inside of the top.
            const TopoDS_Shape innerShape = inward.Shape();
            BrepRef inner = makeBrep(innerShape,
                                     propagateNames(inward, {{s.get()}}, innerShape,
                                                    nameId(salt, IdRole::Wall, 1)));

            BrepRef out = booleanOp(*s, *inner, BooleanOp::Difference, salt, reason);
            if (!out) return {};

            GProp_GProps was, now;
            BRepGProp::VolumeProperties(s->shape, was);
            BRepGProp::VolumeProperties(out->shape, now);
            if (now.Mass() > was.Mass() * 0.999) {
                if (reason) *reason = "the wall is too thick to leave a cavity";
                return {};
            }
            return out;
        } catch (const Standard_Failure& e) {
            if (reason) *reason = kernelReason(e, "the body could not be hollowed");
            return {};
        }
    }

    try {
        TopTools_ListOfShape open;
        for (FaceId f : openFaces) open.Append(faceAt(*s, f));

        BRepOffsetAPI_MakeThickSolid op;
        // Negative, because the wall is measured inward: shelling a 20mm box by
        // 2mm leaves a 20mm box with a 16mm cavity, not a 24mm one. An outward
        // offset is a different operation and not what hollowing means.
        op.MakeThickSolidByJoin(s->shape, open, -thickness, 1e-3);
        op.Build();
        if (!op.IsDone()) {
            if (reason)
                *reason = "the wall does not fit: try a thinner one";
            return {};
        }

        const TopoDS_Shape out = op.Shape();
        if (!acceptable(out, reason)) {
            if (reason && reason->empty()) *reason = "shelling produced no valid solid";
            return {};
        }

        // A wall too thick to leave a cavity is not refused by OCCT: it hands
        // back the solid unhollowed and reports success. That is the quiet
        // no-op this codebase will not ship -- the user asked for a hollow
        // part and would get a solid one with a feature in the timeline
        // claiming otherwise. So compare the volumes and say what happened.
        GProp_GProps was, now;
        BRepGProp::VolumeProperties(s->shape, was);
        BRepGProp::VolumeProperties(out, now);
        if (now.Mass() > was.Mass() * 0.999) {
            if (reason) *reason = whyNotHollowed(*s, openFaces);
            return {};
        }

        // The same provenance mechanism as everything else: the outer faces are
        // Modified from the originals and keep their names, and the wall the
        // offset created is Generated from the face it came from. A feature that
        // referred to the top of a box still refers to it after the box is
        // hollowed out.
        return makeBrep(out, propagateNames(op, {{s.get()}}, out, salt));
    } catch (const Standard_Failure& e) {
        if (reason) *reason = kernelReason(e, "the body could not be hollowed");
        return {};
    }
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

        // Four quarter-arcs of one circle are one cylinder, and two collinear
        // spans are one plane. The sweep gives a face per span regardless,
        // which draws seams down a drawn bore that a cut one does not have,
        // and makes clicking the bore select a quarter of it. Merging the
        // spans that share a surface is what makes a hole the same shape
        // whichever way it was made.
        //
        // Before the names are assigned, so there is no name to reconcile:
        // afterwards the merge would have to decide which of four names the
        // surviving face keeps, and any feature that referred to the other
        // three would have nothing to resolve to.
        TopoDS_Shape shape = solid.Shape();
        try {
            ShapeUpgrade_UnifySameDomain unify(shape, Standard_True, Standard_True,
                                               Standard_False);
            unify.Build();
            const TopoDS_Shape merged = unify.Shape();
            if (!merged.IsNull() && acceptable(merged, nullptr)) shape = merged;
        } catch (const Standard_Failure&) {
            // Keep the sweep as it came. More faces than it needs is a
            // blemish; refusing a solid that is otherwise correct is worse.
        }

        // Named by role, the way a primitive is: the two caps and the wall the
        // profile swept out. The wall may still be several faces -- one per
        // span that did not merge -- so each takes the ordinal of its span.
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
        if (reason) *reason = kernelReason(e, "the profile could not be swept into a solid");
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
