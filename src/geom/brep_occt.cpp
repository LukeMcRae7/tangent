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

#include <cstdlib>
#include "geom/brep_valid.h"
#include "geom/kernel_guard.h"
#include "sketch/sketch.h"
#include "sketch/svg.h"

// STEP. Kept together and commented because these are the only headers here
// that are not modelling -- they come from the DataExchange module.
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <Interface_Static.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Message.hxx>
#include <Message_Messenger.hxx>
#include <Message_Printer.hxx>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepAlgoAPI_BooleanOperation.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Defeaturing.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeShape.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepTools_Modification.hxx>
#include <BRepTools_Modifier.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Line.hxx>
#include <Geom_Circle.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_Circle.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColgp_Array1OfPnt2d.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <NCollection_DataMap.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <gp_GTrsf.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <ShapeFix_Solid.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <TopoDS_Shell.hxx>
#include <gp_Pln.hxx>
#include <BRepAlgoAPI_Splitter.hxx>
#include <TopoDS_Compound.hxx>
#include <TopLoc_Location.hxx>
#include <Geom_Surface.hxx>
#include <TopoDS_Iterator.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepTools_History.hxx>
#include <BRepFeat_SplitShape.hxx>
#include <BRepOffsetAPI_DraftAngle.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepOffset_MakeSimpleOffset.hxx>
#include <BRepTools_ReShape.hxx>
#include <BRepBndLib.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRepOffsetAPI_MakeOffsetShape.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
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
#include <BRepLib.hxx>
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
#include <gp_Circ.hxx>
#include <gp_Trsf.hxx>
#include <Geom_BezierCurve.hxx>
#include <TColgp_Array1OfPnt.hxx>

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <atomic>
#include <OSD_Parallel.hxx>
#include <chrono>
#include <cstdio>

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

Vec3 facePoint(const BrepShape& s, FaceId f) {
    if (!validFace(s, f)) return {0, 0, 0};
    const TopoDS_Face& face = faceAt(s, f);
    Standard_Real u0 = 0, u1 = 0, v0 = 0, v1 = 0;
    BRepTools::UVBounds(face, u0, u1, v0, v1);
    BRepAdaptor_Surface surf(face);
    return toVec3(surf.Value((u0 + u1) * 0.5, (v0 + v1) * 0.5));
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
        if (!shapeIsValid(shape)) {
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
        // A flat face has one normal, worked out once: an extruded drawing is
        // thousands of faces, a good share of them flat, and asking the
        // surface at every node of those was a third of the time to show it.
        bool planeNormal = false;
        Vec3 flatN{0, 0, 1};
        if (surf.GetType() == GeomAbs_Plane) {
            // du x dv, which is the axis for a right-handed frame and its
            // opposite for a left-handed one.
            const gp_Pln pln = surf.Plane();
            gp_Dir d = pln.Axis().Direction();
            if (!pln.Position().Direct()) d.Reverse();
            if (flipped) d.Reverse();
            d.Transform(trsf);
            flatN = {static_cast<Real>(d.X()), static_cast<Real>(d.Y()), static_cast<Real>(d.Z())};
            planeNormal = true;
        }
        for (int i = 1; i <= tri->NbNodes(); ++i) {
            gp_Pnt pnt = tri->Node(i);
            pnt.Transform(trsf);
            out.positions.push_back(toVec3(pnt));

            Vec3 n{0, 0, 1};
            if (planeNormal) {
                n = flatN;
            } else if (haveUV) {
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

namespace {

struct FaceWeight {
    int    wires = 0;
    size_t edges = 0;
};

FaceWeight weigh(const TopoDS_Shape& face) {
    FaceWeight w;
    for (TopoDS_Iterator it(face); it.More(); it.Next()) {
        if (it.Value().ShapeType() != TopAbs_WIRE) continue;
        ++w.wires;
        for (TopoDS_Iterator e(it.Value()); e.More(); e.Next()) ++w.edges;
    }
    return w;
}

bool heavy(const FaceWeight& w, size_t prunedAbove) {
    return w.wires > 1 && w.edges > prunedAbove;
}

// One heavy face, in pieces. Every piece is a face on the same surface holding
// the outer wire and one or two inner ones, and every piece goes through the
// full analyzer -- so each wire is checked for crossing itself, each hole for
// sitting inside the boundary and not crossing it, and each pair of holes that
// could possibly touch for not touching. The pairs that are skipped are the
// ones whose boxes are apart, which no intersection could join.
bool heavyFaceValid(const TopoDS_Face& face) {
    // The wires as they are seen through the face -- its orientation and
    // location folded in -- because that is what BRep_Builder::Add expects: it
    // takes a component as seen from outside and un-folds the parent's
    // orientation and location as it stores it. The pieces are empty copies of
    // this face, so the round trip lands each wire back exactly as it was.
    //
    // Getting that backwards is easy and silent. Taking the wires raw, and
    // letting Add invert a reversed face's orientation on top, wound every hole
    // of a perfectly good plate the wrong way and refused it.
    const TopoDS_Wire outer = BRepTools::OuterWire(face);
    if (outer.IsNull()) return false;

    std::vector<TopoDS_Wire> inner;
    for (TopoDS_Iterator it(face); it.More(); it.Next()) {
        if (it.Value().ShapeType() != TopAbs_WIRE) continue;
        const TopoDS_Wire w = TopoDS::Wire(it.Value());
        if (!w.IsSame(outer)) inner.push_back(w);
    }

    BRep_Builder builder;
    const Standard_Real tol = BRep_Tool::Tolerance(face);
    auto piece = [&](const TopoDS_Wire* a, const TopoDS_Wire* b) {
        TopoDS_Face f = TopoDS::Face(face.EmptyCopied());
        builder.Add(f, outer);
        if (a) builder.Add(f, *a);
        if (b) builder.Add(f, *b);
        return BRepCheck_Analyzer(f).IsValid();
    };

    if (inner.empty()) return piece(nullptr, nullptr);

    // Every piece to check -- each hole alone, and each pair of holes whose
    // boxes meet, found by a sweep along x so thousands of holes cost a sort
    // and a pass rather than every pair -- and then all of them across the
    // cores: the pieces share nothing but the wires they read. An engraved
    // drawing's face has hundreds of holes, and one after another they took
    // seconds.
    std::vector<std::pair<int, int>> jobs;
    for (size_t i = 0; i < inner.size(); ++i) jobs.push_back({static_cast<int>(i), -1});
    struct Box { Standard_Real x0, x1; Bnd_Box box; size_t wire; };
    std::vector<Box> boxes;
    boxes.reserve(inner.size());
    for (size_t i = 0; i < inner.size(); ++i) {
        Bnd_Box b;
        BRepBndLib::Add(inner[i], b);
        b.Enlarge(tol);
        Standard_Real x0, y0, z0, x1, y1, z1;
        b.Get(x0, y0, z0, x1, y1, z1);
        boxes.push_back({x0, x1, b, i});
    }
    std::sort(boxes.begin(), boxes.end(), [](const Box& p, const Box& q) { return p.x0 < q.x0; });
    for (size_t i = 0; i < boxes.size(); ++i)
        for (size_t j = i + 1; j < boxes.size() && boxes[j].x0 <= boxes[i].x1; ++j)
            if (!boxes[i].box.IsOut(boxes[j].box))
                jobs.push_back({static_cast<int>(boxes[i].wire), static_cast<int>(boxes[j].wire)});

    std::atomic<bool> ok{true};
    auto run = [&](int k) {
        if (!ok.load(std::memory_order_relaxed)) return;
        const auto [a, b] = jobs[static_cast<size_t>(k)];
        if (!piece(&inner[static_cast<size_t>(a)], b >= 0 ? &inner[static_cast<size_t>(b)] : nullptr))
            ok.store(false, std::memory_order_relaxed);
    };
    // Not in a child process isolating a trial: threads there are what the
    // isolation keeps away from.
    if (inIsolatedChild()) {
        for (int k = 0; k < static_cast<int>(jobs.size()) && ok; ++k) run(k);
    } else {
        OSD_Parallel::For(0, static_cast<int>(jobs.size()), run);
    }
    return ok.load();
}

// What the whole-shape pass checks across faces: each edge of a solid bounds
// one face going one way and one face going the other. Linear in the edges.
bool edgeUsesConsistent(const TopoDS_Shape& shape) {
    for (TopExp_Explorer solid(shape, TopAbs_SOLID); solid.More(); solid.Next()) {
        TopTools_IndexedMapOfShape edges;
        TopExp::MapShapes(solid.Current(), TopAbs_EDGE, edges);
        std::vector<int> forward(static_cast<size_t>(edges.Extent()) + 1, 0);
        std::vector<int> reversed(forward.size(), 0);
        for (TopExp_Explorer f(solid.Current(), TopAbs_FACE); f.More(); f.Next())
            for (TopExp_Explorer e(f.Current(), TopAbs_EDGE); e.More(); e.Next()) {
                const TopoDS_Edge& edge = TopoDS::Edge(e.Current());
                if (BRep_Tool::Degenerated(edge)) continue;
                const int i = edges.FindIndex(edge);
                if (i <= 0) return false;
                if (edge.Orientation() == TopAbs_FORWARD) ++forward[static_cast<size_t>(i)];
                else if (edge.Orientation() == TopAbs_REVERSED) ++reversed[static_cast<size_t>(i)];
            }
        for (int i = 1; i <= edges.Extent(); ++i) {
            if (BRep_Tool::Degenerated(TopoDS::Edge(edges(i)))) continue;
            if (forward[static_cast<size_t>(i)] != 1 || reversed[static_cast<size_t>(i)] != 1)
                return false;
        }
    }
    return true;
}

} // namespace

bool fullAnalyzerValid(const TopoDS_Shape& shape) {
    return !shape.IsNull() && BRepCheck_Analyzer(shape, Standard_True, !inIsolatedChild()).IsValid();
}

bool shapeIsValid(const TopoDS_Shape& shape, size_t prunedAbove) {
    if (shape.IsNull()) return false;

    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(shape, TopAbs_FACE, faces);
    bool anyHeavy = false;
    for (int i = 1; i <= faces.Extent() && !anyHeavy; ++i)
        anyHeavy = heavy(weigh(faces(i)), prunedAbove);

    // The ordinary case: the whole analyzer, across the cores. Parallel
    // changes how the sub-shapes are shared out, not what is checked.
    if (!anyHeavy) return BRepCheck_Analyzer(shape, Standard_True, !inIsolatedChild()).IsValid();

    // The light faces together, in one pass: each of their edges and vertices
    // is then checked once rather than once per face that touches it, and the
    // analyzer is set up once rather than thousands of times. A compound has no
    // closure or orientation of its own to fail, so this asks exactly what
    // checking them one by one would.
    TopoDS_Compound light;
    BRep_Builder builder;
    builder.MakeCompound(light);
    for (int i = 1; i <= faces.Extent(); ++i) {
        const TopoDS_Face& f = TopoDS::Face(faces(i));
        if (heavy(weigh(f), prunedAbove)) {
            if (!heavyFaceValid(f)) return false;
        } else {
            builder.Add(light, f);
        }
    }
    if (!BRepCheck_Analyzer(light, Standard_True, !inIsolatedChild()).IsValid()) return false;
    return edgeUsesConsistent(shape);
}

bool closedShell(const BrepShape& s) {
    if (s.shape.IsNull() || s.faces.Extent() == 0) return false;
    for (int i = 1; i <= s.edgeFaces.Extent(); ++i) {
        const TopoDS_Edge& e = TopoDS::Edge(s.edgeFaces.FindKey(i));
        if (BRep_Tool::Degenerated(e)) continue;
        const int bounds = s.edgeFaces(i).Extent();
        if (bounds == 2) continue;
        // A seam bounds one face on both sides: the slit where a face that
        // closes on itself is cut open -- down a cylinder, or across the ring
        // left round a collar. The ancestor map counts that face once, and
        // reading it as a hole in the body would call a perfectly closed
        // collar open. So ask the face: an edge it uses twice is a seam, not a
        // boundary, whether or not its surface is periodic.
        if (bounds == 1) {
            int uses = 0;
            for (TopExp_Explorer ex(s.edgeFaces(i).First(), TopAbs_EDGE); ex.More(); ex.Next())
                if (ex.Current().IsSame(e)) ++uses;
            if (uses >= 2) continue;
        }
        return false;
    }
    return true;
}

bool validate(const BrepShape& s, std::string* err) {
    if (s.shape.IsNull()) {
        if (err) *err = "null shape";
        return false;
    }
    try {
        if (!shapeIsValid(s.shape)) {
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

        // Closed means what it means for a mesh: every edge bounds exactly two
        // faces. A seam counts its one face twice, which is right -- the face
        // meets itself there -- and a degenerate edge at a pole bounds nothing.
        //
        // This used to be answered by running the full validity check, which is
        // quadratic in the edges of a face. On a normal part that is nothing;
        // on a converted mesh, where each hole left as a hundred straight
        // segments, one face carries thousands of edges and the check took
        // twelve seconds -- every time the Inspector wanted a volume. Whether a
        // result is *valid* is asked where results are made. Whether a body is
        // closed is this.
        //
        // Self-intersection is not run either: -1 is how MeshHealth says "not
        // asked".
        h.watertight = closedShell(s);
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
            case BooleanOp::Union:        algo = std::make_unique<BRepAlgoAPI_Fuse>(); break;
            case BooleanOp::Difference:   algo = std::make_unique<BRepAlgoAPI_Cut>(); break;
            case BooleanOp::Intersection: algo = std::make_unique<BRepAlgoAPI_Common>(); break;
        }
        if (!algo) return {};
        TopTools_ListOfShape arguments, tools;
        arguments.Append(a.shape);
        tools.Append(b.shape);
        algo->SetArguments(arguments);
        algo->SetTools(tools);
        // Across the cores, and with boxes that turn with the faces they bound.
        // Neither changes what is computed, only how it is found: on a solid
        // converted from a scan -- five thousand faces -- boring one hole spent
        // most of a second finding which faces might meet, on one thread, with
        // axis-aligned boxes that on a tilted facet enclose mostly air.
        algo->SetRunParallel(!inIsolatedChild());
        algo->SetUseOBB(Standard_True);
        algo->Build();
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
                    const std::vector<Real>& radii, ElementId salt, std::string* reason,
                    const std::vector<Real>* endRadii, bool chamfer) {
    if (reason) reason->clear();
    if (edges.empty()) {
        if (reason) *reason = chamfer ? "no edges to cut" : "no edges to round";
        return {};
    }
    try {
        // The two live in different classes but answer the same question, and
        // everything around them -- what is added, how a failure is explained,
        // how the names come out -- is the same either way.
        BRepFilletAPI_MakeFillet fil(s.shape);
        BRepFilletAPI_MakeChamfer cha(s.shape);
        int added = 0;
        for (size_t i = 0; i < edges.size(); ++i) {
            if (!validEdge(s, edges[i])) continue;
            const Real r = i < radii.size() ? radii[i] : (radii.empty() ? Real(1.0) : radii.back());
            if (r <= 0.0) continue;
            const TopoDS_Edge& e = edgeAt(s, edges[i]);
            if (BRep_Tool::Degenerated(e)) continue;

            if (chamfer) {
                // Symmetric: the same distance from both faces, which is what
                // a chamfer means unless someone asks for otherwise.
                cha.Add(r, e);
            } else {
                const Real r2 = endRadii && i < endRadii->size() ? (*endRadii)[i] : Real(-1);
                if (r2 > 0.0 && std::fabs(r2 - r) > 1e-9) fil.Add(r, r2, e);
                else                                      fil.Add(r, e);
            }
            ++added;
        }
        if (added == 0) {
            if (reason)
                *reason = chamfer ? "none of those edges can be cut"
                                  : "none of those edges can be rounded";
            return {};
        }

        if (chamfer) {
            cha.Build();
            if (!cha.IsDone()) {
                if (reason) *reason = "the chamfer could not be built";
                return {};
            }
            const TopoDS_Shape result = cha.Shape();
            if (!acceptable(result, reason)) return {};
            return makeBrep(result, propagateNames(cha, {{&s}}, result, salt));
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

namespace {

// One face swept `distance` along its normal, or along `along` when given, as a
// solid of its own -- named from the face it came from, so that whatever it is
// combined with has something to carry.
// The solid between a face and that face moved along its own surface.
//
// This is what pushing a *curved* face means. A cylinder's wall has a
// different normal at every point of it, so there is no one vector to sweep: a
// band of it pulled out two millimetres is a band of a cylinder two
// millimetres wider, and what stands between the two is a ring. Thickening the
// face is exactly that ring, whatever the surface -- cylinder, cone, sphere or
// a freeform patch -- and it comes back as an ordinary solid to be fused onto
// the body or cut out of it, like the prism a flat face sweeps.
TopoDS_Shape thickenFace(const TopoDS_Face& face, Real distance, std::string* reason) {
    try {
        // Started a hair the other side of the face rather than exactly on it.
        // A tool whose surface is the body's own surface is the boolean's
        // worst case -- here it came back with nothing at all -- and the hair
        // is inside the material for a push and outside it for a cut, so what
        // the boolean makes of it is exact either way.
        const Real eps = 1e-2;
        const Real back = distance > 0.0 ? -eps : eps;
        TopoDS_Face from = face;
        BRepOffset_MakeSimpleOffset simple(face, back);
        simple.Perform();
        if (simple.IsDone() && !simple.GetResultShape().IsNull() &&
            simple.GetResultShape().ShapeType() == TopAbs_FACE)
            from = TopoDS::Face(simple.GetResultShape());

        BRepOffsetAPI_MakeThickSolid ms;
        ms.MakeThickSolidBySimple(from, distance - back);
        ms.Build();
        if (!ms.IsDone() || ms.Shape().IsNull()) {
            if (reason) *reason = "that face will not move that far";
            return {};
        }
        TopoDS_Shape made = ms.Shape();
        // Thickening outward hands back a solid that is inside out: its
        // volume comes back negative and a fuse with it does nothing at all.
        // Turned the right way round here, where it can be seen, rather than
        // leaving every caller to wonder why its boolean was a no-op.
        GProp_GProps props;
        BRepGProp::VolumeProperties(made, props);
        if (props.Mass() < 0.0) made.Reverse();
        return made;
    } catch (const Standard_Failure& e) {
        if (reason) *reason = kernelReason(e, "that face will not move that far");
        return {};
    }
}


BrepRef sweepFace(const BrepShape& step, FaceId use, ElementId target, Real distance, Vec3 along,
                  ElementId salt, std::string* reason) {
    const Vec3 n = faceNormal(step, use);
    if (length(n) < 0.5) {
        if (reason) *reason = "a face has no direction to be pushed along";
        return {};
    }
    // Along the face's own normal unless the caller named a direction. A
    // sweep square to the normal moves the face's plane nowhere, so there is
    // nothing to build and it says so.
    Vec3 push = n;
    if (lengthSq(along) > 1e-12) {
        push = normalize(along);
        if (std::fabs(dot(push, n)) < 1e-3) {
            if (reason) *reason = "that direction runs along the face, not into it";
            return {};
        }
    }
    const gp_Vec sweep(push.x * distance, push.y * distance, push.z * distance);

    // A flat face sweeps; a curved one thickens. The difference is not a
    // refinement: a prism off a cylinder's wall leans away in whichever
    // direction the middle of it happened to face, and what the user asked for
    // was a wall two millimetres further out all the way round.
    const bool curved = BRepAdaptor_Surface(faceAt(step, use)).GetType() != GeomAbs_Plane;
    TopoDS_Shape solid;
    if (curved) {
        if (lengthSq(along) > 1e-12) {
            if (reason) *reason = "a curved face moves along itself, not along an axis";
            return {};
        }
        solid = thickenFace(faceAt(step, use), distance, reason);
        if (solid.IsNull()) return {};
    } else {
        try {
            BRepPrimAPI_MakePrism prism(faceAt(step, use), sweep);
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
    }

    TopTools_IndexedMapOfShape pf;
    TopExp::MapShapes(solid, TopAbs_FACE, pf);
    std::vector<ElementId> prismNames(static_cast<size_t>(pf.Extent()), kNoId);
    const Vec3 startCentre = faceCentroid(step, use);
    // How far each face of the tool has travelled from the one that was
    // pushed. Along the normal for a prism; for a thickened face the two are
    // the same surface at two sizes -- a band of a cylinder and its centroid
    // both sit on the axis -- so the measure is the distance from a point of
    // the original face to the other surface, not a projection onto a normal
    // that means nothing there.
    const TopoDS_Face& pushed = faceAt(step, use);
    BRepAdaptor_Surface probe(pushed);
    const gp_Pnt on = probe.Value((probe.FirstUParameter() + probe.LastUParameter()) * 0.5,
                                  (probe.FirstVParameter() + probe.LastVParameter()) * 0.5);
    for (int k = 0; k < pf.Extent(); ++k) {
        const TopoDS_Face& face = TopoDS::Face(pf(k + 1));
        GProp_GProps props;
        BRepGProp::SurfaceProperties(face, props);
        const Vec3 c = toVec3(props.CentreOfMass());
        Real travelled = dot(c - startCentre, n);
        if (curved) {
            BRepExtrema_DistShapeShape gap(BRepBuilderAPI_MakeVertex(on).Vertex(), face);
            travelled = gap.IsDone() && gap.NbSolution() > 0 ? gap.Value() : 0.0;
        }
        // The cap at the far end carries the *original* face's name,
        // because that is what it is: the face the user selected, moved.
        // A feature that referred to it before the extrude has to go on
        // referring to it after, which is the whole job of the history.
        // The near cap vanishes into the body and gets a derived name.
        if (std::fabs(travelled) > std::fabs(distance) * 0.9)
            prismNames[static_cast<size_t>(k)] = target;
        else if (std::fabs(travelled) < std::fabs(distance) * 0.1)
            prismNames[static_cast<size_t>(k)] = nameId(salt, IdRole::Cap, target);
        else
            prismNames[static_cast<size_t>(k)] =
                nameId(salt, IdRole::Wall, target, static_cast<ElementId>(k));
    }
    return makeBrep(solid, prismNames);
}

} // namespace


namespace {

// Is this wire a loop? A face's wires are supposed to be: its outline and the
// holes in it. A wire that starts somewhere and stops somewhere else bounds
// nothing.
bool wireIsClosed(const TopoDS_Wire& wire) {
    TopTools_IndexedDataMapOfShapeListOfShape ends;
    TopExp::MapShapesAndAncestors(wire, TopAbs_VERTEX, TopAbs_EDGE, ends);
    for (int i = 1; i <= ends.Extent(); ++i) {
        int uses = 0;
        for (TopExp_Explorer ex(wire, TopAbs_EDGE); ex.More(); ex.Next())
            for (TopExp_Explorer vx(ex.Current(), TopAbs_VERTEX); vx.More(); vx.Next())
                if (vx.Current().IsSame(ends.FindKey(i))) ++uses;
        if (uses < 2) return false;
    }
    return ends.Extent() > 0;
}

// A boolean against a curved tool can leave a face carrying a wire that is not
// a loop: a single edge lying across the ring left round a collar, where the
// tool's seam met the body's. It bounds nothing -- the face has the area it
// should -- but it is an edge with one face on it, which is what an open shell
// looks like to everything downstream.
//
// So the face is rebuilt from the wires that are loops. Asked for only when
// the shell does not close, and kept only if it closes it: a repair that does
// not repair is not applied.
BrepRef mendShell(const BrepRef& made) {
    if (!made || made->shape.IsNull() || closedShell(*made)) return made;
    try {
        Handle(BRepTools_ReShape) reshape = new BRepTools_ReShape();
        bool any = false;
        for (int i = 0; i < made->faces.Extent(); ++i)
            for (TopExp_Explorer wx(made->faces(i + 1), TopAbs_WIRE); wx.More(); wx.Next())
                if (!wireIsClosed(TopoDS::Wire(wx.Current()))) {
                    reshape->Remove(wx.Current());
                    any = true;
                }
        if (!any) return made;

        const TopoDS_Shape out = reshape->Apply(made->shape);
        if (out.IsNull() || !shapeIsValid(out)) return made;

        // The names, carried by what each face became: ReShape hands back the
        // new shape for every old one it touched, and left the rest alone.
        TopTools_IndexedMapOfShape after;
        TopExp::MapShapes(out, TopAbs_FACE, after);
        std::vector<ElementId> names(static_cast<size_t>(after.Extent()), kNoId);
        for (int k = 0; k < made->faces.Extent(); ++k) {
            const TopoDS_Shape& was = made->faces(k + 1);
            const TopoDS_Shape now = reshape->Value(was);
            const int at = after.FindIndex(now.IsNull() ? was : now);
            if (at > 0) names[static_cast<size_t>(at - 1)] = made->faceNames[static_cast<size_t>(k)];
        }
        BrepRef fixed = makeBrep(out, names);
        return fixed && closedShell(*fixed) ? fixed : made;
    } catch (const Standard_Failure&) {
        return made;
    }
}

} // namespace

BrepRef extrudeFaces(const BrepRef& s, const std::vector<FaceId>& faces, Real distance,
                     ElementId salt, std::vector<ElementId>* newFaces, std::string* reason,
                     bool mergeFlush, Vec3 along, bool intersect) {
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

        // One prism per face: a face split by an earlier step is still one
        // face as far as the feature is concerned, and the prism off its first
        // piece spans every piece of it already.
        BrepRef tool = sweepFace(*current, at.front(), targets[i], distance, along, salt, reason);
        if (!tool) return {};
        const BooleanOp op = intersect      ? BooleanOp::Intersection
                           : distance > 0.0 ? BooleanOp::Union
                                            : BooleanOp::Difference;
        BrepRef combined = booleanOp(*current, *tool, op, nameId(salt, IdRole::Split, targets[i]), reason);
        if (!combined) return {};
        current = mergeFlush && !intersect
                      ? unifyFlush(combined, current, nameId(salt, IdRole::Patch, targets[i]))
                      : combined;
        current = mendShell(current);
        if (newFaces) newFaces->push_back(targets[i]);
    }
    return current;
}

BrepRef sweptFaces(const BrepRef& s, const std::vector<FaceId>& faces, Real distance, Vec3 along,
                   ElementId salt, std::string* reason) {
    if (reason) reason->clear();
    if (!s || faces.empty()) {
        if (reason) *reason = "nothing to sweep";
        return {};
    }
    if (std::fabs(distance) < 1e-9) {
        if (reason) *reason = "the distance is zero";
        return {};
    }
    BrepRef all;
    for (FaceId f : faces) {
        BrepRef one = sweepFace(*s, f, faceName(*s, f), distance, along, salt, reason);
        if (!one) return {};
        if (!all) { all = std::move(one); continue; }
        all = booleanOp(*all, *one, BooleanOp::Union, nameId(salt, IdRole::Split, faceName(*s, f)), reason);
        if (!all) return {};
    }
    return all;
}

bool touches(const BrepShape& a, const BrepShape& b, Real tol) {
    if (a.shape.IsNull() || b.shape.IsNull()) return false;
    try {
        BRepExtrema_DistShapeShape d(a.shape, b.shape);
        if (!d.IsDone()) return false;
        // Overlapping solids report no distance between their surfaces only
        // when the surfaces cross; one wholly inside the other is the case
        // that needs asking separately.
        return d.Value() <= tol || d.InnerSolution();
    } catch (const Standard_Failure&) {
        return false;
    }
}

BrepRef draftFaces(const BrepRef& s, const std::vector<FaceId>& faces, Real angleRad,
                   Vec3 neutralPoint, Vec3 pull, ElementId salt, std::string* reason) {
    if (reason) reason->clear();
    if (!s || s->shape.IsNull() || faces.empty()) {
        if (reason) *reason = "no faces to draft";
        return {};
    }
    if (std::fabs(angleRad) < 1e-9) {
        if (reason) *reason = "the angle is zero";
        return {};
    }
    if (std::fabs(angleRad) > 1.4) {
        if (reason) *reason = "that is more lean than a wall can take";
        return {};
    }
    if (lengthSq(pull) < 1e-18) {
        if (reason) *reason = "there is no direction to pull in";
        return {};
    }
    const Vec3 dir = normalize(pull);

    try {
        BRepOffsetAPI_DraftAngle draft(s->shape);
        // The plane the part is widest at: everything named narrows away from
        // it, along the pull.
        const gp_Pln neutral(gp_Pnt(neutralPoint.x, neutralPoint.y, neutralPoint.z),
                             gp_Dir(dir.x, dir.y, dir.z));
        int added = 0;
        for (FaceId f : faces) {
            if (!validFace(*s, f)) {
                if (reason) *reason = "a face to draft no longer exists";
                return {};
            }
            const Vec3 n = faceNormal(*s, f);
            if (lengthSq(n) < 1e-18) continue;
            // A face facing the way the part is pulled has no line in the
            // neutral plane to turn about: it is the top or the bottom, not a
            // wall, and drafting it means nothing.
            if (std::fabs(dot(normalize(n), dir)) > 0.999) {
                if (reason) *reason = "a face square to the pull has no wall to lean";
                return {};
            }
            const TopoDS_Face face = TopoDS::Face(s->faces(static_cast<int>(f) + 1));
            draft.Add(face, gp_Dir(dir.x, dir.y, dir.z), static_cast<Standard_Real>(angleRad),
                      neutral);
            if (!draft.AddDone()) {
                if (reason) *reason = "that face will not take a draft";
                return {};
            }
            ++added;
        }
        if (added == 0) {
            if (reason) *reason = "no face could be drafted";
            return {};
        }

        draft.Build();
        if (!draft.IsDone()) {
            if (reason) *reason = "the draft could not be built: try a smaller angle";
            return {};
        }
        const TopoDS_Shape result = draft.Shape();
        if (!acceptable(result, reason)) return {};
        return makeBrep(result, propagateNames(draft, {{s.get()}}, result, salt));
    } catch (const Standard_Failure& e) {
        if (reason) *reason = kernelReason(e, "the draft could not be built");
        return {};
    }
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

    // A round face has no plane to grow in. A cylinder scaled is a cylinder of
    // another radius -- the face moved along its own surface -- so that is what
    // scaling one means, and a bore scaled up is a wider hole rather than a
    // narrower one. Decided here rather than in the panel, so that a scale
    // saved in a history still means the same thing when it is re-run.
    bool round = true;
    for (FaceId f : faces)
        if (!validFace(*s, f) ||
            BRepAdaptor_Surface(faceAt(*s, f)).GetType() != GeomAbs_Cylinder)
            round = false;
    if (round) {
        std::vector<ElementId> targets;
        for (FaceId f : faces) targets.push_back(faceName(*s, f));
        BrepRef current = s;
        for (ElementId id : targets) {
            std::vector<FaceId> at;
            findFaces(*current, id, at);
            if (at.empty()) {
                if (reason) *reason = "a face to scale no longer exists";
                return {};
            }
            Vec3 axisPoint{}, axis{};
            Real radius = 0.0;
            if (!faceCylinder(*current, at.front(), axisPoint, axis, radius) || radius < 1e-9) {
                if (reason) *reason = "that face has no radius to scale";
                return {};
            }
            // Which way its own normal points: out of the material on a boss,
            // into the hole on a bore. Scaling up means a bigger radius either
            // way, so the two go opposite ways along the normal.
            const Vec3 on = facePoint(*current, at.front());
            Vec3 radial = on - axisPoint;
            radial = radial - axis * dot(radial, axis);
            const Real sense = dot(radial, faceNormal(*current, at.front())) >= 0.0 ? 1.0 : -1.0;
            const Real distance = sense * radius * (factor - 1.0);
            if (std::fabs(distance) < 1e-9) {
                if (reason) *reason = "that is the size it already is";
                return {};
            }
            current = extrudeFaces(current, at, distance, salt, nullptr, reason,
                                   /*mergeFlush=*/true, Vec3{}, /*intersect=*/false);
            if (!current) return {};
        }
        return current;
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

            // How far a neighbour reaches back from this face. Each one pivots
            // about its own far end, which is where it stays put -- not the
            // far end of the whole body: the walls of a boss extruded off a
            // box end where the boss meets the box, and pivoting them about
            // the box's bottom dragged their foot out across the top with it.
            auto depthOf = [&](FaceId side) {
                Real deepest = 0.0;
                TopTools_IndexedMapOfShape vs;
                TopExp::MapShapes(s->faces(static_cast<int>(side) + 1), TopAbs_VERTEX, vs);
                for (int i = 1; i <= vs.Extent(); ++i) {
                    const gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(vs(i)));
                    deepest = std::max(deepest, -dot(Vec3{p.X(), p.Y(), p.Z()} - centre, n));
                }
                return deepest;
            };

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
                const Real deepest = depthOf(side);
                if (deepest < 1e-6) continue;              // nothing behind it to lean
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
            if (reason) *reason = "there is no depth behind that face to taper";
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

BrepRef removeFaces(const BrepRef& s, const std::vector<FaceId>& faces, ElementId salt,
                    std::string* reason) {
    if (reason) reason->clear();
    if (!s || s->shape.IsNull() || faces.empty()) {
        if (reason) *reason = "no faces to remove";
        return {};
    }
    try {
        TopTools_ListOfShape drop;
        for (FaceId f : faces) {
            if (!validFace(*s, f)) {
                if (reason) *reason = "a face to remove no longer exists";
                return {};
            }
            drop.Append(s->faces(static_cast<int>(f) + 1));
        }

        BRepAlgoAPI_Defeaturing algo;
        algo.SetShape(s->shape);
        algo.AddFacesToRemove(drop);
        algo.SetRunParallel(!inIsolatedChild());
        algo.SetToFillHistory(Standard_True);
        algo.Build();
        if (!algo.IsDone() || algo.HasErrors() || algo.Shape().IsNull()) {
            if (reason) {
                std::ostringstream os;
                algo.DumpErrors(os);
                // What the kernel says here is not for reading: the useful
                // half is that the faces around the hole could not be grown
                // back over it, which is what this means every time.
                *reason = "the faces around it will not close the gap";
                if (!os.str().empty()) std::fprintf(stderr, "[kernel] defeaturing: %s", os.str().c_str());
            }
            return {};
        }
        const TopoDS_Shape out = algo.Shape();
        if (!acceptable(out, reason)) {
            if (reason && reason->empty()) *reason = "removing it produced no valid solid";
            return {};
        }

        // A face it could not take off comes back as the body it was given,
        // reported as a success. That is the quiet no-op this codebase will
        // not ship: the user asked for a face to go and would be looking at
        // it still there, with nothing said.
        TopTools_IndexedMapOfShape left;
        TopExp::MapShapes(out, TopAbs_FACE, left);
        if (left.Extent() == s->faces.Extent()) {
            GProp_GProps was, now;
            BRepGProp::VolumeProperties(s->shape, was);
            BRepGProp::VolumeProperties(out, now);
            if (std::fabs(now.Mass() - was.Mass()) < std::fabs(was.Mass()) * 1e-9) {
                if (reason) *reason = "there is nothing around it to close the gap";
                return {};
            }
        }
        return makeBrep(out, propagateNames(algo, {{s.get()}}, out, salt));
    } catch (const Standard_Failure& e) {
        if (reason) *reason = kernelReason(e, "those faces could not be removed");
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


// ---------------------------------------------------------------------------
// Drilling
// ---------------------------------------------------------------------------
namespace {

// The tool a hole is cut with, named for what each of its surfaces will become
// in the body: the bore's wall, the pocket's wall and floor, the cone of a
// countersink or of a drill's point. Names are given by what a face *is* and
// where it sits along the axis, not by the order the kernel happens to list
// them, so they hold when the hole moves or changes size.
std::vector<ElementId> nameHoleTool(const TopoDS_Shape& shape, Vec3 at, Vec3 dir,
                                    const HoleCut& cut, ElementId salt) {
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(shape, TopAbs_FACE, faces);
    std::vector<ElementId> names(static_cast<size_t>(faces.Extent()), kNoId);
    const Real bore = cut.diameter * 0.5;

    for (int i = 1; i <= faces.Extent(); ++i) {
        const TopoDS_Face& face = TopoDS::Face(faces(i));
        BRepAdaptor_Surface surf(face);
        // Where the face sits, measured down the hole from its mouth.
        GProp_GProps props;
        BRepGProp::SurfaceProperties(face, props);
        const gp_Pnt c = props.CentreOfMass();
        const Real along = dot(Vec3{c.X(), c.Y(), c.Z()} - at, dir);

        ElementId id = kNoId;
        switch (surf.GetType()) {
            case GeomAbs_Cylinder: {
                const Real r = surf.Cylinder().Radius();
                id = nameId(salt, IdRole::Wall, std::fabs(r - bore) < 1e-6 ? 0 : 1);
                break;
            }
            case GeomAbs_Cone:
                // The one at the mouth is the countersink; the one at the far
                // end is the drill's point.
                id = nameId(salt, IdRole::Wall, along < cut.depth * 0.5 ? 2 : 3);
                break;
            case GeomAbs_Plane:
                // The pocket's floor is the one plane inside the material; the
                // others cap the tool off outside it.
                id = nameId(salt, IdRole::Top,
                            cut.kind == HoleKind::Counterbore && std::fabs(along - cut.headDepth) < 1e-6
                                ? 0 : 1);
                break;
            default:
                id = nameId(salt, IdRole::Patch, static_cast<ElementId>(i));
                break;
        }
        names[static_cast<size_t>(i - 1)] = id;
    }
    return names;
}

} // namespace

BrepRef drillHole(const BrepRef& s, Vec3 at, Vec3 into, const HoleCut& cut, ElementId salt,
                  std::string* reason) {
    if (reason) reason->clear();
    if (!s || s->shape.IsNull()) {
        if (reason) *reason = "there is no body to drill";
        return {};
    }
    if (!(cut.diameter > 1e-6)) {
        if (reason) *reason = "a hole needs a diameter";
        return {};
    }
    if (length(into) < 1e-9) {
        if (reason) *reason = "the hole has no direction to go in";
        return {};
    }
    if (!cut.through && !(cut.depth > 1e-6)) {
        if (reason) *reason = "a hole that does not go through needs a depth";
        return {};
    }
    if (cut.kind != HoleKind::Simple && cut.headDiameter <= cut.diameter + 1e-9) {
        if (reason)
            *reason = cut.kind == HoleKind::Counterbore
                          ? "the counterbore is not wider than the hole"
                          : "the countersink is not wider than the hole";
        return {};
    }
    if (cut.kind == HoleKind::Counterbore && !(cut.headDepth > 1e-6)) {
        if (reason) *reason = "the counterbore needs a depth";
        return {};
    }

    const Vec3 dir = normalize(into);
    const AABB box = bounds(*s);
    const Real span = length(box.size()) + 10.0;
    // The tool starts above the face rather than on it: a cut whose mouth is
    // exactly the surface is the boolean's hardest case and the one that most
    // often comes back with a sliver of a face on it.
    const Real lift = std::max(span * 1e-3, Real(0.05));
    const Vec3 mouth = at - dir * lift;
    const gp_Dir axis(dir.x, dir.y, dir.z);
    const gp_Pnt start(mouth.x, mouth.y, mouth.z);
    const Real bore = cut.diameter * 0.5;
    const Real reach = cut.through ? span : cut.depth;

    try {
        TopoDS_Shape tool = BRepPrimAPI_MakeCylinder(gp_Ax2(start, axis), bore, reach + lift).Shape();
        auto add = [&](const TopoDS_Shape& piece) {
            BRepAlgoAPI_Fuse fuse(tool, piece);
            fuse.SetRunParallel(!inIsolatedChild());
            fuse.Build();
            if (fuse.IsDone() && !fuse.HasErrors()) tool = fuse.Shape();
        };

        if (cut.kind == HoleKind::Counterbore) {
            add(BRepPrimAPI_MakeCylinder(gp_Ax2(start, axis), cut.headDiameter * 0.5,
                                         cut.headDepth + lift).Shape());
        } else if (cut.kind == HoleKind::Countersink) {
            // The cone is the head's width at the face and the bore's width
            // where it runs out, so the part above the face only exists
            // because the tool starts there.
            const Real half = std::clamp(cut.sinkAngle * 0.5, Real(0.05), Real(1.5));
            const Real slope = std::tan(half);
            const Real head = cut.headDiameter * 0.5;
            const Real drop = slope > 1e-6 ? (head - bore) / slope : 0.0;
            add(BRepPrimAPI_MakeCone(gp_Ax2(start, axis), head + lift * slope, bore,
                                     lift + drop).Shape());
        }

        if (!cut.through && cut.drillPoint) {
            // The cone a drill leaves, and the shape a printed hole wants at
            // its far end for the same reason: nothing to bridge.
            const Real half = std::clamp(cut.pointAngle * 0.5, Real(0.05), Real(1.5));
            const Real slope = std::tan(half);
            const Real tip = slope > 1e-6 ? bore / slope : bore;
            const Vec3 bottom = at + dir * cut.depth;
            add(BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(bottom.x, bottom.y, bottom.z), axis), bore,
                                     0.0, tip).Shape());
        }

        if (tool.IsNull()) {
            if (reason) *reason = "the hole could not be made";
            return {};
        }

        GProp_GProps was;
        BRepGProp::VolumeProperties(s->shape, was);

        BrepRef toolShape = makeBrep(tool, nameHoleTool(tool, at, dir, cut, salt));
        if (!toolShape) {
            if (reason) *reason = "the hole could not be made";
            return {};
        }
        BrepRef out = booleanOp(*s, *toolShape, BooleanOp::Difference, salt, reason);
        if (!out) return out;

        GProp_GProps now;
        BRepGProp::VolumeProperties(out->shape, now);
        if (now.Mass() > was.Mass() * 0.999999) {
            // A hole that takes nothing away is a hole in the air. OCCT does
            // not call that an error, and a step in the history claiming to
            // have drilled something is worse than being told.
            if (reason) *reason = "the hole misses the material";
            return {};
        }
        return out;
    } catch (const Standard_Failure& e) {
        if (reason) *reason = kernelReason(e, "the hole could not be drilled");
        return {};
    }
}

namespace {

// One face to sweep: an outline and the loops cut out of it.
struct SweptFace {
    const SketchLoop* outer = nullptr;
    std::vector<const SketchLoop*> holes;
};

SketchId loopKey(const SketchLoop& loop) {
    return loop.entities.empty() ? kNoSketchId
                                 : *std::min_element(loop.entities.begin(), loop.entities.end());
}

// The flat faces a set of regions makes on the sketch plane, lifted along its
// normal, and every edge of them against the sketch entity it came from -- so
// the surface each one sweeps out, straight or turned, can be named for that
// entity.
struct SketchFaces {
    std::vector<TopoDS_Face> flat;
    std::vector<std::pair<TopoDS_Edge, SketchId>> made;
};

bool buildSketchFaces(const Sketch& sk, const std::vector<SweptFace>& faces, Real lift,
                      SketchFaces& out, std::string* reason) {
    const SketchPlane& pl = sk.plane;
    const Vec3 n = pl.normal();
    const Vec3 up = n * lift;
    const gp_Dir normal(n.x, n.y, n.z);
    const gp_Dir xDir(pl.xAxis.x, pl.xAxis.y, pl.xAxis.z);
    auto at3 = [&](Vec2 p) {
        const Vec3 w = pl.toWorld(p) + up;
        return gp_Pnt(w.x, w.y, w.z);
    };
    auto pointOf = [&](SketchId id, Vec2& p) {
        const SketchPoint* q = sk.point(id);
        if (!q) return false;
        p = q->at;
        return true;
    };

    // One edge per entity, as it ended up in the wire, so the wall each one
    // sweeps can be named for the entity rather than for where it happens
    // to sit.
    std::vector<std::pair<TopoDS_Edge, SketchId>>& made = out.made;

    auto edgeFor = [&](const SketchEntity& e, TopoDS_Edge& edge) -> bool {
        switch (e.curve) {
        case SketchCurve::Line: {
            Vec2 a, b;
            if (!pointOf(e.a, a) || !pointOf(e.b, b) || length(b - a) < 1e-9) return false;
            edge = BRepBuilderAPI_MakeEdge(at3(a), at3(b)).Edge();
            return true;
        }
        case SketchCurve::Circle: {
            Vec2 c;
            if (!pointOf(e.a, c) || e.radius <= 1e-9) return false;
            edge = BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(at3(c), normal, xDir), e.radius)).Edge();
            return true;
        }
        case SketchCurve::Arc: {
            Vec2 c, s, t;
            if (!pointOf(e.a, c) || !pointOf(e.b, s) || !pointOf(e.c, t)) return false;
            const Real r = length(s - c);
            if (r <= 1e-9) return false;
            // Counter-clockwise about the plane's normal, start to end: the
            // direction the sketch stores an arc in.
            GC_MakeArcOfCircle arc(gp_Circ(gp_Ax2(at3(c), normal, xDir), r), at3(s), at3(t),
                                   Standard_True);
            if (!arc.IsDone()) return false;
            edge = BRepBuilderAPI_MakeEdge(arc.Value()).Edge();
            return true;
        }
        case SketchCurve::Bezier: {
            Vec2 p0, p1, p2, p3;
            if (!pointOf(e.a, p0) || !pointOf(e.b, p1) || !pointOf(e.c, p2) || !pointOf(e.d, p3))
                return false;
            TColgp_Array1OfPnt poles(1, 4);
            poles.SetValue(1, at3(p0));
            poles.SetValue(2, at3(p1));
            poles.SetValue(3, at3(p2));
            poles.SetValue(4, at3(p3));
            Handle(Geom_BezierCurve) curve = new Geom_BezierCurve(poles);
            edge = BRepBuilderAPI_MakeEdge(curve).Edge();
            return true;
        }
        }
        return false;
    };

    auto wireFor = [&](const SketchLoop& loop, bool counterClockwise, TopoDS_Wire& wireOut) {
        BRepBuilderAPI_MakeWire wire;
        for (size_t k = 0; k < loop.entities.size(); ++k) {
            const SketchEntity* e = sk.entity(loop.entities[k]);
            TopoDS_Edge edge;
            if (!e || !edgeFor(*e, edge)) return false;
            // Added the way the loop runs, so the wire's direction is the
            // loop's and its signed area says which way round it is.
            wire.Add(k < loop.reversed.size() && loop.reversed[k]
                         ? TopoDS::Edge(edge.Reversed()) : edge);
            if (!wire.IsDone()) return false;
            // Not `edge`: joining it to the wire may have copied it onto
            // shared vertices, and only the copy is in the face.
            made.push_back({wire.Edge(), loop.entities[k]});
        }
        wireOut = wire.Wire();
        // A face wants its outline counter-clockwise about its normal and
        // its holes the other way.
        if ((loop.signedArea > 0) != counterClockwise) wireOut.Reverse();
        return true;
    };

    const Vec3 o = pl.origin + up;
    out.flat.reserve(faces.size());
    for (const SweptFace& f : faces) {
        TopoDS_Wire outer;
        if (!wireFor(*f.outer, true, outer)) {
            if (reason) *reason = "the profile does not close";
            return false;
        }
        BRepBuilderAPI_MakeFace face(gp_Pln(gp_Pnt(o.x, o.y, o.z), normal), outer, Standard_True);
        for (const SketchLoop* hole : f.holes) {
            TopoDS_Wire w;
            if (!wireFor(*hole, false, w)) {
                if (reason) *reason = "a hole in the profile does not close";
                return false;
            }
            face.Add(w);
        }
        if (!face.IsDone()) {
            if (reason) *reason = "the profile does not bound a face";
            return false;
        }
        out.flat.push_back(face.Face());
    }

    // What can be wrong with a drawing is in its outlines -- one that
    // crosses itself, a hole that crosses the outline around it. Found on
    // the sketch's own curves, exactly and quickly: the kernel's check of
    // the same faces compares every edge with every other, and took
    // thirteen seconds on a rose of a few thousand curves -- two and a
    // half even told to leave the geometry alone. The rest of what it
    // would catch cannot happen here: every wire was checked closed as it
    // was built, and every hole lies inside its outline because that is
    // how the sketch found it to be a hole.
    size_t bad = 0;
    for (const SweptFace& f : faces) {
        std::vector<std::vector<SvgSegment>> loops{sketchLoopCurves(sk, *f.outer)};
        for (const SketchLoop* h : f.holes) loops.push_back(sketchLoopCurves(sk, *h));
        if (outlineCrossings(loops) > 0) ++bad;
    }
    if (bad > 0) {
        if (reason)
            *reason = out.flat.size() == 1
                          ? std::string("the profile crosses itself")
                          : std::to_string(bad) + " of " + std::to_string(out.flat.size()) +
                                " regions have outlines that cross: leave them out, or fix them in the sketch";
        return false;
    }
    return true;
}

// One solid a face swept out, with whatever the sweep made from the face
// itself: the two caps of a prism, or the flat ends of a part turn.
struct Swept { TopoDS_Shape shape, first, last; };

// Puts the solids together and names every face of them.
//
// One solid is itself; several stand in a compound. They do not touch --
// regions picked inside one another were merged into one face before this --
// so there is nothing to fuse. The caps are named for which end they are, and
// each wall for the sketch entity that swept it: `walls` is what the operation
// itself said each entity generated, and where it is empty the walls are found
// instead through the result's own edge-to-face map, which is what a prism
// leaves behind for any number of prisms at once.
BrepRef assembleSwept(const std::vector<Swept>& solids,
                      const std::vector<std::pair<TopoDS_Edge, SketchId>>& made,
                      const std::vector<std::pair<TopoDS_Shape, SketchId>>& walls,
                      ElementId salt, std::string* reason) {
    TopoDS_Shape shape;
    if (solids.size() == 1) {
        shape = solids.front().shape;
    } else {
        BRep_Builder b;
        TopoDS_Compound c;
        b.MakeCompound(c);
        for (const Swept& sw : solids) b.Add(c, sw.shape);
        shape = c;
    }
    // The faces were checked before they were swept, and a prism of a
    // valid flat face is valid: checking every wall of every prism again
    // was three quarters of the time a drawing of hundreds of letters
    // took to extrude.
    if (shape.IsNull()) {
        if (reason) *reason = "the profile could not be swept into a solid";
        return {};
    }

    TopTools_IndexedMapOfShape fs;
    TopExp::MapShapes(shape, TopAbs_FACE, fs);
    std::vector<ElementId> names(static_cast<size_t>(fs.Extent()), kNoId);
    auto nameFace = [&](const TopoDS_Shape& f, ElementId name) {
        if (f.IsNull()) return;
        const int i = fs.FindIndex(f);
        if (i > 0 && names[static_cast<size_t>(i - 1)] == kNoId)
            names[static_cast<size_t>(i - 1)] = name;
    };
    for (size_t k = 0; k < solids.size(); ++k) {
        nameFace(solids[k].first, nameId(salt, IdRole::Cap, static_cast<ElementId>(2 * k)));
        nameFace(solids[k].last, nameId(salt, IdRole::Cap, static_cast<ElementId>(2 * k + 1)));
    }
    if (!walls.empty()) {
        for (const auto& [face, entity] : walls)
            nameFace(face, nameId(salt, IdRole::Side, entity));
    } else {
        TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
        TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeFaces);
        for (const auto& [edge, entity] : made) {
            const int i = edgeFaces.FindIndex(edge);
            if (i <= 0) continue;
            for (TopTools_ListOfShape::Iterator it(edgeFaces(i)); it.More(); it.Next()) {
                // Not a cap: the caps were named first, and a named face is
                // left alone.
                nameFace(it.Value(), nameId(salt, IdRole::Side, entity));
            }
        }
    }

    // Every face above should have a name by now. One that does not still
    // gets a derived one, so the body is usable, rather than a zero that
    // would collide with every other unnamed face.
    for (size_t i = 0; i < names.size(); ++i)
        if (names[i] == kNoId) names[i] = nameId(salt, IdRole::Patch, static_cast<ElementId>(i));

    return makeBrep(shape, names);
}

BrepRef sweepSketchFaces(const Sketch& sk, const std::vector<SweptFace>& faces, Real from, Real to,
                         ElementId salt, std::string* reason) {
    if (reason) reason->clear();
    const Real lo = std::min(from, to), hi = std::max(from, to);
    if (hi - lo < 1e-9) {
        if (reason) *reason = "the profile has no depth";
        return {};
    }
    if (faces.empty()) {
        if (reason) *reason = "the profile is empty";
        return {};
    }

    try {
        SketchFaces built;
        if (!buildSketchFaces(sk, faces, lo, built, reason)) return {};

        const Vec3 n = sk.plane.normal();
        const Real depth = hi - lo;
        std::vector<Swept> solids;
        solids.reserve(built.flat.size());
        for (const TopoDS_Face& f : built.flat) {
            BRepPrimAPI_MakePrism solid(f, gp_Vec(n.x * depth, n.y * depth, n.z * depth));
            solid.Build();
            if (!solid.IsDone()) {
                if (reason) *reason = "the profile could not be swept into a solid";
                return {};
            }
            solids.push_back({solid.Shape(), solid.FirstShape(), solid.LastShape()});
        }
        return assembleSwept(solids, built.made, {}, salt, reason);
    } catch (const Standard_Failure& e) {
        if (reason) *reason = e.GetMessageString() ? e.GetMessageString() : "the profile threw";
        return {};
    }
}

// Which side of the axis a point is on, in the sketch's own coordinates. Zero
// on the axis, and the sign says which side.
Real sideOfAxis(Vec2 at, Vec2 dir, Vec2 p) {
    return dir.x * (p.y - at.y) - dir.y * (p.x - at.x);
}

BrepRef revolveSketchFaces(const Sketch& sk, const std::vector<SweptFace>& faces, Vec2 axisAt,
                           Vec2 axisDir, Real angle, ElementId salt, std::string* reason) {
    if (reason) reason->clear();
    if (faces.empty()) {
        if (reason) *reason = "the profile is empty";
        return {};
    }
    if (length(axisDir) < 1e-9) {
        if (reason) *reason = "the axis has no direction";
        return {};
    }
    axisDir = normalize(axisDir);
    if (angle < 1e-6) {
        if (reason) *reason = "a turn of nothing makes nothing";
        return {};
    }
    const Real full = 2.0 * kPi;
    if (angle > full + 1e-9) {
        if (reason) *reason = "more than a full turn would sweep over itself";
        return {};
    }
    angle = std::min(angle, full);

    // A profile that straddles the axis would turn through itself, and what
    // comes out is not a solid anybody asked for. Touching the axis is fine --
    // that is how a half-disc makes a sphere -- so this is about crossing it.
    //
    // Measured on the sketch's own curves rather than on its points: a Bézier
    // whose ends sit on one side can still bulge over the axis.
    {
        Real most = 0.0, least = 0.0;
        for (const SweptFace& f : faces) {
            std::vector<const SketchLoop*> loops{f.outer};
            for (const SketchLoop* h : f.holes) loops.push_back(h);
            for (const SketchLoop* loop : loops)
                for (SketchId id : loop->entities)
                    if (const SketchEntity* e = sk.entity(id))
                        for (Vec2 p : sketchEntityPoints(sk, *e, 24)) {
                            const Real s = sideOfAxis(axisAt, axisDir, p);
                            most = std::max(most, s);
                            least = std::min(least, s);
                        }
        }
        if (most > 1e-6 && least < -1e-6) {
            if (reason) *reason = "the profile crosses the axis it turns about";
            return {};
        }
    }

    try {
        SketchFaces built;
        if (!buildSketchFaces(sk, faces, 0.0, built, reason)) return {};

        const SketchPlane& pl = sk.plane;
        const Vec3 o = pl.toWorld(axisAt);
        const Vec3 d = normalize(pl.toWorld(axisAt + axisDir) - o);
        const gp_Ax1 axis(gp_Pnt(o.x, o.y, o.z), gp_Dir(d.x, d.y, d.z));

        std::vector<Swept> solids;
        std::vector<std::pair<TopoDS_Shape, SketchId>> walls;
        solids.reserve(built.flat.size());
        for (const TopoDS_Face& f : built.flat) {
            BRepPrimAPI_MakeRevol solid(f, axis, angle);
            solid.Build();
            if (!solid.IsDone() || solid.Shape().IsNull()) {
                if (reason) *reason = "the profile could not be turned into a solid";
                return {};
            }
            // A full turn has no ends; a part turn has the profile at each.
            const bool whole = angle >= full - 1e-9;
            solids.push_back({solid.Shape(), whole ? TopoDS_Shape() : solid.FirstShape(),
                              whole ? TopoDS_Shape() : solid.LastShape()});
            // What each entity turned into, asked of the operation rather than
            // found afterwards: a full turn closes the profile's edges into
            // seams, and a seam is in two faces at once.
            for (const auto& [edge, entity] : built.made) {
                const TopTools_ListOfShape& gen = solid.Generated(edge);
                for (TopTools_ListOfShape::Iterator it(gen); it.More(); it.Next())
                    walls.push_back({it.Value(), entity});
            }
        }
        // Nothing generated means nothing to name from; the map is the better
        // answer then, and assembleSwept falls back to it on an empty list.
        BrepRef out = assembleSwept(solids, built.made, walls, salt, reason);
        if (!out) return out;
        if (!acceptable(out->shape, reason)) {
            if (reason && reason->empty()) *reason = "the turn produced no valid solid";
            return {};
        }
        return out;
    } catch (const Standard_Failure& e) {
        if (reason) *reason = kernelReason(e, "the profile could not be turned into a solid");
        return {};
    }
}

} // namespace

BrepRef sketchSolid(const Sketch& sk, const SketchProfile& profile, Real from, Real to,
                    ElementId salt, std::string* reason) {
    if (profile.outer.entities.empty()) {
        if (reason) *reason = "the profile is empty";
        return {};
    }
    SweptFace f;
    f.outer = &profile.outer;
    for (const SketchLoop& h : profile.holes) f.holes.push_back(&h);
    return sweepSketchFaces(sk, {f}, from, to, salt, reason);
}

namespace {

// The faces the chosen regions make: a face per picked region whose
// surrounding region is not also picked, holed by the loops inside it that
// were not picked.
bool facesForRegions(const std::vector<SketchProfile>& profiles, const std::vector<SketchId>& keys,
                     std::vector<SweptFace>& out, std::string* reason) {
    std::unordered_map<SketchId, const SketchProfile*> byKey;
    for (const SketchProfile& p : profiles) byKey.emplace(p.key, &p);
    std::unordered_set<SketchId> chosen;
    for (SketchId k : keys) {
        if (!byKey.count(k)) {
            if (reason) *reason = "that region of the sketch no longer closes";
            return false;
        }
        chosen.insert(k);
    }
    // Which region each one is a hole in.
    std::unordered_map<SketchId, SketchId> parent;
    for (const SketchProfile& p : profiles)
        for (const SketchLoop& h : p.holes) parent[loopKey(h)] = p.key;

    // A face starts at each picked region whose surrounding region is not
    // picked, and takes in every picked region inside it: its holes are the
    // loops inside it that are not picked, however deep the picked ones go.
    std::vector<SketchId> tops(keys);
    std::sort(tops.begin(), tops.end());
    tops.erase(std::unique(tops.begin(), tops.end()), tops.end());
    for (SketchId top : tops) {
        const auto up = parent.find(top);
        if (up != parent.end() && chosen.count(up->second)) continue;
        SweptFace f;
        f.outer = &byKey[top]->outer;
        std::vector<const SketchProfile*> open{byKey[top]};
        while (!open.empty()) {
            const SketchProfile* q = open.back();
            open.pop_back();
            for (const SketchLoop& h : q->holes) {
                const SketchId k = loopKey(h);
                auto inner = byKey.find(k);
                if (chosen.count(k) && inner != byKey.end()) open.push_back(inner->second);
                else f.holes.push_back(&h);
            }
        }
        out.push_back(std::move(f));
    }
    return true;
}

} // namespace

BrepRef sketchSolids(const Sketch& sk, const std::vector<SketchProfile>& profiles,
                     const std::vector<SketchId>& keys, Real from, Real to, ElementId salt,
                     std::string* reason) {
    std::vector<SweptFace> faces;
    if (!facesForRegions(profiles, keys, faces, reason)) return {};
    return sweepSketchFaces(sk, faces, from, to, salt, reason);
}

BrepRef revolveSketch(const Sketch& sk, const std::vector<SketchProfile>& profiles,
                      const std::vector<SketchId>& keys, Vec2 axisAt, Vec2 axisDir, Real angle,
                      ElementId salt, std::string* reason) {
    std::vector<SweptFace> faces;
    if (!facesForRegions(profiles, keys, faces, reason)) return {};
    return revolveSketchFaces(sk, faces, axisAt, axisDir, angle, salt, reason);
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

namespace {

// A stretch along the three axes, done exactly.
//
// BRepBuilderAPI_GTransform can stretch anything, and does it by first turning
// every surface into a NURBS one: a box scaled along one axis comes back with
// six freeform faces, none of which Inset will take and none of which anything
// can tell is flat. But an axis-aligned stretch keeps most of what a part is
// made of exactly what it was. A plane stays a plane, a line a line; a cylinder
// standing along one of the axes stays a cylinder so long as the two axes
// across it stretch alike, and a circle in a plane that stretches alike stays a
// circle. Splines just have their poles moved.
//
// The price of keeping them is reparameterising: a plane's (u, v) and a line's
// length both change, so every curve on a face is carried through the same
// linear map the face was. Anything this does not know how to keep exactly
// sets `unsupported`, and the caller stretches the whole shape the general way
// instead -- a correct answer with freeform faces beats a wrong one without.
class AxisStretch : public BRepTools_Modification {
public:
    AxisStretch(const gp_XYZ& factors, const gp_XYZ& shift) : k_(factors), t_(shift) {}

    bool unsupported = false;

    Standard_Boolean NewSurface(const TopoDS_Face& F, Handle(Geom_Surface)& S, TopLoc_Location& L,
                                Standard_Real& Tol, Standard_Boolean& RevWires,
                                Standard_Boolean& RevFace) override {
        RevWires = RevFace = Standard_False;
        const FaceMap& fm = face(F);
        if (fm.surface.IsNull()) return Standard_False;
        S = fm.surface;
        L = TopLoc_Location();
        Tol = BRep_Tool::Tolerance(F) * grow();
        return Standard_True;
    }

    Standard_Boolean NewCurve(const TopoDS_Edge& E, Handle(Geom_Curve)& C, TopLoc_Location& L,
                              Standard_Real& Tol) override {
        const EdgeMap& em = edge(E);
        if (em.curve.IsNull()) return Standard_False;
        C = em.curve;
        L = TopLoc_Location();
        Tol = BRep_Tool::Tolerance(E) * grow();
        return Standard_True;
    }

    Standard_Boolean NewPoint(const TopoDS_Vertex& V, gp_Pnt& P, Standard_Real& Tol) override {
        P = gp_Pnt(apply(BRep_Tool::Pnt(V).XYZ()));
        Tol = BRep_Tool::Tolerance(V) * grow();
        return Standard_True;
    }

    Standard_Boolean NewCurve2d(const TopoDS_Edge& E, const TopoDS_Face& F, const TopoDS_Edge&,
                                const TopoDS_Face&, Handle(Geom2d_Curve)& C,
                                Standard_Real& Tol) override {
        const FaceMap& fm = face(F);
        const EdgeMap& em = edge(E);
        Standard_Real f = 0.0, l = 0.0;
        const Handle(Geom2d_Curve) old = BRep_Tool::CurveOnSurface(E, F, f, l);
        if (old.IsNull()) { unsupported = true; return Standard_False; }
        const Handle(Geom2d_Curve) mapped = map2d(old, fm, em);
        if (mapped.IsNull()) { unsupported = true; return Standard_False; }
        C = mapped;
        Tol = BRep_Tool::Tolerance(E) * grow();
        return Standard_True;
    }

    // An output, not an adjustment: what comes in is not the vertex's old
    // parameter, so it is read from the edge.
    Standard_Boolean NewParameter(const TopoDS_Vertex& V, const TopoDS_Edge& E, Standard_Real& P,
                                  Standard_Real& Tol) override {
        if (V.IsNull()) return Standard_False;
        const EdgeMap& em = edge(E);
        P = em.k * BRep_Tool::Parameter(V, E) + em.b;
        Tol = BRep_Tool::Tolerance(V) * grow();
        return Standard_True;
    }

    GeomAbs_Shape Continuity(const TopoDS_Edge& E, const TopoDS_Face& F1, const TopoDS_Face& F2,
                             const TopoDS_Edge&, const TopoDS_Face&, const TopoDS_Face&) override {
        return BRep_Tool::Continuity(E, F1, F2);
    }

private:
    // How a face's parameters move: (u, v) -> M (u, v) + o.
    struct FaceMap {
        Handle(Geom_Surface) surface;
        double m[2][2] = {{1, 0}, {0, 1}};
        double o[2] = {0, 0};
    };
    // How an edge's parameter moves: s -> k s + b.
    struct EdgeMap {
        Handle(Geom_Curve) curve;
        double k = 1.0, b = 0.0;
    };

    gp_XYZ k_, t_;
    NCollection_DataMap<TopoDS_Shape, FaceMap, TopTools_ShapeMapHasher> faces_;
    NCollection_DataMap<TopoDS_Shape, EdgeMap, TopTools_ShapeMapHasher> edges_;

    double grow() const { return std::max(k_.X(), std::max(k_.Y(), k_.Z())); }
    gp_XYZ apply(const gp_XYZ& p) const {
        return gp_XYZ(p.X() * k_.X() + t_.X(), p.Y() * k_.Y() + t_.Y(), p.Z() * k_.Z() + t_.Z());
    }
    gp_XYZ stretch(const gp_XYZ& v) const {                  // a direction, before normalising
        return gp_XYZ(v.X() * k_.X(), v.Y() * k_.Y(), v.Z() * k_.Z());
    }
    gp_XYZ squeeze(const gp_XYZ& n) const {                  // a normal: the inverse transpose
        return gp_XYZ(n.X() / k_.X(), n.Y() / k_.Y(), n.Z() / k_.Z());
    }
    // Which axis a direction lies along, or -1.
    static int alongAxis(const gp_Dir& d) {
        for (int i = 1; i <= 3; ++i)
            if (std::fabs(std::fabs(d.Coord(i)) - 1.0) < 1e-9) return i - 1;
        return -1;
    }
    double factor(int axis) const { return k_.Coord(axis + 1); }

    const FaceMap& face(const TopoDS_Face& F) {
        if (const FaceMap* hit = faces_.Seek(F)) return *hit;
        FaceMap fm;
        TopLoc_Location loc;
        Handle(Geom_Surface) s = BRep_Tool::Surface(F, loc);
        if (!s.IsNull() && !loc.IsIdentity())
            s = Handle(Geom_Surface)::DownCast(s->Transformed(loc.Transformation()));

        if (auto pl = Handle(Geom_Plane)::DownCast(s)) {
            const gp_Ax3 a = pl->Position();
            const gp_XYZ ax = stretch(a.XDirection().XYZ()), ay = stretch(a.YDirection().XYZ());
            gp_Ax3 n(gp_Pnt(apply(a.Location().XYZ())), gp_Dir(squeeze(a.Direction().XYZ())),
                     gp_Dir(ax));
            if (!a.Direct()) n.YReverse();
            const gp_XYZ nx = n.XDirection().XYZ(), ny = n.YDirection().XYZ();
            fm.m[0][0] = ax.Dot(nx); fm.m[0][1] = ay.Dot(nx);
            fm.m[1][0] = ax.Dot(ny); fm.m[1][1] = ay.Dot(ny);
            fm.surface = new Geom_Plane(n);
        } else if (auto cy = Handle(Geom_CylindricalSurface)::DownCast(s)) {
            // Standing along an axis, with the two across it stretching alike:
            // a wider or narrower cylinder, and taller or shorter by the third.
            const gp_Ax3 a = cy->Position();
            const int axis = alongAxis(a.Direction());
            if (axis >= 0) {
                const double across1 = factor((axis + 1) % 3), across2 = factor((axis + 2) % 3);
                if (std::fabs(across1 - across2) < 1e-12 * std::max(across1, across2)) {
                    gp_Ax3 n = a;
                    n.SetLocation(gp_Pnt(apply(a.Location().XYZ())));
                    fm.surface = new Geom_CylindricalSurface(n, cy->Radius() * across1);
                    fm.m[1][1] = factor(axis);
                }
            }
        } else if (auto bs = Handle(Geom_BSplineSurface)::DownCast(s)) {
            Handle(Geom_BSplineSurface) c = Handle(Geom_BSplineSurface)::DownCast(bs->Copy());
            for (int i = 1; i <= c->NbUPoles(); ++i)
                for (int j = 1; j <= c->NbVPoles(); ++j)
                    c->SetPole(i, j, gp_Pnt(apply(c->Pole(i, j).XYZ())));
            fm.surface = c;
        }
        if (fm.surface.IsNull()) unsupported = true;
        faces_.Bind(F, fm);
        return *faces_.Seek(F);
    }

    const EdgeMap& edge(const TopoDS_Edge& E) {
        if (const EdgeMap* hit = edges_.Seek(E)) return *hit;
        EdgeMap em;
        TopLoc_Location loc;
        Standard_Real f = 0.0, l = 0.0;
        Handle(Geom_Curve) c = BRep_Tool::Curve(E, loc, f, l);
        if (!c.IsNull() && !loc.IsIdentity())
            c = Handle(Geom_Curve)::DownCast(c->Transformed(loc.Transformation()));
        mapCurve(c, em);
        if (em.curve.IsNull()) unsupported = true;
        edges_.Bind(E, em);
        return *edges_.Seek(E);
    }

    // A 3D curve through the stretch, and how its parameter moves; a null
    // curve when it cannot be kept exact. A trimmed curve is its basis curve
    // trimmed again, at the parameters the basis curve's map takes its ends to.
    void mapCurve(const Handle(Geom_Curve)& c, EdgeMap& em) {
        if (auto tr = Handle(Geom_TrimmedCurve)::DownCast(c)) {
            mapCurve(tr->BasisCurve(), em);
            if (em.curve.IsNull()) return;
            em.curve = new Geom_TrimmedCurve(em.curve, em.k * tr->FirstParameter() + em.b,
                                             em.k * tr->LastParameter() + em.b);
            return;
        }
        if (auto ln = Handle(Geom_Line)::DownCast(c)) {
            const gp_Ax1 a = ln->Position();
            const gp_XYZ d = stretch(a.Direction().XYZ());
            em.k = d.Modulus();
            em.curve = new Geom_Line(gp_Pnt(apply(a.Location().XYZ())), gp_Dir(d));
        } else if (auto ci = Handle(Geom_Circle)::DownCast(c)) {
            // Still a circle only where its plane stretches alike both ways.
            const gp_Ax2 a = ci->Position();
            const gp_XYZ ax = stretch(a.XDirection().XYZ()), ay = stretch(a.YDirection().XYZ());
            const double lx = ax.Modulus(), ly = ay.Modulus();
            if (std::fabs(lx - ly) < 1e-12 * std::max(lx, ly) &&
                std::fabs(ax.Dot(ay)) < 1e-12 * lx * ly) {
                const gp_Ax2 n(gp_Pnt(apply(a.Location().XYZ())), gp_Dir(ax.Crossed(ay)), gp_Dir(ax));
                em.curve = new Geom_Circle(n, ci->Radius() * lx);
            }
        } else if (auto bs = Handle(Geom_BSplineCurve)::DownCast(c)) {
            Handle(Geom_BSplineCurve) n = Handle(Geom_BSplineCurve)::DownCast(bs->Copy());
            for (int i = 1; i <= n->NbPoles(); ++i) n->SetPole(i, gp_Pnt(apply(n->Pole(i).XYZ())));
            em.curve = n;
        }
    }

    // A curve on a face, carried through the face's map and the edge's.
    Handle(Geom2d_Curve) map2d(const Handle(Geom2d_Curve)& c, const FaceMap& fm, const EdgeMap& em) {
        auto M = [&](const gp_XY& p) {
            return gp_XY(fm.m[0][0] * p.X() + fm.m[0][1] * p.Y() + fm.o[0],
                         fm.m[1][0] * p.X() + fm.m[1][1] * p.Y() + fm.o[1]);
        };
        auto Mv = [&](const gp_XY& v) {
            return gp_XY(fm.m[0][0] * v.X() + fm.m[0][1] * v.Y(),
                         fm.m[1][0] * v.X() + fm.m[1][1] * v.Y());
        };
        if (auto tr = Handle(Geom2d_TrimmedCurve)::DownCast(c)) {
            const Handle(Geom2d_Curve) basis = map2d(tr->BasisCurve(), fm, em);
            if (basis.IsNull()) return {};
            return new Geom2d_TrimmedCurve(basis, em.k * tr->FirstParameter() + em.b,
                                           em.k * tr->LastParameter() + em.b);
        }
        if (auto ln = Handle(Geom2d_Line)::DownCast(c)) {
            // c(s) = P + s D, and c'(s') = M c((s' - b) / k).
            const gp_XY d = Mv(ln->Direction().XY()) / em.k;
            if (std::fabs(d.Modulus() - 1.0) > 1e-7) return {};
            const gp_XY p = M(ln->Location().XY()) - d * em.b;
            return new Geom2d_Line(gp_Pnt2d(p), gp_Dir2d(d));
        }
        if (auto ci = Handle(Geom2d_Circle)::DownCast(c)) {
            if (std::fabs(em.k - 1.0) > 1e-12 || std::fabs(em.b) > 1e-12) return {};
            const gp_Ax22d a = ci->Position();
            const gp_XY ax = Mv(a.XDirection().XY()), ay = Mv(a.YDirection().XY());
            const double lx = ax.Modulus(), ly = ay.Modulus();
            if (std::fabs(lx - ly) > 1e-9 * std::max(lx, ly) || std::fabs(ax.Dot(ay)) > 1e-9 * lx * ly)
                return {};
            return new Geom2d_Circle(gp_Ax22d(gp_Pnt2d(M(a.Location().XY())), gp_Dir2d(ax), gp_Dir2d(ay)),
                                     ci->Radius() * lx);
        }
        if (auto bs = Handle(Geom2d_BSplineCurve)::DownCast(c)) {
            TColgp_Array1OfPnt2d poles(1, bs->NbPoles());
            for (int i = 1; i <= bs->NbPoles(); ++i) poles(i) = gp_Pnt2d(M(bs->Pole(i).XY()));
            TColStd_Array1OfReal knots(1, bs->NbKnots());
            TColStd_Array1OfInteger mults(1, bs->NbKnots());
            for (int i = 1; i <= bs->NbKnots(); ++i) {
                knots(i) = em.k * bs->Knot(i) + em.b;
                mults(i) = bs->Multiplicity(i);
            }
            if (bs->IsRational()) {
                TColStd_Array1OfReal w(1, bs->NbPoles());
                bs->Weights(w);
                return new Geom2d_BSplineCurve(poles, w, knots, mults, bs->Degree(), bs->IsPeriodic());
            }
            return new Geom2d_BSplineCurve(poles, knots, mults, bs->Degree(), bs->IsPeriodic());
        }
        return {};
    }
};

// Stretches `s` along the axes by `factors` and then moves it by `shift`,
// keeping every surface it can. Null when something in it cannot be kept exact.
TopoDS_Shape stretchExactly(const TopoDS_Shape& s, const gp_XYZ& factors, const gp_XYZ& shift,
                            BRepTools_Modifier& modifier) {
    Handle(AxisStretch) stretch = new AxisStretch(factors, shift);
    modifier.Init(s);
    modifier.Perform(stretch);
    if (!modifier.IsDone() || stretch->unsupported) return {};
    const TopoDS_Shape out = modifier.ModifiedShape(s);
    if (!fullAnalyzerValid(out)) return {};
    return out;
}

} // namespace

BrepRef transformed(const BrepShape& s, const Mat4& m) {
    // A placement -- a rotation and a move, perhaps a uniform scale -- keeps
    // every surface what it was: a plane stays a plane and a cylinder a
    // cylinder. gp_Trsf is exactly that and nothing more, and handed a matrix
    // that stretches one way more than another it quietly averages the stretch
    // into a uniform scale of the same volume. That is how a box scaled along
    // one axis used to arrive at a boolean as a bigger cube.
    //
    // So first: is this a similarity? Its columns have to be at right angles
    // and all the same length.
    const Vec3 c0{m.col[0].x, m.col[0].y, m.col[0].z};
    const Vec3 c1{m.col[1].x, m.col[1].y, m.col[1].z};
    const Vec3 c2{m.col[2].x, m.col[2].y, m.col[2].z};
    const Real l0 = length(c0), l1 = length(c1), l2 = length(c2);
    const Real lmax = std::max(l0, std::max(l1, l2));
    if (lmax < 1e-12) return {};
    const Real tol = 1e-9 * lmax;
    const bool similar = std::fabs(l0 - l1) < 1e-7 * lmax && std::fabs(l0 - l2) < 1e-7 * lmax &&
                         std::fabs(dot(c0, c1)) < tol * lmax && std::fabs(dot(c0, c2)) < tol * lmax &&
                         std::fabs(dot(c1, c2)) < tol * lmax;
    const Real det = dot(c0, cross(c1, c2));
    // A reflection turns a solid inside out; mirrored() is the way to ask for one.
    if (det <= 0.0) return {};

    try {
        if (similar) {
            gp_Trsf t;
            t.SetValues(m.col[0].x, m.col[1].x, m.col[2].x, m.col[3].x,
                        m.col[0].y, m.col[1].y, m.col[2].y, m.col[3].y,
                        m.col[0].z, m.col[1].z, m.col[2].z, m.col[3].z);
            BRepBuilderAPI_Transform xf(s.shape, t, Standard_True);
            if (!xf.IsDone()) return {};
            return makeBrep(xf.Shape(), s.faceNames);
        }

        // A stretch along the axes -- what a Scale step asks for -- keeps its
        // planes, lines, and the cylinders it can. See AxisStretch.
        const bool alongAxes = std::fabs(m.col[0].y) + std::fabs(m.col[0].z) +
                               std::fabs(m.col[1].x) + std::fabs(m.col[1].z) +
                               std::fabs(m.col[2].x) + std::fabs(m.col[2].y) < 1e-12 * lmax;
        if (alongAxes) {
            BRepTools_Modifier modifier(Standard_False);
            const TopoDS_Shape out = stretchExactly(
                s.shape, gp_XYZ(m.col[0].x, m.col[1].y, m.col[2].z),
                gp_XYZ(m.col[3].x, m.col[3].y, m.col[3].z), modifier);
            if (!out.IsNull()) {
                TopTools_IndexedMapOfShape faces;
                TopExp::MapShapes(out, TopAbs_FACE, faces);
                std::vector<ElementId> names(static_cast<size_t>(faces.Extent()), kNoId);
                for (int i = 1; i <= s.faces.Extent(); ++i) {
                    const int at = faces.FindIndex(modifier.ModifiedShape(s.faces(i)));
                    if (at > 0) names[static_cast<size_t>(at - 1)] = s.faceNames[static_cast<size_t>(i - 1)];
                }
                return makeBrep(out, names);
            }
        }

        // Any other stretch. Nothing but a general surface can hold a circle
        // pulled into an ellipse, so the kernel converts what it has to -- the
        // price of the shape being the shape that was asked for. The faces are
        // followed through the modification rather than trusted to come out
        // in the same order.
        gp_GTrsf g;
        g.SetVectorialPart(gp_Mat(m.col[0].x, m.col[1].x, m.col[2].x,
                                  m.col[0].y, m.col[1].y, m.col[2].y,
                                  m.col[0].z, m.col[1].z, m.col[2].z));
        g.SetTranslationPart(gp_XYZ(m.col[3].x, m.col[3].y, m.col[3].z));
        //
        // On a copy, with every curve a flat face needs stored rather than
        // left to be worked out when asked for: OCCT's NURBS conversion reads
        // them as though they were all there, and on the output of a boolean
        // -- where they often are not -- it faults instead of refusing.
        BRepBuilderAPI_Copy copy(s.shape, Standard_True, Standard_False);
        if (!copy.IsDone()) return {};
        const TopoDS_Shape work = copy.Shape();
        for (TopExp_Explorer fx(work, TopAbs_FACE); fx.More(); fx.Next()) {
            const TopoDS_Face& face = TopoDS::Face(fx.Current());
            for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next())
                BRepLib::BuildPCurveForEdgeOnPlane(TopoDS::Edge(ex.Current()), face);
        }
        BRepBuilderAPI_GTransform xf(work, g, Standard_True);
        if (!xf.IsDone()) return {};
        const TopoDS_Shape out = xf.Shape();
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(out, TopAbs_FACE, faces);
        std::vector<ElementId> names(static_cast<size_t>(faces.Extent()), kNoId);
        for (int i = 1; i <= s.faces.Extent(); ++i) {
            const int at = faces.FindIndex(xf.ModifiedShape(copy.ModifiedShape(s.faces(i))));
            if (at > 0) names[static_cast<size_t>(at - 1)] = s.faceNames[static_cast<size_t>(i - 1)];
        }
        if (!fullAnalyzerValid(out)) return {};
        return makeBrep(out, names);
    } catch (const Standard_Failure&) {
        return {};
    }
}

// ---------------------------------------------------------------------------
// STEP.

namespace {

// A face's name on the way in from a file.
//
// There is nothing to inherit -- STEP carries geometry, not whatever Tangent
// called things -- so the name has to come from the face itself. Surface type
// and outward normal are the only stable properties available, and they are
// enough to keep two faces of an imported body distinct and to keep the same
// file importing to the same names twice.
//
// It is deliberately not a promise that the names survive re-importing an
// edited file. They will not, and nothing here pretends otherwise.
// DataExchange narrates. Reading one file prints a banner, a transfer mode, an
// entity count and a "Write Done" to stdout, which in a GUI program goes
// nowhere useful and in a CI log buries the thing you were reading it for.
// Failures are reported by return value here and always have been, so the
// printers are turned down to alarms once and left there.
void hushTheKernel() {
    static bool done = false;
    if (done) return;
    done = true;
    const Handle(Message_Messenger) m = Message::DefaultMessenger();
    if (m.IsNull()) return;
    for (Message_SequenceOfPrinters::Iterator it(m->Printers()); it.More(); it.Next())
        if (!it.Value().IsNull()) it.Value()->SetTraceLevel(Message_Alarm);
}

std::vector<ElementId> nameImportedFaces(const TopoDS_Shape& shape, ElementId salt) {
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(shape, TopAbs_FACE, faces);
    std::vector<ElementId> names(static_cast<size_t>(faces.Extent()), kNoId);

    for (int i = 1; i <= faces.Extent(); ++i) {
        const TopoDS_Face& face = TopoDS::Face(faces(i));
        const Vec3 n = outwardNormal(face);
        const auto type = static_cast<uint64_t>(BRepAdaptor_Surface(face).GetType());

        // Quantised, so that a normal which differs in the last bit between one
        // read and the next does not become a different face.
        auto q = [](Real v) { return static_cast<uint64_t>(std::llround(v * 4096.0)); };
        const uint64_t shape_ = mix64(type * 1000003ull + q(n.x)) ^
                                mix64(q(n.y) * 31ull) ^ mix64(q(n.z) * 131ull);

        names[static_cast<size_t>(i - 1)] =
            nameId(salt, IdRole::Patch, shape_, static_cast<ElementId>(i));
    }
    return names;
}

} // namespace

bool writeStep(const std::vector<const BrepShape*>& shapes, const std::string& path,
               std::string* reason) {
    if (shapes.empty()) {
        if (reason) *reason = "there is nothing to export";
        return false;
    }
    hushTheKernel();
    try {
        // AP214 is the interchange schema every CAD package reads. Units are
        // set explicitly: STEP has no default, and a file that does not say is
        // a file the other end has to guess at.
        STEPControl_Writer writer;
        Interface_Static::SetCVal("write.step.unit", "MM");
        Interface_Static::SetCVal("write.step.schema", "AP214IS");
        // Whole solids rather than shells: the receiving package should get a
        // thing with an inside, not a bag of surfaces.
        Interface_Static::SetIVal("write.step.nonmanifold", 0);

        for (const BrepShape* s : shapes) {
            if (!s || s->shape.IsNull()) continue;
            if (writer.Transfer(s->shape, STEPControl_AsIs) != IFSelect_RetDone) {
                if (reason) *reason = "the kernel would not convert one of the bodies";
                return false;
            }
        }
        if (writer.Write(path.c_str()) != IFSelect_RetDone) {
            if (reason) *reason = "the file could not be written";
            return false;
        }
        return true;
    } catch (const Standard_Failure& e) {
        if (reason) *reason = kernelReason(e, "the STEP file could not be written");
        return false;
    }
}

bool readStep(const std::string& path, ElementId salt, std::vector<BrepRef>& out,
              std::string* reason) {
    out.clear();
    hushTheKernel();
    try {
        STEPControl_Reader reader;
        Interface_Static::SetIVal("read.step.ideas", 1);
        Interface_Static::SetIVal("read.step.nonmanifold", 1);

        if (reader.ReadFile(path.c_str()) != IFSelect_RetDone) {
            if (reason) *reason = "the file could not be read as STEP";
            return false;
        }
        const int roots = reader.NbRootsForTransfer();
        if (roots <= 0) {
            if (reason) *reason = "the file has no geometry in it";
            return false;
        }
        reader.TransferRoots();

        // Every solid in the file becomes its own body, however the file chose
        // to nest them. A STEP assembly arrives as its parts, which is what a
        // program with no assembly structure of its own can honestly do with it.
        ElementId nth = 0;
        for (int i = 1; i <= reader.NbShapes(); ++i) {
            const TopoDS_Shape root = reader.Shape(i);
            if (root.IsNull()) continue;
            for (TopExp_Explorer ex(root, TopAbs_SOLID); ex.More(); ex.Next()) {
                const TopoDS_Shape solid = ex.Current();
                const ElementId mine = nameId(salt, IdRole::Split, ++nth);
                BrepRef r = makeBrep(solid, nameImportedFaces(solid, mine));
                if (r) out.push_back(std::move(r));
            }
        }

        if (out.empty()) {
            // Surfaces without a closed volume. Readable, but not something
            // this program can model on, and saying which it is beats "failed".
            if (reason)
                *reason = "the file has surfaces but no closed solid in it";
            return false;
        }
        return true;
    } catch (const Standard_Failure& e) {
        if (reason) *reason = kernelReason(e, "the STEP file could not be read");
        return false;
    }
}

BrepRef solidFromPlanarRegions(const PlanarRegions& in, ElementId salt, std::string* reason) {
    if (in.regions.size() < 4 || in.edgeEnds.size() % 2 != 0) {
        if (reason) *reason = "there are not enough faces to make a solid";
        return {};
    }
    hushTheKernel();
    try {
        std::vector<TopoDS_Vertex> verts(in.points.size());
        std::vector<char> vertMade(in.points.size(), 0);
        auto vertexAt = [&](uint32_t i) -> const TopoDS_Vertex& {
            if (!vertMade[i]) {
                verts[i] = BRepBuilderAPI_MakeVertex(
                    gp_Pnt(in.points[i].x, in.points[i].y, in.points[i].z));
                vertMade[i] = 1;
            }
            return verts[i];
        };

        const size_t edgeCount = in.edgeEnds.size() / 2;
        std::vector<TopoDS_Edge> edges(edgeCount);
        std::vector<char> edgeMade(edgeCount, 0);
        auto edgeAt = [&](uint32_t e) -> const TopoDS_Edge& {
            if (!edgeMade[e]) {
                BRepBuilderAPI_MakeEdge mk(vertexAt(in.edgeEnds[e * 2]),
                                           vertexAt(in.edgeEnds[e * 2 + 1]));
                if (mk.IsDone()) edges[e] = mk.Edge();
                edgeMade[e] = 1;
            }
            return edges[e];
        };

        BRep_Builder builder;
        TopoDS_Shell shell;
        builder.MakeShell(shell);

        for (const PlanarRegions::Region& region : in.regions) {
            // One wire per loop, from the shared edges. A shared edge is used
            // forwards by one region and backwards by the other, which is what
            // makes the two faces genuinely joined along it.
            std::vector<TopoDS_Wire> wires;
            std::vector<Real> areas;
            for (const PlanarRegions::Loop& loop : region.loops) {
                BRepBuilderAPI_MakeWire wire;
                Vec3 twiceArea{};
                for (size_t k = 0; k < loop.edges.size(); ++k) {
                    const uint32_t e = loop.edges[k];
                    if (e >= edgeCount || edgeAt(e).IsNull()) {
                        if (reason) *reason = "a boundary edge could not be built";
                        return {};
                    }
                    const bool forward = in.edgeEnds[e * 2] == loop.from[k];
                    wire.Add(forward ? edgeAt(e) : TopoDS::Edge(edgeAt(e).Reversed()));
                    if (!wire.IsDone()) {
                        if (reason) *reason = "a face boundary would not close";
                        return {};
                    }
                    const Vec3& p = in.points[loop.from[k]];
                    const Vec3& q = in.points[loop.from[(k + 1) % loop.from.size()]];
                    twiceArea = twiceArea + cross(p, q);
                }
                wires.push_back(wire.Wire());
                areas.push_back(dot(twiceArea, region.normal));
            }
            if (wires.empty()) continue;

            // The outside is the loop that winds positively about the normal and
            // encloses the most; everything else is a hole in it.
            size_t outer = 0;
            for (size_t i = 1; i < areas.size(); ++i)
                if (areas[i] > areas[outer]) outer = i;

            const gp_Pln plane(gp_Pnt(region.point.x, region.point.y, region.point.z),
                               gp_Dir(region.normal.x, region.normal.y, region.normal.z));
            BRepBuilderAPI_MakeFace face(plane, wires[outer], Standard_True);
            if (!face.IsDone()) {
                if (reason) *reason = "a planar face could not be made from its boundary";
                return {};
            }
            for (size_t i = 0; i < wires.size(); ++i)
                if (i != outer) face.Add(wires[i]);
            if (!face.IsDone()) {
                if (reason) *reason = "a hole could not be cut into its face";
                return {};
            }
            builder.Add(shell, face.Face());
        }

        shell.Closed(Standard_True);
        BRepBuilderAPI_MakeSolid mk(shell);
        if (!mk.IsDone()) {
            if (reason) *reason = "the faces would not close into a solid";
            return {};
        }
        TopoDS_Shape solid = mk.Solid();

        // The mesh was consistently wound and every face was built on a plane
        // facing its own outward normal, so the orientation is already right
        // and the shape-fixing pass the triangle route needed -- a third of its
        // time -- has nothing to do. The volume is the check that this held.
        GProp_GProps props;
        BRepGProp::VolumeProperties(solid, props);
        if (props.Mass() < 0.0) solid.Reverse();

        // The check. Not the kernel's general validity analysis, which asks
        // every edge of a face whether it crosses every other and so is
        // quadratic in them -- on a face drilled with 64 holes of a hundred
        // segments each that was twelve of the conversion's thirteen seconds.
        //
        // What can go wrong here is narrower, and two things catch it exactly.
        // Closed: every edge shared by two faces, or a boundary was traced
        // wrong. Same volume as the mesh: a face facing the wrong way, a hole
        // cut into the wrong face or not cut at all, a region missing -- each
        // changes the enclosed volume, and the mesh's own volume is known to
        // float precision from the triangles.
        const Real got = std::fabs(props.Mass());
        const Real want = std::fabs(in.volume);
        if (got < 1e-12 || std::fabs(got - want) > std::max<Real>(want * 1e-6, 1e-9)) {
            if (reason) *reason = "the faces joined but do not enclose what the mesh did";
            return {};
        }
        BrepRef out = makeBrep(solid, nameImportedFaces(solid, salt));
        if (!out || !closedShell(*out)) {
            if (reason) *reason = "the faces joined but left a gap";
            return {};
        }
        return out;
    } catch (const Standard_Failure& e) {
        if (reason) *reason = kernelReason(e, "the mesh could not be made solid");
        return {};
    }
}

BrepRef solidFromTriangles(const std::vector<Vec3>& points,
                           const std::vector<uint32_t>& tris,
                           const std::vector<uint32_t>& edgeEnds,
                           const std::vector<uint32_t>& triEdges,
                           ElementId salt, std::string* reason) {
    if (tris.size() < 9 || tris.size() % 3 != 0 || triEdges.size() != tris.size() ||
        edgeEnds.size() % 2 != 0) {
        if (reason) *reason = "there are not enough triangles to make a solid";
        return {};
    }
    hushTheKernel();
    try {
        // Each point once, each edge once, each face once, and the shell built
        // from them directly. Nothing is searched for: the caller already knows
        // which triangles meet along which edge, which is what sewing spends
        // its time finding out.
        std::vector<TopoDS_Vertex> verts(points.size());
        std::vector<bool> vertMade(points.size(), false);
        auto vertexAt = [&](uint32_t i) -> const TopoDS_Vertex& {
            if (!vertMade[i]) {
                verts[i] = BRepBuilderAPI_MakeVertex(
                    gp_Pnt(points[i].x, points[i].y, points[i].z));
                vertMade[i] = true;
            }
            return verts[i];
        };

        const size_t edgeCount = edgeEnds.size() / 2;
        std::vector<TopoDS_Edge> edges(edgeCount);
        std::vector<bool> edgeMade(edgeCount, false);
        auto edgeAt = [&](uint32_t e) -> const TopoDS_Edge& {
            if (!edgeMade[e]) {
                BRepBuilderAPI_MakeEdge mk(vertexAt(edgeEnds[e * 2]),
                                           vertexAt(edgeEnds[e * 2 + 1]));
                if (mk.IsDone()) edges[e] = mk.Edge();
                edgeMade[e] = true;
            }
            return edges[e];
        };

        BRep_Builder builder;
        TopoDS_Shell shell;
        builder.MakeShell(shell);
        int added = 0;

        for (size_t t = 0; t + 2 < tris.size(); t += 3) {
            BRepBuilderAPI_MakeWire wire;
            bool good = true;
            for (int k = 0; k < 3 && good; ++k) {
                const uint32_t e = triEdges[t + k];
                if (e >= edgeCount) { good = false; break; }
                const TopoDS_Edge& edge = edgeAt(e);
                if (edge.IsNull()) { good = false; break; }
                // The shared edge runs one way; this triangle may want the
                // other. Reversing the *use* rather than making a second edge
                // is what keeps the two faces genuinely joined along it.
                const bool forward = edgeEnds[e * 2] == tris[t + k];
                wire.Add(forward ? edge : TopoDS::Edge(edge.Reversed()));
                if (!wire.IsDone()) good = false;
            }
            if (!good || !wire.IsDone()) continue;

            BRepBuilderAPI_MakeFace face(wire.Wire(), Standard_True);
            if (!face.IsDone()) continue;       // a sliver with no plane in it
            builder.Add(shell, face.Face());
            ++added;
        }
        if (added < 4) {
            if (reason) *reason = "too few of the triangles could be made into faces";
            return {};
        }

        // Closing the shell into a solid is what gives it an inside, and it is
        // also the check that the surface really was closed.
        shell.Closed(BRep_Tool::IsClosed(shell));
        BRepBuilderAPI_MakeSolid mk(shell);
        if (!mk.IsDone()) {
            if (reason) *reason = "the surface would not close into a solid";
            return {};
        }
        ShapeFix_Solid fix(mk.Solid());
        fix.Perform();
        TopoDS_Shape solid = fix.Solid();
        if (solid.IsNull()) solid = mk.Solid();
        if (solid.IsNull()) {
            if (reason) *reason = "the surface closed up but would not become a solid";
            return {};
        }

        // The step that makes the whole thing worth doing: every pair of
        // coplanar neighbours becomes one face. A box that arrived as twelve
        // triangles leaves as six faces with four edges each.
        ShapeUpgrade_UnifySameDomain unify(solid, Standard_True, Standard_True,
                                           Standard_True);
        unify.Build();
        const TopoDS_Shape merged = unify.Shape();
        const TopoDS_Shape result = merged.IsNull() ? solid : merged;

        // Names minted from the faces, as an import's always are -- see
        // nameImportedFaces. There is nothing upstream to inherit from.
        return makeBrep(result, nameImportedFaces(result, salt));
    } catch (const Standard_Failure& e) {
        if (reason) *reason = kernelReason(e, "the mesh could not be made solid");
        return {};
    }
}

size_t separateSolids(const BrepShape& s, std::vector<BrepRef>& out) {
    out.clear();
    if (s.shape.IsNull()) return 0;
    struct Piece { BrepRef shape; Real volume; };
    std::vector<Piece> pieces;
    try {
        TopTools_IndexedMapOfShape solids;
        TopExp::MapShapes(s.shape, TopAbs_SOLID, solids);
        for (int i = 1; i <= solids.Extent(); ++i) {
            const TopoDS_Shape& solid = solids(i);
            // Each face keeps the name it had in the whole: it is the same face,
            // and anything that referred to it should still find it.
            TopTools_IndexedMapOfShape faces;
            TopExp::MapShapes(solid, TopAbs_FACE, faces);
            std::vector<ElementId> names(static_cast<size_t>(faces.Extent()), kNoId);
            for (int f = 1; f <= faces.Extent(); ++f) {
                const int at = s.faces.FindIndex(faces(f));
                if (at > 0 && static_cast<size_t>(at - 1) < s.faceNames.size())
                    names[static_cast<size_t>(f - 1)] = s.faceNames[static_cast<size_t>(at - 1)];
            }
            GProp_GProps props;
            BRepGProp::VolumeProperties(solid, props);
            BrepRef r = makeBrep(solid, names);
            if (r) pieces.push_back({std::move(r), std::fabs(props.Mass())});
        }
    } catch (const Standard_Failure&) {
        out.clear();
        return 0;
    }
    std::stable_sort(pieces.begin(), pieces.end(),
                     [](const Piece& a, const Piece& b) { return a.volume > b.volume; });
    for (Piece& p : pieces) out.push_back(std::move(p.shape));
    return out.size();
}

bool splitByPlane(const BrepShape& s, Vec3 point, Vec3 normal, ElementId salt,
                  BrepRef& above, BrepRef& below, std::string* reason) {
    if (reason) reason->clear();
    const Real len = length(normal);
    if (s.shape.IsNull() || !(len > 1e-12)) {
        if (reason) *reason = "there is nothing to split, or no plane to split it by";
        return false;
    }
    const Vec3 n = normal * (Real(1) / len);
    try {
        // The plane as one face, big enough to reach past the body however the
        // body sits relative to the point it was given.
        Bnd_Box box;
        BRepBndLib::Add(s.shape, box);
        Standard_Real x0, y0, z0, x1, y1, z1;
        box.Get(x0, y0, z0, x1, y1, z1);
        const Vec3 lo{x0, y0, z0}, hi{x1, y1, z1};
        const Real reach = length(hi - lo) + length((lo + hi) * Real(0.5) - point) + 1;
        const gp_Pln plane(gp_Pnt(point.x, point.y, point.z), gp_Dir(n.x, n.y, n.z));
        BRepBuilderAPI_MakeFace face(plane, -reach, reach, -reach, reach);
        if (!face.IsDone()) {
            if (reason) *reason = "the cutting plane could not be made";
            return false;
        }
        const TopoDS_Face tool = face.Face();
        BrepRef toolShape = makeBrep(tool, nameImportedFaces(tool, nameId(salt, IdRole::Split, 1)));

        BRepAlgoAPI_Splitter splitter;
        TopTools_ListOfShape arguments, tools;
        arguments.Append(s.shape);
        tools.Append(tool);
        splitter.SetArguments(arguments);
        splitter.SetTools(tools);
        // As the booleans: across the cores, with boxes that turn with their
        // faces, and never in an isolated child.
        splitter.SetRunParallel(!inIsolatedChild());
        splitter.SetUseOBB(Standard_True);
        splitter.Build();
        if (!splitter.IsDone() || splitter.HasErrors()) {
            if (reason) *reason = "the kernel could not cut it along that plane";
            return false;
        }
        const TopoDS_Shape result = splitter.Shape();
        if (!acceptable(result, reason)) return false;
        const std::vector<ElementId> names =
            propagateNames(splitter, {{&s}, {toolShape.get()}}, result, salt);

        TopTools_IndexedMapOfShape resultFaces;
        TopExp::MapShapes(result, TopAbs_FACE, resultFaces);

        // Each solid to its side. After the cut every solid lies wholly on one
        // side of the plane, touching it at most, so the point of it furthest
        // from the plane says which side.
        //
        // Found as cheaply as the shape allows. Its vertices first: on a part
        // made of flat faces that settles it, and a volume integral to learn one
        // sign cost a tenth of a second on a converted scan. But a curved solid
        // can have every vertex on the cut -- half a cylinder cut through its
        // axis, half a sphere at its equator -- so then points along its edges'
        // actual curves, and, where even those lie in the plane as a torus's do,
        // its volume and the side its centre is on.
        //
        // Nothing with volume is ever dropped. The first version dropped any
        // solid whose vertices were all on the plane as a sliver; the halves of
        // a cylinder, a sphere and a torus all went, and only the fallback
        // below -- triggered because a side came back empty -- hid it. A side
        // holding two solids, one of them like that, would not have been empty,
        // and would have lost a piece without a word.
        BRep_Builder builder;
        TopoDS_Compound up, down;
        builder.MakeCompound(up);
        builder.MakeCompound(down);
        int ups = 0, downs = 0;
        TopoDS_Shape oneUp, oneDown;
        const Real sliver = reach * 1e-9;
        auto signedDistance = [&](const gp_Pnt& p) {
            return (p.X() - point.x) * n.x + (p.Y() - point.y) * n.y + (p.Z() - point.z) * n.z;
        };
        for (TopExp_Explorer ex(result, TopAbs_SOLID); ex.More(); ex.Next()) {
            const TopoDS_Shape& solid = ex.Current();
            Real furthest = 0;
            auto consider = [&](Real d) { if (std::fabs(d) > std::fabs(furthest)) furthest = d; };

            for (TopExp_Explorer v(solid, TopAbs_VERTEX); v.More(); v.Next())
                consider(signedDistance(BRep_Tool::Pnt(TopoDS::Vertex(v.Current()))));

            if (std::fabs(furthest) <= sliver) {
                for (TopExp_Explorer e(solid, TopAbs_EDGE); e.More(); e.Next()) {
                    const TopoDS_Edge& edge = TopoDS::Edge(e.Current());
                    if (BRep_Tool::Degenerated(edge)) continue;
                    BRepAdaptor_Curve curve(edge);
                    const Standard_Real t0 = curve.FirstParameter(), t1 = curve.LastParameter();
                    for (Real f : {0.25, 0.5, 0.75}) consider(signedDistance(curve.Value(t0 + (t1 - t0) * f)));
                }
            }

            if (std::fabs(furthest) <= sliver) {
                GProp_GProps props;
                BRepGProp::VolumeProperties(solid, props);
                // Negligible against a cube the size of the cut, not against zero.
                if (std::fabs(props.Mass()) <= reach * reach * reach * 1e-15) continue;
                consider(signedDistance(props.CentreOfMass()));
                if (furthest == 0) {
                    // A real volume with its centre exactly on the plane cannot
                    // lie wholly on one side of it: the cut did not separate it.
                    if (reason) *reason = "the cut left a solid it could not place on either side";
                    return false;
                }
            }

            if (furthest > 0) { builder.Add(up, solid); oneUp = solid; ++ups; }
            else              { builder.Add(down, solid); oneDown = solid; ++downs; }
        }
        if (ups == 0 || downs == 0) {
            if (reason) *reason = "the plane does not cut through it";
            return false;
        }

        // A side with one solid is that solid, not a compound holding it.
        auto keep = [&](const TopoDS_Shape& shape) {
            TopTools_IndexedMapOfShape faces;
            TopExp::MapShapes(shape, TopAbs_FACE, faces);
            std::vector<ElementId> own(static_cast<size_t>(faces.Extent()), kNoId);
            for (int f = 1; f <= faces.Extent(); ++f) {
                const int at = resultFaces.FindIndex(faces(f));
                if (at > 0) own[static_cast<size_t>(f - 1)] = names[static_cast<size_t>(at - 1)];
            }
            return makeBrep(shape, own);
        };
        above = keep(ups == 1 ? oneUp : TopoDS_Shape(up));
        below = keep(downs == 1 ? oneDown : TopoDS_Shape(down));
        return above && below;
    } catch (const Standard_Failure& e) {
        if (reason) *reason = kernelReason(e, "the body could not be split");
        return false;
    }
}

BrepRef mirrored(const BrepShape& s, Vec3 point, Vec3 normal) {
    const Vec3 n = normalize(normal);
    if (!(length(n) > 0.5)) return {};
    gp_Trsf t;
    // gp_Ax2's main direction is the plane's normal, so this is the mirror in
    // that plane. Building it this way rather than from a matrix is what lets
    // BRepBuilderAPI_Transform reverse the orientations for us.
    t.SetMirror(gp_Ax2(gp_Pnt(point.x, point.y, point.z), gp_Dir(n.x, n.y, n.z)));
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
