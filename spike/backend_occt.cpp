// Tangent - Stage 0 spike: the part catalogue realised in OpenCASCADE.
//
// Written the way the real backend would be, not the way a demo would: every
// operation is checked, every failure carries a reason back to the caller, and
// nothing is assumed about the order OCCT produces its faces and edges in. The
// selection helpers resolve edges by position for exactly that reason.
#include "backends.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBndLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <Poly_Triangulation.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>

#include <cmath>
#include <sstream>

namespace spike {
namespace {

constexpr double kTol = 1e-6;

bool near(double a, double b, double tol = 1e-6) { return std::fabs(a - b) < tol; }

TopoDS_Shape boxAt(double cx, double cy, double z0, double w, double d, double h) {
    return BRepPrimAPI_MakeBox(gp_Pnt(cx - w / 2, cy - d / 2, z0), w, d, h).Shape();
}

TopoDS_Shape cylAt(double cx, double cy, double z0, double dia, double h) {
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(cx, cy, z0), gp_Dir(0, 0, 1)), dia / 2, h).Shape();
}

// Both boolean wrappers report the reason on failure rather than a bool: a
// refusal the interface cannot explain is the thing this project already
// decided it will not ship.
bool combine(TopoDS_Shape& shape, const TopoDS_Shape& tool, bool cut, std::string& why) {
    try {
        if (cut) {
            BRepAlgoAPI_Cut op(shape, tool);
            if (!op.IsDone() || op.HasErrors()) {
                std::ostringstream os; op.DumpErrors(os); why = os.str(); return false;
            }
            shape = op.Shape();
        } else {
            BRepAlgoAPI_Fuse op(shape, tool);
            if (!op.IsDone() || op.HasErrors()) {
                std::ostringstream os; op.DumpErrors(os); why = os.str(); return false;
            }
            shape = op.Shape();
        }
    } catch (const Standard_Failure& e) {
        why = e.GetMessageString() ? e.GetMessageString() : "exception";
        return false;
    }
    return true;
}

// The top of a bore: the boss standing over it if there is one, otherwise the
// plate. Both backends compute this the same way so that "the rim" means the
// same edge in each.
double bodyTopZ(const PartSpec& s, double x, double y) {
    double z = s.plateH;
    for (const Boss& b : s.bosses)
        if (near(b.x, x, 1e-3) && near(b.y, y, 1e-3)) z = s.plateH + b.height;
    return z;
}

void uniqueEdges(const TopoDS_Shape& shape, TopTools_IndexedMapOfShape& out) {
    TopExp::MapShapes(shape, TopAbs_EDGE, out);   // an explorer would visit shared edges twice
}

std::vector<TopoDS_Edge> selectEdges(const TopoDS_Shape& shape, const PartSpec& s) {
    TopTools_IndexedMapOfShape map;
    uniqueEdges(shape, map);
    std::vector<TopoDS_Edge> picked;

    for (int i = 1; i <= map.Extent(); ++i) {
        const TopoDS_Edge& e = TopoDS::Edge(map(i));
        BRepAdaptor_Curve c(e);

        if (s.target == FilletTarget::BoreRims || s.target == FilletTarget::AllTopEdges) {
            if (c.GetType() == GeomAbs_Circle) {
                const gp_Circ circ = c.Circle();
                const gp_Pnt ctr = circ.Location();
                for (const Bore& b : s.bores) {
                    if (near(ctr.X(), b.x, 1e-3) && near(ctr.Y(), b.y, 1e-3) &&
                        near(ctr.Z(), bodyTopZ(s, b.x, b.y), 1e-3) &&
                        near(circ.Radius(), b.dia / 2, 1e-3)) {
                        picked.push_back(e);
                        break;
                    }
                }
                continue;
            }
        }
        if (s.target == FilletTarget::TopOuterEdges || s.target == FilletTarget::AllTopEdges) {
            if (c.GetType() != GeomAbs_Line) continue;
            const gp_Pnt a = BRep_Tool::Pnt(TopExp::FirstVertex(e));
            const gp_Pnt b = BRep_Tool::Pnt(TopExp::LastVertex(e));
            if (!near(a.Z(), s.plateH, 1e-6) || !near(b.Z(), s.plateH, 1e-6)) continue;
            const double mx = (a.X() + b.X()) / 2, my = (a.Y() + b.Y()) / 2;
            if (near(std::fabs(mx), s.plateW / 2, 1e-6) || near(std::fabs(my), s.plateD / 2, 1e-6))
                picked.push_back(e);
        }
    }
    return picked;
}

int faceCount(const TopoDS_Shape& s) {
    TopTools_IndexedMapOfShape m;
    TopExp::MapShapes(s, TopAbs_FACE, m);
    return m.Extent();
}

} // namespace

Result runOcct(const PartSpec& s) {
    Result r;

    // ---- Construction ------------------------------------------------------
    const double t0 = nowMs();
    TopoDS_Shape shape = boxAt(0, 0, 0, s.plateW, s.plateD, s.plateH);
    std::string why;

    if (s.wallH > 0.0) {
        const TopoDS_Shape wall = boxAt(0, -s.plateD / 2 + s.wallT / 2, 0, s.plateW, s.wallT, s.wallH);
        if (!combine(shape, wall, false, why)) { r.buildNote = "wall fuse: " + why; return r; }
    }
    for (const Boss& b : s.bosses) {
        if (!combine(shape, cylAt(b.x, b.y, s.plateH, b.dia, b.height), false, why)) {
            r.buildNote = "boss fuse: " + why; return r;
        }
    }
    for (const Bore& b : s.bores) {
        // Over-length by a millimetre at each end. Flush tool faces are the
        // classic boolean hazard and the mesh kernel is run the same way, so
        // neither backend is being handed an easier problem than the other.
        const double top = bodyTopZ(s, b.x, b.y);
        if (!combine(shape, cylAt(b.x, b.y, -1.0, b.dia, top + 2.0), true, why)) {
            r.buildNote = "bore cut: " + why; return r;
        }
    }
    for (const Pocket& p : s.pockets) {
        const double z0 = s.plateH - p.depth;
        if (!combine(shape, boxAt(p.x, p.y, z0 - 1.0, p.w, p.d, p.depth + 2.0), true, why)) {
            r.buildNote = "pocket cut: " + why; return r;
        }
    }
    r.buildMs = nowMs() - t0;
    r.built = true;

    // ---- Fillet ------------------------------------------------------------
    if (s.target != FilletTarget::None && s.filletRadius > 0.0) {
        const std::vector<TopoDS_Edge> edges = selectEdges(shape, s);
        r.filletEdges = static_cast<int>(edges.size());
        const double t1 = nowMs();
        try {
            BRepFilletAPI_MakeFillet fil(shape);
            for (const TopoDS_Edge& e : edges) fil.Add(s.filletRadius, e);
            fil.Build();
            if (!fil.IsDone()) {
                std::ostringstream os;
                os << "not done";
                if (fil.NbFaultyContours() > 0) os << ", " << fil.NbFaultyContours() << " faulty contours";
                if (fil.NbFaultyVertices() > 0) os << ", " << fil.NbFaultyVertices() << " faulty vertices";
                r.filletNote = os.str();
            } else {
                shape = fil.Shape();
                r.filletOk = true;
            }
        } catch (const Standard_Failure& e) {
            r.filletNote = std::string("exception: ") + (e.GetMessageString() ? e.GetMessageString() : "?");
        }
        r.filletMs = nowMs() - t1;
    } else {
        r.filletOk = true;   // nothing asked for is not a failure
    }

    // ---- What came out -----------------------------------------------------
    r.faces = faceCount(shape);
    try {
        const BRepCheck_Analyzer check(shape);
        r.valid = check.IsValid();
        if (!r.valid) r.validNote = "BRepCheck_Analyzer: invalid";
        GProp_GProps props;
        BRepGProp::VolumeProperties(shape, props);
        r.volumeMm3 = props.Mass();
        if (r.volumeMm3 <= 0.0) { r.valid = false; r.validNote = "volume <= 0"; }
    } catch (const Standard_Failure& e) {
        r.valid = false;
        r.validNote = std::string("check threw: ") + (e.GetMessageString() ? e.GetMessageString() : "?");
    }

    // ---- Display cost ------------------------------------------------------
    // Chord deviation tied to the part, not to a constant: a fixed 0.1mm is
    // wrong on a 5mm part and wasteful on a 500mm one.
    Bnd_Box bb;
    BRepBndLib::Add(shape, bb);
    double xa, ya, za, xb, yb, zb;
    bb.Get(xa, ya, za, xb, yb, zb);
    const double diag = std::sqrt((xb - xa) * (xb - xa) + (yb - ya) * (yb - ya) + (zb - za) * (zb - za));
    r.deviationMm = diag / 2000.0;

    BRepTools::Clean(shape);   // do not time a cached triangulation
    const double t2 = nowMs();
    BRepMesh_IncrementalMesh mesher(shape, r.deviationMm, Standard_False, 0.35, Standard_True);
    r.tessMs = nowMs() - t2;
    (void)mesher;

    int tris = 0;
    for (TopExp_Explorer e(shape, TopAbs_FACE); e.More(); e.Next()) {
        TopLoc_Location loc;
        const Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(TopoDS::Face(e.Current()), loc);
        if (!tri.IsNull()) tris += tri->NbTriangles();
    }
    r.triangles = tris;
    (void)kTol;
    return r;
}

} // namespace spike
