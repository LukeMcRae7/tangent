// Tangent - Stage 0 spike: can the tessellator be made fast enough?
//
// The first run says a 206-face part costs ~170 ms, six times the budget. That
// number is only meaningful if it is the tessellator's best, so this sweeps the
// parameters that actually move it: the angular limit (which is what drives the
// triangle count on cylinders and fillets, not the chord deviation), the
// minimum element size, and OCCT's per-face deflection control.
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBndLib.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <Bnd_Box.hxx>
#include <IMeshTools_Parameters.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>

#include <chrono>
#include <cmath>
#include <cstdio>

using Clock = std::chrono::steady_clock;
static double ms(const Clock::time_point& t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}
static int faceCount(const TopoDS_Shape& s) {
    TopTools_IndexedMapOfShape m; TopExp::MapShapes(s, TopAbs_FACE, m); return m.Extent();
}
static int triangles(const TopoDS_Shape& s) {
    int n = 0;
    for (TopExp_Explorer e(s, TopAbs_FACE); e.More(); e.Next()) {
        TopLoc_Location loc;
        const Handle(Poly_Triangulation) t = BRep_Tool::Triangulation(TopoDS::Face(e.Current()), loc);
        if (!t.IsNull()) n += t->NbTriangles();
    }
    return n;
}

static TopoDS_Shape gridPart(int n, double pitch, double dia, double filletR) {
    const double side = pitch * (n + 1);
    TopoDS_Shape shape = BRepPrimAPI_MakeBox(gp_Pnt(-side / 2, -side / 2, 0), side, side, 10).Shape();
    const double first = -pitch * (n - 1) / 2.0;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            BRepAlgoAPI_Cut cut(shape, BRepPrimAPI_MakeCylinder(
                gp_Ax2(gp_Pnt(first + pitch * i, first + pitch * j, -1), gp_Dir(0, 0, 1)), dia / 2, 12).Shape());
            if (cut.IsDone()) shape = cut.Shape();
        }
    if (filletR <= 0) return shape;
    BRepFilletAPI_MakeFillet fil(shape);
    TopTools_IndexedMapOfShape edges;
    TopExp::MapShapes(shape, TopAbs_EDGE, edges);
    for (int i = 1; i <= edges.Extent(); ++i) {
        const TopoDS_Edge e = TopoDS::Edge(edges(i));
        BRepAdaptor_Curve c(e);
        if (c.GetType() == GeomAbs_Circle && std::fabs(c.Circle().Location().Z() - 10.0) < 1e-6)
            fil.Add(filletR, e);
    }
    fil.Build();
    return fil.IsDone() ? fil.Shape() : shape;
}

int main() {
    const TopoDS_Shape shape = gridPart(10, 18, 8, 1.0);
    Bnd_Box bb; BRepBndLib::Add(shape, bb);
    double xa, ya, za, xb, yb, zb; bb.Get(xa, ya, za, xb, yb, zb);
    const double diag = std::sqrt((xb-xa)*(xb-xa) + (yb-ya)*(yb-ya) + (zb-za)*(zb-za));

    std::printf("\n=== %d faces, %.0f mm across: what the tessellator responds to ===\n\n",
                faceCount(shape), diag);
    std::printf("  %-9s %-7s %-8s %-9s %-9s %10s %10s\n",
                "deviation", "angle", "minsize", "control", "parallel", "triangles", "time");

    struct Cfg { double devDiv, angle, minSize; bool control, parallel; };
    const Cfg cfgs[] = {
        {2000,  0.35, 0,    true,  true},    // what the first run used
        {2000,  0.35, 0,    false, true},
        {2000,  0.7,  0,    true,  true},
        {2000,  1.0,  0,    true,  true},
        {2000,  1.0,  0,    false, true},
        {2000,  1.0,  0.5,  false, true},
        {2000,  1.0,  0.5,  false, false},   // what one core would cost
        {500,   1.0,  0.5,  false, true},
        {8000,  0.7,  0,    false, true},    // zoomed in, still tuned
    };

    for (const Cfg& c : cfgs) {
        const double dev = diag / c.devDiv;
        BRepTools::Clean(shape);

        IMeshTools_Parameters p;
        p.Deflection = dev;
        p.Angle = c.angle;
        p.MinSize = c.minSize > 0 ? c.minSize : dev * 0.1;
        p.InParallel = c.parallel;
        p.ControlSurfaceDeflection = c.control;
        p.Relative = Standard_False;

        const auto t0 = Clock::now();
        BRepMesh_IncrementalMesh mesher(shape, p);
        const double t = ms(t0);
        (void)mesher;
        std::printf("  %-9.4f %-7.2f %-8.2f %-9s %-9s %10d %8.1f ms\n",
                    dev, c.angle, p.MinSize, c.control ? "on" : "off",
                    c.parallel ? "on" : "off", triangles(shape), t);
    }
    // Two questions the table above cannot answer, and both decide whether the
    // cost is a hitch on every frame or a cost paid once per zoom step.
    IMeshTools_Parameters p;
    p.Deflection = diag / 2000;
    p.Angle = 1.0;
    p.MinSize = 0.5;
    p.InParallel = Standard_True;
    p.ControlSurfaceDeflection = Standard_False;

    BRepTools::Clean(shape);
    { const auto t0 = Clock::now(); BRepMesh_IncrementalMesh m(shape, p); (void)m;
      std::printf("\n  cold, whole part                    %8.1f ms\n", ms(t0)); }
    { const auto t0 = Clock::now(); BRepMesh_IncrementalMesh m(shape, p); (void)m;
      std::printf("  again, same parameters             %8.1f ms   <- redraw with no zoom change\n", ms(t0)); }

    // One face re-meshed on its own: what an edit to a single feature should
    // cost if the tessellation is kept per face rather than per body.
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(shape, TopAbs_FACE, faces);
    double oneFace = 0;
    int measured = 0;
    for (int i = 1; i <= faces.Extent() && measured < 10; ++i) {
        const TopoDS_Face f = TopoDS::Face(faces(i));
        BRepTools::Clean(f);
        const auto t0 = Clock::now();
        BRepMesh_IncrementalMesh m(f, p);
        oneFace += ms(t0);
        (void)m;
        ++measured;
    }
    std::printf("  ten faces, one at a time           %8.1f ms   <- %.1f ms each\n",
                oneFace, oneFace / measured);

    std::printf("\n");
    return 0;
}
