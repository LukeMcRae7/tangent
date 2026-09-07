// Tangent - Stage 0 spike: what does display cost on a B-rep?
//
// The exit criterion is a 200-face part tessellated in under 30 ms, because
// that is the budget for a zoom change. The interesting number is not the first
// tessellation, though: it is what a zoom costs, since the whole point of exact
// geometry is that a hole gets rounder as you zoom in, which means re-meshing.
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
    TopTools_IndexedMapOfShape m;
    TopExp::MapShapes(s, TopAbs_FACE, m);
    return m.Extent();
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

// A part with a couple of hundred faces, built the way one gets there in
// practice: a lot of holes, each with a rounded rim.
static TopoDS_Shape gridPart(int n, double pitch, double dia, double filletR, int& filletedEdges) {
    const double side = pitch * (n + 1);
    TopoDS_Shape shape = BRepPrimAPI_MakeBox(gp_Pnt(-side / 2, -side / 2, 0), side, side, 10).Shape();
    const double first = -pitch * (n - 1) / 2.0;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            const TopoDS_Shape tool = BRepPrimAPI_MakeCylinder(
                gp_Ax2(gp_Pnt(first + pitch * i, first + pitch * j, -1), gp_Dir(0, 0, 1)), dia / 2, 12).Shape();
            BRepAlgoAPI_Cut cut(shape, tool);
            if (!cut.IsDone()) { std::printf("  cut %d,%d failed\n", i, j); return shape; }
            shape = cut.Shape();
        }
    }
    if (filletR <= 0) return shape;

    BRepFilletAPI_MakeFillet fil(shape);
    TopTools_IndexedMapOfShape edges;
    TopExp::MapShapes(shape, TopAbs_EDGE, edges);
    filletedEdges = 0;
    for (int i = 1; i <= edges.Extent(); ++i) {
        const TopoDS_Edge e = TopoDS::Edge(edges(i));
        BRepAdaptor_Curve c(e);
        if (c.GetType() != GeomAbs_Circle) continue;
        if (std::fabs(c.Circle().Location().Z() - 10.0) > 1e-6) continue;   // top rims only
        fil.Add(filletR, e);
        ++filletedEdges;
    }
    fil.Build();
    if (!fil.IsDone()) { std::printf("  fillet of %d rims failed\n", filletedEdges); return shape; }
    return fil.Shape();
}

int main() {
    struct Case { const char* name; int n; double pitch, dia, filletR; };
    const Case cases[] = {
        {"16 bores, rims rounded",  4, 18, 8, 1.0},
        {"64 bores, rims rounded",  8, 18, 8, 1.0},
        {"100 bores, rims rounded", 10, 18, 8, 1.0},
    };

    for (const Case& c : cases) {
        int rims = 0;
        const auto t0 = Clock::now();
        const TopoDS_Shape shape = gridPart(c.n, c.pitch, c.dia, c.filletR, rims);
        const double buildMs = ms(t0);

        Bnd_Box bb;
        BRepBndLib::Add(shape, bb);
        double xa, ya, za, xb, yb, zb;
        bb.Get(xa, ya, za, xb, yb, zb);
        const double diag = std::sqrt((xb - xa) * (xb - xa) + (yb - ya) * (yb - ya) + (zb - za) * (zb - za));

        std::printf("\n=== %s: %d faces, %d rims, built in %.0f ms (diag %.0f mm) ===\n",
                    c.name, faceCount(shape), rims, buildMs, diag);
        std::printf("  %-22s %10s %10s %12s\n", "chord deviation", "triangles", "first", "re-mesh");

        // Zoomed out, a working view, and zoomed right in. The third is what a
        // hole looks like when the user leans on the scroll wheel.
        for (double f : {500.0, 2000.0, 8000.0}) {
            const double dev = diag / f;

            BRepTools::Clean(shape);
            const auto t1 = Clock::now();
            BRepMesh_IncrementalMesh m1(shape, dev, Standard_False, 0.35, Standard_True);
            const double firstMs = ms(t1);
            (void)m1;
            const int tris = triangles(shape);

            // A zoom does not start from a clean shape: the previous
            // triangulation is still on it and OCCT has to replace it.
            const auto t2 = Clock::now();
            BRepMesh_IncrementalMesh m2(shape, dev * 0.5, Standard_False, 0.35, Standard_True);
            const double reMs = ms(t2);
            (void)m2;

            std::printf("  %-22.4f %10d %8.1f ms %9.1f ms\n", dev, tris, firstMs, reMs);
        }
    }
    return 0;
}
