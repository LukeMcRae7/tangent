// Does OCCT build, link and run at all, and what does it cost to get there?
#include <BRepAdaptor_Curve.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <Standard_Version.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>

#include <cstdio>

static int count(const TopoDS_Shape& s, TopAbs_ShapeEnum kind) {
    int n = 0;
    for (TopExp_Explorer e(s, kind); e.More(); e.Next()) ++n;
    return n;
}

int main() {
    std::printf("OCCT %s\n", OCC_VERSION_COMPLETE);

    const TopoDS_Shape plate = BRepPrimAPI_MakeBox(gp_Pnt(-50, -50, 0), 100, 100, 10).Shape();
    const TopoDS_Shape bore  = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -1), gp_Dir(0, 0, 1)), 5, 12).Shape();

    BRepAlgoAPI_Cut cut(plate, bore);
    if (!cut.IsDone()) { std::printf("cut failed\n"); return 1; }
    const TopoDS_Shape bored = cut.Shape();
    std::printf("plate with one bore: %d faces, %d edges\n",
                count(bored, TopAbs_FACE), count(bored, TopAbs_EDGE));

    // Round the top rim of the bore. On the mesh kernel this is the operation
    // that refuses; here it is one call and the radius is exact.
    BRepFilletAPI_MakeFillet fil(bored);
    int added = 0;
    for (TopExp_Explorer e(bored, TopAbs_EDGE); e.More(); e.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(e.Current());
        if (BRepAdaptor_Curve(edge).GetType() == GeomAbs_Circle) { fil.Add(2.0, edge); ++added; }
    }
    fil.Build();
    if (!fil.IsDone()) { std::printf("fillet failed (%d circular edges)\n", added); return 1; }
    std::printf("filleted %d circular edges: %d faces\n", added, count(fil.Shape(), TopAbs_FACE));
    return 0;
}
