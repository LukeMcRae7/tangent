// The pruned validity check, held against the analyzer it stands in for.
//
// A validity check that got faster by saying yes more often would be worse
// than a slow one: it is the gate between a kernel result and the model. So
// every case here is asked of both, on shapes heavy enough that the pruned
// path really runs -- valid ones, and ones broken in each way a face with holes
// can be broken -- and the two must agree. Only then is the time compared.
#include "geom/brep.h"

#include <cstdio>
#include <string>

#ifdef TG_HAVE_OCCT
#include "geom/brep_valid.h"

#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <gp_Pln.hxx>
#include <gp_Vec.hxx>

#include <chrono>
#include <cmath>
#include <vector>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}

// A polygon approximating a circle, wound either way.
static TopoDS_Wire ring(double cx, double cy, double r, int n, bool clockwise) {
    BRepBuilderAPI_MakePolygon p;
    for (int i = 0; i < n; ++i) {
        const double a = 2 * M_PI * (clockwise ? -i : i) / n;
        p.Add(gp_Pnt(cx + r * std::cos(a), cy + r * std::sin(a), 0));
    }
    p.Close();
    return p.Wire();
}

static TopoDS_Wire square(double x0, double y0, double x1, double y1) {
    BRepBuilderAPI_MakePolygon p(gp_Pnt(x0, y0, 0), gp_Pnt(x1, y0, 0), gp_Pnt(x1, y1, 0),
                                 gp_Pnt(x0, y1, 0), Standard_True);
    return p.Wire();
}

// A plate from a boundary and holes, extruded without any checking at all, so
// that whatever is wrong with the holes is carried into the solid.
static TopoDS_Shape plate(const std::vector<TopoDS_Wire>& holes) {
    BRepBuilderAPI_MakeFace f(gp_Pln(gp::XOY()), square(0, 0, 120, 120), Standard_True);
    for (const TopoDS_Wire& h : holes) f.Add(h);
    return BRepPrimAPI_MakePrism(f.Face(), gp_Vec(0, 0, 8)).Shape();
}

static std::vector<TopoDS_Wire> grid(int per, int segs) {
    std::vector<TopoDS_Wire> out;
    for (int i = 0; i < per; ++i)
        for (int j = 0; j < per; ++j)
            out.push_back(ring(11 + i * 14.0, 11 + j * 14.0, 2.5, segs, true));
    return out;
}

int main() {
    std::printf("validity\n");

    struct Case { const char* name; TopoDS_Shape shape; bool valid; };
    std::vector<Case> cases;

    cases.push_back({"a drilled plate, 36 holes of 48 segments", plate(grid(6, 48)), true});

    {   // Two holes that overlap.
        auto h = grid(6, 48);
        h.push_back(ring(11 + 2.0, 11, 2.5, 48, true));
        cases.push_back({"two holes overlapping", plate(h), false});
    }
    {   // A hole that crosses the outer boundary.
        auto h = grid(6, 48);
        h.push_back(ring(119, 60, 3, 48, true));
        cases.push_back({"a hole crossing the edge", plate(h), false});
    }
    {   // A hole entirely outside the plate.
        auto h = grid(6, 48);
        h.push_back(ring(200, 60, 3, 48, true));
        cases.push_back({"a hole outside the plate", plate(h), false});
    }
    {   // A hole inside another hole.
        auto h = grid(6, 48);
        h.push_back(ring(11, 11, 1.0, 48, true));
        cases.push_back({"a hole inside a hole", plate(h), false});
    }

    // Sized to be heavy -- 37 wires and 1,732 edges on each drilled face, past
    // the threshold -- while keeping the full analyzer it is compared against
    // to about a second a case.
    for (const Case& c : cases) {
        auto t0 = std::chrono::steady_clock::now();
        const bool full = brep::fullAnalyzerValid(c.shape);
        auto t1 = std::chrono::steady_clock::now();
        const bool pruned = brep::shapeIsValid(c.shape);
        auto t2 = std::chrono::steady_clock::now();
        const bool forced = brep::shapeIsValid(c.shape, 0);   // every face in pieces

        const auto ms = [](auto a, auto b) {
            return std::chrono::duration<double, std::milli>(b - a).count();
        };
        std::printf("  %-44s full=%d (%7.1f ms)  pruned=%d (%6.1f ms)  forced=%d\n", c.name,
                    (int)full, ms(t0, t1), (int)pruned, ms(t1, t2), (int)forced);

        check(full == c.valid, std::string(c.name) + ": the full analyzer says what we expect");
        check(pruned == full, std::string(c.name) + ": the pruned check agrees with it");
        check(forced == full, std::string(c.name) + ": and so does checking every face in pieces");
        if (c.valid)
            check(ms(t1, t2) * 2 < ms(t0, t1), std::string(c.name) + ": and is faster");
    }

    std::printf("%s\n", failures ? "FAILED" : "ok");
    return failures ? 1 : 0;
}

#else
int main() {
    std::printf("exact kernel not built; validity checking needs one\n");
    return 0;
}
#endif
