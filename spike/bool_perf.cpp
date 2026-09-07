// Tangent - Stage 0 spike: how does the boolean scale, and what is it worth
// running it differently?
//
// The feature chain replays one operation at a time, so sequential is the
// number that matters for editing. The batched figure is here because a chain
// that has just been loaded, or one being replayed after a base edit, could
// legitimately group independent cuts -- and if that is worth an order of
// magnitude it belongs in the plan rather than in a footnote.
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <gp_Ax2.hxx>

#include <chrono>
#include <cstdio>
#include <vector>

using Clock = std::chrono::steady_clock;
static double ms(const Clock::time_point& t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}
static int faceCount(const TopoDS_Shape& s) {
    TopTools_IndexedMapOfShape m; TopExp::MapShapes(s, TopAbs_FACE, m); return m.Extent();
}

static std::vector<TopoDS_Shape> tools(int n, double pitch, double dia) {
    std::vector<TopoDS_Shape> out;
    const double first = -pitch * (n - 1) / 2.0;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            out.push_back(BRepPrimAPI_MakeCylinder(
                gp_Ax2(gp_Pnt(first + pitch * i, first + pitch * j, -1), gp_Dir(0, 0, 1)), dia / 2, 12).Shape());
    return out;
}

static TopoDS_Shape plate(int n, double pitch) {
    const double side = pitch * (n + 1);
    return BRepPrimAPI_MakeBox(gp_Pnt(-side / 2, -side / 2, 0), side, side, 10).Shape();
}

int main() {
    std::printf("\n  %-6s %-26s %9s %9s %9s\n", "bores", "how", "total", "first cut", "last cut");
    for (int n : {4, 8, 10}) {
        const std::vector<TopoDS_Shape> cutters = tools(n, 18, 8);

        for (bool parallel : {false, true}) {
            TopoDS_Shape shape = plate(n, 18);
            double firstMs = 0, lastMs = 0;
            const auto t0 = Clock::now();
            for (size_t i = 0; i < cutters.size(); ++i) {
                const auto t1 = Clock::now();
                BRepAlgoAPI_Cut cut(shape, cutters[i]);
                cut.SetRunParallel(parallel);
                cut.Build();
                if (!cut.IsDone()) { std::printf("  cut %zu failed\n", i); break; }
                shape = cut.Shape();
                const double dt = ms(t1);
                if (i == 0) firstMs = dt;
                lastMs = dt;
            }
            std::printf("  %-6d %-26s %7.0f ms %7.1f ms %7.1f ms  (%d faces)\n",
                        n * n, parallel ? "one at a time, parallel" : "one at a time",
                        ms(t0), firstMs, lastMs, faceCount(shape));
        }

        // Everything in one call. Not how a history replays, but it is how a
        // pattern feature could be evaluated.
        TopTools_ListOfShape args, tls;
        args.Append(plate(n, 18));
        for (const TopoDS_Shape& c : cutters) tls.Append(c);
        BRepAlgoAPI_Cut batch;
        batch.SetArguments(args);
        batch.SetTools(tls);
        batch.SetRunParallel(Standard_True);
        const auto t2 = Clock::now();
        batch.Build();
        const double batchMs = ms(t2);
        std::printf("  %-6d %-26s %7.0f ms %9s %9s  (%d faces)\n\n", n * n, "all in one call",
                    batchMs, "-", "-", batch.IsDone() ? faceCount(batch.Shape()) : -1);
    }
    return 0;
}
