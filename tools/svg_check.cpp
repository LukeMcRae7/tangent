// Tangent - what an SVG becomes, and whether it will extrude.
//
//   svg_check drawing.svg [more.svg ...]
//
// When a downloaded drawing will not extrude, this says why without opening
// the application: what the import put right on the way in, how many regions
// the sketch found and which of them it would fill, and -- for any region
// whose outlines still cross -- where, in millimetres on the drawing, so the
// place can be found in the file. Then whether the filled regions sweep, and
// how long each stage took.
#include "geom/body.h"
#include "geom/brep.h"
#include "sketch/sketch.h"
#include "sketch/svg.h"

#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

using namespace tg;

namespace {

double since(std::chrono::steady_clock::time_point t) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
}

int check(const char* path) {
    std::printf("%s\n", path);
    auto t = std::chrono::steady_clock::now();
    const SvgDrawing d = readSvgFile(path);
    std::printf("  read in %.1f ms\n", since(t));
    if (!d.ok) {
        std::printf("  not imported: %s\n", d.error.c_str());
        return 1;
    }
    std::printf("  %zu outlines, %zu circles, %.1f x %.1f mm\n", d.paths.size(), d.circles.size(), d.size().x,
                d.size().y);
    std::printf("  colours:");
    for (const SvgColour& c : d.colours)
        std::printf(" %s (%d, %s)", c.css.c_str(), c.shapes, c.ink ? "ink" : "paper");
    std::printf("\n");
    std::printf("  left out: %d text, %d images, %d page background; %d paths stopped part way\n", d.text,
                d.images, d.background, d.unreadable);
    std::printf("  put right: %d crossings joined up, %d covered shapes dropped, %d specks dropped; "
                "%d outlines could not be untangled\n",
                d.crossings, d.covered, d.specks, d.unresolved);

    Sketch sk;
    insertSvg(sk, d, {});
    t = std::chrono::steady_clock::now();
    const SketchSolve s = solveSketch(sk);
    const double solveMs = since(t);
    t = std::chrono::steady_clock::now();
    const auto regions = sketchProfiles(sk);
    const auto filled = sketchFilledProfiles(sk, regions);
    std::printf("  sketch: %zu entities, %s in %.1f ms; %zu regions, %zu filled, in %.1f ms\n", sk.entities.size(),
                s.solved ? "solves" : s.reason.c_str(), solveMs, regions.size(), filled.size(), since(t));

    // Each region's outlines, checked for crossings as the extrusion checks
    // them; and where, for the ones that do.
    const Vec2 mid = (d.min + d.max) * 0.5;
    size_t bad = 0;
    for (const SketchProfile& r : regions) {
        std::vector<std::vector<SvgSegment>> loops{sketchLoopCurves(sk, r.outer)};
        for (const SketchLoop& h : r.holes) loops.push_back(sketchLoopCurves(sk, h));
        if (outlineCrossings(loops) == 0) continue;
        ++bad;
        // Near where: the region's first point, back in the drawing's frame.
        const SketchEntity* e = sk.entity(r.outer.entities.front());
        const SketchPoint* p = e ? sk.point(e->a) : nullptr;
        const Vec2 at = p ? p->at + mid : Vec2{};
        std::printf("  region %u (%zu pieces, %zu holes) has outlines that cross; it starts near "
                    "(%.2f, %.2f) mm\n",
                    r.key, r.outer.entities.size(), r.holes.size(), at.x, at.y);
    }
    std::printf("  %zu of %zu regions have crossing outlines\n", bad, regions.size());

    if (!brep::available()) {
        std::printf("  (no exact kernel in this build: not swept)\n");
        return bad == 0 ? 0 : 1;
    }
    t = std::chrono::steady_clock::now();
    std::string why;
    BrepRef all = brep::sketchSolids(sk, regions, filled, 0.0, 1.0, 1, &why);
    if (!all) {
        std::printf("  the filled regions do not sweep: %s\n", why.c_str());
        return 1;
    }
    const Body body(std::move(all));
    std::printf("  the filled regions sweep 1 mm deep in %.0f ms: %d faces, %.1f mm3\n", since(t),
                body.faceCount(), body.health(false).volume);
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("usage: svg_check drawing.svg [more.svg ...]\n");
        return 2;
    }
    int failed = 0;
    for (int i = 1; i < argc; ++i) failed += check(argv[i]);
    return failed == 0 ? 0 : 1;
}
