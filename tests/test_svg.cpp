// SVG into a sketch: that a drawing arrives at the size it says it is, the
// right way up, made of the curves it was made of, and closed where it closed.
//
// Every number here is worked out by hand from the drawing, not read back from
// this code: a 200 x 100 viewBox on a 100mm-wide page is 100 x 50 mm, and a
// stadium of two 10mm semicircles on a 20mm square is 400 + 100 pi.
#include "sketch/svg.h"
#include "geom/body.h"
#include "geom/brep.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near(Real a, Real b, Real eps = 1e-6) { return std::fabs(a - b) < eps; }

static std::string page(const std::string& body, const char* attrs = R"(width="100mm" height="100mm" viewBox="0 0 100 100")") {
    return std::string("<?xml version=\"1.0\"?>\n<!-- a comment -->\n<svg xmlns=\"http://www.w3.org/2000/svg\" ") +
           attrs + ">" + body + "</svg>";
}

static int countCurve(const Sketch& sk, SketchCurve c) {
    int n = 0;
    for (const SketchEntity& e : sk.entities) n += e.curve == c;
    return n;
}

static int countRule(const Sketch& sk, SketchRule r) {
    int n = 0;
    for (const SketchConstraint& k : sk.constraints) n += k.rule == r;
    return n;
}

// Swept 1mm, so the volume is the area -- exactly, from the exact kernel.
static Real exactArea(const Sketch& sk, const SketchProfile& p) {
    BrepRef b = brep::sketchSolid(sk, p, 0.0, 1.0, 0, nullptr);
    return b ? Body(std::move(b)).health(false).volume : -1.0;
}

int main() {
    std::printf("--- a viewBox on a page in millimetres is that many millimetres ---\n");
    {
        const SvgDrawing d = parseSvg(page(R"(<rect x="0" y="0" width="200" height="100"/>)",
                                           R"(width="100mm" height="50mm" viewBox="0 0 200 100")"));
        check(d.ok, "read: " + d.error);
        check(near(d.size().x, 100.0) && near(d.size().y, 50.0),
              "100 x 50 mm, got " + std::to_string(d.size().x) + " x " + std::to_string(d.size().y));

        Sketch sk;
        const SvgInsert ins = insertSvg(sk, d, {});
        check(countCurve(sk, SketchCurve::Line) == 4, "four lines");
        check(sk.points.size() == 4, "on four shared corners, got " + std::to_string(sk.points.size()));
        check(countRule(sk, SketchRule::Horizontal) == 2 && countRule(sk, SketchRule::Vertical) == 2,
              "held level and plumb, as drawn");
        const SketchSolve s = solveSketch(sk);
        check(s.solved, "solves: " + s.reason);
        const auto regions = sketchProfiles(sk);
        check(regions.size() == 1 && near(regions[0].area, 5000.0, 1e-6), "one region of 5000 mm2");
        // Centred on the sketch's origin.
        Vec2 lo{1e9, 1e9}, hi{-1e9, -1e9};
        for (const SketchPoint& p : sk.points) {
            lo = {std::min(lo.x, p.at.x), std::min(lo.y, p.at.y)};
            hi = {std::max(hi.x, p.at.x), std::max(hi.y, p.at.y)};
        }
        check(near(lo.x, -50) && near(hi.x, 50) && near(lo.y, -25) && near(hi.y, 25), "centred on the origin");
        check(ins.entities.size() == 4, "and all four remembered for placing");
    }

    std::printf("--- without units, a user unit is a pixel at 96 to the inch ---\n");
    {
        const SvgDrawing d = parseSvg(page(R"(<rect width="96" height="48"/>)", R"(width="96" height="48")"));
        check(near(d.size().x, 25.4) && near(d.size().y, 12.7), "an inch by half an inch");
        const SvgDrawing pt = parseSvg(page(R"(<rect width="72" height="72"/>)",
                                            R"(width="72pt" height="72pt" viewBox="0 0 72 72")"));
        check(near(pt.size().x, 25.4), "72pt is an inch, got " + std::to_string(pt.size().x));
    }

    std::printf("--- the right way up ---\n");
    {
        // A triangle pointing down the page points down the sketch: SVG's y
        // runs down, a sketch's up. Its apex is the lowest point.
        const SvgDrawing d = parseSvg(page(R"(<polygon points="0,0 20,0 10,30"/>)"));
        Sketch sk;
        insertSvg(sk, d, {});
        Real lowest = 1e9, lowestX = 0;
        for (const SketchPoint& p : sk.points)
            if (p.at.y < lowest) { lowest = p.at.y; lowestX = p.at.x; }
        check(near(lowestX, 0.0), "the apex is at the bottom, in the middle");
        solveSketch(sk);
        const auto regions = sketchProfiles(sk);
        check(regions.size() == 1 && near(regions[0].area, 300.0), "area 300");
    }

    std::printf("--- a circle is a circle, and a circular arc an arc ---\n");
    {
        const SvgDrawing d = parseSvg(page(R"(<circle cx="50" cy="50" r="10"/>)"));
        Sketch sk;
        insertSvg(sk, d, {});
        check(countCurve(sk, SketchCurve::Circle) == 1, "one circle");
        check(near(sk.entities.front().radius, 10.0), "of radius 10");

        // A stadium: two semicircles of 10 on the ends of a 20 x 20 square.
        const SvgDrawing st = parseSvg(page(R"(<path d="M0,0 H20 A10,10 0 0,1 20,20 H0 A10 10 0 0 1 0 0 Z"/>)"));
        Sketch s2;
        insertSvg(s2, st, {});
        check(countCurve(s2, SketchCurve::Arc) == 2 && countCurve(s2, SketchCurve::Line) == 2,
              "two arcs and two lines, got " + std::to_string(s2.entities.size()) + " entities");
        const SketchSolve s = solveSketch(s2);
        check(s.solved, "solves: " + s.reason);
        const auto regions = sketchProfiles(s2);
        check(regions.size() == 1, "one region, got " + std::to_string(regions.size()));
        check(near(st.size().x, 40.0, 1e-6) && near(st.size().y, 20.0, 1e-6),
              "40 x 20 across the arcs, got " + std::to_string(st.size().x));
        if (brep::available() && regions.size() == 1) {
            const Real a = exactArea(s2, regions[0]);
            check(near(a, 400.0 + 100.0 * kPi, 1e-6), "exactly 400 + 100 pi, got " + std::to_string(a));
            std::printf("  stadium: %.6f mm2 (400 + 100 pi = %.6f)\n", a, 400.0 + 100.0 * kPi);
        }
    }

    std::printf("--- the path language, in its awkward forms ---\n");
    {
        // Relative commands, implicit repeats, numbers run together, and a
        // quadratic curve whose area is known: the parabolic segment under
        // Q from (0,0) via (10,20) to (20,0) is two thirds of 20 x 10.
        const SvgDrawing d = parseSvg(page(R"(<path d="m0 0q10 20 20 0z"/>)"));
        Sketch sk;
        insertSvg(sk, d, {});
        check(countCurve(sk, SketchCurve::Bezier) == 1 && countCurve(sk, SketchCurve::Line) == 1,
              "a curve and the line closing it");
        const auto regions = sketchProfiles(sk);
        check(regions.size() == 1, "closed");
        if (brep::available() && regions.size() == 1) {
            const Real a = exactArea(sk, regions[0]);
            check(near(a, 2.0 / 3.0 * 20.0 * 10.0, 1e-6), "parabolic segment 133.33, got " + std::to_string(a));
        }

        const SvgDrawing run = parseSvg(page(R"(<path d="M1.5.5l10-0 0,10-10,0z"/>)"));
        check(run.ok && run.paths.size() == 1 && run.paths[0].segments.size() == 4 && run.paths[0].closed,
              "'1.5.5' is two numbers and '10-0' is two: a closed square");
        check(near(run.size().x, 10.0) && near(run.size().y, 10.0), "10 x 10");

        // Two subpaths, the second begun by the first's m after a z.
        const SvgDrawing two = parseSvg(page(R"(<path d="M0 0h30v30h-30z m5 5v20h20v-20z"/>)"));
        check(two.paths.size() == 2, "two subpaths, got " + std::to_string(two.paths.size()));
        Sketch s2;
        insertSvg(s2, two, {});
        const auto rs = sketchProfiles(s2);
        check(rs.size() == 2, "two regions: the frame and what fills it");
        const auto filled = sketchFilledProfiles(s2, rs);
        check(filled.size() == 1, "one of them filled");
        for (const SketchProfile& p : rs)
            if (!filled.empty() && p.key == filled[0])
                check(near(p.area, 900.0 - 400.0), "the frame, 500 mm2, got " + std::to_string(p.area));

        // Arc flags written with nothing between them.
        const SvgDrawing flags = parseSvg(page(R"(<path d="M0 0a10 10 0 0120 0z"/>)"));
        check(flags.ok && flags.paths.size() == 1 && flags.paths[0].segments.size() == 2 &&
                  flags.paths[0].segments[0].kind == SvgSegment::Kind::Arc,
              "'0120' is two flags and a number");
        check(flags.unreadable == 0, "and read to the end");
    }

    std::printf("--- transforms ---\n");
    {
        // Doubled, a circle of 5 is a circle of 10.
        const SvgDrawing d = parseSvg(page(R"s(<g transform="translate(10,0) scale(2)"><circle r="5"/></g>)s"));
        check(d.circles.size() == 1 && near(d.circles[0].radius, 10.0), "a circle of 10");
        check(near(d.circles[0].centre.x, 10.0), "moved 10 along");

        // Stretched one way only, it is an ellipse: four curves around, with
        // the area of pi a b to within what four cubics can do (0.03%).
        const SvgDrawing e = parseSvg(page(R"s(<circle r="10" transform="scale(2,1)"/>)s"));
        check(e.circles.empty() && e.paths.size() == 1 && e.paths[0].segments.size() == 4,
              "an ellipse of four curves");
        Sketch sk;
        insertSvg(sk, e, {});
        const auto rs = sketchProfiles(sk);
        check(rs.size() == 1, "closed");
        if (brep::available() && rs.size() == 1) {
            const Real a = exactArea(sk, rs[0]);
            check(std::fabs(a / (kPi * 20.0 * 10.0) - 1.0) < 3e-4, "pi x 20 x 10, got " + std::to_string(a));
        }

        // Rotated a quarter turn, a 30 x 10 bar stands 10 x 30.
        const SvgDrawing r = parseSvg(page(R"s(<rect width="30" height="10" transform="rotate(90)"/>)s"));
        check(near(r.size().x, 10.0, 1e-9) && near(r.size().y, 30.0, 1e-9), "stood on end");
    }

    std::printf("--- what is not drawn is not read, and what cannot be read is said ---\n");
    {
        const SvgDrawing d = parseSvg(page(
            R"(<defs><rect id="r" width="10" height="10"/></defs>
               <rect width="5" height="5" style="display:none"/>
               <g display="none"><circle r="3"/></g>
               <text x="0" y="0">Hello</text>
               <use href="#r" x="20" y="0"/>
               <use xlink:href="#r" x="40" y="0"/>)"));
        check(d.ok, "read");
        check(d.paths.size() == 2 && d.circles.empty(), "the two uses, and nothing hidden or in defs, got " +
                                                           std::to_string(d.paths.size()));
        check(d.text == 1, "the text counted as left out");
        check(near(d.size().x, 30.0), "from 20 to 50 across");

        const SvgDrawing none = parseSvg("<html><body>no</body></html>");
        check(!none.ok && !none.error.empty(), "not an SVG: " + none.error);
        const SvgDrawing words = parseSvg(page(R"(<text>only words</text>)"));
        check(!words.ok && words.error.find("convert it to paths") != std::string::npos,
              "only text: " + words.error);
    }

    std::printf("--- a path that crosses itself becomes the outlines of what it fills ---\n");
    {
        // A bow tie, lopsided: from (0,0) to (30,20), down to (30,0), across
        // to (0,10) and home. It crosses itself once; a browser fills both
        // lobes, and so does the import -- as two
        // triangles meeting at the crossing, neither crossing anything.
        // Their areas, by hand: the lines y = 2x/3 and y = 10 - x/3 cross
        // at (10, 20/3); the left lobe, a triangle of base 10 and height 10,
        // is 50, and the right, base 20 and height 20, is 200.
        const SvgDrawing d = parseSvg(page(R"(<polygon points="0,0 30,20 30,0 0,10"/>)"));
        check(d.crossings == 1, "one crossing, got " + std::to_string(d.crossings));
        check(d.paths.size() == 2, "two outlines, got " + std::to_string(d.paths.size()));
        Sketch sk;
        insertSvg(sk, d, {});
        const auto rs = sketchProfiles(sk);
        check(rs.size() == 2, "two regions, got " + std::to_string(rs.size()));
        Real total = 0;
        for (const auto& r : rs) total += r.area;
        check(near(total, 250.0, 1e-6), "50 + 200, got " + std::to_string(total));
        if (brep::available() && rs.size() == 2) {
            std::string why;
            check(brep::sketchSolids(sk, rs, sketchFilledProfiles(sk, rs), 0.0, 1.0, 1, &why) != nullptr,
                  "and they sweep: " + why);
        }

        // Drawn by hand, a crossing is still the user's to fix, and says so.
        if (brep::available()) {
            Sketch bad;
            const SketchId p0 = bad.addPoint({0, 0}), p1 = bad.addPoint({30, 20}), p2 = bad.addPoint({30, 0}),
                           p3 = bad.addPoint({0, 10});
            bad.addLine(p0, p1);
            bad.addLine(p1, p2);
            bad.addLine(p2, p3);
            bad.addLine(p3, p0);
            const auto br = sketchProfiles(bad);
            std::string why;
            const bool made = !br.empty() && brep::sketchSolids(bad, br, {br[0].key}, 0.0, 1.0, 1, &why) != nullptr;
            check(!br.empty() && !made && why.find("crosses") != std::string::npos,
                  "a hand-drawn bow tie is refused: " + why);
        }
    }

    std::printf("--- overlapping shapes are one outline, as they are one fill ---\n");
    {
        // Two 20 x 20 squares, the second 10 across and 10 down: 400 + 400 -
        // the 100 they share.
        const SvgDrawing d = parseSvg(page(R"(<rect width="20" height="20"/><rect x="10" y="10" width="20" height="20"/>)"));
        check(d.paths.size() == 1, "one outline, got " + std::to_string(d.paths.size()));
        check(d.crossings == 2, "where they cross, twice");
        Sketch sk;
        insertSvg(sk, d, {});
        const auto rs = sketchProfiles(sk);
        check(rs.size() == 1 && near(rs[0].area, 700.0, 1e-6),
              "700 mm2, got " + (rs.empty() ? std::string("nothing") : std::to_string(rs[0].area)));
        check(countRule(sk, SketchRule::Horizontal) == 4 && countRule(sk, SketchRule::Vertical) == 4,
              "its eight sides still held level and plumb");

        // A circle overlapping a square: cut exactly, so the arc left over is
        // still an arc of radius 10 about the same centre. It crosses the
        // square's right side at x = 20, 5 in from its centre at 25.
        const SvgDrawing c = parseSvg(page(R"(<rect width="20" height="20"/><circle cx="25" cy="10" r="10"/>)"));
        Sketch s2;
        insertSvg(s2, c, {});
        const auto r2 = sketchProfiles(s2);
        check(r2.size() == 1, "square and disc are one region");
        bool arcKept = false;
        for (const SketchEntity& e : s2.entities)
            if (e.curve == SketchCurve::Arc && near(e.radius, 10.0, 1e-9)) arcKept = true;
        check(arcKept, "with the circle's outer half an arc of 10");
        if (brep::available() && r2.size() == 1) {
            // The square, and the disc less the segment of it inside the
            // square: a chord 5 from the centre of a circle of 10 cuts off
            // r^2 acos(d/r) - d sqrt(r^2 - d^2) = 100 pi/3 - 25 sqrt 3.
            const Real segment = 100.0 * kPi / 3.0 - 25.0 * std::sqrt(3.0);
            const Real want = 400.0 + 100.0 * kPi - segment;
            const Real a = exactArea(s2, r2[0]);
            check(near(a, want, 1e-6), std::to_string(want) + ", got " + std::to_string(a));
        }
    }

    std::printf("--- shapes that share an edge are one outline ---\n");
    {
        // Two 10 x 10 squares side by side: one 20 x 10 outline, the side
        // they share gone.
        const SvgDrawing d = parseSvg(page(R"(<rect width="10" height="10"/><rect x="10" width="10" height="10"/>)"));
        check(d.paths.size() == 1 && d.unresolved == 0, "one outline, got " + std::to_string(d.paths.size()) +
                                                            ", unresolved " + std::to_string(d.unresolved));
        Sketch sk;
        insertSvg(sk, d, {});
        const auto rs = sketchProfiles(sk);
        check(rs.size() == 1 && near(rs[0].area, 200.0, 1e-9), "200 mm2");

        // The same, but one square half the height: the long side is cut
        // where the short one ends, and the rest of it is outline. An L of
        // 100 + 50.
        const SvgDrawing l = parseSvg(page(R"(<rect width="10" height="10"/><rect x="10" width="10" height="5"/>)"));
        Sketch s2;
        insertSvg(s2, l, {});
        const auto r2 = sketchProfiles(s2);
        check(l.paths.size() == 1 && r2.size() == 1 && near(r2[0].area, 150.0, 1e-9),
              "an L of 150 mm2, got " + std::to_string(l.paths.size()) + " outlines");

        // Pixel art: a 3 x 3 board with the corners and the middle filled,
        // squares meeting only at corners. Five squares that touch and do not
        // merge -- they share no edge -- and a solid made of them is valid.
        std::string board;
        for (int y = 0; y < 3; ++y)
            for (int x = 0; x < 3; ++x)
                if ((x + y) % 2 == 0)
                    board += "<rect x=\"" + std::to_string(x * 5) + "\" y=\"" + std::to_string(y * 5) +
                             "\" width=\"5\" height=\"5\"/>";
        const SvgDrawing b = parseSvg(page(board));
        check(b.paths.size() == 5 && b.unresolved == 0, "five squares, got " + std::to_string(b.paths.size()));
        Sketch s3;
        insertSvg(s3, b, {});
        const auto r3 = sketchProfiles(s3);
        check(r3.size() == 5, "five regions, got " + std::to_string(r3.size()));
        if (brep::available() && !r3.empty()) {
            std::string why;
            BrepRef all = brep::sketchSolids(s3, r3, sketchFilledProfiles(s3, r3), 0.0, 1.0, 1, &why);
            check(all != nullptr, "and they sweep: " + why);
            if (all) check(near(Body(std::move(all)).health(false).volume, 125.0, 1e-6), "125 mm3");
        }

        // And the full board, every square filled: one 15 x 15 outline.
        std::string full;
        for (int y = 0; y < 3; ++y)
            for (int x = 0; x < 3; ++x)
                full += "<rect x=\"" + std::to_string(x * 5) + "\" y=\"" + std::to_string(y * 5) +
                        "\" width=\"5\" height=\"5\"/>";
        const SvgDrawing f = parseSvg(page(full));
        Sketch s4;
        insertSvg(s4, f, {});
        const auto r4 = sketchProfiles(s4);
        check(f.paths.size() == 1 && r4.size() == 1 && near(r4[0].area, 225.0, 1e-9),
              "nine squares are one of 225 mm2, got " + std::to_string(f.paths.size()) + " outlines");
    }

    std::printf("--- a shape painted inside a filled one bounds nothing ---\n");
    {
        // A dot on a square, both filled: the browser shows a square.
        const SvgDrawing d = parseSvg(page(R"(<rect width="20" height="20"/><circle cx="10" cy="10" r="3"/>)"));
        check(d.paths.size() == 1 && d.circles.empty(), "the square alone");
        check(d.covered == 1, "and the dot counted as covered");
        // The same dot as a hole, the way a font or an icon cuts one: even-odd
        // in one path.
        const SvgDrawing h = parseSvg(page(
            R"(<path fill-rule="evenodd" d="M0 0H20V20H0Z M13 10A3 3 0 0 1 7 10A3 3 0 0 1 13 10Z"/>)"));
        Sketch sk;
        insertSvg(sk, h, {});
        const auto rs = sketchProfiles(sk);
        const auto filled = sketchFilledProfiles(sk, rs);
        check(rs.size() == 2 && filled.size() == 1, "a plate and its hole");
        // Exactly, from the kernel: the sketch's own area is of a sampled
        // outline, a little over for a hole drawn with round sides.
        for (const auto& r : rs)
            if (brep::available() && !filled.empty() && r.key == filled[0]) {
                const Real a = exactArea(sk, r);
                check(near(a, 400.0 - 9.0 * kPi, 1e-6), "400 - 9 pi, got " + std::to_string(a));
            }
        // Nonzero, with the hole drawn the other way round: the same.
        const SvgDrawing nz = parseSvg(page(
            R"(<path d="M0 0H20V20H0Z M13 10A3 3 0 0 0 7 10A3 3 0 0 0 13 10Z"/>)"));
        check(nz.paths.size() + nz.circles.size() == 2, "nonzero keeps a hole drawn backwards");
        const SvgDrawing same = parseSvg(page(
            R"(<path d="M0 0H20V20H0Z M13 10A3 3 0 0 1 7 10A3 3 0 0 1 13 10Z"/>)"));
        check(same.paths.size() + same.circles.size() == 1, "and fills one drawn the same way round");
        // A stroke is not a fill, and is left as drawn.
        const SvgDrawing st = parseSvg(page(R"(<rect width="20" height="20" fill="none" stroke="black"/><circle cx="10" cy="10" r="3" fill="none"/>)"));
        check(st.paths.size() == 1 && st.circles.size() == 1 && st.covered == 0, "outlines only: both kept");
    }

    std::printf("--- what shows is what was painted last, and paper is not ink ---\n");
    {
        // White painted on a black square: a square with a round hole, the
        // way a drawing program cuts a counter. 400 less pi 3 squared.
        const SvgDrawing d = parseSvg(page(R"(<rect width="20" height="20"/><circle cx="10" cy="10" r="3" fill="#fff"/>)"));
        check(d.colours.size() == 2, "two colours");
        Sketch sk;
        insertSvg(sk, d, {});
        const auto rs = sketchProfiles(sk);
        const auto filled = sketchFilledProfiles(sk, rs);
        check(rs.size() == 2 && filled.size() == 1, "a plate and its hole");
        if (brep::available())
            for (const auto& r : rs)
                if (!filled.empty() && r.key == filled[0])
                    check(near(exactArea(sk, r), 400.0 - 9.0 * kPi, 1e-6), "400 - 9 pi");

        // Black on the white circle again: painted last, so it shows. A
        // black dot in a white ring in a black square.
        const SvgDrawing t = parseSvg(page(
            R"(<rect width="20" height="20"/><circle cx="10" cy="10" r="3" fill="white"/><circle cx="10" cy="10" r="1"/>)"));
        check(t.circles.size() + t.paths.size() == 3, "three outlines, got " +
                                                          std::to_string(t.circles.size() + t.paths.size()));

        // Dark work on a pale backing: the backing is paper, and the work is
        // what comes in -- two squares, not the plate under them.
        const SvgDrawing b = parseSvg(page(
            R"(<rect width="50" height="30" fill="#e0e0e0"/><rect x="5" y="5" width="10" height="10" fill="#222"/><rect x="30" y="5" width="10" height="10" fill="#222"/>)"));
        check(b.colours.size() == 2 && !b.colours[0].ink && b.colours[1].ink, "the backing paper, the squares ink");
        check(b.paths.size() == 2 && near(b.size().x, 35.0), "two squares 35 across, got " +
                                                                 std::to_string(b.paths.size()));
        // Made ink, the backing is the part and the squares are in it.
        const SvgDrawing all = recolourSvg(b, {true, true});
        check(all.ok && all.paths.size() == 1 && near(all.size().x, 50.0), "all ink: the plate alone");
        check(all.covered == 2, "its squares painted over, got " + std::to_string(all.covered));
        // Nothing ink at all is said, not imported as nothing.
        const SvgDrawing none = recolourSvg(b, {false, false});
        check(!none.ok && !none.error.empty(), "no ink: " + none.error);
    }

    std::printf("--- the paper behind a drawing is left out ---\n");
    {
        const SvgDrawing d = parseSvg(page(R"(<rect width="100" height="100" fill="#f5f5f5"/><circle cx="50" cy="50" r="10"/>)"));
        check(d.background == 1 && d.circles.size() == 1 && d.paths.empty(), "the circle, not the page");
        const SvgDrawing only = parseSvg(page(R"(<rect width="100" height="100"/>)"));
        check(only.background == 0 && only.paths.size() == 1, "unless the page is all there is");
    }

    std::printf("--- a rounded rectangle keeps its round corners ---\n");
    {
        const SvgDrawing d = parseSvg(page(R"(<rect width="40" height="20" rx="5"/>)"));
        Sketch sk;
        insertSvg(sk, d, {});
        check(countCurve(sk, SketchCurve::Arc) == 4 && countCurve(sk, SketchCurve::Line) == 4,
              "four arcs, four lines");
        const SketchSolve s = solveSketch(sk);
        check(s.solved, "solves: " + s.reason);
        const auto rs = sketchProfiles(sk);
        check(rs.size() == 1, "closed");
        if (brep::available() && rs.size() == 1) {
            // 40 x 20 less the four corners a circle of 5 does not fill.
            const Real want = 800.0 - (4.0 - kPi) * 25.0;
            const Real a = exactArea(sk, rs[0]);
            check(near(a, want, 1e-6), std::to_string(want) + ", got " + std::to_string(a));
        }
    }

    std::printf("--- placed: sized, moved and turned after it lands ---\n");
    {
        const SvgDrawing d = parseSvg(page(R"(<rect width="30" height="10"/><circle cx="5" cy="5" r="2"/>)"));
        Sketch sk;
        SvgInsert ins = insertSvg(sk, d, {});
        SvgPlacement at;
        at.scale = 2.0;
        at.centre = {100, 50};
        at.quarterTurns = 1;
        placeSvg(sk, ins, at);
        const SketchSolve s = solveSketch(sk);
        check(s.solved && s.conflicting.empty(), "still solves turned: level became plumb");
        Vec2 lo{1e9, 1e9}, hi{-1e9, -1e9};
        for (const SketchPoint& p : sk.points) {
            lo = {std::min(lo.x, p.at.x), std::min(lo.y, p.at.y)};
            hi = {std::max(hi.x, p.at.x), std::max(hi.y, p.at.y)};
        }
        check(near(hi.x - lo.x, 20.0) && near(hi.y - lo.y, 60.0), "60 x 20, stood on end at twice the size");
        check(near((lo.x + hi.x) * 0.5, 100.0) && near((lo.y + hi.y) * 0.5, 50.0), "about (100, 50)");
        for (const SketchEntity& e : sk.entities)
            if (e.curve == SketchCurve::Circle) check(near(e.radius, 4.0), "the hole twice as big");
    }

    std::printf("--- a drawing the size of a real logo stays quick ---\n");
    {
        // 240 letter-sized loops of 16 curves each, every one with a counter
        // inside it: 7,680 curves. What it costs to read, solve and find the
        // regions in, because the sketch has to stay usable at this size.
        std::string body;
        char b[512];
        for (int i = 0; i < 240; ++i) {
            const Real x = (i % 20) * 12.0, y = (i / 20) * 12.0;
            auto ring = [&](Real r) {
                // A circle drawn as eight arcs of cubics, as a font outline is.
                std::string d;
                for (int k = 0; k < 8; ++k) {
                    const Real a0 = kTwoPi * k / 8, a1 = kTwoPi * (k + 1) / 8, t = 4.0 / 3.0 * std::tan(kTwoPi / 32);
                    const Vec2 p0{x + r * std::cos(a0), y + r * std::sin(a0)}, p3{x + r * std::cos(a1), y + r * std::sin(a1)};
                    const Vec2 c1 = p0 + Vec2{-std::sin(a0), std::cos(a0)} * (r * t);
                    const Vec2 c2 = p3 - Vec2{-std::sin(a1), std::cos(a1)} * (r * t);
                    if (k == 0) { std::snprintf(b, sizeof b, "M%.4f %.4f", p0.x, p0.y); d += b; }
                    std::snprintf(b, sizeof b, "C%.4f %.4f %.4f %.4f %.4f %.4f", c1.x, c1.y, c2.x, c2.y, p3.x, p3.y);
                    d += b;
                }
                return d + "Z";
            };
            // Even-odd, so the inner circle is a hole whichever way it runs.
            body += "<path fill-rule=\"evenodd\" d=\"" + ring(5.0) + ring(3.0) + "\"/>";
        }
        const auto t0 = std::chrono::steady_clock::now();
        const SvgDrawing d = parseSvg(page(body, R"(width="240mm" height="144mm" viewBox="0 0 240 144")"));
        const auto t1 = std::chrono::steady_clock::now();
        Sketch sk;
        insertSvg(sk, d, {});
        const SketchSolve s = solveSketch(sk);
        const auto t2 = std::chrono::steady_clock::now();
        const auto rs = sketchProfiles(sk);
        const auto filled = sketchFilledProfiles(sk, rs);
        const auto t3 = std::chrono::steady_clock::now();
        auto ms = [](auto a, auto b) { return std::chrono::duration<double, std::milli>(b - a).count(); };
        check(sk.entities.size() == 240u * 16u, "7,680 curves, got " + std::to_string(sk.entities.size()));
        check(s.solved, "solves: " + s.reason);
        check(rs.size() == 480 && filled.size() == 240, "480 regions, the 240 rings filled, got " +
                                                          std::to_string(rs.size()) + " / " +
                                                          std::to_string(filled.size()));
        std::printf("  read %.1f ms, into a sketch and solved %.1f ms, regions %.1f ms\n", ms(t0, t1),
                    ms(t1, t2), ms(t2, t3));
        check(ms(t0, t3) < 3000.0, "all of it in under three seconds");

        // Swept all at once: the rings, each with its counter left open.
        if (brep::available()) {
            const auto a = std::chrono::steady_clock::now();
            std::string why;
            BrepRef all = brep::sketchSolids(sk, rs, filled, 0.0, 3.0, 1, &why);
            const auto b2 = std::chrono::steady_clock::now();
            check(all != nullptr, "the 240 rings swept together: " + why);
            if (all) {
                const Body body(std::move(all));
                // Each ring is the cubic circle of 5 less that of 3, a hair
                // over pi (25 - 9); 240 of them, 3 deep.
                const Real want = 240.0 * kPi * 16.0 * 3.0;
                const Real got = body.health(false).volume;
                check(std::fabs(got / want - 1.0) < 1e-4, std::to_string(want) + " mm3, got " + std::to_string(got));
            }
            std::printf("  swept 240 regions in %.1f ms\n", ms(a, b2));
        }

        // And a drag: one point of one letter pulled, every frame, with the
        // whole drawing in the system.
        SketchSolver solver(sk);
        const auto t4 = std::chrono::steady_clock::now();
        const bool began = solver.beginDrag(sk.points[5].id);
        const auto t5 = std::chrono::steady_clock::now();
        double worst = 0;
        for (int f = 0; f < 10; ++f) {
            const auto a = std::chrono::steady_clock::now();
            solver.dragTo(sk.points[5].at + Vec2{0.1 * f, 0.05 * f});
            worst = std::max(worst, ms(a, std::chrono::steady_clock::now()));
        }
        solver.endDrag();
        check(began, "the drag takes hold");
        std::printf("  drag: begin %.1f ms, worst frame %.2f ms\n", ms(t4, t5), worst);
        check(worst < 50.0, "a drag frame inside 50 ms");
    }

    if (failures) {
        std::printf("%d FAILED\n", failures);
        return 1;
    }
    std::printf("all passed\n");
    return 0;
}
