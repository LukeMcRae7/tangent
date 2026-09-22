// The vendored solver, used directly: does it build here, solve, count what is
// left free, and name the constraints that disagree?
//
// Written against planegcs itself rather than Tangent's sketch, so a failure
// here is about the vendoring -- the shims, Eigen, the flags -- and not about
// anything built on top of it.
#include "planegcs/GCS.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <memory>
#include <string>
#include <vector>

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near(double a, double b, double eps = 1e-7) { return std::fabs(a - b) < eps; }

namespace {

// Owns the doubles the solver works on. A deque, so that adding a parameter
// never moves the ones the solver already holds pointers to.
struct Params {
    std::deque<double> values;
    std::vector<double*> unknowns;
    double* unknown(double v) {
        values.push_back(v);
        unknowns.push_back(&values.back());
        return &values.back();
    }
    double* fixed(double v) {
        values.push_back(v);
        return &values.back();
    }
};

GCS::Point point(Params& p, double x, double y) {
    GCS::Point pt;
    pt.x = p.unknown(x);
    pt.y = p.unknown(y);
    return pt;
}

GCS::Line line(GCS::Point a, GCS::Point b) {
    GCS::Line l;
    l.p1 = a;
    l.p2 = b;
    return l;
}

// Which decomposition the diagnosis uses. planegcs picks dense below a thousand
// parameters by default; every result here is checked with both, because the
// sketch will choose by speed and has to be able to trust either.
void use(GCS::System& sys, GCS::QRAlgorithm qr) {
    sys.autoChooseAlgorithm = false;
    sys.qrAlgorithm = qr;
}

const char* nameOf(GCS::QRAlgorithm qr) { return qr == GCS::EigenDenseQR ? "dense QR" : "sparse QR"; }

// A rectangle drawn roughly, as a hand would: four lines, each with its own
// ends, and nothing yet said about how they relate.
struct Rectangle {
    Params params;
    GCS::Line bottom, right, top, left;
    GCS::System sys;
    int tag = 0;

    Rectangle() {
        bottom = line(point(params, 0.1, -0.2), point(params, 9.7, 0.3));
        right  = line(point(params, 10.2, 0.1), point(params, 9.9, 5.4));
        top    = line(point(params, 10.3, 4.8), point(params, -0.4, 5.2));
        left   = line(point(params, 0.2, 5.1), point(params, -0.1, 0.2));
        sys.addConstraintP2PCoincident(bottom.p2, right.p1, ++tag);
        sys.addConstraintP2PCoincident(right.p2, top.p1, ++tag);
        sys.addConstraintP2PCoincident(top.p2, left.p1, ++tag);
        sys.addConstraintP2PCoincident(left.p2, bottom.p1, ++tag);
        sys.addConstraintHorizontal(bottom, ++tag);
        sys.addConstraintHorizontal(top, ++tag);
        sys.addConstraintVertical(left, ++tag);
        sys.addConstraintVertical(right, ++tag);
    }

    int prepare(GCS::QRAlgorithm qr) {
        use(sys, qr);
        sys.declareUnknowns(params.unknowns);
        sys.initSolution();
        return sys.dofsNumber();
    }
};

} // namespace

int main() {
  for (GCS::QRAlgorithm qr : {GCS::EigenDenseQR, GCS::EigenSparseQR}) {
    std::printf("===== diagnosis by %s =====\n", nameOf(qr));
    std::printf("--- a rectangle, constrained a step at a time ---\n");
    {
        Rectangle r;
        check(r.prepare(qr) == 4, "four lines at right angles leave 4 freedoms: place it (2), size it (2)");

        r.sys.addConstraintCoordinateX(r.bottom.p1, r.params.fixed(0.0), ++r.tag);
        r.sys.addConstraintCoordinateY(r.bottom.p1, r.params.fixed(0.0), ++r.tag);
        check(r.prepare(qr) == 2, "a corner fixed at the origin leaves 2");

        r.sys.addConstraintP2PDistance(r.bottom.p1, r.bottom.p2, r.params.fixed(40.0), ++r.tag);
        r.sys.addConstraintP2PDistance(r.left.p2, r.left.p1, r.params.fixed(25.0), ++r.tag);
        check(r.prepare(qr) == 0, "width and height leave none: fully constrained");

        const int status = r.sys.solve(true, GCS::DogLeg);
        check(status == GCS::Success, "and it solves");
        r.sys.applySolution();
        check(near(*r.bottom.p1.x, 0) && near(*r.bottom.p1.y, 0), "the corner is at the origin");
        check(near(*r.top.p1.x, 40) && near(*r.top.p1.y, 25), "the far corner is at 40, 25");
        check(near(*r.right.p1.x, *r.bottom.p2.x) && near(*r.right.p1.y, *r.bottom.p2.y),
              "corners that were drawn apart now meet");
        std::printf("  solved: far corner at %.9f, %.9f\n", *r.top.p1.x, *r.top.p1.y);
    }

    std::printf("--- constraints that disagree, and ones that repeat ---\n");
    {
        Rectangle r;
        r.sys.addConstraintCoordinateX(r.bottom.p1, r.params.fixed(0.0), ++r.tag);
        r.sys.addConstraintCoordinateY(r.bottom.p1, r.params.fixed(0.0), ++r.tag);
        const int width40 = ++r.tag;
        r.sys.addConstraintP2PDistance(r.bottom.p1, r.bottom.p2, r.params.fixed(40.0), width40);
        const int width30 = ++r.tag;
        r.sys.addConstraintP2PDistance(r.top.p1, r.top.p2, r.params.fixed(30.0), width30);
        r.prepare(qr);

        GCS::VEC_I conflicting;
        r.sys.getConflicting(conflicting);
        auto has = [](const GCS::VEC_I& v, int t) {
            for (int x : v) if (x == t) return true;
            return false;
        };
        check(r.sys.hasConflicting(), "40 wide at the bottom and 30 at the top conflict");
        check(has(conflicting, width40) || has(conflicting, width30),
              "and the conflict names one of the two widths");
        std::printf("  %zu constraints named as conflicting\n", conflicting.size());
    }
    {
        Rectangle r;
        const int again = ++r.tag;
        r.sys.addConstraintHorizontal(r.bottom, again);   // already said
        r.prepare(qr);
        GCS::VEC_I redundant;
        r.sys.getRedundant(redundant);
        check(r.sys.hasRedundant() && !redundant.empty(), "saying a line is horizontal twice is redundant");
        check(r.sys.dofsNumber() == 4, "and changes nothing about the freedoms");
    }

    std::printf("--- a line tangent to a circle ---\n");
    {
        Params p;
        GCS::System sys;
        GCS::Circle c;
        c.center = point(p, 3.0, 4.0);
        c.rad = p.unknown(9.0);
        GCS::Line l = line(point(p, -20, 18), point(p, 20, 15));
        int tag = 0;
        sys.addConstraintCoordinateX(c.center, p.fixed(0.0), ++tag);
        sys.addConstraintCoordinateY(c.center, p.fixed(0.0), ++tag);
        sys.addConstraintCircleRadius(c, p.fixed(10.0), ++tag);
        sys.addConstraintHorizontal(l, ++tag);
        sys.addConstraintTangent(l, c, ++tag);
        use(sys, qr);
        sys.declareUnknowns(p.unknowns);
        sys.initSolution();
        check(sys.solve(true, GCS::DogLeg) == GCS::Success, "solves");
        sys.applySolution();
        check(near(std::fabs(*l.p1.y), 10.0) && near(*l.p1.y, *l.p2.y), "the line sits one radius off the centre");
        std::printf("  line at y = %.9f\n", *l.p1.y);
    }

    std::printf("--- dragging a long chain, a frame at a time ---\n");
    {
        // 200 segments joined end to end, alternately horizontal and vertical,
        // half their lengths set, the first end fixed and the last end dragged:
        // the shape of an interactive drag on a large sketch. The drag holds
        // the point with a temporary constraint, as FreeCAD's does.
        constexpr int kSegments = 200;
        Params p;
        GCS::System sys;
        std::vector<GCS::Line> lines;
        int tag = 0;
        double x = 0, y = 0;
        for (int i = 0; i < kSegments; ++i) {
            const double nx = i % 2 == 0 ? x + 5.0 : x + 0.3;
            const double ny = i % 2 == 0 ? y + 0.2 : y + 5.0;
            lines.push_back(line(point(p, x, y), point(p, nx, ny)));
            x = nx;
            y = ny;
        }
        sys.addConstraintCoordinateX(lines[0].p1, p.fixed(0.0), ++tag);
        sys.addConstraintCoordinateY(lines[0].p1, p.fixed(0.0), ++tag);
        for (int i = 0; i < kSegments; ++i) {
            if (i > 0) sys.addConstraintP2PCoincident(lines[i - 1].p2, lines[i].p1, ++tag);
            if (i % 2 == 0) sys.addConstraintHorizontal(lines[i], ++tag);
            else            sys.addConstraintVertical(lines[i], ++tag);
            // Two lengths in every four are left free, one each way, so the end
            // can move in both directions.
            if (i % 4 < 2) sys.addConstraintP2PDistance(lines[i].p1, lines[i].p2, p.fixed(5.0), ++tag);
        }
        use(sys, qr);
        sys.declareUnknowns(p.unknowns);
        auto t0 = std::chrono::steady_clock::now();
        sys.initSolution();
        const double setupMs =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        const int dofs = sys.dofsNumber();
        check(dofs == kSegments / 2, "a free length in every other segment: " + std::to_string(dofs));
        check(sys.solve(true, GCS::DogLeg) == GCS::Success, "the chain settles");
        sys.applySolution();

        GCS::Point& end = lines.back().p2;
        double dragX = *end.x, dragY = *end.y;
        GCS::Point handle;
        handle.x = p.fixed(dragX);
        handle.y = p.fixed(dragY);
        sys.addConstraintP2PCoincident(handle, end, GCS::DefaultTemporaryConstraint);
        sys.initSolution();

        double worst = 0, total = 0;
        int solved = 0;
        constexpr int kFrames = 60;
        for (int f = 0; f < kFrames; ++f) {
            dragX += 0.5;
            dragY += 0.5;
            *handle.x = dragX;
            *handle.y = dragY;
            const auto a = std::chrono::steady_clock::now();
            const int status = sys.solve(true, GCS::DogLeg);
            const double ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a).count();
            if (status == GCS::Success) {
                sys.applySolution();
                ++solved;
            }
            worst = std::max(worst, ms);
            total += ms;
        }
        check(solved == kFrames, "every frame of the drag solves: " + std::to_string(solved));
        check(near(*end.x, dragX, 1e-6) && near(*end.y, dragY, 1e-6), "and the end follows the pointer");
        std::printf("  %d segments, %d freedoms: set up %.1f ms, drag %.2f ms a frame, worst %.2f ms\n",
                    kSegments, dofs, setupMs, total / kFrames, worst);
    }
  }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
