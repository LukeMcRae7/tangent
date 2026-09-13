// Checking a part against a printer rather than against itself.
//
// The numbers here are ones a person can work out: a box shelled to a 0.3mm
// wall has a 0.3mm wall, a cone with a sixty-degree side leans sixty degrees.
// Anything this reports that does not match those is wrong, however plausible
// it looks on screen.
#include "app/printability.h"
#include "geom/operations.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near(Real a, Real b, Real eps = 0.05) { return std::fabs(a - b) < eps; }

static Body makeBox(Real w, Real d, Real h) {
    PrimitiveSpec s;
    s.kind = PrimitiveKind::Box;
    s.box = {w, d, h};
    Body b;
    makePrimitive(s, b, Backend::Brep);
    return b;
}
static FaceId facing(const Body& b, Vec3 dir) {
    std::vector<FaceId> f;
    b.allFaces(f);
    FaceId best = kInvalid;
    Real bd = -1e30;
    for (FaceId x : f) {
        const Real d = dot(b.faceNormal(x), normalize(dir));
        if (d > bd) { bd = d; best = x; }
    }
    return best;
}
static PrintReport look(const Body& b, const PrintProfile& p = {}) {
    RenderMesh rm;
    b.tessellate(rm);
    return checkPrintability(b, rm, p);
}

int main() {
    if (!brep::available()) {
        std::printf("exact kernel not built\n");
        return 0;
    }

    std::printf("--- a solid box has nothing wrong with it ---\n");
    {
        const PrintReport r = look(makeBox(20, 20, 20));
        check(r.clean(), "a box prints");
        check(r.solid, "and is a solid");
        check(summarise(r).empty(), "with nothing to say about it");
    }

    std::printf("--- a wall thinner than the nozzle is reported ---\n");
    {
        // Shelled to 0.3mm, which is under the 0.8 two perimeters need.
        Body b = makeBox(20, 20, 20);
        std::string why;
        check(shellBody(b, {facing(b, {0, 0, 1})}, 0.3, 1, &why), "shelled thin: " + why);

        const PrintReport r = look(b);
        check(r.thinWalls > 0, "the thin wall was found");
        check(near(r.thinnestWallMm, 0.3), "and measured at 0.3mm, got " +
                                               std::to_string(r.thinnestWallMm));
        std::printf("  %s\n", summarise(r).c_str());
    }

    std::printf("--- a wall the nozzle can manage is not ---\n");
    {
        Body b = makeBox(20, 20, 20);
        std::string why;
        check(shellBody(b, {facing(b, {0, 0, 1})}, 1.2, 2, &why), "shelled: " + why);
        const PrintReport r = look(b);
        check(r.thinWalls == 0, "1.2mm is thick enough to say nothing about");
    }

    std::printf("--- and the profile is what decides which ---\n");
    {
        Body b = makeBox(20, 20, 20);
        std::string why;
        shellBody(b, {facing(b, {0, 0, 1})}, 0.5, 3, &why);

        PrintProfile fine;
        fine.nozzleMm = 0.2;
        fine.minWallMm = 0.4;
        check(look(b, fine).thinWalls == 0, "a finer nozzle prints a 0.5mm wall");

        PrintProfile coarse;
        coarse.nozzleMm = 0.8;
        coarse.minWallMm = 1.6;
        check(look(b, coarse).thinWalls > 0, "a coarser one does not");
    }

    std::printf("--- a face leaning too far needs holding up ---\n");
    {
        // A box with its top grown by half: the sides lean by
        // atan((30-20)/2 / 20) = 14 degrees, which any printer bridges.
        Body gentle = makeBox(20, 20, 20);
        std::string why;
        check(scaleFaces(gentle, {facing(gentle, {0, 0, 1})}, 1.5, 4, &why),
              "tapered gently: " + why);
        check(look(gentle).overhangs == 0, "fourteen degrees needs no support");

        // Grown by four: atan(30/20) = 56 degrees, which does.
        Body steep = makeBox(20, 20, 20);
        check(scaleFaces(steep, {facing(steep, {0, 0, 1})}, 4.0, 5, &why),
              "tapered steeply: " + why);
        const PrintReport r = look(steep);
        check(r.overhangs > 0, "fifty-six degrees does");
        check(near(r.steepestOverhangDeg, 56.0, 1.5),
              "and is measured at 56, got " + std::to_string(r.steepestOverhangDeg));
        std::printf("  %s\n", summarise(r).c_str());
    }

    std::printf("--- the underside of a box is not an overhang ---\n");
    {
        // It sits on the bed. A face pointing straight down is the one case a
        // slicer never has to bridge, and calling it an overhang would flag
        // every part ever made.
        const PrintReport r = look(makeBox(20, 20, 20));
        check(r.overhangs == 0, "the bottom face is not reported");
    }

    std::printf("--- what it costs ---\n");
    {
        // A ray per face against every triangle. Fine on a box; the question is
        // what it does on something with a few hundred faces, because that
        // decides whether it can run whenever the geometry moves or has to be
        // asked for.
        Body plate = makeBox(100, 100, 10);
        std::string why;
        for (int i = 0; i < 8; ++i) {
            PrimitiveSpec bore;
            bore.kind = PrimitiveKind::Cylinder;
            bore.cylinder = {3.3, 40, 32};
            Body tool;
            makePrimitive(bore, tool, Backend::Brep);
            const Real a2 = 2.0 * 3.14159265358979 * i / 8.0;
            tool.transform(translate({35.0 * std::cos(a2), 35.0 * std::sin(a2), 0}));
            Body out;
            if (booleanOp(plate, tool, BooleanOp::Difference, out, 200 + i, false, &why))
                plate = std::move(out);
        }
        RenderMesh rm;
        plate.tessellate(rm);

        const auto t0 = std::chrono::steady_clock::now();
        const PrintReport r = checkPrintability(plate, rm);
        const double ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - t0).count();
        std::printf("  %d faces, %zu triangles: %.1f ms  (%s)\n", plate.faceCount(),
                    rm.triangles.size() / 3, ms,
                    summarise(r).empty() ? "nothing to report" : summarise(r).c_str());
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
