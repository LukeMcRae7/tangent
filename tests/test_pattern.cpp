// Patterns and mirrors.
//
// A pattern is arithmetic you can check: five holes take five holes' worth of
// material out, eight round a circle take eight, and a mirrored half comes to
// twice what the half did. That is the point of testing it this way -- a
// pattern that lands its copies in almost the right place still builds, still
// looks plausible in a screenshot, and is still wrong. The volume is not
// fooled.
#include "geom/operations.h"
#include "scene/feature.h"
#include "scene/serialize.h"
#include "scene/scene.h"
#include "temp_path.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}

static Body makeBox(Real w, Real d, Real h) {
    PrimitiveSpec spec;
    spec.kind = PrimitiveKind::Box;
    spec.box = {w, d, h};
    Body b;
    makePrimitive(spec, b, Backend::Brep);
    return b;
}

static Body makeCyl(Real r, Real h) {
    PrimitiveSpec spec;
    spec.kind = PrimitiveKind::Cylinder;
    spec.cylinder = {r, h, 32};
    Body b;
    makePrimitive(spec, b, Backend::Brep);
    return b;
}

static Body movedTo(Body b, Vec3 t) { b.transform(translate(t)); return b; }

static Real volumeOf(const Body& b) { return b.health(false).volume; }

// Within a tenth of a percent. The kernel is exact; the volume is integrated
// off a tessellation, so a curved body's number is close rather than equal.
static bool near1(Real got, Real want) {
    return std::fabs(got - want) < std::fabs(want) * 1e-3 + 1e-6;
}

int main() {
    if (!brep::available()) {
        std::printf("exact kernel not built; patterning needs one\n");
        return 0;
    }
    std::printf("pattern\n");

    // --- A row of holes ------------------------------------------------
    // One cut, patterned five times along the bar. Each hole is a cylinder of
    // radius 3 through 10mm of plate, so the bar should come out 5 holes light.
    {
        Body bar = makeBox(100, 20, 10);
        const Real barVol = volumeOf(bar);
        check(near1(barVol, 20000.0), "the bar is 20000 mm3");

        Body tool = movedTo(makeCyl(3, 30), {-30, 0, 0});
        PatternSpec s;
        s.mode = PatternMode::Linear;
        s.count = 5;
        s.dir = {1, 0, 0};
        s.step = 15.0;
        s.op = BooleanOp::Difference;

        std::string why;
        const bool ok = patternBody(bar, tool, s, 1, &why);
        check(ok, "the row of holes builds: " + why);
        if (ok) {
            const Real holes = 5.0 * kPi * 9.0 * 10.0;
            const Real got = volumeOf(bar);
            check(near1(got, barVol - holes), "five holes remove five holes");
            if (!near1(got, barVol - holes))
                std::printf("    got %.2f, wanted %.2f\n", got, barVol - holes);
        }
    }

    // --- A bolt circle -------------------------------------------------
    // The same cut, turned 45 degrees at a time about the disc's own axis.
    {
        Body disc = makeCyl(30, 5);
        const Real discVol = volumeOf(disc);

        Body tool = movedTo(makeCyl(3, 20), {20, 0, 0});
        PatternSpec s;
        s.mode = PatternMode::Circular;
        s.count = 8;
        s.dir = {0, 0, 1};
        s.origin = {0, 0, 0};
        s.stepAngle = radians(360.0 / 8.0);
        s.op = BooleanOp::Difference;

        std::string why;
        const bool ok = patternBody(disc, tool, s, 2, &why);
        check(ok, "the bolt circle builds: " + why);
        if (ok) {
            const Real holes = 8.0 * kPi * 9.0 * 5.0;
            check(near1(volumeOf(disc), discVol - holes), "eight holes round the circle");
            // Eight holes is eight more rims, each a cylinder wall and the two
            // faces it broke through are still the two they were.
            check(disc.faceCount() == 3 + 8, "a hole apiece, and nothing else split");
            if (disc.faceCount() != 3 + 8)
                std::printf("    %d faces\n", disc.faceCount());
        }
    }

    // --- A turn that does not close ------------------------------------
    // Four copies 30 degrees apart is a fan, not a circle. The step is the step;
    // it is not divided into a full turn behind the user's back.
    {
        Body disc = makeCyl(30, 5);
        const Real discVol = volumeOf(disc);
        Body tool = movedTo(makeCyl(3, 20), {20, 0, 0});
        PatternSpec s;
        s.mode = PatternMode::Circular;
        s.count = 4;
        s.dir = {0, 0, 1};
        s.stepAngle = radians(30.0);
        s.op = BooleanOp::Difference;
        std::string why;
        check(patternBody(disc, tool, s, 3, &why), "the fan builds: " + why);
        check(near1(volumeOf(disc), discVol - 4.0 * kPi * 9.0 * 5.0), "four holes, not eight");
    }

    // --- Mirroring the body --------------------------------------------
    // Half a part reflected into the whole of it. The box runs 0..10 in x, so
    // the reflection runs -10..0 and the two fuse into one 20mm body.
    {
        Body half = movedTo(makeBox(10, 20, 5), {5, 0, 0});
        PatternSpec s;
        s.mode = PatternMode::Mirror;
        s.dir = {1, 0, 0};
        s.origin = {0, 0, 0};
        s.op = BooleanOp::Union;

        std::string why;
        const bool ok = patternBody(half, Body{}, s, 4, &why);
        check(ok, "the mirror builds: " + why);
        if (ok) {
            check(near1(volumeOf(half), 2000.0), "the halves make a whole");
            AABB b = half.bounds();
            check(std::fabs(b.min.x + 10.0) < 1e-6 && std::fabs(b.max.x - 10.0) < 1e-6,
                  "it reaches the same distance either side");
            // A reflection turns a shape inside out unless the orientations are
            // put back, and a solid describing the void around itself has a
            // negative volume. This is that check.
            check(volumeOf(half) > 0, "the reflected half is solid, not inverted");
        }
    }

    // --- Mirroring a cut -----------------------------------------------
    // The tool form of the same thing: one pocket becomes a matched pair.
    {
        Body plate = makeBox(60, 20, 10);
        Body tool = movedTo(makeCyl(4, 30), {20, 0, 0});
        PatternSpec s;
        s.mode = PatternMode::Mirror;
        s.dir = {1, 0, 0};
        s.op = BooleanOp::Difference;
        std::string why;
        const bool ok = patternBody(plate, tool, s, 5, &why);
        check(ok, "the mirrored cut builds: " + why);
        if (ok)
            check(near1(volumeOf(plate), 12000.0 - 2.0 * kPi * 16.0 * 10.0),
                  "both holes are taken out");
    }

    // --- Refusals ------------------------------------------------------
    {
        Body b = makeBox(10, 10, 10);
        PatternSpec s;
        s.count = 1;
        std::string why;
        check(!patternBody(b, Body{}, s, 6, &why), "one copy is refused");
        check(!why.empty(), "and says why");

        s.count = 3;
        s.dir = {0, 0, 0};
        check(!patternBody(b, Body{}, s, 7, &why), "no direction is refused");
        check(near1(volumeOf(b), 1000.0), "a refused pattern leaves the body alone");
    }

    // --- Through the chain and a file ----------------------------------
    {
        Scene scene;
        scene.setDefaultBackend(Backend::Brep);
        PrimitiveSpec base;
        base.kind = PrimitiveKind::Cylinder;
        base.cylinder = {30, 5, 48};
        const ObjectId id = scene.addPrimitive(PrimitiveKind::Cylinder, base);
        SceneObject* o = scene.find(id);
        check(o != nullptr, "the object is there");
        if (o) {

            Feature f;
            f.kind = FeatureKind::Pattern;
            f.uid = 4242;
            f.patternMode = PatternMode::Circular;
            f.patternCount = 6;
            f.axisDir = {0, 0, 1};
            f.angle = radians(60.0);
            f.booleanOp = BooleanOp::Difference;
            f.bakedBody = movedTo(makeCyl(3, 20), {20, 0, 0});
            o->features.push_back(f);
            scene.rebuild(id);

            check(!o->features.back().errored, "the pattern feature evaluates: " +
                                               o->features.back().error);
            const Real want = kPi * 900.0 * 5.0 - 6.0 * kPi * 9.0 * 5.0;
            check(near1(volumeOf(o->body), want), "six holes through the chain");
            check(std::string(o->features.back().displayKind()) == "Pattern",
                  "a circular pattern is called one");

            // The parametric promise: change the thing the pattern sits on
            // and the pattern comes back, in the right places, on the new
            // shape. A thicker disc is six holes through a thicker disc.
            o->spec.cylinder = {30, 9, 48};   // rebuild() takes the spec as given
            scene.rebuild(id);
            check(!o->features.back().errored,
                  "the pattern survives a root edit: " + o->features.back().error);
            const Real thicker = kPi * 900.0 * 9.0 - 6.0 * kPi * 9.0 * 9.0;
            check(near1(volumeOf(o->body), thicker), "six holes through the thicker disc");
            if (!near1(volumeOf(o->body), thicker))
                std::printf("    got %.2f, wanted %.2f\n", volumeOf(o->body), thicker);
            o->spec.cylinder = {30, 5, 48};
            scene.rebuild(id);

            const std::string path = tempPath("pattern.tgt");
            check(saveProject(scene, path).ok, "it saves");
            Scene back;
            check(loadProject(back, path).ok, "it loads");
            SceneObject* o2 = back.objects().empty() ? nullptr : back.objects()[0].get();
            if (o2 && o2->features.size() == 2) {
                const Feature& g = o2->features[1];
                check(g.kind == FeatureKind::Pattern, "it is still a pattern");
                check(g.patternMode == PatternMode::Circular, "still circular");
                check(g.patternCount == 6, "still six");
                check(near1(volumeOf(o2->body), want), "and still the same solid");
            } else {
                check(false, "the reloaded object has both features");
            }
            std::remove(path.c_str());
        }
    }

    // --- Mirror is called Mirror ---------------------------------------
    {
        Feature f;
        f.kind = FeatureKind::Pattern;
        f.patternMode = PatternMode::Mirror;
        check(std::string(f.displayKind()) == "Mirror", "a mirror is not called a pattern of two");
    }

    std::printf("%s\n", failures ? "FAILED" : "ok");
    return failures ? 1 : 0;
}
