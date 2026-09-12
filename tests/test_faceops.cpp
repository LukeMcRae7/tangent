// Moving a face, and dividing one.
//
// Both are direct modelling: the body stays one body and only the face the
// user pointed at changes, with everything around it stretching or splitting to
// follow. That is a different contract from a boolean, and the ways it goes
// wrong are different too -- a rotation that quietly tips the wrong face, or a
// divide that hands back two solids instead of one solid with more faces.
#include "geom/operations.h"

#include <cmath>
#include <cstdio>
#include <set>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near(Real a, Real b, Real eps = 1e-4) { return std::fabs(a - b) < eps; }

static Body makeBox(Real w, Real d, Real h) {
    PrimitiveSpec spec;
    spec.kind = PrimitiveKind::Box;
    spec.box = {w, d, h};
    Body b;
    makePrimitive(spec, b, Backend::Brep);
    return b;
}

// The face whose normal points most nearly along `dir`.
static FaceId facing(const Body& b, Vec3 dir) {
    std::vector<FaceId> faces;
    b.allFaces(faces);
    FaceId best = kInvalid;
    Real bestDot = -1e30;
    for (FaceId f : faces) {
        const Real d = dot(b.faceNormal(f), normalize(dir));
        if (d > bestDot) { bestDot = d; best = f; }
    }
    return best;
}

int main() {
    if (!brep::available()) {
        std::printf("exact kernel not built; moving a face needs one\n");
        return 0;
    }

    std::printf("--- pushing a face out makes the body bigger, and no other ---\n");
    {
        Body b = makeBox(20, 20, 20);
        const Real before = b.health(false).volume;
        const int facesBefore = b.faceCount();

        std::string why;
        std::vector<FaceId> moved;
        check(extrudeFaces(b, {facing(b, {0, 0, 1})}, 5.0, &moved, 4001,
                           ExtrudeOp::Auto, &why),
              "pushed the top face out: " + why);
        check(near(b.health(false).volume, before + 20.0 * 20.0 * 5.0, 1e-3),
              "by exactly the material it swept");
        std::printf("  faces %d -> %d\n", facesBefore, b.faceCount());
        check(b.faceCount() == facesBefore,
              "and the body still has the faces it started with");
        check(b.validate(), "and is still a solid");
        std::printf("  %.0f -> %.0f mm3\n", before, b.health(false).volume);
    }

    std::printf("--- push-pull and extrude are not the same operation ---\n");
    {
        // Both sweep the face and combine the result, and on the volume they
        // agree exactly. What they disagree about is what is left behind: the
        // prism's walls are flush with the walls they slid along, and whether
        // those merge decides whether you moved a face or grew a boss with an
        // outline of its own.
        Body pushed = makeBox(20, 20, 20);
        Body grown  = makeBox(20, 20, 20);
        std::string why;

        check(extrudeFaces(pushed, {facing(pushed, {0, 0, 1})}, 5.0, nullptr, 4101,
                           ExtrudeOp::Auto, &why, /*mergeFlush=*/true),
              "pushed: " + why);
        check(extrudeFaces(grown, {facing(grown, {0, 0, 1})}, 5.0, nullptr, 4102,
                           ExtrudeOp::Auto, &why, /*mergeFlush=*/false),
              "extruded: " + why);

        check(near(pushed.health(false).volume, grown.health(false).volume, 1e-6),
              "the same material either way");
        check(pushed.faceCount() == 6,
              "pushed: a taller box is a box");
        check(grown.faceCount() == 10,
              "extruded: the boss keeps its own four walls, so ten faces");
        check(pushed.validate() && grown.validate(), "both are solids");
        std::printf("  push %d faces, extrude %d, both %.0f mm3\n",
                    pushed.faceCount(), grown.faceCount(), grown.health(false).volume);

        // And that is the point of the difference: on the extruded one the
        // upper band of a side wall is a face of its own and can be taken hold
        // of, where on the pushed one there is nothing there to take.
        auto bandsOnPlusX = [](const Body& b) {
            std::vector<FaceId> fs;
            b.allFaces(fs);
            int n = 0;
            for (FaceId f : fs)
                if (dot(b.faceNormal(f), Vec3{1, 0, 0}) > 0.99) ++n;
            return n;
        };
        check(bandsOnPlusX(pushed) == 1, "one face on the pushed side");
        check(bandsOnPlusX(grown) == 2, "two on the extruded one");
    }

    std::printf("--- pulling one in takes material away ---\n");
    {
        Body b = makeBox(20, 20, 20);
        const Real before = b.health(false).volume;
        std::string why;
        check(extrudeFaces(b, {facing(b, {0, 0, 1})}, -4.0, nullptr, 4002,
                           ExtrudeOp::Auto, &why),
              "pulled the top face in: " + why);
        check(b.health(false).volume < before, "the body got smaller");
        check(near(b.health(false).volume, before - 20.0 * 20.0 * 4.0, 1e-3),
              "by the material it swept back through");
    }

    std::printf("--- rotating a face tilts it and leaves one solid ---\n");
    {
        Body b = makeBox(20, 20, 20);
        const Real before = b.health(false).volume;
        const FaceId top = facing(b, {0, 0, 1});
        const Vec3 wasNormal = b.faceNormal(top);

        // Hinge on the top face's own edge that runs along X, at the -Y side.
        std::vector<EdgeId> edges;
        b.faceEdges(top, edges);
        EdgeId hinge = kInvalid;
        Real lowest = 1e30;
        for (EdgeId e : edges) {
            Vec3 p, q;
            b.edgePositions(e, p, q);
            if (std::fabs((q - p).x) < 1e-6) continue;     // not along X
            const Real y = (p.y + q.y) * 0.5;
            if (y < lowest) { lowest = y; hinge = e; }
        }
        check(hinge != kInvalid, "found an edge to pivot about");

        Vec3 a, c;
        b.edgePositions(hinge, a, c);
        std::string why;
        check(rotateFaces(b, {top}, radians(12.0), a, normalize(c - a), 4003, &why),
              "rotated the top face: " + why);
        check(b.validate(), "and it is still one solid");

        const FaceId nowTop = facing(b, {0, 0, 1});
        const Vec3 isNormal = b.faceNormal(nowTop);
        const Real turned = degrees(std::acos(clampf(dot(wasNormal, isNormal), -1.0, 1.0)));
        check(near(turned, 12.0, 0.5),
              "by twelve degrees, not " + std::to_string(turned));
        check(std::fabs(b.health(false).volume - before) > 1.0,
              "and the body changed shape");
        std::printf("  tilted %.3f degrees\n", turned);
    }

    std::printf("--- a face can be moved along an axis that is not its normal ---\n");
    {
        // Sweeping along a world axis rather than the face's own normal is the
        // whole of what an axis constraint does underneath.
        Body b = makeBox(20, 20, 20);
        std::string why;
        check(extrudeFaces(b, {facing(b, {0, 0, 1})}, 5.0, nullptr, 4201,
                           ExtrudeOp::Auto, &why, true, Vec3{0, 0, 1}),
              "moved along +Z: " + why);
        check(near(b.health(false).volume, 8000.0 + 2000.0, 1e-3),
              "the same as along its normal, since that is where +Z points");

        // A direction lying in the face moves its plane nowhere, and says so
        // rather than building something that looks like an answer.
        Body flat = makeBox(20, 20, 20);
        check(!extrudeFaces(flat, {facing(flat, {0, 0, 1})}, 5.0, nullptr, 4202,
                            ExtrudeOp::Auto, &why, true, Vec3{1, 0, 0}),
              "refused a sweep along the face");
        check(!why.empty(), "and gave a reason: " + why);
    }

    std::printf("--- a rotation goes both ways ---\n");
    {
        // Positive and negative are mirror images about the hinge, which is
        // what makes the sign mean "which way round" rather than "or not".
        auto tilt = [&](Real deg) {
            Body b = makeBox(20, 20, 20);
            const FaceId top = facing(b, {0, 0, 1});
            std::vector<EdgeId> edges;
            b.faceEdges(top, edges);
            EdgeId hinge = kInvalid;
            Real lowest = 1e30;
            for (EdgeId e : edges) {
                Vec3 p, q;
                b.edgePositions(e, p, q);
                if (std::fabs((q - p).x) < 1e-6) continue;
                const Real y = (p.y + q.y) * 0.5;
                if (y < lowest) { lowest = y; hinge = e; }
            }
            Vec3 a2, c2;
            b.edgePositions(hinge, a2, c2);
            std::string w;
            const bool ok = rotateFaces(b, {top}, radians(deg), a2, normalize(c2 - a2), 4203, &w);
            return ok ? b.health(false).volume : -1.0;
        };
        const Real up = tilt(12.0), down = tilt(-12.0);
        // Which sign tips which way depends on how the hinge happens to be
        // oriented, and that is not a fact worth asserting: what matters is
        // that the two are opposite and equal, so the sign means "which way
        // round" rather than "or not". The tool ties the drag direction to the
        // same hinge, so pulling one way always grows the number.
        check(up > 0.0 && down > 0.0, "both directions build");
        check(std::fabs(up - down) > 1.0, "and they are not the same shape");
        check((up - 8000.0) * (down - 8000.0) < 0.0,
              "one adds material and the other takes it");
        check(near(up + down, 16000.0, 1e-3),
              "mirror images about where it started");
        std::printf("  +12 deg %.1f mm3, -12 deg %.1f\n", up, down);
    }

    std::printf("--- a divide splits faces without splitting the body ---\n");
    {
        Body b = makeBox(30, 20, 10);
        const Real before = b.health(false).volume;
        const int facesBefore = b.faceCount();

        std::string why;
        check(divideBody(b, {0, 0, 0}, {1, 0, 0}, 4004, &why),
              "cut a loop across the middle: " + why);
        check(b.validate(), "still one solid");
        check(near(b.health(false).volume, before, 1e-6),
              "and exactly as much material as before");
        check(b.faceCount() > facesBefore,
              "with more faces than it started with");
        std::printf("  %d faces -> %d\n", facesBefore, b.faceCount());

        // The four faces the plane crossed are each two now; the two ends are
        // untouched. Six becomes ten.
        check(b.faceCount() == 10, "six faces became ten");
    }

    std::printf("--- and the halves can be told apart ---\n");
    {
        // The point of dividing is to take hold of one half. If both came back
        // with the same name, pushing "that face" would push them both and the
        // divide would have achieved nothing.
        Body b = makeBox(30, 20, 10);
        std::string why;
        check(divideBody(b, {0, 0, 0}, {1, 0, 0}, 4005, &why), "divided: " + why);

        // Push one half of the top and check only that half moved: the body
        // gains a step rather than a uniform layer.
        const FaceId top = facing(b, {0, 0, 1});
        const Vec3 c = b.faceCentroid(top);
        std::vector<FaceId> moved;
        check(extrudeFaces(b, {top}, 3.0, &moved, 4006, ExtrudeOp::Auto, &why),
              "pushed one half up: " + why);
        check(b.validate(), "still a solid");

        // Half the top, 15 x 20, raised 3mm.
        const Real added = b.health(false).volume - 30.0 * 20.0 * 10.0;
        check(near(added, 15.0 * 20.0 * 3.0, 1e-3),
              "only half the top moved, adding " + std::to_string(added));
        std::printf("  pushed the half at x %s 0, adding %.0f mm3\n",
                    c.x > 0 ? ">" : "<", added);
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
