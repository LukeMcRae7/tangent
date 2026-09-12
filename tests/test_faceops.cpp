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

    std::printf("--- scaling a face makes a frustum of a known size ---\n");
    {
        // A 40 x 30 x 20 box with its top grown by half is a prismatoid, and a
        // prismatoid's volume is h/6 (bottom + 4 middle + top): 20/6 x (1200 +
        // 4x1875 + 2700) = 38000. Not a number this code produced -- one worked
        // out on paper, which is the only kind worth comparing against.
        Body b = makeBox(40, 30, 20);
        std::string why;
        check(scaleFaces(b, {facing(b, {0, 0, 1})}, 1.5, 4701, &why),
              "scaled the top by half again: " + why);
        check(b.validate(), "still a solid");
        check(near(b.health(false).volume, 38000.0, 1.0),
              "38000 mm3, got " + std::to_string(b.health(false).volume));
        check(b.faceCount() == 6, "and still six faces");

        // The top really is 60 x 45, and the bottom is untouched.
        const AABB box = b.bounds();
        check(near(box.max.x - box.min.x, 60.0, 0.05), "60 across the top");
        check(near(box.max.y - box.min.y, 45.0, 0.05), "45 deep at the top");
        std::printf("  %.1f mm3, %.1f x %.1f overall\n", b.health(false).volume,
                    box.max.x - box.min.x, box.max.y - box.min.y);
    }

    std::printf("--- and shrinking one is the same operation backwards ---\n");
    {
        // Half the size: 20/6 x (1200 + 4x675 + 300) = 20/6 x 4200 = 14000.
        Body b = makeBox(40, 30, 20);
        std::string why;
        check(scaleFaces(b, {facing(b, {0, 0, 1})}, 0.5, 4702, &why),
              "scaled the top to half: " + why);
        check(b.validate(), "still a solid");
        check(near(b.health(false).volume, 14000.0, 1.0),
              "14000 mm3, got " + std::to_string(b.health(false).volume));
        std::printf("  %.1f mm3\n", b.health(false).volume);

        // One is not a change, and nothing is not a face.
        Body u = makeBox(40, 30, 20);
        check(!scaleFaces(u, {facing(u, {0, 0, 1})}, 1.0, 4703, &why),
              "scaling by one is declined");
        check(!scaleFaces(u, {facing(u, {0, 0, 1})}, 0.0, 4704, &why),
              "and so is scaling to nothing");
        check(near(u.health(false).volume, 24000.0), "with the body untouched");
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

    std::printf("--- a move keeps the divisions someone put there ---\n");
    {
        // The seams a push leaves down the walls it slid along are worth
        // merging: nothing intersects there and the line is an artefact. A line
        // the user cut on purpose is the opposite, and a merge that cannot tell
        // them apart destroys work.
        Body b = makeBox(30, 20, 10);
        std::string why;
        check(divideBody(b, {0, 0, 0}, {1, 0, 0}, 4301, &why), "divided: " + why);
        const int afterDivide = b.faceCount();
        check(afterDivide == 10, "ten faces after the cut");

        // Push a face that has nothing to do with the division: an end.
        check(extrudeFaces(b, {facing(b, {0, 1, 0})}, 4.0, nullptr, 4302,
                           ExtrudeOp::Auto, &why, /*mergeFlush=*/true),
              "pushed an end out: " + why);
        check(b.validate(), "still a solid");

        // The cut round the middle is still there.
        int onPlusZ = 0;
        std::vector<FaceId> fs;
        b.allFaces(fs);
        for (FaceId f : fs)
            if (dot(b.faceNormal(f), Vec3{0, 0, 1}) > 0.99) ++onPlusZ;
        check(onPlusZ == 2, "the top is still two faces, not one");
        std::printf("  %d faces after the cut, %d after the push, top in %d\n",
                    afterDivide, b.faceCount(), onPlusZ);
    }

    std::printf("--- and merging is how you ask for them to go ---\n");
    {
        // What a move must not do behind your back is exactly what this does
        // when you ask for it.
        Body b = makeBox(30, 20, 10);
        std::string why;
        check(divideBody(b, {0, 0, 0}, {1, 0, 0}, 4401, &why), "divided: " + why);
        check(b.faceCount() == 10, "ten faces");
        const Real volume = b.health(false).volume;

        check(mergeDivisions(b, 4402, &why), "merged: " + why);
        check(b.faceCount() == 6, "back to six");
        check(near(b.health(false).volume, volume, 1e-6),
              "and not a cubic millimetre different: a division is a line, "
              "not a shape");
        check(b.validate(), "still a solid");

        // Nothing to drop is not a failure to report as a broken operation,
        // but it is not a change either, and saying so beats a no-op that
        // claims to have done something.
        check(!mergeDivisions(b, 4403, &why), "a second merge finds nothing");
        check(!why.empty(), "and says so: " + why);
        check(b.faceCount() == 6, "leaving the body alone");
    }

    std::printf("--- a division that defines the shape is not a division ---\n");
    {
        // The faces either side of a real edge are not coplanar, so there is
        // nothing to merge and the box survives being asked.
        Body b = makeBox(20, 20, 20);
        std::string why;
        check(!mergeDivisions(b, 4404, &why), "a plain box has nothing to merge");
        check(b.faceCount() == 6, "and keeps its six sides");
    }

    std::printf("--- what hollowing can be asked to hollow ---\n");
    {
        auto probe = [&](const char* what, Body b) {
            std::string why;
            Body t = b;
            const bool ok = shellBody(t, {facing(b, {0, 0, -1})}, 2.0, 4501, &why);
            std::printf("  %-32s %s %s\n", what, ok ? "hollowed" : "REFUSED",
                        why.c_str());
            return ok;
        };
        auto anUprightEdge = [](const Body& b) {
            std::vector<EdgeId> es;
            b.allEdges(es);
            for (EdgeId e : es) {
                Vec3 p, q;
                b.edgePositions(e, p, q);
                if (std::fabs((q - p).z) > 5.0) return e;
            }
            return EdgeId(kInvalid);
        };
        std::string w;

        check(probe("a plain box", makeBox(40, 30, 20)), "a plain box hollows");

        { Body b = makeBox(40, 30, 20);
          divideBody(b, {0, 0, 0}, {1, 0, 0}, 4502, &w);
          probe("divided, one half opened", b);

          // The same body, opening both halves of the bottom rather than one.
          std::vector<FaceId> fs, bottom;
          b.allFaces(fs);
          for (FaceId f : fs)
            if (dot(b.faceNormal(f), Vec3{0, 0, -1}) > 0.99) bottom.push_back(f);
          Body t = b;
          std::string w2;
          const bool ok = shellBody(t, bottom, 2.0, 4507, &w2);
          std::printf("  %-32s %s %s  (%zu faces opened)\n",
                      "divided, both halves opened", ok ? "hollowed" : "REFUSED",
                      w2.c_str(), bottom.size()); }

        { Body b = makeBox(40, 30, 20);
          const EdgeId up = anUprightEdge(b);
          FilletSpec sp; sp.edges.push_back({up, 3.0});
          filletEdges(b, sp, &w);
          probe("one vertical edge rounded", b); }

        { Body b = makeBox(40, 30, 20);
          divideBody(b, {0, 0, 0}, {1, 0, 0}, 4503, &w);
          extrudeFaces(b, {facing(b, {0, 0, 1})}, 6.0, nullptr, 4504,
                       ExtrudeOp::Auto, &w, true);
          probe("divided, then half pushed up", b); }

        { Body b = makeBox(40, 30, 20);
          divideBody(b, {0, 0, 0}, {1, 0, 0}, 4505, &w);
          extrudeFaces(b, {facing(b, {0, 0, 1})}, 6.0, nullptr, 4506,
                       ExtrudeOp::Auto, &w, true);
          const EdgeId up = anUprightEdge(b);
          if (up != kInvalid) { FilletSpec sp; sp.edges.push_back({up, 3.0});
                                filletEdges(b, sp, &w); }
          probe("both, then rounded", b); }
    }

    std::printf("--- what a face can be asked to turn about ---\n");
    {
        auto turn = [&](const char* what, Body b, Vec3 dir, int pick) {
            const FaceId f = facing(b, dir);
            std::vector<EdgeId> es;
            b.faceEdges(f, es);
            if (es.empty()) { std::printf("  %-32s no edges\n", what); return false; }
            const EdgeId hinge = es[static_cast<size_t>(pick) % es.size()];
            Vec3 p, q;
            b.edgePositions(hinge, p, q);
            std::string why;
            Body t = b;
            const bool ok = rotateFaces(t, {f}, radians(8.0), p, normalize(q - p),
                                        4601, &why);
            std::printf("  %-32s %s %s\n", what, ok ? "turned" : "REFUSED", why.c_str());
            return ok;
        };
        std::string w;

        check(turn("a plain box, first edge", makeBox(40, 30, 20), {0, 1, 0}, 0),
              "a plain box turns");
        check(turn("a plain box, second edge", makeBox(40, 30, 20), {0, 1, 0}, 1),
              "about any of its edges");

        { Body b = makeBox(40, 30, 20);
          divideBody(b, {0, 0, 0}, {1, 0, 0}, 4602, &w);
          turn("half of a divided face", b, {0, 1, 0}, 0); }

        { Body b = makeBox(40, 30, 20);
          std::vector<EdgeId> es;
          b.allEdges(es);
          EdgeId up = kInvalid;
          for (EdgeId e : es) {
              Vec3 p, q;
              b.edgePositions(e, p, q);
              if (std::fabs((q - p).z) > 5.0) { up = e; break; }
          }
          FilletSpec sp; sp.edges.push_back({up, 3.0});
          filletEdges(b, sp, &w);
          turn("a face next to a round", b, {0, 1, 0}, 0); }
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
