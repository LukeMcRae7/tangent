// The sketch: that it solves to what its dimensions say, counts what is still
// free, names the constraints that disagree, and finds the regions it bounds.
//
// Everything is checked against numbers worked out by hand. A rectangle 40 by
// 25 has a corner at (40, 25) and an area of 1000, and a sketch that merely
// looks like a rectangle has neither.
#include "sketch/sketch.h"
#include "geom/body.h"
#include "geom/brep.h"
#include "scene/scene.h"
#include "scene/serialize.h"
#include "temp_path.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near(Real a, Real b, Real eps = 1e-7) { return std::fabs(a - b) < eps; }

static bool contains(const std::vector<SketchId>& v, SketchId id) {
    for (SketchId x : v) if (x == id) return true;
    return false;
}

// The corner opposite the fixed one, whatever order the lines were added in.
static Vec2 farCorner(const Sketch& sk, const Sketch::Rectangle& r) {
    return sk.point(sk.entity(r.right)->b)->at;
}

int main() {
    std::printf("--- a rectangle solves to its dimensions ---\n");
    {
        Sketch sk;
        const Sketch::Rectangle r = sk.addRectangle({0, 0}, 40, 25);
        // Drawn roughly, as a hand would: nothing is where the dimensions say,
        // so the solver has to move every free corner to get there.
        for (SketchPoint& p : sk.points)
            if (p.at.x > 0 || p.at.y > 0) p.at = {p.at.x * 0.9 + 1.3, p.at.y * 1.1 - 0.7};

        const SketchSolve s = solveSketch(sk);
        check(s.solved, "it solves: " + s.reason);
        check(s.freedoms == 0, "and nothing is left free: " + std::to_string(s.freedoms));
        const Vec2 c = farCorner(sk, r);
        check(near(c.x, 40) && near(c.y, 25), "the far corner is at 40, 25");
        std::printf("  far corner at %.9f, %.9f\n", c.x, c.y);
    }

    std::printf("--- freedoms are counted, and an open sketch still solves ---\n");
    {
        // Four lines at right angles on shared corners and nothing else: the
        // rectangle can still move (2) and resize (2).
        Sketch sk;
        const SketchId p0 = sk.addPoint({0, 0}), p1 = sk.addPoint({10, 0.2});
        const SketchId p2 = sk.addPoint({10.3, 6}), p3 = sk.addPoint({-0.1, 6.1});
        const SketchId bottom = sk.addLine(p0, p1), right = sk.addLine(p1, p2);
        const SketchId top = sk.addLine(p2, p3), left = sk.addLine(p3, p0);
        sk.constrain(SketchRule::Horizontal, bottom);
        sk.constrain(SketchRule::Horizontal, top);
        sk.constrain(SketchRule::Vertical, left);
        sk.constrain(SketchRule::Vertical, right);

        const SketchSolve s = solveSketch(sk);
        check(s.freedoms == 4, "four right angles leave 4 freedoms: " + std::to_string(s.freedoms));
        check(s.solved, "and it is still a definite shape");
    }

    std::printf("--- constraints that disagree are named ---\n");
    {
        Sketch sk;
        const Sketch::Rectangle r = sk.addRectangle({0, 0}, 40, 25);
        const SketchEntity* top = sk.entity(r.top);
        const SketchId narrower = sk.constrain(SketchRule::Distance, top->a, top->b, 30);

        const SketchSolve s = solveSketch(sk);
        check(!s.solved, "40 wide at the bottom and 30 at the top cannot both hold");
        check(contains(s.conflicting, r.width) || contains(s.conflicting, narrower),
              "and the diagnosis names one of the two widths");
        check(!s.reason.empty(), "with a reason: " + s.reason);
        std::printf("  %s\n", s.reason.c_str());
    }

    std::printf("--- a dimension changes without the sketch being described again ---\n");
    {
        Sketch sk;
        const Sketch::Rectangle r = sk.addRectangle({0, 0}, 40, 25);
        SketchSolver solver(sk);
        check(solver.solve().solved, "solves at 40 by 25");

        check(solver.setDimension(r.width, 55), "the width is a dimension it can change");
        check(solver.solve().solved, "and solves again");
        const Vec2 c = farCorner(sk, r);
        check(near(c.x, 55) && near(c.y, 25), "to 55 by 25");
        check(near(sk.constraint(r.width)->value, 55), "and the sketch records the new width");
        check(!solver.setDimension(r.bottom, 3), "a line is not a dimension");
    }

    std::printf("--- what is still free, and what is pinned down ---\n");
    {
        Sketch sk;
        const Sketch::Rectangle r = sk.addRectangle({0, 0}, 40, 25);
        const SketchSolve s = solveSketch(sk);
        check(s.solved && s.freedoms == 0, "a rectangle with both dimensions is fully constrained");
        check(s.freeEntities.empty() && s.freePoints.empty(), "so nothing in it is free");

        // One line more, hanging off a corner, free at its far end.
        const SketchId loose = sk.addPoint({60, 40});
        const SketchId spur = sk.addLine(sk.entity(r.right)->b, loose);
        const SketchSolve s2 = solveSketch(sk);
        check(s2.solved && s2.freedoms == 2, "an added line brings two freedoms with it");
        check(s2.freePoints.size() == 1 && s2.freePoints[0] == loose,
              "the end nothing holds is the free one");
        check(s2.freeEntities.size() == 1 && s2.freeEntities[0] == spur,
              "and the line on it is the free geometry");

        sk.constrain(SketchRule::Fix, loose, kNoSketchId, 60, 40);
        const SketchSolve s3 = solveSketch(sk);
        check(s3.freedoms == 0 && s3.freeEntities.empty(), "fixing that end pins the lot");
    }

    std::printf("--- dragging, with the constraints holding ---\n");
    {
        // A line held level, its left end fixed: dragging the right end can
        // change how long it is and nothing else.
        Sketch sk;
        const SketchId a = sk.addPoint({0, 0}), b = sk.addPoint({20, 0});
        const SketchId line = sk.addLine(a, b);
        sk.constrain(SketchRule::Fix, a, kNoSketchId, 0, 0);
        sk.constrain(SketchRule::Horizontal, line);

        SketchSolver solver(sk);
        check(solver.solve().solved, "it solves to start with");
        check(solver.beginDrag(b), "a point can be taken hold of");
        check(solver.dragging(), "and is held");
        check(solver.dragTo({35, 18}), "dragging it up and to the right moves the sketch");
        check(near(sk.point(b)->at.x, 35.0, 1e-6), "it follows the pointer along the line");
        check(near(sk.point(b)->at.y, 0.0, 1e-6), "and not off it: level is still level");
        check(near(sk.point(a)->at.x, 0.0, 1e-9) && near(sk.point(a)->at.y, 0.0, 1e-9),
              "the fixed end did not move");
        solver.endDrag();
        check(!solver.dragging(), "and it can be let go of");
        const SketchSolve after = solver.solve();
        check(after.solved, "the sketch still solves afterwards: " + after.reason);
    }
    {
        // Fully constrained: a drag is a question the sketch answers with no.
        Sketch sk;
        const Sketch::Rectangle r = sk.addRectangle({0, 0}, 40, 25);
        const SketchId corner = sk.entity(r.right)->b;
        const Vec2 was = sk.point(corner)->at;
        SketchSolver solver(sk);
        solver.solve();
        check(solver.beginDrag(corner), "a corner of a fully constrained rectangle can be grabbed");
        solver.dragTo({80, 80});
        check(near(sk.point(corner)->at.x, was.x, 1e-9) &&
                  near(sk.point(corner)->at.y, was.y, 1e-9),
              "but it does not move: every dimension still holds");
        solver.endDrag();
        check(near(sk.constraint(r.width)->value, 40.0), "and no dimension was quietly rewritten");
    }
    {
        // An arc has no radius dimension of its own, so its rim can be pulled.
        Sketch sk;
        const SketchId c = sk.addPoint({0, 0});
        const SketchId s0 = sk.addPoint({10, 0}), e0 = sk.addPoint({0, 10});
        const SketchId arc = sk.addArc(c, s0, e0);
        sk.constrain(SketchRule::Fix, c, kNoSketchId, 0, 0);
        SketchSolver solver(sk);
        check(solver.solve().solved, "an arc solves");
        check(solver.beginRadiusDrag(arc), "its rim can be taken hold of");
        check(solver.dragRadiusTo(16.0), "and pulled out");
        check(near(sk.entity(arc)->radius, 16.0, 1e-4), "the arc takes the new radius");
        check(near(length(sk.point(s0)->at), 16.0, 1e-4) &&
                  near(length(sk.point(e0)->at), 16.0, 1e-4),
              "and both its ends stay on it");
        solver.endDrag();
    }

    std::printf("--- a drag stays inside a frame ---\n");
    {
        // Performance here is part of the specification, not a hope: the solver
        // runs on every mouse move, so a drag on a sketch larger than anything
        // a person would draw by hand still has to fit in a frame.
        Sketch sk;
        SketchId prev = sk.addPoint({0, 0});
        const SketchId first = prev;
        sk.constrain(SketchRule::Fix, prev, kNoSketchId, 0, 0);
        for (int i = 1; i <= 60; ++i) {
            const Real a = kTwoPi * i / 61.0;
            const SketchId next = sk.addPoint({40.0 * std::cos(a), 40.0 * std::sin(a)});
            sk.addLine(prev, next);
            prev = next;
        }
        sk.addLine(prev, first);

        SketchSolver solver(sk);
        const SketchSolve s = solver.solve();
        check(s.solved, "a 61-sided ring of lines solves: " + s.reason);
        check(solver.beginDrag(prev), "one of its corners can be dragged");

        constexpr int kFrames = 200;
        const auto start = std::chrono::steady_clock::now();
        int moved = 0;
        for (int i = 0; i < kFrames; ++i) {
            const Real t = static_cast<Real>(i) / kFrames;
            if (solver.dragTo({40.0 + 6.0 * t, 6.0 * t})) ++moved;
        }
        const double ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - start).count() / kFrames;
        solver.endDrag();
        check(moved == kFrames, "every drag frame solved");
        std::printf("    %d entities, %.3f ms a drag frame\n",
                    static_cast<int>(sk.entities.size()), ms);
        check(ms < 16.0, "and each one fits in a 60 Hz frame: " + std::to_string(ms) + " ms");
    }

    std::printf("--- a line tangent to a circle ---\n");
    {
        Sketch sk;
        const SketchId centre = sk.addPoint({0, 0});
        const SketchId circle = sk.addCircle(centre, 9.0);
        const SketchId a = sk.addPoint({-20, 18}), b = sk.addPoint({20, 15});
        const SketchId line = sk.addLine(a, b);
        sk.constrain(SketchRule::Fix, centre, kNoSketchId, 0, 0);
        sk.constrain(SketchRule::Radius, circle, kNoSketchId, 10);
        sk.constrain(SketchRule::Horizontal, line);
        // Given circle first, to check the order a person says it in does not matter.
        sk.constrain(SketchRule::Tangent, circle, line);

        const SketchSolve s = solveSketch(sk);
        check(s.solved, "solves: " + s.reason);
        check(near(std::fabs(sk.point(a)->at.y), 10.0, 1e-6) &&
                  near(sk.point(a)->at.y, sk.point(b)->at.y, 1e-6),
              "the line sits one radius off the centre");
        check(near(sk.entity(circle)->radius, 10.0, 1e-9), "and the circle took its radius");
    }

    std::printf("--- the regions a sketch bounds ---\n");
    {
        Sketch sk;
        sk.addRectangle({0, 0}, 40, 25);
        const std::vector<SketchProfile> p = sketchProfiles(sk);
        check(p.size() == 1, "a rectangle is one region");
        if (!p.empty()) {
            check(p[0].outer.entities.size() == 4, "bounded by its four lines");
            check(p[0].holes.empty(), "with nothing cut out of it");
            check(near(p[0].area, 1000.0, 1e-9), "and an area of exactly 1000");
            check(p[0].outer.signedArea > 0, "wound counter-clockwise in the plane");
        }
    }
    {
        // A hole: the circle lies inside the rectangle, so it is cut out of the
        // plate -- and the disc it bounds is a region too, for a boss.
        Sketch sk;
        const Sketch::Rectangle plate = sk.addRectangle({0, 0}, 40, 25);
        const SketchId disc = sk.addCircle(sk.addPoint({20, 12.5}), 5);
        const std::vector<SketchProfile> p = sketchProfiles(sk);
        check(p.size() == 2, "a rectangle with a circle inside it is a plate and a disc");
        if (p.size() == 2) {
            check(p[0].key == plate.bottom && p[1].key == disc, "keyed by their own entities");
            check(p[1].holes.empty() && std::fabs(p[1].area - kPi * 25.0) < kPi * 25.0 * 0.005,
                  "the disc whole");
        }
        if (!p.empty()) {
            check(p[0].holes.size() == 1, "with one hole");
            // The area of a sampled loop is a polygon's, so it is checked to
            // within the sampling rather than exactly. The solid swept from it
            // is where the exact number is checked.
            const Real exact = 1000.0 - kPi * 25.0;
            check(std::fabs(p[0].area - exact) < exact * 0.005,
                  "and the hole's area taken away: " + std::to_string(p[0].area));
        }
    }
    {
        Sketch sk;
        const Sketch::Rectangle a = sk.addRectangle({0, 0}, 10, 10);
        const Sketch::Rectangle b = sk.addRectangle({30, 0}, 10, 10);
        const std::vector<SketchProfile> p = sketchProfiles(sk);
        check(p.size() == 2, "two separate rectangles are two regions");
        if (p.size() == 2) {
            check(p[0].key == a.bottom && p[1].key == b.bottom,
                  "each keyed by its own lowest id, in order");
        }
    }
    {
        // An open chain bounds nothing.
        Sketch sk;
        const SketchId p0 = sk.addPoint({0, 0}), p1 = sk.addPoint({10, 0});
        const SketchId p2 = sk.addPoint({10, 10}), p3 = sk.addPoint({0, 10});
        sk.addLine(p0, p1);
        sk.addLine(p1, p2);
        sk.addLine(p2, p3);
        check(sketchProfiles(sk).empty(), "three sides of a square bound no region");
    }
    {
        // Closed by constraints rather than by shared corners: eight separate
        // points joined in pairs. It is the same square either way.
        Sketch sk;
        const Vec2 c[4] = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
        SketchId start[4], end[4];
        for (int i = 0; i < 4; ++i) {
            start[i] = sk.addPoint(c[i]);
            end[i] = sk.addPoint(c[(i + 1) % 4]);
            sk.addLine(start[i], end[i]);
        }
        for (int i = 0; i < 4; ++i) sk.constrain(SketchRule::Coincident, end[i], start[(i + 1) % 4]);
        const std::vector<SketchProfile> p = sketchProfiles(sk);
        check(p.size() == 1 && near(p[0].area, 100.0, 1e-9),
              "corners made coincident by constraint close the same as shared ones");
    }
    {
        // Construction geometry constrains and does not bound anything.
        Sketch sk;
        const Sketch::Rectangle r = sk.addRectangle({0, 0}, 20, 10);
        const SketchId diag = sk.addLine(sk.entity(r.bottom)->a, sk.entity(r.top)->a);
        sk.entity(diag)->construction = true;
        const std::vector<SketchProfile> p = sketchProfiles(sk);
        check(p.size() == 1 && p[0].outer.entities.size() == 4,
              "a construction diagonal leaves the rectangle one region of four lines");
    }
    {
        // A branch. The rectangle is still visibly there, but one corner joins
        // three lines, and the rule is to leave a component that does not close
        // cleanly out entirely rather than choose a loop out of it. Stated here
        // so that relaxing it is a decision rather than an accident.
        Sketch sk;
        const Sketch::Rectangle r = sk.addRectangle({0, 0}, 20, 10);
        sk.addLine(sk.entity(r.right)->b, sk.addPoint({30, 20}));
        check(sketchProfiles(sk).empty(), "a line branching off a corner leaves no region, by rule");
    }
    {
        // A slot: two lines and two half-circles, with the arcs' ends shared
        // with the lines' -- the shape a sagitta-per-span profile gets wrong.
        Sketch sk;
        const SketchId lc = sk.addPoint({0, 0}), rc = sk.addPoint({30, 0});
        const SketchId p0 = sk.addPoint({0, -5}), p1 = sk.addPoint({30, -5});
        const SketchId p2 = sk.addPoint({30, 5}), p3 = sk.addPoint({0, 5});
        sk.addLine(p0, p1);
        sk.addArc(rc, p1, p2);   // counter-clockwise, round the right end
        sk.addLine(p2, p3);
        sk.addArc(lc, p3, p0);   // and round the left
        const std::vector<SketchProfile> p = sketchProfiles(sk);
        check(p.size() == 1, "a slot is one region");
        if (!p.empty()) {
            const Real exact = 30.0 * 10.0 + kPi * 25.0;
            check(p[0].outer.entities.size() == 4, "of two lines and two arcs");
            check(std::fabs(p[0].area - exact) < exact * 0.005,
                  "with a rectangle and a circle's area: " + std::to_string(p[0].area));
        }
    }
    {
        Sketch sk;
        const SketchId a = sk.addPoint({0, 0}), b = sk.addPoint({20, 0});
        const SketchId c1 = sk.addPoint({15, 12}), c2 = sk.addPoint({5, 12});
        sk.addLine(a, b);
        sk.addBezier(b, c1, c2, a);
        check(sketchProfiles(sk).size() == 1, "a line closed by a curve is a region");
    }

    std::printf("--- a sketch of its own, in the scene and through a file ---\n");
    {
        // An object that is nothing but a sketch. It has no body, and adding it,
        // re-running its history, saving it and opening it again all have to be
        // fine with that -- a sketch is a thing in the model before anything is
        // built from it. None of this needs the exact kernel.
        Scene scene;
        Sketch plan;
        plan.addRectangle({0, 0}, 20, 12);
        Feature s;
        s.kind = FeatureKind::Sketch;
        s.uid = scene.takeFeatureUid();
        s.sketch = plan;
        s.sketchShown = false;   // hidden, to check the flag survives the file

        std::string why;
        const ObjectId id = scene.addFeatureChain({s}, "Sketch", &why);
        check(id != kNoObject, "a chain of nothing but a sketch makes an object: " + why);
        if (id != kNoObject) {
            check(scene.find(id)->body.empty(), "with no body");
            check(scene.reevaluate(id), "and a history that re-runs");
        }

        const std::string path = tempPath("sketch_only.tng");
        check(saveProject(scene, path).ok, "saved");
        Scene back;
        const ProjectResult loaded = loadProject(back, path);
        check(loaded.ok, "loaded: " + loaded.error);
        if (loaded.ok && back.objectCount() == 1) {
            const SceneObject* o = back.objects().front().get();
            check(o->features.size() == 1 && o->features[0].kind == FeatureKind::Sketch,
                  "the sketch came back");
            check(o->features[0].sketch.entities.size() == 4, "with its four lines");
            check(!o->features[0].sketchShown, "and hidden, as it was saved");
            check(o->body.empty(), "still with no body");
        } else {
            check(false, "one object came back");
        }
    }

    if (!brep::available()) {
        std::printf("\n[sketch] exact kernel not built; sweeping is not tested\n");
    } else {
        // What a sweep produced: its volume, its faces, and how many are round.
        struct Swept {
            Body body;
            Real volume = 0;
            int faces = 0, cylinders = 0;
            bool solid = false;
        };
        auto sweep = [](Sketch& sk, SketchId key, Real from, Real to, ElementId salt) {
            Swept out;
            for (const SketchProfile& p : sketchProfiles(sk)) {
                if (p.key != key) continue;
                std::string why;
                BrepRef s = brep::sketchSolid(sk, p, from, to, salt, &why);
                if (!s) {
                    std::printf("  sweep refused: %s\n", why.c_str());
                    return out;
                }
                out.body = Body(std::move(s));
            }
            if (out.body.empty()) return out;
            out.volume = out.body.health(false).volume;
            out.solid = out.body.health().solid();
            std::vector<FaceId> fs;
            out.body.allFaces(fs);
            out.faces = static_cast<int>(fs.size());
            for (FaceId f : fs)
                if (out.body.faceKind(f) == SurfaceKind::Cylinder) ++out.cylinders;
            return out;
        };

        std::printf("--- a profile swept into a solid ---\n");
        {
            Sketch sk;
            const Sketch::Rectangle r = sk.addRectangle({0, 0}, 40, 25);
            solveSketch(sk);
            const Swept s = sweep(sk, r.bottom, 0, 10, 1);
            check(s.solid, "a rectangle sweeps into a solid");
            check(near(s.volume, 10000.0, 1e-6), "of exactly 40 x 25 x 10: " + std::to_string(s.volume));
            check(s.faces == 6, "with six faces");
        }
        {
            // The exact number the profile's sampled area could only approximate.
            Sketch sk;
            const Sketch::Rectangle r = sk.addRectangle({0, 0}, 40, 25);
            sk.addCircle(sk.addPoint({20, 12.5}), 5);
            solveSketch(sk);
            const Swept s = sweep(sk, r.bottom, 0, 10, 1);
            const Real exact = (1000.0 - kPi * 25.0) * 10.0;
            check(s.solid, "a rectangle with a hole sweeps into a solid");
            check(near(s.volume, exact, 1e-6),
                  "with the hole's exact volume taken away: " + std::to_string(s.volume));
            check(s.cylinders == 1, "and the bore is one cylindrical face");
            check(s.faces == 7, "four walls, a bore and two caps: " + std::to_string(s.faces));
        }
        {
            // The case the create tool got wrong: a circle drawn as four
            // quarter-arcs swept into four faces. A sketch circle is one circle.
            Sketch sk;
            const SketchId circle = sk.addCircle(sk.addPoint({0, 0}), 10);
            solveSketch(sk);
            const Swept s = sweep(sk, circle, 0, 25, 1);
            check(near(s.volume, kPi * 100.0 * 25.0, 1e-6),
                  "a circle sweeps to a cylinder's exact volume: " + std::to_string(s.volume));
            check(s.faces == 3 && s.cylinders == 1, "one wall and two caps, not four quarters");
        }
        {
            Sketch sk;
            const SketchId lc = sk.addPoint({0, 0}), rc = sk.addPoint({30, 0});
            const SketchId p0 = sk.addPoint({0, -5}), p1 = sk.addPoint({30, -5});
            const SketchId p2 = sk.addPoint({30, 5}), p3 = sk.addPoint({0, 5});
            const SketchId bottom = sk.addLine(p0, p1);
            sk.addArc(rc, p1, p2);
            sk.addLine(p2, p3);
            sk.addArc(lc, p3, p0);
            const Swept s = sweep(sk, bottom, 0, 6, 1);
            const Real exact = (300.0 + kPi * 25.0) * 6.0;
            check(s.solid, "a slot sweeps into a solid");
            check(near(s.volume, exact, 1e-6), "of a rectangle and a circle, exactly: " +
                                                   std::to_string(s.volume));
            check(s.faces == 6 && s.cylinders == 2, "two flat walls, two round ends, two caps");
        }
        {
            // Off the XY plane. The sketch's own x is world x and its y is world
            // z, so its normal -- x cross y -- points down world y.
            Sketch sk;
            sk.plane.xAxis = {1, 0, 0};
            sk.plane.yAxis = {0, 0, 1};
            const Sketch::Rectangle r = sk.addRectangle({0, 0}, 20, 10);
            solveSketch(sk);
            const Swept s = sweep(sk, r.bottom, 0, 4, 1);
            const AABB b = s.body.bounds();
            check(near(s.volume, 800.0, 1e-6), "on the XZ plane it is still 20 x 10 x 4");
            check(near(b.max.x - b.min.x, 20, 1e-6) && near(b.max.z - b.min.z, 10, 1e-6) &&
                      near(b.max.y - b.min.y, 4, 1e-6),
                  "lying in x and z, and swept along y");
            check(b.max.y < 1e-6, "towards negative y, which is the way its normal points");
        }
        {
            Sketch sk;
            const SketchId a = sk.addPoint({0, 0}), b = sk.addPoint({20, 0});
            const SketchId bottom = sk.addLine(a, b);
            sk.addBezier(b, sk.addPoint({15, 12}), sk.addPoint({5, 12}), a);
            const Swept s = sweep(sk, bottom, 0, 3, 1);
            check(s.solid && s.volume > 0, "a curve closing a line sweeps into a solid");
        }
        {
            Sketch sk;
            sk.addRectangle({0, 0}, 10, 10);
            solveSketch(sk);
            std::string why;
            const std::vector<SketchProfile> p = sketchProfiles(sk);
            check(!brep::sketchSolid(sk, p.front(), 5, 5, 1, &why), "a sweep with no depth is refused");
            check(!why.empty(), "and says why: " + why);
        }

        std::printf("--- a dimension changes, and nothing is renamed ---\n");
        {
            // The reason a sketch is in the history at all. Widen the rectangle
            // and every face keeps its name, so anything built on one of them --
            // a fillet on the right-hand edge -- still finds it.
            Sketch sk;
            const Sketch::Rectangle r = sk.addRectangle({0, 0}, 40, 25);
            SketchSolver solver(sk);
            solver.solve();
            const Swept before = sweep(sk, r.bottom, 0, 10, 77);

            auto namesOf = [](const Body& b) {
                std::vector<FaceId> fs;
                b.allFaces(fs);
                std::vector<ElementId> ids;
                for (FaceId f : fs) ids.push_back(b.faceName(f));
                std::sort(ids.begin(), ids.end());
                return ids;
            };
            const ElementId rightWall = nameId(77, IdRole::Side, r.right);

            solver.setDimension(r.width, 55);
            solver.solve();
            const Swept after = sweep(sk, r.bottom, 0, 10, 77);

            check(near(after.volume, 13750.0, 1e-6), "it is 55 wide now: " + std::to_string(after.volume));
            check(namesOf(before.body) == namesOf(after.body), "and every face has the name it had");
            const FaceId right = after.body.findFace(rightWall);
            check(right != kNoFace, "the wall swept from the right-hand line is still called that");
            if (right != kNoFace)
                check(near(after.body.faceCentroid(right).x, 55.0, 1e-6),
                      "and it is the wall that moved to x = 55");
        }

        // ---- In the history -------------------------------------------------
        auto sketchStep = [](ElementId uid, const Sketch& sk) {
            Feature f;
            f.kind = FeatureKind::Sketch;
            f.uid = uid;
            f.sketch = sk;
            return f;
        };
        auto extrudeStep = [](ElementId uid, ElementId sketchUid, SketchId key, Real distance,
                              ExtrudeOp op = ExtrudeOp::Auto) {
            Feature f;
            f.kind = FeatureKind::ExtrudeProfile;
            f.uid = uid;
            f.sketchUid = sketchUid;
            f.profileKey = key;
            f.distance = distance;
            f.extrudeOp = op;
            return f;
        };

        std::printf("--- a part that begins as a sketch ---\n");
        {
            Sketch plan;
            const Sketch::Rectangle r = plan.addRectangle({0, 0}, 40, 25);
            std::vector<Feature> chain{sketchStep(10, plan), extrudeStep(20, 10, r.bottom, 10)};
            Body body;
            check(evaluateFeatures(chain, body), "a sketch and an extrusion make a body with no primitive");
            check(!chain[0].errored && !chain[1].errored, "neither step failed");
            check(chain[0].sketchFreedoms == 0, "and the sketch reports itself fully constrained");
            check(near(body.health(false).volume, 10000.0, 1e-6), "of exactly 40 x 25 x 10");

            const ElementId rightWall = nameId(20, IdRole::Side, r.right);
            check(body.findFace(rightWall) != kNoFace, "its right wall is named for the right-hand line");

            // Change the dimension in the history, as the History panel does, and
            // run the chain again.
            chain[0].sketch.constraint(r.width)->value = 55;
            Body wider;
            check(evaluateFeatures(chain, wider), "re-runs after the width changes");
            check(near(wider.health(false).volume, 13750.0, 1e-6), "and the part is 55 wide");
            const FaceId right = wider.findFace(rightWall);
            check(right != kNoFace && near(wider.faceCentroid(right).x, 55.0, 1e-6),
                  "with the same right wall, moved rather than renamed");
        }

        std::printf("--- a second sketch cuts a hole ---\n");
        {
            Sketch plan;
            const Sketch::Rectangle r = plan.addRectangle({0, 0}, 40, 40);
            // On the top face: 20 up, with the same axes, so its normal is +z and
            // a negative distance goes down into the block.
            Sketch top;
            top.plane.origin = {0, 0, 20};
            const SketchId bore = top.addCircle(top.addPoint({20, 20}), 5);
            top.constrain(SketchRule::Radius, bore, kNoSketchId, 5);

            std::vector<Feature> chain{sketchStep(1, plan), extrudeStep(2, 1, r.bottom, 20),
                                       sketchStep(3, top), extrudeStep(4, 3, bore, -8)};
            Body body;
            check(evaluateFeatures(chain, body), "the chain evaluates");
            for (const Feature& f : chain)
                check(!f.errored, std::string(featureKindName(f.kind)) + " did not fail: " + f.error);
            const Real exact = 40.0 * 40.0 * 20.0 - kPi * 25.0 * 8.0;
            check(near(body.health(false).volume, exact, 1e-6),
                  "a blind hole 8 deep, exactly: " + std::to_string(body.health(false).volume));
            check(body.health().solid(), "and it is still a solid");
            check(body.findFace(nameId(2, IdRole::Side, r.right)) != kNoFace,
                  "the first sketch's walls keep their names through the cut");
        }

        std::printf("--- a region stays itself when the sketch grows ---\n");
        {
            // A circle added beside the rectangle is a second region. Keyed by
            // index, the extrusion might now pick it; keyed by the rectangle's
            // lowest id, it cannot.
            Sketch plan;
            const Sketch::Rectangle r = plan.addRectangle({0, 0}, 40, 25);
            std::vector<Feature> chain{sketchStep(1, plan), extrudeStep(2, 1, r.bottom, 10)};
            chain[0].sketch.addCircle(chain[0].sketch.addPoint({-50, -50}), 3);
            Body body;
            check(evaluateFeatures(chain, body), "evaluates with a second region in the sketch");
            check(near(body.health(false).volume, 10000.0, 1e-6),
                  "and still sweeps the rectangle, not the circle");
        }

        std::printf("--- a sketch that will not solve is the step that fails ---\n");
        {
            Sketch plan;
            const Sketch::Rectangle r = plan.addRectangle({0, 0}, 40, 25);
            const SketchEntity* topLine = plan.entity(r.top);
            plan.constrain(SketchRule::Distance, topLine->a, topLine->b, 30);
            std::vector<Feature> chain{sketchStep(1, plan), extrudeStep(2, 1, r.bottom, 10)};
            Body body;
            evaluateFeatures(chain, body);
            check(chain[0].errored, "the sketch is marked as failing");
            check(chain[0].error.find("cannot all hold") != std::string::npos,
                  "saying its constraints disagree: " + chain[0].error);
            check(chain[1].errored && chain[1].error.find("does not solve") != std::string::npos,
                  "and the extrusion says why it has nothing to sweep: " + chain[1].error);
        }
        {
            Sketch plan;
            const Sketch::Rectangle r = plan.addRectangle({0, 0}, 10, 10);
            std::vector<Feature> chain{sketchStep(1, plan),
                                       extrudeStep(2, 1, r.bottom, 5, ExtrudeOp::Cut)};
            Body body;
            evaluateFeatures(chain, body);
            check(chain[1].errored, "a cut with no body to cut from is refused");
            check(!chain[1].error.empty(), "with a reason: " + chain[1].error);
        }

        std::printf("--- through a file and back ---\n");
        {
            Scene scene;
            const ObjectId id = scene.addPrimitive(PrimitiveKind::Box);
            SceneObject* obj = scene.find(id);
            const ElementId s1 = scene.takeFeatureUid(), e1 = scene.takeFeatureUid();
            const ElementId s2 = scene.takeFeatureUid(), e2 = scene.takeFeatureUid();

            Sketch plan;
            const Sketch::Rectangle r = plan.addRectangle({0, 0}, 30, 30);
            Sketch top;
            top.plane.origin = {0, 0, 12};
            const SketchId bore = top.addCircle(top.addPoint({15, 15}), 4);
            top.constrain(SketchRule::Radius, bore, kNoSketchId, 4);
            // Through, and past the bottom, so the cut does not end exactly on
            // the block's lower face -- this test is about the file, not about
            // coplanar booleans.
            obj->features = {sketchStep(s1, plan), extrudeStep(e1, s1, r.bottom, 12),
                             sketchStep(s2, top), extrudeStep(e2, s2, bore, -15)};
            check(scene.reevaluate(id), "the scene evaluates the sketched part");
            const Real volume = scene.find(id)->body.health(false).volume;
            check(near(volume, 30.0 * 30.0 * 12.0 - kPi * 16.0 * 12.0, 1e-6),
                  "a block with a hole straight through it");

            const std::string path = tempPath("sketch.tng");
            check(saveProject(scene, path).ok, "saved");
            Scene back;
            const ProjectResult loaded = loadProject(back, path);
            check(loaded.ok, "loaded: " + loaded.error);
            if (loaded.ok && back.objectCount() == 1) {
                const SceneObject* o = back.objects().front().get();
                check(o->features.size() == 4, "all four steps came back");
                check(o->features[0].kind == FeatureKind::Sketch &&
                          o->features[3].kind == FeatureKind::ExtrudeProfile,
                      "as sketches and extrusions");
                check(o->features[0].sketch.entities.size() == 4 &&
                          o->features[0].sketch.constraints.size() == plan.constraints.size(),
                      "with every entity and constraint");
                check(near(o->features[0].sketch.constraint(r.width)->value, 30.0),
                      "and the dimensions they were saved with");
                check(o->features[3].sketchUid == s2 && o->features[3].profileKey == bore,
                      "each extrusion still pointing at its sketch and region");
                check(near(o->body.health(false).volume, volume, 1e-6),
                      "re-evaluating to the same part");
            }
            std::remove(path.c_str());
            std::printf("[sketch] a sketched part round-trips at version %u\n", kProjectVersion);
        }
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
