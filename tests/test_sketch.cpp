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
#include "scene/sketch_project.h"
#include "temp_path.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <set>
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

        std::printf("--- a profile turned about an axis ---\n");
        {
            // A ring: a 10 x 10 square standing 20 mm off the axis, turned all
            // the way round. Pappus gives the volume without the kernel having
            // any say in it -- the area times the circle its centroid travels.
            Sketch sk;
            sk.addRectangle({20, 0}, 10, 10);
            solveSketch(sk);
            const std::vector<SketchProfile> rs = sketchProfiles(sk);
            check(rs.size() == 1, "the square is one region");
            std::string why;
            BrepRef ring = brep::revolveSketch(sk, rs, {rs.front().key}, {0, 0}, {0, 1},
                                               2.0 * kPi, 11, &why);
            check(ring != nullptr, "a square off the axis turns into a ring: " + why);
            if (ring) {
                const Body b(std::move(ring));
                check(b.health().solid(), "and it is a solid");
                check(near(b.health(false).volume, 2.0 * kPi * 25.0 * 100.0, 1.0),
                      "with the volume Pappus says: " +
                          std::to_string(b.health(false).volume));
                std::vector<FaceId> fs;
                b.allFaces(fs);
                check(fs.size() == 4, "four faces: two round, two flat");
                int cyl = 0;
                for (FaceId f : fs)
                    if (b.faceKind(f) == SurfaceKind::Cylinder) ++cyl;
                check(cyl == 2, "the inside and the outside are cylinders");
                std::set<ElementId> names;
                for (FaceId f : fs) names.insert(b.faceName(f));
                check(names.size() == fs.size() && !names.count(0),
                      "every face has a name of its own");
            }
        }
        {
            // A part turn is that ring's quarter, and the profile itself stands
            // at each end of it.
            Sketch sk;
            sk.addRectangle({20, 0}, 10, 10);
            solveSketch(sk);
            const std::vector<SketchProfile> rs = sketchProfiles(sk);
            std::string why;
            BrepRef part = brep::revolveSketch(sk, rs, {rs.front().key}, {0, 0}, {0, 1},
                                               kPi * 0.5, 12, &why);
            check(part != nullptr, "a quarter turn builds: " + why);
            if (part) {
                const Body b(std::move(part));
                check(near(b.health(false).volume, 2.0 * kPi * 25.0 * 100.0 * 0.25, 1.0),
                      "a quarter of the volume: " + std::to_string(b.health(false).volume));
                std::vector<FaceId> fs;
                b.allFaces(fs);
                check(fs.size() == 6, "six faces: the four walls and the two ends");
            }
        }
        {
            // Touching the axis is not crossing it: this is how a half-disc
            // becomes a sphere, and the test the refusal below must not catch.
            Sketch sk;
            const SketchId c = sk.addPoint({0, 0});
            const SketchId top = sk.addPoint({0, 10}), bottom = sk.addPoint({0, -10});
            sk.addArc(c, bottom, top);          // counter-clockwise, out through +x
            sk.addLine(top, bottom);
            solveSketch(sk);
            const std::vector<SketchProfile> rs = sketchProfiles(sk);
            check(rs.size() == 1, "the half-disc is one region");
            std::string why;
            BrepRef ball = rs.empty() ? BrepRef{}
                                      : brep::revolveSketch(sk, rs, {rs.front().key}, {0, 0},
                                                            {0, 1}, 2.0 * kPi, 13, &why);
            check(ball != nullptr, "a half-disc on the axis turns into a ball: " + why);
            if (ball) {
                const Body b(std::move(ball));
                check(near(b.health(false).volume, 4.0 / 3.0 * kPi * 1000.0, 1.0),
                      "with a sphere's volume: " + std::to_string(b.health(false).volume));
            }
        }
        {
            // Straddling the axis would turn the profile through itself. The
            // ends of this square are on both sides of x = 0.
            Sketch sk;
            sk.addRectangle({-5, 0}, 10, 10);
            solveSketch(sk);
            const std::vector<SketchProfile> rs = sketchProfiles(sk);
            std::string why;
            check(!brep::revolveSketch(sk, rs, {rs.front().key}, {0, 0}, {0, 1}, 2.0 * kPi, 14, &why),
                  "a profile across the axis is refused");
            check(why.find("crosses the axis") != std::string::npos, "and says why: " + why);

            why.clear();
            check(!brep::revolveSketch(sk, rs, {rs.front().key}, {-20, 0}, {0, 1}, 0.0, 15, &why),
                  "and a turn of no angle is refused");
            check(!why.empty(), "with a reason: " + why);
        }

        std::printf("--- a path, joined end to end ---\n");
        // Standing up through the origin: x across, z up. Where the paths below
        // are drawn, so they leave the top plane a profile is drawn on.
        auto standing = [] {
            Sketch s;
            s.plane.xAxis = {1, 0, 0};
            s.plane.yAxis = {0, 0, 1};
            return s;
        };
        {
            Sketch s = standing();
            const SketchId a = s.addPoint({0, 0}), b = s.addPoint({0, 20}), c = s.addPoint({15, 20});
            const SketchId up = s.addLine(a, b), over = s.addLine(b, c);
            SketchPath p;
            std::string why;
            check(sketchPathOf(s, {over, up}, p, &why), "two lines meeting make a path: " + why);
            check(p.entities.size() == 2 && !p.closed, "an open one of two curves");
            const std::vector<Vec2> pts = sketchPathPoints(s, p);
            check(pts.size() == 3, "three points along it, the joint once");
            check(near(length(pts.front() - pts.back()), std::sqrt(15.0 * 15.0 + 20.0 * 20.0)),
                  "running from one end to the other");

            SketchPath through;
            check(sketchPathThrough(s, over, through, &why) && through.entities.size() == 2,
                  "clicking either line picks the whole path");

            // A third line off the corner is a branch no sweep can follow.
            const SketchId d = s.addPoint({-10, 20});
            const SketchId spur = s.addLine(b, d);
            check(!sketchPathOf(s, {up, over, spur}, p, &why), "three lines at one point are refused");
            check(why.find("branches") != std::string::npos, "as a branch: " + why);
            check(sketchPathThrough(s, up, through) && through.entities.size() == 1,
                  "and clicking one stops at the branch");

            const SketchId far1 = s.addPoint({40, 0}), far2 = s.addPoint({40, 10});
            const SketchId apart = s.addLine(far1, far2);
            check(!sketchPathOf(s, {up, apart}, p, &why), "two lines that do not meet are refused");
        }

        std::printf("--- a profile carried along a path ---\n");
        auto square = [](Real side) {
            Sketch sk;
            sk.addRectangle({-side / 2, -side / 2}, side, side);
            solveSketch(sk);
            return sk;
        };
        {
            // Straight up: a prism by another name, 4 x 4 x 30.
            const Sketch sk = square(4);
            const std::vector<SketchProfile> rs = sketchProfiles(sk);
            Sketch s = standing();
            const SketchId line = s.addLine(s.addPoint({0, 0}), s.addPoint({0, 30}));
            std::string why;
            BrepRef made = brep::sweepSketch(sk, rs, {rs.front().key}, s, {line}, 31, &why);
            check(made != nullptr, "a square swept up a line builds: " + why);
            if (made) {
                const Body b(std::move(made));
                check(b.health().solid(), "a solid");
                check(near(b.health(false).volume, 16.0 * 30.0, 1e-4),
                      "of 4 x 4 x 30: " + std::to_string(b.health(false).volume));
                std::vector<FaceId> fs;
                b.allFaces(fs);
                check(fs.size() == 6, "with six faces");
                std::set<ElementId> names;
                for (FaceId f : fs) names.insert(b.faceName(f));
                check(names.size() == fs.size() && !names.count(0), "every one named on its own");
                check(b.findFace(nameId(31, IdRole::Cap, 0)) != kNoFace &&
                          b.findFace(nameId(31, IdRole::Cap, 1)) != kNoFace,
                      "the two ends are the caps");
            }

            // Drawn from the top down it is the same path, and the square is
            // carried from the end it sits at rather than from the far one.
            Sketch down = standing();
            const SketchId back = down.addLine(down.addPoint({0, 30}), down.addPoint({0, 0}));
            BrepRef same = brep::sweepSketch(sk, rs, {rs.front().key}, down, {back}, 32, &why);
            check(same != nullptr, "the path drawn the other way builds too: " + why);
            if (same) {
                const Body b(std::move(same));
                const AABB box = b.bounds();
                check(near(box.min.z, 0.0, 1e-6) && near(box.max.z, 30.0, 1e-6),
                      "from the profile up, not from the top of the path");
            }
        }
        {
            // Round a corner: 20 up, then 15 across. A mitred corner on a
            // section symmetric about the bend adds outside what it takes
            // inside, so the volume is the area times the centreline's length.
            const Sketch sk = square(4);
            const std::vector<SketchProfile> rs = sketchProfiles(sk);
            Sketch s = standing();
            const SketchId b = s.addPoint({0, 20});
            const SketchId up = s.addLine(s.addPoint({0, 0}), b);
            const SketchId over = s.addLine(b, s.addPoint({15, 20}));
            std::string why;
            BrepRef made = brep::sweepSketch(sk, rs, {rs.front().key}, s, {up, over}, 33, &why);
            check(made != nullptr, "a square round a corner builds: " + why);
            if (made) {
                const Body body(std::move(made));
                check(body.health().solid(), "a solid");
                check(near(body.health(false).volume, 16.0 * 35.0, 1e-3),
                      "of the area times 35 along the middle: " +
                          std::to_string(body.health(false).volume));
            }
        }
        {
            // A round bar bent a quarter of the way round a 20 mm radius:
            // Pappus again, pi r^2 times the quarter circle the centre travels.
            Sketch sk;
            sk.addCircle(sk.addPoint({20, 0}), 2);
            solveSketch(sk);
            const std::vector<SketchProfile> rs = sketchProfiles(sk);
            Sketch s = standing();
            const SketchId arc = s.addArc(s.addPoint({0, 0}), s.addPoint({20, 0}), s.addPoint({0, 20}));
            std::string why;
            BrepRef made = brep::sweepSketch(sk, rs, {rs.front().key}, s, {arc}, 34, &why);
            check(made != nullptr, "a circle round an arc builds: " + why);
            if (made) {
                const Body b(std::move(made));
                check(b.health().solid(), "a solid");
                const Real exact = kPi * 4.0 * (20.0 * kPi / 2.0);
                check(near(b.health(false).volume, exact, 1e-3),
                      "of the volume Pappus says: " + std::to_string(b.health(false).volume) +
                          " against " + std::to_string(exact));
            }
        }
        {
            // A square tube: the bore is carried along with the outline and
            // taken out of it.
            Sketch sk;
            sk.addRectangle({-5, -5}, 10, 10);
            sk.addCircle(sk.addPoint({0, 0}), 2);
            solveSketch(sk);
            const std::vector<SketchProfile> rs = sketchProfiles(sk);
            const std::vector<SketchId> filled = sketchFilledProfiles(sk, rs);
            Sketch s = standing();
            const SketchId line = s.addLine(s.addPoint({0, 0}), s.addPoint({0, 10}));
            std::string why;
            BrepRef made = brep::sweepSketch(sk, rs, filled, s, {line}, 35, &why);
            check(made != nullptr, "a holed square swept up a line builds: " + why);
            if (made) {
                const Body b(std::move(made));
                check(b.health().solid(), "a solid");
                check(near(b.health(false).volume, (100.0 - kPi * 4.0) * 10.0, 1e-3),
                      "with the bore gone all the way: " + std::to_string(b.health(false).volume));
            }
        }
        {
            // A path lying in the profile's own plane would carry it edgeways.
            const Sketch sk = square(4);
            const std::vector<SketchProfile> rs = sketchProfiles(sk);
            Sketch flat;
            const SketchId line = flat.addLine(flat.addPoint({0, 0}), flat.addPoint({30, 0}));
            std::string why;
            check(!brep::sweepSketch(sk, rs, {rs.front().key}, flat, {line}, 36, &why),
                  "a path along the profile's plane is refused");
            check(why.find("along the profile's plane") != std::string::npos, "and says so: " + why);
        }

        std::printf("--- built from a body's own faces and edges ---\n");
        {
            PrimitiveSpec spec;
            spec.kind = PrimitiveKind::Box;
            spec.box = {10, 10, 10};
            const Body box(brep::primitive(spec));
            const AABB bb = box.bounds();
            std::vector<FaceId> fs;
            box.allFaces(fs);
            auto faceFacing = [&](Vec3 n) {
                for (FaceId f : fs)
                    if (dot(normalize(box.faceNormal(f)), n) > 0.999) return f;
                return kNoFace;
            };
            const FaceId side = faceFacing({1, 0, 0}), top = faceFacing({0, 0, 1});
            check(side != kNoFace && top != kNoFace, "the box has a side and a top");

            // The side face turned a quarter of the way round its own upright
            // edge: a quarter cylinder, of the face's width in radius.
            std::string why;
            const Vec3 hinge{bb.max.x, bb.min.y, bb.min.z};
            BrepRef turned = brep::revolveOutline({nullptr, nullptr, {}, &box.brep(), {side}}, hinge, {0, 0, 1},
                                                  kPi * 0.5, 51, &why);
            check(turned != nullptr, "a box's face turns about its own edge: " + why);
            if (turned) {
                const Body b(std::move(turned));
                check(b.health().solid(), "a solid");
                check(near(b.health(false).volume, kPi * 100.0 * 10.0 / 4.0, 1e-3),
                      "a quarter cylinder: " + std::to_string(b.health(false).volume));
                std::set<ElementId> names;
                std::vector<FaceId> tf;
                b.allFaces(tf);
                for (FaceId f : tf) names.insert(b.faceName(f));
                check(names.size() == tf.size(), "every face named on its own");
            }

            // The top face swept down one of the box's own upright edges: the
            // same box again, swept from the end of the edge the face is at.
            EdgeId upright = kInvalid;
            std::vector<EdgeId> es;
            box.allEdges(es);
            for (EdgeId e : es) {
                Vec3 a, c;
                box.edgePositions(e, a, c);
                if (std::fabs(a.x - c.x) < 1e-9 && std::fabs(a.y - c.y) < 1e-9) { upright = e; break; }
            }
            BrepRef swept = brep::sweepOutline({nullptr, nullptr, {}, &box.brep(), {top}},
                                               {nullptr, {}, &box.brep(), {upright}}, 52, &why);
            check(swept != nullptr, "a box's top swept down its own edge: " + why);
            if (swept)
                check(near(Body(std::move(swept)).health(false).volume, 1000.0, 1e-3),
                      "makes the box's volume again");

            // The top face lofted to a 5 mm square standing 10 above it.
            Sketch above;
            above.plane.origin = {(bb.min.x + bb.max.x) / 2, (bb.min.y + bb.max.y) / 2, bb.max.z + 10};
            above.addRectangle({-2.5, -2.5}, 5, 5);
            solveSketch(above);
            const std::vector<SketchProfile> ra = sketchProfiles(above);
            BrepRef lofted = brep::loftOutlines({{nullptr, nullptr, {}, &box.brep(), {top}},
                                                 {&above, &ra, {ra.front().key}, nullptr, {}}},
                                                true, 53, &why);
            check(lofted != nullptr, "a box's top lofts to a sketch above it: " + why);
            if (lofted)
                check(near(Body(std::move(lofted)).health(false).volume, 10.0 / 3.0 * 175.0, 1e-3),
                      "as a frustum");

            // A face that is not flat is refused as a profile.
            spec.kind = PrimitiveKind::Cylinder;
            spec.cylinder = {20, 10, 64};
            const Body can(brep::primitive(spec));
            std::vector<FaceId> cf;
            can.allFaces(cf);
            FaceId round = kNoFace, capTop = kNoFace;
            for (FaceId f : cf) {
                if (can.faceKind(f) == SurfaceKind::Cylinder) round = f;
                else if (normalize(can.faceNormal(f)).z > 0.999) capTop = f;
            }
            check(!brep::revolveOutline({nullptr, nullptr, {}, &can.brep(), {round}}, {0, 0, 0}, {0, 0, 1},
                                        kPi, 54, &why),
                  "a round face is refused as a profile");
            check(why.find("flat") != std::string::npos, "and says so: " + why);

            // A small circle carried round the rim of the can's top: a torus,
            // 2 pi^2 R r^2 -- the path a closed edge of a body.
            std::vector<EdgeId> rim;
            can.faceEdges(capTop, rim);
            const AABB cb = can.bounds();
            Sketch ring;
            ring.plane.xAxis = {1, 0, 0};
            ring.plane.yAxis = {0, 0, 1};
            ring.addCircle(ring.addPoint({cb.max.x, cb.max.z}), 2);
            solveSketch(ring);
            const std::vector<SketchProfile> rr = sketchProfiles(ring);
            BrepRef torus = brep::sweepOutline({&ring, &rr, {rr.front().key}, nullptr, {}},
                                               {nullptr, {}, &can.brep(), rim}, 55, &why);
            check(torus != nullptr, "a circle carried round a rim builds: " + why);
            if (torus)
                check(near(Body(std::move(torus)).health(false).volume, 2.0 * kPi * kPi * 20.0 * 4.0, 1e-2),
                      "a torus");
        }

        std::printf("--- outlines lofted into a solid ---\n");
        auto lifted = [](Sketch sk, Real z) {
            sk.plane.origin = {0, 0, z};
            return sk;
        };
        {
            // A 10 square at the bottom and a 5 square 10 up, joined straight:
            // a frustum, h / 3 (A1 + A2 + sqrt(A1 A2)).
            const Sketch bottom = square(10);
            const Sketch top = lifted(square(5), 10);
            const std::vector<SketchProfile> rb = sketchProfiles(bottom), rt = sketchProfiles(top);
            std::string why;
            BrepRef made = brep::loftSketches({{&bottom, &rb.front()}, {&top, &rt.front()}}, true, 41, &why);
            check(made != nullptr, "two squares loft into a frustum: " + why);
            if (made) {
                const Body b(std::move(made));
                check(b.health().solid(), "a solid");
                const Real exact = 10.0 / 3.0 * (100.0 + 25.0 + 50.0);
                check(near(b.health(false).volume, exact, 1e-3),
                      "of the frustum's volume: " + std::to_string(b.health(false).volume));
                std::vector<FaceId> fs;
                b.allFaces(fs);
                check(fs.size() == 6, "six faces: four walls and two ends");
                std::set<ElementId> names;
                for (FaceId f : fs) names.insert(b.faceName(f));
                check(names.size() == fs.size() && !names.count(0), "every one named on its own");
                check(b.findFace(nameId(41, IdRole::Cap, 0)) != kNoFace, "the bottom is the first cap");
            }
        }
        {
            // Two equal circles, smoothly: a cylinder.
            Sketch a;
            a.addCircle(a.addPoint({0, 0}), 5);
            solveSketch(a);
            const Sketch b = lifted(a, 10);
            const std::vector<SketchProfile> ra = sketchProfiles(a), rb = sketchProfiles(b);
            std::string why;
            BrepRef made = brep::loftSketches({{&a, &ra.front()}, {&b, &rb.front()}}, false, 42, &why);
            check(made != nullptr, "two circles loft: " + why);
            if (made)
                check(near(Body(std::move(made)).health(false).volume, kPi * 25.0 * 10.0, 1e-3),
                      "into a cylinder's volume");
        }
        {
            Sketch holed;
            holed.addRectangle({-5, -5}, 10, 10);
            holed.addCircle(holed.addPoint({0, 0}), 2);
            solveSketch(holed);
            const Sketch top = lifted(square(5), 10);
            const std::vector<SketchProfile> rh = sketchProfiles(holed), rt = sketchProfiles(top);
            const SketchProfile* ring = nullptr;
            for (const SketchProfile& p : rh) if (!p.holes.empty()) ring = &p;
            std::string why;
            check(ring && !brep::loftSketches({{&holed, ring}, {&top, &rt.front()}}, true, 43, &why),
                  "a region with a hole is refused");
            check(why.find("hole") != std::string::npos, "and says why: " + why);

            const Sketch same = square(5);
            const std::vector<SketchProfile> rs = sketchProfiles(same);
            why.clear();
            check(!brep::loftSketches({{&same, &rs.front()}, {&same, &rs.front()}}, true, 44, &why),
                  "two outlines on one plane are refused");
            check(why.find("same plane") != std::string::npos, "and say why: " + why);
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
            f.profileKeys = {key};
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

        auto revolveStep = [](ElementId uid, ElementId sketchUid, SketchId key, Vec2 at, Vec2 dir,
                              Real angle, ExtrudeOp op = ExtrudeOp::Auto) {
            Feature f;
            f.kind = FeatureKind::RevolveProfile;
            f.uid = uid;
            f.sketchUid = sketchUid;
            f.profileKeys = {key};
            f.revolveAxisAt = at;
            f.revolveAxisDir = dir;
            f.revolveAngle = angle;
            f.extrudeOp = op;
            return f;
        };

        std::printf("--- a part that begins as a turn ---\n");
        {
            // The same square, turned rather than pushed: a ring, with the
            // volume Pappus gives and walls still named for the lines that
            // swept them.
            Sketch plan;
            const Sketch::Rectangle r = plan.addRectangle({20, 0}, 10, 10);
            std::vector<Feature> chain{sketchStep(10, plan),
                                       revolveStep(20, 10, r.bottom, {0, 0}, {0, 1}, 2.0 * kPi)};
            Body body;
            check(evaluateFeatures(chain, body), "a sketch and a turn make a body: " + chain[1].error);
            check(near(body.health(false).volume, 2.0 * kPi * 25.0 * 100.0, 1e-3),
                  "of the volume the profile sweeps: " + std::to_string(body.health(false).volume));
            check(body.findFace(nameId(20, IdRole::Side, r.right)) != kNoFace,
                  "its outer wall is named for the line that swept it");

            // The angle is a parameter like any other, and the history edits it.
            chain[1].revolveAngle = kPi;
            Body half;
            check(evaluateFeatures(chain, half), "re-runs at half a turn");
            check(near(half.health(false).volume, kPi * 25.0 * 100.0, 1e-3),
                  "for half the material");

            // And a turn whose profile straddles the axis is the step that
            // fails, with a reason, rather than a body that is wrong.
            chain[1].revolveAngle = 2.0 * kPi;
            chain[1].revolveAxisAt = {25, 0};
            Body none;
            evaluateFeatures(chain, none);
            check(chain[1].errored && chain[1].error.find("crosses the axis") != std::string::npos,
                  "a profile across the axis fails the step: " + chain[1].error);
        }

        std::printf("--- a turn cuts a groove ---\n");
        {
            // A ring cut out of a block: the tool crosses the block's own face
            // on purpose, so what comes away is the part inside it.
            Sketch plan;
            // Centred on the axis, so the ring cut out of it is wholly inside.
            const Sketch::Rectangle r = plan.addRectangle({-20, -20}, 40, 40);
            Sketch side;
            // Standing up, through the block: x across, z up.
            side.plane.origin = {0, 0, 0};
            side.plane.xAxis = {1, 0, 0};
            side.plane.yAxis = {0, 0, 1};
            const Sketch::Rectangle g = side.addRectangle({8, 5}, 6, 6);
            std::vector<Feature> chain{sketchStep(1, plan), extrudeStep(2, 1, r.bottom, 20),
                                       sketchStep(3, side),
                                       revolveStep(4, 3, g.bottom, {0, 0}, {0, 1}, 2.0 * kPi,
                                                   ExtrudeOp::Cut)};
            Body body;
            check(evaluateFeatures(chain, body), "the chain evaluates");
            for (const Feature& f : chain)
                check(!f.errored, std::string(featureKindName(f.kind)) + " did not fail: " + f.error);
            // The groove is a ring 8 to 14 out and 6 tall, all of it inside a
            // block 40 square and 20 high: 2 pi * 11 * 36 gone from 32000.
            const Real exact = 40.0 * 40.0 * 20.0 - 2.0 * kPi * 11.0 * 36.0;
            check(near(body.health(false).volume, exact, 1e-3),
                  "and a ring is gone from it: " + std::to_string(body.health(false).volume));
            check(body.health().solid(), "leaving a solid");
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
                check(o->features[3].sketchUid == s2 && o->features[3].profileKeys == std::vector<SketchId>{bore},
                      "each extrusion still pointing at its sketch and region");
                check(near(o->body.health(false).volume, volume, 1e-6),
                      "re-evaluating to the same part");
            }
            std::remove(path.c_str());
            std::printf("[sketch] a sketched part round-trips at version %u\n", kProjectVersion);
        }
        {
            // A turn carries an axis and an angle that nothing else in the
            // file has, so it goes through one of its own.
            Scene scene;
            Sketch plan;
            const Sketch::Rectangle r = plan.addRectangle({20, 0}, 10, 10);
            Feature sk;
            sk.kind = FeatureKind::Sketch;
            sk.uid = 1;
            sk.sketch = plan;
            Feature turn;
            turn.kind = FeatureKind::RevolveProfile;
            turn.uid = 2;
            turn.sketchUid = 1;
            turn.profileKeys = {r.bottom};
            turn.revolveAxisAt = {-3, 1.5};
            turn.revolveAxisDir = {0, 1};
            turn.revolveAngle = kPi * 0.75;
            std::string why;
            const ObjectId id = scene.addFeatureChain({sk, turn}, "Turned", &why);
            check(id != kNoObject, "a turned part is made from a chain: " + why);
            const Real volume = id == kNoObject ? 0.0 : scene.find(id)->body.health(false).volume;

            const std::string path = tempPath("revolve.tng");
            check(saveProject(scene, path).ok, "saved");
            Scene back;
            const ProjectResult loaded = loadProject(back, path);
            check(loaded.ok, "loaded: " + loaded.error);
            if (loaded.ok && back.objectCount() == 1) {
                const SceneObject* o = back.objects().front().get();
                check(o->features.size() == 2 &&
                          o->features[1].kind == FeatureKind::RevolveProfile,
                      "the turn came back as a turn");
                check(near(o->features[1].revolveAngle, kPi * 0.75) &&
                          near(o->features[1].revolveAxisAt.x, -3.0) &&
                          near(o->features[1].revolveAxisAt.y, 1.5),
                      "with the axis and the angle it was saved with");
                check(near(o->body.health(false).volume, volume, 1e-6),
                      "and it re-evaluates to the same part");
            }
            std::remove(path.c_str());
        }

        std::printf("--- a revolve of the part's own face about its own edge ---\n");
        {
            // A 10 mm box's side face turned a quarter round its upright edge.
            // One way the quarter cylinder swings out of the box and adds to
            // it; the other way it swings into the box and adds nothing. The
            // edge has no direction of its own, so Reverse picks which.
            Scene scene;
            PrimitiveSpec spec;
            spec.kind = PrimitiveKind::Box;
            spec.box = {10, 10, 10};
            const ObjectId id = scene.addPrimitive(PrimitiveKind::Box, spec, {0, 0, 0});
            const Body before = scene.find(id)->body;
            const AABB bb = before.bounds();
            std::vector<FaceId> fs;
            before.allFaces(fs);
            FaceId side = kNoFace;
            for (FaceId f : fs)
                if (normalize(before.faceNormal(f)).x > 0.999) side = f;
            std::vector<EdgeId> es;
            before.allEdges(es);
            EdgeId hinge = kInvalid;
            for (EdgeId e : es) {
                Vec3 a, b;
                before.edgePositions(e, a, b);
                if (std::fabs(a.x - bb.max.x) < 1e-9 && std::fabs(b.x - bb.max.x) < 1e-9 &&
                    std::fabs(a.y - bb.min.y) < 1e-9 && std::fabs(b.y - bb.min.y) < 1e-9)
                    hinge = e;
            }
            Feature turn;
            turn.kind = FeatureKind::RevolveProfile;
            turn.uid = scene.takeFeatureUid();
            turn.faces = nameFaces(before, {side});
            turn.edges = nameEdges(before, {hinge}, false);
            turn.revolveAngle = kPi * 0.5;
            turn.extrudeOp = ExtrudeOp::Join;
            std::string why;
            check(side != kNoFace && hinge != kInvalid, "the box has the face and the edge");
            check(scene.addFeature(id, turn, &why), "a revolve of its own face goes in its history: " + why);
            const Real one = scene.find(id)->body.health(false).volume;

            std::vector<Feature> chain = scene.find(id)->features;
            chain.back().revolveReverse = true;
            check(scene.setFeatures(id, chain, &why), "and turned the other way: " + why);
            const Real other = scene.find(id)->body.health(false).volume;
            const Real out = 1000.0 + kPi * 100.0 * 10.0 / 4.0;
            check((near(one, out, 1e-3) && near(other, 1000.0, 1e-3)) ||
                      (near(one, 1000.0, 1e-3) && near(other, out, 1e-3)),
                  "one way adds a quarter cylinder, the other nothing: " + std::to_string(one) + ", " +
                      std::to_string(other));

            // Make it the way that adds, and send it through a file.
            if (near(other, 1000.0, 1e-3)) {
                chain.back().revolveReverse = false;
                scene.setFeatures(id, chain, &why);
            }
            const std::string file = tempPath("face_revolve.tng");
            check(saveProject(scene, file).ok, "saved");
            Scene back;
            const ProjectResult loaded = loadProject(back, file);
            check(loaded.ok, "loaded: " + loaded.error);
            if (loaded.ok && back.objectCount() == 1) {
                const Feature& f = back.objects().front()->features.back();
                check(f.kind == FeatureKind::RevolveProfile && f.sketchUid == 0 && !f.faces.empty() &&
                          !f.edges.empty(),
                      "the revolve came back naming the face and the edge");
                check(near(back.objects().front()->body.health(false).volume, out, 1e-3),
                      "and re-evaluates to the same part");
            }
            std::remove(file.c_str());
        }

        std::printf("--- a smooth join and a projection through a file ---\n");
        {
            Sketch sk;
            const SketchId a = sk.addPoint({0, 0}), h1 = sk.addPoint({5, 0}), h2 = sk.addPoint({10, 5});
            const SketchId m = sk.addPoint({15, 5}), h3 = sk.addPoint({20, 5}), h4 = sk.addPoint({25, 0});
            const SketchId e = sk.addPoint({30, 0});
            const SketchId c1 = sk.addBezier(a, h1, h2, m);
            const SketchId c2 = sk.addBezier(m, h3, h4, e);
            sk.constrain(SketchRule::Smooth, c1, c2);
            check(solveSketch(sk).solved, "a smooth join solves");
            // And a box's top face projected into the same sketch, following it.
            Scene scene;
            PrimitiveSpec spec;
            spec.kind = PrimitiveKind::Box;
            spec.box = {10, 10, 10};
            const ObjectId id = scene.addPrimitive(PrimitiveKind::Box, spec, {0, 0, 0});
            const Body& box = scene.find(id)->body;
            std::vector<FaceId> fs;
            box.allFaces(fs);
            FaceId top = kNoFace;
            for (FaceId f : fs)
                if (normalize(box.faceNormal(f)).z > 0.999) top = f;
            sk.plane.origin = {0, 0, box.bounds().max.z};
            std::string why;
            check(projectFace(sk, box, Mat4::identity(), top, true, nullptr, &why), "the top face projects: " + why);
            const SketchId linked = sk.entities.back().id;
            const uint64_t edgeName = sk.entities.back().source;
            Feature step = sketchStep(scene.takeFeatureUid(), sk);
            check(scene.addFeature(id, step, &why), "into the box's history: " + why);
            const std::string file = tempPath("smooth.tng");
            check(saveProject(scene, file).ok, "saved");
            Scene back;
            const ProjectResult loaded = loadProject(back, file);
            check(loaded.ok, "loaded: " + loaded.error);
            if (loaded.ok && back.objectCount() == 1) {
                const Feature& f2 = back.objects().front()->features.back();
                const Sketch& s2 = f2.sketch;
                check(std::any_of(s2.constraints.begin(), s2.constraints.end(),
                                  [](const SketchConstraint& k) { return k.rule == SketchRule::Smooth; }),
                      "the smooth join came back");
                check(edgeName != 0 && s2.entity(linked) && s2.entity(linked)->source == edgeName &&
                          s2.entity(c1)->source == 0,
                      "and so did which edge a projected line follows");
                check(!f2.errored, "and the sketch finds that edge again: " + f2.error);
            }
            std::remove(file.c_str());
        }

        std::printf("--- a sweep and a loft in the history ---\n");
        {
            // A 4 mm square up a path 30 long, then the path lengthened in its
            // own sketch: the sweep follows it, and keeps its faces' names.
            Sketch plan;
            plan.addRectangle({-2, -2}, 4, 4);
            Sketch path = standing();
            const SketchId top = path.addPoint({0, 30});
            const SketchId line = path.addLine(path.addPoint({0, 0}), top);
            Feature sweep;
            sweep.kind = FeatureKind::SweepProfile;
            sweep.uid = 3;
            sweep.sketchUid = 1;
            sweep.profileKeys = {sketchProfiles(plan).front().key};
            sweep.pathSketchUid = 2;
            sweep.pathEntities = {line};
            std::vector<Feature> chain{sketchStep(1, plan), sketchStep(2, path), sweep};
            Body body;
            check(evaluateFeatures(chain, body), "a sketch, a path and a sweep make a body: " + chain[2].error);
            check(near(body.health(false).volume, 480.0, 1e-4), "of 4 x 4 x 30");
            const FaceId end = body.findFace(nameId(3, IdRole::Cap, 1));
            check(end != kNoFace && near(body.faceCentroid(end).z, 30.0, 1e-6), "its far end is at the top");

            chain[1].sketch.point(top)->at = {0, 45};
            Body longer;
            check(evaluateFeatures(chain, longer), "re-runs with the path longer");
            check(near(longer.health(false).volume, 16.0 * 45.0, 1e-4), "and the part follows it");
            const FaceId end2 = longer.findFace(nameId(3, IdRole::Cap, 1));
            check(end2 != kNoFace && near(longer.faceCentroid(end2).z, 45.0, 1e-6),
                  "with the same far end, moved rather than renamed");

            // A path that is no longer earlier in the history fails the step.
            std::vector<Feature> missing{sketchStep(1, plan), sweep};
            Body none;
            evaluateFeatures(missing, none);
            check(missing[1].errored && missing[1].error.find("path") != std::string::npos,
                  "a sweep without its path fails, saying so: " + missing[1].error);

            // Through a file and back.
            Scene scene;
            std::string why;
            const ObjectId id = scene.addFeatureChain({sketchStep(1, plan), sketchStep(2, path), sweep},
                                                      "Swept", &why);
            check(id != kNoObject, "a swept part is made from a chain: " + why);
            const std::string file = tempPath("sweep.tng");
            check(saveProject(scene, file).ok, "saved");
            Scene back;
            const ProjectResult loaded = loadProject(back, file);
            check(loaded.ok, "loaded: " + loaded.error);
            if (loaded.ok && back.objectCount() == 1) {
                const Feature& f = back.objects().front()->features.back();
                check(f.kind == FeatureKind::SweepProfile && f.pathSketchUid == 2 &&
                          f.pathEntities == std::vector<SketchId>{line},
                      "the sweep came back with its path");
                check(near(back.objects().front()->body.health(false).volume, 480.0, 1e-4),
                      "and re-evaluates to the same part");
            }
            std::remove(file.c_str());
        }
        {
            // A 10 square lofted to a 5 square 10 up, straight: the frustum.
            Sketch bottom;
            bottom.addRectangle({-5, -5}, 10, 10);
            Sketch top;
            top.plane.origin = {0, 0, 10};
            top.addRectangle({-2.5, -2.5}, 5, 5);
            Feature loft;
            loft.kind = FeatureKind::LoftProfile;
            loft.uid = 3;
            loft.sketchUid = 1;
            loft.profileKeys = {sketchProfiles(bottom).front().key};
            loft.loftSketchUids = {2};
            loft.loftKeys = {sketchProfiles(top).front().key};
            loft.loftRuled = true;
            std::vector<Feature> chain{sketchStep(1, bottom), sketchStep(2, top), loft};
            Body body;
            check(evaluateFeatures(chain, body), "two sketches and a loft make a body: " + chain[2].error);
            check(near(body.health(false).volume, 10.0 / 3.0 * 175.0, 1e-3),
                  "of the frustum's volume: " + std::to_string(body.health(false).volume));

            // Raised in its own sketch, the top takes the loft with it.
            chain[1].sketch.plane.origin.z = 20;
            Body taller;
            check(evaluateFeatures(chain, taller), "re-runs with the top outline higher");
            check(near(taller.health(false).volume, 20.0 / 3.0 * 175.0, 1e-3), "twice the height, twice the volume");

            const std::string file = tempPath("loft.tng");
            Scene scene;
            std::string why;
            const ObjectId id = scene.addFeatureChain(chain, "Lofted", &why);
            check(id != kNoObject, "a lofted part is made from a chain: " + why);
            check(saveProject(scene, file).ok, "saved");
            Scene back;
            const ProjectResult loaded = loadProject(back, file);
            check(loaded.ok, "loaded: " + loaded.error);
            if (loaded.ok && back.objectCount() == 1) {
                const Feature& f = back.objects().front()->features.back();
                check(f.kind == FeatureKind::LoftProfile && f.loftRuled &&
                          f.loftSketchUids == std::vector<ElementId>{2} && f.loftKeys == loft.loftKeys,
                      "the loft came back with its outlines");
                check(near(back.objects().front()->body.health(false).volume, 20.0 / 3.0 * 175.0, 1e-3),
                      "and re-evaluates to the same part");
            }
            std::remove(file.c_str());
        }
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
