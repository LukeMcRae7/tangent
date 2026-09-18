// Tangent - the sketch tool, driven the way the pointer drives it.
//
// Every drawing call here is what a click at that place on the plane does:
// `clickAt` resolves to an existing point the way a click on one would, and the
// shapes go through the same path the viewport's clicks take. The extruding half
// runs on the exact kernel and is checked against the volume it has to produce.
#include "app/sketch_tool.h"
#include "app/undo.h"
#include "geom/brep.h"
#include "scene/scene.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

using namespace tg;

namespace {

int gFailures = 0;
int gChecks = 0;

void check(bool ok, const std::string& what) {
    ++gChecks;
    if (!ok) {
        std::printf("  [FAIL] %s\n", what.c_str());
        ++gFailures;
    }
}

bool near(double a, double b, double eps = 1e-6) { return std::fabs(a - b) <= eps; }

PlaneFrame topPlane(Real z = 0.0) {
    PlaneFrame f;
    f.origin = {0, 0, z};
    f.u = {1, 0, 0};
    f.v = {0, 1, 0};
    f.normal = {0, 0, 1};
    return f;
}

size_t countRule(const Sketch& sk, SketchRule rule) {
    return static_cast<size_t>(std::count_if(sk.constraints.begin(), sk.constraints.end(),
        [rule](const SketchConstraint& k) { return k.rule == rule; }));
}

const SketchConstraint* findRule(const Sketch& sk, SketchRule rule, size_t nth = 0) {
    for (const SketchConstraint& k : sk.constraints)
        if (k.rule == rule && nth-- == 0) return &k;
    return nullptr;
}

Real maxX(const Sketch& sk) {
    Real m = -1e300;
    for (const SketchPoint& p : sk.points) m = std::max(m, p.at.x);
    return m;
}

void drawRectangle(SketchTool& tool, Vec2 a, Vec2 b) {
    tool.setMode(SketchMode::Rectangle);
    tool.clickAt(a);
    tool.clickAt(b);
}

void drawCircle(SketchTool& tool, Vec2 centre, Real r) {
    tool.setMode(SketchMode::Circle);
    tool.clickAt(centre);
    tool.clickAt(centre + Vec2{r, 0});
}

} // namespace

void testDrawing() {
    std::printf("--- drawing ---\n");

    {
        SketchTool tool;
        tool.start();
        check(tool.stage() == SketchStage::SelectPlane, "a new sketch asks for its plane first");
        tool.setPlane(topPlane(), kNoObject, nullptr);
        check(tool.stage() == SketchStage::Draw, "and then draws on it");

        drawRectangle(tool, {0, 0}, {40, 25});
        const Sketch& sk = tool.sketch();
        check(sk.entities.size() == 4 && sk.points.size() == 4, "a rectangle is four lines on four corners");
        check(tool.regions().size() == 1 && near(tool.regions()[0].area, 1000.0),
              "which close one region of 40 x 25");
        check(tool.solveState().solved && tool.solveState().freedoms == 0,
              "held level, plumb, sized and placed: fully constrained");
        check(countRule(sk, SketchRule::Distance) == 2, "with its width and height as dimensions");
    }

    {
        SketchTool tool;
        tool.start();
        tool.setPlane(topPlane(), kNoObject, nullptr);
        tool.setMode(SketchMode::Line);
        tool.clickAt({0, 0});
        check(tool.pending() && tool.sketch().entities.empty(), "one click is a start, not yet a line");
        tool.clickAt({30, 0.4});
        check(tool.sketch().entities.size() == 1, "two clicks are a line");
        const SketchPoint* end = tool.sketch().point(tool.sketch().entities[0].b);
        check(end && near(end->at.y, 0.0), "drawn within a degree of level, it is made level");
        check(countRule(tool.sketch(), SketchRule::Horizontal) == 1, "and held that way");
        check(tool.pending(), "the chain goes on from where the line ended");

        tool.clickAt({0, 30});
        tool.clickAt({0.1, 0.1});   // on the first corner
        const Sketch& sk = tool.sketch();
        check(sk.entities.size() == 3 && sk.points.size() == 3,
              "ending on the first corner shares that corner rather than doubling it");
        check(!tool.pending(), "closing the loop ends the chain");
        check(countRule(sk, SketchRule::Vertical) == 1, "the closing line, plumb, is held plumb");
        check(tool.regions().size() == 1 && near(tool.regions()[0].area, 450.0),
              "a closed triangle is a region");
        check(tool.solveState().freedoms > 0, "with nothing sizing it, it is still free");
    }

    {
        SketchTool tool;
        tool.start();
        tool.setPlane(topPlane(), kNoObject, nullptr);
        drawCircle(tool, {10, 10}, 5);
        const Sketch& sk = tool.sketch();
        check(sk.entities.size() == 1 && sk.entities[0].curve == SketchCurve::Circle,
              "a centre and a rim are a circle");
        const SketchConstraint* r = findRule(sk, SketchRule::Radius);
        check(r && near(r->value, 5.0), "carrying its radius");
        check(tool.solveState().freedoms == 0, "and, its new centre held, fully constrained");
        check(tool.regions().size() == 1, "a circle is a region on its own");

        // Inside it, a smaller one: a ring.
        drawCircle(tool, {10, 10}, 2);
        check(tool.regions().size() == 2, "a circle inside a circle makes a ring and a disc");
        check(tool.sketch().points.size() == 1, "sharing the centre it was drawn on");
    }

    {
        // A slot: two lines and two arcs sharing their ends.
        SketchTool tool;
        tool.start();
        tool.setPlane(topPlane(), kNoObject, nullptr);
        tool.setMode(SketchMode::Line);
        tool.clickAt({0, 0});
        tool.clickAt({20, 0});
        tool.clearPending();
        tool.clickAt({20, 10});
        tool.clickAt({0, 10});
        tool.clearPending();
        tool.setMode(SketchMode::Arc);
        tool.clickAt({20, 5});
        tool.clickAt({20, 0});
        tool.clickAt({20, 10});
        tool.clickAt({0, 5});
        tool.clickAt({0, 10});
        tool.clickAt({0, 0});
        const Sketch& sk = tool.sketch();
        check(sk.entities.size() == 4, "a slot is two lines and two arcs");
        check(sk.points.size() == 6, "on four shared ends and two centres");
        check(tool.regions().size() == 1 &&
                  std::fabs(tool.regions()[0].area - (200.0 + kPi * 25.0)) < 0.5,
              "closing one region of 20 x 10 plus a circle of 5");
    }
}

void testSizes() {
    std::printf("--- sizes ---\n");
    {
        SketchTool tool;
        tool.start();
        tool.setPlane(topPlane(), kNoObject, nullptr);
        tool.setMode(SketchMode::Line);
        tool.clickAt({0, 0});
        check(tool.typeKey('2') && tool.typeKey('5'), "a length can be typed while drawing");
        check(tool.commitPending(), "and Enter commits it");
        const Sketch& sk = tool.sketch();
        const SketchConstraint* d = findRule(sk, SketchRule::Distance);
        check(sk.entities.size() == 1 && d && near(d->value, 25.0),
              "the line carries the typed length as a dimension");
        const SketchPoint* a = sk.point(sk.entities[0].a);
        const SketchPoint* b = sk.point(sk.entities[0].b);
        check(a && b && near(length(b->at - a->at), 25.0, 1e-6), "and is that long");
    }

    {
        SketchTool tool;
        tool.start();
        tool.setPlane(topPlane(), kNoObject, nullptr);
        drawRectangle(tool, {0, 0}, {40, 25});
        const SketchId width = findRule(tool.sketch(), SketchRule::Distance, 0)->id;

        check(tool.setDimension(width, 55.0), "a dimension can be changed");
        check(near(maxX(tool.sketch()), 55.0, 1e-6), "and the rectangle follows it");
        check(near(tool.regions()[0].area, 55.0 * 25.0, 1e-6), "and so does its region");

        check(!tool.setDimension(width, -3.0), "a negative length is refused");
        check(!tool.takeError().empty(), "with a reason");
        check(near(maxX(tool.sketch()), 55.0, 1e-6), "leaving the sketch as it was");

        // The bottom line's length is the width already; asking for it again
        // picks the same number rather than adding a second one.
        tool.setMode(SketchMode::Dimension);
        const SketchId bottom = tool.sketch().entities[0].id;
        check(tool.dimensionEntity(bottom) == width, "dimensioning the bottom finds the width");
        const size_t before = tool.sketch().constraints.size();
        const SketchId top = tool.sketch().entities[2].id;
        check(tool.dimensionEntity(top) == kNoSketchId,
              "the top's length is already decided, so a second dimension is refused");
        check(tool.sketch().constraints.size() == before, "and not left behind");
        check(!tool.takeError().empty(), "with a reason");

        check(tool.undoEdit(), "edits step back");
        check(near(maxX(tool.sketch()), 40.0, 1e-6), "to the width as it was drawn");
    }

    {
        // A triangle dimensioned by hand.
        SketchTool tool;
        tool.start();
        tool.setPlane(topPlane(), kNoObject, nullptr);
        tool.setMode(SketchMode::Line);
        tool.clickAt({0, 0});
        tool.clickAt({30, 0});
        tool.clickAt({10, 20});
        tool.clickAt({0, 0});
        tool.setMode(SketchMode::Dimension);
        const SketchId base = tool.dimensionEntity(tool.sketch().entities[0].id);
        check(base != kNoSketchId, "a line with no size can be given one");
        check(tool.activeDimension() == base, "and that dimension is the one being typed into");
        check(tool.setDimension(base, 45.0), "which then drives it");
        const SketchEntity& e = tool.sketch().entities[0];
        check(near(length(tool.sketch().point(e.b)->at - tool.sketch().point(e.a)->at), 45.0, 1e-6),
              "the base is 45 long");
    }

    {
        SketchTool tool;
        tool.start();
        tool.setPlane(topPlane(), kNoObject, nullptr);
        drawRectangle(tool, {0, 0}, {40, 25});
        const Sketch sk = tool.sketch();
        check(tool.deleteEntity(sk.entities[1].id), "a side can be deleted");
        check(tool.sketch().entities.size() == 3, "leaving three");
        check(tool.sketch().points.size() == 4, "and every corner something still uses");
        check(tool.regions().empty(), "and nothing closed");
        check(countRule(tool.sketch(), SketchRule::Distance) == 2,
              "the width and height still run along sides that remain");
        check(tool.deleteEntity(sk.entities[0].id), "the bottom as well");
        check(countRule(tool.sketch(), SketchRule::Distance) == 1,
              "and the width it carried goes with it");

        check(tool.toggleConstruction(tool.sketch().entities[0].id), "a line can be made construction");
        check(tool.sketch().entities[0].construction, "and is");
    }
}

void testEditMode() {
    std::printf("--- edit mode ---\n");
    {
        // Dragging: what is free moves, and the constraints decide how far.
        SketchTool tool;
        tool.start();
        tool.setPlane(topPlane(), kNoObject, nullptr);
        tool.setMode(SketchMode::Line);
        tool.clickAt({0, 0});
        tool.clickAt({30, 0});
        tool.clickAt({10, 20});
        tool.clickAt({0, 0});

        tool.setMode(SketchMode::Select);
        const SketchId apex = tool.sketch().entities[1].b;
        check(tool.beginDrag(apex), "a point can be dragged");
        check(tool.dragging(), "and the tool says so");
        check(tool.dragTo({14, 26}), "it moves with the pointer");
        const SketchPoint* p = tool.sketch().point(apex);
        check(p && near(p->at.x, 14.0, 1e-4) && near(p->at.y, 26.0, 1e-4),
              "to where the pointer is, nothing holding it back");
        tool.handleMouseUp();
        check(!tool.dragging(), "letting go ends the drag");
        check(tool.regions().size() == 1 && tool.regions()[0].area > 180.0,
              "and the region it bounds is remeasured");

        check(tool.undoEdit(), "one drag is one step back");
        const SketchPoint* back = tool.sketch().point(apex);
        check(back && near(back->at.x, 10.0, 1e-6) && near(back->at.y, 20.0, 1e-6),
              "which puts the point back where it was");

        // A line held level stays level while it is dragged: the constraint is
        // what holds, not the position it happened to be drawn at.
        const SketchEntity base = tool.sketch().entities[0];
        tool.beginDrag(base.b);
        tool.dragTo({45, 12});
        tool.handleMouseUp();
        const SketchPoint* from = tool.sketch().point(base.a);
        const SketchPoint* to = tool.sketch().point(base.b);
        check(from && to && near(from->at.y, to->at.y, 1e-6),
              "a level line is still level after a drag that pulled off it");
        check(to && near(to->at.x, 45.0, 1e-3), "and the end followed the pointer along it");
    }

    {
        // Colours: what has freedom left, and what has none.
        SketchTool tool;
        tool.start();
        tool.setPlane(topPlane(), kNoObject, nullptr);
        drawRectangle(tool, {0, 0}, {40, 25});
        check(tool.solveState().freeEntities.empty(),
              "a dimensioned rectangle has nothing left free");
        tool.setMode(SketchMode::Line);
        tool.clickAt({50, 50});
        tool.clickAt({70, 60});
        tool.clearPending();
        check(tool.solveState().freeEntities.size() == 1,
              "a line drawn with no dimensions is the one free thing");
        check(tool.solveState().freePoints.size() == 2, "on two free points");
    }

    {
        // A conflict cannot be drawn here, but one can be arrived at: the tool
        // refuses the edit and names what disagrees.
        SketchTool tool;
        tool.start();
        tool.setPlane(topPlane(), kNoObject, nullptr);
        drawRectangle(tool, {0, 0}, {40, 25});
        tool.setMode(SketchMode::Dimension);
        check(tool.dimensionEntity(tool.sketch().entities[2].id) == kNoSketchId,
              "a second length for the same side is refused");
        const std::string why = tool.takeError();
        check(why.find("already") != std::string::npos, "saying it is already decided: " + why);
        check(tool.solveState().solved && tool.solveState().conflicting.empty(),
              "and the sketch is left solving");
    }

    {
        // Dragging the rim of a dimensioned circle drives the dimension.
        SketchTool tool;
        tool.start();
        tool.setPlane(topPlane(), kNoObject, nullptr);
        drawCircle(tool, {0, 0}, 5);
        const SketchId circle = tool.sketch().entities[0].id;
        const SketchId radius = findRule(tool.sketch(), SketchRule::Radius)->id;
        tool.setMode(SketchMode::Select);
        check(tool.beginRadiusDrag(circle), "its rim can be taken hold of");
        check(tool.dragTo({12, 0}), "and pulled out");
        tool.handleMouseUp();
        check(near(tool.sketch().entity(circle)->radius, 12.0, 1e-4), "the circle follows");
        check(near(tool.sketch().constraint(radius)->value, 12.0, 1e-4),
              "and its radius dimension is what moved");
        check(tool.solveState().freedoms == 0, "so it is still fully constrained");
    }
}

void testSketchObjects() {
    std::printf("--- a sketch of its own in the scene ---\n");
    Scene scene;
    UndoStack undo;
    Camera camera;

    SketchTool tool;
    tool.start();
    tool.setPlane(topPlane(), kNoObject, nullptr);
    drawRectangle(tool, {0, 0}, {40, 25});
    check(tool.finish(scene, camera, undo, false), "a sketch is kept without extruding it");
    check(scene.objectCount() == 1, "as an object of its own");

    const SceneObject* o = scene.objects().front().get();
    const ObjectId id = o->id;
    check(o->name == "Sketch", "called what it is");
    check(o->features.size() == 1 && o->features[0].kind == FeatureKind::Sketch,
          "holding just the sketch");
    check(o->features[0].sketchShown, "which is drawn, since nothing else stands there");
    check(o->body.empty(), "and it has no body yet");
    check(scene.reevaluate(id), "its history re-runs happily with no solid in it");

    check(undo.undo(scene) && scene.objectCount() == 0, "undo takes it away");
    check(undo.redo(scene) && scene.objectCount() == 1, "redo brings it back");

    if (!brep::available()) {
        std::printf("  exact kernel not built; extruding the sketch is not tested\n");
        return;
    }

    // Later, that sketch becomes a part: re-opened, swept, and the drawing put
    // away now that there is a solid standing where it was.
    const ElementId uid = scene.find(id)->features[0].uid;
    SketchTool again;
    check(again.startEdit(scene, id, uid, camera), "the sketch re-opens from the scene");
    check(again.mode() == SketchMode::Select, "in the tool that moves things");
    check(again.beginExtrude() && again.beginDepth(), "and can be extruded");
    again.setDepth(6.0);
    const bool swept = again.finish(scene, camera, undo, true);
    check(swept, "into the same object: " + again.takeError());

    const SceneObject* part = scene.find(id);
    check(part->features.size() == 2, "the sketch and the sweep");
    check(!part->features[0].sketchShown, "the drawing is put away once a solid stands there");
    check(near(part->body.health(false).volume, 40.0 * 25.0 * 6.0, 1e-6), "40 x 25 x 6");
}

void testExtruding() {
    std::printf("--- extruding ---\n");
    if (!brep::available()) {
        std::printf("  exact kernel not built; extruding is not tested\n");
        return;
    }

    Scene scene;
    UndoStack undo;
    Camera camera;

    ObjectId part = kNoObject;
    {
        SketchTool tool;
        tool.start();
        tool.setPlane(topPlane(), kNoObject, nullptr);
        check(!tool.finish(scene, camera, undo, false),
              "a sketch on a plane of its own is not kept without a region swept from it");
        check(!tool.takeError().empty(), "and says so");

        drawRectangle(tool, {0, 0}, {40, 25});
        check(tool.beginExtrude(), "extruding begins");
        check(tool.chosenRegions().size() == 1, "with the only region already chosen");
        check(tool.beginDepth(), "then the depth");
        tool.setDepth(10.0);
        check(tool.resolvedOp() == CreateOp::NewBody, "out of an origin plane: a new part");
        check(tool.finish(scene, camera, undo, true), "and it is made: " + tool.takeError());
        check(!tool.active(), "which ends the tool");
        check(scene.objectCount() == 1, "one part in the scene");
        const SceneObject* o = scene.objects().front().get();
        part = o->id;
        check(o->features.size() == 2 && o->features[0].kind == FeatureKind::Sketch &&
                  o->features[1].kind == FeatureKind::ExtrudeProfile,
              "its history is the sketch and the extrusion");
        check(near(o->body.health(false).volume, 10000.0, 1e-6), "40 x 25 x 10");

        check(undo.undo(scene) && scene.objectCount() == 0, "undo takes the part away");
        check(undo.redo(scene) && scene.objectCount() == 1, "redo brings it back");
    }

    {
        // A blind hole, drawn on the part's top face.
        SketchTool tool;
        tool.start();
        tool.setPlane(topPlane(10.0), part, nullptr);
        drawCircle(tool, {20, 12.5}, 5);
        check(tool.beginExtrude() && tool.beginDepth(), "the circle is chosen");
        tool.setDepth(-4.0);
        check(tool.resolvedOp() == CreateOp::Cut, "pushed in: a cut");
        check(tool.finish(scene, camera, undo, true), "the hole is cut: " + tool.takeError());
        const SceneObject* o = scene.find(part);
        check(o->features.size() == 4, "into the same part's history");
        check(near(o->body.health(false).volume, 10000.0 - kPi * 25.0 * 4.0, 1e-6),
              "exactly a cylinder of 5 by 4 the less");
    }

    {
        // A through hole, from the origin plane beneath the part: the cut finds
        // what it passes through.
        SketchTool tool;
        tool.start();
        tool.setPlane(topPlane(), kNoObject, nullptr);
        drawCircle(tool, {6, 6}, 2);
        tool.beginExtrude();
        tool.beginDepth();
        tool.setOp(CreateOp::Cut);
        tool.setDepth(12.0);
        check(tool.finish(scene, camera, undo, true), "a cut from a bare plane: " + tool.takeError());
        const SceneObject* o = scene.find(part);
        check(o->features.size() == 6, "lands in the part it passes through");
        check(near(o->body.health(false).volume, 10000.0 - kPi * 100.0 - kPi * 4.0 * 10.0, 1e-6),
              "and goes straight through");
    }

    {
        SketchTool tool;
        tool.start();
        tool.setPlane(topPlane(), kNoObject, nullptr);
        drawCircle(tool, {200, 200}, 2);
        tool.beginExtrude();
        tool.beginDepth();
        tool.setOp(CreateOp::Cut);
        tool.setDepth(5.0);
        check(!tool.finish(scene, camera, undo, true), "a cut through nothing is refused");
        check(tool.takeError() == "Nothing there to cut into", "and says why");
        check(tool.active(), "leaving the sketch open to change");
    }

    {
        // Back into the first sketch, with the part moved: the plane is found
        // where the part now is, and kept in the part's own frame.
        scene.find(part)->transform.position = {100, 0, 0};
        const ElementId first = scene.find(part)->features[0].uid;
        SketchTool tool;
        check(tool.startEdit(scene, part, first, camera), "a sketch in the history re-opens");
        check(tool.editing() && tool.stage() == SketchStage::Draw, "to be drawn on again");
        check(near(tool.plane().origin.x, 100.0), "on its plane where the part now stands");
        check(tool.sketch().entities.size() == 4, "with what was drawn");

        const SketchId width = findRule(tool.sketch(), SketchRule::Distance, 0)->id;
        check(tool.setDimension(width, 50.0), "its width changes");
        check(tool.finish(scene, camera, undo, false), "and the change is kept: " + tool.takeError());
        const SceneObject* o = scene.find(part);
        check(o->features.size() == 6, "in place, not as another step");
        check(near(o->features[0].sketch.plane.origin.x, 0.0),
              "still stored in the part's own frame");
        check(near(o->body.health(false).volume,
                   50.0 * 25.0 * 10.0 - kPi * 100.0 - kPi * 4.0 * 10.0, 1e-6),
              "and everything built from it follows");

        check(undo.undo(scene), "undo");
        check(near(scene.find(part)->body.health(false).volume,
                   10000.0 - kPi * 100.0 - kPi * 4.0 * 10.0, 1e-6),
              "puts the width back");
    }

    {
        const ElementId first = scene.find(part)->features[0].uid;
        SketchTool tool;
        tool.startEdit(scene, part, first, camera);
        check(tool.beginExtrude() && tool.beginDepth(), "an existing sketch can sweep again");
        tool.setOp(CreateOp::NewBody);
        tool.setDepth(5.0);
        check(!tool.finish(scene, camera, undo, true), "but not into a separate part");
        check(!tool.takeError().empty(), "which it says");
    }
}

int main() {
    testDrawing();
    testSizes();
    testEditMode();
    testSketchObjects();
    testExtruding();
    std::printf("[sketch tool] %s (%d checks, %d failures)\n",
                gFailures == 0 ? "ALL PASS" : "FAILED", gChecks, gFailures);
    return gFailures ? 1 : 0;
}
