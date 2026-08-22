// Transform tool and undo/redo. Headless: no GL context is created, and the
// tool's overlay drawing (the only part that touches the renderer) is not
// exercised here.
#include "app/transform_tool.h"
#include "app/undo.h"

#include <cstdio>
#include <string>
#include <vector>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near(float a, float b, float eps = 1e-3f) { return std::fabs(a - b) < eps; }
static bool nearV(Vec3 a, Vec3 b, float eps = 1e-3f) {
    return near(a.x, b.x, eps) && near(a.y, b.y, eps) && near(a.z, b.z, eps);
}

static Camera makeCamera() {
    Camera c;
    c.viewportW = 1000;
    c.viewportH = 800;
    c.distance = 120.0f;
    c.yaw = radians(-35.0f);
    c.pitch = radians(25.0f);
    return c;
}

int main() {
    // ---- Undo stack basics -------------------------------------------------
    {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        UndoStack u;
        check(!u.canUndo() && !u.canRedo(), "stack starts empty");

        const Transform before = s.find(id)->transform;
        Transform after = before;
        after.position = {10, 0, 0};
        s.find(id)->transform = after;

        u.push(std::make_unique<TransformCommand>(
            std::vector<TransformCommand::Entry>{{id, before, after}}, "Move"));
        check(u.canUndo() && !u.canRedo(), "one entry after push");

        u.undo(s);
        check(nearV(s.find(id)->transform.position, before.position), "undo restores position");
        check(u.canRedo(), "redo available after undo");

        u.redo(s);
        check(nearV(s.find(id)->transform.position, after.position), "redo reapplies");

        // A fresh edit must abandon the redo branch.
        u.undo(s);
        u.push(std::make_unique<TransformCommand>(
            std::vector<TransformCommand::Entry>{{id, before, after}}, "Move"));
        check(!u.canRedo(), "new edit discards the redo branch");
        std::printf("[undo] basics ok\n");
    }

    // ---- Create / delete round-trip preserves identity ---------------------
    {
        Scene s;
        UndoStack u;
        const ObjectId a = s.addPrimitive(PrimitiveKind::Box);
        u.push(ExistenceCommand::forCreate(s, {a}));
        check(s.objectCount() == 1, "object created");

        u.undo(s);
        check(s.objectCount() == 0, "undo removes the created object");
        u.redo(s);
        check(s.objectCount() == 1, "redo restores it");
        check(s.find(a) != nullptr, "restored under the same id");

        // Deleting and undoing must bring back the same object, mesh included:
        // selections and later feature references are held by id.
        const int facesBefore = s.find(a)->mesh.faceCount();
        s.find(a)->transform.position = {7, 8, 9};
        u.push(ExistenceCommand::forDelete(s, {a}));
        check(s.objectCount() == 0, "delete removes");
        check(s.find(a) == nullptr, "gone from the scene");

        u.undo(s);
        check(s.objectCount() == 1, "undo restores the deleted object");
        const SceneObject* back = s.find(a);
        check(back != nullptr, "restored with the original id");
        check(back && back->mesh.faceCount() == facesBefore, "mesh survived the round trip");
        check(back && nearV(back->transform.position, {7, 8, 9}), "transform survived");

        // A restored id must not collide with a later creation.
        const ObjectId c = s.addPrimitive(PrimitiveKind::Sphere);
        check(c != a, "new object gets a fresh id after a restore");
        std::printf("[undo] create/delete round-trip ok\n");
    }

    // ---- Merging collapses a drag into one step ---------------------------
    {
        Scene s;
        UndoStack u;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        Transform t0 = s.find(id)->transform;

        for (int i = 1; i <= 5; ++i) {
            Transform t1 = t0;
            t1.position = {static_cast<float>(i), 0, 0};
            u.push(std::make_unique<TransformCommand>(
                std::vector<TransformCommand::Entry>{{id, t0, t1}}, "Transform"), true);
            s.find(id)->transform = t1;
        }
        u.undo(s);
        check(nearV(s.find(id)->transform.position, t0.position),
              "merged drag undoes to the start of the gesture");
        check(!u.canUndo(), "five merged steps are a single entry");
        std::printf("[undo] merge ok\n");
    }

    // ---- Merging must not span separate gestures ---------------------------
    {
        Scene s;
        UndoStack u;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        const Transform t0 = s.find(id)->transform;

        Transform t1 = t0; t1.position = {5, 0, 0};
        u.push(std::make_unique<TransformCommand>(
            std::vector<TransformCommand::Entry>{{id, t0, t1}}, "Transform"), true);

        // Releasing the mouse ends the gesture; the next drag is its own entry.
        u.breakMergeChain();
        Transform t2 = t1; t2.position = {9, 0, 0};
        u.push(std::make_unique<TransformCommand>(
            std::vector<TransformCommand::Entry>{{id, t1, t2}}, "Transform"), true);
        s.find(id)->transform = t2;

        u.undo(s);
        check(nearV(s.find(id)->transform.position, t1.position),
              "second gesture undoes to the end of the first, not past it");
        check(u.canUndo(), "the first gesture is still on the stack");
        u.undo(s);
        check(nearV(s.find(id)->transform.position, t0.position), "first gesture undoes");
        std::printf("[undo] gestures stay separate ok\n");
    }

    // ---- An edit after an undo must not merge across it --------------------
    {
        Scene s;
        UndoStack u;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        const PrimitiveSpec s0 = s.find(id)->spec;

        PrimitiveSpec s1 = s0; s1.box.width = 40.0f;
        u.push(std::make_unique<ParameterCommand>(id, s0, s1), true);
        s.find(id)->spec = s1; s.rebuild(id);

        u.undo(s);   // back to 20 mm
        check(near(s.find(id)->localBounds.size().x, 20.0f), "undo returned to 20 mm");

        // A new edit here must start a fresh entry rather than fold into the
        // command that was just undone.
        PrimitiveSpec s2 = s0; s2.box.width = 80.0f;
        u.push(std::make_unique<ParameterCommand>(id, s0, s2), true);
        s.find(id)->spec = s2; s.rebuild(id);

        u.undo(s);
        check(near(s.find(id)->localBounds.size().x, 20.0f),
              "undo after a post-undo edit returns to the pre-edit state");
        std::printf("[undo] no merging across an undo ok\n");
    }

    // ---- Parameter changes -------------------------------------------------
    {
        Scene s;
        UndoStack u;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        const PrimitiveSpec before = s.find(id)->spec;
        PrimitiveSpec after = before;
        after.box.width = 50.0f;
        s.find(id)->spec = after;
        s.rebuild(id);
        u.push(std::make_unique<ParameterCommand>(id, before, after));

        check(near(s.find(id)->localBounds.size().x, 50.0f), "parameter applied");
        u.undo(s);
        check(near(s.find(id)->localBounds.size().x, 20.0f), "undo re-evaluates the mesh");
        u.redo(s);
        check(near(s.find(id)->localBounds.size().x, 50.0f), "redo re-evaluates again");
        std::printf("[undo] parameter change ok\n");
    }

    // ---- Tool: preconditions ----------------------------------------------
    {
        Scene s;
        Camera cam = makeCamera();
        TransformTool tool;
        check(!tool.begin(TransformMode::Translate, s, cam, {0, 0}),
              "cannot transform with nothing selected");
        check(!tool.active(), "tool stays inactive");
    }

    // ---- Tool: exact numeric translate -------------------------------------
    {
        Scene s;
        Camera cam = makeCamera();
        TransformTool tool;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        s.select(id);
        const Vec3 start = s.find(id)->transform.position;

        check(tool.begin(TransformMode::Translate, s, cam, {500, 400}), "translate begins");
        tool.setConstraint(Constraint::AxisX);
        for (char ch : std::string("12.5")) tool.typeCharacter(ch);
        tool.update(s, cam, {500, 400}, false);
        check(nearV(s.find(id)->transform.position, start + Vec3{12.5f, 0, 0}),
              "typed distance moves exactly along X");

        // Toggling the same axis clears the constraint.
        tool.setConstraint(Constraint::AxisX);
        tool.update(s, cam, {500, 400}, false);
        check(nearV(s.find(id)->transform.position, start),
              "clearing the axis drops the typed translation");

        tool.setConstraint(Constraint::AxisZ);
        tool.update(s, cam, {500, 400}, false);
        check(nearV(s.find(id)->transform.position, start + Vec3{0, 0, 12.5f}),
              "same value now applies along Z");

        auto cmd = tool.confirm(s);
        check(cmd != nullptr, "confirm yields an undo command");
        check(!tool.active(), "tool ends after confirm");

        UndoStack u;
        u.push(std::move(cmd));
        u.undo(s);
        check(nearV(s.find(id)->transform.position, start), "undo reverts the move");
        std::printf("[tool] numeric translate ok\n");
    }

    // ---- Tool: cancel restores exactly -------------------------------------
    {
        Scene s;
        Camera cam = makeCamera();
        TransformTool tool;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        s.select(id);
        const Transform before = s.find(id)->transform;

        tool.begin(TransformMode::Translate, s, cam, {400, 300});
        tool.update(s, cam, {650, 480}, false);
        check(!nearV(s.find(id)->transform.position, before.position), "drag moved it");
        tool.cancel(s);
        check(nearV(s.find(id)->transform.position, before.position), "cancel restores");
        check(!tool.active(), "tool ends after cancel");

        // A transform that never moved should not produce an undo entry.
        tool.begin(TransformMode::Translate, s, cam, {400, 300});
        tool.update(s, cam, {400, 300}, false);
        check(tool.confirm(s) == nullptr, "a no-op transform produces no command");
        std::printf("[tool] cancel ok\n");
    }

    // ---- Tool: rotate about the selection pivot ----------------------------
    {
        Scene s;
        Camera cam = makeCamera();
        TransformTool tool;
        // Two boxes either side of the origin: the pivot is the midpoint, so a
        // 90-degree turn about Z must swap them onto the Y axis.
        const ObjectId a = s.addPrimitive(PrimitiveKind::Box, {}, Vec3{ 30, 0, 0});
        const ObjectId b = s.addPrimitive(PrimitiveKind::Box, {}, Vec3{-30, 0, 0});
        s.select(a); s.select(b, true);

        tool.begin(TransformMode::Rotate, s, cam, {500, 400});
        tool.setConstraint(Constraint::AxisZ);
        for (char ch : std::string("90")) tool.typeCharacter(ch);
        tool.update(s, cam, {500, 400}, false);

        check(nearV(s.find(a)->transform.position, {0, 30, 0}, 1e-2f),
              "first object rotates onto +Y");
        check(nearV(s.find(b)->transform.position, {0, -30, 0}, 1e-2f),
              "second object rotates onto -Y");

        // The objects' own orientation must turn too, not just their position.
        const Vec3 axisX = rotate(s.find(a)->transform.rotation, Vec3{1, 0, 0});
        check(nearV(axisX, {0, 1, 0}, 1e-2f), "object orientation rotated as well");
        tool.cancel(s);
        check(nearV(s.find(a)->transform.position, {30, 0, 0}), "rotate cancel restores");
        std::printf("[tool] rotate about pivot ok\n");
    }

    // ---- Tool: scale --------------------------------------------------------
    {
        Scene s;
        Camera cam = makeCamera();
        TransformTool tool;
        const ObjectId a = s.addPrimitive(PrimitiveKind::Box, {}, Vec3{40, 0, 0});
        const ObjectId b = s.addPrimitive(PrimitiveKind::Box, {}, Vec3{-40, 0, 0});
        s.select(a); s.select(b, true);

        tool.begin(TransformMode::Scale, s, cam, {500, 400});
        for (char ch : std::string("2")) tool.typeCharacter(ch);
        tool.update(s, cam, {500, 400}, false);
        check(nearV(s.find(a)->transform.scale, {2, 2, 2}), "uniform scale applied");
        check(nearV(s.find(a)->transform.position, {80, 0, 0}),
              "positions scale away from the pivot");

        // Constrained scale touches one axis only -- this is the "stretch" case.
        tool.setConstraint(Constraint::AxisZ);
        tool.update(s, cam, {500, 400}, false);
        check(nearV(s.find(a)->transform.scale, {1, 1, 2}), "axis scale affects only Z");
        check(nearV(s.find(a)->transform.position, {40, 0, 0}),
              "position is unchanged along unscaled axes");
        tool.cancel(s);
        std::printf("[tool] scale ok\n");
    }

    // ---- Tool: mouse-driven translate tracks the cursor --------------------
    {
        Scene s;
        Camera cam = makeCamera();
        cam.pitch = radians(89.0f);          // look almost straight down
        cam.yaw = 0.0f;
        TransformTool tool;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        s.select(id);
        const Vec3 start = s.find(id)->transform.position;

        // Dragging right, with the camera overhead, must move the object along
        // +X and leave its height alone.
        tool.begin(TransformMode::Translate, s, cam, {500, 400});
        tool.setConstraint(Constraint::AxisX);
        tool.update(s, cam, {620, 400}, false);
        const Vec3 moved = s.find(id)->transform.position;
        check(moved.x > start.x + 1.0f, "dragging right moves along +X");
        check(near(moved.y, start.y) && near(moved.z, start.z),
              "axis constraint leaves the other axes untouched");

        // Snapping quantises to whole millimetres.
        tool.update(s, cam, {620, 400}, true);
        const float snapped = s.find(id)->transform.position.x - start.x;
        check(near(snapped, std::round(snapped)), "snap quantises to 1 mm");
        std::printf("[tool] mouse translate ok (moved %.2f mm, snapped %.2f)\n",
                    moved.x - start.x, snapped);
        tool.cancel(s);
    }

    // ---- Element transforms: move a face, not the object -------------------
    {
        Scene s;
        Camera cam = makeCamera();
        TransformTool tool;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);   // 20mm cube

        // Top face, found by normal.
        Index top = kInvalid;
        for (Index f = 0; f < s.find(id)->mesh.faceCount(); ++f)
            if (dot(s.find(id)->mesh.faceNormal(f), Vec3{0, 0, 1}) > 0.99f) top = f;
        check(top != kInvalid, "found the top face");

        s.selectElement({id, ElementKind::Face, top});
        const Transform objBefore = s.find(id)->transform;

        check(tool.begin(TransformMode::Translate, s, cam, {500, 400}), "element move begins");
        check(tool.target() == TransformTarget::Elements,
              "an element selection targets vertices, not the object");

        tool.setConstraint(Constraint::AxisZ);
        for (char ch : std::string("6")) tool.typeCharacter(ch);
        tool.update(s, cam, {500, 400}, false);

        const SceneObject* o = s.find(id);
        check(o->transform.position == objBefore.position, "the object itself did not move");
        // Moving the top face up 6mm turns the 20mm cube into a 20x20x26 box.
        check(near(o->localBounds.size().z, 26.0f), "the mesh got 6mm taller");
        check(near(o->localBounds.size().x, 20.0f), "and no wider");

        auto cmd = tool.confirm(s);
        check(cmd != nullptr, "confirm yields a command");
        UndoStack u;
        u.push(std::move(cmd));
        u.undo(s);
        check(near(s.find(id)->localBounds.size().z, 20.0f), "undo restores the mesh");
        u.redo(s);
        check(near(s.find(id)->localBounds.size().z, 26.0f), "redo re-applies");
        std::printf("[element] face move + undo ok\n");
    }

    // Cancelling an element move must restore the vertices exactly.
    {
        Scene s;
        Camera cam = makeCamera();
        TransformTool tool;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        s.selectElement({id, ElementKind::Face, 0});

        const AABB before = s.find(id)->localBounds;
        tool.begin(TransformMode::Translate, s, cam, {400, 300});
        tool.update(s, cam, {700, 520}, false);
        check(!nearV(s.find(id)->localBounds.size(), before.size()), "the drag changed the mesh");
        tool.cancel(s);
        check(nearV(s.find(id)->localBounds.min, before.min), "cancel restores exactly");
        check(nearV(s.find(id)->localBounds.max, before.max), "cancel restores exactly");
        std::printf("[element] cancel ok\n");
    }

    // The gesture is resolved in world space, so it stays correct when the
    // object carries a rotation and a non-uniform scale.
    {
        Scene s;
        Camera cam = makeCamera();
        TransformTool tool;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        SceneObject* o = s.find(id);
        o->transform.rotation = Quat::fromAxisAngle({0, 0, 1}, radians(90.0f));
        o->transform.scale = {2.0f, 1.0f, 1.0f};

        Index top = kInvalid;
        for (Index f = 0; f < o->mesh.faceCount(); ++f)
            if (dot(o->mesh.faceNormal(f), Vec3{0, 0, 1}) > 0.99f) top = f;
        s.selectElement({id, ElementKind::Face, top});

        tool.begin(TransformMode::Translate, s, cam, {500, 400});
        tool.setConstraint(Constraint::AxisZ);
        for (char ch : std::string("10")) tool.typeCharacter(ch);
        tool.update(s, cam, {500, 400}, false);

        // Z is unaffected by a Z-rotation or an X-scale, so a 10mm world move
        // must show up as exactly 10mm of extra local height.
        check(near(s.find(id)->localBounds.size().z, 30.0f),
              "world-space move is correct through rotation and scale");
        tool.cancel(s);
        std::printf("[element] correct under object rotation and scale ok\n");
    }

    // An edge drag moves only its two vertices.
    {
        Scene s;
        Camera cam = makeCamera();
        TransformTool tool;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        const int vertsBefore = s.find(id)->mesh.vertexCount();

        // Snapshot every position rather than testing a coordinate threshold:
        // which end of the box half-edge 0 happens to lie on is an internal
        // detail of the generator, not something the test should assume.
        std::vector<Vec3> original;
        for (Index v = 0; v < vertsBefore; ++v)
            original.push_back(s.find(id)->mesh.verts[v].position);

        s.selectElement({id, ElementKind::Edge, 0});
        tool.begin(TransformMode::Translate, s, cam, {500, 400});
        tool.setConstraint(Constraint::AxisZ);
        for (char ch : std::string("5")) tool.typeCharacter(ch);
        tool.update(s, cam, {500, 400}, false);

        check(s.find(id)->mesh.vertexCount() == vertsBefore, "no vertices added");
        int moved = 0;
        for (Index v = 0; v < vertsBefore; ++v) {
            const Vec3 now = s.find(id)->mesh.verts[v].position;
            if (now != original[v]) {
                ++moved;
                check(near(now.z - original[v].z, 5.0f), "moved exactly 5mm along Z");
            }
        }
        check(moved == 2, "exactly the edge's two vertices moved");
        tool.cancel(s);
        std::printf("[element] edge drag moves two vertices ok\n");
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
