// Parametric feature history: does editing an earlier step re-apply the later
// ones, and does a step whose references have gone stale fail visibly rather
// than producing wrong geometry?
#include "scene/scene.h"

#include <cstdio>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near(float a, float b, float eps = 1e-3f) { return std::fabs(a - b) < eps; }

static double volumeOf(const Mesh& m) {
    RenderMesh rm;
    m.buildRenderMesh(rm);
    double s6 = 0.0;
    for (size_t i = 0; i < rm.triangles.size(); i += 3)
        s6 += dot(rm.positions[rm.triangles[i]],
                  cross(rm.positions[rm.triangles[i + 1]], rm.positions[rm.triangles[i + 2]]));
    return s6 / 6.0;
}

static Index faceFacing(const Mesh& m, Vec3 dir) {
    Index best = kInvalid; float bestDot = -2.0f;
    for (Index f = 0; f < m.faceCount(); ++f) {
        const float d = dot(m.faceNormal(f), dir);
        if (d > bestDot) { bestDot = d; best = f; }
    }
    return best;
}

int main() {
    // ---- A new object starts as a one-feature chain ------------------------
    {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        const SceneObject* o = s.find(id);
        check(o->features.size() == 1, "one feature to begin with");
        check(o->features[0].kind == FeatureKind::Primitive, "and it is the primitive");
        check(near(static_cast<float>(volumeOf(o->mesh)), 8000.0f, 1e-1f), "20mm cube");
        std::printf("[features] base chain ok\n");
    }

    // ---- Editing the base re-applies everything after it -------------------
    {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        SceneObject* o = s.find(id);

        Feature ext;
        ext.kind = FeatureKind::Extrude;
        ext.faces = {faceFacing(o->mesh, {0, 0, 1})};
        ext.distance = 10.0f;
        check(s.addFeature(id, ext), "extrude added");
        check(o->features.size() == 2, "chain has two features");
        check(near(static_cast<float>(volumeOf(o->mesh)), 12000.0f, 1e-1f),
              "cube plus a 10mm extrusion");

        // Widen the base. The extrusion must re-apply to the wider box, which
        // is the entire point: 40 x 20 x 20 plus 40 x 20 x 10.
        o->spec.box.width = 40.0f;
        check(s.rebuild(id), "re-evaluated after a base change");
        check(o->features.size() == 2, "the extrude survived");
        check(!o->features[1].errored, "and did not error");
        check(near(static_cast<float>(volumeOf(o->mesh)), 16000.0f + 8000.0f, 1e-1f),
              "extrusion re-applied to the wider base");
        std::printf("[features] volume after widening: %.1f\n", volumeOf(o->mesh));
    }

    // ---- Disabling a feature skips it, without losing it -------------------
    {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        SceneObject* o = s.find(id);

        Feature bev;
        bev.kind = FeatureKind::Bevel;
        bev.width = 3.0f;
        check(s.addFeature(id, bev), "bevel added");
        const double beveled = volumeOf(o->mesh);
        check(beveled < 8000.0, "bevel removed material");

        o->features[1].enabled = false;
        check(s.reevaluate(id), "re-evaluated with the bevel off");
        check(near(static_cast<float>(volumeOf(o->mesh)), 8000.0f, 1e-1f),
              "back to the plain cube");
        check(o->features.size() == 2, "the disabled feature is still in the chain");

        o->features[1].enabled = true;
        check(s.reevaluate(id), "re-evaluated with it back on");
        check(near(static_cast<float>(volumeOf(o->mesh)),
                   static_cast<float>(beveled), 1e-1f), "and the bevel returns");
        std::printf("[features] enable/disable ok\n");
    }

    // ---- A feature whose references went stale fails visibly ---------------
    //
    // This is the topological naming limitation, pinned down so it cannot
    // regress into silent corruption: the extrude names a face by index, an
    // earlier bevel renumbers every face, and the extrude must then refuse
    // rather than acting on whatever face now holds that index.
    {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        SceneObject* o = s.find(id);

        Feature ext;
        ext.kind = FeatureKind::Extrude;
        ext.faces = {40};                 // no such face on a six-sided box
        ext.distance = 5.0f;
        check(!s.addFeature(id, ext), "an unresolvable feature is refused outright");
        check(o->features.size() == 1, "and is not left in the chain");
        check(near(static_cast<float>(volumeOf(o->mesh)), 8000.0f, 1e-1f),
              "geometry untouched");

        // Now the same thing arising later: a valid extrude, then a base
        // change that leaves the reference dangling.
        Feature good;
        good.kind = FeatureKind::Extrude;
        good.faces = {faceFacing(o->mesh, {0, 0, 1})};
        good.distance = 5.0f;
        check(s.addFeature(id, good), "valid extrude added");

        // Force the reference out of range by hand, then re-evaluate.
        o->features[1].faces = {99};
        check(s.reevaluate(id), "chain still evaluates");
        check(o->features[1].errored, "the broken step is marked errored");
        check(!o->features[1].error.empty(), "with a reason for the timeline");
        check(near(static_cast<float>(volumeOf(o->mesh)), 8000.0f, 1e-1f),
              "and it is skipped rather than applied to the wrong face");
        std::printf("[features] stale reference -> '%s'\n", o->features[1].error.c_str());
    }

    // ---- A chain that produces nothing must not destroy the model ----------
    {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        SceneObject* o = s.find(id);
        const double before = volumeOf(o->mesh);

        o->features[0].primitive.box.width = -5.0f;   // degenerate
        check(!s.reevaluate(id), "a chain producing nothing reports failure");
        check(near(static_cast<float>(volumeOf(o->mesh)),
                   static_cast<float>(before), 1e-1f), "previous mesh survives");
        std::printf("[features] degenerate chain leaves the mesh intact\n");
    }

    // ---- Free-form vertex edits ride along in the chain --------------------
    {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        SceneObject* o = s.find(id);

        Feature edit;
        edit.kind = FeatureKind::VertexEdit;
        edit.verts = {0};
        edit.offsets = {{0.0f, 0.0f, 12.0f}};
        check(s.addFeature(id, edit), "vertex edit added");

        // Vertex 0 of a 20mm box is the (-10,-10,-10) corner; the offset lifts
        // it to z = +2. Checking the vertex itself, not the bounding box: the
        // other seven corners still reach z = -10, so the bounds do not move
        // and would make a bounds-based assertion pass for the wrong reason.
        check(near(o->mesh.verts[0].position.z, 2.0f), "the offset was applied");
        check(near(o->mesh.verts[0].position.x, -10.0f), "and only along Z");

        // Re-evaluating from a changed base must keep the vertex edit applied.
        o->spec.box.width = 30.0f;
        check(s.rebuild(id), "re-evaluated");
        check(near(o->mesh.verts[0].position.z, 2.0f), "vertex edit survived the rebuild");
        check(near(o->mesh.verts[0].position.x, -15.0f),
              "on the vertex as the wider base now places it");
        check(near(o->localBounds.size().x, 30.0f), "and the base change took effect");
        std::printf("[features] vertex edit survives re-evaluation\n");
    }

    // ---- Duplicating an object copies its history --------------------------
    {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        Feature bev;
        bev.kind = FeatureKind::Bevel;
        bev.width = 2.0f;
        check(s.addFeature(id, bev), "bevel added");

        const ObjectId copy = s.duplicateObject(id);
        check(s.find(copy)->features.size() == 2, "the copy carries the chain");
        check(near(static_cast<float>(volumeOf(s.find(copy)->mesh)),
                   static_cast<float>(volumeOf(s.find(id)->mesh)), 1e-1f),
              "and evaluates the same");
        std::printf("[features] duplicate copies history\n");
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
