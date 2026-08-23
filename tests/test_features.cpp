// Parametric feature history: does editing an earlier step re-apply the later
// ones, and does a step whose references have gone stale fail visibly rather
// than producing wrong geometry?
#include "mesh/health.h"
#include "mesh/operations.h"
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
        ext.faces = nameFaces(o->mesh, {faceFacing(o->mesh, {0, 0, 1})});
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
        bev.edges.kind = ElementRefs::Kind::All;
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
        ext.faces.ids = {40};             // no such name on a six-sided box
        ext.distance = 5.0f;
        check(!s.addFeature(id, ext), "an unresolvable feature is refused outright");
        check(o->features.size() == 1, "and is not left in the chain");
        check(near(static_cast<float>(volumeOf(o->mesh)), 8000.0f, 1e-1f),
              "geometry untouched");

        // Now the same thing arising later: a valid extrude, then a base
        // change that leaves the reference dangling.
        Feature good;
        good.kind = FeatureKind::Extrude;
        good.faces = nameFaces(o->mesh, {faceFacing(o->mesh, {0, 0, 1})});
        good.distance = 5.0f;
        check(s.addFeature(id, good), "valid extrude added");

        // Force the reference out of range by hand, then re-evaluate.
        o->features[1].faces.ids = {99};
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
        edit.verts = nameVertices(o->mesh, {0});
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
        bev.edges.kind = ElementRefs::Kind::All;
        bev.width = 2.0f;
        check(s.addFeature(id, bev), "bevel added");

        const ObjectId copy = s.duplicateObject(id);
        check(s.find(copy)->features.size() == 2, "the copy carries the chain");
        check(near(static_cast<float>(volumeOf(s.find(copy)->mesh)),
                   static_cast<float>(volumeOf(s.find(id)->mesh)), 1e-1f),
              "and evaluates the same");
        std::printf("[features] duplicate copies history\n");
    }

    // ---- Boolean as a feature ----------------------------------------------
    {
        Scene s;
        const ObjectId a = s.addPrimitive(PrimitiveKind::Box);            // 20mm at origin
        const ObjectId b = s.addPrimitive(PrimitiveKind::Box, {}, Vec3{10, 0, 0});

        // Bake B into A's local space, the way the application does.
        SceneObject* A = s.find(a);
        SceneObject* B = s.find(b);
        const Mat4 toLocal = inverse(A->modelMatrix()) * B->modelMatrix();
        Mesh baked = B->mesh;
        for (MeshVertex& v : baked.verts) v.position = transformPoint(toLocal, v.position);

        Feature f;
        f.kind = FeatureKind::Boolean;
        f.booleanOp = BooleanOp::Difference;
        f.bakedMesh = baked;
        check(s.addFeature(a, f), "boolean feature applied");
        check(near(static_cast<Real>(volumeOf(A->mesh)), 4000.0, 1e-2),
              "A minus B leaves 4000");

        // And it re-evaluates: widen the base and the cut re-applies.
        A->spec.box.width = 30.0f;
        check(s.rebuild(a), "re-evaluated after a base change");
        check(!A->features[1].errored, "the boolean survived");
        // Base now x in [-15,15]; the tool still occupies x in [0,20].
        check(near(static_cast<Real>(volumeOf(A->mesh)), 15.0 * 20.0 * 20.0, 1e-2),
              "cut re-applied to the wider base");
        std::printf("[features] boolean re-evaluates: %.1f mm3\n", volumeOf(A->mesh));
    }

    // The tool's own transform has to be taken into account, not just its mesh.
    {
        Scene s;
        const ObjectId a = s.addPrimitive(PrimitiveKind::Box);
        const ObjectId b = s.addPrimitive(PrimitiveKind::Box);
        s.find(b)->transform.position = {10, 0, 0};   // moved by transform only

        SceneObject* A = s.find(a);
        SceneObject* B = s.find(b);
        const Mat4 toLocal = inverse(A->modelMatrix()) * B->modelMatrix();
        Mesh baked = B->mesh;
        for (MeshVertex& v : baked.verts) v.position = transformPoint(toLocal, v.position);

        Feature f;
        f.kind = FeatureKind::Boolean;
        f.booleanOp = BooleanOp::Difference;
        f.bakedMesh = baked;
        check(s.addFeature(a, f), "boolean with a transformed tool");
        check(near(static_cast<Real>(volumeOf(A->mesh)), 4000.0, 1e-2),
              "transform folded into the bake");
        std::printf("[features] transformed tool: %.1f mm3\n", volumeOf(A->mesh));
    }

    // A boolean that cannot produce a solid is refused, chain untouched.
    {
        Scene s;
        const ObjectId a = s.addPrimitive(PrimitiveKind::Box);
        Mesh far;
        BoxParams p;
        makeBox(far, p);
        for (MeshVertex& v : far.verts) v.position += Vec3{500, 0, 0};

        Feature f;
        f.kind = FeatureKind::Boolean;
        f.booleanOp = BooleanOp::Intersection;   // nothing in common
        f.bakedMesh = far;
        check(!s.addFeature(a, f), "impossible boolean is refused");
        check(s.find(a)->features.size() == 1, "and leaves no dead feature");
        check(near(static_cast<Real>(volumeOf(s.find(a)->mesh)), 8000.0, 1e-2),
              "geometry untouched");
        std::printf("[features] impossible boolean refused\n");
    }

    // ---- Split into bodies ---------------------------------------------------
    {
        // One box either side of a gap, joined into a single mesh by a union.
        Mesh left, right, both;
        BoxParams p;
        makeBox(left, p);
        makeBox(right, p);
        for (MeshVertex& v : right.verts) v.position += Vec3{100, 0, 0};
        check(meshBoolean(left, right, BooleanOp::Union, both), "union of two disjoint boxes");

        std::vector<Mesh> bodies;
        check(splitShells(both, bodies) == 2, "splits into two bodies");
        check(bodies.size() == 2, "two meshes out");
        for (const Mesh& m2 : bodies) {
            std::string err;
            check(m2.validate(&err), std::string("body is valid: ") + err);
            check(near(static_cast<Real>(volumeOf(m2)), 8000.0, 1e-2), "each body is 8000");
        }
        std::printf("[features] split: %zu bodies of %.0f mm3 each\n",
                    bodies.size(), volumeOf(bodies[0]));

        // A single body splits to itself, so a caller can always use the result.
        std::vector<Mesh> one;
        check(splitShells(left, one) == 1, "one body stays one");
        check(near(static_cast<Real>(volumeOf(one[0])), 8000.0, 1e-2), "and is unchanged");
    }

    // A BaseMesh chain root carries geometry that has no parameters.
    {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        Mesh sphere;
        makeSphere(sphere);

        Feature base;
        base.kind = FeatureKind::BaseMesh;
        base.bakedMesh = sphere;
        s.find(id)->features = {base};
        check(s.reevaluate(id), "BaseMesh evaluates");
        check(s.find(id)->mesh.faceCount() == sphere.faceCount(), "geometry came through");
        std::printf("[features] BaseMesh root: %d faces\n", s.find(id)->mesh.faceCount());
    }

    // ---- An upstream change the fillet has to survive ----------------------
    // The reason features name what they act on. A rim fillet, then the
    // cylinder's segment count raised -- a routine smoothness tweak.
    //
    // With edges stored as indices this came back silently wrong: the indices
    // resolved, to different edges, and the model passed a health check with
    // the rim sharp and a fillet somewhere else entirely. Nothing reported it.
    {
        Scene s;
        PrimitiveSpec spec;
        spec.cylinder.segments = 16;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Cylinder, spec);
        SceneObject* o = s.find(id);

        const AABB b = o->mesh.bounds();
        std::vector<Index> rim;
        for (Index h = 0; h < o->mesh.halfedgeCount(); ++h) {
            if (h > o->mesh.halfedges[h].twin) continue;
            const Vec3 p = o->mesh.verts[o->mesh.fromVertex(h)].position;
            const Vec3 q = o->mesh.verts[o->mesh.halfedges[h].vertex].position;
            if (std::fabs(p.z - b.max.z) < 1e-9 && std::fabs(q.z - b.max.z) < 1e-9)
                rim.push_back(h);
        }
        check(rim.size() == 16, "sixteen rim edges");

        Feature fil;
        fil.kind = FeatureKind::Bevel;
        fil.edges = nameEdges(o->mesh, rim);
        fil.width = 0.3;
        fil.segments = 6;

        // Picking a whole rim is recorded as the rim, not as the edges that
        // happen to make it up today. That is what lets it survive.
        check(fil.edges.kind == ElementRefs::Kind::FaceBoundary,
              "a full rim is recorded as the cap's boundary");
        check(s.addFeature(id, fil), "rim fillet added");

        o = s.find(id);
        o->features[0].primitive.cylinder.segments = 24;
        check(s.reevaluate(id), "chain re-evaluates after the segment change");
        o = s.find(id);
        check(!o->features[1].errored,
              std::string("the fillet still resolves: ") + o->features[1].error);
        check(checkHealth(o->mesh).solid(), "and still produces a solid");

        // Measured, not assumed: against a plain 24-segment cylinder, the
        // fillet must have removed the sliver a 0.3mm round of that rim takes.
        Mesh plain;
        CylinderParams cp;
        cp.segments = 24;
        makeCylinder(plain, cp);

        double rimLen = 0.0;
        const AABB pb = plain.bounds();
        for (Index h = 0; h < plain.halfedgeCount(); ++h) {
            if (h > plain.halfedges[h].twin) continue;
            const Vec3 p = plain.verts[plain.fromVertex(h)].position;
            const Vec3 q = plain.verts[plain.halfedges[h].vertex].position;
            if (std::fabs(p.z - pb.max.z) < 1e-9 && std::fabs(q.z - pb.max.z) < 1e-9)
                rimLen += length(q - p);
        }
        const double r = 0.3;
        const double sector = 0.5 * 6 * r * r * std::sin(kPi / 12.0);
        const double predicted = rimLen * (r * r - sector);
        const double removed = volumeOf(plain) - volumeOf(o->mesh);
        check(std::fabs(removed - predicted) < 0.05,
              "the fillet ran over the whole new rim: removed " +
                  std::to_string(removed) + " against " + std::to_string(predicted));
        std::printf("[features] rim fillet survives 16 -> 24 segments: "
                    "removed %.4f mm3 (analytic %.4f over %.2f mm of rim)\n",
                    removed, predicted, rimLen);
    }

    // ---- Acting on a body that has been cut --------------------------------
    // Every face of a boolean's output used to come back nameless. A feature
    // acting on one stored nothing, and on the next evaluation nothing matched
    // the first face in the mesh -- so extruding the top of a bored block
    // quietly moved a different face instead. It happened after almost every
    // boolean, which is what made it so visible.
    {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);

        CylinderParams bore;
        bore.radius = 6;
        bore.height = 40;
        bore.segments = 32;
        Mesh tool;
        makeCylinder(tool, bore);
        for (auto& v : tool.verts) v.position += Vec3{3, 3, 10};

        Feature cut;
        cut.kind = FeatureKind::Boolean;
        cut.booleanOp = BooleanOp::Difference;
        cut.bakedMesh = tool;
        check(s.addFeature(id, cut), "bore a hole");

        SceneObject* o = s.find(id);
        check(checkHealth(o->mesh).solid(), "the bored block is solid");

        int unnamed = 0;
        for (const MeshVertex& v : o->mesh.verts) if (v.id == kNoId) ++unnamed;
        for (const MeshFace& f : o->mesh.faces) if (f.id == kNoId) ++unnamed;
        check(unnamed == 0, "a boolean names everything it produces (" +
                                std::to_string(unnamed) + " unnamed)");

        // Pick the top and extrude it.
        Index top = 0;
        for (Index f = 0; f < o->mesh.faceCount(); ++f)
            if (dot(o->mesh.faceNormal(f), Vec3{0, 0, 1}) > 0.99 &&
                o->mesh.faceCentroid(f).z > o->mesh.faceCentroid(top).z) top = f;
        const Real bottomBefore = o->mesh.bounds().min.z;

        Feature ext;
        ext.kind = FeatureKind::Extrude;
        ext.faces = nameFaces(o->mesh, {top});
        ext.distance = 15.0;
        check(s.addFeature(id, ext), "extrude the top of the bored block");

        o = s.find(id);
        check(!o->features.back().errored,
              std::string("the extrude resolves: ") + o->features.back().error);
        check(std::fabs(o->mesh.bounds().max.z - 25.0) < 1e-6,
              "the top moved up by 15");
        check(std::fabs(o->mesh.bounds().min.z - bottomBefore) < 1e-6,
              "and the bottom did not move");
        std::printf("[features] extrude after a boolean moves the picked face: "
                    "z %.1f..%.1f\n", o->mesh.bounds().min.z, o->mesh.bounds().max.z);
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
