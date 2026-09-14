// Parametric feature history: does editing an earlier step re-apply the later
// ones, and does a step whose references have gone stale fail visibly rather
// than producing wrong geometry?
#include "mesh/health.h"
#include "geom/operations.h"
#include "scene/scene.h"

#include <cstdio>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near(float a, float b, float eps = 1e-3f) { return std::fabs(a - b) < eps; }

static double volumeOf(const Body& m) {
    RenderMesh rm;
    m.tessellate(rm);
    double s6 = 0.0;
    for (size_t i = 0; i < rm.triangles.size(); i += 3)
        s6 += dot(rm.positions[rm.triangles[i]],
                  cross(rm.positions[rm.triangles[i + 1]], rm.positions[rm.triangles[i + 2]]));
    return s6 / 6.0;
}

static FaceId faceFacing(const Body& b, Vec3 dir) {
    std::vector<FaceId> faces;
    b.allFaces(faces);
    FaceId best = kNoFace; float bestDot = -2.0f;
    for (FaceId f : faces) {
        const float d = dot(b.faceNormal(f), dir);
        if (d > bestDot) { bestDot = d; best = f; }
    }
    return best;
}

int main() {
    // Modelling is the exact kernel's. Without it the chain still holds a
    // primitive, a mesh root and vertex edits, and those sections run; the
    // rest say they were skipped rather than failing for want of a kernel.
    const bool exact = brep::available();
    if (!exact) std::printf("[features] no exact kernel: modelling sections skipped\n");

    // ---- A new object starts as a one-feature chain ------------------------
    {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        const SceneObject* o = s.find(id);
        check(o->features.size() == 1, "one feature to begin with");
        check(o->features[0].kind == FeatureKind::Primitive, "and it is the primitive");
        check(near(static_cast<float>(volumeOf(o->body)), 8000.0f, 1e-1f), "20mm cube");
        std::printf("[features] base chain ok\n");
    }

    // ---- Editing the base re-applies everything after it -------------------
    if (exact) {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        SceneObject* o = s.find(id);

        Feature ext;
        ext.kind = FeatureKind::Extrude;
        ext.faces = nameFaces(o->body, {faceFacing(o->body, {0, 0, 1})});
        ext.distance = 10.0f;
        check(s.addFeature(id, ext), "extrude added");
        check(o->features.size() == 2, "chain has two features");
        check(near(static_cast<float>(volumeOf(o->body)), 12000.0f, 1e-1f),
              "cube plus a 10mm extrusion");

        // Widen the base. The extrusion must re-apply to the wider box, which
        // is the entire point: 40 x 20 x 20 plus 40 x 20 x 10.
        o->spec.box.width = 40.0f;
        check(s.rebuild(id), "re-evaluated after a base change");
        check(o->features.size() == 2, "the extrude survived");
        check(!o->features[1].errored, "and did not error");
        check(near(static_cast<float>(volumeOf(o->body)), 16000.0f + 8000.0f, 1e-1f),
              "extrusion re-applied to the wider base");
        std::printf("[features] volume after widening: %.1f\n", volumeOf(o->body));
    }

    // ---- Disabling a feature skips it, without losing it -------------------
    if (exact) {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        SceneObject* o = s.find(id);

        Feature bev;
        bev.kind = FeatureKind::Bevel;
        bev.edges.kind = ElementRefs::Kind::All;
        bev.width = 3.0f;
        check(s.addFeature(id, bev), "bevel added");
        const double beveled = volumeOf(o->body);
        check(beveled < 8000.0, "bevel removed material");

        o->features[1].enabled = false;
        check(s.reevaluate(id), "re-evaluated with the bevel off");
        check(near(static_cast<float>(volumeOf(o->body)), 8000.0f, 1e-1f),
              "back to the plain cube");
        check(o->features.size() == 2, "the disabled feature is still in the chain");

        o->features[1].enabled = true;
        check(s.reevaluate(id), "re-evaluated with it back on");
        check(near(static_cast<float>(volumeOf(o->body)),
                   static_cast<float>(beveled), 1e-1f), "and the bevel returns");
        std::printf("[features] enable/disable ok\n");
    }

    // ---- A feature whose references went stale fails visibly ---------------
    //
    // This is the topological naming limitation, pinned down so it cannot
    // regress into silent corruption: the extrude names a face by index, an
    // earlier bevel renumbers every face, and the extrude must then refuse
    // rather than acting on whatever face now holds that index.
    if (exact) {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        SceneObject* o = s.find(id);

        Feature ext;
        ext.kind = FeatureKind::Extrude;
        ext.faces.ids = {40};             // no such name on a six-sided box
        ext.distance = 5.0f;
        check(!s.addFeature(id, ext), "an unresolvable feature is refused outright");
        check(o->features.size() == 1, "and is not left in the chain");
        check(near(static_cast<float>(volumeOf(o->body)), 8000.0f, 1e-1f),
              "geometry untouched");

        // Now the same thing arising later: a valid extrude, then a base
        // change that leaves the reference dangling.
        Feature good;
        good.kind = FeatureKind::Extrude;
        good.faces = nameFaces(o->body, {faceFacing(o->body, {0, 0, 1})});
        good.distance = 5.0f;
        check(s.addFeature(id, good), "valid extrude added");

        // Force the reference out of range by hand, then re-evaluate.
        o->features[1].faces.ids = {99};
        check(s.reevaluate(id), "chain still evaluates");
        check(o->features[1].errored, "the broken step is marked errored");
        check(!o->features[1].error.empty(), "with a reason for the timeline");
        check(near(static_cast<float>(volumeOf(o->body)), 8000.0f, 1e-1f),
              "and it is skipped rather than applied to the wrong face");
        std::printf("[features] stale reference -> '%s'\n", o->features[1].error.c_str());
    }

    // ---- A chain that produces nothing must not destroy the model ----------
    {
        Scene s;
        // On a mesh, so it runs with or without the exact kernel.
        s.setDefaultBackend(Backend::Mesh);
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        SceneObject* o = s.find(id);
        const double before = volumeOf(o->body);

        o->features[0].primitive.box.width = -5.0f;   // degenerate
        check(!s.reevaluate(id), "a chain producing nothing reports failure");
        check(near(static_cast<float>(volumeOf(o->body)),
                   static_cast<float>(before), 1e-1f), "previous mesh survives");
        std::printf("[features] degenerate chain leaves the mesh intact\n");
    }

    // ---- Free-form vertex edits ride along in the chain --------------------
    {
        Scene s;
        // Vertex edits are a mesh's: an exact body has no free vertices.
        s.setDefaultBackend(Backend::Mesh);
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        SceneObject* o = s.find(id);

        Feature edit;
        edit.kind = FeatureKind::VertexEdit;
        edit.verts = nameVertices(o->body, {0});
        edit.offsets = {{0.0f, 0.0f, 12.0f}};
        check(s.addFeature(id, edit), "vertex edit added");

        // Vertex 0 of a 20mm box is the (-10,-10,-10) corner; the offset lifts
        // it to z = +2. Checking the vertex itself, not the bounding box: the
        // other seven corners still reach z = -10, so the bounds do not move
        // and would make a bounds-based assertion pass for the wrong reason.
        check(near(o->body.vertexPosition(0).z, 2.0f), "the offset was applied");
        check(near(o->body.vertexPosition(0).x, -10.0f), "and only along Z");

        // Re-evaluating from a changed base must keep the vertex edit applied.
        o->spec.box.width = 30.0f;
        check(s.rebuild(id), "re-evaluated");
        check(near(o->body.vertexPosition(0).z, 2.0f), "vertex edit survived the rebuild");
        check(near(o->body.vertexPosition(0).x, -15.0f),
              "on the vertex as the wider base now places it");
        check(near(o->localBounds.size().x, 30.0f), "and the base change took effect");
        std::printf("[features] vertex edit survives re-evaluation\n");
    }

    // ---- A boolean's operation is editable after the fact ------------------
    //
    // The tool body is baked into the feature, so its shape cannot be changed
    // here -- but which way it combines can, and the History panel now offers
    // it. This is the behaviour behind that control.
    if (exact) {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);   // 20mm cube
        SceneObject* o = s.find(id);

        Body tool;
        PrimitiveSpec spec;
        spec.kind = PrimitiveKind::Box;
        spec.box = {10.0f, 10.0f, 40.0f};
        check(makePrimitive(spec, tool, s.defaultBackend()), "tool body built");

        Feature cut;
        cut.kind = FeatureKind::Boolean;
        cut.booleanOp = BooleanOp::Difference;
        cut.bakedBody = tool;
        check(s.addFeature(id, cut), "difference added");
        const double cutVolume = volumeOf(o->body);
        check(near(static_cast<float>(cutVolume), 8000.0f - 10.0f * 10.0f * 20.0f, 1.0f),
              "a 10x10 hole went through the cube");

        o->features[1].booleanOp = BooleanOp::Union;
        check(s.reevaluate(id), "re-evaluated as a union");
        const double joinVolume = volumeOf(o->body);
        check(joinVolume > cutVolume, "switching to union puts material back");
        check(near(static_cast<float>(joinVolume), 8000.0f + 10.0f * 10.0f * 20.0f, 1.0f),
              "and the tool is now part of the body");
        check(s.takeChainNotice().empty(), "a working switch reports nothing");
        std::printf("[features] boolean op switched in place: %.0f -> %.0f mm3\n",
                    cutVolume, joinVolume);
    }

    // ---- A step that drops out of the chain says so ------------------------
    //
    // The History panel has always marked a failed feature, but the chain is
    // re-run by edits that have nothing to do with the step that breaks. Losing
    // a fillet because a base dimension moved should not be silent.
    if (exact) {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        SceneObject* o = s.find(id);

        Feature bev;
        bev.kind = FeatureKind::Bevel;
        bev.edges.kind = ElementRefs::Kind::All;
        bev.width = 4.0f;
        check(s.addFeature(id, bev), "fillet added to a 20mm cube");
        check(s.takeChainNotice().empty(), "a chain that worked says nothing");

        // Thin the cube until the fillet cannot fit in it.
        o->spec.box.height = 2.0f;
        s.rebuild(id);
        const std::string notice = s.takeChainNotice();
        check(!notice.empty(), "the lost fillet is reported");
        check(notice.find(o->features[1].displayKind()) != std::string::npos,
              "the notice names the step the way the timeline does");
        check(o->features[1].errored, "and the step itself is marked");

        // Still broken, but no longer news: a slider drag re-evaluates every
        // frame and must not shout on each of them.
        s.rebuild(id);
        check(s.takeChainNotice().empty(), "the same failure is not reported twice");

        // Fixed: the fillet comes back and stays quiet.
        o->spec.box.height = 20.0f;
        s.rebuild(id);
        check(!o->features[1].errored, "the fillet returns when it fits again");
        check(s.takeChainNotice().empty(), "recovery is not reported as a failure");
        std::printf("[features] a failed step reports once: %s\n", notice.c_str());
    }

    // ---- Duplicating an object copies its history --------------------------
    if (exact) {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        Feature bev;
        bev.kind = FeatureKind::Bevel;
        bev.edges.kind = ElementRefs::Kind::All;
        bev.width = 2.0f;
        check(s.addFeature(id, bev), "bevel added");

        const ObjectId copy = s.duplicateObject(id);
        check(s.find(copy)->features.size() == 2, "the copy carries the chain");
        check(near(static_cast<float>(volumeOf(s.find(copy)->body)),
                   static_cast<float>(volumeOf(s.find(id)->body)), 1e-1f),
              "and evaluates the same");
        std::printf("[features] duplicate copies history\n");
    }

    // ---- Boolean as a feature ----------------------------------------------
    if (exact) {
        Scene s;
        const ObjectId a = s.addPrimitive(PrimitiveKind::Box);            // 20mm at origin
        const ObjectId b = s.addPrimitive(PrimitiveKind::Box, {}, Vec3{10, 0, 0});

        // Bake B into A's local space, the way the application does.
        SceneObject* A = s.find(a);
        SceneObject* B = s.find(b);
        const Mat4 toLocal = inverse(A->modelMatrix()) * B->modelMatrix();
        Body baked = B->body;
        baked.transform(toLocal);

        Feature f;
        f.kind = FeatureKind::Boolean;
        f.booleanOp = BooleanOp::Difference;
        f.bakedBody = baked;
        check(s.addFeature(a, f), "boolean feature applied");
        check(near(static_cast<Real>(volumeOf(A->body)), 4000.0, 1e-2),
              "A minus B leaves 4000");

        // And it re-evaluates: widen the base and the cut re-applies.
        A->spec.box.width = 30.0f;
        check(s.rebuild(a), "re-evaluated after a base change");
        check(!A->features[1].errored, "the boolean survived");
        // Base now x in [-15,15]; the tool still occupies x in [0,20].
        check(near(static_cast<Real>(volumeOf(A->body)), 15.0 * 20.0 * 20.0, 1e-2),
              "cut re-applied to the wider base");
        std::printf("[features] boolean re-evaluates: %.1f mm3\n", volumeOf(A->body));
    }

    // The tool's own transform has to be taken into account, not just its mesh.
    if (exact) {
        Scene s;
        const ObjectId a = s.addPrimitive(PrimitiveKind::Box);
        const ObjectId b = s.addPrimitive(PrimitiveKind::Box);
        s.find(b)->transform.position = {10, 0, 0};   // moved by transform only

        SceneObject* A = s.find(a);
        SceneObject* B = s.find(b);
        const Mat4 toLocal = inverse(A->modelMatrix()) * B->modelMatrix();
        Body baked = B->body;
        baked.transform(toLocal);

        Feature f;
        f.kind = FeatureKind::Boolean;
        f.booleanOp = BooleanOp::Difference;
        f.bakedBody = baked;
        check(s.addFeature(a, f), "boolean with a transformed tool");
        check(near(static_cast<Real>(volumeOf(A->body)), 4000.0, 1e-2),
              "transform folded into the bake");
        std::printf("[features] transformed tool: %.1f mm3\n", volumeOf(A->body));
    }

    // A boolean that cannot produce a solid is refused, chain untouched.
    if (exact) {
        Scene s;
        const ObjectId a = s.addPrimitive(PrimitiveKind::Box);
        PrimitiveSpec farSpec;
        farSpec.kind = PrimitiveKind::Box;
        Body farBody;
        check(makePrimitive(farSpec, farBody, Backend::Brep), "far box built");
        farBody.transform(translate(Vec3{500, 0, 0}));

        Feature f;
        f.kind = FeatureKind::Boolean;
        f.booleanOp = BooleanOp::Intersection;   // nothing in common
        f.bakedBody = farBody;
        check(!s.addFeature(a, f), "impossible boolean is refused");
        check(s.find(a)->features.size() == 1, "and leaves no dead feature");
        check(near(static_cast<Real>(volumeOf(s.find(a)->body)), 8000.0, 1e-2),
              "geometry untouched");
        std::printf("[features] impossible boolean refused\n");
    }

    // ---- Split into bodies ---------------------------------------------------
    if (exact) {
        // One box either side of a gap, joined into a single body by a union.
        PrimitiveSpec boxSpec;
        boxSpec.kind = PrimitiveKind::Box;
        Body leftBody, rightBody;
        check(makePrimitive(boxSpec, leftBody, Backend::Brep) &&
              makePrimitive(boxSpec, rightBody, Backend::Brep), "two boxes built");
        rightBody.transform(translate(Vec3{100, 0, 0}));
        Body bothBody;
        check(booleanOp(leftBody, rightBody, BooleanOp::Union, bothBody),
              "union of two disjoint boxes");

        std::vector<Body> bodies;
        check(splitBodies(bothBody, bodies) == 2, "splits into two bodies");
        check(bodies.size() == 2, "two bodies out");
        for (const Body& m2 : bodies) {
            std::string err;
            check(m2.validate(&err), std::string("body is valid: ") + err);
            check(near(static_cast<Real>(volumeOf(m2)), 8000.0, 1e-2), "each body is 8000");
        }
        std::printf("[features] split: %zu bodies of %.0f mm3 each\n",
                    bodies.size(), volumeOf(bodies[0]));

        // A single body splits to itself, so a caller can always use the result.
        std::vector<Body> one;
        check(splitBodies(leftBody, one) == 1, "one body stays one");
        check(near(static_cast<Real>(volumeOf(one[0])), 8000.0, 1e-2), "and is unchanged");
    }

    // A BaseMesh chain root carries geometry that has no parameters.
    {
        Scene s;
        // A mesh root, so it runs with or without the exact kernel.
        s.setDefaultBackend(Backend::Mesh);
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        Mesh sphereMesh;
        makeSphere(sphereMesh);
        Body sphere(std::move(sphereMesh));

        Feature base;
        base.kind = FeatureKind::BaseMesh;
        base.bakedBody = sphere;
        s.find(id)->features = {base};
        check(s.reevaluate(id), "BaseMesh evaluates");
        check(s.find(id)->body.faceCount() == sphere.faceCount(), "geometry came through");
        std::printf("[features] BaseMesh root: %d faces\n", s.find(id)->body.faceCount());
    }

    // ---- Acting on a body that has been cut --------------------------------
    // Every face of a boolean's output used to come back nameless. A feature
    // acting on one stored nothing, and on the next evaluation nothing matched
    // the first face in the mesh -- so extruding the top of a bored block
    // quietly moved a different face instead. It happened after almost every
    // boolean, which is what made it so visible.
    if (exact) {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);

        PrimitiveSpec bore;
        bore.kind = PrimitiveKind::Cylinder;
        bore.cylinder.radius = 6;
        bore.cylinder.height = 40;
        Body tool;
        check(makePrimitive(bore, tool, Backend::Brep), "bore tool built");
        tool.transform(translate(Vec3{3, 3, 10}));

        Feature cut;
        cut.kind = FeatureKind::Boolean;
        cut.booleanOp = BooleanOp::Difference;
        cut.bakedBody = tool;
        check(s.addFeature(id, cut), "bore a hole");

        SceneObject* o = s.find(id);
        check(o->body.health().solid(), "the bored block is solid");

        int unnamed = 0;
        std::vector<VertexId> nv;
        std::vector<FaceId> nf;
        o->body.allVertices(nv);
        o->body.allFaces(nf);
        for (VertexId v : nv) if (o->body.vertexName(v) == kNoId) ++unnamed;
        for (FaceId f : nf) if (o->body.faceName(f) == kNoId) ++unnamed;
        check(unnamed == 0, "a boolean names everything it produces (" +
                                std::to_string(unnamed) + " unnamed)");

        // Pick the top and extrude it.
        const FaceId top = faceFacing(o->body, {0, 0, 1});
        const Real bottomBefore = o->body.bounds().min.z;

        Feature ext;
        ext.kind = FeatureKind::Extrude;
        ext.faces = nameFaces(o->body, {top});
        ext.distance = 15.0;
        check(s.addFeature(id, ext), "extrude the top of the bored block");

        o = s.find(id);
        check(!o->features.back().errored,
              std::string("the extrude resolves: ") + o->features.back().error);
        check(std::fabs(o->body.bounds().max.z - 25.0) < 1e-6,
              "the top moved up by 15");
        check(std::fabs(o->body.bounds().min.z - bottomBefore) < 1e-6,
              "and the bottom did not move");
        std::printf("[features] extrude after a boolean moves the picked face: "
                    "z %.1f..%.1f\n", o->body.bounds().min.z, o->body.bounds().max.z);
    }

    // ---- Names are derived, so a chain evaluates to the same names twice ----
    //
    // This is the property the whole parametric history rests on: names are
    // propagated through the kernel's own history of each operation, and have
    // to come out the same every time the same chain runs.
    auto namesOf = [](const Body& b) {
        std::vector<ElementId> out;
        std::vector<FaceId> faces;
        std::vector<EdgeId> edges;
        std::vector<VertexId> verts;
        b.allFaces(faces); b.allEdges(edges); b.allVertices(verts);
        for (FaceId f : faces)   out.push_back(b.faceName(f));
        for (EdgeId e : edges)   out.push_back(b.edgeName(e));
        for (VertexId v : verts) out.push_back(b.vertexName(v));
        std::sort(out.begin(), out.end());
        return out;
    };

    if (exact) {
        // A chain with something of everything on it.
        Scene s;
        PrimitiveSpec ps;
        ps.kind = PrimitiveKind::Box;
        ps.box.width = 40.0; ps.box.depth = 30.0; ps.box.height = 20.0;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box, ps);
        SceneObject* o = s.find(id);

        Feature ext;
        ext.kind = FeatureKind::Extrude;
        ext.faces = nameFaces(o->body, {faceFacing(o->body, {0, 0, 1})});
        ext.distance = 8.0;
        check(s.addFeature(id, ext), "extrude for the naming chain");

        PrimitiveSpec cutterSpec;
        cutterSpec.kind = PrimitiveKind::Box;
        cutterSpec.box = {10.0, 10.0, 60.0};
        Body cutter;
        check(makePrimitive(cutterSpec, cutter, Backend::Brep), "cutter built");
        Feature cut;
        cut.kind = FeatureKind::Boolean;
        cut.booleanOp = BooleanOp::Difference;
        cut.bakedBody = cutter;
        check(s.addFeature(id, cut), "bore for the naming chain");

        const std::vector<ElementId> first = namesOf(s.find(id)->body);
        check(!first.empty(), "the chain produced named geometry");
        check(std::find(first.begin(), first.end(), kNoId) == first.end(),
              "and nothing came out unnamed");

        // 1. Evaluating the same chain again produces the same names.
        check(s.reevaluate(id), "chain re-evaluates");
        check(namesOf(s.find(id)->body) == first, "the same chain names the same things");

        // 2. And again from a cold cache, which is the path a freshly loaded
        //    project takes.
        s.find(id)->featureCache.clear();
        check(s.reevaluate(id), "chain re-evaluates from a cold cache");
        check(namesOf(s.find(id)->body) == first, "a cold cache names them the same");

        // 3. Changing a parameter must not disturb what it did not touch: the
        //    features after the change still resolve, which is only possible if
        //    the names they hold survived.
        o = s.find(id);
        o->spec.box.width = 50.0;
        check(s.rebuild(id), "re-evaluated after widening the base");
        o = s.find(id);
        check(!o->features[1].errored,
              std::string("the extrude still resolves: ") + o->features[1].error);
        check(!o->features[2].errored,
              std::string("the bore still resolves: ") + o->features[2].error);

        const std::vector<ElementId> wider = namesOf(s.find(id)->body);
        check(wider.size() == first.size(),
              "the wider body has the same number of elements");
        std::printf("[naming] %zu names, stable across re-evaluation and a base edit\n",
                    first.size());
    }

    std::printf("--- a list of edges keeps the order it was given in ---\n");
    {
        // Feature::radii runs alongside the names: the third name is the third
        // radius. So the order is not presentation, it is the pairing, and
        // anything that reorders puts every radius on another edge.
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        const Body& body = s.find(id)->body;

        std::vector<EdgeId> all;
        body.allEdges(all);
        check(all.size() >= 4, "a box has edges");

        // Deliberately not in the body's own order.
        std::vector<EdgeId> picked{all[3], all[0], all[2], all[1]};
        const ElementRefs named = nameEdges(body, picked, /*allowBoundary=*/false);
        check(named.kind == ElementRefs::Kind::Explicit, "kept as a list");

        std::vector<EdgeId> back;
        check(named.resolveEdges(body, back), "resolved");
        check(back == picked, "and came back in the order it went in");
    }

    std::printf("--- a rim is only used where the order cannot matter ---\n");
    {
        // A set of edges that happens to be exactly a face's rim is better
        // stored as that rim: it survives the face being renumbered or
        // re-split. But a rim resolves in the body's order, not the caller's,
        // so it is only offered when every radius is the same -- which is what
        // allowBoundary says.
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        const Body& body = s.find(id)->body;

        Index top = kInvalid;
        for (Index f = 0; f < body.faceCount(); ++f)
            if (dot(body.faceNormal(f), Vec3{0, 0, 1}) > 0.99) top = f;
        check(top != kInvalid, "found the top");

        std::vector<EdgeId> rim;
        body.faceEdges(top, rim);
        std::vector<EdgeId> shuffled{rim[2], rim[0], rim[3], rim[1]};

        check(nameEdges(body, shuffled, true).kind == ElementRefs::Kind::FaceBoundary,
              "one radius for all of them: the rim is the sturdier name");

        const ElementRefs asList = nameEdges(body, shuffled, false);
        check(asList.kind == ElementRefs::Kind::Explicit,
              "radii that differ: a list, so each keeps its own");
        std::vector<EdgeId> back;
        check(asList.resolveEdges(body, back) && back == shuffled,
              "and in the order that pairs them");
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
