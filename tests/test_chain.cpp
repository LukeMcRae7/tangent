// Does a model rebuild into the model it was?
//
// A feature chain is a promise: the geometry on screen is not the thing being
// kept, the steps are, and running them again gives it back. Everything rests
// on that -- saving, undo, editing a dimension near the root, opening the file
// next year. When it does not hold it does not announce itself: a face comes
// back a different face, an operation lands somewhere else, and the part is
// quietly not the part.
//
// So: build one chain out of every step that works on an exact body, and ask
// three things of it. Does it come back the same when run again. Does it come
// back the same through a file. Does it survive its root being changed.
#include "scene/scene.h"
#include "scene/serialize.h"
#include "temp_path.h"
#include "geom/brep.h"
#include "geom/operations.h"
#include "mesh/decimate.h"
#include "mesh/health.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near(Real a, Real b, Real eps = 1e-6) { return std::fabs(a - b) < eps; }

// What a body is, compactly enough to compare two of them.
struct Shape {
    int faces = 0, edges = 0;
    Real volume = 0.0;
    bool solid = false;
};
static Shape describe(const Body& b) {
    std::vector<EdgeId> es;
    b.allEdges(es);
    return {b.faceCount(), static_cast<int>(es.size()), b.health(false).volume,
            b.validate()};
}
static bool same(const Shape& a, const Shape& b) {
    return a.faces == b.faces && a.edges == b.edges && a.solid == b.solid &&
           near(a.volume, b.volume, 1e-4);
}
static void show(const char* what, const Shape& s) {
    std::printf("  %-22s %3d faces, %3d edges, %10.3f mm3, %s\n", what, s.faces,
                s.edges, s.volume, s.solid ? "solid" : "NOT SOLID");
}

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

// One chain with a step of every kind an exact body accepts, each acting on
// what the one before it left.
static ObjectId buildChain(Scene& s) {
    PrimitiveSpec spec;
    spec.kind = PrimitiveKind::Box;
    spec.box = {40, 30, 20};
    const ObjectId id = s.addPrimitive(PrimitiveKind::Box, spec);
    SceneObject* o = s.find(id);

    // Taper the sides by growing the top, while the box is still a box.
    {
        Feature f;
        f.kind = FeatureKind::FaceScale;
        f.scale = 1.2;
        f.faces = nameFaces(o->body, {facing(o->body, {0, 0, 1})});
        std::string why;
        if (!s.addFeature(id, f, &why)) std::printf("  (scale refused: %s)\n", why.c_str());
    }

    // A line across the middle, so there is a half to take hold of.
    {
        Feature f;
        f.kind = FeatureKind::Divide;
        f.axisPoint = {0, 0, 0};
        f.axisDir = {1, 0, 0};
        if (!s.addFeature(id, f)) { std::printf("  (divide refused)\n"); return id; }
    }
    // Push one half of the top up.
    {
        Feature f;
        f.kind = FeatureKind::Extrude;
        f.distance = 6.0;
        f.mergeFlush = true;
        f.faces = nameFaces(o->featureCache.back(), {facing(o->body, {0, 0, 1})});
        std::string why;
        if (!s.addFeature(id, f, &why)) std::printf("  (push refused: %s)\n", why.c_str());
    }
    // Tip an end.
    {
        SceneObject* cur = s.find(id);
        const FaceId end = facing(cur->body, {0, 1, 0});
        std::vector<EdgeId> es;
        cur->body.faceEdges(end, es);
        Vec3 a, b;
        cur->body.edgePositions(es.front(), a, b);
        Feature f;
        f.kind = FeatureKind::FaceRotate;
        f.angle = radians(8.0);
        f.axisPoint = a;
        f.axisDir = normalize(b - a);
        f.faces = nameFaces(cur->body, {end});
        std::string why;
        if (!s.addFeature(id, f, &why)) std::printf("  (rotate refused: %s)\n", why.c_str());
    }
    // Round a vertical edge.
    {
        SceneObject* cur = s.find(id);
        std::vector<EdgeId> es;
        cur->body.allEdges(es);
        EdgeId upright = kInvalid;
        for (EdgeId e : es) {
            Vec3 a, b;
            cur->body.edgePositions(e, a, b);
            if (std::fabs((b - a).z) > 5.0) { upright = e; break; }
        }
        if (upright != kInvalid) {
            Feature f;
            f.kind = FeatureKind::Bevel;
            f.edges = nameEdges(cur->body, {upright}, false);
            f.width = 3.0;
            std::string why;
            if (!s.addFeature(id, f, &why)) std::printf("  (fillet refused: %s)\n", why.c_str());
        }
    }
    // Hollow it, and tidy what the steps left behind.
    {
        SceneObject* cur = s.find(id);
        // Every face of the underside, not just one of them: a divide leaves
        // the bottom in two, and an offset asked to open half a flat region
        // quietly hollows nothing.
        std::vector<FaceId> all, bottom;
        cur->body.allFaces(all);
        for (FaceId x : all)
            if (dot(cur->body.faceNormal(x), Vec3{0, 0, -1}) > 0.99) bottom.push_back(x);
        Feature f;
        f.kind = FeatureKind::Shell;
        f.thickness = 2.0;
        f.faces = nameFaces(cur->body, bottom);

        std::string why;
        if (!s.addFeature(id, f, &why)) std::printf("  (shell refused: %s)\n", why.c_str());
    }
    {
        Feature f;
        f.kind = FeatureKind::Merge;
        std::string why;
        if (!s.addFeature(id, f, &why)) std::printf("  (merge found nothing: %s)\n", why.c_str());
    }
    return id;
}

// A vertical edge of a body: the first one taller than `atLeast`.
static EdgeId upright(const Body& b, Real atLeast) {
    std::vector<EdgeId> es;
    b.allEdges(es);
    for (EdgeId e : es) {
        Vec3 p, q;
        b.edgePositions(e, p, q);
        if (std::fabs((q - p).z) > atLeast && std::fabs(q.x - p.x) < 1e-9 && std::fabs(q.y - p.y) < 1e-9) return e;
    }
    return kInvalid;
}

static void testHistory() {
    std::printf("--- rolled back, a step put in, rolled forward ---\n");
    {
        // A 20 mm cube with one upright edge rounded 3 mm. Roll back to the
        // bare cube, push its top up 10 mm there, and roll forward: the round
        // has to find its edge on a taller cube, and round all 30 mm of it.
        Scene s;
        PrimitiveSpec spec;
        spec.kind = PrimitiveKind::Box;
        spec.box = {20, 20, 20};
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box, spec);
        Feature round;
        round.kind = FeatureKind::Bevel;
        round.edges = nameEdges(s.find(id)->body, {upright(s.find(id)->body, 5.0)}, false);
        round.width = 3.0;
        round.label = "Rounded corner";
        std::string why;
        check(s.addFeature(id, round, &why), "the round goes on: " + why);
        const Real corner = 9.0 * (1.0 - kPi / 4.0);   // what a 3 mm round takes off, per mm of edge
        check(near(s.find(id)->body.health(false).volume, 8000.0 - corner * 20.0, 1e-3), "off a 20 mm edge");

        check(s.rollTo(id, 1), "rolled back to the cube");
        SceneObject* o = s.find(id);
        check(o->features.size() == 1 && o->ahead.size() == 1, "the round waits after the marker");
        check(near(o->body.health(false).volume, 8000.0, 1e-6), "and the model is the bare cube");

        Feature push;
        push.kind = FeatureKind::Extrude;
        push.distance = 10.0;
        push.mergeFlush = true;
        push.faces = nameFaces(o->body, {facing(o->body, {0, 0, 1})});
        check(s.addFeature(id, push, &why), "a step goes in at the marker: " + why);
        check(o->features.size() == 2 && o->features.back().kind == FeatureKind::Extrude &&
                  o->ahead.size() == 1,
              "before the round, which still waits");

        check(s.rollTo(id, s.historyLength(id)), "rolled forward");
        check(o->features.size() == 3 && o->ahead.empty(), "every step runs");
        check(!o->features.back().errored, "the round found its edge again: " + o->features.back().error);
        check(o->features.back().label == "Rounded corner", "and kept its name");
        check(near(o->body.health(false).volume, 12000.0 - corner * 30.0, 1e-3),
              "rounding the whole 30 mm of it: " + std::to_string(o->body.health(false).volume));
    }

    std::printf("--- two steps swapped ---\n");
    {
        // Pushing the top up 5 and the right side out 5 do not depend on each
        // other: in either order the part is 25 x 20 x 25.
        Scene s;
        PrimitiveSpec spec;
        spec.kind = PrimitiveKind::Box;
        spec.box = {20, 20, 20};
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box, spec);
        auto pushFace = [&](Vec3 dir) {
            Feature f;
            f.kind = FeatureKind::Extrude;
            f.distance = 5.0;
            f.mergeFlush = true;
            f.faces = nameFaces(s.find(id)->body, {facing(s.find(id)->body, dir)});
            std::string why;
            check(s.addFeature(id, f, &why), "pushed: " + why);
        };
        pushFace({0, 0, 1});
        pushFace({1, 0, 0});
        std::vector<Feature> chain = s.find(id)->features;
        std::swap(chain[1], chain[2]);
        std::string why;
        check(s.setFeatures(id, chain, &why), "the other way round builds: " + why);
        const SceneObject* o = s.find(id);
        check(!o->features[1].errored && !o->features[2].errored, "each step still finds its face");
        check(near(o->body.health(false).volume, 25.0 * 20.0 * 25.0, 1e-6),
              "into the same part: " + std::to_string(o->body.health(false).volume));
    }

    std::printf("--- a failed step, pointed again ---\n");
    {
        // A push that names a face nobody has: it fails, and is marked. Rolled
        // back to just before it and pointed at the top, it builds.
        Scene s;
        PrimitiveSpec spec;
        spec.kind = PrimitiveKind::Box;
        spec.box = {20, 20, 20};
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box, spec);
        std::vector<Feature> chain = s.find(id)->features;
        Feature lost;
        lost.kind = FeatureKind::Extrude;
        lost.distance = 5.0;
        lost.mergeFlush = true;
        lost.faces.ids = {0xdeadbeefULL};
        chain.push_back(lost);
        // As a file with it in would come back: the step there, marked failed.
        s.find(id)->features = chain;
        s.reevaluate(id);
        SceneObject* o = s.find(id);
        check(o->features.size() == 2 && o->features.back().errored, "a step that names nothing there fails");

        s.rollTo(id, 1);
        if (o->ahead.empty()) return;
        o->ahead.front().faces = nameFaces(o->body, {facing(o->body, {0, 0, 1})});
        s.rollTo(id, s.historyLength(id));
        check(!o->features.back().errored, "pointed at the top, it builds: " + o->features.back().error);
        check(near(o->body.health(false).volume, 20.0 * 20.0 * 25.0, 1e-6), "and the part is 25 high");
    }

    std::printf("--- a re-run starts at the change, and is right ---\n");
    {
        // Re-runs start at the first step that differs from what was built.
        // What that must never do is hand back a stale body: a step changed and
        // changed back, edited in place, or put back by undo, has to come out
        // as the arithmetic says every time.
        Scene s;
        PrimitiveSpec spec;
        spec.kind = PrimitiveKind::Box;
        spec.box = {20, 20, 20};
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box, spec);
        auto push = [&](Vec3 dir, Real by) {
            Feature f;
            f.kind = FeatureKind::Extrude;
            f.distance = by;
            f.mergeFlush = true;
            f.faces = nameFaces(s.find(id)->body, {facing(s.find(id)->body, dir)});
            s.addFeature(id, f, nullptr);
        };
        push({0, 0, 1}, 5);
        push({1, 0, 0}, 5);
        const std::vector<Feature> original = s.find(id)->features;
        auto volume = [&] { return s.find(id)->body.health(false).volume; };
        check(near(volume(), 25.0 * 20.0 * 25.0, 1e-6), "25 x 20 x 25 to start");

        std::vector<Feature> taller = original;
        taller[1].distance = 10.0;
        std::string why;
        check(s.setFeatures(id, taller, &why), "the first push made 10: " + why);
        check(near(volume(), 25.0 * 20.0 * 30.0, 1e-6), "25 x 20 x 30: " + std::to_string(volume()));
        check(s.setFeatures(id, original, &why), "and put back: " + why);
        check(near(volume(), 25.0 * 20.0 * 25.0, 1e-6), "25 x 20 x 25 again, not the cached 30: " +
                                                            std::to_string(volume()));

        // Edited in place, the way the history panel edits, then re-run.
        s.find(id)->features[2].distance = 15.0;
        check(s.reevaluate(id), "an edit in place re-runs");
        check(near(volume(), 35.0 * 20.0 * 25.0, 1e-6), "35 x 20 x 25: " + std::to_string(volume()));
        s.find(id)->features[2].distance = 5.0;
        check(s.reevaluate(id), "and back");
        check(near(volume(), 25.0 * 20.0 * 25.0, 1e-6), "25 x 20 x 25: " + std::to_string(volume()));

        // Nothing changed: nothing runs, and the body is the same.
        const auto t0 = std::chrono::steady_clock::now();
        check(s.reevaluate(id), "a re-run of nothing changed");
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        check(near(volume(), 25.0 * 20.0 * 25.0, 1e-6), "is the same body");
        std::printf("  a re-run with nothing changed: %.2f ms\n", ms);
    }

    std::printf("--- rolled back, through a file ---\n");
    {
        Scene s;
        PrimitiveSpec spec;
        spec.kind = PrimitiveKind::Box;
        spec.box = {20, 20, 20};
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box, spec);
        Feature push;
        push.kind = FeatureKind::Extrude;
        push.distance = 5.0;
        push.mergeFlush = true;
        push.faces = nameFaces(s.find(id)->body, {facing(s.find(id)->body, {0, 0, 1})});
        push.label = "Taller";
        s.addFeature(id, push, nullptr);
        s.rollTo(id, 1);
        const std::string file = tempPath("rolled.tng");
        check(saveProject(s, file).ok, "saved rolled back");
        Scene back;
        const ProjectResult loaded = loadProject(back, file);
        check(loaded.ok, "loaded: " + loaded.error);
        if (loaded.ok && back.objectCount() == 1) {
            SceneObject* o = back.objects().front().get();
            check(o->features.size() == 1 && o->ahead.size() == 1, "still rolled back, the step waiting");
            check(o->ahead.front().label == "Taller", "with its name");
            check(near(o->body.health(false).volume, 8000.0, 1e-6), "the model as it stood at the marker");
            back.rollTo(o->id, back.historyLength(o->id));
            check(near(o->body.health(false).volume, 10000.0, 1e-6), "and rolled forward, the rest");
        }
        std::remove(file.c_str());
    }
}

int main() {
    if (!brep::available()) {
        std::printf("exact kernel not built; a chain needs one\n");
        return 0;
    }
    // Unbuffered, so a step that takes the process down leaves the checks
    // before it on the screen.
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    testHistory();

    Scene s;
    const ObjectId id = buildChain(s);
    SceneObject* o = s.find(id);
    const Shape built = describe(o->body);
    show("as built", built);
    check(built.solid, "the chain makes a solid");
    check(o->features.size() >= 5, "and has the steps it was given");
    for (const Feature& f : o->features)
        check(!f.errored, std::string("no step errored: ") + f.summary());

    std::printf("--- run it again and it is the same body ---\n");
    {
        // The cache is what the screen is drawn from; running the steps again
        // from the root is what an edit does. They must not differ.
        check(s.reevaluate(id), "re-evaluated");
        const Shape again = describe(s.find(id)->body);
        show("re-evaluated", again);
        check(same(built, again), "the same body");
    }

    std::printf("--- and the same after a trip through a file ---\n");
    {
        const std::string path = tempPath("chain.tgt");
        check(saveProject(s, path).ok, "saved");

        Scene loaded;
        const ProjectResult res = loadProject(loaded, path);
        check(res.ok, "loaded: " + res.error);
        check(loaded.objectCount() == 1, "one object came back");

        if (loaded.objectCount() == 1) {
            SceneObject* l = loaded.objects().front().get();
            check(l->features.size() == o->features.size(),
                  "with every step it went in with");
            for (const Feature& f : l->features)
                check(!f.errored, std::string("no step errored on load: ") + f.summary());

            const Shape fromFile = describe(l->body);
            show("from the file", fromFile);
            check(same(built, fromFile), "the same body as was saved");

            // And running it again on the far side, in case load only cached.
            check(loaded.reevaluate(l->id), "re-evaluated after loading");
            const Shape rebuilt = describe(loaded.objects().front()->body);
            show("rebuilt from file", rebuilt);
            check(same(built, rebuilt), "still the same body");
        }
        std::remove(path.c_str());
    }

    std::printf("--- change the root and every step still finds its own work ---\n");
    {
        // The whole reason steps name what they act on. A box that grows by a
        // millimetre must not strand the fillet on an edge that has moved.
        SceneObject* cur = s.find(id);
        cur->features.front().primitive.box = {44, 30, 20};
        check(s.reevaluate(id), "re-evaluated after growing the box");

        int errored = 0;
        for (const Feature& f : cur->features)
            if (f.errored) {
                ++errored;
                std::printf("    lost: %s -- %s\n", f.summary().c_str(), f.error.c_str());
            }
        check(errored == 0, "no step lost what it was acting on");

        const Shape wider = describe(cur->body);
        show("wider box", wider);
        check(wider.solid, "and it is still a solid");
        check(wider.volume > built.volume, "with more material in it");
    }

    // --- a reduced mesh, through the chain and a file ----------------------
    std::printf("--- a reduced mesh, in the chain and in a file ---\n");
    {
        // A lumpy closed mesh, built directly: the stand-in for an import.
        const int rings = 40, segs = 80;
        std::vector<Vec3> p;
        std::vector<uint32_t> sizes, idx;
        auto at = [&](Real th, Real ph) {
            const Real r = 12 * (1 + 0.08 * std::sin(4 * th) * std::cos(5 * ph));
            return Vec3{r * std::sin(th) * std::cos(ph), r * std::sin(th) * std::sin(ph), r * std::cos(th)};
        };
        p.push_back(at(0, 0));
        for (int i = 1; i < rings; ++i)
            for (int j = 0; j < segs; ++j) p.push_back(at(kPi * i / rings, 2 * kPi * j / segs));
        p.push_back(at(kPi, 0));
        const uint32_t south = static_cast<uint32_t>(p.size() - 1);
        auto V = [&](int i, int j) { return static_cast<uint32_t>(1 + (i - 1) * segs + (j % segs)); };
        auto tri = [&](uint32_t a, uint32_t b, uint32_t c) { sizes.push_back(3); idx.insert(idx.end(), {a, b, c}); };
        for (int j = 0; j < segs; ++j) tri(0, V(1, j), V(1, j + 1));
        for (int i = 1; i < rings - 1; ++i)
            for (int j = 0; j < segs; ++j) { tri(V(i, j), V(i + 1, j), V(i + 1, j + 1)); tri(V(i, j), V(i + 1, j + 1), V(i, j + 1)); }
        for (int j = 0; j < segs; ++j) tri(V(rings - 1, j), south, V(rings - 1, j + 1));
        Mesh m;
        check(m.build(p, sizes, idx), "the mesh builds");

        Scene s;
        const ObjectId id = s.addImportedBody(Body(m), "lumpy");
        check(id != kNoObject, "and comes in as an object");
        SceneObject* obj = s.find(id);

        Feature f;
        f.kind = FeatureKind::Reduce;
        f.uid = 777001;
        f.reduceTolerance = 0.05;
        f.reduceTarget = 1200;
        f.reduceLoosen = true;

        // The way the editor commits: the result built beforehand, from the
        // same body with the same uid, handed over instead of rebuilt.
        Body preview = obj->body;
        ReduceOptions o;
        o.toleranceMm = f.reduceTolerance;
        o.targetTriangles = static_cast<size_t>(f.reduceTarget);
        o.loosenToReachTarget = f.reduceLoosen;
        ReduceResult rr;
        check(reduceBody(preview, o, f.uid, rr), "the preview reduces: " + rr.error);
        s.addFeatureWithResult(id, f, preview);
        const Body committed = obj->body;

        // And the way a chain is rebuilt: from nothing.
        check(s.reevaluate(id), "the chain re-evaluates");
        check(!obj->features.back().errored, "the reduction evaluates: " + obj->features.back().error);
        auto sameMesh = [](const Body& a, const Body& b) {
            if (!a.isMesh() || !b.isMesh()) return false;
            const Mesh& x = a.mesh();
            const Mesh& y = b.mesh();
            if (x.verts.size() != y.verts.size() || x.faces.size() != y.faces.size()) return false;
            for (size_t i = 0; i < x.verts.size(); ++i)
                if (x.verts[i].position.x != y.verts[i].position.x || x.verts[i].position.y != y.verts[i].position.y ||
                    x.verts[i].position.z != y.verts[i].position.z || x.verts[i].id != y.verts[i].id)
                    return false;
            for (size_t i = 0; i < x.faces.size(); ++i)
                if (x.faces[i].id != y.faces[i].id) return false;
            return true;
        };
        check(sameMesh(committed, obj->body), "rebuilding gives exactly what was committed, names and all");
        check(obj->body.faceCount() <= 1200, "within the count it was asked for");

        const std::string path = tempPath("chain_reduce.tgt");
        check(saveProject(s, path).ok, "saved");
        Scene back;
        const ProjectResult res = loadProject(back, path);
        check(res.ok, "loaded: " + res.error);
        SceneObject* again = back.objects().empty() ? nullptr : back.objects().front().get();
        check(again && again->features.size() == 2, "with both steps");
        if (again && again->features.size() == 2) {
            const Feature& g = again->features[1];
            check(g.kind == FeatureKind::Reduce && g.reduceTolerance == 0.05 && g.reduceTarget == 1200 &&
                      g.reduceLoosen, "the reduction's settings come back");
            check(sameMesh(committed, again->body), "and so does exactly the same mesh");
        }
        std::remove(path.c_str());
    }

    // --- an imported exact body, in a file ----------------------------------
    // What a STEP import makes: an object whose chain starts with an exact
    // solid rather than a primitive. Saved, it used to refuse to open.
    if (brep::available()) {
        std::printf("--- an imported exact body survives a file ---\n");
        PrimitiveSpec spec;
        spec.kind = PrimitiveKind::Cylinder;
        spec.cylinder = {8, 20, 48};
        Body solid;
        check(makePrimitive(spec, solid, Backend::Brep), "a solid to import");
        const Real vol = solid.health(false).volume;
        const int faces = solid.faceCount();

        Scene s;
        const ObjectId id = s.addImportedBody(solid, "bracket");
        check(id != kNoObject, "comes in as an object");
        // Where it was brought in: the placement its history starts from.
        s.setBasePlacement(id, Transform{{5, 6, 7}, Quat{}, {1, 1, 1}});

        const std::string path = tempPath("chain_import.tgt");
        check(saveProject(s, path).ok, "saved");
        Scene back;
        const ProjectResult res = loadProject(back, path);
        check(res.ok, "loaded: " + res.error);
        const SceneObject* o = back.objects().empty() ? nullptr : back.objects().front().get();
        check(o != nullptr, "the object is there");
        if (o) {
            check(o->name == "bracket", "with its name");
            check(!o->body.isMesh() && o->body.faceCount() == faces, "still exact, with its faces");
            check(std::fabs(o->body.health(false).volume - vol) < vol * 1e-9, "and its volume");
            check(length(o->transform.position - Vec3{5, 6, 7}) < 1e-12, "where it was put");
        }
        std::remove(path.c_str());
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
