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
#include "geom/operations.h"

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

int main() {
    if (!brep::available()) {
        std::printf("exact kernel not built; a chain needs one\n");
        return 0;
    }

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
        const std::string path =
            std::string(std::getenv("TMPDIR") ? std::getenv("TMPDIR") : "/tmp") +
            "/tangent-chain.tgt";
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

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
