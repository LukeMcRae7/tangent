// Surviving a kernel call that does not come back.
//
// The case that prompted this is in here as a test, which is the only honest
// way to keep it: a fillet whose radius is exactly the thickness of the wall it
// rounds. 1.9mm builds, 3.0mm refuses politely, 2.0mm dereferences a null
// pointer somewhere inside OpenCASCADE. There is no exception to catch, so the
// only question is whether the process that dies owns anything.
#include "geom/kernel_guard.h"
#include "geom/operations.h"
#include "scene/scene.h"

#include <cstdio>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}

int main() {
    std::printf("--- what a guarded call reports ---\n");
    {
        check(tryInChild([] { return true; }) == Attempt::Ok, "success is success");
        check(tryInChild([] { return false; }) == Attempt::Refused, "a refusal is a refusal");
        check(tryInChild([]() -> bool { throw std::runtime_error("no"); }) == Attempt::Refused,
              "an exception is a refusal, not a crash");

        if (childIsolationAvailable()) {
            // The point of the whole thing: this would end the process, and
            // instead it ends a process that owns nothing.
            const Attempt a = tryInChild([]() -> bool {
                volatile int* nowhere = nullptr;
                return *nowhere == 0;
            });
            check(a == Attempt::Crashed, "a crash is reported rather than suffered");
            std::printf("  a null dereference in a guarded call: reported, and we are still here\n");
        } else {
            std::printf("  no isolation on this platform; a crash here is still fatal\n");
        }
    }

    if (!brep::available() || !childIsolationAvailable()) {
        std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
        return failures ? 1 : 0;
    }

    std::printf("--- the fillet that takes the process with it ---\n");
    {
        // A 20mm cube, shelled to a 2mm wall with the top open, then the four
        // edges of one outer wall rounded.
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        SceneObject* o = s.find(id);
        check(!o->body.isMesh(), "the cube is exact");

        std::vector<FaceId> faces;
        o->body.allFaces(faces);
        FaceId top = kNoFace;
        for (FaceId f : faces)
            if (dot(o->body.faceNormal(f), Vec3{0, 0, 1}) > 0.99) top = f;

        Feature shell;
        shell.kind = FeatureKind::Shell;
        shell.thickness = 2.0;
        shell.faces = nameFaces(o->body, {top});
        check(s.addFeature(id, shell), "shelled to a 2mm wall");

        o = s.find(id);
        o->body.allFaces(faces);
        FaceId wall = kNoFace;
        for (FaceId f : faces)
            if (dot(o->body.faceNormal(f), Vec3{0, -1, 0}) > 0.99 &&
                o->body.faceArea(f) > 350.0)
                wall = f;
        check(wall != kNoFace, "found the outer wall");

        std::vector<EdgeId> edges;
        o->body.faceEdges(wall, edges);
        check(edges.size() == 4, "and its four edges");

        auto attempt = [&](Real r) {
            const Body start = o->body;
            return tryInChild([&] {
                Body test = start;
                FilletSpec spec;
                spec.segments = 3;
                for (EdgeId e : edges) spec.edges.push_back({e, r});
                return filletEdges(test, spec);
            });
        };

        check(attempt(1.0) == Attempt::Ok, "1.0mm builds");
        check(attempt(1.9) == Attempt::Ok, "1.9mm builds");
        check(attempt(2.0) == Attempt::Crashed,
              "2.0mm -- the wall's own thickness -- takes its process down");
        check(attempt(3.0) == Attempt::Refused, "3.0mm refuses in the ordinary way");
        std::printf("  1.0 ok, 1.9 ok, 2.0 crashed, 3.0 refused -- and this process is still here\n");
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
