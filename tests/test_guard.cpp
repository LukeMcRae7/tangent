// Surviving a kernel call that does not come back.
//
// The case that prompted this is in here as a test, which is the only honest
// way to keep it: a fillet whose radius is exactly the thickness of the wall it
// rounds. 1.9mm builds, 3.0mm refuses politely, 2.0mm dereferences a null
// pointer somewhere inside OpenCASCADE. There is no exception to catch, so the
// only question is whether the process that dies owns anything.
//
// Run twice by ctest: once as the platform chooses (fork, on Linux), and once
// with TANGENT_ISOLATION=worker, which is the path Windows takes -- the work
// written down, handed to tangent_trial, and the answer read back.
#include "geom/kernel_guard.h"
#include "geom/operations.h"
#include "scene/scene.h"
#include "scene/serialize.h"
#include "scene/trials.h"

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}

static const char* nameOf(Isolation i) {
    switch (i) {
        case Isolation::Fork:   return "fork";
        case Isolation::Worker: return "worker process";
        case Isolation::None:   return "none";
    }
    return "?";
}

static Attempt waitFor(AsyncTrial& trial) {
    for (int i = 0; i < 60000 && !trial.poll(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return trial.result();
}

int main() {
    std::printf("isolation: %s\n", nameOf(isolation()));
    const bool isolated = childIsolationAvailable();

    std::printf("--- what a guarded call reports ---\n");
    {
        check(tryIsolated(selfTestTrial(SelfTest::Build)) == Attempt::Ok, "success is success");
        check(tryIsolated(selfTestTrial(SelfTest::Refuse)) == Attempt::Refused,
              "a refusal is a refusal");
        check(tryIsolated(selfTestTrial(SelfTest::Throw)) == Attempt::Refused,
              "an exception is a refusal, not a crash");
        check(tryIsolated(selfTestTrial(SelfTest::Chatter)) == Attempt::Ok &&
                  tryIsolated(selfTestTrial(SelfTest::Refuse)) == Attempt::Refused,
              "work that prints is still answered, and the next answer is not thrown off");

        if (isolated) {
            // The point of the whole thing: this would end the process, and
            // instead it ends a process that owns nothing.
            check(tryIsolated(selfTestTrial(SelfTest::Crash)) == Attempt::Crashed,
                  "a crash is reported rather than suffered");
            // And whatever died is replaced: the next trial is answered.
            check(tryIsolated(selfTestTrial(SelfTest::Build)) == Attempt::Ok,
                  "the trial after a crash is answered");
            check(tryIsolated(selfTestTrial(SelfTest::Crash)) == Attempt::Crashed &&
                      tryIsolated(selfTestTrial(SelfTest::Crash)) == Attempt::Crashed &&
                      tryIsolated(selfTestTrial(SelfTest::Refuse)) == Attempt::Refused,
                  "and so is one after two crashes in a row");
            std::printf("  a null dereference in a guarded call: reported, and we are still here\n");
        } else {
            std::printf("  no isolation here; a crash would still be fatal\n");
        }
    }

    std::printf("--- without waiting ---\n");
    if (isolated) {
        AsyncTrial trial;
        trial.start(selfTestTrial(SelfTest::Build));
        check(waitFor(trial) == Attempt::Ok, "an async trial builds");
        trial.start(selfTestTrial(SelfTest::Crash));
        check(waitFor(trial) == Attempt::Crashed, "an async trial crashes, and says so");
        trial.start(selfTestTrial(SelfTest::Refuse));
        check(waitFor(trial) == Attempt::Refused, "the same trial object is usable after");

        // Given up on while running, then used again straight away.
        trial.start(selfTestTrial(SelfTest::Build));
        trial.abandon();
        check(!trial.running() && !trial.finished(), "an abandoned trial is neither running nor done");
        trial.start(selfTestTrial(SelfTest::Build));
        check(trial.wait() == Attempt::Ok, "and the next one is answered");
        std::printf("  async: built, crashed, refused, abandoned, built again\n");
    }

    if (!brep::available()) {
        std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
        return failures ? 1 : 0;
    }

    // A 20mm cube, shelled to a 2mm wall with the top open: the body the
    // fillet trials below are all run on.
    Scene s;
    const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
    SceneObject* o = s.find(id);
    check(!o->body.isMesh(), "the cube is exact");
    {
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
    }
    std::vector<FaceId> faces;
    o->body.allFaces(faces);
    FaceId wall = kNoFace;
    for (FaceId f : faces)
        if (dot(o->body.faceNormal(f), Vec3{0, -1, 0}) > 0.99 && o->body.faceArea(f) > 350.0)
            wall = f;
    check(wall != kNoFace, "found the outer wall");
    std::vector<EdgeId> edges;
    o->body.faceEdges(wall, edges);
    check(edges.size() == 4, "and its four edges");

    auto specAt = [&](Real r) {
        FilletSpec spec;
        for (EdgeId e : edges) spec.edges.push_back({e, r});
        return spec;
    };

    std::printf("--- the work, written down and read back ---\n");
    {
        // A worker can only be trusted with work that means the same thing
        // once it has been through the bytes: the same edges, the same chain.
        // Checked here in this process, where no crash can hide an answer.
        const std::vector<Feature> chain = o->features;
        std::vector<Feature> back;
        check(decodeFeatures(encodeFeatures(chain), back) && back.size() == chain.size(),
              "a chain survives being written down");

        Feature holder;
        holder.kind = FeatureKind::BaseMesh;
        holder.backend = Backend::Brep;
        holder.bakedBody = o->body;
        std::vector<Feature> decoded;
        check(decodeFeatures(encodeFeatures({holder}), decoded) && decoded.size() == 1,
              "a body survives being written down");
        if (decoded.size() == 1) {
            const Body& b = decoded.front().bakedBody;
            std::vector<EdgeId> all;
            o->body.allEdges(all);
            int moved = 0;
            for (EdgeId e : all)
                if (!b.hasEdge(e) || length(b.edgeMidpoint(e) - o->body.edgeMidpoint(e)) > 1e-9)
                    ++moved;
            check(moved == 0, "and every edge handle names the same edge after: " +
                                  std::to_string(moved) + " do not");
        }

        for (Real r : {1.0, 3.0}) {
            const GuardedWork w = filletTrial(o->body, specAt(r));
            check(runEncodedTrial(w.kind, w.encode()) == w.run(),
                  "a fillet at " + std::to_string(r) + " answers the same from its bytes");
        }
        const GuardedWork c = chainTrial(chain);
        check(runEncodedTrial(c.kind, c.encode()) == c.run(), "so does a chain");
        std::string garbage = "not a trial at all";
        check(!runEncodedTrial(static_cast<uint32_t>(TrialKind::Fillet), garbage) &&
                  !runEncodedTrial(static_cast<uint32_t>(TrialKind::Chain), garbage) &&
                  !runEncodedTrial(999, garbage),
              "bytes that are not work are refused, not run");
    }

    if (!isolated) {
        std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
        return failures ? 1 : 0;
    }

    std::printf("--- the fillet that takes the process with it ---\n");
    {
        auto attempt = [&](Real r) { return tryIsolated(filletTrial(o->body, specAt(r))); };
        check(attempt(1.0) == Attempt::Ok, "1.0mm builds");
        check(attempt(1.9) == Attempt::Ok, "1.9mm builds");
        check(attempt(2.0) == Attempt::Crashed,
              "2.0mm -- the wall's own thickness -- takes its process down");
        check(attempt(3.0) == Attempt::Refused, "3.0mm refuses in the ordinary way");

        // The way the fillet gesture uses it: one trial at a time, polled
        // between frames, with the crash in the middle of the search.
        AsyncTrial search;
        std::string answers;
        for (Real r : {1.0, 2.0, 1.9, 3.0}) {
            search.start(filletTrial(o->body, specAt(r)));
            const Attempt a = waitFor(search);
            answers += a == Attempt::Ok ? "ok " : a == Attempt::Refused ? "refused " : "crashed ";
        }
        check(answers == "ok crashed ok refused ", "a search through the crash: " + answers);

        // And the chain a history edit tries before it commits.
        Feature round;
        round.kind = FeatureKind::Bevel;
        round.edges = nameEdges(o->body, edges, false);
        round.radii.assign(edges.size(), 2.0);
        round.width = 2.0;
        std::vector<Feature> chain = o->features;
        chain.push_back(round);
        check(tryIsolated(chainTrial(chain)) == Attempt::Crashed,
              "a history edit to the fatal radius is caught before it is evaluated here");
        chain.back().radii.assign(edges.size(), 1.0);
        check(tryIsolated(chainTrial(chain)) == Attempt::Ok, "and one to 1.0mm is let through");
        std::printf("  1.0 ok, 1.9 ok, 2.0 crashed, 3.0 refused -- and this process is still here\n");
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
