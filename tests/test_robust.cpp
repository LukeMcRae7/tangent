// What does the kernel do with inputs nobody should have asked for?
//
// There are three answers an operation can give: it builds, it refuses, or it
// takes the process with it. The first two are both fine -- a modelling tool
// may decline an operation. The third is not survivable: there is no catch for
// it, the model is gone, and the user was not asked whether to save.
//
// OpenCASCADE gives the third answer on some inputs. That is known: it is why
// geom/kernel_guard.h exists and why the fillet verifies a radius in a child
// process before building it. Nothing else does, so this asks how much of a
// problem that is -- every operation, over a range that includes the values a
// drag actually passes through, each in a process that can afford to die.
//
// A crash here is a real defect. A refusal is not.
#include "geom/kernel_guard.h"
#include "geom/operations.h"

#include <cmath>
#include <functional>
#include <cstdio>
#include <string>
#include <vector>

using namespace tg;

namespace {

// A crash is not automatically a defect: the fillet already runs its trials
// where one is survivable, and an input it never offers the user cannot hurt
// them. What is a defect is a crash the application can actually walk into --
// in an operation with no guard at all, or at a value inside the range the
// fillet's search hands out.
int reachable = 0, refusals = 0, builds = 0, guardedCrashes = 0;

struct Tally {
    const char* what;
    int ok = 0, refused = 0, crashed = 0;
};

Body box(Real w, Real d, Real h) {
    PrimitiveSpec spec;
    spec.kind = PrimitiveKind::Box;
    spec.box = {w, d, h};
    Body b;
    makePrimitive(spec, b, Backend::Brep);
    return b;
}

Body cylinder(Real r, Real h) {
    PrimitiveSpec spec;
    spec.kind = PrimitiveKind::Cylinder;
    spec.cylinder = {r, h, 32};
    Body b;
    makePrimitive(spec, b, Backend::Brep);
    return b;
}

FaceId facing(const Body& b, Vec3 dir) {
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

// Runs one attempt where a crash is survivable, and records which of the three
// answers came back.
void attempt(Tally& t, const std::function<bool()>& work, bool guarded = false) {
    switch (tryInChild(work)) {
        case Attempt::Ok:      ++t.ok;      ++builds;   break;
        case Attempt::Refused: ++t.refused; ++refusals; break;
        case Attempt::Crashed:
            ++t.crashed;
            if (guarded) ++guardedCrashes;
            else         ++reachable;
            break;
    }
}

void report(const Tally& t) {
    std::printf("  %-22s %4d built  %4d refused  %4d crashed%s\n", t.what, t.ok,
                t.refused, t.crashed, t.crashed ? "  <--" : "");
}

// The values a gesture actually passes through, plus the ones that break things:
// nothing, everything, exactly the wall, past the end.
const Real kDistances[] = {0.0, 1e-7, 0.01, 0.05, 0.1, 0.5, 1.0, 1.9, 1.999, 2.0,
                           2.001, 2.5, 4.0, 5.0, 8.0, 9.0, 9.99, 10.0, 10.01,
                           12.0, 19.999, 20.0, 25.0, 100.0, 1e4};

} // namespace

int main() {
    if (!brep::available()) {
        std::printf("exact kernel not built; nothing to be robust about\n");
        return 0;
    }
    // Every attempt here goes through tryInChild, which isolates only where it
    // can fork. Windows has isolation too -- the worker, which test_guard
    // drives through a real crash -- so asking childIsolationAvailable said yes
    // and then ran these closures in this process, where the first input that
    // takes OpenCASCADE down took the whole run with it. A closure cannot be
    // sent to a worker, so this probe is a fork-platform probe; what it finds
    // is a fact about the kernel, not about the platform.
    if (!forkIsolationAvailable()) {
        std::printf("no fork here, so these closures cannot be isolated; "
                    "the worker path is covered by test_guard\n");
        return 0;
    }

    std::printf("--- rounding an edge ---\n");
    {
        Tally t{"fillet, one edge"};
        Tally w{"fillet, a shelled wall"};   // guarded: the app never offers these
        Body plain = box(20, 20, 20);
        std::vector<EdgeId> es;
        plain.allEdges(es);

        for (Real r : kDistances) {
            attempt(t, [&] {
                Body b = plain;
                FilletSpec spec;
                spec.edges.push_back({es.front(), r});
                return filletEdges(b, spec);
            });
        }

        // The case that started all of this: a radius equal to the wall it is
        // rounding.
        Body shelled = box(20, 20, 20);
        std::string why;
        if (shellBody(shelled, {facing(shelled, {0, 0, 1})}, 2.0, 1, &why)) {
            std::vector<EdgeId> se;
            shelled.allEdges(se);
            FaceId side = facing(shelled, {0, -1, 0});
            std::vector<EdgeId> sideEdges;
            shelled.faceEdges(side, sideEdges);
            for (Real r : kDistances) {
                attempt(w, [&] {
                    Body b = shelled;
                    FilletSpec spec;
                    for (EdgeId e : sideEdges) spec.edges.push_back({e, r});
                    return filletEdges(b, spec);
                }, /*guarded=*/true);
            }
        }
        report(t);
        report(w);
    }

    std::printf("--- where exactly the fillet gives out ---\n");
    {
        // The guard the fillet uses rests on an assumption: that if a radius
        // builds, every smaller one does too. The search verifies the largest
        // that works and then previews anything below it without a net. If a
        // fatal radius sits *inside* that range, the drag walks straight onto
        // it with nothing to catch it.
        Body shelled = box(20, 20, 20);
        std::string why;
        if (shellBody(shelled, {facing(shelled, {0, 0, 1})}, 2.0, 11, &why)) {
            const FaceId side = facing(shelled, {0, -1, 0});
            std::vector<EdgeId> sideEdges;
            shelled.faceEdges(side, sideEdges);

            Real largestOk = 0.0;
            Real firstCrash = -1.0;
            int crashed = 0;
            for (int i = 1; i <= 260; ++i) {
                const Real r = i * 0.01;      // 0.01 .. 2.60
                const Attempt a2 = tryInChild([&] {
                    Body b = shelled;
                    FilletSpec spec;
                    for (EdgeId e : sideEdges) spec.edges.push_back({e, r});
                    return filletEdges(b, spec);
                });
                if (a2 == Attempt::Ok) largestOk = std::max(largestOk, r);
                if (a2 == Attempt::Crashed) {
                    ++crashed;
                    ++guardedCrashes;
                    if (firstCrash < 0.0) firstCrash = r;
                    std::printf("  crashes at %.2f mm\n", r);
                }
            }
            std::printf("  largest that builds: %.2f mm; %d of 260 crashed\n",
                        largestOk, crashed);
            if (firstCrash >= 0.0 && firstCrash < largestOk) {
                std::printf("  a fatal radius sits BELOW one that works: the drag can "
                            "reach it\n");
                ++reachable;
            } else if (firstCrash >= 0.0) {
                std::printf("  every fatal radius is above everything that works, so "
                            "the search stops short of them\n");
            }
        }
    }

    std::printf("--- moving a face ---\n");
    {
        Tally push{"push / pull"}, grow{"extrude"}, tilt{"rotate"};
        Body plain = box(20, 20, 20);
        const FaceId top = facing(plain, {0, 0, 1});

        for (Real d : kDistances) {
            for (Real sign : {1.0, -1.0}) {
                attempt(push, [&] {
                    Body b = plain;
                    return extrudeFaces(b, {top}, d * sign, nullptr, 1, ExtrudeOp::Auto,
                                        nullptr, true);
                });
                attempt(grow, [&] {
                    Body b = plain;
                    return extrudeFaces(b, {top}, d * sign, nullptr, 2, ExtrudeOp::Auto,
                                        nullptr, false);
                });
            }
        }

        std::vector<EdgeId> te;
        plain.faceEdges(top, te);
        Vec3 a, c;
        plain.edgePositions(te.front(), a, c);
        for (Real deg : {0.0, 1e-6, 0.5, 5.0, 15.0, 45.0, 80.0, 89.0, 89.999, 90.0,
                         91.0, 120.0, 180.0, 270.0, 400.0}) {
            for (Real sign : {1.0, -1.0}) {
                attempt(tilt, [&] {
                    Body b = plain;
                    return rotateFaces(b, {top}, radians(deg * sign), a,
                                       normalize(c - a), 3);
                });
            }
        }
        report(push);
        report(grow);
        report(tilt);
    }

    std::printf("--- dividing and merging ---\n");
    {
        Tally cut{"divide"}, join{"merge"};
        Body plain = box(30, 20, 10);
        for (Real at : {-100.0, -15.001, -15.0, -14.999, -7.5, 0.0, 7.5, 14.999,
                        15.0, 15.001, 100.0}) {
            for (Vec3 n : {Vec3{1, 0, 0}, Vec3{0, 0, 1}, Vec3{1, 1, 1}}) {
                attempt(cut, [&] {
                    Body b = plain;
                    return divideBody(b, {at, 0, 0}, n, 4);
                });
            }
        }

        Body divided = plain;
        std::string why;
        divideBody(divided, {0, 0, 0}, {1, 0, 0}, 5, &why);
        for (int i = 0; i < 3; ++i) {
            attempt(join, [&] {
                Body b = divided;
                return mergeDivisions(b, 6);
            });
        }
        report(cut);
        report(join);
    }

    std::printf("--- hollowing out ---\n");
    {
        Tally t{"shell"};
        Body plain = box(20, 20, 20);
        const FaceId top = facing(plain, {0, 0, 1});
        for (Real wall : kDistances) {
            attempt(t, [&] {
                Body b = plain;
                return shellBody(b, {top}, wall, 7);
            });
            attempt(t, [&] {
                Body b = plain;
                return shellBody(b, {}, wall, 8);       // a sealed cavity
            });
        }
        report(t);
    }

    std::printf("--- combining two bodies ---\n");
    {
        Tally t{"boolean"};
        Body plate = box(40, 40, 10);
        for (Real r : {0.001, 0.1, 1.0, 5.0, 19.999, 20.0, 20.001, 100.0}) {
            for (Real z : {0.0, 5.0, 5.001, 10.0, 1000.0}) {
                for (BooleanOp op : {BooleanOp::Union, BooleanOp::Difference,
                                     BooleanOp::Intersection}) {
                    attempt(t, [&] {
                        Body tool = cylinder(r, 40);
                        tool.transform(translate({0, 0, z}));
                        Body out;
                        return booleanOp(plate, tool, op, out, 9, false, nullptr);
                    });
                }
            }
        }
        report(t);
    }

    std::printf("\n%d built, %d refused, %d crashed where a crash is survivable, "
                "%d reachable\n", builds, refusals, guardedCrashes, reachable);
    if (reachable)
        std::printf("FAILED: an input the application can reach takes the process "
                    "with it\n");
    else
        std::printf("ALL PASS: every operation without a guard declined everything "
                    "it could not build, and the fillet's fatal radii are all above "
                    "the ones it offers\n");
    return reachable ? 1 : 0;
}
