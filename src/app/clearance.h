// Tangent - how close parts come to one another.
//
// For printed parts the question is not whether two solids intersect -- a
// boolean answers that -- but whether there is enough between them everywhere
// for both to come off the printer as separate things: a pin that turns in its
// hole, a lid that lifts off its box. So what is measured is the gap: the
// nearest two surfaces come, pair by pair, and every part of either surface
// that comes closer than the gap asked for, so it can be drawn where it is.
//
// Measured on triangles, exactly: each body is meshed finely -- to a chord
// tolerance well under the gap -- and every pair of triangles near enough to
// matter is measured triangle to triangle, not vertex to surface, so two walls
// crossing at an angle give the distance between them and not between their
// nearest corners. The tolerance is reported beside the answer, since that is
// how far a mesh can be from the surface it stands for.
//
// Two trees walked together find the pairs near enough to matter, so parts far
// apart cost almost nothing and parts that nearly touch cost the triangles
// where they nearly do. It is still work a frame cannot afford on a real part,
// so the application runs it on a worker.
#pragma once

#include "geom/body.h"
#include "scene/assembly.h"

#include <atomic>
#include <memory>
#include <string>
#include <vector>

namespace tg {

struct ClearanceBody {
    ObjectId id = 0;
    std::string name;
    // Meshed already, in the body's own space -- a mesh kept from an earlier
    // run whose body has not changed since -- or null, and `body` is meshed.
    std::shared_ptr<const RenderMesh> mesh;
    Body body;                      // detached: meshing it writes into it
    Mat4 model;                     // where it is
    bool moving = false;            // one of the parts a sweep moves
};

struct ClearanceRequest {
    std::vector<ClearanceBody> bodies;
    Real required = 0.2;            // the gap every pair should keep, mm
    Real deviation = 0.01;          // chord tolerance to mesh to, mm

    // Through a joint's motion: the moving bodies are checked against the rest
    // at each of these moves, applied to where they are now. Empty: as they
    // stand. `sampleValues` is what each is, in the joint's own terms.
    std::vector<Rigid> samples;
    std::vector<Real> sampleValues;
};

struct ClearancePair {
    ObjectId a = 0, b = 0;
    Real gap = 0.0;                 // the nearest the two surfaces come; 0 when they meet
    bool overlap = false;           // the solids run into each other
    bool touching = false;          // the surfaces meet, and go no further
    Vec3 pa{}, pb{};                // where the gap is, on each, in the world
    int sample = -1;                // through a motion: where it was tightest
};

struct ClearanceResult {
    bool ok = false;
    // Every pair whose surfaces come within `limit`; pairs further apart than
    // that are clear by more than it, and are not listed.
    std::vector<ClearancePair> pairs;
    // The triangles closer than `required` to another part, as triples in the
    // world -- the regions to draw -- and those that run into one.
    struct Marks {
        ObjectId id = 0;
        std::vector<Vec3> close;
        std::vector<Vec3> overlap;
    };
    std::vector<Marks> marks;
    Real required = 0.0;
    Real limit = 0.0;
    Real deviation = 0.0;
    double ms = 0.0;
    size_t trianglePairs = 0;
    bool partial = false;           // the pair budget ran out before every pair was measured
    bool cancelled = false;
    // Through a motion: the tightest gap at each sample -- -1 where parts run
    // into each other -- and which was tightest.
    std::vector<Real> sampleGaps;
    std::vector<Real> sampleValues;     // as the request gave them
    int worstSample = -1;
    Rigid worstMove;                    // the move of the moving parts there
    // The meshes it made, for the next run to reuse.
    std::vector<std::pair<ObjectId, std::shared_ptr<const RenderMesh>>> meshes;
};

// The chord tolerance a gap wants: a tenth of it, within reason.
Real clearanceDeviation(Real required);

ClearanceResult checkClearance(ClearanceRequest request, const std::atomic<bool>* cancel = nullptr);

// The distance between two triangles, and the nearest point on each; zero, with
// a point where they cross, when they intersect. Public for the test.
Real triangleDistance(const Vec3 a[3], const Vec3 b[3], Vec3& pa, Vec3& pb);

} // namespace tg
