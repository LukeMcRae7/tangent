#include "app/printability.h"

#include "core/bvh.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace tg {
namespace {

// How much material lies behind a face, measured by looking straight into it.
//
// The same question the fillet asks before it offers a radius, and asked the
// same way: stand on the face, look along its inward normal, and see how far it
// is to the other side. Rays go against the tessellation because it is already
// built and a chord tolerance of microns is far below what a nozzle cares
// about.
// How far it is to the next surface, going inward.
//
// Through a tree, because the honest version of this -- ask every triangle --
// is O(faces x triangles), and on a 62,000-triangle import that was nearly four
// billion tests and forty seconds. The tree is built once for the body and
// walked once per face.
Real materialBehind(const TriangleBvh& bvh, Vec3 from, Vec3 inward, Real limit) {
    return bvh.nearestHit(from + inward * Real(1e-3), inward, Real(1e-3), limit);
}


} // namespace

PrintReport checkPrintability(const Body& body, const RenderMesh& render,
                              const PrintProfile& profile) {
    PrintReport out;
    if (body.empty()) return out;

    // Solidity first: a body a slicer cannot tell the inside of is a problem
    // that outranks every other, and nothing else here is worth saying about
    // one. The panel used to report this and no longer does, so it is said
    // here, where the rest of the print problems are.
    const MeshHealth h = body.health(false);
    out.solid = h.watertight && h.volume > 0.0;
    if (!out.solid) out.findings.push_back({kInvalid, PrintIssue::NotSolid, 0.0});

    std::vector<FaceId> faces;
    body.allFaces(faces);
    const AABB bounds = body.bounds();
    const Vec3 extent = bounds.size();
    const Real reach = std::max({extent.x, extent.y, extent.z, Real(1)}) * 1.5;

    TriangleBvh bvh;
    bvh.build(render.positions, render.triangles);

    for (FaceId f : faces) {
        const Vec3 n = body.faceNormal(f);
        if (lengthSq(n) < 1e-18) continue;
        const Vec3 normal = normalize(n);

        // Thinner than the nozzle can lay down. A face with nothing behind it
        // within the body is not thin, it is the outside of something solid.
        //
        // This is the whole check now. Overhang detection used to sit here too
        // and has gone: a slicer decides about supports, does it better because
        // it knows the machine, and does it anyway -- so the only thing a
        // second opinion here bought was a viewport full of amber.
        //
        // Wall thickness is different. A slicer will not warn you: it quietly
        // drops a wall it cannot lay and the part comes off the bed with a hole
        // in it, which is the kind of thing worth knowing before printing.
        // The ray only has to go as far as the limit. Whether a wall is thin is
        // decided by whether material ends within minWallMm; how far away it
        // ends past that is never used. Letting the ray run the width of the
        // part walked most of the tree for an answer nobody reads.
        const Real limit = std::min(reach, profile.minWallMm);
        const Real thickness = materialBehind(bvh, body.faceCentroid(f), -normal, limit);
        if (thickness < limit) {
            ++out.thinWalls;
            out.thinnestWallMm = out.thinnestWallMm > 0.0
                                     ? std::min(out.thinnestWallMm, thickness)
                                     : thickness;
            // Capped. Every offending face is counted, but only so many are
            // kept to be drawn: ten thousand red faces on an imported model is
            // a colour, not information, and holding them all costs memory to
            // say nothing.
            if (out.findings.size() < kMaxDrawnFindings)
                out.findings.push_back({f, PrintIssue::ThinWall, thickness});
        }
    }
    return out;
}

std::string summarise(const PrintReport& report) {
    if (!report.solid) return "Not a closed solid: a slicer cannot tell the inside";
    if (report.clean()) return {};

    char buf[160];
    std::snprintf(buf, sizeof buf, "%d wall%s thinner than the nozzle can lay (%.2f mm)",
                  report.thinWalls, report.thinWalls == 1 ? "" : "s",
                  report.thinnestWallMm);
    return buf;
}

} // namespace tg
