#include "app/printability.h"

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
Real materialBehind(const RenderMesh& render, Vec3 from, Vec3 inward, Real limit) {
    Real nearest = limit;
    const Ray r{from + inward * Real(1e-3), inward};
    for (size_t i = 0; i + 2 < render.triangles.size(); i += 3) {
        Real t = 0.0;
        if (!rayTriangle(r, render.positions[render.triangles[i + 0]],
                         render.positions[render.triangles[i + 1]],
                         render.positions[render.triangles[i + 2]], t))
            continue;
        if (t > 1e-3 && t < nearest) nearest = t;
    }
    return nearest;
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
    out.solid = body.validate() && body.health(false).volume > 0.0;
    if (!out.solid) out.findings.push_back({kInvalid, PrintIssue::NotSolid, 0.0});

    const Vec3 up = normalize(profile.up);

    std::vector<FaceId> faces;
    body.allFaces(faces);
    const AABB bounds = body.bounds();
    const Vec3 extent = bounds.size();
    const Real reach = std::max({extent.x, extent.y, extent.z, Real(1)}) * 1.5;
    const Real bedHeight = dot(bounds.min, up);

    for (FaceId f : faces) {
        const Vec3 n = body.faceNormal(f);
        if (lengthSq(n) < 1e-18) continue;
        const Vec3 normal = normalize(n);

        // How far it leans, the way a slicer counts it: a vertical wall is
        // zero, a horizontal ceiling is ninety, and anything facing upward is
        // not an overhang at all.
        const Real facingDown = dot(normal, up);            // -1 straight down
        if (facingDown < 0.0) {
            const Real lean = degrees(std::asin(clampf(-facingDown, Real(0), Real(1))));

            // Unless it is sitting on the bed. The underside of every part ever
            // made points straight down and is held up by the machine; calling
            // that a ninety-degree overhang would flag everything and mean
            // nothing. A face raised above the bed -- a bridge, a ledge -- is
            // a different matter and is still reported.
            const Vec3 centre = body.faceCentroid(f);
            const bool onTheBed = dot(centre, up) - bedHeight < profile.nozzleMm;

            if (lean > profile.maxOverhangDeg && !onTheBed) {
                out.findings.push_back({f, PrintIssue::Overhang, lean});
                ++out.overhangs;
                out.steepestOverhangDeg = std::max(out.steepestOverhangDeg, lean);
            }
        }

        // Thinner than the nozzle can lay down. A face with nothing behind it
        // within the body is not thin, it is the outside of something solid.
        const Real thickness = materialBehind(render, body.faceCentroid(f), -normal, reach);
        if (thickness < profile.minWallMm && thickness < reach) {
            out.findings.push_back({f, PrintIssue::ThinWall, thickness});
            ++out.thinWalls;
            out.thinnestWallMm = out.thinnestWallMm > 0.0
                                     ? std::min(out.thinnestWallMm, thickness)
                                     : thickness;
        }
    }
    return out;
}

std::string summarise(const PrintReport& report) {
    if (!report.solid) return "Not a closed solid: a slicer cannot tell the inside";
    if (report.clean()) return {};

    char buf[160];
    if (report.thinWalls && report.overhangs)
        std::snprintf(buf, sizeof buf,
                      "%d thin wall%s (%.2f mm) and %d overhang%s (%.0f deg)",
                      report.thinWalls, report.thinWalls == 1 ? "" : "s",
                      report.thinnestWallMm, report.overhangs,
                      report.overhangs == 1 ? "" : "s", report.steepestOverhangDeg);
    else if (report.thinWalls)
        std::snprintf(buf, sizeof buf, "%d wall%s thinner than the nozzle can lay (%.2f mm)",
                      report.thinWalls, report.thinWalls == 1 ? "" : "s",
                      report.thinnestWallMm);
    else
        std::snprintf(buf, sizeof buf, "%d face%s need support (%.0f deg over)",
                      report.overhangs, report.overhangs == 1 ? "" : "s",
                      report.steepestOverhangDeg);
    return buf;
}

} // namespace tg
