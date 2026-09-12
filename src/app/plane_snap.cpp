#include "app/plane_snap.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace tg {

GridLevels gridLevelsAt(const Camera& camera, Vec3 at, Real spacing, Real subdivide) {
    GridLevels g;
    if (spacing <= 0.0 || subdivide <= 1.0) return g;

    // Straight out of shaders/grid.frag: the level is chosen so the finest cell
    // drawn stays at least kPixelsPerCell wide, which means rounding the level
    // up. Rounding down would pick the level below what the screen can resolve.
    const Real cell = std::max(static_cast<Real>(camera.pixelWorldSize(at)), Real(1e-9));
    const Real lodF = std::log(std::max(cell * 12.0 / spacing, Real(1e-8))) / std::log(subdivide);
    const Real lod  = std::max(std::ceil(lodF), Real(-1));

    g.main  = spacing * std::pow(subdivide, lod);
    g.fine  = g.main / subdivide;
    g.major = g.main * subdivide;
    return g;
}

namespace {

// The coarsest drawn line within reach, and the step it belongs to.
//
// Coarsest first is the whole of "prefer a whole number": on a decimal grid the
// coarser level *is* the rounder number, so a cursor near 50 is given 50 rather
// than 49.8, and one near 49.8 is given 49.8 rather than nothing. Preferring the
// finest would make every position equally snapped and none of them meaningful.
struct AxisGrid {
    Real value = 0.0;
    Real step  = 0.0;      // 0 when nothing was close enough to be worth saying
    bool notable = false;  // on a line the user can actually see
};

AxisGrid nearestGridLine(Real value, const GridLevels& g, Real tolWorld) {
    AxisGrid out{value, 0.0, false};
    for (Real step : {g.major, g.main, g.fine}) {
        if (step <= 0.0) continue;
        const Real at = std::round(value / step) * step;
        if (std::fabs(at - value) <= tolWorld) {
            out.value = at;
            out.step = step;
            out.notable = step >= g.main * 0.999;   // fine lines are a step, not a snap
            return out;
        }
    }
    // Nothing within reach: hold the finest line anyway, so free movement still
    // advances in steps rather than in floating-point dust, but say nothing.
    if (g.fine > 0.0) {
        out.value = std::round(value / g.fine) * g.fine;
        out.step = g.fine;
    }
    return out;
}

} // namespace

namespace {

// What a reference is, said the way a person would say it.
const char* refName(SnapKind k) {
    switch (k) {
        case SnapKind::Vertex:       return "a corner";
        case SnapKind::CircleCentre: return "a centre";
        case SnapKind::ArcQuadrant:  return "a quadrant";
        case SnapKind::EdgeMidpoint: return "a midpoint";
        case SnapKind::FaceCentre:   return "a face centre";
        default:                     return "a point";
    }
}

std::string millimetres(Real v) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%g", static_cast<double>(v));
    return buf;
}

} // namespace

std::string describeSnap(const PlaneSnap& snap) {
    switch (snap.kind) {
        case SnapKind::Vertex:       return "Corner";
        case SnapKind::CircleCentre: return "Centre";
        case SnapKind::ArcQuadrant:  return "Quadrant";
        case SnapKind::EdgeMidpoint: return "Midpoint";
        case SnapKind::FaceCentre:   return "Face centre";

        case SnapKind::Alignment:
            return std::string("In line with ") + refName(snap.refs[0].kind);

        case SnapKind::Intersection:
            return std::string("Where ") + refName(snap.refs[0].kind) + " and " +
                   refName(snap.refs[1].kind) + " cross";

        // Both axes landed on a drawn line, so the weaker of the two is the
        // honest one to name: a point on a 100mm line and a 10mm line is on a
        // 10mm grid.
        case SnapKind::GridPoint:
            return "On the grid, " + millimetres(std::min(snap.stepU, snap.stepV)) + " mm";

        // Only one axis is on a drawn line; the other is merely stepping along
        // the finest, which is smaller and is not what happened.
        case SnapKind::GridLine:
            return "On a grid line, " + millimetres(std::max(snap.stepU, snap.stepV)) + " mm";

        case SnapKind::None:
            break;
    }
    return {};
}

PlaneSnap snapOnPlane(const Scene& scene, const Camera& camera,
                      const PlaneFrame& plane, Vec2 mousePx, Vec2 freeUV,
                      const PlaneSnapConfig& config,
                      const std::vector<SnapPoint>& extra) {
    PlaneSnap out;
    out.uv = freeUV;
    out.point = plane.toWorld(freeUV);

    const Vec3 at = plane.toWorld(freeUV);
    const Real px = std::max(static_cast<Real>(camera.pixelWorldSize(at)), Real(1e-9));
    const Real alignTol = px * config.alignRadiusPx;
    const Real gridTol  = px * config.gridRadiusPx;

    // ---- 1. On a feature ---------------------------------------------------
    //
    // Only points on or very near the plane. Snapping a sketch point to
    // something floating above it would put it somewhere nobody pointed at.
    {
        const SnapHit hit = findSnap(scene, camera, mousePx, config.points);
        if (hit.valid() && plane.distanceTo(hit.point) < px * 4.0) {
            out.kind = hit.kind;
            out.uv = plane.toUV(hit.point);
            out.point = hit.point;
            out.radius = hit.radius;
            return out;
        }
    }
    {
        const SnapPoint* nearest = nullptr;
        Real nearestPx = config.points.radiusPx;
        for (const SnapPoint& p : extra) {
            Vec2 sp{};
            if (!camera.projectToPixel(p.point, sp)) continue;
            const Real d = length(sp - mousePx);
            if (d >= nearestPx) continue;
            nearestPx = d;
            nearest = &p;
        }
        if (nearest) {
            out.kind = nearest->kind;
            out.uv = nearest->uv;
            out.point = nearest->point;
            out.radius = nearest->radius;
            return out;
        }
    }

    // ---- 2. Lined up with something ----------------------------------------
    //
    // The best reference along each axis independently. "Best" is nearest to
    // the cursor's line, not nearest to the cursor: a corner at the far end of
    // the part is exactly as good a thing to be level with as one nearby, and
    // ranking by distance to the cursor would always pick the near one and make
    // the far one unreachable.
    //
    // Each axis is searched over every reference, not over the references left
    // after the other axis has taken its pick. Assigning a reference to one
    // axis up front loses crossings: a corner three units off in u and five in
    // v would be claimed by u, and then beaten there by something closer --
    // taking the v it could have held down with it, and with it the crossing.
    std::vector<SnapPoint> refs;
    int bestUAt = -1, bestVAt = -1;
    Real bestUErr = alignTol, bestVErr = alignTol;

    if (config.alignments) {
        collectSnapPoints(scene, plane, freeUV, alignTol, refs, config.maxRefs,
                          config.points);
        for (const SnapPoint& p : extra) refs.push_back(p);

        for (size_t i = 0; i < refs.size(); ++i) {
            const Real du = std::fabs(refs[i].uv.x - freeUV.x);
            const Real dv = std::fabs(refs[i].uv.y - freeUV.y);
            if (du < bestUErr) { bestUErr = du; bestUAt = static_cast<int>(i); }
            if (dv < bestVErr) { bestVErr = dv; bestVAt = static_cast<int>(i); }
        }

        // One reference cannot hold both axes: that would be the point itself,
        // which the direct test above has already declined -- it is further
        // away on screen than a hit allows. It keeps the axis it is more
        // convincingly on and the other falls to the grid.
        if (bestUAt >= 0 && bestUAt == bestVAt) {
            if (bestUErr <= bestVErr) bestVAt = -1;
            else                      bestUAt = -1;
        }
    }

    const bool haveU = bestUAt >= 0;
    const bool haveV = bestVAt >= 0;
    const Real uValue = haveU ? refs[bestUAt].uv.x : 0.0;
    const Real vValue = haveV ? refs[bestVAt].uv.y : 0.0;
    const SnapRef bestU = haveU ? SnapRef{refs[bestUAt].point, refs[bestUAt].kind, true} : SnapRef{};
    const SnapRef bestV = haveV ? SnapRef{refs[bestVAt].point, refs[bestVAt].kind, false} : SnapRef{};

    // ---- 3. Whatever is left falls to the grid -----------------------------
    const GridLevels levels = config.grid ? gridLevelsAt(camera, at) : GridLevels{0, 0, 0};
    AxisGrid gu{freeUV.x, 0.0, false}, gv{freeUV.y, 0.0, false};
    if (config.grid) {
        if (!haveU) gu = nearestGridLine(freeUV.x, levels, gridTol);
        if (!haveV) gv = nearestGridLine(freeUV.y, levels, gridTol);
    }

    out.uv = {haveU ? uValue : gu.value, haveV ? vValue : gv.value};
    out.point = plane.toWorld(out.uv);
    out.stepU = haveU ? 0.0 : gu.step;
    out.stepV = haveV ? 0.0 : gv.step;

    if (haveU && haveV) {
        out.kind = SnapKind::Intersection;
        out.refs[0] = bestU;
        out.refs[1] = bestV;
        out.refCount = 2;
    } else if (haveU || haveV) {
        out.kind = SnapKind::Alignment;
        out.refs[0] = haveU ? bestU : bestV;
        out.refCount = 1;
    } else if (gu.notable && gv.notable) {
        out.kind = SnapKind::GridPoint;
    } else if (gu.notable || gv.notable) {
        out.kind = SnapKind::GridLine;
    }
    return out;
}

} // namespace tg
