// Tangent - Stage 0 spike: the same catalogue on the kernel that ships today.
//
// This is the control. It builds each part from the same numbers, through the
// same sequence of operations, so that a difference in the results is a
// difference between the kernels and not between two people's idea of a
// bracket. Segment count is a parameter because it is the mesh kernel's only
// dial for precision, and the whole question is what that dial costs.
#include "backends.h"

#include "mesh/boolean.h"
#include "mesh/halfedge.h"
#include "mesh/health.h"
#include "mesh/operations.h"
#include "mesh/primitives.h"

#include <cmath>
#include <string>

using namespace tg;

namespace spike {
namespace {

bool near(double a, double b, double tol = 1e-4) { return std::fabs(a - b) < tol; }

void translate(Mesh& m, double x, double y, double z) {
    for (MeshVertex& v : m.verts)
        v.position += Vec3{static_cast<Real>(x), static_cast<Real>(y), static_cast<Real>(z)};
}

Mesh boxAt(double cx, double cy, double z0, double w, double d, double h) {
    BoxParams bp;
    bp.width = static_cast<Real>(w); bp.depth = static_cast<Real>(d); bp.height = static_cast<Real>(h);
    Mesh m; makeBox(m, bp);
    translate(m, cx, cy, z0 + h / 2);
    return m;
}

Mesh cylAt(double cx, double cy, double z0, double dia, double h, int segments) {
    CylinderParams cp;
    cp.radius = static_cast<Real>(dia / 2); cp.height = static_cast<Real>(h); cp.segments = segments;
    Mesh m; makeCylinder(m, cp);
    translate(m, cx, cy, z0 + h / 2);
    return m;
}

double bodyTopZ(const PartSpec& s, double x, double y) {
    double z = s.plateH;
    for (const Boss& b : s.bosses)
        if (near(b.x, x, 1e-3) && near(b.y, y, 1e-3)) z = s.plateH + b.height;
    return z;
}

bool combine(Mesh& shape, const Mesh& tool, BooleanOp op) {
    Mesh out;
    if (!meshBoolean(shape, tool, op, out)) return false;   // no reason available: see the report
    shape = std::move(out);
    return true;
}

// The mesh equivalent of the OCCT selector, resolved by position for the same
// reason: nothing may depend on the order the boolean left the elements in.
std::vector<Index> selectEdges(const Mesh& m, const PartSpec& s) {
    std::vector<Index> picked;
    for (Index h = 0; h < m.halfedgeCount(); ++h) {
        const Index tw = m.halfedges[h].twin;
        if (tw < h) continue;                      // one half-edge per edge
        const Vec3 a = m.verts[m.halfedges[h].vertex].position;
        const Vec3 b = m.verts[m.halfedges[tw].vertex].position;

        if (s.target == FilletTarget::BoreRims || s.target == FilletTarget::AllTopEdges) {
            bool hit = false;
            for (const Bore& bore : s.bores) {
                const double top = bodyTopZ(s, bore.x, bore.y);
                if (!near(a.z, top, 1e-3) || !near(b.z, top, 1e-3)) continue;
                const double ra = std::hypot(a.x - bore.x, a.y - bore.y);
                const double rb = std::hypot(b.x - bore.x, b.y - bore.y);
                // A faceted bore's rim vertices sit on the circle; the tolerance
                // is on the radius, not on a facet count this code should not
                // have to know.
                if (near(ra, bore.dia / 2, 1e-3) && near(rb, bore.dia / 2, 1e-3)) { hit = true; break; }
            }
            if (hit) { picked.push_back(h); continue; }
        }
        if (s.target == FilletTarget::TopOuterEdges || s.target == FilletTarget::AllTopEdges) {
            if (!near(a.z, s.plateH, 1e-4) || !near(b.z, s.plateH, 1e-4)) continue;
            const double mx = (a.x + b.x) / 2, my = (a.y + b.y) / 2;
            if (near(std::fabs(mx), s.plateW / 2, 1e-4) || near(std::fabs(my), s.plateD / 2, 1e-4))
                picked.push_back(h);
        }
    }
    return picked;
}

} // namespace

Result runMesh(const PartSpec& s, int segments) {
    Result r;

    const double t0 = nowMs();
    Mesh shape = boxAt(0, 0, 0, s.plateW, s.plateD, s.plateH);

    if (s.wallH > 0.0) {
        if (!combine(shape, boxAt(0, -s.plateD / 2 + s.wallT / 2, 0, s.plateW, s.wallT, s.wallH),
                     BooleanOp::Union)) { r.buildNote = "wall fuse refused"; return r; }
    }
    for (size_t i = 0; i < s.bosses.size(); ++i) {
        const Boss& b = s.bosses[i];
        if (!combine(shape, cylAt(b.x, b.y, s.plateH, b.dia, b.height, segments), BooleanOp::Union)) {
            r.buildNote = "boss " + std::to_string(i + 1) + "/" + std::to_string(s.bosses.size())
                        + " refused after " + std::to_string(shape.faceCount()) + " faces";
            return r;
        }
    }
    for (size_t i = 0; i < s.bores.size(); ++i) {
        const Bore& b = s.bores[i];
        const double top = bodyTopZ(s, b.x, b.y);
        if (!combine(shape, cylAt(b.x, b.y, -1.0, b.dia, top + 2.0, segments), BooleanOp::Difference)) {
            r.buildNote = "bore " + std::to_string(i + 1) + "/" + std::to_string(s.bores.size())
                        + " refused after " + std::to_string(shape.faceCount()) + " faces";
            return r;
        }
    }
    for (size_t i = 0; i < s.pockets.size(); ++i) {
        const Pocket& p = s.pockets[i];
        const double z0 = s.plateH - p.depth;
        if (!combine(shape, boxAt(p.x, p.y, z0 - 1.0, p.w, p.d, p.depth + 2.0), BooleanOp::Difference)) {
            r.buildNote = "pocket " + std::to_string(i + 1) + "/" + std::to_string(s.pockets.size())
                        + " refused after " + std::to_string(shape.faceCount()) + " faces";
            return r;
        }
    }
    r.buildMs = nowMs() - t0;
    r.built = true;

    if (s.target != FilletTarget::None && s.filletRadius > 0.0) {
        FilletSpec fs;
        for (Index h : selectEdges(shape, s)) fs.edges.push_back({h, static_cast<Real>(s.filletRadius)});
        fs.segments = 4;   // a real arc rather than a chamfer
        r.filletEdges = static_cast<int>(fs.edges.size());
        std::string why;
        const double t1 = nowMs();
        Mesh work = shape;
        r.filletOk = filletEdges(work, fs, &why);
        r.filletMs = nowMs() - t1;
        if (r.filletOk) shape = std::move(work); else r.filletNote = why.empty() ? "refused" : why;
    } else {
        r.filletOk = true;
    }

    r.faces = shape.faceCount();
    const MeshHealth h = checkHealth(shape, true);
    r.valid = h.solid();
    r.volumeMm3 = h.volume;
    if (!r.valid) {
        r.validNote = h.watertight ? "" : "not watertight; ";
        if (h.degenerateFaces) r.validNote += std::to_string(h.degenerateFaces) + " degenerate; ";
        if (h.selfIntersections > 0) r.validNote += std::to_string(h.selfIntersections) + " self-int";
    }

    // The mesh kernel has no chord deviation: what you see is the segment count
    // the cylinder was created with. Recorded as zero to make that plain.
    r.deviationMm = 0.0;
    const double t2 = nowMs();
    RenderMesh rm;
    shape.buildRenderMesh(rm);
    r.tessMs = nowMs() - t2;
    r.triangles = static_cast<int>(rm.triangles.size() / 3);
    return r;
}

} // namespace spike
