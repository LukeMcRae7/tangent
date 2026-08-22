// Measurement. Every case uses a 20mm cube at the origin, where the right
// answer can be worked out by hand.
#include "app/measure.h"

#include <cstdio>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near(Real a, Real b, Real eps = 1e-9) { return std::fabs(a - b) < eps; }

static Index faceFacing(const Mesh& m, Vec3 dir) {
    Index best = kInvalid; Real bestDot = -2.0;
    for (Index f = 0; f < m.faceCount(); ++f) {
        const Real d = dot(m.faceNormal(f), dir);
        if (d > bestDot) { bestDot = d; best = f; }
    }
    return best;
}
static Index vertexAt(const Mesh& m, Vec3 p) {
    for (Index v = 0; v < m.vertexCount(); ++v)
        if (lengthSq(m.verts[v].position - p) < 1e-12) return v;
    return kInvalid;
}

int main() {
    // ---- Precision guard ----------------------------------------------------
    //
    // Geometry is double, so the constants must be too. Writing them with an
    // `f` suffix rounds them to float32 before they are ever used: pi came out
    // 8.7e-08 high, and degrees(acos(0)) gave 89.99999749 instead of 90. That
    // is invisible until an angle is measured, which is exactly what surfaced
    // it, so it is pinned down here.
    {
        check(near(kPi, 3.14159265358979323846, 1e-15), "kPi has full double precision");
        check(near(degrees(std::acos(0.0)), 90.0, 1e-12), "acos(0) is 90 degrees");
        check(near(degrees(kPi), 180.0, 1e-12), "pi radians is 180 degrees");
        check(near(radians(90.0), kHalfPi, 1e-15), "90 degrees is pi/2");
        std::printf("[measure] constants: pi = %.17g, degrees(acos(0)) = %.17g\n",
                    kPi, degrees(std::acos(0.0)));
    }

    Scene s;
    const ObjectId id = s.addPrimitive(PrimitiveKind::Box);   // 20mm, centred
    const Mesh& m = s.find(id)->mesh;

    const Index top    = faceFacing(m, {0, 0, 1});
    const Index bottom = faceFacing(m, {0, 0, -1});
    const Index right  = faceFacing(m, {1, 0, 0});

    MeasureTool t;
    t.begin();

    // ---- One entity ---------------------------------------------------------
    {
        const Index v = vertexAt(m, {-10, -10, -10});
        check(v != kInvalid, "found the corner");
        t.pick({id, ElementKind::Vertex, v});
        const MeasureResult r = t.compute(s);
        check(r.valid, "vertex measures");
        check(near(r.from.x, -10.0) && near(r.from.z, -10.0), "reports its position");
        std::printf("[measure] vertex: %s\n", r.summary.c_str());
        t.clearPicks();
    }
    {
        // Any edge of the cube is 20mm.
        t.pick({id, ElementKind::Edge, 0});
        const MeasureResult r = t.compute(s);
        check(r.valid && r.hasLength, "edge measures a length");
        check(near(r.length, 20.0, 1e-9), "cube edge is 20mm");
        std::printf("[measure] edge:   %s\n", r.summary.c_str());
        t.clearPicks();
    }
    {
        t.pick({id, ElementKind::Face, top});
        const MeasureResult r = t.compute(s);
        check(r.valid && r.hasArea, "face measures an area");
        check(near(r.area, 400.0, 1e-9), "20x20 face is 400mm2");
        check(near(r.perimeter, 80.0, 1e-9), "perimeter is 80mm");
        std::printf("[measure] face:   %s, perimeter %.3f\n", r.summary.c_str(), r.perimeter);
        t.clearPicks();
    }

    // ---- Two entities -------------------------------------------------------
    {
        // Opposite corners of the cube: the space diagonal, 20*sqrt(3).
        const Index a = vertexAt(m, {-10, -10, -10});
        const Index b = vertexAt(m, { 10,  10,  10});
        t.pick({id, ElementKind::Vertex, a});
        t.pick({id, ElementKind::Vertex, b});
        const MeasureResult r = t.compute(s);
        check(near(r.distance, 20.0 * std::sqrt(3.0), 1e-9), "space diagonal");
        check(near(r.delta.x, 20.0) && near(r.delta.y, 20.0) && near(r.delta.z, 20.0),
              "per-axis breakdown");
        std::printf("[measure] corner to corner: %.6f mm (expect %.6f)\n",
                    r.distance, 20.0 * std::sqrt(3.0));
        t.clearPicks();
    }
    {
        // Parallel faces: this is the wall-thickness case, and the minimum is
        // between interior points, not between any pair of corners.
        t.pick({id, ElementKind::Face, top});
        t.pick({id, ElementKind::Face, bottom});
        const MeasureResult r = t.compute(s);
        check(near(r.distance, 20.0, 1e-9), "parallel faces are 20mm apart");
        check(r.hasAngle && near(r.angleDeg, 0.0, 1e-6), "and parallel");
        std::printf("[measure] top to bottom: %.6f mm, %.3f deg\n", r.distance, r.angleDeg);
        t.clearPicks();
    }
    {
        // Perpendicular faces meet, so the distance is zero and the angle 90.
        t.pick({id, ElementKind::Face, top});
        t.pick({id, ElementKind::Face, right});
        const MeasureResult r = t.compute(s);
        check(near(r.distance, 0.0, 1e-9), "adjacent faces touch");
        check(r.hasAngle && near(r.angleDeg, 90.0, 1e-6), "at 90 degrees");
        std::printf("[measure] top to side:   %.6f mm, %.3f deg\n", r.distance, r.angleDeg);
        t.clearPicks();
    }
    {
        // A corner to the opposite face: the perpendicular distance is the
        // cube's height, not the longer diagonal to the face's own corners.
        const Index v = vertexAt(m, {-10, -10, 10});
        t.pick({id, ElementKind::Vertex, v});
        t.pick({id, ElementKind::Face, bottom});
        const MeasureResult r = t.compute(s);
        check(near(r.distance, 20.0, 1e-9), "perpendicular distance, not to a corner");
        std::printf("[measure] corner to opposite face: %.6f mm\n", r.distance);
        t.clearPicks();
    }

    // ---- Two objects --------------------------------------------------------
    {
        const ObjectId b = s.addPrimitive(PrimitiveKind::Box, {}, Vec3{50, 0, 0});
        const Mesh& mb = s.find(b)->mesh;
        // Gap between the facing walls: 50 - 10 - 10 = 30mm.
        t.pick({id, ElementKind::Face, faceFacing(m, {1, 0, 0})});
        t.pick({b,  ElementKind::Face, faceFacing(mb, {-1, 0, 0})});
        const MeasureResult r = t.compute(s);
        check(near(r.distance, 30.0, 1e-9), "clearance between two parts");
        check(r.hasAngle && near(r.angleDeg, 0.0, 1e-6), "facing walls are parallel");
        std::printf("[measure] clearance between parts: %.6f mm\n", r.distance);
        t.clearPicks();
    }

    // Object transforms are respected.
    {
        s.find(id)->transform.position = {0, 0, 100};
        const Index v = vertexAt(m, {-10, -10, -10});
        t.pick({id, ElementKind::Vertex, v});
        const MeasureResult r = t.compute(s);
        check(near(r.from.z, 90.0, 1e-9), "measured in world space, not object space");
        std::printf("[measure] moved object: z = %.3f\n", r.from.z);
        t.clearPicks();
        s.find(id)->transform.position = {0, 0, 0};
    }

    // ---- Pick bookkeeping ---------------------------------------------------
    {
        const ElementRef f{id, ElementKind::Face, top};
        t.pick(f);
        check(t.picks().size() == 1, "one pick");
        t.pick(f);
        check(t.picks().empty(), "picking the same entity again removes it");

        t.pick({id, ElementKind::Face, top});
        t.pick({id, ElementKind::Face, bottom});
        t.pick({id, ElementKind::Face, right});
        check(t.picks().size() == 1, "a third pick starts a new measurement");
        check(t.picks()[0].index == right, "from the entity just clicked");
        std::printf("[measure] pick bookkeeping ok\n");
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
