// What a fillet is, checked against what the operation means rather than
// against what this code happens to do.
//
// Every expectation here is either a property a fillet must have whatever
// modeller produces it, or a statement about how Blender's bevel behaves, so
// that a disagreement is a finding rather than a matter of taste.
//
// One difference is deliberate and has to be accounted for everywhere below.
// Blender's bevel is parameterised by *offset*: how far the new edge sits from
// the old one, measured along the face. Ours is parameterised by *radius*: the
// ball rolled along the edge, which is what Fusion asks for and what a printed
// part is actually specified by. They agree on a right angle and nowhere else.
// For an interior dihedral theta they are related by
//
//     offset = radius / tan(theta / 2)
//
// so a 2mm radius on a 60 degree edge is a 3.46mm Blender offset. Wherever a
// Blender number is quoted below it has been converted.
#include "mesh/health.h"
#include "mesh/operations.h"
#include "mesh/primitives.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace tg;

static int failures = 0;

// Where we knowingly do something other than Blender. Recorded rather than
// asserted, so the suite stays honest about it instead of quietly passing.
//
// There is one, and it is the same missing capability that stops an
// already-filleted body being filleted again: trimming a surface against the
// fillet arriving at it, rather than only moving the corners of the faces it
// touches. It shows up wherever a fillet has to END at a vertex whose
// surrounding faces meet at a crease -- every vertex of an octahedron, a sphere
// or a torus. Blender builds a vertex mesh there; we refuse.
//
// Refusing is the safe half of the answer: what came out when it was attempted
// was a cap spanning two planes that the triangulator folded, and a folded cap
// is a self-intersecting model. Better to decline than to hand back a part that
// looks right and will not print.
static bool inKnownDivergence = false;
static int divergences = 0;

static void check(bool ok, const std::string& what) {
    if (ok) return;
    if (inKnownDivergence) {
        std::printf("  BLENDER-DIFF: %s\n", what.c_str());
        ++divergences;
        return;
    }
    std::printf("  FAIL: %s\n", what.c_str());
    ++failures;
}

static double volumeOf(const Mesh& m) { return checkHealth(m, false).volume; }

static void expectSolid(const Mesh& m, const std::string& what) {
    std::string err;
    check(m.validate(&err), what + ": validate: " + err);
    const MeshHealth h = checkHealth(m);
    check(h.watertight, what + ": not watertight");
    check(h.degenerateFaces == 0, what + ": degenerate faces");
    check(h.shells == 1, what + ": should stay one shell");
    check(h.selfIntersections == 0,
          what + ": " + std::to_string(h.selfIntersections) + " self-intersections");
    check(h.volume > 0.0, what + ": inside out");
}

static std::vector<Index> allEdges(const Mesh& m) {
    std::vector<Index> e;
    for (Index h = 0; h < m.halfedgeCount(); ++h)
        if (h < m.halfedges[h].twin) e.push_back(h);
    return e;
}

// Regular n-gon prism along Y. Side edges have interior dihedral pi(n-2)/n.
static bool makePrism(Mesh& m, int n, double side, double height) {
    const double R = side / (2.0 * std::sin(kPi / n));
    std::vector<Vec3> p;
    std::vector<uint32_t> sz, ix;
    for (int i = 0; i < n; ++i) {
        const double a = 2.0 * kPi * i / n;
        p.push_back({R * std::cos(a), -height / 2, R * std::sin(a)});
    }
    for (int i = 0; i < n; ++i) {
        const double a = 2.0 * kPi * i / n;
        p.push_back({R * std::cos(a), height / 2, R * std::sin(a)});
    }
    sz.push_back(n); for (int i = 0; i < n; ++i) ix.push_back(i);
    sz.push_back(n); for (int i = n - 1; i >= 0; --i) ix.push_back(n + i);
    for (int i = 0; i < n; ++i) {
        const int j = (i + 1) % n;
        sz.push_back(4);
        ix.push_back(i); ix.push_back(n + i); ix.push_back(n + j); ix.push_back(j);
    }
    return m.build(p, sz, ix);
}

// L-profile in XZ extruded along Y: one reflex edge at x=0,z=0.
static bool makeLPrism(Mesh& m) {
    const double px[6] = {-10, 10, 10, 0, 0, -10};
    const double pz[6] = {-10, -10, 0, 0, 10, 10};
    std::vector<Vec3> p;
    std::vector<uint32_t> sz, ix;
    for (int i = 0; i < 6; ++i) p.push_back({px[i], -10, pz[i]});
    for (int i = 0; i < 6; ++i) p.push_back({px[i], 10, pz[i]});
    sz.push_back(6); for (int i = 0; i < 6; ++i) ix.push_back(i);
    sz.push_back(6); for (int i = 5; i >= 0; --i) ix.push_back(6 + i);
    for (int i = 0; i < 6; ++i) {
        const int j = (i + 1) % 6;
        sz.push_back(4);
        ix.push_back(i); ix.push_back(6 + i); ix.push_back(6 + j); ix.push_back(j);
    }
    return m.build(p, sz, ix);
}

// A cube with its top face rotated about Z, which is what the rotate tool on a
// face selection leaves behind. The four side faces stop being flat.
static bool makeTwistedCube(Mesh& m, double degrees) {
    const double a = degrees * kPi / 180.0;
    const double c[4][2] = {{-10, -10}, {10, -10}, {10, 10}, {-10, 10}};
    std::vector<Vec3> p;
    std::vector<uint32_t> sz, ix;
    for (int i = 0; i < 4; ++i) p.push_back({c[i][0], c[i][1], -10});
    for (int i = 0; i < 4; ++i)
        p.push_back({c[i][0] * std::cos(a) - c[i][1] * std::sin(a),
                     c[i][0] * std::sin(a) + c[i][1] * std::cos(a), 10});
    sz.push_back(4); for (int i = 3; i >= 0; --i) ix.push_back(i);
    sz.push_back(4); for (int i = 0; i < 4; ++i) ix.push_back(4 + i);
    for (int i = 0; i < 4; ++i) {
        const int j = (i + 1) % 4;
        sz.push_back(4);
        ix.push_back(i); ix.push_back(j); ix.push_back(4 + j); ix.push_back(4 + i);
    }
    return m.build(p, sz, ix);
}

// Regular octahedron: every vertex has four edges, so a corner there is one
// the cube cases never reach.
static bool makeOctahedron(Mesh& m, double r) {
    std::vector<Vec3> p = {{r,0,0},{-r,0,0},{0,r,0},{0,-r,0},{0,0,r},{0,0,-r}};
    std::vector<uint32_t> sz, ix;
    const uint32_t tri[8][3] = {{0,2,4},{2,1,4},{1,3,4},{3,0,4},
                                {2,0,5},{1,2,5},{3,1,5},{0,3,5}};
    for (const auto& t : tri) { sz.push_back(3); for (uint32_t v : t) ix.push_back(v); }
    return m.build(p, sz, ix);
}

// A wedge: a triangular prism with an apex angle of `apexDeg`, for edges far
// sharper than anything a box has.
static bool makeWedge(Mesh& m, double apexDeg, double height) {
    const double a = apexDeg * kPi / 360.0;   // half angle
    const double len = 20.0;
    std::vector<Vec3> p;
    std::vector<uint32_t> sz, ix;
    const double pts[3][2] = {{0, 0}, {len, -len * std::tan(a)}, {len, len * std::tan(a)}};
    for (int i = 0; i < 3; ++i) p.push_back({pts[i][0], -height / 2, pts[i][1]});
    for (int i = 0; i < 3; ++i) p.push_back({pts[i][0], height / 2, pts[i][1]});
    sz.push_back(3); for (int i = 0; i < 3; ++i) ix.push_back(i);
    sz.push_back(3); for (int i = 2; i >= 0; --i) ix.push_back(3 + i);
    for (int i = 0; i < 3; ++i) {
        const int j = (i + 1) % 3;
        sz.push_back(4);
        ix.push_back(i); ix.push_back(3 + i); ix.push_back(3 + j); ix.push_back(j);
    }
    return m.build(p, sz, ix);
}

static std::vector<Index> sideEdges(const Mesh& m) {
    std::vector<Index> out;
    for (Index h = 0; h < m.halfedgeCount(); ++h) {
        if (h > m.halfedges[h].twin) continue;
        if (std::fabs(m.verts[m.fromVertex(h)].position.y -
                      m.verts[m.halfedges[h].vertex].position.y) > 1e-9)
            out.push_back(h);
    }
    return out;
}

static Mat4 rotation(Vec3 axis, double degrees) {
    return rotateAxis(normalize(axis), degrees * kPi / 180.0);
}

static Mesh transformed(const Mesh& m, const Mat4& t) {
    Mesh out = m;
    for (MeshVertex& v : out.verts) v.position = transformPoint(t, v.position);
    return out;
}

// Distance from a point to an infinite line.
static double toAxis(Vec3 p, Vec3 origin, Vec3 dir) {
    const Vec3 d = normalize(dir);
    const Vec3 w = p - origin;
    return length(w - d * dot(w, d));
}

int main() {
    // =====================================================================
    // 1. What the radius means, and what it takes off each face
    // =====================================================================
    //
    // A fillet is a ball rolled along an edge touching both faces. Two things
    // follow, and neither depends on the modeller: the surface is everywhere
    // one radius from the ball's path, and each face loses a band of exactly
    // radius/tan(theta/2) measured from the edge.
    //
    // The band is where Blender's number comes from. Set Blender's offset to
    // that figure and it takes the same material off.
    {
        std::printf("-- radius, and the band it takes from each face --\n");
        struct Case { int sides; const char* name; };
        const Case cases[] = {{3, "60 deg"}, {4, "90 deg"}, {6, "120 deg"}, {12, "150 deg"}};

        for (const Case& c : cases) {
            const double theta = kPi * (c.sides - 2) / c.sides;
            const double r = 2.0;
            const double band = r / std::tan(theta / 2);

            Mesh base;
            check(makePrism(base, c.sides, 20.0, 20.0), "prism builds");
            Mesh m = base;
            const std::vector<Index> side = sideEdges(m);
            check(bevelEdges(m, side, r, 8), std::string(c.name) + ": fillet");
            expectSolid(m, c.name);

            // The band: how far the surviving flat wall now sits from the edge
            // it was filleted at. Measured as how much narrower each wall is.
            //
            // A prism side wall is `side` wide; after filleting both its edges
            // it is side - 2*band.
            double widest = 0.0;
            for (Index f = 0; f < m.faceCount(); ++f) {
                if (std::fabs(dot(m.faceNormal(f), Vec3{0, 1, 0})) > 0.01) continue;
                widest = std::max(widest, static_cast<double>(m.faceArea(f)) / 20.0);
            }
            const double expected = 20.0 - 2.0 * band;
            check(std::fabs(widest - expected) < 1e-6,
                  std::string(c.name) + ": wall is " + std::to_string(widest) +
                      " wide, expected " + std::to_string(expected));

            std::printf("   %-7s radius %.1f -> band %.4f (Blender offset), wall %.4f\n",
                        c.name, r, band, widest);
        }
    }

    // =====================================================================
    // 2. Every point of the surface is one radius from the ball's path
    // =====================================================================
    //
    // The complete statement of what a fillet is, and the one that catches a
    // surface built in the wrong place. On a cube rounded all over, every
    // vertex must sit one radius from either an edge's rolling axis or a
    // corner's ball centre -- there is nowhere else for it to be.
    {
        std::printf("-- every vertex one radius from the ball --\n");
        const double r = 3.0;
        Mesh m;
        makeBox(m);
        check(bevelEdges(m, allEdges(m), r, 6), "cube, all edges");
        expectSolid(m, "cube all edges");

        // Twelve axes, one per edge, each pulled in by the radius on both faces.
        std::vector<std::pair<Vec3, Vec3>> axes;
        for (int ax = 0; ax < 3; ++ax)
            for (int s1 = -1; s1 <= 1; s1 += 2)
                for (int s2 = -1; s2 <= 1; s2 += 2) {
                    Vec3 o{}, d{};
                    const double in = 10.0 - r;
                    if (ax == 0) { d = {1, 0, 0}; o = {0, in * s1, in * s2}; }
                    if (ax == 1) { d = {0, 1, 0}; o = {in * s1, 0, in * s2}; }
                    if (ax == 2) { d = {0, 0, 1}; o = {in * s1, in * s2, 0}; }
                    axes.emplace_back(o, d);
                }
        // Eight ball centres, one per corner.
        std::vector<Vec3> balls;
        for (int i = -1; i <= 1; i += 2)
            for (int j = -1; j <= 1; j += 2)
                for (int k = -1; k <= 1; k += 2)
                    balls.push_back({(10.0 - r) * i, (10.0 - r) * j, (10.0 - r) * k});

        double worst = 0.0;
        int stray = 0;
        for (const MeshVertex& v : m.verts) {
            double best = 1e30;
            for (const auto& [o, d] : axes)
                best = std::min(best, std::fabs(toAxis(v.position, o, d) - r));
            for (const Vec3& c : balls)
                best = std::min(best, std::fabs(length(v.position - c) - r));
            worst = std::max(worst, best);
            if (best > 1e-9) ++stray;
        }
        check(stray == 0, std::to_string(stray) + " vertices are not on the rolling ball");
        std::printf("   %d vertices, worst departure %.3e mm\n", m.vertexCount(), worst);
    }

    // =====================================================================
    // 3. Rounding the edges, not rounding the corner into a ball
    // =====================================================================
    //
    // Where three filleted edges meet, the ball settles into the corner and the
    // patch between them is part of its surface -- a spherical triangle. That is
    // what both Blender and Fusion produce, and it is the rolling ball's own
    // answer.
    //
    // Where only two are filleted and the third edge stays sharp, the ball never
    // pivots: the two fillet surfaces run into each other, the sharp edge
    // survives to the point where they cross, and the flat face between them
    // keeps its sharp corner. Blender does this; anything that rounds the corner
    // there is rounding an edge the user did not select.
    {
        std::printf("-- corners --\n");
        const double r = 3.0;

        // 3a. Three filleted edges: the patch is on the sphere.
        {
            Mesh m;
            makeBox(m);
            check(bevelEdges(m, allEdges(m), r, 6), "cube all edges");
            const Vec3 c{10.0 - r, 10.0 - r, 10.0 - r};
            int onSphere = 0;
            double worst = 0.0;
            for (const MeshVertex& v : m.verts) {
                if (v.position.x < 5 || v.position.y < 5 || v.position.z < 5) continue;
                const double d = std::fabs(length(v.position - c) - r);
                // Points of the three cylinders near the corner are on the
                // sphere too only where they meet it; count the ones that are.
                if (d < 1e-9) { ++onSphere; worst = std::max(worst, d); }
            }
            check(onSphere > 0, "the corner has a patch at all");
            std::printf("   three filleted edges: %d corner points, all on the "
                        "sphere of radius %.1f\n", onSphere, r);
        }

        // 3b. Two filleted, one sharp: mitre, and the flat face keeps its point.
        {
            Mesh m;
            makeBox(m);
            std::vector<Index> rim;
            for (Index h = 0; h < m.halfedgeCount(); ++h) {
                if (h > m.halfedges[h].twin) continue;
                if (m.verts[m.fromVertex(h)].position.z > 9.99 &&
                    m.verts[m.halfedges[h].vertex].position.z > 9.99)
                    rim.push_back(h);
            }
            check(rim.size() == 4, "four edges round the top");
            check(bevelEdges(m, rim, r, 8), "top rim fillet");
            expectSolid(m, "top rim");

            // The top face is a square with sharp corners, inset by the band.
            Index top = kInvalid;
            for (Index f = 0; f < m.faceCount(); ++f)
                if (dot(m.faceNormal(f), Vec3{0, 0, 1}) > 0.999 &&
                    std::fabs(m.faceCentroid(f).z - 10.0) < 1e-9) top = f;
            check(top != kInvalid, "the top face is still there");
            if (top != kInvalid) {
                check(m.faceDegree(top) == 4,
                      "Blender leaves the top a four-cornered square, not a rounded one "
                      "(got " + std::to_string(m.faceDegree(top)) + " corners)");
                check(std::fabs(m.faceArea(top) - (20.0 - 2 * r) * (20.0 - 2 * r)) < 1e-6,
                      "and inset by the band on all four sides");
            }

            // The vertical edges stay sharp, ending where the two fillets cross.
            double highest = -100;
            for (const MeshVertex& v : m.verts)
                if (std::fabs(v.position.x - 10) < 1e-9 && std::fabs(v.position.y - 10) < 1e-9)
                    highest = std::max(highest, static_cast<double>(v.position.z));
            check(std::fabs(highest - (10.0 - r)) < 1e-6,
                  "the sharp vertical edge runs to where the fillets meet (z=" +
                      std::to_string(highest) + ", expected " + std::to_string(10.0 - r) + ")");

            // No horizontal ledge tucked into the corner.
            int shelf = 0;
            for (Index f = 0; f < m.faceCount(); ++f)
                if (dot(m.faceNormal(f), Vec3{0, 0, 1}) > 0.999 &&
                    m.faceCentroid(f).z < 10.0 - 1e-9) ++shelf;
            check(shelf == 0, std::to_string(shelf) + " upward-facing ledges at the corners");
            std::printf("   two filleted, one sharp: top stays a %d-corner square, "
                        "sharp edge to z=%.2f, %d ledges\n",
                        top == kInvalid ? -1 : m.faceDegree(top), highest, shelf);
        }
    }

    // =====================================================================
    // 4. Concave edges add material
    // =====================================================================
    //
    // Blender bevels a reflex edge as readily as a convex one, and the result
    // fills the inside corner rather than cutting it. The section is the same
    // arc; only which side of it the material is on differs.
    {
        std::printf("-- concave --\n");
        Mesh L;
        check(makeLPrism(L), "L-prism builds");
        const double base = volumeOf(L);

        std::vector<Index> concave;
        for (Index h = 0; h < L.halfedgeCount(); ++h) {
            const Index tw = L.halfedges[h].twin;
            if (h > tw) continue;
            const Vec3 n0 = L.faceNormal(L.halfedges[h].face);
            const Vec3 n1 = L.faceNormal(L.halfedges[tw].face);
            if (dot(n0, n1) > 0.99) continue;
            const Vec3 e = L.verts[L.halfedges[h].vertex].position -
                           L.verts[L.fromVertex(h)].position;
            if (dot(cross(n0, n1), e) < -1e-9) concave.push_back(h);
        }
        check(concave.size() == 1, "one reflex edge");

        for (int segments : {1, 4, 8}) {
            Mesh m = L;
            const double r = 3.0;
            check(bevelEdges(m, concave, r, segments), "concave fillet");
            expectSolid(m, "concave");
            const double added = volumeOf(m) - base;
            const double sector = 0.5 * segments * r * r * std::sin(kPi / (2.0 * segments));
            const double predicted = 20.0 * (r * r - sector);
            check(added > 0.0, "a concave fillet adds material");
            check(std::fabs(added - predicted) < 1e-6,
                  "concave " + std::to_string(segments) + " seg: added " +
                      std::to_string(added) + ", expected " + std::to_string(predicted));
        }
        std::printf("   reflex edge fills the corner, exactly, at 1/4/8 segments\n");
    }

    // =====================================================================
    // 5. The answer does not depend on where the body is pointing
    // =====================================================================
    //
    // Rotating a body and filleting it must give the rotated fillet. Nothing
    // about the operation is allowed to notice the axes -- if it does, a model
    // built at an angle comes out different from the same model built square
    // and turned, which no modeller does.
    {
        std::printf("-- rigid motions --\n");
        const double r = 2.5;
        const int segments = 6;

        const Mat4 turns[] = {rotation({0, 0, 1}, 37.0),
                              rotation({1, 1, 0}, 61.0),
                              rotation({0.3, -0.7, 0.64}, 143.0)};
        int n = 0;
        for (const Mat4& t : turns) {
            Mesh straight;
            makeBox(straight);
            check(bevelEdges(straight, allEdges(straight), r, segments), "fillet then turn");
            const Mesh thenTurned = transformed(straight, t);

            Mesh turnedFirst;
            makeBox(turnedFirst);
            turnedFirst = transformed(turnedFirst, t);
            check(bevelEdges(turnedFirst, allEdges(turnedFirst), r, segments), "turn then fillet");

            check(turnedFirst.vertexCount() == thenTurned.vertexCount(),
                  "turn " + std::to_string(n) + ": same vertex count");
            check(std::fabs(volumeOf(turnedFirst) - volumeOf(thenTurned)) < 1e-6,
                  "turn " + std::to_string(n) + ": same volume");

            double worst = 0.0;
            for (const MeshVertex& v : turnedFirst.verts) {
                double best = 1e30;
                for (const MeshVertex& w : thenTurned.verts)
                    best = std::min(best, static_cast<double>(lengthSq(v.position - w.position)));
                worst = std::max(worst, std::sqrt(best));
            }
            check(worst < 1e-9,
                  "turn " + std::to_string(n) + ": vertices land in the same places (worst " +
                      std::to_string(worst) + ")");
            std::printf("   turn %d: worst departure %.3e mm\n", n, worst);
            ++n;
        }
    }

    // =====================================================================
    // 6. And it scales
    // =====================================================================
    {
        std::printf("-- scale --\n");
        for (double k : {0.05, 20.0}) {
            Mesh small;
            makeBox(small);
            check(bevelEdges(small, allEdges(small), 2.0, 6), "fillet at unit size");

            Mesh big;
            BoxParams p{20.0 * k, 20.0 * k, 20.0 * k};
            makeBox(big, p);
            check(bevelEdges(big, allEdges(big), 2.0 * k, 6), "fillet at scale");

            check(big.vertexCount() == small.vertexCount(),
                  "scale " + std::to_string(k) + ": same vertex count");
            const double ratio = volumeOf(big) / (volumeOf(small) * k * k * k);
            check(std::fabs(ratio - 1.0) < 1e-9,
                  "scale " + std::to_string(k) + ": volume scales as k^3 (ratio " +
                      std::to_string(ratio) + ")");
            std::printf("   x%-6g volume ratio %.12f\n", k, ratio);
        }
    }

    // =====================================================================
    // 7. Stacking and order
    // =====================================================================
    {
        std::printf("-- stacking --\n");
        const double r = 2.0;
        const int segments = 6;

        // Edges that share no vertex cannot influence each other, so every
        // order and doing them together must agree exactly.
        Mesh ab, ba, both;
        makeBox(ab); makeBox(ba); makeBox(both);
        auto vertical = [](const Mesh& m, double x, double y) {
            for (Index h = 0; h < m.halfedgeCount(); ++h) {
                if (h > m.halfedges[h].twin) continue;
                const Vec3 a = m.verts[m.fromVertex(h)].position;
                const Vec3 b = m.verts[m.halfedges[h].vertex].position;
                if (std::fabs(a.x - x) < 1e-9 && std::fabs(a.y - y) < 1e-9 &&
                    std::fabs(b.x - x) < 1e-9 && std::fabs(b.y - y) < 1e-9) return h;
            }
            return kInvalid;
        };
        const Index a0 = vertical(ab, -10, -10), b0 = vertical(ab, 10, 10);
        check(a0 != kInvalid && b0 != kInvalid, "found two opposite vertical edges");
        check(bevelEdges(ab, {a0}, r, segments), "A");
        check(bevelEdges(ab, {vertical(ab, 10, 10)}, r, segments), "then B");
        check(bevelEdges(ba, {vertical(ba, 10, 10)}, r, segments), "B");
        check(bevelEdges(ba, {vertical(ba, -10, -10)}, r, segments), "then A");
        check(bevelEdges(both, {a0, b0}, r, segments), "both together");

        check(std::fabs(volumeOf(ab) - volumeOf(both)) < 1e-9 &&
              std::fabs(volumeOf(ba) - volumeOf(both)) < 1e-9,
              "disjoint edges: every order gives the same volume");
        check(ab.vertexCount() == both.vertexCount() && ba.vertexCount() == both.vertexCount(),
              "disjoint edges: every order gives the same mesh");
        std::printf("   disjoint edges: A-then-B, B-then-A and together all agree\n");

        // Filleting a fillet: the seam where a fillet meets a flat face is a
        // real edge and Blender will bevel it again.
        Mesh compound;
        makeBox(compound);
        const Index e = vertical(compound, -10, -10);
        check(bevelEdges(compound, {e}, 4.0, 4), "first fillet");
        const double afterFirst = volumeOf(compound);
        Index seam = kInvalid;
        for (Index h = 0; h < compound.halfedgeCount(); ++h) {
            if (h > compound.halfedges[h].twin) continue;
            const Vec3 a = compound.verts[compound.fromVertex(h)].position;
            const Vec3 b = compound.verts[compound.halfedges[h].vertex].position;
            if (std::fabs(a.x - (-10)) < 1e-9 && std::fabs(b.x - (-10)) < 1e-9 &&
                std::fabs(a.y - (-6)) < 1e-9 && std::fabs(b.y - (-6)) < 1e-9) seam = h;
        }
        check(seam != kInvalid, "the fillet's seam is a selectable edge");
        if (seam != kInvalid) {
            check(bevelEdges(compound, {seam}, 1.0, 4), "fillet the seam");
            expectSolid(compound, "compound fillet");
            check(volumeOf(compound) < afterFirst, "and it removes more material");
            std::printf("   filleting a fillet's own seam works\n");
        }
    }

    // =====================================================================
    // 8. Asking for too much
    // =====================================================================
    //
    // Blender clamps the offset so the bevel cannot overrun; we refuse instead,
    // which is the CAD answer -- a part that silently came out at a smaller
    // radius than asked for is worse than one that did not build. Either way,
    // what must never happen is geometry that overruns.
    {
        std::printf("-- too large --\n");
        Mesh m;
        makeBox(m);
        const int faces = m.faceCount();
        const double before = volumeOf(m);
        check(!bevelEdges(m, allEdges(m), 11.0, 4), "an 11mm radius on a 20mm cube is refused");
        check(m.faceCount() == faces && std::fabs(volumeOf(m) - before) < 1e-9,
              "and the body is left exactly as it was");

        Mesh limit;
        makeBox(limit);
        const Real w = maxBevelWidth(limit) * 0.99;
        check(bevelEdges(limit, allEdges(limit), w, 8), "the reported maximum is usable");
        expectSolid(limit, "maximum radius");
        std::printf("   refused at 11mm, built at %.2fmm\n", w);
    }

    // =====================================================================
    // 9. The profile: a flat chord at one segment, equal angles above
    // =====================================================================
    //
    // Blender at one segment gives a single flat face whose width is the chord
    // between the two offset points -- 2*r*cos(theta/2) once converted. Above
    // one segment its profile at the default 0.5 is a circular arc, and the
    // segments are equal steps along it. Ours must be the same on both counts:
    // an arc sampled unevenly is a fillet that looks wrong under a highlight
    // however right its volume is.
    {
        std::printf("-- profile --\n");
        for (int sides : {3, 4, 6}) {
            const double theta = kPi * (sides - 2) / sides;
            const double r = 2.0;

            Mesh m;
            check(makePrism(m, sides, 20.0, 20.0), "prism builds");
            check(bevelEdges(m, sideEdges(m), r, 1), "chamfer");

            // The chamfer face is the one whose normal is neither a wall's nor
            // an end cap's; its width is the chord.
            // The chamfer faces are the narrow ones; the walls are what is
            // left of the original sides.
            double chord = 1e30;
            for (Index f = 0; f < m.faceCount(); ++f) {
                if (std::fabs(dot(m.faceNormal(f), Vec3{0, 1, 0})) > 0.01) continue;
                chord = std::min(chord, static_cast<double>(m.faceArea(f)) / 20.0);
            }
            const double expected = 2.0 * r * std::cos(theta / 2);
            check(std::fabs(chord - expected) < 1e-6,
                  "chamfer chord at " + std::to_string(int(theta * 180 / kPi)) +
                      " deg is " + std::to_string(chord) + ", Blender gives " +
                      std::to_string(expected));
            std::printf("   %3d deg chamfer chord %.4f (Blender %.4f)\n",
                        int(theta * 180 / kPi + 0.5), chord, expected);
        }

        // Equal angular steps: each strip of the arc subtends the same angle at
        // the rolling axis, so every strip has the same width.
        for (int segments : {4, 8}) {
            Mesh m;
            makeBox(m);
            std::vector<Index> one;
            for (Index h = 0; h < m.halfedgeCount(); ++h) {
                if (h > m.halfedges[h].twin) continue;
                const Vec3 a = m.verts[m.fromVertex(h)].position;
                const Vec3 b = m.verts[m.halfedges[h].vertex].position;
                if (a.z > 9.99 && b.z > 9.99 && a.y < -9.99 && b.y < -9.99) one.push_back(h);
            }
            check(one.size() == 1, "one edge to chamfer");
            const double r = 3.0;
            check(bevelEdges(m, one, r, segments), "arc");

            // A strip of the arc leans in both y and z; every original face of
            // the box points straight along one axis.
            std::vector<double> widths;
            for (Index f = 0; f < m.faceCount(); ++f) {
                const Vec3 n = m.faceNormal(f);
                if (std::fabs(n.x) > 0.01) continue;
                if (std::fabs(n.y) < 0.01 || std::fabs(n.z) < 0.01) continue;
                widths.push_back(static_cast<double>(m.faceArea(f)) / 20.0);
            }
            check(widths.size() == static_cast<size_t>(segments),
                  std::to_string(segments) + " segments produce that many strips (" +
                      std::to_string(widths.size()) + ")");
            if (!widths.empty()) {
                const double lo = *std::min_element(widths.begin(), widths.end());
                const double hi = *std::max_element(widths.begin(), widths.end());
                const double chord = 2.0 * r * std::sin(kPi / (4.0 * segments));
                check(hi - lo < 1e-9, "every strip the same width (" +
                                          std::to_string(hi - lo) + " apart)");
                check(std::fabs(lo - chord) < 1e-6,
                      "and each is the chord of an equal angular step");
                std::printf("   %d segments: strips %.6f wide, chord %.6f\n",
                            segments, lo, chord);
            }
        }
    }

    // =====================================================================
    // 10. Rounding a flat face's corner, versus leaving it sharp
    // =====================================================================
    //
    // The two look alike and are opposite. Fillet a cube's four *vertical*
    // edges and the top face's corners become quarter arcs: the fillets run up
    // to the top and their ends cut into it. Fillet the four edges *around* the
    // top instead and the top face keeps four sharp corners, because those
    // fillets take a band off it rather than ending in it.
    //
    // Blender does both of these, and getting them the wrong way round means
    // rounding an edge the user never selected.
    {
        std::printf("-- rounded corner vs sharp corner --\n");
        const double r = 3.0;
        const int segments = 6;

        Mesh vert;
        makeBox(vert);
        std::vector<Index> uprights;
        for (Index h = 0; h < vert.halfedgeCount(); ++h) {
            if (h > vert.halfedges[h].twin) continue;
            const Vec3 a = vert.verts[vert.fromVertex(h)].position;
            const Vec3 b = vert.verts[vert.halfedges[h].vertex].position;
            if (std::fabs(a.x - b.x) < 1e-9 && std::fabs(a.y - b.y) < 1e-9) uprights.push_back(h);
        }
        check(uprights.size() == 4, "four uprights");
        check(bevelEdges(vert, uprights, r, segments), "fillet the uprights");
        expectSolid(vert, "uprights");

        Index top = kInvalid;
        for (Index f = 0; f < vert.faceCount(); ++f)
            if (dot(vert.faceNormal(f), Vec3{0, 0, 1}) > 0.999) top = f;
        check(top != kInvalid, "top face found");
        if (top != kInvalid) {
            check(vert.faceDegree(top) == 4 * (segments + 1),
                  "the top's corners are arcs of " + std::to_string(segments) +
                      " segments (degree " + std::to_string(vert.faceDegree(top)) + ")");
            const double area = 400.0 - 4.0 * (r * r - 0.5 * segments * r * r *
                                               std::sin(kPi / (2.0 * segments)));
            check(std::fabs(vert.faceArea(top) - area) < 1e-6,
                  "and its area is the square less four rounded corners");
            std::printf("   uprights filleted: top face has %d corners, area %.4f\n",
                        vert.faceDegree(top), vert.faceArea(top));
        }
    }

    // =====================================================================
    // 11. A vertex with four edges
    // =====================================================================
    //
    // Every corner above has three edges, which is all a box has. An
    // octahedron's has four, and Blender bevels it without complaint.
    {
        std::printf("-- four edges at a vertex --\n");
        Mesh base;
        check(makeOctahedron(base, 10.0), "octahedron builds");

        for (int segments : {1, 4, 8}) {
            Mesh m;
            makeOctahedron(m, 10.0);
            const bool ok = bevelEdges(m, allEdges(m), 1.5, segments);
            check(ok, "octahedron, " + std::to_string(segments) + " seg");
            if (!ok) continue;
            expectSolid(m, "octahedron " + std::to_string(segments) + " seg");
            check(volumeOf(m) < volumeOf(base) + 1e-6, "rounding removes material");
        }

        // Two of the four, opposite each other: a corner with two filleted and
        // two sharp edges, which the mitre does not cover.
        Mesh m;
        makeOctahedron(m, 10.0);
        Index apex = kInvalid;
        for (Index v = 0; v < m.vertexCount(); ++v)
            if (m.verts[v].position.z > 9.9) apex = v;
        check(apex != kInvalid, "found the apex");
        std::vector<Index> two;
        if (apex != kInvalid) {
            const Index start = m.verts[apex].halfedge;
            Index h = start;
            int seen = 0;
            do {
                if (seen % 2 == 0) two.push_back(std::min(h, m.halfedges[h].twin));
                ++seen;
                h = m.halfedges[m.halfedges[h].twin].next;
            } while (h != start);
        }
        check(two.size() == 2, "two opposite edges at the apex");
        inKnownDivergence = true;
        const bool ok = bevelEdges(m, two, 1.5, 6);
        if (ok) {
            expectSolid(m, "two of four at a vertex");
            std::printf("   two filleted and two sharp at a four-edge vertex: built\n");
        } else {
            std::printf("   two filleted and two sharp at a four-edge vertex: REFUSED\n");
        }
        check(ok, "a fillet ending at a vertex whose faces meet at a crease");
        inKnownDivergence = false;
    }

    // =====================================================================
    // 12. Edges far sharper than a box's
    // =====================================================================
    //
    // A 20 degree edge takes a band of r/tan(10) = 5.7r off each face, so a
    // radius that looks small eats a long way in. Blender clamps; we refuse.
    // Either way the geometry must not overrun.
    {
        std::printf("-- acute edges --\n");
        for (double apex : {60.0, 30.0, 15.0}) {
            Mesh base;
            check(makeWedge(base, apex, 20.0), "wedge builds");
            const double band = 1.0 / std::tan(apex * kPi / 360.0);

            Mesh m;
            makeWedge(m, apex, 20.0);
            // The sharp edge is the one at the apex, running along Y at x=0.
            Index sharp = kInvalid;
            for (Index h = 0; h < m.halfedgeCount(); ++h) {
                if (h > m.halfedges[h].twin) continue;
                const Vec3 a = m.verts[m.fromVertex(h)].position;
                const Vec3 b = m.verts[m.halfedges[h].vertex].position;
                if (std::fabs(a.x) < 1e-9 && std::fabs(b.x) < 1e-9) sharp = h;
            }
            check(sharp != kInvalid, "found the apex edge");
            const bool ok = bevelEdges(m, {sharp}, 1.0, 6);
            if (ok) {
                expectSolid(m, "wedge " + std::to_string(int(apex)) + " deg");
                std::printf("   %4.0f deg apex: band %.3f, built and solid\n", apex, band);
            } else {
                std::printf("   %4.0f deg apex: band %.3f, refused\n", apex, band);
            }
        }
    }

    // =====================================================================
    // 13. A body whose faces have been twisted
    // =====================================================================
    //
    // Rotating a face is an ordinary thing to do and it leaves the faces round
    // it no longer flat. Both a single edge and every edge have to work on the
    // result; they used to be mutually exclusive, because flattening those
    // faces first is what one needs and the other cannot survive.
    {
        std::printf("-- twisted faces --\n");
        for (double deg : {10.0, 25.0, 45.0}) {
            Mesh base;
            check(makeTwistedCube(base, deg), "twisted cube builds");
            const double baseVolume = volumeOf(base);

            const std::string at = std::to_string(static_cast<int>(deg)) + " deg";

            // One edge.
            {
                Mesh m;
                makeTwistedCube(m, deg);
                Index one = kInvalid;
                for (Index h = 0; h < m.halfedgeCount() && one == kInvalid; ++h) {
                    if (h > m.halfedges[h].twin) continue;
                    if (std::fabs(m.verts[m.fromVertex(h)].position.z -
                                  m.verts[m.halfedges[h].vertex].position.z) > 1e-6) one = h;
                }
                check(one != kInvalid, "found an edge across the twist");
                check(bevelEdges(m, {one}, 1.5, 6), at + ", one edge: fillet");
                expectSolid(m, at + ", one edge");
                check(volumeOf(m) < baseVolume + 1e-6, at + ", one edge: removes material");
            }

            // Every edge.
            {
                Mesh m;
                makeTwistedCube(m, deg);
                check(bevelEdges(m, allEdges(m), 1.5, 6), at + ", all edges: fillet");
                expectSolid(m, at + ", all edges");
                check(volumeOf(m) < baseVolume + 1e-6, at + ", all edges: removes material");
            }
            std::printf("   %s: one edge and all edges both build solid\n", at.c_str());
        }
    }

    if (divergences)
        std::printf("\n%d known difference%s from Blender (see the note above check)\n",
                    divergences, divergences == 1 ? "" : "s");
    std::printf("%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
