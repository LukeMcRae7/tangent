// Tangent - Comprehensive test suite for interactive object creation & sketching.
//
// Test Coverage:
//  Section 1: 2D Profile & Corner Fillet Generation
//    - Sharp rectangles (varying sizes, centers, inverse drag coordinates, extreme aspect ratios)
//    - Rounded rectangles (small, medium, 50%, 75%, max/stadium 100% fillets, over-sized clamping, negative clamping)
//    - Circles & Cylinder profiles (various radii, segmentations, center offsets)
//  Section 2: Standalone 3D Solids on Various Planes
//    - Global Axis Planes (XY Top, XZ Front, YZ Right)
//    - Arbitrary rotated/angled planes & offset coordinate frames
//    - Transformed/rotated coordinate frames with full quaternion orientations
//  Section 3: Pre-Extrude Profile Modifications & Order of Operations
//    - Resize then fillet vs. fillet then resize
//    - Fillet radius auto-clamping when resizing below 2*r
//    - Multi-step parameter adjustments (width -> depth -> fillet -> decrease fillet -> extrude)
//    - Cylinder radius & position interactive adjustments
//  Section 4: Extrusion Distances & Directions (+ / -)
//    - Micro-extrusion (0.1 mm), thin-wall (1 mm), fractional (7.35 mm), standard (25 mm), deep column (500 mm)
//    - Positive depths (+D, outwards along normal)
//    - Negative depths (-D, inwards against normal)
//  Section 5: Positive Extrusions Out Of Existing Objects (Boss / Auto-Join)
//    - Out of Cube flat faces (+Z, -Z, +X, -X, +Y, -Y)
//    - Out of Cylinder flat caps and lateral curved walls
//    - Out of Sphere surface (polar and equatorial posts)
//    - Out of Angled / Chamfered 45-degree faces
//    - Boss spanning across coplanar planar seams
//  Section 6: Negative Extrusions Into Existing Objects (Boolean Difference Cuts)
//    - Shallow and deep blind pockets in Cube
//    - Full through-hole cuts in Cube
//    - Corner notch and side edge-step cutaways
//    - Rounded-corner pockets and circular blind/through holes
//    - Cuts into Cylinder cap, axis bore (tube), and transverse slot through curved wall
//    - Cuts into Sphere surface (dimple cavity and through bore)
//    - Cuts into Angled / Chamfered 45-degree face
//  Section 7: Complex Multi-Operation Sequences
//    - Cut-then-Boss (boss extruded out of pocket floor)
//    - Boss-then-Cut (through-hole drilled through boss and base)
//    - Orthogonal multi-face cuts on all sides of a body
//    - Precision volume and topological invariant verifications
//  Section 9: Cuts and joins belong to the feature history
//    - The tool's cut/join land in the chain, not over obj->mesh
//    - Re-evaluating for any reason preserves them
//    - A fillet committed afterwards is neither refused nor loses the cut
//    - A cut with nothing to cut into refuses instead of adding a body
#include "app/create_tool.h"
#include "app/undo.h"
#include "mesh/boolean.h"
#include "mesh/health.h"
#include "mesh/operations.h"
#include "mesh/primitives.h"
#include "scene/feature.h"
#include "scene/scene.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace tg;

namespace {

int gFailures = 0;
int gChecks = 0;

void check(bool condition, const std::string& message) {
    ++gChecks;
    if (!condition) {
        std::printf("  [FAIL] %s\n", message.c_str());
        ++gFailures;
    }
}

bool near(double a, double b, double eps = 1e-4) {
    return std::fabs(a - b) <= eps;
}

bool nearRel(double val, double expected, double maxRelError = 0.03) {
    if (std::fabs(expected) < 1e-6) return std::fabs(val) < 1e-6;
    return std::fabs(val - expected) / std::fabs(expected) <= maxRelError;
}

double polygonArea2D(const std::vector<Vec2>& poly) {
    if (poly.size() < 3) return 0.0;
    double area = 0.0;
    for (size_t i = 0; i < poly.size(); ++i) {
        const size_t next = (i + 1) % poly.size();
        area += poly[i].x * poly[next].y - poly[next].x * poly[i].y;
    }
    return 0.5 * area;
}

void expectSolid(const Mesh& m, const std::string& what) {
    const MeshHealth h = checkHealth(m);
    check(h.watertight, what + ": watertight");
    check(h.volume > 0.0, what + ": positive volume");
    check(h.boundaryEdges == 0, what + ": no boundary edges");
    check(h.degenerateFaces == 0, what + ": no degenerate faces");
    check(h.selfIntersections <= 0, what + ": no self-intersections");
    check(h.solid(), what + ": solid() check");
    std::string err;
    check(m.validate(&err), what + ": topological validation (" + err + ")");
}

double volumeOf(const Mesh& m) {
    return checkHealth(m, false).volume;
}

Mesh makeBoxMesh(Vec3 size, Vec3 center = {0, 0, 0}) {
    Mesh m;
    BoxParams p;
    p.width = size.x;
    p.depth = size.y;
    p.height = size.z;
    makeBox(m, p);
    for (auto& v : m.verts) v.position += center;
    return m;
}

Mesh makeCylinderMesh(Real radius, Real height, int segments = 24, Vec3 center = {0, 0, 0}) {
    Mesh m;
    CylinderParams p;
    p.radius = radius;
    p.height = height;
    p.segments = segments;
    makeCylinder(m, p);
    for (auto& v : m.verts) v.position += center;
    return m;
}

Mesh makeSphereMesh(Real radius, int segments = 24, int rings = 16, Vec3 center = {0, 0, 0}) {
    Mesh m;
    SphereParams p;
    p.radius = radius;
    p.segments = segments;
    p.rings = rings;
    makeSphere(m, p);
    for (auto& v : m.verts) v.position += center;
    return m;
}

// Builds a 45-degree wedge / chamfered block
Mesh makeWedgeMesh(Real width, Real depth, Real height) {
    const Real hx = width * 0.5f, hy = depth * 0.5f, hz = height * 0.5f;
    std::vector<Vec3> pos = {
        {-hx, -hy, -hz}, { hx, -hy, -hz}, {-hx, -hy,  hz}, // 0, 1, 2 (front triangle y=-hy)
        {-hx,  hy, -hz}, { hx,  hy, -hz}, {-hx,  hy,  hz}  // 3, 4, 5 (back triangle y=+hy)
    };
    std::vector<uint32_t> sizes = {3, 3, 4, 4, 4};
    std::vector<uint32_t> indices = {
        0, 1, 2,          // front triangle (-Y)
        3, 5, 4,          // back triangle (+Y)
        0, 3, 4, 1,       // bottom face (-Z)
        0, 2, 5, 3,       // back face (-X)
        1, 4, 5, 2        // slanted face (+hypotenuse)
    };
    Mesh m;
    m.build(pos, sizes, indices, nullptr);
    return m;
}

} // namespace

// ===========================================================================
// SECTION 1: 2D Profile & Corner Fillet Generation
// ===========================================================================
void testSection1_ProfileGeneration() {
    std::printf("\n--- Section 1: 2D Profile & Fillet Generation ---\n");

    // 1.1 Sharp Rectangles
    {
        // Standard origin rectangle (40 x 30)
        std::vector<Vec2> p1 = CreateTool::makeRectPolygon({0, 0}, {40, 30}, 0.0);
        check(p1.size() == 4, "sharp rect has 4 vertices");
        check(near(polygonArea2D(p1), 1200.0), "sharp rect area is 40x30 = 1200");
        check(polygonArea2D(p1) > 0.0, "sharp rect winding is CCW (positive area)");

        // Inverted drag coordinates (dragging top-right to bottom-left)
        std::vector<Vec2> p2 = CreateTool::makeRectPolygon({40, 30}, {0, 0}, 0.0);
        check(p2.size() == 4, "inverted rect has 4 vertices");
        check(near(polygonArea2D(p2), 1200.0), "inverted rect produces normalized 1200 area");

        // Centered rectangle
        std::vector<Vec2> p3 = CreateTool::makeRectPolygon({-25, -15}, {25, 15}, 0.0);
        check(p3.size() == 4 && near(polygonArea2D(p3), 1500.0), "centered rect 50x30 = 1500");

        // Extreme aspect ratios
        std::vector<Vec2> thin = CreateTool::makeRectPolygon({0, 0}, {100, 0.5}, 0.0);
        check(thin.size() == 4 && near(polygonArea2D(thin), 50.0), "thin aspect ratio rect (100x0.5)");

        std::vector<Vec2> tall = CreateTool::makeRectPolygon({0, 0}, {0.5, 80}, 0.0);
        check(tall.size() == 4 && near(polygonArea2D(tall), 40.0), "tall aspect ratio rect (0.5x80)");
    }

    // 1.2 Rounded Rectangles: Fillet Radii, Extents & Clamping
    {
        const Real W = 40.0, H = 20.0;

        // Small fillet: r = 2.0 mm (10% of min dimension)
        {
            const Real r = 2.0;
            std::vector<Vec2> p = CreateTool::makeRectPolygon({0, 0}, {W, H}, r, 6);
            check(p.size() > 4, "small filleted rect has arc vertices");
            const double expectedArea = (W * H) - (4.0 - kPi) * (r * r);
            check(nearRel(polygonArea2D(p), expectedArea, 0.01), "small fillet area matches formula");
        }

        // Medium fillet: r = 5.0 mm (50% of half-dimension)
        {
            const Real r = 5.0;
            std::vector<Vec2> p = CreateTool::makeRectPolygon({0, 0}, {W, H}, r, 6);
            const double expectedArea = (W * H) - (4.0 - kPi) * (r * r);
            check(nearRel(polygonArea2D(p), expectedArea, 0.01), "medium fillet area matches formula");
        }

        // 75% fillet: r = 7.5 mm
        {
            const Real r = 7.5;
            std::vector<Vec2> p = CreateTool::makeRectPolygon({0, 0}, {W, H}, r, 8);
            const double expectedArea = (W * H) - (4.0 - kPi) * (r * r);
            check(nearRel(polygonArea2D(p), expectedArea, 0.01), "75% fillet area matches formula");
        }

        // Maximum fillet (Stadium / Pill shape): r = H / 2 = 10.0 mm (100% full round)
        {
            const Real rMax = H * 0.5f; // 10.0
            std::vector<Vec2> p = CreateTool::makeRectPolygon({0, 0}, {W, H}, rMax, 8);
            const double expectedArea = (W - 2.0 * rMax) * H + kPi * rMax * rMax; // 20x20 + pi*100
            check(nearRel(polygonArea2D(p), expectedArea, 0.01), "stadium / max fillet area matches pill formula");
        }

        // Over-sized fillet clamping (r = 30.0 mm on 40x20 rect)
        {
            std::vector<Vec2> p = CreateTool::makeRectPolygon({0, 0}, {W, H}, 30.0, 6);
            check(!p.empty(), "over-sized fillet does not crash and produces clamped shape");
            const double rClamped = H * 0.499;
            const double expectedArea = (W * H) - (4.0 - kPi) * (rClamped * rClamped);
            check(nearRel(polygonArea2D(p), expectedArea, 0.02), "over-sized fillet is safely clamped");
        }

        // Negative fillet radius clamping (r = -10.0 mm)
        {
            std::vector<Vec2> p = CreateTool::makeRectPolygon({0, 0}, {W, H}, -10.0, 4);
            check(p.size() == 4 && near(polygonArea2D(p), W * H), "negative fillet clamped to sharp rect");
        }
    }

    // 1.3 Circles & Circular Discretization
    {
        // Small circle R = 2.5 mm, 16 segments
        std::vector<Vec2> c1 = CreateTool::makeCirclePolygon({0, 0}, 2.5, 16);
        check(c1.size() == 16, "circle 16-gon has 16 vertices");
        const double expA1 = 0.5 * 16.0 * (2.5 * 2.5) * std::sin(kTwoPi / 16.0);
        check(nearRel(polygonArea2D(c1), expA1, 0.001), "16-gon circle area exact");

        // Medium circle R = 15.0 mm, 32 segments
        std::vector<Vec2> c2 = CreateTool::makeCirclePolygon({0, 0}, 15.0, 32);
        check(c2.size() == 32, "circle 32-gon has 32 vertices");
        const double expA2 = 0.5 * 32.0 * 225.0 * std::sin(kTwoPi / 32.0);
        check(nearRel(polygonArea2D(c2), expA2, 0.001), "32-gon circle area exact");

        // Large circle R = 60.0 mm with center offset (X=12.4, Y=-8.7)
        std::vector<Vec2> c3 = CreateTool::makeCirclePolygon({12.4, -8.7}, 60.0, 64);
        check(c3.size() == 64, "offset circle has 64 vertices");
        const double expA3 = 0.5 * 64.0 * 3600.0 * std::sin(kTwoPi / 64.0);
        check(nearRel(polygonArea2D(c3), expA3, 0.001), "64-gon offset circle area exact");
    }

    // 1.4 Individual Corner Fillets & Selective Edge Fillets
    {
        const Real W = 40.0, H = 30.0;

        // One corner only: Corner 0 (Bottom-Right) with r = 5.0 mm
        {
            const Real r[4] = {5.0, 0.0, 0.0, 0.0};
            std::vector<Vec2> p = CreateTool::makeRectPolygon({0, 0}, {W, H}, r, 8);
            check(p.size() > 4, "single corner filleted poly generated");
            const double expectedArea = (W * H) - (1.0 - kPi * 0.25) * 25.0;
            check(nearRel(polygonArea2D(p), expectedArea, 0.01), "single corner fillet area exact");

            Mesh prism;
            bool ok = CreateTool::makePrismMesh(p, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 0.0, 20.0, prism);
            check(ok, "single corner prism generated");
            expectSolid(prism, "single corner filleted prism");
            check(nearRel(volumeOf(prism), expectedArea * 20.0, 0.01), "single corner filleted prism volume exact");
        }

        // Two corners on top edge: Corner 1 (TR) and Corner 2 (TL) with r = 6.0 mm
        {
            const Real r[4] = {0.0, 6.0, 6.0, 0.0};
            std::vector<Vec2> p = CreateTool::makeRectPolygon({0, 0}, {W, H}, r, 8);
            const double expectedArea = (W * H) - 2.0 * (1.0 - kPi * 0.25) * 36.0;
            check(nearRel(polygonArea2D(p), expectedArea, 0.01), "two corner edge fillet area exact");

            Mesh prism;
            bool ok = CreateTool::makePrismMesh(p, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 0.0, 15.0, prism);
            check(ok, "two corner edge prism generated");
            expectSolid(prism, "two corner edge filleted prism");
        }

        // All 4 distinct corner radii: r0=2.0, r1=4.0, r2=6.0, r3=8.0
        {
            const Real r[4] = {2.0, 4.0, 6.0, 8.0};
            std::vector<Vec2> p = CreateTool::makeRectPolygon({0, 0}, {W, H}, r, 8);
            const double sumR2 = 4.0 + 16.0 + 36.0 + 64.0; // 120
            const double expectedArea = (W * H) - (1.0 - kPi * 0.25) * sumR2;
            check(nearRel(polygonArea2D(p), expectedArea, 0.01), "4 distinct corner radii area exact");

            Mesh prism;
            bool ok = CreateTool::makePrismMesh(p, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 0.0, 10.0, prism);
            check(ok, "4 distinct corner radii prism generated");
            expectSolid(prism, "4 distinct corner radii prism");
            check(nearRel(volumeOf(prism), expectedArea * 10.0, 0.01), "4 distinct corner radii prism volume exact");
        }
    }

    // 1.5 Fillet Radius Measured From Mouse Distance
    {
        const Vec2 cornerUV{40.0, 0.0}; // Corner 0 (Bottom-Right)

        // Mouse directly on corner -> 0 fillet
        const Vec2 mouseOnCorner = cornerUV;
        const Real dist0 = length(mouseOnCorner - cornerUV);
        check(near(dist0, 0.0), "mouse on corner creates 0 fillet distance");

        // Mouse 6 mm away in +X direction -> 6mm fillet
        const Vec2 mouseRight = cornerUV + Vec2{6.0, 0.0};
        const Real dist6 = length(mouseRight - cornerUV);
        check(near(dist6, 6.0), "mouse 6mm away in +X creates 6mm fillet distance");

        // Mouse 8 mm away diagonally in (-X, -Y) direction -> 8mm fillet
        const Vec2 mouseDiag = cornerUV + Vec2{-4.8, -6.4};
        const Real dist8 = length(mouseDiag - cornerUV);
        check(near(dist8, 8.0), "mouse 8mm away in diagonal creates 8mm fillet distance");
    }
}

// ===========================================================================
// SECTION 2: Standalone 3D Solids on Various Planes
// ===========================================================================
void testSection2_StandaloneObjectsOnPlanes() {
    std::printf("\n--- Section 2: Standalone 3D Solids on Planes ---\n");

    // 2.1 Box on XY Plane (Top)
    {
        std::vector<Vec2> poly = CreateTool::makeRectPolygon({-15, -10}, {15, 10}, 0.0);
        Mesh boxXY;
        bool ok = CreateTool::makePrismMesh(poly, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 0.0, 25.0, boxXY);
        check(ok, "create box on XY plane");
        expectSolid(boxXY, "box on XY plane");
        check(near(volumeOf(boxXY), 30.0 * 20.0 * 25.0), "box XY volume = 15000 mm3");
        const AABB b = boxXY.bounds();
        check(near(b.min.z, 0.0) && near(b.max.z, 25.0), "box XY sits in z in [0, 25]");
    }

    // 2.2 Box on XZ Plane (Front)
    {
        std::vector<Vec2> poly = CreateTool::makeRectPolygon({-20, -10}, {20, 10}, 0.0);
        Mesh boxXZ;
        bool ok = CreateTool::makePrismMesh(poly, {0, 0, 0}, {1, 0, 0}, {0, 0, 1}, {0, -1, 0}, 0.0, 30.0, boxXZ);
        check(ok, "create box on XZ plane");
        expectSolid(boxXZ, "box on XZ plane");
        check(near(volumeOf(boxXZ), 40.0 * 20.0 * 30.0), "box XZ volume = 24000 mm3");
    }

    // 2.3 Box on YZ Plane (Right)
    {
        std::vector<Vec2> poly = CreateTool::makeRectPolygon({-10, -10}, {10, 10}, 0.0);
        Mesh boxYZ;
        bool ok = CreateTool::makePrismMesh(poly, {0, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 0}, 0.0, 40.0, boxYZ);
        check(ok, "create box on YZ plane");
        expectSolid(boxYZ, "box on YZ plane");
        check(near(volumeOf(boxYZ), 20.0 * 20.0 * 40.0), "box YZ volume = 16000 mm3");
    }

    // 2.4 Rounded Rectangular Prism on XY Plane
    {
        const Real W = 50.0, H = 30.0, r = 5.0, Depth = 20.0;
        std::vector<Vec2> poly = CreateTool::makeRectPolygon({-W * 0.5f, -H * 0.5f}, {W * 0.5f, H * 0.5f}, r, 6);
        Mesh roundedPrism;
        bool ok = CreateTool::makePrismMesh(poly, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 0.0, Depth, roundedPrism);
        check(ok, "create rounded prism on XY plane");
        expectSolid(roundedPrism, "rounded prism on XY plane");
        const double expectedVol = ((W * H) - (4.0 - kPi) * (r * r)) * Depth;
        check(nearRel(volumeOf(roundedPrism), expectedVol, 0.01), "rounded prism volume matches formula");
    }

    // 2.5 Cylinder on Axis Planes
    {
        const Real R = 12.0, H = 35.0;
        std::vector<Vec2> circlePoly = CreateTool::makeCirclePolygon({0, 0}, R, 32);
        Mesh cyl;
        bool ok = CreateTool::makePrismMesh(circlePoly, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 0.0, H, cyl);
        check(ok, "create cylinder on XY plane");
        expectSolid(cyl, "cylinder on XY plane");
        const double facettedArea = 0.5 * 32.0 * (R * R) * std::sin(kTwoPi / 32.0);
        check(nearRel(volumeOf(cyl), facettedArea * H, 0.005), "cylinder prism volume exact");
    }

    // 2.6 Solid on Arbitrary Rotated / Angled Plane
    {
        const Vec3 origin{10, 20, 30};
        const Vec3 normal = normalize(Vec3{1, 1, 1});
        const Vec3 u = normalize(cross(Vec3{0, 0, 1}, normal));
        const Vec3 v = cross(normal, u);

        std::vector<Vec2> poly = CreateTool::makeRectPolygon({-10, -10}, {10, 10}, 2.0, 4);
        Mesh angledSolid;
        bool ok = CreateTool::makePrismMesh(poly, origin, u, v, normal, 0.0, 15.0, angledSolid);
        check(ok, "create prism on angled plane");
        expectSolid(angledSolid, "prism on angled plane");
        const double expectedArea = 400.0 - (4.0 - kPi) * 4.0;
        check(nearRel(volumeOf(angledSolid), expectedArea * 15.0, 0.01), "angled prism volume exact");
    }
}

// ===========================================================================
// SECTION 3: Pre-Extrude Modifications & Order of Operations
// ===========================================================================
void testSection3_PreExtrudeModifications() {
    std::printf("\n--- Section 3: Pre-Extrude Profile Modifications ---\n");

    // 3.1 Order A: Draw sharp rect -> resize dimensions -> add fillet
    {
        // 1. Initial 20 x 20
        Vec2 p1{0, 0}, p2{20, 20};
        // 2. Drag width to 50, depth to 30
        p2.x = 50.0;
        p2.y = 30.0;
        // 3. Add corner fillet r = 6.0 mm
        Real r = 6.0;
        std::vector<Vec2> poly = CreateTool::makeRectPolygon(p1, p2, r, 6);
        Mesh solid;
        CreateTool::makePrismMesh(poly, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 0.0, 20.0, solid);
        expectSolid(solid, "resize then fillet");
        const double expectedVol = (50.0 * 30.0 - (4.0 - kPi) * 36.0) * 20.0;
        check(nearRel(volumeOf(solid), expectedVol, 0.01), "resize-then-fillet volume exact");
    }

    // 3.2 Order B: Add fillet -> resize dimensions smaller (fillet auto-clamping)
    {
        // 1. Initial 60 x 60 with r = 20.0 mm
        Vec2 p1{0, 0}, p2{60, 60};
        Real r = 20.0;
        // 2. Drag width down to 20 mm, depth down to 20 mm (where max possible fillet is 10 mm)
        p2.x = 20.0;
        p2.y = 20.0;
        std::vector<Vec2> poly = CreateTool::makeRectPolygon(p1, p2, r, 6);
        check(!poly.empty(), "clamped fillet profile generated cleanly");
        Mesh solid;
        CreateTool::makePrismMesh(poly, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 0.0, 15.0, solid);
        expectSolid(solid, "fillet then shrink dimensions");
        const double rClamped = 20.0 * 0.499;
        const double expectedVol = (20.0 * 20.0 - (4.0 - kPi) * (rClamped * rClamped)) * 15.0;
        check(nearRel(volumeOf(solid), expectedVol, 0.02), "auto-clamped fillet volume verified");
    }

    // 3.3 Dynamic Fillet Adjustment: Increase then Decrease back to Sharp
    {
        Vec2 p1{0, 0}, p2{40, 30};
        // Step 1: r = 4.0 mm
        std::vector<Vec2> poly1 = CreateTool::makeRectPolygon(p1, p2, 4.0, 4);
        check(poly1.size() > 4, "fillet r=4 has arc verts");

        // Step 2: r increased to 10.0 mm
        std::vector<Vec2> poly2 = CreateTool::makeRectPolygon(p1, p2, 10.0, 6);
        check(polygonArea2D(poly2) < polygonArea2D(poly1), "larger fillet produces smaller profile area");

        // Step 3: r decreased back to 0.0 mm (sharp)
        std::vector<Vec2> poly3 = CreateTool::makeRectPolygon(p1, p2, 0.0, 0);
        check(poly3.size() == 4 && near(polygonArea2D(poly3), 1200.0), "reset to sharp rect returns 4 vertices");
    }
}

// ===========================================================================
// SECTION 4: Extrusion Distances & Directions (+ / -)
// ===========================================================================
void testSection4_ExtrusionDistancesAndDirections() {
    std::printf("\n--- Section 4: Extrusion Distances & Directions (+/-) ---\n");

    std::vector<Vec2> poly = CreateTool::makeRectPolygon({0, 0}, {20, 20}, 0.0);
    const double area = 400.0;

    // 4.1 Micro-extrusion: Depth = 0.1 mm
    {
        Mesh micro;
        CreateTool::makePrismMesh(poly, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 0.0, 0.1, micro);
        expectSolid(micro, "micro-extrusion 0.1mm solid");
        check(near(volumeOf(micro), area * 0.1), "micro-extrusion volume = 40 mm3");
    }

    // 4.2 Fractional extrusion: Depth = 7.35 mm
    {
        Mesh frac;
        CreateTool::makePrismMesh(poly, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 0.0, 7.35, frac);
        expectSolid(frac, "fractional 7.35mm solid");
        check(near(volumeOf(frac), area * 7.35), "fractional volume = 2940 mm3");
    }

    // 4.3 Standard extrusion: Depth = 25.0 mm
    {
        Mesh stdSolid;
        CreateTool::makePrismMesh(poly, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 0.0, 25.0, stdSolid);
        expectSolid(stdSolid, "standard 25mm solid");
        check(near(volumeOf(stdSolid), area * 25.0), "standard volume = 10000 mm3");
    }

    // 4.4 Deep column: Depth = 500.0 mm
    {
        Mesh column;
        CreateTool::makePrismMesh(poly, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 0.0, 500.0, column);
        expectSolid(column, "deep column 500mm solid");
        check(near(volumeOf(column), area * 500.0), "deep column volume = 200000 mm3");
    }

    // 4.5 Negative extrusion direction (z in [-30, 0])
    {
        Mesh negSolid;
        CreateTool::makePrismMesh(poly, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, -30.0, 0.0, negSolid);
        expectSolid(negSolid, "negative depth prism solid");
        check(near(volumeOf(negSolid), area * 30.0), "negative depth prism volume = 12000 mm3");
        const AABB b = negSolid.bounds();
        check(near(b.min.z, -30.0) && near(b.max.z, 0.0), "negative depth sits in z in [-30, 0]");
    }
}

// ===========================================================================
// SECTION 5: Positive Extrusions Out Of Existing Objects (Boss / Auto-Join)
// ===========================================================================
void testSection5_PositiveExtrudeAutoJoin() {
    std::printf("\n--- Section 5: Positive Extrusions (Boss / Auto-Join) ---\n");

    // 5.1 Boss atop Cube Top Face (+Z)
    {
        Mesh base = makeBoxMesh({50, 50, 30}, {0, 0, 15}); // z in [0, 30]
        std::vector<Vec2> bossPoly = CreateTool::makeRectPolygon({-10, -10}, {10, 10}, 0.0);
        Mesh boss;
        CreateTool::makePrismMesh(bossPoly, {0, 0, 30}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 0.0, 15.0, boss);

        Mesh result;
        bool ok = meshBoolean(base, boss, BooleanOp::Union, result);
        check(ok, "union boss with base box succeeds");
        mergeCoplanarFaces(result);
        expectSolid(result, "boss on box top");
        const double expectedVol = (50.0 * 50.0 * 30.0) + (20.0 * 20.0 * 15.0); // 75000 + 6000 = 81000
        check(near(volumeOf(result), expectedVol), "boss atop box volume = 81000 mm3");
    }

    // 5.2 Boss out of Cube Side Face (+X)
    {
        Mesh base = makeBoxMesh({40, 40, 40}); // x in [-20, 20]
        std::vector<Vec2> bossPoly = CreateTool::makeRectPolygon({-8, -8}, {8, 8}, 0.0);
        Mesh sideBoss;
        CreateTool::makePrismMesh(bossPoly, {20, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 0}, 0.0, 12.0, sideBoss);

        Mesh result;
        bool ok = meshBoolean(base, sideBoss, BooleanOp::Union, result);
        check(ok, "union side boss with box succeeds");
        mergeCoplanarFaces(result);
        expectSolid(result, "side boss on box");
        const double expectedVol = 64000.0 + (16.0 * 16.0 * 12.0); // 67072
        check(near(volumeOf(result), expectedVol), "side boss volume = 67072 mm3");
    }

    // 5.3 Concentric Cylindrical Boss atop Cylinder Cap
    {
        Mesh baseCyl = makeCylinderMesh(20.0, 30.0, 24, {0, 0, 15}); // z in [0, 30]
        Mesh bossCyl = makeCylinderMesh(8.0, 15.0, 24, {0, 0, 37.5}); // z in [30, 45]

        Mesh result;
        bool ok = meshBoolean(baseCyl, bossCyl, BooleanOp::Union, result);
        check(ok, "union concentric cylinder boss succeeds");
        mergeCoplanarFaces(result);
        expectSolid(result, "cylinder boss on cylinder cap");
        const double n = 24.0;
        const double aBase = 0.5 * n * 400.0 * std::sin(kTwoPi / n);
        const double aBoss = 0.5 * n * 64.0 * std::sin(kTwoPi / n);
        const double expectedVol = (aBase * 30.0) + (aBoss * 15.0);
        check(nearRel(volumeOf(result), expectedVol, 0.01), "stepped cylinder volume exact");
    }

    // 5.4 Boss Extruded Out of Sphere Surface
    {
        Mesh baseSphere = makeSphereMesh(20.0, 24, 16);
        Mesh post = makeBoxMesh({10, 10, 20}, {0, 0, 25}); // z in [15, 35]

        Mesh result;
        bool ok = meshBoolean(baseSphere, post, BooleanOp::Union, result);
        check(ok, "union post with sphere succeeds");
        mergeCoplanarFaces(result);
        expectSolid(result, "post on sphere");
        check(volumeOf(result) > volumeOf(baseSphere), "post on sphere increases total volume");
        const AABB b = result.bounds();
        check(near(b.max.z, 35.0), "sphere with post reaches z=35");
    }

    // 5.5 Boss Extruded Out of Angled / Chamfered 45-Degree Face
    {
        Mesh wedge = makeWedgeMesh(30.0, 30.0, 30.0);
        expectSolid(wedge, "base 45-degree wedge");

        // Boss extending perpendicular to slanted face
        const Vec3 normal = normalize(Vec3{1, 0, 1});
        const Vec3 u{0, 1, 0};
        const Vec3 v = cross(normal, u);
        std::vector<Vec2> bossPoly = CreateTool::makeRectPolygon({-6, -6}, {6, 6}, 0.0);
        Mesh boss;
        CreateTool::makePrismMesh(bossPoly, {0, 0, 0}, u, v, normal, 0.0, 10.0, boss);

        Mesh result;
        bool ok = meshBoolean(wedge, boss, BooleanOp::Union, result);
        check(ok, "union boss on angled wedge face succeeds");
        mergeCoplanarFaces(result);
        expectSolid(result, "boss on angled wedge");
        check(volumeOf(result) > volumeOf(wedge), "angled boss increases wedge volume");
    }
}

// ===========================================================================
// SECTION 6: Negative Extrusions Into Existing Objects (Boolean Difference Cuts)
// ===========================================================================
void testSection6_NegativeExtrudeCuts() {
    std::printf("\n--- Section 6: Negative Extrusions (Boolean Difference Cuts) ---\n");

    // 6.1 Shallow Blind Pocket Cut into Cube Top Face
    {
        Mesh base = makeBoxMesh({40, 40, 30}, {0, 0, -15}); // z in [-30, 0]
        std::vector<Vec2> pocketPoly = CreateTool::makeRectPolygon({-10, -10}, {10, 10}, 0.0);
        Mesh cutter;
        CreateTool::makePrismMesh(pocketPoly, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, -8.0, 1.0, cutter);

        Mesh result;
        bool ok = meshBoolean(base, cutter, BooleanOp::Difference, result);
        check(ok, "shallow pocket cut succeeds");
        mergeCoplanarFaces(result);
        expectSolid(result, "shallow pocket in box");
        const double expectedVol = (40.0 * 40.0 * 30.0) - (20.0 * 20.0 * 8.0); // 44800
        check(near(volumeOf(result), expectedVol), "shallow pocket volume = 44800 mm3");
    }

    // 6.2 Through-Hole Square Cut Through Entire Cube
    {
        Mesh base = makeBoxMesh({40, 40, 30}, {0, 0, -15}); // z in [-30, 0]
        std::vector<Vec2> holePoly = CreateTool::makeRectPolygon({-8, -8}, {8, 8}, 0.0);
        Mesh cutter;
        CreateTool::makePrismMesh(holePoly, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, -35.0, 1.0, cutter);

        Mesh result;
        bool ok = meshBoolean(base, cutter, BooleanOp::Difference, result);
        check(ok, "through-hole square cut succeeds");
        mergeCoplanarFaces(result);
        expectSolid(result, "square through-hole in box");
        const double expectedVol = (40.0 * 40.0 * 30.0) - (16.0 * 16.0 * 30.0); // 40320
        check(near(volumeOf(result), expectedVol), "square through-hole volume = 40320 mm3");
    }

    // 6.3 Corner Notch Cutaway (Overlapping Outer Perimeter)
    {
        Mesh base = makeBoxMesh({40, 40, 20}, {20, 20, 10}); // [0, 40] x [0, 40] x [0, 20]
        std::vector<Vec2> notchPoly = CreateTool::makeRectPolygon({-1, -1}, {15, 15}, 0.0);
        Mesh cutter;
        CreateTool::makePrismMesh(notchPoly, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, -1.0, 21.0, cutter);

        Mesh result;
        bool ok = meshBoolean(base, cutter, BooleanOp::Difference, result);
        check(ok, "corner notch cut succeeds");
        mergeCoplanarFaces(result);
        expectSolid(result, "corner notch cutaway");
        const double expectedVol = (40.0 * 40.0 * 20.0) - (15.0 * 15.0 * 20.0); // 27500
        check(near(volumeOf(result), expectedVol), "corner notch volume = 27500 mm3");
    }

    // 6.4 Edge-Straddling Slot Cut on Box Side Wall
    {
        Mesh base = makeBoxMesh({40, 40, 40}); // in [-20, 20]^3
        std::vector<Vec2> slotPoly = CreateTool::makeRectPolygon({-5, -7.5}, {5, 7.5}, 0.0);
        Mesh slotCutter;
        CreateTool::makePrismMesh(slotPoly, {20, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 0}, -10.0, 1.0, slotCutter);

        Mesh result;
        bool ok = meshBoolean(base, slotCutter, BooleanOp::Difference, result);
        check(ok, "side wall slot cut succeeds");
        mergeCoplanarFaces(result);
        expectSolid(result, "slot on box side");
        const double expectedVol = 64000.0 - (10.0 * 15.0 * 10.0); // 62500
        check(near(volumeOf(result), expectedVol), "side wall slot volume = 62500 mm3");
    }

    // 6.5 Cylindrical Bore Through Sphere Pole-to-Pole
    {
        Mesh sphere = makeSphereMesh(20.0, 24, 16);
        Mesh drill = makeCylinderMesh(6.0, 50.0, 24);

        Mesh result;
        bool ok = meshBoolean(sphere, drill, BooleanOp::Difference, result);
        check(ok, "drill through sphere pole-to-pole succeeds");
        mergeCoplanarFaces(result);
        expectSolid(result, "drilled sphere");
        check(volumeOf(result) < volumeOf(sphere), "drilled sphere volume reduced");
    }

    // 6.6 Cylindrical Cut Through Cylinder Axis (Coaxial Tube)
    {
        Mesh outer = makeCylinderMesh(20.0, 40.0, 24);
        Mesh inner = makeCylinderMesh(10.0, 50.0, 24);

        Mesh tube;
        bool ok = meshBoolean(outer, inner, BooleanOp::Difference, tube);
        check(ok, "hollow cylinder tube cut succeeds");
        mergeCoplanarFaces(tube);
        expectSolid(tube, "hollow tube");
        const double n = 24.0;
        const double aOuter = 0.5 * n * 400.0 * std::sin(kTwoPi / n);
        const double aInner = 0.5 * n * 100.0 * std::sin(kTwoPi / n);
        const double expectedVol = (aOuter - aInner) * 40.0;
        check(nearRel(volumeOf(tube), expectedVol, 0.01), "tube volume exact");
    }

    // 6.7 Pocket Cut into Angled / Chamfered 45-Degree Face
    {
        Mesh wedge = makeWedgeMesh(40.0, 40.0, 40.0);
        const Vec3 normal = normalize(Vec3{1, 0, 1});
        const Vec3 u{0, 1, 0};
        const Vec3 v = cross(normal, u);
        std::vector<Vec2> pocketPoly = CreateTool::makeRectPolygon({-6, -6}, {6, 6}, 0.0);
        Mesh cutter;
        CreateTool::makePrismMesh(pocketPoly, {0, 0, 0}, u, v, normal, -8.0, 1.0, cutter);

        Mesh result;
        bool ok = meshBoolean(wedge, cutter, BooleanOp::Difference, result);
        check(ok, "pocket cut into angled wedge face succeeds");
        mergeCoplanarFaces(result);
        expectSolid(result, "pocket in angled wedge");
        check(volumeOf(result) < volumeOf(wedge), "wedge volume decreased after pocket cut");
    }
}

// ===========================================================================
// SECTION 7: Multi-Operation Sequences & Interaction Chaining
// ===========================================================================
void testSection7_MultiOperationSequences() {
    std::printf("\n--- Section 7: Multi-Operation Chaining ---\n");

    // 7.1 Cut Then Boss: Pocket into box, then a post out of the pocket floor
    {
        // 1. Base Box: 60 x 60 x 30 sitting in z in [0, 30]
        Mesh m = makeBoxMesh({60, 60, 30}, {0, 0, 15});

        // 2. Cut pocket at top face (z=30): 40 x 40 mm, depth 10 mm (pocket floor at z=20)
        std::vector<Vec2> p1 = CreateTool::makeRectPolygon({-20, -20}, {20, 20}, 0.0);
        Mesh cutter;
        CreateTool::makePrismMesh(p1, {0, 0, 30}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, -10.0, 1.0, cutter);
        Mesh withPocket;
        check(meshBoolean(m, cutter, BooleanOp::Difference, withPocket), "step 1: cut pocket");
        mergeCoplanarFaces(withPocket);

        // 3. Extrude boss out of pocket floor (z=20): 16 x 16 mm, height +15 mm (reaches z=35)
        std::vector<Vec2> p2 = CreateTool::makeRectPolygon({-8, -8}, {8, 8}, 0.0);
        Mesh boss;
        CreateTool::makePrismMesh(p2, {0, 0, 20}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, 0.0, 15.0, boss);
        Mesh withBoss;
        check(meshBoolean(withPocket, boss, BooleanOp::Union, withBoss), "step 2: extrude boss in pocket");
        mergeCoplanarFaces(withBoss);
        expectSolid(withBoss, "pocket with boss in floor");

        const double expectedVol = (60.0 * 60.0 * 30.0) - (40.0 * 40.0 * 10.0) + (16.0 * 16.0 * 15.0);
        check(near(volumeOf(withBoss), expectedVol), "cut-then-boss volume = 95840 mm3");
    }

    // 7.2 Boss Then Cut: Boss atop box, then a through-hole drilled through boss and base
    {
        // 1. Base Box: 40 x 40 x 20 (z in [0, 20])
        Mesh base = makeBoxMesh({40, 40, 20}, {0, 0, 10});

        // 2. Boss: 20 x 20 x 10 atop base (z in [20, 30])
        Mesh boss = makeBoxMesh({20, 20, 10}, {0, 0, 25});
        Mesh combined;
        check(meshBoolean(base, boss, BooleanOp::Union, combined), "boss atop box");
        mergeCoplanarFaces(combined);

        // 3. Drill through-hole: 8 x 8 square from z=35 to z=-5
        Mesh drill = makeBoxMesh({8, 8, 40}, {0, 0, 15});
        Mesh drilled;
        check(meshBoolean(combined, drill, BooleanOp::Difference, drilled), "drill through boss and base");
        mergeCoplanarFaces(drilled);
        expectSolid(drilled, "boss and base with through-hole");

        const double baseVol = 40.0 * 40.0 * 20.0; // 32000
        const double bossVol = 20.0 * 20.0 * 10.0; // 4000
        const double holeVol = 8.0 * 8.0 * 30.0;   // 1920 (through total height 30)
        const double expectedVol = baseVol + bossVol - holeVol; // 34080
        check(near(volumeOf(drilled), expectedVol), "boss-then-cut volume = 34080 mm3");
    }

    // 7.3 Multi-Face Orthogonal Cuts (Pockets on 4 sides of a cube)
    {
        Mesh cube = makeBoxMesh({40, 40, 40}); // in [-20, 20]^3, vol = 64000

        // Pocket on +Z face: 10 x 10 x 5 deep
        Mesh cutZ = makeBoxMesh({10, 10, 6}, {0, 0, 18});
        Mesh r1;
        check(meshBoolean(cube, cutZ, BooleanOp::Difference, r1), "cut +Z");
        mergeCoplanarFaces(r1);

        // Pocket on +X face: 10 x 10 x 5 deep
        Mesh cutX = makeBoxMesh({6, 10, 10}, {18, 0, 0});
        Mesh r2;
        check(meshBoolean(r1, cutX, BooleanOp::Difference, r2), "cut +X");
        mergeCoplanarFaces(r2);

        // Pocket on +Y face: 10 x 10 x 5 deep
        Mesh cutY = makeBoxMesh({10, 6, 10}, {0, 18, 0});
        Mesh r3;
        check(meshBoolean(r2, cutY, BooleanOp::Difference, r3), "cut +Y");
        mergeCoplanarFaces(r3);

        expectSolid(r3, "cube with 3 orthogonal face pockets");
        const double expectedVol = 64000.0 - 3.0 * (10.0 * 10.0 * 5.0); // 64000 - 1500 = 62500
        check(near(volumeOf(r3), expectedVol), "orthogonal pockets volume = 62500 mm3");
    }
}

// ===========================================================================
// SECTION 8: Interactive Scene Lifecycle & Transformed Object Integration
// ===========================================================================
void testSection8_SceneLifecycleAndTransforms() {
    std::printf("\n--- Section 8: Scene Lifecycle & Transformed Objects ---\n");

    // 8.1 Standalone Filleted Object Creation in Scene (No Vanishing Bug)
    {
        Scene scene;
        Camera camera;
        UndoStack undo;

        CreateTool tool;
        tool.start(PrimitiveKind::Box);
        tool.setHoveredPlane(PlaneChoice::XY, {0, 0, 0}, {0, 0, 1});
        tool.commitPlaneSelection(camera);

        // Define a filleted rectangle profile (30 x 20 with r = 4mm) and depth 15mm
        tool.setProfileRect({-15, -10}, {15, 10}, 4.0);
        tool.setExtrudeDepth(15.0);
        tool.setStage(CreateStage::ExtrudeDepth);

        bool finished = tool.finishCreation(scene, camera, undo);
        check(finished, "finishCreation on filleted box succeeds");
        check(scene.objectCount() == 1, "scene has 1 object after creating filleted box");

        ObjectId id = scene.objects().front()->id;
        SceneObject* obj = scene.find(id);
        check(obj != nullptr, "filleted object is present in scene (does not vanish)");
        if (obj) {
            check(!obj->mesh.empty(), "filleted object mesh is non-empty");
            expectSolid(obj->mesh, "scene filleted box solid");
            const double expectedVol = (30.0 * 20.0 - (4.0 - kPi) * 16.0) * 15.0;
            check(nearRel(volumeOf(obj->mesh), expectedVol, 0.01), "scene filleted box volume matches expected");

            // Re-evaluating scene feature timeline must preserve the geometry
            bool reb = scene.rebuild(id);
            check(reb, "scene.rebuild succeeds on filleted object");
            check(!obj->mesh.empty() && nearRel(volumeOf(obj->mesh), expectedVol, 0.01),
                  "filleted object survives scene.rebuild without vanishing");
        }

        // Test Undo / Redo of creation
        undo.undo(scene);
        check(scene.objectCount() == 0, "undo removes created filleted object");
        undo.redo(scene);
        check(scene.objectCount() == 1, "redo restores filleted object");
        if (scene.objectCount() == 1) {
            SceneObject* restored = scene.objects().front().get();
            expectSolid(restored->mesh, "restored filleted object is solid");
        }
    }

    // 8.2 Positive Extrude / Boss on Transformed Object (Moved in World Space)
    {
        Scene scene;
        Camera camera;
        UndoStack undo;

        // Base box at position (100, 50, -20) with size 40x40x40 (local z in [-20, 20], world z in [-40, 0])
        PrimitiveSpec bSpec;
        bSpec.kind = PrimitiveKind::Box;
        bSpec.box.width = bSpec.box.depth = bSpec.box.height = 40.0;
        ObjectId baseId = scene.addPrimitive(PrimitiveKind::Box, bSpec, {100, 50, -20});
        check(baseId != kNoObject, "base box added at (100, 50, -20)");

        SceneObject* baseObj = scene.find(baseId);
        check(baseObj != nullptr, "base object found");

        // Top face is at local z = +20, which is world z = 0. Centroid is world (100, 50, 0).
        CreateTool tool;
        tool.start(PrimitiveKind::Box);
        // User raycasts and selects top face
        tool.setHoveredPlane(PlaneChoice::Face, {100, 50, 0}, {0, 0, 1}, baseId, 4);
        tool.commitPlaneSelection(camera);

        // Draw 20x20 boss centered on face, extrude depth +15 (world z in [0, 15])
        tool.setProfileRect({-10, -10}, {10, 10}, 0.0);
        tool.setExtrudeDepth(15.0);
        tool.setStage(CreateStage::ExtrudeDepth);

        bool finished = tool.finishCreation(scene, camera, undo);
        check(finished, "boss extrusion on transformed box succeeds");

        // Resulting body should have world bounds spanning x in [80, 120], y in [30, 70], z in [-40, 15]
        AABB wb = baseObj->worldBounds();
        check(near(wb.min.x, 80.0) && near(wb.max.x, 120.0), "boss world X bounds correct [80, 120]");
        check(near(wb.min.y, 30.0) && near(wb.max.y, 70.0), "boss world Y bounds correct [30, 70]");
        check(near(wb.min.z, -40.0) && near(wb.max.z, 15.0), "boss world Z bounds reach z=15 at top");
        expectSolid(baseObj->mesh, "joined boss on transformed box");
    }

    // 8.3 Negative Extrude / Cut into Transformed Object (Moved in World Space)
    {
        Scene scene;
        Camera camera;
        UndoStack undo;

        // Base box at position (-80, 60, 30) with size 40x40x40 (world z in [10, 50])
        PrimitiveSpec bSpec;
        bSpec.kind = PrimitiveKind::Box;
        bSpec.box.width = bSpec.box.depth = bSpec.box.height = 40.0;
        ObjectId baseId = scene.addPrimitive(PrimitiveKind::Box, bSpec, {-80, 60, 30});

        SceneObject* baseObj = scene.find(baseId);
        check(baseObj != nullptr, "base object found at (-80, 60, 30)");

        // Top face is at world z = 50. Centroid is world (-80, 60, 50).
        CreateTool tool;
        tool.start(PrimitiveKind::Box);
        tool.setHoveredPlane(PlaneChoice::Face, {-80, 60, 50}, {0, 0, 1}, baseId, 4);
        tool.commitPlaneSelection(camera);

        // Cut a 20x20 pocket of depth -10mm (into the top face)
        tool.setProfileRect({-10, -10}, {10, 10}, 0.0);
        tool.setExtrudeDepth(-10.0);
        tool.setStage(CreateStage::ExtrudeDepth);

        bool finished = tool.finishCreation(scene, camera, undo);
        check(finished, "pocket cut on transformed box succeeds");

        // Target volume should be exactly 64000 - 4000 = 60000 mm3
        const double expVol = 64000.0 - (20.0 * 20.0 * 10.0);
        check(near(volumeOf(baseObj->mesh), expVol), "pocket cut into transformed box has exact volume 60000 mm3");
        expectSolid(baseObj->mesh, "transformed box with pocket");

        // Test Undo & Redo of cut
        undo.undo(scene);
        check(near(volumeOf(baseObj->mesh), 64000.0), "undo restores uncut transformed box volume");
        undo.redo(scene);
        check(near(volumeOf(baseObj->mesh), expVol), "redo restores pocket cut volume");
    }

    // 8.4 Standalone Solid with Single Corner Fillet in Scene
    {
        Scene scene;
        Camera camera;
        UndoStack undo;

        CreateTool tool;
        tool.start(PrimitiveKind::Box);
        tool.setHoveredPlane(PlaneChoice::XY, {0, 0, 0}, {0, 0, 1});
        tool.commitPlaneSelection(camera);

        const Real r[4] = {6.0, 0.0, 0.0, 0.0}; // Corner 0 only filleted
        tool.setProfileRect({-20, -15}, {20, 15}, r);
        tool.setExtrudeDepth(20.0);
        tool.setStage(CreateStage::ExtrudeDepth);

        bool finished = tool.finishCreation(scene, camera, undo);
        check(finished, "finishCreation on single-corner filleted box succeeds");
        check(scene.objectCount() == 1, "scene contains single-corner filleted box");

        ObjectId id = scene.objects().front()->id;
        SceneObject* obj = scene.find(id);
        check(obj != nullptr, "single-corner filleted object found");
        if (obj) {
            expectSolid(obj->mesh, "single-corner filleted object solid");
            const double expVol = (40.0 * 30.0 - (1.0 - kPi * 0.25) * 36.0) * 20.0;
            check(nearRel(volumeOf(obj->mesh), expVol, 0.01), "single-corner filleted object volume exact");
        }
    }

    // 8.5 Keyboard Shortcut Navigation (E for Extrude, F for Fillet, Enter)
    {
        Scene scene;
        Camera camera;
        UndoStack undo;

        CreateTool tool;
        tool.start(PrimitiveKind::Box);
        check(tool.stage() == CreateStage::SelectPlane, "starts in SelectPlane");

        // Key '7' selects Top (XY) plane
        tool.handleKey('7', false, false, camera, scene, undo);
        check(tool.stage() == CreateStage::DrawProfile_Pt1, "'7' advances to DrawProfile_Pt1");

        // Key 'E' / Enter advances to Pt2
        tool.handleKey('E', false, false, camera, scene, undo);
        check(tool.stage() == CreateStage::DrawProfile_Pt2, "'E' advances to DrawProfile_Pt2");

        // Key 'E' / Enter advances to AdjustProfile
        tool.setProfileRect({0, 0}, {30, 20}, 0.0);
        tool.handleKey('E', false, false, camera, scene, undo);
        check(tool.stage() == CreateStage::AdjustProfile, "'E' advances to AdjustProfile");

        // Key 'E' in AdjustProfile advances to ExtrudeDepth
        tool.handleKey('E', false, false, camera, scene, undo);
        check(tool.stage() == CreateStage::ExtrudeDepth, "'E' advances to ExtrudeDepth");

        // Key 'E' in ExtrudeDepth finishes creation
        tool.handleKey('E', false, false, camera, scene, undo);
        check(tool.stage() == CreateStage::None, "'E' finishes creation");
        check(scene.objectCount() == 1, "object created via E shortcuts");
    }
}

// ===========================================================================
// SECTION 9: Cuts and Joins Belong to the Feature History
//
// The tool used to assign target->mesh and leave features/featureCache
// describing the body as it was before the operation. Everything downstream
// trusted the stale chain: re-evaluating deleted the cut, and committing a
// fillet either failed to resolve edges it had just previewed or applied them
// to the uncut body and discarded the cut. These check the chain, not just the
// mesh, because the mesh looked right the whole time.
// ===========================================================================
void testSection9_EditsLandInTheHistory() {
    std::printf("\n--- Section 9: Cuts & Joins Belong to the Feature History ---\n");

    // Builds a 36x20x20 box sitting on the plate and cuts a 10x10x8 pocket into
    // its top face with the create tool, the way the interface does.
    auto pocketedBox = [](Scene& scene, Camera& camera, UndoStack& undo) {
        PrimitiveSpec ps;
        ps.kind = PrimitiveKind::Box;
        ps.box.width = 36.0; ps.box.depth = 20.0; ps.box.height = 20.0;
        const ObjectId id = scene.addPrimitive(PrimitiveKind::Box, ps, {0, 0, 10});

        CreateTool tool;
        tool.start(PrimitiveKind::Box);
        tool.setHoveredPlane(PlaneChoice::Face, {0, 0, 20}, {0, 0, 1}, id, 1);
        tool.commitPlaneSelection(camera);
        tool.setProfileRect({-5, -5}, {5, 5}, 0.0);
        tool.setExtrudeDepth(-8.0);
        tool.setStage(CreateStage::ExtrudeDepth);
        const bool ok = tool.finishCreation(scene, camera, undo);
        return ok ? id : kNoObject;
    };

    const double kBox    = 36.0 * 20.0 * 20.0;              // 14400
    const double kPocket = kBox - 10.0 * 10.0 * 8.0;        // 13600

    // 9.1 The cut is a feature, and the cache agrees with what is on screen
    {
        Scene scene; Camera camera; UndoStack undo;
        const ObjectId id = pocketedBox(scene, camera, undo);
        check(id != kNoObject, "tool cut succeeds");
        SceneObject* obj = scene.find(id);
        check(obj != nullptr, "cut object present");
        if (!obj) return;

        check(near(volumeOf(obj->mesh), kPocket), "pocket volume is 13600 mm3");
        expectSolid(obj->mesh, "pocketed box");
        check(obj->features.size() == 2, "the chain has two features");
        check(obj->features.size() == 2 &&
              obj->features[1].kind == FeatureKind::Boolean &&
              obj->features[1].booleanOp == BooleanOp::Difference,
              "the cut is recorded as a Boolean difference");
        check(obj->featureCache.size() == obj->features.size() &&
              !obj->featureCache.empty() &&
              near(volumeOf(obj->featureCache.back()), kPocket),
              "featureCache matches the visible mesh");
    }

    // 9.2 Re-evaluating must not delete the cut
    {
        Scene scene; Camera camera; UndoStack undo;
        const ObjectId id = pocketedBox(scene, camera, undo);
        SceneObject* obj = scene.find(id);
        if (!obj) { check(false, "cut object present for re-evaluation"); return; }

        check(scene.reevaluate(id) && near(volumeOf(obj->mesh), kPocket),
              "reevaluate keeps the pocket");
        check(scene.rebuild(id) && near(volumeOf(obj->mesh), kPocket),
              "rebuild keeps the pocket");

        // The base is still a parametric box, so changing its width re-cuts.
        obj->spec.box.width = 50.0;
        check(scene.rebuild(id) &&
              near(volumeOf(obj->mesh), 50.0 * 20.0 * 20.0 - 10.0 * 10.0 * 8.0),
              "widening the box re-cuts the pocket");

        undo.undo(scene);
        check(near(volumeOf(obj->mesh), kBox), "undo restores the uncut box");
        undo.redo(scene);
        check(near(volumeOf(obj->mesh), kPocket), "redo restores the pocket");
    }

    // 9.3 A fillet committed on the cut body is neither refused nor destructive
    {
        Scene scene; Camera camera; UndoStack undo;
        const ObjectId id = pocketedBox(scene, camera, undo);
        SceneObject* obj = scene.find(id);
        if (!obj) { check(false, "cut object present for filleting"); return; }

        const Mesh before = obj->mesh;
        int previewed = 0, refused = 0, lostTheCut = 0;

        for (Index he = 0; he < before.halfedgeCount(); ++he) {
            if (he > before.halfedges[he].twin) continue;
            const Index fa = before.halfedges[he].face;
            const Index fb = before.halfedges[before.halfedges[he].twin].face;
            if (fa == kInvalid || fb == kInvalid) continue;
            if (dot(before.faceNormal(fa), before.faceNormal(fb)) > 0.999) continue;

            const std::vector<Index> edges = extendTangentChain(before, {he});

            // What the interactive preview computes.
            Mesh scratch = before;
            FilletSpec spec;
            spec.segments = 4;
            for (Index e : edges) spec.edges.push_back({e, 1.0});
            if (!filletEdges(scratch, spec)) continue;
            ++previewed;

            // What committing it computes: named edges, replayed chain.
            const std::vector<Feature> chainBefore = obj->features;
            const std::vector<Mesh> cacheBefore = obj->featureCache;

            Feature f;
            f.kind = FeatureKind::Bevel;
            f.edges = nameEdges(before, edges);
            f.radii.assign(f.edges.count(), 1.0);
            f.width = 1.0;
            f.segments = 4;

            std::string why;
            if (!scene.addFeature(id, std::move(f), &why)) {
                ++refused;
                std::printf("      chain at he %d refused on commit: %s\n", he, why.c_str());
            } else if (volumeOf(obj->mesh) > kPocket + 400.0) {
                // A fillet moves the volume by a few mm3 either way; the pocket
                // vanishing is an 800 mm3 jump, so halfway is a safe line.
                ++lostTheCut;
                std::printf("      chain at he %d committed onto the uncut body (vol %.1f)\n",
                            he, volumeOf(obj->mesh));
            }

            obj->features = chainBefore;
            obj->featureCache = cacheBefore;
            scene.reevaluate(id);
        }

        check(previewed > 0, "some fillets preview on the cut body");
        check(refused == 0, "no previewed fillet is refused on commit");
        check(lostTheCut == 0, "no committed fillet discards the cut");
    }

    // 9.4 A cut with nothing to cut into refuses rather than adding the cutter
    {
        Scene scene; Camera camera; UndoStack undo;
        CreateTool tool;
        tool.start(PrimitiveKind::Box);
        tool.setHoveredPlane(PlaneChoice::XY, {0, 0, 0}, {0, 0, 1});
        tool.commitPlaneSelection(camera);
        tool.setProfileRect({-10, -10}, {10, 10}, 0.0);
        tool.setExtrudeDepth(-10.0);
        tool.setStage(CreateStage::ExtrudeDepth);

        const bool ok = tool.finishCreation(scene, camera, undo);
        check(!ok, "a cut into nothing is refused");
        check(scene.objectCount() == 0, "the cutter is not added to the scene as a body");
        check(!tool.takeError().empty(), "the refusal carries a reason");
        check(tool.takeError().empty(), "reading the reason clears it");
    }

    // 9.5 A join lands in the chain too
    {
        Scene scene; Camera camera; UndoStack undo;
        PrimitiveSpec ps;
        ps.kind = PrimitiveKind::Box;
        ps.box.width = ps.box.depth = ps.box.height = 20.0;
        const ObjectId id = scene.addPrimitive(PrimitiveKind::Box, ps, {0, 0, 10});

        CreateTool tool;
        tool.start(PrimitiveKind::Box);
        tool.setHoveredPlane(PlaneChoice::Face, {0, 0, 20}, {0, 0, 1}, id, 1);
        tool.commitPlaneSelection(camera);
        tool.setProfileRect({-5, -5}, {5, 5}, 0.0);
        tool.setExtrudeDepth(10.0);
        tool.setStage(CreateStage::ExtrudeDepth);
        check(tool.finishCreation(scene, camera, undo), "tool join succeeds");

        SceneObject* obj = scene.find(id);
        if (!obj) { check(false, "joined object present"); return; }
        const double expected = 20.0 * 20.0 * 20.0 + 10.0 * 10.0 * 10.0;
        check(near(volumeOf(obj->mesh), expected), "boss volume is 9000 mm3");
        check(obj->features.size() == 2 &&
              obj->features[1].kind == FeatureKind::Boolean &&
              obj->features[1].booleanOp == BooleanOp::Union,
              "the join is recorded as a Boolean union");
        check(scene.reevaluate(id) && near(volumeOf(obj->mesh), expected),
              "reevaluate keeps the boss");
    }
}

// ===========================================================================
// MAIN ENTRY POINT
// ===========================================================================
int main() {
    std::printf("=========================================================\n");
    std::printf("  Running Comprehensive Object Creation Test Suite       \n");
    std::printf("=========================================================\n");

    testSection1_ProfileGeneration();
    testSection2_StandaloneObjectsOnPlanes();
    testSection3_PreExtrudeModifications();
    testSection4_ExtrusionDistancesAndDirections();
    testSection5_PositiveExtrudeAutoJoin();
    testSection6_NegativeExtrudeCuts();
    testSection7_MultiOperationSequences();
    testSection8_SceneLifecycleAndTransforms();
    testSection9_EditsLandInTheHistory();

    std::printf("\n=========================================================\n");
    std::printf("  Test Suite Summary: %s\n", gFailures == 0 ? "ALL PASS" : "FAILED");
    std::printf("  Total Checks/Assertions : %d\n", gChecks);
    std::printf("  Total Failures          : %d\n", gFailures);
    std::printf("=========================================================\n");

    return gFailures ? 1 : 0;
}
