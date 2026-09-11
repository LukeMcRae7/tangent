// The exact backend: the operations, and the names that have to survive them.
//
// Everything here goes through Body and src/geom/operations.h -- the same calls
// the feature history makes -- rather than reaching for OpenCASCADE directly.
// The parts are the ones the mesh kernel refuses, because that is the whole
// argument for the backend existing.
#include "geom/body.h"
#include "geom/operations.h"

#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near(Real a, Real b, Real eps = 1e-6) { return std::fabs(a - b) < eps; }

static Body plate(Real w = 100, Real d = 100, Real h = 10) {
    PrimitiveSpec spec;
    spec.kind = PrimitiveKind::Box;
    spec.box.width = w; spec.box.depth = d; spec.box.height = h;
    Body b;
    makePrimitive(spec, b, Backend::Brep);
    return b;
}

static Body bore(Real x, Real y, Real dia, Real h = 40) {
    PrimitiveSpec spec;
    spec.kind = PrimitiveKind::Cylinder;
    spec.cylinder.radius = dia / 2;
    spec.cylinder.height = h;
    Body b;
    makePrimitive(spec, b, Backend::Brep);
    b.transform(translate({x, y, 0}));
    return b;
}

// The face a body's own name says is the top of it, without looking at any
// geometry to find it.
static FaceId topFace(const Body& b) {
    std::vector<FaceId> faces;
    b.allFaces(faces);
    for (FaceId f : faces)
        if (dot(b.faceNormal(f), Vec3{0, 0, 1}) > 0.99) return f;
    return kNoFace;
}

int main() {
    if (!brep::available()) {
        std::printf("B-rep backend not built (TANGENT_BREP=OFF); nothing to test\n");
        return 0;
    }

    std::printf("--- a rim is drawn along its curve, not across it ---\n");
    {
        // The bug this covers: anything drawing an edge from its two ends put a
        // chord across a bore rather than a line around it, so a hole's rim and
        // a fillet read as polygons however finely the body was tessellated.
        Body body = plate();
        Body out;
        std::string why;
        check(booleanOp(body, bore(0, 0, 20.0), BooleanOp::Difference, out, 1, false, &why),
              "bored a 20mm hole");
        body = std::move(out);

        const FaceId top = topFace(body);
        check(top != kNoFace, "found the top face");

        std::vector<EdgeId> edges;
        body.faceEdges(top, edges);

        const Real asked = 0.01;
        int lines = 0, curved = 0;
        Real worstPoint = 0.0, worstChord = 0.0;
        std::vector<Vec3> pts;
        for (EdgeId e : edges) {
            body.edgePolyline(e, asked, pts);
            if (body.edgeKind(e) == CurveKind::Line) {
                ++lines;
                check(pts.size() == 2, "a straight edge is its two ends and nothing more");
                continue;
            }
            ++curved;
            check(pts.size() > 2, "a curved edge is more than one chord");

            // Every sampled point sits on the bore, and the chords between them
            // stay inside the tolerance that was asked for.
            for (const Vec3& p : pts)
                worstPoint = std::max(worstPoint, std::fabs(std::hypot(p.x, p.y) - 10.0));
            for (size_t k = 1; k < pts.size(); ++k) {
                const Vec3 mid{(pts[k - 1].x + pts[k].x) * 0.5,
                               (pts[k - 1].y + pts[k].y) * 0.5, pts[k].z};
                worstChord = std::max(worstChord, 10.0 - std::hypot(mid.x, mid.y));
            }
        }
        check(curved > 0, "the bored top face has a curved edge");
        check(lines == 4, "and the plate's four straight ones");
        check(worstPoint < 1e-6, "every sampled point is on the bore");
        check(worstChord <= asked + 1e-9, "and no chord strays past the tolerance asked for");
        std::printf("[rim] %d curved, %d straight; worst point %.2e mm, worst chord %.5f mm\n",
                    curved, lines, worstPoint, worstChord);
    }

    std::printf("--- shell: the operation a printed part is hollowed with ---\n");
    {
        // A 60 x 60 x 20 box, shelled to a 2mm wall with its top left open, is
        // a tray. The volume is the arithmetic anyone would do by hand, which
        // is the point of checking it that way.
        const Real w = 60, d = 60, h = 20, t = 2;
        Body body = plate(w, d, h);
        const Real solidVolume = body.health(false).volume;
        check(near(solidVolume, w * d * h, 1e-6), "the solid block measures up");

        const FaceId top = topFace(body);
        check(top != kNoFace, "found the top face to open");

        std::string why;
        check(shellBody(body, {top}, t, 11, &why), std::string("shelled it: ") + why);
        check(body.health().solid(), "and the result is a closed solid");

        // Walls on four sides and a floor; the cavity is open at the top.
        const Real cavity = (w - 2 * t) * (d - 2 * t) * (h - t);
        check(near(body.health(false).volume, solidVolume - cavity, 1e-6),
              "the wall left behind is exactly the wall that was asked for");
        std::printf("  tray: %.1f mm3 of %.1f mm3 left at a %.0fmm wall, %d faces\n",
                    body.health(false).volume, solidVolume, t, body.faceCount());

        // The name of the face that was open survives: a feature that referred
        // to the top of the box still refers to it after the hollowing.
        Body fresh = plate(w, d, h);
        const ElementId topName = fresh.faceName(topFace(fresh));
        Body shelled = fresh;
        check(shellBody(shelled, {topFace(shelled)}, t, 11, &why), "shelled again");
        std::vector<FaceId> still;
        shelled.findFaces(topName, still);
        check(!still.empty(), "and the opened face keeps its name");

        // Refusals carry a reason rather than a self-intersecting solid.
        Body tooThick = plate(w, d, h);
        check(!shellBody(tooThick, {topFace(tooThick)}, 40.0, 12, &why),
              "a wall thicker than the part is refused");
        check(!why.empty(), "and says why: " + why);

        Body zero = plate(w, d, h);
        check(!shellBody(zero, {topFace(zero)}, 0.0, 13, &why),
              "so is a wall of no thickness");

        // A mesh body says plainly that this is not its operation.
        PrimitiveSpec boxSpec;
        boxSpec.kind = PrimitiveKind::Box;
        boxSpec.box.width = boxSpec.box.depth = boxSpec.box.height = 20;
        Body meshBox;
        check(makePrimitive(boxSpec, meshBox, Backend::Mesh), "a mesh box");
        check(!shellBody(meshBox, {}, t, 14, &why), "a mesh body refuses to shell");
        check(why.find("mesh") != std::string::npos, "and names the reason: " + why);
    }

    std::printf("--- a drawn circle is one cylinder, not four quarters ---\n");
    {
        // The create tool draws a circle as four quarter-arcs, because a
        // sagitta per span is all the profile format carries. Swept naively
        // that gives four wall faces: seams drawn down a bore, and a click
        // that selects a quarter of it.
        const Real r = 10.0, h = 12.0;
        std::vector<Vec3> pts;
        std::vector<Real> arcs;
        for (int i = 0; i < 4; ++i) {
            const Real a = kHalfPi * i;
            pts.push_back({r * std::cos(a), r * std::sin(a), 0});
            // Negative: the profile winds counter-clockwise, and the arc has
            // to stand off the chord away from the centre to stay on the circle.
            arcs.push_back(-r * (1.0 - std::sqrt(2.0) * 0.5));
        }

        Body solid;
        std::string why;
        check(makeProfileSolid(pts, arcs, {0, 0, 1}, 0, h, solid, 9, &why),
              std::string("swept the drawn circle: ") + why);

        std::vector<FaceId> faces;
        solid.allFaces(faces);
        int cyls = 0, planes = 0;
        FaceId wall = kNoFace;
        for (FaceId f : faces) {
            if (solid.faceKind(f) == SurfaceKind::Cylinder) { ++cyls; wall = f; }
            else if (solid.faceKind(f) == SurfaceKind::Plane) ++planes;
        }
        std::printf("  %zu faces: %d cylinder, %d plane\n", faces.size(), cyls, planes);
        check(cyls == 1, "the wall is one cylindrical face, not four quarters");
        check(planes == 2, "with a cap at each end");
        if (wall != kNoFace) {
            Vec3 p, ax;
            Real got = 0;
            check(solid.faceCylinder(wall, p, ax, got), "and it knows it is a cylinder");
            check(near(got, r, 1e-6), "of the radius that was drawn");
            check(near(solid.faceArea(wall), 2.0 * kPi * r * h, 1e-4),
                  "carrying the whole wall, not a quarter of it");
        }
        check(solid.health().solid(), "and the result is still a solid");
    }

    std::printf("--- what a bore is made of ---\n");
    {
        Body body = plate(60, 60, 10);
        Body drill = bore(0, 0, 20.0, 40);
        drill.transform(translate({0, 0, -10}));     // clear through, no tangency
        Body out;
        std::string why;
        check(booleanOp(body, drill, BooleanOp::Difference, out, 2, false, &why),
              std::string("bored through: ") + why);
        body = std::move(out);

        std::vector<FaceId> faces;
        body.allFaces(faces);
        int planes = 0, cyls = 0;
        for (FaceId f : faces) {
            if (body.faceKind(f) == SurfaceKind::Plane) ++planes;
            if (body.faceKind(f) == SurfaceKind::Cylinder) ++cyls;
        }
        std::printf("  faces: %zu total, %d plane, %d cylinder\n", faces.size(), planes, cyls);

        std::vector<EdgeId> edges;
        body.allEdges(edges);
        int lines = 0, circles = 0, other = 0;
        for (EdgeId e : edges) {
            if (body.edgeKind(e) == CurveKind::Line) ++lines;
            else if (body.edgeKind(e) == CurveKind::Circle) ++circles;
            else ++other;
        }
        std::printf("  edges: %zu total, %d line, %d circle, %d other\n",
                    edges.size(), lines, circles, other);

        for (FaceId f : faces) {
            if (body.faceKind(f) != SurfaceKind::Cylinder) continue;
            std::vector<EdgeId> fe;
            body.faceEdges(f, fe);
            Vec3 p, ax;
            Real r = 0;
            body.faceCylinder(f, p, ax, r);
            std::printf("  a cylinder face: r=%.3f, %zu edges, area %.3f (whole wall = %.3f)\n",
                        r, fe.size(), body.faceArea(f), 2.0 * kPi * 10.0 * 10.0);
        }
        check(cyls >= 1, "the bore is a cylinder, not a prism");

        // One wall, not four. A bore cut by a boolean stays a single
        // cylindrical face carrying the whole 2*pi*r*h of it, which is what
        // lets clicking the bore select the bore. A profile swept from four
        // quarter-arcs gives four faces instead, and reads as a cylinder with
        // seams drawn down it.
        FaceId wall = kNoFace;
        for (FaceId f : faces) if (body.faceKind(f) == SurfaceKind::Cylinder) wall = f;
        check(cyls == 1, "the bore is one face");
        check(near(body.faceArea(wall), 2.0 * kPi * 10.0 * 10.0, 1e-6),
              "holding the whole wall, not a quarter of it");

        // And it is *drawn* round: the deviation lives at the middle of a
        // chord, since the triangle vertices sit on the surface and measuring
        // those measures nothing.
        RenderMesh rm;
        body.tessellate(rm);
        Real worst = 0.0;
        for (size_t t = 0; t < rm.triangleFace.size(); ++t) {
            if (rm.triangleFace[t] != wall) continue;
            for (int k = 0; k < 3; ++k) {
                const Vec3 a = rm.positions[rm.triangles[t * 3 + k]];
                const Vec3 b = rm.positions[rm.triangles[t * 3 + (k + 1) % 3]];
                if (std::fabs(a.z - b.z) > 1e-9) continue;      // not a chord around
                const Vec3 mid{(a.x + b.x) * 0.5, (a.y + b.y) * 0.5, a.z};
                worst = std::max(worst, 10.0 - std::hypot(mid.x, mid.y));
            }
        }
        check(worst < 0.05, "and no facet stands more than 0.05mm inside it");
        std::printf("  wall is one face of %.3f mm2, drawn %.4f mm inside a 10mm bore\n",
                    body.faceArea(wall), worst);
    }

    std::printf("--- a bolt circle, cut and rounded in one go ---\n");
    {
        // The part the mesh kernel refuses outright: eight holes on a pitch
        // circle and one in the middle. Stage 0 measured it as the case where
        // the mesh boolean gave up at hole six.
        Body body = plate();
        const Real startVolume = body.health(false).volume;
        check(near(startVolume, 100.0 * 100.0 * 10.0, 1e-6), "plate is 100 x 100 x 10");

        std::vector<Vec2> holes;
        for (int i = 0; i < 8; ++i) {
            const Real a = 2.0 * kPi * i / 8.0;
            holes.push_back({35.0 * std::cos(a), 35.0 * std::sin(a)});
        }
        holes.push_back({0, 0});

        int cut = 0;
        for (size_t i = 0; i < holes.size(); ++i) {
            const Real dia = (i + 1 == holes.size()) ? 20.0 : 6.6;
            Body out;
            std::string why;
            if (!booleanOp(body, bore(holes[i].x, holes[i].y, dia), BooleanOp::Difference,
                           out, static_cast<ElementId>(100 + i), false, &why)) {
                std::printf("  refused at hole %zu: %s\n", i + 1, why.c_str());
                break;
            }
            body = std::move(out);
            ++cut;
        }
        check(cut == 9, "all nine holes went through");
        // Six sides and one wall per hole. The mesh kernel's answer to the same
        // part, before the coplanar rebuild was repaired, was in the thousands;
        // at 32 segments and working, it is 278.
        check(body.faceCount() == 15, "the plate is fifteen faces: six sides and nine walls");

        const Real expected = 100.0 * 100.0 * 10.0
                            - 8.0 * kPi * 3.3 * 3.3 * 10.0
                            - kPi * 10.0 * 10.0 * 10.0;
        check(near(body.health(false).volume, expected, 1e-6),
              "the volume is exact, not a polygon's approximation of it");

        // Every rim, rounded in one operation. This is the Stage 0 criterion
        // and the thing the mesh kernel could not do at any segment count.
        std::vector<EdgeId> rims;
        std::vector<EdgeId> edges;
        body.allEdges(edges);
        for (EdgeId e : edges) {
            Vec3 a, b;
            body.edgePositions(e, a, b);
            if (near(a.z, 5.0, 1e-6) && near(b.z, 5.0, 1e-6) &&
                std::hypot(a.x, a.y) < 49.0)
                rims.push_back(e);
        }
        check(!rims.empty(), "found the rims");

        FilletSpec spec;
        spec.salt = 999;
        for (EdgeId e : rims) spec.edges.push_back({e, 1.5});
        std::string why;
        Body rounded = body;
        check(filletEdges(rounded, spec, &why),
              "every rim rounds at once: " + why);
        check(rounded.health(false).volume < body.health(false).volume,
              "and rounding took material off");
        std::printf("  9 holes, %zu rims filleted, %d faces, %.1f mm3\n",
                    rims.size(), rounded.faceCount(), rounded.health(false).volume);
    }

    std::printf("--- names survive the operations that make them ---\n");
    {
        Body body = plate(60, 60, 10);
        const ElementId topName = body.faceName(topFace(body));
        check(topName != kNoId, "the plate's top face has a name");

        Body tool = bore(0, 0, 12);
        const ElementId wallName = [&] {
            std::vector<FaceId> fs;
            tool.allFaces(fs);
            for (FaceId f : fs)
                if (std::fabs(tool.faceNormal(f).z) < 0.5) return tool.faceName(f);
            return kNoId;
        }();
        check(wallName != kNoId, "the tool's wall has a name");

        Body bored;
        std::string why;
        check(booleanOp(body, tool, BooleanOp::Difference, bored, 7, false, &why),
              "the bore goes through: " + why);

        check(bored.findFace(topName) != kInvalid, "the top face keeps its name through the cut");
        check(bored.findFace(wallName) != kInvalid, "and the hole's wall is named from the tool");

        // The rim, found by asking which edge lies between two named faces --
        // no geometry, no searching for circles.
        const FaceId top = bored.findFace(topName);
        const FaceId wall = bored.findFace(wallName);
        EdgeId rim = kInvalid;
        std::vector<EdgeId> edges;
        bored.allEdges(edges);
        for (EdgeId e : edges) {
            FaceId a = kNoFace, b = kNoFace;
            bored.edgeFaces(e, a, b);
            if ((a == top && b == wall) || (a == wall && b == top)) { rim = e; break; }
        }
        check(rim != kInvalid, "the rim is the edge between the two named faces");

        FilletSpec spec;
        spec.salt = 11;
        spec.edges.push_back({rim, 2.0});
        Body rounded = bored;
        check(filletEdges(rounded, spec, &why), "the rim rounds: " + why);
        check(rounded.findFace(topName) != kInvalid, "the top face survives the fillet");
        check(rounded.findFace(wallName) != kInvalid, "so does the wall");
        check(rounded.faceCount() == bored.faceCount() + 1,
              "and exactly one face was added -- the fillet itself");

        // Every face in the result has a name from provenance. A count of
        // unnamed faces is the number that says whether the mechanism leaks.
        std::vector<FaceId> fs;
        rounded.allFaces(fs);
        int unnamed = 0;
        std::set<ElementId> names;
        for (FaceId f : fs) {
            if (rounded.faceName(f) == kNoId) ++unnamed;
            names.insert(rounded.faceName(f));
        }
        check(unnamed == 0, "no face came out unnamed");
        check(names.size() == fs.size(), "and no two faces share a name here");
        std::printf("  plate, bore, fillet: %d faces, %zu names, %d unnamed\n",
                    rounded.faceCount(), names.size(), unnamed);
    }

    std::printf("--- the same chain twice, and after a parameter change ---\n");
    {
        auto run = [](Real thickness, Real dia) {
            Body body = plate(60, 60, thickness);
            Body out;
            booleanOp(body, bore(0, 0, dia), BooleanOp::Difference, out, 7, false, nullptr);
            std::set<ElementId> names;
            std::vector<FaceId> fs;
            out.allFaces(fs);
            for (FaceId f : fs) names.insert(out.faceName(f));
            return names;
        };
        const std::set<ElementId> a = run(10, 12);
        const std::set<ElementId> b = run(10, 12);
        const std::set<ElementId> c = run(14, 12);
        const std::set<ElementId> d = run(10, 16);
        check(a == b, "the same chain twice gives the same names");
        check(a == c, "a thicker plate changes no name");
        check(a == d, "a wider bore changes no name");
        std::printf("  %zu names, stable across a re-run and two parameter changes\n", a.size());
    }

    std::printf("--- extrude, the operation the workflow is built on ---\n");
    {
        Body body = plate(40, 40, 10);
        const ElementId topName = body.faceName(topFace(body));

        // A boss pushed out of the top face.
        std::vector<FaceId> newFaces;
        std::string why;
        check(extrudeFaces(body, {topFace(body)}, 12.0, &newFaces, 21,
                           ExtrudeOp::Auto, &why), "extrude out: " + why);
        check(near(body.health(false).volume, 40.0 * 40.0 * 22.0, 1e-6),
              "the body grew by exactly the swept volume");
        check(!newFaces.empty(), "and it reports where the face went");
        check(body.findFace(topName) != kInvalid,
              "the extruded face keeps the name it was selected by");

        // A pocket pushed into it, from the face the last step left on top.
        const FaceId nowTop = topFace(body);
        check(nowTop != kNoFace, "there is a top face to push into");
        Body pocketed = body;
        check(extrudeFaces(pocketed, {nowTop}, -4.0, nullptr, 22, ExtrudeOp::Auto, &why),
              "extrude in: " + why);
        check(pocketed.health(false).volume < body.health(false).volume,
              "pushing into the body takes material away");
        std::printf("  boss then pocket: %d faces, %.1f mm3\n",
                    pocketed.faceCount(), pocketed.health(false).volume);
    }

    std::printf("--- inset, and what a pocket is drawn from ---\n");
    {
        Body body = plate(40, 40, 10);
        const FaceId top = topFace(body);
        const ElementId topName = body.faceName(top);
        const int facesBefore = body.faceCount();
        const Real volumeBefore = body.health(false).volume;

        std::vector<FaceId> inner;
        std::string why;
        check(insetFaces(body, {top}, 5.0, &inner, 41, &why), "the top face insets: " + why);
        check(body.faceCount() == facesBefore + 1, "the face is now two: an inner and a ring");
        check(near(body.health(false).volume, volumeBefore, 1e-6),
              "and no material moved -- an inset is a split, not a cut");
        check(!inner.empty(), "it reports the inner face");

        // The inner face keeps the name: it is the one a person would point at
        // next, and the ring around it takes a derived name.
        check(body.findFace(topName) != kInvalid, "the inner face answers to the original name");
        const FaceId keptName = body.findFace(topName);
        check(near(body.faceArea(keptName), 30.0 * 30.0, 1e-6),
              "and it is the inner one: 30 x 30 inside a 40 x 40 face");

        // A pocket: inset, then push the inner face in.
        check(extrudeFaces(body, {keptName}, -3.0, nullptr, 42, ExtrudeOp::Auto, &why),
              "the inner face pushes in: " + why);
        check(near(body.health(false).volume, volumeBefore - 30.0 * 30.0 * 3.0, 1e-6),
              "which takes exactly the pocket out");
        std::printf("  40mm plate, 5mm inset, 3mm pocket: %d faces, %.1f mm3\n",
                    body.faceCount(), body.health(false).volume);
    }

    std::printf("--- a curved face is refused rather than approximated ---\n");
    {
        PrimitiveSpec cs;
        cs.kind = PrimitiveKind::Cylinder;
        cs.cylinder.radius = 10;
        cs.cylinder.height = 20;
        Body cyl;
        check(makePrimitive(cs, cyl, Backend::Brep), "cylinder built");

        FaceId wall = kNoFace;
        std::vector<FaceId> faces;
        cyl.allFaces(faces);
        for (FaceId f : faces)
            if (std::fabs(cyl.faceNormal(f).z) < 0.5) wall = f;
        check(wall != kNoFace, "found the wall");

        std::string why;
        check(!insetFaces(cyl, {wall}, 2.0, nullptr, 43, &why), "insetting a curved face is refused");
        check(why.find("flat") != std::string::npos, "and says why: " + why);
    }

    std::printf("--- a profile with real arcs in it ---\n");
    {
        // A 40 x 20 rectangle with 5mm rounded corners, as the create tool
        // draws it -- but with the corners as actual arcs rather than the
        // polyline the mesh backend has to settle for.
        const Real w = 40, d = 20, r = 5;
        std::vector<Vec3> pts;
        std::vector<Real> arcs;
        auto add = [&](Real x, Real y, Real bulge) {
            pts.push_back({x, y, 0});
            arcs.push_back(bulge);
        };
        // Corner arcs bulge inward by the sagitta of a quarter circle.
        const Real sag = r * (1.0 - std::sqrt(2.0) / 2.0);
        add(-w / 2 + r, -d / 2, 0);           add(w / 2 - r, -d / 2, -sag);
        add(w / 2, -d / 2 + r, 0);            add(w / 2, d / 2 - r, -sag);
        add(w / 2 - r, d / 2, 0);             add(-w / 2 + r, d / 2, -sag);
        add(-w / 2, d / 2 - r, 0);            add(-w / 2, -d / 2 + r, -sag);

        Body solid;
        std::string why;
        check(makeProfileSolid(pts, arcs, {0, 0, 1}, 0, 10, solid, 31, &why),
              "the rounded profile sweeps into a solid: " + why);
        check(!solid.empty() && solid.validate(), "and it is valid");

        // Area of a rounded rectangle: the rectangle less the corners the
        // rounding cut off. If the corners were polylines this would be short.
        const Real area = w * d - (4.0 - kPi) * r * r;
        check(near(solid.health(false).volume, area * 10.0, 1e-6),
              "with the volume of a rounded rectangle, not a chamfered one");
        std::printf("  rounded rectangle: %d faces, %.3f mm3 (exact %.3f)\n",
                    solid.faceCount(), solid.health(false).volume, area * 10.0);
    }

    std::printf("--- what gets drawn is what the part looks like ---\n");
    {
        // A rounded cube with a bore: every kind of surface at once, and the
        // shape the first exact bodies were rendered as.
        PrimitiveSpec bs;
        bs.kind = PrimitiveKind::Box;
        bs.box = {20, 20, 20};
        Body body;
        check(makePrimitive(bs, body, Backend::Brep), "cube built");

        std::vector<EdgeId> edges;
        body.allEdges(edges);
        FilletSpec fs;
        fs.salt = 51;
        for (EdgeId e : edges) fs.edges.push_back({e, 4.0});
        std::string why;
        check(filletEdges(body, fs, &why), "every edge rounded: " + why);

        PrimitiveSpec cs;
        cs.kind = PrimitiveKind::Cylinder;
        cs.cylinder.radius = 3;
        cs.cylinder.height = 60;
        Body tool;
        check(makePrimitive(cs, tool, Backend::Brep), "bore tool built");
        tool.transform(toMat4(Quat::fromAxisAngle({0, 1, 0}, static_cast<Real>(kHalfPi))));
        Body out;
        check(booleanOp(body, tool, BooleanOp::Difference, out, 52, false, &why),
              "bored through: " + why);
        body = std::move(out);

        RenderMesh rm;
        body.tessellate(rm);

        // Every face has to reach the screen. A face with no triangles is a
        // hole in the model as far as the user is concerned.
        std::vector<FaceId> faces;
        body.allFaces(faces);
        std::set<Index> drawn(rm.triangleFace.begin(), rm.triangleFace.end());
        check(static_cast<int>(drawn.size()) == body.faceCount(),
              "every face contributes triangles");

        // No triangles of zero area: they draw as a speck at best, and a ray
        // that hit one would resolve to a face nobody can see.
        int degenerate = 0;
        for (size_t i = 0; i + 2 < rm.triangles.size(); i += 3) {
            const Vec3 a2 = rm.positions[rm.triangles[i]];
            const Vec3 b2 = rm.positions[rm.triangles[i + 1]];
            const Vec3 c2 = rm.positions[rm.triangles[i + 2]];
            if (length(cross(b2 - a2, c2 - a2)) < 1e-12) ++degenerate;
        }
        check(degenerate == 0, "no triangles of zero area");

        // And no cracks: welded by position, every triangle edge is shared by
        // exactly two triangles, or the silhouette shows a notch.
        std::map<std::string, int> ids;
        std::vector<int> welded(rm.positions.size(), 0);
        for (size_t i = 0; i < rm.positions.size(); ++i) {
            const Vec3 p = rm.positions[i];
            char buf[96];
            std::snprintf(buf, sizeof buf, "%lld,%lld,%lld",
                          static_cast<long long>(std::llround(p.x * 1e4)),
                          static_cast<long long>(std::llround(p.y * 1e4)),
                          static_cast<long long>(std::llround(p.z * 1e4)));
            const auto [it, fresh] = ids.emplace(buf, static_cast<int>(ids.size()));
            welded[i] = it->second;
        }
        std::map<std::pair<int, int>, int> used;
        for (size_t i = 0; i + 2 < rm.triangles.size(); i += 3) {
            const int t[3] = {welded[rm.triangles[i]], welded[rm.triangles[i + 1]],
                              welded[rm.triangles[i + 2]]};
            for (int k = 0; k < 3; ++k) {
                const int x = t[k], y = t[(k + 1) % 3];
                if (x != y) ++used[{std::min(x, y), std::max(x, y)}];
            }
        }
        int cracks = 0, overshared = 0;
        for (const auto& [e, n] : used) {
            if (n == 1) ++cracks;
            else if (n > 2) ++overshared;
        }
        check(cracks == 0, "the tessellated surface is closed: no cracks between faces");
        check(overshared == 0, "and no edge is shared by more than two triangles");

        // The wireframe draws the part's edges and not the parameterisation's.
        // A hole has a seam running across it where its surface is cut open to
        // be parameterised, and drawing that puts a line across the opening.
        int acrossTheBore = 0;
        for (size_t i = 0; i + 1 < rm.edgeLines.size(); i += 2) {
            const Vec3 a2 = rm.positions[rm.edgeLines[i]];
            const Vec3 b2 = rm.positions[rm.edgeLines[i + 1]];
            // A seam of this bore runs along X, inside the hole's radius.
            if (std::fabs(a2.y) < 3.01 && std::fabs(a2.z) < 3.01 &&
                std::fabs(b2.y) < 3.01 && std::fabs(b2.z) < 3.01 &&
                std::fabs(b2.x - a2.x) > 1.0)
                ++acrossTheBore;
        }
        check(acrossTheBore == 0, "no seam line is drawn across the bore");
        std::printf("  rounded cube with a bore: %d faces, %zu triangles, %zu lines\n",
                    body.faceCount(), rm.triangles.size() / 3, rm.edgeLines.size() / 2);
    }

    std::printf("--- refusals say why ---\n");
    {
        Body body = plate(20, 20, 20);
        std::vector<EdgeId> edges;
        body.allEdges(edges);

        FilletSpec spec;
        spec.edges.push_back({edges.front(), 50.0});   // wider than the body
        std::string why;
        check(!filletEdges(body, spec, &why), "an impossible radius is refused");
        check(!why.empty(), "and it says why: " + why);

        // A mesh body and an exact one cannot be combined, and saying so is
        // better than quietly giving back a mesh.
        Body meshBody;
        PrimitiveSpec cube;
        cube.kind = PrimitiveKind::Box;
        check(makePrimitive(cube, meshBody, Backend::Mesh), "a mesh body for comparison");
        Body out;
        std::string mixWhy;
        check(!booleanOp(body, meshBody, BooleanOp::Union, out, 1, false, &mixWhy),
              "mixing backends is refused");
        check(!mixWhy.empty(), "with a reason: " + mixWhy);
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
