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
