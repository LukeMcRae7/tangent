// Tangent - the part on the website, built by the kernel that builds parts.
//
// The hero of the site is a printable part turning slowly under the light. It
// is not a stock model and not a render from somewhere else: it is built here,
// by the operations a person would use -- a rounded plate, countersunk screw
// holes, a hex boss rounded into the plate, and a real external thread -- and
// written out as the triangles and edges the renderer would draw.
//
// Run it by hand when the part changes, and commit what it produces:
//
//     cmake --build build --target site_model
//     ./build/site_model site/model/part.bin
//
// Beside it goes part.json: what the part is, and where on it the site pins
// its dimension callouts, both taken from the same numbers that built it.
//
// The file is small and plain, read by site/js/hero.js:
//
//     char[4]  "TGM1"
//     uint32   vertex count, triangle count, edge segment count
//     float32  bounds min xyz, max xyz
//     float32  positions  [vertices * 3]
//     int16    normals    [vertices * 3], scaled to 32767
//     uint8    group      [vertices]: which operation made the face, see Group
//     uint32   triangles  [triangles * 3]
//     float32  edge points[segments * 6], two ends per segment
#include "geom/operations.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

using namespace tg;

namespace {

// Every dimension in millimetres, in one place, so the part can be changed
// without reading the construction.
constexpr Real kPlateW      = 72.0;
constexpr Real kPlateD      = 52.0;
constexpr Real kPlateH      = 6.0;
constexpr Real kPlateCorner = 9.0;
constexpr Real kPlateRound  = 2.0;    // the top rim
constexpr Real kFootChamfer = 0.8;    // the bottom rim: elephant's foot
constexpr Real kHexFlats    = 26.0;
constexpr Real kHexH        = 9.0;
constexpr Real kHexRound    = 2.5;    // where the boss meets the plate
constexpr Real kShaftR      = 8.0;    // M16
constexpr Real kShaftH      = 20.0;
constexpr Real kPitch       = 2.0;
constexpr Real kBore        = 8.4;
constexpr Real kHoleX       = 27.0;
constexpr Real kHoleY       = 17.0;

// What made a face, which is what the features section lights up as it
// scrolls past. Found from names rather than from geometry: a face an
// operation creates carries a name nothing before it had, and keeps it
// through everything after.
enum Group : uint8_t { kBody = 0, kRounds = 1, kHoles = 2, kThread = 3 };
std::map<ElementId, Group> g_made;
Real g_lift = 0;   // how far the part was moved to centre it

void tag(const Body& b, Group g) {
    std::vector<FaceId> fs;
    b.allFaces(fs);
    for (FaceId f : fs) g_made.emplace(b.faceName(f), g);   // the first to make it keeps it
}

bool fail(const char* what, const std::string& why) {
    std::fprintf(stderr, "site_model: %s: %s\n", what, why.empty() ? "refused" : why.c_str());
    return false;
}

// A rounded rectangle, counter-clockwise, as the create tool would draw it:
// straight spans between arcs of the corner radius.
void roundedRect(Real w, Real d, Real r, std::vector<Vec3>& pts, std::vector<Real>& arcs) {
    const Real x = w * 0.5, y = d * 0.5;
    const Real sag = -r * (1.0 - std::sqrt(2.0) * 0.5);
    const Vec3 p[8] = {{ x, -y + r, 0}, { x,  y - r, 0}, { x - r,  y, 0}, {-x + r,  y, 0},
                       {-x,  y - r, 0}, {-x, -y + r, 0}, {-x + r, -y, 0}, { x - r, -y, 0}};
    for (int i = 0; i < 8; ++i) {
        pts.push_back(p[i]);
        arcs.push_back(i % 2 ? sag : 0.0);   // odd spans turn a corner
    }
}

// Edges lying in the plane z, whose midpoints are between rMin and rMax of
// the axis.
std::vector<EdgeId> edgesAt(const Body& b, Real z, Real rMin, Real rMax) {
    std::vector<EdgeId> all, out;
    b.allEdges(all);
    for (EdgeId e : all) {
        Vec3 a, c;
        b.edgePositions(e, a, c);
        const Vec3 m = b.edgeMidpoint(e);
        const Real r = std::sqrt(m.x * m.x + m.y * m.y);
        if (std::fabs(a.z - z) < 1e-4 && std::fabs(c.z - z) < 1e-4 &&
            std::fabs(m.z - z) < 1e-4 && r >= rMin && r <= rMax)
            out.push_back(e);
    }
    return out;
}

bool round(Body& b, const std::vector<EdgeId>& edges, Real r, bool chamfer, const char* what) {
    FilletSpec spec;
    spec.chamfer = chamfer;
    static ElementId salt = 100;
    spec.salt = ++salt;
    for (EdgeId e : edges) spec.edges.push_back({e, r});
    std::string why;
    if (edges.empty()) return fail(what, "no edges found");
    if (!filletEdges(b, spec, &why)) return fail(what, why);
    tag(b, kRounds);
    return true;
}

FaceId cylinderOfRadius(const Body& b, Real radius) {
    std::vector<FaceId> fs;
    b.allFaces(fs);
    for (FaceId f : fs) {
        Vec3 p, axis;
        Real r;
        if (b.faceCylinder(f, p, axis, r) && std::fabs(r - radius) < 1e-4) return f;
    }
    return kNoFace;
}

bool build(Body& part) {
    std::string why;

    // The plate, its rims done while they are the only rims there are.
    std::vector<Vec3> pts;
    std::vector<Real> arcs;
    roundedRect(kPlateW, kPlateD, kPlateCorner, pts, arcs);
    if (!makeProfileSolid(pts, arcs, {0, 0, 1}, 0, kPlateH, part, 1, &why)) return fail("plate", why);
    tag(part, kBody);
    if (!round(part, edgesAt(part, 0, 0, 1e9), kFootChamfer, true, "foot chamfer")) return false;
    if (!round(part, edgesAt(part, kPlateH, 0, 1e9), kPlateRound, false, "plate rim")) return false;

    // The hex boss, flats facing along x, rounded down into the plate.
    pts.clear(); arcs.clear();
    const Real circum = kHexFlats * 0.5 / std::cos(kPi / 6.0);
    for (int i = 0; i < 6; ++i) {
        const Real a = kPi / 6.0 + i * kPi / 3.0;
        pts.push_back({circum * std::cos(a), circum * std::sin(a), 0});
        arcs.push_back(0);
    }
    Body hex, joined;
    if (!makeProfileSolid(pts, arcs, {0, 0, 1}, kPlateH, kPlateH + kHexH, hex, 2, &why))
        return fail("hex", why);
    if (!booleanOp(part, hex, BooleanOp::Union, joined, 3, false, &why)) return fail("hex join", why);
    part = std::move(joined);
    tag(part, kBody);
    if (!round(part, edgesAt(part, kPlateH, 0, circum + 0.1), kHexRound, false, "hex root"))
        return false;
    if (!round(part, edgesAt(part, kPlateH + kHexH, kShaftR + 1, circum + 0.1), 1.0, true, "hex top"))
        return false;

    // The shaft.
    PrimitiveSpec cs;
    cs.kind = PrimitiveKind::Cylinder;
    cs.cylinder.radius = kShaftR;
    cs.cylinder.height = kShaftH + 1.0;   // one mm down into the boss
    Body shaft;
    if (!makePrimitive(cs, shaft, Backend::Brep)) return fail("shaft", "");
    shaft.transform(translate({0, 0, kPlateH + kHexH + kShaftH * 0.5 - 0.5}));
    if (!booleanOp(part, shaft, BooleanOp::Union, joined, 4, false, &why)) return fail("shaft join", why);
    part = std::move(joined);
    tag(part, kBody);

    // A bore down the middle, and four countersunk M4 holes.
    const Real top = kPlateH + kHexH + kShaftH;
    HoleCut bore;
    bore.diameter = kBore;
    if (!drillHole(part, {0, 0, top}, {0, 0, -1}, bore, 5, &why)) return fail("bore", why);
    tag(part, kHoles);

    HoleCut sink;
    sink.kind = HoleKind::Countersink;
    sink.diameter = 4.5;
    sink.headDiameter = 8.6;
    for (int i = 0; i < 4; ++i) {
        const Vec3 at{(i & 1) ? kHoleX : -kHoleX, (i & 2) ? kHoleY : -kHoleY, kPlateH};
        if (!drillHole(part, at, {0, 0, -1}, sink, 10 + i, &why)) return fail("countersink", why);
    }
    tag(part, kHoles);

    // The thread last: it is the most faces, and nothing after it has to
    // find its way around them.
    const FaceId wall = cylinderOfRadius(part, kShaftR);
    if (wall == kNoFace) return fail("thread", "no shaft face");
    if (!threadFace(part, wall, kPitch, 0.5413 * kPitch, /*external=*/true, 20, &why))
        return fail("thread", why);
    tag(part, kThread);

    // Centred on the origin, so the site turns it about its middle.
    const AABB bb = part.bounds();
    g_lift = -(bb.min.z + bb.max.z) * 0.5;
    part.transform(translate({0, 0, g_lift}));
    return true;
}

template <class T> void put(std::FILE* f, const T* p, size_t n) { std::fwrite(p, sizeof(T), n, f); }

} // namespace

int main(int argc, char** argv) {
    const char* path = argc > 1 ? argv[1] : "site/model/part.bin";

    Body part;
    if (!build(part)) return 1;
    const MeshHealth h = part.health(false);
    std::printf("part: %d faces, %.0f mm3, %s\n", part.faceCount(), h.volume,
                h.solid() ? "solid" : "NOT SOLID");

    TessellationQuality q;
    q.deviationMm = 0.03;
    q.angleRad = 0.2;
    q.independent = true;
    RenderMesh rm;
    part.tessellate(rm, q);

    // Edges drawn as the renderer draws them: the curve, not its two ends.
    std::vector<float> lines;
    std::vector<EdgeId> edges;
    part.allEdges(edges);
    std::vector<Vec3> poly;
    for (EdgeId e : edges) {
        poly.clear();
        part.edgePolyline(e, 0.03, poly);
        for (size_t i = 0; i + 1 < poly.size(); ++i)
            for (const Vec3& v : {poly[i], poly[i + 1]}) {
                lines.push_back(float(v.x)); lines.push_back(float(v.y)); lines.push_back(float(v.z));
            }
    }

    std::FILE* f = std::fopen(path, "wb");
    if (!f) { std::perror(path); return 1; }
    const uint32_t counts[3] = {uint32_t(rm.positions.size()), uint32_t(rm.triangles.size() / 3),
                                uint32_t(lines.size() / 6)};
    const AABB bb = part.bounds();
    const float box[6] = {float(bb.min.x), float(bb.min.y), float(bb.min.z),
                          float(bb.max.x), float(bb.max.y), float(bb.max.z)};
    std::fwrite("TGM1", 1, 4, f);
    put(f, counts, 3);
    put(f, box, 6);
    // Faces are meshed apart, so a vertex belongs to the face of any triangle
    // that uses it.
    std::vector<uint8_t> group(rm.positions.size(), kBody);
    int counted[4] = {};
    for (size_t t = 0; t < rm.triangleFace.size(); ++t) {
        const auto it = g_made.find(part.faceName(rm.triangleFace[t]));
        const Group g = it == g_made.end() ? kBody : it->second;
        ++counted[g];
        for (int k = 0; k < 3; ++k) group[rm.triangles[t * 3 + k]] = g;
    }
    std::printf("triangles by group: body %d, rounds %d, holes %d, thread %d\n",
                counted[0], counted[1], counted[2], counted[3]);

    std::vector<float> pos;
    std::vector<int16_t> nrm;
    for (size_t i = 0; i < rm.positions.size(); ++i) {
        const Vec3 p = rm.positions[i], n = rm.normals[i];
        pos.insert(pos.end(), {float(p.x), float(p.y), float(p.z)});
        for (Real c : {n.x, n.y, n.z})
            nrm.push_back(int16_t(std::lround(std::fmax(-1.0, std::fmin(1.0, c)) * 32767.0)));
    }
    put(f, pos.data(), pos.size());
    put(f, nrm.data(), nrm.size());
    put(f, group.data(), group.size());
    put(f, rm.triangles.data(), rm.triangles.size());
    put(f, lines.data(), lines.size());
    std::fclose(f);

    // The callouts, each a point on the surface, the way that surface faces,
    // and what a drawing would write beside it.
    struct Callout { const char* label; Vec3 at, facing; int group; };
    const Real top = kPlateH + kHexH + kShaftH;
    const Real a = -kPi / 4.0;
    char thread[32], bore[32], sink[40], root[16], foot[24];
    std::snprintf(thread, sizeof thread, "M%.0f \u00d7 %.1f", kShaftR * 2, kPitch);
    std::snprintf(bore, sizeof bore, "\u2300%.1f THRU", kBore);
    std::snprintf(sink, sizeof sink, "4\u00d7 \u2300%.1f CSK 90\u00b0", 4.5);
    std::snprintf(root, sizeof root, "R%.1f", kHexRound);
    std::snprintf(foot, sizeof foot, "%.1f \u00d7 45\u00b0", kFootChamfer);
    const Callout callouts[] = {
        {thread, {kShaftR * std::cos(a), kShaftR * std::sin(a), kPlateH + kHexH + kShaftH * 0.62},
         {std::cos(a), std::sin(a), 0}, kThread},
        {bore, {0, kBore * 0.5, top}, {0, 0, 1}, kHoles},
        {sink, {kHoleX, -kHoleY, kPlateH}, {0, 0, 1}, kHoles},
        {root, {kHexFlats * 0.5 + kHexRound * 0.3, 0, kPlateH + kHexRound * 0.3}, {0.7071, 0, 0.7071}, kRounds},
        {foot, {0, -kPlateD * 0.5, kFootChamfer * 0.5}, {0, -0.7071, -0.7071}, kRounds},
    };

    const std::string meta = std::string(path).substr(0, std::string(path).rfind('.')) + ".json";
    std::FILE* j = std::fopen(meta.c_str(), "w");
    if (!j) { std::perror(meta.c_str()); return 1; }
    std::fprintf(j, "{\n  \"faces\": %d,\n  \"volume_mm3\": %.1f,\n  \"solid\": %s,\n",
                 part.faceCount(), h.volume, h.solid() ? "true" : "false");
    std::fprintf(j, "  \"size_mm\": [%.1f, %.1f, %.1f],\n  \"callouts\": [\n",
                 kPlateW, kPlateD, top);   // not bounds(), which the kernel pads
    const size_t n = sizeof callouts / sizeof callouts[0];
    for (size_t i = 0; i < n; ++i) {
        const Callout& c = callouts[i];
        std::fprintf(j, "    {\"label\": \"%s\", \"group\": %d, \"at\": [%.2f, %.2f, %.2f], "
                        "\"facing\": [%.3f, %.3f, %.3f]}%s\n",
                     c.label, c.group, c.at.x, c.at.y, c.at.z + g_lift, c.facing.x, c.facing.y,
                     c.facing.z, i + 1 < n ? "," : "");
    }
    std::fprintf(j, "  ]\n}\n");
    std::fclose(j);

    std::printf("wrote %s: %u vertices, %u triangles, %u edge segments\n", path, counts[0],
                counts[1], counts[2]);
    return 0;
}
