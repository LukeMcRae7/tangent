// STL and 3MF export, and project round-trips.
#include "mesh/export_3mf.h"
#include "mesh/export_stl.h"
#include "scene/serialize.h"

#include <zlib.h>

#include <map>
#include <unordered_map>

#include <cmath>
#include <filesystem>
#include <iterator>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near(double a, double b, double eps = 1e-6) { return std::fabs(a - b) < eps; }

// Honours TMPDIR on Linux and TEMP on Windows. A hardcoded /tmp resolves to
// C:\tmp there, which does not exist, and the write that then failed surfaced
// as a crash further down rather than as a failed check.
static std::string tmp(const char* name) {
    return (std::filesystem::temp_directory_path() /
            ("tangent_test_" + std::string(name)))
        .string();
}

// Minimal binary STL reader, so the test checks the bytes rather than trusting
// the writer's own report.
struct StlTri { float n[3], v[3][3]; };
static bool readBinaryStl(const std::string& path, std::vector<StlTri>& out) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return false;
    const std::streamsize size = f.tellg();
    f.seekg(0);
    std::vector<char> buf(static_cast<size_t>(size));
    if (!f.read(buf.data(), size)) return false;
    if (buf.size() < 84) return false;

    uint32_t count = 0;
    std::memcpy(&count, buf.data() + 80, 4);
    if (buf.size() != 84 + static_cast<size_t>(count) * 50) return false;

    out.clear();
    for (uint32_t i = 0; i < count; ++i) {
        const char* p = buf.data() + 84 + static_cast<size_t>(i) * 50;
        StlTri t;
        std::memcpy(t.n, p, 12);
        std::memcpy(t.v, p + 12, 36);
        out.push_back(t);
    }
    return true;
}

static double stlVolume(const std::vector<StlTri>& tris) {
    double s6 = 0.0;
    for (const StlTri& t : tris) {
        const Vec3 a{t.v[0][0], t.v[0][1], t.v[0][2]};
        const Vec3 b{t.v[1][0], t.v[1][1], t.v[1][2]};
        const Vec3 c{t.v[2][0], t.v[2][1], t.v[2][2]};
        s6 += dot(a, cross(b, c));
    }
    return s6 / 6.0;
}

// STL export at a stated tolerance: the number a printer cares about, and the
// thing a mesh body cannot offer because its resolution was decided when it was
// made.
static void testExportTolerance() {
    if (!brep::available()) {
        std::printf("[stl] exact kernel not built; tolerance test skipped\n");
        return;
    }

    // A 20mm cylinder: the shape where the tolerance is visible as a number.
    Scene s;
    PrimitiveSpec spec;
    spec.kind = PrimitiveKind::Cylinder;
    spec.cylinder.radius = 10;
    spec.cylinder.height = 20;
    const ObjectId id = s.addPrimitive(PrimitiveKind::Cylinder, spec);
    check(id != kNoObject, "exact cylinder created");
    check(!s.find(id)->body.isMesh(), "and it is exact");

    struct Case { Real dev; size_t coarser; };
    size_t previous = 0;
    for (Real dev : {0.2, 0.05, 0.01}) {
        StlOptions opt;
        opt.deviationMm = dev;
        const std::string path = tmp("tol.stl");
        const StlResult r = exportStl(s, path, opt);
        check(r.ok, "export at " + std::to_string(dev) + "mm");
        check(r.triangles > 0, "wrote triangles");
        if (previous) check(r.triangles >= previous, "a finer tolerance never writes fewer triangles");
        previous = r.triangles;

        // The real assertion: how far the written surface actually sits from
        // the cylinder it stands for. Every vertex of the wall must be inside
        // the tolerance of the true radius -- that is what the number promises.
        Real worst = 0.0;
        RenderMesh rm;
        TessellationQuality q;
        q.deviationMm = dev;
        q.angleRad = 0.5;
        q.independent = true;
        s.find(id)->body.tessellate(rm, q);
        // The deviation lives at the middle of a chord: a tessellation puts its
        // vertices *on* the surface, so measuring those measures nothing. Only
        // the curved wall is asked about -- a flat cap is exact whatever the
        // tolerance, and its triangles run straight across the disc.
        const Body& b = s.find(id)->body;
        for (size_t t = 0; t < rm.triangleFace.size(); ++t) {
            const FaceId f = rm.triangleFace[t];
            if (std::fabs(b.faceNormal(f).z) > 0.5) continue;     // a cap
            for (int k = 0; k < 3; ++k) {
                const Vec3 a2 = rm.positions[rm.triangles[t * 3 + k]];
                const Vec3 b2 = rm.positions[rm.triangles[t * 3 + (k + 1) % 3]];
                if (std::fabs(a2.z - b2.z) > 1e-9) continue;      // a vertical chord is exact
                const Vec3 mid{(a2.x + b2.x) * 0.5, (a2.y + b2.y) * 0.5, a2.z};
                worst = std::max(worst, 10.0 - std::hypot(mid.x, mid.y));
            }
        }
        check(worst <= dev + 1e-9,
              "the surface stays within " + std::to_string(dev) + "mm (worst " +
              std::to_string(worst) + ")");
        std::printf("[stl] %.3f mm tolerance: %zu triangles, worst deviation %.5f mm\n",
                    dev, r.triangles, worst);
        std::remove(path.c_str());
    }

    // Exporting coarsely must not leave the screen showing the coarse result.
    // Meshing writes into the shape, which every Body that copied it shares, so
    // an export that did not work on a copy would degrade the viewport.
    {
        RenderMesh screen;
        s.find(id)->body.tessellate(screen);
        const size_t before = screen.triangles.size();

        StlOptions coarse;
        coarse.deviationMm = 0.5;
        const std::string coarsePath = tmp("tol_coarse.stl");
        const StlResult cr = exportStl(s, coarsePath, coarse);
        check(cr.ok, "coarse export");

        RenderMesh after;
        s.find(id)->body.tessellate(after);
        check(after.triangles.size() == before,
              "the screen keeps its own tessellation after a coarse export");
        std::remove(coarsePath.c_str());
        std::printf("[stl] screen kept %zu triangles across a 0.5mm export of %zu\n",
                    before / 3, cr.triangles);
    }

    // A mesh body is written as it stands, and the result says so rather than
    // implying the tolerance was honoured.
    Scene m;
    m.setDefaultBackend(Backend::Mesh);
    m.addPrimitive(PrimitiveKind::Cylinder, spec);
    StlOptions opt;
    opt.deviationMm = 0.001;
    const std::string meshPath = tmp("tol_mesh.stl");
    const StlResult r = exportStl(m, meshPath, opt);
    check(r.ok && r.meshBodies == 1, "a mesh body is reported as written at its own resolution");
    std::remove(meshPath.c_str());
    std::printf("[stl] mesh body: %zu triangles, tolerance not applicable\n", r.triangles);
}

// A project written by an older build still opens. The version bump that let
// bodies be exact added a field to every feature; refusing to read a file
// without it would be losing someone's work over a field with one possible
// value.
static void testOlderFileOpens() {
    Scene s;
    s.setDefaultBackend(Backend::Mesh);
    const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
    s.find(id)->name = "Legacy";
    const std::string path = tmp("v4.tng");
    check(saveProject(s, path).ok, "saved a project");

    // Rewrite the version word in place, and strip what version 5 added: one
    // u32 backend field at the end of each feature. With a single-feature box
    // that is the last four bytes of the file.
    std::vector<char> bytes;
    {
        std::ifstream in(path, std::ios::binary);
        bytes.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }
    check(bytes.size() > 12, "the file has content");
    const size_t versionAt = 8;   // after the magic
    uint32_t four = 4;
    std::memcpy(bytes.data() + versionAt, &four, sizeof four);
    bytes.resize(bytes.size() - 4);
    {
        std::ofstream out(path, std::ios::binary);
        out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    }

    Scene loaded;
    const ProjectResult r = loadProject(loaded, path);
    check(r.ok, "a version 4 project still opens: " + r.error);
    check(loaded.objectCount() == 1, "with its object");
    if (loaded.objectCount() == 1) {
        const SceneObject* o = loaded.objects().front().get();
        check(o->name == "Legacy", "and its name");
        check(o->features.size() == 1 && o->features[0].backend == Backend::Mesh,
              "and everything in it is a mesh, which is all a v4 file could hold");
        check(o->body.faceCount() == 6, "and it re-evaluates");
    }

    // A file from the future is refused, because there is no way to know what a
    // field this build has never heard of means.
    uint32_t future = kProjectVersion + 1;
    std::memcpy(bytes.data() + versionAt, &future, sizeof future);
    {
        std::ofstream out(path, std::ios::binary);
        out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    }
    Scene tooNew;
    check(!loadProject(tooNew, path).ok, "a newer project is refused");
    std::remove(path.c_str());
    std::printf("[project] version 4 opens, version %u refused\n", kProjectVersion + 1);
}

// ---- 3MF ------------------------------------------------------------------
//
// Read back independently of the writer: the archive's central directory is
// walked, every part inflated and its CRC checked, and the model parsed for
// its vertices and triangles. What is checked is what a slicer relies on --
// millimetres, every object separate and named, each mesh closed and wound
// outward, and the volume it encloses the volume of the part.

static uint32_t rd32(const std::string& b, size_t at) {
    uint32_t v = 0;
    std::memcpy(&v, b.data() + at, 4);
    return v;
}
static uint16_t rd16(const std::string& b, size_t at) {
    uint16_t v = 0;
    std::memcpy(&v, b.data() + at, 2);
    return v;
}

static bool readZip(const std::string& path, std::map<std::string, std::string>& parts,
                    std::string& why) {
    std::ifstream f(path, std::ios::binary);
    if (!f) { why = "cannot open"; return false; }
    const std::string b((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (b.size() < 22) { why = "too short"; return false; }
    const size_t end = b.size() - 22;               // no archive comment is written
    if (rd32(b, end) != 0x06054b50) { why = "no end of central directory"; return false; }
    const uint16_t count = rd16(b, end + 10);
    size_t at = rd32(b, end + 16);
    for (uint16_t i = 0; i < count; ++i) {
        if (at + 46 > b.size() || rd32(b, at) != 0x02014b50) { why = "bad directory entry"; return false; }
        const uint16_t method = rd16(b, at + 10);
        const uint32_t crc = rd32(b, at + 16);
        const uint32_t packed = rd32(b, at + 20), size = rd32(b, at + 24);
        const uint16_t nameLen = rd16(b, at + 28), extra = rd16(b, at + 30), comment = rd16(b, at + 32);
        const uint32_t local = rd32(b, at + 42);
        const std::string name = b.substr(at + 46, nameLen);
        at += 46 + nameLen + extra + comment;

        if (rd32(b, local) != 0x04034b50) { why = "bad local header for " + name; return false; }
        if (rd32(b, local + 14) != crc || rd32(b, local + 18) != packed || rd32(b, local + 22) != size) {
            why = "local header disagrees with the directory for " + name;
            return false;
        }
        const size_t data = local + 30 + rd16(b, local + 26) + rd16(b, local + 28);
        std::string out(size, '\0');
        if (method == 0) {
            out = b.substr(data, size);
        } else if (method == 8) {
            z_stream zs{};
            inflateInit2(&zs, -15);
            zs.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(b.data() + data));
            zs.avail_in = packed;
            zs.next_out = reinterpret_cast<Bytef*>(out.data());
            zs.avail_out = size;
            const int rc = inflate(&zs, Z_FINISH);
            inflateEnd(&zs);
            if (rc != Z_STREAM_END || zs.avail_out != 0) { why = "inflate failed for " + name; return false; }
        } else {
            why = "unknown method for " + name;
            return false;
        }
        const uLong got = crc32(crc32(0L, Z_NULL, 0), reinterpret_cast<const Bytef*>(out.data()),
                                static_cast<uInt>(out.size()));
        if (got != crc) { why = "CRC mismatch for " + name; return false; }
        parts[name] = std::move(out);
    }
    return true;
}

struct ReadObject {
    std::string name;
    std::vector<Vec3> vertices;
    std::vector<uint32_t> triangles;
};

static std::string attr(const std::string& tag, const char* key) {
    const std::string k = std::string(" ") + key + "=\"";
    const size_t a = tag.find(k);
    if (a == std::string::npos) return {};
    const size_t v = a + k.size();
    return tag.substr(v, tag.find('"', v) - v);
}

static std::vector<ReadObject> parseModel(const std::string& xml) {
    std::vector<ReadObject> objects;
    size_t at = 0;
    while ((at = xml.find('<', at)) != std::string::npos) {
        const size_t close = xml.find('>', at);
        const std::string tag = xml.substr(at, close - at + 1);
        at = close;
        if (tag.rfind("<object ", 0) == 0) {
            objects.push_back({attr(tag, "name"), {}, {}});
        } else if (tag.rfind("<vertex ", 0) == 0 && !objects.empty()) {
            objects.back().vertices.push_back({std::stod(attr(tag, "x")), std::stod(attr(tag, "y")),
                                               std::stod(attr(tag, "z"))});
        } else if (tag.rfind("<triangle ", 0) == 0 && !objects.empty()) {
            for (const char* k : {"v1", "v2", "v3"})
                objects.back().triangles.push_back(static_cast<uint32_t>(std::stoul(attr(tag, k))));
        }
    }
    return objects;
}

static double enclosed(const ReadObject& o) {
    double s6 = 0.0;
    for (size_t i = 0; i + 2 < o.triangles.size(); i += 3)
        s6 += dot(o.vertices[o.triangles[i]],
                  cross(o.vertices[o.triangles[i + 1]], o.vertices[o.triangles[i + 2]]));
    return s6 / 6.0;
}

static bool closedOutward(const ReadObject& o) {
    std::map<std::pair<uint32_t, uint32_t>, int> edges;
    for (size_t i = 0; i + 2 < o.triangles.size(); i += 3)
        for (int k = 0; k < 3; ++k)
            if (++edges[{o.triangles[i + k], o.triangles[i + (k + 1) % 3]}] > 1) return false;
    for (const auto& [e, n] : edges)
        if (!edges.count({e.second, e.first})) return false;
    for (uint32_t v : o.triangles)
        if (v >= o.vertices.size()) return false;
    return !o.triangles.empty() && enclosed(o) > 0.0;
}

static void test3mf() {
    // Built on whichever kernel the build has. With the exact one, a box and a
    // placed, turned cylinder go out tessellated and welded; a mesh sphere goes
    // out with the vertices it has. A reflected copy checks the winding turns
    // with it rather than coming out inside out.
    Scene s;
    const bool exact = brep::available();
    s.setDefaultBackend(exact ? Backend::Brep : Backend::Mesh);

    PrimitiveSpec boxSpec;
    boxSpec.kind = PrimitiveKind::Box;
    boxSpec.box = {30.0, 20.0, 10.0};
    const ObjectId box = s.addPrimitive(PrimitiveKind::Box, boxSpec, {0, 0, 5});
    s.find(box)->name = "Base <plate> & 'lid'";

    PrimitiveSpec cylSpec;
    cylSpec.kind = PrimitiveKind::Cylinder;
    cylSpec.cylinder.radius = 6.0;
    cylSpec.cylinder.height = 25.0;
    const ObjectId cyl = s.addPrimitive(PrimitiveKind::Cylinder, cylSpec, {60, 10, 0});
    s.find(cyl)->name = "Peg";
    s.find(cyl)->transform.rotation = Quat::fromAxisAngle(normalize(Vec3{1, 1, 0}), 0.7);

    Mesh sphere;
    makeSphere(sphere, {8.0, 32, 16});
    const ObjectId ball = s.addBody(Body(std::move(sphere)), {-40, 0, 8}, "Ball");

    const ObjectId flipped = s.addPrimitive(PrimitiveKind::Box, boxSpec, {0, 50, 5});
    s.find(flipped)->name = "Mirrored";
    s.find(flipped)->transform.scale = {-1.0, 1.0, 1.0};
    s.reevaluate(flipped);

    const std::string path = tmp("parts.3mf");
    ThreeMfOptions opt;
    opt.deviationMm = 0.01;
    const ThreeMfResult r = export3mf(s, path, opt);
    check(r.ok, "3MF export succeeds: " + r.error);
    check(r.objects == 4, "four objects written");
    check(r.openObjects == 0, "every one of them closed");
    check(r.meshBodies == (exact ? 1u : 4u), "the mesh bodies are counted");

    std::map<std::string, std::string> parts;
    std::string why;
    check(readZip(path, parts, why), "the archive reads back: " + why);
    check(parts.size() == 3, "three parts in the package");
    check(parts.count("[Content_Types].xml") && parts["[Content_Types].xml"].find(
              "application/vnd.ms-package.3dmanufacturing-3dmodel+xml") != std::string::npos,
          "the content types name the model");
    check(parts.count("_rels/.rels") &&
              parts["_rels/.rels"].find("Target=\"/3D/3dmodel.model\"") != std::string::npos,
          "the relationship points at the model");
    const std::string& xml = parts["3D/3dmodel.model"];
    check(xml.find("unit=\"millimeter\"") != std::string::npos, "in millimetres");

    const std::vector<ReadObject> objects = parseModel(xml);
    check(objects.size() == 4, "four objects in the model");
    check(xml.find("name=\"Base &lt;plate&gt; &amp; &apos;lid&apos;\"") != std::string::npos,
          "a name with markup in it is escaped");
    size_t items = 0;
    for (size_t at = 0; (at = xml.find("<item ", at)) != std::string::npos; ++at) ++items;
    check(items == 4, "and each is placed in the build");

    // What each object ought to enclose, from the scene itself.
    std::unordered_map<std::string, double> want;
    for (ObjectId id : {box, cyl, ball, flipped}) {
        const SceneObject* o = s.find(id);
        want[o->name] = o->body.health(false).volume;
    }
    for (const ReadObject& o : objects) {
        const std::string unescaped = o.name == "Base &lt;plate&gt; &amp; &apos;lid&apos;"
                                          ? "Base <plate> & 'lid'" : o.name;
        check(closedOutward(o), o.name + ": closed, and wound outward");
        const double v = enclosed(o);
        const double expected = want[unescaped];
        // The box is flat-faced, so its triangles are exact. The cylinder's
        // walls sit inside the true surface by at most the tolerance; the mesh
        // sphere goes out as it is, so it matches its own volume exactly.
        const double slack = unescaped == "Peg" ? 2.0 * kPi * 6.0 * 25.0 * 0.01 : 1e-6 * expected;
        check(std::fabs(v - expected) <= slack + 1e-6,
              o.name + ": encloses " + std::to_string(v) + " of " + std::to_string(expected));
    }

    // Where they are: the peg's bounds in the file are its world bounds.
    for (const ReadObject& o : objects) {
        if (o.name != "Peg") continue;
        AABB b;
        for (const Vec3& p : o.vertices) b.expand(p);
        const AABB w = s.find(cyl)->worldBounds();
        check(near(b.center().x, w.center().x, 0.1) && near(b.center().y, w.center().y, 0.1) &&
                  near(b.center().z, w.center().z, 0.1),
              "the peg is written where it stands in the scene");
    }
    std::printf("[3mf] %zu objects, %zu triangles, %zu vertices, %zu bytes of model\n",
                r.objects, r.triangles, r.vertices, xml.size());
    std::remove(path.c_str());

    // Selection only takes what is selected.
    s.clearSelection();
    s.select(cyl);
    opt.selectionOnly = true;
    const ThreeMfResult one = export3mf(s, path, opt);
    check(one.ok && one.objects == 1, "selection only writes the one selected object");
    parts.clear();
    check(readZip(path, parts, why) && parseModel(parts["3D/3dmodel.model"]).size() == 1,
          "and the file holds just that one");
    std::remove(path.c_str());

    // Nothing to write is a refusal, and leaves no file behind.
    s.clearSelection();
    const ThreeMfResult none = export3mf(s, path, opt);
    check(!none.ok && !none.error.empty(), "an empty selection is refused with a reason");
    check(!std::filesystem::exists(path), "and no file is left behind");

    // A place that cannot be written to is a refusal too.
    opt.selectionOnly = false;
    const ThreeMfResult nowhere = export3mf(s, tmp("no_such_dir/parts.3mf"), opt);
    check(!nowhere.ok && !nowhere.error.empty(), "an unwritable path is refused with a reason");
    std::printf("[3mf] selection, empty selection and bad path handled\n");
}

// One STL per object, named for the object.
static void testStlPerObject() {
    Scene s;
    s.setDefaultBackend(brep::available() ? Backend::Brep : Backend::Mesh);
    const ObjectId a = s.addPrimitive(PrimitiveKind::Box, {}, {0, 0, 10});
    const ObjectId b = s.addPrimitive(PrimitiveKind::Box, {}, {40, 0, 10});
    const ObjectId c = s.addPrimitive(PrimitiveKind::Box, {}, {80, 0, 10});
    s.find(a)->name = "Lid";
    s.find(b)->name = "Lid";            // the same name twice
    s.find(c)->name = "Base/Left";      // a character no file name can have

    const std::string base = tmp("kit.stl");
    StlOptions opt;
    opt.separateFiles = true;
    const StlResult r = exportStl(s, base, opt);
    check(r.ok && r.files.size() == 3, "three files, one per object: " + r.error);
    const std::vector<std::string> want = {tmp("kit - Lid.stl"), tmp("kit - Lid 2.stl"),
                                           tmp("kit - Base_Left.stl")};
    check(r.files == want, "named for their objects, a repeat numbered, a slash replaced");
    for (const std::string& file : r.files) {
        std::vector<StlTri> tris;
        check(readBinaryStl(file, tris) && near(stlVolume(tris), 8000.0, 1e-2),
              file + " holds one 20mm cube");
        std::remove(file.c_str());
    }
    std::printf("[stl] one file per object: %zu files\n", r.files.size());
}

int main() {
    testExportTolerance();
    testOlderFileOpens();
    test3mf();
    testStlPerObject();

    // ---- STL ---------------------------------------------------------------
    {
        Scene s;
        s.addPrimitive(PrimitiveKind::Box);
        const std::string path = tmp("box.stl");

        const StlResult r = exportStl(s, path);
        check(r.ok, "binary export succeeds: " + r.error);
        check(r.triangles == 12, "a box is twelve triangles");
        check(r.objects == 1, "one object written");

        std::vector<StlTri> tris;
        check(readBinaryStl(path, tris), "file parses as binary STL");
        check(tris.size() == 12, "twelve triangles on disk");

        // Volume from the written bytes: the geometry survived, right way out.
        check(near(stlVolume(tris), 8000.0, 1e-2), "written volume is 8000");
        std::printf("[stl] box: %zu triangles, volume %.2f\n", tris.size(), stlVolume(tris));

        // Normals must agree with the winding, or a slicer sees inverted faces.
        int disagreeing = 0;
        for (const StlTri& t : tris) {
            const Vec3 a{t.v[0][0], t.v[0][1], t.v[0][2]};
            const Vec3 b{t.v[1][0], t.v[1][1], t.v[1][2]};
            const Vec3 c{t.v[2][0], t.v[2][1], t.v[2][2]};
            const Vec3 geo = normalize(cross(b - a, c - a));
            const Vec3 n{t.n[0], t.n[1], t.n[2]};
            if (dot(geo, n) < 0.99) ++disagreeing;
        }
        check(disagreeing == 0, "every normal matches its winding");
    }

    // An object's transform has to be baked in: STL has no concept of one.
    {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        s.find(id)->transform.position = {100, 0, 0};
        s.find(id)->transform.scale = {2, 1, 1};

        const std::string path = tmp("moved.stl");
        check(exportStl(s, path).ok, "export a transformed object");

        std::vector<StlTri> tris;
        check(readBinaryStl(path, tris), "parses");
        float minX = 1e9f, maxX = -1e9f;
        for (const StlTri& t : tris)
            for (int k = 0; k < 3; ++k) {
                minX = std::min(minX, t.v[k][0]);
                maxX = std::max(maxX, t.v[k][0]);
            }
        check(near(minX, 80.0, 1e-3) && near(maxX, 120.0, 1e-3),
              "position and scale are baked into the coordinates");
        check(near(stlVolume(tris), 16000.0, 1e-1), "and the volume doubled");
        std::printf("[stl] transformed: x %.1f..%.1f, volume %.1f\n",
                    minX, maxX, stlVolume(tris));
    }

    // Selection-only, and refusing to write nothing.
    {
        Scene s;
        const ObjectId a = s.addPrimitive(PrimitiveKind::Box);
        s.addPrimitive(PrimitiveKind::Box, {}, Vec3{100, 0, 0});
        s.select(a);

        StlOptions opt;
        opt.selectionOnly = true;
        const StlResult r = exportStl(s, tmp("sel.stl"), opt);
        check(r.ok && r.objects == 1 && r.triangles == 12, "selection only writes one object");

        s.clearSelection();
        const StlResult none = exportStl(s, tmp("none.stl"), opt);
        check(!none.ok, "nothing selected is refused");
        check(!none.error.empty(), "with a reason");
        std::printf("[stl] empty selection: %s\n", none.error.c_str());
    }

    // ASCII output.
    {
        Scene s;
        s.addPrimitive(PrimitiveKind::Box);
        StlOptions opt;
        opt.binary = false;
        const std::string path = tmp("box_ascii.stl");
        check(exportStl(s, path, opt).ok, "ascii export succeeds");

        std::ifstream f(path);
        std::string text((std::istreambuf_iterator<char>(f)),
                         std::istreambuf_iterator<char>());
        size_t facets = 0;
        for (size_t at = text.find("facet normal"); at != std::string::npos;
             at = text.find("facet normal", at + 1)) ++facets;
        check(facets == 12, "twelve facets in the text");
        check(text.rfind("solid ", 0) == 0, "starts with solid");
        std::printf("[stl] ascii: %zu facets, %zu bytes\n", facets, text.size());
    }

    // ---- Project round-trip -------------------------------------------------
    // The chain has modelling steps in it, so it needs the exact kernel.
    if (brep::available()) {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        s.find(id)->name = "Bracket";
        // Moved the way a user moves it: a step in its history, which has to
        // come back from the file and put the object back where it was.
        check(s.recordMove(id, {5, 6, 7}), "moved");

        // A chain with something of every interesting kind in it.
        //
        // The face is chosen by which way it points, not by handle: handles are
        // whatever the backend numbered them, and "face 0" is the top face on
        // one kernel and a side face on the other. Asking for the top face is
        // what the test means, and it is what a user does.
        const Body& start = s.find(id)->body;
        FaceId top = kNoFace;
        {
            std::vector<FaceId> faces;
            start.allFaces(faces);
            for (FaceId f : faces)
                if (dot(start.faceNormal(f), Vec3{0, 0, 1}) > 0.99) top = f;
        }
        check(top != kNoFace, "found the top face to extrude");

        Feature ext;
        ext.kind = FeatureKind::Extrude;
        ext.faces = nameFaces(start, {top});
        ext.distance = 6.0;
        check(s.addFeature(id, ext), "extrude added");

        // Two edges of the face that is now on top, for the same reason.
        std::vector<EdgeId> topEdges;
        {
            const Body& b = s.find(id)->body;
            std::vector<FaceId> faces;
            b.allFaces(faces);
            for (FaceId f : faces)
                if (dot(b.faceNormal(f), Vec3{0, 0, 1}) > 0.99) { b.faceEdges(f, topEdges); break; }
        }
        check(topEdges.size() >= 2, "the top face has edges to round");

        Feature fil;
        fil.kind = FeatureKind::Bevel;
        fil.edges.ids = {s.find(id)->body.edgeName(topEdges[0]),
                         s.find(id)->body.edgeName(topEdges[1])};
        fil.radii = {2.0, 3.5};   // a radius per edge, as Fusion's fillet has
        fil.width = 2.0;
        fil.segments = 4;
        check(s.addFeature(id, fil), "fillet added");

        const int facesBefore = s.find(id)->body.faceCount();
        const AABB boundsBefore = s.find(id)->localBounds;
        const size_t chainBefore = s.find(id)->features.size();

        const std::string path = tmp("project.tangent");
        const ProjectResult saved = saveProject(s, path);
        check(saved.ok, "save succeeds: " + saved.error);

        Scene loaded;
        const ProjectResult read = loadProject(loaded, path);
        check(read.ok, "load succeeds: " + read.error);
        check(loaded.objectCount() == 1, "one object back");

        const SceneObject* o = loaded.objects().front().get();
        check(o->name == "Bracket", "name survived");
        check(near(o->transform.position.x, 5.0) && near(o->transform.position.z, 7.0),
              "transform survived");
        check(o->features.size() == chainBefore, "the whole chain survived");
        {
            const Feature& f = o->features.back();
            check(f.kind == FeatureKind::Bevel, "the fillet is still a fillet");
            check(f.radii.size() == 2 && near(f.radii[0], 2.0) && near(f.radii[1], 3.5),
                  "each edge kept its own radius");
        }
        check(o->body.faceCount() == facesBefore, "re-evaluates to the same mesh");
        check(near(o->localBounds.size().z, boundsBefore.size().z, 1e-9),
              "same dimensions");
        // Version 6 added the shell's wall thickness. A field written at the
        // end of a feature is the easiest kind to get subtly wrong, so it is
        // checked by value rather than by the file merely loading.
        if (brep::available()) {
            Scene s2;
            s2.setDefaultBackend(Backend::Brep);
            const ObjectId id2 = s2.addPrimitive(PrimitiveKind::Box);
            Feature sh;
            sh.kind = FeatureKind::Shell;
            sh.thickness = 1.75;
            std::string why2;
            check(s2.addFeature(id2, sh, &why2), std::string("shelled: ") + why2);
            const Real hollow = s2.find(id2)->body.health(false).volume;

            const std::string p2 = tmp("shell.tng");
            check(saveProject(s2, p2).ok, "saved a shelled project");
            Scene back;
            check(loadProject(back, p2).ok, "loaded it again");
            const SceneObject* o2 = back.objects().front().get();
            check(o2->features.size() == 2, "both features came back");
            check(o2->features[1].kind == FeatureKind::Shell, "the shell is still a shell");
            check(near(o2->features[1].thickness, 1.75), "and kept its wall thickness");
            check(near(o2->body.health(false).volume, hollow, 1e-6),
                  "re-evaluating gives the same hollow body");
            std::remove(p2.c_str());
            std::printf("[project] a shell round-trips at version %u\n", kProjectVersion);
        }

        std::printf("[project] round trip: %zu features, %d faces\n",
                    o->features.size(), o->body.faceCount());

        // And it is still parametric after loading.
        SceneObject* rw = loaded.find(o->id);
        rw->spec.box.width = 40.0;
        check(loaded.rebuild(rw->id), "re-evaluates after loading");
        check(near(rw->localBounds.size().x, 40.0, 1e-9), "base change applies");
        std::printf("[project] still parametric after load\n");
    }

    // A boolean's baked tool body has to survive too. Booleans are the exact
    // kernel's, so this needs one.
    if (brep::available()) {
        Scene s;
        s.setDefaultBackend(Backend::Brep);
        const ObjectId a = s.addPrimitive(PrimitiveKind::Box);
        PrimitiveSpec toolSpec;
        toolSpec.kind = PrimitiveKind::Box;
        Body tool;
        check(makePrimitive(toolSpec, tool, Backend::Brep), "tool built");
        tool.transform(translate(Vec3{10, 0, 0}));

        Feature b;
        b.kind = FeatureKind::Boolean;
        b.booleanOp = BooleanOp::Difference;
        b.bakedBody = tool;
        check(s.addFeature(a, b), "boolean added");
        const int facesBefore = s.find(a)->body.faceCount();

        const std::string path = tmp("bool.tangent");
        check(saveProject(s, path).ok, "save");
        Scene loaded;
        check(loadProject(loaded, path).ok, "load");
        check(loaded.objects().front()->body.faceCount() == facesBefore,
              "baked tool body survived");
        std::printf("[project] baked boolean body survived (%d faces)\n", facesBefore);
    }

    // ---- Refusing bad files -------------------------------------------------
    {
        Scene s;
        s.addPrimitive(PrimitiveKind::Sphere);
        const size_t before = s.objectCount();

        check(!loadProject(s, tmp("does_not_exist.tangent")).ok, "missing file refused");
        check(s.objectCount() == before, "scene untouched");

        // Something that is not a project at all.
        const std::string junk = tmp("junk.tangent");
        { std::ofstream f(junk, std::ios::binary); f << "not a tangent project at all"; }
        const ProjectResult r = loadProject(s, junk);
        check(!r.ok, "foreign file refused");
        check(s.objectCount() == before, "scene still untouched");
        std::printf("[project] foreign file: %s\n", r.error.c_str());

        // A truncated project: valid header, cut off mid-way.
        Scene good;
        good.addPrimitive(PrimitiveKind::Box);
        const std::string full = tmp("full.tangent");
        check(saveProject(good, full).ok, "save a good one");

        std::ifstream in(full, std::ios::binary | std::ios::ate);
        const std::streamsize size = in.tellg();
        in.seekg(0);
        std::vector<char> bytes(static_cast<size_t>(size));
        in.read(bytes.data(), size);

        const std::string cut = tmp("cut.tangent");
        { std::ofstream f(cut, std::ios::binary);
          f.write(bytes.data(), static_cast<std::streamsize>(bytes.size() / 2)); }

        const ProjectResult t = loadProject(s, cut);
        check(!t.ok, "truncated file refused");
        check(s.objectCount() == before, "scene survives a truncated load");
        std::printf("[project] truncated file: %s\n", t.error.c_str());

        // A future version.
        const std::string future = tmp("future.tangent");
        { std::vector<char> v = bytes;
          v[8] = 99;   // version field
          std::ofstream f(future, std::ios::binary);
          f.write(v.data(), static_cast<std::streamsize>(v.size())); }
        const ProjectResult fv = loadProject(s, future);
        check(!fv.ok, "unknown version refused rather than guessed at");
        std::printf("[project] version check: %s\n", fv.error.c_str());
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
