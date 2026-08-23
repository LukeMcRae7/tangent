// STL export and project round-trips.
#include "mesh/export_stl.h"
#include "scene/serialize.h"

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

static std::string tmp(const char* name) {
    const char* dir = std::getenv("TMPDIR");
    return std::string(dir ? dir : "/tmp") + "/tangent_test_" + name;
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

int main() {
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
    {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        s.find(id)->name = "Bracket";
        s.find(id)->transform.position = {5, 6, 7};

        // A chain with something of every interesting kind in it.
        Feature ext;
        ext.kind = FeatureKind::Extrude;
        ext.faces = {0};
        ext.distance = 6.0;
        check(s.addFeature(id, ext), "extrude added");

        Feature fil;
        fil.kind = FeatureKind::Bevel;
        fil.edges = {0, 2};
        fil.radii = {2.0, 3.5};   // a radius per edge, as Fusion's fillet has
        fil.width = 2.0;
        fil.segments = 4;
        check(s.addFeature(id, fil), "fillet added");

        const int facesBefore = s.find(id)->mesh.faceCount();
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
        check(o->mesh.faceCount() == facesBefore, "re-evaluates to the same mesh");
        check(near(o->localBounds.size().z, boundsBefore.size().z, 1e-9),
              "same dimensions");
        std::printf("[project] round trip: %zu features, %d faces\n",
                    o->features.size(), o->mesh.faceCount());

        // And it is still parametric after loading.
        SceneObject* rw = loaded.find(o->id);
        rw->spec.box.width = 40.0;
        check(loaded.rebuild(rw->id), "re-evaluates after loading");
        check(near(rw->localBounds.size().x, 40.0, 1e-9), "base change applies");
        std::printf("[project] still parametric after load\n");
    }

    // A boolean's baked tool body has to survive too.
    {
        Scene s;
        const ObjectId a = s.addPrimitive(PrimitiveKind::Box);
        Mesh tool;
        BoxParams p;
        makeBox(tool, p);
        for (MeshVertex& v : tool.verts) v.position += Vec3{10, 0, 0};

        Feature b;
        b.kind = FeatureKind::Boolean;
        b.booleanOp = BooleanOp::Difference;
        b.bakedMesh = tool;
        check(s.addFeature(a, b), "boolean added");
        const int facesBefore = s.find(a)->mesh.faceCount();

        const std::string path = tmp("bool.tangent");
        check(saveProject(s, path).ok, "save");
        Scene loaded;
        check(loadProject(loaded, path).ok, "load");
        check(loaded.objects().front()->mesh.faceCount() == facesBefore,
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
