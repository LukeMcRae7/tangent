#include "scene/serialize.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <vector>

namespace tg {
namespace {

constexpr char kMagic[8] = {'T', 'A', 'N', 'G', 'E', 'N', 'T', '\0'};

// ---- writing --------------------------------------------------------------
struct Writer {
    std::vector<unsigned char> buf;

    void raw(const void* p, size_t n) {
        const unsigned char* b = static_cast<const unsigned char*>(p);
        buf.insert(buf.end(), b, b + n);
    }
    void u8(uint8_t v)   { buf.push_back(v); }
    void u32(uint32_t v) { for (int i = 0; i < 4; ++i) buf.push_back((v >> (i * 8)) & 0xFF); }
    void i32(int32_t v)  { u32(static_cast<uint32_t>(v)); }
    void f64(double v)   { uint64_t b; std::memcpy(&b, &v, 8);
                           for (int i = 0; i < 8; ++i) buf.push_back((b >> (i * 8)) & 0xFF); }
    void vec3(const Vec3& v) { f64(v.x); f64(v.y); f64(v.z); }
    void u64(uint64_t v) { for (int i = 0; i < 8; ++i) buf.push_back(static_cast<uint8_t>((v >> (i * 8)) & 0xFF)); }
    void ids(const std::vector<ElementId>& v) {
        u32(static_cast<uint32_t>(v.size()));
        for (ElementId x : v) u64(x);
    }
    void refs(const ElementRefs& r) {
        u32(static_cast<uint32_t>(r.kind));
        ids(r.ids);
        u64(r.face);
    }
    void text(const std::string& s) { u32(static_cast<uint32_t>(s.size())); raw(s.data(), s.size()); }

    template <typename T> void indices(const std::vector<T>& v) {
        u32(static_cast<uint32_t>(v.size()));
        for (T x : v) i32(static_cast<int32_t>(x));
    }
};

// ---- reading --------------------------------------------------------------
struct Reader {
    const unsigned char* p = nullptr;
    const unsigned char* end = nullptr;
    bool bad = false;

    bool need(size_t n) {
        if (bad || static_cast<size_t>(end - p) < n) { bad = true; return false; }
        return true;
    }
    uint8_t u8() { if (!need(1)) return 0; return *p++; }
    uint32_t u32() {
        if (!need(4)) return 0;
        uint32_t v = 0;
        for (int i = 0; i < 4; ++i) v |= static_cast<uint32_t>(*p++) << (i * 8);
        return v;
    }
    int32_t i32() { return static_cast<int32_t>(u32()); }
    uint64_t u64() {
        if (!need(8)) return 0;
        uint64_t v = 0;
        for (int i = 0; i < 8; ++i) v |= static_cast<uint64_t>(*p++) << (i * 8);
        return v;
    }
    std::vector<ElementId> ids() {
        const uint32_t n = u32();
        std::vector<ElementId> v;
        if (!need(static_cast<size_t>(n) * 8)) return v;
        v.reserve(n);
        for (uint32_t i = 0; i < n; ++i) v.push_back(u64());
        return v;
    }
    ElementRefs refs() {
        ElementRefs r;
        const uint32_t k = u32();
        r.kind = k <= static_cast<uint32_t>(ElementRefs::Kind::All)
               ? static_cast<ElementRefs::Kind>(k) : ElementRefs::Kind::Explicit;
        r.ids = ids();
        r.face = u64();
        return r;
    }
    double f64() {
        if (!need(8)) return 0.0;
        uint64_t b = 0;
        for (int i = 0; i < 8; ++i) b |= static_cast<uint64_t>(*p++) << (i * 8);
        double v;
        std::memcpy(&v, &b, 8);
        return v;
    }
    Vec3 vec3() { const double x = f64(), y = f64(), z = f64(); return {x, y, z}; }
    std::string text() {
        const uint32_t n = u32();
        // A corrupt length must not be trusted as an allocation size.
        if (!need(n)) return {};
        std::string s(reinterpret_cast<const char*>(p), n);
        p += n;
        return s;
    }
    template <typename T> std::vector<T> indices() {
        const uint32_t n = u32();
        if (!need(static_cast<size_t>(n) * 4)) return {};
        std::vector<T> v;
        v.reserve(n);
        for (uint32_t i = 0; i < n; ++i) v.push_back(static_cast<T>(i32()));
        return v;
    }
};

// ---- mesh -----------------------------------------------------------------
void writeMesh(Writer& w, const Mesh& m) {
    w.u32(static_cast<uint32_t>(m.verts.size()));
    for (const MeshVertex& v : m.verts) w.vec3(v.position);

    // Stored as a polygon soup, not as half-edge links: connectivity is
    // derived on load through Mesh::build, which re-runs the manifold checks.
    // A file can therefore never introduce topology the kernel would reject.
    w.u32(static_cast<uint32_t>(m.faceCount()));
    std::vector<Index> verts;
    for (Index f = 0; f < m.faceCount(); ++f) {
        m.faceVertices(f, verts);
        w.u32(static_cast<uint32_t>(verts.size()));
        for (Index v : verts) w.i32(v);
    }
}

// A body is written with a tag for which representation it is, so a file
// written before a second backend exists still reads afterwards, and one
// written with a B-rep body fails loudly on a build that has no backend for it
// rather than being misread as a mesh.
enum : uint32_t { kBodyMesh = 1, kBodyBrep = 2 };

bool readMesh(Reader& r, Mesh& out) {
    const uint32_t vertCount = r.u32();
    if (r.bad || !r.need(static_cast<size_t>(vertCount) * 24)) return false;

    std::vector<Vec3> positions;
    positions.reserve(vertCount);
    for (uint32_t i = 0; i < vertCount; ++i) positions.push_back(r.vec3());

    const uint32_t faceCount = r.u32();
    std::vector<uint32_t> sizes, idx;
    sizes.reserve(faceCount);
    for (uint32_t f = 0; f < faceCount; ++f) {
        const uint32_t n = r.u32();
        if (r.bad || n < 3 || n > 100000) return false;
        sizes.push_back(n);
        for (uint32_t k = 0; k < n; ++k) {
            const int32_t v = r.i32();
            if (v < 0 || static_cast<uint32_t>(v) >= vertCount) return false;
            idx.push_back(static_cast<uint32_t>(v));
        }
    }
    if (r.bad) return false;
    if (faceCount == 0) { out.clear(); return true; }
    return out.build(positions, sizes, idx);
}

// ---- feature --------------------------------------------------------------
void writeSpec(Writer& w, const PrimitiveSpec& s) {
    w.u32(static_cast<uint32_t>(s.kind));
    w.f64(s.box.width); w.f64(s.box.depth); w.f64(s.box.height);
    w.f64(s.cylinder.radius); w.f64(s.cylinder.height); w.i32(s.cylinder.segments);
    w.f64(s.sphere.radius); w.i32(s.sphere.segments); w.i32(s.sphere.rings);
    w.f64(s.cone.bottomRadius); w.f64(s.cone.topRadius); w.f64(s.cone.height);
    w.i32(s.cone.segments);
    w.f64(s.torus.majorRadius); w.f64(s.torus.minorRadius);
    w.i32(s.torus.majorSegments); w.i32(s.torus.minorSegments);
    w.f64(s.plane.width); w.f64(s.plane.depth);
}

bool readSpec(Reader& r, PrimitiveSpec& s) {
    const uint32_t kind = r.u32();
    if (kind > static_cast<uint32_t>(PrimitiveKind::Custom)) return false;
    s.kind = static_cast<PrimitiveKind>(kind);
    s.box.width = r.f64(); s.box.depth = r.f64(); s.box.height = r.f64();
    s.cylinder.radius = r.f64(); s.cylinder.height = r.f64(); s.cylinder.segments = r.i32();
    s.sphere.radius = r.f64(); s.sphere.segments = r.i32(); s.sphere.rings = r.i32();
    s.cone.bottomRadius = r.f64(); s.cone.topRadius = r.f64(); s.cone.height = r.f64();
    s.cone.segments = r.i32();
    s.torus.majorRadius = r.f64(); s.torus.minorRadius = r.f64();
    s.torus.majorSegments = r.i32(); s.torus.minorSegments = r.i32();
    s.plane.width = r.f64(); s.plane.depth = r.f64();
    return !r.bad;
}

// A sketch is written whole -- plane, points, entities, constraints -- with
// every reference kept as the id it had. Ids are never reused within a sketch,
// so reading it back gives the same sketch rather than one renumbered.
void writeSketch(Writer& w, const Sketch& s) {
    w.vec3(s.plane.origin);
    w.vec3(s.plane.xAxis);
    w.vec3(s.plane.yAxis);
    w.u32(s.nextId);
    w.u32(static_cast<uint32_t>(s.points.size()));
    for (const SketchPoint& p : s.points) {
        w.u32(p.id);
        w.f64(p.at.x);
        w.f64(p.at.y);
    }
    w.u32(static_cast<uint32_t>(s.entities.size()));
    for (const SketchEntity& e : s.entities) {
        w.u32(e.id);
        w.u8(static_cast<uint8_t>(e.curve));
        w.u8(e.construction ? 1 : 0);
        w.u32(e.a);
        w.u32(e.b);
        w.u32(e.c);
        w.u32(e.d);
        w.f64(e.radius);
    }
    w.u32(static_cast<uint32_t>(s.constraints.size()));
    for (const SketchConstraint& k : s.constraints) {
        w.u32(k.id);
        w.u8(static_cast<uint8_t>(k.rule));
        w.u32(k.first);
        w.u32(k.second);
        w.f64(k.value);
        w.f64(k.value2);
    }
}

bool readSketch(Reader& r, Sketch& s) {
    s.plane.origin = r.vec3();
    s.plane.xAxis = r.vec3();
    s.plane.yAxis = r.vec3();
    s.nextId = r.u32();

    // Each count is checked against what is left of the file before anything is
    // allocated for it: a corrupt count must not become a four-gigabyte vector.
    const uint32_t points = r.u32();
    if (r.bad || !r.need(static_cast<size_t>(points) * 20)) return false;
    s.points.resize(points);
    for (SketchPoint& p : s.points) {
        p.id = r.u32();
        p.at.x = r.f64();
        p.at.y = r.f64();
    }

    const uint32_t entities = r.u32();
    if (r.bad || !r.need(static_cast<size_t>(entities) * 30)) return false;
    s.entities.resize(entities);
    for (SketchEntity& e : s.entities) {
        e.id = r.u32();
        const uint8_t curve = r.u8();
        if (curve > static_cast<uint8_t>(SketchCurve::Bezier)) return false;
        e.curve = static_cast<SketchCurve>(curve);
        e.construction = r.u8() != 0;
        e.a = r.u32();
        e.b = r.u32();
        e.c = r.u32();
        e.d = r.u32();
        e.radius = r.f64();
    }

    const uint32_t constraints = r.u32();
    if (r.bad || !r.need(static_cast<size_t>(constraints) * 29)) return false;
    s.constraints.resize(constraints);
    for (SketchConstraint& k : s.constraints) {
        k.id = r.u32();
        const uint8_t rule = r.u8();
        if (rule > static_cast<uint8_t>(SketchRule::Angle)) return false;
        k.rule = static_cast<SketchRule>(rule);
        k.first = r.u32();
        k.second = r.u32();
        k.value = r.f64();
        k.value2 = r.f64();
    }
    return !r.bad;
}

void writeFeature(Writer& w, const Feature& f) {
    w.u32(static_cast<uint32_t>(f.kind));
    w.u8(f.enabled ? 1 : 0);
    writeSpec(w, f.primitive);
    w.u64(f.uid);
    w.refs(f.faces);
    w.refs(f.edges);
    w.u32(static_cast<uint32_t>(f.radii.size()));
    for (Real x : f.radii) w.f64(x);
    w.f64(f.distance);
    w.f64(f.amount);
    w.f64(f.width);
    w.i32(f.segments);
    w.u32(static_cast<uint32_t>(f.booleanOp));
    w.ids(f.verts);
    w.u32(static_cast<uint32_t>(f.offsets.size()));
    for (const Vec3& o : f.offsets) w.vec3(o);
    if (f.bakedBody.isMesh()) {
        w.u32(kBodyMesh);
        writeMesh(w, f.bakedBody.mesh());
    } else {
        // The shape in OCCT's own text, and the names beside it. Writing an
        // empty mesh here instead -- which is what a tag-blind writer would do
        // -- would save a file that loads as a model with a hole where the cut
        // used to be, and say nothing about it.
        std::string shape;
        std::vector<ElementId> names;
        if (brep::encode(f.bakedBody.brep(), shape, names)) {
            w.u32(kBodyBrep);
            w.text(shape);
            w.ids(names);
        } else {
            w.u32(kBodyMesh);
            writeMesh(w, Mesh{});
        }
    }
    w.u32(static_cast<uint32_t>(f.backend));
    w.f64(f.thickness);

    // Version 7: the angle a face was tipped through, and the line it was
    // tipped about -- which is also the plane a divide cuts on.
    w.f64(f.angle);
    w.f64(f.axisPoint.x); w.f64(f.axisPoint.y); w.f64(f.axisPoint.z);
    w.f64(f.axisDir.x);   w.f64(f.axisDir.y);   w.f64(f.axisDir.z);
    w.u32(f.mergeFlush ? 1u : 0u);

    // Version 8: how much a face was grown or shrunk.
    w.f64(f.scale);

    // Version 9: whether a bevel cuts flat, and where its radius ends up.
    w.u32(f.chamfer ? 1u : 0u);
    w.f64(f.endWidth);
    // v10.
    w.u32(static_cast<uint32_t>(f.patternMode));
    w.i32(f.patternCount);
    // v11.
    w.f64(f.reduceTolerance);
    w.i32(f.reduceTarget);
    w.u32(f.reduceLoosen ? 1u : 0u);
    // v12: a sketch, and which region of which sketch an extrusion sweeps.
    writeSketch(w, f.sketch);
    w.u64(f.sketchUid);
    w.u32(f.profileKeys.empty() ? kNoSketchId : f.profileKeys.front());
    w.u8(f.sketchShown ? 1 : 0);
    // v14: where a Move or Rotate took the object, and a Scale's stretch.
    w.vec3(f.moveBy);
    w.f64(f.turnBy.x); w.f64(f.turnBy.y); w.f64(f.turnBy.z); w.f64(f.turnBy.w);
    w.vec3(f.turnAbout);
    w.vec3(f.scaleBy);
    w.vec3(f.scaleAbout);
    w.text(f.toolName);
    // v15: the regions after the first, for an extrusion of several.
    w.u32(static_cast<uint32_t>(f.profileKeys.size() > 1 ? f.profileKeys.size() - 1 : 0));
    for (size_t i = 1; i < f.profileKeys.size(); ++i) w.u32(f.profileKeys[i]);
    // v16: whether a face was pulled along an axis rather than its own normal.
    w.u8(f.alongAxis ? 1 : 0);
    // v17: the axis a revolve turns about, in its sketch's own coordinates,
    // and how far round.
    w.f64(f.revolveAxisAt.x);  w.f64(f.revolveAxisAt.y);
    w.f64(f.revolveAxisDir.x); w.f64(f.revolveAxisDir.y);
    w.f64(f.revolveAngle);
}

// `version` is the file's, not this build's: a project written before bodies
// could be exact has no representation tag to read and no backend field after
// it, and refusing to open it would be losing someone's work over a field that
// has one possible value.
bool readFeature(Reader& r, Feature& f, uint32_t version) {
    const uint32_t kind = r.u32();
    // The last of the enum, kept beside it: a kind added later and not added
    // here would be read as a corrupt file by the build that has it.
    if (kind > static_cast<uint32_t>(kLastFeatureKind)) return false;
    f.kind = static_cast<FeatureKind>(kind);
    f.enabled = r.u8() != 0;
    if (!readSpec(r, f.primitive)) return false;
    f.uid   = r.u64();
    f.faces = r.refs();
    f.edges = r.refs();
    {
        const uint32_t n = r.u32();
        if (n > f.edges.count()) return false;
        f.radii.resize(n);
        for (uint32_t i = 0; i < n; ++i) f.radii[i] = r.f64();
    }
    f.distance = r.f64();
    f.amount = r.f64();
    f.width = r.f64();
    f.segments = r.i32();
    const uint32_t op = r.u32();
    if (op > static_cast<uint32_t>(BooleanOp::Intersection)) return false;
    f.booleanOp = static_cast<BooleanOp>(op);
    f.verts = r.ids();
    const uint32_t offsetCount = r.u32();
    if (r.bad || !r.need(static_cast<size_t>(offsetCount) * 24)) return false;
    f.offsets.clear();
    f.offsets.reserve(offsetCount);
    for (uint32_t i = 0; i < offsetCount; ++i) f.offsets.push_back(r.vec3());
    const uint32_t bodyTag = r.u32();
    if (version < 5 && bodyTag != kBodyMesh) return false;   // v4 had only meshes
    if (bodyTag == kBodyMesh) {
        Mesh m;
        if (!readMesh(r, m)) return false;
        f.bakedBody = Body(std::move(m));
    } else if (bodyTag == kBodyBrep) {
        const std::string shape = r.text();
        const std::vector<ElementId> names = r.ids();
        if (r.bad) return false;
        BrepRef s = brep::decode(shape, names);
        if (!s) return false;      // a build without the backend says so loudly
        f.bakedBody = Body(std::move(s));
    } else {
        return false;
    }
    if (version >= 5) {
        const uint32_t backend = r.u32();
        if (backend > static_cast<uint32_t>(Backend::Brep)) return false;
        f.backend = static_cast<Backend>(backend);
    } else {
        f.backend = Backend::Mesh;   // everything in a v4 file was a mesh
    }
    // Version 6 added the shell. A file older than that has no shell features
    // in it, so the default wall stands and nothing reads it.
    if (version >= 6) f.thickness = r.f64();

    // Version 7 added rotating a face and dividing one, and the geometry both
    // of those hang on. At the end of the record, not in the middle of it: a
    // field written among the others is seven doubles an older file does not
    // have, and everything after it would be read from the wrong place.
    if (version >= 7) {
        f.angle = r.f64();
        f.axisPoint = {r.f64(), r.f64(), r.f64()};
        f.axisDir   = {r.f64(), r.f64(), r.f64()};
        // A file written before this was an extrude, which kept its outline.
        f.mergeFlush = r.u32() != 0;
    } else {
        f.mergeFlush = false;
    }
    // A file older than this has no scaled faces in it, so the multiple that
    // changes nothing is the right one to leave standing.
    if (version >= 8) f.scale = r.f64();
    // Older files have no chamfers and no tapers: every bevel in them is a
    // round of one radius, which is what these defaults say.
    if (version >= 9) {
        f.chamfer = r.u32() != 0;
        f.endWidth = r.f64();
    }
    if (version >= 10) {
        const uint32_t mode = r.u32();
        if (mode > static_cast<uint32_t>(PatternMode::Mirror)) return false;
        f.patternMode = static_cast<PatternMode>(mode);
        f.patternCount = r.i32();
        if (f.patternCount < 1 || f.patternCount > 4096) return false;
    }
    // Older files have no reductions in them.
    if (version >= 11) {
        f.reduceTolerance = r.f64();
        f.reduceTarget = r.i32();
        f.reduceLoosen = r.u32() != 0;
        if (!(f.reduceTolerance > 0) || f.reduceTarget < 0) return false;
    }
    // Older files have no sketches in them.
    if (version >= 12) {
        if (!readSketch(r, f.sketch)) return false;
        f.sketchUid = r.u64();
        const SketchId first = r.u32();
        f.profileKeys.clear();
        if (first != kNoSketchId) f.profileKeys.push_back(first);
    }
    // A sketch in a version 12 file was drawn but never stood on its own, so
    // there was nothing to hide it from: shown is what it was.
    if (version >= 13) f.sketchShown = r.u8() != 0;
    // Older files have no moves, turns or scales in their history: those lived
    // on the object, and the loader turns them into steps.
    if (version >= 14) {
        f.moveBy = r.vec3();
        f.turnBy.x = r.f64(); f.turnBy.y = r.f64(); f.turnBy.z = r.f64(); f.turnBy.w = r.f64();
        f.turnAbout = r.vec3();
        f.scaleBy = r.vec3();
        f.scaleAbout = r.vec3();
        f.toolName = r.text();
    }
    if (version >= 15) {
        const uint32_t more = r.u32();
        if (more > 1000000u) return false;
        for (uint32_t i = 0; i < more && !r.bad; ++i) f.profileKeys.push_back(r.u32());
    }
    // Before this, an extrusion always went along the face's own normal.
    if (version >= 16) f.alongAxis = r.u8() != 0;
    // Before this there were no revolves to read one for.
    if (version >= 17) {
        f.revolveAxisAt.x  = r.f64(); f.revolveAxisAt.y  = r.f64();
        f.revolveAxisDir.x = r.f64(); f.revolveAxisDir.y = r.f64();
        f.revolveAngle = r.f64();
        if (!(f.revolveAngle > 0.0) || length(f.revolveAxisDir) < 1e-9) return false;
    }
    return !r.bad;
}

} // namespace

// ---------------------------------------------------------------------------
ProjectResult saveProject(const Scene& scene, const std::string& path) {
    ProjectResult res;

    Writer w;
    w.raw(kMagic, sizeof(kMagic));
    w.u32(kProjectVersion);
    w.u64(scene.nextFeatureUid());
    w.u32(static_cast<uint32_t>(scene.objectCount()));

    for (const auto& obj : scene.objects()) {
        w.u32(obj->id);
        w.text(obj->name);
        // Where it was made. Where it is now is that and the moves in its
        // history, which are written with the rest of the chain.
        w.vec3(obj->base.position);
        w.f64(obj->base.rotation.x); w.f64(obj->base.rotation.y);
        w.f64(obj->base.rotation.z); w.f64(obj->base.rotation.w);
        w.vec3(Vec3{1, 1, 1});
        w.u8(obj->visible ? 1 : 0);
        writeSpec(w, obj->spec);

        w.u32(static_cast<uint32_t>(obj->features.size()));
        for (const Feature& f : obj->features) writeFeature(w, f);
        ++res.objects;
    }

    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) { res.error = "cannot open " + path; return res; }
    out.write(reinterpret_cast<const char*>(w.buf.data()),
              static_cast<std::streamsize>(w.buf.size()));
    if (!out) { res.error = "write failed"; return res; }

    res.ok = true;
    return res;
}

ProjectResult loadProject(Scene& scene, const std::string& path) {
    ProjectResult res;

    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in) { res.error = "cannot open " + path; return res; }
    const std::streamsize size = in.tellg();
    if (size <= 0) { res.error = "empty file"; return res; }
    in.seekg(0);

    std::vector<unsigned char> data(static_cast<size_t>(size));
    if (!in.read(reinterpret_cast<char*>(data.data()), size)) {
        res.error = "read failed";
        return res;
    }

    Reader r{data.data(), data.data() + data.size(), false};
    if (!r.need(sizeof(kMagic)) || std::memcmp(r.p, kMagic, sizeof(kMagic)) != 0) {
        res.error = "not a tangent project";
        return res;
    }
    r.p += sizeof(kMagic);

    const uint32_t version = r.u32();
    // Older files are read; newer ones are not, because there is no way to know
    // what a field this build has never heard of means.
    if (version > kProjectVersion || version < kMinReadableVersion) {
        res.error = "project version " + std::to_string(version) +
                    " (this build reads " + std::to_string(kMinReadableVersion) +
                    " to " + std::to_string(kProjectVersion) + ")";
        return res;
    }

    const uint64_t nextUid = r.u64();

    // Built into a scratch scene first: a truncated or corrupt file must not
    // leave the user with half their model gone.
    Scene loaded;
    loaded.setNextFeatureUid(nextUid);
    const uint32_t count = r.u32();
    if (r.bad) { res.error = "truncated header"; return res; }

    for (uint32_t i = 0; i < count; ++i) {
        const uint32_t id = r.u32();
        (void)id;   // ids are reassigned; nothing outside a file references them
        const std::string name = r.text();

        Transform t;
        t.position = r.vec3();
        t.rotation.x = r.f64(); t.rotation.y = r.f64();
        t.rotation.z = r.f64(); t.rotation.w = r.f64();
        t.scale = r.vec3();
        const bool visible = r.u8() != 0;

        PrimitiveSpec spec;
        if (!readSpec(r, spec)) { res.error = "bad primitive parameters"; return res; }

        const uint32_t featureCount = r.u32();
        if (r.bad || featureCount > 100000) { res.error = "bad feature count"; return res; }

        std::vector<Feature> chain;
        chain.reserve(featureCount);
        for (uint32_t k = 0; k < featureCount; ++k) {
            Feature f;
            if (!readFeature(r, f, version)) { res.error = "bad feature"; return res; }
            chain.push_back(std::move(f));
        }
        if (r.bad) { res.error = "truncated file"; return res; }

        // Before version 13 a sketch could not be shown or hidden, because a
        // sketch could not stand on its own: every one of them was drawn in
        // order to sweep something. So one that was swept is scaffolding and
        // starts hidden, and one that nothing was built from starts shown --
        // which is what saving those same files from here would now record.
        if (version < 13) {
            for (Feature& f : chain) {
                if (f.kind != FeatureKind::Sketch) continue;
                f.sketchShown = std::none_of(chain.begin(), chain.end(), [&](const Feature& g) {
                    return g.kind == FeatureKind::ExtrudeProfile && g.sketchUid == f.uid;
                });
            }
        }

        // An object that came from a file -- a STEP import, a mesh, a piece of a
        // split -- has no primitive to start from: its chain begins with the
        // geometry itself. Starting every object from a primitive refused
        // these outright, so a project holding one saved and then would not
        // open.
        const bool fromGeometry = !chain.empty() && chain.front().kind == FeatureKind::BaseMesh;
        // A part that starts from a sketch has no primitive to fall back on:
        // the chain itself is the object, so it is built from the chain.
        const bool fromSketch = !chain.empty() && chain.front().kind == FeatureKind::Sketch;
        ObjectId newId = kNoObject;
        if (fromSketch)        newId = loaded.addChainAsIs(chain, name);
        else if (fromGeometry) newId = loaded.addImportedBody(chain.front().bakedBody, name);
        else                   newId = loaded.addPrimitive(spec.kind, spec, t.position);
        if (newId == kNoObject) { res.error = "object '" + name + "' failed to build"; return res; }

        // Before version 14 the transform was where the object was, scale and
        // all, and nothing in the history moved it. Where it was is where it
        // was made, then; and a scale -- which the exact kernel never saw, so
        // that anything baking this body into another got it unscaled --
        // becomes the step it always should have been, at the end of the
        // chain where it acted.
        const bool scaled = std::fabs(t.scale.x - 1.0) > 1e-9 || std::fabs(t.scale.y - 1.0) > 1e-9 ||
                            std::fabs(t.scale.z - 1.0) > 1e-9;
        if (scaled && t.scale.x > 0.0 && t.scale.y > 0.0 && t.scale.z > 0.0) {
            const bool hasBody = std::any_of(chain.begin(), chain.end(), [](const Feature& f) {
                return f.kind != FeatureKind::Sketch && !isPlacement(f.kind);
            });
            if (hasBody) {
                Feature sc;
                sc.kind = FeatureKind::Scale;
                sc.uid = loaded.takeFeatureUid();
                sc.scaleBy = t.scale;
                chain.push_back(std::move(sc));
            }
        }

        SceneObject* o = loaded.find(newId);
        o->name = name;
        o->visible = visible;
        o->features = std::move(chain);
        loaded.setBasePlacement(newId, t);
        // Re-runs the recipe. A chain that no longer evaluates leaves the
        // object as its base primitive rather than failing the whole load.
        loaded.reevaluate(newId);
        ++res.objects;
    }

    scene = std::move(loaded);
    res.ok = true;
    return res;
}

// ---------------------------------------------------------------------------
std::string encodeFeatures(const std::vector<Feature>& features) {
    Writer w;
    w.u32(kProjectVersion);
    w.u32(static_cast<uint32_t>(features.size()));
    for (const Feature& f : features) writeFeature(w, f);
    return std::string(reinterpret_cast<const char*>(w.buf.data()), w.buf.size());
}

bool decodeFeatures(const std::string& bytes, std::vector<Feature>& out) {
    out.clear();
    Reader r{reinterpret_cast<const unsigned char*>(bytes.data()),
             reinterpret_cast<const unsigned char*>(bytes.data()) + bytes.size(), false};
    const uint32_t version = r.u32();
    if (r.bad || version != kProjectVersion) return false;
    const uint32_t n = r.u32();
    // Every feature is far more than a byte, so a count past that is corrupt.
    if (r.bad || n > bytes.size()) return false;
    out.resize(n);
    for (Feature& f : out)
        if (!readFeature(r, f, version)) { out.clear(); return false; }
    if (r.bad || r.p != r.end) { out.clear(); return false; }
    return true;
}

} // namespace tg
