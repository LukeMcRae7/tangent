#include "scene/serialize.h"

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
    writeMesh(w, f.bakedMesh);
}

bool readFeature(Reader& r, Feature& f) {
    const uint32_t kind = r.u32();
    if (kind > static_cast<uint32_t>(FeatureKind::Boolean)) return false;
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
    if (!readMesh(r, f.bakedMesh)) return false;
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
        w.vec3(obj->transform.position);
        w.f64(obj->transform.rotation.x); w.f64(obj->transform.rotation.y);
        w.f64(obj->transform.rotation.z); w.f64(obj->transform.rotation.w);
        w.vec3(obj->transform.scale);
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
    if (version != kProjectVersion) {
        res.error = "project version " + std::to_string(version) +
                    " (this build reads " + std::to_string(kProjectVersion) + ")";
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
            if (!readFeature(r, f)) { res.error = "bad feature"; return res; }
            chain.push_back(std::move(f));
        }
        if (r.bad) { res.error = "truncated file"; return res; }

        const ObjectId newId = loaded.addPrimitive(spec.kind, spec, t.position);
        if (newId == kNoObject) { res.error = "object '" + name + "' failed to build"; return res; }

        SceneObject* o = loaded.find(newId);
        o->name = name;
        o->transform = t;
        o->visible = visible;
        o->features = std::move(chain);
        // Re-runs the recipe. A chain that no longer evaluates leaves the
        // object as its base primitive rather than failing the whole load.
        loaded.reevaluate(newId);
        ++res.objects;
    }

    scene = std::move(loaded);
    res.ok = true;
    return res;
}

} // namespace tg
