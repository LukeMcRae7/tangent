#include "mesh/export_stl.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>

namespace tg {
namespace {

void writeLE32(std::vector<unsigned char>& out, uint32_t v) {
    out.push_back(static_cast<unsigned char>(v & 0xFF));
    out.push_back(static_cast<unsigned char>((v >> 8) & 0xFF));
    out.push_back(static_cast<unsigned char>((v >> 16) & 0xFF));
    out.push_back(static_cast<unsigned char>((v >> 24) & 0xFF));
}

void writeFloat(std::vector<unsigned char>& out, Real v) {
    const float f = static_cast<float>(v);
    uint32_t bits;
    std::memcpy(&bits, &f, sizeof(bits));
    writeLE32(out, bits);
}

struct Tri {
    Vec3 n, a, b, c;
};

} // namespace

const RenderMesh& exportTriangles(const SceneObject& obj, Real deviationMm, RenderMesh& scratch) {
    // An exact body is tessellated again, to the export's tolerance rather than
    // the screen's. A mesh body has no choice in the matter and goes out as it
    // stands.
    scratch.clear();
    if (!obj.body.isMesh() && deviationMm > 0.0) {
        TessellationQuality q;
        q.deviationMm = deviationMm;
        // Loose enough that the tolerance the user asked for is what decides
        // the result on any feature of a normal size, and tight enough that a
        // 1mm hole does not come out a hexagon just because a coarse tolerance
        // allows it.
        q.angleRad = 0.5;
        q.independent = true;
        obj.body.tessellate(scratch, q);
    }
    return scratch.triangles.empty() ? obj.render : scratch;
}

std::vector<const SceneObject*> exportedObjects(const Scene& scene, bool selectionOnly) {
    std::vector<const SceneObject*> out;
    for (const auto& obj : scene.objects()) {
        if (selectionOnly ? !scene.isSelected(obj->id) : !obj->visible) continue;
        if (obj->body.empty()) continue;
        out.push_back(obj.get());
    }
    return out;
}

bool mirrors(const Mat4& m) {
    // The sign of the linear part's determinant. A negative one turns the
    // surface inside out unless the triangles are wound the other way to match.
    const Vec3 x{m[0].x, m[0].y, m[0].z};
    const Vec3 y{m[1].x, m[1].y, m[1].z};
    const Vec3 z{m[2].x, m[2].y, m[2].z};
    return dot(x, cross(y, z)) < 0.0;
}

std::string exportFileFor(const std::string& path, const std::string& objectName,
                          std::vector<std::string>& taken) {
    // "parts/plate.stl" and "Bracket 2" -> "parts/plate - Bracket 2.stl". The
    // name is kept readable; only what a file system would refuse is replaced.
    const size_t slash = path.find_last_of("/\\");
    const size_t dot = path.find_last_of('.');
    const bool hasExt = dot != std::string::npos && (slash == std::string::npos || dot > slash);
    const std::string stem = hasExt ? path.substr(0, dot) : path;
    const std::string ext = hasExt ? path.substr(dot) : std::string(".stl");

    std::string clean;
    for (char c : objectName) {
        const bool bad = c == '/' || c == '\\' || c == ':' || c == '*' || c == '?' ||
                         c == '"' || c == '<' || c == '>' || c == '|' ||
                         static_cast<unsigned char>(c) < 0x20;
        clean.push_back(bad ? '_' : c);
    }
    while (!clean.empty() && (clean.back() == ' ' || clean.back() == '.')) clean.pop_back();
    if (clean.empty()) clean = "Object";

    std::string candidate = stem + " - " + clean + ext;
    for (int n = 2; std::find(taken.begin(), taken.end(), candidate) != taken.end(); ++n)
        candidate = stem + " - " + clean + " " + std::to_string(n) + ext;
    taken.push_back(candidate);
    return candidate;
}

namespace {

// World-space triangles for one object.
void gather(const SceneObject& obj, const StlOptions& opt, std::vector<Tri>& tris) {
    RenderMesh scratch;
    const RenderMesh& rm = exportTriangles(obj, opt.deviationMm, scratch);
    const Mat4 model = obj.modelMatrix();
    const Mat4 nrm = normalMatrix(model);
    const bool flip = mirrors(model);

    for (size_t i = 0; i + 2 < rm.triangles.size(); i += 3) {
        Tri t;
        t.a = transformPoint(model, rm.positions[rm.triangles[i + 0]]);
        t.b = transformPoint(model, rm.positions[rm.triangles[i + (flip ? 2 : 1)]]);
        t.c = transformPoint(model, rm.positions[rm.triangles[i + (flip ? 1 : 2)]]);

        // Recomputed from the transformed triangle rather than carried over: a
        // mirrored or non-uniformly scaled object would otherwise export
        // normals that disagree with its winding.
        const Vec3 geo = cross(t.b - t.a, t.c - t.a);
        t.n = lengthSq(geo) > 1e-24 ? normalize(geo)
                                    : normalize(transformVector(nrm, rm.normals[rm.triangles[i]]));
        tris.push_back(t);
    }
}

bool writeStl(const std::string& path, const std::vector<Tri>& tris, const StlOptions& options,
              std::string& error) {
    FILE* f = std::fopen(path.c_str(), options.binary ? "wb" : "w");
    if (!f) { error = "cannot open " + path; return false; }

    if (options.binary) {
        std::vector<unsigned char> buf;
        buf.reserve(84 + tris.size() * 50);

        // 80-byte header. Deliberately not starting with "solid": some readers
        // sniff that word and try to parse a binary file as ASCII.
        char header[80] = {};
        std::snprintf(header, sizeof(header), "Tangent binary STL - %s",
                      options.solidName.c_str());
        buf.insert(buf.end(), header, header + 80);
        writeLE32(buf, static_cast<uint32_t>(tris.size()));

        for (const Tri& t : tris) {
            for (const Vec3& v : {t.n, t.a, t.b, t.c}) {
                writeFloat(buf, v.x);
                writeFloat(buf, v.y);
                writeFloat(buf, v.z);
            }
            buf.push_back(0);   // attribute byte count
            buf.push_back(0);
        }

        const size_t wrote = std::fwrite(buf.data(), 1, buf.size(), f);
        const bool closed = std::fclose(f) == 0;
        if (wrote != buf.size() || !closed) { error = "short write to " + path; return false; }
    } else {
        std::fprintf(f, "solid %s\n", options.solidName.c_str());
        for (const Tri& t : tris) {
            std::fprintf(f, "  facet normal %.6e %.6e %.6e\n    outer loop\n",
                         t.n.x, t.n.y, t.n.z);
            for (const Vec3& v : {t.a, t.b, t.c})
                std::fprintf(f, "      vertex %.6e %.6e %.6e\n", v.x, v.y, v.z);
            std::fprintf(f, "    endloop\n  endfacet\n");
        }
        std::fprintf(f, "endsolid %s\n", options.solidName.c_str());
        if (std::ferror(f)) { std::fclose(f); error = "short write to " + path; return false; }
        if (std::fclose(f) != 0) { error = "short write to " + path; return false; }
    }
    return true;
}

} // namespace

StlResult exportStl(const Scene& scene, const std::string& path,
                    const StlOptions& options) {
    StlResult r;
    const std::vector<const SceneObject*> objects = exportedObjects(scene, options.selectionOnly);

    auto nothing = [&] {
        r.error = options.selectionOnly ? "nothing selected to export"
                                        : "nothing visible to export";
        return r;
    };

    if (!options.separateFiles) {
        std::vector<Tri> tris;
        for (const SceneObject* obj : objects) {
            const size_t before = tris.size();
            gather(*obj, options, tris);
            if (tris.size() == before) continue;
            ++r.objects;
            if (obj->body.isMesh()) ++r.meshBodies;
        }
        if (tris.empty()) return nothing();
        if (!writeStl(path, tris, options, r.error)) return r;
        r.ok = true;
        r.triangles = tris.size();
        r.files.push_back(path);
        return r;
    }

    // One file per object, each named for the object it holds. A multi-part
    // print is loaded part by part, and the names are what tell them apart.
    std::vector<std::string> taken;
    for (const SceneObject* obj : objects) {
        std::vector<Tri> tris;
        gather(*obj, options, tris);
        if (tris.empty()) continue;
        const std::string file = exportFileFor(path, obj->name, taken);
        if (!writeStl(file, tris, options, r.error)) return r;
        ++r.objects;
        if (obj->body.isMesh()) ++r.meshBodies;
        r.triangles += tris.size();
        r.files.push_back(file);
    }
    if (r.files.empty()) return nothing();
    r.ok = true;
    return r;
}

} // namespace tg
