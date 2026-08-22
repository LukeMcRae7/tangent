#include "mesh/export_stl.h"

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

// Collects world-space triangles from everything that should be written.
bool gather(const Scene& scene, const StlOptions& opt,
            std::vector<Tri>& tris, size_t& objects) {
    for (const auto& obj : scene.objects()) {
        if (opt.selectionOnly) {
            if (!scene.isSelected(obj->id)) continue;
        } else if (!obj->visible) {
            continue;
        }

        const RenderMesh& rm = obj->render;
        if (rm.triangles.empty()) continue;

        const Mat4 model = obj->modelMatrix();
        const Mat4 nrm = normalMatrix(model);

        for (size_t i = 0; i + 2 < rm.triangles.size(); i += 3) {
            Tri t;
            t.a = transformPoint(model, rm.positions[rm.triangles[i + 0]]);
            t.b = transformPoint(model, rm.positions[rm.triangles[i + 1]]);
            t.c = transformPoint(model, rm.positions[rm.triangles[i + 2]]);

            // Recomputed from the transformed triangle rather than carried
            // over: a mirrored or non-uniformly scaled object would otherwise
            // export normals that disagree with its winding.
            const Vec3 geo = cross(t.b - t.a, t.c - t.a);
            t.n = lengthSq(geo) > 1e-24 ? normalize(geo)
                                        : normalize(transformVector(nrm, rm.normals[rm.triangles[i]]));
            tris.push_back(t);
        }
        ++objects;
    }
    return !tris.empty();
}

} // namespace

StlResult exportStl(const Scene& scene, const std::string& path,
                    const StlOptions& options) {
    StlResult r;

    std::vector<Tri> tris;
    if (!gather(scene, options, tris, r.objects)) {
        r.error = options.selectionOnly ? "nothing selected to export"
                                        : "nothing visible to export";
        return r;
    }

    FILE* f = std::fopen(path.c_str(), options.binary ? "wb" : "w");
    if (!f) { r.error = "cannot open " + path; return r; }

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
        std::fclose(f);
        if (wrote != buf.size()) { r.error = "short write"; return r; }
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
        std::fclose(f);
    }

    r.ok = true;
    r.triangles = tris.size();
    return r;
}

} // namespace tg
