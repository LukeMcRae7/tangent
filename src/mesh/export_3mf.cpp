#include "mesh/export_3mf.h"

#include "mesh/export_stl.h"
#include "mesh/weld.h"
#include "mesh/zip_writer.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <unordered_map>

namespace tg {
namespace {

// How close two corners have to be to be one vertex. A millionth of a
// millimetre: a tessellation puts the corners two faces share at the same
// point to within rounding, and nothing a printer makes is within a thousand
// times this of anything else.
constexpr Real kWeldMm = 1e-6;

void appendNumber(std::string& out, Real v) {
    // Locale-proof and shortest-exact at the precision asked for. Ten
    // significant figures is below a nanometre on a metre-long part.
    if (std::fabs(v) < 1e-12) v = 0.0;
    char buf[32];
    const auto res = std::to_chars(buf, buf + sizeof buf, static_cast<double>(v),
                                   std::chars_format::general, 10);
    out.append(buf, res.ptr);
}

void appendInt(std::string& out, uint64_t v) {
    char buf[24];
    const auto res = std::to_chars(buf, buf + sizeof buf, v);
    out.append(buf, res.ptr);
}

void appendEscaped(std::string& out, const std::string& text) {
    for (char c : text) {
        switch (c) {
            case '&':  out += "&amp;";  break;
            case '<':  out += "&lt;";   break;
            case '>':  out += "&gt;";   break;
            case '"':  out += "&quot;"; break;
            case '\'': out += "&apos;"; break;
            default:
                // Control characters are not allowed in XML 1.0 at all.
                if (static_cast<unsigned char>(c) >= 0x20 || c == '\t') out.push_back(c);
                break;
        }
    }
}

// Every edge used exactly once in each direction: closed, and consistently
// wound, which is what a slicer means by a mesh it does not have to repair.
bool isClosed(const std::vector<uint32_t>& tris) {
    if (tris.empty()) return false;
    std::unordered_map<uint64_t, int> directed;
    directed.reserve(tris.size());
    for (size_t i = 0; i + 2 < tris.size(); i += 3) {
        for (int k = 0; k < 3; ++k) {
            const uint64_t a = tris[i + k], b = tris[i + (k + 1) % 3];
            if (++directed[(a << 32) | b] > 1) return false;
        }
    }
    for (const auto& [key, count] : directed) {
        const uint64_t reverse = ((key & 0xFFFFFFFFull) << 32) | (key >> 32);
        if (directed.find(reverse) == directed.end()) return false;
    }
    return true;
}

} // namespace

void exportMeshOf(const SceneObject& obj, Real deviationMm, ExportMesh& out) {
    out = ExportMesh{};
    const Mat4 model = obj.modelMatrix();
    const bool flip = mirrors(model);

    auto emit = [&](uint32_t a, uint32_t b, uint32_t c) {
        if (a == b || b == c || a == c) return;          // collapsed by the weld
        out.triangles.insert(out.triangles.end(), {a, flip ? c : b, flip ? b : c});
    };

    if (obj.body.isMesh()) {
        // A mesh already shares its vertices; they go out as they are, so what
        // was closed on screen is closed in the file without any welding.
        const Mesh& m = obj.body.mesh();
        out.vertices.reserve(m.verts.size());
        for (const MeshVertex& v : m.verts) out.vertices.push_back(transformPoint(model, v.position));
        std::vector<Index> corners, local;
        for (Index f = 0; f < m.faceCount(); ++f) {
            m.faceVertices(f, corners);
            local.clear();                               // it appends
            m.triangulateFacePublic(f, local);
            for (size_t i = 0; i + 2 < local.size(); i += 3)
                emit(static_cast<uint32_t>(corners[local[i]]),
                     static_cast<uint32_t>(corners[local[i + 1]]),
                     static_cast<uint32_t>(corners[local[i + 2]]));
        }
    } else {
        // An exact body is tessellated a face at a time, each face with its own
        // copy of the points it shares with its neighbours. Welded here, in
        // world space, so the placement cannot pull two copies apart.
        RenderMesh scratch;
        const RenderMesh& rm = exportTriangles(obj, deviationMm, scratch);
        Welder weld(kWeldMm);
        weld.reserve(rm.positions.size());
        std::vector<uint32_t> remap(rm.positions.size());
        for (size_t i = 0; i < rm.positions.size(); ++i)
            remap[i] = weld.add(transformPoint(model, rm.positions[i]));
        for (size_t i = 0; i + 2 < rm.triangles.size(); i += 3)
            emit(remap[rm.triangles[i]], remap[rm.triangles[i + 1]], remap[rm.triangles[i + 2]]);
        out.vertices = std::move(weld.positions);
    }

    // Only the vertices a triangle uses: a weld or a dropped sliver can leave
    // one behind, and a reader is entitled to object to it.
    std::vector<uint32_t> used(out.vertices.size(), UINT32_MAX);
    std::vector<Vec3> kept;
    kept.reserve(out.vertices.size());
    for (uint32_t& v : out.triangles) {
        if (used[v] == UINT32_MAX) {
            used[v] = static_cast<uint32_t>(kept.size());
            kept.push_back(out.vertices[v]);
        }
        v = used[v];
    }
    out.vertices = std::move(kept);
    out.closed = isClosed(out.triangles);
}

ThreeMfResult export3mf(const Scene& scene, const std::string& path,
                        const ThreeMfOptions& options) {
    ThreeMfResult r;
    const std::vector<const SceneObject*> objects = exportedObjects(scene, options.selectionOnly);

    std::string model;
    model.reserve(1 << 20);
    model += "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
             "<model unit=\"millimeter\" xml:lang=\"en-US\" "
             "xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">\n"
             " <metadata name=\"Application\">Tangent</metadata>\n"
             " <resources>\n";

    std::vector<uint32_t> ids;
    ExportMesh mesh;
    for (const SceneObject* obj : objects) {
        exportMeshOf(*obj, options.deviationMm, mesh);
        if (mesh.triangles.empty()) continue;

        const uint32_t id = static_cast<uint32_t>(ids.size() + 1);
        ids.push_back(id);
        ++r.objects;
        if (obj->body.isMesh()) ++r.meshBodies;
        if (!mesh.closed) ++r.openObjects;
        r.vertices += mesh.vertices.size();
        r.triangles += mesh.triangles.size() / 3;

        model += "  <object id=\"";
        appendInt(model, id);
        model += "\" type=\"model\" name=\"";
        appendEscaped(model, obj->name);
        model += "\">\n   <mesh>\n    <vertices>\n";
        for (const Vec3& v : mesh.vertices) {
            model += "     <vertex x=\"";
            appendNumber(model, v.x);
            model += "\" y=\"";
            appendNumber(model, v.y);
            model += "\" z=\"";
            appendNumber(model, v.z);
            model += "\"/>\n";
        }
        model += "    </vertices>\n    <triangles>\n";
        for (size_t i = 0; i + 2 < mesh.triangles.size(); i += 3) {
            model += "     <triangle v1=\"";
            appendInt(model, mesh.triangles[i]);
            model += "\" v2=\"";
            appendInt(model, mesh.triangles[i + 1]);
            model += "\" v3=\"";
            appendInt(model, mesh.triangles[i + 2]);
            model += "\"/>\n";
        }
        model += "    </triangles>\n   </mesh>\n  </object>\n";
    }

    if (ids.empty()) {
        r.error = options.selectionOnly ? "nothing selected to export"
                                        : "nothing visible to export";
        return r;
    }

    model += " </resources>\n <build>\n";
    for (uint32_t id : ids) {
        model += "  <item objectid=\"";
        appendInt(model, id);
        model += "\"/>\n";
    }
    model += " </build>\n</model>\n";

    static constexpr const char* kContentTypes =
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        "<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">\n"
        " <Default Extension=\"rels\" "
        "ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/>\n"
        " <Default Extension=\"model\" "
        "ContentType=\"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\"/>\n"
        "</Types>\n";
    static constexpr const char* kRelationships =
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">\n"
        " <Relationship Target=\"/3D/3dmodel.model\" Id=\"rel0\" "
        "Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\"/>\n"
        "</Relationships>\n";

    ZipWriter zip;
    if (!zip.open(path, &r.error)) return r;
    if (!zip.add("[Content_Types].xml", kContentTypes, &r.error) ||
        !zip.add("_rels/.rels", kRelationships, &r.error) ||
        !zip.add("3D/3dmodel.model", model, &r.error) ||
        !zip.finish(&r.error)) {
        // Half an archive is not a file anyone can open, and leaving it where
        // the user asked for the export would look like one that worked.
        zip.finish(nullptr);
        std::remove(path.c_str());
        return r;
    }

    r.ok = true;
    return r;
}

} // namespace tg
