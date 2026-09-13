#include "mesh/import_mesh.h"

#include "mesh/halfedge.h"
#include "mesh/health.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace tg {
namespace {

// STL stores every triangle with its own three vertices and no index at all, so
// a cube arrives as thirty-six unrelated points. Welding them is not a tidy-up:
// without it there are no shared edges, so nothing is adjacent to anything, the
// surface is not closed, and none of the operations that make a mesh useful --
// measuring it, telling whether it is solid, turning it into a body -- can say
// anything true about it.
//
// A hundredth of a micron. Far below any tolerance a printer or a CAD package
// works to, and far above the noise in a float32 coordinate.
constexpr Real kWeld = 1e-5;

struct Welder {
    std::vector<Vec3> positions;
    std::unordered_map<uint64_t, std::vector<uint32_t>> grid;

    static uint64_t cell(Vec3 p) {
        auto q = [](Real v) { return static_cast<int64_t>(std::llround(v / kWeld)); };
        const uint64_t a = static_cast<uint64_t>(q(p.x));
        const uint64_t b = static_cast<uint64_t>(q(p.y));
        const uint64_t c = static_cast<uint64_t>(q(p.z));
        return a * 0x9E3779B97F4A7C15ull ^ (b * 0xC2B2AE3D27D4EB4Full) ^
               (c * 0x165667B19E3779F9ull);
    }

    uint32_t add(Vec3 p) {
        const uint64_t key = cell(p);
        auto& bucket = grid[key];
        for (uint32_t i : bucket)
            if (lengthSq(positions[i] - p) <= kWeld * kWeld) return i;
        positions.push_back(p);
        const auto idx = static_cast<uint32_t>(positions.size() - 1);
        bucket.push_back(idx);
        return idx;
    }
};

bool endsWith(const std::string& s, const char* suffix) {
    const size_t n = std::strlen(suffix);
    if (s.size() < n) return false;
    for (size_t i = 0; i < n; ++i)
        if (std::tolower(static_cast<unsigned char>(s[s.size() - n + i])) != suffix[i])
            return false;
    return true;
}

// A binary STL is 84 bytes of header and then exactly 50 per triangle. That
// arithmetic is the only reliable way to tell the two apart: an ASCII file may
// begin with the word "solid", and so may a binary one whose header happens to.
bool looksBinary(std::ifstream& f, uint32_t& countOut) {
    f.seekg(0, std::ios::end);
    const std::streamoff size = f.tellg();
    if (size < 84) return false;
    f.seekg(80, std::ios::beg);
    uint32_t n = 0;
    f.read(reinterpret_cast<char*>(&n), 4);
    countOut = n;
    return static_cast<std::streamoff>(84) + static_cast<std::streamoff>(n) * 50 == size;
}

bool readBinaryStl(std::ifstream& f, uint32_t count, Welder& w,
                   std::vector<uint32_t>& sizes, std::vector<uint32_t>& idx) {
    f.seekg(84, std::ios::beg);
    std::vector<char> rec(50);
    for (uint32_t t = 0; t < count; ++t) {
        f.read(rec.data(), 50);
        if (!f) return false;
        // Bytes 0..11 are the stored normal, which is ignored: it is redundant
        // with the winding and disagrees with it often enough in real files
        // that trusting it would import parts inside out.
        uint32_t v[3];
        for (int i = 0; i < 3; ++i) {
            float xyz[3];
            std::memcpy(xyz, rec.data() + 12 + i * 12, 12);
            v[i] = w.add({xyz[0], xyz[1], xyz[2]});
        }
        if (v[0] == v[1] || v[1] == v[2] || v[0] == v[2]) continue;  // degenerate
        sizes.push_back(3);
        idx.insert(idx.end(), {v[0], v[1], v[2]});
    }
    return true;
}

bool readAsciiStl(std::ifstream& f, Welder& w,
                  std::vector<uint32_t>& sizes, std::vector<uint32_t>& idx) {
    f.clear();
    f.seekg(0, std::ios::beg);
    std::string word;
    std::vector<uint32_t> loop;
    while (f >> word) {
        if (word == "vertex") {
            double x = 0, y = 0, z = 0;
            if (!(f >> x >> y >> z)) return false;
            loop.push_back(w.add({static_cast<Real>(x), static_cast<Real>(y),
                                  static_cast<Real>(z)}));
        } else if (word == "endloop") {
            if (loop.size() == 3 && loop[0] != loop[1] && loop[1] != loop[2] &&
                loop[0] != loop[2]) {
                sizes.push_back(3);
                idx.insert(idx.end(), loop.begin(), loop.end());
            }
            loop.clear();
        }
    }
    return true;
}

bool readObj(std::ifstream& f, Welder& w,
             std::vector<uint32_t>& sizes, std::vector<uint32_t>& idx) {
    // OBJ has its own vertex list and indexes into it, so the file's numbering
    // is kept and mapped through the welder rather than replaced by it -- two
    // vertices the file deliberately kept apart at the same position stay one
    // vertex here, which is what makes the surface closed.
    std::vector<uint32_t> fileToWelded;
    std::string line;
    std::vector<uint32_t> face;

    while (std::getline(f, line)) {
        if (line.empty()) continue;
        if (line[0] == 'v' && line.size() > 1 && (line[1] == ' ' || line[1] == '\t')) {
            double x = 0, y = 0, z = 0;
            if (std::sscanf(line.c_str() + 1, "%lf %lf %lf", &x, &y, &z) == 3)
                fileToWelded.push_back(w.add({static_cast<Real>(x), static_cast<Real>(y),
                                              static_cast<Real>(z)}));
            continue;
        }
        if (line[0] != 'f' || line.size() < 2 || (line[1] != ' ' && line[1] != '\t'))
            continue;

        face.clear();
        const char* p = line.c_str() + 1;
        while (*p) {
            while (*p == ' ' || *p == '\t') ++p;
            if (!*p) break;
            // "12", "12/3", "12//4", "12/3/4" -- only the position index counts.
            long v = std::strtol(p, const_cast<char**>(&p), 10);
            while (*p && *p != ' ' && *p != '\t') ++p;      // skip the rest
            if (v == 0) continue;
            // Negative indices count back from the end, which is legal OBJ and
            // is what several exporters emit.
            const size_t at = v > 0 ? static_cast<size_t>(v - 1)
                                    : fileToWelded.size() - static_cast<size_t>(-v);
            if (at < fileToWelded.size()) face.push_back(fileToWelded[at]);
        }
        if (face.size() < 3) continue;

        // Fanned, because the rest of the program wants triangles and an OBJ
        // n-gon is planar and convex often enough for a fan to be right. A
        // concave one comes out wrong; it is also vanishingly rare in an
        // exported mesh, and the alternative is a full ear-clip for a case
        // this importer does not otherwise care about.
        for (size_t i = 1; i + 1 < face.size(); ++i) {
            if (face[0] == face[i] || face[i] == face[i + 1] || face[0] == face[i + 1])
                continue;
            sizes.push_back(3);
            idx.insert(idx.end(), {face[0], face[i], face[i + 1]});
        }
    }
    return true;
}

} // namespace

MeshFormat meshFormatOf(const std::string& path) {
    if (endsWith(path, ".stl")) return MeshFormat::Stl;
    if (endsWith(path, ".obj")) return MeshFormat::Obj;
    return MeshFormat::Unknown;
}

MeshImport readMesh(const std::string& path, Body& out) {
    MeshImport r;
    const MeshFormat fmt = meshFormatOf(path);
    if (fmt == MeshFormat::Unknown) {
        r.error = "that is not a file extension this reads: .stl or .obj";
        return r;
    }

    std::ifstream f(path, std::ios::binary);
    if (!f) { r.error = "the file could not be opened"; return r; }

    Welder w;
    std::vector<uint32_t> sizes, idx;
    bool read = false;
    if (fmt == MeshFormat::Stl) {
        uint32_t count = 0;
        read = looksBinary(f, count) ? readBinaryStl(f, count, w, sizes, idx)
                                     : readAsciiStl(f, w, sizes, idx);
    } else {
        read = readObj(f, w, sizes, idx);
    }
    if (!read)      { r.error = "the file is truncated or malformed"; return r; }
    if (sizes.empty()) { r.error = "there are no triangles in it"; return r; }

    Mesh m;
    if (!m.build(w.positions, sizes, idx)) {
        // build() refuses a soup it cannot make a half-edge structure from --
        // typically three faces meeting along one edge, which no surface does.
        r.error = "the triangles do not make a surface (is it self-intersecting?)";
        return r;
    }

    r.triangles = sizes.size();
    r.closed = checkHealth(m, false).watertight;
    out = Body(std::move(m));
    r.ok = true;
    return r;
}

SolidifyResult toSolid(Body& body, ElementId salt, int maxFaces) {
    SolidifyResult r;
    if (!body.isMesh()) { r.error = "that body is already exact"; return r; }
    if (body.empty())   { r.error = "there is nothing to convert"; return r; }

    const Mesh& m = body.mesh();
    r.facesBefore = static_cast<int>(m.faces.size());
    if (r.facesBefore > maxFaces) {
        char buf[160];
        std::snprintf(buf, sizeof buf,
                      "%d triangles is too many to convert; the limit is %d and the "
                      "result would not be workable anyway",
                      r.facesBefore, maxFaces);
        r.error = buf;
        return r;
    }
    if (!checkHealth(m, false).watertight) {
        r.error = "the surface has holes in it, and only a closed one can be a solid";
        return r;
    }

    std::vector<Vec3> pts;
    pts.reserve(m.verts.size());
    for (const MeshVertex& v : m.verts) pts.push_back(v.position);

    std::vector<uint32_t> tris;
    tris.reserve(m.faces.size() * 3);
    std::vector<Index> loop;
    for (size_t f = 0; f < m.faces.size(); ++f) {
        m.faceVertices(static_cast<Index>(f), loop);
        for (size_t i = 1; i + 1 < loop.size(); ++i) {
            tris.push_back(static_cast<uint32_t>(loop[0]));
            tris.push_back(static_cast<uint32_t>(loop[i]));
            tris.push_back(static_cast<uint32_t>(loop[i + 1]));
        }
    }

    std::string why;
    BrepRef solid = brep::solidFromTriangles(pts, tris, salt, &why);
    if (!solid) {
        r.error = why.empty() ? "the triangles would not sew into a solid" : why;
        return r;
    }

    Body converted(std::move(solid));
    r.facesAfter = converted.faceCount();
    body = std::move(converted);
    r.ok = true;
    return r;
}

} // namespace tg
