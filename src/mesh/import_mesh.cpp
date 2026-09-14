#include "mesh/import_mesh.h"

#include "mesh/halfedge.h"
#include "mesh/health.h"
#include "geom/brep.h"

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

namespace {

// Connected patches of coplanar triangles: the faces a conversion will produce.
//
// Flood filled from a seed triangle, testing every candidate against the
// *seed's* plane rather than its neighbour's. Against the neighbour, a finely
// faceted curve would creep from strip to strip a hundredth of a degree at a
// time and come back as one "flat" face; against the seed, the error cannot
// accumulate past the tolerance.
struct Regions {
    std::vector<int32_t> of;        // per mesh face
    std::vector<Vec3> normal, point;
    int count = 0;
};

Regions findRegions(const Mesh& m) {
    Regions r;
    const size_t nf = m.faces.size();
    r.of.assign(nf, -1);
    if (nf == 0) return r;

    Vec3 lo{1e30, 1e30, 1e30}, hi{-1e30, -1e30, -1e30};
    for (const MeshVertex& v : m.verts) {
        lo = {std::min(lo.x, v.position.x), std::min(lo.y, v.position.y), std::min(lo.z, v.position.z)};
        hi = {std::max(hi.x, v.position.x), std::max(hi.y, v.position.y), std::max(hi.z, v.position.z)};
    }
    // A float32 STL coordinate is good to about seven significant figures, so
    // a millionth of the part's size is the finest distance that means anything.
    const Real tol = std::max<Real>(length(hi - lo) * 1e-6, 1e-7);
    constexpr Real kCos = 1.0 - 1e-6;       // about a twelfth of a degree

    std::vector<Vec3> fn(nf);
    for (size_t f = 0; f < nf; ++f) {
        const Vec3 n = m.faceNormal(static_cast<Index>(f));
        fn[f] = lengthSq(n) > 1e-24 ? normalize(n) : Vec3{};
    }

    auto onPlane = [&](Index g, Vec3 n, Real d) {
        const Index h0 = m.faces[g].halfedge;
        Index h = h0;
        do {
            if (std::fabs(dot(n, m.verts[m.halfedges[h].vertex].position) - d) > tol) return false;
            h = m.halfedges[h].next;
        } while (h != h0);
        return true;
    };

    std::vector<Index> stack;
    // Two passes: real triangles seed regions; slivers with no normal only ever
    // join one, since a plane through a sliver is whatever rounding says it is.
    for (int pass = 0; pass < 2; ++pass) {
        for (size_t seed = 0; seed < nf; ++seed) {
            if (r.of[seed] != -1) continue;
            const bool degenerate = lengthSq(fn[seed]) == 0.0;
            if (pass == 0 && degenerate) continue;

            const int id = r.count++;
            const Vec3 n = degenerate ? Vec3{0, 0, 1} : fn[seed];
            const Vec3 p = m.faceCentroid(static_cast<Index>(seed));
            const Real d = dot(n, p);
            r.normal.push_back(n);
            r.point.push_back(p);
            r.of[seed] = id;
            stack.assign(1, static_cast<Index>(seed));

            while (!stack.empty()) {
                const Index f = stack.back();
                stack.pop_back();
                const Index h0 = m.faces[f].halfedge;
                Index h = h0;
                do {
                    const Index t = m.halfedges[h].twin;
                    const Index g = t >= 0 ? m.halfedges[t].face : kInvalid;
                    if (g >= 0 && r.of[g] == -1 &&
                        (lengthSq(fn[g]) == 0.0 || dot(fn[g], n) > kCos) && onPlane(g, n, d)) {
                        r.of[g] = id;
                        stack.push_back(g);
                    }
                    h = m.halfedges[h].next;
                } while (h != h0);
            }
        }
    }
    return r;
}

// Every region's boundary as loops of shared edges.
bool traceRegions(const Mesh& m, const Regions& reg, brep::PlanarRegions& out) {
    out = brep::PlanarRegions{};
    out.points.reserve(m.verts.size());
    for (const MeshVertex& v : m.verts) out.points.push_back(v.position);
    out.regions.resize(static_cast<size_t>(reg.count));
    {
        // Divergence theorem over the mesh's own faces, fanned. The solid built
        // from the regions has to enclose exactly this.
        Real v = 0.0;
        std::vector<Index> loop;
        for (size_t f = 0; f < m.faces.size(); ++f) {
            m.faceVertices(static_cast<Index>(f), loop);
            for (size_t i = 1; i + 1 < loop.size(); ++i)
                v += dot(m.verts[loop[0]].position,
                         cross(m.verts[loop[i]].position, m.verts[loop[i + 1]].position));
        }
        out.volume = v / 6.0;
    }
    for (int i = 0; i < reg.count; ++i) {
        out.regions[static_cast<size_t>(i)].normal = reg.normal[static_cast<size_t>(i)];
        out.regions[static_cast<size_t>(i)].point = reg.point[static_cast<size_t>(i)];
    }

    auto regionOf = [&](Index he) -> int32_t {
        const Index f = m.halfedges[he].face;
        return f >= 0 ? reg.of[f] : -1;
    };
    auto isBoundary = [&](Index he) {
        const Index t = m.halfedges[he].twin;
        return t < 0 || regionOf(t) != regionOf(he);
    };
    auto source = [&](Index he) { return m.halfedges[m.halfedges[he].prev].vertex; };

    std::unordered_map<uint64_t, uint32_t> edgeIndex;
    edgeIndex.reserve(m.halfedges.size() / 4);
    auto edgeFor = [&](uint32_t a, uint32_t b) {
        const uint64_t lo = std::min(a, b), hi = std::max(a, b);
        const uint64_t key = (lo << 32) | hi;
        auto it = edgeIndex.find(key);
        if (it != edgeIndex.end()) return it->second;
        const auto id = static_cast<uint32_t>(out.edgeEnds.size() / 2);
        out.edgeEnds.push_back(a);
        out.edgeEnds.push_back(b);
        edgeIndex.emplace(key, id);
        return id;
    };

    std::vector<char> used(m.halfedges.size(), 0);
    for (size_t start = 0; start < m.halfedges.size(); ++start) {
        const Index s0 = static_cast<Index>(start);
        if (used[start] || m.halfedges[s0].face < 0 || !isBoundary(s0)) continue;
        const int32_t R = regionOf(s0);

        brep::PlanarRegions::Loop loop;
        Index h = s0;
        // Bounded, so a boundary that does not close -- a pinched or broken
        // region -- fails rather than spins.
        for (size_t guard = 0; guard <= m.halfedges.size(); ++guard) {
            used[static_cast<size_t>(h)] = 1;
            const auto a = static_cast<uint32_t>(source(h));
            const auto b = static_cast<uint32_t>(m.halfedges[h].vertex);
            loop.from.push_back(a);
            loop.edges.push_back(edgeFor(a, b));

            // The next boundary half-edge leaving where this one arrived: turn
            // about that vertex, inside the region, until the region ends.
            Index n = m.halfedges[h].next;
            size_t turns = 0;
            while (!isBoundary(n)) {
                n = m.halfedges[m.halfedges[n].twin].next;
                if (++turns > m.halfedges.size()) return false;
            }
            h = n;
            if (h == s0) break;
            if (used[static_cast<size_t>(h)] || regionOf(h) != R) return false;
        }
        if (h != s0 || loop.from.size() < 3) return false;
        out.regions[static_cast<size_t>(R)].loops.push_back(std::move(loop));
    }

    for (const brep::PlanarRegions::Region& region : out.regions)
        if (region.loops.empty()) return false;
    return true;
}

} // namespace

int predictSolidFaces(const Body& body) {
    if (!body.isMesh() || body.empty()) return 0;
    // The same partition the conversion uses, so the number said up front is
    // the number that comes out rather than an estimate of it.
    return findRegions(body.mesh()).count;
}

SolidifyResult toSolid(Body& body, ElementId salt, int maxFaces) {
    SolidifyResult r;
    if (!body.isMesh()) { r.error = "that body is already exact"; return r; }
    if (body.empty())   { r.error = "there is nothing to convert"; return r; }

    const Mesh& m = body.mesh();
    r.facesBefore = static_cast<int>(m.faces.size());

    // The limit is on what comes out, not on what goes in, and it is checked
    // before any work is done. A million triangles that are really a bracket
    // should convert; sixty thousand that are really a sculpture should not,
    // and the difference is not the triangle count.
    const int predicted = predictSolidFaces(body);
    if (predicted > maxFaces) {
        // Two different reasons to be over, and they want different words. A
        // scan or a sculpt has a plane per triangle and nothing to merge. A part
        // that was CAD merges well and is still over because its curved faces
        // left as hundreds of flat strips each -- which do not come back as
        // curves, so the result would be a solid you still could not fillet a
        // hole edge on.
        char buf[320];
        if (predicted * 2 > r.facesBefore)
            std::snprintf(buf, sizeof buf,
                          "this would give %d faces, over the limit of %d: its %d triangles "
                          "lie on %d different planes, so it looks scanned or sculpted and "
                          "there is almost nothing to merge. Modify > Reduce Mesh first",
                          predicted, maxFaces, r.facesBefore, predicted);
        else
            std::snprintf(buf, sizeof buf,
                          "this would give %d faces, over the limit of %d: the flat faces "
                          "merge, but its curved surfaces were exported as many thin strips. "
                          "Modify > Reduce Mesh first",
                          predicted, maxFaces);
        r.error = buf;
        return r;
    }
    if (!checkHealth(m, false).watertight) {
        r.error = "the surface has holes in it, and only a closed one can be a solid";
        return r;
    }

    // The fast route: the regions, traced from the mesh, handed over whole.
    {
        const Regions reg = findRegions(m);
        brep::PlanarRegions planar;
        if (traceRegions(m, reg, planar)) {
            std::string why;
            BrepRef solid = brep::solidFromPlanarRegions(planar, salt, &why);
            if (solid) {
                Body converted(std::move(solid));
                r.facesAfter = converted.faceCount();
                body = std::move(converted);
                r.ok = true;
                r.viaRegions = true;
                return r;
            }
        }
        // Falls through to building a face per triangle and letting the kernel
        // sew and merge. Slow, and tolerant of the meshes the tracing refuses --
        // a region pinched at a vertex, a sliver that fits no plane cleanly.
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

    // Which triangles share which edge. The kernel would otherwise work this
    // out by searching on position, which is where the time went; here it is a
    // hash of each vertex pair and one pass.
    std::vector<uint32_t> edgeEnds, triEdges;
    triEdges.resize(tris.size());
    {
        std::unordered_map<uint64_t, uint32_t> seen;
        seen.reserve(tris.size());
        for (size_t t = 0; t + 2 < tris.size(); t += 3) {
            for (int k = 0; k < 3; ++k) {
                const uint32_t a = tris[t + k];
                const uint32_t b = tris[t + (k + 1) % 3];
                const uint64_t lo = a < b ? a : b;
                const uint64_t hi = a < b ? b : a;
                const uint64_t key = (lo << 32) | hi;

                auto it = seen.find(key);
                if (it == seen.end()) {
                    const auto id = static_cast<uint32_t>(edgeEnds.size() / 2);
                    edgeEnds.push_back(a);      // stored the way it first ran
                    edgeEnds.push_back(b);
                    seen.emplace(key, id);
                    triEdges[t + k] = id;
                } else {
                    triEdges[t + k] = it->second;
                }
            }
        }
    }

    std::string why;
    BrepRef solid = brep::solidFromTriangles(pts, tris, edgeEnds, triEdges, salt, &why);
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
