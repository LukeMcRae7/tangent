#include "mesh/health.h"

#include <algorithm>
#include <unordered_map>
#include <vector>

namespace tg {
namespace {

// Does a segment cross a triangle? Moller-Trumbore with the ray parameter
// bounded to the segment. Testing all six edges of a pair against the opposite
// triangle catches every non-coplanar crossing, and is far easier to get right
// than a dedicated triangle-triangle routine.
bool segmentHitsTriangle(Vec3 p0, Vec3 p1, Vec3 a, Vec3 b, Vec3 c) {
    const Vec3 dir = p1 - p0;
    const Vec3 e1 = b - a, e2 = c - a;
    const Vec3 pv = cross(dir, e2);
    const float det = dot(e1, pv);
    if (std::fabs(det) < 1e-12f) return false;      // parallel

    const float inv = 1.0f / det;
    const Vec3 tv = p0 - a;
    const float u = dot(tv, pv) * inv;
    if (u < 1e-6f || u > 1.0f - 1e-6f) return false;

    const Vec3 qv = cross(tv, e1);
    const float v = dot(dir, qv) * inv;
    if (v < 1e-6f || u + v > 1.0f - 1e-6f) return false;

    const float t = dot(e2, qv) * inv;
    // Strictly inside the segment: touching exactly at an endpoint is what
    // legitimately adjacent triangles do.
    return t > 1e-5f && t < 1.0f - 1e-5f;
}

struct Tri {
    uint32_t a, b, c;
    Vec3 pa, pb, pc;
    AABB box;
};

bool trianglesCross(const Tri& x, const Tri& y) {
    // Triangles that meet at a shared corner touch legitimately and must be
    // excluded. Comparing indices is not enough: the render mesh gives every
    // face its own copy of each corner (that is what lets adjacent faces
    // disagree about the normal), so neighbouring triangles never share an
    // index even though they share a position.
    const Vec3 xs[3] = {x.pa, x.pb, x.pc};
    const Vec3 ys[3] = {y.pa, y.pb, y.pc};
    for (const Vec3& p : xs)
        for (const Vec3& q : ys)
            if (p == q) return false;

    return segmentHitsTriangle(x.pa, x.pb, y.pa, y.pb, y.pc) ||
           segmentHitsTriangle(x.pb, x.pc, y.pa, y.pb, y.pc) ||
           segmentHitsTriangle(x.pc, x.pa, y.pa, y.pb, y.pc) ||
           segmentHitsTriangle(y.pa, y.pb, x.pa, x.pb, x.pc) ||
           segmentHitsTriangle(y.pb, y.pc, x.pa, x.pb, x.pc) ||
           segmentHitsTriangle(y.pc, y.pa, x.pa, x.pb, x.pc);
}

} // namespace

MeshHealth checkHealth(const Mesh& mesh, bool checkIntersections) {
    MeshHealth h;
    h.selfIntersections = checkIntersections ? 0 : -1;
    if (mesh.empty()) return h;

    for (Index he = 0; he < mesh.halfedgeCount(); ++he)
        if (mesh.halfedges[he].face == kInvalid) ++h.boundaryEdges;
    h.watertight = h.boundaryEdges == 0;

    for (Index f = 0; f < mesh.faceCount(); ++f)
        if (mesh.faceArea(f) < 1e-9f) ++h.degenerateFaces;

    // Connected components, walked across shared edges.
    {
        std::vector<bool> seen(static_cast<size_t>(mesh.faceCount()), false);
        std::vector<Index> stack;
        for (Index f = 0; f < mesh.faceCount(); ++f) {
            if (seen[f]) continue;
            ++h.shells;
            stack.push_back(f);
            seen[f] = true;
            while (!stack.empty()) {
                const Index cur = stack.back();
                stack.pop_back();
                const Index start = mesh.faces[cur].halfedge;
                Index he = start;
                do {
                    const Index nf = mesh.halfedges[mesh.halfedges[he].twin].face;
                    if (nf != kInvalid && !seen[nf]) { seen[nf] = true; stack.push_back(nf); }
                    he = mesh.halfedges[he].next;
                } while (he != start);
            }
        }
    }

    RenderMesh rm;
    mesh.buildRenderMesh(rm);

    // Signed volume by the divergence theorem; negative means inside out.
    double s6 = 0.0;
    for (size_t i = 0; i < rm.triangles.size(); i += 3)
        s6 += dot(rm.positions[rm.triangles[i]],
                  cross(rm.positions[rm.triangles[i + 1]], rm.positions[rm.triangles[i + 2]]));
    h.volume = s6 / 6.0;

    if (!checkIntersections) return h;

    // ---- Self-intersection, broad-phased through a uniform grid -----------
    std::vector<Tri> tris;
    tris.reserve(rm.triangles.size() / 3);
    double edgeSum = 0.0;
    for (size_t i = 0; i < rm.triangles.size(); i += 3) {
        Tri t;
        t.a = rm.triangles[i]; t.b = rm.triangles[i + 1]; t.c = rm.triangles[i + 2];
        t.pa = rm.positions[t.a]; t.pb = rm.positions[t.b]; t.pc = rm.positions[t.c];
        t.box.expand(t.pa); t.box.expand(t.pb); t.box.expand(t.pc);
        edgeSum += length(t.box.size());
        tris.push_back(t);
    }
    if (tris.size() < 2) return h;

    // Cell about the size of an average triangle: small enough to cut the pair
    // count down, large enough that triangles do not smear across many cells.
    const float cell = std::max(static_cast<float>(edgeSum / static_cast<double>(tris.size())),
                                1e-4f);

    auto key = [&](int x, int y, int z) {
        return (static_cast<uint64_t>(static_cast<uint32_t>(x)) * 73856093u) ^
               (static_cast<uint64_t>(static_cast<uint32_t>(y)) * 19349663u) ^
               (static_cast<uint64_t>(static_cast<uint32_t>(z)) * 83492791u);
    };

    std::unordered_map<uint64_t, std::vector<uint32_t>> grid;
    grid.reserve(tris.size() * 2);
    for (uint32_t i = 0; i < tris.size(); ++i) {
        const Tri& t = tris[i];
        const int x0 = static_cast<int>(std::floor(t.box.min.x / cell));
        const int x1 = static_cast<int>(std::floor(t.box.max.x / cell));
        const int y0 = static_cast<int>(std::floor(t.box.min.y / cell));
        const int y1 = static_cast<int>(std::floor(t.box.max.y / cell));
        const int z0 = static_cast<int>(std::floor(t.box.min.z / cell));
        const int z1 = static_cast<int>(std::floor(t.box.max.z / cell));
        // A triangle spanning a huge number of cells would defeat the point;
        // such a triangle is rare and is simply left to the pair scan.
        if (static_cast<int64_t>(x1 - x0 + 1) * (y1 - y0 + 1) * (z1 - z0 + 1) > 512) continue;
        for (int x = x0; x <= x1; ++x)
            for (int y = y0; y <= y1; ++y)
                for (int z = z0; z <= z1; ++z)
                    grid[key(x, y, z)].push_back(i);
    }

    // A pair of triangles can share several cells, so each pair must be tested
    // once. Recording every tested pair in a hash set is the obvious way and
    // was by far the dominant cost -- millions of insertions. Instead, test a
    // pair only in the one cell that is canonical for it: the minimum corner
    // of the overlap between their two boxes. That is O(1) to compute and
    // needs no memory.
    for (const auto& entry : grid) {
        const int64_t cellX = static_cast<int64_t>(entry.first & 0xFFFFF);
        (void)cellX;
        const std::vector<uint32_t>& bucket = entry.second;
        for (size_t i = 0; i < bucket.size(); ++i) {
            for (size_t j = i + 1; j < bucket.size(); ++j) {
                const uint32_t a = bucket[i], b = bucket[j];
                const AABB& ba = tris[a].box;
                const AABB& bb = tris[b].box;

                // Cheap reject before the six segment tests.
                if (ba.max.x < bb.min.x || bb.max.x < ba.min.x ||
                    ba.max.y < bb.min.y || bb.max.y < ba.min.y ||
                    ba.max.z < bb.min.z || bb.max.z < ba.min.z) continue;

                // Canonical cell for this pair: where their overlap starts.
                const int ox = static_cast<int>(std::floor(std::max(ba.min.x, bb.min.x) / cell));
                const int oy = static_cast<int>(std::floor(std::max(ba.min.y, bb.min.y) / cell));
                const int oz = static_cast<int>(std::floor(std::max(ba.min.z, bb.min.z) / cell));
                if (key(ox, oy, oz) != entry.first) continue;

                if (trianglesCross(tris[a], tris[b])) ++h.selfIntersections;
            }
        }
    }
    return h;
}

} // namespace tg
