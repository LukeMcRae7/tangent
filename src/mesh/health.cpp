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
    const Real det = dot(e1, pv);
    // Parallel, tested scale-free. An absolute cutoff is a different test on a
    // 2 mm part than on a 2 m one: det carries the cube of a length, so at
    // 1e-12 a 20 mm model's coplanar pairs sit right on the threshold, and
    // whichever side of it rounding puts them on decides the answer. Comparing
    // against the magnitudes it was built from is the same test at any size.
    if (std::fabs(det) <= 1e-10 * length(e1) * length(pv)) return false;

    const Real inv = 1.0f / det;
    const Vec3 tv = p0 - a;
    const Real u = dot(tv, pv) * inv;
    if (u < 1e-6f || u > 1.0f - 1e-6f) return false;

    const Vec3 qv = cross(tv, e1);
    const Real v = dot(dir, qv) * inv;
    if (v < 1e-6f || u + v > 1.0f - 1e-6f) return false;

    const Real t = dot(e2, qv) * inv;
    // Strictly inside the segment: touching exactly at an endpoint is what
    // legitimately adjacent triangles do.
    return t > 1e-5f && t < 1.0f - 1e-5f;
}

struct Tri {
    uint32_t a, b, c;
    Vec3 pa, pb, pc;
    AABB box;
    Index face = kInvalid;   // the polygon it was triangulated from
};

bool trianglesCross(const Tri& x, const Tri& y) {
    // Two triangles of the same polygon are that polygon's triangulation, not
    // two pieces of surface. Ear clipping cannot make them overlap, and being
    // coplanar is the case the crossing test is least stable on. A face that
    // genuinely folds through itself is a degenerate face, counted as one.
    if (x.face != kInvalid && x.face == y.face) return false;

    // Triangles sharing an edge meet along it legitimately and must be
    // excluded. Comparing indices is not enough: the render mesh gives every
    // face its own copy of each corner (that is what lets adjacent faces
    // disagree about the normal), so neighbouring triangles never share an
    // index even though they share a position -- hence the comparison by
    // position.
    //
    // Two coincident corners, not one. Excluding any pair that shared a single
    // corner let through the one shape this check exists to catch: a fan that
    // folds back through itself pivots on its apex, so every pair in it shares
    // that corner and every pair was waved past. The fillet's acceptance gate
    // is checkHealth().solid(), which left it blind to its own characteristic
    // failure. Touching *at* a shared corner is already excluded by the strict
    // parameter bounds in segmentHitsTriangle -- an edge leaving that corner
    // meets the other triangle's plane at t = 0.
    const Vec3 xs[3] = {x.pa, x.pb, x.pc};
    const Vec3 ys[3] = {y.pa, y.pb, y.pc};
    int shared = 0;
    for (const Vec3& p : xs)
        for (const Vec3& q : ys)
            if (lengthSq(p - q) < 1e-12) ++shared;
    if (shared >= 2) return false;

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
        if (i / 3 < rm.triangleFace.size()) t.face = rm.triangleFace[i / 3];
        t.box.expand(t.pa); t.box.expand(t.pb); t.box.expand(t.pc);
        edgeSum += length(t.box.size());
        tris.push_back(t);
    }
    if (tris.size() < 2) return h;

    // Cell about the size of an average triangle: small enough to cut the pair
    // count down, large enough that triangles do not smear across many cells.
    const Real cell = std::max(edgeSum / static_cast<Real>(tris.size()), Real(1e-4));

    auto key = [&](int x, int y, int z) {
        return (static_cast<uint64_t>(static_cast<uint32_t>(x)) * 73856093u) ^
               (static_cast<uint64_t>(static_cast<uint32_t>(y)) * 19349663u) ^
               (static_cast<uint64_t>(static_cast<uint32_t>(z)) * 83492791u);
    };

    std::unordered_map<uint64_t, std::vector<uint32_t>> grid;
    grid.reserve(tris.size() * 2);
    // A triangle spanning a huge number of cells would defeat the point of
    // having a grid, so it is held out of it and scanned separately below.
    // Holding it out and then never scanning it -- which is what "left to the
    // pair scan" meant, there being no pair scan -- exempted exactly the
    // triangles most likely to be crossed: the big flat wall a part is built
    // from, next to the small facets an operation covers it in.
    std::vector<uint32_t> oversized;
    for (uint32_t i = 0; i < tris.size(); ++i) {
        const Tri& t = tris[i];
        const int x0 = static_cast<int>(std::floor(t.box.min.x / cell));
        const int x1 = static_cast<int>(std::floor(t.box.max.x / cell));
        const int y0 = static_cast<int>(std::floor(t.box.min.y / cell));
        const int y1 = static_cast<int>(std::floor(t.box.max.y / cell));
        const int z0 = static_cast<int>(std::floor(t.box.min.z / cell));
        const int z1 = static_cast<int>(std::floor(t.box.max.z / cell));
        if (static_cast<int64_t>(x1 - x0 + 1) * (y1 - y0 + 1) * (z1 - z0 + 1) > 512) {
            oversized.push_back(i);
            continue;
        }
        for (int x = x0; x <= x1; ++x)
            for (int y = y0; y <= y1; ++y)
                for (int z = z0; z <= z1; ++z)
                    grid[key(x, y, z)].push_back(i);
    }

    // Each held-out triangle against every other, and against each other once.
    // There are few of them by construction -- one has to span 512 cells to
    // qualify -- so this stays far below the grid's own cost.
    for (size_t a = 0; a < oversized.size(); ++a) {
        const uint32_t i = oversized[a];
        for (uint32_t j = 0; j < tris.size(); ++j) {
            if (j == i) continue;
            // Pairs of held-out triangles are tested once, from the lower one.
            if (j < i && std::binary_search(oversized.begin(), oversized.end(), j))
                continue;
            if (!tris[i].box.overlaps(tris[j].box)) continue;
            if (trianglesCross(tris[i], tris[j])) ++h.selfIntersections;
        }
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
