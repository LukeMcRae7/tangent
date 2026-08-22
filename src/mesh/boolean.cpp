#include "mesh/boolean.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <utility>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

namespace tg {

const char* booleanOpName(BooleanOp op) {
    switch (op) {
        case BooleanOp::Union:        return "Union";
        case BooleanOp::Difference:   return "Difference";
        case BooleanOp::Intersection: return "Intersection";
    }
    return "Boolean";
}

namespace {

// How far from a plane a point may sit and still count as on it. Generous
// relative to double's resolution: the aim is to make near-coincident geometry
// decide one way consistently rather than to resolve it exactly.
constexpr Real kOnPlane = 1e-9;

struct Poly {
    std::vector<Vec3> v;
    Vec3 normal;
    Real w = 0.0;      // plane offset: dot(normal, p) == w for p on the plane

    bool computePlane() {
        if (v.size() < 3) return false;
        // Newell, so a slightly non-planar polygon still gets a sane plane.
        Vec3 n{};
        for (size_t i = 0; i < v.size(); ++i) {
            const Vec3& p = v[i];
            const Vec3& q = v[(i + 1) % v.size()];
            n.x += (p.y - q.y) * (p.z + q.z);
            n.y += (p.z - q.z) * (p.x + q.x);
            n.z += (p.x - q.x) * (p.y + q.y);
        }
        if (lengthSq(n) < 1e-24) return false;
        normal = normalize(n);
        w = dot(normal, v[0]);
        return true;
    }

    void flip() {
        std::reverse(v.begin(), v.end());
        normal = -normal;
        w = -w;
    }
};

enum : int { kCoplanar = 0, kFront = 1, kBack = 2, kSpanning = 3 };

// Splits `poly` against the plane (n, w), routing each piece to one of four
// buckets. This is the heart of the algorithm and the only place geometry is
// actually created.
void splitPolygon(const Poly& poly, Vec3 n, Real w,
                  std::vector<Poly>& coplanarFront, std::vector<Poly>& coplanarBack,
                  std::vector<Poly>& front, std::vector<Poly>& back) {
    int polyType = 0;
    std::vector<int> types;
    types.reserve(poly.v.size());

    for (const Vec3& p : poly.v) {
        const Real d = dot(n, p) - w;
        const int t = d < -kOnPlane ? kBack : (d > kOnPlane ? kFront : kCoplanar);
        polyType |= t;
        types.push_back(t);
    }

    switch (polyType) {
    case kCoplanar:
        // A coplanar face belongs with whichever side it faces, so that
        // touching solids do not leave a doubled surface.
        (dot(n, poly.normal) > 0 ? coplanarFront : coplanarBack).push_back(poly);
        break;
    case kFront:
        front.push_back(poly);
        break;
    case kBack:
        back.push_back(poly);
        break;
    default: {
        Poly f, b;
        const size_t count = poly.v.size();
        for (size_t i = 0; i < count; ++i) {
            const size_t j = (i + 1) % count;
            const int ti = types[i], tj = types[j];
            const Vec3& vi = poly.v[i];
            const Vec3& vj = poly.v[j];

            if (ti != kBack)  f.v.push_back(vi);
            if (ti != kFront) b.v.push_back(vi);

            if ((ti | tj) == kSpanning) {
                // Both halves get the *same* computed point, so the two new
                // edges match exactly and the seam can be welded later.
                const Real t = (w - dot(n, vi)) / dot(n, vj - vi);
                const Vec3 cut = lerp(vi, vj, t);
                f.v.push_back(cut);
                b.v.push_back(cut);
            }
        }
        f.normal = poly.normal; f.w = poly.w;
        b.normal = poly.normal; b.w = poly.w;
        if (f.v.size() >= 3) front.push_back(f);
        if (b.v.size() >= 3) back.push_back(b);
        break;
    }
    }
}

// BSP node. Built lazily from a polygon list; the first polygon's plane splits
// the rest.
struct Node {
    Vec3 normal;
    Real w = 0.0;
    bool hasPlane = false;
    std::vector<Poly> polys;
    std::unique_ptr<Node> front, back;

    void build(std::vector<Poly> input) {
        if (input.empty()) return;
        if (!hasPlane) {
            normal = input[0].normal;
            w = input[0].w;
            hasPlane = true;
        }

        std::vector<Poly> f, b;
        for (const Poly& p : input)
            splitPolygon(p, normal, w, polys, polys, f, b);

        if (!f.empty()) {
            if (!front) front = std::make_unique<Node>();
            front->build(std::move(f));
        }
        if (!b.empty()) {
            if (!back) back = std::make_unique<Node>();
            back->build(std::move(b));
        }
    }

    // Removes the parts of `input` that lie inside this solid.
    std::vector<Poly> clipPolygons(const std::vector<Poly>& input) const {
        if (!hasPlane) return input;

        std::vector<Poly> f, b;
        for (const Poly& p : input)
            splitPolygon(p, normal, w, f, b, f, b);

        std::vector<Poly> result = front ? front->clipPolygons(f) : f;
        if (back) {
            std::vector<Poly> clipped = back->clipPolygons(b);
            result.insert(result.end(), clipped.begin(), clipped.end());
        }
        // No back child means everything behind this plane is inside the
        // solid, so it is dropped.
        return result;
    }

    void clipTo(const Node& other) {
        polys = other.clipPolygons(polys);
        if (front) front->clipTo(other);
        if (back)  back->clipTo(other);
    }

    void invert() {
        for (Poly& p : polys) p.flip();
        normal = -normal;
        w = -w;
        std::swap(front, back);
        if (front) front->invert();
        if (back)  back->invert();
    }

    void gather(std::vector<Poly>& out) const {
        out.insert(out.end(), polys.begin(), polys.end());
        if (front) front->gather(out);
        if (back)  back->gather(out);
    }
};

std::vector<Poly> toPolygons(const Mesh& m) {
    std::vector<Poly> out;
    out.reserve(static_cast<size_t>(m.faceCount()));
    std::vector<Index> verts;
    for (Index f = 0; f < m.faceCount(); ++f) {
        m.faceVertices(f, verts);
        Poly p;
        p.v.reserve(verts.size());
        for (Index v : verts) p.v.push_back(m.verts[v].position);
        if (p.computePlane()) out.push_back(std::move(p));
    }
    return out;
}

// Rebuilds a mesh from loose polygons, welding coincident corners.
//
// Splitting produces the same cut point from both sides of an edge, but the
// arithmetic that produced it differs, so the values agree only to within a
// few ulps. Without welding, every seam would come back as a pair of open
// boundaries and build() would reject the result.
bool weldAndBuild(const std::vector<Poly>& polys, Mesh& out) {
    const bool dbg = std::getenv("TANGENT_BOOL_DEBUG") != nullptr;
    if (dbg) std::fprintf(stderr, "[bool] weld input: %zu polys\n", polys.size());
    if (polys.empty()) return false;

    AABB bounds;
    for (const Poly& p : polys)
        for (const Vec3& v : p.v) bounds.expand(v);
    if (!bounds.valid()) return false;

    // Tolerance scaled to the model: absolute epsilons are wrong for a part
    // that might be 2mm or 2000mm across.
    const Real scale = std::max(maxComponent(bounds.size()), Real(1));
    const Real weld = scale * 1e-9;
    const Real inv = 1.0 / weld;

    // Keyed on the exact quantised cell, never on a hash of it. Hashing the
    // cell and using that as the identity merges any two points whose hashes
    // collide -- which for symmetric geometry is constant, and silently fuses
    // opposite corners of the model.
    std::map<std::array<int64_t, 3>, uint32_t> lookup;
    std::vector<Vec3> positions;
    std::vector<uint32_t> faceSizes, faceIndices;

    auto cellOf = [&](const Vec3& p) {
        return std::array<int64_t, 3>{static_cast<int64_t>(std::llround(p.x * inv)),
                                      static_cast<int64_t>(std::llround(p.y * inv)),
                                      static_cast<int64_t>(std::llround(p.z * inv))};
    };

    auto findOrAdd = [&](const Vec3& p) {
        const std::array<int64_t, 3> c = cellOf(p);
        // Two points a hair apart can still land either side of a cell
        // boundary, so probe the neighbours rather than trusting the cell.
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz) {
                    auto it = lookup.find({c[0] + dx, c[1] + dy, c[2] + dz});
                    if (it != lookup.end() &&
                        lengthSq(positions[it->second] - p) <= weld * weld)
                        return it->second;
                }
        const uint32_t idx = static_cast<uint32_t>(positions.size());
        positions.push_back(p);
        lookup.emplace(c, idx);
        return idx;
    };

    for (const Poly& p : polys) {
        std::vector<uint32_t> loop;
        loop.reserve(p.v.size());
        for (const Vec3& v : p.v) loop.push_back(findOrAdd(v));

        // Drop repeated corners introduced by welding, and any polygon that
        // collapsed to a sliver in the process.
        std::vector<uint32_t> clean;
        for (size_t i = 0; i < loop.size(); ++i)
            if (loop[i] != loop[(i + 1) % loop.size()]) clean.push_back(loop[i]);
        if (clean.size() < 3) continue;

        faceSizes.push_back(static_cast<uint32_t>(clean.size()));
        faceIndices.insert(faceIndices.end(), clean.begin(), clean.end());
    }

    if (dbg) std::fprintf(stderr, "[bool] after weld: %zu verts, %zu faces\n",
                          positions.size(), faceSizes.size());
    if (faceSizes.empty()) return false;

    // Repair T-junctions.
    //
    // Clipping splits one polygon and not its neighbour, so a split point ends
    // up sitting partway along the neighbour's edge, touching it but not
    // referenced by it. The surface looks closed and is not: the two edges do
    // not match, so build() sees unpaired half-edges and rejects the result.
    // This is inherent to BSP CSG rather than a bug in the clipping.
    //
    // The fix is to insert any vertex lying on an edge into that edge. Vertices
    // are bucketed so each edge only examines the ones near it.
    {
        const Real snap = weld * 100.0;   // generous: these are true coincidences
        const Real cellSize = std::max(scale / 64.0, snap * 4.0);
        std::map<std::array<int64_t, 3>, std::vector<uint32_t>> buckets;
        auto bucketOf = [&](const Vec3& p) {
            return std::array<int64_t, 3>{
                static_cast<int64_t>(std::floor(p.x / cellSize)),
                static_cast<int64_t>(std::floor(p.y / cellSize)),
                static_cast<int64_t>(std::floor(p.z / cellSize))};
        };
        for (uint32_t i = 0; i < positions.size(); ++i)
            buckets[bucketOf(positions[i])].push_back(i);

        std::vector<uint32_t> rebuiltSizes, rebuiltIndices;
        size_t at = 0;
        for (uint32_t fs : faceSizes) {
            std::vector<uint32_t> loop(faceIndices.begin() + static_cast<long>(at),
                                       faceIndices.begin() + static_cast<long>(at + fs));
            at += fs;

            std::vector<uint32_t> grown;
            for (size_t i = 0; i < loop.size(); ++i) {
                const uint32_t ia = loop[i], ib = loop[(i + 1) % loop.size()];
                const Vec3 A = positions[ia], B = positions[ib];
                grown.push_back(ia);

                const Vec3 ab = B - A;
                const Real len2 = lengthSq(ab);
                if (len2 < 1e-24) continue;

                // Candidates from the cells the edge passes through.
                AABB ebox; ebox.expand(A); ebox.expand(B);
                std::vector<std::pair<Real, uint32_t>> onEdge;
                const auto lo = bucketOf(ebox.min), hi = bucketOf(ebox.max);
                for (int64_t x = lo[0]; x <= hi[0]; ++x)
                for (int64_t y = lo[1]; y <= hi[1]; ++y)
                for (int64_t z = lo[2]; z <= hi[2]; ++z) {
                    auto it = buckets.find({x, y, z});
                    if (it == buckets.end()) continue;
                    for (uint32_t v : it->second) {
                        if (v == ia || v == ib) continue;
                        const Vec3 P = positions[v];
                        const Real t = dot(P - A, ab) / len2;
                        if (t <= 1e-9 || t >= 1.0 - 1e-9) continue;
                        if (lengthSq(P - (A + ab * t)) > snap * snap) continue;
                        onEdge.emplace_back(t, v);
                    }
                }
                std::sort(onEdge.begin(), onEdge.end());
                for (const auto& [t, v] : onEdge) {
                    (void)t;
                    if (grown.back() != v) grown.push_back(v);
                }
            }

            if (grown.size() < 3) continue;
            rebuiltSizes.push_back(static_cast<uint32_t>(grown.size()));
            rebuiltIndices.insert(rebuiltIndices.end(), grown.begin(), grown.end());
        }
        faceSizes.swap(rebuiltSizes);
        faceIndices.swap(rebuiltIndices);
    }

    if (dbg) std::fprintf(stderr, "[bool] after T-junction repair: %zu faces\n",
                          faceSizes.size());
    if (faceSizes.empty()) return false;

    // Resolve coincident faces.
    //
    // When an operand's face is exactly coplanar with the other's -- cutting
    // along a plane a previous cut already left, which is completely ordinary
    // in real modelling -- clipping can keep both copies. Two faces on the same
    // corners walk the same directed edge, and build() rightly refuses that.
    //
    // Same winding means the surface is simply there twice: keep one. Opposite
    // winding means an inward and an outward skin back to back, enclosing
    // nothing: drop both, or the result has an internal wall.
    {
        // Rotate each loop to start at its lowest index so the same cycle
        // always yields the same key, then compare against the reverse to tell
        // the two windings apart.
        auto canonical = [](std::vector<uint32_t> loop) {
            const size_t n = loop.size();
            const size_t start = static_cast<size_t>(
                std::min_element(loop.begin(), loop.end()) - loop.begin());
            std::vector<uint32_t> out;
            out.reserve(n);
            for (size_t i = 0; i < n; ++i) out.push_back(loop[(start + i) % n]);
            return out;
        };

        std::map<std::vector<uint32_t>, size_t> seen;   // sorted corners -> face
        std::vector<std::vector<uint32_t>> loops;
        std::vector<bool> dropped;

        size_t at = 0;
        for (uint32_t fs : faceSizes) {
            loops.emplace_back(faceIndices.begin() + static_cast<long>(at),
                               faceIndices.begin() + static_cast<long>(at + fs));
            dropped.push_back(false);
            at += fs;
        }

        for (size_t i = 0; i < loops.size(); ++i) {
            std::vector<uint32_t> key = loops[i];
            std::sort(key.begin(), key.end());
            auto it = seen.find(key);
            if (it == seen.end()) { seen.emplace(std::move(key), i); continue; }

            const size_t j = it->second;
            if (dropped[j]) { seen[key] = i; continue; }

            const std::vector<uint32_t> ci = canonical(loops[i]);
            std::vector<uint32_t> rev = loops[i];
            std::reverse(rev.begin(), rev.end());
            const std::vector<uint32_t> cr = canonical(std::move(rev));
            const std::vector<uint32_t> cj = canonical(loops[j]);

            if (cj == ci)      dropped[i] = true;                 // duplicate skin
            else if (cj == cr) { dropped[i] = true; dropped[j] = true; }  // cancelling pair
        }

        std::vector<uint32_t> keptSizes, keptIndices;
        for (size_t i = 0; i < loops.size(); ++i) {
            if (dropped[i]) continue;
            keptSizes.push_back(static_cast<uint32_t>(loops[i].size()));
            keptIndices.insert(keptIndices.end(), loops[i].begin(), loops[i].end());
        }
        if (dbg && keptSizes.size() != faceSizes.size())
            std::fprintf(stderr, "[bool] coincident faces: %zu -> %zu\n",
                         faceSizes.size(), keptSizes.size());
        faceSizes.swap(keptSizes);
        faceIndices.swap(keptIndices);
    }

    if (faceSizes.empty()) return false;

    Mesh built;
    if (!built.build(positions, faceSizes, faceIndices)) {
        if (std::getenv("TANGENT_BOOL_DEBUG")) {
            // How many directed edges are unpaired, and are any faces slivers?
            std::map<std::pair<uint32_t,uint32_t>, int> dir;
            size_t at = 0; int degen = 0;
            for (uint32_t fs : faceSizes) {
                Real area = 0.0;
                for (uint32_t i = 0; i < fs; ++i) {
                    const uint32_t a2 = faceIndices[at + i];
                    const uint32_t b2 = faceIndices[at + (i + 1) % fs];
                    ++dir[{a2, b2}];
                    if (i >= 2)
                        area += length(cross(positions[faceIndices[at+i-1]] - positions[faceIndices[at]],
                                             positions[faceIndices[at+i]] - positions[faceIndices[at]]));
                }
                if (area < 1e-12) ++degen;
                at += fs;
            }
            int unpaired = 0, dupes = 0;
            for (const auto& [k, n] : dir) {
                if (n > 1) ++dupes;
                if (!dir.count({k.second, k.first})) ++unpaired;
            }
            std::fprintf(stderr,
                "[bool] REJECTED: %zu polys, %zu verts, %zu faces, %d unpaired edges, "
                "%d duplicated directed edges, %d sliver faces\n",
                polys.size(), positions.size(), faceSizes.size(), unpaired, dupes, degen);
        }
        return false;
    }
    out = std::move(built);
    return true;
}

bool isClosed(const Mesh& m) {
    if (m.empty()) return false;
    for (Index he = 0; he < m.halfedgeCount(); ++he)
        if (m.halfedges[he].face == kInvalid) return false;
    return true;
}

} // namespace

bool meshBoolean(const Mesh& a, const Mesh& b, BooleanOp op, Mesh& out) {
    // An open surface has no inside, so "inside the other solid" is undefined.
    if (!isClosed(a) || !isClosed(b)) return false;

    Node na, nb;
    na.build(toPolygons(a));
    nb.build(toPolygons(b));

    // The three operations are the same clipping sequence with different
    // inversions, which is what makes a BSP formulation worth the slivers.
    switch (op) {
    case BooleanOp::Union:
        na.clipTo(nb);
        nb.clipTo(na);
        // Drop nb's surface that coincides with na's, or the shared skin comes
        // back twice.
        nb.invert();
        nb.clipTo(na);
        nb.invert();
        break;

    case BooleanOp::Intersection:
        na.invert();
        nb.clipTo(na);
        nb.invert();
        na.clipTo(nb);
        nb.clipTo(na);
        na.invert();
        nb.invert();
        break;

    case BooleanOp::Difference:
        na.invert();
        na.clipTo(nb);
        nb.clipTo(na);
        nb.invert();
        nb.clipTo(na);
        nb.invert();
        // The closing inversion applies to *both* sets, not just the first.
        // The cutter's faces become the cavity wall, so they have to end up
        // facing into the void; leaving them as they were gives the result two
        // oppositely wound halves and build() refuses it.
        na.invert();
        nb.invert();
        break;
    }

    std::vector<Poly> result;
    na.gather(result);
    nb.gather(result);

    return weldAndBuild(result, out);
}

} // namespace tg
