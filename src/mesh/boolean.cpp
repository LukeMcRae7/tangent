#include "mesh/boolean.h"

#include "mesh/operations.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <functional>
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

    // The face this piece was cut from, and which operand it belongs to. The
    // result is named from these: without them every face of a boolean's output
    // comes back nameless, a feature that acted on one stores nothing, and on
    // the next evaluation "nothing" matches the first face in the mesh. An
    // extrude of the top of a cut body would quietly move the bottom instead.
    ElementId src = kNoId;
    bool fromB = false;

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
        f.normal = poly.normal; f.w = poly.w; f.src = poly.src; f.fromB = poly.fromB;
        b.normal = poly.normal; b.w = poly.w; b.src = poly.src; b.fromB = poly.fromB;
        if (f.v.size() >= 3) front.push_back(f);
        if (b.v.size() >= 3) back.push_back(b);
        break;
    }
    }
}

// BSP node. Built lazily from a polygon list; the first polygon's plane splits
// the rest.
// The BSP tree that used to drive this is gone.
//
// It was only ever a way to decide which side of one solid a piece of the other
// was on, and it decided it by where the piece ended up in a tree of planes --
// which made the answer depend on how the tree was built, split faces against
// planes nowhere near them, and could not be asked twice about the same body.
// Asking the geometry directly costs a solid-angle sum per piece and has none
// of those properties.

std::vector<Poly> toPolygons(const Mesh& m, bool fromB) {
    std::vector<Poly> out;
    out.reserve(static_cast<size_t>(m.faceCount()) * 2);
    std::vector<Index> verts, tris;
    for (Index f = 0; f < m.faceCount(); ++f) {
        m.faceVertices(f, verts);
        if (verts.size() == 3) {
            Poly p;
            p.src = m.faces[f].id;
            p.fromB = fromB;
            for (Index v : verts) p.v.push_back(m.verts[v].position);
            if (p.computePlane()) out.push_back(std::move(p));
            continue;
        }
        tris.clear();   // it appends, and the corner indices are per face
        m.triangulateFacePublic(f, tris);
        for (size_t i = 0; i + 2 < tris.size(); i += 3) {
            Poly p;
            p.src = m.faces[f].id;
            p.fromB = fromB;
            p.v = {m.verts[verts[tris[i]]].position,
                   m.verts[verts[tris[i + 1]]].position,
                   m.verts[verts[tris[i + 2]]].position};
            if (p.computePlane()) out.push_back(std::move(p));
        }
    }
    return out;
}

// Rebuilds a mesh from loose polygons, welding coincident corners.
//
// Splitting produces the same cut point from both sides of an edge, but the
// arithmetic that produced it differs, so the values agree only to within a
// few ulps. Without welding, every seam would come back as a pair of open
// boundaries and build() would reject the result.
bool weldAndBuild(const std::vector<Poly>& polys, const Mesh& a, const Mesh& b,
                  ElementId salt, Mesh& out) {
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

    // Names carried alongside, because every step below can drop a face and the
    // built mesh has to end up knowing what each of its faces used to be.
    std::vector<ElementId> faceName;
    std::map<std::pair<ElementId, bool>, uint64_t> fragmentOf;

    auto cellOf = [&](const Vec3& p) {
        return std::array<int64_t, 3>{static_cast<int64_t>(std::llround(p.x * inv)),
                                      static_cast<int64_t>(std::llround(p.y * inv)),
                                      static_cast<int64_t>(std::llround(p.z * inv))};
    };

    // A point that was already a vertex of an operand keeps that operand's
    // name. The second operand's names are re-derived: two boxes name their
    // corners identically, and letting both through would put the same name on
    // two different vertices of the result.
    std::map<std::array<int64_t, 3>, ElementId> knownVertex;
    for (const MeshVertex& mv : a.verts)
        if (mv.id != kNoId) knownVertex.emplace(cellOf(mv.position), mv.id);
    for (const MeshVertex& mv : b.verts)
        if (mv.id != kNoId)
            knownVertex.emplace(cellOf(mv.position),
                                nameId(salt, IdRole::Vertex, mv.id, 1));

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

        // One source face can arrive in several pieces, so they are counted
        // apart. Emission order is fixed by the inputs, so re-running the same
        // operation names them the same way again.
        const uint64_t k = fragmentOf[{p.src, p.fromB}]++;
        faceName.push_back(nameId(salt, IdRole::Face, p.src, p.fromB ? 1 : 0, k));
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
        std::vector<ElementId> rebuiltNames;
        size_t at = 0, face = 0;
        for (uint32_t fs : faceSizes) {
            const size_t self = face++;
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
            rebuiltNames.push_back(faceName[self]);
            rebuiltSizes.push_back(static_cast<uint32_t>(grown.size()));
            rebuiltIndices.insert(rebuiltIndices.end(), grown.begin(), grown.end());
        }
        faceSizes.swap(rebuiltSizes);
        faceIndices.swap(rebuiltIndices);
        faceName.swap(rebuiltNames);
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
        std::vector<ElementId> keptNames;
        for (size_t i = 0; i < loops.size(); ++i) {
            if (dropped[i]) continue;
            keptNames.push_back(faceName[i]);
            keptSizes.push_back(static_cast<uint32_t>(loops[i].size()));
            keptIndices.insert(keptIndices.end(), loops[i].begin(), loops[i].end());
        }
        if (dbg && keptSizes.size() != faceSizes.size())
            std::fprintf(stderr, "[bool] coincident faces: %zu -> %zu\n",
                         faceSizes.size(), keptSizes.size());
        faceSizes.swap(keptSizes);
        faceIndices.swap(keptIndices);
        faceName.swap(keptNames);
    }

    if (faceSizes.empty()) return false;

    Mesh::Names names;
    names.faces = faceName;
    names.vertices.reserve(positions.size());
    for (const Vec3& p : positions) {
        auto it = knownVertex.find(cellOf(p));
        if (it == knownVertex.end()) {
            // Cut through neighbouring cells too, or a point a hair either side
            // of a boundary is treated as new when it is not.
            const auto cc = cellOf(p);
            for (int dx = -1; dx <= 1 && it == knownVertex.end(); ++dx)
                for (int dy = -1; dy <= 1 && it == knownVertex.end(); ++dy)
                    for (int dz = -1; dz <= 1 && it == knownVertex.end(); ++dz)
                        it = knownVertex.find({cc[0] + dx, cc[1] + dy, cc[2] + dz});
        }
        if (it != knownVertex.end()) { names.vertices.push_back(it->second); continue; }

        // Born on the intersection curve, so it has no earlier name. Derived
        // from where it is, which is reproducible for the same inputs.
        const auto cc = cellOf(p);
        names.vertices.push_back(nameId(salt, IdRole::Vertex,
                                        static_cast<ElementId>(cc[0]),
                                        static_cast<ElementId>(cc[1]),
                                        static_cast<uint64_t>(cc[2])));
    }

    Mesh built;
    if (!built.build(positions, faceSizes, faceIndices, &names)) {
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

// Is `p` inside the solid these triangles bound?
//
// By solid angle, not by counting ray crossings. A ray has to be aimed, and any
// aim can graze an edge or pass exactly through a vertex -- firing at a cube's
// centre along (1,1,1) leaves through the corner, and the count is then a coin
// toss. Nudging the ray and retrying only moves the problem.
//
// The solid angle each triangle subtends at `p` has no such special cases. Sum
// them and the total is a full sphere for a point inside a closed surface and
// nothing for one outside, with no direction to choose and nothing to graze.
bool pointInside(const std::vector<Poly>& tris, Vec3 p) {
    Real total = 0.0;
    for (const Poly& t : tris) {
        if (t.v.size() < 3) continue;
        // Fan the polygon; every piece's solid angle adds.
        for (size_t i = 1; i + 1 < t.v.size(); ++i) {
            const Vec3 a = t.v[0] - p, b = t.v[i] - p, cc = t.v[i + 1] - p;
            const Real la = length(a), lb = length(b), lc = length(cc);
            if (la < 1e-12 || lb < 1e-12 || lc < 1e-12) return true;   // on the surface
            const Real num = dot(a, cross(b, cc));
            const Real den = la * lb * lc + dot(a, b) * lc + dot(b, cc) * la +
                             dot(cc, a) * lb;
            total += 2.0 * std::atan2(num, den);
        }
    }
    return std::fabs(total) > 2.0 * kPi;   // half of a full sphere
}

AABB polyBounds(const Poly& p) {
    AABB b;
    for (const Vec3& q : p.v) b.expand(q);
    return b;
}

bool boxesOverlap(const AABB& a, const AABB& b, Real slack) {
    return a.min.x - slack <= b.max.x && b.min.x - slack <= a.max.x &&
           a.min.y - slack <= b.max.y && b.min.y - slack <= a.max.y &&
           a.min.z - slack <= b.max.z && b.min.z - slack <= a.max.z;
}

// Where a piece of surface sits relative to the other solid.
enum class Side { Outside, Inside, OnSurface };

// Probes just off each face of the fragment. A fragment lying in the other
// solid's surface reads differently on its two sides, and that is the case that
// has to be recognised rather than guessed at: it is a shared skin, and which
// copy survives is a question about the operation, not about geometry.
Side sideOf(const Poly& frag, const std::vector<Poly>& otherTris, Real eps,
            bool* sameFacing = nullptr) {
    Vec3 c{};
    for (const Vec3& q : frag.v) c += q;
    c = c / static_cast<Real>(frag.v.size());

    const bool behind = pointInside(otherTris, c - frag.normal * eps);
    const bool ahead  = pointInside(otherTris, c + frag.normal * eps);
    if (behind == ahead) return behind ? Side::Inside : Side::Outside;

    // Material behind it means the other solid's surface faces the same way
    // here; material ahead means the two solids are back to back and this is an
    // internal wall.
    if (sameFacing) *sameFacing = behind;
    return Side::OnSurface;
}

// Keeps whole every face the other solid does not reach, splits the ones it
// does, and decides each piece on its own.
//
// A BSP's planes are infinite: clipping a face against the tree splits it at
// every plane it happens to cross on the way down, however far that is from any
// real intersection. A cube with a small notch taken out came back with all six
// walls diced into fans, and feeding that result into a second boolean turned
// forty triangles into three and a half thousand -- which is why a body could
// only be cut once.
// `ownsShared` decides what happens to the skin the two solids have in common.
//
// It cannot be settled by matching identical faces afterwards, which is what
// used to be attempted: the two solids triangulate that skin independently, and
// their pieces have different diagonals, so nothing matches. It is not a
// geometric question anyway. Where two surfaces coincide and face the same way,
// the result needs exactly one copy of it, and saying which solid provides it
// is simpler and always right.
void classify(const std::vector<Poly>& faces, const std::vector<Poly>& otherTris,
              bool keepInside, bool ownsShared, std::vector<Poly>& out) {
    AABB otherBox;
    for (const Poly& t : otherTris)
        for (const Vec3& q : t.v) otherBox.expand(q);

    const Real scale = std::max(length(otherBox.size()), Real(1.0));
    const Real slack = scale * 1e-9;
    const Real eps   = scale * 1e-6;

    const bool dbgc = std::getenv("TANGENT_BOOL_DEBUG") != nullptr;
    size_t whole = 0;

    std::vector<Poly> straddling;
    for (const Poly& f : faces) {
        const AABB fb = polyBounds(f);

        bool touches = false;
        if (boxesOverlap(fb, otherBox, slack))
            for (const Poly& t : otherTris)
                if (boxesOverlap(fb, polyBounds(t), slack)) { touches = true; break; }

        if (!touches) {
            ++whole;
            // Nothing of the other solid comes near it, so the whole face is on
            // one side and one probe settles which.
            bool sameFacing = false;
            const Side s = sideOf(f, otherTris, eps, &sameFacing);
            if (s == Side::OnSurface) {
                if (ownsShared && sameFacing) out.push_back(f);
            } else if ((s == Side::Inside) == keepInside) {
                out.push_back(f);
            }
            continue;
        }
        straddling.push_back(f);
    }

    // Split each face against the planes of the triangles that actually reach
    // it, and nothing else.
    //
    // Not by walking the tree. A polygon lying in one of the tree's own planes
    // gets routed into whichever subtree that plane faces, and if that subtree
    // is empty it is emitted there and then -- never meeting the planes further
    // down that are the ones which cut it. Two overlapping cubes share four
    // planes, and their side walls came back uncut because of exactly that.
    std::vector<Poly> pieces;
    for (const Poly& f : straddling) {
        const AABB fb = polyBounds(f);

        std::vector<std::pair<Vec3, Real>> planes;
        for (const Poly& t : otherTris) {
            if (!boxesOverlap(fb, polyBounds(t), slack)) continue;
            bool seen = false;
            for (const auto& [pn, pw] : planes)
                if (dot(pn, t.normal) > 0.9999999 && std::fabs(pw - t.w) < slack) {
                    seen = true;
                    break;
                }
            if (!seen) planes.emplace_back(t.normal, t.w);
        }

        std::vector<Poly> cur{f}, next;
        for (const auto& [pn, pw] : planes) {
            next.clear();
            for (const Poly& p : cur) splitPolygon(p, pn, pw, next, next, next, next);
            cur.swap(next);
        }
        for (Poly& p : cur) pieces.push_back(std::move(p));
    }

    size_t kept = 0, shared = 0;
    for (const Poly& p : pieces) {
        bool sameFacing = false;
        const Side s = sideOf(p, otherTris, eps, &sameFacing);
        if (s == Side::OnSurface) {
            // Back-to-back surfaces are an internal wall in every operation and
            // belong to neither result.
            if (ownsShared && sameFacing) { out.push_back(p); ++shared; }
            continue;
        }
        if ((s == Side::Inside) == keepInside) { out.push_back(p); ++kept; }
    }

    if (dbgc)
        std::fprintf(stderr,
                     "[bool] keepInside=%d: %zu faces, %zu whole, %zu split into %zu "
                     "(%zu kept, %zu shared)\n",
                     (int)keepInside, faces.size(), whole, straddling.size(),
                     pieces.size(), kept, shared);
}

} // namespace

// Test hook for the ray classifier.
bool debugPointInsideMesh(const Mesh& m, Vec3 p) {
    return pointInside(toPolygons(m, false), p);
}

bool meshBoolean(const Mesh& a, const Mesh& b, BooleanOp op, Mesh& out, ElementId salt) {
    // An open surface has no inside, so "inside the other solid" is undefined.
    if (!isClosed(a) || !isClosed(b)) return false;

    const std::vector<Poly> pa = toPolygons(a, false);
    const std::vector<Poly> pb = toPolygons(b, true);
    if (pa.empty() || pb.empty()) return false;

    // Stated directly rather than as a sequence of inversions: each operation
    // is which side of each solid survives, and whether the second one is
    // turned inside out to become the wall of a cavity.
    std::vector<Poly> result;
    switch (op) {
    case BooleanOp::Union:
        classify(pa, pb, false, true,  result);
        classify(pb, pa, false, false, result);
        break;
    case BooleanOp::Intersection:
        classify(pa, pb, true, true,  result);
        classify(pb, pa, true, false, result);
        break;
    case BooleanOp::Difference: {
        // Neither keeps the shared skin. Where the two surfaces coincide and
        // face the same way, the material behind that patch belongs to both --
        // so subtracting takes it away, and the patch is not on the result at
        // all. Union and intersection both keep it, and take it from the first
        // solid so there is exactly one copy.
        classify(pa, pb, false, false, result);
        std::vector<Poly> inner;
        classify(pb, pa, true, false, inner);
        for (Poly& p : inner) { p.flip(); result.push_back(std::move(p)); }
        break;
    }
    }

    if (!weldAndBuild(result, a, b, salt, out)) return false;

    // A boolean cuts every face it touches into triangles and leaves a fan of
    // them where one flat surface used to be, plus a seam wherever the two
    // solids' surfaces met in the same plane. None of those edges are on the
    // model; they are only in the data, and picking one selects a sliver of
    // what the user sees as a face.
    mergeCoplanarFaces(out);
    return true;
}

} // namespace tg
