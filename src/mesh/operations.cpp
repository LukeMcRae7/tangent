#include "mesh/operations.h"

#include "mesh/boolean.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>
#include <limits>
#include <unordered_map>
#include <functional>
#include <utility>
#include <unordered_set>

namespace tg {
namespace {

// Polygon soup under construction, with the vertex compaction every operation
// needs at the end (faces get replaced, orphaning their old vertices).
struct Soup {
    static constexpr size_t kDropped = static_cast<size_t>(-1);

    std::vector<Vec3>     positions;
    std::vector<uint32_t> faceSizes;
    std::vector<uint32_t> faceIndices;

    // Stable names, parallel to the three arrays above. An operation that has
    // nothing to say about an element leaves kNoId there and the element comes
    // out unnamed, which a feature referring to it will notice.
    std::vector<ElementId> vertexNames;
    std::vector<ElementId> faceNames;

    uint32_t vertex(Vec3 p, ElementId id = kNoId) {
        positions.push_back(p);
        vertexNames.push_back(id);
        return static_cast<uint32_t>(positions.size() - 1);
    }
    void face(const std::vector<uint32_t>& loop, ElementId id = kNoId) {
        if (loop.size() < 3) return;
        faceSizes.push_back(static_cast<uint32_t>(loop.size()));
        faceIndices.insert(faceIndices.end(), loop.begin(), loop.end());
        faceNames.push_back(id);
    }

    // Drops vertices no face references and renumbers the rest.
    void compact() {
        std::vector<int32_t> remap(positions.size(), -1);
        std::vector<Vec3> kept;
        std::vector<ElementId> keptNames;
        kept.reserve(positions.size());
        keptNames.reserve(positions.size());
        for (uint32_t& idx : faceIndices) {
            if (remap[idx] < 0) {
                remap[idx] = static_cast<int32_t>(kept.size());
                kept.push_back(positions[idx]);
                keptNames.push_back(idx < vertexNames.size() ? vertexNames[idx] : kNoId);
            }
            idx = static_cast<uint32_t>(remap[idx]);
        }
        positions.swap(kept);
        vertexNames.swap(keptNames);
    }

    // Absorbs each listed face into the neighbour it is coplanar with.
    //
    // Where a fillet runs off the end of an edge its cap is flat, and it lies
    // in the plane of the face across the end -- so emitting it as its own
    // face draws a seam across a face the user never edited. Splicing it into
    // that face removes the seam and leaves the n-gon a B-rep would have had.
    //
    // Only a neighbour sharing exactly one edge will do. Two shared edges mean
    // splicing would leave a polygon that touches itself.
    void absorb(const std::vector<size_t>& faces, Real cosTol, bool chain = false) {
        if (faces.empty()) return;

        const size_t n = faceSizes.size();
        std::vector<size_t> off(n + 1, 0);
        for (size_t f = 0; f < n; ++f) off[f + 1] = off[f] + faceSizes[f];

        auto normalOf = [&](size_t f) {
            Vec3 a{};
            const size_t k = faceSizes[f];
            for (size_t i = 0; i < k; ++i)
                a += cross(positions[faceIndices[off[f] + i]],
                           positions[faceIndices[off[f] + (i + 1) % k]]);
            return lengthSq(a) > 1e-24 ? normalize(a) : Vec3{};
        };

        std::map<std::pair<uint32_t, uint32_t>, std::vector<size_t>> byEdge;
        for (size_t f = 0; f < n; ++f) {
            const size_t k = faceSizes[f];
            for (size_t i = 0; i < k; ++i) {
                const uint32_t u = faceIndices[off[f] + i];
                const uint32_t v = faceIndices[off[f] + (i + 1) % k];
                byEdge[{std::min(u, v), std::max(u, v)}].push_back(f);
            }
        }

        std::vector<std::vector<uint32_t>> loops(n);
        for (size_t f = 0; f < n; ++f)
            loops[f].assign(faceIndices.begin() + static_cast<long>(off[f]),
                            faceIndices.begin() + static_cast<long>(off[f + 1]));
        std::vector<bool> gone(n, false);
        std::vector<bool> listed(n, false);
        for (size_t f : faces) if (f < n) listed[f] = true;

        for (size_t f : faces) {
            if (f >= n || gone[f]) continue;
            const Vec3 nf = normalOf(f);
            if (lengthSq(nf) < 0.5) continue;

            // Count shared edges per neighbour.
            std::map<size_t, int> shared;
            const size_t k = faceSizes[f];
            for (size_t i = 0; i < k; ++i) {
                const uint32_t u = faceIndices[off[f] + i];
                const uint32_t v = faceIndices[off[f] + (i + 1) % k];
                for (size_t g : byEdge[{std::min(u, v), std::max(u, v)}])
                    if (g != f) ++shared[g];
            }

            for (const auto& [g, count] : shared) {
                if (count != 1 || gone[g]) continue;
                if (listed[g] && !chain) continue;
                if (dot(nf, normalOf(g)) < cosTol) continue;

                // The shared edge, as f walks it and as g walks it back.
                size_t fi = loops[f].size(), gi = loops[g].size();
                for (size_t i = 0; i < loops[f].size() && fi == loops[f].size(); ++i) {
                    const uint32_t u = loops[f][i];
                    const uint32_t v = loops[f][(i + 1) % loops[f].size()];
                    for (size_t j = 0; j < loops[g].size(); ++j)
                        if (loops[g][j] == v && loops[g][(j + 1) % loops[g].size()] == u) {
                            fi = i; gi = j; break;
                        }
                }
                if (fi == loops[f].size()) continue;

                std::vector<uint32_t> merged;
                const size_t fn = loops[f].size(), gn = loops[g].size();
                for (size_t i = 0; i <= gi; ++i) merged.push_back(loops[g][i]);
                for (size_t i = 2; i < fn; ++i) merged.push_back(loops[f][(fi + i) % fn]);
                for (size_t i = gi + 1; i < gn; ++i) merged.push_back(loops[g][i]);

                // Splicing can bring the two faces' copies of the same point
                // together. A loop that visits a vertex twice is not a face.
                std::vector<uint32_t> seenIdx = merged;
                std::sort(seenIdx.begin(), seenIdx.end());
                if (std::adjacent_find(seenIdx.begin(), seenIdx.end()) != seenIdx.end())
                    continue;

                loops[g].swap(merged);
                gone[f] = true;
                break;
            }
        }

        std::vector<uint32_t> sizes, indices;
        std::vector<ElementId> fnames;
        for (size_t f = 0; f < n; ++f) {
            if (gone[f]) continue;
            sizes.push_back(static_cast<uint32_t>(loops[f].size()));
            indices.insert(indices.end(), loops[f].begin(), loops[f].end());
            fnames.push_back(f < faceNames.size() ? faceNames[f] : kNoId);
        }
        faceSizes.swap(sizes);
        faceIndices.swap(indices);
        faceNames.swap(fnames);
    }

    // Merges vertices that landed in the same place and drops the faces that
    // leaves with no area.
    //
    // A fillet produces these legitimately. Two filleted edges that run
    // straight through a vertex -- the seam an extrude leaves behind, say --
    // have nothing to blend there: both sections land on the same points, and
    // the corner patch between them is a polygon of zero width. Dropping that
    // patch alone would open a hole, because the two edge strips reach it
    // through separate vertices in the same position. Welding first is what
    // lets the strips meet each other directly instead.
    void weld(Real eps, std::vector<size_t>* faceRemap = nullptr) {
        const Real inv = 1.0 / eps;
        struct Key { int64_t x, y, z; bool operator==(const Key& o) const {
            return x == o.x && y == o.y && z == o.z; } };
        struct Hash { size_t operator()(const Key& k) const {
            return std::hash<int64_t>{}(k.x * 73856093LL ^ k.y * 19349663LL ^ k.z * 83492791LL); } };

        std::unordered_map<Key, uint32_t, Hash> seen;
        std::vector<uint32_t> remap(positions.size());
        for (uint32_t i = 0; i < positions.size(); ++i) {
            const Vec3& p = positions[i];
            const Key k{static_cast<int64_t>(std::llround(p.x * inv)),
                        static_cast<int64_t>(std::llround(p.y * inv)),
                        static_cast<int64_t>(std::llround(p.z * inv))};
            // Probing the neighbours keeps two points either side of a cell
            // boundary together, which quantising alone does not.
            uint32_t hit = ~0u;
            for (int dx = -1; dx <= 1 && hit == ~0u; ++dx)
                for (int dy = -1; dy <= 1 && hit == ~0u; ++dy)
                    for (int dz = -1; dz <= 1 && hit == ~0u; ++dz) {
                        auto it = seen.find({k.x + dx, k.y + dy, k.z + dz});
                        if (it != seen.end() &&
                            lengthSq(positions[it->second] - p) < eps * eps)
                            hit = it->second;
                    }
            if (hit == ~0u) { seen.emplace(k, i); hit = i; }
            remap[i] = hit;
        }

        // Two points in the same place are one vertex, so they must settle on
        // one name. The smaller wins, which makes the choice independent of the
        // order they happened to be emitted in.
        for (uint32_t i = 0; i < positions.size(); ++i) {
            if (remap[i] == i) continue;
            ElementId& keep = vertexNames[remap[i]];
            const ElementId other = vertexNames[i];
            if (keep == kNoId) keep = other;
            else if (other != kNoId && other < keep) keep = other;
        }

        std::vector<uint32_t> sizes, indices, loop;
        std::vector<ElementId> fnames;
        if (faceRemap) faceRemap->assign(faceSizes.size(), kDropped);
        size_t at = 0;
        size_t which = 0;
        for (uint32_t n : faceSizes) {
            const size_t self = which++;
            loop.clear();
            for (uint32_t i = 0; i < n; ++i) {
                const uint32_t idx = remap[faceIndices[at + i]];
                if (loop.empty() || loop.back() != idx) loop.push_back(idx);
            }
            at += n;
            while (loop.size() > 1 && loop.front() == loop.back()) loop.pop_back();
            if (loop.size() < 3) continue;

            // Collinear-but-distinct points still leave no area.
            Vec3 area{};
            for (size_t i = 0; i < loop.size(); ++i)
                area += cross(positions[loop[i]], positions[loop[(i + 1) % loop.size()]]);
            if (lengthSq(area) < 1e-18) continue;

            if (faceRemap) (*faceRemap)[self] = sizes.size();
            sizes.push_back(static_cast<uint32_t>(loop.size()));
            indices.insert(indices.end(), loop.begin(), loop.end());
            fnames.push_back(self < faceNames.size() ? faceNames[self] : kNoId);
        }
        faceSizes.swap(sizes);
        faceIndices.swap(indices);
        faceNames.swap(fnames);
    }

    bool commit(Mesh& mesh) {
        compact();
        Mesh::Names names;
        names.vertices = vertexNames;
        names.faces    = faceNames;
        Mesh next;
        if (!next.build(positions, faceSizes, faceIndices, &names)) return false;
        mesh = std::move(next);
        return true;
    }
};

std::vector<uint32_t> faceLoop(const Mesh& m, Index f) {
    std::vector<Index> verts;
    m.faceVertices(f, verts);
    std::vector<uint32_t> out;
    out.reserve(verts.size());
    for (Index v : verts) out.push_back(static_cast<uint32_t>(v));
    return out;
}

// In-plane offset of a face's corners, each edge moved inward by `amount`.
//
// For corner i the offset lands where the two neighbouring offset edges meet.
// With n0 and n1 the inward edge normals, the displacement d must satisfy
// dot(d,n0) = dot(d,n1) = amount, which solves exactly to
// amount * (n0 + n1) / (1 + dot(n0, n1)). Moving corners toward the centroid
// instead would only be correct for regular polygons.
bool insetPolygon(const std::vector<Vec3>& poly, Vec3 normal, Real amount,
                  std::vector<Vec3>& out) {
    const size_t n = poly.size();
    if (n < 3) return false;
    out.assign(n, Vec3{});

    for (size_t i = 0; i < n; ++i) {
        const Vec3& prev = poly[(i + n - 1) % n];
        const Vec3& cur  = poly[i];
        const Vec3& next = poly[(i + 1) % n];

        const Vec3 ePrev = cur - prev;
        const Vec3 eNext = next - cur;
        if (lengthSq(ePrev) < 1e-12f || lengthSq(eNext) < 1e-12f) return false;

        // Inward normal of an edge is cross(faceNormal, edge).
        const Vec3 n0 = normalize(cross(normal, ePrev));
        const Vec3 n1 = normalize(cross(normal, eNext));

        const Real denom = 1.0f + dot(n0, n1);
        if (denom < 1e-4f) return false;    // edges double back; no finite offset
        out[i] = cur + (n0 + n1) * (amount / denom);
    }
    return true;
}

// Signed area of a polygon about its own normal.
Real signedArea(const std::vector<Vec3>& poly, Vec3 normal) {
    Vec3 acc{};
    const size_t n = poly.size();
    for (size_t i = 0; i < n; ++i) acc += cross(poly[i], poly[(i + 1) % n]);
    return dot(acc, normal) * 0.5f;
}

// Has the inset overshot the polygon's interior?
//
// Signed area alone does NOT answer this. Offsetting a square past its centre
// shrinks it to a point and then re-expands it -- congruent, and still wound
// counter-clockwise, so the area comes back positive and a sign test happily
// accepts a face that has turned itself inside out. What actually changes is
// that each edge reverses direction, so that is what we check, along with the
// area to catch the degenerate moment at the centre.
bool insetIsValid(const std::vector<Vec3>& orig, const std::vector<Vec3>& inset,
                  Vec3 normal) {
    const size_t n = orig.size();
    if (inset.size() != n) return false;
    if (signedArea(inset, normal) <= 1e-6f) return false;

    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        const Vec3 before = orig[j] - orig[i];
        const Vec3 after  = inset[j] - inset[i];
        if (lengthSq(after) < 1e-12f) return false;
        if (dot(normalize(before), normalize(after)) <= 0.0f) return false;
    }
    return true;
}

} // namespace

// Signed volume, for telling a solid from something turned inside out.
static Real checkVolume(const Mesh& m) {
    RenderMesh rm;
    m.buildRenderMesh(rm);
    Real s6 = 0.0;
    for (size_t i = 0; i < rm.triangles.size(); i += 3)
        s6 += dot(rm.positions[rm.triangles[i]],
                  cross(rm.positions[rm.triangles[i + 1]], rm.positions[rm.triangles[i + 2]]));
    return s6 / 6.0;
}

// The solid a face region sweeps out as it travels `offset`.
//
// Bottom cap the region as it stands, top cap the region moved, walls between.
// Wound so the bottom faces back down the sweep and the top faces along it,
// which makes it a closed solid whichever way the offset points.
static bool sweptSolid(const Mesh& mesh, const std::unordered_set<Index>& region,
                       Vec3 offset, ElementId salt, Mesh& out) {
    Soup soup;
    std::unordered_map<Index, uint32_t> low, high;
    std::vector<Index> fv;

    auto lowOf = [&](Index v) {
        auto it = low.find(v);
        if (it != low.end()) return it->second;
        const uint32_t idx = soup.vertex(mesh.verts[v].position, mesh.verts[v].id);
        low.emplace(v, idx);
        return idx;
    };
    auto highOf = [&](Index v) {
        auto it = high.find(v);
        if (it != high.end()) return it->second;
        const uint32_t idx = soup.vertex(mesh.verts[v].position + offset,
                                         nameId(salt, IdRole::Top, mesh.verts[v].id));
        high.emplace(v, idx);
        return idx;
    };

    // Sweeping against the region's own normal produces the same solid wound
    // inside out, so the caps and walls are emitted the other way round.
    Vec3 avg{};
    for (Index f : region) avg += mesh.faceNormal(f) * mesh.faceArea(f);
    const bool forward = dot(avg, offset) > 0.0;

    for (Index f : region) {
        mesh.faceVertices(f, fv);
        std::vector<uint32_t> bottom, top;
        for (Index v : fv) top.push_back(highOf(v));
        for (size_t i = fv.size(); i-- > 0;) bottom.push_back(lowOf(fv[i]));
        if (!forward) { std::reverse(bottom.begin(), bottom.end());
                        std::reverse(top.begin(), top.end()); }
        // The cap that ends up on the model is the face the user picked, moved,
        // so it carries that face's name onward. The other cap is consumed by
        // the combine and only needs to be distinct.
        soup.face(bottom, nameId(salt, IdRole::Cap, mesh.faces[f].id, 0));
        soup.face(top, mesh.faces[f].id);
    }

    for (Index f : region) {
        const Index start = mesh.faces[f].halfedge;
        Index h = start;
        do {
            const Index twinFace = mesh.halfedges[mesh.halfedges[h].twin].face;
            if (twinFace == kInvalid || !region.count(twinFace)) {
                const Index v0 = mesh.fromVertex(h), v1 = mesh.halfedges[h].vertex;
                std::vector<uint32_t> wall{lowOf(v0), lowOf(v1), highOf(v1), highOf(v0)};
                if (!forward) std::reverse(wall.begin(), wall.end());
                soup.face(wall, nameId(salt, IdRole::Wall, mesh.edgeId(h)));
            }
            h = mesh.halfedges[h].next;
        } while (h != start);
    }

    Mesh built;
    if (!soup.commit(built)) return false;
    // Wound for a forward sweep; a backward one comes out inside out.
    if (built.faceCount() == 0) return false;
    out = std::move(built);
    return true;
}


// ---------------------------------------------------------------------------
bool extrudeFaces(Mesh& mesh, const std::vector<Index>& faces, Real distance,
                  std::vector<Index>* newFaces, ElementId salt) {
    if (faces.empty() || mesh.empty()) return false;

    std::unordered_set<Index> region(faces.begin(), faces.end());
    for (Index f : region)
        if (f < 0 || f >= mesh.faceCount()) return false;

    // Area weighting keeps a large face from being outvoted by a sliver.
    Vec3 dir{};
    for (Index f : region) dir += mesh.faceNormal(f) * mesh.faceArea(f);
    if (lengthSq(dir) < 1e-12f) return false;
    dir = normalize(dir);
    const Vec3 offset = dir * distance;

    // An extrude is the solid the face sweeps out, combined with the body.
    //
    // Lifting the face and stitching walls onto it is only the same thing while
    // the sweep meets nothing. Push a face into the body and the walls fold back
    // through it, leaving a shell inside the solid rather than the face simply
    // moving back. Push one into another part of the same body and the two
    // surfaces pass through each other instead of joining. Sweeping and then
    // combining has neither problem: material added is a union, material removed
    // is a difference, and anything the sweep runs into is resolved rather than
    // ignored.
    //
    // A zero-length extrude has no solid to sweep and keeps the old path: it is
    // the deliberate one, duplicating a face without moving it.
    const Real scale = std::max(length(mesh.bounds().size()), Real(1.0));

    // A zero-length extrude leaves the mesh alone and simply reports the face
    // back, so pressing extrude and then dragging works from the face already
    // there.
    //
    // It used to build the extrusion anyway, which at zero length meant four
    // walls of no area wrapped around a face in its original position: geometry
    // that is not on the model, cannot be selected, and a slicer will reject.
    // Doing it twice stacked another four. Reporting the face and stopping gives
    // the same behaviour to drag against with none of that.
    if (std::fabs(distance) <= scale * 1e-9) {
        if (newFaces) *newFaces = faces;
        return true;
    }

    // Combining only means anything for a closed solid. An open surface -- a
    // plane, a sheet -- has no inside for a union or a difference to be about,
    // so it is lifted and walled directly, which is all an extrude of a sheet
    // is anyway.
    bool closed = true;
    for (Index h = 0; h < mesh.halfedgeCount() && closed; ++h)
        if (mesh.halfedges[h].face == kInvalid) closed = false;

    Mesh sweep;
    if (closed && std::fabs(distance) > scale * 1e-9 &&
        sweptSolid(mesh, region, offset, salt, sweep)) {
        Mesh combined;
        const bool combinedOk =
            meshBoolean(mesh, sweep,
                        distance > 0 ? BooleanOp::Union : BooleanOp::Difference,
                        combined, nameId(salt, IdRole::Wall, 0), true);

        // Cutting away at least as much as there was leaves nothing to model,
        // and a face driven clean through the far side is that. Refuse it --
        // the old path answered with a shell turned inside out, which is not a
        // solid and is exactly what should never be produced.
        if (!combinedOk || combined.empty() || checkVolume(combined) <= 0.0)
            return false;

        {

            if (newFaces) {
                // The face the user goes on to drag: whatever now lies in the
                // plane the sweep ended at, facing the way it swept.
                newFaces->clear();
                const Vec3 target = mesh.faceCentroid(*region.begin()) + offset;
                for (Index f = 0; f < combined.faceCount(); ++f) {
                    if (dot(combined.faceNormal(f), dir) < 0.999) continue;
                    if (std::fabs(dot(combined.faceCentroid(f) - target, dir)) > scale * 1e-7)
                        continue;
                    newFaces->push_back(f);
                }
            }
            mesh = std::move(combined);
            return true;
        }
    }

    Soup soup;
    soup.positions.reserve(mesh.verts.size() * 2);
    for (const MeshVertex& v : mesh.verts) soup.vertex(v.position, v.id);

    // One raised copy per vertex touched by the region. Vertices used *only*
    // by region faces are left behind and compaction removes them.
    std::unordered_map<Index, uint32_t> raised;
    for (Index f : region) {
        std::vector<Index> verts;
        mesh.faceVertices(f, verts);
        for (Index v : verts)
            if (!raised.count(v))
                raised[v] = soup.vertex(mesh.verts[v].position + offset,
                                        nameId(salt, IdRole::Top, mesh.verts[v].id));
    }

    // Faces outside the region are carried over untouched.
    for (Index f = 0; f < mesh.faceCount(); ++f)
        if (!region.count(f)) soup.face(faceLoop(mesh, f), mesh.faces[f].id);

    const size_t firstMoved = soup.faceSizes.size();

    // The region itself, lifted.
    for (Index f : faces) {
        std::vector<Index> verts;
        mesh.faceVertices(f, verts);
        std::vector<uint32_t> loop;
        loop.reserve(verts.size());
        for (Index v : verts) loop.push_back(raised[v]);
        // The same face, moved: it keeps its name.
        soup.face(loop, mesh.faces[f].id);
    }

    // Side walls along the region's boundary. For a half-edge v0->v1 whose
    // twin lies outside the region, the wall (v0, v1, v1', v0') is wound so
    // its normal points away from the solid.
    for (Index f : region) {
        const Index start = mesh.faces[f].halfedge;
        Index h = start;
        do {
            const Index twinFace = mesh.halfedges[mesh.halfedges[h].twin].face;
            if (twinFace == kInvalid || !region.count(twinFace)) {
                const Index v0 = mesh.fromVertex(h);
                const Index v1 = mesh.halfedges[h].vertex;
                soup.face({static_cast<uint32_t>(v0), static_cast<uint32_t>(v1),
                           raised[v1], raised[v0]},
                          nameId(salt, IdRole::Wall, mesh.edgeId(h)));
            }
            h = mesh.halfedges[h].next;
        } while (h != start);
    }

    if (!soup.commit(mesh)) return false;

    if (newFaces) {
        newFaces->clear();
        for (size_t i = 0; i < faces.size(); ++i)
            newFaces->push_back(static_cast<Index>(firstMoved + i));
    }
    return true;
}

// ---------------------------------------------------------------------------
bool insetFaces(Mesh& mesh, const std::vector<Index>& faces, Real amount,
                std::vector<Index>* newFaces, ElementId salt) {
    if (faces.empty() || mesh.empty() || amount <= 0.0f) return false;

    std::unordered_set<Index> region(faces.begin(), faces.end());
    for (Index f : region)
        if (f < 0 || f >= mesh.faceCount()) return false;

    Soup soup;
    for (const MeshVertex& v : mesh.verts) soup.vertex(v.position, v.id);

    for (Index f = 0; f < mesh.faceCount(); ++f)
        if (!region.count(f)) soup.face(faceLoop(mesh, f), mesh.faces[f].id);

    // Each inset face emits its inner face followed by one rim quad per edge,
    // so the inner faces are not contiguous. Record where each one lands as we
    // go: after commit() the original mesh is gone and its degrees with it.
    std::vector<size_t> innerAt;
    innerAt.reserve(faces.size());

    for (Index f : faces) {
        std::vector<Index> verts;
        mesh.faceVertices(f, verts);

        std::vector<Vec3> poly;
        poly.reserve(verts.size());
        for (Index v : verts) poly.push_back(mesh.verts[v].position);

        const Vec3 nrm = mesh.faceNormal(f);
        std::vector<Vec3> inner;
        if (!insetPolygon(poly, nrm, amount, inner)) return false;
        if (!insetIsValid(poly, inner, nrm)) return false;

        std::vector<uint32_t> innerIdx;
        innerIdx.reserve(inner.size());
        for (size_t i = 0; i < inner.size(); ++i)
            innerIdx.push_back(soup.vertex(
                inner[i], nameId(salt, IdRole::Top, mesh.verts[verts[i]].id)));

        innerAt.push_back(soup.faceSizes.size());
        // The shrunk face is the same face: it keeps its name.
        soup.face(innerIdx, mesh.faces[f].id);

        // Rim: (v_i, v_i+1, inner_i+1, inner_i) carries the face's own normal.
        // Each rim quad belongs to the edge it was raised from.
        const size_t n = verts.size();
        Index h = mesh.faces[f].halfedge;
        for (size_t i = 0; i < n; ++i) {
            const size_t j = (i + 1) % n;
            soup.face({static_cast<uint32_t>(verts[i]), static_cast<uint32_t>(verts[j]),
                       innerIdx[j], innerIdx[i]},
                      nameId(salt, IdRole::Wall, mesh.edgeId(h)));
            h = mesh.halfedges[h].next;
        }
    }

    if (!soup.commit(mesh)) return false;

    if (newFaces) {
        newFaces->clear();
        for (size_t at : innerAt) newFaces->push_back(static_cast<Index>(at));
    }
    return true;
}

// ---------------------------------------------------------------------------
bool moveFaces(Mesh& mesh, const std::vector<Index>& faces, Vec3 delta) {
    if (faces.empty() || mesh.empty()) return false;

    std::unordered_set<Index> touched;
    for (Index f : faces) {
        if (f < 0 || f >= mesh.faceCount()) return false;
        std::vector<Index> verts;
        mesh.faceVertices(f, verts);
        touched.insert(verts.begin(), verts.end());
    }

    // Topology is untouched, so the half-edge links stay valid and only the
    // positions move.
    for (Index v : touched) mesh.verts[v].position += delta;
    return true;
}

// ---------------------------------------------------------------------------
size_t splitShells(const Mesh& mesh, std::vector<Mesh>& out) {
    out.clear();
    if (mesh.empty()) return 0;

    // Flood fill across shared edges to label each face with its body.
    std::vector<int> shell(static_cast<size_t>(mesh.faceCount()), -1);
    std::vector<Index> stack;
    int count = 0;

    for (Index f = 0; f < mesh.faceCount(); ++f) {
        if (shell[f] >= 0) continue;
        const int id = count++;
        stack.push_back(f);
        shell[f] = id;
        while (!stack.empty()) {
            const Index cur = stack.back();
            stack.pop_back();
            const Index start = mesh.faces[cur].halfedge;
            Index he = start;
            do {
                const Index nf = mesh.halfedges[mesh.halfedges[he].twin].face;
                if (nf != kInvalid && shell[nf] < 0) { shell[nf] = id; stack.push_back(nf); }
                he = mesh.halfedges[he].next;
            } while (he != start);
        }
    }

    for (int id = 0; id < count; ++id) {
        Soup soup;
        // Vertices are re-indexed per body; Soup::compact drops the rest.
        soup.positions.reserve(mesh.verts.size());
        for (const MeshVertex& v : mesh.verts) soup.vertex(v.position, v.id);
        for (Index f = 0; f < mesh.faceCount(); ++f)
            if (shell[f] == id) soup.face(faceLoop(mesh, f));

        Mesh piece;
        if (soup.commit(piece)) out.push_back(std::move(piece));
    }

    // Largest first: the biggest body is the one the user thinks of as "the
    // part", so it should keep the original object.
    std::sort(out.begin(), out.end(),
              [](const Mesh& a, const Mesh& b) { return a.faceCount() > b.faceCount(); });
    return out.size();
}

// ---------------------------------------------------------------------------
Real maxBevelWidth(const Mesh& mesh) {
    // The binding constraint is the face that runs out of room first. Bisect
    // on "does every face still inset to a positive area", which is monotone.
    Real lo = 0.0f, hi = 0.0f;
    for (Index f = 0; f < mesh.faceCount(); ++f) hi = std::max(hi, mesh.faceArea(f));
    hi = std::sqrt(std::max(hi, Real(1e-6)));

    auto fits = [&](Real w) {
        for (Index f = 0; f < mesh.faceCount(); ++f) {
            std::vector<Index> verts;
            mesh.faceVertices(f, verts);
            std::vector<Vec3> poly;
            for (Index v : verts) poly.push_back(mesh.verts[v].position);
            const Vec3 nrm = mesh.faceNormal(f);
            std::vector<Vec3> inner;
            if (!insetPolygon(poly, nrm, w, inner)) return false;
            if (!insetIsValid(poly, inner, nrm)) return false;
        }
        return true;
    };

    if (!fits(hi)) {
        for (int i = 0; i < 40; ++i) {
            const Real mid = (lo + hi) * 0.5f;
            if (fits(mid)) lo = mid; else hi = mid;
        }
        return lo;
    }
    return hi;
}

// ---------------------------------------------------------------------------
namespace {

// Canonical name for an edge: the lower of its two half-edge indices.
inline Index edgeOf(const Mesh& m, Index h) { return std::min(h, m.halfedges[h].twin); }

// Where a face's corner moves when one or both of its edges there is pulled
// back. Generalises insetPolygon to unequal offsets, which is what lets a
// single edge be beveled without disturbing the rest of the face.
//
// Writing n0 and n1 for the inward normals of the two edges meeting at the
// corner, we want the displacement d with dot(d,n0) = a0 and dot(d,n1) = a1.
// In the basis {n0, n1} that is a 2x2 solve.
bool cornerOffset(Vec3 normal, Vec3 ePrev, Vec3 eNext, Real a0, Real a1, Vec3& out) {
    if (lengthSq(ePrev) < 1e-18 || lengthSq(eNext) < 1e-18) return false;

    const Vec3 n0 = normalize(cross(normal, ePrev));
    const Vec3 n1 = normalize(cross(normal, eNext));
    const Real c = dot(n0, n1);

    if (c > 1.0 - 1e-12) {
        // Collinear edges: one constraint, and it is only satisfiable if both
        // ask for the same offset.
        if (std::fabs(a0 - a1) > 1e-9) return false;
        out = n0 * a0;
        return true;
    }
    if (c < -1.0 + 1e-12) return false;   // edges double back; no finite corner

    const Real denom = 1.0 - c * c;
    const Real alpha = (a0 - c * a1) / denom;
    const Real beta  = (a1 - c * a0) / denom;
    out = n0 * alpha + n1 * beta;
    return true;
}

// The circular arc bridging one beveled edge at one of its ends.
//
// The centre is the point equidistant from both face planes, found by walking
// from p0 along face 0's inward normal until the distance to face 1's plane
// matches how far we have walked. Solving it this way rather than from the
// dihedral angle keeps it correct for concave edges, where the centre lands on
// the other side and the arc bulges into the material instead of out of it.
struct Arc {
    Vec3 centre;
    bool circular = false;   // false when the faces are parallel; then it is a chord
};

Arc solveArc(Vec3 p0, Vec3 p1, Vec3 inward0, Vec3 inward1) {
    Arc a;
    const Real denom = 1.0 - dot(inward0, inward1);
    if (std::fabs(denom) < 1e-9) return a;          // parallel faces: flat edge

    const Real t = dot(p0 - p1, inward1) / denom;
    if (!std::isfinite(t) || std::fabs(t) < 1e-12) return a;

    a.centre = p0 + inward0 * t;
    a.circular = true;
    return a;
}

// Points along the arc from p0 to p1, inclusive, in equal angular steps.
// Falls back to a straight chord when there is no well-defined centre.
void sampleArc(const Arc& arc, Vec3 p0, Vec3 p1, Vec3 axis, int segments,
               std::vector<Vec3>& out) {
    out.clear();
    out.push_back(p0);

    if (!arc.circular || segments < 2) {
        for (int s = 1; s < segments; ++s)
            out.push_back(lerp(p0, p1, static_cast<Real>(s) / segments));
        out.push_back(p1);
        return;
    }

    const Vec3 u = p0 - arc.centre;
    const Vec3 v = p1 - arc.centre;
    const Real r0 = length(u), r1 = length(v);

    // Signed sweep about the edge, so the arc turns the short way round the
    // material rather than the long way round the outside.
    const Real sweep = std::atan2(dot(cross(u, v), axis), dot(u, v));

    for (int s = 1; s < segments; ++s) {
        const Real f = static_cast<Real>(s) / segments;
        const Quat q = Quat::fromAxisAngle(axis, sweep * f);
        Vec3 dir = rotate(q, u);
        const Real len = length(dir);
        if (len < 1e-12) { out.push_back(lerp(p0, p1, f)); continue; }
        // Radii should match; interpolating guards against small asymmetry in
        // the two offset corners.
        out.push_back(arc.centre + dir * (lerpf(r0, r1, f) / len));
    }
    out.push_back(p1);
}

} // namespace

bool filletEdges(Mesh& mesh, const FilletSpec& spec) {
    const int segments = spec.segments;
    const ElementId salt = spec.salt;
    const bool dbg = std::getenv("TANGENT_BEVEL_DEBUG") != nullptr;
    auto bail = [&](const char* why) { if (dbg) std::fprintf(stderr, "[bevel] %s\n", why); return false; };
    if (mesh.empty() || spec.edges.empty() || segments < 1) return bail("bad arguments");

    const Index heCount = mesh.halfedgeCount();

    // An open boundary has no second face to bevel against.
    for (Index h = 0; h < heCount; ++h)
        if (mesh.halfedges[h].face == kInvalid) return bail("open surface");

    // Radius per half-edge, shared with its twin. Naming an edge twice with
    // different radii takes the larger, which is the one the geometry has to
    // clear.
    std::vector<bool> beveled(static_cast<size_t>(heCount), false);
    std::vector<Real> radiusOf(static_cast<size_t>(heCount), 0.0);
    size_t chosen = 0;
    for (const FilletEdge& fe : spec.edges) {
        const Index e = fe.edge;
        if (e < 0 || e >= heCount) return bail("edge index out of range");
        if (fe.radius <= 0.0) return bail("radius must be positive");
        const Index tw = mesh.halfedges[e].twin;
        if (!beveled[e]) ++chosen;
        beveled[e] = beveled[tw] = true;
        radiusOf[e] = radiusOf[tw] = std::max(radiusOf[e], fe.radius);
    }
    if (chosen == 0) return bail("no edges chosen");

    // Scaled to the model, not fixed. Sections that should coincide are
    // computed two ways -- offset from a face, and cut back along an edge --
    // and agree to within rounding, which on a 20mm part is a few nanometres
    // and on a 2m one is a few microns. An absolute tolerance is right for
    // exactly one model size; this is right for all of them, and still a
    // thousand times finer than anything a printer resolves.
    const Real weldEps = std::max(length(mesh.bounds().size()), Real(1e-3)) * 1e-9;

    std::vector<Vec3> faceNormals(static_cast<size_t>(mesh.faceCount()));
    for (Index f = 0; f < mesh.faceCount(); ++f) faceNormals[f] = mesh.faceNormal(f);

    // An edge whose two faces are coplanar is not an edge the surface turns at,
    // and there is nothing there to round: the ball touches both faces in the
    // same place, so the section has zero width. Drop those rather than emit
    // slivers -- Fusion likewise refuses to fillet a tangent edge. It matters
    // for whole-part rounding, where a model carries seams left by an extrude
    // that the user does not think of as edges at all.
    constexpr Real kFlatCos = 0.9999619;   // half a degree
    for (Index h = 0; h < heCount; ++h) {
        if (!beveled[h]) continue;
        const Index tw = mesh.halfedges[h].twin;
        if (dot(faceNormals[mesh.halfedges[h].face],
                faceNormals[mesh.halfedges[tw].face]) < kFlatCos) continue;
        beveled[h] = beveled[tw] = false;
        radiusOf[h] = radiusOf[tw] = 0.0;
        --chosen;
    }
    if (chosen == 0) return bail("every chosen edge is flat");

    auto posOf = [&](Index v) { return mesh.verts[v].position; };
    auto dirOf = [&](Index h) {
        return posOf(mesh.halfedges[h].vertex) - posOf(mesh.fromVertex(h));
    };

    // How many filleted edges arrive at each vertex. Two or more means the
    // rolling ball has to pivot there, and a blend surface exists; exactly one
    // means it rolls straight off the end and there is nothing to blend.
    std::vector<int> filletsAt(static_cast<size_t>(mesh.vertexCount()), 0);
    for (Index h = 0; h < heCount; ++h) {
        if (!beveled[h] || h > mesh.halfedges[h].twin) continue;
        ++filletsAt[mesh.fromVertex(h)];
        ++filletsAt[mesh.halfedges[h].vertex];
    }
    auto blendsAt = [&](Index v) { return filletsAt[v] >= 2; };

    // Unfilleted edges arriving at each vertex. One or more means the corner is
    // a mitre rather than a ball; see the patch loop.
    std::vector<int> sharpAt(static_cast<size_t>(mesh.vertexCount()), 0);
    for (Index h = 0; h < heCount; ++h) {
        if (beveled[h] || h > mesh.halfedges[h].twin) continue;
        ++sharpAt[mesh.fromVertex(h)];
        ++sharpAt[mesh.halfedges[h].vertex];
    }

    // The ball that sits in the corner at each vertex.
    //
    // This is the whole of the corner model. A fillet is the surface of a ball
    // rolled along an edge, so at a vertex the ball settles into the corner:
    // tangent to every face that a filleted edge runs along, and as close to
    // the vertex as those tangencies allow. Where it touches each of those
    // faces is where that face's boundary turns, and the ball's surface between
    // those points is the corner blend.
    //
    // Everything else follows from it. One fillet arriving gives two tangent
    // planes, and the ball slides freely along the edge to sit at the vertex --
    // no setback, a flat cap. Three at a cube corner give one point, and the
    // blend is a spherical triangle. Two filleted edges and a sharp one give
    // the same ball, tangent to all three faces, and the sharp edge is cut back
    // to where its two faces have been pulled to -- a pinch, exactly as Fusion
    // leaves the vertical edge of a filleted box rim.
    //
    // Critically, the setback is *solved* rather than assumed to be the radius.
    // It equals the radius only where the faces meet at right angles. Between
    // two facets of a cylinder wall, 11 degrees apart, it is almost nothing --
    // and taking the radius there pulls both ends of a 2mm facet 2mm inward and
    // collapses it, which is why a rim fillet on a curved surface used to fail.
    //
    // Solved as a least-squares fit: find u minimising sum over the tangent
    // faces of (n_i . u + r_i)^2, which is (sum n_i n_i^T) u = -sum r_i n_i.
    // The vertex lies on every one of those planes, so u is the offset from it.
    // Three independent planes make this exact; more are averaged, which is
    // what a faceted surface needs; two leave a free direction, and a small
    // regularisation picks the point nearest the vertex, which is the right
    // answer for a fillet running off the end of an edge.
    std::vector<Vec3> ballAt(static_cast<size_t>(mesh.vertexCount()));
    std::vector<bool> ballOk(static_cast<size_t>(mesh.vertexCount()), false);

    // Radius the corner ball uses against the face on the far side of `h` --
    // the larger of the filleted edges bounding that face at this vertex.
    auto faceRadiusAt = [&](Index h) {
        const Index p = mesh.halfedges[h].prev;
        Real r = 0.0;
        if (beveled[h]) r = std::max(r, radiusOf[h]);
        if (beveled[p]) r = std::max(r, radiusOf[p]);
        return r;
    };

    for (Index v = 0; v < mesh.vertexCount(); ++v) {
        if (filletsAt[v] == 0) continue;

        // The distinct planes at this vertex. Two faces with the same normal
        // are the same plane -- every face here passes through v -- and
        // counting one twice only makes the fit worse.
        Vec3 nrm[16];
        Real rad[16];
        int planes = 0;

        const Index start = mesh.verts[v].halfedge;
        Index h = start;
        do {
            const Real r = faceRadiusAt(h);
            if (r > 0.0) {
                const Vec3 n = faceNormals[mesh.halfedges[h].face];
                int at = -1;
                for (int i = 0; i < planes; ++i)
                    if (dot(nrm[i], n) > 1.0 - 1e-12) { at = i; break; }
                if (at >= 0) rad[at] = std::max(rad[at], r);
                else if (planes < 16) { nrm[planes] = n; rad[planes] = r; ++planes; }
            }
            h = mesh.halfedges[mesh.halfedges[h].twin].next;
        } while (h != start);

        if (planes < 2) continue;

        Vec3 u{};
        bool solved = false;

        if (planes >= 3) {
            // Over- or exactly-determined: least squares on the offsets.
            Real m[6] = {0, 0, 0, 0, 0, 0};   // xx xy xz yy yz zz
            Vec3 rhs{};
            for (int i = 0; i < planes; ++i) {
                const Vec3& n = nrm[i];
                m[0] += n.x * n.x; m[1] += n.x * n.y; m[2] += n.x * n.z;
                m[3] += n.y * n.y; m[4] += n.y * n.z; m[5] += n.z * n.z;
                rhs -= n * rad[i];
            }
            const Real c00 = m[3] * m[5] - m[4] * m[4];
            const Real c01 = m[2] * m[4] - m[1] * m[5];
            const Real c02 = m[1] * m[4] - m[2] * m[3];
            const Real det = m[0] * c00 + m[1] * c01 + m[2] * c02;
            // Scale-free rank test: a flat or nearly flat fan of normals spans
            // a plane, not space, and must be solved in that plane instead.
            const Real trace = m[0] + m[3] + m[5];
            if (std::fabs(det) > 1e-10 * trace * trace * trace) {
                const Real c11 = m[0] * m[5] - m[2] * m[2];
                const Real c12 = m[1] * m[2] - m[0] * m[4];
                const Real c22 = m[0] * m[3] - m[1] * m[1];
                u = Vec3{(c00 * rhs.x + c01 * rhs.y + c02 * rhs.z) / det,
                         (c01 * rhs.x + c11 * rhs.y + c12 * rhs.z) / det,
                         (c02 * rhs.x + c12 * rhs.y + c22 * rhs.z) / det};
                solved = true;
            }
        }

        if (!solved) {
            // Under-determined: the constraints leave a free direction, and the
            // answer is the smallest offset that satisfies them -- which is the
            // one lying in the span of the normals. Solving that directly is
            // exact. Nudging the singular system with a small diagonal instead,
            // as this used to, leaves a determinant near zero and throws away
            // most of the mantissa: it put points that should have coincided
            // microns apart, which is a crack, not a rounding difference.
            //
            // Pick the two least parallel planes; a third would be dependent.
            int p0 = 0, p1 = 1;
            Real worst = 2.0;
            for (int i = 0; i < planes; ++i)
                for (int j = i + 1; j < planes; ++j) {
                    const Real c = std::fabs(dot(nrm[i], nrm[j]));
                    if (c < worst) { worst = c; p0 = i; p1 = j; }
                }
            const Real c = dot(nrm[p0], nrm[p1]);
            const Real det = 1.0 - c * c;
            if (det < 1e-12) continue;   // parallel: no corner to speak of
            const Real l0 = (-rad[p0] + c * rad[p1]) / det;
            const Real l1 = (-rad[p1] + c * rad[p0]) / det;
            u = nrm[p0] * l0 + nrm[p1] * l1;
        }

        ballAt[v] = posOf(v) + u;
        ballOk[v] = true;
    }

    // Where a fillet runs off the end of an edge it does not stop in mid-air:
    // it runs on until it meets the face across the end, and the cap is the
    // curve where the two meet, lying in that face's plane.
    //
    // Capping it in the plane perpendicular to the edge instead is only right
    // when the end face happens to be perpendicular -- true of a box, and of
    // almost nothing else. On a cylinder rim, a cone, a torus, the cap came out
    // as a polygon whose points were not coplanar, ear clipping folded it, and
    // the result self-intersected at any radius at all, however small.
    //
    // endFaceAt[v] is that face, and endDirAt[v] points from v along the edge
    // the fillet arrives on. Only meaningful where exactly one fillet ends at
    // v and exactly one face there is clear of it.
    std::vector<Index> endFaceAt(static_cast<size_t>(mesh.vertexCount()), kInvalid);
    std::vector<Vec3>  endDirAt(static_cast<size_t>(mesh.vertexCount()));
    for (Index v = 0; v < mesh.vertexCount(); ++v) {
        if (filletsAt[v] != 1) continue;

        Vec3 dir{};
        const Index start = mesh.verts[v].halfedge;
        Index h = start;
        do {
            if (beveled[h]) dir = normalize(dirOf(h));
            h = mesh.halfedges[mesh.halfedges[h].twin].next;
        } while (h != start);
        if (lengthSq(dir) < 0.5) continue;

        // Of the faces at v that no fillet touches, the one the edge runs into
        // first. Where several are clear -- a vertex on a quad mesh has two --
        // they are the faces of one smooth surface and only degrees apart, so
        // taking the nearest is a small approximation rather than a wrong
        // answer. It also keeps the cap a single planar curve, which is the
        // property that matters: capping across two planes is what folded.
        Index clear = kInvalid;
        Real bestT = std::numeric_limits<Real>::max();
        h = start;
        do {
            if (!beveled[h] && !beveled[mesh.halfedges[h].prev]) {
                const Index f = mesh.halfedges[h].face;
                const Real denom = dot(dir, faceNormals[f]);
                if (std::fabs(denom) > 1e-9) {
                    // Measured at the corner ball, not at v: every face here
                    // contains v, so v cannot tell them apart. How far the
                    // section has to slide to reach each plane can.
                    const Vec3 c = ballOk[v] ? ballAt[v] : posOf(v);
                    const Real t = std::fabs(dot(posOf(v) - c, faceNormals[f]) / denom);
                    if (t < bestT) { bestT = t; clear = f; }
                }
            }
            h = mesh.halfedges[mesh.halfedges[h].twin].next;
        } while (h != start);

        // Every face across the end runs parallel to the edge, so the fillet
        // never reaches one and there is nothing to trim against.
        if (clear == kInvalid) return bail("fillet ends without a face to stop at");

        // Taking the nearest is only defensible while the faces it stands in
        // for lie in nearly the same plane. Across a real crease -- the corner
        // of a quad mesh, where they can be thirty degrees apart -- the cap
        // genuinely spans two planes, and trimming it into one puts geometry
        // through the surface. Splitting a cap across a fan of faces is an
        // operation we do not have, so refuse rather than build that.
        h = start;
        do {
            if (!beveled[h] && !beveled[mesh.halfedges[h].prev]) {
                const Index f = mesh.halfedges[h].face;
                if (dot(faceNormals[f], faceNormals[clear]) < 0.996)   // five degrees
                    return bail("fillet ends on a corner of several faces");
            }
            h = mesh.halfedges[mesh.halfedges[h].twin].next;
        } while (h != start);

        endFaceAt[v] = clear;
        endDirAt[v] = dir;
    }

    // Slides a point along the edge until it lands on the end face's plane.
    auto trimToEnd = [&](Index v, Vec3 p) {
        const Index g = endFaceAt[v];
        if (g == kInvalid) return p;
        const Vec3 n = faceNormals[g];
        const Vec3 d = endDirAt[v];
        return p + d * (dot(posOf(v) - p, n) / dot(d, n));
    };

    // Where a face's corner moves: the point the corner ball touches it.
    auto crossOffsetAt = [&](Index h, Vec3& d) {
        const Index v = mesh.fromVertex(h);
        const Real r = faceRadiusAt(h);
        if (ballOk[v] && r > 0.0) {
            d = (ballAt[v] + faceNormals[mesh.halfedges[h].face] * r) - posOf(v);
            return true;
        }
        // No ball to solve against: fall back to the plain two-sided inset.
        const Index p = mesh.halfedges[h].prev;
        return cornerOffset(faceNormals[mesh.halfedges[h].face],
                            posOf(v) - posOf(mesh.fromVertex(p)), dirOf(h),
                            beveled[p] ? radiusOf[p] : 0.0,
                            beveled[h] ? radiusOf[h] : 0.0, d);
    };

    // The same point, moved onto the plane of the face across the end. This is
    // the one that gets built; the untrimmed point above is what the section's
    // arc is solved from, so the fillet keeps its true radius and only its
    // termination follows the end face.
    auto crossPointEmit = [&](Index h, Vec3& out) {
        Vec3 d;
        if (!crossOffsetAt(h, d)) return false;
        const Index v = mesh.fromVertex(h);
        out = trimToEnd(v, posOf(v) + d);
        return true;
    };

    // True when either edge at this corner is being filleted, so the corner
    // has a cross-section point at all.
    auto cornerActive = [&](Index h) {
        return beveled[h] || beveled[mesh.halfedges[h].prev];
    };

    // How far back an *unbeveled* edge is cut at each of its ends.
    //
    // This is what a naive per-edge bevel misses. Pulling one face back leaves
    // its neighbour across an unbeveled edge still meeting the original vertex,
    // so the shared edge becomes two edges and the surface is no longer closed.
    // Both faces have to agree on a point along that edge, and the distance is
    // set by whichever side is adjacent to a bevel. Where both sides demand
    // one, the larger wins: the corner has to clear both cuts.
    std::vector<Real> cutBack(static_cast<size_t>(heCount), 0.0);
    for (Index h = 0; h < heCount; ++h) {
        if (beveled[h]) continue;
        const Vec3 dir = normalize(dirOf(h));
        Real t = 0.0;

        // Both faces meeting along this edge get a say; the deeper cut wins,
        // because the corner has to clear every fillet arriving at it.
        if (cornerActive(h)) {
            Vec3 pt;
            if (!crossPointEmit(h, pt)) return bail("corner offset is undefined");
            t = std::max(t, dot(pt - posOf(mesh.fromVertex(h)), dir));
        }
        const Index across = mesh.halfedges[mesh.halfedges[h].twin].next;
        if (cornerActive(across)) {
            Vec3 pt;
            if (!crossPointEmit(across, pt)) return bail("corner offset is undefined");
            t = std::max(t, dot(pt - posOf(mesh.fromVertex(across)), dir));
        }
        cutBack[h] = std::max(t, 0.0);
    }

    Soup soup;

    // One vertex per original vertex, used wherever nothing pulled it back.
    // Sharing it matters: two faces that both leave the corner alone must
    // reference the same vertex or the edge between them will not pair.
    std::vector<uint32_t> baseIdx(static_cast<size_t>(mesh.vertexCount()));
    std::vector<bool> baseUsed(static_cast<size_t>(mesh.vertexCount()), false);
    auto baseVertex = [&](Index v) {
        // Untouched, so it keeps the name it already had.
        if (!baseUsed[v]) { baseIdx[v] = soup.vertex(posOf(v), mesh.verts[v].id); baseUsed[v] = true; }
        return baseIdx[v];
    };

    // One shared vertex per (unbeveled edge, end), referenced from both faces.
    std::vector<uint32_t> sharedPt(static_cast<size_t>(heCount), 0u);
    std::vector<bool> sharedSet(static_cast<size_t>(heCount), false);
    auto edgePoint = [&](Index h) {
        if (sharedSet[h]) return sharedPt[h];
        const Index v = mesh.fromVertex(h);
        sharedPt[h] = cutBack[h] <= 1e-12
                    ? baseVertex(v)
                    : soup.vertex(posOf(v) + normalize(dirOf(h)) * cutBack[h],
                                  nameId(salt, IdRole::Cross, mesh.edgeId(h), mesh.verts[v].id));
        sharedSet[h] = true;
        return sharedPt[h];
    };

    // corner[h] is the point face(h) contributes on the *next* side of its
    // corner at fromVertex(h) -- the one the strip along edge(h) attaches to.
    std::vector<uint32_t> corner(static_cast<size_t>(heCount), 0u);
    std::vector<Vec3> cornerPos(static_cast<size_t>(heCount));

    // The cross-section point face(h) contributes at fromVertex(h), kept apart
    // from corner[] because a corner can now carry both a cross-section and a
    // cut-back point, and the vertex patch has to walk both.
    constexpr uint32_t kNoVertex = ~0u;
    constexpr Real kWeldEps = 1e-18;   // squared; a nanometre at model scale
    std::vector<uint32_t> crossIdx(static_cast<size_t>(heCount), 0u);
    std::vector<Vec3> crossPos(static_cast<size_t>(heCount));
    std::vector<bool> hasCross(static_cast<size_t>(heCount), false);

    for (Index f = 0; f < mesh.faceCount(); ++f) {
        const Index start = mesh.faces[f].halfedge;

        std::vector<uint32_t> loop;
        std::vector<Vec3> poly, moved;

        Index h = start;
        do {
            const Index p = mesh.halfedges[h].prev;
            const Index v = mesh.fromVertex(h);
            const bool bevPrev = beveled[p], bevNext = beveled[h];

            poly.push_back(posOf(v));

            // Boundary order runs from the previous edge round to the next
            // one. An unfilleted edge contributes the point it was cut back
            // to; a filleted one contributes the cross-section, which both of
            // them share when both are filleted.
            if (!bevPrev) loop.push_back(edgePoint(mesh.halfedges[p].twin));

            if (bevPrev || bevNext) {
                Vec3 d;
                if (!crossOffsetAt(h, d)) return bail("corner offset is undefined");
                const Vec3 ptTrue = posOf(v) + d;
                const Vec3 pt = trimToEnd(v, ptTrue);

                // Where the offset runs purely along an unfilleted edge, the
                // cross-section lands exactly on that edge's cut-back point.
                // They have to be one vertex, not two in the same place: a
                // duplicate here leaves a zero-area sliver in the face and the
                // edge fails to pair on the next operation.
                uint32_t idx = kNoVertex;
                if (!bevPrev) {
                    const uint32_t e = edgePoint(mesh.halfedges[p].twin);
                    if (lengthSq(soup.positions[e] - pt) < kWeldEps) idx = e;
                }
                if (idx == kNoVertex && !bevNext) {
                    const uint32_t e = edgePoint(h);
                    if (lengthSq(soup.positions[e] - pt) < kWeldEps) idx = e;
                }
                if (idx == kNoVertex)
                    idx = soup.vertex(pt, nameId(salt, IdRole::Cross, mesh.verts[v].id,
                                                 mesh.faces[mesh.halfedges[h].face].id));

                if (loop.empty() || loop.back() != idx) loop.push_back(idx);
                crossIdx[h] = idx;
                crossPos[h] = ptTrue;
                hasCross[h] = true;
                corner[h] = idx;
                cornerPos[h] = pt;
                moved.push_back(pt);
            } else {
                moved.push_back(soup.positions[edgePoint(mesh.halfedges[p].twin)]);
            }

            if (!bevNext) {
                const uint32_t idx = edgePoint(h);
                if (loop.empty() || loop.back() != idx) loop.push_back(idx);
                corner[h] = idx;
                cornerPos[h] = soup.positions[idx];
            }

            h = mesh.halfedges[h].next;
        } while (h != start);

        // The walk can close on the point it started from.
        while (loop.size() > 1 && loop.front() == loop.back()) loop.pop_back();

        if (loop.size() < 3) return bail("face collapsed");
        if (!insetIsValid(poly, moved, faceNormals[f])) {
            if (dbg) {
                std::fprintf(stderr, "[bevel] face %d (deg %zu) inverts:\n", f, poly.size());
                for (size_t i = 0; i < poly.size(); ++i)
                    std::fprintf(stderr, "        (%7.3f %7.3f %7.3f) -> (%7.3f %7.3f %7.3f)\n",
                                 poly[i].x, poly[i].y, poly[i].z,
                                 moved[i].x, moved[i].y, moved[i].z);
            }
            return bail("face inverts at this width");
        }

        // `moved` carries one representative per original corner, so it tests
        // the corner-to-corner correspondence but not the denser polygon that
        // actually gets emitted. Area-check that one too.
        {
            std::vector<Vec3> loopPos;
            loopPos.reserve(loop.size());
            for (uint32_t idx : loop) loopPos.push_back(soup.positions[idx]);
            if (signedArea(loopPos, faceNormals[f]) <= 1e-9)
                return bail("emitted loop inverts at this width");
        }

        // Shrunk, not replaced: it is the same face and keeps its name.
        soup.face(loop, mesh.faces[f].id);
    }

    // Arc points along each beveled edge, at both ends, keyed by half-edge so
    // the vertex patches can pick them up in the right order.
    // ringAt[h] runs from face(h)'s corner to face(twin(h))'s corner, at
    // fromVertex(h).
    std::vector<std::vector<uint32_t>> ringAt(static_cast<size_t>(heCount));
    std::vector<Arc> arcAt(static_cast<size_t>(heCount));

    for (Index h = 0; h < heCount; ++h) {
        if (!beveled[h]) continue;
        const Index tw = mesh.halfedges[h].twin;
        const Index f0 = mesh.halfedges[h].face;
        const Index f1 = mesh.halfedges[tw].face;

        // Both ends of the cross-section must be the *cross-section* points,
        // not whatever else those corners carry. The far face's corner can now
        // also hold a cut-back point for an unfilleted edge, and attaching the
        // strip to that is what skews the section.
        const Index far = mesh.halfedges[tw].next;
        if (!hasCross[h] || !hasCross[far]) return bail("edge has no cross-section");
        const Vec3 p0 = crossPos[h];
        const Vec3 p1 = crossPos[far];
        const Vec3 axis = normalize(mesh.verts[mesh.halfedges[h].vertex].position -
                                    mesh.verts[mesh.fromVertex(h)].position);

        const Arc arc = solveArc(p0, p1, -faceNormals[f0], -faceNormals[f1]);
        arcAt[h] = arc;

        std::vector<Vec3> pts;
        sampleArc(arc, p0, p1, axis, segments, pts);

        std::vector<uint32_t>& ring = ringAt[h];
        ring.push_back(crossIdx[h]);
        for (size_t i = 1; i + 1 < pts.size(); ++i)
            ring.push_back(soup.vertex(
                trimToEnd(mesh.fromVertex(h), pts[i]),
                nameId(salt, IdRole::Ring, mesh.edgeId(h),
                       mesh.verts[mesh.fromVertex(h)].id, i)));
        ring.push_back(crossIdx[far]);
    }

    // One strip per beveled edge, emitted once per twin pair.
    for (Index h = 0; h < heCount; ++h) {
        if (!beveled[h]) continue;
        const Index tw = mesh.halfedges[h].twin;
        if (tw < h) continue;

        // Ring at the far end, reversed so both rings run the same way across
        // the strip.
        const std::vector<uint32_t>& a = ringAt[h];         // at fromVertex(h)
        std::vector<uint32_t> b = ringAt[tw];               // at toVertex(h)
        std::reverse(b.begin(), b.end());
        if (a.size() != b.size() || a.size() < 2) return bail("strip rings disagree");

        // Wound so the strip faces out of the solid: running a-then-b in
        // index order gives the inward face, which build() then refuses as two
        // faces sharing a directed edge.
        for (size_t s = 0; s + 1 < a.size(); ++s)
            soup.face({a[s + 1], b[s + 1], b[s], a[s]},
                      nameId(salt, IdRole::Strip, mesh.edgeId(h), 0, s));
    }

    // Vertex patches, wherever at least one incident edge was beveled.
    // Flat end caps, to be spliced into the coplanar face across the end.
    std::vector<size_t> capFaces;

    for (Index v = 0; v < mesh.vertexCount(); ++v) {
        const Index start = mesh.verts[v].halfedge;
        if (start == kInvalid) continue;

        bool any = false;
        Index h = start;
        do {
            if (beveled[h]) { any = true; break; }
            h = mesh.halfedges[mesh.halfedges[h].twin].next;
        } while (h != start);
        if (!any) continue;

        // Running counters so every point and face a patch emits gets its own
        // name. The code path is deterministic, so re-evaluating names them the
        // same way again.
        uint64_t patchV = 0, patchF = 0;
        auto patchVertex = [&](Vec3 p) {
            return soup.vertex(p, nameId(salt, IdRole::Patch, mesh.verts[v].id, 1, patchV++));
        };
        auto patchFace = [&](const std::vector<uint32_t>& l) {
            soup.face(l, nameId(salt, IdRole::Patch, mesh.verts[v].id, 2, patchF++));
        };

        // A corner where two edges are filleted and at least one stays sharp is
        // not a ball corner. The ball only settles into a corner when every
        // face there is tangent to it, and the faces along a sharp edge are
        // not. What happens instead is that the two fillet surfaces run into
        // each other and are trimmed along the curve where they cross -- a
        // mitre -- and the sharp edge runs up to the single point where that
        // curve ends. It is what Fusion does, and Blender, and it is why a
        // filleted box rim has a sharp vertical edge and no shelf at the
        // corner.
        //
        // Building a sphere there instead leaves a gap between the sphere and
        // the sharp edge, which then has to be covered by a flat gusset: a
        // horizontal ledge tucked into the corner, facing up, that no fillet
        // should ever produce.
        //
        // The mitre is exact and cheap. Take the section at the end of one
        // fillet and slide each of its points along that edge until it lands on
        // the other fillet's surface. Every point of the result is on both
        // cylinders by construction, so the two ruled patches either side of it
        // meet without a seam.
        if (filletsAt[v] == 2 && sharpAt[v] == 1 && segments >= 1) {
            Index g0 = kInvalid, g1 = kInvalid;
            Index k = start;
            do {
                if (beveled[k]) { (g0 == kInvalid ? g0 : g1) = k; }
                k = mesh.halfedges[mesh.halfedges[k].twin].next;
            } while (k != start);

            // The face both fillets bound. Their sections start there, and so
            // does the mitre.
            Index shared = kInvalid;
            if (g0 != kInvalid && g1 != kInvalid) {
                const Index a0 = mesh.halfedges[g0].face;
                const Index a1 = mesh.halfedges[mesh.halfedges[g0].twin].face;
                const Index b0 = mesh.halfedges[g1].face;
                const Index b1 = mesh.halfedges[mesh.halfedges[g1].twin].face;
                if (a0 == b0 || a0 == b1) shared = a0;
                else if (a1 == b0 || a1 == b1) shared = a1;
            }

            // Two fillets meeting at a vertex are trimmed against each other
            // whether they turn a right angle or continue almost straight. On a
            // rim following a curved wall the turn is a few degrees and the
            // trim is correspondingly small -- but it is still the right
            // surface, and it is a handful of faces where a blend patch at
            // every vertex around the rim was hundreds.
            //
            // Exactly collinear is the one case with nothing to trim: the two
            // cylinders coincide and there is no curve where they cross.
            const bool turns =
                g0 != kInvalid && g1 != kInvalid &&
                dot(normalize(dirOf(g0)), normalize(dirOf(g1))) > -0.99999;

            if (turns && shared != kInvalid &&
                arcAt[g0].circular && arcAt[g1].circular &&
                ringAt[g0].size() == ringAt[g1].size() && ringAt[g0].size() >= 2) {

                // Which of the two lies *in* the shared face decides the way
                // round the patch is wound, and that is not the order the
                // circulation happened to visit them in -- the sharp edge can
                // fall either side. Naming them by their role rather than by
                // their order is what makes the winding independent of it.
                if (mesh.halfedges[g0].face != shared) std::swap(g0, g1);

                // ringAt runs from face(h)'s corner round to the far face's, so
                // the second section is read backwards. Both then start at the
                // shared corner and their indices correspond.
                std::vector<uint32_t> ra = ringAt[g0], rb = ringAt[g1];
                if (mesh.halfedges[g0].face != shared) std::reverse(ra.begin(), ra.end());
                if (mesh.halfedges[g1].face != shared) std::reverse(rb.begin(), rb.end());

                // Slides `q` along `u` until it sits at radius r about the axis
                // of the other fillet. Nearest non-negative root: the section
                // starts on its own cylinder and moves toward the corner.
                auto slideOnto = [&](Vec3 q, Vec3 u, Vec3 axisPt, Vec3 axisDir, Real r,
                                     Real& t) {
                    const Vec3 a = u - axisDir * dot(u, axisDir);
                    const Vec3 w = q - axisPt;
                    const Vec3 b = w - axisDir * dot(w, axisDir);
                    const Real aa = lengthSq(a);
                    if (aa < 1e-18) return false;
                    const Real bb = dot(a, b), cc = lengthSq(b) - r * r;
                    const Real disc = bb * bb - aa * cc;
                    if (disc < 0.0) return false;
                    const Real root = std::sqrt(disc);
                    const Real t0 = (-bb - root) / aa, t1 = (-bb + root) / aa;
                    t = (t0 >= -1e-9) ? t0 : t1;
                    return t >= -1e-9;
                };

                const Vec3 u0 = -normalize(dirOf(g0));
                const Vec3 d1 = normalize(dirOf(g1));
                const Real r1 = radiusOf[g1];

                // The point the sharp edge runs up to. The faces either side of
                // that edge have already been cut back to it, so the mitre has
                // to end on that exact vertex rather than on its own idea of
                // where the curve finishes -- otherwise the patch and the faces
                // meet at two points a hair apart and the surface does not
                // close.
                Index sharp = kInvalid;
                Index sh = start;
                do {
                    if (!beveled[sh]) sharp = sh;
                    sh = mesh.halfedges[mesh.halfedges[sh].twin].next;
                } while (sh != start);

                std::vector<uint32_t> mid;
                bool ok = sharp != kInvalid;
                for (size_t i = 0; ok && i < ra.size(); ++i) {
                    // The ends belong to what is already there: the shared
                    // face's corner, and the top of the sharp edge.
                    if (i == 0) { mid.push_back(ra[0]); continue; }
                    if (i + 1 == ra.size()) { mid.push_back(edgePoint(sharp)); continue; }

                    Real t = 0.0;
                    const Vec3 q = soup.positions[ra[i]];
                    if (!slideOnto(q, u0, arcAt[g1].centre, d1, r1, t)) { ok = false; break; }

                    // Where the two fillets differ sharply in how far they set
                    // their faces back -- a 90 degree edge meeting a 22 degree
                    // one -- the curve where they cross can run outside the
                    // solid entirely. A fillet on a convex corner only ever
                    // removes material, so a point outside any face at this
                    // vertex means the mitre is not the right construction
                    // here, and the corner falls back to a blend.
                    const Vec3 p = q + u0 * t;
                    Index fk = start;
                    do {
                        const Index face = mesh.halfedges[fk].face;
                        if (face != kInvalid &&
                            dot(p - posOf(v), faceNormals[face]) > 1e-9) ok = false;
                        fk = mesh.halfedges[mesh.halfedges[fk].twin].next;
                    } while (fk != start && ok);
                    if (!ok) break;

                    mid.push_back(patchVertex(p));
                }

                // Every piece of the patch has to face out of the solid. A
                // mitre between two fillets that set their faces back by very
                // different amounts can fold back on itself, and a folded patch
                // is a self-intersecting model, so check before committing to
                // it rather than after.
                Vec3 outward{};
                {
                    Index fk = start;
                    do {
                        const Index face = mesh.halfedges[fk].face;
                        if (face != kInvalid) outward += faceNormals[face];
                        fk = mesh.halfedges[mesh.halfedges[fk].twin].next;
                    } while (fk != start);
                    if (lengthSq(outward) < 1e-18) ok = false;
                    else outward = normalize(outward);
                }

                auto facesOut = [&](const std::vector<uint32_t>& l) {
                    Vec3 n{};
                    for (size_t i = 0; i < l.size(); ++i)
                        n += cross(soup.positions[l[i]],
                                   soup.positions[l[(i + 1) % l.size()]]);
                    return lengthSq(n) < 1e-24 || dot(normalize(n), outward) > 0.0;
                };

                std::vector<std::vector<uint32_t>> pieces;
                for (size_t i = 0; ok && i + 1 < ra.size(); ++i) {
                    pieces.push_back({ra[i], mid[i], mid[i + 1], ra[i + 1]});
                    pieces.push_back({mid[i], rb[i], rb[i + 1], mid[i + 1]});
                    if (!facesOut(pieces[pieces.size() - 2]) || !facesOut(pieces.back()))
                        ok = false;
                }

                if (ok) {
                    for (const auto& piece : pieces) patchFace(piece);
                    continue;
                }
            }
        }

        // Walk the outgoing half-edges. Each face contributes its corner --
        // which may be two points where it split -- and each beveled edge
        // contributes the arc crossing it, which the strip already shares.
        std::vector<uint32_t> loop;
        std::vector<Vec3> loopPos;
        std::vector<Vec3> centres;
        std::vector<size_t> cornerSlots;   // where the face corners sit in `loop`

        auto push = [&](uint32_t idx) {
            if (!loop.empty() && loop.back() == idx) return;   // corners can coincide
            loop.push_back(idx);
            loopPos.push_back(soup.positions[idx]);
        };

        h = start;
        do {
            const Index p = mesh.halfedges[h].prev;
            // Mirrors how the face boundary was emitted, so the patch closes
            // against it exactly.
            if (!beveled[p]) push(edgePoint(mesh.halfedges[p].twin));
            if (hasCross[h]) {
                cornerSlots.push_back(loop.size());
                push(crossIdx[h]);
            }
            if (!beveled[h]) push(edgePoint(h));
            if (beveled[h]) {
                const std::vector<uint32_t>& ring = ringAt[h];
                for (size_t i = 1; i + 1 < ring.size(); ++i) push(ring[i]);
                if (arcAt[h].circular) centres.push_back(arcAt[h].centre);
            }
            h = mesh.halfedges[mesh.halfedges[h].twin].next;
        } while (h != start && loop.size() < 4096);

        // The walk can close on the point it started from.
        while (loop.size() > 1 && loop.front() == loop.back()) {
            loop.pop_back();
            loopPos.pop_back();
        }

        std::reverse(loop.begin(), loop.end());
        std::reverse(loopPos.begin(), loopPos.end());
        // The slots were recorded against the forward walk, so reversing the
        // loop has to carry them with it or every later step reads the wrong
        // three points.
        {
            const size_t n = loop.size();
            std::vector<size_t> flipped;
            for (size_t s : cornerSlots)
                if (s < n) flipped.push_back(n - 1 - s);
            std::sort(flipped.begin(), flipped.end());
            cornerSlots.swap(flipped);
        }
        if (loop.size() < 3) continue;

        // Where only one fillet arrives, the cap is flat and coplanar with
        // the face across the end -- the section was not set back, so it lies
        // in that plane. Curving it would push geometry off a flat face.
        //
        // A chamfer's corner is genuinely a flat facet too.
        if (!blendsAt(v) || segments == 1 || loop.size() <= 3 || centres.empty()) {
            // A cap closing the end of a single fillet is flat and sits in the
            // plane of the face across the end. Note it so it can be spliced
            // into that face rather than left as a seam across it.
            if (filletsAt[v] == 1) capFaces.push_back(soup.faceSizes.size());

            // A corner where three edges meet closes with a triangle, which is
            // planar whatever its corners do. Four or more need not be: at a
            // vertex of a quad mesh a chamfer's corner is four points that in
            // general lie on no common plane, and handing that to the
            // triangulator -- which has to flatten it to work at all -- folds
            // it over itself. Fan it from a point on the blend sphere instead,
            // so every triangle is planar and the patch stays on the surface
            // the fillet is made of.
            Vec3 mid{};
            for (const Vec3& q : loopPos) mid += q;
            mid = mid / static_cast<Real>(loopPos.size());

            Real flatness = 0.0;
            if (loop.size() > 3) {
                Vec3 nrm{};
                for (size_t i = 0; i < loopPos.size(); ++i)
                    nrm += cross(loopPos[i], loopPos[(i + 1) % loopPos.size()]);
                if (lengthSq(nrm) > 1e-24) {
                    nrm = normalize(nrm);
                    for (const Vec3& q : loopPos)
                        flatness = std::max(flatness, std::fabs(dot(q - mid, nrm)));
                }
            }

            if (flatness <= weldEps * 1e3) {
                patchFace(loop);
                continue;
            }

            // Onto the sphere the surrounding sections are tangent to, so the
            // fan sits on the blend rather than cutting across it.
            if (ballOk[v]) {
                const Vec3 away = mid - ballAt[v];
                if (lengthSq(away) > 1e-24)
                    mid = ballAt[v] + normalize(away) * length(loopPos[0] - ballAt[v]);
            }
            const uint32_t hub = patchVertex(mid);
            for (size_t i = 0; i < loop.size(); ++i)
                patchFace({loop[i], loop[(i + 1) % loop.size()], hub});
            continue;
        }

        // A corner where some arriving edges are filleted and some are not has
        // a boundary that is partly on the blend sphere and partly not: a
        // corner cut back along an unfilleted edge lands at r*sqrt(2). Letting
        // those points drag the whole patch turns a mostly-spherical corner
        // into a cone with a pinch point.
        //
        // Split it instead. Replace each run of off-sphere points with an arc
        // generated across the gap on the sphere, blend the resulting all-on-
        // sphere loop properly, and fill what was cut away with a flat gusset
        // between the generated arc and the points it replaced.
        {
            Vec3 sc{};
            for (const Vec3& p : centres) sc += p;
            sc = sc / static_cast<Real>(centres.size());

            Real sphereR = std::numeric_limits<Real>::max();
            for (const Vec3& p : loopPos) sphereR = std::min(sphereR, length(p - sc));

            std::vector<bool> onIt(loopPos.size(), false);
            size_t offCount = 0;
            for (size_t i = 0; i < loopPos.size(); ++i) {
                onIt[i] = length(loopPos[i] - sc) <= sphereR * 1.02;
                if (!onIt[i]) ++offCount;
            }

            if (offCount > 0 && offCount < loopPos.size() - 2) {
                // Carried through the rebuild: after bridging, a corner where
                // two fillets and one sharp edge met becomes an ordinary
                // three-arc loop, and should get the pole-free lattice like any
                // other.
                std::vector<bool> isCorner(loopPos.size(), false);
                for (size_t s : cornerSlots)
                    if (s < isCorner.size()) isCorner[s] = true;
                std::vector<bool> sphereCorner;
                auto slerpTo = [&](Vec3 p, Vec3 q, Real t) {
                    const Vec3 u = p - sc, v2 = q - sc;
                    const Real ru = length(u), rv = length(v2);
                    if (ru < 1e-12 || rv < 1e-12) return lerp(p, q, t);
                    const Vec3 un = u / ru, vn = v2 / rv;
                    const Real ang = std::acos(clampf(dot(un, vn), -1.0, 1.0));
                    const Vec3 ax = cross(un, vn);
                    if (ang < 1e-9 || lengthSq(ax) < 1e-18) return lerp(p, q, t);
                    return sc + rotate(Quat::fromAxisAngle(normalize(ax), ang * t), un)
                              * lerpf(ru, rv, t);
                };

                std::vector<uint32_t> sphereLoop;
                std::vector<Vec3> spherePos;
                std::vector<std::pair<std::vector<uint32_t>, std::vector<uint32_t>>> gussets;

                const size_t n0 = loopPos.size();
                size_t startAt = 0;
                while (startAt < n0 && !onIt[startAt]) ++startAt;

                for (size_t k = 0; k < n0; ) {
                    const size_t i = (startAt + k) % n0;
                    if (onIt[i]) {
                        sphereLoop.push_back(loop[i]);
                        spherePos.push_back(loopPos[i]);
                        sphereCorner.push_back(isCorner[i]);
                        ++k;
                        continue;
                    }
                    // A run of off-sphere points: collect it, then bridge.
                    std::vector<uint32_t> run;
                    size_t k2 = k;
                    while (k2 < n0 && !onIt[(startAt + k2) % n0]) {
                        run.push_back(loop[(startAt + k2) % n0]);
                        ++k2;
                    }
                    const Vec3 a = spherePos.back();
                    const Vec3 b = loopPos[(startAt + (k2 % n0)) % n0];

                    std::vector<uint32_t> bridge;
                    bridge.push_back(sphereLoop.back());
                    for (int s = 1; s < segments; ++s) {
                        const Vec3 p = slerpTo(a, b, static_cast<Real>(s) / segments);
                        const uint32_t idx = patchVertex(p);
                        bridge.push_back(idx);
                        sphereLoop.push_back(idx);
                        spherePos.push_back(p);
                        sphereCorner.push_back(false);   // arc interior
                    }
                    bridge.push_back(loop[(startAt + (k2 % n0)) % n0]);
                    gussets.emplace_back(bridge, run);
                    k = k2;
                }

                // Fill between each generated arc and the points it replaced.
                for (const auto& g : gussets) {
                    const std::vector<uint32_t>& arc = g.first;
                    const std::vector<uint32_t>& run = g.second;
                    for (size_t i = 0; i + 1 < arc.size(); ++i)
                        patchFace({arc[i + 1], arc[i], run.front()});
                    for (size_t i = 0; i + 1 < run.size(); ++i)
                        patchFace({arc.back(), run[i], run[i + 1]});
                }

                loop.swap(sphereLoop);
                loopPos.swap(spherePos);
                cornerSlots.clear();
                for (size_t i = 0; i < sphereCorner.size(); ++i)
                    if (sphereCorner[i]) cornerSlots.push_back(i);
            }
        }

        // Rolling-ball model, which is what a CAD kernel implements: a ball of
        // the fillet radius rolls touching both faces, sweeping a cylinder
        // along an edge, and at a vertex it pivots in place and sweeps part of
        // a sphere. Every edge arc meeting here was solved about that same
        // centre, so the patch interior belongs on the sphere. Fanning it flat
        // from a hub is what made the corner read as a cut facet rather than a
        // blend.
        Vec3 c{};
        for (const Vec3& p : centres) c += p;
        c = c / static_cast<Real>(centres.size());

        std::vector<Vec3> dirs(loopPos.size());
        std::vector<Real> radii(loopPos.size());
        Vec3 poleDir{};
        Real poleRadius = 0.0;
        bool degenerate = false;

        poleRadius = std::numeric_limits<Real>::max();
        for (size_t i = 0; i < loopPos.size(); ++i) {
            const Vec3 d = loopPos[i] - c;
            const Real len = length(d);
            if (len < 1e-12) { degenerate = true; break; }
            dirs[i] = d / len;
            radii[i] = len;
            poleDir += dirs[i];
            // The smallest, not the average.
            //
            // The rolling ball's sphere has exactly the fillet radius, and the
            // arc points sit on it. Boundary points that do not -- a corner cut
            // back along an edge that was left sharp lands at r*sqrt(2) -- are
            // further out, and averaging them in lifts the whole patch off the
            // sphere and outside the original solid. Filleting a convex box was
            // *adding* 107mm^3 and pushing 160 vertices past the faces they
            // should sit inside.
            poleRadius = std::min(poleRadius, len);
        }
        if (degenerate || lengthSq(poleDir) < 1e-18) { patchFace(loop); continue; }
        poleDir = normalize(poleDir);

        // Three arcs meeting at three tangent points is the ordinary corner --
        // a box corner, and most corners on a real part. The patch there is a
        // spherical triangle, and it can be tessellated as a triangular
        // lattice with no pole at all, which is what a CAD kernel shows. The
        // ring scheme below is the general fallback for every other valence.
        if (cornerSlots.size() == 3 && loop.size() == static_cast<size_t>(3 * segments)) {
            const Vec3 A = loopPos[cornerSlots[0]];
            const Vec3 B = loopPos[cornerSlots[1]];
            const Vec3 C3 = loopPos[cornerSlots[2]];

            // Great-circle step about the corner sphere. The edge arcs were
            // swept about axes through this same centre, so they *are* great
            // circles here and the lattice boundary lands exactly on them.
            auto slerpSphere = [&](Vec3 p, Vec3 q, Real t) {
                const Vec3 u = p - c, v = q - c;
                const Real ru = length(u), rv = length(v);
                if (ru < 1e-12 || rv < 1e-12) return lerp(p, q, t);
                const Vec3 un = u / ru, vn = v / rv;
                const Real ang = std::acos(clampf(dot(un, vn), -1.0, 1.0));
                const Vec3 ax = cross(un, vn);
                if (ang < 1e-9 || lengthSq(ax) < 1e-18) return lerp(p, q, t);
                return c + rotate(Quat::fromAxisAngle(normalize(ax), ang * t), un)
                         * lerpf(ru, rv, t);
            };

            // Spherical barycentric: slerp in from each corner toward the
            // opposite edge, then average. On any edge two of the three agree
            // with the third, so the boundary reduces to a plain slerp and
            // meets the strips exactly.
            auto patchPoint = [&](Real wa, Real wb, Real wg) {
                Vec3 acc{};
                int n = 0;
                if (wb + wg > 1e-12) { acc += slerpSphere(A,  slerpSphere(B, C3, wg / (wb + wg)), wb + wg); ++n; }
                if (wg + wa > 1e-12) { acc += slerpSphere(B,  slerpSphere(C3, A, wa / (wg + wa)), wg + wa); ++n; }
                if (wa + wb > 1e-12) { acc += slerpSphere(C3, slerpSphere(A, B,  wb / (wa + wb)), wa + wb); ++n; }
                if (n == 0) return A;
                const Vec3 avg = acc / static_cast<Real>(n);
                const Vec3 d = avg - c;
                if (lengthSq(d) < 1e-18) return avg;
                const Real rad = wa * length(A - c) + wb * length(B - c) + wg * length(C3 - c);
                return c + normalize(d) * rad;
            };

            // Lattice indexed by (i, j) with i down from corner A and j across.
            // Row i has i+1 points; the boundary rows reuse the vertices the
            // strips already created so nothing has to be welded afterwards.
            const int N = segments;
            std::vector<std::vector<uint32_t>> rows(static_cast<size_t>(N) + 1);
            auto boundary = [&](size_t slotFrom, int step) {
                return loop[(cornerSlots[slotFrom] + static_cast<size_t>(step)) % loop.size()];
            };

            rows[0].push_back(loop[cornerSlots[0]]);
            for (int i = 1; i <= N; ++i) {
                std::vector<uint32_t>& row = rows[static_cast<size_t>(i)];
                row.reserve(static_cast<size_t>(i) + 1);
                for (int j = 0; j <= i; ++j) {
                    if (j == 0)      { row.push_back(boundary(0, i)); continue; }      // arc A->B
                    if (j == i && i < N) { row.push_back(boundary(2, N - i)); continue; }  // arc C->A
                    if (i == N)      { row.push_back(boundary(1, j)); continue; }      // arc B->C
                    const Real wa = static_cast<Real>(N - i) / N;
                    const Real wb = static_cast<Real>(i - j) / N;
                    const Real wg = static_cast<Real>(j) / N;
                    row.push_back(patchVertex(patchPoint(wa, wb, wg)));
                }
            }

            for (int i = 0; i < N; ++i) {
                const std::vector<uint32_t>& up = rows[static_cast<size_t>(i)];
                const std::vector<uint32_t>& lo2 = rows[static_cast<size_t>(i) + 1];
                for (int j = 0; j <= i; ++j) {
                    patchFace({up[static_cast<size_t>(j)], lo2[static_cast<size_t>(j)],
                               lo2[static_cast<size_t>(j) + 1]});
                    if (j < i)
                        patchFace({up[static_cast<size_t>(j)], lo2[static_cast<size_t>(j) + 1],
                                   up[static_cast<size_t>(j) + 1]});
                }
            }
            continue;
        }

        // Radius is carried per boundary point rather than fixed at the
        // fillet's. Where only some edges at a vertex are rounded, part of the
        // boundary sits on the sphere and part does not -- a corner cut back
        // along an unrounded edge is further out -- and interpolating both the
        // direction and the radius blends between them instead of tearing the
        // patch away from geometry it has to meet.
        // Bulging outward is only correct when the whole boundary already sits
        // on the sphere -- an all-rounded corner of any valence. Where it does
        // not, curving the patch out pushes it through faces of the original
        // solid, so interpolate straight instead: both ends are inside a convex
        // body, so every point on the segment is too. Slightly less round, and
        // never wrong.
        Real widest = 0.0;
        for (Real rad : radii) widest = std::max(widest, rad);
        const bool onSphere = widest <= poleRadius * 1.02;

        const int rings = std::max(1, (segments + 1) / 2);
        const size_t n = loop.size();
        const Vec3 poleAt = c + poleDir * poleRadius;

        std::vector<uint32_t> prev = loop;
        for (int j = 1; j < rings; ++j) {
            const Real t = static_cast<Real>(j) / static_cast<Real>(rings);
            std::vector<uint32_t> ring;
            ring.reserve(n);
            for (size_t i = 0; i < n; ++i) {
                const Vec3 p = onSphere
                             ? c + normalize(lerp(dirs[i], poleDir, t)) *
                                   lerpf(radii[i], poleRadius, t)
                             : lerp(loopPos[i], poleAt, t);
                ring.push_back(patchVertex(p));
            }
            for (size_t i = 0; i < n; ++i) {
                const size_t k = (i + 1) % n;
                patchFace({prev[i], prev[k], ring[k], ring[i]});
            }
            prev.swap(ring);
        }

        const uint32_t pole = patchVertex(poleAt);
        for (size_t i = 0; i < prev.size(); ++i)
            patchFace({prev[i], prev[(i + 1) % prev.size()], pole});
    }

    // Weld before merging, not after. Two sections that landed in the same
    // place are one vertex, and until that is settled a cap and the face it
    // belongs to look like they meet along one edge when they really touch at
    // a second point too -- and splicing them there folds the polygon onto
    // itself. A nanometre is far below anything a fillet resolves and far
    // above the arithmetic's noise.
    std::vector<size_t> movedTo;
    soup.weld(weldEps, &movedTo);

    std::vector<size_t> caps;
    caps.reserve(capFaces.size());
    for (size_t f : capFaces)
        if (f < movedTo.size() && movedTo[f] != Soup::kDropped) caps.push_back(movedTo[f]);

    // Half a degree, matching the tolerance that decides an edge is flat.
    soup.absorb(caps, 0.9999619);
    soup.weld(weldEps);

    if (dbg) {
        size_t at = 0;
        for (size_t f = 0; f < soup.faceSizes.size(); ++f) {
            const uint32_t n = soup.faceSizes[f];
            std::vector<uint32_t> s(soup.faceIndices.begin() + static_cast<long>(at),
                                    soup.faceIndices.begin() + static_cast<long>(at + n));
            at += n;
            std::vector<uint32_t> t = s;
            std::sort(t.begin(), t.end());
            if (std::adjacent_find(t.begin(), t.end()) == t.end()) continue;
            std::fprintf(stderr, "[bevel] face %zu repeats a vertex:", f);
            for (uint32_t i : s) std::fprintf(stderr, " %u", i);
            std::fprintf(stderr, "\n");
            for (uint32_t i : s)
                std::fprintf(stderr, "          %u (%.4f %.4f %.4f)\n", i,
                             soup.positions[i].x, soup.positions[i].y, soup.positions[i].z);
        }
    }

    if (!soup.commit(mesh)) {
        if (dbg) {
            std::map<std::pair<uint32_t,uint32_t>,int> dir;
            size_t at = 0; 
            for (uint32_t fs : soup.faceSizes) {
                for (uint32_t i = 0; i < fs; ++i)
                    ++dir[{soup.faceIndices[at+i], soup.faceIndices[at+(i+1)%fs]}];
                at += fs;
            }
            int unpaired = 0, dupes = 0;
            for (const auto& [k,n] : dir) {
                if (n > 1) ++dupes;
                if (!dir.count({k.second, k.first})) ++unpaired;
            }
            std::fprintf(stderr, "[bevel] rebuild refused: %zu verts %zu faces, "
                         "%d unpaired, %d duplicated\n",
                         soup.positions.size(), soup.faceSizes.size(), unpaired, dupes);
        }
        return false;
    }
    return true;
}

bool bevelEdges(Mesh& mesh, const std::vector<Index>& edges, Real width, int segments) {
    FilletSpec spec;
    spec.segments = segments;
    spec.edges.reserve(edges.size());
    for (Index e : edges) spec.edges.push_back({e, width});
    return filletEdges(mesh, spec);
}

namespace {

// One flat region of the surface: every face lying in a common plane.
struct PlaneGroup {
    Vec3 normal;
    Real offset = 0.0;
    std::vector<Index> faces;
};

// Basis for measuring inside a plane.
void planeBasis(Vec3 n, Vec3& u, Vec3& v) {
    const Vec3 seed = std::fabs(n.z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0};
    u = normalize(cross(seed, n));
    v = cross(n, u);
}

} // namespace

int mergeCoplanarFaces(Mesh& mesh, Real toleranceDegrees) {
    if (mesh.empty()) return 0;

    const Real cosTol = std::cos(toleranceDegrees * kPi / 180.0);
    const Real offTol = std::max(length(mesh.bounds().size()), Real(1.0)) * 1e-7;

    // ---- Gather the flat regions ------------------------------------------
    std::vector<PlaneGroup> groups;
    std::vector<int> groupOf(static_cast<size_t>(mesh.faceCount()), -1);
    for (Index f = 0; f < mesh.faceCount(); ++f) {
        if (mesh.faceArea(f) < 1e-14) continue;
        const Vec3 n = mesh.faceNormal(f);
        const Real d = dot(n, mesh.faceCentroid(f));
        int at = -1;
        for (size_t g = 0; g < groups.size(); ++g)
            if (dot(groups[g].normal, n) > cosTol &&
                std::fabs(groups[g].offset - d) < offTol) { at = static_cast<int>(g); break; }
        if (at < 0) {
            at = static_cast<int>(groups.size());
            groups.push_back({n, d, {}});
        }
        groups[at].faces.push_back(f);
        groupOf[f] = at;
    }

    // ---- Rebuild each region from its own outline --------------------------
    //
    // Not by merging faces two at a time. A boolean shreds one flat face into a
    // fan of dozens, and pairwise merging stalls the moment the running polygon
    // stops being simple -- which is why a subtracted sphere used to leave the
    // cube's walls covered in zigzags.
    //
    // Instead: take every directed edge of every face in the region, and cancel
    // the ones that appear in both directions. Those are interior to the region
    // and the surface does not turn there. What survives is the region's actual
    // outline, and it does not matter how many pieces it arrived in.
    Soup soup;
    soup.positions.reserve(mesh.verts.size());
    for (const MeshVertex& mv : mesh.verts) soup.vertex(mv.position, mv.id);

    int merged = 0;
    std::vector<bool> emitted(static_cast<size_t>(mesh.faceCount()), false);
    std::vector<Index> fv;

    for (const PlaneGroup& g : groups) {
        auto keepOriginals = [&] {
            for (Index f : g.faces) {
                if (emitted[f]) continue;
                emitted[f] = true;
                soup.face(faceLoop(mesh, f), mesh.faces[f].id);
            }
        };

        if (g.faces.size() == 1) { keepOriginals(); continue; }

        // Directed edges, with the interior ones cancelled.
        std::map<std::pair<Index, Index>, int> dir;
        for (Index f : g.faces) {
            mesh.faceVertices(f, fv);
            for (size_t i = 0; i < fv.size(); ++i)
                ++dir[{fv[i], fv[(i + 1) % fv.size()]}];
        }
        std::map<Index, std::vector<Index>> outgoing;
        bool ambiguous = false;
        for (const auto& [e, n] : dir) {
            if (n != 1) { ambiguous = true; break; }
            if (dir.count({e.second, e.first})) continue;   // interior: cancels
            outgoing[e.first].push_back(e.second);
        }
        // A vertex the outline passes through twice cannot be chained without
        // guessing which way to go, so that region is left as it came.
        for (const auto& [from, to] : outgoing)
            if (to.size() != 1) { ambiguous = true; break; }
        if (ambiguous || outgoing.empty()) { keepOriginals(); continue; }

        // Chain the outline into closed loops.
        std::vector<std::vector<Index>> loops;
        std::set<Index> used;
        bool broken = false;
        for (const auto& [from, to] : outgoing) {
            if (used.count(from)) continue;
            std::vector<Index> loop;
            Index at = from;
            while (!used.count(at)) {
                used.insert(at);
                loop.push_back(at);
                auto it = outgoing.find(at);
                if (it == outgoing.end()) { broken = true; break; }
                at = it->second.front();
            }
            if (broken || at != from || loop.size() < 3) { broken = true; break; }
            loops.push_back(std::move(loop));
        }
        if (broken || loops.empty()) { keepOriginals(); continue; }

        // Keep the name of the largest piece: it is the one the user is most
        // likely to have selected, and the merged face is the same surface.
        Index best = g.faces.front();
        for (Index f : g.faces) if (mesh.faceArea(f) > mesh.faceArea(best)) best = f;

        auto asSoup = [&](const std::vector<Index>& l) {
            std::vector<uint32_t> out;
            out.reserve(l.size());
            for (Index v : l) out.push_back(static_cast<uint32_t>(v));
            return out;
        };

        if (loops.size() == 1) {
            soup.face(asSoup(loops[0]), mesh.faces[best].id);
            for (Index f : g.faces) emitted[f] = true;
            merged += static_cast<int>(g.faces.size()) - 1;
            continue;
        }

        // A region with a hole needs a face with a hole, and this mesh cannot
        // hold one. It can hold two faces that share two edges, though, so cut
        // the ring across in two places. That is two edges where the surface
        // does not really turn -- against the dozens the fan had -- and it is
        // the face a slot or a drilled hole leaves behind, which is the case
        // that matters.
        //
        // Anything more tangled than one hole is left alone rather than guessed
        // at.
        Vec3 pu, pv;
        planeBasis(g.normal, pu, pv);
        auto flat = [&](Index vtx) {
            const Vec3 p = mesh.verts[vtx].position;
            return Vec2{dot(p, pu), dot(p, pv)};
        };
        auto areaOf = [&](const std::vector<Index>& l) {
            Real s = 0.0;
            for (size_t i = 0; i < l.size(); ++i) {
                const Vec2 a = flat(l[i]), b = flat(l[(i + 1) % l.size()]);
                s += a.x * b.y - b.x * a.y;
            }
            return s * 0.5;
        };

        int outerAt = -1, holeAt = -1;
        bool tangled = loops.size() != 2;
        for (size_t i = 0; !tangled && i < loops.size(); ++i)
            ((areaOf(loops[i]) > 0.0) ? outerAt : holeAt) = static_cast<int>(i);
        if (tangled || outerAt < 0 || holeAt < 0) { keepOriginals(); continue; }

        const std::vector<Index>& O = loops[outerAt];
        const std::vector<Index>& H = loops[holeAt];

        // Two cuts, from opposite sides of the hole to whichever outer vertex
        // is nearest. Opposite sides so the two halves are both substantial
        // rather than one being a sliver.
        auto nearestOuter = [&](Index h) {
            size_t best2 = 0;
            Real bd = 1e300;
            for (size_t i = 0; i < O.size(); ++i) {
                const Real d = lengthSq(mesh.verts[O[i]].position - mesh.verts[h].position);
                if (d < bd) { bd = d; best2 = i; }
            }
            return best2;
        };
        const size_t hi0 = 0, hi1 = H.size() / 2;
        const size_t oi0 = nearestOuter(H[hi0]), oi1 = nearestOuter(H[hi1]);

        if (H.size() < 3 || O.size() < 3 || hi0 == hi1 || oi0 == oi1) {
            keepOriginals();
            continue;
        }

        auto walk = [](const std::vector<Index>& l, size_t from, size_t to) {
            std::vector<Index> out;
            size_t i = from;
            for (;;) {
                out.push_back(l[i]);
                if (i == to) break;
                i = (i + 1) % l.size();
            }
            return out;
        };

        std::vector<Index> p1 = walk(O, oi0, oi1);
        for (Index x : walk(H, hi1, hi0)) p1.push_back(x);
        std::vector<Index> p2 = walk(O, oi1, oi0);
        for (Index x : walk(H, hi0, hi1)) p2.push_back(x);

        // The cuts must not cross the outline or each other, or the two halves
        // would overlap. Cheaper to check than to repair.
        auto simple = [&](const std::vector<Index>& l) {
            const size_t n = l.size();
            if (n < 3) return false;
            auto seg = [&](size_t i, Vec2& a, Vec2& b) {
                a = flat(l[i]); b = flat(l[(i + 1) % n]);
            };
            for (size_t i = 0; i < n; ++i) {
                Vec2 a, b; seg(i, a, b);
                for (size_t j = i + 1; j < n; ++j) {
                    if (j == i || (j + 1) % n == i || (i + 1) % n == j) continue;
                    Vec2 c, d; seg(j, c, d);
                    auto side = [](Vec2 p, Vec2 q, Vec2 r) {
                        return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
                    };
                    const Real d1 = side(a, b, c), d2 = side(a, b, d);
                    const Real d3 = side(c, d, a), d4 = side(c, d, b);
                    if (((d1 > 1e-12 && d2 < -1e-12) || (d1 < -1e-12 && d2 > 1e-12)) &&
                        ((d3 > 1e-12 && d4 < -1e-12) || (d3 < -1e-12 && d4 > 1e-12)))
                        return false;
                }
            }
            return true;
        };

        if (!simple(p1) || !simple(p2)) { keepOriginals(); continue; }

        // Both halves need a name. The second used to go out without one, and
        // a nameless face is worse than an unnamed one: every reference to it
        // stores nothing, and nothing matches the first nameless face in the
        // mesh. Picking the bored top of a cut body and extruding moved some
        // other face entirely.
        soup.face(asSoup(p1), mesh.faces[best].id);
        soup.face(asSoup(p2), nameId(0, IdRole::Split, mesh.faces[best].id));
        for (Index f : g.faces) emitted[f] = true;
        merged += static_cast<int>(g.faces.size()) - 2;
    }

    // Faces too small to have a reliable plane were never grouped.
    for (Index f = 0; f < mesh.faceCount(); ++f)
        if (!emitted[f]) soup.face(faceLoop(mesh, f), mesh.faces[f].id);

    if (std::getenv("TANGENT_MERGE_DEBUG"))
        std::fprintf(stderr, "[merge] %d faces in %zu planes -> %zu faces\n",
                     mesh.faceCount(), groups.size(), soup.faceSizes.size());
    if (merged == 0) return 0;

    Mesh next = mesh;
    if (!soup.commit(next)) {
        if (std::getenv("TANGENT_MERGE_DEBUG"))
            std::fprintf(stderr, "[merge] rebuild REFUSED the merged soup\n");
        return 0;   // leave the mesh alone rather than risk it
    }
    mesh = std::move(next);
    return merged;
}

bool bevelAllEdges(Mesh& mesh, Real width, int segments) {
    std::vector<Index> all;
    all.reserve(static_cast<size_t>(mesh.halfedgeCount()) / 2);
    for (Index h = 0; h < mesh.halfedgeCount(); ++h)
        if (edgeOf(mesh, h) == h) all.push_back(h);
    return bevelEdges(mesh, all, width, segments);
}

} // namespace tg
