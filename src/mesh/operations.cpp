#include "mesh/operations.h"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

namespace tg {
namespace {

// Polygon soup under construction, with the vertex compaction every operation
// needs at the end (faces get replaced, orphaning their old vertices).
struct Soup {
    std::vector<Vec3>     positions;
    std::vector<uint32_t> faceSizes;
    std::vector<uint32_t> faceIndices;

    uint32_t vertex(Vec3 p) {
        positions.push_back(p);
        return static_cast<uint32_t>(positions.size() - 1);
    }
    void face(const std::vector<uint32_t>& loop) {
        if (loop.size() < 3) return;
        faceSizes.push_back(static_cast<uint32_t>(loop.size()));
        faceIndices.insert(faceIndices.end(), loop.begin(), loop.end());
    }

    // Drops vertices no face references and renumbers the rest.
    void compact() {
        std::vector<int32_t> remap(positions.size(), -1);
        std::vector<Vec3> kept;
        kept.reserve(positions.size());
        for (uint32_t& idx : faceIndices) {
            if (remap[idx] < 0) {
                remap[idx] = static_cast<int32_t>(kept.size());
                kept.push_back(positions[idx]);
            }
            idx = static_cast<uint32_t>(remap[idx]);
        }
        positions.swap(kept);
    }

    bool commit(Mesh& mesh) {
        compact();
        Mesh next;
        if (!next.build(positions, faceSizes, faceIndices)) return false;
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
bool insetPolygon(const std::vector<Vec3>& poly, Vec3 normal, float amount,
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

        const float denom = 1.0f + dot(n0, n1);
        if (denom < 1e-4f) return false;    // edges double back; no finite offset
        out[i] = cur + (n0 + n1) * (amount / denom);
    }
    return true;
}

// Signed area of a polygon about its own normal.
float signedArea(const std::vector<Vec3>& poly, Vec3 normal) {
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

// ---------------------------------------------------------------------------
bool extrudeFaces(Mesh& mesh, const std::vector<Index>& faces, float distance,
                  std::vector<Index>* newFaces) {
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

    Soup soup;
    soup.positions.reserve(mesh.verts.size() * 2);
    for (const MeshVertex& v : mesh.verts) soup.vertex(v.position);

    // One raised copy per vertex touched by the region. Vertices used *only*
    // by region faces are left behind and compaction removes them.
    std::unordered_map<Index, uint32_t> raised;
    for (Index f : region) {
        std::vector<Index> verts;
        mesh.faceVertices(f, verts);
        for (Index v : verts)
            if (!raised.count(v)) raised[v] = soup.vertex(mesh.verts[v].position + offset);
    }

    // Faces outside the region are carried over untouched.
    for (Index f = 0; f < mesh.faceCount(); ++f)
        if (!region.count(f)) soup.face(faceLoop(mesh, f));

    const size_t firstMoved = soup.faceSizes.size();

    // The region itself, lifted.
    for (Index f : faces) {
        std::vector<Index> verts;
        mesh.faceVertices(f, verts);
        std::vector<uint32_t> loop;
        loop.reserve(verts.size());
        for (Index v : verts) loop.push_back(raised[v]);
        soup.face(loop);
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
                           raised[v1], raised[v0]});
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
bool insetFaces(Mesh& mesh, const std::vector<Index>& faces, float amount,
                std::vector<Index>* newFaces) {
    if (faces.empty() || mesh.empty() || amount <= 0.0f) return false;

    std::unordered_set<Index> region(faces.begin(), faces.end());
    for (Index f : region)
        if (f < 0 || f >= mesh.faceCount()) return false;

    Soup soup;
    for (const MeshVertex& v : mesh.verts) soup.vertex(v.position);

    for (Index f = 0; f < mesh.faceCount(); ++f)
        if (!region.count(f)) soup.face(faceLoop(mesh, f));

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
        for (const Vec3& p : inner) innerIdx.push_back(soup.vertex(p));

        innerAt.push_back(soup.faceSizes.size());
        soup.face(innerIdx);

        // Rim: (v_i, v_i+1, inner_i+1, inner_i) carries the face's own normal.
        const size_t n = verts.size();
        for (size_t i = 0; i < n; ++i) {
            const size_t j = (i + 1) % n;
            soup.face({static_cast<uint32_t>(verts[i]), static_cast<uint32_t>(verts[j]),
                       innerIdx[j], innerIdx[i]});
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
float maxBevelWidth(const Mesh& mesh) {
    // The binding constraint is the face that runs out of room first. Bisect
    // on "does every face still inset to a positive area", which is monotone.
    float lo = 0.0f, hi = 0.0f;
    for (Index f = 0; f < mesh.faceCount(); ++f) hi = std::max(hi, mesh.faceArea(f));
    hi = std::sqrt(std::max(hi, 1e-6f));

    auto fits = [&](float w) {
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
            const float mid = (lo + hi) * 0.5f;
            if (fits(mid)) lo = mid; else hi = mid;
        }
        return lo;
    }
    return hi;
}

// ---------------------------------------------------------------------------
namespace {

// One flat chamfer pass: every face shrinks inward by `width`, every edge
// becomes a quad spanning the two shrunken faces, and every vertex becomes a
// face closing the corner. (This is the Conway truncation operator.)
bool chamferOnce(Mesh& mesh, float width) {
    if (mesh.empty() || width <= 0.0f) return false;

    const Index faceCount = mesh.faceCount();
    const Index heCount = mesh.halfedgeCount();

    // Open surfaces have boundary half-edges with no second face to chamfer
    // against, so there is no well-defined corner to cut.
    for (Index h = 0; h < heCount; ++h)
        if (mesh.halfedges[h].face == kInvalid) return false;

    Soup soup;

    // corner[h] is the inset vertex at fromVertex(h) within face(h). All three
    // families of output face are expressed in terms of those corners.
    std::vector<uint32_t> corner(static_cast<size_t>(heCount), 0u);

    for (Index f = 0; f < faceCount; ++f) {
        std::vector<Index> verts;
        mesh.faceVertices(f, verts);

        std::vector<Vec3> poly;
        poly.reserve(verts.size());
        for (Index v : verts) poly.push_back(mesh.verts[v].position);

        const Vec3 nrm = mesh.faceNormal(f);
        std::vector<Vec3> inner;
        if (!insetPolygon(poly, nrm, width, inner)) return false;
        if (!insetIsValid(poly, inner, nrm)) return false;

        std::vector<uint32_t> loop;
        loop.reserve(inner.size());
        for (const Vec3& p : inner) loop.push_back(soup.vertex(p));

        size_t k = 0;
        const Index start = mesh.faces[f].halfedge;
        Index h = start;
        do { corner[h] = loop[k++]; h = mesh.halfedges[h].next; } while (h != start);

        soup.face(loop);
    }

    // One quad per edge, emitted once per twin pair. Running from the twin's
    // corners to this half-edge's corners is the order that winds it outward.
    for (Index h = 0; h < heCount; ++h) {
        const Index tw = mesh.halfedges[h].twin;
        if (tw < h) continue;
        soup.face({corner[mesh.halfedges[tw].next], corner[tw],
                   corner[mesh.halfedges[h].next],  corner[h]});
    }

    // One face per original vertex. Circulating outgoing half-edges visits the
    // surrounding faces in order; the loop needs reversing to wind outward.
    for (Index v = 0; v < mesh.vertexCount(); ++v) {
        const Index start = mesh.verts[v].halfedge;
        if (start == kInvalid) continue;

        std::vector<uint32_t> loop;
        Index h = start;
        do {
            loop.push_back(corner[h]);
            h = mesh.halfedges[mesh.halfedges[h].twin].next;
        } while (h != start && loop.size() < 256);

        std::reverse(loop.begin(), loop.end());
        soup.face(loop);
    }

    return soup.commit(mesh);
}

} // namespace

bool bevelAllEdges(Mesh& mesh, float width, int segments) {
    if (mesh.empty() || width <= 0.0f || segments < 1) return false;

    // Work on a copy so a failure part-way through leaves the caller's mesh
    // exactly as it was.
    Mesh work = mesh;
    if (!chamferOnce(work, width)) return false;

    // Rounding by subdividing the edge strip does not work: the strip can be
    // cut into an arc easily enough, but the vertex corners would then need
    // spherical patches to meet it, and without them the surface is left with
    // holes. Chamfering the chamfer instead rounds edges *and* corners, and
    // every pass is a complete, validated manifold operation, so the result
    // cannot be malformed.
    //
    // The trade-off is that `width` is the first cut's width rather than an
    // exact fillet radius, so this approximates a fillet rather than producing
    // one analytically.
    float w = width * 0.42f;
    for (int i = 1; i < segments; ++i) {
        // Never cut deeper than the new, smaller faces can take.
        const float limit = maxBevelWidth(work) * 0.7f;
        const float step = std::min(w, limit);
        if (step <= 1e-5f) break;
        // Stop refining rather than fail: the caller asked for a rounder edge
        // and a slightly less round one is a better answer than none.
        if (!chamferOnce(work, step)) break;
        w *= 0.42f;
    }

    mesh = std::move(work);
    return true;
}

} // namespace tg
