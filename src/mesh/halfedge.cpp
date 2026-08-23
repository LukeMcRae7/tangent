#include "mesh/halfedge.h"

#include <cassert>
#include <cstdio>
#include <unordered_set>
#include <unordered_map>
#include <vector>

namespace tg {
namespace {

} // namespace

// ---------------------------------------------------------------------------
bool Mesh::build(const std::vector<Vec3>& positions,
                 const std::vector<uint32_t>& faceSizes,
                 const std::vector<uint32_t>& faceIndices,
                 const Names* names) {
    clear();

    // Names are optional and all-or-nothing per array: a caller that supplies
    // the wrong length is telling us something is out of step, and silently
    // half-naming a mesh would be worse than not naming it.
    const bool haveVertexNames = names && names->vertices.size() == positions.size();
    const bool haveFaceNames   = names && names->faces.size() == faceSizes.size();

    verts.resize(positions.size());
    for (size_t i = 0; i < positions.size(); ++i) {
        verts[i].position = positions[i];
        if (haveVertexNames) verts[i].id = names->vertices[i];
    }

    const Index nVerts = static_cast<Index>(positions.size());
    faces.reserve(faceSizes.size());

    // Twin pairing runs off per-vertex buckets rather than a hash of the whole
    // edge set. Hashing 200k directed edges dominated build time, and build
    // runs inside every mesh operation; bucketing is a linear pass plus a scan
    // of one vertex's handful of neighbours.
    std::vector<Index> heFrom;
    heFrom.reserve(faceIndices.size());

    size_t cursor = 0;
    for (uint32_t degree : faceSizes) {
        if (degree < 3 || cursor + degree > faceIndices.size()) { clear(); return false; }

        const Index faceId  = static_cast<Index>(faces.size());
        const Index firstHe = static_cast<Index>(halfedges.size());

        for (uint32_t k = 0; k < degree; ++k) {
            const Index from = static_cast<Index>(faceIndices[cursor + k]);
            const Index to   = static_cast<Index>(faceIndices[cursor + (k + 1) % degree]);
            if (from < 0 || from >= nVerts || to < 0 || to >= nVerts || from == to) {
                clear(); return false;
            }

            const Index he = static_cast<Index>(halfedges.size());
            HalfEdge h;
            h.vertex = to;
            h.face   = faceId;
            h.next   = firstHe + static_cast<Index>((k + 1) % degree);
            h.prev   = firstHe + static_cast<Index>((k + degree - 1) % degree);
            halfedges.push_back(h);
            heFrom.push_back(from);

            // Provisional; boundary half-edges take priority in the pass below.
            if (verts[from].halfedge == kInvalid) verts[from].halfedge = he;
        }

        faces.push_back(MeshFace{firstHe, haveFaceNames ? names->faces[faces.size()] : kNoId});
        cursor += degree;
    }
    if (cursor != faceIndices.size()) { clear(); return false; }

    const Index interiorCount = static_cast<Index>(halfedges.size());

    edgeIds.assign(halfedges.size(), kNoId);

    // Bucket half-edges by their originating vertex (counting sort into CSR).
    std::vector<Index> bucketStart(static_cast<size_t>(nVerts) + 1, 0);
    for (Index he = 0; he < interiorCount; ++he) ++bucketStart[heFrom[he] + 1];
    for (Index v = 0; v < nVerts; ++v) bucketStart[v + 1] += bucketStart[v];

    std::vector<Index> bucket(static_cast<size_t>(interiorCount));
    {
        std::vector<Index> fill(bucketStart.begin(), bucketStart.end() - 1);
        for (Index he = 0; he < interiorCount; ++he) bucket[fill[heFrom[he]]++] = he;
    }

    // Two faces walking the same directed edge means the surface is
    // non-manifold or inconsistently wound.
    for (Index v = 0; v < nVerts; ++v) {
        for (Index i = bucketStart[v]; i < bucketStart[v + 1]; ++i)
            for (Index j = i + 1; j < bucketStart[v + 1]; ++j)
                if (halfedges[bucket[i]].vertex == halfedges[bucket[j]].vertex) {
                    clear(); return false;
                }
    }

    // Pair interior twins: for u -> v, look through v's bucket for v -> u.
    for (Index he = 0; he < interiorCount; ++he) {
        if (halfedges[he].twin != kInvalid) continue;
        const Index from = heFrom[he];
        const Index to   = halfedges[he].vertex;
        for (Index i = bucketStart[to]; i < bucketStart[to + 1]; ++i) {
            const Index cand = bucket[i];
            if (halfedges[cand].vertex != from) continue;
            halfedges[he].twin = cand;
            halfedges[cand].twin = he;
            break;
        }
    }

    // Every unpaired half-edge gets a boundary partner (face == kInvalid),
    // so twin() is total and circulators never need a null check.
    std::unordered_map<Index, Index> boundaryFrom;  // start vertex -> boundary he
    for (Index he = 0; he < interiorCount; ++he) {
        if (halfedges[he].twin != kInvalid) continue;
        const Index b = static_cast<Index>(halfedges.size());
        HalfEdge h;
        h.vertex = fromVertex(he);   // boundary runs opposite the interior edge
        h.face   = kInvalid;
        h.twin   = he;
        halfedges.push_back(h);
        edgeIds.push_back(kNoId);
        halfedges[he].twin = b;

        const Index start = halfedges[he].vertex;  // b originates here
        if (!boundaryFrom.emplace(start, b).second) { clear(); return false; }
    }

    // Stitch boundary loops: the successor of a boundary half-edge is the
    // unique boundary half-edge leaving the vertex it points at.
    for (auto& [start, b] : boundaryFrom) {
        (void)start;
        auto it = boundaryFrom.find(halfedges[b].vertex);
        if (it == boundaryFrom.end()) { clear(); return false; }
        halfedges[b].next = it->second;
        halfedges[it->second].prev = b;
    }

    // Prefer a boundary-adjacent outgoing half-edge so one full circulation
    // from verts[v].halfedge visits the entire fan on open surfaces.
    for (auto& [start, b] : boundaryFrom) verts[start].halfedge = b;

    // Reject non-manifold vertices.
    //
    // The duplicate-directed-edge check above catches an edge shared by more
    // than two faces, but not two surface sheets meeting at a single point --
    // two tetrahedra joined at one apex, say. Every directed edge there is
    // still unique. What gives it away is that circulating around the vertex
    // reaches only one of the fans, so the walk visits fewer half-edges than
    // actually start at that vertex. Such a vertex has no single well-defined
    // neighbourhood, and it is unprintable: a slicer cannot decide what is
    // inside at that point.
    {
        std::vector<int> outgoing(verts.size(), 0);
        for (const HalfEdge& h : halfedges) {
            const Index from = halfedges[h.prev].vertex;
            if (from >= 0 && from < static_cast<Index>(verts.size())) ++outgoing[from];
        }
        for (Index v = 0; v < vertexCount(); ++v) {
            const Index start = verts[v].halfedge;
            if (start == kInvalid) continue;
            int reached = 0;
            Index he = start;
            do {
                ++reached;
                he = halfedges[halfedges[he].twin].next;
            } while (he != start && reached <= outgoing[v]);
            if (reached != outgoing[v]) { clear(); return false; }
        }
    }

    nameEdges();
    return true;
}

// ---------------------------------------------------------------------------
int Mesh::faceDegree(Index f) const {
    const Index start = faces[f].halfedge;
    int n = 0;
    Index he = start;
    do { ++n; he = halfedges[he].next; } while (he != start);
    return n;
}

void Mesh::faceVertices(Index f, std::vector<Index>& out) const {
    out.clear();
    const Index start = faces[f].halfedge;
    Index he = start;
    do { out.push_back(fromVertex(he)); he = halfedges[he].next; } while (he != start);
}

// Newell's method: exact for planar polygons and well-behaved for the slightly
// non-planar n-gons that modelling operations tend to produce.
Vec3 Mesh::faceNormal(Index f) const {
    Vec3 n{};
    const Index start = faces[f].halfedge;
    Index he = start;
    do {
        const Vec3& a = verts[fromVertex(he)].position;
        const Vec3& b = verts[halfedges[he].vertex].position;
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
        he = halfedges[he].next;
    } while (he != start);
    return normalize(n);
}

Vec3 Mesh::faceCentroid(Index f) const {
    Vec3 c{};
    int n = 0;
    const Index start = faces[f].halfedge;
    Index he = start;
    do { c += verts[fromVertex(he)].position; ++n; he = halfedges[he].next; } while (he != start);
    return n > 0 ? c / static_cast<Real>(n) : c;
}

Real Mesh::faceArea(Index f) const {
    // Fan the polygon from its first corner; valid for planar polygons and a
    // good approximation otherwise.
    const Index start = faces[f].halfedge;
    const Vec3& a = verts[fromVertex(start)].position;
    Real area = 0.0f;
    Index he = halfedges[start].next;
    while (halfedges[he].next != start) {
        const Vec3& b = verts[fromVertex(he)].position;
        const Vec3& c = verts[halfedges[he].vertex].position;
        area += length(cross(b - a, c - a)) * 0.5f;
        he = halfedges[he].next;
    }
    return area;
}

AABB Mesh::bounds() const {
    AABB b;
    for (const MeshVertex& v : verts) b.expand(v.position);
    return b;
}

// ---------------------------------------------------------------------------
// Ear clipping in the face's own plane. Handles concave polygons, which fan
// triangulation would get wrong once booleans start producing them.
void Mesh::triangulateFace(Index f, std::vector<Index>& out) const {
    std::vector<Index> loop;
    faceVertices(f, loop);
    const int n = static_cast<int>(loop.size());
    if (n < 3) return;

    if (n == 3) { out.insert(out.end(), {0, 1, 2}); return; }

    const Vec3 nrm = faceNormal(f);
    const Vec3 tx  = perpendicular(nrm);
    const Vec3 ty  = cross(nrm, tx);
    const Vec3 org = verts[loop[0]].position;

    std::vector<Vec2> p(n);
    for (int i = 0; i < n; ++i) {
        const Vec3 d = verts[loop[i]].position - org;
        p[i] = {dot(d, tx), dot(d, ty)};
    }

    // Signed area fixes the winding so "convex" below has a consistent sense.
    Real area2 = 0.0f;
    for (int i = 0; i < n; ++i) {
        const Vec2& a = p[i];
        const Vec2& b = p[(i + 1) % n];
        area2 += a.x * b.y - b.x * a.y;
    }

    std::vector<int> idx(n);
    for (int i = 0; i < n; ++i) idx[i] = (area2 < 0.0f) ? (n - 1 - i) : i;

    auto cross2 = [&](const Vec2& a, const Vec2& b, const Vec2& c) {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    };

    std::vector<int> rem = idx;
    int guard = 0;
    while (static_cast<int>(rem.size()) > 3 && guard++ < 4 * n) {
        bool clipped = false;
        const int m = static_cast<int>(rem.size());
        for (int i = 0; i < m; ++i) {
            const int i0 = rem[(i + m - 1) % m], i1 = rem[i], i2 = rem[(i + 1) % m];
            if (cross2(p[i0], p[i1], p[i2]) <= 0.0f) continue;   // reflex corner

            bool contains = false;
            for (int j = 0; j < m && !contains; ++j) {
                const int k = rem[j];
                if (k == i0 || k == i1 || k == i2) continue;
                contains = cross2(p[i0], p[i1], p[k]) >= 0.0f &&
                           cross2(p[i1], p[i2], p[k]) >= 0.0f &&
                           cross2(p[i2], p[i0], p[k]) >= 0.0f;
            }
            if (contains) continue;

            out.insert(out.end(), {i0, i1, i2});
            rem.erase(rem.begin() + i);
            clipped = true;
            break;
        }
        // Self-intersecting or fully degenerate input: fall back to a fan so
        // the face still renders instead of vanishing.
        if (!clipped) break;
    }

    if (rem.size() == 3) {
        out.insert(out.end(), {rem[0], rem[1], rem[2]});
    } else {
        for (size_t i = 1; i + 1 < rem.size(); ++i)
            out.insert(out.end(), {rem[0], rem[i], rem[i + 1]});
    }
}

// ---------------------------------------------------------------------------
void Mesh::buildRenderMesh(RenderMesh& out, Real creaseAngleDeg) const {
    out.clear();
    if (faces.empty()) return;

    const Real creaseCos = std::cos(radians(creaseAngleDeg));

    std::vector<Vec3> faceNormals(faces.size());
    for (size_t f = 0; f < faces.size(); ++f)
        faceNormals[f] = faceNormal(static_cast<Index>(f));

    // Corner-per-half-edge layout: each face owns its own copy of every corner,
    // which is what lets adjacent faces disagree about the normal at a shared
    // vertex (the whole point of crease-angle shading).
    std::vector<uint32_t> cornerOf(halfedges.size(), 0u);

    for (size_t f = 0; f < faces.size(); ++f) {
        const Vec3 fn = faceNormals[f];
        const Index start = faces[f].halfedge;

        Index he = start;
        do {
            const Index v = fromVertex(he);
            cornerOf[he] = static_cast<uint32_t>(out.positions.size());

            // Average in the neighbouring faces that are smooth across their
            // shared edge; the rest keep this face's flat normal.
            Vec3 acc = fn;
            Index cur = he;
            do {
                const Index nf = halfedges[cur].face;
                if (nf != kInvalid && nf != static_cast<Index>(f) &&
                    dot(faceNormals[nf], fn) >= creaseCos) {
                    acc += faceNormals[nf];
                }
                cur = halfedges[halfedges[cur].twin].next;   // next outgoing at v
            } while (cur != he);

            out.positions.push_back(verts[v].position);
            out.normals.push_back(normalize(acc));
            he = halfedges[he].next;
        } while (he != start);
    }

    // Triangles, tagged with their source face for picking.
    std::vector<Index> tri;
    for (size_t f = 0; f < faces.size(); ++f) {
        tri.clear();
        triangulateFace(static_cast<Index>(f), tri);

        // Local corner k of face f lives at cornerOf[k-th half-edge].
        std::vector<uint32_t> base;
        const Index start = faces[f].halfedge;
        Index he = start;
        do { base.push_back(cornerOf[he]); he = halfedges[he].next; } while (he != start);

        for (size_t i = 0; i + 2 < tri.size(); i += 3) {
            out.triangles.push_back(base[tri[i + 0]]);
            out.triangles.push_back(base[tri[i + 1]]);
            out.triangles.push_back(base[tri[i + 2]]);
            out.triangleFace.push_back(static_cast<Index>(f));
        }
    }

    // Wireframe follows polygon edges only, so n-gons do not show their
    // internal triangulation.
    for (Index he = 0; he < static_cast<Index>(halfedges.size()); ++he) {
        if (halfedges[he].face == kInvalid) continue;            // boundary loop
        const Index tw = halfedges[he].twin;
        if (halfedges[tw].face != kInvalid && tw < he) continue;  // emit once
        out.edgeLines.push_back(cornerOf[he]);
        out.edgeLines.push_back(cornerOf[halfedges[he].next]);
    }
}

// ---------------------------------------------------------------------------
Index Mesh::findVertex(ElementId id) const {
    if (id == kNoId) return kInvalid;
    for (Index v = 0; v < vertexCount(); ++v)
        if (verts[v].id == id) return v;
    return kInvalid;
}

Index Mesh::findFace(ElementId id) const {
    if (id == kNoId) return kInvalid;
    for (Index f = 0; f < faceCount(); ++f)
        if (faces[f].id == id) return f;
    return kInvalid;
}

Index Mesh::findEdge(ElementId id) const {
    if (id == kNoId) return kInvalid;
    for (Index he = 0; he < static_cast<Index>(edgeIds.size()); ++he)
        if (edgeIds[he] == id) return he;
    return kInvalid;
}

bool Mesh::named() const {
    if (verts.empty()) return false;
    std::unordered_set<ElementId> seen;
    for (const MeshVertex& v : verts)
        if (v.id == kNoId || !seen.insert(v.id).second) return false;
    seen.clear();
    for (const MeshFace& f : faces)
        if (f.id == kNoId || !seen.insert(f.id).second) return false;
    seen.clear();
    for (Index he = 0; he < static_cast<Index>(edgeIds.size()); ++he) {
        if (he > halfedges[he].twin) continue;
        if (edgeIds[he] == kNoId || !seen.insert(edgeIds[he]).second) return false;
    }
    return true;
}

// An edge takes its name from the two vertices it joins, so it is the same
// name whichever half-edge asks and whichever operation built it.
void Mesh::nameEdges() {
    edgeIds.assign(halfedges.size(), kNoId);
    for (Index he = 0; he < halfedgeCount(); ++he) {
        const ElementId a = verts[fromVertex(he)].id;
        const ElementId b = verts[halfedges[he].vertex].id;
        if (a == kNoId || b == kNoId) continue;
        edgeIds[he] = edgeNameFrom(0, a, b);
    }
}

bool Mesh::validate(std::string* err) const {
    auto fail = [&](const std::string& m) { if (err) *err = m; return false; };
    const Index nh = static_cast<Index>(halfedges.size());

    for (Index he = 0; he < nh; ++he) {
        const HalfEdge& h = halfedges[he];
        if (h.twin < 0 || h.twin >= nh)  return fail("half-edge " + std::to_string(he) + " has no twin");
        if (h.next < 0 || h.next >= nh)  return fail("half-edge " + std::to_string(he) + " has no next");
        if (h.prev < 0 || h.prev >= nh)  return fail("half-edge " + std::to_string(he) + " has no prev");
        if (halfedges[h.twin].twin != he) return fail("twin is not symmetric at " + std::to_string(he));
        if (h.twin == he)                 return fail("half-edge " + std::to_string(he) + " is its own twin");
        if (halfedges[h.next].prev != he) return fail("next/prev disagree at " + std::to_string(he));
        if (h.vertex < 0 || h.vertex >= vertexCount())
            return fail("half-edge " + std::to_string(he) + " has a bad vertex");
        // next must continue from where this half-edge ends.
        if (fromVertex(h.next) != h.vertex)
            return fail("face loop is broken at " + std::to_string(he));
        if (halfedges[h.next].face != h.face)
            return fail("face loop spans two faces at " + std::to_string(he));
    }

    for (Index f = 0; f < faceCount(); ++f) {
        if (faces[f].halfedge < 0 || faces[f].halfedge >= nh) return fail("face has no half-edge");
        if (halfedges[faces[f].halfedge].face != f)           return fail("face/half-edge mismatch");
        if (faceDegree(f) < 3)                                return fail("degenerate face");
    }

    std::vector<int> outgoing(verts.size(), 0);
    for (Index he = 0; he < nh; ++he) ++outgoing[fromVertex(he)];

    for (Index v = 0; v < vertexCount(); ++v) {
        const Index he = verts[v].halfedge;
        if (he == kInvalid) continue;                 // isolated vertex is allowed
        if (he < 0 || he >= nh)     return fail("vertex has a bad half-edge");
        if (fromVertex(he) != v)    return fail("vertex half-edge does not originate there");

        // One circulation must reach every half-edge at the vertex; falling
        // short means two separate fans meet here.
        int reached = 0;
        Index cur = he;
        do {
            ++reached;
            cur = halfedges[halfedges[cur].twin].next;
        } while (cur != he && reached <= outgoing[v]);
        if (reached != outgoing[v])
            return fail("non-manifold vertex " + std::to_string(v));
    }
    return true;
}

} // namespace tg
