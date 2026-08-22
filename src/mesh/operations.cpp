#include "mesh/operations.h"

#include <algorithm>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <unordered_map>
#include <utility>
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

// ---------------------------------------------------------------------------
bool extrudeFaces(Mesh& mesh, const std::vector<Index>& faces, Real distance,
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
bool insetFaces(Mesh& mesh, const std::vector<Index>& faces, Real amount,
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
        for (const MeshVertex& v : mesh.verts) soup.vertex(v.position);
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

bool bevelEdges(Mesh& mesh, const std::vector<Index>& edges, Real width, int segments) {
    const bool dbg = std::getenv("TANGENT_BEVEL_DEBUG") != nullptr;
    auto bail = [&](const char* why) { if (dbg) std::fprintf(stderr, "[bevel] %s\n", why); return false; };
    if (mesh.empty() || width <= 0.0 || segments < 1) return bail("bad arguments");

    const Index heCount = mesh.halfedgeCount();

    // An open boundary has no second face to bevel against.
    for (Index h = 0; h < heCount; ++h)
        if (mesh.halfedges[h].face == kInvalid) return bail("open surface");

    std::vector<bool> beveled(static_cast<size_t>(heCount), false);
    size_t chosen = 0;
    for (Index e : edges) {
        if (e < 0 || e >= heCount) return bail("edge index out of range");
        const Index tw = mesh.halfedges[e].twin;
        if (!beveled[e]) ++chosen;
        beveled[e] = true;
        beveled[tw] = true;
    }
    if (chosen == 0) return bail("no edges chosen");

    std::vector<Vec3> faceNormals(static_cast<size_t>(mesh.faceCount()));
    for (Index f = 0; f < mesh.faceCount(); ++f) faceNormals[f] = mesh.faceNormal(f);

    auto posOf = [&](Index v) { return mesh.verts[v].position; };
    auto dirOf = [&](Index h) {
        return posOf(mesh.halfedges[h].vertex) - posOf(mesh.fromVertex(h));
    };

    // The displacement a face's corner takes at the start of half-edge h.
    auto cornerAt = [&](Index h, Vec3& d) {
        const Index p = mesh.halfedges[h].prev;
        const Vec3 v = posOf(mesh.fromVertex(h));
        return cornerOffset(faceNormals[mesh.halfedges[h].face],
                            v - posOf(mesh.fromVertex(p)), dirOf(h),
                            beveled[p] ? width : 0.0,
                            beveled[h] ? width : 0.0, d);
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

        // The face on this side: its other edge at the vertex is prev(h).
        if (beveled[mesh.halfedges[h].prev]) {
            Vec3 d;
            if (!cornerAt(h, d)) return bail("corner offset is undefined");
            t = std::max(t, dot(d, dir));
        }
        // The face across. Its corner at this vertex sits between the twin of
        // h and the half-edge after it, so *that* is its other edge -- not h's
        // own previous edge, and not this edge itself.
        const Index across = mesh.halfedges[mesh.halfedges[h].twin].next;
        if (beveled[across]) {
            Vec3 d;
            if (!cornerAt(across, d)) return bail("corner offset is undefined");
            // Projected onto this edge, which is the direction the corner is
            // free to slide along.
            t = std::max(t, dot(d, dir));
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
        if (!baseUsed[v]) { baseIdx[v] = soup.vertex(posOf(v)); baseUsed[v] = true; }
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
                    : soup.vertex(posOf(v) + normalize(dirOf(h)) * cutBack[h]);
        sharedSet[h] = true;
        return sharedPt[h];
    };

    // corner[h] is the point face(h) contributes on the *next* side of its
    // corner at fromVertex(h) -- the one the strip along edge(h) attaches to.
    std::vector<uint32_t> corner(static_cast<size_t>(heCount), 0u);
    std::vector<Vec3> cornerPos(static_cast<size_t>(heCount));

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

            if (bevPrev && bevNext) {
                Vec3 d;
                if (!cornerAt(h, d)) return bail("corner offset is undefined");
                const Vec3 pt = posOf(v) + d;
                const uint32_t idx = soup.vertex(pt);
                loop.push_back(idx);
                corner[h] = idx;
                cornerPos[h] = pt;
                moved.push_back(pt);
            } else if (bevPrev) {
                // Cut back along the next edge only.
                const uint32_t idx = edgePoint(h);
                loop.push_back(idx);
                corner[h] = idx;
                cornerPos[h] = soup.positions[idx];
                moved.push_back(cornerPos[h]);
            } else if (bevNext) {
                // Cut back along the previous edge, which runs from v toward
                // the previous vertex -- that is the twin of prev.
                const uint32_t idx = edgePoint(mesh.halfedges[p].twin);
                loop.push_back(idx);
                corner[h] = idx;
                cornerPos[h] = soup.positions[idx];
                moved.push_back(cornerPos[h]);
            } else {
                // Neither edge is beveled. The corner still splits if a bevel
                // further round the vertex pulled either edge back.
                const uint32_t a = edgePoint(mesh.halfedges[p].twin);
                const uint32_t b = edgePoint(h);
                loop.push_back(a);
                if (b != a) loop.push_back(b);
                corner[h] = b;
                cornerPos[h] = soup.positions[b];
                moved.push_back(soup.positions[a]);
            }
            h = mesh.halfedges[h].next;
        } while (h != start);

        if (loop.size() < 3) return bail("face collapsed");
        if (!insetIsValid(poly, moved, faceNormals[f]))
            return bail("face inverts at this width");

        soup.face(loop);
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

        // At fromVertex(h): face(h)'s corner is corner[h]; the other face's
        // corner at the same vertex is corner[next(twin)].
        const Vec3 p0 = cornerPos[h];
        const Vec3 p1 = cornerPos[mesh.halfedges[tw].next];
        const Vec3 axis = normalize(mesh.verts[mesh.halfedges[h].vertex].position -
                                    mesh.verts[mesh.fromVertex(h)].position);

        const Arc arc = solveArc(p0, p1, -faceNormals[f0], -faceNormals[f1]);
        arcAt[h] = arc;

        std::vector<Vec3> pts;
        sampleArc(arc, p0, p1, axis, segments, pts);

        std::vector<uint32_t>& ring = ringAt[h];
        ring.push_back(corner[h]);
        for (size_t i = 1; i + 1 < pts.size(); ++i) ring.push_back(soup.vertex(pts[i]));
        ring.push_back(corner[mesh.halfedges[tw].next]);
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
            soup.face({a[s + 1], b[s + 1], b[s], a[s]});
    }

    // Vertex patches, wherever at least one incident edge was beveled.
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

        // Walk the outgoing half-edges. Each face contributes its corner --
        // which may be two points where it split -- and each beveled edge
        // contributes the arc crossing it, which the strip already shares.
        std::vector<uint32_t> loop;
        std::vector<Vec3> loopPos;
        std::vector<Vec3> centres;

        auto push = [&](uint32_t idx) {
            if (!loop.empty() && loop.back() == idx) return;   // corners can coincide
            loop.push_back(idx);
            loopPos.push_back(soup.positions[idx]);
        };

        h = start;
        do {
            const Index p = mesh.halfedges[h].prev;
            if (!beveled[p]) push(edgePoint(mesh.halfedges[p].twin));
            push(corner[h]);
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
        if (loop.size() < 3) continue;

        if (loop.size() <= 4 || centres.empty()) {
            soup.face(loop);
            continue;
        }

        // A rounded corner's patch is not planar, and fanning a non-planar
        // n-gon from one of its own corners gives a lopsided, sometimes folded
        // result. Add a centre that sits on the same sphere the edge arcs do,
        // and fan from that instead.
        Vec3 c{};
        for (const Vec3& p : centres) c += p;
        c = c / static_cast<Real>(centres.size());

        Vec3 ringMid{};
        Real radius = 0.0;
        for (const Vec3& p : loopPos) { ringMid += p; radius += length(p - c); }
        ringMid = ringMid / static_cast<Real>(loopPos.size());
        radius /= static_cast<Real>(loopPos.size());

        const Vec3 dir = ringMid - c;
        if (lengthSq(dir) < 1e-18) { soup.face(loop); continue; }

        const uint32_t hub = soup.vertex(c + normalize(dir) * radius);
        for (size_t i = 0; i < loop.size(); ++i)
            soup.face({hub, loop[i], loop[(i + 1) % loop.size()]});
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

bool bevelAllEdges(Mesh& mesh, Real width, int segments) {
    std::vector<Index> all;
    all.reserve(static_cast<size_t>(mesh.halfedgeCount()) / 2);
    for (Index h = 0; h < mesh.halfedgeCount(); ++h)
        if (edgeOf(mesh, h) == h) all.push_back(h);
    return bevelEdges(mesh, all, width, segments);
}

} // namespace tg
