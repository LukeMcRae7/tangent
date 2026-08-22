#include "mesh/operations.h"

#include <algorithm>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <limits>
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

    // Where a face's corner sits when *both* its edges there are pulled back.
    //
    // Always the symmetric offset, even when only one of the two edges is
    // actually being filleted. That point is the fillet's true cross-section:
    // offset by the radius from the edge, and set back along it by the same
    // amount, which is where the corner blend takes over from the cylinder.
    // Using the one-sided offset instead leaves the cross-section skewed --
    // its two ends at different positions along the edge -- and because a
    // strip is ruled between its two end sections, that skew corrupts the
    // fillet along the entire edge, not just near the corner.
    auto symmetricAt = [&](Index h, Vec3& d) {
        const Index p = mesh.halfedges[h].prev;
        const Vec3 v = posOf(mesh.fromVertex(h));
        return cornerOffset(faceNormals[mesh.halfedges[h].face],
                            v - posOf(mesh.fromVertex(p)), dirOf(h),
                            width, width, d);
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
            Vec3 d;
            if (!symmetricAt(h, d)) return bail("corner offset is undefined");
            t = std::max(t, dot(d, dir));
        }
        const Index across = mesh.halfedges[mesh.halfedges[h].twin].next;
        if (cornerActive(across)) {
            Vec3 d;
            if (!symmetricAt(across, d)) return bail("corner offset is undefined");
            t = std::max(t, dot(d, dir));
        }
        cutBack[h] = std::max(t, 0.0);
    }

    // A vertex is convex when every neighbour lies on the inner side of every
    // face meeting there. At such a vertex a fillet can only ever remove
    // material, so anything generated near it that ends up outside one of
    // those faces is an artefact and can be pushed back onto the plane. At a
    // concave vertex the opposite is true -- the fillet fills the notch and
    // legitimately lies outside the original surface -- so this must not be
    // applied there.
    std::vector<bool> convexVertex(static_cast<size_t>(mesh.vertexCount()), true);
    for (Index v = 0; v < mesh.vertexCount(); ++v) {
        const Index start = mesh.verts[v].halfedge;
        if (start == kInvalid) continue;
        Index h = start;
        do {
            const Index f = mesh.halfedges[h].face;
            const Vec3 n = faceNormals[f];
            const Vec3 base = posOf(mesh.fromVertex(h));
            Index g = start;
            do {
                if (dot(posOf(mesh.halfedges[g].vertex) - base, n) > 1e-9) {
                    convexVertex[v] = false;
                    break;
                }
                g = mesh.halfedges[mesh.halfedges[g].twin].next;
            } while (g != start);
            if (!convexVertex[v]) break;
            h = mesh.halfedges[mesh.halfedges[h].twin].next;
        } while (h != start);
    }

    auto keepInside = [&](Vec3 p, Index v) {
        if (!convexVertex[v]) return p;
        const Index start = mesh.verts[v].halfedge;
        if (start == kInvalid) return p;
        Index h = start;
        do {
            const Vec3 n = faceNormals[mesh.halfedges[h].face];
            const Real d = dot(p - posOf(v), n);
            if (d > 0.0) p -= n * d;      // back onto the face plane
            h = mesh.halfedges[mesh.halfedges[h].twin].next;
        } while (h != start);
        return p;
    };

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

    // The cross-section point face(h) contributes at fromVertex(h), kept apart
    // from corner[] because a corner can now carry both a cross-section and a
    // cut-back point, and the vertex patch has to walk both.
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
                if (!symmetricAt(h, d)) return bail("corner offset is undefined");
                const Vec3 pt = posOf(v) + d;
                const uint32_t idx = soup.vertex(pt);
                if (loop.empty() || loop.back() != idx) loop.push_back(idx);
                crossIdx[h] = idx;
                crossPos[h] = pt;
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

        const Index atVertex = mesh.fromVertex(h);
        std::vector<uint32_t>& ring = ringAt[h];
        ring.push_back(crossIdx[h]);
        for (size_t i = 1; i + 1 < pts.size(); ++i)
            ring.push_back(soup.vertex(keepInside(pts[i], atVertex)));
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
        if (loop.size() < 3) continue;

        // A chamfer's corner really is a flat facet, so leave it as one
        // polygon. Only a rounded fillet needs a curved patch.
        if (segments == 1 || loop.size() <= 3 || centres.empty()) {
            soup.face(loop);
            continue;
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
        if (degenerate || lengthSq(poleDir) < 1e-18) { soup.face(loop); continue; }
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
                    row.push_back(soup.vertex(keepInside(patchPoint(wa, wb, wg), v)));
                }
            }

            for (int i = 0; i < N; ++i) {
                const std::vector<uint32_t>& up = rows[static_cast<size_t>(i)];
                const std::vector<uint32_t>& lo2 = rows[static_cast<size_t>(i) + 1];
                for (int j = 0; j <= i; ++j) {
                    soup.face({up[static_cast<size_t>(j)], lo2[static_cast<size_t>(j)],
                               lo2[static_cast<size_t>(j) + 1]});
                    if (j < i)
                        soup.face({up[static_cast<size_t>(j)], lo2[static_cast<size_t>(j) + 1],
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
                ring.push_back(soup.vertex(keepInside(p, v)));
            }
            for (size_t i = 0; i < n; ++i) {
                const size_t k = (i + 1) % n;
                soup.face({prev[i], prev[k], ring[k], ring[i]});
            }
            prev.swap(ring);
        }

        const uint32_t pole = soup.vertex(keepInside(poleAt, v));
        for (size_t i = 0; i < prev.size(); ++i)
            soup.face({prev[i], prev[(i + 1) % prev.size()], pole});
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
