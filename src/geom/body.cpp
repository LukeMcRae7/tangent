#include "geom/body.h"

namespace tg {

// An edge is named by the lower of its two half-edges. That is a mesh-backend
// detail and it stops here: every handle this file hands out is already
// canonical, so no caller ever has to know the rule or apply it.
static inline EdgeId canonical(const Mesh& m, Index he) {
    const Index tw = m.halfedges[he].twin;
    return tw == kInvalid ? he : std::min(he, tw);
}

bool Body::hasEdge(EdgeId e) const {
    if (brep_) return brep::hasEdge(*brep_, e);
    // In range, and canonical: a handle that names an edge by its other
    // half-edge is not one this Body ever handed out.
    return e >= 0 && e < mesh_.halfedgeCount() && canonical(mesh_, e) == e;
}

int Body::edgeCount() const {
    if (brep_) return brep::edgeCount(*brep_);
    int n = 0;
    for (Index h = 0; h < mesh_.halfedgeCount(); ++h)
        if (canonical(mesh_, h) == h) ++n;
    return n;
}

void Body::allFaces(std::vector<FaceId>& out) const {
    if (brep_) { brep::allFaces(*brep_, out); return; }
    out.clear();
    out.reserve(static_cast<size_t>(mesh_.faceCount()));
    for (Index f = 0; f < mesh_.faceCount(); ++f) out.push_back(f);
}

void Body::allEdges(std::vector<EdgeId>& out) const {
    if (brep_) { brep::allEdges(*brep_, out); return; }
    out.clear();
    out.reserve(static_cast<size_t>(mesh_.halfedgeCount()) / 2);
    for (Index h = 0; h < mesh_.halfedgeCount(); ++h)
        if (canonical(mesh_, h) == h) out.push_back(h);
}

void Body::allVertices(std::vector<VertexId>& out) const {
    if (brep_) { brep::allVertices(*brep_, out); return; }
    out.clear();
    out.reserve(static_cast<size_t>(mesh_.vertexCount()));
    for (Index v = 0; v < mesh_.vertexCount(); ++v) out.push_back(v);
}

void Body::faceEdges(FaceId f, std::vector<EdgeId>& out) const {
    if (brep_) { brep::faceEdges(*brep_, f, out); return; }
    out.clear();
    if (f < 0 || f >= mesh_.faceCount()) return;
    const Index start = mesh_.faces[f].halfedge;
    if (start == kInvalid) return;
    Index h = start;
    do {
        out.push_back(canonical(mesh_, h));
        h = mesh_.halfedges[h].next;
    } while (h != start && h != kInvalid);
}

void Body::faceVertices(FaceId f, std::vector<VertexId>& out) const {
    if (brep_) { brep::faceVertices(*brep_, f, out); return; }
    out.clear();
    if (f < 0 || f >= mesh_.faceCount()) return;
    mesh_.faceVertices(f, out);
}

void Body::edgeEnds(EdgeId e, VertexId& a, VertexId& b) const {
    if (brep_) { brep::edgeEnds(*brep_, e, a, b); return; }
    a = b = kInvalid;
    if (e < 0 || e >= mesh_.halfedgeCount()) return;
    a = mesh_.fromVertex(e);
    b = mesh_.halfedges[e].vertex;
}

void Body::edgeFaces(EdgeId e, FaceId& a, FaceId& b) const {
    if (brep_) { brep::edgeFaces(*brep_, e, a, b); return; }
    a = b = kNoFace;
    if (e < 0 || e >= mesh_.halfedgeCount()) return;
    a = mesh_.halfedges[e].face;
    const Index tw = mesh_.halfedges[e].twin;
    b = tw == kInvalid ? kNoFace : mesh_.halfedges[tw].face;
}

void Body::vertexEdges(VertexId v, std::vector<EdgeId>& out) const {
    if (brep_) { brep::vertexEdges(*brep_, v, out); return; }
    out.clear();
    if (v < 0 || v >= mesh_.vertexCount()) return;
    const Index start = mesh_.verts[v].halfedge;
    if (start == kInvalid) return;
    // Circulate the fan. On an open surface verts[v].halfedge is the boundary
    // half-edge, which is what makes one lap reach the whole of it.
    Index h = start;
    do {
        const EdgeId e = canonical(mesh_, h);
        bool seen = false;
        for (EdgeId x : out)
            if (x == e) { seen = true; break; }
        if (!seen) out.push_back(e);
        const Index tw = mesh_.halfedges[h].twin;
        if (tw == kInvalid) break;
        h = mesh_.halfedges[tw].next;
    } while (h != start && h != kInvalid);
}

void Body::edgePositions(EdgeId e, Vec3& a, Vec3& b) const {
    if (brep_) { brep::edgePositions(*brep_, e, a, b); return; }
    VertexId va = kInvalid, vb = kInvalid;
    edgeEnds(e, va, vb);
    a = va != kInvalid ? mesh_.verts[va].position : Vec3{};
    b = vb != kInvalid ? mesh_.verts[vb].position : Vec3{};
}

Vec3 Body::edgeDirection(EdgeId e) const {
    if (brep_) return brep::edgeDirection(*brep_, e);
    Vec3 a{}, b{};
    edgePositions(e, a, b);
    return normalize(b - a);
}

void Body::transform(const Mat4& m) {
    if (brep_) {
        // A new shape rather than an edit: the old one may be held by a feature
        // cache, an undo entry, or another Body that copied this one.
        brep_ = brep::transformed(*brep_, m);
        return;
    }
    for (MeshVertex& v : mesh_.verts) v.position = transformPoint(m, v.position);
}

void Body::coplanarFaceGroup(FaceId f, std::vector<FaceId>& out) const {
    if (brep_) {
        // A B-rep face is already the whole flat region: there is nothing to
        // group, and this is the artefact the seam exists to hide.
        out.clear();
        if (brep::hasFace(*brep_, f)) out.push_back(f);
        return;
    }
    mesh_.coplanarFaceGroup(f, out);
}

void Body::findFaces(ElementId id, std::vector<FaceId>& out) const {
    if (brep_) { brep::findFaces(*brep_, id, out); return; }
    out.clear();
    // The mesh backend gives every face its own name, so this is the one-or-
    // none case -- but callers are written against the set either way.
    const FaceId f = mesh_.findFace(id);
    if (f != kInvalid) out.push_back(f);
}

EdgeId Body::findEdge(ElementId id) const {
    if (brep_) return brep::findEdge(*brep_, id);
    const Index he = mesh_.findEdge(id);
    return he == kInvalid ? kInvalid : canonical(mesh_, he);
}

} // namespace tg
