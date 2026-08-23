#include "scene/feature.h"

#include "mesh/operations.h"

#include <algorithm>
#include <algorithm>
#include <cstdio>
#include <unordered_map>
#include <unordered_set>

namespace tg {

const char* featureKindName(FeatureKind k) {
    switch (k) {
        case FeatureKind::Primitive:  return "Primitive";
        case FeatureKind::BaseMesh:   return "Mesh";
        case FeatureKind::Boolean:    return "Boolean";
        case FeatureKind::Extrude:    return "Extrude";
        case FeatureKind::Inset:      return "Inset";
        case FeatureKind::Bevel:      return "Bevel";
        case FeatureKind::VertexEdit: return "Edit Vertices";
    }
    return "Feature";
}

std::string Feature::summary() const {
    char buf[96];
    switch (kind) {
        case FeatureKind::Primitive:
            std::snprintf(buf, sizeof(buf), "%s", primitiveName(primitive.kind));
            break;
        case FeatureKind::BaseMesh:
            std::snprintf(buf, sizeof(buf), "Mesh  (%d faces)", bakedMesh.faceCount());
            break;
        case FeatureKind::Boolean:
            std::snprintf(buf, sizeof(buf), "%s  (%d faces)",
                          booleanOpName(booleanOp), bakedMesh.faceCount());
            break;
        case FeatureKind::Extrude:
            std::snprintf(buf, sizeof(buf), "Extrude  %.2f mm  (%s)",
                          static_cast<double>(distance), faces.describe("face").c_str());
            break;
        case FeatureKind::Inset:
            std::snprintf(buf, sizeof(buf), "Inset  %.2f mm  (%s)",
                          static_cast<double>(amount), faces.describe("face").c_str());
            break;
        case FeatureKind::Bevel: {
            // A single segment is a flat cut, which is a chamfer, not a fillet.
            const char* what = segments == 1 ? "Chamfer" : "Fillet";

            // Say so when the edges do not all share a radius, rather than
            // showing one of them as though it applied to the whole feature.
            Real lo = radiusFor(0), hi = lo;
            for (size_t i = 1; i < edges.count(); ++i) {
                lo = std::min(lo, radiusFor(i));
                hi = std::max(hi, radiusFor(i));
            }
            if (hi - lo > 1e-9)
                std::snprintf(buf, sizeof(buf), "%s  %.2f-%.2f mm  x%d  (%s)",
                              what, static_cast<double>(lo), static_cast<double>(hi),
                              segments, edges.describe("edge").c_str());
            else
                std::snprintf(buf, sizeof(buf), "%s  %.2f mm  x%d  (%s)", what,
                              static_cast<double>(lo), segments,
                              edges.describe("edge").c_str());
            break;
        }
        case FeatureKind::VertexEdit:
            std::snprintf(buf, sizeof(buf), "Edit  %zu vert%s", verts.size(),
                          verts.size() == 1 ? "ex" : "ices");
            break;
    }
    return buf;
}

namespace {

bool buildPrimitive(const PrimitiveSpec& spec, Mesh& out) {
    switch (spec.kind) {
        case PrimitiveKind::Box:      return makeBox(out, spec.box);
        case PrimitiveKind::Cylinder: return makeCylinder(out, spec.cylinder);
        case PrimitiveKind::Sphere:   return makeSphere(out, spec.sphere);
        case PrimitiveKind::Cone:     return makeCone(out, spec.cone);
        case PrimitiveKind::Torus:    return makeTorus(out, spec.torus);
        case PrimitiveKind::Plane:    return makePlane(out, spec.plane);
        case PrimitiveKind::Custom:   return false;
    }
    return false;
}

// True if every face index still exists in the mesh the chain has built so far.
} // namespace

ElementRefs nameFaces(const Mesh& mesh, const std::vector<Index>& faces) {
    ElementRefs r;
    for (Index f : faces)
        if (f >= 0 && f < mesh.faceCount()) r.ids.push_back(mesh.faces[f].id);
    return r;
}

ElementRefs nameEdges(const Mesh& mesh, const std::vector<Index>& edges) {
    ElementRefs r;
    if (edges.empty()) return r;

    std::unordered_set<Index> chosen;
    for (Index e : edges) {
        if (e < 0 || e >= mesh.halfedgeCount()) continue;
        chosen.insert(std::min(e, mesh.halfedges[e].twin));
    }
    if (chosen.empty()) return r;

    // Does some face's boundary consist of exactly these edges?
    for (Index e : chosen) {
        for (Index side : {e, mesh.halfedges[e].twin}) {
            const Index f = mesh.halfedges[side].face;
            if (f == kInvalid) continue;
            size_t n = 0;
            bool all = true;
            const Index start = mesh.faces[f].halfedge;
            Index h = start;
            do {
                if (!chosen.count(std::min(h, mesh.halfedges[h].twin))) { all = false; break; }
                ++n;
                h = mesh.halfedges[h].next;
            } while (h != start);

            if (all && n == chosen.size()) {
                r.kind = ElementRefs::Kind::FaceBoundary;
                r.face = mesh.faces[f].id;
                return r;
            }
        }
        break;   // one edge is enough to reach every candidate face
    }

    for (Index e : chosen) r.ids.push_back(mesh.edgeId(e));
    std::sort(r.ids.begin(), r.ids.end());
    return r;
}

std::vector<ElementId> nameVertices(const Mesh& mesh, const std::vector<Index>& verts) {
    std::vector<ElementId> out;
    for (Index v : verts)
        if (v >= 0 && v < mesh.vertexCount()) out.push_back(mesh.verts[v].id);
    return out;
}

std::string ElementRefs::describe(const char* noun) const {
    switch (kind) {
        case Kind::All:          return std::string("every ") + noun;
        case Kind::FaceBoundary: return std::string("a face's ") +
                                        (std::string(noun) == "edge" ? "rim" : "boundary");
        case Kind::Explicit:     break;
    }
    return std::to_string(ids.size()) + " " + noun + (ids.size() == 1 ? "" : "s");
}

bool ElementRefs::resolveFaces(const Mesh& mesh, std::vector<Index>& out) const {
    out.clear();
    if (kind == Kind::All) {
        for (Index f = 0; f < mesh.faceCount(); ++f) out.push_back(f);
        return !out.empty();
    }
    if (kind == Kind::FaceBoundary) {
        const Index f = mesh.findFace(face);
        if (f == kInvalid) return false;
        out.push_back(f);
        return true;
    }
    if (ids.empty()) return false;

    std::unordered_map<ElementId, Index> byName;
    byName.reserve(static_cast<size_t>(mesh.faceCount()));
    for (Index f = 0; f < mesh.faceCount(); ++f) byName.emplace(mesh.faces[f].id, f);
    for (ElementId id : ids) {
        auto it = byName.find(id);
        if (it == byName.end()) return false;
        out.push_back(it->second);
    }
    return true;
}

bool ElementRefs::resolveEdges(const Mesh& mesh, std::vector<Index>& out) const {
    out.clear();
    if (kind == Kind::All) {
        for (Index h = 0; h < mesh.halfedgeCount(); ++h)
            if (h < mesh.halfedges[h].twin) out.push_back(h);
        return !out.empty();
    }
    if (kind == Kind::FaceBoundary) {
        const Index f = mesh.findFace(face);
        if (f == kInvalid) return false;
        const Index start = mesh.faces[f].halfedge;
        Index h = start;
        do {
            out.push_back(std::min(h, mesh.halfedges[h].twin));
            h = mesh.halfedges[h].next;
        } while (h != start);
        return !out.empty();
    }
    if (ids.empty()) return false;

    std::unordered_map<ElementId, Index> byName;
    byName.reserve(static_cast<size_t>(mesh.halfedgeCount()));
    for (Index h = 0; h < mesh.halfedgeCount(); ++h)
        if (h < mesh.halfedges[h].twin) byName.emplace(mesh.edgeId(h), h);
    for (ElementId id : ids) {
        auto it = byName.find(id);
        if (it == byName.end()) return false;
        out.push_back(it->second);
    }
    return true;
}

bool evaluateFrom(std::vector<Feature>& features, size_t from,
                  std::vector<Mesh>& cache, Mesh& out) {
    // The cache must actually hold the requested starting point; anything else
    // (a freshly loaded object, a chain that shrank) means starting over.
    if (from > 0 && (cache.size() < from || from > features.size())) from = 0;

    Mesh mesh;
    if (from > 0) mesh = cache[from - 1];

    cache.resize(features.size());
    bool any = from > 0 || false;

    std::vector<Index> scratchFaces, scratchEdges;

    for (size_t i = from; i < features.size(); ++i) {
        Feature& f = features[i];
        f.errored = false;
        f.error.clear();
        if (!f.enabled) continue;

        auto fail = [&](const char* why) { f.errored = true; f.error = why; };

        switch (f.kind) {
        case FeatureKind::Primitive:
            if (!buildPrimitive(f.primitive, mesh)) fail("degenerate parameters");
            else any = true;
            break;

        case FeatureKind::Extrude:
            if (mesh.empty()) fail("nothing to extrude");
            else if (!f.faces.resolveFaces(mesh, scratchFaces)) fail("faces no longer exist");
            // The operation is transactional, so a rejection leaves the mesh as
            // it was and the chain carries on from there.
            else if (!extrudeFaces(mesh, scratchFaces, f.distance, nullptr, f.uid))
                fail("extrude failed");
            break;

        case FeatureKind::Inset:
            if (mesh.empty()) fail("nothing to inset");
            else if (!f.faces.resolveFaces(mesh, scratchFaces)) fail("faces no longer exist");
            else if (!insetFaces(mesh, scratchFaces, f.amount, nullptr, f.uid))
                fail("inset too large");
            break;

        case FeatureKind::Bevel: {
            if (mesh.empty()) { fail("nothing to bevel"); break; }
            if (!f.edges.resolveEdges(mesh, scratchEdges)) {
                fail("edges no longer exist");
                break;
            }
            FilletSpec spec;
            spec.segments = f.segments;
            spec.salt = f.uid;
            spec.edges.reserve(scratchEdges.size());
            for (size_t i = 0; i < scratchEdges.size(); ++i)
                spec.edges.push_back({scratchEdges[i], f.radiusFor(i)});
            if (!filletEdges(mesh, spec)) fail("radius too large for these edges");
            break;
        }

        case FeatureKind::BaseMesh:
            if (f.bakedMesh.empty()) fail("no geometry");
            else { mesh = f.bakedMesh; any = true; }
            break;

        case FeatureKind::Boolean: {
            if (mesh.empty()) { fail("nothing to combine with"); break; }
            if (f.bakedMesh.empty()) { fail("tool body is missing"); break; }
            Mesh combined;
            if (!meshBoolean(mesh, f.bakedMesh, f.booleanOp, combined))
                fail("boolean produced no valid solid");
            else
                mesh = std::move(combined);
            break;
        }

        case FeatureKind::VertexEdit: {
            if (mesh.empty()) { fail("nothing to edit"); break; }

            std::unordered_map<ElementId, Index> byName;
            byName.reserve(static_cast<size_t>(mesh.vertexCount()));
            for (Index v = 0; v < mesh.vertexCount(); ++v)
                byName.emplace(mesh.verts[v].id, v);

            std::vector<Index> hit;
            bool ok = true;
            for (size_t i = 0; i < f.verts.size() && i < f.offsets.size(); ++i) {
                auto it = byName.find(f.verts[i]);
                if (it == byName.end()) { ok = false; break; }
                hit.push_back(it->second);
            }
            if (!ok) { fail("vertices no longer exist"); break; }
            for (size_t i = 0; i < hit.size(); ++i)
                mesh.verts[hit[i]].position += f.offsets[i];
            break;
        }
        }

        // Snapshot after each step so a later edit can resume from here.
        cache[i] = mesh;
    }

    if (!any || mesh.empty()) return false;
    out = std::move(mesh);
    return true;
}

bool evaluateFeatures(std::vector<Feature>& features, Mesh& out) {
    std::vector<Mesh> scratch;
    return evaluateFrom(features, 0, scratch, out);
}

} // namespace tg
