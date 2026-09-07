#include "scene/feature.h"

#include "geom/operations.h"

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
            std::snprintf(buf, sizeof(buf), "Mesh  (%d faces)", bakedBody.faceCount());
            break;
        case FeatureKind::Boolean:
            std::snprintf(buf, sizeof(buf), "%s  (%d faces)",
                          booleanOpName(booleanOp), bakedBody.faceCount());
            break;
        case FeatureKind::Extrude:
            if (extrudeOp == ExtrudeOp::Auto) {
                std::snprintf(buf, sizeof(buf), "Extrude  %.2f mm  (%s)",
                              static_cast<double>(distance), faces.describe("face").c_str());
            } else {
                std::snprintf(buf, sizeof(buf), "Extrude %s  %.2f mm  (%s)",
                              extrudeOpName(extrudeOp), static_cast<double>(distance), faces.describe("face").c_str());
            }
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

// True if every face index still exists in the mesh the chain has built so far.
} // namespace

ElementRefs nameFaces(const Body& body, const std::vector<FaceId>& faces) {
    ElementRefs r;
    for (FaceId f : faces)
        if (f >= 0 && f < body.faceCount()) r.ids.push_back(body.faceName(f));
    return r;
}

ElementRefs nameEdges(const Body& body, const std::vector<EdgeId>& edges) {
    ElementRefs r;
    if (edges.empty()) return r;

    // Handles from the Body are already canonical, so this is a plain set.
    std::unordered_set<EdgeId> chosen(edges.begin(), edges.end());
    if (chosen.empty()) return r;

    // Does some face's boundary consist of exactly these edges?
    std::vector<EdgeId> fe;
    for (EdgeId e : chosen) {
        FaceId sides[2] = {kNoFace, kNoFace};
        body.edgeFaces(e, sides[0], sides[1]);
        for (FaceId f : sides) {
            if (f == kNoFace) continue;
            body.faceEdges(f, fe);
            if (fe.size() != chosen.size()) continue;
            bool all = true;
            for (EdgeId x : fe)
                if (!chosen.count(x)) { all = false; break; }
            if (all) {
                r.kind = ElementRefs::Kind::FaceBoundary;
                r.face = body.faceName(f);
                return r;
            }
        }
        break;   // one edge is enough to reach every candidate face
    }

    for (EdgeId e : chosen) r.ids.push_back(body.edgeName(e));
    std::sort(r.ids.begin(), r.ids.end());
    return r;
}

std::vector<ElementId> nameVertices(const Body& body, const std::vector<VertexId>& verts) {
    std::vector<ElementId> out;
    for (VertexId v : verts)
        if (v >= 0 && v < body.vertexCount()) out.push_back(body.vertexName(v));
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

bool ElementRefs::resolveFaces(const Body& body, std::vector<FaceId>& out) const {
    out.clear();
    if (kind == Kind::All) {
        body.allFaces(out);
        return !out.empty();
    }
    if (kind == Kind::FaceBoundary) {
        const FaceId f = body.findFace(face);
        if (f == kNoFace) return false;
        out.push_back(f);
        return true;
    }
    if (ids.empty()) return false;

    // One pass to build the map, rather than a linear findFace per name: a
    // feature naming twenty faces of a thousand-face body would otherwise cost
    // twenty thousand comparisons on every evaluation.
    std::vector<FaceId> all;
    body.allFaces(all);
    std::unordered_map<ElementId, FaceId> byName;
    byName.reserve(all.size());
    for (FaceId f : all) byName.emplace(body.faceName(f), f);
    for (ElementId id : ids) {
        auto it = byName.find(id);
        if (it == byName.end()) return false;
        out.push_back(it->second);
    }
    return true;
}

bool ElementRefs::resolveEdges(const Body& body, std::vector<EdgeId>& out) const {
    out.clear();
    if (kind == Kind::All) {
        body.allEdges(out);
        return !out.empty();
    }
    if (kind == Kind::FaceBoundary) {
        const FaceId f = body.findFace(face);
        if (f == kNoFace) return false;
        body.faceEdges(f, out);
        return !out.empty();
    }
    if (ids.empty()) return false;

    std::vector<EdgeId> all;
    body.allEdges(all);
    std::unordered_map<ElementId, EdgeId> byName;
    byName.reserve(all.size());
    for (EdgeId e : all) byName.emplace(body.edgeName(e), e);
    for (ElementId id : ids) {
        auto it = byName.find(id);
        if (it == byName.end()) return false;
        out.push_back(it->second);
    }
    return true;
}

bool evaluateFrom(std::vector<Feature>& features, size_t from,
                  std::vector<Body>& cache, Body& out) {
    // The cache must actually hold the requested starting point; anything else
    // (a freshly loaded object, a chain that shrank) means starting over.
    if (from > 0 && (cache.size() < from || from > features.size())) from = 0;

    Body body;
    if (from > 0) body = cache[from - 1];

    cache.resize(features.size());
    bool any = from > 0 || false;

    std::vector<FaceId> scratchFaces;
    std::vector<EdgeId> scratchEdges;

    for (size_t i = from; i < features.size(); ++i) {
        Feature& f = features[i];
        f.errored = false;
        f.error.clear();
        if (!f.enabled) continue;

        auto fail = [&](const char* why) { f.errored = true; f.error = why; };

        switch (f.kind) {
        case FeatureKind::Primitive:
            if (!makePrimitive(f.primitive, body)) fail("degenerate parameters");
            else any = true;
            break;

        case FeatureKind::Extrude:
            if (body.empty()) fail("nothing to extrude");
            else if (!f.faces.resolveFaces(body, scratchFaces)) fail("faces no longer exist");
            // The operation is transactional, so a rejection leaves the body as
            // it was and the chain carries on from there.
            else if (!extrudeFaces(body, scratchFaces, f.distance, nullptr, f.uid, f.extrudeOp))
                fail("extrude failed");
            break;

        case FeatureKind::Inset:
            if (body.empty()) fail("nothing to inset");
            else if (!f.faces.resolveFaces(body, scratchFaces)) fail("faces no longer exist");
            else if (!insetFaces(body, scratchFaces, f.amount, nullptr, f.uid))
                fail("inset too large");
            break;

        case FeatureKind::Bevel: {
            if (body.empty()) { fail("nothing to bevel"); break; }
            if (!f.edges.resolveEdges(body, scratchEdges)) {
                fail("edges no longer exist");
                break;
            }
            FilletSpec spec;
            spec.segments = f.segments;
            spec.salt = f.uid;
            spec.edges.reserve(scratchEdges.size());
            for (size_t i = 0; i < scratchEdges.size(); ++i)
                spec.edges.push_back({scratchEdges[i], f.radiusFor(i)});
            // The radius is only one of the two dozen reasons a fillet refuses,
            // and naming it unconditionally was wrong far more often than right.
            std::string reason;
            if (!filletEdges(body, spec, &reason))
                fail(reason.empty() ? "the fillet could not be built" : reason.c_str());
            break;
        }

        case FeatureKind::BaseMesh:
            if (f.bakedBody.empty()) fail("no geometry");
            else { body = f.bakedBody; any = true; }
            break;

        case FeatureKind::Boolean: {
            if (body.empty()) { fail("nothing to combine with"); break; }
            if (f.bakedBody.empty()) { fail("tool body is missing"); break; }
            Body combined;
            if (!booleanOp(body, f.bakedBody, f.booleanOp, combined, f.uid))
                fail("boolean produced no valid solid");
            else
                body = std::move(combined);
            break;
        }

        case FeatureKind::VertexEdit: {
            if (body.empty()) { fail("nothing to edit"); break; }

            std::vector<VertexId> all;
            body.allVertices(all);
            std::unordered_map<ElementId, VertexId> byName;
            byName.reserve(all.size());
            for (VertexId v : all) byName.emplace(body.vertexName(v), v);

            std::vector<VertexId> hit;
            bool ok = true;
            for (size_t i = 0; i < f.verts.size() && i < f.offsets.size(); ++i) {
                auto it = byName.find(f.verts[i]);
                if (it == byName.end()) { ok = false; break; }
                hit.push_back(it->second);
            }
            if (!ok) { fail("vertices no longer exist"); break; }
            for (size_t i = 0; i < hit.size(); ++i)
                body.moveVertex(hit[i], f.offsets[i]);
            break;
        }
        }

        // Snapshot after each step so a later edit can resume from here.
        cache[i] = body;
    }

    if (!any || body.empty()) return false;
    out = std::move(body);
    return true;
}

bool evaluateFeatures(std::vector<Feature>& features, Body& out) {
    std::vector<Body> scratch;
    return evaluateFrom(features, 0, scratch, out);
}

} // namespace tg
