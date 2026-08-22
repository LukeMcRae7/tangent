#include "scene/feature.h"

#include "mesh/operations.h"

#include <cstdio>

namespace tg {

const char* featureKindName(FeatureKind k) {
    switch (k) {
        case FeatureKind::Primitive:  return "Primitive";
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
        case FeatureKind::Extrude:
            std::snprintf(buf, sizeof(buf), "Extrude  %.2f mm  (%zu face%s)",
                          static_cast<double>(distance), faces.size(),
                          faces.size() == 1 ? "" : "s");
            break;
        case FeatureKind::Inset:
            std::snprintf(buf, sizeof(buf), "Inset  %.2f mm  (%zu face%s)",
                          static_cast<double>(amount), faces.size(),
                          faces.size() == 1 ? "" : "s");
            break;
        case FeatureKind::Bevel:
            std::snprintf(buf, sizeof(buf), "Bevel  %.2f mm  x%d",
                          static_cast<double>(width), segments);
            break;
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
bool facesResolve(const std::vector<Index>& faces, const Mesh& mesh) {
    if (faces.empty()) return false;
    for (Index f : faces)
        if (f < 0 || f >= mesh.faceCount()) return false;
    return true;
}

} // namespace

bool evaluateFeatures(std::vector<Feature>& features, Mesh& out) {
    Mesh mesh;
    bool any = false;

    for (Feature& f : features) {
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
            else if (!facesResolve(f.faces, mesh)) fail("faces no longer exist");
            // The operation is transactional, so a rejection leaves the mesh as
            // it was and the chain carries on from there.
            else if (!extrudeFaces(mesh, f.faces, f.distance, nullptr)) fail("extrude failed");
            break;

        case FeatureKind::Inset:
            if (mesh.empty()) fail("nothing to inset");
            else if (!facesResolve(f.faces, mesh)) fail("faces no longer exist");
            else if (!insetFaces(mesh, f.faces, f.amount, nullptr)) fail("inset too large");
            break;

        case FeatureKind::Bevel:
            if (mesh.empty()) fail("nothing to bevel");
            else if (!bevelAllEdges(mesh, f.width, f.segments)) fail("width too large");
            break;

        case FeatureKind::VertexEdit: {
            if (mesh.empty()) { fail("nothing to edit"); break; }
            bool ok = true;
            for (size_t i = 0; i < f.verts.size() && i < f.offsets.size(); ++i) {
                if (f.verts[i] < 0 || f.verts[i] >= mesh.vertexCount()) { ok = false; break; }
            }
            if (!ok) { fail("vertices no longer exist"); break; }
            for (size_t i = 0; i < f.verts.size() && i < f.offsets.size(); ++i)
                mesh.verts[f.verts[i]].position += f.offsets[i];
            break;
        }
        }
    }

    if (!any || mesh.empty()) return false;
    out = std::move(mesh);
    return true;
}

} // namespace tg
