#include "geom/operations.h"

namespace tg {

// Every function here is a forwarder while there is one backend. They are not
// pointless: they are the list of what a second backend must provide, and they
// are the only place in the codebase that will need a branch when it arrives.

bool makePrimitive(const PrimitiveSpec& spec, Body& out, Backend backend) {
    if (backend == Backend::Brep) {
        BrepRef s = brep::primitive(spec);
        if (!s) return false;
        out = Body(std::move(s));
        return true;
    }

    Mesh m;
    bool ok = false;
    switch (spec.kind) {
        case PrimitiveKind::Box:      ok = makeBox(m, spec.box);           break;
        case PrimitiveKind::Cylinder: ok = makeCylinder(m, spec.cylinder); break;
        case PrimitiveKind::Sphere:   ok = makeSphere(m, spec.sphere);     break;
        case PrimitiveKind::Cone:     ok = makeCone(m, spec.cone);         break;
        case PrimitiveKind::Torus:    ok = makeTorus(m, spec.torus);       break;
        case PrimitiveKind::Plane:    ok = makePlane(m, spec.plane);       break;
        case PrimitiveKind::Custom:   ok = false;                          break;
    }
    if (!ok) return false;
    out = Body(std::move(m));
    return true;
}

bool extrudeFaces(Body& body, const std::vector<FaceId>& faces, Real distance,
                  std::vector<FaceId>* newFaces, ElementId salt, ExtrudeOp op) {
    // Transactional, as every operation here is: the mesh function leaves its
    // input untouched on failure, so a refused edit cannot half-apply.
    return extrudeFaces(body.mesh(), faces, distance, newFaces, salt, op);
}

bool insetFaces(Body& body, const std::vector<FaceId>& faces, Real amount,
                std::vector<FaceId>* newFaces, ElementId salt) {
    return insetFaces(body.mesh(), faces, amount, newFaces, salt);
}

bool filletEdges(Body& body, const FilletSpec& spec, std::string* reason) {
    if (!body.isMesh()) {
        std::vector<EdgeId> edges;
        std::vector<Real> radii;
        edges.reserve(spec.edges.size());
        radii.reserve(spec.edges.size());
        for (const FilletEdge& e : spec.edges) {
            edges.push_back(e.edge);
            radii.push_back(e.radius);
        }
        BrepRef result = brep::filletEdges(body.brep(), edges, radii, spec.salt, reason);
        if (!result) return false;
        body = Body(std::move(result));
        return true;
    }
    return filletEdges(body.mesh(), spec, reason);
}

bool booleanOp(const Body& a, const Body& b, BooleanOp op, Body& out,
               ElementId salt, bool trustBNames, std::string* reason) {
    if (reason) reason->clear();
    if (a.isMesh() != b.isMesh()) {
        if (reason) *reason = "one body is a mesh and the other is exact";
        return false;
    }
    if (!a.isMesh()) {
        BrepRef result = brep::booleanOp(a.brep(), b.brep(), op, salt, reason);
        if (!result) return false;
        out = Body(std::move(result));
        return true;
    }

    Mesh combined;
    if (!meshBoolean(a.mesh(), b.mesh(), op, combined, salt, trustBNames)) {
        // The mesh boolean reports only that it refused. What it does guarantee
        // is that it refused rather than handing back something broken.
        if (reason) *reason = "no valid solid came out of it";
        return false;
    }
    out = Body(std::move(combined));
    return true;
}

Real maxFilletRadius(const Body& body) {
    return maxBevelWidth(body.mesh());
}

std::vector<EdgeId> extendTangentChain(const Body& body, const std::vector<EdgeId>& edges) {
    return extendTangentChain(body.mesh(), edges);
}

size_t splitBodies(const Body& body, std::vector<Body>& out) {
    std::vector<Mesh> pieces;
    const size_t n = splitShells(body.mesh(), pieces);
    out.clear();
    out.reserve(pieces.size());
    for (Mesh& m : pieces) out.push_back(Body(std::move(m)));
    return n;
}

bool splitByPlane(const Body& body, Vec3 planePoint, Vec3 planeNormal,
                  Body& a, Body& b) {
    Mesh ma, mb;
    if (!splitBodyByPlane(body.mesh(), planePoint, planeNormal, ma, mb)) return false;
    a = Body(std::move(ma));
    b = Body(std::move(mb));
    return true;
}

} // namespace tg
