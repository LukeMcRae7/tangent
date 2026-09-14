#include "geom/operations.h"

#include "mesh/decimate.h"

#include <cstdio>

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
                  std::vector<FaceId>* newFaces, ElementId salt, ExtrudeOp op,
                  std::string* reason, bool mergeFlush, Vec3 along) {
    if (!body.isMesh()) {
        // The exact backend sweeps the face and combines the result, so which
        // way the push goes decides whether that is a join or a cut. ExtrudeOp
        // says it outright when the caller knows better than the sign does.
        Real signed_ = distance;
        if (op == ExtrudeOp::Cut && signed_ > 0) signed_ = -signed_;
        if (op == ExtrudeOp::Join && signed_ < 0) signed_ = -signed_;

        std::vector<ElementId> names;
        BrepRef result = brep::extrudeFaces(body.brepRef(), faces, signed_, salt,
                                            newFaces ? &names : nullptr, reason,
                                            mergeFlush, along);
        if (!result) return false;
        body = Body(std::move(result));
        if (newFaces) {
            newFaces->clear();
            std::vector<FaceId> at;
            for (ElementId id : names) {
                body.findFaces(id, at);
                for (FaceId f : at) newFaces->push_back(f);
            }
        }
        return true;
    }

    // Transactional, as every operation here is: the mesh function leaves its
    // input untouched on failure, so a refused edit cannot half-apply.
    if (!extrudeFaces(body.mesh(), faces, distance, newFaces, salt, op)) {
        if (reason) *reason = "the extrude could not be built";
        return false;
    }
    return true;
}

bool makeProfileSolid(const std::vector<Vec3>& points, const std::vector<Real>& arcs,
                      Vec3 planeNormal, Real z0, Real z1, Body& out,
                      ElementId salt, std::string* reason) {
    BrepRef s = brep::prism(points, arcs, planeNormal, z0, z1, salt, reason);
    if (!s) return false;
    out = Body(std::move(s));
    return true;
}

bool shellBody(Body& body, const std::vector<FaceId>& openFaces, Real thickness,
               ElementId salt, std::string* reason) {
    if (body.isMesh()) {
        if (reason) *reason = "shelling needs the exact kernel, and this body is a mesh";
        return false;
    }
    BrepRef result = brep::shell(body.brepRef(), openFaces, thickness, salt, reason);
    if (!result) return false;
    body = Body(std::move(result));
    return true;
}

bool insetFaces(Body& body, const std::vector<FaceId>& faces, Real amount,
                std::vector<FaceId>* newFaces, ElementId salt, std::string* reason) {
    if (!body.isMesh()) {
        std::vector<ElementId> names;
        BrepRef result = brep::insetFaces(body.brepRef(), faces, amount, salt,
                                          newFaces ? &names : nullptr, reason);
        if (!result) return false;
        body = Body(std::move(result));
        if (newFaces) {
            newFaces->clear();
            std::vector<FaceId> at;
            for (ElementId id : names) {
                body.findFaces(id, at);
                for (FaceId f : at) newFaces->push_back(f);
            }
        }
        return true;
    }
    if (!insetFaces(body.mesh(), faces, amount, newFaces, salt)) {
        if (reason) *reason = "inset too large";
        return false;
    }
    return true;
}

bool rotateFaces(Body& body, const std::vector<FaceId>& faces, Real angleRad,
                 Vec3 hingePoint, Vec3 hingeDir, ElementId salt, std::string* reason) {
    if (body.isMesh()) {
        if (reason) *reason = "rotating a face needs the exact kernel, and this body is a mesh";
        return false;
    }
    BrepRef result = brep::rotateFaces(body.brepRef(), faces, angleRad, hingePoint,
                                       hingeDir, salt, reason);
    if (!result) return false;
    body = Body(std::move(result));
    return true;
}

bool scaleFaces(Body& body, const std::vector<FaceId>& faces, Real factor,
                ElementId salt, std::string* reason) {
    if (body.isMesh()) {
        if (reason) *reason = "scaling a face needs the exact kernel, and this body is a mesh";
        return false;
    }
    BrepRef result = brep::scaleFaces(body.brepRef(), faces, factor, salt, reason);
    if (!result) return false;
    body = Body(std::move(result));
    return true;
}

bool mergeDivisions(Body& body, ElementId salt, std::string* reason) {
    if (body.isMesh()) {
        if (reason) *reason = "merging faces needs the exact kernel, and this body is a mesh";
        return false;
    }
    BrepRef result = brep::mergeDivisions(body.brepRef(), salt, reason);
    if (!result) return false;
    body = Body(std::move(result));
    return true;
}

bool divideBody(Body& body, Vec3 planePoint, Vec3 planeNormal, ElementId salt,
                std::string* reason) {
    if (body.isMesh()) {
        if (reason) *reason = "dividing a face needs the exact kernel, and this body is a mesh";
        return false;
    }
    BrepRef result = brep::divideBody(body.brepRef(), planePoint, planeNormal, salt, reason);
    if (!result) return false;
    body = Body(std::move(result));
    return true;
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
        std::vector<Real> ends;
        ends.reserve(spec.edges.size());
        for (const FilletEdge& e : spec.edges) ends.push_back(e.endRadius);
        BrepRef result = brep::filletEdges(body.brep(), edges, radii, spec.salt, reason,
                                           &ends, spec.chamfer);
        if (!result) return false;
        body = Body(std::move(result));
        return true;
    }
    return filletEdges(body.mesh(), spec, reason);
}

const char* patternModeName(PatternMode m) {
    switch (m) {
        case PatternMode::Linear:   return "Linear";
        case PatternMode::Circular: return "Circular";
        case PatternMode::Mirror:   return "Mirror";
    }
    return "Pattern";
}

Mat4 patternPlacement(const PatternSpec& spec, int i) {
    if (i == 0) return Mat4{};                  // the original, where it is
    switch (spec.mode) {
        case PatternMode::Linear:
            return translate(normalize(spec.dir) * (spec.step * i));
        case PatternMode::Circular: {
            const Vec3 a = normalize(spec.dir);
            return translate(spec.origin) * rotateAxis(a, spec.stepAngle * i) *
                   translate(spec.origin * -1.0);
        }
        case PatternMode::Mirror:
            break;                              // not a matrix; see below
    }
    return Mat4{};
}

bool patternBody(Body& body, const Body& tool, const PatternSpec& spec,
                 ElementId salt, std::string* reason) {
    const bool usingTool = !tool.empty();
    const Body& seed = usingTool ? tool : body;
    if (body.empty()) {
        if (reason) *reason = "there is nothing to pattern";
        return false;
    }
    if (body.isMesh() || seed.isMesh()) {
        if (reason) *reason = "patterning needs the exact kernel, and this body is a mesh";
        return false;
    }
    const int n = spec.mode == PatternMode::Mirror ? 2 : spec.count;
    if (n < 2) {
        if (reason) *reason = "a pattern of one copy is the thing it started from";
        return false;
    }
    if (!(length(spec.dir) > 1e-9)) {
        if (reason) *reason = "the pattern has no direction to follow";
        return false;
    }

    // Built into a scratch body so that a copy that will not combine leaves the
    // model as it was rather than half-patterned.
    Body out = body;

    // A tool pattern has to place its first copy too: the boolean that made the
    // original is gone, replaced by this feature. A body pattern already has
    // its first copy -- it is the body.
    for (int i = usingTool ? 0 : 1; i < n; ++i) {
        Body copy = seed;
        if (spec.mode == PatternMode::Mirror) {
            if (i > 0 && !copy.mirror(spec.origin, spec.dir)) {
                if (reason) *reason = "the reflection could not be built";
                return false;
            }
        } else if (i > 0) {
            copy.transform(patternPlacement(spec, i));
            if (copy.empty()) {
                if (reason) *reason = "a copy could not be placed";
                return false;
            }
        }

        Body combined;
        std::string why;
        if (!booleanOp(out, copy, spec.op, combined,
                       nameId(salt, IdRole::Copy, static_cast<ElementId>(i)), false, &why)) {
            if (reason) {
                char buf[160];
                std::snprintf(buf, sizeof(buf), "copy %d %s", i + 1,
                              why.empty() ? "produced no valid solid" : why.c_str());
                *reason = buf;
            }
            return false;
        }
        out = std::move(combined);
    }

    body = std::move(out);
    return true;
}

bool reduceBody(Body& body, const ReduceOptions& options, ElementId salt, ReduceResult& result) {
    if (!body.isMesh()) {
        result = ReduceResult{};
        result.error = "this body is already exact, so it has no triangles to reduce";
        return false;
    }
    Mesh out;
    result = reduceMesh(body.mesh(), options, out, salt);
    if (!result.ok) return false;
    body = Body(std::move(out));
    return true;
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
    if (!body.isMesh()) {
        std::vector<BrepRef> solids;
        brep::separateSolids(body.brep(), solids);
        out.clear();
        for (BrepRef& r : solids) out.push_back(Body(std::move(r)));
        if (out.empty() && !body.empty()) out.push_back(body);
        return out.size();
    }
    std::vector<Mesh> pieces;
    const size_t n = splitShells(body.mesh(), pieces);
    out.clear();
    out.reserve(pieces.size());
    for (Mesh& m : pieces) out.push_back(Body(std::move(m)));
    return n;
}

bool splitByPlane(const Body& body, Vec3 planePoint, Vec3 planeNormal,
                  Body& a, Body& b) {
    if (!body.isMesh()) {
        const Real nLen = length(planeNormal);
        if (!(nLen > 1e-12) || body.empty()) return false;
        const Vec3 n = planeNormal * (Real(1) / nLen);


        // One pass through the splitter first. It is about half the work of
        // the two booleans below, which remain for a body it refuses. It
        // refuses a side with nothing of substance on it itself, so what it
        // returns needs no checking here.
        {
            BrepRef up, down;
            if (brep::splitByPlane(body.brep(), planePoint, n, 0x5711C, up, down, nullptr)) {
                a = Body(std::move(up));
                b = Body(std::move(down));
                return true;
            }
        }
        const Real whole0 = std::fabs(body.health(false).volume);

        // A frame with the plane's normal as its z axis.
        const Vec3 helper = std::fabs(n.x) < 0.9 ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
        const Vec3 u = normalize(cross(helper, n));
        const Vec3 v = cross(n, u);
        const Mat4 frame(Vec4(u, 0), Vec4(v, 0), Vec4(n, 0), Vec4(planePoint, 1));

        // Boxes several times the body's size, so that each covers everything on
        // its side of the plane however the body sits.
        const AABB box = body.bounds();
        const Real reach = (length(box.size()) + length(box.center() - planePoint)) * 2 + 1;
        PrimitiveSpec half;
        half.kind = PrimitiveKind::Box;
        half.box = {reach * 2, reach * 2, reach};
        Body above, below;
        if (!makePrimitive(half, above, Backend::Brep)) return false;
        below = above;
        above.transform(frame * translate({0, 0, reach * 0.5}));
        below.transform(frame * translate({0, 0, -reach * 0.5}));

        Body sideA, sideB;
        if (!booleanOp(body, above, BooleanOp::Intersection, sideA, 0x5711A, false, nullptr) ||
            !booleanOp(body, below, BooleanOp::Intersection, sideB, 0x5711B, false, nullptr))
            return false;
        // A plane that misses, or only touches, leaves one side with nothing in
        // it; that is not a split.
        const Real whole = whole0;
        const Real va = sideA.empty() ? 0 : std::fabs(sideA.health(false).volume);
        const Real vb = sideB.empty() ? 0 : std::fabs(sideB.health(false).volume);
        const Real crumb = whole * 1e-9;
        if (va <= crumb || vb <= crumb) return false;
        a = std::move(sideA);
        b = std::move(sideB);
        return true;
    }
    Mesh ma, mb;
    if (!splitBodyByPlane(body.mesh(), planePoint, planeNormal, ma, mb)) return false;
    a = Body(std::move(ma));
    b = Body(std::move(mb));
    return true;
}

} // namespace tg
