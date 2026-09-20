#include "geom/operations.h"

#include "mesh/decimate.h"

#include <cstdio>

namespace tg {

const char* booleanOpName(BooleanOp op) {
    switch (op) {
        // The words the interface uses for them, everywhere it uses them.
        case BooleanOp::Union:        return "Join";
        case BooleanOp::Difference:   return "Cut";
        case BooleanOp::Intersection: return "Intersect";
    }
    return "Boolean";
}

const char* extrudeOpName(ExtrudeOp op) {
    switch (op) {
        case ExtrudeOp::Auto:      return "Auto";
        case ExtrudeOp::Join:      return "Join";
        case ExtrudeOp::Cut:       return "Cut";
        case ExtrudeOp::Intersect: return "Intersect";
        case ExtrudeOp::NewBody:   return "New Body";
    }
    return "Extrude";
}

namespace {

// One refusal for every operation a mesh cannot have, so that each of them says
// the same thing and says what to do about it. `what` is the operation, as a
// gerund, lower case like every other reason: "extruding a face".
bool refuseMesh(const char* what, std::string* reason) {
    if (reason) {
        *reason = what;
        *reason += brep::available()
            ? " needs a solid, and this body is a mesh: Modify > Convert to Solid first"
            : " needs the exact kernel, which this build does not have";
    }
    return false;
}

} // namespace

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
    if (reason) reason->clear();
    if (!body.isMesh()) {
        // The exact backend sweeps the face and combines the result, so which
        // way the push goes decides whether that is a join or a cut. ExtrudeOp
        // says it outright when the caller knows better than the sign does.
        //
        // For the body the faces belong to there is one way each operation
        // means anything: a join grows out of it, and a cut or an intersect
        // goes into it. What they do to other bodies is the caller's, with the
        // swept solid from sweptFaces.
        Real signed_ = distance;
        if ((op == ExtrudeOp::Cut || op == ExtrudeOp::Intersect) && signed_ > 0) signed_ = -signed_;
        if (op == ExtrudeOp::Join && signed_ < 0) signed_ = -signed_;

        std::vector<ElementId> names;
        BrepRef result = brep::extrudeFaces(body.brepRef(), faces, signed_, salt,
                                            newFaces ? &names : nullptr, reason,
                                            mergeFlush, along, op == ExtrudeOp::Intersect);
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

    return refuseMesh("extruding a face", reason);
}

bool sweepFaces(const Body& body, const std::vector<FaceId>& faces, Real distance, Vec3 along,
                ElementId salt, Body& out, std::string* reason) {
    if (reason) reason->clear();
    if (body.isMesh()) return refuseMesh("sweeping a face", reason);
    BrepRef s = brep::sweptFaces(body.brepRef(), faces, distance, along, salt, reason);
    if (!s) return false;
    out = Body(std::move(s));
    return true;
}

bool bodiesTouch(const Body& a, const Body& b, Real tol) {
    if (a.isMesh() || b.isMesh() || a.empty() || b.empty()) return false;
    return brep::touches(a.brep(), b.brep(), tol);
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
    if (body.isMesh()) return refuseMesh("shelling", reason);
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
    return refuseMesh("insetting a face", reason);
}

bool rotateFaces(Body& body, const std::vector<FaceId>& faces, Real angleRad,
                 Vec3 hingePoint, Vec3 hingeDir, ElementId salt, std::string* reason) {
    if (body.isMesh()) return refuseMesh("rotating a face", reason);
    BrepRef result = brep::rotateFaces(body.brepRef(), faces, angleRad, hingePoint,
                                       hingeDir, salt, reason);
    if (!result) return false;
    body = Body(std::move(result));
    return true;
}

bool scaleFaces(Body& body, const std::vector<FaceId>& faces, Real factor,
                ElementId salt, std::string* reason) {
    if (body.isMesh()) return refuseMesh("scaling a face", reason);
    BrepRef result = brep::scaleFaces(body.brepRef(), faces, factor, salt, reason);
    if (!result) return false;
    body = Body(std::move(result));
    return true;
}

bool mergeDivisions(Body& body, ElementId salt, std::string* reason) {
    if (body.isMesh()) return refuseMesh("merging faces", reason);
    BrepRef result = brep::mergeDivisions(body.brepRef(), salt, reason);
    if (!result) return false;
    body = Body(std::move(result));
    return true;
}

bool divideBody(Body& body, Vec3 planePoint, Vec3 planeNormal, ElementId salt,
                std::string* reason) {
    if (body.isMesh()) return refuseMesh("dividing a face", reason);
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
    return refuseMesh(spec.chamfer ? "chamfering an edge" : "filleting an edge", reason);
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
    if (body.isMesh() || seed.isMesh()) return refuseMesh("patterning", reason);
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
    (void)trustBNames;
    if (reason) reason->clear();
    // An empty Body answers isMesh() too, and "this body is a mesh" would be
    // the wrong thing to tell someone who has nothing there at all.
    if (a.empty() || b.empty()) {
        if (reason) *reason = "there is nothing to combine";
        return false;
    }
    if (a.isMesh() || b.isMesh()) return refuseMesh("combining bodies", reason);
    BrepRef result = brep::booleanOp(a.brep(), b.brep(), op, salt, reason);
    if (!result) return false;
    out = Body(std::move(result));
    return true;
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
    // A mesh is cut after it is converted, where the cut is exact.
    if (body.isMesh()) return false;
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
    const Real whole = std::fabs(body.health(false).volume);

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
    const Real va = sideA.empty() ? 0 : std::fabs(sideA.health(false).volume);
    const Real vb = sideB.empty() ? 0 : std::fabs(sideB.health(false).volume);
    const Real crumb = whole * 1e-9;
    if (va <= crumb || vb <= crumb) return false;
    a = std::move(sideA);
    b = std::move(sideB);
    return true;
}

} // namespace tg
