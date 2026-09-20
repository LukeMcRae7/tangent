#include "scene/feature.h"

#include "mesh/decimate.h"

#include "geom/operations.h"

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
        case FeatureKind::Shell:      return "Shell";
        case FeatureKind::FaceRotate: return "Rotate Face";
        case FeatureKind::FaceScale:  return "Scale Face";
        case FeatureKind::Divide:     return "Divide";
        case FeatureKind::Merge:      return "Merge Faces";
        case FeatureKind::Pattern:    return "Pattern";
        case FeatureKind::Reduce:     return "Reduce Mesh";
        case FeatureKind::Sketch:     return "Sketch";
        case FeatureKind::ExtrudeProfile: return "Extrude Profile";
        case FeatureKind::RevolveProfile: return "Revolve";
        case FeatureKind::Hole:       return "Hole";
        case FeatureKind::Move:       return "Move";
        case FeatureKind::Rotate:     return "Rotate";
        case FeatureKind::Scale:      return "Scale";
    }
    return "Feature";
}

namespace {
// " along X" when a face was pulled along an axis rather than its own normal,
// and nothing at all when it went the way the face faces.
const char* axisWord(const Feature& f) {
    if (!f.alongAxis) return "";
    const Vec3 d = f.axisDir;
    if (std::fabs(d.x) > 0.9) return " along X";
    if (std::fabs(d.y) > 0.9) return " along Y";
    if (std::fabs(d.z) > 0.9) return " along Z";
    return " along an axis";
}
} // namespace

PatternSpec Feature::pattern() const {
    PatternSpec s;
    s.mode = patternMode;
    s.count = patternCount;
    s.origin = axisPoint;
    s.dir = axisDir;
    s.step = distance;
    s.stepAngle = angle;
    s.op = booleanOp;
    return s;
}

const char* Feature::displayKind() const {
    // What it is, not how many segments a mesh would have used to approximate
    // it. An exact round has no segments, and reading the leftover default as
    // "chamfer" labelled every fillet in the history as one.
    if (kind == FeatureKind::Bevel) return chamfer ? "Chamfer" : "Fillet";
    // Two operations share the kind: the one that moved a face and the one that
    // grew a boss off it. What a person calls them is the difference.
    if (kind == FeatureKind::Extrude) return mergeFlush ? "Push / Pull" : "Extrude";
    // A mirror is a pattern of two, but nobody calls it that.
    if (kind == FeatureKind::Pattern && patternMode == PatternMode::Mirror) return "Mirror";
    return featureKindName(kind);
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
            if (!toolName.empty())
                std::snprintf(buf, sizeof(buf), "%s  %s", booleanOpName(booleanOp), toolName.c_str());
            else
                std::snprintf(buf, sizeof(buf), "%s  (%d faces)",
                              booleanOpName(booleanOp), bakedBody.faceCount());
            break;
        case FeatureKind::Move:
            std::snprintf(buf, sizeof(buf), "Move  %.2f, %.2f, %.2f mm", static_cast<double>(moveBy.x),
                          static_cast<double>(moveBy.y), static_cast<double>(moveBy.z));
            break;
        case FeatureKind::Rotate: {
            // As an angle about an axis, which is how a turn is thought of --
            // and when the axis is one of the three, by its name.
            const Real w = std::clamp(turnBy.w, Real(-1), Real(1));
            Real angle = degrees(2.0 * std::acos(w));
            Vec3 axis{turnBy.x, turnBy.y, turnBy.z};
            if (length(axis) < 1e-12) axis = {0, 0, 1};
            axis = normalize(axis);
            if (angle > 180.0) { angle = 360.0 - angle; axis = -axis; }
            const char* name = nullptr;
            for (int i = 0; i < 3 && !name; ++i)
                if (std::fabs(std::fabs(axis[i]) - 1.0) < 1e-6) {
                    static const char* const kPos[] = {"X", "Y", "Z"};
                    if (axis[i] < 0) angle = -angle;
                    name = kPos[i];
                }
            if (name)
                std::snprintf(buf, sizeof(buf), "Rotate  %.1f\xC2\xB0 about %s", static_cast<double>(angle), name);
            else
                std::snprintf(buf, sizeof(buf), "Rotate  %.1f\xC2\xB0", static_cast<double>(angle));
            break;
        }
        case FeatureKind::Scale:
            if (std::fabs(scaleBy.x - scaleBy.y) < 1e-9 && std::fabs(scaleBy.x - scaleBy.z) < 1e-9)
                std::snprintf(buf, sizeof(buf), "Scale  %.3g\xC3\x97", static_cast<double>(scaleBy.x));
            else
                std::snprintf(buf, sizeof(buf), "Scale  %.3g \xC3\x97 %.3g \xC3\x97 %.3g",
                              static_cast<double>(scaleBy.x), static_cast<double>(scaleBy.y),
                              static_cast<double>(scaleBy.z));
            break;
        case FeatureKind::Extrude:
            // Push / pull says nothing of joining or cutting: moving a face
            // out adds and in takes away, and that is all it can mean. An
            // extrude names its operation, a step from before "Auto" was
            // dropped the one its direction made it.
            if (mergeFlush) {
                std::snprintf(buf, sizeof(buf), "Push / Pull  %.2f mm%s  (%s)",
                              static_cast<double>(distance), axisWord(*this),
                              faces.describe("face").c_str());
            } else {
                const ExtrudeOp shown = extrudeOp == ExtrudeOp::Auto
                                            ? (distance < 0.0 ? ExtrudeOp::Cut : ExtrudeOp::Join)
                                            : extrudeOp;
                std::snprintf(buf, sizeof(buf), "Extrude %s  %.2f mm%s  (%s)", extrudeOpName(shown),
                              static_cast<double>(distance), axisWord(*this),
                              faces.describe("face").c_str());
            }
            break;
        case FeatureKind::Inset:
            std::snprintf(buf, sizeof(buf), "Inset  %.2f mm  (%s)",
                          static_cast<double>(amount), faces.describe("face").c_str());
            break;
        case FeatureKind::Bevel: {
            const char* what = displayKind();

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
        case FeatureKind::Shell:
            // An open face is the point of most shells, so say when there is
            // none rather than leaving the reader to infer it from a blank.
            if (faces.empty())
                std::snprintf(buf, sizeof(buf), "Shell  %.2f mm  (sealed)",
                              static_cast<double>(thickness));
            else
                std::snprintf(buf, sizeof(buf), "Shell  %.2f mm  (%s open)",
                              static_cast<double>(thickness),
                              faces.describe("face").c_str());
            break;
        case FeatureKind::FaceScale:
            std::snprintf(buf, sizeof(buf), "Scale  %.3g x  (%s)",
                          static_cast<double>(scale), faces.describe("face").c_str());
            break;
        case FeatureKind::FaceRotate:
            std::snprintf(buf, sizeof(buf), "Rotate  %.1f deg  (%s)",
                          static_cast<double>(degrees(angle)),
                          faces.describe("face").c_str());
            break;
        case FeatureKind::Merge:
            std::snprintf(buf, sizeof(buf), "Merge faces");
            break;
        case FeatureKind::Pattern:
            switch (patternMode) {
                case PatternMode::Mirror:
                    std::snprintf(buf, sizeof(buf), "Mirror  (%s)",
                                  bakedBody.empty() ? "the body" : "the cut");
                    break;
                case PatternMode::Linear:
                    std::snprintf(buf, sizeof(buf), "Pattern  %d x  %.2f mm apart",
                                  patternCount, static_cast<double>(distance));
                    break;
                case PatternMode::Circular:
                    std::snprintf(buf, sizeof(buf), "Pattern  %d x  %.1f deg apart",
                                  patternCount, static_cast<double>(degrees(angle)));
                    break;
            }
            break;
        case FeatureKind::Reduce:
            if (reduceTarget > 0)
                std::snprintf(buf, sizeof(buf), "Reduce  %s %.3g mm, to %d triangles",
                              reduceLoosen ? "from" : "within",
                              static_cast<double>(reduceTolerance), reduceTarget);
            else
                std::snprintf(buf, sizeof(buf), "Reduce  within %.3g mm",
                              static_cast<double>(reduceTolerance));
            break;
        case FeatureKind::Sketch: {
            // Whether it is fully constrained is the thing a person wants to
            // know about a sketch at a glance, so it is in the line itself.
            const size_t dims = static_cast<size_t>(std::count_if(
                sketch.constraints.begin(), sketch.constraints.end(),
                [](const SketchConstraint& k) { return isDimension(k.rule); }));
            if (sketchFreedoms == 0)
                std::snprintf(buf, sizeof(buf), "Sketch  %zu entities, %zu dims, fully constrained",
                              sketch.entities.size(), dims);
            else
                std::snprintf(buf, sizeof(buf), "Sketch  %zu entities, %zu dims, %d free",
                              sketch.entities.size(), dims, sketchFreedoms);
            break;
        }
        case FeatureKind::ExtrudeProfile: {
            const ExtrudeOp shown = extrudeOp == ExtrudeOp::Auto
                                        ? (distance < 0.0 ? ExtrudeOp::Cut : ExtrudeOp::Join)
                                        : extrudeOp;
            if (profileKeys.size() > 1)
                std::snprintf(buf, sizeof(buf), "Extrude Profile  %.2f mm  (%s, %zu regions)",
                              static_cast<double>(distance), extrudeOpName(shown), profileKeys.size());
            else
                std::snprintf(buf, sizeof(buf), "Extrude Profile  %.2f mm  (%s)",
                              static_cast<double>(distance), extrudeOpName(shown));
            break;
        }
        case FeatureKind::RevolveProfile: {
            const ExtrudeOp shown = extrudeOp == ExtrudeOp::Auto ? ExtrudeOp::Join : extrudeOp;
            const double deg = static_cast<double>(revolveAngle) * kRad2Deg;
            if (profileKeys.size() > 1)
                std::snprintf(buf, sizeof(buf), "Revolve  %.1f deg  (%s, %zu regions)", deg,
                              extrudeOpName(shown), profileKeys.size());
            else
                std::snprintf(buf, sizeof(buf), "Revolve  %.1f deg  (%s)", deg,
                              extrudeOpName(shown));
            break;
        }
        case FeatureKind::Hole: {
            // What was asked for, not what came out of the table: "M4
            // clearance, counterbored" is what the step is, and 4.70 mm is
            // what it happens to measure.
            char what[64];
            if (holeFastener >= 0)
                std::snprintf(what, sizeof(what), "%s %s", fastenerAt(holeFastener).name,
                              holeFit == HoleFit::Tapped ? "tapped" : holeFitName(holeFit));
            else
                std::snprintf(what, sizeof(what), "%.2f mm", static_cast<double>(hole.diameter));
            char how[48];
            if (hole.kind == HoleKind::Simple) how[0] = '\0';
            else std::snprintf(how, sizeof(how), ", %s", holeKindName(hole.kind));
            if (hole.through)
                std::snprintf(buf, sizeof(buf), "Hole  %s, through%s", what, how);
            else
                std::snprintf(buf, sizeof(buf), "Hole  %s, %.2f mm deep%s", what,
                              static_cast<double>(hole.depth), how);
            break;
        }
        case FeatureKind::Divide:
            std::snprintf(buf, sizeof(buf), "Divide  at %.2f, %.2f, %.2f",
                          static_cast<double>(axisPoint.x),
                          static_cast<double>(axisPoint.y),
                          static_cast<double>(axisPoint.z));
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

ElementRefs nameEdges(const Body& body, const std::vector<EdgeId>& edges,
                      bool allowBoundary) {
    ElementRefs r;
    if (edges.empty()) return r;

    // Order is part of what this list means: `Feature::radii` runs alongside
    // it, so the third name is the third radius. A set would lose that, and a
    // sort would scramble it -- which it did, and the radii landed on other
    // edges. Duplicates go, the order stays.
    std::vector<EdgeId> chosen;
    chosen.reserve(edges.size());
    {
        std::unordered_set<EdgeId> seen;
        for (EdgeId e : edges)
            if (seen.insert(e).second) chosen.push_back(e);
    }
    if (chosen.empty()) return r;

    // Does some face's boundary consist of exactly these edges?
    std::vector<EdgeId> fe;
    if (allowBoundary) for (EdgeId e : chosen) {
        FaceId sides[2] = {kNoFace, kNoFace};
        body.edgeFaces(e, sides[0], sides[1]);
        for (FaceId f : sides) {
            if (f == kNoFace) continue;
            body.faceEdges(f, fe);
            if (fe.size() != chosen.size()) continue;
            bool all = true;
            for (EdgeId x : fe)
                if (std::find(chosen.begin(), chosen.end(), x) == chosen.end()) {
                    all = false;
                    break;
                }
            if (all) {
                r.kind = ElementRefs::Kind::FaceBoundary;
                r.face = body.faceName(f);
                return r;
            }
        }
        break;   // one edge is enough to reach every candidate face
    }

    for (EdgeId e : chosen) r.ids.push_back(body.edgeName(e));
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

    // A chain can legitimately produce no solid: a sketch on its own is a thing
    // in the scene, drawn and saved and later extruded, and refusing it would
    // mean a sketch could only exist inside a part that already had a body.
    bool drew = false;

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
            if (!makePrimitive(f.primitive, body, f.backend))
                fail(f.backend == Backend::Brep && !brep::available()
                         ? "this file needs the exact kernel, which this build does not have"
                         : "degenerate parameters");
            else any = true;
            break;

        case FeatureKind::Extrude:
            if (body.empty()) fail("nothing to extrude");
            else if (!f.faces.resolveFaces(body, scratchFaces)) fail("faces no longer exist");
            // The operation is transactional, so a rejection leaves the body as
            // it was and the chain carries on from there.
            else {
                std::string why;
                if (!extrudeFaces(body, scratchFaces, f.distance, nullptr, f.uid, f.extrudeOp,
                                  &why, f.mergeFlush, f.alongAxis ? f.axisDir : Vec3{}))
                    fail(why.empty() ? "extrude failed" : why.c_str());
            }
            break;

        case FeatureKind::Inset:
            if (body.empty()) fail("nothing to inset");
            else if (!f.faces.resolveFaces(body, scratchFaces)) fail("faces no longer exist");
            else {
                std::string why;
                if (!insetFaces(body, scratchFaces, f.amount, nullptr, f.uid, &why))
                    fail(why.empty() ? "inset too large" : why.c_str());
            }
            break;

        case FeatureKind::Bevel: {
            if (body.empty()) { fail("nothing to bevel"); break; }
            if (!f.edges.resolveEdges(body, scratchEdges)) {
                fail("edges no longer exist");
                break;
            }
            FilletSpec spec;
            spec.salt = f.uid;
            spec.chamfer = f.chamfer;
            spec.edges.reserve(scratchEdges.size());
            for (size_t i = 0; i < scratchEdges.size(); ++i)
                spec.edges.push_back({scratchEdges[i], f.radiusFor(i), f.endWidth});
            // The radius is only one of the two dozen reasons a fillet refuses,
            // and naming it unconditionally was wrong far more often than right.
            std::string reason;
            if (!filletEdges(body, spec, &reason))
                fail(reason.empty() ? "the fillet could not be built" : reason.c_str());
            break;
        }

        case FeatureKind::Shell: {
            if (body.empty()) { fail("nothing to shell"); break; }
            // An empty selection means a sealed cavity, not "every face", so it
            // resolves to nothing rather than going through resolveFaces.
            scratchFaces.clear();
            if (!f.faces.empty() && !f.faces.resolveFaces(body, scratchFaces)) {
                fail("the faces to open no longer exist");
                break;
            }
            std::string why;
            if (!shellBody(body, scratchFaces, f.thickness, f.uid, &why))
                fail(why.empty() ? "the shell could not be built" : why.c_str());
            break;
        }

        case FeatureKind::FaceScale: {
            if (body.empty()) { fail("nothing to scale"); break; }
            scratchFaces.clear();
            if (!f.faces.resolveFaces(body, scratchFaces) || scratchFaces.empty()) {
                fail("the face to scale no longer exists");
                break;
            }
            std::string why;
            if (!scaleFaces(body, scratchFaces, f.scale, f.uid, &why))
                fail(why.empty() ? "the scale could not be built" : why.c_str());
            break;
        }

        case FeatureKind::FaceRotate: {
            if (body.empty()) { fail("nothing to rotate"); break; }
            scratchFaces.clear();
            if (!f.faces.resolveFaces(body, scratchFaces) || scratchFaces.empty()) {
                fail("the face to rotate no longer exists");
                break;
            }
            std::string why;
            if (!rotateFaces(body, scratchFaces, f.angle, f.axisPoint, f.axisDir,
                             f.uid, &why))
                fail(why.empty() ? "the rotation could not be built" : why.c_str());
            break;
        }

        case FeatureKind::Merge: {
            if (body.empty()) { fail("nothing to merge"); break; }
            std::string why;
            if (!mergeDivisions(body, f.uid, &why))
                fail(why.empty() ? "the merge could not be built" : why.c_str());
            break;
        }

        case FeatureKind::Pattern: {
            if (body.empty()) { fail("nothing to pattern"); break; }
            std::string why;
            if (!patternBody(body, f.bakedBody, f.pattern(), f.uid, &why))
                fail(why.empty() ? "the pattern could not be built" : why.c_str());
            break;
        }

        case FeatureKind::Reduce: {
            if (body.empty()) { fail("nothing to reduce"); break; }
            ReduceOptions options;
            options.toleranceMm = f.reduceTolerance;
            options.targetTriangles = f.reduceTarget > 0 ? static_cast<size_t>(f.reduceTarget) : 0;
            options.loosenToReachTarget = f.reduceLoosen;
            ReduceResult result;
            Body reduced = body;
            if (!reduceBody(reduced, options, f.uid, result)) {
                fail(result.error.empty() ? "the mesh could not be reduced" : result.error.c_str());
                break;
            }
            // The tolerance is the promise. A result that could not be brought
            // within it is not quietly kept.
            if (!result.withinTolerance) {
                char why[160];
                std::snprintf(why, sizeof why,
                              "could only hold %.3g mm, not the %.3g it set out to",
                              static_cast<double>(result.deviationMm),
                              static_cast<double>(result.toleranceUsedMm));
                fail(why);
                break;
            }
            body = std::move(reduced);
            break;
        }

        case FeatureKind::Divide: {
            if (body.empty()) { fail("nothing to divide"); break; }
            std::string why;
            if (!divideBody(body, f.axisPoint, f.axisDir, f.uid, &why))
                fail(why.empty() ? "the divide could not be built" : why.c_str());
            break;
        }

        case FeatureKind::Sketch: {
            drew = true;
            // A sketch changes no body. It is solved here so that what follows
            // sees the shape its constraints describe -- and so that when its
            // constraints disagree, the sketch is the step named as failing,
            // not the extrude that happened to be built from it.
            const SketchSolve solved = solveSketch(f.sketch);
            f.sketchFreedoms = solved.freedoms;
            if (!solved.solved) fail(solved.reason.c_str());
            break;
        }

        case FeatureKind::ExtrudeProfile:
        case FeatureKind::RevolveProfile: {
            // The two differ only in what the regions are swept into: pushed
            // along the plane's normal, or turned about an axis lying in it.
            // Everything else -- finding the sketch, checking that its regions
            // still close, and what to do with the solid that comes out -- is
            // the same, and is written once.
            const bool turn = f.kind == FeatureKind::RevolveProfile;
            const Feature* source = nullptr;
            for (size_t j = 0; j < i; ++j)
                if (features[j].kind == FeatureKind::Sketch && features[j].uid == f.sketchUid)
                    source = &features[j];
            if (!source) { fail(turn ? "the sketch it turns is not earlier in the history"
                                     : "the sketch it extrudes is not earlier in the history"); break; }
            if (!source->enabled) { fail(turn ? "the sketch it turns is turned off"
                                             : "the sketch it extrudes is turned off"); break; }
            if (source->errored) { fail(turn ? "the sketch it turns does not solve"
                                            : "the sketch it extrudes does not solve"); break; }

            const std::vector<SketchProfile> regions = sketchProfiles(source->sketch);
            if (f.profileKeys.empty()) { fail(turn ? "it names no region to turn"
                                                  : "it names no region to sweep"); break; }
            const bool allClose = std::all_of(f.profileKeys.begin(), f.profileKeys.end(), [&](SketchId k) {
                return std::any_of(regions.begin(), regions.end(),
                                   [k](const SketchProfile& p) { return p.key == k; });
            });
            if (!allClose) {
                fail(f.profileKeys.size() == 1 ? "that region of the sketch no longer closes"
                                               : "a region of the sketch it sweeps no longer closes");
                break;
            }

            const bool cut = f.extrudeOp == ExtrudeOp::Cut ||
                             (!turn && f.extrudeOp == ExtrudeOp::Auto && f.distance < 0.0);
            std::string why;
            BrepRef tool = turn ? brep::revolveSketch(source->sketch, regions, f.profileKeys,
                                                      f.revolveAxisAt, f.revolveAxisDir,
                                                      f.revolveAngle, f.uid, &why)
                                : brep::sketchSolids(source->sketch, regions, f.profileKeys, 0.0,
                                                     f.distance, f.uid, &why);
            if (!tool) {
                fail(why.empty() ? (turn ? "the region could not be turned"
                                         : "the region could not be swept")
                                 : why.c_str());
                break;
            }

            if (body.empty()) {
                // The first solid in the chain, which is how a part drawn as a
                // sketch begins. There is nothing to cut from or intersect with
                // yet, and saying so beats quietly adding instead.
                if (cut || f.extrudeOp == ExtrudeOp::Intersect) {
                    fail("there is no body yet to take this away from");
                    break;
                }
                body = Body(std::move(tool));
                any = true;
                break;
            }
            if (f.extrudeOp == ExtrudeOp::NewBody) {
                fail("a separate body belongs in a separate object");
                break;
            }
            const BooleanOp op = cut ? BooleanOp::Difference
                               : f.extrudeOp == ExtrudeOp::Intersect ? BooleanOp::Intersection
                                                                     : BooleanOp::Union;
            Body combined;
            if (!booleanOp(body, Body(std::move(tool)), op, combined, f.uid, false, &why))
                fail(why.empty() ? (turn ? "the turned solid could not be combined with the body"
                                         : "the extrusion could not be combined with the body")
                                 : why.c_str());
            else
                body = std::move(combined);
            break;
        }

        case FeatureKind::Hole: {
            if (body.empty()) { fail("there is no body to drill"); break; }
            Vec3 at = f.axisPoint;
            Vec3 into = f.axisDir;
            if (length(into) < 1e-9) { fail("the hole has no direction to go in"); break; }
            into = normalize(into);

            // A hole belongs to the face it was drilled into: if that face has
            // moved or tipped since, the hole goes with it, square to it and
            // in the same place on it. The point is carried onto the face
            // along the old axis, which is exact for a face that moved and
            // right for one that tipped a little.
            std::vector<FaceId> fs;
            if (!f.faces.empty()) {
                if (!f.faces.resolveFaces(body, fs) || fs.empty()) {
                    fail("the face it was drilled into is gone");
                    break;
                }
                const Vec3 n = body.faceNormal(fs.front());
                if (length(n) > 1e-9) {
                    const Vec3 unit = normalize(n);
                    const Real facing = dot(unit, into);
                    if (std::fabs(facing) > 1e-6) {
                        const Real t = dot(body.faceCentroid(fs.front()) - at, unit) / facing;
                        at = at + into * t;
                    }
                    into = unit * Real(-1.0);
                }
            }
            std::string why;
            if (!drillHole(body, at, into, f.hole, f.uid, &why))
                fail(why.empty() ? "the hole could not be drilled" : why.c_str());
            break;
        }

        case FeatureKind::Move:
        case FeatureKind::Rotate:
            // Where the object stands, not what it is: the body goes through
            // untouched, and the scene composes these onto its placement.
            break;

        case FeatureKind::Scale:
            if (body.empty()) { fail("there is no body yet to scale"); break; }
            if (!(f.scaleBy.x > 0.0) || !(f.scaleBy.y > 0.0) || !(f.scaleBy.z > 0.0)) {
                fail("a scale has to be above zero every way: a negative one is a mirror");
                break;
            }
            if (!body.scale(f.scaleBy, f.scaleAbout)) fail("the body could not be scaled");
            break;

        case FeatureKind::BaseMesh:
            if (f.bakedBody.empty()) fail("no geometry");
            else { body = f.bakedBody; any = true; }
            break;

        case FeatureKind::Boolean: {
            if (body.empty()) { fail("nothing to combine with"); break; }
            if (f.bakedBody.empty()) { fail("tool body is missing"); break; }
            Body combined;
            std::string why;
            if (!booleanOp(body, f.bakedBody, f.booleanOp, combined, f.uid, false, &why))
                fail(why.empty() ? "boolean produced no valid solid" : why.c_str());
            else
                body = std::move(combined);
            break;
        }

        case FeatureKind::VertexEdit: {
            if (body.empty()) { fail("nothing to edit"); break; }
            if (!body.canMoveVertices()) {
                // A vertex on an exact body is where surfaces meet, not a free
                // point. Refusing is the honest answer; approximating would
                // quietly turn the body into something it is not.
                fail("an exact body has no free vertices to move");
                break;
            }

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

    // Nothing but sketches: the chain ran, and what it drew is all there is.
    if (!any && drew) {
        out = Body{};
        return true;
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
