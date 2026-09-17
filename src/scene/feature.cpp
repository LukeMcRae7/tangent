#include "scene/feature.h"

#include "mesh/decimate.h"

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
        case FeatureKind::Shell:      return "Shell";
        case FeatureKind::FaceRotate: return "Rotate Face";
        case FeatureKind::FaceScale:  return "Scale Face";
        case FeatureKind::Divide:     return "Divide";
        case FeatureKind::Merge:      return "Merge Faces";
        case FeatureKind::Pattern:    return "Pattern";
        case FeatureKind::Reduce:     return "Reduce Mesh";
        case FeatureKind::Sketch:     return "Sketch";
        case FeatureKind::ExtrudeProfile: return "Extrude Profile";
    }
    return "Feature";
}

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
        case FeatureKind::ExtrudeProfile:
            std::snprintf(buf, sizeof(buf), "Extrude Profile  %.2f mm  (%s)",
                          static_cast<double>(distance), extrudeOpName(extrudeOp));
            break;
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
                                  &why, f.mergeFlush))
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
            // A sketch changes no body. It is solved here so that what follows
            // sees the shape its constraints describe -- and so that when its
            // constraints disagree, the sketch is the step named as failing,
            // not the extrude that happened to be built from it.
            const SketchSolve solved = solveSketch(f.sketch);
            f.sketchFreedoms = solved.freedoms;
            if (!solved.solved) fail(solved.reason.c_str());
            break;
        }

        case FeatureKind::ExtrudeProfile: {
            const Feature* source = nullptr;
            for (size_t j = 0; j < i; ++j)
                if (features[j].kind == FeatureKind::Sketch && features[j].uid == f.sketchUid)
                    source = &features[j];
            if (!source) { fail("the sketch it extrudes is not earlier in the history"); break; }
            if (!source->enabled) { fail("the sketch it extrudes is turned off"); break; }
            if (source->errored) { fail("the sketch it extrudes does not solve"); break; }

            const std::vector<SketchProfile> regions = sketchProfiles(source->sketch);
            const auto region = std::find_if(regions.begin(), regions.end(),
                [&](const SketchProfile& p) { return p.key == f.profileKey; });
            if (region == regions.end()) {
                fail("that region of the sketch no longer closes");
                break;
            }

            const bool cut = f.extrudeOp == ExtrudeOp::Cut ||
                             (f.extrudeOp == ExtrudeOp::Auto && f.distance < 0.0);
            std::string why;
            BrepRef tool = brep::sketchSolid(source->sketch, *region, 0.0, f.distance, f.uid, &why);
            if (!tool) {
                fail(why.empty() ? "the region could not be swept" : why.c_str());
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
                fail(why.empty() ? "the extrusion could not be combined with the body" : why.c_str());
            else
                body = std::move(combined);
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

    if (!any || body.empty()) return false;
    out = std::move(body);
    return true;
}

bool evaluateFeatures(std::vector<Feature>& features, Body& out) {
    std::vector<Body> scratch;
    return evaluateFrom(features, 0, scratch, out);
}

} // namespace tg
