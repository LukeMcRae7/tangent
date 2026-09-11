#include "scene/scene.h"

#include <algorithm>
#include <limits>

namespace tg {

const char* elementKindName(ElementKind k) {
    switch (k) {
        case ElementKind::Vertex: return "Vertex";
        case ElementKind::Edge:   return "Edge";
        case ElementKind::Face:   return "Face";
        case ElementKind::None:   return "None";
    }
    return "None";
}

// ---------------------------------------------------------------------------
AABB SceneObject::worldBounds() const {
    AABB out;
    if (!localBounds.valid()) return out;
    // Transform all eight corners: rotating the local box and re-fitting is
    // correct, whereas transforming only min/max is not.
    const Mat4 m = modelMatrix();
    const Vec3 lo = localBounds.min, hi = localBounds.max;
    for (int i = 0; i < 8; ++i) {
        const Vec3 corner{(i & 1) ? hi.x : lo.x,
                          (i & 2) ? hi.y : lo.y,
                          (i & 4) ? hi.z : lo.z};
        out.expand(transformPoint(m, corner));
    }
    return out;
}

// ---------------------------------------------------------------------------
std::string Scene::uniqueName(const std::string& base) const {
    bool taken = false;
    for (const auto& o : objects_) if (o->name == base) { taken = true; break; }
    if (!taken) return base;

    // Blender-style numeric suffix: Box, Box.001, Box.002 ...
    for (int n = 1; n < 10000; ++n) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%s.%03d", base.c_str(), n);
        std::string candidate(buf);
        bool used = false;
        for (const auto& o : objects_) if (o->name == candidate) { used = true; break; }
        if (!used) return candidate;
    }
    return base;
}

ObjectId Scene::addPrimitive(PrimitiveKind kind, const PrimitiveSpec& spec, Vec3 position) {
    auto obj = std::make_unique<SceneObject>();
    obj->spec = spec;
    obj->spec.kind = kind;

    Feature base;
    base.kind = FeatureKind::Primitive;
    base.primitive = obj->spec;
    base.backend = defaultBackend_;
    base.uid = nextFeatureUid_++;
    obj->features.push_back(base);

    // A plane is a surface, not a solid, and the exact kernel builds solids.
    // Falling back keeps the construction plane working rather than refusing
    // to make one at all.
    if (base.backend == Backend::Brep && kind == PrimitiveKind::Plane)
        obj->features.back().backend = Backend::Mesh;

    if (!evaluateFeatures(obj->features, obj->body)) return kNoObject;

    obj->id = nextId_++;
    obj->name = uniqueName(primitiveName(kind));
    obj->transform.position = position;
    obj->body.tessellate(obj->render);
    obj->localBounds = obj->body.bounds();

    const ObjectId id = obj->id;
    objects_.push_back(std::move(obj));
    return id;
}

ObjectId Scene::addBody(Body body, Vec3 position, const std::string& name) {
    if (body.empty()) return kNoObject;
    auto obj = std::make_unique<SceneObject>();
    obj->spec.kind = PrimitiveKind::Custom;
    obj->body = std::move(body);

    Feature base;
    base.kind = FeatureKind::BaseMesh;
    base.bakedBody = obj->body;
    base.uid = nextFeatureUid_++;
    obj->features.push_back(base);

    obj->id = nextId_++;
    obj->name = uniqueName(name.empty() ? "Object" : name);
    obj->transform.position = position;
    obj->body.tessellate(obj->render);
    obj->localBounds = obj->body.bounds();

    const ObjectId id = obj->id;
    objects_.push_back(std::move(obj));
    return id;
}

bool Scene::removeObject(ObjectId id) {
    auto it = std::find_if(objects_.begin(), objects_.end(),
                           [&](const auto& o) { return o->id == id; });
    if (it == objects_.end()) return false;
    objects_.erase(it);
    selection_.erase(std::remove(selection_.begin(), selection_.end(), id), selection_.end());
    elements_.erase(std::remove_if(elements_.begin(), elements_.end(),
        [id](const ElementRef& e) { return e.object == id; }), elements_.end());
    return true;
}

ObjectId Scene::duplicateObject(ObjectId id) {
    const SceneObject* src = find(id);
    if (!src) return kNoObject;

    auto obj = std::make_unique<SceneObject>();
    obj->spec        = src->spec;
    obj->features    = src->features;   // cache is left cold: copying every
                                        // intermediate mesh would cost more
                                        // than re-running the chain once
    obj->transform   = src->transform;
    obj->body        = src->body;
    obj->render      = src->render;
    obj->localBounds = src->localBounds;
    obj->visible     = src->visible;
    obj->id          = nextId_++;
    // Strip any existing .NNN suffix so copies of Box.001 become Box.002.
    std::string base = src->name;
    if (base.size() > 4 && base[base.size() - 4] == '.' &&
        std::all_of(base.end() - 3, base.end(), [](char c) { return c >= '0' && c <= '9'; })) {
        base.resize(base.size() - 4);
    }
    obj->name = uniqueName(base);

    const ObjectId newId = obj->id;
    objects_.push_back(std::move(obj));
    return newId;
}

std::unique_ptr<SceneObject> Scene::takeObject(ObjectId id) {
    auto it = std::find_if(objects_.begin(), objects_.end(),
                           [&](const auto& o) { return o->id == id; });
    if (it == objects_.end()) return nullptr;

    std::unique_ptr<SceneObject> out = std::move(*it);
    objects_.erase(it);
    selection_.erase(std::remove(selection_.begin(), selection_.end(), id), selection_.end());
    elements_.erase(std::remove_if(elements_.begin(), elements_.end(),
        [id](const ElementRef& e) { return e.object == id; }), elements_.end());
    return out;
}

void Scene::insertObject(std::unique_ptr<SceneObject> obj) {
    if (!obj) return;
    // Keep the id allocator ahead of anything restored, so a later create
    // cannot collide with an object that undo brought back.
    if (obj->id >= nextId_) nextId_ = obj->id + 1;
    objects_.push_back(std::move(obj));
}

SceneObject* Scene::find(ObjectId id) {
    for (auto& o : objects_) if (o->id == id) return o.get();
    return nullptr;
}

const SceneObject* Scene::find(ObjectId id) const {
    for (const auto& o : objects_) if (o->id == id) return o.get();
    return nullptr;
}

void Scene::clear() {
    objects_.clear();
    selection_.clear();
    elements_.clear();
    nextId_ = 1;
}

bool Scene::rebuild(ObjectId id) {
    SceneObject* obj = find(id);
    if (!obj) return false;

    // The inspector edits obj->spec directly, so push that into the base
    // feature before evaluating; the chain is the authority, not the spec.
    for (Feature& f : obj->features)
        if (f.kind == FeatureKind::Primitive) { f.primitive = obj->spec; break; }

    return reevaluate(id);
}

void Scene::noteNewFailures(const SceneObject& obj,
                            const std::vector<ElementId>& wasBroken) {
    const Feature* first = nullptr;
    int count = 0;
    for (const Feature& f : obj.features) {
        if (!f.errored) continue;
        if (std::find(wasBroken.begin(), wasBroken.end(), f.uid) != wasBroken.end()) continue;
        if (!first) first = &f;
        ++count;
    }
    if (!first) return;

    chainNotice_ = std::string(first->displayKind()) + " failed";
    if (!first->error.empty()) chainNotice_ += ": " + first->error;
    if (count > 1) chainNotice_ += " (and " + std::to_string(count - 1) + " more)";
}

bool Scene::reevaluateFrom(ObjectId id, size_t fromFeature) {
    SceneObject* obj = find(id);
    if (!obj) return false;

    // What was already broken before this run, so that only a step that has
    // newly failed is reported. Held by uid rather than by position: a chain
    // can be reordered between runs.
    std::vector<ElementId> wasBroken;
    for (const Feature& f : obj->features)
        if (f.errored) wasBroken.push_back(f.uid);

    // Evaluate into a scratch body: a chain that produces nothing must not
    // destroy the geometry the user can still see.
    Body next;
    const bool ok = evaluateFrom(obj->features, fromFeature, obj->featureCache, next);

    // Either way: evaluateFrom has marked each step it ran, and a chain that
    // produced nothing at all is exactly the case worth saying something about.
    noteNewFailures(*obj, wasBroken);
    if (!ok) return false;

    obj->body = std::move(next);
    obj->refreshDerived();
    // Face numbering does not survive a re-evaluation.
    pruneElementSelection();
    return true;
}

bool Scene::addFeature(ObjectId id, Feature feature, std::string* error) {
    if (error) error->clear();
    SceneObject* obj = find(id);
    if (!obj) return false;

    if (feature.uid == 0) feature.uid = nextFeatureUid_++;
    obj->features.push_back(std::move(feature));

    // Only the new feature needs running; everything before it is cached.
    Body next;
    if (!evaluateFrom(obj->features, obj->features.size() - 1,
                      obj->featureCache, next)) {
        if (error) *error = obj->features.back().error;
        obj->features.pop_back();
        return false;
    }
    // A feature that evaluated but errored did nothing; keeping it would leave
    // a step in the timeline that has no effect and cannot be fixed.
    if (obj->features.back().errored) {
        if (error) *error = obj->features.back().error;
        obj->features.pop_back();
        obj->featureCache.resize(obj->features.size());
        return false;
    }

    obj->body = std::move(next);
    obj->refreshDerived();
    pruneElementSelection();
    return true;
}

// ---------------------------------------------------------------------------
bool Scene::isSelected(ObjectId id) const {
    return std::find(selection_.begin(), selection_.end(), id) != selection_.end();
}

void Scene::clearSelection() { selection_.clear(); }

void Scene::select(ObjectId id, bool additive) {
    if (!additive) selection_.clear();
    if (id == kNoObject) return;
    if (!isSelected(id)) selection_.push_back(id);
    else {
        // Re-selecting promotes to active, matching how the inspector follows
        // the most recently clicked object.
        selection_.erase(std::remove(selection_.begin(), selection_.end(), id), selection_.end());
        selection_.push_back(id);
    }
}

void Scene::toggleSelect(ObjectId id) {
    if (id == kNoObject) return;
    if (isSelected(id))
        selection_.erase(std::remove(selection_.begin(), selection_.end(), id), selection_.end());
    else
        selection_.push_back(id);
}

void Scene::selectAll() {
    selection_.clear();
    for (const auto& o : objects_) if (o->visible) selection_.push_back(o->id);
}

// ---------------------------------------------------------------------------
AABB Scene::bounds() const {
    AABB b;
    for (const auto& o : objects_) if (o->visible) b.expand(o->worldBounds());
    return b;
}

AABB Scene::selectionBounds() const {
    AABB b;
    for (ObjectId id : selection_)
        if (const SceneObject* o = find(id)) b.expand(o->worldBounds());
    return b;
}

Vec3 Scene::selectionCenter() const {
    const AABB b = selectionBounds();
    return b.valid() ? b.center() : Vec3{};
}

namespace {

// Projects to pixels using the same convention as Camera::projectToPixel:
// origin top-left, Y downward. Returns false behind the camera.
bool projectPx(const Mat4& viewProj, int w, int h, Vec3 world, Vec2& out) {
    const Vec4 clip = viewProj * Vec4(world, 1.0f);
    if (clip.w <= 1e-6f) return false;
    const Vec3 ndc = clip.xyz() / clip.w;
    out = {(ndc.x * 0.5f + 0.5f) * static_cast<float>(w),
           (1.0f - (ndc.y * 0.5f + 0.5f)) * static_cast<float>(h)};
    return true;
}

float distToSegment(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const float len2 = lengthSq(ab);
    if (len2 < 1e-9f) return length(p - a);
    const float t = clampf(dot(p - a, ab) / len2, 0.0f, 1.0f);
    return length(p - (a + ab * t));
}

// Pixel distance from the cursor to an edge *as it is drawn*: along the curve,
// rather than across the chord between its two ends. Negative when nothing
// projected in front of the camera.
//
// This is where measuring the chord failed worst. A closed rim's two ends are
// the same point, so the chord collapsed to that point and the only part of a
// whole circle that could be clicked was the handful of pixels around its
// seam. Sampled fine enough to stay inside a pixel at any zoom worth clicking
// at; picking runs on a click and not per frame, so this is not a frame cost.
float distToEdgePx(const Body& body, const Mat4& model, EdgeId e,
                   const Mat4& viewProj, int w, int h, Vec2 cursorPx,
                   std::vector<Vec3>& scratch) {
    body.edgePolyline(e, body.edgeLength(e) * 0.0005, scratch);

    float best = -1.0f;
    Vec2 prev{};
    bool havePrev = false;
    for (const Vec3& local : scratch) {
        Vec2 px;
        if (!projectPx(viewProj, w, h, transformPoint(model, local), px)) {
            havePrev = false;
            continue;
        }
        if (havePrev) {
            const float d = distToSegment(cursorPx, prev, px);
            if (best < 0.0f || d < best) best = d;
        }
        prev = px;
        havePrev = true;
    }
    // One lone point still has a distance: a degenerate edge should not become
    // unpickable just because it has no length to measure along.
    if (best < 0.0f && havePrev) best = length(cursorPx - prev);
    return best;
}

} // namespace

ElementHit Scene::pickElement(const Ray& ray, const Mat4& viewProj,
                              int viewportW, int viewportH, Vec2 cursorPx,
                              float vertexTolPx, float edgeTolPx) const {
    ElementHit out;

    const RayHit surface = raycast(ray);
    if (surface.hit()) {
        const SceneObject* obj = find(surface.object);
        if (obj && surface.face != kInvalid) {
            out.ref = {surface.object, ElementKind::Face, surface.face};
            out.t = surface.t;
            out.point = surface.point;

            const Mat4 model = obj->modelMatrix();
            const Body& body = obj->body;

            float bestVert = vertexTolPx, bestEdge = edgeTolPx;
            VertexId vertPick = kInvalid;
            EdgeId   edgePick = kInvalid;

            auto atPixel = [&](Vec3 local, Vec2& px) {
                return projectPx(viewProj, viewportW, viewportH,
                                 transformPoint(model, local), px);
            };

            std::vector<VertexId> fv;
            body.faceVertices(surface.face, fv);
            for (VertexId v : fv) {
                Vec2 p;
                if (!atPixel(body.vertexPosition(v), p)) continue;
                const float d = length(cursorPx - p);
                if (d < bestVert) { bestVert = d; vertPick = v; }
            }

            std::vector<EdgeId> fe;
            std::vector<Vec3> edgePts;
            body.faceEdges(surface.face, fe);
            for (EdgeId e : fe) {
                // A cut that only exists because a face cannot hold a hole is
                // not drawn, so it must not be pickable either -- clicking one
                // would select a line the user cannot see.
                if (body.isBridgeEdge(e)) continue;
                const float d = distToEdgePx(body, model, e, viewProj, viewportW,
                                             viewportH, cursorPx, edgePts);
                if (d >= 0.0f && d < bestEdge) { bestEdge = d; edgePick = e; }
            }

            if (vertPick != kInvalid)      out.ref = {surface.object, ElementKind::Vertex, vertPick};
            else if (edgePick != kInvalid) out.ref = {surface.object, ElementKind::Edge, edgePick};
            return out;
        }
    }

    // If raycast missed or didn't hit a surface, check nearby vertices and edges
    // of visible objects on screen (off-silhouette generous picking).
    // A vertex hit and an edge hit are tracked with their own owning object.
    // Sharing one `bestObj` between them meant a vertex winning on one object
    // and an edge later winning on another returned that second object with the
    // first one's vertex handle.
    float bestVert = vertexTolPx, bestEdge = edgeTolPx;
    ObjectId vertObj = kNoObject, edgeObj = kNoObject;
    VertexId vertPick = kInvalid;
    EdgeId   edgePick = kInvalid;

    std::vector<VertexId> verts;
    std::vector<EdgeId> edges;
    std::vector<Vec3> edgePts;
    for (const auto& obj : objects_) {
        if (!obj->visible || obj->body.empty()) continue;
        const Mat4 model = obj->modelMatrix();
        const Body& body = obj->body;

        auto atPixel = [&](Vec3 local, Vec2& px) {
            return projectPx(viewProj, viewportW, viewportH,
                             transformPoint(model, local), px);
        };

        body.allVertices(verts);
        for (VertexId v : verts) {
            Vec2 p;
            if (!atPixel(body.vertexPosition(v), p)) continue;
            const float d = length(cursorPx - p);
            if (d < bestVert) { bestVert = d; vertPick = v; vertObj = obj->id; }
        }

        body.allEdges(edges);
        for (EdgeId e : edges) {
            if (body.isBridgeEdge(e)) continue;
            const float d = distToEdgePx(body, model, e, viewProj, viewportW,
                                         viewportH, cursorPx, edgePts);
            if (d >= 0.0f && d < bestEdge) { bestEdge = d; edgePick = e; edgeObj = obj->id; }
        }
    }

    if (vertPick != kInvalid && vertObj != kNoObject) {
        out.ref = {vertObj, ElementKind::Vertex, vertPick};
    } else if (edgePick != kInvalid && edgeObj != kNoObject) {
        out.ref = {edgeObj, ElementKind::Edge, edgePick};
    }

    return out;
}

// ---------------------------------------------------------------------------
bool Scene::isElementSelected(const ElementRef& e) const {
    return std::find(elements_.begin(), elements_.end(), e) != elements_.end();
}

// Picking one piece of a face that was split because a face cannot have a hole
// selects the whole of it. The pieces are joined by cuts that are not drawn and
// cannot be clicked, so anything else would have the user selecting two thirds
// of a surface with no way to see why.
//
// Only those cuts are crossed. A face the user divided -- the section line an
// extrude leaves down a wall -- stays two faces, picked and acted on
// separately, which is what it is for.
std::vector<ElementRef> Scene::faceGroup(const ElementRef& e) const {
    if (e.kind != ElementKind::Face) return {e};
    const SceneObject* o = find(e.object);
    if (!o || e.index < 0 || e.index >= o->body.faceCount()) return {e};

    std::vector<Index> group;
    o->body.coplanarFaceGroup(e.index, group);
    std::vector<ElementRef> out;
    out.reserve(group.size());
    for (Index f : group) out.push_back({e.object, ElementKind::Face, f});
    return out;
}

void Scene::selectElement(const ElementRef& e, bool additive) {
    if (!additive) elements_.clear();
    if (!e.valid()) return;
    for (const ElementRef& r : faceGroup(e))
        if (!isElementSelected(r)) elements_.push_back(r);
}

void Scene::toggleElement(const ElementRef& e) {
    if (!e.valid()) return;
    // The group goes in and out together, or a shift-click would peel one
    // invisible piece off a face and leave the rest selected.
    const std::vector<ElementRef> group = faceGroup(e);
    if (isElementSelected(e)) {
        elements_.erase(std::remove_if(elements_.begin(), elements_.end(),
                            [&](const ElementRef& x) {
                                return std::find(group.begin(), group.end(), x) != group.end();
                            }),
                        elements_.end());
        return;
    }
    for (const ElementRef& r : group)
        if (!isElementSelected(r)) elements_.push_back(r);
}

std::vector<Index> Scene::selectedFaces(ObjectId id) const {
    std::vector<Index> out;
    for (const ElementRef& e : elements_)
        if (e.object == id && e.kind == ElementKind::Face) out.push_back(e.index);
    return out;
}

std::vector<EdgeId> Scene::selectedEdges(ObjectId id) const {
    std::vector<EdgeId> out;
    const SceneObject* o = find(id);
    if (!o) return out;
    for (const ElementRef& e : elements_) {
        if (e.object != id || e.kind != ElementKind::Edge) continue;
        // Handles are already canonical wherever they came from, so this is a
        // validity check and a de-duplication, not a normalisation.
        if (!o->body.hasEdge(e.index)) continue;
        if (std::find(out.begin(), out.end(), e.index) == out.end())
            out.push_back(e.index);
    }
    return out;
}

void Scene::pruneElementSelection() {
    elements_.erase(std::remove_if(elements_.begin(), elements_.end(),
        [this](const ElementRef& e) {
            const SceneObject* o = find(e.object);
            if (!o) return true;
            switch (e.kind) {
                case ElementKind::Face:   return !o->body.hasFace(e.index);
                case ElementKind::Edge:   return !o->body.hasEdge(e.index);
                case ElementKind::Vertex: return !o->body.hasVertex(e.index);
                case ElementKind::None:   return true;
            }
            return true;
        }), elements_.end());
}

RayHit Scene::raycast(const Ray& ray) const {
    RayHit best;
    float bestT = std::numeric_limits<float>::max();

    for (const auto& o : objects_) {
        if (!o->visible || o->render.triangles.empty()) continue;

        // Test in object space so the mesh needs no per-frame transformation.
        const Mat4 model = o->modelMatrix();
        const Mat4 inv   = inverse(model);
        Ray local{transformPoint(inv, ray.origin), transformVector(inv, ray.dir)};

        // Non-uniform scale makes the local direction non-unit; normalising it
        // keeps `t` measured in local units, converted back to world below.
        const float dirScale = length(local.dir);
        if (dirScale < 1e-9f) continue;
        local.dir = local.dir / dirScale;

        Real boxT = 0.0;
        if (!rayAABB(local, o->localBounds, boxT)) continue;
        if (boxT / dirScale > bestT) continue;   // whole object is behind a closer hit

        const RenderMesh& rm = o->render;
        for (size_t i = 0; i + 2 < rm.triangles.size(); i += 3) {
            Real t = 0.0;
            if (!rayTriangle(local, rm.positions[rm.triangles[i + 0]],
                                    rm.positions[rm.triangles[i + 1]],
                                    rm.positions[rm.triangles[i + 2]], t)) continue;

            const float worldT = t / dirScale;
            if (worldT >= bestT) continue;

            bestT = worldT;
            best.object = o->id;
            best.face   = rm.triangleFace[i / 3];
            best.t      = worldT;
            best.point  = ray.origin + ray.dir * worldT;
            best.normal = normalize(transformVector(normalMatrix(model),
                                                    o->body.faceNormal(best.face)));
        }
    }
    return best;
}

} // namespace tg
