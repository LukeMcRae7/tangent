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
    base.uid = nextFeatureUid_++;
    obj->features.push_back(base);

    if (!evaluateFeatures(obj->features, obj->mesh)) return kNoObject;

    obj->id = nextId_++;
    obj->name = uniqueName(primitiveName(kind));
    obj->transform.position = position;
    obj->mesh.buildRenderMesh(obj->render);
    obj->localBounds = obj->mesh.bounds();

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
    obj->mesh        = src->mesh;
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

bool Scene::reevaluateFrom(ObjectId id, size_t fromFeature) {
    SceneObject* obj = find(id);
    if (!obj) return false;

    // Evaluate into a scratch mesh: a chain that produces nothing must not
    // destroy the geometry the user can still see.
    Mesh next;
    if (!evaluateFrom(obj->features, fromFeature, obj->featureCache, next)) return false;

    obj->mesh = std::move(next);
    obj->refreshDerived();
    // Face numbering does not survive a re-evaluation.
    pruneElementSelection();
    return true;
}

bool Scene::addFeature(ObjectId id, Feature feature) {
    SceneObject* obj = find(id);
    if (!obj) return false;

    if (feature.uid == 0) feature.uid = nextFeatureUid_++;
    obj->features.push_back(std::move(feature));

    // Only the new feature needs running; everything before it is cached.
    Mesh next;
    if (!evaluateFrom(obj->features, obj->features.size() - 1,
                      obj->featureCache, next)) {
        obj->features.pop_back();
        return false;
    }
    // A feature that evaluated but errored did nothing; keeping it would leave
    // a step in the timeline that has no effect and cannot be fixed.
    if (obj->features.back().errored) {
        obj->features.pop_back();
        obj->featureCache.resize(obj->features.size());
        return false;
    }

    obj->mesh = std::move(next);
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
            const Mesh& mesh = obj->mesh;

            float bestVert = vertexTolPx, bestEdge = edgeTolPx;
            Index vertPick = kInvalid, edgePick = kInvalid;

            const Index start = mesh.faces[surface.face].halfedge;
            Index h = start;
            do {
                const Index v0 = mesh.fromVertex(h);
                const Index v1 = mesh.halfedges[h].vertex;

                Vec2 p0, p1;
                const bool ok0 = projectPx(viewProj, viewportW, viewportH,
                                           transformPoint(model, mesh.verts[v0].position), p0);
                const bool ok1 = projectPx(viewProj, viewportW, viewportH,
                                           transformPoint(model, mesh.verts[v1].position), p1);

                if (ok0) {
                    const float d = length(cursorPx - p0);
                    if (d < bestVert) { bestVert = d; vertPick = v0; }
                }
                if (ok0 && ok1) {
                    const float d = distToSegment(cursorPx, p0, p1);
                    if (d < bestEdge) {
                        bestEdge = d;
                        edgePick = std::min(h, mesh.halfedges[h].twin);
                    }
                }
                h = mesh.halfedges[h].next;
            } while (h != start);

            if (vertPick != kInvalid)      out.ref = {surface.object, ElementKind::Vertex, vertPick};
            else if (edgePick != kInvalid) out.ref = {surface.object, ElementKind::Edge, edgePick};
            return out;
        }
    }

    // If raycast missed or didn't hit a surface, check nearby vertices and edges
    // of visible objects on screen (off-silhouette generous picking).
    float bestVert = vertexTolPx, bestEdge = edgeTolPx;
    ObjectId bestObj = kNoObject;
    Index vertPick = kInvalid, edgePick = kInvalid;

    for (const auto& obj : objects_) {
        if (!obj->visible || obj->mesh.empty()) continue;
        const Mat4 model = obj->modelMatrix();
        const Mesh& mesh = obj->mesh;

        for (Index v = 0; v < mesh.vertexCount(); ++v) {
            Vec2 p;
            if (projectPx(viewProj, viewportW, viewportH,
                          transformPoint(model, mesh.verts[v].position), p)) {
                const float d = length(cursorPx - p);
                if (d < bestVert) {
                    bestVert = d;
                    vertPick = v;
                    bestObj = obj->id;
                }
            }
        }

        for (Index h = 0; h < mesh.halfedgeCount(); ++h) {
            if (h > mesh.halfedges[h].twin) continue;
            const Index v0 = mesh.fromVertex(h);
            const Index v1 = mesh.halfedges[h].vertex;
            Vec2 p0, p1;
            const bool ok0 = projectPx(viewProj, viewportW, viewportH,
                                       transformPoint(model, mesh.verts[v0].position), p0);
            const bool ok1 = projectPx(viewProj, viewportW, viewportH,
                                       transformPoint(model, mesh.verts[v1].position), p1);
            if (ok0 && ok1) {
                const float d = distToSegment(cursorPx, p0, p1);
                if (d < bestEdge) {
                    bestEdge = d;
                    edgePick = h;
                    bestObj = obj->id;
                }
            }
        }
    }

    if (vertPick != kInvalid && bestObj != kNoObject) {
        out.ref = {bestObj, ElementKind::Vertex, vertPick};
    } else if (edgePick != kInvalid && bestObj != kNoObject) {
        out.ref = {bestObj, ElementKind::Edge, edgePick};
    }

    return out;
}

// ---------------------------------------------------------------------------
bool Scene::isElementSelected(const ElementRef& e) const {
    return std::find(elements_.begin(), elements_.end(), e) != elements_.end();
}

void Scene::selectElement(const ElementRef& e, bool additive) {
    if (!additive) elements_.clear();
    if (!e.valid()) return;
    if (!isElementSelected(e)) elements_.push_back(e);
}

void Scene::toggleElement(const ElementRef& e) {
    if (!e.valid()) return;
    auto it = std::find(elements_.begin(), elements_.end(), e);
    if (it != elements_.end()) elements_.erase(it);
    else elements_.push_back(e);
}

std::vector<Index> Scene::selectedFaces(ObjectId id) const {
    std::vector<Index> out;
    for (const ElementRef& e : elements_)
        if (e.object == id && e.kind == ElementKind::Face) out.push_back(e.index);
    return out;
}

std::vector<Index> Scene::selectedEdges(ObjectId id) const {
    std::vector<Index> out;
    const SceneObject* o = find(id);
    if (!o) return out;
    for (const ElementRef& e : elements_) {
        if (e.object != id || e.kind != ElementKind::Edge) continue;
        if (e.index < 0 || e.index >= o->mesh.halfedgeCount()) continue;
        const Index canonical = std::min(e.index, o->mesh.halfedges[e.index].twin);
        if (std::find(out.begin(), out.end(), canonical) == out.end())
            out.push_back(canonical);
    }
    return out;
}

void Scene::pruneElementSelection() {
    elements_.erase(std::remove_if(elements_.begin(), elements_.end(),
        [this](const ElementRef& e) {
            const SceneObject* o = find(e.object);
            if (!o) return true;
            switch (e.kind) {
                case ElementKind::Face:   return e.index >= o->mesh.faceCount();
                case ElementKind::Edge:   return e.index >= o->mesh.halfedgeCount();
                case ElementKind::Vertex: return e.index >= o->mesh.vertexCount();
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
                                                    o->mesh.faceNormal(best.face)));
        }
    }
    return best;
}

} // namespace tg
