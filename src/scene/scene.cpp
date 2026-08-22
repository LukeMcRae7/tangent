#include "scene/scene.h"

#include <algorithm>
#include <limits>

namespace tg {

const char* primitiveName(PrimitiveKind k) {
    switch (k) {
        case PrimitiveKind::Box:      return "Box";
        case PrimitiveKind::Cylinder: return "Cylinder";
        case PrimitiveKind::Sphere:   return "Sphere";
        case PrimitiveKind::Cone:     return "Cone";
        case PrimitiveKind::Torus:    return "Torus";
        case PrimitiveKind::Plane:    return "Plane";
        case PrimitiveKind::Custom:   return "Mesh";
    }
    return "Object";
}

namespace {

bool generate(const PrimitiveSpec& spec, Mesh& out) {
    switch (spec.kind) {
        case PrimitiveKind::Box:      return makeBox(out, spec.box);
        case PrimitiveKind::Cylinder: return makeCylinder(out, spec.cylinder);
        case PrimitiveKind::Sphere:   return makeSphere(out, spec.sphere);
        case PrimitiveKind::Cone:     return makeCone(out, spec.cone);
        case PrimitiveKind::Torus:    return makeTorus(out, spec.torus);
        case PrimitiveKind::Plane:    return makePlane(out, spec.plane);
        case PrimitiveKind::Custom:   return false;   // edited meshes are not regenerated
    }
    return false;
}

} // namespace

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

    if (!generate(obj->spec, obj->mesh)) return kNoObject;

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
    return true;
}

ObjectId Scene::duplicateObject(ObjectId id) {
    const SceneObject* src = find(id);
    if (!src) return kNoObject;

    auto obj = std::make_unique<SceneObject>();
    obj->spec        = src->spec;
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
    nextId_ = 1;
}

bool Scene::rebuild(ObjectId id) {
    SceneObject* obj = find(id);
    if (!obj) return false;

    // Build into a scratch mesh first: degenerate parameters must not destroy
    // the geometry the user can still see.
    Mesh next;
    if (!generate(obj->spec, next)) return false;

    obj->mesh = std::move(next);
    obj->mesh.buildRenderMesh(obj->render);
    obj->localBounds = obj->mesh.bounds();
    obj->markMeshChanged();
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

        float boxT = 0.0f;
        if (!rayAABB(local, o->localBounds, boxT)) continue;
        if (boxT / dirScale > bestT) continue;   // whole object is behind a closer hit

        const RenderMesh& rm = o->render;
        for (size_t i = 0; i + 2 < rm.triangles.size(); i += 3) {
            float t = 0.0f;
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
