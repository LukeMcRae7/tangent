#include "app/transform_tool.h"
#include "core/palette.h"

#include <cstdio>
#include <cstdlib>

namespace tg {
namespace {

// Parameter along the line (origin + dir * s) closest to the given ray. This
// is what makes axis-constrained dragging track the cursor exactly instead of
// approximating with a screen-space projection.
bool closestOnLine(const Ray& ray, Vec3 origin, Vec3 dir, float& s) {
    const Vec3 w0 = origin - ray.origin;
    const float a = dot(dir, dir);
    const float b = dot(dir, ray.dir);
    const float c = dot(ray.dir, ray.dir);
    const float d = dot(dir, w0);
    const float e = dot(ray.dir, w0);
    const float denom = a * c - b * b;
    if (std::fabs(denom) < 1e-9f) return false;   // axis points at the camera
    s = (b * e - c * d) / denom;
    return true;
}

bool intersectPlane(const Ray& ray, Vec3 point, Vec3 normal, Vec3& hit) {
    const float denom = dot(normal, ray.dir);
    if (std::fabs(denom) < 1e-7f) return false;
    const float t = dot(point - ray.origin, normal) / denom;
    if (t <= 0.0f) return false;
    hit = ray.at(t);
    return true;
}

float snapTo(float v, float step) {
    return step > 0.0f ? std::round(v / step) * step : v;
}

// Snap increment sized so one step is about the same distance on screen at any
// zoom, then rounded to a value a person would pick. Without the zoom term a
// fixed 1mm step is uselessly fine when zoomed out to a whole plate and far too
// coarse when zoomed in on a 0.4mm wall.
constexpr float kSnapPixels = 42.0f;

float snapStepFor(const Camera& camera, Vec3 at) {
    return niceStep(camera.pixelWorldSize(at) * kSnapPixels);
}

} // namespace

const char* transformModeName(TransformMode m) {
    switch (m) {
        case TransformMode::Translate: return "Move";
        case TransformMode::Rotate:    return "Rotate";
        case TransformMode::Scale:     return "Scale";
        case TransformMode::None:      return "";
    }
    return "";
}

// ---------------------------------------------------------------------------
Vec3 TransformTool::constraintAxis() const {
    switch (constraint_) {
        case Constraint::AxisX: case Constraint::PlaneX: return {1, 0, 0};
        case Constraint::AxisY: case Constraint::PlaneY: return {0, 1, 0};
        case Constraint::AxisZ: case Constraint::PlaneZ: return {0, 0, 1};
        case Constraint::Custom: return customAxis_;
        case Constraint::None:  return {};
    }
    return {};
}

void TransformTool::setCustomAxis(Vec3 axis, const char* label) {
    if (lengthSq(axis) < 1e-12f) return;
    customAxis_ = normalize(axis);
    customLabel_ = label ? label : "";
    constraint_ = Constraint::Custom;
}

bool TransformTool::isPlane() const {
    return constraint_ == Constraint::PlaneX ||
           constraint_ == Constraint::PlaneY ||
           constraint_ == Constraint::PlaneZ;
}

// ---------------------------------------------------------------------------
bool TransformTool::begin(TransformMode mode, Scene& scene, const Camera& camera,
                          Vec2 mousePx) {
    entries_.clear();
    vertexEntries_.clear();
    elementObject_ = kNoObject;
    customLabel_.clear();
    Vec3 pivot{};

    if (!scene.elementSelection().empty()) {
        // Element mode: gather the union of vertices behind every picked
        // element. A face contributes its whole loop, an edge its two ends.
        target_ = TransformTarget::Elements;
        elementObject_ = scene.elementSelection().front().object;
        const SceneObject* obj = scene.find(elementObject_);
        if (!obj) return false;

        std::vector<bool> seen(static_cast<size_t>(obj->mesh.vertexCount()), false);
        auto take = [&](Index v) {
            if (v < 0 || v >= obj->mesh.vertexCount() || seen[v]) return;
            seen[v] = true;
            vertexEntries_.push_back({v, obj->mesh.verts[v].position});
        };

        for (const ElementRef& e : scene.elementSelection()) {
            // Elements on other objects are ignored: one gesture edits one mesh.
            if (e.object != elementObject_) continue;
            switch (e.kind) {
            case ElementKind::Vertex: take(e.index); break;
            case ElementKind::Edge:
                if (e.index < obj->mesh.halfedgeCount()) {
                    take(obj->mesh.fromVertex(e.index));
                    take(obj->mesh.halfedges[e.index].vertex);
                }
                break;
            case ElementKind::Face:
                if (e.index < obj->mesh.faceCount()) {
                    std::vector<Index> verts;
                    obj->mesh.faceVertices(e.index, verts);
                    for (Index v : verts) take(v);
                }
                break;
            case ElementKind::None: break;
            }
        }
        if (vertexEntries_.empty()) return false;

        // Pivot is the centroid of the moving vertices, in world space.
        Vec3 acc{};
        for (const VertexEntry& v : vertexEntries_) acc += v.before;
        pivot = transformPoint(obj->modelMatrix(),
                               acc / static_cast<float>(vertexEntries_.size()));
    } else {
        if (scene.selection().empty()) return false;
        target_ = TransformTarget::Objects;
        for (ObjectId id : scene.selection())
            if (const SceneObject* o = scene.find(id)) entries_.push_back({id, o->transform});
        if (entries_.empty()) return false;
        pivot = scene.selectionCenter();
    }

    mode_ = mode;
    constraint_ = Constraint::None;
    typed_.clear();
    amount_ = 0.0f;
    delta_ = Vec3{};

    pivot_ = pivot;
    startMouse_ = mousePx;
    if (!camera.projectToPixel(pivot_, pivotPx_)) pivotPx_ = mousePx;

    rotateAccum_ = 0.0f;
    rotateLast_ = std::atan2(mousePx.y - pivotPx_.y, mousePx.x - pivotPx_.x);
    return true;
}

void TransformTool::setConstraint(Constraint c) {
    constraint_ = (constraint_ == c) ? Constraint::None : c;
}

void TransformTool::typeCharacter(char c) {
    if (c == '-') {
        // Leading minus toggles sign, as in Blender.
        if (!typed_.empty() && typed_[0] == '-') typed_.erase(typed_.begin());
        else typed_.insert(typed_.begin(), '-');
        return;
    }
    if (c == '.' && typed_.find('.') != std::string::npos) return;
    if (typed_.size() < 16) typed_.push_back(c);
}

void TransformTool::backspace() {
    if (!typed_.empty()) typed_.pop_back();
}

// ---------------------------------------------------------------------------
void TransformTool::update(Scene& scene, const Camera& camera, Vec2 mousePx, bool snap) {
    if (!active()) return;
    apply(scene, camera, mousePx, snap);
}

void TransformTool::apply(Scene& scene, const Camera& camera, Vec2 mousePx, bool snap) {
    const bool typedValue = !typed_.empty();
    const float typedNum = typedValue ? std::strtof(typed_.c_str(), nullptr) : 0.0f;

    const Ray rayNow   = camera.rayThroughPixel(mousePx.x, mousePx.y);
    const Ray rayStart = camera.rayThroughPixel(startMouse_.x, startMouse_.y);

    const float step = snapStepFor(camera, pivot_);
    snapStep_ = snap ? step : 0.0f;

    // The gesture is resolved once, in world space, and then applied to either
    // object transforms or mesh vertices. Keeping that split at the end means
    // the two targets can never drift apart in how they interpret a drag.
    Vec3 delta{};
    Quat rot{};
    Vec3 scl{1.0f, 1.0f, 1.0f};

    switch (mode_) {
    // ---------------------------------------------------------------- move --
    case TransformMode::Translate: {
        const bool axisLocked = constraint_ != Constraint::None && !isPlane();
        if (typedValue) {
            // An exact distance needs an axis to travel along; without one
            // there is no defined direction, so the entry simply waits.
            if (axisLocked) delta = constraintAxis() * typedNum;
        } else if (axisLocked) {
            const Vec3 axis = constraintAxis();
            float s0 = 0.0f, s1 = 0.0f;
            if (closestOnLine(rayStart, pivot_, axis, s0) &&
                closestOnLine(rayNow,   pivot_, axis, s1)) {
                float travel = s1 - s0;
                if (snap) travel = snapTo(travel, step);
                delta = axis * travel;
            }
        } else {
            // Free move, or move within a plane: drag along a plane through the
            // pivot, so the geometry stays under the cursor.
            const Vec3 normal = isPlane() ? constraintAxis() : -camera.forward();
            Vec3 p0, p1;
            if (intersectPlane(rayStart, pivot_, normal, p0) &&
                intersectPlane(rayNow,   pivot_, normal, p1)) {
                delta = p1 - p0;
                if (snap) delta = {snapTo(delta.x, step), snapTo(delta.y, step),
                                   snapTo(delta.z, step)};
            }
        }
        delta_ = delta;
        amount_ = length(delta);
        break;
    }

    // -------------------------------------------------------------- rotate --
    case TransformMode::Rotate: {
        // Unconstrained rotation turns about the view direction, which is what
        // makes a free spin feel like turning the object on screen.
        const Vec3 axis = constraint_ == Constraint::None ? -camera.forward()
                                                          : constraintAxis();
        float angle;
        if (typedValue) {
            angle = radians(typedNum);
        } else {
            const float now = std::atan2(mousePx.y - pivotPx_.y, mousePx.x - pivotPx_.x);
            float delta2 = now - rotateLast_;
            // Unwrap so a sweep through the +/-pi seam keeps accumulating.
            while (delta2 >  kPi) delta2 -= kTwoPi;
            while (delta2 < -kPi) delta2 += kTwoPi;
            rotateLast_ = now;
            rotateAccum_ += delta2;
            angle = -rotateAccum_;   // screen Y grows downward
            if (snap) angle = radians(snapTo(degrees(angle), 5.0f));
        }
        amount_ = degrees(angle);
        rot = Quat::fromAxisAngle(axis, angle);
        break;
    }

    // --------------------------------------------------------------- scale --
    case TransformMode::Scale: {
        float factor;
        if (typedValue) {
            factor = typedNum;
        } else {
            const float d0 = length(startMouse_ - pivotPx_);
            const float d1 = length(mousePx - pivotPx_);
            factor = d0 > 1e-3f ? d1 / d0 : 1.0f;
            if (snap) factor = snapTo(factor, 0.1f);
        }
        // Per-axis factors: uniform, one axis, or the two axes of a plane.
        switch (constraint_) {
            case Constraint::AxisX: scl = {factor, 1, 1}; break;
            case Constraint::AxisY: scl = {1, factor, 1}; break;
            case Constraint::AxisZ: scl = {1, 1, factor}; break;
            case Constraint::PlaneX: scl = {1, factor, factor}; break;
            case Constraint::PlaneY: scl = {factor, 1, factor}; break;
            case Constraint::PlaneZ: scl = {factor, factor, 1}; break;
            case Constraint::Custom:
            case Constraint::None: scl = {factor, factor, factor}; break;
        }
        amount_ = factor;
        break;
    }

    case TransformMode::None:
        return;
    }

    // How the gesture moves an arbitrary world point.
    auto mapWorld = [&](Vec3 p) -> Vec3 {
        switch (mode_) {
            case TransformMode::Translate: return p + delta;
            case TransformMode::Rotate:    return pivot_ + rotate(rot, p - pivot_);
            case TransformMode::Scale:     return pivot_ + (p - pivot_) * scl;
            case TransformMode::None:      return p;
        }
        return p;
    };

    if (target_ == TransformTarget::Elements) {
        SceneObject* obj = scene.find(elementObject_);
        if (!obj) return;

        // Vertices live in object space, so each one round-trips through the
        // model matrix. Doing the gesture in world space keeps it correct under
        // rotation and non-uniform scale, which a naive local-space delta would
        // not be.
        const Mat4 model = obj->modelMatrix();
        const Mat4 inv = inverse(model);
        for (const VertexEntry& ve : vertexEntries_) {
            if (ve.vertex >= obj->mesh.vertexCount()) continue;
            const Vec3 world = transformPoint(model, ve.before);
            obj->mesh.verts[ve.vertex].position = transformPoint(inv, mapWorld(world));
        }
        obj->refreshDerived();
        return;
    }

    for (const Entry& e : entries_) {
        SceneObject* o = scene.find(e.id);
        if (!o) continue;
        switch (mode_) {
        case TransformMode::Translate:
            o->transform.position = e.before.position + delta;
            break;
        case TransformMode::Rotate:
            o->transform.rotation = normalize(rot * e.before.rotation);
            o->transform.position = pivot_ + rotate(rot, e.before.position - pivot_);
            break;
        case TransformMode::Scale: {
            Vec3 next = e.before.scale * scl;
            // A scale of exactly zero makes the model matrix singular, which
            // silently breaks picking and normals. Keep a hair of thickness.
            constexpr float kMinScale = 1e-4f;
            for (int i = 0; i < 3; ++i)
                if (std::fabs(next[i]) < kMinScale)
                    next[i] = next[i] < 0.0f ? -kMinScale : kMinScale;
            o->transform.scale = next;
            o->transform.position = pivot_ + (e.before.position - pivot_) * scl;
            break;
        }
        case TransformMode::None:
            break;
        }
    }
}

// ---------------------------------------------------------------------------
std::unique_ptr<Command> TransformTool::confirm(Scene& scene) {
    if (!active()) return nullptr;

    if (target_ == TransformTarget::Elements) {
        const std::string what = transformModeName(mode_);
        const ObjectId id = elementObject_;
        std::vector<VertexEntry> moved = std::move(vertexEntries_);

        mode_ = TransformMode::None;
        vertexEntries_.clear();
        entries_.clear();
        typed_.clear();

        const SceneObject* obj = scene.find(id);
        if (!obj) return nullptr;

        std::vector<Index> verts;
        std::vector<Vec3> before, after;
        for (const VertexEntry& ve : moved) {
            if (ve.vertex >= obj->mesh.vertexCount()) continue;
            const Vec3 now = obj->mesh.verts[ve.vertex].position;
            if (now == ve.before) continue;
            verts.push_back(ve.vertex);
            before.push_back(ve.before);
            after.push_back(now);
        }
        if (verts.empty()) return nullptr;   // a click that did not move anything
        return std::make_unique<VertexCommand>(id, std::move(verts), std::move(before),
                                               std::move(after), what);
    }

    std::vector<TransformCommand::Entry> changed;
    for (const Entry& e : entries_) {
        const SceneObject* o = scene.find(e.id);
        if (!o) continue;
        const Transform& a = e.before;
        const Transform& b = o->transform;
        const bool moved = a.position != b.position || a.scale != b.scale ||
                           a.rotation.x != b.rotation.x || a.rotation.y != b.rotation.y ||
                           a.rotation.z != b.rotation.z || a.rotation.w != b.rotation.w;
        if (moved) changed.push_back({e.id, a, b});
    }

    const std::string what = transformModeName(mode_);
    mode_ = TransformMode::None;
    entries_.clear();
    typed_.clear();

    if (changed.empty()) return nullptr;   // a click that did not move anything
    return std::make_unique<TransformCommand>(std::move(changed), what);
}

void TransformTool::cancel(Scene& scene) {
    if (target_ == TransformTarget::Elements) {
        if (SceneObject* obj = scene.find(elementObject_)) {
            for (const VertexEntry& ve : vertexEntries_)
                if (ve.vertex < obj->mesh.vertexCount())
                    obj->mesh.verts[ve.vertex].position = ve.before;
            obj->refreshDerived();
        }
    } else {
        for (const Entry& e : entries_)
            if (SceneObject* o = scene.find(e.id)) o->transform = e.before;
    }
    mode_ = TransformMode::None;
    entries_.clear();
    vertexEntries_.clear();
    typed_.clear();
}

// ---------------------------------------------------------------------------
std::string TransformTool::statusText() const {
    if (!active()) return {};

    std::string customName = customLabel_.empty() ? "" : (" " + customLabel_);
    const char* axisName = "";
    switch (constraint_) {
        case Constraint::AxisX: axisName = " X"; break;
        case Constraint::AxisY: axisName = " Y"; break;
        case Constraint::AxisZ: axisName = " Z"; break;
        case Constraint::PlaneX: axisName = " YZ"; break;
        case Constraint::PlaneY: axisName = " XZ"; break;
        case Constraint::PlaneZ: axisName = " XY"; break;
        case Constraint::Custom: axisName = customName.c_str(); break;
        case Constraint::None: break;
    }

    char buf[192];
    if (!typed_.empty()) {
        const bool needsAxis = mode_ == TransformMode::Translate &&
                               (constraint_ == Constraint::None || isPlane());
        // A caret makes it obvious the value is being typed rather than
        // dragged, which is otherwise ambiguous from the number alone.
        std::snprintf(buf, sizeof(buf), "%s%s  %s|%s%s",
                      transformModeName(mode_), axisName, typed_.c_str(),
                      mode_ == TransformMode::Rotate ? " deg" :
                      mode_ == TransformMode::Translate ? " mm" : "",
                      needsAxis ? "   (press X, Y or Z)" : "");
        return buf;
    }

    switch (mode_) {
        case TransformMode::Translate: {
            char snapNote[48] = "";
            if (snapStep_ > 0.0f)
                std::snprintf(snapNote, sizeof(snapNote), "   snap %g mm",
                              static_cast<double>(snapStep_));
            if (constraint_ != Constraint::None && !isPlane())
                std::snprintf(buf, sizeof(buf), "Move%s  %.2f mm%s", axisName,
                              dot(delta_, constraintAxis()), snapNote);
            else
                std::snprintf(buf, sizeof(buf), "Move%s  %.2f, %.2f, %.2f mm%s",
                              axisName, delta_.x, delta_.y, delta_.z, snapNote);
            break;
        }
        case TransformMode::Rotate:
            std::snprintf(buf, sizeof(buf), "Rotate%s  %.1f deg", axisName, amount_);
            break;
        case TransformMode::Scale:
            std::snprintf(buf, sizeof(buf), "Scale%s  %.3f", axisName, amount_);
            break;
        case TransformMode::None:
            return {};
    }
    return buf;
}

void TransformTool::drawOverlay(Renderer& renderer, const Camera& camera) const {
    if (!active()) return;

    // Extend the constraint line far enough to read as infinite at any zoom.
    const float reach = std::max(camera.distance * 6.0f, 200.0f);

    auto axisColour = [](int i) -> Vec4 {
        switch (i) {
            case 0: return toVec4(palette::kGridAxisX, 0.95f);
            case 1: return toVec4(palette::kGridAxisY, 0.95f);
            default: return Vec4{0.35f, 0.55f, 0.95f, 0.95f};
        }
    };

    if (constraint_ != Constraint::None && !isPlane()) {
        const Vec3 a = constraintAxis();
        const int idx = a.x != 0.0f ? 0 : (a.y != 0.0f ? 1 : 2);
        renderer.addLine(pivot_ - a * reach, pivot_ + a * reach, axisColour(idx));
    } else if (isPlane()) {
        // Show the two axes that remain free rather than the one that is locked.
        const Vec3 n = constraintAxis();
        for (int i = 0; i < 3; ++i) {
            Vec3 a{};
            a[i] = 1.0f;
            if (dot(a, n) != 0.0f) continue;
            renderer.addLine(pivot_ - a * reach, pivot_ + a * reach, axisColour(i));
        }
    }

    // A small cross marks the pivot the transform is measured from.
    const float s = camera.pixelWorldSize(pivot_) * 7.0f;
    const Vec4 c = toVec4(palette::kBrand, 1.0f);
    renderer.addLine(pivot_ - Vec3{s, 0, 0}, pivot_ + Vec3{s, 0, 0}, c);
    renderer.addLine(pivot_ - Vec3{0, s, 0}, pivot_ + Vec3{0, s, 0}, c);
    renderer.addLine(pivot_ - Vec3{0, 0, s}, pivot_ + Vec3{0, 0, s}, c);
}

} // namespace tg
