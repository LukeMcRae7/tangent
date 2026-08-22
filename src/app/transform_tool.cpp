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
        case Constraint::None:  return {};
    }
    return {};
}

bool TransformTool::isPlane() const {
    return constraint_ == Constraint::PlaneX ||
           constraint_ == Constraint::PlaneY ||
           constraint_ == Constraint::PlaneZ;
}

// ---------------------------------------------------------------------------
bool TransformTool::begin(TransformMode mode, Scene& scene, const Camera& camera,
                          Vec2 mousePx) {
    if (scene.selection().empty()) return false;

    entries_.clear();
    for (ObjectId id : scene.selection())
        if (const SceneObject* o = scene.find(id)) entries_.push_back({id, o->transform});
    if (entries_.empty()) return false;

    mode_ = mode;
    constraint_ = Constraint::None;
    typed_.clear();
    amount_ = 0.0f;
    delta_ = Vec3{};

    pivot_ = scene.selectionCenter();
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

    switch (mode_) {
    // ---------------------------------------------------------------- move --
    case TransformMode::Translate: {
        Vec3 delta{};

        if (typedValue) {
            // An exact distance needs an axis to travel along; without one
            // there is no defined direction, so the entry simply waits.
            if (constraint_ != Constraint::None && !isPlane())
                delta = constraintAxis() * typedNum;
        } else if (constraint_ != Constraint::None && !isPlane()) {
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
        for (const Entry& e : entries_)
            if (SceneObject* o = scene.find(e.id))
                o->transform.position = e.before.position + delta;
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
            float step = now - rotateLast_;
            // Unwrap so a sweep through the +/-pi seam keeps accumulating.
            while (step >  kPi) step -= kTwoPi;
            while (step < -kPi) step += kTwoPi;
            rotateLast_ = now;
            rotateAccum_ += step;
            angle = -rotateAccum_;   // screen Y grows downward
            if (snap) angle = radians(snapTo(degrees(angle), 5.0f));
        }

        amount_ = degrees(angle);
        const Quat q = Quat::fromAxisAngle(axis, angle);
        for (const Entry& e : entries_) {
            SceneObject* o = scene.find(e.id);
            if (!o) continue;
            o->transform.rotation = normalize(q * e.before.rotation);
            o->transform.position = pivot_ + rotate(q, e.before.position - pivot_);
        }
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
        Vec3 s{factor, factor, factor};
        switch (constraint_) {
            case Constraint::AxisX: s = {factor, 1, 1}; break;
            case Constraint::AxisY: s = {1, factor, 1}; break;
            case Constraint::AxisZ: s = {1, 1, factor}; break;
            case Constraint::PlaneX: s = {1, factor, factor}; break;
            case Constraint::PlaneY: s = {factor, 1, factor}; break;
            case Constraint::PlaneZ: s = {factor, factor, 1}; break;
            case Constraint::None: break;
        }

        amount_ = factor;
        for (const Entry& e : entries_) {
            SceneObject* o = scene.find(e.id);
            if (!o) continue;
            Vec3 next = e.before.scale * s;
            // A scale of exactly zero makes the model matrix singular, which
            // silently breaks picking and normals. Keep a hair of thickness.
            constexpr float kMinScale = 1e-4f;
            for (int i = 0; i < 3; ++i)
                if (std::fabs(next[i]) < kMinScale)
                    next[i] = next[i] < 0.0f ? -kMinScale : kMinScale;
            o->transform.scale = next;
            o->transform.position = pivot_ + (e.before.position - pivot_) * s;
        }
        break;
    }

    case TransformMode::None:
        break;
    }
}

// ---------------------------------------------------------------------------
std::unique_ptr<Command> TransformTool::confirm(Scene& scene) {
    if (!active()) return nullptr;

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
    for (const Entry& e : entries_)
        if (SceneObject* o = scene.find(e.id)) o->transform = e.before;
    mode_ = TransformMode::None;
    entries_.clear();
    typed_.clear();
}

// ---------------------------------------------------------------------------
std::string TransformTool::statusText() const {
    if (!active()) return {};

    const char* axisName = "";
    switch (constraint_) {
        case Constraint::AxisX: axisName = " X"; break;
        case Constraint::AxisY: axisName = " Y"; break;
        case Constraint::AxisZ: axisName = " Z"; break;
        case Constraint::PlaneX: axisName = " YZ"; break;
        case Constraint::PlaneY: axisName = " XZ"; break;
        case Constraint::PlaneZ: axisName = " XY"; break;
        case Constraint::None: break;
    }

    char buf[192];
    if (!typed_.empty()) {
        const bool needsAxis = mode_ == TransformMode::Translate &&
                               (constraint_ == Constraint::None || isPlane());
        std::snprintf(buf, sizeof(buf), "%s%s  %s%s%s",
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
