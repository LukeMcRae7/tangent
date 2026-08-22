#include "app/camera.h"

namespace tg {
namespace {
// Just short of the pole: the basis below stays valid there, but clamping
// keeps the turntable from tumbling past vertical the way a trackball would.
constexpr float kPitchLimit = 1.5706f;   // ~89.99 degrees
constexpr float kOrbitRadiansPerPixel = 0.010f;
constexpr float kMinDistance = 0.02f;
constexpr float kMaxDistance = 12000.0f;
}

// Direction from the target out to the eye.
static Vec3 radialDir(float yaw, float pitch) {
    const float cp = std::cos(pitch);
    return {cp * std::sin(yaw), -cp * std::cos(yaw), std::sin(pitch)};
}

Vec3 Camera::eye() const { return target + radialDir(yaw, pitch) * distance; }
Vec3 Camera::forward() const { return -radialDir(yaw, pitch); }

// Horizontal and always unit-length, so it never degenerates at the poles --
// which is why the view basis is built by hand instead of via lookAt(up=+Z).
Vec3 Camera::right() const { return {std::cos(yaw), std::sin(yaw), 0.0f}; }
Vec3 Camera::up() const { return cross(right(), forward()); }

Mat4 Camera::view() const {
    const Vec3 e = eye(), f = forward(), r = right(), u = up();
    Mat4 m;
    m.col[0] = {r.x, u.x, -f.x, 0.0f};
    m.col[1] = {r.y, u.y, -f.y, 0.0f};
    m.col[2] = {r.z, u.z, -f.z, 0.0f};
    m.col[3] = {-dot(r, e), -dot(u, e), dot(f, e), 1.0f};
    return m;
}

Mat4 Camera::projection() const {
    if (!orthographic) return perspective(fovY, aspect(), zNear, zFar);
    const float h = orthoHeight() * 0.5f, w = h * aspect();
    // Symmetric depth range keeps geometry behind the target visible in ortho.
    return orthoProjection(-w, w, -h, h, -kMaxDistance, kMaxDistance);
}

void Camera::orbit(float dxPixels, float dyPixels) {
    const float dx = dxPixels * (invertOrbitX ? -1.0f : 1.0f);
    const float dy = dyPixels * (invertOrbitY ? -1.0f : 1.0f);

    // Applied straight to the live camera: the view must sit exactly where the
    // drag has put it, this frame.
    yaw   -= dx * kOrbitRadiansPerPixel;
    pitch = clampf(pitch + dy * kOrbitRadiansPerPixel, -kPitchLimit, kPitchLimit);
    animating_ = false;
    syncGoal();
}

void Camera::pan(float dxPixels, float dyPixels) {
    // Match cursor movement one-to-one at the target plane.
    const float worldPerPixel = orthoHeight() / static_cast<float>(viewportH > 0 ? viewportH : 1);
    target -= right() * (dxPixels * worldPerPixel);
    target += up()    * (dyPixels * worldPerPixel);
    animating_ = false;
    syncGoal();
}

void Camera::dolly(float steps) {
    // Multiplicative, so each notch feels the same at any zoom level.
    distance = clampf(distance * std::pow(0.85f, steps), kMinDistance, kMaxDistance);
    animating_ = false;
    syncGoal();
}

void Camera::setStandardView(StandardView v) {
    syncGoal();
    animating_ = true;
    switch (v) {
        case StandardView::Front:  goal_.yaw = 0.0f;          goal_.pitch = 0.0f; break;
        case StandardView::Back:   goal_.yaw = kPi;           goal_.pitch = 0.0f; break;
        case StandardView::Right:  goal_.yaw = kHalfPi;       goal_.pitch = 0.0f; break;
        case StandardView::Left:   goal_.yaw = -kHalfPi;      goal_.pitch = 0.0f; break;
        case StandardView::Top:    goal_.pitch =  kPitchLimit; break;
        case StandardView::Bottom: goal_.pitch = -kPitchLimit; break;
    }
}

void Camera::frame(const AABB& box) {
    if (!box.valid()) return;
    syncGoal();
    animating_ = true;

    goal_.target = box.center();
    const float radius = std::max(box.radius(), Real(0.5));
    // Fit by the tighter of the two field-of-view axes, then leave a margin.
    const float fovX = 2.0f * std::atan(std::tan(fovY * 0.5f) * aspect());
    const float fit  = radius / std::sin(std::max(std::min(fovY, fovX) * 0.5f, 0.01f));
    goal_.distance = clampf(fit * 1.55f, kMinDistance, kMaxDistance);
}

Ray Camera::rayThroughPixel(float px, float py) const {
    // Pixel centre -> NDC, with Y flipped (window Y grows downward).
    const float ndcX = (px / static_cast<float>(viewportW)) * 2.0f - 1.0f;
    const float ndcY = 1.0f - (py / static_cast<float>(viewportH)) * 2.0f;

    if (orthographic) {
        const float h = orthoHeight() * 0.5f, w = h * aspect();
        const Vec3 origin = eye() + right() * (ndcX * w) + up() * (ndcY * h);
        return {origin - forward() * kMaxDistance, forward()};
    }

    const float ty = std::tan(fovY * 0.5f);
    const Vec3 dir = normalize(forward() + right() * (ndcX * ty * aspect()) + up() * (ndcY * ty));
    return {eye(), dir};
}

bool Camera::projectToPixel(Vec3 world, Vec2& outPixel) const {
    const Vec4 clip = viewProjection() * Vec4(world, 1.0f);
    if (clip.w <= 1e-6f) return false;
    const Vec3 ndc = clip.xyz() / clip.w;
    outPixel = {(ndc.x * 0.5f + 0.5f) * static_cast<float>(viewportW),
                (1.0f - (ndc.y * 0.5f + 0.5f)) * static_cast<float>(viewportH)};
    return true;
}

float Camera::pixelWorldSize(Vec3 atPoint) const {
    const int h = viewportH > 0 ? viewportH : 1;
    if (orthographic) return orthoHeight() / static_cast<float>(h);
    const Real depth = std::max(dot(atPoint - eye(), forward()), Real(1e-4));
    return 2.0f * depth * std::tan(fovY * 0.5f) / static_cast<float>(h);
}

void Camera::snapToGoal() {
    if (!hasGoal_) return;
    target = goal_.target; distance = goal_.distance;
    yaw = goal_.yaw; pitch = goal_.pitch;
    animating_ = false;
}

void Camera::update(float dt) {
    if (!hasGoal_ || !animating_) return;

    // Close enough to be indistinguishable: land exactly and stop, so the
    // camera is not perpetually creeping by fractions of a pixel.
    if (std::fabs(yaw - goal_.yaw) < 1e-5f && std::fabs(pitch - goal_.pitch) < 1e-5f &&
        std::fabs(distance - goal_.distance) < 1e-4f &&
        lengthSq(target - goal_.target) < 1e-8f) {
        snapToGoal();
        return;
    }
    // Exponential ease that is frame-rate independent: the same fraction of
    // the remaining distance is covered per unit time, not per frame.
    const float k = 1.0f - std::exp(-22.0f * clampf(dt, 0.0f, 0.1f));
    target   = lerp(target, goal_.target, k);
    distance = lerpf(distance, goal_.distance, k);
    yaw      = lerpf(yaw, goal_.yaw, k);
    pitch    = lerpf(pitch, goal_.pitch, k);
}

} // namespace tg
