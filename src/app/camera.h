// Tangent - orbit ("turntable") camera, Z-up.
//
// Navigation follows Blender, which the brief names as the reference:
//   MMB drag          orbit (direction configurable, see invertOrbitX/Y)
//   Shift + MMB drag  pan
//   Wheel             dolly
//   Numpad 1/3/7      front / right / top   (Ctrl for the opposite side)
//   Numpad 5          toggle perspective / orthographic
//   Numpad .          frame selection        Home  frame everything
#pragma once

#include "core/math.h"

namespace tg {

enum class StandardView { Front, Back, Left, Right, Top, Bottom };

class Camera {
public:
    // ---- State -----------------------------------------------------------
    Vec3  target{0.0f, 0.0f, 0.0f};
    float distance = 90.0f;      // mm from target
    float yaw   = radians(-35.0f);
    float pitch = radians( 28.0f);
    bool  orthographic = false;

    // Orbit direction. The default drags the scene with the cursor: moving the
    // mouse left swings the view to the right. Flip either axis to taste.
    bool  invertOrbitX = false;
    bool  invertOrbitY = false;

    float fovY   = radians(45.0f);
    float zNear  = 0.05f;
    float zFar   = 20000.0f;
    int   viewportW = 1600, viewportH = 900;

    // ---- Derived ---------------------------------------------------------
    Vec3 eye() const;
    Vec3 forward() const;
    Vec3 right() const;
    Vec3 up() const;

    Mat4 view() const;
    Mat4 projection() const;
    Mat4 viewProjection() const { return projection() * view(); }
    float aspect() const {
        return viewportH > 0 ? static_cast<float>(viewportW) / static_cast<float>(viewportH) : 1.0f;
    }
    // Vertical world-space extent covered at the target plane; ties the
    // orthographic framing to the perspective one so toggling does not jump.
    float orthoHeight() const { return 2.0f * distance * std::tan(fovY * 0.5f); }

    // ---- Navigation ------------------------------------------------------
    void orbit(float dxPixels, float dyPixels);
    void pan(float dxPixels, float dyPixels);
    void dolly(float steps);                 // wheel notches; positive = closer
    void setStandardView(StandardView v);
    void frame(const AABB& box);             // fit a box, leaving a margin

    // Eye ray through a pixel, for picking. Origin is on the near plane.
    Ray rayThroughPixel(float px, float py) const;

    // Project a world point to pixels; returns false if behind the camera.
    bool projectToPixel(Vec3 world, Vec2& outPixel) const;

    // World-space size of one pixel at a given point, used to keep gizmos and
    // hit-test tolerances constant on screen regardless of zoom.
    float pixelWorldSize(Vec3 atPoint) const;

    // ---- Smoothing -------------------------------------------------------
    // Navigation writes the goal; update() eases the live values toward it.
    void snapToGoal();
    void update(float dt);

private:
    struct Goal { Vec3 target; float distance; float yaw, pitch; };
    Goal goal_{target, distance, yaw, pitch};
    bool hasGoal_ = false;
};

} // namespace tg
