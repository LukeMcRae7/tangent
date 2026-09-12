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

    // Orthographic is the working state and perspective is the exception.
    //
    // A part is designed against dimensions, and in perspective no two edges
    // of the same length are the same length on screen. Perspective is for
    // looking at the thing, so it is what an orbit gives you -- and `preferOrtho`
    // remembers that this is a detour, so snapping back to a named view comes
    // home to square again. The toggle is what changes the preference itself.
    bool  orthographic = true;
    bool  preferOrtho  = true;

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

    // Look at the model from `dir`, which points from the target out to where
    // the eye should go. The view cube has twenty-six of these and only six of
    // them have names.
    //
    // Animates, like the standard views do: a view that jumps leaves the user
    // to work out for themselves what happened to their model.
    void setViewDirection(Vec3 dir);

    // The whole framing at once, over the same short animation, with the
    // projection left alone. For a tool that takes the view somewhere on
    // purpose and has to give it back afterwards.
    void animateTo(Vec3 at, float dist, float yawRad, float pitchRad);

    // The turntable angles that look from `dir`. Pitch stops just short of the
    // pole, where the basis gives out.
    static void anglesFor(Vec3 dir, float& yawRad, float& pitchRad);

    // The projection the user asked for, after an orbit has taken it away.
    void restorePreferredProjection() { orthographic = preferOrtho; }

    // Asking for a projection is also saying which one you want to come back
    // to. Toggling is asking for the other of what is on screen -- not the
    // other of the preference, which after an orbit is not the same thing and
    // would leave the menu item doing nothing.
    void setOrthographic(bool on) { preferOrtho = on; orthographic = on; }
    void toggleProjection() { setOrthographic(!orthographic); }
    void frame(const AABB& box);             // fit a box, leaving a margin

    // Eye ray through a pixel, for picking. Origin is on the near plane.
    Ray rayThroughPixel(float px, float py) const;

    // Project a world point to pixels; returns false if behind the camera.
    bool projectToPixel(Vec3 world, Vec2& outPixel) const;

    // World-space size of one pixel at a given point, used to keep gizmos and
    // hit-test tolerances constant on screen regardless of zoom.
    float pixelWorldSize(Vec3 atPoint) const;

    // The snap increment to use for a gesture happening near `atPoint`.
    //
    // Sized so one step is about the same distance on screen at any zoom, then
    // rounded to a value a person would pick: a fixed 1mm step is uselessly
    // fine when zoomed out to a whole plate and far too coarse when zoomed in
    // on a 0.4mm wall. Lives here so every tool snaps the same way -- the
    // create tool used to carry its own hardcoded 5mm, 1mm and 0.5mm steps.
    float snapStep(Vec3 atPoint) const {
        // Ten pixels a step, not forty.
        //
        // The step is the smallest change the user can ask for, so it decides
        // how much control they have: at forty pixels a full-width drag offers
        // about thirty distinct values and the hand is never the limit -- the
        // step is. At ten, a hand that can hold itself within a few millimetres
        // of screen can reach every one of them, which is the point.
        //
        // Still rounded to a value a person would choose, so the numbers stay
        // 0.5 and 2 rather than 0.34 and 1.87.
        return static_cast<float>(niceStep(pixelWorldSize(atPoint) * 10.0f));
    }

    // Placing things is not sizing them: a point lands on a line the user can
    // see, which is the viewport's own grid rather than a step of the camera's
    // choosing. See gridLevelsAt in app/plane_snap.h, which the sketch grid is
    // drawn from and the snapper pulls to, so the two cannot disagree.

    // ---- Smoothing -------------------------------------------------------
    // Only view *snaps* animate. Orbit, pan and dolly are applied immediately:
    // easing direct manipulation puts the view behind the cursor by the
    // smoothing time constant, which reads as floaty and imprecise. Animation
    // is for jumps the user did not drag out by hand.
    void snapToGoal();
    void update(float dt);
    bool animating() const { return animating_; }

private:
    struct Goal { Vec3 target; float distance; float yaw, pitch; };
    Goal goal_{target, distance, yaw, pitch};
    bool hasGoal_ = false;
    bool animating_ = false;

    void syncGoal() { goal_ = {target, distance, yaw, pitch}; hasGoal_ = true; }
};

} // namespace tg
