// Tangent - application shell: window, event loop, input and tool dispatch.
#pragma once

#include "app/camera.h"
#include "app/transform_tool.h"
#include "app/undo.h"
#include "render/renderer.h"
#include "scene/scene.h"
#include "ui/panels.h"

#include <string>
#include <vector>

struct SDL_Window;
union SDL_Event;

namespace tg {

class Application {
public:
    bool init();
    int  run();
    void shutdown();

    // Renders `frames` and exits; used to smoke-test startup non-interactively.
    void setSmokeTest(int frames) { smokeFrames_ = frames; }

    // Places the camera explicitly. Makes captures reproducible, which is what
    // lets grid behaviour be compared across viewing angles.
    void setCamera(float yawDeg, float pitchDeg, float dist) {
        camera_.yaw = radians(yawDeg);
        camera_.pitch = radians(pitchDeg);
        if (dist > 0.0f) camera_.distance = dist;
        camera_.snapToGoal();
        fixedCamera_ = true;
    }

    // Starts with no objects, so a measurement sees only the grid.
    void setStartEmpty() { startEmpty_ = true; }
    void setNoGrid() { view_.showGrid = false; }

    // Sweeps the camera through a yaw range, printing the mean luminance of
    // the viewport at each step. A grid that is stable under rotation produces
    // a smooth curve; popping or breathing lines show up as high-frequency
    // steps, which tests/grid_stability.py checks for numerically.
    void setGridProbe(float yaw0Deg, float yaw1Deg, int steps) {
        probeYaw0_ = yaw0Deg;
        probeYaw1_ = yaw1Deg;
        probeSteps_ = steps > 1 ? steps : 2;
        probeActive_ = true;
    }

    // Writes the viewport to a PPM after `afterFrames` frames. Reads back this
    // process's own GL framebuffer rather than going through the compositor, so
    // it captures only Tangent and works regardless of what else is on screen.
    void setScreenshot(const std::string& path, int afterFrames) {
        screenshotPath_ = path;
        screenshotFrame_ = afterFrames;
    }

private:
    void handleEvent(const SDL_Event& e);
    void handleViewportMouse();
    void handleShortcuts();
    void handleTransformKeys();
    Vec2 mouseInViewport() const;
    void beginTransform(TransformMode mode);
    void applyActions();
    void buildUi();
    void drawFrame();

    // Where the 3D view lives inside the window, in ImGui's logical points.
    struct ViewRect { float x = 0, y = 0, w = 0, h = 0; };

    void addPrimitiveAtCursor(PrimitiveKind kind);

    // Drops an object so its lowest point rests on z = 0. For print design the
    // ground plane is the build plate, so parts belong on it rather than
    // centred through it -- which also stops the grid drawing across them.
    void placeOnBuildPlate(ObjectId id);

    SDL_Window* window_  = nullptr;
    void*       glCtx_   = nullptr;
    std::string shaderDir_;

    Scene         scene_;
    UndoStack     undo_;
    TransformTool tool_;
    Camera      camera_;
    Renderer    renderer_;
    ViewOptions view_;
    UiContext   ui_;

    ViewRect viewRect_;
    float    pixelScaleX_ = 1.0f, pixelScaleY_ = 1.0f;

    bool  running_     = true;
    bool  firstLayout_ = true;
    int   smokeFrames_ = 0;
    float frameMs_     = 0.0f;

    // Set for one frame when Shift+A asks for the add menu at the cursor.
    bool  openAddMenu_ = false;

    void captureFramebuffer(int width, int height) const;
    std::string screenshotPath_;
    int         screenshotFrame_ = -1;
    bool        fixedCamera_ = false;
    bool        startEmpty_ = false;

    double meanViewportLuminance() const;
    void   readViewport(std::vector<unsigned char>& out) const;
    std::vector<unsigned char> probePrev_;
    PixelRect lastViewportPx_;
    bool  probeActive_ = false;
    float probeYaw0_ = 0.0f, probeYaw1_ = 90.0f;
    int   probeSteps_ = 2, probeIndex_ = 0, probeSettle_ = 0;
};

} // namespace tg
