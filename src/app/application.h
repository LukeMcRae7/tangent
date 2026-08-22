// Tangent - application shell: window, event loop, input and tool dispatch.
#pragma once

#include "app/camera.h"
#include "render/renderer.h"
#include "scene/scene.h"
#include "ui/panels.h"

#include <string>

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

    Scene       scene_;
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
};

} // namespace tg
