// Tangent - application shell: window, event loop, input and tool dispatch.
#pragma once

#include "app/camera.h"
#include "app/transform_tool.h"
#include "mesh/operations.h"
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

    // Checks that grid lines are drawn at the world coordinates they belong to.
    // Samples the rendered image at points known to lie exactly on a major grid
    // line, and at control points deliberately off every line, for each yaw in
    // a sweep. A grid anchored to the world keeps the on-line samples bright at
    // every angle; one whose phase drifts with the camera does not.
    void setGridAlign(float yaw0Deg, float yaw1Deg, int steps) {
        probeYaw0_ = yaw0Deg;
        probeYaw1_ = yaw1Deg;
        probeSteps_ = steps > 1 ? steps : 2;
        probeActive_ = true;
        alignProbe_ = true;
    }

    // Pre-selects a face of the first object. Diagnostic only: it makes the
    // selection highlight and the mesh operations reproducible in a capture,
    // which a click cannot be.
    void setPickFace(int index) { pickFace_ = index; }
    void setAutoExtrude(float mm) { autoExtrude_ = true; autoExtrudeMm_ = mm; }

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
    void handleViewportClick(bool shift, bool ctrl);
    void drawSelectionHighlights();
    void extrudeSelection();
    void bevelActiveObject();

    // Single exit points for a modal transform, so the extrude-drag's extra
    // bookkeeping cannot be forgotten at one of the several call sites.
    void commitTransform();
    void abortTransform();
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
    float lastDt_      = 0.0f;

    // Seconds the geometry has been unchanged. The printability check is far
    // too expensive to run mid-drag, so it waits for things to settle.
    float healthIdle_  = 0.0f;

    // Was the object a clean solid when the current gesture started? An edit
    // is only refused for breaking something that was not already broken.
    bool  preEditSolid_ = false;

    // Transient message shown in the status bar, e.g. a refused edit.
    std::string notice_;
    float       noticeAge_ = 0.0f;
    void setNotice(const std::string& text) { notice_ = text; noticeAge_ = 0.0f; }

    // Reverts a just-applied edit that would leave the model unprintable.
    bool editKeepsSolid(ObjectId id);

    // Set for one frame when Shift+A asks for the add menu at the cursor.
    bool  openAddMenu_ = false;

    // True while a middle-drag navigation gesture is in progress.
    bool  navigating_ = false;

    void captureFramebuffer(int width, int height) const;
    std::string screenshotPath_;
    int         screenshotFrame_ = -1;
    bool        fixedCamera_ = false;
    bool        startEmpty_ = false;
    // Set while a transform is finishing an operation that also changed
    // topology, so commit records one undo entry covering both.
    ObjectId             pendingMeshObject_ = kNoObject;
    Mesh                 pendingMeshBefore_;
    std::vector<Feature> pendingChainBefore_;
    std::vector<Index>   pendingExtrudeFaces_;
    std::vector<Index>   pendingNewFaces_;
    Vec3                 pendingLocalNormal_{0, 0, 1};
    std::string          pendingLabel_;

    int         pickFace_ = -1;
    bool        autoExtrude_ = false;
    float       autoExtrudeMm_ = 10.0f;

    double meanViewportLuminance() const;
    void   readViewport(std::vector<unsigned char>& out) const;
    std::vector<unsigned char> probePrev_;
    PixelRect lastViewportPx_;
    bool  probeActive_ = false;
    float probeYaw0_ = 0.0f, probeYaw1_ = 90.0f;
    int   probeSteps_ = 2, probeIndex_ = 0, probeSettle_ = 0;
    bool  alignProbe_ = false;
    void  sampleGridAlignment(float& onLine, float& offLine) const;
};

} // namespace tg
