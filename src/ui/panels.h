// Tangent - interface panels.
//
// Panels never mutate the application directly; they record intent in
// UiActions and the application applies it. That keeps undo (and later, the
// feature history) able to see every edit in one place.
#pragma once

#include "render/renderer.h"
#include "scene/scene.h"
#include "app/camera.h"

namespace tg {

struct UiActions {
    bool          addRequested = false;
    PrimitiveKind addKind = PrimitiveKind::Box;

    bool deleteSelected    = false;
    bool duplicateSelected = false;
    bool frameSelected     = false;
    bool frameAll          = false;
    bool resetView         = false;
    bool quit              = false;

    // Set when an inspector field changed the object's parameters and the
    // geometry has to be regenerated.
    ObjectId rebuildObject = kNoObject;
};

struct UiStats {
    float  frameMs   = 0.0f;
    size_t triangles = 0;
    size_t vertices  = 0;
};

struct UiContext {
    Scene*       scene  = nullptr;
    Camera*      camera = nullptr;
    ViewOptions* view   = nullptr;
    UiStats      stats;
    UiActions    actions;
};

void drawMenuBar(UiContext& ctx);
void drawOutliner(UiContext& ctx);
void drawInspector(UiContext& ctx);
void drawStatusBar(UiContext& ctx);

// Body of the add-object menu, shared by the menu bar and the Shift+A popup.
void drawAddMenuItems(UiContext& ctx);

// Compact viewport corner readout: view name, projection, navigation hints.
void drawViewportOverlay(UiContext& ctx, float x, float y, float w, float h);

} // namespace tg
