// Tangent - interface panels.
//
// Panels never mutate the application directly; they record intent in
// UiActions and the application applies it. That keeps undo (and later, the
// feature history) able to see every edit in one place.
//
// The shape of the screen, top to bottom and left to right:
//
//   the top bar     the logo, the tools in four groups with a menu under
//                   each, and -- when the application draws its own frame --
//                   the window's own buttons at the right
//   the outliner    every body, sketch and mesh, with an eye to hide it
//   the viewport    the model, the view cube, and the operation's own panel
//                   floating at the bottom
//   the inspector   the selected thing: its name, transform, shape and history
#pragma once

#include "render/renderer.h"
#include "scene/scene.h"
#include "app/camera.h"
#include "app/measure.h"
#include "geom/op_types.h"

#include <string>
#include <vector>

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
    // geometry has to be regenerated. The previous spec travels with it so the
    // edit can be pushed onto the undo stack.
    ObjectId      rebuildObject = kNoObject;
    PrimitiveSpec specBefore;

    // Set when an inspector transform field was dragged. The application
    // turns what it came to into a Move, Rotate or Scale in the history.
    ObjectId  transformEdited = kNoObject;
    Transform transformBefore;

    // Back to the origin, unturned, at full size -- as steps, like any other
    // move.
    ObjectId  resetTransform = kNoObject;

    // A body's row in the outliner was clicked: what Ctrl+clicking the body
    // in the view does. Shift or Ctrl on the row adds or takes away.
    ObjectId  pickObject = kNoObject;
    bool      pickObjectAdditive = false;

    bool undo = false;
    bool redo = false;

    // File operations. The application owns the prompt and the current path.
    bool newProject = false;
    bool openProject = false;
    bool saveProject = false;
    bool saveProjectAs = false;
    bool exportStl = false;
    bool export3mf = false;
    bool exportStep = false;
    bool importStep = false;
    bool importMesh = false;
    bool importSvg = false;
    bool convertToSolid = false;
    bool pushPull = false;
    bool extrude = false;
    bool extrudeCut = false;       // Shift+E: an extrude with Cut already picked
    bool rotateFace = false;
    bool scaleFace = false;
    bool divide = false;
    bool mergeFaces = false;
    bool pattern = false;
    bool mirror = false;
    bool reduceMesh = false;
    bool inset = false;
    bool bevel = false;
    bool split = false;
    bool fillet = false;
    bool shell = false;
    bool hole = false;
    bool draft = false;
    bool deleteFace = false;

    // The modal transforms, from the bar or a menu.
    bool moveObject = false;
    bool rotateObject = false;
    bool scaleObject = false;

    // Measuring, toggled.
    bool toggleMeasure = false;

    // A new sketch, or one already in an object's history re-opened to edit.
    bool      sketch = false;
    ObjectId  editSketchObject = kNoObject;
    ElementId editSketchUid = 0;

    bool      booleanRequested = false;
    BooleanOp booleanOp = BooleanOp::Difference;

    // Set when the timeline changed a feature: the chain as it was, so the
    // edit can be re-evaluated and recorded.
    ObjectId             featuresEdited = kNoObject;
    std::vector<Feature> featuresBefore;
};

struct UiStats {
    float  frameMs   = 0.0f;
    size_t triangles = 0;
    size_t vertices  = 0;
};

// The window's own frame, when the application draws it. The bar along the
// top is the title bar: dragging it moves the window, double-clicking it
// maximises, and its right-hand end holds the three buttons every window has.
struct FrameState {
    bool  customFrame = false;
    bool  maximized = false;
    float barHeight = 0.0f;

    // Where the bar's own controls are this frame, in window coordinates, so
    // a press on one of them is a press on a button and not a drag of the
    // window. Rebuilt every frame by drawTopBar.
    struct Rect { float x0, y0, x1, y1; };
    std::vector<Rect> noDrag;

    // While a menu is open the bar is not a handle.
    bool popupOpen = false;

    // Asked for by the bar's buttons; the application carries them out.
    bool wantMinimize = false;
    bool wantToggleMaximize = false;
    bool wantClose = false;
};

struct UiContext {
    // Text for the active modal operation, shown at the top of the viewport.
    std::string  toolStatus;
    std::string  notice;
    float        noticeAge = 0.0f;

    // The file, for the title.
    std::string  projectName;
    bool         dirty = false;

    // Live measurement, shown while the measure tool is active.
    bool          measuring = false;
    MeasureResult measurement;
    size_t        measurePicks = 0;
    bool         canUndo = false;
    bool         canRedo = false;

    Scene*       scene  = nullptr;
    Camera*      camera = nullptr;
    ViewOptions* view   = nullptr;
    UiStats      stats;
    UiActions    actions;
    FrameState   frame;
};

// The logo, from assets. Without it the bar shows the wordmark in text.
bool loadBrandAssets(const std::string& assetDir);
void unloadBrandAssets();

// The bar along the top. Returns the height it took.
float drawTopBar(UiContext& ctx);

void drawOutliner(UiContext& ctx);
void drawInspector(UiContext& ctx);

// What floats over the viewport that is not the model: the operation's
// status at the top left, a notice when there is one, and the scene's numbers
// at the bottom right.
void drawViewportOverlays(UiContext& ctx, float x, float y, float w, float h);

void drawMeasurePanel(UiContext& ctx);

// Body of the add-object menu, shared by the bar and the Shift+A popup.
void drawAddMenuItems(UiContext& ctx);

} // namespace tg
