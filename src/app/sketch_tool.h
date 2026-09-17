// Tangent - drawing a sketch, and sweeping a region of it into a solid.
//
// The create tool draws a profile and consumes it: the prism is built and the
// rectangle it came from is gone. This tool draws a sketch that stays -- lines,
// rectangles, circles and arcs on a plane, held together by the constraints the
// drawing implies and sized by dimensions that can be changed afterwards -- and
// commits it to the history as a Sketch feature, with an Extrude Profile for
// each region swept out of it.
//
// The steps:
//
//   SelectPlane   an origin plane or a face, the same as the create tool
//   Draw          click out geometry; the solver runs after every change
//   Regions       pick the closed regions to sweep (one is picked for you)
//   Depth         pull or type the depth, choose join / cut / new body
//
// What drawing implies, it records. A line that ends on an existing point shares
// that point rather than sitting on top of it, so corners stay corners when a
// dimension moves them. A line drawn level or plumb is held that way. A
// rectangle comes with its width and height, a circle with its radius. Nothing
// else is guessed: the sketch says what was drawn, and the rest is the user's to
// add with the Dimension mode.
//
// The same tool re-opens a sketch already in the history, from the History
// panel: the change replaces that sketch where it stands, and everything built
// from it is re-run.
#pragma once

#include "app/camera.h"
#include "app/create_tool.h"
#include "app/plane_snap.h"
#include "app/undo.h"
#include "render/renderer.h"
#include "scene/scene.h"
#include "sketch/sketch.h"

#include <string>
#include <vector>

namespace tg {

enum class SketchStage { None, SelectPlane, Draw, Regions, Depth };

enum class SketchMode { Line, Rectangle, Circle, Arc, Dimension };

const char* sketchModeName(SketchMode mode);

class SketchTool {
public:
    bool active() const { return stage_ != SketchStage::None; }
    SketchStage stage() const { return stage_; }
    SketchMode mode() const { return mode_; }

    // A new sketch: a plane is chosen first.
    void start();

    // Re-opens the Sketch feature `sketchUid` of `object` for editing, squared
    // up to its plane. False, with an error, if there is no such feature.
    bool startEdit(const Scene& scene, ObjectId object, ElementId sketchUid, Camera& camera);

    // Leaves without changing anything.
    void cancel(Camera& camera);

    // ---- Plane ------------------------------------------------------------
    void choosePlane(PlaneChoice choice, Camera& camera);

    // Draws on `frame`. `faceObject` is the object whose face it lies on, or
    // kNoObject for a plane standing on its own. Squares the camera up to it
    // when one is given.
    void setPlane(const PlaneFrame& frame, ObjectId faceObject, Camera* camera);
    const PlaneFrame& plane() const { return plane_; }
    ObjectId faceObject() const { return faceObject_; }

    // ---- Drawing ----------------------------------------------------------
    void setMode(SketchMode mode);

    // A click at plane coordinates: exactly what clicking there does, with the
    // pointer's own snapping already decided. An existing point within
    // `pickRadiusMm()` of it is reused rather than doubled.
    void clickAt(Vec2 uv);

    // Commits the shape being drawn with the typed or previewed size. What
    // Enter does while a shape is part-way drawn.
    bool commitPending();

    // Ends a chain of lines, or abandons a half-drawn shape.
    void clearPending();
    bool pending() const { return !clicks_.empty(); }

    // Types one character into the size of what is being drawn, or of the
    // dimension picked in Dimension mode. Digits, '.', '-', backspace (8) and
    // tab (9, next field). False for anything else.
    bool typeKey(int key);

    // Adds or selects the dimension of an entity: a line's length, a circle's
    // or an arc's radius. Returns the constraint id, or kNoSketchId.
    SketchId dimensionEntity(SketchId entity);

    // Changes a dimension and re-solves. On a value the sketch cannot reach it
    // stays as it was, and false is returned with the reason in takeError().
    bool setDimension(SketchId constraint, Real value);

    // Removes an entity, the constraints on it, and any point nothing else is
    // built on.
    bool deleteEntity(SketchId entity);
    bool toggleConstruction(SketchId entity);

    // Steps back through the edits made in this session of the tool.
    bool undoEdit();

    const Sketch& sketch() const { return sketch_; }
    const SketchSolve& solveState() const { return solved_; }
    SketchId activeDimension() const { return activeDim_; }

    // ---- Extruding ----------------------------------------------------------
    // From drawing to choosing regions, handing `camera` back to the view it
    // was in before the sketch squared it up. False, with an error, when
    // nothing in the sketch closes.
    bool beginExtrude(Camera* camera = nullptr);
    const std::vector<SketchProfile>& regions() const { return regions_; }
    const std::vector<SketchId>& chosenRegions() const { return chosen_; }
    void toggleRegion(SketchId key);

    // From regions to depth.
    bool beginDepth();
    void setDepth(Real depth) { depth_ = depth; }
    Real depth() const { return depth_; }
    void setOp(CreateOp op) { op_ = op; }
    CreateOp op() const { return op_; }
    CreateOp resolvedOp() const;

    // Commits what was drawn: the sketch, and an Extrude Profile for each chosen
    // region when `extrude` is set. Without a region the sketch is only kept
    // when there is a part to keep it in -- one drawn on a face, or one being
    // edited -- since a sketch on its own is not yet an object.
    bool finish(Scene& scene, Camera& camera, UndoStack& undo, bool extrude);
    bool canFinishWithoutExtrude() const { return editing() || faceObject_ != kNoObject; }

    bool editing() const { return editObject_ != kNoObject; }

    // Why the last step refused, or empty. Reading it clears it.
    std::string takeError() { std::string e; e.swap(error_); return e; }

    // ---- Per frame ----------------------------------------------------------
    void update(const Scene& scene, const Camera& camera, Vec2 mousePx, bool snap);
    void handleMouseDown(Scene& scene, Camera& camera, UndoStack& undo);
    void handleRightClick(Camera& camera);

    // Keys as characters: letters upper-case, 27 escape, 13 enter, 8 backspace,
    // 9 tab, 127 delete. False when the key means nothing here.
    bool handleKey(int key, bool shift, bool ctrl, Scene& scene, Camera& camera, UndoStack& undo);

    void drawOverlay(const Scene& scene, const Camera& camera, Renderer& renderer) const;
    void drawHud(Scene& scene, Camera& camera, UndoStack& undo, bool& finished);

    // Where the dialog goes, and where the viewport's own corner is so labels
    // on the geometry land on it.
    void setHudOrigin(float x, float y) { hudX_ = x; hudY_ = y; }
    void setViewportOrigin(float x, float y) { viewX_ = x; viewY_ = y; }

    Real pickRadiusMm() const { return pickMm_; }

private:
    SketchStage stage_ = SketchStage::None;
    SketchMode  mode_  = SketchMode::Line;

    PlaneFrame plane_;
    PlaneChoice hoveredChoice_ = PlaneChoice::XY;
    ObjectId faceObject_ = kNoObject;
    ObjectId editObject_ = kNoObject;
    ElementId editUid_ = 0;
    SavedCamera savedCamera_;
    bool cameraSaved_ = false;

    Sketch sketch_;
    SketchSolve solved_;
    std::vector<Sketch> history_;
    std::vector<SketchProfile> regions_;

    // The shape being drawn: where each click landed, and the point it reused.
    std::vector<Vec2> clicks_;
    std::vector<SketchId> clickPoints_;

    // Where the next click goes, and what it is on.
    Vec2 cursor_{0, 0};
    bool cursorValid_ = false;
    SketchId hoverPoint_ = kNoSketchId;
    SketchId hoverEntity_ = kNoSketchId;
    SketchId hoverRegion_ = kNoSketchId;
    PlaneSnap snap_;
    enum class Lock { None, Horizontal, Vertical } lock_ = Lock::None;
    Real pickMm_ = 0.5;

    // A size typed for the shape being drawn: 0 is the length, radius or width,
    // 1 a rectangle's height. Or, in Dimension mode, the value of activeDim_.
    std::string typed_;
    int typedField_ = 0;
    bool fixed_[2] = {false, false};
    Real fixedValue_[2] = {0, 0};
    SketchId activeDim_ = kNoSketchId;

    std::vector<SketchId> chosen_;
    Real depth_ = 10.0;
    Real depthBase_ = 10.0;
    bool depthTyped_ = false;
    CreateOp op_ = CreateOp::Auto;

    bool escapeArmed_ = false;
    std::string error_;

    float hudX_ = 20.0f, hudY_ = 20.0f;
    float viewX_ = 0.0f, viewY_ = 0.0f;

    // Solves after an edit. When the sketch will not solve, the edit is taken
    // back and `what` is reported with the reason.
    bool settle(Sketch before, const char* what);
    SketchSolve solveNow();
    void refreshRegions();

    SketchId pointAt(Vec2 uv, SketchId reuse);
    SketchId nearestPoint(Vec2 uv, Real radius) const;
    Vec2 pointUV(SketchId id) const;

    // The second point of the shape being drawn, after locks and typed sizes.
    Vec2 previewEnd() const;

    void squareUp(Camera& camera);
    void restoreCamera(Camera& camera);
    void resetDrawing();

    bool unproject(const Camera& camera, Vec2 mousePx, Vec2& uv) const;
    Real depthFromPointer(const Camera& camera, Vec2 mousePx) const;
    Vec2 chosenCentre() const;

    // The world frame the sketch is drawn in, and the object's own frame it is
    // stored in when it belongs to an object.
    static SketchPlane planeToLocal(const SketchPlane& world, const Mat4& model);
    static SketchPlane planeToWorld(const SketchPlane& local, const Mat4& model);
};

} // namespace tg
