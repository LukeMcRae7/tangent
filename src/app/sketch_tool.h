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
#include "app/extrude_ops.h"
#include "app/plane_snap.h"
#include "app/undo.h"
#include "render/renderer.h"
#include "scene/scene.h"
#include "sketch/sketch.h"
#include "sketch/svg.h"

#include <chrono>
#include <memory>
#include <unordered_map>
#include <string>
#include <vector>

namespace tg {

// Applied: extruded, and the panel adjusts the extrusion until Done.
enum class SketchStage { None, SelectPlane, Draw, Regions, Depth, Applied };

// Select comes first because it is what editing a sketch that already exists is
// mostly made of: taking hold of something and moving it.
enum class SketchMode { Select, Line, Rectangle, Circle, Arc, Dimension };

const char* sketchModeName(SketchMode mode);

class SketchTool {
public:
    // Drawing or extruding: the tool has the viewport. Once applied it does not.
    bool active() const { return stage_ != SketchStage::None && stage_ != SketchStage::Applied; }
    bool applied() const { return stage_ == SketchStage::Applied; }
    SketchStage stage() const { return stage_; }
    SketchMode mode() const { return mode_; }

    // A new sketch: a plane is chosen first.
    void start();

    // Re-opens the Sketch feature `sketchUid` of `object` for editing, squared
    // up to its plane. False, with an error, if there is no such feature.
    bool startEdit(const Scene& scene, ObjectId object, ElementId sketchUid, Camera& camera);

    // A new sketch with an SVG drawing waiting to go in it: a plane or a face
    // is chosen first, the way a new sketch starts, and the drawing lands on it
    // centred, at the size the file gives, to be sized and moved from there.
    void startImport(SvgDrawing drawing, std::string name);

    // An SVG drawing into the sketch being drawn, centred on the sketch's
    // origin. What it added stays adjustable -- see placement() -- until
    // something else is drawn. False, with an error, outside drawing.
    bool importSvg(SvgDrawing drawing, std::string name);

    // The drawing is waiting for a plane.
    bool importPending() const { return pendingSvg_.ok; }

    // The panel's Import SVG button was pressed: the application owns the file
    // chooser, so it asks, and hands what was chosen to importSvg().
    bool takeImportRequest() { const bool r = importRequested_; importRequested_ = false; return r; }

    // ---- The drawing just imported, while it can still be placed -----------
    bool placing() const { return !svg_.empty(); }
    const SvgPlacement& placement() const { return svgPlace_; }
    const SvgInsert& placed() const { return svg_; }
    // Moves, sizes or turns it; the sketch is solved again.
    bool setPlacement(const SvgPlacement& placement);
    // Turns the view to take in the whole of it.
    void frameDrawing(Camera& camera);
    // Which of its colours are ink -- what becomes the part -- parallel to the
    // drawing's colours. The drawing is outlined again and put back in, where
    // it was, as the same one step.
    bool setInkColours(const std::vector<bool>& ink);
    const SvgDrawing& placedDrawing() const { return svgDrawing_; }
    // Leaves it where it is, as ordinary sketch geometry.
    void endPlacing() {
        svg_ = SvgInsert{};
        svgDrawing_ = SvgDrawing{};
        svgBefore_ = Sketch{};
    }

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

    // ---- Dragging -----------------------------------------------------------
    //
    // Takes hold of a point, or of the rim of a circle or an arc, and moves it
    // as far as the constraints allow: a point held level with another slides
    // along that line, and a fully constrained sketch does not move at all.
    // The solver is built once here and re-solved on every move.
    bool beginDrag(SketchId point);
    bool beginRadiusDrag(SketchId entity);
    bool dragTo(Vec2 at);
    void endDrag();
    bool dragging() const { return drag_ != nullptr; }
    SketchId draggedPoint() const { return dragPoint_; }

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
    // Every region; none; or the filled ones -- those inside an even number of
    // others, the way an SVG fills a letter and leaves its counter open.
    void chooseAllRegions();
    void chooseNoRegions() { chosen_.clear(); }
    void chooseFilledRegions();

    // From regions to depth.
    bool beginDepth();
    void setDepth(Real depth) { depth_ = depth; }
    Real depth() const { return depth_; }

    // What the regions do to the bodies they reach. Until one is picked the
    // depth decides -- see ExtrudeChoice -- and op() says what it decided.
    ExtrudeOp op() const {
        ExtrudeChoice c = choice_;
        c.follow(depth_, owner() != kNoObject);
        return c.op;
    }
    bool opFollowsDrag() const { return choice_.automatic; }
    void setOp(ExtrudeOp op) { choice_.pick(op); }
    const ExtrudeReach& reach() const { return reach_; }
    void toggleBody(ObjectId id) { reach_.toggle(id); }
    // `now`: measure it whatever it costs, as a commit must. Otherwise a
    // drawing of hundreds of regions is not swept again on every frame of a
    // depth drag -- see the .cpp.
    void refreshReach(const Scene& scene, bool now = false);

    // Once applied, as the create tool: whether the panel changed something,
    // the extrusion made again as it now stands, and the panel put away.
    bool takeAdjusted() { const bool a = adjusted_; adjusted_ = false; return a; }
    bool recommit(Scene& scene, UndoStack& undo);
    void dismissApplied();

    // Commits what was drawn: the sketch, and an Extrude Profile for each chosen
    // region when `extrude` is set. Without one it is kept as a sketch -- in the
    // part it was drawn on, or, drawn on a plane of its own, as its own object
    // in the outliner, to be extruded whenever.
    bool finish(Scene& scene, Camera& camera, UndoStack& undo, bool extrude);

    bool editing() const { return editObject_ != kNoObject; }
    ObjectId editingObject() const { return editObject_; }
    ElementId editingUid() const { return editUid_; }

    // Why the last step refused, or empty. Reading it clears it.
    std::string takeError() { std::string e; e.swap(error_); return e; }

    // ---- Per frame ----------------------------------------------------------
    void update(const Scene& scene, const Camera& camera, Vec2 mousePx, bool snap);
    void handleMouseDown(Scene& scene, Camera& camera, UndoStack& undo);
    void handleMouseUp();
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
    // The curve last pointed at, in a drawing with too many curves to show
    // every handle: its handles stay shown after the pointer leaves the curve,
    // so they can be reached and taken hold of.
    SketchId handleCurve_ = kNoSketchId;
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

    // A drag in progress: the solver it is running on, what is held, and the
    // sketch as it was when the drag started -- one drag is one step back.
    std::unique_ptr<SketchSolver> drag_;
    Sketch dragBefore_;
    SketchId dragPoint_ = kNoSketchId;
    SketchId dragEntity_ = kNoSketchId;
    // Set when what is being dragged is held by a dimension, so the drag drives
    // that number instead of pulling against it.
    SketchId dragDim_ = kNoSketchId;
    bool dragMoved_ = false;

    std::vector<SketchId> chosen_;
    Real depth_ = 10.0;
    Real depthBase_ = 10.0;
    bool depthTyped_ = false;
    ExtrudeChoice choice_;
    ExtrudeReach reach_;
    bool adjusted_ = false;
    std::string reachToolKey_;
    Body reachTool_;
    double reachBuildMs_ = 0.0;
    std::chrono::steady_clock::time_point reachBuiltAt_{};

    // The object the sketch belongs to: the one being edited, or the one whose
    // face it was drawn on. kNoObject for a sketch on a plane of its own.
    ObjectId owner() const { return editObject_ != kNoObject ? editObject_ : faceObject_; }

    // The chosen regions swept to the depth, in the world.
    Body sweptRegions(std::string* why) const;
    bool commitExtrusion(Scene& scene, UndoStack& undo);

    bool escapeArmed_ = false;
    std::string error_;

    // An import: waiting for a plane, and once in, what it added and where.
    SvgDrawing pendingSvg_;
    std::string pendingSvgName_;
    SvgInsert svg_;
    SvgDrawing svgDrawing_;   // as imported, to outline again in other colours
    Sketch svgBefore_;        // the sketch before it went in
    SvgPlacement svgPlace_;
    std::string svgName_;
    bool importRequested_ = false;
    void drawPlacementRows();

    // ---- What is drawn, kept between frames -----------------------------------
    //
    // An imported drawing is thousands of curves. Sampling each one, and
    // hatching each region, afresh every frame is what made a large one crawl,
    // so the results are kept: a curve is sampled again only when its points
    // move or the zoom changes by half an octave -- and then as finely as its
    // size on screen needs, not 48 pieces for a letter three pixels high -- and
    // a region's outline and hatching only when one of its curves changes.
    struct EntityDraw {
        SketchId id = kNoSketchId;
        SketchCurve curve = SketchCurve::Line;
        Vec2 ctrl[4];
        Real radius = 0;
        int bucket = 0;
        uint32_t version = 0;
        std::vector<Vec2> line;       // in plane coordinates
        Vec2 lo, hi;                  // around it
    };
    struct RegionDraw {
        uint64_t signature = 0;
        std::vector<std::vector<Vec2>> loops;   // outer first, then its holes
        Vec2 lo, hi;
        Real fineMm = -1, coarseMm = -1;
        std::vector<Vec2> fine, coarse;        // hatch strokes, two points each
    };
    mutable std::vector<EntityDraw> entityDraw_;
    mutable std::unordered_map<SketchId, size_t> entityDrawAt_;
    mutable std::unordered_map<SketchId, RegionDraw> regionDraw_;
    mutable uint32_t drawVersion_ = 0;
    mutable int drawBucket_ = 0;
    mutable Real drawMmPerPx_ = 1.0;
    void refreshDrawCache(const Camera& camera) const;
    const EntityDraw* drawOf(SketchId entity) const;
    const RegionDraw& regionDrawOf(const SketchProfile& region) const;
    const std::vector<Vec2>& hatchOf(const SketchProfile& region, Real spacingPx, bool fine) const;
    bool regionHas(const SketchProfile& region, Vec2 uv) const;

    float hudX_ = 20.0f, hudY_ = 20.0f;
    float viewX_ = 0.0f, viewY_ = 0.0f;

    // Solves after an edit. When the sketch will not solve, the edit is taken
    // back and `what` is reported with the reason.
    bool settle(Sketch before, const char* what);
    SketchSolve solveNow();
    void refreshRegions();

    SketchId pointAt(Vec2 uv, SketchId reuse);

    // The dimension already sizing an entity -- a line's length, a circle's
    // radius -- or kNoSketchId.
    SketchId existingDimension(SketchId entity) const;
    bool inConflict(const SketchEntity& entity) const;
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
