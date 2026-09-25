#pragma once

#include "app/camera.h"
#include "app/extrude_ops.h"
#include "app/plane_pick.h"
#include "app/plane_snap.h"
#include "app/snap.h"
#include "app/undo.h"
#include "core/math.h"
#include "geom/body.h"
#include "mesh/primitives.h"
#include "render/renderer.h"
#include "scene/scene.h"

#include <string>
#include <vector>

namespace tg {

enum class CreateStage {
    None,
    SelectPlane,       // 1. Hover/select an origin plane (XY/XZ/YZ) or an object face
    DrawProfile_Pt1,   // 2. Head-on orthographic: Click Corner 1 / Center point
    DrawProfile_Pt2,   // 3. Head-on orthographic: Move mouse to Corner 2 / Radius -> Click to commit
    AdjustProfile,     // 4. Return to perspective: Drag edges/corners, adjust fillet radius, [OK] button
    ExtrudeDepth,      // 5. Perspective: Mouse movement sets depth (positive = solid/join, negative = boolean cut)
    Applied            // 6. Done, and adjustable from the panel until Done is pressed
};


struct SavedCamera {
    Vec3 target{0.0f, 0.0f, 0.0f};
    float distance = 90.0f;
    float yaw = 0.0f;
    float pitch = 0.0f;
    bool orthographic = false;
};

class CreateTool {
public:
    CreateTool() = default;

    // Drawing or extruding: the tool has the viewport. Once applied it no
    // longer does -- the result is in the scene and the panel only adjusts it.
    bool active() const { return stage_ != CreateStage::None && stage_ != CreateStage::Applied; }
    bool applied() const { return stage_ == CreateStage::Applied; }
    CreateStage stage() const { return stage_; }
    PrimitiveKind kind() const { return kind_; }

    void start(PrimitiveKind kind);
    void cancel(Camera& camera);

    // Plane selection
    void setHoveredPlane(PlaneChoice choice, Vec3 point, Vec3 normal,
                         ObjectId faceObj = kNoObject, Index faceIdx = kInvalid);
    void commitPlaneSelection(Camera& camera);
    void choosePlane(PlaneChoice choice, Camera& camera, const Scene& scene);

    // Per-frame mouse update and snapping
    void update(const Scene& scene, const Camera& camera, Vec2 mousePx, bool snap);

    // Mouse handlers (press-and-hold to drag, release to commit)
    void handleMouseDown(Vec2 mousePx, Scene& scene, Camera& camera, UndoStack& undo);
    void handleMouseUp(Vec2 mousePx, Camera& camera);
    void handleLeftClick(Scene& scene, Camera& camera, UndoStack& undo);
    void handleRightClick(Camera& camera);

    // Numeric typing and shortcut support (E for Extrude, F for Fillet, Esc for Cancel, 1/3/7 for Planes)
    bool handleKey(int key, bool shift, bool ctrl, Camera& camera, Scene& scene, UndoStack& undo);
    bool handleKey(int key, bool shift, bool ctrl, Camera& camera); // backwards compatibility overload

    // Render 3D overlays (origin plane tiles, 2D grid, 2D profile outlines, handles, 3D extrusion preview)
    void drawOverlay(const Scene& scene, const Camera& camera, Renderer& renderer) const;

    // Render 2D ImGui HUD (floating dimensions, action buttons, quick plane selectors)
    bool drawHud(Scene& scene, Camera& camera, UndoStack& undo, bool& outFinished);

    // Execute final creation (solid creation or boolean cut)
    bool finishCreation(Scene& scene, Camera& camera, UndoStack& undo);

    // Why the last finishCreation refused, or empty. A cut that cannot be made
    // has to say so: it used to fall through and add the cutter to the scene as
    // a solid body, which looks like the tool working and is the opposite of
    // what was asked for. Reading it clears it.
    std::string takeError() { std::string e; e.swap(lastError_); return e; }

    // Configuration / Testing setters
    void setProfileRect(Vec2 p1, Vec2 p2, Real cornerRadius = 0.0) {
        pt1_ = p1;
        pt2_ = p2;
        for (int i = 0; i < 4; ++i) cornerRadii_[i] = cornerRadius;
        currentWidth_ = std::fabs(p2.x - p1.x);
        currentDepth_ = std::fabs(p2.y - p1.y);
    }
    void setProfileRect(Vec2 p1, Vec2 p2, const Real radii[4]) {
        pt1_ = p1;
        pt2_ = p2;
        for (int i = 0; i < 4; ++i) cornerRadii_[i] = radii[i];
        currentWidth_ = std::fabs(p2.x - p1.x);
        currentDepth_ = std::fabs(p2.y - p1.y);
    }
    void setCornerRadius(int idx, Real r) { if (idx >= 0 && idx < 4) cornerRadii_[idx] = r; }
    void setCornerRadius(Real r) { for (int i = 0; i < 4; ++i) cornerRadii_[i] = r; }
    Real cornerRadius(int idx) const { return (idx >= 0 && idx < 4) ? cornerRadii_[idx] : 0.0; }
    Real uniformCornerRadius() const { return (cornerRadii_[0] + cornerRadii_[1] + cornerRadii_[2] + cornerRadii_[3]) * 0.25; }

    void setProfileCircle(Vec2 center, Real radius) {
        pt1_ = center;
        currentRadius_ = radius;
    }
    void setExtrudeDepth(Real depth) { extrudeDepth_ = depth; }
    void setStage(CreateStage s) { stage_ = s; }
    void setHudOrigin(float x, float y) { hudX_ = x; hudY_ = y; }

    // What the cursor is currently snapped to, if anything, and why. The
    // overlay marks it and the HUD names it: a snap that happens invisibly is
    // indistinguishable from the tool being imprecise.
    const PlaneSnap& activeSnap() const { return activeSnap_; }

    // The sketch plane, as the snapper wants it.
    PlaneFrame plane() const { return {planeOrigin_, planeU_, planeV_, planeNormal_}; }

    // What is being typed, for the HUD and for tests. Empty when nothing is
    // half-entered, which is the normal state.
    const std::string& typedValue() const { return typedValue_; }
    int typedField() const { return typedField_; }
    bool typing() const { return !typedValue_.empty(); }

    // Whether a dimension has been pinned to a typed number, and to what.
    bool fieldFixed(int field) const { return field >= 0 && field < 2 && fieldFixed_[field]; }
    Real fieldValue(int field) const { return field >= 0 && field < 2 ? fieldValue_[field] : 0.0; }

    // How many dimensions this stage has: one for a circle, a depth or a
    // fillet, two for a rectangle.
    int fieldCount() const;
    const char* fieldName(int field) const;

    // What the dimension currently reads, fixed or not.
    Real fieldDisplay(int field) const;

    // The profile's extent on its plane, as the tool currently holds it.
    Vec2 profileMin() const { return pt1_; }
    Vec2 profileMax() const { return pt2_; }
    bool rounding() const { return isFilleting_; }

    // What the solid does to the bodies it reaches. Until one is picked the
    // depth decides -- see ExtrudeChoice -- and op() says what it decided.
    ExtrudeOp op() const {
        ExtrudeChoice c = choice_;
        c.follow(extrudeDepth_, faceObject_ != kNoObject);
        return c.op;
    }
    bool opFollowsDrag() const { return choice_.automatic; }
    void setOp(ExtrudeOp op) { choice_.pick(op); }

    // The bodies the extrusion reaches, and which of them it acts on.
    const ExtrudeReach& reach() const { return reach_; }
    void toggleBody(ObjectId id) { reach_.toggle(id); }
    void refreshReach(const Scene& scene);

    // Once applied: whether the panel has changed something since, so the
    // application can take the extrusion back and make it again as it now
    // stands. Reading it clears it.
    bool takeAdjusted() { const bool a = adjusted_; adjusted_ = false; return a; }
    bool recommit(Scene& scene, Camera& camera, UndoStack& undo);
    void dismissApplied() { if (stage_ == CreateStage::Applied) stage_ = CreateStage::None; }

    // True when the profile was drawn on an object's face, so there is a body
    // to combine with and the choice means anything.
    bool hasTargetBody() const { return faceObject_ != kNoObject; }

    // Utilities for 2D profile and 3D prism generation (also exposed for testing and future sketching)
    static std::vector<Vec2> makeRectPolygon(Vec2 p1, Vec2 p2, const Real cornerRadii[4], int arcSegments = 6);
    static std::vector<Vec2> makeRectPolygon(Vec2 p1, Vec2 p2, Real cornerRadius, int arcSegments = 6);
    static std::vector<Vec2> makeCirclePolygon(Vec2 center, Real radius, int segments = 32);
    static bool makePrismMesh(const std::vector<Vec2>& poly2D,
                             Vec3 planeOrigin, Vec3 planeU, Vec3 planeV, Vec3 planeN,
                             Real z0, Real z1, Mesh& out);

private:
    CreateStage stage_ = CreateStage::None;
    PrimitiveKind kind_ = PrimitiveKind::Box;

    // Plane definition
    PlaneChoice hoveredPlane_ = PlaneChoice::XY;
    // Asks for the plane the way the sketch tool does. adoptPickedPlane takes
    // what it chose as the plane hovered.
    PlanePicker picker_;
    void adoptPickedPlane();
    // How far the plane stands off where it was put, and how far it leans:
    // the same two numbers a sketch's plane has.
    PlaneFrame planeBase_;
    Real planeOffset_ = 0.0;
    Real planeTilt_ = 0.0;
    void applyPlaneShift();
    PlaneChoice selectedPlane_ = PlaneChoice::None;
    Vec3 planeOrigin_{0, 0, 0};
    Vec3 planeNormal_{0, 0, 1};
    Vec3 planeU_{1, 0, 0};
    Vec3 planeV_{0, 1, 0};
    ObjectId faceObject_ = kNoObject;
    Index faceIndex_ = kInvalid;

    SavedCamera savedCamera_;

    // 2D Profile coordinates on plane (in mm)
    Vec2 pt1_{0, 0};
    Vec2 pt2_{0, 0};
    Real cornerRadii_[4] = {0.0, 0.0, 0.0, 0.0}; // 0: BR, 1: TR, 2: TL, 3: BL
    Real currentRadius_ = 10.0; // for cylinder/circle
    Real currentWidth_ = 20.0;  // for rectangle/box
    Real currentDepth_ = 20.0;  // for rectangle/box

    // Active drag handle during AdjustProfile
    enum class HandleId {
        None,
        EdgeLeft,
        EdgeRight,
        EdgeBottom,
        EdgeTop,
        Corner0, // Bottom-Right
        Corner1, // Top-Right
        Corner2, // Top-Left
        Corner3, // Bottom-Left
        RadiusHandle,
        FaceCenter
    };
    HandleId hoveredHandle_ = HandleId::None;
    HandleId activeHandle_ = HandleId::None;
    HandleId selectedElement_ = HandleId::None;

    bool isMouseDown_ = false;
    bool isDragging_ = false;
    bool isFilleting_ = false;

    Vec2 dragStartMouse_{0, 0};
    Vec2 dragStartMouseUV_{0, 0};

    // Where the handle sat relative to the cursor when it was grabbed. Held so
    // a handle does not jump to the pointer on the first frame of a drag, and
    // so it is the handle's own point that gets snapped rather than the
    // cursor's -- a corner brought level with a hole has to be the corner.
    Vec2 dragGrabOffset_{0, 0};
    Vec2 dragStartPt1_{0, 0};
    Vec2 dragStartPt2_{0, 0};
    Real dragStartFillets_[4] = {0.0, 0.0, 0.0, 0.0};

    PlaneSnap activeSnap_;

    // Where the dialog sits. Set by the application from the viewport's own
    // corner, since the tool has no idea where the viewport is.
    float hudX_ = 20.0f, hudY_ = 20.0f;

    Vec2 filletRefUV_{0, 0};
    std::vector<int> activeFilletCorners_;

    ExtrudeChoice choice_;
    ExtrudeReach reach_;
    bool adjusted_ = false;
    std::string reachToolKey_;           // the tool reachTool_ was built for
    Body reachTool_;

    // Makes the extrusion as the tool now stands: what finishCreation does,
    // and what an adjustment does again.
    bool commitExtrusion(Scene& scene, UndoStack& undo);

    // Extrusion depth (in mm)
    Real extrudeDepth_ = 20.0;
    Vec2 extrudeStartMouse_{0, 0};
    Real extrudeBaseDepth_ = 20.0;

    // Numeric entry.
    //
    // A dimension that has been typed is *fixed*: the mouse stops driving that
    // one and goes on driving the others. Typing a width and then sweeping the
    // depth out by hand is the whole reason to be able to type at all, and the
    // tool used to freeze the mouse entirely while anything was in the buffer
    // -- which made every dimension wait on one of them.
    //
    // The digits apply as they are typed rather than on Enter, so the profile
    // follows them. Backspacing the field empty hands it back to the mouse.
    std::string typedValue_;
    int  typedField_ = 0;              // 0 = width, radius or depth; 1 = depth
    bool fieldFixed_[2] = {false, false};
    Real fieldValue_[2] = {0.0, 0.0};

    // Set by finishCreation when it refuses; drained by the application.
    std::string lastError_;

    // Helpers
    // Accumulates one keystroke into typedValue_. Returns false for a key that
    // is not part of a number, so the caller can go on to its own shortcuts.
    bool handleTypedKey(int key);

    // Parses what was typed into the dimension the current stage is about, and
    // clears the buffer. Returns false if it does not parse, leaving the
    // dimension alone -- a half-typed "12." must not become 12.
    bool applyTypedValue();

    void restoreCamera(Camera& camera);

    // Where a handle sits, in plane coordinates.
    Vec2 handlePointUV(HandleId id) const;

    // Corner radii that no longer fit after a resize. A rectangle dragged down
    // to eight millimetres cannot keep the five-millimetre rounds it had.
    void clampCornerRadii();

    // Releases the dimensions a handle moves from whatever was typed for them.
    void clearLocks(HandleId id);

    // Pins one dimension to a value, applying it to the profile as it goes.
    void setField(int field, Real v);

    // Re-reads the buffer into the field being typed. Called on every
    // keystroke, so "2", "25", "25." and "25.4" each land as they are typed.
    void syncTypedField();

    // Forgets every fixed dimension. Each stage asks its own questions.
    // The row of dimensions in the banner: name, value, and whether the
    // keyboard is holding it or the mouse still has it.
    void drawDimensionFields();

    void clearFields() {
        typedValue_.clear();
        typedField_ = 0;
        fieldFixed_[0] = fieldFixed_[1] = false;
    }
    void computePlaneBasis(Vec3 normal);
    bool unprojectToPlane(const Camera& camera, Vec2 mousePx, Vec2& outUV) const;
    Real rayPlaneExtrudeDepth(const Camera& camera, Vec2 mousePx) const;
    std::vector<Vec2> getCurrentProfile() const;

    // The profile as spans rather than as points: a rounded corner is one arc
    // and a circle is four, so the exact kernel gets the shape that was drawn
    // instead of a polygon that resembles it. `arcs` is parallel to `points`
    // and holds each span's sagitta, zero for a straight one.
    void getCurrentProfileArcs(std::vector<Vec3>& points, std::vector<Real>& arcs) const;

    Body buildCurrentSolid(Real depth, Backend backend) const;
};

} // namespace tg
