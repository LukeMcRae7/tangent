#pragma once

#include "app/camera.h"
#include "app/undo.h"
#include "core/math.h"
#include "mesh/halfedge.h"
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
    ExtrudeDepth       // 5. Perspective: Mouse movement sets depth (positive = solid/join, negative = boolean cut)
};

enum class PlaneChoice { None, XY, XZ, YZ, Face };

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

    bool active() const { return stage_ != CreateStage::None; }
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
    Vec2 dragStartPt1_{0, 0};
    Vec2 dragStartPt2_{0, 0};
    Real dragStartFillets_[4] = {0.0, 0.0, 0.0, 0.0};

    Vec2 filletRefUV_{0, 0};
    std::vector<int> activeFilletCorners_;

    // Extrusion depth (in mm)
    Real extrudeDepth_ = 20.0;
    Vec2 extrudeStartMouse_{0, 0};
    Real extrudeBaseDepth_ = 20.0;

    // Numeric input buffer
    std::string typedValue_;

    // Helpers
    void computePlaneBasis(Vec3 normal);
    bool unprojectToPlane(const Camera& camera, Vec2 mousePx, Vec2& outUV) const;
    Real rayPlaneExtrudeDepth(const Camera& camera, Vec2 mousePx) const;
    std::vector<Vec2> getCurrentProfile() const;
    Mesh buildCurrentSolid(Real depth) const;
};

} // namespace tg
