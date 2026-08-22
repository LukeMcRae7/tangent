// Tangent - modal move / rotate / scale.
//
// Blender's modal grammar: press G, R or S, move the mouse, then X/Y/Z to
// constrain to an axis (Shift+X/Y/Z for the perpendicular plane), or type a
// number for an exact value. Left click or Enter commits; Escape or right
// click restores exactly what was there before.
//
// The tool mutates transforms live so the viewport shows the real result, and
// hands back a single undo command on commit rather than one per frame.
#pragma once

#include "app/camera.h"
#include "app/undo.h"
#include "render/renderer.h"
#include "scene/scene.h"

#include <memory>
#include <string>
#include <vector>

namespace tg {

enum class TransformMode { None, Translate, Rotate, Scale };

enum class Constraint {
    None,
    AxisX, AxisY, AxisZ,       // along one axis
    PlaneX, PlaneY, PlaneZ,    // within the plane perpendicular to that axis
};

class TransformTool {
public:
    bool          active() const { return mode_ != TransformMode::None; }
    TransformMode mode() const { return mode_; }

    // Returns false if there is nothing selected to transform.
    bool begin(TransformMode mode, Scene& scene, const Camera& camera, Vec2 mousePx);

    // Pressing the axis already in force clears it, matching Blender.
    void setConstraint(Constraint c);

    void typeCharacter(char c);   // digits, '.', '-' for exact entry
    void backspace();
    bool hasTypedValue() const { return !typed_.empty(); }

    void update(Scene& scene, const Camera& camera, Vec2 mousePx, bool snap);

    // Commits and yields the undo command, or nullptr if nothing moved.
    std::unique_ptr<Command> confirm(Scene& scene);
    void cancel(Scene& scene);

    // One-line readout for the status bar, e.g. "Move X  12.50 mm".
    std::string statusText() const;

    // The snap increment currently in force, in millimetres. Zero when the
    // tool is not snapping.
    float snapStep() const { return snapStep_; }

    void drawOverlay(Renderer& renderer, const Camera& camera) const;

private:
    struct Entry {
        ObjectId  id;
        Transform before;
    };

    Vec3  constraintAxis() const;
    bool  isPlane() const;
    void  apply(Scene& scene, const Camera& camera, Vec2 mousePx, bool snap);

    TransformMode mode_ = TransformMode::None;
    Constraint    constraint_ = Constraint::None;

    std::vector<Entry> entries_;
    Vec3  pivot_;
    Vec2  startMouse_;
    Vec2  pivotPx_;

    // Rotation accumulates across frames so a sweep past 180 degrees keeps
    // going rather than wrapping back on itself.
    float rotateAccum_ = 0.0f;
    float rotateLast_  = 0.0f;

    std::string typed_;
    float snapStep_ = 0.0f;
    float amount_ = 0.0f;    // last applied scalar, for the readout
    Vec3  delta_{};          // last applied translation, for the readout
};

const char* transformModeName(TransformMode m);

} // namespace tg
