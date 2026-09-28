// Tangent - joining one part to another.
//
// Two things to point at, in the order it is said: the part that moves, then
// the part it goes on. Each is a face, an edge or a corner, and the frame it
// gives -- see scene/assembly.h -- is drawn under the pointer before it is
// clicked, so what a click will mean is never a guess. Once the first is
// picked, pointing at the second shows the moving part, see-through, where the
// joint would put it.
//
// The second click makes the joint and the panel stays, adjusting it: what
// kind of joint, which way round, how far apart, how far turned, and the
// motion itself as a bar that can be dragged -- which is how a hinge is swung
// or a drawer pulled to see that it clears. Done puts the panel away. The same
// panel opens on a joint picked from the outliner.
#pragma once

#include "app/camera.h"
#include "app/undo.h"
#include "render/renderer.h"
#include "scene/scene.h"

#include <functional>
#include <string>
#include <vector>

namespace tg {

class JointTool {
public:
    bool active() const { return stage_ != Stage::None; }
    // Still choosing what to join: the view's clicks are the tool's.
    bool picking() const { return stage_ == Stage::Moving || stage_ == Stage::Fixed; }
    // Made, and being adjusted in the panel.
    bool adjusting() const { return stage_ == Stage::Adjust; }
    uint32_t jointId() const { return stage_ == Stage::Adjust ? joint_ : 0; }

    // A new joint. A face, edge or corner already selected on a body is taken
    // as the first pick.
    void start(const Scene& scene);
    // The panel on a joint that is already there.
    void edit(Scene& scene, uint32_t id);
    // Leaves: while picking, nothing was made; while adjusting, the joint
    // stays as the panel left it.
    void finish(UndoStack& undo);
    void cancel() { stage_ = Stage::None; hover_ = Hover{}; }

    // ---- Picking, as a click does it -------------------------------------------
    // What `at`/`element` name on `object`, as handles; `onFace` is the face
    // the ray met. False, with the reason in takeError(), for something a
    // joint cannot stand on, or a second pick on the same part.
    bool pick(Scene& scene, UndoStack& undo, ObjectId object, JointAt at, Index element, Index onFace);

    // ---- Adjusting ----------------------------------------------------------------
    // Each applies to the joint at once and is one undo step with everything
    // done in the panel since it opened.
    void setKind(Scene& scene, UndoStack& undo, JointKind kind);
    void setFlip(Scene& scene, UndoStack& undo, bool flip);
    void setReverse(Scene& scene, UndoStack& undo, bool reverse);
    // Keeps the moving part where it was built, or lays it onto the fixed pick.
    void setAsBuilt(Scene& scene, UndoStack& undo, bool asBuilt);
    void setOffset(Scene& scene, UndoStack& undo, Real mm);
    void setAngle(Scene& scene, UndoStack& undo, Real radians);
    void setMotion(Scene& scene, UndoStack& undo, Real value);   // radians or mm, by kind
    // Takes the joint away, leaving what it placed where it is.
    void remove(Scene& scene, UndoStack& undo);

    std::string prompt() const;
    std::string takeError() { std::string e; e.swap(error_); return e; }

    // A bar in the panel was clicked and a number is being typed into it: the
    // digits, the point, the minus, Backspace and Enter are the tool's.
    bool typing() const { return typing_ != Field::None; }

    // ---- Per frame ----------------------------------------------------------------
    void update(const Scene& scene, const Camera& camera, Vec2 mousePx);
    void handleMouseDown(Scene& scene, UndoStack& undo);
    // 27 escape, 13 enter, 8 backspace, 'F' flip. False when it means nothing.
    bool handleKey(int key, Scene& scene, UndoStack& undo);
    void drawOverlay(const Scene& scene, const Camera& camera, Renderer& renderer) const;
    void drawHud(Scene& scene, UndoStack& undo, bool& finished);
    void drawPointerPrompt(Vec2 mouseScreen) const;

private:
    enum class Stage { None, Moving, Fixed, Adjust };
    Stage stage_ = Stage::None;

    JointSide moving_;                 // the first pick, once made
    bool movingAxial_ = false;         // it has an axis a part turns about
    const char* movingNoun_ = "Face";
    uint32_t joint_ = 0;               // the joint being adjusted
    std::string error_;

    // What the pointer is over, and the frame it would give, in the world.
    struct Hover {
        ObjectId object = kNoObject;
        JointAt at = JointAt::Face;
        Index element = kInvalid;
        Index onFace = kInvalid;
        JointFrame frame;              // in the body's own space
        bool ok = false;
        std::string why;
        const char* noun = "Face";
        std::vector<Vec3> line;        // an edge, as drawn, in the body's own space
        // Where the moving part would go, as a move from where it is: the
        // see-through preview while the second pick is chosen.
        bool placed = false;
        Rigid delta;
        std::vector<ObjectId> unit;
    };
    Hover hover_;

    // Every change the panel makes goes through here: applied, solved, and
    // folded into the one undo step the panel makes, keyed by `mergeKey_`.
    std::string mergeKey_;
    uint32_t session_ = 0;
    void apply(Scene& scene, UndoStack& undo, const char* what, const std::function<void(Joint&)>& change);
    Joint* joint(Scene& scene) const { return scene.assembly().joint(joint_); }

    // Which bar a number is being typed into, and what has been typed.
    enum class Field { None, Offset, Angle, Motion, SlideX, SlideY, Lo, Hi };
    Field typing_ = Field::None;
    std::string typed_;
    void applyTyped(Scene& scene, UndoStack& undo, double v);

    // Animating the motion through its range, for looking at, from the panel.
    bool sweeping_ = false;
    Real sweepT_ = 0.0;
};

} // namespace tg
