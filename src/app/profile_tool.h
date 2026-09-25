// Tangent - revolve, sweep and loft: a solid from an outline and something
// that says where it goes.
//
// Each is a short list of things to point at, filled in order, with the panel
// saying at every moment what the next click is for:
//
//   Revolve   a profile, then an axis        (and an angle)
//   Sweep     a profile, then a path
//   Loft      outlines, in the order the solid runs through them
//
// A profile or an outline is either drawn -- a region of a sketch -- or already
// there: a flat face of a body. An axis is a line of a sketch, a straight edge of
// a body, or one of the world's three. A path is curves of a sketch, joined end to
// end, or edges of a body. The solid it will make is shown, see-through, as soon
// as there is enough to make it, and Finish puts it in the history of the part it
// was built from, as a step that can be edited afterwards like any other.
//
// What it acts on follows from what it was built from. A face or an edge belongs
// to a part, and the step goes into that part's history: its own faces are named
// there, and re-run with it. Sketches that stood on their own in the outliner are
// moved into the part's history, before the step that uses them.
#pragma once

#include "app/camera.h"
#include "app/extrude_ops.h"
#include "app/undo.h"
#include "mesh/halfedge.h"
#include "render/renderer.h"
#include "scene/scene.h"
#include "sketch/sketch.h"

#include <string>
#include <vector>

namespace tg {

enum class ProfileBuild { Revolve, Sweep, Loft };

const char* profileBuildName(ProfileBuild b);

// What the next click fills.
enum class ProfileSlot { Profile, Axis, Path, Outlines };

// A closed outline: regions of a sketch, or one flat face of a body.
struct OutlinePick {
    ObjectId object = kNoObject;
    ElementId sketchUid = 0;          // non-zero: regions `keys` of that sketch
    std::vector<SketchId> keys;
    FaceId face = kNoFace;            // otherwise: this face of the object's body
    bool empty() const { return object == kNoObject; }
    bool fromSketch() const { return sketchUid != 0; }
};

// A path: curves of one sketch, or edges of one body.
struct PathPick {
    ObjectId object = kNoObject;
    ElementId sketchUid = 0;
    std::vector<SketchId> entities;
    std::vector<EdgeId> edges;
    bool empty() const { return object == kNoObject; }
    bool fromSketch() const { return sketchUid != 0; }
};

// An axis: one of the world's, a line of a sketch, or a straight edge.
struct AxisPick {
    enum class Kind { None, World, SketchLine, Edge } kind = Kind::None;
    int world = 2;                    // 0 X, 1 Y, 2 Z
    ObjectId object = kNoObject;
    ElementId sketchUid = 0;
    SketchId entity = kNoSketchId;
    EdgeId edge = kInvalid;
};

class ProfileTool {
public:
    bool active() const { return active_; }
    ProfileBuild build() const { return build_; }
    ProfileSlot slot() const { return slot_; }

    // Starts empty -- or, when faces or edges are selected, with them taken as
    // what they can be: a flat face as the profile, edges as the path.
    void start(ProfileBuild build, const Scene& scene);
    // Starts with regions of a sketch already picked as the profile, or as the
    // first outline: how the sketch tool hands a drawing over.
    void startWith(ProfileBuild build, const Scene& scene, ObjectId object, ElementId sketchUid,
                   const std::vector<SketchId>& keys);
    void cancel();

    // ---- Picking, as a click does it ----------------------------------------
    // Each fills the slot it belongs to and moves on to the next empty one.
    // False, with the reason in takeError(), when it does not fit.
    bool pickRegion(const Scene& scene, ObjectId object, ElementId sketchUid, SketchId key);
    bool pickFace(const Scene& scene, ObjectId object, FaceId face);
    bool pickCurve(const Scene& scene, ObjectId object, ElementId sketchUid, SketchId entity);
    bool pickEdge(const Scene& scene, ObjectId object, EdgeId edge);
    void setWorldAxis(int axis);
    void setSlot(ProfileSlot slot) { slot_ = slot; }

    const OutlinePick& profile() const { return profile_; }
    const AxisPick& axis() const { return axis_; }
    const PathPick& path() const { return path_; }
    const std::vector<OutlinePick>& outlines() const { return outlines_; }

    void setAngle(Real radians) { angle_ = radians; }
    Real angle() const { return angle_; }
    void setReverse(bool r) { reverse_ = r; }
    void setRuled(bool r) { ruled_ = r; }
    void setOp(ExtrudeOp op) { choice_.pick(op); }
    ExtrudeOp op() const { return choice_.op; }

    // Whether what is picked builds, and if not, why. Measured when the picking
    // changes, not every frame.
    bool buildable() const { return previewOk_; }
    const std::string& refusal() const { return previewWhy_; }
    // What the next click is for, in a few words: shown by the pointer.
    std::string prompt() const;

    // Makes it. False, with the reason in takeError(), and the tool still open.
    bool finish(Scene& scene, UndoStack& undo);

    std::string takeError() { std::string e; e.swap(error_); return e; }

    // ---- Per frame -------------------------------------------------------------
    void update(const Scene& scene, const Camera& camera, Vec2 mousePx);
    void handleMouseDown(const Scene& scene);
    // Keys as characters: 27 escape, 13 enter, 8 backspace. False when the key
    // means nothing here.
    bool handleKey(int key, Scene& scene, UndoStack& undo);
    void drawOverlay(const Scene& scene, const Camera& camera, Renderer& renderer) const;
    // The panel. `finished` is set when the tool closes, built or cancelled.
    void drawHud(Scene& scene, UndoStack& undo, bool& finished);
    // The prompt beside the pointer, in screen pixels.
    void drawPointerPrompt(Vec2 mouseScreen) const;

    // A sketch in the scene, in the world, as it is picked from.
    struct SceneSketch {
        ObjectId object = kNoObject;
        ElementId uid = 0;
        bool shown = true;
        Sketch sketch;
        std::vector<SketchProfile> regions;
        std::vector<std::vector<Vec3>> lines;   // each entity's drawn line
        std::vector<SketchId> lineOf;
    };
    const std::vector<SceneSketch>& sketches() const { return sketches_; }

private:
    bool active_ = false;
    ProfileBuild build_ = ProfileBuild::Revolve;
    ProfileSlot slot_ = ProfileSlot::Profile;

    OutlinePick profile_;
    AxisPick axis_;
    PathPick path_;
    std::vector<OutlinePick> outlines_;
    Real angle_ = 2.0 * kPi;
    bool reverse_ = false;
    bool ruled_ = false;
    ExtrudeChoice choice_;
    ExtrudeReach reach_;
    std::string error_;

    std::vector<SceneSketch> sketches_;
    void gatherSketches(const Scene& scene);
    const SceneSketch* sketchOf(ObjectId object, ElementId uid) const;

    // What the pointer is over, for the slot being filled.
    enum class Hover { None, Region, Curve, Face, Edge };
    Hover hover_ = Hover::None;
    ObjectId hoverObject_ = kNoObject;
    ElementId hoverSketch_ = 0;
    SketchId hoverItem_ = kNoSketchId;    // region key or entity
    Index hoverElement_ = kInvalid;       // face or edge
    SketchPath hoverPath_;

    // The solid as it stands, in the world, and its mesh for drawing.
    std::string previewKey_;
    bool previewOk_ = false;
    std::string previewWhy_;
    Body preview_;
    RenderMesh previewMesh_;
    void refreshPreview(const Scene& scene);
    std::string pickKey() const;

    // Which part it all belongs to: the one any face or edge is on, else the
    // one the first sketch is in. kNoObject, with the reason, when faces and
    // edges come from more than one.
    ObjectId home(const Scene& scene, std::string* why) const;
    bool usesBody() const;

    // The step as it would go into `home`'s history, with every sketch it
    // names brought in: `chain` is that history, added to. Sketches brought
    // from elsewhere are listed in `brought`.
    bool makeStep(const Scene& scene, ObjectId home, const Mat4& homeModel, std::vector<Feature>& chain,
                  Scene* uids, std::vector<std::pair<ObjectId, ElementId>>& brought,
                  std::string* why) const;
    // Builds that step on a copy of the part and returns the solid it made,
    // in the world -- the preview, and the tool for bodies other than home.
    Body buildTool(const Scene& scene, std::string* why) const;

    void advance();
    bool slotFilled(ProfileSlot s) const;
};

} // namespace tg
