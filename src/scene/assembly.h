// Tangent - how parts relate: groups, and the joints that place one part
// against another.
//
// A group is a container in the outliner and nothing more geometric than that:
// it has no placement of its own, because placement lives in each body's
// history. What a group does give is a unit. A joint between two bodies moves
// the whole of whatever the moving body belongs to -- the group that sits
// directly under the lowest group holding both bodies, or the body itself when
// there is none. So a hinge built as a group of two leaves turns one leaf
// inside it, and the same hinge jointed to a box moves as one thing.
//
// A joint names geometry on each side rather than holding a transform: a face,
// an edge or a corner, picked. From each pick comes a frame -- an origin, and a
// Z that is the joint's axis: a flat face's outward normal, a round face's or a
// circular edge's axis, a straight edge's own direction. Joining lays the
// moving part's frame onto the fixed part's with the Z axes opposed, the way
// two faces meet, and then the joint's own motion -- a turn about Z, a slide,
// a slide in the plane -- is applied between them.
//
// The picks are held by name as well as by frame. When the part under a joint
// changes -- the hole moves, the pin gets longer -- the frame is found again on
// the geometry as it now is, and the joint follows. When the name has gone the
// last frame is kept and the joint says so, rather than guessing.
#pragma once

#include "core/math.h"
#include "mesh/element_id.h"
#include "mesh/halfedge.h"
#include "scene/feature.h"

#include <cstdint>
#include <string>
#include <vector>

namespace tg {

using ObjectId = uint32_t;
using GroupId = uint32_t;
inline constexpr GroupId kNoGroup = 0;

struct Group {
    GroupId     id = kNoGroup;
    std::string name;
    GroupId     parent = kNoGroup;     // kNoGroup: at the top of the outliner
    bool        expanded = true;       // in the outliner; saved with the file
};

enum class JointKind : uint32_t {
    Rigid,      // held where it was put
    Revolute,   // turns about the axis
    Slider,     // slides along one direction
    Planar,     // slides anywhere in the plane, and turns in it
};
inline constexpr JointKind kLastJointKind = JointKind::Planar;
const char* jointKindName(JointKind k);

// Which direction a slider moves, in the joint's own frame. The axis is right
// for a pin in a hole or a rail along an edge; across is right for one flat
// face sliding over another, where along the axis would lift it off.
enum class SlideAxis : uint32_t { Z, X, Y };

// A rigid placement: rotate, then translate. What a joint frame is, and what
// moving a part by one comes to.
struct Rigid {
    Quat q{};
    Vec3 t{};

    Vec3 apply(Vec3 p) const { return rotate(q, p) + t; }
    Vec3 applyVector(Vec3 v) const { return rotate(q, v); }
    Rigid operator*(const Rigid& o) const { return {normalize(q * o.q), rotate(q, o.t) + t}; }
    Rigid inverse() const {
        const Quat c = conjugate(q);
        return {c, rotate(c, t) * Real(-1)};
    }
};

// An origin and two axes in a body's own space: Z is the joint's axis, X
// fixes the turn about it, and Y completes a right-handed frame.
struct JointFrame {
    Vec3 origin{};
    Vec3 x{1, 0, 0};
    Vec3 z{0, 0, 1};
    Vec3 y() const { return cross(z, x); }
    Rigid rigid() const;
};

// What a click on a body named, as a joint keeps it.
enum class JointAt : uint32_t { Face, Edge, Vertex };

struct JointSide {
    ObjectId  object = 0;
    JointAt   at = JointAt::Face;
    ElementId element = kNoId;       // the face, edge or vertex, by name
    ElementId onFace = kNoId;        // the face the click landed on
    JointFrame frame;                // as last found, in the body's own space

    // Which body geometry `frame` was found on, so it is looked for again
    // only when that changes. Not saved.
    uint32_t foundOn = 0;
    // A straight edge: Z runs along it and X is off the face it was picked on.
    // Found with the frame, and not saved.
    bool straightEdge = false;
};

struct Joint {
    uint32_t    id = 0;
    std::string name;
    JointKind   kind = JointKind::Rigid;

    // The part that moves, and the one it is joined to. Picked in that order,
    // the way it is said: this goes on that.
    JointSide moving;
    JointSide fixed;

    // Z axes together instead of opposed: a pin whose axis happened to come
    // out pointing the other way.
    bool flip = false;
    Real offset = 0.0;              // along the axis: the gap between two faces
    Real angle = 0.0;               // about the axis, radians: how it is turned

    // Which way the motion counts. The axis a hinge turns about runs whichever
    // way its edge happened to be made, so without this a positive turn opens
    // one lid and drives the next one into its box. Set when the joint is made
    // so that positive moves the part away from what it is joined to.
    bool reverse = false;

    // Joined where the parts already are. A lid modelled sitting on its box
    // is where it belongs already; laying one pick's frame onto the other's
    // would move it somewhere else. `built` is what is left between the two
    // frames with the motion at nothing, so the part stays put and still
    // turns about the fixed pick's axis.
    bool asBuilt = false;
    Rigid built{};

    // The motion. A revolute's turn, radians; a slider's travel, mm; a
    // planar's slide in X and Y and its turn.
    Real turn = 0.0;
    Real travel = 0.0;
    Real slideX = 0.0, slideY = 0.0;
    SlideAxis slideAxis = SlideAxis::Z;

    // How far the motion may go: a lid that opens to 110 degrees, a drawer that
    // pulls out 80 mm. Only the primary motion -- the turn, or the travel.
    bool limited = false;
    Real lo = 0.0, hi = 0.0;

    // Why it is not doing what it says, or empty. Worked out by the solve and
    // not saved.
    std::string problem;

    // The motion between the two frames, fixed side first.
    Rigid motion() const;
    // The one number the joint moves by, in the units the panel shows it in
    // (degrees or millimetres), and whether it has one.
    bool hasMotion() const { return kind == JointKind::Revolute || kind == JointKind::Slider; }
};

struct Assembly {
    std::vector<Group> groups;
    std::vector<Joint> joints;
    GroupId  nextGroup = 1;
    uint32_t nextJoint = 1;

    const Group* group(GroupId id) const;
    Group*       group(GroupId id);
    const Joint* joint(uint32_t id) const;
    Joint*       joint(uint32_t id);
    bool empty() const { return groups.empty() && joints.empty(); }
    void clear() { *this = Assembly{}; }
};

class Scene;
class Body;

// ---- Groups -------------------------------------------------------------------

// Every group from `g` up to the top, `g` first; empty for kNoGroup.
std::vector<GroupId> groupChain(const Scene& scene, GroupId g);

// Whether `g` is `ancestor` or sits somewhere inside it.
bool groupWithin(const Scene& scene, GroupId g, GroupId ancestor);

// The objects in a group, and in every group inside it.
std::vector<ObjectId> groupMembers(const Scene& scene, GroupId g);

// A group with nothing in it, directly or further down.
bool groupEmpty(const Scene& scene, GroupId g);

// Takes out every group that holds nothing. Returns how many went.
size_t pruneEmptyGroups(Scene& scene);

// What a selection comes to as things in the outliner: a group whose every
// member is selected stands for all of them, unless its own parent does too.
struct OutlinerNode {
    bool isGroup = false;
    ObjectId object = 0;
    GroupId  group = kNoGroup;
};
std::vector<OutlinerNode> selectionNodes(const Scene& scene);

// The lowest group holding every node, or kNoGroup for the top.
GroupId commonParent(const Scene& scene, const std::vector<OutlinerNode>& nodes);

// Makes a group of the selection, under whatever group holds all of it. Returns
// the new group, or kNoGroup when there was nothing to group.
GroupId groupSelection(Scene& scene, const std::string& name = "Group");

// Takes a group apart: what was in it moves up to its parent.
bool ungroup(Scene& scene, GroupId g);

// Moves an object, or a group, into `into` (kNoGroup for the top). A group
// cannot go inside itself.
bool moveToGroup(Scene& scene, const OutlinerNode& node, GroupId into);

// ---- Joints -------------------------------------------------------------------

// A frame from a click on a body, in the body's own space. `at` and `element`
// say what was clicked, as handles into `body`; `onFace` is the face the ray
// met, which gives a corner or a straight edge the direction it lacks. False,
// with the reason, for something a joint cannot stand on.
bool jointFrameFrom(const Body& body, JointAt at, Index element, Index onFace, JointFrame& out,
                    std::string* why = nullptr);

// Whether a pick is a straight edge, whose frame's Z runs along it.
bool jointSideStraight(const Body& body, JointAt at, Index element);

// The same, found again by name on the body as it now stands.
bool jointFrameByName(const Body& body, const JointSide& side, JointFrame& out);

// What a joint moves: the objects in the unit the moving body belongs to, as
// seen from the fixed one. Empty when both sides are in the same unit.
std::vector<ObjectId> jointUnit(const Scene& scene, const Joint& joint);

// Where the moving part's unit goes for the joint as it stands, as the move
// that takes it there from where it is now.
bool jointDelta(const Scene& scene, const Joint& joint, Rigid& delta);

// Places every jointed part: each unit a joint moves is put back where its
// history says and then laid onto what it is joined to, joints in turn until
// nothing moves. Cheap -- a few multiplications a joint -- so it runs every
// frame, which is what lets a part follow the one it is joined to while that
// one is being dragged. Returns false when something could not be placed; the
// joint that could not says why.
bool solveAssembly(Scene& scene);

// Whether an object is placed by a joint rather than by its history, and by
// which. 0 when it is free.
uint32_t placingJoint(const Scene& scene, ObjectId id);

// The joint that would come of `candidate`, checked: both sides on bodies, in
// different units, the moving unit not already placed by another joint (the
// sides are swapped when only the other one is free), and no loop. False with
// the reason when it cannot be made.
bool checkJoint(const Scene& scene, Joint& candidate, std::string* why);

// Turns a new joint the way two parts picked like that are most likely meant
// to go together: of its layouts -- the axes facing or together, turned by a
// half or, on an edge, a quarter -- the one that leaves the moving part clear
// of the fixed one and nearest it. Faces put together come out face to face; a
// lid hinged on two edges comes out closed on the box, not inside it, hanging
// off it or upside down on it, whichever faces beside the edges were clicked.
// A revolute or a slider is then counted so that positive moves the part away.
//
// Parts that already touch or overlap are taken to have been built together,
// and are joined where they are (Joint::asBuilt); `placing` can insist on
// either.
enum class JointPlacing { Auto, WhereBuilt, Snap };
void settleJointLayout(const Scene& scene, Joint& joint, JointPlacing placing = JointPlacing::Auto);

// Makes a joint keep the moving part where it is now, turning about the fixed
// pick's axis from there. See Joint::asBuilt.
void jointAsBuilt(const Scene& scene, Joint& joint);

// Takes a joint out, leaving what it placed where it is: the placement it had
// becomes Move and Rotate steps in each body's history.
bool removeJointKeepingPlace(Scene& scene, uint32_t id);

// Everything above as one value, for undo -- and the histories of `chainsOf`,
// for an edit that leaves a part where a joint had it by writing Move and
// Rotate steps into its history.
struct AssemblyState {
    Assembly assembly;
    std::vector<std::pair<ObjectId, GroupId>> membership;
    std::vector<std::pair<ObjectId, std::vector<Feature>>> chains;
};
AssemblyState assemblyState(const Scene& scene, const std::vector<ObjectId>& chainsOf = {});
void restoreAssembly(Scene& scene, const AssemblyState& state);

// The parts that joints to any of `gone` place, which keep their place when
// those go: what forgetObjects writes steps into.
std::vector<ObjectId> placedFrom(const Scene& scene, const std::vector<ObjectId>& gone);

// Takes out the joints that name any of `gone`, leaving what they placed where
// it is. Called while `gone` is still in the scene -- a joint's unit is worked
// out from both of its parts -- and followed, once they have been taken out,
// by pruneEmptyGroups.
void forgetObjects(Scene& scene, const std::vector<ObjectId>& gone);

} // namespace tg
