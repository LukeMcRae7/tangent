// Groups and joints: what a group holds and moves, where a joint puts a part,
// and that both survive undo and a file. The joint arithmetic is checked with
// frames set by hand, so it runs without the exact kernel; the frames found
// on real geometry -- and found again after that geometry changes -- need it.
#include "scene/assembly.h"
#include "scene/scene.h"
#include "scene/serialize.h"
#include "app/undo.h"
#include "app/clearance.h"
#include "temp_path.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near(Real a, Real b, Real eps = 1e-6) { return std::fabs(a - b) < eps; }
static bool near(Vec3 a, Vec3 b, Real eps = 1e-6) { return length(a - b) < eps; }
static Vec3 world(const Scene& s, ObjectId id, Vec3 local) {
    const SceneObject* o = s.find(id);
    return o ? transformPoint(o->modelMatrix(), local) : Vec3{};
}

// A frame on a box's face, in its own space: centred, looking out.
static JointSide faceSide(ObjectId id, Vec3 origin, Vec3 z, Vec3 x = {1, 0, 0}) {
    JointSide s;
    s.object = id;
    s.frame.origin = origin;
    s.frame.z = z;
    s.frame.x = x;
    // No name: the frame is what it is, and nothing will look for it again.
    s.foundOn = 0xffffffffu;
    return s;
}

static uint32_t addJoint(Scene& s, Joint j) {
    j.id = s.assembly().nextJoint++;
    if (j.name.empty()) j.name = std::string(jointKindName(j.kind)) + " " + std::to_string(j.id);
    std::string why;
    if (!checkJoint(s, j, &why)) return 0;
    s.assembly().joints.push_back(j);
    solveAssembly(s);
    return j.id;
}

// Keeps a hand-set frame from being looked for again by name.
static void pinFrames(Scene& s) {
    for (Joint& j : s.assembly().joints) {
        if (j.moving.element == kNoId) j.moving.foundOn = s.find(j.moving.object)->geometryVersion;
        if (j.fixed.element == kNoId) j.fixed.foundOn = s.find(j.fixed.object)->geometryVersion;
    }
}

int main() {
    // ---- Groups ------------------------------------------------------------------
    {
        Scene s;
        const ObjectId a = s.addPrimitive(PrimitiveKind::Box);
        const ObjectId b = s.addPrimitive(PrimitiveKind::Box);
        const ObjectId c = s.addPrimitive(PrimitiveKind::Box);
        s.select(a);
        s.select(b, true);
        const GroupId g = groupSelection(s, "Hinge");
        check(g != kNoGroup, "two selected boxes make a group");
        check(s.find(a)->group == g && s.find(b)->group == g && s.find(c)->group == kNoGroup,
              "the two are in it and the third is not");
        check(groupMembers(s, g).size() == 2, "the group holds two");

        // Selecting both members stands for the group; grouping that with c
        // puts the group, not its members, inside the new one.
        s.select(c, true);
        const std::vector<OutlinerNode> nodes = selectionNodes(s);
        check(nodes.size() == 2 && nodes[0].isGroup && nodes[0].group == g,
              "a wholly selected group stands for its members");
        const GroupId outer = groupSelection(s, "Assembly");
        check(s.assembly().group(g)->parent == outer, "the group went inside the new one whole");
        check(s.find(a)->group == g, "its members stayed in it");
        check(s.find(c)->group == outer, "the loose box went in beside it");
        check(groupMembers(s, outer).size() == 3, "the outer group holds all three, through the inner one");
        check(!moveToGroup(s, {true, 0, outer}, g), "a group cannot go inside its own child");

        check(ungroup(s, g), "the inner group comes apart");
        check(s.find(a)->group == outer && s.find(b)->group == outer, "what it held moved up a level");

        // Deleting everything in a group takes the group with it, and undo
        // brings both back.
        UndoStack undo;
        undo.push(ExistenceCommand::forDelete(s, {a, b, c}));
        check(s.assembly().groups.empty(), "a group left with nothing goes");
        undo.undo(s);
        check(s.assembly().group(outer) && s.find(a) && s.find(a)->group == outer,
              "undo brings the group back with its members in it");
        std::printf("[groups] nested, collapsed, ungrouped, pruned and restored\n");
    }

    // ---- A rigid joint lays one face on another -----------------------------------
    {
        Scene s;
        const ObjectId base = s.addPrimitive(PrimitiveKind::Box);              // 20 cube, centred
        PrimitiveSpec lid;
        lid.kind = PrimitiveKind::Box;
        lid.box = {10.0, 10.0, 4.0};
        const ObjectId top = s.addPrimitive(PrimitiveKind::Box, lid, {50, 20, 0});

        Joint j;
        j.kind = JointKind::Rigid;
        j.moving = faceSide(top, {0, 0, -2}, {0, 0, -1});   // the lid's underside
        j.fixed = faceSide(base, {0, 0, 10}, {0, 0, 1});    // the base's top
        const uint32_t id = addJoint(s, j);
        pinFrames(s);
        solveAssembly(s);
        check(id != 0, "the joint is made");
        check(near(s.find(top)->transform.position, {0, 0, 12}), "the lid sits on the base, centred");
        check(near(world(s, top, {5, 5, -2}), {5, 5, 10}), "its underside corner is in the base's top plane");

        // An offset lifts it; an angle turns it about the axis.
        Joint* jj = s.assembly().joint(id);
        jj->offset = 1.5;
        jj->angle = kHalfPi;
        solveAssembly(s);
        check(near(s.find(top)->transform.position, {0, 0, 13.5}), "an offset leaves a gap of that much");
        check(near(world(s, top, {5, 0, 0}), {0, 5, 13.5}), "the angle turns it about the joint's axis");

        // The fixed part moved; the lid follows.
        s.recordMove(base, {7, 0, 0});
        solveAssembly(s);
        check(near(s.find(top)->transform.position, {7, 0, 13.5}), "moving the base carries the lid");
        check(placingJoint(s, top) == id && placingJoint(s, base) == 0,
              "the lid is placed by the joint and the base is not");

        // Deleting the base leaves the lid where the joint had it, as steps in
        // its own history; undo puts the joint and the history back.
        UndoStack undo;
        const size_t steps = s.find(top)->features.size();
        undo.push(ExistenceCommand::forDelete(s, {base}));
        check(s.assembly().joints.empty(), "the joint went with the base");
        check(near(s.find(top)->transform.position, {7, 0, 13.5}), "the lid stayed where it was");
        check(s.find(top)->features.size() > steps, "by steps written into its history");
        undo.undo(s);
        check(s.assembly().joints.size() == 1 && s.find(top)->features.size() == steps,
              "undo brings the joint back and takes the steps out");
        check(near(s.find(top)->transform.position, {7, 0, 13.5}), "and the lid is where the joint puts it");
        std::printf("[rigid] lid at (%.2f, %.2f, %.2f)\n", s.find(top)->transform.position.x,
                    s.find(top)->transform.position.y, s.find(top)->transform.position.z);
    }

    // ---- A revolute turns; a slider slides; limits hold ------------------------------
    {
        Scene s;
        const ObjectId frame = s.addPrimitive(PrimitiveKind::Box);
        PrimitiveSpec doorSpec;
        doorSpec.kind = PrimitiveKind::Box;
        doorSpec.box = {20.0, 2.0, 20.0};
        const ObjectId door = s.addPrimitive(PrimitiveKind::Box, doorSpec, {40, 0, 0});

        // Hinged along the frame's edge at x = 10, y = -10, running up Z. The
        // door's own edge at its x = -10, y = -1.
        Joint j;
        j.kind = JointKind::Revolute;
        j.moving = faceSide(door, {-10, -1, 0}, {0, 0, 1});
        j.fixed = faceSide(frame, {10, -10, 0}, {0, 0, 1});
        j.flip = true;          // both axes up: a hinge, not two faces meeting
        const uint32_t id = addJoint(s, j);
        pinFrames(s);
        solveAssembly(s);
        check(near(world(s, door, {-10, -1, 0}), {10, -10, 0}), "the door's edge is on the hinge line");
        check(near(world(s, door, {10, -1, 0}), {30, -10, 0}), "and it stands out along X, shut");

        Joint* jj = s.assembly().joint(id);
        jj->turn = kHalfPi;
        solveAssembly(s);
        check(near(world(s, door, {10, -1, 0}), {10, 10, 0}), "a quarter turn swings it round to Y");
        check(near(world(s, door, {-10, -1, 5}), {10, -10, 5}), "and the hinge line has not moved");

        jj->limited = true;
        jj->lo = 0.0;
        jj->hi = kPi / 3.0;
        solveAssembly(s);
        const Vec3 tip = world(s, door, {10, -1, 0});
        check(near(std::atan2(tip.y + 10.0, tip.x - 10.0), kPi / 3.0), "a limit stops it at sixty degrees");

        // As a slider along its axis instead: the door lifts off the hinge.
        jj->kind = JointKind::Slider;
        jj->limited = false;
        jj->travel = 6.0;
        jj->slideAxis = SlideAxis::Z;
        solveAssembly(s);
        check(near(world(s, door, {-10, -1, 0}), {10, -10, 6}), "a slider moves it along the axis by its travel");
        jj->slideAxis = SlideAxis::X;
        solveAssembly(s);
        check(near(world(s, door, {-10, -1, 0}), {16, -10, 0}), "or across it, when told to");
        std::printf("[motion] revolute, limits and slider agree\n");
    }

    // ---- A joint moves the whole unit; chains follow; loops are refused ------------
    {
        Scene s;
        const ObjectId a = s.addPrimitive(PrimitiveKind::Box);
        const ObjectId b = s.addPrimitive(PrimitiveKind::Box, {}, {100, 0, 0});
        const ObjectId c = s.addPrimitive(PrimitiveKind::Box, {}, {130, 0, 0});
        const ObjectId d = s.addPrimitive(PrimitiveKind::Box, {}, {0, 100, 0});
        s.select(b);
        s.select(c, true);
        const GroupId g = groupSelection(s, "Pair");

        Joint j;
        j.kind = JointKind::Rigid;
        j.moving = faceSide(b, {0, 0, -10}, {0, 0, -1});
        j.fixed = faceSide(a, {0, 0, 10}, {0, 0, 1});
        const uint32_t ab = addJoint(s, j);
        pinFrames(s);
        solveAssembly(s);
        check(ab != 0, "b joins a");
        check(near(s.find(b)->transform.position, {0, 0, 20}), "b sits on a");
        check(near(s.find(c)->transform.position, {30, 0, 20}), "c, in b's group, came with it and kept its place");
        check(jointUnit(s, *s.assembly().joint(ab)).size() == 2, "the unit is the group");

        // d on top of c: moving a now carries all three.
        Joint k;
        k.kind = JointKind::Rigid;
        k.moving = faceSide(d, {0, 0, -10}, {0, 0, -1});
        k.fixed = faceSide(c, {0, 0, 10}, {0, 0, 1});
        const uint32_t cd = addJoint(s, k);
        pinFrames(s);
        s.recordMove(a, {0, 0, 5});
        solveAssembly(s);
        check(cd != 0, "d joins c");
        check(near(s.find(d)->transform.position, {30, 0, 45}), "a chain follows its root");

        // a on top of d would close the loop.
        Joint loop;
        loop.kind = JointKind::Rigid;
        loop.moving = faceSide(a, {0, 0, -10}, {0, 0, -1});
        loop.fixed = faceSide(d, {0, 0, 10}, {0, 0, 1});
        loop.id = 99;
        std::string why;
        check(!checkJoint(s, loop, &why), "a joint that closes a loop is refused");
        check(why.find("loop") != std::string::npos, "and says it is a loop: " + why);

        // b is placed already; a second joint picked with b first moves the
        // other part instead.
        const ObjectId e = s.addPrimitive(PrimitiveKind::Box, {}, {0, -100, 0});
        Joint twice;
        twice.kind = JointKind::Rigid;
        twice.moving = faceSide(b, {0, 10, 0}, {0, 1, 0});
        twice.fixed = faceSide(e, {0, -10, 0}, {0, -1, 0});
        twice.id = 100;
        check(checkJoint(s, twice, &why) && twice.moving.object == e,
              "picking a part that is already placed moves the other one instead");
        (void)g;
        std::printf("[units] groups move whole, chains follow, loops refused\n");
    }

    // ---- Taking a joint away leaves the part where it is ----------------------------
    {
        Scene s;
        const ObjectId a = s.addPrimitive(PrimitiveKind::Box);
        const ObjectId b = s.addPrimitive(PrimitiveKind::Box, {}, {60, 0, 0});
        Joint j;
        j.kind = JointKind::Revolute;
        j.moving = faceSide(b, {0, 0, -10}, {0, 0, -1});
        j.fixed = faceSide(a, {0, 0, 10}, {0, 0, 1});
        j.turn = 0.3;
        const uint32_t id = addJoint(s, j);
        pinFrames(s);
        solveAssembly(s);
        const Transform was = s.find(b)->transform;
        removeJointKeepingPlace(s, id);
        solveAssembly(s);
        const Transform now = s.find(b)->transform;
        check(s.assembly().joints.empty(), "the joint is gone");
        check(near(now.position, was.position) && near(std::fabs(dot(Vec3{now.rotation.x, now.rotation.y, now.rotation.z},
                                                                     Vec3{was.rotation.x, was.rotation.y, was.rotation.z}) +
                                                                 now.rotation.w * was.rotation.w),
                                                   1.0, 1e-9),
              "the part stayed exactly where the joint had it");
        const Transform history = placementOf(s.find(b)->base, s.find(b)->features);
        check(near(history.position, was.position), "because its history now puts it there");
        std::printf("[release] kept at (%.2f, %.2f, %.2f)\n", now.position.x, now.position.y, now.position.z);
    }

    // ---- Groups and joints survive a file ------------------------------------------
    {
        Scene s;
        const ObjectId a = s.addPrimitive(PrimitiveKind::Box);
        const ObjectId b = s.addPrimitive(PrimitiveKind::Box, {}, {60, 0, 0});
        const ObjectId c = s.addPrimitive(PrimitiveKind::Box, {}, {90, 0, 0});
        s.select(b);
        s.select(c, true);
        groupSelection(s, "Top");
        Joint j;
        j.kind = JointKind::Slider;
        j.moving = faceSide(b, {0, 0, -10}, {0, 0, -1});
        j.fixed = faceSide(a, {0, 0, 10}, {0, 0, 1});
        j.slideAxis = SlideAxis::X;
        j.travel = 4.0;
        j.limited = true;
        j.lo = -5.0;
        j.hi = 5.0;
        j.name = "Rail";
        addJoint(s, j);
        pinFrames(s);
        solveAssembly(s);
        const Vec3 cAt = s.find(c)->transform.position;

        const std::string path = tempPath("assembly.tgt");
        check(saveProject(s, path).ok, "saved");
        Scene back;
        const ProjectResult r = loadProject(back, path);
        check(r.ok, "loaded: " + r.error);
        check(back.assembly().groups.size() == 1 && back.assembly().groups[0].name == "Top", "the group came back");
        check(back.assembly().joints.size() == 1, "the joint came back");
        if (back.assembly().joints.size() == 1) {
            const Joint& jb = back.assembly().joints[0];
            check(jb.name == "Rail" && jb.kind == JointKind::Slider && jb.slideAxis == SlideAxis::X &&
                      jb.limited && near(jb.hi, 5.0) && near(jb.travel, 4.0),
                  "with everything it was");
            // The objects were given new ids; the joint names the new ones.
            const SceneObject* bb = back.find(jb.moving.object);
            check(bb && bb->name == s.find(b)->name, "and names the right part under its new id");
        }
        const SceneObject* cc = nullptr;
        for (const auto& o : back.objects()) if (o->name == s.find(c)->name) cc = o.get();
        check(cc && near(cc->transform.position, cAt), "the group moves together after loading too");
        std::printf("[file] groups and joints round trip\n");
    }

    // ---- Frames found on real geometry, and found again after it changes ------------
    if (brep::available()) {
        Scene s;
        PrimitiveSpec holeSpec;
        holeSpec.kind = PrimitiveKind::Cylinder;
        holeSpec.cylinder.radius = 10.0;
        holeSpec.cylinder.height = 20.0;
        const ObjectId socket = s.addPrimitive(PrimitiveKind::Cylinder, holeSpec);
        PrimitiveSpec pinSpec;
        pinSpec.kind = PrimitiveKind::Cylinder;
        pinSpec.cylinder.radius = 3.0;
        pinSpec.cylinder.height = 20.0;
        const ObjectId pin = s.addPrimitive(PrimitiveKind::Cylinder, pinSpec, {40, 0, 0});

        // The rims: the socket's top, the pin's bottom.
        auto rim = [&](ObjectId id, Real z) -> EdgeId {
            const Body& body = s.find(id)->body;
            std::vector<EdgeId> edges;
            body.allEdges(edges);
            for (EdgeId e : edges) {
                Vec3 c, axis;
                Real r;
                if (body.edgeCircle(e, c, axis, r) && near(c.z, z, 1e-6)) return e;
            }
            return kInvalid;
        };
        const EdgeId top = rim(socket, 10.0), bottom = rim(pin, -10.0);
        check(top != kInvalid && bottom != kInvalid, "found both rims");

        auto sideAt = [&](ObjectId id, EdgeId e) {
            JointSide side;
            side.object = id;
            side.at = JointAt::Edge;
            side.element = s.find(id)->body.edgeName(e);
            std::string why;
            check(jointFrameFrom(s.find(id)->body, JointAt::Edge, e, kInvalid, side.frame, &why), why);
            side.foundOn = s.find(id)->geometryVersion;
            return side;
        };
        Joint j;
        j.kind = JointKind::Revolute;
        j.moving = sideAt(pin, bottom);
        j.fixed = sideAt(socket, top);
        check(near(j.fixed.frame.z, {0, 0, 1}) && near(j.moving.frame.z, {0, 0, -1}),
              "a rim looks out of the flat face beside it");
        addJoint(s, j);
        solveAssembly(s);
        check(near(world(s, pin, {0, 0, -10}), {0, 0, 10}, 1e-6) && near(world(s, pin, {0, 0, 10}), {0, 0, 30}, 1e-6),
              "the pin stands on the socket's rim, on its axis");

        // Its offset sinks it into where a hole would be.
        s.assembly().joints[0].offset = -8.0;
        solveAssembly(s);
        check(near(world(s, pin, {0, 0, -10}), {0, 0, 2}, 1e-6), "an offset of -8 sinks it 8 mm");

        // The socket made taller: its top rim moves up 5, and the pin with it,
        // because the rim is found again by name.
        SceneObject* so = s.find(socket);
        so->spec.cylinder.height = 30.0;
        s.rebuild(socket);
        solveAssembly(s);
        check(s.assembly().joints[0].problem.empty(), "the rim was found again: " + s.assembly().joints[0].problem);
        check(near(world(s, pin, {0, 0, -10}), {0, 0, 7}, 1e-6), "the pin followed the taller socket's rim");

        // A round face is an axis: pin wall to socket wall puts one on the other's axis.
        auto wall = [&](ObjectId id) -> FaceId {
            const Body& body = s.find(id)->body;
            std::vector<FaceId> faces;
            body.allFaces(faces);
            for (FaceId f : faces) if (body.faceKind(f) == SurfaceKind::Cylinder) return f;
            return kInvalid;
        };
        JointFrame w;
        std::string why;
        check(jointFrameFrom(s.find(socket)->body, JointAt::Face, wall(socket), kInvalid, w, &why) &&
                  near(std::fabs(w.z.z), 1.0) && near(w.origin, {0, 0, 0}, 1e-6),
              "a round face's frame is its axis, half way along it");
        std::printf("[kernel] rims and walls give frames, and a rim is found again after an edit\n");
    } else {
        std::printf("[kernel] skipped: no exact kernel in this build\n");
    }

    // ---- A chest lid hinged on two edges -------------------------------------------
    //
    // The way a person makes one: the lid's bottom edge, then the box's top
    // edge, along the back, the front or a side. Which face beside each edge
    // the click lands on decides which way that frame's X points, and the two
    // can come out a quarter turn apart; whichever they are, the lid has to
    // close on the box, and a positive turn has to open it rather than drive
    // it into the box.
    if (brep::available()) {
        auto alongEdge = [](const Body& b, bool alongX, Real at, Real z) {
            std::vector<EdgeId> es;
            b.allEdges(es);
            for (EdgeId e : es) {
                Vec3 p, q;
                b.edgePositions(e, p, q);
                const Real pa = alongX ? p.y : p.x, qa = alongX ? q.y : q.x;
                if (near(pa, at) && near(qa, at) && near(p.z, z) && near(q.z, z)) return e;
            }
            return EdgeId(kInvalid);
        };
        auto facing = [](const Body& b, Vec3 n) {
            for (FaceId f = 0; f < b.faceCount(); ++f)
                if (dot(b.faceNormal(f), n) > 0.99) return f;
            return FaceId(kNoFace);
        };
        auto edgeSide = [](const Scene& s, ObjectId id, EdgeId e, FaceId on) {
            const Body& b = s.find(id)->body;
            JointSide side;
            side.object = id;
            side.at = JointAt::Edge;
            side.element = b.edgeName(e);
            side.onFace = b.faceName(on);
            jointFrameFrom(b, JointAt::Edge, e, on, side.frame);
            side.foundOn = s.find(id)->geometryVersion;
            side.straightEdge = true;
            return side;
        };
        const char* hinges[3] = {"back", "front", "side"};
        int good = 0;
        for (int c = 0; c < 12; ++c) {
            Scene s;
            PrimitiveSpec base;
            base.kind = PrimitiveKind::Box;
            base.box = {40, 30, 20};
            const ObjectId box = s.addPrimitive(PrimitiveKind::Box, base, {0, 0, 10});   // z 0..20
            PrimitiveSpec lidSpec;
            lidSpec.kind = PrimitiveKind::Box;
            lidSpec.box = {40, 30, 5};
            const ObjectId lid = s.addPrimitive(PrimitiveKind::Box, lidSpec, {100, 50, 2.5});
            const Body& lb = s.find(lid)->body;
            const Body& bb = s.find(box)->body;

            const int where = c / 4;
            const Vec3 out = where == 0 ? Vec3{0, -1, 0} : where == 1 ? Vec3{0, 1, 0} : Vec3{1, 0, 0};
            const bool alongX = where != 2;
            const Real at = where == 0 ? -15.0 : where == 1 ? 15.0 : 20.0;
            Joint j;
            j.kind = JointKind::Revolute;
            j.moving = edgeSide(s, lid, alongEdge(lb, alongX, at, -2.5),
                                (c & 1) ? facing(lb, out) : facing(lb, {0, 0, -1}));
            j.fixed = edgeSide(s, box, alongEdge(bb, alongX, at, 10.0),
                               (c & 2) ? facing(bb, out) : facing(bb, {0, 0, 1}));
            std::string why;
            check(checkJoint(s, j, &why), why);
            settleJointLayout(s, j);
            const uint32_t id = addJoint(s, j);
            const std::string what = std::string(hinges[where]) + " hinge, picked beside the " +
                                     ((c & 1) ? "lid's side" : "lid's underside") + " and the " +
                                     ((c & 2) ? "box's side" : "box's top");

            const AABB shut = s.find(lid)->worldBounds();
            const bool closed = near(shut.min, {-20, -15, 20}, 1e-6) && near(shut.max, {20, 15, 25}, 1e-6);
            check(closed, what + ": the lid closes on the box");

            s.assembly().joint(id)->turn = kHalfPi;
            solveAssembly(s);
            const AABB open = s.find(lid)->worldBounds();
            // Standing up past the hinge line, clear of the box.
            const bool upright = near(open.min.z, 20.0, 1e-6) && near(open.max.z - open.min.z, where == 2 ? 40.0 : 30.0, 1e-6) &&
                                 (where == 0 ? near(open.max.y, -15.0, 1e-6)
                                  : where == 1 ? near(open.min.y, 15.0, 1e-6)
                                               : near(open.min.x, 20.0, 1e-6));
            check(upright, what + ": ninety degrees opens it upright behind the hinge");
            if (closed && upright) ++good;
        }
        // A lid modelled where it goes: sitting on the box, its rim a little
        // down over the box's top. It is joined there, not moved off to where
        // two edges would meet, and it still opens up and over the hinge.
        {
            Scene s;
            PrimitiveSpec base;
            base.kind = PrimitiveKind::Box;
            base.box = {20, 20, 12};
            const ObjectId box = s.addPrimitive(PrimitiveKind::Box, base, {0, 0, 6});        // z 0..12
            PrimitiveSpec lidSpec;
            lidSpec.kind = PrimitiveKind::Box;
            lidSpec.box = {21, 20, 8};
            const ObjectId lid = s.addPrimitive(PrimitiveKind::Box, lidSpec, {0.5, 0, 14});   // z 10..18
            const Body& lb = s.find(lid)->body;
            const Body& bb = s.find(box)->body;
            Joint j;
            j.kind = JointKind::Revolute;
            j.moving = edgeSide(s, lid, alongEdge(lb, false, -10.5, -4.0), facing(lb, {0, 0, -1}));
            j.fixed = edgeSide(s, box, alongEdge(bb, false, -10.0, 6.0), facing(bb, {0, 0, 1}));
            std::string why;
            check(checkJoint(s, j, &why), why);
            settleJointLayout(s, j);
            check(j.asBuilt, "parts modelled together are joined where they are");
            const uint32_t id = addJoint(s, j);
            const AABB rest = s.find(lid)->worldBounds();
            check(near(rest.min, {-10, -10, 10}, 1e-6) && near(rest.max, {11, 10, 18}, 1e-6),
                  "the lid has not moved");
            s.assembly().joint(id)->turn = kHalfPi;
            solveAssembly(s);
            const AABB open = s.find(lid)->worldBounds();
            check(open.max.z > 25.0 && open.max.x < 0.0, "ninety degrees opens it up over the hinge, not into the box");
            std::printf("[chest] a lid built on its box is hinged where it sits\n");
        }

        // Saved, the way the motion counts comes back with it.
        {
            Scene s;
            const ObjectId a = s.addPrimitive(PrimitiveKind::Box);
            const ObjectId b = s.addPrimitive(PrimitiveKind::Box, PrimitiveSpec{}, {40, 0, 0});
            Joint j;
            j.kind = JointKind::Revolute;
            j.moving = faceSide(a, {0, 0, -10}, {0, 0, -1});
            j.fixed = faceSide(b, {0, 0, 10}, {0, 0, 1});
            j.reverse = true;
            j.asBuilt = true;
            j.built = {Quat::fromAxisAngle({0, 0, 1}, 0.3), {1, 2, 3}};
            addJoint(s, j);
            const std::string path = tempPath("tangent_joint_reverse.tgp");
            check(saveProject(s, path).ok, "saved");
            Scene back;
            const ProjectResult r = loadProject(back, path);
            check(r.ok && back.assembly().joints.size() == 1 && back.assembly().joints[0].reverse,
                  "a reversed joint is still reversed after a file");
            check(r.ok && back.assembly().joints.size() == 1 && back.assembly().joints[0].asBuilt &&
                      near(back.assembly().joints[0].built.t, {1, 2, 3}) &&
                      near(back.assembly().joints[0].built.q.z, std::sin(0.15)),
                  "and one joined where it was built keeps that");
            std::remove(path.c_str());
        }
        std::printf("[chest] %d of 12 ways of picking a hinge close the lid and open it\n", good);
    }

    // ---- A flat face stays one a joint can stand on, however it was made -------------
    //
    // A box with its corners rounded and then stretched along one axis: the
    // rounds become elliptic, the kernel stretches the whole shape the general
    // way, and every face comes back a spline -- the flat ones included. They
    // are still flat, and a joint has to be able to go on them.
    if (brep::available()) {
        Scene s;
        PrimitiveSpec spec;
        spec.kind = PrimitiveKind::Box;
        spec.box = {20, 20, 20};
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box, spec);
        {
            const Body& b = s.find(id)->body;
            std::vector<EdgeId> es, upright;
            b.allEdges(es);
            for (EdgeId e : es) {
                Vec3 p, q;
                b.edgePositions(e, p, q);
                if (near(p.x, q.x) && near(p.y, q.y)) upright.push_back(e);
            }
            Feature round;
            round.kind = FeatureKind::Bevel;
            round.edges = nameEdges(b, upright, false);
            round.width = 4.0;
            std::string why;
            check(s.addFeature(id, round, &why), "the corners round: " + why);
        }
        std::string why;
        check(s.recordScale(id, {1.09, 1.0, 0.95}, {0, 0, 0}, &why), "stretched: " + why);
        const Body& b = s.find(id)->body;
        FaceId front = kNoFace;
        for (FaceId f = 0; f < b.faceCount(); ++f)
            if (dot(b.faceNormal(f), {0, 1, 0}) > 0.999) front = f;
        check(front != kNoFace, "the front is found");
        JointFrame frame;
        const bool ok = front != kNoFace && jointFrameFrom(b, JointAt::Face, front, front, frame, &why);
        check(ok, "a joint stands on the stretched box's flat front: " + why);
        check(!ok || (near(frame.z, {0, 1, 0}, 1e-6) && near(frame.origin.y, 10.0, 1e-6)),
              "looking straight out of it, on it");
        std::printf("[stretched] a flat face made a spline by a stretch still takes a joint\n");
    }

    // ---- Clearance: the gap between parts, measured ------------------------------
    {
        // Two triangles in parallel planes 0.3 apart, and two that cross.
        const Vec3 a[3] = {{0, 0, 0}, {10, 0, 0}, {0, 10, 0}};
        const Vec3 b[3] = {{1, 1, 0.3}, {9, 1, 0.3}, {1, 9, 0.3}};
        const Vec3 c[3] = {{2, 2, -1}, {2, 2, 1}, {6, 6, 0}};
        Vec3 pa, pb;
        check(near(triangleDistance(a, b, pa, pb), 0.3), "parallel triangles are their spacing apart");
        check(near(triangleDistance(a, c, pa, pb), 0.0), "crossing triangles are no distance apart");
        const Vec3 d[3] = {{12, 0, 0}, {20, 0, 0}, {12, 8, 5}};
        check(near(triangleDistance(a, d, pa, pb), 2.0), "a corner two millimetres past an edge");

        auto boxes = [&](Scene& s, Real xB, Real sizeB = 20.0) {
            const ObjectId p = s.addPrimitive(PrimitiveKind::Box);
            PrimitiveSpec spec;
            spec.kind = PrimitiveKind::Box;
            spec.box = {sizeB, sizeB, sizeB};
            const ObjectId q = s.addPrimitive(PrimitiveKind::Box, spec, {xB, 0, 0});
            return std::pair{p, q};
        };
        auto request = [&](Scene& s, Real required) {
            ClearanceRequest r;
            r.required = required;
            r.deviation = clearanceDeviation(required);
            for (const auto& o : s.objects()) {
                ClearanceBody b;
                b.id = o->id;
                b.body = o->body.detached();
                b.model = o->modelMatrix();
                r.bodies.push_back(b);
            }
            return r;
        };
        {
            Scene s;
            boxes(s, 20.15);
            const ClearanceResult r = checkClearance(request(s, 0.2));
            check(r.ok && r.pairs.size() == 1, "two boxes close together make one pair");
            if (!r.pairs.empty()) {
                check(near(r.pairs[0].gap, 0.15, 1e-9) && !r.pairs[0].overlap && !r.pairs[0].touching,
                      "boxes 0.15 apart have a gap of 0.15, and neither touch nor overlap");
                check(near(r.pairs[0].pa.x, 10.0, 1e-9) && near(r.pairs[0].pb.x, 10.15, 1e-9),
                      "and the gap is between the two facing walls");
            }
            size_t close = 0;
            for (const auto& m : r.marks) close += m.close.size();
            check(close > 0, "the facing walls are marked as too close for 0.2");
            std::printf("[clearance] boxes 0.15 apart: gap %.4f, %zu triangle pairs, %.1f ms\n",
                        r.pairs.empty() ? -1.0 : r.pairs[0].gap, r.trianglePairs, r.ms);
        }
        {
            Scene s;
            boxes(s, 20.5);
            const ClearanceResult r = checkClearance(request(s, 0.2));
            check(r.pairs.size() == 1 && near(r.pairs[0].gap, 0.5, 1e-9) && r.marks.empty(),
                  "boxes 0.5 apart clear 0.2, and nothing is marked");
        }
        {
            Scene s;
            boxes(s, 30.0);
            const ClearanceResult r = checkClearance(request(s, 0.2));
            check(r.pairs.empty(), "boxes 10 apart are clear by more than the limit, and not listed");
        }
        {
            Scene s;
            boxes(s, 20.0);
            const ClearanceResult r = checkClearance(request(s, 0.2));
            check(r.pairs.size() == 1 && r.pairs[0].touching && !r.pairs[0].overlap && near(r.pairs[0].gap, 0.0),
                  "boxes face to face touch and do not overlap");
        }
        {
            Scene s;
            boxes(s, 19.0);
            const ClearanceResult r = checkClearance(request(s, 0.2));
            check(r.pairs.size() == 1 && r.pairs[0].overlap, "boxes a millimetre into each other overlap");
            size_t marked = 0;
            for (const auto& m : r.marks) marked += m.overlap.size();
            check(marked > 0, "and where they run into each other is marked");
        }
        {
            Scene s;
            boxes(s, 0.0, 4.0);
            const ClearanceResult r = checkClearance(request(s, 0.2));
            check(r.pairs.size() == 1 && r.pairs[0].overlap, "a small box wholly inside a big one overlaps it");
        }
        {
            // One box sat on another and turned 8 degrees: their walls cross
            // each other's planes where they meet the face they share, and
            // meet only on it. Touching, and nothing to paint.
            Scene s;
            const ObjectId lower = s.addPrimitive(PrimitiveKind::Box);
            const ObjectId upper = s.addPrimitive(PrimitiveKind::Box, {}, {0, 0, 20});
            s.recordRotate(upper, Quat::fromAxisAngle({0, 0, 1}, 8.0 * kDeg2Rad), {0, 0, 20});
            (void)lower;
            const ClearanceResult r = checkClearance(request(s, 0.2));
            check(r.pairs.size() == 1 && r.pairs[0].touching && !r.pairs[0].overlap,
                  "a box sat on another and turned touches it and does not overlap it");
            check(r.marks.empty(), "and where two parts only touch is not painted");
        }
        // Through a motion: a box slid toward another, 0 to 10 mm in 11 steps,
        // starting 10.4 away -- tightest at the end, 0.4 apart.
        {
            Scene s;
            const auto [fixedBox, movingBox] = boxes(s, 30.4);
            ClearanceRequest r = request(s, 0.2);
            for (ClearanceBody& b : r.bodies) b.moving = b.id == movingBox;
            for (int k = 0; k <= 10; ++k) {
                r.samples.push_back(Rigid{Quat{}, Vec3{-static_cast<Real>(k), 0, 0}});
                r.sampleValues.push_back(k);
            }
            const ClearanceResult res = checkClearance(r);
            check(res.worstSample == 10 && res.pairs.size() == 1 && near(res.pairs[0].gap, 0.4, 1e-9),
                  "through a slide, the tightest is at the end of it, 0.4 apart");
            (void)fixedBox;
        }
        if (brep::available()) {
            // A 4.8 mm pin in a 5 mm hole: a tenth all round.
            Scene s;
            const ObjectId plate = s.addPrimitive(PrimitiveKind::Box);
            Feature hole;
            hole.kind = FeatureKind::Hole;
            hole.uid = s.takeFeatureUid();
            hole.axisPoint = {0, 0, 10};
            hole.axisDir = {0, 0, -1};
            hole.hole.diameter = 5.0;
            hole.hole.through = true;
            check(s.addFeature(plate, hole), "the hole is drilled");
            PrimitiveSpec pinSpec;
            pinSpec.kind = PrimitiveKind::Cylinder;
            pinSpec.cylinder.radius = 2.4;
            pinSpec.cylinder.height = 30.0;
            s.addPrimitive(PrimitiveKind::Cylinder, pinSpec, {0, 0, 0});
            const ClearanceResult r = checkClearance(request(s, 0.2));
            check(r.pairs.size() == 1 && !r.pairs[0].overlap, "the pin in the hole is one pair, not overlapping");
            if (!r.pairs.empty())
                check(std::fabs(r.pairs[0].gap - 0.1) <= r.deviation * 1.5,
                      "and the gap is a tenth, within the mesh tolerance: " + std::to_string(r.pairs[0].gap));
            std::printf("[clearance] pin in hole: gap %.4f (0.1 within %.3f), %zu triangle pairs, %.1f ms\n",
                        r.pairs.empty() ? -1.0 : r.pairs[0].gap, r.deviation, r.trianglePairs, r.ms);
        }
    }

    if (failures) { std::printf("%d check(s) failed\n", failures); return 1; }
    std::printf("all assembly checks passed\n");
    return 0;
}
