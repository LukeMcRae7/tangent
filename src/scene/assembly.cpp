#include "scene/assembly.h"

#include "scene/scene.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unordered_map>
#include <unordered_set>

namespace tg {

const char* jointKindName(JointKind k) {
    switch (k) {
        case JointKind::Rigid:    return "Rigid";
        case JointKind::Revolute: return "Revolute";
        case JointKind::Slider:   return "Slider";
        case JointKind::Planar:   return "Planar";
    }
    return "Joint";
}

Rigid JointFrame::rigid() const {
    const Vec3 zz = normalize(z);
    Vec3 xx = normalize(x - zz * dot(x, zz));
    if (length(xx) < 0.5) xx = normalize(perpendicular(zz));
    return {Quat::fromFrame(xx, cross(zz, xx), zz), origin};
}

Rigid Joint::motion() const {
    auto clampMotion = [&](Real v) { return limited && hi >= lo ? std::clamp(v, lo, hi) : v; };
    const Rigid lift{Quat{}, Vec3{0, 0, offset}};
    switch (kind) {
        case JointKind::Rigid:
            return lift * Rigid{Quat::fromAxisAngle({0, 0, 1}, angle), {}};
        case JointKind::Revolute:
            return lift * Rigid{Quat::fromAxisAngle({0, 0, 1}, angle + clampMotion(turn)), {}};
        case JointKind::Slider: {
            const Vec3 dir = slideAxis == SlideAxis::X ? Vec3{1, 0, 0}
                           : slideAxis == SlideAxis::Y ? Vec3{0, 1, 0}
                                                       : Vec3{0, 0, 1};
            return lift * Rigid{Quat{}, dir * clampMotion(travel)} *
                   Rigid{Quat::fromAxisAngle({0, 0, 1}, angle), {}};
        }
        case JointKind::Planar:
            return lift * Rigid{Quat{}, {slideX, slideY, 0}} *
                   Rigid{Quat::fromAxisAngle({0, 0, 1}, angle + turn), {}};
    }
    return {};
}

const Group* Assembly::group(GroupId id) const {
    for (const Group& g : groups) if (g.id == id) return &g;
    return nullptr;
}
Group* Assembly::group(GroupId id) {
    for (Group& g : groups) if (g.id == id) return &g;
    return nullptr;
}
const Joint* Assembly::joint(uint32_t id) const {
    for (const Joint& j : joints) if (j.id == id) return &j;
    return nullptr;
}
Joint* Assembly::joint(uint32_t id) {
    for (Joint& j : joints) if (j.id == id) return &j;
    return nullptr;
}

// ---------------------------------------------------------------------------
// Groups
// ---------------------------------------------------------------------------

std::vector<GroupId> groupChain(const Scene& scene, GroupId g) {
    std::vector<GroupId> out;
    // Bounded, so a parent loop in a damaged file ends rather than hangs.
    for (int guard = 0; g != kNoGroup && guard < 256; ++guard) {
        const Group* grp = scene.assembly().group(g);
        if (!grp) break;
        out.push_back(g);
        g = grp->parent;
    }
    return out;
}

bool groupWithin(const Scene& scene, GroupId g, GroupId ancestor) {
    if (ancestor == kNoGroup) return true;
    const std::vector<GroupId> chain = groupChain(scene, g);
    return std::find(chain.begin(), chain.end(), ancestor) != chain.end();
}

std::vector<ObjectId> groupMembers(const Scene& scene, GroupId g) {
    std::vector<ObjectId> out;
    if (g == kNoGroup) return out;
    for (const auto& o : scene.objects())
        if (o->group != kNoGroup && groupWithin(scene, o->group, g)) out.push_back(o->id);
    return out;
}

bool groupEmpty(const Scene& scene, GroupId g) {
    for (const auto& o : scene.objects())
        if (o->group != kNoGroup && groupWithin(scene, o->group, g)) return false;
    return true;
}

size_t pruneEmptyGroups(Scene& scene) {
    std::vector<Group>& groups = scene.assembly().groups;
    std::vector<GroupId> gone;
    for (const Group& g : groups)
        if (groupEmpty(scene, g.id)) gone.push_back(g.id);
    groups.erase(std::remove_if(groups.begin(), groups.end(),
                                [&](const Group& g) {
                                    return std::find(gone.begin(), gone.end(), g.id) != gone.end();
                                }),
                 groups.end());
    return gone.size();
}

namespace {

std::string uniqueGroupName(const Scene& scene, const std::string& base) {
    auto taken = [&](const std::string& n) {
        for (const Group& g : scene.assembly().groups) if (g.name == n) return true;
        return false;
    };
    if (!taken(base)) return base;
    char buf[160];
    for (int i = 1; i < 10000; ++i) {
        std::snprintf(buf, sizeof buf, "%s.%03d", base.c_str(), i);
        if (!taken(buf)) return buf;
    }
    return base;
}

GroupId parentOf(const Scene& scene, const OutlinerNode& n) {
    if (n.isGroup) {
        const Group* g = scene.assembly().group(n.group);
        return g ? g->parent : kNoGroup;
    }
    const SceneObject* o = scene.find(n.object);
    return o ? o->group : kNoGroup;
}

// The lowest group two chains share, kNoGroup for the top.
GroupId lowestShared(const std::vector<GroupId>& a, const std::vector<GroupId>& b) {
    for (GroupId g : a)
        if (std::find(b.begin(), b.end(), g) != b.end()) return g;
    return kNoGroup;
}

} // namespace

std::vector<OutlinerNode> selectionNodes(const Scene& scene) {
    const std::vector<ObjectId>& sel = scene.selection();
    std::unordered_set<ObjectId> chosen(sel.begin(), sel.end());
    auto whole = [&](GroupId g) {
        const std::vector<ObjectId> m = groupMembers(scene, g);
        return !m.empty() && std::all_of(m.begin(), m.end(), [&](ObjectId id) { return chosen.count(id) > 0; });
    };

    std::vector<OutlinerNode> out;
    std::unordered_set<GroupId> groupsTaken;
    for (ObjectId id : sel) {
        const SceneObject* o = scene.find(id);
        if (!o) continue;
        GroupId highest = kNoGroup;
        for (GroupId g : groupChain(scene, o->group)) {
            if (whole(g)) highest = g;
            else break;
        }
        if (highest != kNoGroup) {
            if (groupsTaken.insert(highest).second) out.push_back({true, 0, highest});
        } else {
            out.push_back({false, id, kNoGroup});
        }
    }
    return out;
}

GroupId commonParent(const Scene& scene, const std::vector<OutlinerNode>& nodes) {
    if (nodes.empty()) return kNoGroup;
    std::vector<GroupId> shared = groupChain(scene, parentOf(scene, nodes.front()));
    for (size_t i = 1; i < nodes.size() && !shared.empty(); ++i) {
        const GroupId g = lowestShared(shared, groupChain(scene, parentOf(scene, nodes[i])));
        shared = groupChain(scene, g);
    }
    return shared.empty() ? kNoGroup : shared.front();
}

GroupId groupSelection(Scene& scene, const std::string& name) {
    const std::vector<OutlinerNode> nodes = selectionNodes(scene);
    if (nodes.empty()) return kNoGroup;
    Assembly& a = scene.assembly();
    Group g;
    g.id = a.nextGroup++;
    g.name = uniqueGroupName(scene, name);
    g.parent = commonParent(scene, nodes);
    a.groups.push_back(g);
    for (const OutlinerNode& n : nodes) moveToGroup(scene, n, g.id);
    return g.id;
}

bool ungroup(Scene& scene, GroupId id) {
    Assembly& a = scene.assembly();
    const Group* g = a.group(id);
    if (!g) return false;
    const GroupId up = g->parent;
    for (const auto& o : scene.objects())
        if (o->group == id) o->group = up;
    for (Group& child : a.groups)
        if (child.parent == id) child.parent = up;
    a.groups.erase(std::remove_if(a.groups.begin(), a.groups.end(),
                                  [&](const Group& x) { return x.id == id; }),
                   a.groups.end());
    return true;
}

bool moveToGroup(Scene& scene, const OutlinerNode& node, GroupId into) {
    Assembly& a = scene.assembly();
    if (into != kNoGroup && !a.group(into)) return false;
    if (node.isGroup) {
        Group* g = a.group(node.group);
        if (!g || groupWithin(scene, into, node.group)) return false;   // not into itself
        g->parent = into;
        return true;
    }
    SceneObject* o = scene.find(node.object);
    if (!o) return false;
    o->group = into;
    return true;
}

// ---------------------------------------------------------------------------
// Frames from geometry
// ---------------------------------------------------------------------------

namespace {

// The direction a flat face is laid out along: its longest straight edge, so a
// rectangle joined to a rectangle comes out square to it. A face with no
// straight edge -- a disc -- takes the world's X laid into its plane.
Vec3 planarX(const Body& body, Index face, Vec3 z) {
    std::vector<EdgeId> edges;
    body.faceEdges(face, edges);
    Real longest = 0.0;
    Vec3 best{};
    for (EdgeId e : edges) {
        if (body.edgeKind(e) != CurveKind::Line) continue;
        Vec3 a, b;
        body.edgePositions(e, a, b);
        const Vec3 d = b - a;
        // A hair's preference for the longer one, so two equal sides do not
        // flip between answers on a rebuild that renumbers them.
        if (length(d) > longest * (1.0 + 1e-6)) {
            longest = length(d);
            best = d;
        }
    }
    for (Vec3 cand : {best, Vec3{1, 0, 0}, Vec3{0, 1, 0}}) {
        const Vec3 x = cand - z * dot(cand, z);
        if (length(x) > 1e-6) return normalize(x);
    }
    return normalize(perpendicular(z));
}

void orthonormal(JointFrame& f) {
    f.z = normalize(f.z);
    Vec3 x = f.x - f.z * dot(f.x, f.z);
    if (length(x) < 1e-6) x = perpendicular(f.z);
    f.x = normalize(x);
}

} // namespace

bool jointFrameFrom(const Body& body, JointAt at, Index element, Index onFace, JointFrame& out,
                    std::string* why) {
    auto refuse = [&](const char* msg) {
        if (why) *why = msg;
        return false;
    };
    JointFrame f;
    switch (at) {
    case JointAt::Face: {
        if (!body.hasFace(element)) return refuse("That face is not there any more");
        const SurfaceKind kind = body.faceKind(element);
        if (kind == SurfaceKind::Plane) {
            f.origin = body.faceCentroid(element);
            f.z = body.faceNormal(element);
            f.x = planarX(body, element, normalize(f.z));
        } else if (kind == SurfaceKind::Cylinder) {
            Vec3 point, axis;
            Real radius = 0.0;
            if (!body.faceCylinder(element, point, axis, radius))
                return refuse("That round face has no axis the kernel will give");
            f.z = normalize(axis);
            // Half way along the face: the centre of mass of a cylinder's wall
            // is on its axis, and that is where two round faces line up.
            const Vec3 c = body.faceCentroid(element);
            f.origin = point + f.z * dot(c - point, f.z);
            f.x = perpendicular(f.z);
        } else {
            return refuse("A joint stands on a flat face, a round face, an edge or a corner");
        }
        break;
    }
    case JointAt::Edge: {
        if (!body.hasEdge(element)) return refuse("That edge is not there any more");
        const CurveKind kind = body.edgeKind(element);
        if (kind == CurveKind::Circle) {
            Vec3 centre, axis;
            Real radius = 0.0;
            if (!body.edgeCircle(element, centre, axis, radius))
                return refuse("That circle has no centre the kernel will give");
            f.origin = centre;
            f.z = normalize(axis);
            // Out of the flat face beside it, when there is one: the rim of a
            // hole on a plate's top looks up, the rim of a pin's end looks down,
            // and those two meet the way a pin goes into a hole.
            FaceId a = kNoFace, b = kNoFace;
            body.edgeFaces(element, a, b);
            for (FaceId face : {a, b}) {
                if (face == kNoFace || body.faceKind(face) != SurfaceKind::Plane) continue;
                if (dot(body.faceNormal(face), f.z) < 0.0) f.z = f.z * Real(-1);
                break;
            }
            f.x = perpendicular(f.z);
        } else if (kind == CurveKind::Line) {
            Vec3 a, b;
            body.edgePositions(element, a, b);
            if (length(b - a) < 1e-9) return refuse("That edge has no length");
            f.origin = (a + b) * 0.5;
            f.z = normalize(b - a);
            // Across the face it was clicked on, so a hinge along an edge keeps
            // the two faces beside it in line.
            f.x = body.hasFace(onFace) ? body.faceNormal(onFace) : perpendicular(f.z);
        } else {
            return refuse("A curved edge that is not a circle has no one axis to join on");
        }
        break;
    }
    case JointAt::Vertex: {
        if (!body.hasVertex(element)) return refuse("That corner is not there any more");
        f.origin = body.vertexPosition(element);
        if (body.hasFace(onFace) && body.faceKind(onFace) == SurfaceKind::Plane) {
            f.z = body.faceNormal(onFace);
            f.x = planarX(body, onFace, normalize(f.z));
        } else {
            f.z = {0, 0, 1};
            f.x = {1, 0, 0};
        }
        break;
    }
    }
    if (length(f.z) < 1e-9) return refuse("That has no direction to join along");
    orthonormal(f);
    out = f;
    return true;
}

bool jointSideStraight(const Body& body, JointAt at, Index element) {
    return at == JointAt::Edge && body.hasEdge(element) && body.edgeKind(element) == CurveKind::Line;
}

bool jointFrameByName(const Body& body, const JointSide& side, JointFrame& out) {
    if (body.empty() || side.element == kNoId) return false;
    const Index onFace = side.onFace != kNoId ? body.findFace(side.onFace) : kInvalid;
    Index element = kInvalid;
    switch (side.at) {
        case JointAt::Face:   element = body.findFace(side.element); break;
        case JointAt::Edge:   element = body.findEdge(side.element); break;
        case JointAt::Vertex: element = body.findVertex(side.element); break;
    }
    if (element == kInvalid) return false;
    return jointFrameFrom(body, side.at, element, onFace, out);
}

// ---------------------------------------------------------------------------
// Units, and the solve
// ---------------------------------------------------------------------------

namespace {

Rigid rigidOf(const Transform& t) { return {normalize(t.rotation), t.position}; }

// The moving object's unit, as seen from the fixed one: the group directly
// under the lowest one holding both, or the object itself.
bool unitNode(const Scene& scene, ObjectId moving, ObjectId fixed, OutlinerNode& out) {
    const SceneObject* m = scene.find(moving);
    const SceneObject* f = scene.find(fixed);
    if (!m || !f || moving == fixed) return false;
    const std::vector<GroupId> mc = groupChain(scene, m->group);
    const GroupId shared = lowestShared(mc, groupChain(scene, f->group));
    // mc runs from the object's own group up; the unit is the last one before
    // the shared group.
    GroupId unit = kNoGroup;
    for (GroupId g : mc) {
        if (g == shared) break;
        unit = g;
    }
    out = unit == kNoGroup ? OutlinerNode{false, moving, kNoGroup} : OutlinerNode{true, 0, unit};
    return true;
}

std::vector<ObjectId> nodeMembers(const Scene& scene, const OutlinerNode& n) {
    return n.isGroup ? groupMembers(scene, n.group) : std::vector<ObjectId>{n.object};
}

uint64_t nodeKey(const OutlinerNode& n) {
    return n.isGroup ? (uint64_t{1} << 40) | n.group : n.object;
}

// A delta measured as how far it moves a point a part's width away: a turn of
// a millionth of a radian on a 200 mm part is a fifth of a micron.
Real deltaSize(const Rigid& d) {
    const Real w = std::clamp(std::fabs(d.q.w), Real(0), Real(1));
    const Real angle = 2.0 * std::acos(w);
    return length(d.t) + angle * 100.0;
}

struct Poses {
    const Scene& scene;
    std::unordered_map<ObjectId, Rigid> pose;
    Rigid get(ObjectId id) const {
        auto it = pose.find(id);
        if (it != pose.end()) return it->second;
        const SceneObject* o = scene.find(id);
        return o ? rigidOf(o->transform) : Rigid{};
    }
};

// Where `joint` takes its moving part, as the move from where it now is.
Rigid deltaFor(const Joint& joint, const Poses& poses) {
    const Rigid flip = joint.flip ? Rigid{} : Rigid{Quat::fromAxisAngle({1, 0, 0}, kPi), {}};
    const Rigid target = poses.get(joint.fixed.object) * joint.fixed.frame.rigid() * joint.motion() * flip;
    const Rigid movingNow = poses.get(joint.moving.object);
    const Rigid movingNew = target * joint.moving.frame.rigid().inverse();
    return movingNew * movingNow.inverse();
}

void refreshFrame(Scene& scene, JointSide& side, bool& lost) {
    const SceneObject* o = scene.find(side.object);
    // A frame with no name behind it is the whole of what the side is.
    if (!o || side.element == kNoId) return;
    if (side.foundOn == o->geometryVersion) return;
    side.foundOn = o->geometryVersion;
    JointFrame f;
    if (jointFrameByName(o->body, side, f)) {
        side.frame = f;
        side.straightEdge = side.at == JointAt::Edge &&
                            jointSideStraight(o->body, side.at, o->body.findEdge(side.element));
    } else {
        lost = true;
    }
}

} // namespace

std::vector<ObjectId> jointUnit(const Scene& scene, const Joint& joint) {
    OutlinerNode n;
    if (!unitNode(scene, joint.moving.object, joint.fixed.object, n)) return {};
    return nodeMembers(scene, n);
}

bool jointDelta(const Scene& scene, const Joint& joint, Rigid& delta) {
    if (!scene.find(joint.moving.object) || !scene.find(joint.fixed.object)) return false;
    Poses poses{scene, {}};
    delta = deltaFor(joint, poses);
    return true;
}

bool solveAssembly(Scene& scene) {
    std::vector<Joint>& joints = scene.assembly().joints;
    if (joints.empty()) return true;

    struct Active {
        Joint* joint;
        std::vector<ObjectId> members;
    };
    std::vector<Active> active;
    std::unordered_map<uint64_t, uint32_t> placedBy;   // unit -> joint
    bool ok = true;

    for (Joint& j : joints) {
        j.problem.clear();
        const SceneObject* m = scene.find(j.moving.object);
        const SceneObject* f = scene.find(j.fixed.object);
        if (!m || !f) {
            j.problem = "One of the parts it joined is not there";
            ok = false;
            continue;
        }
        // The frames are found again when the geometry under them changes --
        // a hole moved, a pin lengthened -- and kept as they were when what
        // they were found on has gone.
        bool lostMoving = false, lostFixed = false;
        refreshFrame(scene, j.moving, lostMoving);
        refreshFrame(scene, j.fixed, lostFixed);
        if (lostMoving || lostFixed) {
            j.problem = std::string("The ") + (lostMoving ? m->name : f->name) +
                        " geometry it was joined at has gone; it holds where it last was";
        }
        OutlinerNode unit;
        if (!unitNode(scene, j.moving.object, j.fixed.object, unit)) {
            j.problem = "Both sides are the same part";
            ok = false;
            continue;
        }
        const uint64_t key = nodeKey(unit);
        if (auto it = placedBy.find(key); it != placedBy.end()) {
            const Joint* first = scene.assembly().joint(it->second);
            j.problem = std::string("That part is already placed by ") + (first ? first->name : "another joint");
            ok = false;
            continue;
        }
        placedBy[key] = j.id;
        active.push_back({&j, nodeMembers(scene, unit)});
    }

    // Every part a joint moves starts from where its own history puts it, so
    // the answer does not depend on where it was last frame.
    Poses poses{scene, {}};
    for (const Active& a : active)
        for (ObjectId id : a.members)
            if (const SceneObject* o = scene.find(id))
                poses.pose[id] = rigidOf(placementOf(o->base, o->features));

    // In turn until nothing moves. A tree of joints settles in as many passes
    // as it is deep; one that is still moving after more passes than there are
    // joints is pulling against itself.
    const int passes = static_cast<int>(active.size()) + 2;
    bool settled = false;
    std::vector<uint8_t> stillMoving(active.size(), 0);
    for (int pass = 0; pass < passes && !settled; ++pass) {
        settled = true;
        for (size_t i = 0; i < active.size(); ++i) {
            const Rigid d = deltaFor(*active[i].joint, poses);
            const bool moved = deltaSize(d) > 1e-9;
            stillMoving[i] = moved;
            if (!moved) continue;
            settled = false;
            for (ObjectId id : active[i].members) poses.pose[id] = d * poses.get(id);
        }
    }
    if (!settled) {
        ok = false;
        for (size_t i = 0; i < active.size(); ++i)
            if (stillMoving[i]) active[i].joint->problem = "It pulls against another joint and cannot be met";
    }

    // Written back only where it changed, so an assembly at rest touches
    // nothing from one frame to the next.
    for (const auto& [id, r] : poses.pose) {
        SceneObject* o = scene.find(id);
        if (!o) continue;
        const Transform& t = o->transform;
        if (length(t.position - r.t) < 1e-12 && std::fabs(t.rotation.x - r.q.x) < 1e-12 &&
            std::fabs(t.rotation.y - r.q.y) < 1e-12 && std::fabs(t.rotation.z - r.q.z) < 1e-12 &&
            std::fabs(t.rotation.w - r.q.w) < 1e-12)
            continue;
        o->transform.position = r.t;
        o->transform.rotation = r.q;
    }
    return ok;
}

uint32_t placingJoint(const Scene& scene, ObjectId id) {
    for (const Joint& j : scene.assembly().joints) {
        if (!scene.find(j.moving.object) || !scene.find(j.fixed.object)) continue;
        const std::vector<ObjectId> unit = jointUnit(scene, j);
        if (std::find(unit.begin(), unit.end(), id) != unit.end()) return j.id;
    }
    return 0;
}

bool checkJoint(const Scene& scene, Joint& c, std::string* why) {
    auto refuse = [&](const std::string& msg) {
        if (why) *why = msg;
        return false;
    };
    const SceneObject* m = scene.find(c.moving.object);
    const SceneObject* f = scene.find(c.fixed.object);
    if (!m || !f) return refuse("Pick something on each of two parts");
    if (m->id == f->id) return refuse("Both picks are on " + m->name + ": a joint is between two parts");
    if (m->body.empty() || f->body.empty()) return refuse("A joint needs a body on each side");

    // Which units are already placed, and by what.
    std::unordered_map<uint64_t, const Joint*> placed;
    for (const Joint& j : scene.assembly().joints) {
        if (j.id == c.id) continue;
        OutlinerNode n;
        if (unitNode(scene, j.moving.object, j.fixed.object, n)) placed.emplace(nodeKey(n), &j);
    }
    auto unitOf = [&](const Joint& j, OutlinerNode& n) { return unitNode(scene, j.moving.object, j.fixed.object, n); };

    OutlinerNode unit;
    if (!unitOf(c, unit)) return refuse("Both picks are on the same part");
    if (placed.count(nodeKey(unit))) {
        // The first pick is the part that moves, but when that one is already
        // placed and the other is not, it is the other that has to move --
        // which is what was meant. Swapping is exact here: a new joint has no
        // offset or turn yet, and its two frames meet either way round.
        Joint swapped = c;
        std::swap(swapped.moving, swapped.fixed);
        OutlinerNode other;
        if (!unitOf(swapped, other) || placed.count(nodeKey(other))) {
            const Joint* by = placed[nodeKey(unit)];
            const SceneObject* mm = scene.find(c.moving.object);
            return refuse((mm ? mm->name : std::string("That part")) + " is already placed by " +
                          (by ? by->name : std::string("another joint")) + ", and so is the other");
        }
        c = swapped;
        unit = other;
    }

    // A loop: the fixed part's place already depends, through other joints,
    // on something this joint would move.
    const std::vector<ObjectId> moving = nodeMembers(scene, unit);
    std::unordered_set<ObjectId> movingSet(moving.begin(), moving.end());
    std::vector<ObjectId> frontier{c.fixed.object};
    std::unordered_set<ObjectId> seen{c.fixed.object};
    while (!frontier.empty()) {
        const ObjectId cur = frontier.back();
        frontier.pop_back();
        for (const Joint& j : scene.assembly().joints) {
            if (j.id == c.id) continue;
            const std::vector<ObjectId> u = jointUnit(scene, j);
            if (std::find(u.begin(), u.end(), cur) == u.end()) continue;
            if (movingSet.count(j.fixed.object)) {
                const SceneObject* fo = scene.find(c.fixed.object);
                return refuse("That would close a loop: " + (fo ? fo->name : std::string("it")) +
                              " is already placed from this side, through " + j.name);
            }
            if (seen.insert(j.fixed.object).second) frontier.push_back(j.fixed.object);
        }
    }
    return true;
}

void settleJointLayout(const Scene& scene, Joint& joint) {
    const SceneObject* fixed = scene.find(joint.fixed.object);
    if (!fixed) return;
    const std::vector<ObjectId> unit = jointUnit(scene, joint);
    if (unit.empty()) return;
    // A hair inside each box, so parts that only touch do not count as
    // overlapping.
    AABB target = fixed->worldBounds();
    if (!target.valid()) return;
    const Vec3 shrink{0.01, 0.01, 0.01};
    target.min += shrink;
    target.max -= shrink;
    const Vec3 targetCentre = (target.min + target.max) * 0.5;

    Real bestOverlap = 1e300, bestGap = 1e300;
    bool bestFlip = joint.flip;
    Real bestAngle = joint.angle;
    for (int k = 0; k < 4; ++k) {
        Joint trial = joint;
        trial.flip = (k & 1) != 0;
        trial.angle = (k & 2) ? kPi : 0.0;
        Rigid delta;
        if (!jointDelta(scene, trial, delta)) return;
        const Mat4 m = translate(delta.t) * toMat4(delta.q);
        AABB moved;
        for (ObjectId id : unit) {
            const SceneObject* o = scene.find(id);
            if (!o) continue;
            const AABB b = o->worldBounds();
            if (!b.valid()) continue;
            for (int c = 0; c < 8; ++c)
                moved.expand(transformPoint(m, {(c & 1) ? b.max.x : b.min.x, (c & 2) ? b.max.y : b.min.y,
                                                (c & 4) ? b.max.z : b.min.z}));
        }
        if (!moved.valid()) return;
        const Vec3 lo = maxv(moved.min, target.min), hi = minv(moved.max, target.max);
        const Real overlap = lo.x < hi.x && lo.y < hi.y && lo.z < hi.z ? (hi.x - lo.x) * (hi.y - lo.y) * (hi.z - lo.z)
                                                                         : 0.0;
        const Real gap = length((moved.min + moved.max) * 0.5 - targetCentre);
        // Clear first, then nearest; ties keep the earlier layout, which is
        // the axes facing and unturned -- faces meeting face to face.
        if (overlap < bestOverlap - 1e-9 || (std::fabs(overlap - bestOverlap) <= 1e-9 && gap < bestGap - 1e-6)) {
            bestOverlap = overlap;
            bestGap = gap;
            bestFlip = trial.flip;
            bestAngle = trial.angle;
        }
    }
    joint.flip = bestFlip;
    joint.angle = bestAngle;
}

bool removeJointKeepingPlace(Scene& scene, uint32_t id) {
    Assembly& a = scene.assembly();
    const Joint* j = a.joint(id);
    if (!j) return false;
    // Where each part is now, as steps in its own history, so taking the
    // joint away leaves it there rather than dropping it back where it was
    // before it was joined.
    for (ObjectId m : jointUnit(scene, *j)) {
        SceneObject* o = scene.find(m);
        if (!o) continue;
        const Transform now = o->transform;
        const Transform was = placementOf(o->base, o->features);
        const Quat turn = normalize(now.rotation * conjugate(was.rotation));
        if (std::fabs(std::fabs(turn.w) - 1.0) > 1e-12) scene.recordRotate(m, turn, was.position);
        const Vec3 by = now.position - was.position;
        if (length(by) > 1e-12) scene.recordMove(m, by);
        scene.place(*o);
    }
    a.joints.erase(std::remove_if(a.joints.begin(), a.joints.end(),
                                  [&](const Joint& x) { return x.id == id; }),
                   a.joints.end());
    solveAssembly(scene);
    return true;
}

AssemblyState assemblyState(const Scene& scene, const std::vector<ObjectId>& chainsOf) {
    AssemblyState s;
    s.assembly = scene.assembly();
    for (const auto& o : scene.objects())
        if (o->group != kNoGroup) s.membership.emplace_back(o->id, o->group);
    for (ObjectId id : chainsOf)
        if (const SceneObject* o = scene.find(id)) s.chains.emplace_back(id, o->features);
    return s;
}

void restoreAssembly(Scene& scene, const AssemblyState& state) {
    // What joints place now: whatever stops being jointed goes back where its
    // history puts it, and nothing else is touched -- a part being shown
    // somewhere by a gesture keeps where it is shown.
    std::unordered_set<ObjectId> placed;
    auto gather = [&] {
        for (const Joint& j : scene.assembly().joints)
            for (ObjectId id : jointUnit(scene, j)) placed.insert(id);
    };
    gather();
    scene.assembly() = state.assembly;
    std::unordered_map<ObjectId, GroupId> in(state.membership.begin(), state.membership.end());
    for (const auto& o : scene.objects()) {
        auto it = in.find(o->id);
        o->group = it == in.end() ? kNoGroup : it->second;
    }
    for (const auto& [id, chain] : state.chains) {
        SceneObject* o = scene.find(id);
        if (!o) continue;
        o->features = chain;
        scene.reevaluate(id);
    }
    // Found again against the geometry as it is now, not as it was saved.
    for (Joint& j : scene.assembly().joints) j.moving.foundOn = j.fixed.foundOn = 0;
    // A part no longer jointed goes back where its history puts it; one now
    // jointed is put where its joint says.
    gather();
    for (ObjectId id : placed)
        if (SceneObject* o = scene.find(id)) scene.place(*o);
    solveAssembly(scene);
}

std::vector<ObjectId> placedFrom(const Scene& scene, const std::vector<ObjectId>& gone) {
    auto named = [&](ObjectId id) { return std::find(gone.begin(), gone.end(), id) != gone.end(); };
    std::vector<ObjectId> out;
    for (const Joint& j : scene.assembly().joints) {
        if (!named(j.fixed.object) || named(j.moving.object)) continue;
        for (ObjectId id : jointUnit(scene, j))
            if (!named(id) && std::find(out.begin(), out.end(), id) == out.end()) out.push_back(id);
    }
    return out;
}

void forgetObjects(Scene& scene, const std::vector<ObjectId>& gone) {
    auto named = [&](ObjectId id) { return std::find(gone.begin(), gone.end(), id) != gone.end(); };
    // What a vanished part placed keeps its place, as steps in its history.
    std::vector<uint32_t> orphaned;
    for (const Joint& j : scene.assembly().joints)
        if (named(j.fixed.object) && !named(j.moving.object)) orphaned.push_back(j.id);
    for (uint32_t id : orphaned) removeJointKeepingPlace(scene, id);
    std::vector<Joint>& joints = scene.assembly().joints;
    joints.erase(std::remove_if(joints.begin(), joints.end(),
                                [&](const Joint& j) { return named(j.moving.object) || named(j.fixed.object); }),
                 joints.end());
}

} // namespace tg
