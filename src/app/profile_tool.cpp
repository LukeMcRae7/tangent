#include "app/profile_tool.h"

#include "core/palette.h"
#include "geom/brep.h"
#include "ui/command_panel.h"
#include "ui/widgets.h"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <tuple>

namespace tg {

const char* profileBuildName(ProfileBuild b) {
    switch (b) {
        case ProfileBuild::Revolve: return "Revolve";
        case ProfileBuild::Sweep:   return "Sweep";
        case ProfileBuild::Loft:    return "Loft";
    }
    return "Revolve";
}

namespace {

constexpr Real kCurvePickPx = 9.0;
constexpr float kEdgePickPx = 10.0f;

Real distanceToSegment(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const Real len = lengthSq(ab);
    const Real t = len > 1e-18 ? std::clamp(dot(p - a, ab) / len, Real(0), Real(1)) : Real(0);
    return length(p - (a + ab * t));
}

// A sketch's plane moved between the world and an object's own frame.
SketchPlane planeInto(const SketchPlane& p, const Mat4& m) {
    SketchPlane out;
    out.origin = transformPoint(m, p.origin);
    out.xAxis = transformVector(m, p.xAxis);
    out.yAxis = transformVector(m, p.yAxis);
    return out;
}

// The slots each build fills, in order.
std::vector<ProfileSlot> slotsOf(ProfileBuild b) {
    switch (b) {
        case ProfileBuild::Revolve: return {ProfileSlot::Profile, ProfileSlot::Axis};
        case ProfileBuild::Sweep:   return {ProfileSlot::Profile, ProfileSlot::Path};
        case ProfileBuild::Loft:    return {ProfileSlot::Outlines};
    }
    return {};
}

const char* slotName(ProfileSlot s) {
    switch (s) {
        case ProfileSlot::Profile:  return "Profile";
        case ProfileSlot::Axis:     return "Axis";
        case ProfileSlot::Path:     return "Path";
        case ProfileSlot::Outlines: return "Outlines";
    }
    return "";
}

Glyph glyphOf(ProfileBuild b) {
    switch (b) {
        case ProfileBuild::Revolve: return Glyph::Revolve;
        case ProfileBuild::Sweep:   return Glyph::Sweep;
        case ProfileBuild::Loft:    return Glyph::Loft;
    }
    return Glyph::Revolve;
}

bool exactBody(const SceneObject* o) { return o && !o->body.empty() && !o->body.isMesh(); }

} // namespace

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void ProfileTool::start(ProfileBuild build, const Scene& scene) {
    *this = ProfileTool{};
    build_ = build;
    active_ = true;
    gatherSketches(scene);

    // A selected sketch is what to build from: its filled regions are the
    // profile, or -- the first of them -- a loft's first outline.
    if (const Scene::SketchRef sk = scene.selectedSketch(); sk.valid()) {
        if (const SceneSketch* s = sketchOf(sk.object, sk.uid)) {
            const std::vector<SketchId> filled = sketchFilledProfiles(s->sketch, s->regions);
            if (!filled.empty()) {
                if (build_ == ProfileBuild::Loft) outlines_.push_back({sk.object, sk.uid, {filled.front()}, kNoFace});
                else                              profile_ = {sk.object, sk.uid, filled, kNoFace};
            }
        }
        error_.clear();
        advance();
        refreshPreview(scene);
        return;
    }

    // Whatever is selected, taken as what it can be: a flat face is a profile
    // (or the first outline), edges are a path, one straight edge an axis.
    const ObjectId id = scene.contextObject();
    const SceneObject* o = scene.find(id);
    if (exactBody(o)) {
        for (FaceId f : scene.selectedFaces(id)) {
            if (o->body.faceKind(f) != SurfaceKind::Plane) continue;
            if (build_ == ProfileBuild::Loft) pickFace(scene, id, f);
            else if (profile_.empty()) pickFace(scene, id, f);
        }
        const std::vector<EdgeId> es = scene.selectedEdges(id);
        if (build_ == ProfileBuild::Sweep && !es.empty()) {
            path_ = PathPick{id, 0, {}, es};
        } else if (build_ == ProfileBuild::Revolve && es.size() == 1 && o->body.edgeKind(es.front()) == CurveKind::Line) {
            axis_.kind = AxisPick::Kind::Edge;
            axis_.object = id;
            axis_.edge = es.front();
        }
    }
    error_.clear();
    advance();
    refreshPreview(scene);
}

void ProfileTool::startWith(ProfileBuild build, const Scene& scene, ObjectId object, ElementId sketchUid,
                            const std::vector<SketchId>& keys) {
    *this = ProfileTool{};
    build_ = build;
    active_ = true;
    gatherSketches(scene);
    if (build_ == ProfileBuild::Loft) {
        // A loft runs through one region of each outline: the first picked.
        if (!keys.empty()) outlines_.push_back({object, sketchUid, {keys.front()}, kNoFace});
    } else {
        profile_ = {object, sketchUid, keys, kNoFace};
    }
    advance();
    refreshPreview(scene);
}

void ProfileTool::cancel() { *this = ProfileTool{}; }

void ProfileTool::gatherSketches(const Scene& scene) {
    sketches_.clear();
    for (const auto& o : scene.objects()) {
        if (!o->visible) continue;
        const Mat4 model = o->modelMatrix();
        for (const Feature& f : o->features) {
            if (f.kind != FeatureKind::Sketch || !f.enabled || f.errored) continue;
            SceneSketch s;
            s.object = o->id;
            s.uid = f.uid;
            s.shown = f.sketchShown;
            s.sketch = f.sketch;
            s.sketch.plane = planeInto(f.sketch.plane, model);
            s.regions = sketchProfiles(s.sketch);
            for (const SketchEntity& e : s.sketch.entities) {
                if (e.construction) continue;
                std::vector<Vec3> line;
                for (Vec2 p : sketchEntityPoints(s.sketch, e, 24)) line.push_back(s.sketch.plane.toWorld(p));
                s.lines.push_back(std::move(line));
                s.lineOf.push_back(e.id);
            }
            sketches_.push_back(std::move(s));
        }
    }
}

const ProfileTool::SceneSketch* ProfileTool::sketchOf(ObjectId object, ElementId uid) const {
    for (const SceneSketch& s : sketches_)
        if (s.object == object && s.uid == uid) return &s;
    return nullptr;
}

bool ProfileTool::slotFilled(ProfileSlot s) const {
    switch (s) {
        case ProfileSlot::Profile:  return !profile_.empty();
        case ProfileSlot::Axis:     return axis_.kind != AxisPick::Kind::None;
        case ProfileSlot::Path:     return !path_.empty();
        case ProfileSlot::Outlines: return outlines_.size() >= 2;
    }
    return false;
}

void ProfileTool::advance() {
    for (ProfileSlot s : slotsOf(build_))
        if (!slotFilled(s)) { slot_ = s; return; }
    // Everything is filled: stay on the last, where one more click changes it.
    const std::vector<ProfileSlot> all = slotsOf(build_);
    if (std::find(all.begin(), all.end(), slot_) == all.end()) slot_ = all.back();
}

// ---------------------------------------------------------------------------
// Picking
// ---------------------------------------------------------------------------

bool ProfileTool::pickRegion(const Scene& scene, ObjectId object, ElementId sketchUid, SketchId key) {
    error_.clear();
    const SceneSketch* s = sketchOf(object, sketchUid);
    const SketchProfile* r = nullptr;
    if (s) for (const SketchProfile& p : s->regions) if (p.key == key) r = &p;
    if (!r) {
        error_ = "That is not a closed region of a sketch";
        return false;
    }
    if (slot_ == ProfileSlot::Outlines) {
        auto it = std::find_if(outlines_.begin(), outlines_.end(), [&](const OutlinePick& o) {
            return o.object == object && o.sketchUid == sketchUid && o.keys == std::vector<SketchId>{key};
        });
        if (it != outlines_.end()) {
            outlines_.erase(it);
        } else if (!r->holes.empty()) {
            error_ = "A loft goes through outlines: a region with a hole in it cannot be one";
            return false;
        } else {
            outlines_.push_back({object, sketchUid, {key}, kNoFace});
        }
    } else if (slot_ == ProfileSlot::Profile) {
        // More regions of the same sketch go in with it; one of another
        // sketch starts again from that one.
        if (profile_.object == object && profile_.sketchUid == sketchUid) {
            auto it = std::find(profile_.keys.begin(), profile_.keys.end(), key);
            if (it != profile_.keys.end()) profile_.keys.erase(it);
            else profile_.keys.push_back(key);
            if (profile_.keys.empty()) profile_ = OutlinePick{};
        } else {
            profile_ = {object, sketchUid, {key}, kNoFace};
        }
        if (!profile_.empty() && profile_.keys.size() == 1) advance();
    } else {
        return false;
    }
    refreshPreview(scene);
    return true;
}

bool ProfileTool::pickFace(const Scene& scene, ObjectId object, FaceId face) {
    error_.clear();
    const SceneObject* o = scene.find(object);
    if (!o || o->body.empty()) return false;
    if (o->body.isMesh()) {
        error_ = "A mesh's faces cannot be built from: Modify > Convert to Solid first";
        return false;
    }
    if (o->body.faceKind(face) != SurfaceKind::Plane) {
        error_ = "Only a flat face can be an outline";
        return false;
    }
    if (slot_ == ProfileSlot::Outlines) {
        auto it = std::find_if(outlines_.begin(), outlines_.end(),
                               [&](const OutlinePick& p) { return p.object == object && !p.fromSketch() && p.face == face; });
        if (it != outlines_.end()) outlines_.erase(it);
        else outlines_.push_back({object, 0, {}, face});
    } else if (slot_ == ProfileSlot::Profile) {
        profile_ = {object, 0, {}, face};
        advance();
    } else {
        return false;
    }
    refreshPreview(scene);
    return true;
}

bool ProfileTool::pickCurve(const Scene& scene, ObjectId object, ElementId sketchUid, SketchId entity) {
    error_.clear();
    const SceneSketch* s = sketchOf(object, sketchUid);
    const SketchEntity* e = s ? s->sketch.entity(entity) : nullptr;
    if (!e) return false;
    if (slot_ == ProfileSlot::Axis) {
        if (e->curve != SketchCurve::Line) {
            error_ = "An axis is a straight line: pick a line, not a curve";
            return false;
        }
        axis_ = AxisPick{};
        axis_.kind = AxisPick::Kind::SketchLine;
        axis_.object = object;
        axis_.sketchUid = sketchUid;
        axis_.entity = entity;
        advance();
    } else if (slot_ == ProfileSlot::Path) {
        SketchPath p;
        std::string why;
        if (!sketchPathThrough(s->sketch, entity, p, &why)) {
            error_ = "That is not a path: " + why;
            return false;
        }
        // The same path clicked again is taken out.
        if (path_.fromSketch() && path_.object == object && path_.sketchUid == sketchUid &&
            std::find(path_.entities.begin(), path_.entities.end(), entity) != path_.entities.end())
            path_ = PathPick{};
        else
            path_ = PathPick{object, sketchUid, p.entities, {}};
        advance();
    } else {
        return false;
    }
    refreshPreview(scene);
    return true;
}

bool ProfileTool::pickEdge(const Scene& scene, ObjectId object, EdgeId edge) {
    error_.clear();
    const SceneObject* o = scene.find(object);
    if (!exactBody(o)) {
        error_ = "A mesh's edges cannot be built from: Modify > Convert to Solid first";
        return false;
    }
    if (slot_ == ProfileSlot::Axis) {
        if (o->body.edgeKind(edge) != CurveKind::Line) {
            error_ = "An axis is a straight line: pick a straight edge";
            return false;
        }
        axis_ = AxisPick{};
        axis_.kind = AxisPick::Kind::Edge;
        axis_.object = object;
        axis_.edge = edge;
        advance();
    } else if (slot_ == ProfileSlot::Path) {
        // Edges of one body make a path together: each click adds one, or
        // takes it back out.
        if (path_.fromSketch() || path_.object != object) path_ = PathPick{object, 0, {}, {}};
        auto it = std::find(path_.edges.begin(), path_.edges.end(), edge);
        if (it != path_.edges.end()) path_.edges.erase(it);
        else path_.edges.push_back(edge);
        if (path_.edges.empty()) path_ = PathPick{};
        advance();
    } else {
        return false;
    }
    refreshPreview(scene);
    return true;
}

void ProfileTool::setWorldAxis(int axis) {
    axis_ = AxisPick{};
    axis_.kind = AxisPick::Kind::World;
    axis_.world = std::clamp(axis, 0, 2);
    previewKey_.clear();
    advance();
}

std::string ProfileTool::prompt() const {
    switch (slot_) {
        case ProfileSlot::Profile:
            return profile_.empty() ? "Profile: click a region of a sketch, or a flat face"
                                    : "Profile: click to change it, or another region to add";
        case ProfileSlot::Axis:
            return "Axis: click a sketch line or a straight edge, or pick X, Y or Z";
        case ProfileSlot::Path:
            return path_.empty()        ? "Path: click a sketch curve, or edges of the part"
                 : path_.fromSketch()   ? "Path: click another curve or edge to change it"
                                        : "Path: click more edges to extend it, or one to take it out";
        case ProfileSlot::Outlines:
            return outlines_.empty() ? "Outline 1: click a region of a sketch, or a flat face"
                                     : "Outline " + std::to_string(outlines_.size() + 1) +
                                           ": click the next, in order";
    }
    return "";
}

// ---------------------------------------------------------------------------
// Where it goes, and what it makes
// ---------------------------------------------------------------------------

bool ProfileTool::usesBody() const {
    if (!profile_.empty() && !profile_.fromSketch() && build_ != ProfileBuild::Loft) return true;
    if (build_ == ProfileBuild::Revolve && axis_.kind == AxisPick::Kind::Edge) return true;
    if (build_ == ProfileBuild::Sweep && !path_.empty() && !path_.fromSketch()) return true;
    if (build_ == ProfileBuild::Loft)
        for (const OutlinePick& o : outlines_) if (!o.fromSketch()) return true;
    return false;
}

ObjectId ProfileTool::home(const Scene& scene, std::string* why) const {
    std::vector<ObjectId> bodies;
    auto body = [&](ObjectId id) {
        if (std::find(bodies.begin(), bodies.end(), id) == bodies.end()) bodies.push_back(id);
    };
    ObjectId firstSketch = kNoObject;
    if (build_ == ProfileBuild::Loft) {
        for (const OutlinePick& o : outlines_) {
            if (!o.fromSketch()) body(o.object);
            else if (firstSketch == kNoObject) firstSketch = o.object;
        }
    } else {
        if (!profile_.empty()) {
            if (profile_.fromSketch()) firstSketch = profile_.object;
            else body(profile_.object);
        }
        if (build_ == ProfileBuild::Revolve && axis_.kind == AxisPick::Kind::Edge) body(axis_.object);
        if (build_ == ProfileBuild::Sweep && !path_.empty() && !path_.fromSketch()) body(path_.object);
    }
    if (bodies.size() > 1) {
        if (why) *why = "The faces and edges have to come from one part";
        return kNoObject;
    }
    const ObjectId id = bodies.empty() ? firstSketch : bodies.front();
    if (!scene.find(id)) {
        if (why) *why = "There is nothing picked yet";
        return kNoObject;
    }
    return id;
}

bool ProfileTool::makeStep(const Scene& scene, ObjectId homeId, const Mat4& homeModel,
                           std::vector<Feature>& chain, Scene* uids,
                           std::vector<std::pair<ObjectId, ElementId>>& brought, std::string* why) const {
    const SceneObject* homeObj = scene.find(homeId);
    const Body* homeBody = homeObj ? &homeObj->body : nullptr;
    const Mat4 toLocal = inverse(homeModel);
    ElementId nextUid = 1u << 30;
    // Each sketch from elsewhere is brought in once, however many times it is named.
    std::vector<std::tuple<ObjectId, ElementId, ElementId>> copies;

    // A sketch named where it is, or brought into this history.
    auto sketchIn = [&](ObjectId object, ElementId uid) -> ElementId {
        if (object == homeId) {
            for (Feature& f : chain)
                if (f.kind == FeatureKind::Sketch && f.uid == uid) f.sketchShown = false;
            return uid;
        }
        for (const auto& [from, fromUid, here] : copies)
            if (from == object && fromUid == uid) return here;
        const SceneSketch* s = sketchOf(object, uid);
        if (!s) return 0;
        Feature k;
        k.kind = FeatureKind::Sketch;
        k.uid = uids ? uids->takeFeatureUid() : nextUid++;
        k.sketch = s->sketch;
        // Brought into another part, it can no longer follow the edges of the
        // one it came from: what it projected stays where it is.
        for (SketchEntity& e : k.sketch.entities) e.source = 0;
        k.sketch.plane = planeInto(s->sketch.plane, toLocal);
        k.sketchShown = false;
        chain.push_back(std::move(k));
        brought.push_back({object, uid});
        copies.push_back({object, uid, chain.back().uid});
        return chain.back().uid;
    };
    auto needBody = [&](const char* what) {
        if (homeBody && !homeBody->empty() && !homeBody->isMesh()) return true;
        if (why) *why = what;
        return false;
    };

    Feature f;
    f.kind = build_ == ProfileBuild::Revolve ? FeatureKind::RevolveProfile
           : build_ == ProfileBuild::Sweep   ? FeatureKind::SweepProfile
                                             : FeatureKind::LoftProfile;
    f.extrudeOp = choice_.op == ExtrudeOp::NewBody ? ExtrudeOp::Join : choice_.op;
    f.revolveAngle = angle_;
    f.revolveReverse = reverse_;
    f.loftRuled = ruled_;

    auto setProfile = [&](const OutlinePick& p) {
        if (p.fromSketch()) {
            f.sketchUid = sketchIn(p.object, p.sketchUid);
            f.profileKeys = p.keys;
            std::sort(f.profileKeys.begin(), f.profileKeys.end());
            return f.sketchUid != 0;
        }
        if (!needBody("The face it builds from is on another part")) return false;
        f.sketchUid = 0;
        f.faces = nameFaces(*homeBody, {p.face});
        return true;
    };

    if (build_ == ProfileBuild::Loft) {
        if (outlines_.size() < 2) {
            if (why) *why = "A loft needs at least two outlines";
            return false;
        }
        if (!setProfile(outlines_.front())) return false;
        for (size_t i = 1; i < outlines_.size(); ++i) {
            const OutlinePick& o = outlines_[i];
            if (o.fromSketch()) {
                f.loftSketchUids.push_back(sketchIn(o.object, o.sketchUid));
                f.loftKeys.push_back(o.keys.front());
                f.loftFaceNames.push_back(kNoId);
            } else {
                if (!needBody("A face it lofts through is on another part")) return false;
                f.loftSketchUids.push_back(0);
                f.loftKeys.push_back(kNoSketchId);
                f.loftFaceNames.push_back(homeBody->faceName(o.face));
            }
        }
    } else {
        if (profile_.empty()) {
            if (why) *why = "Pick a profile";
            return false;
        }
        if (!setProfile(profile_)) return false;
    }

    if (build_ == ProfileBuild::Revolve) {
        switch (axis_.kind) {
        case AxisPick::Kind::None:
            if (why) *why = "Pick an axis";
            return false;
        case AxisPick::Kind::World: {
            Vec3 d{0, 0, 0};
            (&d.x)[axis_.world] = 1.0;
            f.revolveAxis3D = true;
            f.axisPoint = transformPoint(toLocal, Vec3{0, 0, 0});
            f.axisDir = transformVector(toLocal, d);
            break;
        }
        case AxisPick::Kind::SketchLine: {
            const SceneSketch* s = sketchOf(axis_.object, axis_.sketchUid);
            const SketchEntity* e = s ? s->sketch.entity(axis_.entity) : nullptr;
            const SketchPoint* a = e ? s->sketch.point(e->a) : nullptr;
            const SketchPoint* b = e ? s->sketch.point(e->b) : nullptr;
            if (!a || !b) {
                if (why) *why = "The line it turns about is gone";
                return false;
            }
            if (profile_.fromSketch() && axis_.object == profile_.object && axis_.sketchUid == profile_.sketchUid) {
                // A line of the profile's own sketch: kept in the sketch's
                // terms, so moving the line in the sketch moves the axis.
                f.revolveAxisAt = a->at;
                f.revolveAxisDir = b->at - a->at;
            } else {
                f.revolveAxis3D = true;
                f.axisPoint = transformPoint(toLocal, s->sketch.plane.toWorld(a->at));
                f.axisDir = transformVector(toLocal, s->sketch.plane.toWorld(b->at) - s->sketch.plane.toWorld(a->at));
            }
            break;
        }
        case AxisPick::Kind::Edge:
            if (!needBody("The edge it turns about is on another part")) return false;
            f.edges = nameEdges(*homeBody, {axis_.edge}, false);
            break;
        }
    }

    if (build_ == ProfileBuild::Sweep) {
        if (path_.empty()) {
            if (why) *why = "Pick a path";
            return false;
        }
        if (path_.fromSketch()) {
            f.pathSketchUid = sketchIn(path_.object, path_.sketchUid);
            f.pathEntities = path_.entities;
        } else {
            if (!needBody("The edges of the path are on another part")) return false;
            f.pathSketchUid = 0;
            f.edges = nameEdges(*homeBody, path_.edges, true);
        }
    }

    f.uid = uids ? uids->takeFeatureUid() : nextUid++;
    chain.push_back(std::move(f));
    return true;
}

Body ProfileTool::buildTool(const Scene& scene, std::string* why) const {
    const ObjectId h = home(scene, why);
    const SceneObject* o = scene.find(h);
    if (!o) return Body{};
    // The step exactly as Finish would write it into the part's history --
    // the part's own sketches as they are, the others brought into its frame
    // -- and then the solid built from the same sources the step names, on
    // the part's body as it stands. The solid alone, not the part with it
    // joined on: that is what is shown, and what other bodies are given.
    std::vector<Feature> chain;
    std::vector<std::pair<ObjectId, ElementId>> brought;
    for (const Feature& f : o->features) if (f.kind == FeatureKind::Sketch) chain.push_back(f);
    if (!makeStep(scene, h, o->modelMatrix(), chain, nullptr, brought, why)) return Body{};
    const Feature step = chain.back();

    std::deque<std::vector<SketchProfile>> held;
    auto sketchById = [&](ElementId uid) -> const Sketch* {
        for (const Feature& f : chain) if (f.kind == FeatureKind::Sketch && f.uid == uid) return &f.sketch;
        return nullptr;
    };
    auto outlineOf = [&](ElementId uid, const std::vector<SketchId>& keys, bool fromFaces, FaceId face,
                         brep::OutlineSource& out) {
        if (fromFaces) {
            out.body = &o->body.brep();
            out.faces = {face};
            return true;
        }
        const Sketch* s = sketchById(uid);
        if (!s) return false;
        held.push_back(sketchProfiles(*s));
        out.sketch = s;
        out.regions = &held.back();
        out.keys = keys;
        return true;
    };

    std::string w;
    BrepRef made;
    const bool exact = exactBody(o);
    if (build_ == ProfileBuild::Loft) {
        std::vector<brep::OutlineSource> outlines(outlines_.size());
        for (size_t i = 0; i < outlines_.size(); ++i) {
            const OutlinePick& p = outlines_[i];
            const ElementId uid = i == 0 ? step.sketchUid : step.loftSketchUids[i - 1];
            if (!p.fromSketch() && !exact) return Body{};
            if (!outlineOf(uid, p.keys, !p.fromSketch(), p.face, outlines[i])) return Body{};
        }
        made = brep::loftOutlines(outlines, ruled_, 7101, &w);
    } else {
        brep::OutlineSource outline;
        if (!profile_.fromSketch() && !exact) return Body{};
        if (!outlineOf(step.sketchUid, step.profileKeys, !profile_.fromSketch(), profile_.face, outline))
            return Body{};
        if (build_ == ProfileBuild::Revolve) {
            Vec3 at, dir;
            if (!step.edges.empty()) {
                Vec3 b;
                o->body.edgePositions(axis_.edge, at, b);
                dir = b - at;
            } else if (step.revolveAxis3D) {
                at = step.axisPoint;
                dir = step.axisDir;
            } else {
                const Sketch* s = sketchById(step.sketchUid);
                at = s->plane.toWorld(step.revolveAxisAt);
                dir = s->plane.toWorld(step.revolveAxisAt + step.revolveAxisDir) - at;
            }
            if (reverse_) dir = -dir;
            made = brep::revolveOutline(outline, at, dir, angle_, 7101, &w);
        } else {
            brep::PathSource path;
            if (path_.fromSketch()) {
                path.sketch = sketchById(step.pathSketchUid);
                path.entities = step.pathEntities;
            } else {
                if (!exact) return Body{};
                path.body = &o->body.brep();
                path.edges = path_.edges;
            }
            made = brep::sweepOutline(outline, path, 7101, &w);
        }
    }
    if (!made) {
        if (why) *why = w.empty() ? "It does not build" : w;
        return Body{};
    }
    Body tool(std::move(made));
    if (!tool.transform(o->modelMatrix())) return Body{};
    return tool;
}

std::string ProfileTool::pickKey() const {
    std::string k = std::to_string(static_cast<int>(build_));
    auto outline = [&](const OutlinePick& p) {
        k += "|" + std::to_string(p.object) + ":" + std::to_string(p.sketchUid) + ":" + std::to_string(p.face);
        for (SketchId key : p.keys) k += "," + std::to_string(key);
    };
    outline(profile_);
    for (const OutlinePick& p : outlines_) outline(p);
    k += "|a" + std::to_string(static_cast<int>(axis_.kind)) + ":" + std::to_string(axis_.world) + ":" +
         std::to_string(axis_.object) + ":" + std::to_string(axis_.sketchUid) + ":" +
         std::to_string(axis_.entity) + ":" + std::to_string(axis_.edge);
    k += "|p" + std::to_string(path_.object) + ":" + std::to_string(path_.sketchUid);
    for (SketchId e : path_.entities) k += "," + std::to_string(e);
    for (EdgeId e : path_.edges) k += ";" + std::to_string(e);
    char b[64];
    std::snprintf(b, sizeof b, "|%.9g|%d%d", angle_, reverse_ ? 1 : 0, ruled_ ? 1 : 0);
    return k + b;
}

void ProfileTool::refreshPreview(const Scene& scene) {
    const std::string key = pickKey();
    if (key == previewKey_) return;
    previewKey_ = key;
    previewOk_ = false;
    previewWhy_.clear();
    preview_ = Body{};
    previewMesh_.clear();
    for (ProfileSlot s : slotsOf(build_))
        if (!slotFilled(s)) {
            previewWhy_ = prompt();
            return;
        }
    std::string why;
    preview_ = buildTool(scene, &why);
    previewOk_ = !preview_.empty();
    if (previewOk_) preview_.tessellate(previewMesh_);
    else previewWhy_ = why.empty() ? "It does not build" : why;
}

// ---------------------------------------------------------------------------
// Finishing
// ---------------------------------------------------------------------------

bool ProfileTool::finish(Scene& scene, UndoStack& undo) {
    refreshPreview(scene);
    if (!previewOk_) {
        error_ = previewWhy_.empty() ? "Nothing to build yet" : previewWhy_;
        return false;
    }
    std::string why;
    const ObjectId owner = home(scene, &why);
    const SceneObject* ownerObj = scene.find(owner);
    if (!ownerObj) {
        error_ = why;
        return false;
    }
    choice_.follow(1.0, !ownerObj->body.empty());
    reach_.refresh(scene, preview_, owner, choice_.op, 1.0, previewKey_);
    ExtrudeOp op = choice_.op;
    std::vector<ObjectId> bodies = reach_.included();
    const bool pinned = usesBody();

    // A sketch standing on its own becomes a part when it is joined: its first
    // solid is this, with anything else a join reaches merged into it.
    const bool firstSolid = ownerObj->body.empty() && (op == ExtrudeOp::Join || op == ExtrudeOp::NewBody);
    if (firstSolid) {
        if (op == ExtrudeOp::NewBody) bodies.clear();
        op = ExtrudeOp::Join;
        bodies.erase(std::remove(bodies.begin(), bodies.end(), owner), bodies.end());
        bodies.insert(bodies.begin(), owner);
    }
    // A step built from a part's own faces or edges is that part's.
    if (pinned && op != ExtrudeOp::NewBody && std::find(bodies.begin(), bodies.end(), owner) == bodies.end())
        bodies.insert(bodies.begin(), owner);

    const bool asNewBody = op == ExtrudeOp::NewBody || (op == ExtrudeOp::Join && bodies.empty());
    if (!asNewBody && bodies.empty()) {
        error_ = op == ExtrudeOp::Cut ? "Nothing there to cut into" : "Nothing there to intersect with";
        return false;
    }
    const std::string verb = profileBuildName(build_);
    const std::string label = verb + (asNewBody ? "" : op == ExtrudeOp::Join ? " Join"
                                                    : op == ExtrudeOp::Cut  ? " Cut" : " Intersect");

    std::vector<std::pair<ObjectId, ElementId>> brought;
    // The sketches that stood on their own and were brought in are taken from
    // where they stood -- the whole object, when that was all it held. One in
    // another part is left: something there may be built on it.
    auto takeAwayBrought = [&](std::vector<std::unique_ptr<Command>>& parts) {
        std::vector<ObjectId> emptied, seen;
        for (const auto& [src, uid] : brought) {
            if (std::find(seen.begin(), seen.end(), src) != seen.end()) continue;
            seen.push_back(src);
            SceneObject* o = scene.find(src);
            if (!o || !o->body.empty()) continue;
            const bool onlySketches = std::all_of(o->features.begin(), o->features.end(),
                                                  [](const Feature& f) { return f.kind == FeatureKind::Sketch; });
            if (!onlySketches) continue;
            std::vector<Feature> left;
            for (const Feature& f : o->features) {
                const bool taken = std::any_of(brought.begin(), brought.end(),
                                               [&](const auto& b) { return b.first == src && b.second == f.uid; });
                if (!taken) left.push_back(f);
            }
            if (left.empty()) {
                emptied.push_back(src);
            } else {
                const std::vector<Feature> before = o->features;
                if (scene.setFeatures(src, left, nullptr))
                    parts.push_back(std::make_unique<FeatureCommand>(src, before, left, label));
            }
        }
        if (!emptied.empty()) parts.push_back(ExistenceCommand::forDelete(scene, emptied));
    };
    auto pushUndo = [&](std::vector<std::unique_ptr<Command>>& parts) {
        if (parts.size() == 1) undo.push(std::move(parts.front()));
        else                   undo.push(std::make_unique<CompositeCommand>(std::move(parts), label));
    };

    if (asNewBody) {
        std::vector<std::unique_ptr<Command>> parts;
        ObjectId id = kNoObject;
        if (pinned) {
            // Built from a part's own faces, as a body of its own: the solid as
            // it stands, since the faces it came from are that part's.
            id = scene.addBody(preview_, {}, verb);
        } else {
            std::vector<Feature> chain;
            if (!makeStep(scene, kNoObject, Mat4::identity(), chain, &scene, brought, &why)) {
                error_ = why;
                return false;
            }
            id = scene.addFeatureChain(std::move(chain), "Part", &why);
        }
        if (id == kNoObject) {
            error_ = verb + " failed: " + (why.empty() ? std::string("no solid came out of it") : why);
            return false;
        }
        parts.push_back(ExistenceCommand::forCreate(scene, {id}));
        takeAwayBrought(parts);
        pushUndo(parts);
        scene.select(id);
        *this = ProfileTool{};
        return true;
    }

    const ObjectId homeId = firstSolid || pinned || reach_.includes(owner) ? owner : bodies.front();
    const bool homeActs = std::find(bodies.begin(), bodies.end(), homeId) != bodies.end();
    SceneObject* obj = scene.find(homeId);
    if (!obj) {
        error_ = "The part it goes into is gone";
        return false;
    }
    if (!obj->body.empty() && obj->body.isMesh()) {
        error_ = "Building onto a mesh needs a solid: Modify > Convert to Solid first";
        return false;
    }
    const std::vector<Feature> before = obj->features;
    std::vector<Feature> chain = before;
    if (homeActs) {
        choice_.pick(op);
        if (!makeStep(scene, homeId, obj->modelMatrix(), chain, &scene, brought, &why)) {
            error_ = why;
            return false;
        }
    }
    if (!scene.setFeatures(homeId, chain, &why)) {
        error_ = verb + " failed: " + (why.empty() ? std::string("the history would not evaluate") : why);
        return false;
    }
    std::vector<std::unique_ptr<Command>> parts;
    parts.push_back(std::make_unique<FeatureCommand>(homeId, before, scene.find(homeId)->features, label));
    if (bodies.size() > (homeActs ? 1u : 0u)) {
        std::string error;
        if (!applyExtrude(scene, preview_, op, bodies, homeActs ? homeId : kNoObject, verb, label.c_str(), parts,
                          error)) {
            unwind(scene, parts);
            error_ = error;
            return false;
        }
    }
    takeAwayBrought(parts);
    pushUndo(parts);
    if (scene.find(homeId)) scene.select(homeId);
    *this = ProfileTool{};
    return true;
}

// ---------------------------------------------------------------------------
// Per frame
// ---------------------------------------------------------------------------

void ProfileTool::update(const Scene& scene, const Camera& camera, Vec2 mousePx) {
    if (!active_) return;
    hover_ = Hover::None;
    hoverObject_ = kNoObject;
    hoverSketch_ = 0;
    hoverItem_ = kNoSketchId;
    hoverElement_ = kInvalid;
    hoverPath_ = SketchPath{};

    const bool outlines = slot_ == ProfileSlot::Profile || slot_ == ProfileSlot::Outlines;
    const Ray ray = camera.rayThroughPixel(static_cast<float>(mousePx.x), static_cast<float>(mousePx.y));

    if (outlines) {
        // The nearest of: a region of a sketch the ray passes through, and a
        // flat face it hits. A sketch drawn on a face lies in it, so on a tie
        // the region wins -- it is the thing drawn to be picked.
        Real bestT = 1e300;
        for (const SceneSketch& s : sketches_) {
            const SketchPlane& pl = s.sketch.plane;
            const Vec3 n = pl.normal();
            const Real denom = dot(n, ray.dir);
            if (std::fabs(denom) < 1e-9) continue;
            const Real t = dot(pl.origin - ray.origin, n) / denom;
            if (t < 0.0 || t >= bestT) continue;
            const Vec3 d = ray.origin + ray.dir * t - pl.origin;
            const Vec2 uv{dot(d, pl.xAxis) / lengthSq(pl.xAxis), dot(d, pl.yAxis) / lengthSq(pl.yAxis)};
            Real smallest = 1e300;
            SketchId found = kNoSketchId;
            for (const SketchProfile& r : s.regions)
                if (r.area < smallest && sketchProfileContains(s.sketch, r, uv)) {
                    smallest = r.area;
                    found = r.key;
                }
            if (found == kNoSketchId) continue;
            bestT = t;
            hover_ = Hover::Region;
            hoverObject_ = s.object;
            hoverSketch_ = s.uid;
            hoverItem_ = found;
        }
        const RayHit hit = scene.raycast(ray);
        if (hit.hit() && hit.face != kInvalid && static_cast<Real>(hit.t) < bestT - 1e-3) {
            const SceneObject* o = scene.find(hit.object);
            if (exactBody(o) && o->body.faceKind(hit.face) == SurfaceKind::Plane) {
                hover_ = Hover::Face;
                hoverObject_ = hit.object;
                hoverElement_ = hit.face;
            }
        }
        return;
    }

    // An axis or a path: a curve of a sketch near the pointer on the screen,
    // then an edge of a body. The sketches are drawn over the bodies, so what
    // looks nearest is a curve.
    const bool axisOnly = slot_ == ProfileSlot::Axis;
    Real best = kCurvePickPx;
    for (const SceneSketch& s : sketches_) {
        for (size_t k = 0; k < s.lines.size(); ++k) {
            const SketchEntity* e = s.sketch.entity(s.lineOf[k]);
            if (!e || (axisOnly && e->curve != SketchCurve::Line)) continue;
            Vec2 prev{};
            bool have = false;
            for (const Vec3& w : s.lines[k]) {
                Vec2 px{};
                const bool on = camera.projectToPixel(w, px);
                if (on && have) {
                    const Real d = distanceToSegment(mousePx, prev, px);
                    if (d <= best) {
                        best = d;
                        hover_ = Hover::Curve;
                        hoverObject_ = s.object;
                        hoverSketch_ = s.uid;
                        hoverItem_ = s.lineOf[k];
                    }
                }
                prev = px;
                have = on;
            }
        }
    }
    if (hover_ == Hover::Curve) {
        if (!axisOnly)
            if (const SceneSketch* s = sketchOf(hoverObject_, hoverSketch_))
                sketchPathThrough(s->sketch, hoverItem_, hoverPath_);
        return;
    }
    const ElementHit hit = scene.pickElement(ray, camera.viewProjection(), camera.viewportW, camera.viewportH,
                                             mousePx, 0.0f, kEdgePickPx);
    if (hit.hit() && hit.ref.kind == ElementKind::Edge) {
        const SceneObject* o = scene.find(hit.ref.object);
        if (exactBody(o) && (!axisOnly || o->body.edgeKind(hit.ref.index) == CurveKind::Line)) {
            hover_ = Hover::Edge;
            hoverObject_ = hit.ref.object;
            hoverElement_ = hit.ref.index;
        }
    }
}

void ProfileTool::handleMouseDown(const Scene& scene) {
    if (!active_) return;
    switch (hover_) {
        case Hover::Region: pickRegion(scene, hoverObject_, hoverSketch_, hoverItem_); break;
        case Hover::Face:   pickFace(scene, hoverObject_, hoverElement_); break;
        case Hover::Curve:  pickCurve(scene, hoverObject_, hoverSketch_, hoverItem_); break;
        case Hover::Edge:   pickEdge(scene, hoverObject_, hoverElement_); break;
        case Hover::None:   break;
    }
}

bool ProfileTool::handleKey(int key, Scene& scene, UndoStack& undo) {
    if (!active_) return false;
    if (key == 27) { cancel(); return true; }
    if (key == 13) { finish(scene, undo); return true; }
    if (key == 8) {
        // Takes back the last thing picked in the slot being filled.
        if (slot_ == ProfileSlot::Outlines && !outlines_.empty()) outlines_.pop_back();
        else if (slot_ == ProfileSlot::Path) path_ = PathPick{};
        else if (slot_ == ProfileSlot::Axis) axis_ = AxisPick{};
        else if (slot_ == ProfileSlot::Profile) profile_ = OutlinePick{};
        refreshPreview(scene);
        return true;
    }
    if (build_ == ProfileBuild::Revolve && slot_ == ProfileSlot::Axis) {
        if (key == 'X') { setWorldAxis(0); refreshPreview(scene); return true; }
        if (key == 'Y') { setWorldAxis(1); refreshPreview(scene); return true; }
        if (key == 'Z') { setWorldAxis(2); refreshPreview(scene); return true; }
    }
    ExtrudeOp picked;
    if (extrudeOpForKey(key, picked)) {
        choice_.pick(picked);
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------

void ProfileTool::drawOverlay(const Scene& scene, const Camera& camera, Renderer& renderer) const {
    if (!active_) return;
    const bool removes = choice_.op == ExtrudeOp::Cut || choice_.op == ExtrudeOp::Intersect;
    const Vec4 brand = removes ? Vec4{1.0f, 0.4f, 0.3f, 0.95f} : toVec4(palette::kBrand, 0.95f);
    const Vec4 lit{1.0f, 0.82f, 0.35f, 1.0f};
    const Vec4 faint{0.55f, 0.62f, 0.72f, 0.45f};

    auto polyline = [&](const std::vector<Vec3>& pts, const Vec4& c, Real w, bool closed) {
        for (size_t i = 0; i + 1 < pts.size(); ++i) renderer.addFrontLine(camera, pts[i], pts[i + 1], c, w);
        if (closed && pts.size() > 2) renderer.addFrontLine(camera, pts.back(), pts.front(), c, w);
    };
    auto regionLine = [&](ObjectId object, ElementId uid, SketchId key, const Vec4& c, Real w) {
        const SceneSketch* s = sketchOf(object, uid);
        if (!s) return;
        for (const SketchProfile& r : s->regions) {
            if (r.key != key) continue;
            std::vector<Vec3> pts;
            for (Vec2 q : sketchLoopPoints(s->sketch, r.outer)) pts.push_back(s->sketch.plane.toWorld(q));
            polyline(pts, c, w, true);
            for (const SketchLoop& h : r.holes) {
                pts.clear();
                for (Vec2 q : sketchLoopPoints(s->sketch, h)) pts.push_back(s->sketch.plane.toWorld(q));
                polyline(pts, c, w * 0.7, true);
            }
        }
    };
    auto faceTint = [&](ObjectId object, FaceId face, const Vec4& c) {
        const SceneObject* o = scene.find(object);
        if (!o) return;
        const Mat4 model = o->modelMatrix();
        const RenderMesh& m = o->render;
        for (size_t t = 0; t < m.triangleFace.size(); ++t) {
            if (m.triangleFace[t] != face) continue;
            renderer.addTriangle(transformPoint(model, m.positions[m.triangles[3 * t]]),
                                 transformPoint(model, m.positions[m.triangles[3 * t + 1]]),
                                 transformPoint(model, m.positions[m.triangles[3 * t + 2]]), c);
        }
    };
    auto edgeLine = [&](ObjectId object, EdgeId edge, const Vec4& c, Real w) {
        const SceneObject* o = scene.find(object);
        if (!exactBody(o)) return;
        std::vector<Vec3> pts;
        o->body.edgePolyline(edge, 0.05, pts);
        const Mat4 model = o->modelMatrix();
        for (Vec3& p : pts) p = transformPoint(model, p);
        polyline(pts, c, w, false);
    };
    auto outlinePick = [&](const OutlinePick& p, const Vec4& c) {
        if (p.fromSketch()) {
            for (SketchId k : p.keys) regionLine(p.object, p.sketchUid, k, c, 3.0);
        } else {
            faceTint(p.object, p.face, Vec4{c.x, c.y, c.z, 0.35f});
            const SceneObject* o = scene.find(p.object);
            if (exactBody(o)) {
                std::vector<EdgeId> es;
                o->body.faceEdges(p.face, es);
                for (EdgeId e : es) edgeLine(p.object, e, c, 2.6);
            }
        }
    };

    // Every sketch it can pick from: the ones hidden in the outliner too,
    // faintly, since a sketch already built from is often the one wanted.
    for (const SceneSketch& s : sketches_) {
        if (s.shown) continue;
        for (const std::vector<Vec3>& line : s.lines) polyline(line, faint, 1.2, false);
    }

    // The solid it will make.
    if (previewOk_) {
        const Vec4 fill{brand.x, brand.y, brand.z, 0.22f};
        const Vec4 edge{brand.x, brand.y, brand.z, 0.85f};
        const RenderMesh& m = previewMesh_;
        for (size_t i = 0; i + 2 < m.triangles.size(); i += 3)
            renderer.addTriangle(m.positions[m.triangles[i]], m.positions[m.triangles[i + 1]],
                                 m.positions[m.triangles[i + 2]], fill);
        for (size_t i = 0; i + 1 < m.edgeLines.size(); i += 2)
            renderer.addLine(m.positions[m.edgeLines[i]], m.positions[m.edgeLines[i + 1]], edge);
    }

    // What is picked.
    if (build_ == ProfileBuild::Loft) {
        for (const OutlinePick& p : outlines_) outlinePick(p, brand);
    } else if (!profile_.empty()) {
        outlinePick(profile_, brand);
    }
    if (build_ == ProfileBuild::Revolve && axis_.kind != AxisPick::Kind::None) {
        Vec3 at{}, dir{};
        bool ok = true;
        if (axis_.kind == AxisPick::Kind::World) {
            (&dir.x)[axis_.world] = 1.0;
        } else if (axis_.kind == AxisPick::Kind::SketchLine) {
            const SceneSketch* s = sketchOf(axis_.object, axis_.sketchUid);
            const SketchEntity* e = s ? s->sketch.entity(axis_.entity) : nullptr;
            ok = e && s->sketch.point(e->a) && s->sketch.point(e->b);
            if (ok) {
                at = s->sketch.plane.toWorld(s->sketch.point(e->a)->at);
                dir = s->sketch.plane.toWorld(s->sketch.point(e->b)->at) - at;
            }
        } else {
            const SceneObject* o = scene.find(axis_.object);
            ok = exactBody(o);
            if (ok) {
                Vec3 b;
                o->body.edgePositions(axis_.edge, at, b);
                at = transformPoint(o->modelMatrix(), at);
                dir = transformPoint(o->modelMatrix(), b) - at;
            }
        }
        if (ok && length(dir) > 1e-9) {
            const Vec3 d = normalize(dir);
            const Real span = std::max<Real>(camera.distance * 1.6, 20.0);
            renderer.addFrontDashes(camera, at - d * span, at + d * span, brand, 1.8, 8.0, 5.0);
        }
    }
    if (build_ == ProfileBuild::Sweep && !path_.empty()) {
        if (path_.fromSketch()) {
            if (const SceneSketch* s = sketchOf(path_.object, path_.sketchUid)) {
                SketchPath p;
                if (sketchPathOf(s->sketch, path_.entities, p)) {
                    std::vector<Vec3> pts;
                    for (Vec2 q : sketchPathPoints(s->sketch, p, 32)) pts.push_back(s->sketch.plane.toWorld(q));
                    polyline(pts, brand, 3.4, false);
                }
            }
        } else {
            for (EdgeId e : path_.edges) edgeLine(path_.object, e, brand, 3.4);
        }
    }

    // What the pointer is over.
    switch (hover_) {
        case Hover::Region: regionLine(hoverObject_, hoverSketch_, hoverItem_, lit, 2.6); break;
        case Hover::Face:
            faceTint(hoverObject_, hoverElement_, Vec4{lit.x, lit.y, lit.z, 0.30f});
            break;
        case Hover::Curve:
            if (const SceneSketch* s = sketchOf(hoverObject_, hoverSketch_)) {
                if (!hoverPath_.entities.empty()) {
                    std::vector<Vec3> pts;
                    for (Vec2 q : sketchPathPoints(s->sketch, hoverPath_, 32)) pts.push_back(s->sketch.plane.toWorld(q));
                    polyline(pts, lit, 3.0, false);
                } else {
                    for (size_t k = 0; k < s->lineOf.size(); ++k)
                        if (s->lineOf[k] == hoverItem_) polyline(s->lines[k], lit, 3.0, false);
                }
            }
            break;
        case Hover::Edge: edgeLine(hoverObject_, hoverElement_, lit, 3.0); break;
        case Hover::None: break;
    }
}

void ProfileTool::drawPointerPrompt(Vec2 mouseScreen) const {
    if (!active_) return;
    std::string text = prompt();
    if (!error_.empty()) text = error_;
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    const ImVec2 size = ImGui::CalcTextSize(text.c_str());
    const ImVec2 at(static_cast<float>(mouseScreen.x) + 18.0f, static_cast<float>(mouseScreen.y) + 16.0f);
    dl->AddRectFilled(ImVec2(at.x - 6, at.y - 4), ImVec2(at.x + size.x + 6, at.y + size.y + 4),
                      IM_COL32(24, 24, 28, 225), 4.0f);
    dl->AddRect(ImVec2(at.x - 6, at.y - 4), ImVec2(at.x + size.x + 6, at.y + size.y + 4),
                error_.empty() ? IM_COL32(243, 68, 37, 200) : IM_COL32(240, 90, 70, 255), 4.0f);
    dl->AddText(at, IM_COL32(236, 236, 240, 255), text.c_str());
}

void ProfileTool::drawHud(Scene& scene, UndoStack& undo, bool& finished) {
    finished = false;
    if (!active_) return;
    refreshPreview(scene);
    std::string why;
    const ObjectId owner = home(scene, &why);
    const SceneObject* ownerObj = scene.find(owner);
    choice_.follow(1.0, ownerObj && !ownerObj->body.empty());
    if (previewOk_) reach_.refresh(scene, preview_, owner, choice_.op, 1.0, previewKey_);

    if (!ui::beginCommand("##profile", profileBuildName(build_), glyphOf(build_))) return;

    auto nameOf = [&](ObjectId id) {
        const SceneObject* o = scene.find(id);
        return o ? o->name : std::string("?");
    };
    auto describe = [&](const OutlinePick& p) {
        if (p.empty()) return std::string();
        if (p.fromSketch())
            return (p.keys.size() == 1 ? std::string("Region") : std::to_string(p.keys.size()) + " regions") +
                   " in " + nameOf(p.object);
        return "Face of " + nameOf(p.object);
    };

    // One row per thing to point at. The one the next click fills is lit;
    // clicking a row makes it the one.
    auto slotRow = [&](ProfileSlot s, const std::string& filled) {
        ui::commandRow(slotName(s));
        const bool on = slot_ == s;
        const std::string text = filled.empty() ? std::string(on ? "click in the view..." : "not picked yet")
                                                : filled;
        ImGui::PushID(static_cast<int>(s));
        if (ui::pillButton(text.c_str(), on, ImVec2(ImGui::GetContentRegionAvail().x, 0))) slot_ = s;
        ImGui::PopID();
    };

    if (build_ == ProfileBuild::Loft) {
        ui::commandRow("Outlines");
        ImGui::TextDisabled("in the order the solid runs through them");
        for (size_t i = 0; i < outlines_.size(); ++i) {
            char label[16];
            std::snprintf(label, sizeof label, "%zu", i + 1);
            ui::commandRow(label);
            ImGui::TextUnformatted(describe(outlines_[i]).c_str());
            ImGui::SameLine();
            ImGui::PushID(static_cast<int>(i));
            if (ui::quietButton("x")) {
                outlines_.erase(outlines_.begin() + static_cast<long>(i));
                refreshPreview(scene);
                ImGui::PopID();
                break;
            }
            ui::hoverTip("Take this outline out");
            ImGui::PopID();
        }
        slot_ = ProfileSlot::Outlines;
        char next[16];
        std::snprintf(next, sizeof next, "%zu", outlines_.size() + 1);
        ui::commandRow(next);
        ImGui::TextColored(ImVec4(palette::kBrand.r, palette::kBrand.g, palette::kBrand.b, 1.0f),
                           "click a region or a flat face");
        static const ui::Choice kWalls[2] = {
            {Glyph::Loft, "Smooth",   nullptr, "Walls that run smoothly through every outline"},
            {Glyph::Line, "Straight", nullptr, "Straight walls from each outline to the next"},
        };
        const int pick = ui::commandChoices("Walls", kWalls, 2, ruled_ ? 1 : 0);
        if (pick >= 0) { ruled_ = pick == 1; refreshPreview(scene); }
    } else {
        slotRow(ProfileSlot::Profile, describe(profile_));
        if (build_ == ProfileBuild::Revolve) {
            std::string axis;
            switch (axis_.kind) {
                case AxisPick::Kind::World:      axis = std::string(1, "XYZ"[axis_.world]) + " axis of the world"; break;
                case AxisPick::Kind::SketchLine: axis = "Line in " + nameOf(axis_.object); break;
                case AxisPick::Kind::Edge:       axis = "Edge of " + nameOf(axis_.object); break;
                case AxisPick::Kind::None:       break;
            }
            slotRow(ProfileSlot::Axis, axis);
            ui::commandRow("");
            for (int a = 0; a < 3; ++a) {
                if (a) ImGui::SameLine(0.0f, 3.0f);
                const char* names[3] = {"X", "Y", "Z"};
                const bool on = axis_.kind == AxisPick::Kind::World && axis_.world == a;
                if (ui::pillButton(names[a], on)) { setWorldAxis(a); refreshPreview(scene); }
            }
            ImGui::SameLine(0.0f, 10.0f);
            if (ui::pillButton("Reverse", reverse_)) { reverse_ = !reverse_; refreshPreview(scene); }
            ui::hoverTip("Turn the other way round the axis");

            double deg = angle_ * kRad2Deg;
            const ui::NumberEdit e = ui::commandNumber("Angle", deg, "\xC2\xB0", false, false, nullptr, 1.0, 360.0);
            if (e.dragged) { angle_ = std::clamp(e.value, 1.0, 360.0) * kDeg2Rad; refreshPreview(scene); }
            ui::commandRow("");
            if (ui::quietButton("Full turn")) { angle_ = 2.0 * kPi; refreshPreview(scene); }
            ImGui::SameLine();
            if (ui::quietButton("Half")) { angle_ = kPi; refreshPreview(scene); }
            ImGui::SameLine();
            if (ui::quietButton("Quarter")) { angle_ = kPi * 0.5; refreshPreview(scene); }
        } else {
            std::string path;
            if (!path_.empty())
                path = path_.fromSketch()
                           ? std::to_string(path_.entities.size()) + (path_.entities.size() == 1 ? " curve" : " curves") +
                                 " in " + nameOf(path_.object)
                           : std::to_string(path_.edges.size()) + (path_.edges.size() == 1 ? " edge" : " edges") +
                                 " of " + nameOf(path_.object);
            slotRow(ProfileSlot::Path, path);
        }
    }

    // What to do now, or what is wrong: said once, in the panel and by the
    // pointer.
    bool complete = true;
    for (ProfileSlot s : slotsOf(build_)) complete = complete && slotFilled(s);
    if (!error_.empty()) ui::commandRefused(error_.c_str());
    else if (complete && !previewOk_ && !previewWhy_.empty()) ui::commandRefused(previewWhy_.c_str());
    else if (!complete) ui::commandHint(prompt().c_str());
    // With nothing in the scene to point at, the prompt alone is a dead end.
    const bool anyBody = std::any_of(scene.objects().begin(), scene.objects().end(),
                                     [](const auto& o) { return o->visible && exactBody(o.get()); });
    if (sketches_.empty() && !anyBody)
        ui::commandRefused("There is nothing to build from yet: draw a sketch (Shift+S) or add a shape first");

    if (drawExtrudeChoice(choice_, true)) previewKey_.clear();
    if (previewOk_) {
        const ObjectId toggled = drawReachedBodies(scene, reach_, choice_.op, owner);
        if (toggled != kNoObject) reach_.toggle(toggled);
    }
    ui::commandHint("Click in the view to pick; Backspace takes back the last pick. Hidden sketches show "
                    "faintly and can be picked too. J joins, D cuts, I intersects, N makes a new body. "
                    "Enter finishes, Esc cancels.");
    const int footer = ui::commandFooter("Finish", previewOk_, "Cancel");
    ui::endCommand();

    if (footer > 0) {
        if (finish(scene, undo)) finished = true;
    } else if (footer < 0) {
        cancel();
        finished = true;
    }
}

} // namespace tg
