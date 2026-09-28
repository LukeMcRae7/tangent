#include "app/joint_tool.h"
#include "core/units.h"

#include "core/palette.h"
#include "ui/command_panel.h"
#include "ui/widgets.h"

#include <imgui.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>

namespace tg {
namespace {

constexpr float kVertexPickPx = 9.0f;
constexpr float kEdgePickPx = 9.0f;

// A pick that gives the joint an axis a part can turn about: a round face, a
// circular edge, a straight edge. Two parts joined by those most often turn.
bool axial(const Body& body, JointAt at, Index element) {
    if (at == JointAt::Face) return body.hasFace(element) && body.faceKind(element) == SurfaceKind::Cylinder;
    if (at == JointAt::Edge)
        return body.hasEdge(element) &&
               (body.edgeKind(element) == CurveKind::Circle || body.edgeKind(element) == CurveKind::Line);
    return false;
}

const char* pickNoun(const Body& body, JointAt at, Index element) {
    switch (at) {
        case JointAt::Face:
            return body.hasFace(element) && body.faceKind(element) == SurfaceKind::Cylinder ? "Round face" : "Face";
        case JointAt::Edge:
            return body.hasEdge(element) && body.edgeKind(element) == CurveKind::Circle ? "Circular edge" : "Edge";
        case JointAt::Vertex: return "Corner";
    }
    return "Pick";
}

const char* sideNoun(JointAt at) {
    switch (at) {
        case JointAt::Face:   return "Face";
        case JointAt::Edge:   return "Edge";
        case JointAt::Vertex: return "Corner";
    }
    return "Pick";
}

Mat4 matrixOf(const Rigid& r) { return translate(r.t) * toMat4(r.q); }
Rigid rigidOf(const Transform& t) { return {normalize(t.rotation), t.position}; }

std::string objectName(const Scene& scene, ObjectId id) {
    const SceneObject* o = scene.find(id);
    return o ? o->name : std::string("?");
}

// The joint's frame drawn where it is: a ring round the origin, the axis as an
// arrow out of it, and a short tick for X. Sized on the screen, so it reads the
// same at any zoom.
void drawFrameGlyph(Renderer& r, const Camera& camera, Vec3 origin, Vec3 x, Vec3 z, Vec4 colour, bool strong) {
    const Real s = camera.pixelWorldSize(origin);
    const Vec3 zz = normalize(z), xx = normalize(x), yy = cross(zz, xx);
    const Real len = s * (strong ? 52.0 : 42.0);
    const Vec3 tip = origin + zz * len;
    r.addFrontLine(camera, origin, tip - zz * (s * 9.0), colour, strong ? 2.6 : 2.0);
    const Vec3 side = xx * (s * 5.5);
    r.addFrontTriangle(tip, tip - zz * (s * 11.0) + side, tip - zz * (s * 11.0) - side, colour);
    r.addFrontTriangle(tip, tip - zz * (s * 11.0) + yy * (s * 5.5), tip - zz * (s * 11.0) - yy * (s * 5.5), colour);
    r.addFrontLine(camera, origin, origin + xx * (s * 18.0), Vec4{colour.x, colour.y, colour.z, colour.w * 0.6}, 1.6);
    const int n = 24;
    const Real rad = s * 9.0;
    for (int i = 0; i < n; ++i) {
        const Real a0 = kTwoPi * i / n, a1 = kTwoPi * (i + 1) / n;
        r.addFrontLine(camera, origin + (xx * std::cos(a0) + yy * std::sin(a0)) * rad,
                       origin + (xx * std::cos(a1) + yy * std::sin(a1)) * rad, colour, 1.6);
    }
}

} // namespace

// ---------------------------------------------------------------------------
// Starting and leaving
// ---------------------------------------------------------------------------

void JointTool::start(const Scene& scene) {
    stage_ = Stage::Moving;
    moving_ = JointSide{};
    joint_ = 0;
    hover_ = Hover{};
    error_.clear();
    sweeping_ = false;
    typing_ = Field::None;
    typed_.clear();

    // Something already picked on a body stands for the first click.
    if (scene.elementSelection().size() == 1) {
        const ElementRef& e = scene.elementSelection().front();
        const SceneObject* o = scene.find(e.object);
        if (o && !o->body.empty()) {
            const JointAt at = e.kind == ElementKind::Face ? JointAt::Face
                             : e.kind == ElementKind::Edge ? JointAt::Edge
                                                           : JointAt::Vertex;
            JointFrame f;
            if (jointFrameFrom(o->body, at, e.index, at == JointAt::Face ? e.index : kInvalid, f)) {
                moving_.object = e.object;
                moving_.at = at;
                moving_.element = at == JointAt::Face ? o->body.faceName(e.index)
                                : at == JointAt::Edge ? o->body.edgeName(e.index)
                                                      : o->body.vertexName(e.index);
                moving_.onFace = at == JointAt::Face ? moving_.element : kNoId;
                moving_.frame = f;
                moving_.foundOn = o->geometryVersion;
                movingAxial_ = axial(o->body, at, e.index);
                movingNoun_ = pickNoun(o->body, at, e.index);
                stage_ = Stage::Fixed;
            }
        }
    }
}

void JointTool::edit(Scene& scene, uint32_t id) {
    if (!scene.assembly().joint(id)) return;
    stage_ = Stage::Adjust;
    joint_ = id;
    hover_ = Hover{};
    error_.clear();
    sweeping_ = false;
    typing_ = Field::None;
    typed_.clear();
    ++session_;
    mergeKey_ = "joint:" + std::to_string(id) + ":" + std::to_string(session_);
}

void JointTool::finish(UndoStack& undo) {
    if (stage_ == Stage::Adjust) undo.breakMergeChain();
    stage_ = Stage::None;
    hover_ = Hover{};
    sweeping_ = false;
    typing_ = Field::None;
    typed_.clear();
}

std::string JointTool::prompt() const {
    switch (stage_) {
        case Stage::Moving: return "Point at the part that moves: a face, an edge or a corner";
        case Stage::Fixed:  return "Now the part it goes on";
        default:            return {};
    }
}

// ---------------------------------------------------------------------------
// Picking
// ---------------------------------------------------------------------------

bool JointTool::pick(Scene& scene, UndoStack& undo, ObjectId object, JointAt at, Index element, Index onFace) {
    const SceneObject* o = scene.find(object);
    if (!o || o->body.empty()) { error_ = "Point at a part"; return false; }
    JointFrame f;
    std::string why;
    if (!jointFrameFrom(o->body, at, element, onFace, f, &why)) { error_ = why; return false; }

    JointSide side;
    side.object = object;
    side.at = at;
    side.element = at == JointAt::Face ? o->body.faceName(element)
                 : at == JointAt::Edge ? o->body.edgeName(element)
                                       : o->body.vertexName(element);
    side.onFace = o->body.hasFace(onFace) ? o->body.faceName(onFace) : kNoId;
    side.frame = f;
    side.foundOn = o->geometryVersion;
    side.straightEdge = jointSideStraight(o->body, at, element);

    if (stage_ == Stage::Moving) {
        moving_ = side;
        movingAxial_ = axial(o->body, at, element);
        movingNoun_ = pickNoun(o->body, at, element);
        stage_ = Stage::Fixed;
        error_.clear();
        return true;
    }
    if (stage_ != Stage::Fixed) return false;
    if (object == moving_.object) {
        error_ = "That is the same part: point at the one it goes on";
        return false;
    }

    Joint j;
    j.id = scene.assembly().nextJoint;
    j.moving = moving_;
    j.fixed = side;
    // What two parts joined by these are most likely doing: a pin in a hole
    // or a leaf on a hinge line turns; two faces put together stay.
    j.kind = movingAxial_ || axial(o->body, at, element) ? JointKind::Revolute : JointKind::Rigid;
    if (!checkJoint(scene, j, &why)) { error_ = why; return false; }
    settleJointLayout(scene, j);

    const AssemblyState before = assemblyState(scene);
    ++scene.assembly().nextJoint;
    j.name = std::string(jointKindName(j.kind)) + " " + std::to_string(j.id);
    scene.assembly().joints.push_back(j);
    solveAssembly(scene);

    joint_ = j.id;
    stage_ = Stage::Adjust;
    hover_ = Hover{};
    error_.clear();
    ++session_;
    mergeKey_ = "joint:" + std::to_string(j.id) + ":" + std::to_string(session_);
    undo.push(std::make_unique<AssemblyCommand>(before, assemblyState(scene), "Joint", mergeKey_), true);
    return true;
}

// ---------------------------------------------------------------------------
// Adjusting
// ---------------------------------------------------------------------------

void JointTool::apply(Scene& scene, UndoStack& undo, const char* what, const std::function<void(Joint&)>& change) {
    Joint* j = joint(scene);
    if (!j) return;
    const AssemblyState before = assemblyState(scene);
    const JointKind wasKind = j->kind;
    change(*j);
    // A joint still called what it was made as follows its kind.
    if (j->kind != wasKind && j->name == std::string(jointKindName(wasKind)) + " " + std::to_string(j->id))
        j->name = std::string(jointKindName(j->kind)) + " " + std::to_string(j->id);
    solveAssembly(scene);
    undo.push(std::make_unique<AssemblyCommand>(before, assemblyState(scene), what, mergeKey_), true);
}

void JointTool::setKind(Scene& scene, UndoStack& undo, JointKind kind) {
    // A slider on two flat faces slides across them; along the axis would lift
    // one off the other. On anything with an axis -- a round face, a rim, an
    // edge -- it slides along it.
    auto flat = [&](const JointSide& side) {
        const SceneObject* o = scene.find(side.object);
        if (!o || side.at != JointAt::Face) return false;
        const Index f = o->body.findFace(side.element);
        return o->body.hasFace(f) && o->body.faceKind(f) == SurfaceKind::Plane;
    };
    const Joint* now = joint(scene);
    const bool faces = now && flat(now->moving) && flat(now->fixed);
    apply(scene, undo, "Joint", [&](Joint& j) {
        j.kind = kind;
        if (kind == JointKind::Slider) j.slideAxis = faces ? SlideAxis::X : SlideAxis::Z;
    });
}
void JointTool::setFlip(Scene& scene, UndoStack& undo, bool flip) {
    apply(scene, undo, "Joint", [&](Joint& j) { j.flip = flip; });
}
void JointTool::setReverse(Scene& scene, UndoStack& undo, bool reverse) {
    // The part stays where it is: the number is what changes sign, and the
    // limits with it.
    apply(scene, undo, "Joint", [&](Joint& j) {
        j.reverse = reverse;
        j.turn = -j.turn;
        j.travel = -j.travel;
        if (j.limited) {
            const Real lo = -j.hi, hi = -j.lo;
            j.lo = lo;
            j.hi = hi;
        }
    });
}
void JointTool::setAsBuilt(Scene& scene, UndoStack& undo, bool asBuilt) {
    // Worked out with the part where the joint had it at rest, not wherever
    // the motion has swung it.
    apply(scene, undo, "Joint", [&](Joint& j) {
        j.turn = j.travel = j.slideX = j.slideY = 0.0;
        j.asBuilt = false;
        j.flip = false;
        j.angle = 0.0;
        // Where it was built is its history's own placement, not wherever the
        // joint has it now.
        if (asBuilt)
            for (ObjectId id : jointUnit(scene, j))
                if (SceneObject* o = scene.find(id)) scene.place(*o);
        settleJointLayout(scene, j, asBuilt ? JointPlacing::WhereBuilt : JointPlacing::Snap);
    });
}
void JointTool::setOffset(Scene& scene, UndoStack& undo, Real mm) {
    apply(scene, undo, "Joint", [&](Joint& j) { j.offset = mm; });
}
void JointTool::setAngle(Scene& scene, UndoStack& undo, Real radians) {
    apply(scene, undo, "Joint", [&](Joint& j) { j.angle = radians; });
}
void JointTool::setMotion(Scene& scene, UndoStack& undo, Real value) {
    apply(scene, undo, "Move Joint", [&](Joint& j) {
        if (j.kind == JointKind::Slider) j.travel = value;
        else                             j.turn = value;
        if (j.limited && j.hi >= j.lo) {
            if (j.kind == JointKind::Slider) j.travel = std::clamp(j.travel, j.lo, j.hi);
            else                             j.turn = std::clamp(j.turn, j.lo, j.hi);
        }
    });
}

void JointTool::remove(Scene& scene, UndoStack& undo) {
    Joint* j = joint(scene);
    if (!j) { stage_ = Stage::None; return; }
    const std::vector<ObjectId> unit = jointUnit(scene, *j);
    const AssemblyState before = assemblyState(scene, unit);
    removeJointKeepingPlace(scene, joint_);
    undo.breakMergeChain();
    undo.push(std::make_unique<AssemblyCommand>(before, assemblyState(scene, unit), "Delete Joint"));
    undo.breakMergeChain();
    stage_ = Stage::None;
    joint_ = 0;
}

// ---------------------------------------------------------------------------
// Per frame
// ---------------------------------------------------------------------------

void JointTool::update(const Scene& scene, const Camera& camera, Vec2 mousePx) {
    if (!picking()) return;
    const Ray ray = camera.rayThroughPixel(static_cast<float>(mousePx.x), static_cast<float>(mousePx.y));
    const ElementHit hit = scene.pickElement(ray, camera.viewProjection(), camera.viewportW, camera.viewportH,
                                             mousePx, kVertexPickPx, kEdgePickPx);
    Hover h;
    if (hit.hit()) {
        const SceneObject* o = scene.find(hit.ref.object);
        if (o && !o->body.empty()) {
            h.object = hit.ref.object;
            h.at = hit.ref.kind == ElementKind::Face ? JointAt::Face
                 : hit.ref.kind == ElementKind::Edge ? JointAt::Edge
                                                     : JointAt::Vertex;
            h.element = hit.ref.index;
            const RayHit face = scene.raycast(ray);
            h.onFace = face.hit() && face.object == h.object ? face.face
                     : h.at == JointAt::Face                   ? h.element
                                                                : kInvalid;
        }
    }
    const bool same = h.object == hover_.object && h.at == hover_.at && h.element == hover_.element &&
                      h.onFace == hover_.onFace;
    if (same && h.object != kNoObject) {
        // The frame is what it was; only where the moving part would land can
        // have changed, if something moved underneath.
    } else {
        hover_ = h;
        if (h.object != kNoObject) {
            const SceneObject* o = scene.find(h.object);
            hover_.ok = jointFrameFrom(o->body, h.at, h.element, h.onFace, hover_.frame, &hover_.why);
            hover_.line.clear();
            if (h.at == JointAt::Edge) o->body.edgePolyline(h.element, 0.05, hover_.line);
            hover_.noun = pickNoun(o->body, h.at, h.element);
        }
    }

    // The moving part, see-through, where it would go.
    hover_.placed = false;
    if (stage_ == Stage::Fixed && hover_.ok && hover_.object != moving_.object) {
        Joint j;
        j.moving = moving_;
        j.fixed.object = hover_.object;
        j.fixed.frame = hover_.frame;
        settleJointLayout(scene, j);
        if (jointDelta(scene, j, hover_.delta)) {
            hover_.unit = jointUnit(scene, j);
            hover_.placed = !hover_.unit.empty();
        }
    }
}

void JointTool::handleMouseDown(Scene& scene, UndoStack& undo) {
    if (!picking()) return;
    if (hover_.object == kNoObject) {
        error_ = "Point at a face, an edge or a corner of a part";
        return;
    }
    pick(scene, undo, hover_.object, hover_.at, hover_.element, hover_.onFace);
}

bool JointTool::handleKey(int key, Scene& scene, UndoStack& undo) {
    if (!active()) return false;
    if (typing_ != Field::None) {
        if (key == 27) { typing_ = Field::None; typed_.clear(); return true; }
        if (key == 8) { if (!typed_.empty()) typed_.pop_back(); return true; }
        if ((key >= '0' && key <= '9') || key == '.' || (key == '-' && typed_.empty())) {
            typed_ += static_cast<char>(key);
            return true;
        }
        if (key == 13) {
            char* end = nullptr;
            const double v = std::strtod(typed_.c_str(), &end);
            if (end && end != typed_.c_str()) applyTyped(scene, undo, v);
            typing_ = Field::None;
            typed_.clear();
            return true;
        }
        return false;
    }
    if (key == 27) {
        if (picking()) cancel();
        else           finish(undo);
        return true;
    }
    if (key == 13 && adjusting()) { finish(undo); return true; }
    if (key == 8 && stage_ == Stage::Fixed) {
        moving_ = JointSide{};
        stage_ = Stage::Moving;
        return true;
    }
    if (key == 'F' && adjusting()) {
        if (const Joint* j = joint(scene)) setFlip(scene, undo, !j->flip);
        return true;
    }
    return false;
}

void JointTool::applyTyped(Scene& scene, UndoStack& undo, double v) {
    Joint* j = joint(scene);
    if (!j) return;
    // Lengths are typed in the unit shown; turns in degrees.
    const double mm = units::fromShown(v);
    switch (typing_) {
        case Field::Offset: setOffset(scene, undo, mm); break;
        case Field::Angle:  setAngle(scene, undo, v * kDeg2Rad); break;
        case Field::Motion: setMotion(scene, undo, j->kind == JointKind::Slider ? mm : v * kDeg2Rad); break;
        case Field::SlideX: apply(scene, undo, "Move Joint", [&](Joint& k) { k.slideX = mm; }); break;
        case Field::SlideY: apply(scene, undo, "Move Joint", [&](Joint& k) { k.slideY = mm; }); break;
        case Field::Lo:
            apply(scene, undo, "Joint", [&](Joint& k) { k.lo = k.kind == JointKind::Slider ? mm : v * kDeg2Rad; });
            break;
        case Field::Hi:
            apply(scene, undo, "Joint", [&](Joint& k) { k.hi = k.kind == JointKind::Slider ? mm : v * kDeg2Rad; });
            break;
        case Field::None: break;
    }
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------

void JointTool::drawOverlay(const Scene& scene, const Camera& camera, Renderer& renderer) const {
    if (!active()) return;
    const Vec4 brand = toVec4(palette::kBrand, 0.95f);
    const Vec4 lit{1.0f, 0.82f, 0.35f, 1.0f};
    const Vec4 bad{1.0f, 0.4f, 0.3f, 0.95f};

    auto worldFrame = [&](ObjectId id, const JointFrame& f, Vec3& o, Vec3& x, Vec3& z) {
        const SceneObject* obj = scene.find(id);
        if (!obj) return false;
        const Mat4 m = obj->modelMatrix();
        o = transformPoint(m, f.origin);
        x = transformVector(m, f.x);
        z = transformVector(m, f.z);
        return true;
    };
    auto tintFace = [&](ObjectId id, Index face, const Vec4& c) {
        const SceneObject* o = scene.find(id);
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

    if (picking()) {
        // The first pick, held where it was made.
        if (stage_ == Stage::Fixed) {
            Vec3 o, x, z;
            if (worldFrame(moving_.object, moving_.frame, o, x, z)) drawFrameGlyph(renderer, camera, o, x, z, brand, true);
        }
        if (hover_.object != kNoObject) {
            const SceneObject* obj = scene.find(hover_.object);
            const Vec4 c = hover_.ok ? lit : bad;
            if (obj) {
                const Mat4 m = obj->modelMatrix();
                if (hover_.at == JointAt::Face) {
                    tintFace(hover_.object, hover_.element, Vec4{c.x, c.y, c.z, 0.28f});
                } else if (hover_.at == JointAt::Edge) {
                    for (size_t i = 0; i + 1 < hover_.line.size(); ++i)
                        renderer.addFrontLine(camera, transformPoint(m, hover_.line[i]),
                                              transformPoint(m, hover_.line[i + 1]), c, 3.0);
                }
            }
            Vec3 o, x, z;
            if (hover_.ok && worldFrame(hover_.object, hover_.frame, o, x, z))
                drawFrameGlyph(renderer, camera, o, x, z, c, false);
        }
        // Where the moving part would go, see-through.
        if (hover_.placed) {
            size_t tris = 0;
            for (ObjectId id : hover_.unit)
                if (const SceneObject* o = scene.find(id)) tris += o->render.triangles.size() / 3;
            const Vec4 fill{brand.x, brand.y, brand.z, 0.16f};
            const Vec4 edge{brand.x, brand.y, brand.z, 0.85f};
            for (ObjectId id : hover_.unit) {
                const SceneObject* o = scene.find(id);
                if (!o || !o->visible) continue;
                const Mat4 m = matrixOf(hover_.delta * rigidOf(o->transform));
                const RenderMesh& rm = o->render;
                // A part of tens of thousands of triangles is drawn by its
                // edges alone: the outline says where it goes, and filling it
                // would cost more than the frame has.
                if (tris <= 40000)
                    for (size_t i = 0; i + 2 < rm.triangles.size(); i += 3)
                        renderer.addTriangle(transformPoint(m, rm.positions[rm.triangles[i]]),
                                             transformPoint(m, rm.positions[rm.triangles[i + 1]]),
                                             transformPoint(m, rm.positions[rm.triangles[i + 2]]), fill);
                for (size_t i = 0; i + 1 < rm.edgeLines.size(); i += 2)
                    renderer.addLine(transformPoint(m, rm.positions[rm.edgeLines[i]]),
                                     transformPoint(m, rm.positions[rm.edgeLines[i + 1]]), edge);
            }
        }
        return;
    }

    // Adjusting: both frames where they now meet, and the line it moves along.
    const Joint* j = scene.assembly().joint(joint_);
    if (!j) return;
    Vec3 fo, fx, fz;
    if (!worldFrame(j->fixed.object, j->fixed.frame, fo, fx, fz)) return;
    drawFrameGlyph(renderer, camera, fo, fx, fz, j->problem.empty() ? brand : bad, true);
    const Real span = std::max<Real>(camera.pixelWorldSize(fo) * 260.0, 10.0);
    if (j->kind == JointKind::Revolute) {
        const Vec3 d = normalize(fz);
        renderer.addFrontDashes(camera, fo - d * span, fo + d * span, brand, 1.6, 8.0, 5.0);
    } else if (j->kind == JointKind::Slider) {
        const Vec3 fy = cross(normalize(fz), normalize(fx));
        const Vec3 d = normalize(j->slideAxis == SlideAxis::X ? fx : j->slideAxis == SlideAxis::Y ? fy : fz);
        const Vec3 at = fo + normalize(fz) * j->offset;
        if (j->limited && j->hi > j->lo) {
            renderer.addFrontLine(camera, at + d * j->lo, at + d * j->hi, brand, 2.2);
        } else {
            renderer.addFrontDashes(camera, at - d * span, at + d * span, brand, 1.6, 8.0, 5.0);
        }
    }
}

void JointTool::drawPointerPrompt(Vec2 mouseScreen) const {
    if (!picking()) return;
    std::string text = prompt();
    if (!error_.empty()) text = error_;
    else if (hover_.object != kNoObject && !hover_.ok && !hover_.why.empty()) text = hover_.why;
    else if (hover_.object != kNoObject && hover_.ok) text = std::string(hover_.noun) + (stage_ == Stage::Moving
                                                          ? ": click to move this part by it"
                                                          : ": click to put it here");
    const bool warn = !error_.empty() || (hover_.object != kNoObject && !hover_.ok);
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    const ImVec2 size = ImGui::CalcTextSize(text.c_str());
    const ImVec2 at(std::floor(static_cast<float>(mouseScreen.x)) + 18.0f,
                    std::floor(static_cast<float>(mouseScreen.y)) + 16.0f);
    dl->AddRectFilled(ImVec2(at.x - 6, at.y - 4), ImVec2(at.x + size.x + 6, at.y + size.y + 4),
                      ui::u32(palette::kCommand, 0.94f), 4.0f);
    dl->AddRect(ImVec2(at.x - 6, at.y - 4), ImVec2(at.x + size.x + 6, at.y + size.y + 4),
                warn ? ui::u32(palette::kBrandHover) : ui::u32(palette::kBrand, 0.8f), 4.0f);
    dl->AddText(at, ui::u32(palette::kText), text.c_str());
}

void JointTool::drawHud(Scene& scene, UndoStack& undo, bool& finished) {
    finished = false;
    if (!active()) return;

    if (picking()) {
        if (!ui::beginCommand("##joint", "Joint", Glyph::Joint)) return;
        auto slot = [&](const char* label, bool filled, bool on, const std::string& text) {
            ui::commandRow(label);
            const std::string shown = filled ? text : on ? std::string("click in the view...") : std::string("not picked yet");
            ui::pillButton(shown.c_str(), on, ImVec2(ImGui::GetContentRegionAvail().x, 0));
        };
        const bool second = stage_ == Stage::Fixed;
        slot("Moves", second, !second,
             second ? std::string(movingNoun_) + " of " + objectName(scene, moving_.object) : std::string());
        slot("Goes on", false, second, std::string());
        if (!error_.empty()) ui::commandRefused(error_.c_str());
        ui::commandHint("Pick the part that moves, then the part it goes on. A flat face joins face to face, "
                        "a round face or circular edge on its axis, and a straight edge as a hinge or rail.");
        const int footer = ui::commandFooter(nullptr, false, "Cancel");
        ui::endCommand();
        if (footer < 0) { cancel(); finished = true; }
        return;
    }

    Joint* j = joint(scene);
    if (!j) { stage_ = Stage::None; finished = true; return; }
    const uint32_t id = j->id;

    // Swinging it through its range: a frame's worth of the motion at a time.
    if (sweeping_ && j->hasMotion()) {
        const bool slider = j->kind == JointKind::Slider;
        const Real lo = j->limited && j->hi > j->lo ? j->lo : (slider ? -20.0 : -kPi);
        const Real hi = j->limited && j->hi > j->lo ? j->hi : (slider ? 20.0 : kPi);
        sweepT_ += ImGui::GetIO().DeltaTime / 2.4;
        const Real phase = 0.5 - 0.5 * std::cos(sweepT_ * kTwoPi);
        if (slider) j->travel = lo + (hi - lo) * phase;
        else        j->turn = lo + (hi - lo) * phase;
        solveAssembly(scene);
    }

    if (!ui::beginCommand("##joint", "Joint", Glyph::Joint, j->name.c_str())) return;

    {
        // What moves on what, and by what: "Lid on Box, edge to edge".
        std::string at = sideNoun(j->moving.at);
        std::string to = sideNoun(j->fixed.at);
        for (char& c : to) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        for (char& c : at) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        char parts[240];
        std::snprintf(parts, sizeof parts, "%s on %s, %s to %s", objectName(scene, j->moving.object).c_str(),
                      objectName(scene, j->fixed.object).c_str(), at.c_str(), to.c_str());
        ui::commandValue("Joins", parts);
    }

    {
        static const ui::Choice kKinds[4] = {
            {Glyph::Lock,          "Rigid",    nullptr, "Held where it is put"},
            {Glyph::JointRevolute, "Revolute", nullptr, "Turns about the axis: a hinge, a wheel, a pin in a hole"},
            {Glyph::JointSlider,   "Slider",   nullptr, "Slides along one line: a drawer, a rail"},
            {Glyph::Move,          "Planar",   nullptr, "Slides anywhere in the plane, and turns in it"},
        };
        const int on = static_cast<int>(j->kind);
        const int pick = ui::commandChoices("Kind", kKinds, 4, on);
        if (pick >= 0 && pick != on) { setKind(scene, undo, static_cast<JointKind>(pick)); j = joint(scene); }
    }

    // The size of what is joined, for how far the bars run.
    Real size = 20.0;
    {
        AABB box;
        for (ObjectId oid : {j->moving.object, j->fixed.object})
            if (const SceneObject* o = scene.find(oid)) box.expand(o->worldBounds());
        if (box.valid()) size = std::max<Real>(length(box.size()), 1.0);
    }
    auto number = [&](Field field, const char* label, double value, const char* unit, double lo, double hi,
                      bool signedRange) -> ui::NumberEdit {
        const bool editing = typing_ == field;
        ui::NumberEdit e = ui::commandNumber(label, value, unit, editing, editing, typed_.c_str(), lo, hi, signedRange);
        if (e.clicked) { typing_ = field; typed_.clear(); }
        if (e.dragged && typing_ == field) { typing_ = Field::None; typed_.clear(); }
        return e;
    };

    {
        ui::commandRow("Placed");
        static const ui::Choice kPlaced[2] = {
            {Glyph::Count, "Where built", nullptr, "Kept where it was modelled, turning about the joint from there"},
            {Glyph::Count, "Snap", nullptr, "Laid onto the second pick, the way two faces meet"},
        };
        const int pick = ui::commandChoices("Placed", kPlaced, 2, j->asBuilt ? 0 : 1, /*compact=*/true);
        if (pick >= 0 && (pick == 0) != j->asBuilt) setAsBuilt(scene, undo, pick == 0);
        j = joint(scene);
    }
    {
        ui::commandRow("Direction");
        if (ui::pillButton("Flip", j->flip)) setFlip(scene, undo, !j->flip);
        ui::hoverTip("Turn it over on the joint: the two axes together instead of facing  (F)");
        ImGui::SameLine(0.0f, 3.0f);
        if (ui::pillButton("Turn 90\xC2\xB0", false)) {
            Real a = std::remainder(j->angle + kHalfPi, kTwoPi);
            setAngle(scene, undo, a);
        }
        ui::hoverTip("A quarter turn about the joint's axis");
        if (j->hasMotion()) {
            ImGui::SameLine(0.0f, 3.0f);
            if (ui::pillButton("Reverse", j->reverse)) setReverse(scene, undo, !j->reverse);
            ui::hoverTip("Count the motion the other way, so the part opens as the number goes up");
        }
    }
    if (ui::NumberEdit e = number(Field::Offset, "Offset", j->offset, "mm", -size, size, true); e.dragged)
        setOffset(scene, undo, e.value);
    if (ui::NumberEdit e = number(Field::Angle, "Angle", j->angle * kRad2Deg, "\xC2\xB0", -180.0, 180.0, true); e.dragged)
        setAngle(scene, undo, e.value * kDeg2Rad);

    j = joint(scene);
    if (j && j->hasMotion()) {
        const bool slider = j->kind == JointKind::Slider;
        if (slider) {
            static const ui::Choice kAxes[3] = {
                {Glyph::Count, "Along", nullptr, "Along the joint's axis: a pin in a hole, a rail on an edge"},
                {Glyph::Count, "Across X", nullptr, "Across the axis, along the frame's X: one face on another"},
                {Glyph::Count, "Across Y", nullptr, "Across the axis, along the frame's Y"},
            };
            const int on = static_cast<int>(j->slideAxis);
            const int pick = ui::commandChoices("Along", kAxes, 3, on, /*compact=*/true);
            if (pick >= 0 && pick != on)
                apply(scene, undo, "Joint", [&](Joint& k) { k.slideAxis = static_cast<SlideAxis>(pick); });
            j = joint(scene);
        }
        const double scale = slider ? 1.0 : kRad2Deg;
        const char* unit = slider ? "mm" : "\xC2\xB0";
        const bool bounded = j->limited && j->hi > j->lo;
        const double lo = bounded ? j->lo * scale : (slider ? -size : -180.0);
        const double hi = bounded ? j->hi * scale : (slider ? size : 180.0);
        const double v = (slider ? j->travel : j->turn) * scale;
        if (ui::NumberEdit e = number(Field::Motion, slider ? "Travel" : "Turn", v, unit, lo, hi, !bounded);
            e.dragged) {
            sweeping_ = false;
            setMotion(scene, undo, e.value / scale);
        }
        j = joint(scene);
        ui::commandRow("");
        if (ui::pillButton(sweeping_ ? "Stop" : "Play", sweeping_)) {
            sweeping_ = !sweeping_;
            sweepT_ = 0.0;
            if (!sweeping_) setMotion(scene, undo, slider ? j->travel : j->turn);
        }
        ui::hoverTip("Swing it back and forth through its range, to see that it clears");
        ImGui::SameLine(0.0f, 3.0f);
        if (ui::pillButton("Limits", j->limited)) {
            apply(scene, undo, "Joint", [&](Joint& k) {
                k.limited = !k.limited;
                if (k.limited && !(k.hi > k.lo)) {
                    k.lo = slider ? 0.0 : 0.0;
                    k.hi = slider ? size * 0.5 : kHalfPi;
                }
            });
            j = joint(scene);
        }
        ui::hoverTip("How far it may go: a lid that opens so far, a drawer that pulls out so far");
        if (j->limited) {
            const double range = slider ? size * 2.0 : 360.0;
            if (ui::NumberEdit e = number(Field::Lo, "From", j->lo * scale, unit, -range, range, true); e.dragged)
                apply(scene, undo, "Joint", [&](Joint& k) { k.lo = std::min(e.value / scale, k.hi); });
            if (ui::NumberEdit e = number(Field::Hi, "To", j->hi * scale, unit, -range, range, true); e.dragged)
                apply(scene, undo, "Joint", [&](Joint& k) { k.hi = std::max(e.value / scale, k.lo); });
            j = joint(scene);
        }
    } else if (j && j->kind == JointKind::Planar) {
        if (ui::NumberEdit e = number(Field::SlideX, "Slide X", j->slideX, "mm", -size, size, true); e.dragged)
            apply(scene, undo, "Move Joint", [&](Joint& k) { k.slideX = e.value; });
        if (ui::NumberEdit e = number(Field::SlideY, "Slide Y", j->slideY, "mm", -size, size, true); e.dragged)
            apply(scene, undo, "Move Joint", [&](Joint& k) { k.slideY = e.value; });
        if (ui::NumberEdit e = number(Field::Motion, "Turn", j->turn * kRad2Deg, "\xC2\xB0", -180.0, 180.0, true);
            e.dragged)
            setMotion(scene, undo, e.value * kDeg2Rad);
        j = joint(scene);
    }

    if (j && !j->problem.empty()) ui::commandRefused(j->problem.c_str());
    else                          ui::commandApplied("Joint");
    ui::commandHint("Sets where the part sits on the joint; it moves with its group and follows the part it is on. "
                    "Delete removes the joint and leaves the part where it is.");
    const int footer = ui::commandFooter("Done", true, "Delete");
    ui::endCommand();

    if (footer > 0) { finish(undo); finished = true; }
    else if (footer < 0) {
        joint_ = id;
        remove(scene, undo);
        finished = true;
    }
}

} // namespace tg
