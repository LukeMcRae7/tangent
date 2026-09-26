// Tangent - the section view: the model drawn cut by a plane that slides.
//
// Nothing here edits anything. The plane is handed to the scene each frame --
// the renderer cuts and caps by it, picking ignores what it takes away -- and
// the rest is choosing the plane: one of the three through the origin, or a
// flat face of the model; then where along it, by the panel's bar, by typing,
// or by the arrow in the view.
#include "app/application.h"

#include "core/palette.h"
#include "ui/command_panel.h"
#include "ui/widgets.h"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace tg {

namespace {

// How long the arrow in the view is, in pixels: long enough to grab and to
// see which way it points, short enough not to cover the part.
constexpr Real kHandlePx = 64.0;

const char* kPlaneNames[4] = {"Top", "Front", "Right", "Face"};

// Two directions in the plane of `n`, for laying the outline out.
void planeBasis(Vec3 n, Vec3& u, Vec3& v) {
    u = normalize(std::fabs(n.z) < 0.9 ? cross(n, Vec3{0.0, 0.0, 1.0}) : cross(n, Vec3{1.0, 0.0, 0.0}));
    v = cross(n, u);
}

Real distToSegmentPx(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const Real len2 = lengthSq(ab);
    if (len2 < 1e-9) return length(p - a);
    const Real t = std::clamp(dot(p - a, ab) / len2, Real(0), Real(1));
    return length(p - (a + ab * t));
}

} // namespace

Vec3 Application::sectionAxis() const {
    switch (section_.plane) {
    case 0: return {0.0, 0.0, 1.0};
    case 1: return {0.0, 1.0, 0.0};
    case 2: return {1.0, 0.0, 0.0};
    default: return section_.faceNormal;
    }
}

bool Application::sectionRange(Vec3 axis, Real& lo, Real& hi) const {
    lo = 1e300;
    hi = -1e300;
    for (const auto& o : scene_.objects()) {
        if (!o->visible || o->body.empty()) continue;
        const AABB b = o->worldBounds();
        if (!b.valid()) continue;
        for (int c = 0; c < 8; ++c) {
            const Vec3 p{(c & 1) ? b.max.x : b.min.x, (c & 2) ? b.max.y : b.min.y, (c & 4) ? b.max.z : b.min.z};
            lo = std::min(lo, dot(axis, p));
            hi = std::max(hi, dot(axis, p));
        }
    }
    return hi >= lo;
}

bool Application::sectionPanelShown() const {
    return section_.on && section_.panel && !commandCornerTaken();
}

void Application::sectionPlane(int plane) {
    section_.pickingFace = false;
    section_.typing = false;
    if (plane == 3) {
        // The face comes from the next click; until then the cut stays where it is.
        section_.pickingFace = true;
        return;
    }
    section_.plane = plane;
    const Vec3 n = sectionAxis();
    Real lo = 0.0, hi = 0.0;
    if (sectionRange(n, lo, hi)) section_.offset = (lo + hi) * 0.5;
    // The half nearer the eye goes, so the inside is what is looked at.
    const Vec3 mid = n * section_.offset;
    section_.sign = dot(n, camera_.eye() - mid) >= 0.0 ? 1.0 : -1.0;
}

void Application::toggleSection() {
    // Off from its own panel; back to the panel when it is on but put away.
    if (section_.on && sectionPanelShown()) {
        section_ = SectionState{};
        return;
    }
    if (section_.on) {
        section_.panel = true;
        if (commandCornerTaken()) setNotice("The section's panel comes back when this operation is done");
        return;
    }
    const bool any = std::any_of(scene_.objects().begin(), scene_.objects().end(),
                                 [](const auto& o) { return o->visible && !o->body.empty(); });
    if (!any) { setNotice("A section view cuts the model, and there is nothing to cut"); return; }
    section_ = SectionState{};
    section_.on = true;
    section_.panel = true;

    // A flat face picked already: the cut goes there, taking away the side the
    // face looks out to -- nothing at first, and the arrow slides it in.
    for (const ElementRef& e : scene_.elementSelection()) {
        const SceneObject* o = scene_.find(e.object);
        if (!o || e.kind != ElementKind::Face || o->body.faceKind(e.index) != SurfaceKind::Plane) continue;
        const Mat4 model = o->modelMatrix();
        section_.plane = 3;
        section_.faceNormal = normalize(transformVector(normalMatrix(model), o->body.faceNormal(e.index)));
        section_.offset = dot(section_.faceNormal, transformPoint(model, o->body.facePoint(e.index)));
        section_.faceAt = section_.offset;
        section_.sign = 1.0;
        return;
    }
    sectionPlane(1);
}

void Application::stepSection() {
    SectionCut cut;
    if (section_.on) {
        cut.on = true;
        cut.normal = sectionAxis() * section_.sign;
        cut.offset = section_.offset * section_.sign;
    }
    scene_.setSection(cut);
    if (!sectionPanelShown()) {
        section_.pickingFace = false;
        section_.dragging = false;
        section_.typing = false;
    }

    // The hatch to the model's scale, the way a drawing's is: a line every
    // fortieth of the model, rounded to a step a person would choose.
    if (cut.on) {
        const AABB b = scene_.bounds();
        const Real size = b.valid() ? length(b.max - b.min) : 10.0;
        view_.sectionHatch = std::max(niceStep(size / 40.0), Real(0.05));
    }
}

bool Application::sectionMouse(bool uiPointer, bool uiClicks, bool overViewport) {
    section_.hoverHandle = false;
    section_.hoverFace = false;
    if (!sectionPanelShown()) return false;
    const Vec2 m = mouseInViewport();
    // A demo's pointer is over the view wherever it is put.
    if (mouseOverride_.x >= 0.0) { overViewport = true; uiPointer = false; }
    const Vec3 n = sectionAxis() * section_.sign;

    if (section_.dragging) {
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left) && sectionDemo_ != 2) {
            section_.dragging = false;
            return true;
        }
        // Along the arrow as it lies on the screen, a pixel worth what the
        // arrow's own length says one is worth there.
        Real lo = 0.0, hi = 0.0;
        sectionRange(sectionAxis(), lo, hi);
        const AABB b = scene_.bounds();
        const Vec3 centre = b.valid() ? b.center() - sectionAxis() * (dot(sectionAxis(), b.center()) - section_.dragFrom)
                                      : sectionAxis() * section_.dragFrom;
        const Real len = kHandlePx * static_cast<Real>(camera_.pixelWorldSize(centre));
        Vec2 a{}, tip{};
        if (camera_.projectToPixel(centre, a) && camera_.projectToPixel(centre + n * len, tip)) {
            const Vec2 along = tip - a;
            const Real px = length(along);
            if (px > 4.0) {
                const Real moved = dot(m - section_.dragStartPx, along / px) * (len / px);
                section_.offset = std::clamp(section_.dragFrom + moved * section_.sign, lo, hi);
            }
        }
        return true;
    }

    if (section_.pickingFace) {
        if (!uiPointer && overViewport) {
            const RayHit hit = scene_.raycast(camera_.rayThroughPixel(static_cast<float>(m.x), static_cast<float>(m.y)));
            if (const SceneObject* o = hit.hit() ? scene_.find(hit.object) : nullptr;
                o && o->body.faceKind(hit.face) == SurfaceKind::Plane) {
                section_.hoverFace = true;
                section_.hoverNormal = normalize(hit.normal);
                section_.hoverOffset = dot(section_.hoverNormal, hit.point);
            }
        }
        if (uiClicks || !overViewport) return false;
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
            section_.pickingFace = false;
            justFinishedModal_ = true;
            return true;
        }
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            justFinishedModal_ = true;
            if (!section_.hoverFace) {
                setNotice("A section goes on a flat face: that one is curved, or not a face");
                return true;
            }
            section_.plane = 3;
            section_.faceNormal = section_.hoverNormal;
            section_.offset = section_.hoverOffset;
            section_.faceAt = section_.offset;
            section_.sign = 1.0;
            section_.pickingFace = false;
            return true;
        }
        return false;
    }

    // Over the arrow: it lights, and a press takes it.
    if (uiPointer || !overViewport) return false;
    const AABB b = scene_.bounds();
    if (!b.valid()) return false;
    const Vec3 axis = sectionAxis();
    const Vec3 centre = b.center() - axis * (dot(axis, b.center()) - section_.offset);
    const Real len = kHandlePx * static_cast<Real>(camera_.pixelWorldSize(centre));
    Vec2 a{}, tip{};
    if (!camera_.projectToPixel(centre, a) || !camera_.projectToPixel(centre + n * len, tip)) return false;
    if (length(tip - a) < 10.0) return false;           // end-on: the panel's bar does it
    section_.hoverHandle = distToSegmentPx(m, a, tip) < 9.0;
    if (section_.hoverHandle && !uiClicks && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        section_.dragging = true;
        section_.dragFrom = section_.offset;
        section_.dragStartPx = m;
        section_.typing = false;
        justFinishedModal_ = true;      // the release is the end of the drag, not a pick
        return true;
    }
    return false;
}

void Application::drawSectionOverlay() {
    if (!sectionPanelShown()) return;
    const AABB b = scene_.bounds();
    if (!b.valid()) return;

    // The plane as a sheet a little larger than the model, where it cuts.
    auto sheet = [&](Vec3 n, Real offset, Vec4 fill, Vec4 edge) {
        Vec3 u, v;
        planeBasis(n, u, v);
        Real u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
        for (int c = 0; c < 8; ++c) {
            const Vec3 p{(c & 1) ? b.max.x : b.min.x, (c & 2) ? b.max.y : b.min.y, (c & 4) ? b.max.z : b.min.z};
            u0 = std::min(u0, dot(u, p)); u1 = std::max(u1, dot(u, p));
            v0 = std::min(v0, dot(v, p)); v1 = std::max(v1, dot(v, p));
        }
        const Real mu = (u1 - u0) * 0.08 + 1.0, mv = (v1 - v0) * 0.08 + 1.0;
        u0 -= mu; u1 += mu; v0 -= mv; v1 += mv;
        const Vec3 o = n * offset;
        const Vec3 q[4] = {o + u * u0 + v * v0, o + u * u1 + v * v0, o + u * u1 + v * v1, o + u * u0 + v * v1};
        renderer_.addTriangle(q[0], q[1], q[2], fill);
        renderer_.addTriangle(q[0], q[2], q[3], fill);
        for (int i = 0; i < 4; ++i) renderer_.addLine(q[i], q[(i + 1) % 4], edge);
        return (q[0] + q[2]) * 0.5;
    };

    if (section_.pickingFace) {
        if (section_.hoverFace)
            sheet(section_.hoverNormal, section_.hoverOffset, toVec4(palette::kBrand, 0.10f),
                  toVec4(palette::kBrand, 0.9f));
        return;
    }

    const Vec3 axis = sectionAxis();
    const Vec3 n = axis * section_.sign;
    const bool lit = section_.hoverHandle || section_.dragging;
    sheet(axis, section_.offset, toVec4(palette::kBrand, lit ? 0.09f : 0.05f),
          toVec4(palette::kBrand, lit ? 0.85f : 0.55f));

    // The arrow, out of the side that is taken away: pull it back to cut
    // deeper. In front of everything, since it is there to be grabbed.
    const Vec3 centre = b.center() - axis * (dot(axis, b.center()) - section_.offset);
    const Real px = static_cast<Real>(camera_.pixelWorldSize(centre));
    const Vec3 tip = centre + n * (kHandlePx * px);
    Vec2 a{}, t{};
    if (!camera_.projectToPixel(centre, a) || !camera_.projectToPixel(tip, t) || length(t - a) < 10.0) return;
    const Vec4 c = lit ? toVec4(palette::kBrand, 1.0f) : toVec4(palette::kBrand, 0.85f);
    renderer_.addFrontLine(camera_, centre, tip - n * (12.0 * px), c, lit ? 3.2 : 2.4);
    Vec3 across = cross(n, normalize(camera_.eye() - tip));
    if (lengthSq(across) > 1e-12) {
        across = normalize(across) * (px * (lit ? 7.0 : 6.0));
        const Vec3 base = tip - n * (14.0 * px);
        renderer_.addFrontTriangle(tip, base + across, base - across, c);
    }
    // A dot where it sits on the plane, to grab it by.
    Vec3 u, v;
    planeBasis(n, u, v);
    const Real r = px * (lit ? 5.0 : 4.0);
    for (int i = 0; i < 12; ++i) {
        const Real a0 = kTwoPi * i / 12.0, a1 = kTwoPi * (i + 1) / 12.0;
        renderer_.addFrontTriangle(centre, centre + (u * std::cos(a0) + v * std::sin(a0)) * r,
                                   centre + (u * std::cos(a1) + v * std::sin(a1)) * r, c);
    }
}

void Application::drawSectionPanel() {
    ui_.sectionOn = section_.on;
    if (!sectionPanelShown()) return;
    if (!ui::beginCommand("##section", "Section View", Glyph::Section)) return;

    const ui::Choice planes[4] = {
        {Glyph::Plane, kPlaneNames[0], nullptr, "Cut level, through the XY plane"},
        {Glyph::Plane, kPlaneNames[1], nullptr, "Cut front to back, through the XZ plane"},
        {Glyph::Plane, kPlaneNames[2], nullptr, "Cut side to side, through the YZ plane"},
        {Glyph::Plane, kPlaneNames[3], nullptr, "Cut along a flat face: click one in the view"},
    };
    const int active = section_.pickingFace ? 3 : section_.plane;
    if (const int pick = ui::commandChoices("Plane", planes, 4, active, true); pick >= 0) sectionPlane(pick);

    // Where the plane is: along its axis from the origin, or in from the face
    // it was put on.
    const Vec3 axis = sectionAxis();
    Real lo = 0.0, hi = 0.0;
    sectionRange(axis, lo, hi);
    // A face's plane measures inwards from the face, the way the cut goes
    // into the part; the others are where they are on their axis.
    const bool fromFace = section_.plane == 3;
    const Real shown = fromFace ? section_.faceAt - section_.offset : section_.offset;
    const Real barLo = fromFace ? section_.faceAt - hi : lo;
    const Real barHi = fromFace ? section_.faceAt - lo : hi;
    const bool editing = section_.typing;
    const char* label = fromFace ? "Depth" : section_.plane == 0 ? "Z" : section_.plane == 1 ? "Y" : "X";
    const ui::NumberEdit e = ui::commandNumber(label, shown, "mm", editing, editing,
                                               section_.typed.c_str(), barLo, barHi, false);
    if (e.clicked) { section_.typing = true; section_.typed.clear(); }
    if (e.dragged) {
        const Real v = std::clamp(static_cast<Real>(e.value), barLo, barHi);
        section_.offset = fromFace ? section_.faceAt - v : v;
        section_.typing = false;
    }

    ui::commandRow("Keep");
    // A button, not a switch: which side went first was the view's choice,
    // and lighting it would say it had been the user's.
    if (ui::pillButton("Flip", false)) section_.sign = -section_.sign;
    ui::hoverTip("Keep the other side");

    size_t cut = 0, all = 0;
    SectionCut plane = scene_.section();
    for (const auto& o : scene_.objects()) {
        if (!o->visible || o->body.empty()) continue;
        ++all;
        const AABB b = o->worldBounds();
        if (!b.valid()) continue;
        Real l = 1e300, h = -1e300;
        for (int c = 0; c < 8; ++c) {
            const Vec3 p{(c & 1) ? b.max.x : b.min.x, (c & 2) ? b.max.y : b.min.y, (c & 4) ? b.max.z : b.min.z};
            l = std::min(l, dot(plane.normal, p) - plane.offset);
            h = std::max(h, dot(plane.normal, p) - plane.offset);
        }
        if (l < 0.0 && h > 0.0) ++cut;
    }
    char through[64];
    if (section_.pickingFace) std::snprintf(through, sizeof through, "Click a flat face in the view");
    else if (all == 1)        std::snprintf(through, sizeof through, cut ? "The part" : "Nothing: slide it in");
    else                      std::snprintf(through, sizeof through, "%zu of %zu parts", cut, all);
    ui::commandValue("Cuts", through);

    ui::commandHint("A view: nothing about the parts changes. Drag the arrow in the view, or the bar, to slide "
                    "the plane; the side the arrow points to is taken away, and the cut is filled and hatched, "
                    "each part the other way to the next.");
    ui::commandHint("It stays while you work -- measure a wall, fillet an edge inside -- and clicks do not "
                    "reach what it has taken away. Done puts the panel away and leaves the model cut; Section "
                    "on the bar brings it back. Remove puts the model back whole.");
    const int footer = ui::commandFooter("Done", true, "Remove");
    ui::endCommand();
    if (footer > 0) { section_.panel = false; section_.pickingFace = false; }
    if (footer < 0) section_ = SectionState{};
    ui_.sectionOn = section_.on;
}

// ---------------------------------------------------------------------------
// Demos
// ---------------------------------------------------------------------------

void Application::setupSectionDemo() {
    scene_.clear();
    undo_.clear();
    const int step = sectionDemo_;
    camera_.yaw = 0.62f;
    camera_.pitch = 0.42f;
    auto frameOn = [&](Vec3 subject, float dist) {
        camera_.distance = dist;
        camera_.target = subject - camera_.right() * (dist * 0.05f) + camera_.up() * (dist * 0.12f);
        camera_.animateTo(camera_.target, dist, camera_.yaw, camera_.pitch);
        camera_.snapToGoal();
        fixedCamera_ = true;
    };
    auto box = [&](Vec3 size, Vec3 at, const char* name) {
        PrimitiveSpec spec;
        spec.kind = PrimitiveKind::Box;
        spec.box = {size.x, size.y, size.z};
        const ObjectId id = scene_.addPrimitive(PrimitiveKind::Box, spec, at);
        if (SceneObject* o = scene_.find(id)) o->name = name;
        return id;
    };
    auto cylinder = [&](Real r, Real h, Vec3 at, const char* name) {
        PrimitiveSpec spec;
        spec.kind = PrimitiveKind::Cylinder;
        spec.cylinder.radius = r;
        spec.cylinder.height = h;
        const ObjectId id = scene_.addPrimitive(PrimitiveKind::Cylinder, spec, at);
        if (SceneObject* o = scene_.find(id)) o->name = name;
        return id;
    };
    auto drill = [&](ObjectId id, Vec3 at, Real diameter) {
        Feature hole;
        hole.kind = FeatureKind::Hole;
        hole.uid = scene_.takeFeatureUid();
        hole.axisPoint = at;
        hole.axisDir = {0, 0, -1};
        hole.hole.diameter = diameter;
        hole.hole.through = true;
        scene_.addFeature(id, hole, nullptr);
    };

    if (step == 1 || step == 2) {
        frameOn({4, 0, 6}, 215.0f);
        const ObjectId plate = box({60, 40, 12}, {0, 0, 6}, "Plate");
        drill(plate, {-15, 0, 12}, 8.0);
        drill(plate, {15, 0, 12}, 5.0);
        cylinder(2.4, 26, {15, 0, 9}, "Pin");
        if (step == 1) {
            toggleSection();                    // the Front plane, through the middle
            stepSection();
            // Checked against the arithmetic: the plane through y = 0 cuts
            // both; a ray down through the middle of the big hole passes
            // through the cut, and one through the solid plate stops at the
            // cut face -- the cap -- and hits nothing.
            bool capped = false;
            const Vec3 eye = camera_.eye();
            const RayHit throughHole = scene_.raycast({{-15, -30, 6}, normalize(Vec3{0, 1, 0})});
            const RayHit plateSide = scene_.raycast({{0, -30, 6}, {0, 1, 0}}, &capped);
            const RayHit front = scene_.raycast({{0, 40, 6}, {0, -1, 0}});
            (void)eye;
            std::fprintf(stderr, "[section-demo] 1: plane=%s sign=%+.0f offset=%.3f  hole ray hits %s at y=%.3f; "
                         "plate ray capped=%d; from behind hits y=%.3f  agrees=%d\n",
                         kPlaneNames[section_.plane], section_.sign, section_.offset,
                         throughHole.hit() ? scene_.find(throughHole.object)->name.c_str() : "nothing",
                         throughHole.hit() ? throughHole.point.y : 0.0, capped ? 1 : 0,
                         front.hit() ? front.point.y : 0.0,
                         (section_.plane == 1 && std::fabs(section_.offset) < 1e-9 && capped && !plateSide.hit() &&
                          front.hit() && std::fabs(front.point.y - 20.0) < 1e-3 &&
                          // In the hole the ray passes the cut half and meets the far wall.
                          throughHole.hit() && std::fabs(throughHole.point.y - 4.0) < 0.05) ? 1 : 0);
        } else {
            // The plane put on the plate's end face and slid in from it by the
            // arrow, dragged by the demo's pointer a few frames in.
            section_ = SectionState{};
            section_.on = true;
            section_.panel = true;
            section_.plane = 3;
            section_.faceNormal = {1, 0, 0};
            section_.offset = 30.0;
            section_.faceAt = 30.0;
            section_.sign = 1.0;
            stepSection();
        }
        return;
    }
    if (step == 3) {
        // Forty drilled plates in a stack, each with a pin, cut down the middle.
        frameOn({0, 0, 60}, 330.0f);
        for (int i = 0; i < 20; ++i) {
            char name[32];
            std::snprintf(name, sizeof name, "Plate %02d", i + 1);
            const ObjectId p = box({60, 40, 5}, {0, 0, 2.5 + i * 6.0}, name);
            drill(p, {0, 0, 5.0 + i * 6.0}, 6.0);
            std::snprintf(name, sizeof name, "Pin %02d", i + 1);
            cylinder(2.8, 5.5, {0, 0, 2.75 + i * 6.0}, name);
        }
        toggleSection();
        stepSection();
        return;
    }
}

void Application::stepSectionDemo() {
    if (sectionDemo_ != 2) return;
    ++sectionDemoFrame_;
    if (camera_.viewportW <= 0) return;
    // Onto the arrow, press, and pull it back 22 mm into the plate.
    const AABB b = scene_.bounds();
    if (!b.valid()) return;
    const Vec3 centre{30.0, b.center().y, b.center().z};
    const Real px = static_cast<Real>(camera_.pixelWorldSize(centre));
    Vec2 a{}, tip{};
    if (!camera_.projectToPixel(centre, a) || !camera_.projectToPixel(centre + Vec3{kHandlePx * px, 0, 0}, tip)) return;
    const Vec2 grab = a + (tip - a) * 0.6;
    // Done by the sixth frame, which is the one a screenshot takes.
    if (sectionDemoFrame_ == 2) {
        mouseOverride_ = grab;
    } else if (sectionDemoFrame_ == 3) {
        section_.dragging = section_.hoverHandle;
        section_.dragFrom = section_.offset;
        section_.dragStartPx = grab;
        std::fprintf(stderr, "[section-demo] 2: the arrow lit under the pointer=%d\n", section_.hoverHandle ? 1 : 0);
    } else if (sectionDemoFrame_ == 4 || sectionDemoFrame_ == 5) {
        // 22 mm back along the arrow, as the arrow measures it on the screen.
        const Vec2 along = (tip - a) / length(tip - a);
        const Real perMm = length(tip - a) / (kHandlePx * px);
        mouseOverride_ = grab - along * (22.0 * perMm * (sectionDemoFrame_ - 3) / 2.0);
    } else if (sectionDemoFrame_ == 6) {
        section_.dragging = false;
        std::fprintf(stderr, "[section-demo] 2: dragged to offset=%.3f (expected 8.000)  agrees=%d\n",
                     section_.offset, std::fabs(section_.offset - 8.0) < 0.05 ? 1 : 0);
    }
}

} // namespace tg

