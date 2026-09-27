// Tangent - the application's side of assemblies: groups made and taken
// apart, rows moved between them, and the joint tool started and fed.
//
// Everything here changes how parts relate rather than what they are, so each
// edit is an AssemblyCommand: the whole of the groups and joints either side,
// which is small.
#include "app/application.h"
#include "core/units.h"

#include "core/palette.h"
#include "ui/command_panel.h"
#include "ui/widgets.h"

#include <imgui.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <future>
#include <memory>
#include <unordered_map>

namespace tg {

void Application::applyAssemblyActions() {
    UiActions& a = ui_.actions;

    if (a.pickGroup != kNoGroup) {
        const std::vector<ObjectId> members = groupMembers(scene_, a.pickGroup);
        scene_.clearElementSelection();
        if (!a.pickGroupAdditive) scene_.clearSelection();
        for (ObjectId id : members) scene_.select(id, /*additive=*/true);
    }

    if (a.groupSelected || a.ungroupSelected || a.moveNodeRequested || a.joint || a.editJoint)
        dismissSettled();

    if (a.groupSelected) {
        if (scene_.selection().empty()) {
            setNotice("Select the parts to group first");
        } else {
            const AssemblyState before = assemblyState(scene_);
            const size_t n = selectionNodes(scene_).size();
            const GroupId g = groupSelection(scene_);
            if (g != kNoGroup) {
                undo_.push(std::make_unique<AssemblyCommand>(before, assemblyState(scene_), "Group"));
                const Group* grp = scene_.assembly().group(g);
                setNotice("Grouped " + std::to_string(n) + (n == 1 ? " thing" : " things") + " as " +
                          (grp ? grp->name : std::string("a group")));
            }
        }
    }

    if (a.ungroupSelected) {
        // A whole group selected comes apart; a part picked out of a group
        // comes out of it, up one level.
        const AssemblyState before = assemblyState(scene_);
        bool changed = false;
        for (const OutlinerNode& n : selectionNodes(scene_)) {
            if (n.isGroup) {
                changed |= ungroup(scene_, n.group);
            } else if (const SceneObject* o = scene_.find(n.object); o && o->group != kNoGroup) {
                const Group* g = scene_.assembly().group(o->group);
                changed |= moveToGroup(scene_, n, g ? g->parent : kNoGroup);
            }
        }
        pruneEmptyGroups(scene_);
        if (changed) undo_.push(std::make_unique<AssemblyCommand>(before, assemblyState(scene_), "Ungroup"));
        else         setNotice("Nothing selected is in a group");
    }

    if (a.moveNodeRequested) {
        const AssemblyState before = assemblyState(scene_);
        const OutlinerNode n = a.moveNode;
        const GroupId was = n.isGroup ? (scene_.assembly().group(n.group) ? scene_.assembly().group(n.group)->parent
                                                                           : kNoGroup)
                                      : (scene_.find(n.object) ? scene_.find(n.object)->group : kNoGroup);
        if (was != a.moveInto && !(n.isGroup && n.group == a.moveInto)) {
            if (moveToGroup(scene_, n, a.moveInto)) {
                pruneEmptyGroups(scene_);
                undo_.push(std::make_unique<AssemblyCommand>(before, assemblyState(scene_),
                                                             a.moveInto == kNoGroup ? "Out of Group" : "Into Group"));
            } else if (n.isGroup) {
                setNotice("A group cannot go inside itself");
            }
        }
    }

    if (a.clearance) toggleClearance();
    if (a.explode) toggleExplode();
    if (a.joint) beginJoint();
    if (a.editJoint) {
        if (editToolActive()) {
            setNotice("Finish the current operation first");
        } else {
            jointTool_.edit(scene_, a.editJoint);
        }
    }
}

void Application::beginJoint() {
    if (editToolActive()) { setNotice("Finish the current operation first"); return; }
    const size_t bodies = std::count_if(scene_.objects().begin(), scene_.objects().end(),
                                        [](const auto& o) { return !o->body.empty(); });
    if (bodies < 2) { setNotice("A joint is between two parts, and there is only one"); return; }
    dismissSettled();
    measure_.end();
    jointTool_.start(scene_);
}

// ---------------------------------------------------------------------------
// Clearance
// ---------------------------------------------------------------------------

bool Application::commandCornerTaken() const {
    return settled_ != Settled::None || editToolActive() || jointTool_.active() || tool_.active() ||
           createTool_.active() || createTool_.applied() || sketchTool_.active() || sketchTool_.applied() ||
           measure_.active();
}

void Application::toggleClearance() {
    if (clearance_.open) {
        clearance_.open = false;
        if (clearance_.cancel) clearance_.cancel->store(true);
        return;
    }
    const size_t bodies = std::count_if(scene_.objects().begin(), scene_.objects().end(),
                                        [](const auto& o) { return o->visible && !o->body.empty(); });
    if (bodies < 2) { setNotice("Clearance is between parts, and there are not two to measure"); return; }
    endExplode();
    clearance_.open = true;
    clearance_.focus = -1;
    // Two or more picked: those. Otherwise everything on show.
    size_t picked = 0;
    for (ObjectId id : scene_.selection())
        if (const SceneObject* o = scene_.find(id); o && !o->body.empty()) ++picked;
    clearance_.selectedOnly = picked >= 2;
    clearance_.shownKey = 0;
}

std::vector<ObjectId> Application::clearanceBodies() const {
    std::vector<ObjectId> out;
    if (clearance_.selectedOnly) {
        for (ObjectId id : scene_.selection())
            if (const SceneObject* o = scene_.find(id); o && !o->body.empty()) out.push_back(id);
        if (out.size() >= 2) return out;
        out.clear();
    }
    for (const auto& o : scene_.objects())
        if (o->visible && !o->body.empty()) out.push_back(o->id);
    return out;
}

uint64_t Application::clearanceKey() const {
    uint64_t h = 1469598103934665603ULL;
    auto mix = [&](const void* p, size_t n) {
        const unsigned char* b = static_cast<const unsigned char*>(p);
        for (size_t i = 0; i < n; ++i) { h ^= b[i]; h *= 1099511628211ULL; }
    };
    mix(&clearance_.required, sizeof clearance_.required);
    mix(&clearance_.sweepJoint, sizeof clearance_.sweepJoint);
    if (const Joint* j = scene_.assembly().joint(clearance_.sweepJoint)) {
        mix(&j->kind, sizeof j->kind);
        mix(&j->limited, sizeof j->limited);
        mix(&j->lo, sizeof j->lo);
        mix(&j->hi, sizeof j->hi);
        mix(&j->slideAxis, sizeof j->slideAxis);
    }
    for (ObjectId id : clearanceBodies()) {
        const SceneObject* o = scene_.find(id);
        mix(&id, sizeof id);
        mix(&o->geometryVersion, sizeof o->geometryVersion);
        mix(&o->transform.position, sizeof o->transform.position);
        mix(&o->transform.rotation, sizeof o->transform.rotation);
    }
    return h;
}

void Application::startClearanceJob(uint64_t key) {
    ClearanceRequest req;
    req.required = clearance_.required;
    req.deviation = clearanceDeviation(clearance_.required);
    std::vector<ObjectId> moving;
    const Joint* sweep = scene_.assembly().joint(clearance_.sweepJoint);
    if (sweep && sweep->hasMotion()) moving = jointUnit(scene_, *sweep);
    clearance_.jobVersions.clear();
    for (ObjectId id : clearanceBodies()) {
        const SceneObject* o = scene_.find(id);
        ClearanceBody b;
        b.id = id;
        b.name = o->name;
        b.model = o->modelMatrix();
        b.moving = std::find(moving.begin(), moving.end(), id) != moving.end();
        auto it = clearance_.meshes.find(id);
        if (it != clearance_.meshes.end() && it->second.version == o->geometryVersion &&
            it->second.deviation == req.deviation)
            b.mesh = it->second.mesh;
        else
            b.body = o->body;          // a handle; the worker takes its own copy
        clearance_.jobVersions[id] = o->geometryVersion;
        req.bodies.push_back(std::move(b));
    }
    // Through the joint's motion: its range, or all the way round -- or a
    // part's width either way for a slide -- in 36 steps, each a move of the
    // parts it places from where they are now.
    if (sweep && sweep->hasMotion() && !moving.empty()) {
        const bool slider = sweep->kind == JointKind::Slider;
        Real size = 20.0;
        AABB box;
        for (ObjectId id : clearanceBodies()) box.expand(scene_.find(id)->worldBounds());
        if (box.valid()) size = length(box.size());
        const bool bounded = sweep->limited && sweep->hi > sweep->lo;
        const Real lo = bounded ? sweep->lo : (slider ? -size * 0.5 : -kPi);
        const Real hi = bounded ? sweep->hi : (slider ? size * 0.5 : kPi);
        const int n = 36;
        for (int k = 0; k <= n; ++k) {
            Joint at = *sweep;
            at.limited = false;
            const Real v = lo + (hi - lo) * k / n;
            if (slider) at.travel = v;
            else        at.turn = v;
            Rigid d;
            if (!jointDelta(scene_, at, d)) break;
            req.samples.push_back(d);
            req.sampleValues.push_back(slider ? v : v * kRad2Deg);
        }
    }
    auto cancel = std::make_shared<std::atomic<bool>>(false);
    clearance_.cancel = cancel;
    clearance_.jobKey = key;
    clearance_.running = true;
    clearance_.job = std::async(std::launch::async, [req = std::move(req), cancel]() mutable {
        // Each body copied on this thread, not the frame's: meshing writes
        // into the shape, and a copy of a large one is not free.
        for (ClearanceBody& b : req.bodies)
            if (!b.mesh) b.body = b.body.detached();
        return checkClearance(std::move(req), cancel.get());
    });
}

void Application::stepClearance() {
    auto ready = [](std::future<ClearanceResult>& f) {
        return f.valid() && f.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    };
    // Jobs let go of are kept until they finish: a future from std::async
    // waits for its task when it is destroyed, and that would be the frame.
    clearance_.retired.erase(std::remove_if(clearance_.retired.begin(), clearance_.retired.end(), ready),
                             clearance_.retired.end());
    if (!clearance_.open) {
        if (clearance_.running) {
            if (clearance_.cancel) clearance_.cancel->store(true);
            clearance_.retired.push_back(std::move(clearance_.job));
            clearance_.running = false;
        }
        return;
    }
    if (clearance_.running && ready(clearance_.job)) {
        ClearanceResult r = clearance_.job.get();
        clearance_.running = false;
        if (!r.cancelled) {
            for (auto& [id, mesh] : r.meshes) {
                auto v = clearance_.jobVersions.find(id);
                if (v != clearance_.jobVersions.end())
                    clearance_.meshes[id] = {v->second, r.deviation, mesh};
            }
            r.meshes.clear();
            clearance_.shown = std::move(r);
            clearance_.shownKey = clearance_.jobKey;
            if (clearance_.focus >= static_cast<int>(clearance_.shown.pairs.size())) clearance_.focus = -1;
        }
    }
    // A mesh for a body that has changed is no use to anyone.
    for (auto it = clearance_.meshes.begin(); it != clearance_.meshes.end();) {
        const SceneObject* o = scene_.find(it->first);
        if (!o || o->geometryVersion != it->second.version) it = clearance_.meshes.erase(it);
        else ++it;
    }

    const uint64_t key = clearanceKey();
    if (key == clearance_.shownKey || (clearance_.running && clearance_.jobKey == key)) return;
    // Once what is measured has held still for a moment -- not on every frame
    // of a drag, which would start a check each frame and finish none -- and
    // at once the first time, so opening the panel is answered straight away.
    if (key != clearance_.wantKey) { clearance_.wantKey = key; clearance_.stableFor = 0.0f; }
    else                           clearance_.stableFor += lastDt_;
    if (clearance_.shownKey != 0 && clearance_.stableFor < 0.08f) return;
    if (clearance_.running) {
        clearance_.cancel->store(true);
        clearance_.retired.push_back(std::move(clearance_.job));
        clearance_.running = false;
    }
    startClearanceJob(key);
}

namespace {

std::string gapText(const ClearancePair& p) {
    char b[64];
    if (p.overlap)       std::snprintf(b, sizeof b, "overlap");
    else if (p.touching) std::snprintf(b, sizeof b, "touching");
    else                 std::snprintf(b, sizeof b, "%s", units::length(p.gap).c_str());
    return b;
}

} // namespace

void Application::drawClearanceOverlay() {
    if (!clearance_.open || !clearance_.shown.ok) return;
    const ClearanceResult& r = clearance_.shown;
    // Amber where it is closer than asked, red where the parts run into each
    // other: on the surface, as the print problems are.
    // Uploaded once per result and drawn from the GPU after that: the marks
    // on a busy assembly are tens of thousands of triangles, which rebuilt
    // every frame were most of the frame.
    const Vec4 close{1.0f, 0.72f, 0.22f, 0.55f};
    const Vec4 over{1.0f, 0.25f, 0.2f, 0.62f};
    if (clearance_.uploaded != clearance_.shownKey) {
        std::vector<Vec3> closeTris, overTris;
        for (const ClearanceResult::Marks& m : r.marks) {
            closeTris.insert(closeTris.end(), m.close.begin(), m.close.end());
            overTris.insert(overTris.end(), m.overlap.begin(), m.overlap.end());
        }
        renderer_.setStatic(0, clearance_.shownKey, closeTris, close);
        renderer_.setStatic(1, clearance_.shownKey, overTris, over);
        clearance_.uploaded = clearance_.shownKey;
    }
    renderer_.showStatic(0);
    renderer_.showStatic(1);
    // Through a motion, the moving parts where it was tightest, see-through:
    // the marks are where they were then, not where they rest.
    if (const Joint* j = scene_.assembly().joint(clearance_.sweepJoint); j && r.worstSample >= 0) {
        const Mat4 move = translate(r.worstMove.t) * toMat4(r.worstMove.q);
        const Vec4 edge = toVec4(palette::kBrand, 0.85f);
        const Vec4 fill = toVec4(palette::kBrand, 0.10f);
        for (ObjectId id : jointUnit(scene_, *j)) {
            const SceneObject* o = scene_.find(id);
            if (!o || !o->visible) continue;
            const Mat4 m = move * o->modelMatrix();
            const RenderMesh& rm = o->render;
            if (rm.triangles.size() / 3 <= 40000)
                for (size_t i = 0; i + 2 < rm.triangles.size(); i += 3)
                    renderer_.addTriangle(transformPoint(m, rm.positions[rm.triangles[i]]),
                                          transformPoint(m, rm.positions[rm.triangles[i + 1]]),
                                          transformPoint(m, rm.positions[rm.triangles[i + 2]]), fill);
            for (size_t i = 0; i + 1 < rm.edgeLines.size(); i += 2)
                renderer_.addLine(transformPoint(m, rm.positions[rm.edgeLines[i]]),
                                  transformPoint(m, rm.positions[rm.edgeLines[i + 1]]), edge);
        }
    }

    // The gap itself, between the two points it is measured between, for the
    // pair picked in the list or the tightest one.
    if (r.pairs.empty()) return;
    const int at = clearance_.focus >= 0 ? clearance_.focus : 0;
    const ClearancePair& p = r.pairs[static_cast<size_t>(at)];
    const bool bad = p.overlap || p.gap < r.required;
    const Vec4 c = bad ? Vec4{1.0f, 0.45f, 0.3f, 1.0f} : Vec4{0.45f, 0.85f, 0.6f, 1.0f};
    if (length(p.pb - p.pa) > 1e-6) renderer_.addFrontLine(camera_, p.pa, p.pb, c, 2.4);
    const Real s = camera_.pixelWorldSize(p.pa);
    for (const Vec3& q : {p.pa, p.pb}) {
        const Vec3 x = camera_.right() * (s * 4.0), y = camera_.up() * (s * 4.0);
        renderer_.addFrontTriangle(q - x - y, q + x - y, q + x + y, c);
        renderer_.addFrontTriangle(q - x - y, q + x + y, q - x + y, c);
    }
}

void Application::drawClearancePanel() {
    ui_.clearanceOpen = clearance_.open;
    if (!clearance_.open) return;
    const ClearanceResult& r = clearance_.shown;
    const bool fresh = clearance_.shownKey == clearanceKey();
    auto nameOf = [&](ObjectId id) {
        const SceneObject* o = scene_.find(id);
        return o ? o->name : std::string("?");
    };
    size_t over = 0, tight = 0, touch = 0;
    for (const ClearancePair& p : r.pairs) {
        if (p.overlap) ++over;
        else if (p.touching) ++touch;
        else if (p.gap < r.required) ++tight;
    }

    // Another operation has the corner: the answer in a line at the foot of
    // the view, still live.
    if (commandCornerTaken()) {
        if (!r.ok) return;
        std::string line = "Clearance  ";
        if (r.pairs.empty()) {
            char b[64];
            std::snprintf(b, sizeof b, "clear by more than %s", units::length(r.limit, 1).c_str());
            line += b;
        } else {
            const ClearancePair& p = r.pairs.front();
            line += gapText(p) + "  " + nameOf(p.a) + " / " + nameOf(p.b);
        }
        if (!fresh || clearance_.running) line += "  ...";
        const bool bad = over + tight > 0;
        const float x = viewRect_.x + 16.0f;
        const float y = viewRect_.y + viewRect_.h - 44.0f;
        drawReadout(line, x, y, bad);
        return;
    }

    char context[32];
    std::snprintf(context, sizeof context, "%zu parts", clearanceBodies().size());
    if (!ui::beginCommand("##clearance", "Clearance", Glyph::Clearance, context)) return;

    {
        size_t picked = 0;
        for (ObjectId id : scene_.selection())
            if (const SceneObject* o = scene_.find(id); o && !o->body.empty()) ++picked;
        ui::commandRow("Between");
        if (ui::pillButton("Everything shown", !clearance_.selectedOnly)) clearance_.selectedOnly = false;
        ImGui::SameLine(0.0f, 3.0f);
        char sel[32];
        std::snprintf(sel, sizeof sel, "Selected (%zu)", picked);
        if (ui::pillButton(sel, clearance_.selectedOnly, ImVec2(0, 0), picked >= 2)) clearance_.selectedOnly = true;
        if (picked < 2) ui::hoverTip("Select two or more parts to measure only those");
    }
    {
        const bool editing = clearance_.typing;
        const ui::NumberEdit e = ui::commandNumber("Needs", clearance_.required, "mm", editing, editing,
                                                   clearance_.typed.c_str(), 0.0, 1.0);
        if (e.clicked) { clearance_.typing = true; clearance_.typed.clear(); }
        if (e.dragged) { clearance_.required = std::max(0.0, e.value); clearance_.typing = false; }
        struct Fit { const char* label; Real mm; const char* tip; };
        static const Fit kFits[4] = {{"0.1", 0.1, "Snug: a press or a close sliding fit on a good printer"},
                                     {"0.2", 0.2, "The usual gap for parts that must come apart"},
                                     {"0.3", 0.3, "Loose: parts that move freely"},
                                     {"0.5", 0.5, "Very loose, for a coarse nozzle or an uncalibrated printer"}};
        ui::commandRow("");
        for (int i = 0; i < 4; ++i) {
            if (i) ImGui::SameLine(0.0f, 3.0f);
            // The presets are printer gaps in millimetres; in another unit
            // they are shown converted, and say which unit they are in.
            const std::string label = units::current() == units::Length::Millimetre
                                          ? std::string(kFits[i].label) : units::number(kFits[i].mm, 3);
            ImGui::PushID(i);
            const bool picked = ui::pillButton(label.c_str(), std::fabs(clearance_.required - kFits[i].mm) < 1e-9);
            ImGui::PopID();
            if (picked)
                clearance_.required = kFits[i].mm;
            ui::hoverTip(kFits[i].tip);
        }
    }
    {
        // Through a joint's motion, when there is one that moves.
        std::vector<const Joint*> moving;
        for (const Joint& j : scene_.assembly().joints) if (j.hasMotion()) moving.push_back(&j);
        if (!moving.empty()) {
            ui::commandRow("Through");
            if (ui::pillButton("As it stands", clearance_.sweepJoint == 0)) clearance_.sweepJoint = 0;
            ui::hoverTip("The parts where they are now");
            for (const Joint* j : moving) {
                ui::commandNextPill(j->name.c_str(), 3.0f);
                ImGui::PushID(static_cast<int>(j->id));
                if (ui::pillButton(j->name.c_str(), clearance_.sweepJoint == j->id)) clearance_.sweepJoint = j->id;
                ui::hoverTip("Through the whole of this joint's motion -- its limits, or all the way round");
                ImGui::PopID();
            }
        }
        if (clearance_.sweepJoint && !scene_.assembly().joint(clearance_.sweepJoint)) clearance_.sweepJoint = 0;
    }

    // What it came to.
    if (r.ok) {
        char summary[160];
        if (r.pairs.empty()) {
            std::snprintf(summary, sizeof summary, "Every part is clear by more than %s", units::length(r.limit, 1).c_str());
        } else if (over + tight + touch == 0) {
            std::snprintf(summary, sizeof summary, "Every pair keeps %s", units::length(r.required).c_str());
        } else {
            std::string s;
            auto add = [&](size_t n, const char* what) {
                if (!n) return;
                if (!s.empty()) s += ", ";
                s += std::to_string(n) + " " + what;
            };
            add(over, "overlapping");
            add(tight, "too close");
            add(touch, "touching");
            std::snprintf(summary, sizeof summary, "%s", s.c_str());
        }
        ui::commandValue("Result", summary);
        // Through a motion: where in it the tightest moment came.
        if (clearance_.sweepJoint && !r.pairs.empty() && r.pairs.front().sample >= 0 &&
            static_cast<size_t>(r.pairs.front().sample) < r.sampleValues.size()) {
            const Joint* j = scene_.assembly().joint(clearance_.sweepJoint);
            const bool slider = j && j->kind == JointKind::Slider;
            const ClearancePair& p = r.pairs.front();
            char at[96];
            const Real sv = r.sampleValues[static_cast<size_t>(p.sample)];
            if (slider) std::snprintf(at, sizeof at, "%s at %s", gapText(p).c_str(), units::length(sv, 1).c_str());
            else        std::snprintf(at, sizeof at, "%s at %.1f\xC2\xB0", gapText(p).c_str(), sv);
            ui::commandValue("Tightest", at);
        }

        // The pairs, tightest first. A click frames the two parts on the gap.
        const size_t shownPairs = std::min<size_t>(r.pairs.size(), 8);
        for (size_t i = 0; i < shownPairs; ++i) {
            const ClearancePair& p = r.pairs[i];
            ImGui::PushID(static_cast<int>(i));
            const std::string label = nameOf(p.a) + " / " + nameOf(p.b);
            ui::commandRow(i == 0 ? "Pairs" : "");
            const bool bad = p.overlap || p.gap < r.required;
            const ImVec4 col = p.overlap ? ui::im(palette::kBrandHover)
                             : bad       ? ui::im(palette::kWarn)
                                         : ui::im(palette::kValid);
            const float w = ImGui::GetContentRegionAvail().x;
            const ImVec2 lo = ImGui::GetCursorScreenPos();
            if (ImGui::InvisibleButton("##pair", ImVec2(w, ImGui::GetFrameHeight()))) {
                clearance_.focus = static_cast<int>(i);
                AABB b;
                b.expand(p.pa);
                b.expand(p.pb);
                const SceneObject* oa = scene_.find(p.a);
                const SceneObject* ob = scene_.find(p.b);
                AABB both;
                if (oa) both.expand(oa->worldBounds());
                if (ob) both.expand(ob->worldBounds());
                const Real pad = both.valid() ? std::max<Real>(3.0, length(both.size()) * 0.15) : 5.0;
                b.min -= Vec3{pad, pad, pad};
                b.max += Vec3{pad, pad, pad};
                camera_.frame(b);
            }
            const bool hot = ImGui::IsItemHovered();
            ImDrawList* dl = ImGui::GetWindowDrawList();
            const ImVec2 hi(lo.x + w, lo.y + ImGui::GetFrameHeight());
            if (hot || clearance_.focus == static_cast<int>(i))
                dl->AddRectFilled(lo, hi, ui::u32(palette::kRaised), 5.0f);
            const float ty = lo.y + (ImGui::GetFrameHeight() - ImGui::GetTextLineHeight()) * 0.5f;
            dl->AddCircleFilled(ImVec2(lo.x + 8.0f, lo.y + ImGui::GetFrameHeight() * 0.5f), 3.5f,
                                ImGui::GetColorU32(col));
            const std::string gap = gapText(p);
            const float gw = ImGui::CalcTextSize(gap.c_str()).x;
            dl->PushClipRect(ImVec2(lo.x + 16.0f, lo.y), ImVec2(hi.x - gw - 12.0f, hi.y), true);
            dl->AddText(ImVec2(lo.x + 18.0f, ty), ui::u32(palette::kText), label.c_str());
            dl->PopClipRect();
            dl->AddText(ImVec2(hi.x - gw - 6.0f, ty), ImGui::GetColorU32(col), gap.c_str());
            if (hot) ImGui::SetTooltip("Click to look at where they come closest");
            ImGui::PopID();
        }
        if (r.pairs.size() > shownPairs) {
            char more[48];
            std::snprintf(more, sizeof more, "and %zu more", r.pairs.size() - shownPairs);
            ui::commandValue("", more);
        }
        if (r.partial) ui::commandRefused("Stopped part way: the parts touch over so much that measuring every "
                                          "pair would take too long. Measure fewer parts at once.");
        char how[128];
        std::snprintf(how, sizeof how, "%s  %.0f ms, to within %s", clearance_.running || !fresh ? "Measuring..."
                                                                                                : "Measured in",
                      r.ms, units::length(r.deviation, 3).c_str());
        ui::commandValue("", how);
    } else {
        ui::commandValue("Result", "Measuring...");
    }
    ui::commandHint("Where two parts come closer than the gap asked for, both are marked in amber; where they "
                    "run into each other, in red. A part that has to turn or slide against another wants the "
                    "gap all round, after printing: 0.2 mm on a well-calibrated printer.");
    ui::commandHint("Measured on the surfaces meshed to within the tolerance shown, triangle to triangle. "
                    "It keeps measuring while you move parts or turn a joint, and stays drawn until Done.");
    const int footer = ui::commandFooter("Done", true, nullptr);
    ui::endCommand();
    if (footer > 0) toggleClearance();
}

// ---------------------------------------------------------------------------
// Exploded view
// ---------------------------------------------------------------------------

void Application::endExplode() {
    if (!explode_.open && explode_.shown <= 0.0) return;
    explode_.open = false;
    explode_.shown = 0.0;
    explode_.trailLines.clear();
    for (const auto& o : scene_.objects()) scene_.place(*o);
    solveAssembly(scene_);
}

void Application::toggleExplode() {
    if (explode_.open) { explode_.open = false; return; }
    // The gaps between parts pulled apart for a picture are not the gaps
    // between the parts.
    if (clearance_.open) toggleClearance();
    const size_t bodies = std::count_if(scene_.objects().begin(), scene_.objects().end(),
                                        [](const auto& o) { return o->visible && !o->body.empty(); });
    if (bodies < 2) { setNotice("An exploded view pulls parts apart, and there are not two"); return; }
    if (editToolActive() || jointTool_.active() || tool_.active()) {
        setNotice("Finish the current operation first");
        return;
    }
    dismissSettled();
    explode_.open = true;
}

namespace {

// How far a set of parts reaches along `d`.
Real extentAlong(const Scene& scene, const std::vector<ObjectId>& ids, Vec3 d) {
    Real lo = 1e300, hi = -1e300;
    for (ObjectId id : ids) {
        const SceneObject* o = scene.find(id);
        if (!o) continue;
        const AABB b = o->worldBounds();
        if (!b.valid()) continue;
        for (int c = 0; c < 8; ++c) {
            const Vec3 p{(c & 1) ? b.max.x : b.min.x, (c & 2) ? b.max.y : b.min.y, (c & 4) ? b.max.z : b.min.z};
            lo = std::min(lo, dot(p, d));
            hi = std::max(hi, dot(p, d));
        }
    }
    return hi > lo ? hi - lo : 0.0;
}

AABB boundsOf(const Scene& scene, const std::vector<ObjectId>& ids) {
    AABB b;
    for (ObjectId id : ids)
        if (const SceneObject* o = scene.find(id)) b.expand(o->worldBounds());
    return b;
}

} // namespace

void Application::stepExplode() {
    // Anything else starting puts the parts back, at once: whatever started
    // is about to read where they are.
    if ((explode_.open || explode_.shown > 0.0) && (commandCornerTaken() || settled_ != Settled::None)) {
        endExplode();
        return;
    }
    const Real goal = explode_.open ? explode_.amount : 0.0;
    if (!explode_.open && explode_.shown <= 0.0) { explode_.trailLines.clear(); return; }
    // Eased, quickly: long enough to see which way each part goes, short
    // enough not to wait for.
    const Real rate = 1.0 - std::exp(-static_cast<Real>(lastDt_) * 14.0);
    explode_.shown += (goal - explode_.shown) * rate;
    if (std::fabs(goal - explode_.shown) < 1e-3) explode_.shown = goal;

    // Every part from where its history and its joints put it -- the
    // assembled view -- and then moved.
    for (const auto& o : scene_.objects()) scene_.place(*o);
    solveAssembly(scene_);
    explode_.trailLines.clear();
    if (explode_.shown <= 0.0) return;
    const Real k = explode_.shown;

    std::unordered_map<ObjectId, Vec3> offset;
    auto off = [&](ObjectId id) {
        auto it = offset.find(id);
        return it == offset.end() ? Vec3{} : it->second;
    };

    // Along the joints. Each joint's moving part goes out from its fixed part
    // along the joint's axis -- or, for a hinge on an edge, off the face the
    // edge was picked on -- by its own depth along that way and a little
    // more, on top of however far the part it is on went. In turn until
    // nothing changes, which for a tree of joints is as many passes as it is
    // deep.
    std::vector<ObjectId> jointed;
    std::unordered_map<uint32_t, std::pair<Vec3, Vec3>> jointTrail;   // from, and by how much
    const auto& joints = scene_.assembly().joints;
    for (size_t pass = 0; pass <= joints.size(); ++pass) {
        bool changed = false;
        for (const Joint& j : joints) {
            const SceneObject* f = scene_.find(j.fixed.object);
            if (!f || !j.problem.empty()) continue;
            const std::vector<ObjectId> unit = jointUnit(scene_, j);
            if (unit.empty()) continue;
            const Mat4 fm = f->modelMatrix();
            // A hinge's axis runs along its edge: sliding along it would pull
            // the lid sideways off the box rather than up off it.
            Vec3 d = normalize(transformVector(fm, j.fixed.straightEdge ? j.fixed.frame.x : j.fixed.frame.z));
            const AABB mb = boundsOf(scene_, unit), fb = f->worldBounds();
            if (mb.valid() && fb.valid() && dot(d, (mb.min + mb.max) * 0.5 - (fb.min + fb.max) * 0.5) < 0.0)
                d = d * Real(-1);
            const Real reach = extentAlong(scene_, unit, d);
            const Vec3 by = off(j.fixed.object) + d * (k * (reach * 1.15 + 4.0));
            for (ObjectId id : unit) {
                if (length(off(id) - by) > 1e-9) { offset[id] = by; changed = true; }
                if (std::find(jointed.begin(), jointed.end(), id) == jointed.end()) jointed.push_back(id);
            }
            // The trail runs from where the joint is on the fixed part to where
            // that point went with the moving one: out of the hole, up off the
            // face -- not from the middle of a part, where it would be hidden.
            if (pass == 0 || changed) {
                jointTrail[j.id] = {transformPoint(fm, j.fixed.frame.origin) + off(j.fixed.object),
                                    by - off(j.fixed.object)};
            }
            if (std::find(jointed.begin(), jointed.end(), j.fixed.object) == jointed.end())
                jointed.push_back(j.fixed.object);
        }
        if (!changed) break;
    }

    // Everything no joint touches: out from the middle of the whole, each
    // group as one, by about half its own size.
    std::vector<ObjectId> shown;
    for (const auto& o : scene_.objects()) if (o->visible && !o->body.empty()) shown.push_back(o->id);
    const AABB all = boundsOf(scene_, shown);
    if (all.valid()) {
        const Vec3 centre = (all.min + all.max) * 0.5;
        std::unordered_map<uint64_t, std::vector<ObjectId>> nodes;
        for (ObjectId id : shown) {
            if (std::find(jointed.begin(), jointed.end(), id) != jointed.end()) continue;
            const SceneObject* o = scene_.find(id);
            const std::vector<GroupId> chain = groupChain(scene_, o->group);
            const uint64_t key = chain.empty() ? id : (uint64_t{1} << 40) | chain.back();
            nodes[key].push_back(id);
        }
        for (const auto& [key, ids] : nodes) {
            // A group some of whose parts are jointed stays with them.
            if (key >> 40) {
                const std::vector<ObjectId> members = groupMembers(scene_, static_cast<GroupId>(key & 0xffffffffu));
                if (members.size() != ids.size()) continue;
            }
            const AABB b = boundsOf(scene_, ids);
            if (!b.valid()) continue;
            Vec3 d = (b.min + b.max) * 0.5 - centre;
            if (length(d) < 1e-6) continue;             // at the middle: it stays
            d = normalize(d);
            const Vec3 by = d * (k * (extentAlong(scene_, ids, d) * 0.6 + 4.0));
            for (ObjectId id : ids) offset[id] = by;
        }
    }

    for (const auto& [id, fromBy] : jointTrail)
        if (length(fromBy.second) > 1e-9) explode_.trailLines.push_back({fromBy.first, fromBy.first + fromBy.second});
    for (const auto& [id, by] : offset) {
        SceneObject* o = scene_.find(id);
        if (!o || length(by) < 1e-9) continue;
        const bool viaJoint = std::find(jointed.begin(), jointed.end(), id) != jointed.end();
        if (!viaJoint && o->worldBounds().valid()) {
            const Vec3 from = (o->worldBounds().min + o->worldBounds().max) * 0.5;
            explode_.trailLines.push_back({from, from + by});
        }
        o->transform.position += by;
    }
}

void Application::drawExplodeOverlay() {
    if (!explode_.trails || explode_.shown <= 0.0) return;
    // Dashed, in the model rather than over it, so a trail behind a part is
    // hidden by the part the way it would be on paper.
    const Vec4 c = toVec4(palette::kBrand, 0.8f);
    for (const ExplodeState::Trail& t : explode_.trailLines) {
        const Vec3 d = t.to - t.from;
        const Real len = length(d);
        if (len < 1e-6) continue;
        const Vec3 u = d / len;
        const Real dash = std::max<Real>(camera_.pixelWorldSize(t.from) * 7.0, 0.3);
        for (Real s = 0.0; s < len; s += dash * 2.0)
            renderer_.addLine(t.from + u * s, t.from + u * std::min(len, s + dash), c);
    }
}

void Application::drawExplodePanel() {
    ui_.explodeOpen = explode_.open;
    if (!explode_.open || commandCornerTaken()) return;
    if (!ui::beginCommand("##explode", "Exploded View", Glyph::Explode)) return;
    const bool editing = explode_.typing;
    const ui::NumberEdit e = ui::commandNumber("Apart", explode_.amount * 100.0, "%", editing, editing,
                                               explode_.typed.c_str(), 0.0, 200.0);
    if (e.clicked) { explode_.typing = true; explode_.typed.clear(); }
    if (e.dragged) {
        explode_.amount = std::max(0.0, e.value / 100.0);
        explode_.shown = explode_.amount;          // a drag is followed at once
        explode_.typing = false;
    }
    ui::commandRow("");
    if (ui::pillButton("Trails", explode_.trails)) explode_.trails = !explode_.trails;
    ui::hoverTip("Dashed lines from each part back to where it goes");
    size_t joints = 0;
    for (const Joint& j : scene_.assembly().joints) if (j.problem.empty()) ++joints;
    char how[96];
    if (joints) std::snprintf(how, sizeof how, "Along %zu joint%s, the rest out from the middle", joints,
                              joints == 1 ? "" : "s");
    else        std::snprintf(how, sizeof how, "Out from the middle: join parts to pull them apart along their joints");
    ui::commandValue("How", how);
    ui::commandHint("A view: nothing about the parts changes, and it closes when anything else starts. A part "
                    "joined to another comes away along the joint -- a pin out of its hole, a lid up off its "
                    "box -- and what is joined to it comes with it and further again.");
    const int footer = ui::commandFooter("Done", true, nullptr);
    ui::endCommand();
    if (footer > 0) explode_.open = false;
}

// ---------------------------------------------------------------------------
// Demos
// ---------------------------------------------------------------------------

namespace {

ObjectId addBox(Scene& scene, Vec3 size, Vec3 at, const char* name) {
    PrimitiveSpec spec;
    spec.kind = PrimitiveKind::Box;
    spec.box = {size.x, size.y, size.z};
    const ObjectId id = scene.addPrimitive(PrimitiveKind::Box, spec, at);
    if (SceneObject* o = scene.find(id)) o->name = name;
    return id;
}

ObjectId addCylinder(Scene& scene, Real radius, Real height, Vec3 at, const char* name) {
    PrimitiveSpec spec;
    spec.kind = PrimitiveKind::Cylinder;
    spec.cylinder.radius = radius;
    spec.cylinder.height = height;
    const ObjectId id = scene.addPrimitive(PrimitiveKind::Cylinder, spec, at);
    if (SceneObject* o = scene.find(id)) o->name = name;
    return id;
}

// The face of a body that looks along `n`, in its own space.
Index faceFacing(const Scene& scene, ObjectId id, Vec3 n) {
    const SceneObject* o = scene.find(id);
    std::vector<FaceId> fs;
    if (o) o->body.allFaces(fs);
    for (FaceId f : fs)
        if (o->body.faceKind(f) == SurfaceKind::Plane && dot(normalize(o->body.faceNormal(f)), n) > 0.999) return f;
    return kInvalid;
}

// The straight edge of a body whose midpoint is nearest `p`, in its own space.
Index edgeNear(const Scene& scene, ObjectId id, Vec3 p) {
    const SceneObject* o = scene.find(id);
    std::vector<EdgeId> es;
    if (o) o->body.allEdges(es);
    Index best = kInvalid;
    Real bestD = 1e300;
    for (EdgeId e : es) {
        const Real d = length(o->body.edgeMidpoint(e) - p);
        if (d < bestD) { bestD = d; best = e; }
    }
    return best;
}

// The circular edge of a body centred nearest `p`.
Index rimNear(const Scene& scene, ObjectId id, Vec3 p) {
    const SceneObject* o = scene.find(id);
    std::vector<EdgeId> es;
    if (o) o->body.allEdges(es);
    Index best = kInvalid;
    Real bestD = 1e300;
    for (EdgeId e : es) {
        Vec3 c, axis;
        Real r;
        if (!o->body.edgeCircle(e, c, axis, r)) continue;
        if (length(c - p) < bestD) { bestD = length(c - p); best = e; }
    }
    return best;
}

Vec3 worldOf(const Scene& scene, ObjectId id, Vec3 local) {
    const SceneObject* o = scene.find(id);
    return o ? transformPoint(o->modelMatrix(), local) : Vec3{};
}

} // namespace

void Application::setupAssemblyDemo() {
    scene_.clear();
    undo_.clear();
    camera_.yaw = 0.75f;
    camera_.pitch = 0.45f;
    camera_.distance = 230.0f;
    camera_.target = {10, 0, 12};
    camera_.snapToGoal();
    const int step = assemblyDemo_;
    // What the demo is about, put below and to the right of the panel in the
    // view's top left: the camera looks at a point up and to the left of it.
    auto frameOn = [&](Vec3 subject, float dist) {
        camera_.distance = dist;
        camera_.target = subject;
        camera_.target = subject - camera_.right() * (dist * 0.05f) + camera_.up() * (dist * 0.12f);
        camera_.animateTo(camera_.target, dist, camera_.yaw, camera_.pitch);
        camera_.snapToGoal();
        fixedCamera_ = true;
    };

    if (step == 1 || step == 2) {
        frameOn(step == 1 ? Vec3{25, 0, 10} : Vec3{0, -8, 24}, step == 1 ? 210.0f : 190.0f);
        const ObjectId base = addBox(scene_, {40, 30, 20}, {0, 0, 10}, "Box");
        const ObjectId lid = addBox(scene_, {40, 30, 4}, {60, 0, 2}, "Lid");
        jointTool_.start(scene_);
        if (step == 1) {
            // The lid's underside picked; the pointer goes to the box's top a
            // few frames in, once the view has a size to aim with.
            jointTool_.pick(scene_, undo_, lid, JointAt::Face, faceFacing(scene_, lid, {0, 0, -1}),
                            faceFacing(scene_, lid, {0, 0, -1}));
            std::fprintf(stderr, "[assembly-demo] 1: first pick made=%d\n", jointTool_.picking() ? 1 : 0);
            return;
        }
        // Hinged along the back: the lid's back lower edge on the box's back
        // upper edge.
        jointTool_.pick(scene_, undo_, lid, JointAt::Edge, edgeNear(scene_, lid, {0, -15, -2}),
                        faceFacing(scene_, lid, {0, 0, -1}));
        jointTool_.pick(scene_, undo_, base, JointAt::Edge, edgeNear(scene_, base, {0, -15, 10}),
                        faceFacing(scene_, base, {0, 0, 1}));
        const Joint* j = scene_.assembly().joints.empty() ? nullptr : &scene_.assembly().joints.front();
        const Vec3 closed = scene_.find(lid)->transform.position;
        const bool shut = j && length(closed - Vec3{0, 0, 22}) < 1e-6;
        // Opened: whichever way the edges happened to run, one way of turning
        // lifts the lid and the other drives it into the box. The panel's
        // limits go the way that opens.
        jointTool_.setMotion(scene_, undo_, 70.0 * kDeg2Rad);
        const bool up = scene_.find(lid)->transform.position.z > 22.0;
        const Real sign = up ? 1.0 : -1.0;
        if (Joint* jj = scene_.assembly().joints.empty() ? nullptr : &scene_.assembly().joints.front()) {
            jj->limited = true;
            jj->lo = std::min(0.0, sign * 110.0 * kDeg2Rad);
            jj->hi = std::max(0.0, sign * 110.0 * kDeg2Rad);
        }
        jointTool_.setMotion(scene_, undo_, sign * 70.0 * kDeg2Rad);
        // The hinge line is the box's back top edge, y = -15, z = 20: the lid's
        // centre stays as far from it as it was shut, and swings up off the box.
        const Vec3 c = scene_.find(lid)->transform.position;
        const Real r = std::hypot(c.y + 15.0, c.z - 20.0);
        const Real expectR = std::hypot(15.0, 2.0);
        const Real angle = std::atan2(c.z - 20.0, c.y + 15.0) - std::atan2(2.0, 15.0);
        const bool agrees = shut && std::fabs(r - expectR) < 1e-6 && std::fabs(std::fabs(angle) - 70.0 * kDeg2Rad) < 1e-6 &&
                            c.z > 22.0;
        std::fprintf(stderr, "[assembly-demo] 2: %s, shut at (%.2f, %.2f, %.2f), opened 70: centre %.3f from the "
                     "hinge (arithmetic says %.3f), turned %.2f deg, agrees=%d\n",
                     j ? jointKindName(j->kind) : "no joint", closed.x, closed.y, closed.z, r, expectR,
                     std::fabs(angle) * kRad2Deg, agrees ? 1 : 0);
        return;
    }

    if (step == 3) {
        frameOn({5, 0, 12}, 210.0f);
        const ObjectId base = addBox(scene_, {40, 30, 20}, {0, 0, 10}, "Case");
        const ObjectId lid = addBox(scene_, {40, 30, 4}, {0, 0, 22}, "Case lid");
        const ObjectId leafA = addBox(scene_, {24, 2, 16}, {45, 0, 8}, "Leaf");
        const ObjectId leafB = addBox(scene_, {24, 2, 16}, {45, 6, 8}, "Leaf");
        const ObjectId pin = addCylinder(scene_, 1.5, 26, {45, 3, 17}, "Pin");
        addBox(scene_, {10, 10, 10}, {-40, 0, 5}, "Bracket");
        scene_.clearSelection();
        scene_.select(leafA);
        scene_.select(leafB, true);
        scene_.select(pin, true);
        groupSelection(scene_, "Hinge");
        scene_.clearSelection();
        scene_.select(base);
        scene_.select(lid, true);
        groupSelection(scene_, "Case");
        // The lid hinged on the case, inside the group.
        jointTool_.start(scene_);
        jointTool_.pick(scene_, undo_, lid, JointAt::Edge, edgeNear(scene_, lid, {0, -15, -2}),
                        faceFacing(scene_, lid, {0, 0, -1}));
        jointTool_.pick(scene_, undo_, base, JointAt::Edge, edgeNear(scene_, base, {0, -15, 10}),
                        faceFacing(scene_, base, {0, 0, 1}));
        jointTool_.finish(undo_);
        // The Hinge group selected: the inspector shows the group.
        scene_.clearSelection();
        for (ObjectId id : groupMembers(scene_, scene_.assembly().groups.front().id)) scene_.select(id, true);
        const std::vector<OutlinerNode> nodes = selectionNodes(scene_);
        std::fprintf(stderr, "[assembly-demo] 3: %zu groups, %zu joint, selection is %s\n",
                     scene_.assembly().groups.size(), scene_.assembly().joints.size(),
                     nodes.size() == 1 && nodes[0].isGroup ? "the Hinge group" : "not one group");
        return;
    }

    if (step == 5 || step == 6) {
        clearance_.required = 0.2;
        if (step == 5) {
            frameOn({0, 0, 8}, 170.0f);
            const ObjectId plate = addBox(scene_, {60, 40, 8}, {0, 0, 4}, "Plate");
            Feature hole;
            hole.kind = FeatureKind::Hole;
            hole.uid = scene_.takeFeatureUid();
            hole.axisPoint = {0, 0, 4};
            hole.axisDir = {0, 0, -1};
            hole.hole.diameter = 5.0;
            hole.hole.through = true;
            scene_.addFeature(plate, hole, nullptr);
            // A 4.8 mm pin, a tenth all round, through it; and a bracket half
            // a millimetre into the plate's end.
            addCylinder(scene_, 2.4, 20, {0, 0, 6}, "Pin");
            addBox(scene_, {10, 20, 16}, {34.5, 0, 8}, "Bracket");
        } else {
            // Along the hinge line, from the side, where the swing reads as an arc.
            camera_.yaw = 1.25f;
            camera_.pitch = 0.22f;
            frameOn({0, -8, 26}, 190.0f);
            const ObjectId base = addBox(scene_, {40, 30, 20}, {0, 0, 10}, "Box");
            const ObjectId lid = addBox(scene_, {40, 30, 4}, {60, 0, 2}, "Lid");
            addBox(scene_, {40, 6, 36}, {0, -21.5, 18}, "Post");
            jointTool_.start(scene_);
            jointTool_.pick(scene_, undo_, lid, JointAt::Edge, edgeNear(scene_, lid, {0, -15, -2}),
                            faceFacing(scene_, lid, {0, 0, -1}));
            jointTool_.pick(scene_, undo_, base, JointAt::Edge, edgeNear(scene_, base, {0, -15, 10}),
                            faceFacing(scene_, base, {0, 0, 1}));
            // Its limits the way that opens, as the hinge demo finds them.
            jointTool_.setMotion(scene_, undo_, 30.0 * kDeg2Rad);
            const Real sign = scene_.find(lid)->transform.position.z > 22.0 ? 1.0 : -1.0;
            jointTool_.setMotion(scene_, undo_, 0.0);
            if (Joint* j = scene_.assembly().joints.empty() ? nullptr : &scene_.assembly().joints.front()) {
                j->limited = true;
                j->lo = std::min(0.0, sign * 120.0 * kDeg2Rad);
                j->hi = std::max(0.0, sign * 120.0 * kDeg2Rad);
                clearance_.sweepJoint = j->id;
            }
            jointTool_.finish(undo_);
        }
        toggleClearance();
        clearance_.selectedOnly = false;
        // Measured here as well, to the same request the panel makes, so the
        // numbers can be checked against arithmetic before the first frame.
        startClearanceJob(clearanceKey());
        clearance_.job.wait();
        stepClearance();
        const ClearanceResult& r = clearance_.shown;
        auto named = [&](const ClearancePair& p, const char* a, const char* b) {
            const std::string na = scene_.find(p.a) ? scene_.find(p.a)->name : "";
            const std::string nb = scene_.find(p.b) ? scene_.find(p.b)->name : "";
            return (na == a && nb == b) || (na == b && nb == a);
        };
        if (step == 5) {
            const ClearancePair* pin = nullptr;
            const ClearancePair* bracket = nullptr;
            for (const ClearancePair& p : r.pairs) {
                if (named(p, "Pin", "Plate")) pin = &p;
                if (named(p, "Bracket", "Plate")) bracket = &p;
            }
            const bool agrees = pin && !pin->overlap && std::fabs(pin->gap - 0.1) <= r.deviation * 1.5 &&
                                bracket && bracket->overlap;
            std::fprintf(stderr, "[assembly-demo] 5: %zu pairs; pin in hole %.4f mm (a tenth, within %.3f), "
                         "bracket %s; %.1f ms, agrees=%d\n", r.pairs.size(), pin ? pin->gap : -1.0, r.deviation,
                         bracket && bracket->overlap ? "overlaps" : "does not overlap", r.ms, agrees ? 1 : 0);
        } else {
            const ClearancePair* hit = nullptr;
            for (const ClearancePair& p : r.pairs) if (named(p, "Lid", "Post")) hit = &p;
            const Real at = hit && hit->sample >= 0 && static_cast<size_t>(hit->sample) < r.sampleValues.size()
                                ? std::fabs(r.sampleValues[static_cast<size_t>(hit->sample)]) : -1.0;
            // The lid's inner back corner is 4 mm above the hinge line and
            // swings back to y = -15 - 4 sin a; the post's face is at -18.5, so
            // it first meets it at asin(3.5 / 4) = 61.04 degrees. The swing is
            // measured every 120/36 degrees, so the first step that overlaps is
            // the first at or past that.
            const Real first = std::asin(3.5 / 4.0) * kRad2Deg;
            const Real step = 120.0 / 36.0;
            const bool agrees = hit && hit->overlap && at >= first && at < first + step;
            std::fprintf(stderr, "[assembly-demo] 6: through %zu steps of the swing; the lid %s the post at %.1f deg "
                         "(arithmetic says first at %.2f, measured every %.2f); %.1f ms, agrees=%d\n",
                         r.sampleGaps.size(), hit && hit->overlap ? "runs into" : "clears", at, first, step, r.ms,
                         agrees ? 1 : 0);
        }
        return;
    }

    if (step == 8) {
        // Forty filleted plates, each face to face on the one below, and every
        // other one turned a little by its joint: a chain as deep as it gets.
        frameOn({0, 0, 60}, 420.0f);
        std::vector<ObjectId> plates;
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < 40; ++i) {
            char name[32];
            std::snprintf(name, sizeof name, "Plate %02d", i + 1);
            const ObjectId id = addBox(scene_, {40, 30, 3}, {static_cast<Real>(i % 8) * 50.0 - 175.0,
                                                             static_cast<Real>(i / 8) * 40.0 - 80.0, 1.5}, name);
            plates.push_back(id);
            // Rounded, so each is a few dozen faces of real surface.
            SceneObject* o = scene_.find(id);
            std::vector<EdgeId> edges;
            o->body.allEdges(edges);
            std::vector<EdgeId> vertical;
            for (EdgeId e : edges) {
                Vec3 a, b;
                o->body.edgePositions(e, a, b);
                if (std::fabs(a.z - b.z) > 1.0) vertical.push_back(e);
            }
            Feature f;
            f.kind = FeatureKind::Bevel;
            f.uid = scene_.takeFeatureUid();
            f.edges = nameEdges(o->body, vertical);
            f.width = 4.0;
            scene_.addFeature(id, f, nullptr);
        }
        const double buildMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        for (size_t i = 1; i < plates.size(); ++i) {
            jointTool_.start(scene_);
            jointTool_.pick(scene_, undo_, plates[i], JointAt::Face, faceFacing(scene_, plates[i], {0, 0, -1}),
                            faceFacing(scene_, plates[i], {0, 0, -1}));
            jointTool_.pick(scene_, undo_, plates[i - 1], JointAt::Face, faceFacing(scene_, plates[i - 1], {0, 0, 1}),
                            faceFacing(scene_, plates[i - 1], {0, 0, 1}));
            if (i % 2) jointTool_.setAngle(scene_, undo_, 8.0 * kDeg2Rad);
            jointTool_.finish(undo_);
        }
        // Every joint solved, the way each frame solves them: timed over many.
        const int runs = 200;
        const auto t1 = std::chrono::steady_clock::now();
        for (int r = 0; r < runs; ++r) solveAssembly(scene_);
        const double solveMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t1).count() / runs;
        const Vec3 top = scene_.find(plates.back())->transform.position;
        // Forty plates 3 mm thick stacked: the top one's middle at 118.5.
        const bool agrees = scene_.assembly().joints.size() == 39 && std::fabs(top.z - 118.5) < 1e-6;
        std::fprintf(stderr, "[assembly-demo] 8: 40 plates built in %.0f ms, 39 joints, solved in %.3f ms a frame; "
                     "top plate at z %.3f (arithmetic says 118.5), agrees=%d\n",
                     buildMs, solveMs, top.z, agrees ? 1 : 0);
        // The stack's clearance, the way the panel measures it: once meshing
        // every plate, and again from the meshes kept, as a joint being
        // turned would.
        toggleClearance();
        for (int pass = 0; pass < 2; ++pass) {
            clearance_.shownKey = 0;
            startClearanceJob(clearanceKey() + static_cast<uint64_t>(pass));
            clearance_.job.wait();
            stepClearance();
            const ClearanceResult& r = clearance_.shown;
            size_t touching = 0, marked = 0, overlapping = 0;
            for (const ClearancePair& p : r.pairs) { touching += p.touching ? 1 : 0; overlapping += p.overlap ? 1 : 0; }
            std::fprintf(stderr, "[assembly-demo] 8: %zu overlapping\n", overlapping);
            for (const auto& m : r.marks) marked += (m.close.size() + m.overlap.size()) / 3;
            std::fprintf(stderr, "[assembly-demo] 8: %zu mark triangles\n", marked);
            std::fprintf(stderr, "[assembly-demo] 8: clearance %s: %zu pairs, %zu touching, %zu triangle pairs, "
                         "%.1f ms\n", pass == 0 ? "meshing" : "from kept meshes", r.pairs.size(), touching,
                         r.trianglePairs, r.ms);
        }
        return;
    }

    if (step == 7) {
        // A box with its lid hinged on, a plate under it with a pin through a
        // hole in it, and a loose bracket: pulled apart.
        frameOn({0, 0, 20}, 260.0f);
        const ObjectId plate = addBox(scene_, {60, 40, 8}, {0, 0, 4}, "Plate");
        Feature hole;
        hole.kind = FeatureKind::Hole;
        hole.uid = scene_.takeFeatureUid();
        hole.axisPoint = {22, 12, 4};
        hole.axisDir = {0, 0, -1};
        hole.hole.diameter = 5.0;
        hole.hole.through = true;
        scene_.addFeature(plate, hole, nullptr);
        const ObjectId base = addBox(scene_, {40, 30, 20}, {-4, -2, 18}, "Box");
        const ObjectId lid = addBox(scene_, {40, 30, 4}, {80, 0, 2}, "Lid");
        const ObjectId pin = addCylinder(scene_, 2.4, 16, {80, 40, 8}, "Pin");
        addBox(scene_, {10, 10, 10}, {-60, 0, 5}, "Bracket");
        auto join = [&](ObjectId m, JointAt ma, Index me, Index mf, ObjectId f, JointAt fa, Index fe, Index ff) {
            jointTool_.start(scene_);
            jointTool_.pick(scene_, undo_, m, ma, me, mf);
            jointTool_.pick(scene_, undo_, f, fa, fe, ff);
            jointTool_.finish(undo_);
        };
        // The box on the plate, face to face; the lid hinged on the box; the
        // pin down through the plate's hole.
        join(base, JointAt::Face, faceFacing(scene_, base, {0, 0, -1}), faceFacing(scene_, base, {0, 0, -1}),
             plate, JointAt::Face, faceFacing(scene_, plate, {0, 0, 1}), faceFacing(scene_, plate, {0, 0, 1}));
        join(lid, JointAt::Edge, edgeNear(scene_, lid, {0, -15, -2}), faceFacing(scene_, lid, {0, 0, -1}),
             base, JointAt::Edge, edgeNear(scene_, base, {0, -15, 10}), faceFacing(scene_, base, {0, 0, 1}));
        join(pin, JointAt::Edge, rimNear(scene_, pin, {0, 0, -8}), kInvalid,
             plate, JointAt::Edge, rimNear(scene_, plate, {22, 12, 4}), kInvalid);
        if (!scene_.assembly().joints.empty()) scene_.assembly().joints.back().offset = -8.0;
        solveAssembly(scene_);
        const Vec3 pinWas = scene_.find(pin)->transform.position;
        const Vec3 lidWas = scene_.find(lid)->transform.position;
        const Vec3 boxWas = scene_.find(base)->transform.position;
        toggleExplode();
        explode_.amount = 0.7;
        explode_.shown = 0.7;
        lastDt_ = 0.016f;
        stepExplode();
        const Vec3 pinBy = scene_.find(pin)->transform.position - pinWas;
        const Vec3 lidBy = scene_.find(lid)->transform.position - lidWas;
        const Vec3 boxBy = scene_.find(base)->transform.position - boxWas;
        // The pin straight up out of its hole, the box straight up off the
        // plate, and the lid up off the box and further than the box went.
        const bool upright = std::fabs(pinBy.x) < 1e-9 && std::fabs(pinBy.y) < 1e-9 && pinBy.z > 0.0 &&
                             std::fabs(boxBy.x) < 1e-9 && std::fabs(boxBy.y) < 1e-9 && boxBy.z > 0.0 &&
                             std::fabs(lidBy.x) < 1e-9 && std::fabs(lidBy.y) < 1e-9 && lidBy.z > boxBy.z;
        std::fprintf(stderr, "[assembly-demo] 7: %zu joints; pin up %.2f, box up %.2f, lid up %.2f; "
                     "agrees=%d\n", scene_.assembly().joints.size(), pinBy.z, boxBy.z, lidBy.z, upright ? 1 : 0);
        return;
    }

    if (step == 4) {
        // A plate with a 5 mm hole through it, and a pin beside it.
        const ObjectId plate = addBox(scene_, {60, 40, 8}, {0, 0, 4}, "Plate");
        Feature hole;
        hole.kind = FeatureKind::Hole;
        hole.uid = scene_.takeFeatureUid();
        hole.axisPoint = {0, 0, 4};
        hole.axisDir = {0, 0, -1};
        hole.hole.diameter = 5.0;
        hole.hole.through = true;
        std::string why;
        const bool drilled = scene_.addFeature(plate, hole, &why);
        const ObjectId pin = addCylinder(scene_, 2.4, 20, {45, 0, 10}, "Pin");
        jointTool_.start(scene_);
        jointTool_.pick(scene_, undo_, pin, JointAt::Edge, rimNear(scene_, pin, {0, 0, -10}), kInvalid);
        jointTool_.pick(scene_, undo_, plate, JointAt::Edge, rimNear(scene_, plate, {0, 0, 4}), kInvalid);
        jointTool_.setOffset(scene_, undo_, -8.0);
        // Through the plate, on the hole's axis, its end flush with the
        // plate's underside.
        const Vec3 foot = worldOf(scene_, pin, {0, 0, -10});
        const Vec3 head = worldOf(scene_, pin, {0, 0, 10});
        const Joint* j = scene_.assembly().joints.empty() ? nullptr : &scene_.assembly().joints.front();
        const bool agrees = drilled && j && length(foot - Vec3{0, 0, 0}) < 1e-6 && length(head - Vec3{0, 0, 20}) < 1e-6;
        std::fprintf(stderr, "[assembly-demo] 4: %s, pin foot at (%.3f, %.3f, %.3f), head at (%.3f, %.3f, %.3f), "
                     "agrees=%d%s%s\n", j ? jointKindName(j->kind) : "no joint", foot.x, foot.y, foot.z,
                     head.x, head.y, head.z, agrees ? 1 : 0, why.empty() ? "" : "  ", why.c_str());
        frameOn({0, 0, 8}, 170.0f);
        return;
    }
}

void Application::stepAssemblyDemo() {
    if (assemblyDemo_ <= 0) return;
    ++assemblyDemoFrame_;
    if (camera_.viewportW <= 0) return;
    if (assemblyDemo_ != 1 || assemblyDemoFrame_ < 3) return;
    // Over the middle of the box's top.
    Vec2 px;
    if (camera_.projectToPixel({4, 3, 20}, px)) mouseOverride_ = px;
    if (assemblyDemoFrame_ == 6)
        std::fprintf(stderr, "[assembly-demo] 1: pointing at the box: shows where the lid goes=%d\n",
                     jointTool_.picking() ? 1 : 0);
}

} // namespace tg
