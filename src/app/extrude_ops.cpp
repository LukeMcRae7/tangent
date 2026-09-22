#include "app/extrude_ops.h"

#include "geom/operations.h"
#include "ui/command_panel.h"
#include "ui/theme.h"
#include "ui/widgets.h"

#include "core/palette.h"

#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace tg {

bool extrudeOpForKey(int key, ExtrudeOp& out) {
    switch (key) {
        case 'J': case 'j': out = ExtrudeOp::Join;      return true;
        case 'D': case 'd': out = ExtrudeOp::Cut;       return true;
        case 'I': case 'i': out = ExtrudeOp::Intersect; return true;
        case 'N': case 'n': out = ExtrudeOp::NewBody;   return true;
        default: return false;
    }
}

const char* extrudeOpVerb(ExtrudeOp op) {
    switch (op) {
        case ExtrudeOp::Join:      return "joins";
        case ExtrudeOp::Cut:       return "cuts";
        case ExtrudeOp::Intersect: return "intersects";
        case ExtrudeOp::NewBody:   return "makes";
        case ExtrudeOp::Auto:      break;
    }
    return "joins";
}

// ---------------------------------------------------------------------------
void ExtrudeReach::refresh(const Scene& scene, const Body& toolWorld, ObjectId owner, ExtrudeOp op,
                           Real depth, const std::string& toolKey) {
    // What would change the answer: the tool, the operation, which side of the
    // owner the tool is on, and every body that might be reached -- by its
    // geometry and by where it stands.
    std::string key = toolKey;
    char b[160];
    std::snprintf(b, sizeof b, "|%d|%u|%d", static_cast<int>(op), owner, depth < 0.0 ? -1 : 1);
    key += b;
    for (const auto& o : scene.objects()) {
        const Transform& t = o->transform;
        std::snprintf(b, sizeof b, "|%u:%u:%d:%.6g,%.6g,%.6g:%.6g,%.6g,%.6g,%.6g", o->id, o->meshVersion,
                      o->visible ? 1 : 0, t.position.x, t.position.y, t.position.z, t.rotation.x,
                      t.rotation.y, t.rotation.z, t.rotation.w);
        key += b;
    }
    if (key == key_) return;
    key_ = key;

    bodies_.clear();
    if (op == ExtrudeOp::NewBody || toolWorld.empty() || toolWorld.isMesh()) return;

    const AABB reach = toolWorld.bounds();
    for (const auto& o : scene.objects()) {
        if (!o->visible || o->body.empty() || o->body.isMesh()) continue;
        if (o->id == owner) {
            // Out of the body it came from, a join grows it and a cut does
            // nothing to it; into it, the other way round.
            const bool acts = op == ExtrudeOp::Join ? depth > 0.0 : depth < 0.0;
            if (!acts) continue;
        }
        if (!reach.overlaps(o->worldBounds(), 1e-4)) continue;
        Body local = toolWorld;
        if (!local.transform(inverse(o->modelMatrix()))) continue;
        // Whether they really touch, measured on the surfaces -- except when
        // both are big enough that measuring it costs more than the answer is
        // worth. Asking the kernel for the distance between a two-thousand
        // face drawing and a two-thousand face part takes minutes, and this
        // list is a row of pills a person is about to correct anyway: for
        // those the boxes above are the answer, which errs towards listing a
        // body rather than missing one. A cut that turns out to take nothing
        // away is dropped when it is applied.
        constexpr int kMeasureUpTo = 400;
        if (local.faceCount() <= kMeasureUpTo || o->body.faceCount() <= kMeasureUpTo)
            if (!bodiesTouch(local, o->body)) continue;
        const bool out = std::find(excluded_.begin(), excluded_.end(), o->id) != excluded_.end();
        bodies_.push_back({o->id, !out});
    }
    // The owner first: it is the body a join is merged into.
    std::stable_partition(bodies_.begin(), bodies_.end(),
                          [owner](const ReachedBody& r) { return r.id == owner; });
}

void ExtrudeReach::toggle(ObjectId id) {
    for (ReachedBody& r : bodies_) {
        if (r.id != id) continue;
        r.included = !r.included;
        auto it = std::find(excluded_.begin(), excluded_.end(), id);
        if (!r.included && it == excluded_.end()) excluded_.push_back(id);
        if (r.included && it != excluded_.end()) excluded_.erase(it);
    }
}

bool ExtrudeReach::includes(ObjectId id) const {
    for (const ReachedBody& r : bodies_)
        if (r.id == id) return r.included;
    return false;
}

std::vector<ObjectId> ExtrudeReach::included() const {
    std::vector<ObjectId> out;
    for (const ReachedBody& r : bodies_)
        if (r.included) out.push_back(r.id);
    return out;
}

// ---------------------------------------------------------------------------
void unwind(Scene& scene, std::vector<std::unique_ptr<Command>>& parts) {
    for (auto it = parts.rbegin(); it != parts.rend(); ++it) (*it)->undo(scene);
    parts.clear();
}

bool applyExtrude(Scene& scene, const Body& toolWorld, ExtrudeOp op,
                  const std::vector<ObjectId>& bodies, ObjectId ownerDone,
                  const std::string& toolName, const char* label,
                  std::vector<std::unique_ptr<Command>>& parts, std::string& error) {
    if (op == ExtrudeOp::NewBody || op == ExtrudeOp::Auto || bodies.empty()) return true;

    // What this call did, and only that, is taken back on a refusal: the
    // caller may already have changed the owner, and that is its to undo.
    std::vector<std::unique_ptr<Command>> mine;
    auto refuse = [&](const SceneObject* o, const std::string& why) {
        error = std::string(extrudeOpName(op)) + " refused";
        if (o) error += " on " + o->name;
        if (!why.empty()) error += ": " + why;
        unwind(scene, mine);
        return false;
    };

    // One boolean step on one body, from something in the world.
    enum class Added { Yes, Nothing, Refused };
    auto combine = [&](ObjectId id, const Body& world, BooleanOp how, const std::string& name,
                       bool dropIfUnchanged) {
        SceneObject* o = scene.find(id);
        if (!o) return Added::Nothing;
        Body local = world;
        if (!local.transform(inverse(o->modelMatrix()))) {
            refuse(o, "it could not be placed on it");
            return Added::Refused;
        }
        std::vector<Feature> before = o->features;
        const double volBefore = dropIfUnchanged ? o->body.health(false).volume : 0.0;

        Feature f;
        f.kind = FeatureKind::Boolean;
        f.booleanOp = how;
        f.bakedBody = std::move(local);
        f.toolName = name;
        std::string why;
        if (!scene.addFeature(id, std::move(f), &why)) {
            refuse(o, why.empty() ? "no valid solid came out of it" : why);
            return Added::Refused;
        }
        if (dropIfUnchanged) {
            const double volAfter = o->body.health(false).volume;
            if (std::fabs(volAfter - volBefore) <= 1e-7 * std::max(1.0, std::fabs(volBefore))) {
                o->features = std::move(before);
                scene.reevaluate(id);
                return Added::Nothing;
            }
        }
        mine.push_back(std::make_unique<FeatureCommand>(id, std::move(before), o->features, label));
        return Added::Yes;
    };

    if (op == ExtrudeOp::Join) {
        const ObjectId into = bodies.front();
        if (into != ownerDone &&
            combine(into, toolWorld, BooleanOp::Union, toolName, false) == Added::Refused)
            return false;
        for (size_t i = 1; i < bodies.size(); ++i) {
            const SceneObject* other = scene.find(bodies[i]);
            if (!other || bodies[i] == into) continue;
            Body world = other->body;
            if (!world.transform(other->modelMatrix())) return refuse(other, "it could not be placed");
            if (combine(into, world, BooleanOp::Union, other->name, false) == Added::Refused) return false;
            mine.push_back(ExistenceCommand::forDelete(scene, {bodies[i]}));
        }
    } else {
        const BooleanOp how = op == ExtrudeOp::Cut ? BooleanOp::Difference : BooleanOp::Intersection;
        for (ObjectId id : bodies) {
            if (id == ownerDone) continue;
            if (combine(id, toolWorld, how, toolName, op == ExtrudeOp::Cut) == Added::Refused) return false;
        }
    }
    for (auto& p : mine) parts.push_back(std::move(p));
    return true;
}

// ---------------------------------------------------------------------------
bool drawExtrudeChoice(ExtrudeChoice& choice, bool allowNewBody) {
    static const ui::Choice kOps[4] = {
        {Glyph::Union,      "Join",      "J", "Add it to the bodies it reaches  (J)"},
        {Glyph::Difference, "Cut",       "D", "Take it away from the bodies it reaches  (D)"},
        {Glyph::Intersect,  "Intersect", "I", "Keep only what they share with it  (I)"},
        {Glyph::NewBody,    "New Body",  "N", "A body of its own, leaving the rest alone  (N)"},
    };
    static const ExtrudeOp kOf[4] = {ExtrudeOp::Join, ExtrudeOp::Cut, ExtrudeOp::Intersect,
                                     ExtrudeOp::NewBody};
    int on = 0;
    for (int i = 0; i < 4; ++i)
        if (kOf[i] == choice.op) on = i;
    const int pick = ui::commandChoices(choice.automatic ? "Operation   \xC2\xB7  following the drag until you pick"
                                                         : "Operation",
                                        kOps, allowNewBody ? 4 : 3, on);
    if (pick < 0) return false;
    choice.pick(kOf[pick]);
    return true;
}

ObjectId drawReachedBodies(const Scene& scene, const ExtrudeReach& reach, ExtrudeOp op, ObjectId owner) {
    ui::commandRow("Bodies");
    if (op == ExtrudeOp::NewBody) {
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(ui::im(palette::kTextDim), "a new one, of its own");
        return kNoObject;
    }
    if (reach.bodies().empty()) {
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(ui::im(palette::kTextDim), "%s",
                           op == ExtrudeOp::Join ? "none reached: it will be a body of its own"
                                                 : "none reached: nothing to act on");
        return kNoObject;
    }

    // One pill per body, wrapping under the label column. Lit is in.
    ObjectId clicked = kNoObject;
    const float right = ImGui::GetWindowContentRegionMax().x;
    const float gap = ImGui::GetStyle().ItemSpacing.x * 0.5f;
    bool first = true;
    for (const ReachedBody& r : reach.bodies()) {
        const SceneObject* o = scene.find(r.id);
        if (!o) continue;
        char label[96];
        std::snprintf(label, sizeof label, "%s%s", o->name.c_str(), r.id == owner ? "  (from)" : "");
        const float w = ImGui::CalcTextSize(label).x + 24.0f;
        if (!first) {
            ImGui::SameLine(0.0f, gap);
            if (ImGui::GetCursorPosX() + w > right) {
                ImGui::NewLine();
                ImGui::SetCursorPosX(ui::commandLabelWidth());
            }
        }
        first = false;
        ImGui::PushID(static_cast<int>(r.id));
        if (ui::pillButton(label, r.included)) clicked = r.id;
        ui::hoverTip(r.included ? "Included. Click to leave it out." : "Left out. Click to include it.");
        ImGui::PopID();
    }
    return clicked;
}

} // namespace tg
