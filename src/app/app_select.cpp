// Tangent - choosing what to work on: the selection filter and box select.
//
// A filter says which kind of thing a click takes -- faces, edges, corners or
// whole parts -- for a body of three hundred faces where the one edge wanted is
// a pixel from the face beside it. A box takes everything of that kind at once:
// dragged to the right, what is wholly inside it; to the left, whatever it
// touches, the way every CAD tool has done it since drawing boards.
#include "app/application.h"

#include "core/palette.h"
#include "ui/glyph.h"
#include "ui/theme.h"
#include "ui/widgets.h"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace tg {

std::vector<ElementHit> Application::filteredPicks(const Ray& ray, Vec2 m) const {
    // Wider reach for the kind asked for, since nothing else competes for it.
    float vertexTol = 10.0f, edgeTol = 7.0f;
    switch (pickFilter_) {
        case PickFilter::Faces:  vertexTol = 0.0f; edgeTol = 0.0f; break;
        case PickFilter::Edges:  vertexTol = 0.0f; edgeTol = 12.0f; break;
        case PickFilter::Points: vertexTol = 16.0f; edgeTol = 0.0f; break;
        case PickFilter::Any:
        case PickFilter::Parts:  break;
    }
    std::vector<ElementHit> picks =
        scene_.pickElements(ray, camera_.viewProjection(), camera_.viewportW, camera_.viewportH, m, vertexTol, edgeTol);
    const ElementKind only = pickFilter_ == PickFilter::Faces  ? ElementKind::Face
                           : pickFilter_ == PickFilter::Edges  ? ElementKind::Edge
                           : pickFilter_ == PickFilter::Points ? ElementKind::Vertex
                                                               : ElementKind::None;
    if (only != ElementKind::None)
        picks.erase(std::remove_if(picks.begin(), picks.end(), [&](const ElementHit& h) { return h.ref.kind != only; }),
                    picks.end());
    return picks;
}

// ---------------------------------------------------------------------------
// The filter, in the view's bottom-left corner
// ---------------------------------------------------------------------------

void Application::drawPickFilterBar() {
    if (!viewRect_.w) return;
    struct Option { PickFilter f; Glyph g; const char* label; const char* tip; };
    static const Option kOptions[] = {
        {PickFilter::Any, Glyph::Select, "Any", "A click takes the face, edge or corner under it"},
        {PickFilter::Faces, Glyph::Body, "Faces", "Only faces: clicks near an edge still take the face"},
        {PickFilter::Edges, Glyph::Line, "Edges", "Only edges, with more reach"},
        {PickFilter::Points, Glyph::Dot, "Corners", "Only corners, with more reach"},
        {PickFilter::Parts, Glyph::Box, "Parts", "Whole parts, as Ctrl+click takes them"},
    };
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->Pos.x + viewRect_.x + 12.0f, vp->Pos.y + viewRect_.y + viewRect_.h - 12.0f),
                            ImGuiCond_Always, ImVec2(0.0f, 1.0f));
    ImGui::SetNextWindowBgAlpha(0.92f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(4.0f, 4.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(2.0f, 2.0f));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ui::im(palette::kCommand));
    ImGui::PushStyleColor(ImGuiCol_Border, ui::im(palette::kBorder));
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                                   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                                   ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;
    if (ImGui::Begin("##pickfilter", nullptr, flags)) {
        // Words when the view is wide enough for them; pictures when not.
        const bool words = viewRect_.w > 520.0f;
        ImDrawList* dl = ImGui::GetWindowDrawList();
        for (const Option& o : kOptions) {
            ImGui::PushID(static_cast<int>(o.f));
            if (o.f != PickFilter::Any) ImGui::SameLine();
            pushFont(FontWeight::Medium, uiFonts().size * 0.85f);
            const float w = words ? ImGui::CalcTextSize(o.label).x + 32.0f : 28.0f;
            const float h = 24.0f;
            const ImVec2 at = ImGui::GetCursorScreenPos();
            if (ImGui::InvisibleButton("##f", ImVec2(w, h))) pickFilter_ = o.f;
            const bool on = pickFilter_ == o.f, hot = ImGui::IsItemHovered();
            if (on || hot) dl->AddRectFilled(at, ImVec2(at.x + w, at.y + h), on ? ui::u32(palette::kBrand, 0.18f)
                                                                                : ui::u32(palette::kHover), 6.0f);
            const ImU32 c = on ? ui::u32(palette::kBrand) : ui::u32(palette::kTextDim);
            drawGlyph(dl, o.g, ImVec2(at.x + 14.0f, at.y + h * 0.5f), 14.0f, c);
            if (words)
                dl->AddText(ImVec2(at.x + 25.0f, at.y + (h - ImGui::GetTextLineHeight()) * 0.5f),
                            on ? ui::u32(palette::kText) : ui::u32(palette::kTextDim), o.label);
            ImGui::PopFont();
            if (hot) ui::hoverTip(words ? o.tip : (std::string(o.label) + ": " + o.tip).c_str());
            ImGui::PopID();
        }
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar(4);
}

// ---------------------------------------------------------------------------
// Box select
// ---------------------------------------------------------------------------

void Application::finishBoxSelect(bool additive) {
    const Vec2 a = boxFrom_, b = mouseInViewport();
    const float x0 = static_cast<float>(std::min(a.x, b.x)), x1 = static_cast<float>(std::max(a.x, b.x));
    const float y0 = static_cast<float>(std::min(a.y, b.y)), y1 = static_cast<float>(std::max(a.y, b.y));
    // Right to left takes what it touches; left to right what is inside.
    const bool crossing = b.x < a.x;
    auto inside = [&](Vec3 world) {
        if (scene_.section().removes(world)) return false;
        Vec2 px{};
        if (!camera_.projectToPixel(world, px)) return false;
        return px.x >= x0 && px.x <= x1 && px.y >= y0 && px.y <= y1;
    };
    // On the screen, for the crossing box, which takes what it overlaps --
    // a part's middle clipped by the box has none of its corners in it.
    auto project = [&](Vec3 w, Vec2& p) { return !scene_.section().removes(w) && camera_.projectToPixel(w, p); };
    auto segHits = [&](Vec2 p, Vec2 q) {
        // Liang-Barsky: what is left of the segment clipped to the box.
        double t0 = 0.0, t1 = 1.0;
        const double dx = q.x - p.x, dy = q.y - p.y;
        const double pp[4] = {-dx, dx, -dy, dy};
        const double qq[4] = {p.x - x0, x1 - p.x, p.y - y0, y1 - p.y};
        for (int i = 0; i < 4; ++i) {
            if (std::fabs(pp[i]) < 1e-12) { if (qq[i] < 0.0) return false; continue; }
            const double t = qq[i] / pp[i];
            if (pp[i] < 0.0) t0 = std::max(t0, t);
            else             t1 = std::min(t1, t);
            if (t0 > t1) return false;
        }
        return true;
    };
    auto triHits = [&](Vec2 a2, Vec2 b2, Vec2 c2) {
        if (segHits(a2, b2) || segHits(b2, c2) || segHits(c2, a2)) return true;
        // The box wholly inside the triangle.
        const Vec2 m{(x0 + x1) * 0.5, (y0 + y1) * 0.5};
        auto side = [](Vec2 p, Vec2 q, Vec2 r) { return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x); };
        const double s1 = side(a2, b2, m), s2 = side(b2, c2, m), s3 = side(c2, a2, m);
        return (s1 >= 0 && s2 >= 0 && s3 >= 0) || (s1 <= 0 && s2 <= 0 && s3 <= 0);
    };
    // Every point in, for a window. For a crossing, any triangle (`tris`) or
    // any piece of the line (otherwise) that the box overlaps.
    auto takes = [&](const std::vector<Vec3>& pts, bool tris) {
        if (pts.empty()) return false;
        if (!crossing) {
            for (const Vec3& p : pts) if (!inside(p)) return false;
            return true;
        }
        if (tris) {
            for (size_t i = 0; i + 2 < pts.size(); i += 3) {
                Vec2 a2{}, b2{}, c2{};
                if (project(pts[i], a2) && project(pts[i + 1], b2) && project(pts[i + 2], c2) && triHits(a2, b2, c2))
                    return true;
            }
            return false;
        }
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            Vec2 a2{}, b2{};
            if (project(pts[i], a2) && project(pts[i + 1], b2) && segHits(a2, b2)) return true;
        }
        return pts.size() == 1 && inside(pts[0]);
    };

    if (!additive) {
        scene_.clearSelection();
        scene_.clearElementSelection();
    }
    size_t taken = 0;
    std::vector<Vec3> pts;
    for (const auto& obj : scene_.objects()) {
        const SceneObject& o = *obj;
        if (!o.visible || o.body.empty()) continue;
        const Mat4 model = o.modelMatrix();
        const RenderMesh& rm = o.render;
        if (pickFilter_ == PickFilter::Any || pickFilter_ == PickFilter::Parts) {
            pts.clear();
            for (uint32_t idx : rm.triangles) pts.push_back(transformPoint(model, rm.positions[idx]));
            if (takes(pts, true)) {
                if (!scene_.isSelected(o.id)) scene_.select(o.id, true);
                ++taken;
            }
            continue;
        }
        if (pickFilter_ == PickFilter::Faces) {
            // The corners of each face's triangles, gathered once for the body.
            std::vector<std::vector<Vec3>> byFace(static_cast<size_t>(std::max(0, o.body.faceCount())));
            for (size_t t = 0; t < rm.triangleFace.size(); ++t) {
                const Index f = rm.triangleFace[t];
                if (f < 0 || static_cast<size_t>(f) >= byFace.size()) continue;
                for (int k = 0; k < 3; ++k) byFace[static_cast<size_t>(f)].push_back(transformPoint(model, rm.positions[rm.triangles[t * 3 + k]]));
            }
            for (size_t f = 0; f < byFace.size(); ++f)
                if (takes(byFace[f], true)) {
                    const ElementRef r{o.id, ElementKind::Face, static_cast<Index>(f)};
                    if (!scene_.isElementSelected(r)) scene_.toggleElement(r);
                    ++taken;
                }
        } else if (pickFilter_ == PickFilter::Edges) {
            std::vector<EdgeId> es;
            o.body.allEdges(es);
            const Real pixel = std::max<Real>(camera_.pixelWorldSize(camera_.target), 1e-6);
            for (EdgeId e : es) {
                if (o.body.isBridgeEdge(e)) continue;
                o.body.edgePolyline(e, pixel, pts);
                for (Vec3& p : pts) p = transformPoint(model, p);
                if (takes(pts, false)) {
                    const ElementRef r{o.id, ElementKind::Edge, e};
                    if (!scene_.isElementSelected(r)) scene_.toggleElement(r);
                    ++taken;
                }
            }
        } else if (pickFilter_ == PickFilter::Points) {
            std::vector<VertexId> vs;
            o.body.allVertices(vs);
            for (VertexId v : vs)
                if (inside(transformPoint(model, o.body.vertexPosition(v)))) {
                    const ElementRef r{o.id, ElementKind::Vertex, v};
                    if (!scene_.isElementSelected(r)) scene_.toggleElement(r);
                    ++taken;
                }
        }
    }
    const char* what = pickFilter_ == PickFilter::Faces ? "faces" : pickFilter_ == PickFilter::Edges ? "edges"
                     : pickFilter_ == PickFilter::Points ? "corners" : "parts";
    char msg[96];
    if (taken) std::snprintf(msg, sizeof msg, "%zu %s selected", taken, what);
    else       std::snprintf(msg, sizeof msg, "No %s %s the box", what, crossing ? "touch" : "are wholly in");
    setNotice(msg);
}

// ---------------------------------------------------------------------------
// Kept measurements
// ---------------------------------------------------------------------------

void Application::keepMeasurement() {
    const std::vector<ElementRef>& picks = measure_.picks();
    if (picks.empty()) return;
    KeptMeasure m;
    m.id = scene_.takeMeasureId();
    for (const ElementRef& e : picks) {
        if (m.count >= 2) break;
        const SceneObject* o = scene_.find(e.object);
        if (!o) return;
        KeptMeasure::End end;
        end.object = e.object;
        end.kind = e.kind;
        end.name = e.kind == ElementKind::Face   ? o->body.faceName(e.index)
                 : e.kind == ElementKind::Edge   ? o->body.edgeName(e.index)
                 : e.kind == ElementKind::Vertex ? o->body.vertexName(e.index) : 0;
        if (end.name == 0) { setNotice("That cannot be kept: what it measures has no name to follow"); return; }
        m.ends[m.count++] = end;
    }
    const std::vector<KeptMeasure> before = scene_.measures();
    scene_.measures().push_back(m);
    undo_.push(std::make_unique<MeasuresCommand>(before, scene_.measures(), "Keep Measurement"));
    measure_.clearPicks();
    setNotice("Kept on the model: it follows the part as it changes");
}

void Application::stepKeptMeasures() {
    ui_.measureLabels.clear();
    for (const KeptMeasure& m : scene_.measures()) {
        // What the reading depends on: the geometry and the place of each part.
        uint64_t key = 1469598103934665603ull;
        auto mix = [&key](uint64_t v) { key = (key ^ v) * 1099511628211ull; };
        bool ok = true;
        ElementRef refs[2];
        for (int i = 0; i < m.count; ++i) {
            refs[i] = scene_.resolve(m.ends[i]);
            const SceneObject* o = scene_.find(m.ends[i].object);
            if (!refs[i].valid() || !o) { ok = false; break; }
            mix(o->id); mix(o->geometryVersion);
            const Vec3 p = o->transform.position;
            const Quat q = o->transform.rotation;
            for (Real v : {p.x, p.y, p.z, q.x, q.y, q.z, q.w}) { uint64_t b; std::memcpy(&b, &v, sizeof b); mix(b); }
        }
        if (!ok) { keptReadings_.erase(m.id); continue; }
        KeptReading& r = keptReadings_[m.id];
        if (r.key != key) {
            MeasureTool t;
            t.begin();
            for (int i = 0; i < m.count; ++i) t.pick(refs[i]);
            r.result = t.compute(scene_);
            r.key = key;
        }
        if (r.result.valid) ui_.measureLabels.push_back({m.id, r.result.summary});
    }
}

void Application::drawKeptMeasures() {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const Vec4 col = toVec4(palette::kInfo, 0.95f);
    for (const KeptMeasure& m : scene_.measures()) {
        if (!m.visible) continue;
        auto it = keptReadings_.find(m.id);
        if (it == keptReadings_.end() || !it->second.result.valid) continue;
        const MeasureResult& r = it->second.result;
        // The line and its end ticks, in the model, in their own colour so a
        // kept reading is not taken for the one being made.
        const Real tick = camera_.pixelWorldSize(r.from) * 4.0;
        if (lengthSq(r.to - r.from) > 1e-18) {
            renderer_.addLine(r.from, r.to, col);
            const Vec3 axis = normalize(r.to - r.from);
            const Vec3 side = perpendicular(axis);
            for (const Vec3& p : {r.from, r.to}) renderer_.addLine(p - side * tick, p + side * tick, col);
        } else {
            for (int k = 0; k < 3; ++k) {
                Vec3 d{};
                d[k] = tick;
                renderer_.addLine(r.from - d, r.from + d, col);
            }
        }
        // Its reading, at the middle.
        Vec2 px{};
        if (!camera_.projectToPixel((r.from + r.to) * 0.5, px)) continue;
        pushFont(FontWeight::Medium, uiFonts().size * 0.85f);
        const ImVec2 ts = ImGui::CalcTextSize(r.summary.c_str());
        const ImVec2 at(std::floor(vp->Pos.x + viewRect_.x + static_cast<float>(px.x) - ts.x * 0.5f),
                        std::floor(vp->Pos.y + viewRect_.y + static_cast<float>(px.y) - ts.y - 8.0f));
        dl->AddRectFilled(ImVec2(at.x - 6, at.y - 3), ImVec2(at.x + ts.x + 6, at.y + ts.y + 3),
                          ui::u32(palette::kCommand, 0.9f), 4.0f);
        dl->AddRectFilled(ImVec2(at.x - 6, at.y - 3), ImVec2(at.x - 3, at.y + ts.y + 3), ui::u32(palette::kInfo), 2.0f);
        dl->AddText(at, ui::u32(palette::kText), r.summary.c_str());
        ImGui::PopFont();
    }
}

void Application::runSelectDemo() {
    if (selectDemo_ == 2) {
        // Two 20 mm boxes 20 mm apart; the gap between their facing faces
        // kept; the first box made 30 wide about its middle, so the gap closes
        // by 5 to 15 -- and the kept reading has to say so.
        Scene& s = scene_;
        s.clear();
        const ObjectId a = s.addPrimitive(PrimitiveKind::Box, {}, Vec3{0, 0, 10});
        const ObjectId b = s.addPrimitive(PrimitiveKind::Box, {}, Vec3{40, 0, 10});
        auto faceFacing = [&](ObjectId id, Vec3 n) {
            const SceneObject* o = s.find(id);
            std::vector<FaceId> fs;
            o->body.allFaces(fs);
            for (FaceId f : fs) if (dot(normalize(o->body.faceNormal(f)), n) > 0.999) return f;
            return kNoFace;
        };
        measure_.begin();
        measure_.pick({a, ElementKind::Face, faceFacing(a, {1, 0, 0})});
        measure_.pick({b, ElementKind::Face, faceFacing(b, {-1, 0, 0})});
        keepMeasurement();
        measure_.end();
        stepKeptMeasures();
        const Real before = keptReadings_.empty() ? -1.0 : keptReadings_.begin()->second.result.distance;
        SceneObject* o = s.find(a);
        o->spec.box.width = 30.0;
        s.rebuild(a);
        stepKeptMeasures();
        const Real after = keptReadings_.empty() ? -1.0 : keptReadings_.begin()->second.result.distance;
        std::fprintf(stderr, "[select-demo] 2: kept gap %.3f, after widening %.3f (expected 20, 15)  agrees=%d\n",
                     before, after, std::fabs(before - 20.0) < 1e-6 && std::fabs(after - 15.0) < 1e-6 ? 1 : 0);
        camera_.animateTo({20, 0, 10}, 150.0f, 0.7f, 0.5f);
        camera_.snapToGoal();
        fixedCamera_ = true;
        return;
    }
    // Three 20 mm boxes in a row, seen from the top: a window round the first
    // two takes those two; a crossing box that only clips the third takes it;
    // edges in a window round the first box are its twelve.
    Scene& s = scene_;
    s.clear();
    for (int i = 0; i < 3; ++i) s.addPrimitive(PrimitiveKind::Box, {}, Vec3{i * 40.0, 0.0, 10.0});
    camera_.setStandardView(StandardView::Top);
    camera_.snapToGoal();
    camera_.animateTo({40, 0, 10}, 320.0f, 0.0f, camera_.pitch);
    camera_.snapToGoal();
    fixedCamera_ = true;
    if (camera_.viewportW <= 0) return;
    auto px = [&](Vec3 w) { Vec2 p{}; camera_.projectToPixel(w, p); return p; };
    auto box = [&](Vec3 a, Vec3 b, PickFilter f) {
        pickFilter_ = f;
        boxFrom_ = px(a);
        mouseOverride_ = px(b);
        finishBoxSelect(false);
        mouseOverride_ = {-1.0, -1.0};
    };
    box({-15, 15, 10}, {55, -15, 10}, PickFilter::Parts);
    const size_t window = s.selection().size();
    box({95, -5, 10}, {75, 5, 10}, PickFilter::Parts);          // right to left, over the third's edge
    const size_t crossing = s.selection().size();
    const bool third = s.isSelected(s.objects()[2]->id);
    box({-15, 15, 10}, {15, -15, 10}, PickFilter::Edges);
    const size_t edges = s.elementSelection().size();
    std::fprintf(stderr, "[select-demo] window %zu (2), crossing %zu (1) third=%d, edges %zu (12)  agrees=%d\n",
                 window, crossing, third ? 1 : 0, edges, window == 2 && crossing == 1 && third && edges == 12 ? 1 : 0);
    pickFilter_ = PickFilter::Any;
}

void Application::drawBoxSelect() {
    if (!boxSelecting_) return;
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const Vec2 a = boxFrom_, b = mouseInViewport();
    const ImVec2 p0(vp->Pos.x + viewRect_.x + static_cast<float>(std::min(a.x, b.x)),
                    vp->Pos.y + viewRect_.y + static_cast<float>(std::min(a.y, b.y)));
    const ImVec2 p1(vp->Pos.x + viewRect_.x + static_cast<float>(std::max(a.x, b.x)),
                    vp->Pos.y + viewRect_.y + static_cast<float>(std::max(a.y, b.y)));
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    const bool crossing = b.x < a.x;
    dl->AddRectFilled(p0, p1, ui::u32(palette::kBrand, crossing ? 0.05f : 0.09f));
    if (!crossing) {
        dl->AddRect(p0, p1, ui::u32(palette::kBrand, 0.9f), 0.0f, 0, 1.5f);
        return;
    }
    // Dashed: it takes what it touches.
    auto dashes = [&](ImVec2 s, ImVec2 e) {
        const float len = std::hypot(e.x - s.x, e.y - s.y);
        if (len < 1.0f) return;
        const ImVec2 d((e.x - s.x) / len, (e.y - s.y) / len);
        for (float t = 0.0f; t < len; t += 10.0f) {
            const float u = std::min(len, t + 6.0f);
            dl->AddLine(ImVec2(s.x + d.x * t, s.y + d.y * t), ImVec2(s.x + d.x * u, s.y + d.y * u),
                        ui::u32(palette::kBrand, 0.9f), 1.5f);
        }
    };
    dashes(p0, ImVec2(p1.x, p0.y));
    dashes(ImVec2(p1.x, p0.y), p1);
    dashes(p1, ImVec2(p0.x, p1.y));
    dashes(ImVec2(p0.x, p1.y), p0);
}

} // namespace tg
