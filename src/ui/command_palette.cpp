#include "ui/command_palette.h"

#include "core/palette.h"
#include "geom/brep.h"
#include "ui/glyph.h"
#include "ui/theme.h"
#include "ui/widgets.h"

#include "imgui.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

namespace tg::ui {

namespace {

bool g_open = false;
bool g_focus = false;
char g_query[128] = "";
int  g_selected = 0;

struct Command {
    const char* group;
    const char* name;
    const char* keys;           // or nullptr
    const char* also;           // other words it answers to
    Glyph glyph;
    const char* why;            // why it cannot run now; nullptr when it can
    std::function<void()> run;
};

// How well `query` matches: every character of it in order, scored for
// runs of them and for landing on the start of a word. -1 is no match.
int score(const std::string& text, const std::string& query) {
    if (query.empty()) return 0;
    int s = 0, run = 0;
    size_t qi = 0;
    for (size_t i = 0; i < text.size() && qi < query.size(); ++i) {
        const char a = static_cast<char>(std::tolower(static_cast<unsigned char>(text[i])));
        if (a != query[qi]) { run = 0; continue; }
        const bool wordStart = i == 0 || text[i - 1] == ' ' || text[i - 1] == '/' || text[i - 1] == '-';
        s += 1 + run * 3 + (wordStart ? 6 : 0);
        ++run;
        ++qi;
    }
    return qi == query.size() ? s : -1;
}

std::vector<Command> commands(UiContext& ctx) {
    Scene& scene = *ctx.scene;
    UiActions& a = ctx.actions;
    const ObjectId obj = scene.contextObject();
    const bool hasObject = obj != kNoObject;
    const bool hasSel = !scene.selection().empty() || !scene.elementSelection().empty();
    const size_t faces = hasObject ? scene.selectedFaces(obj).size() : 0;
    const size_t edges = hasObject ? scene.selectedEdges(obj).size() : 0;
    const bool exact = brep::available();
    const size_t bodies = std::count_if(scene.objects().begin(), scene.objects().end(),
                                        [](const auto& o) { return !o->body.empty(); });
    const SceneObject* o = scene.find(obj);
    const bool isMesh = o && !o->body.empty() && o->body.isMesh();
    const char* noKernel = exact ? nullptr : "needs the exact kernel, which this build does not have";
    auto need = [](bool ok, const char* why) { return ok ? nullptr : why; };
    auto either = [](const char* a1, const char* b) { return a1 ? a1 : b; };

    std::vector<Command> c;
    auto add = [&](const char* group, const char* name, const char* keys, const char* also, Glyph g,
                   const char* why, std::function<void()> run) {
        c.push_back({group, name, keys, also, g, why, std::move(run)});
    };
    auto shape = [&](const char* name, PrimitiveKind k, Glyph g) {
        add("Create", name, nullptr, "add shape primitive", g, nullptr, [&a, k] {
            a.addRequested = true;
            a.addKind = k;
        });
    };

    // File
    add("File", "New Project", "Ctrl+N", "blank empty", Glyph::New, nullptr, [&a] { a.newProject = true; });
    add("File", "Open...", "Ctrl+O", "load project", Glyph::Open, nullptr, [&a] { a.openProject = true; });
    add("File", "Save", "Ctrl+S", "write", Glyph::Save, nullptr, [&a] { a.saveProject = true; });
    add("File", "Save As...", nullptr, "copy rename", Glyph::Save, nullptr, [&a] { a.saveProjectAs = true; });
    add("File", "Import STEP...", nullptr, "stp cad", Glyph::Import, nullptr, [&a] { a.importStep = true; });
    add("File", "Import Mesh...", nullptr, "stl obj", Glyph::Mesh, nullptr, [&a] { a.importMesh = true; });
    add("File", "Import SVG...", nullptr, "drawing vector outline", Glyph::Sketch, nullptr, [&a] { a.importSvg = true; });
    add("File", "Export 3MF...", "Ctrl+Shift+E", "slicer print", Glyph::Export, nullptr, [&a] { a.export3mf = true; });
    add("File", "Export STL...", "Ctrl+E", "slicer print triangles", Glyph::Export, nullptr, [&a] { a.exportStl = true; });
    add("File", "Export STEP...", nullptr, "stp cad exact", Glyph::Export, nullptr, [&a] { a.exportStep = true; });
    add("File", "Preferences...", "Ctrl+,", "settings options units inches theme dark light printer nozzle autosave",
        Glyph::Settings, nullptr, [&a] { a.openPreferences = true; });
    add("File", "Undo", "Ctrl+Z", "back", Glyph::Undo, need(ctx.canUndo, "nothing to undo"), [&a] { a.undo = true; });
    add("File", "Redo", "Ctrl+Shift+Z", "again", Glyph::Redo, need(ctx.canRedo, "nothing to redo"), [&a] { a.redo = true; });

    // Create
    shape("Box", PrimitiveKind::Box, Glyph::Box);
    shape("Cylinder", PrimitiveKind::Cylinder, Glyph::Cylinder);
    shape("Sphere", PrimitiveKind::Sphere, Glyph::Sphere);
    shape("Cone", PrimitiveKind::Cone, Glyph::Cone);
    shape("Torus", PrimitiveKind::Torus, Glyph::Torus);
    shape("Plate", PrimitiveKind::Plane, Glyph::Plane);
    add("Create", "Sketch", "Shift+S", "draw line circle rectangle arc 2d", Glyph::Sketch, noKernel, [&a] { a.sketch = true; });
    add("Create", "Revolve", nullptr, "turn lathe spin", Glyph::Revolve, noKernel, [&a] { a.revolve = true; });
    add("Create", "Sweep", nullptr, "path pipe", Glyph::Sweep, noKernel, [&a] { a.sweep = true; });
    add("Create", "Loft", nullptr, "blend between outlines", Glyph::Loft, noKernel, [&a] { a.loft = true; });

    // Modify
    const char* selectSomething = "select something first";
    add("Modify", "Move", "G", "translate grab position", Glyph::Move, need(hasSel, selectSomething), [&a] { a.moveObject = true; });
    add("Modify", "Rotate", "R", "turn spin", Glyph::Rotate, need(hasSel, selectSomething), [&a] { a.rotateObject = true; });
    add("Modify", "Scale", "S", "resize size", Glyph::Scale, need(hasSel, selectSomething), [&a] { a.scaleObject = true; });
    add("Modify", "Extrude", "E", "boss pad push pull face", Glyph::Extrude, need(faces > 0, "select a face first"),
        [&a] { a.extrude = true; });
    add("Modify", "Extrude as a Cut", "Shift+E", "pocket remove", Glyph::Extrude, need(faces > 0, "select a face first"),
        [&a] { a.extrude = true; a.extrudeCut = true; });
    add("Modify", "Fillet or Chamfer", "F", "round bevel edge", Glyph::Fillet,
        either(noKernel, need(hasObject, "select a body, a face or an edge first")), [&a] { a.fillet = true; });
    add("Modify", "Hole", "H", "drill bore screw counterbore countersink", Glyph::Hole, noKernel, [&a] { a.hole = true; });
    add("Modify", "Thread", nullptr, "screw tap helix", Glyph::Thread, either(noKernel, need(faces > 0, "select a round face first")),
        [&a] { a.thread = true; });
    add("Modify", "Shell", nullptr, "hollow wall", Glyph::Shell, either(noKernel, need(hasObject, "select a body first")),
        [&a] { a.shell = true; });
    add("Modify", "Draft", nullptr, "taper angle mould", Glyph::Draft, either(noKernel, need(faces > 0, "select a face first")),
        [&a] { a.draft = true; });
    add("Modify", "Offset", nullptr, "grow shrink thicken", Glyph::Offset, either(noKernel, need(hasObject, "select a body first")),
        [&a] { a.offset = true; });
    add("Modify", "Inset Face", nullptr, "border", Glyph::Inset, need(faces > 0, "select a face first"), [&a] { a.inset = true; });
    add("Modify", "Split", nullptr, "cut in two slice", Glyph::Split, either(noKernel, need(hasObject, "select a body first")),
        [&a] { a.split = true; });
    add("Modify", "Divide", "K", "split edge face knife", Glyph::Divide, need(edges > 0 || faces > 0, "select an edge or a face first"),
        [&a] { a.divide = true; });
    add("Modify", "Pattern", "P", "array repeat copy row ring", Glyph::Pattern, need(hasObject, "select a body first"),
        [&a] { a.pattern = true; });
    add("Modify", "Mirror", "M", "reflect symmetry", Glyph::Mirror, need(hasObject, "select a body first"), [&a] { a.mirror = true; });
    add("Modify", "Combine", "Ctrl+Shift+U", "boolean join union", Glyph::Boolean, need(bodies >= 2, "needs two bodies"),
        [&a] { a.booleanRequested = true; a.booleanOp = BooleanOp::Union; });
    add("Modify", "Cut", "Ctrl+Shift+D", "boolean subtract difference", Glyph::Difference, need(bodies >= 2, "needs two bodies"),
        [&a] { a.booleanRequested = true; a.booleanOp = BooleanOp::Difference; });
    add("Modify", "Intersect", "Ctrl+Shift+I", "boolean common", Glyph::Intersect, need(bodies >= 2, "needs two bodies"),
        [&a] { a.booleanRequested = true; a.booleanOp = BooleanOp::Intersection; });
    add("Modify", "Merge Faces", nullptr, "clean tidy", Glyph::Merge, need(hasObject, "select a body first"), [&a] { a.mergeFaces = true; });
    add("Modify", "Delete Face", nullptr, "remove heal", Glyph::DeleteFace, either(noKernel, need(faces > 0, "select a face first")),
        [&a] { a.deleteFace = true; });
    add("Modify", "Duplicate", "Shift+D", "copy clone", Glyph::Duplicate, need(hasSel, selectSomething), [&a] { a.duplicateSelected = true; });
    add("Modify", "Delete", "X", "remove erase", Glyph::Trash, need(hasSel, selectSomething), [&a] { a.deleteSelected = true; });
    add("Modify", "Convert to Solid", nullptr, "mesh stl brep", Glyph::Convert, need(isMesh, "for an imported mesh"),
        [&a] { a.convertToSolid = true; });
    add("Modify", "Reduce Mesh", nullptr, "decimate simplify", Glyph::Reduce, need(isMesh, "for an imported mesh"),
        [&a] { a.reduceMesh = true; });

    // Assemble
    add("Assemble", "Joint", "J", "mate connect hinge pin slider", Glyph::Joint, need(bodies >= 2, "needs two parts"),
        [&a] { a.joint = true; });
    add("Assemble", "Group", "Ctrl+G", "sub-assembly", Glyph::Group, need(!scene.selection().empty(), "select parts first"),
        [&a] { a.groupSelected = true; });
    add("Assemble", "Ungroup", "Ctrl+Shift+G", "", Glyph::Group, need(!scene.selection().empty(), "select parts first"),
        [&a] { a.ungroupSelected = true; });
    add("Assemble", "Clearance", nullptr, "gap interference collision fit tolerance", Glyph::Clearance,
        need(bodies >= 2, "needs two parts"), [&a] { a.clearance = true; });
    add("Assemble", "Exploded View", nullptr, "apart instructions", Glyph::Explode, need(bodies >= 2, "needs two parts"),
        [&a] { a.explode = true; });

    // Inspect and view
    ViewOptions& v = *ctx.view;
    Camera& cam = *ctx.camera;
    add("Inspect", "Measure", "D", "distance ruler dimension angle", Glyph::Measure, nullptr, [&a] { a.toggleMeasure = true; });
    add("Inspect", "Section View", "V", "cut clip plane inside cross-section", Glyph::Section, nullptr, [&a] { a.section = true; });
    add("Inspect", v.showPrintIssues ? "Hide Print Problems" : "Show Print Problems", nullptr, "thin walls check",
        Glyph::Alert, nullptr, [&v] { v.showPrintIssues = !v.showPrintIssues; });
    add("View", v.showGrid ? "Hide Grid" : "Show Grid", nullptr, "floor ground", Glyph::Grid, nullptr,
        [&v] { v.showGrid = !v.showGrid; });
    add("View", v.showWireframe ? "Hide Edges" : "Show Edges", "Z", "wireframe lines", Glyph::Wire, nullptr,
        [&v] { v.showWireframe = !v.showWireframe; });
    add("View", v.showKeyHints ? "Hide Key Hints" : "Show Key Hints", nullptr, "shortcuts keyboard help reminder",
        Glyph::Help, nullptr, [&v] { v.showKeyHints = !v.showKeyHints; });
    add("View", cam.orthographic ? "Perspective" : "Orthographic", "Numpad 5", "projection camera", Glyph::Orthographic,
        nullptr, [&cam] { cam.setOrthographic(!cam.orthographic); });
    add("View", "Frame Selected", "Numpad .", "zoom fit focus", Glyph::FrameSelected, need(hasSel, selectSomething),
        [&a] { a.frameSelected = true; });
    add("View", "Frame All", "Home", "zoom fit everything", Glyph::FrameAll, nullptr, [&a] { a.frameAll = true; });
    struct V { const char* name; const char* keys; StandardView view; };
    static const V kViews[] = {{"Front View", "Numpad 1", StandardView::Front}, {"Back View", "Ctrl+Numpad 1", StandardView::Back},
                               {"Right View", "Numpad 3", StandardView::Right}, {"Left View", "Ctrl+Numpad 3", StandardView::Left},
                               {"Top View", "Numpad 7", StandardView::Top},     {"Bottom View", "Ctrl+Numpad 7", StandardView::Bottom}};
    for (const V& sv : kViews)
        add("View", sv.name, sv.keys, "look camera", Glyph::Eye, nullptr, [&cam, view = sv.view] { cam.setStandardView(view); });
    return c;
}

} // namespace

void openCommandPalette(const char* query) {
    g_open = true;
    g_focus = true;
    std::snprintf(g_query, sizeof g_query, "%s", query ? query : "");
    g_selected = 0;
}

bool commandPaletteOpen() { return g_open; }


void drawCommandPalette(UiContext& ctx) {
    if (!g_open) return;
    std::vector<Command> all = commands(ctx);

    // The query, lower case, spaces dropped: "fil ch" finds Fillet or Chamfer.
    std::string q;
    for (const char* p = g_query; *p; ++p)
        if (*p != ' ') q += static_cast<char>(std::tolower(static_cast<unsigned char>(*p)));
    struct Hit { const Command* c; int s; };
    std::vector<Hit> hits;
    for (const Command& c : all) {
        int s = score(c.name, q);
        if (s < 0) {
            // Its other words count for less, and only a word that begins
            // with what is typed: "cham" finds Fillet or Chamfer, and "fil"
            // does not find every command with an f, i and l in it.
            // The group's whole name lists the group.
            int t = -1;
            std::string word;
            for (const char* p = c.also ? c.also : ""; ; ++p) {
                if (*p && *p != ' ') { word += static_cast<char>(std::tolower(static_cast<unsigned char>(*p))); continue; }
                if (!word.empty() && word.rfind(q, 0) == 0) t = std::max(t, 4 + static_cast<int>(q.size()));
                word.clear();
                if (!*p) break;
            }
            std::string group;
            for (const char* p = c.group; *p; ++p) group += static_cast<char>(std::tolower(static_cast<unsigned char>(*p)));
            if (group == q) t = std::max(t, 2);
            if (t < 0) continue;
            s = t;
        }
        if (c.why) s -= 4;                // what can run now comes first
        hits.push_back({&c, s});
    }
    if (!q.empty())
        std::stable_sort(hits.begin(), hits.end(), [](const Hit& a, const Hit& b) { return a.s > b.s; });
    g_selected = hits.empty() ? 0 : std::clamp(g_selected, 0, static_cast<int>(hits.size()) - 1);

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float width = std::min(560.0f, vp->WorkSize.x - 32.0f);
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + (vp->WorkSize.x - width) * 0.5f, vp->WorkPos.y + 84.0f));
    ImGui::SetNextWindowSize(ImVec2(width, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 10.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, im(palette::kCommand));
    ImGui::PushStyleColor(ImGuiCol_Border, im(palette::kBorderStrong));
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoSavedSettings |
                                   ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoMove |
                                   ImGuiWindowFlags_AlwaysAutoResize;
    bool close = false;
    const Command* chosen = nullptr;
    if (ImGui::Begin("##palette", nullptr, flags)) {
        // The search line.
        pushFont(FontWeight::Medium, uiFonts().size * 1.05f);
        ImGui::SetNextItemWidth(-1.0f);
        if (g_focus) { ImGui::SetKeyboardFocusHere(); g_focus = false; }
        ImGui::PushStyleColor(ImGuiCol_FrameBg, im(palette::kField));
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10.0f, 8.0f));
        if (ImGui::InputTextWithHint("##q", "Type a command: fillet, section, inches...", g_query, sizeof g_query))
            g_selected = 0;
        ImGui::PopStyleVar();
        ImGui::PopStyleColor();
        ImGui::PopFont();

        if (ImGui::IsKeyPressed(ImGuiKey_DownArrow)) g_selected = std::min(g_selected + 1, static_cast<int>(hits.size()) - 1);
        if (ImGui::IsKeyPressed(ImGuiKey_UpArrow))   g_selected = std::max(g_selected - 1, 0);
        if (ImGui::IsKeyPressed(ImGuiKey_PageDown))  g_selected = std::min(g_selected + 8, static_cast<int>(hits.size()) - 1);
        if (ImGui::IsKeyPressed(ImGuiKey_PageUp))    g_selected = std::max(g_selected - 8, 0);
        if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) close = true;
        const bool enter = ImGui::IsKeyPressed(ImGuiKey_Enter, false) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false);
        if (enter && !hits.empty() && !hits[static_cast<size_t>(g_selected)].c->why) chosen = hits[static_cast<size_t>(g_selected)].c;

        ImGui::Dummy(ImVec2(0, 4));
        const float rowH = 30.0f;
        const float listH = std::min(static_cast<float>(hits.size()), 11.0f) * rowH;
        if (hits.empty()) {
            ImGui::TextColored(im(palette::kTextDim), "  Nothing by that name");
        } else if (ImGui::BeginChild("##list", ImVec2(0, listH), ImGuiChildFlags_None)) {
            ImDrawList* dl = ImGui::GetWindowDrawList();
            for (size_t i = 0; i < hits.size(); ++i) {
                const Command& c = *hits[i].c;
                ImGui::PushID(static_cast<int>(i));
                const ImVec2 at = ImGui::GetCursorScreenPos();
                const float w = ImGui::GetContentRegionAvail().x;
                const bool clicked = ImGui::InvisibleButton("##row", ImVec2(w, rowH));
                const bool hot = ImGui::IsItemHovered();
                if (hot && ImGui::GetIO().MouseDelta.x != 0.0f) g_selected = static_cast<int>(i);
                const bool sel = static_cast<int>(i) == g_selected;
                if (sel) {
                    dl->AddRectFilled(at, ImVec2(at.x + w, at.y + rowH), u32(palette::kHover), 6.0f);
                    if (enter || ImGui::IsKeyPressed(ImGuiKey_DownArrow) || ImGui::IsKeyPressed(ImGuiKey_UpArrow))
                        ImGui::SetScrollHereY(0.5f);
                }
                const float alpha = c.why ? 0.45f : 1.0f;
                drawGlyph(dl, c.glyph, ImVec2(at.x + 16.0f, at.y + rowH * 0.5f), 16.0f, u32(palette::kBrand, alpha));
                pushFont(FontWeight::Medium, uiFonts().size * 0.95f);
                const float th = ImGui::GetTextLineHeight();
                dl->AddText(ImVec2(at.x + 34.0f, at.y + (rowH - th) * 0.5f), u32(palette::kText, alpha), c.name);
                const float nw = ImGui::CalcTextSize(c.name).x;
                ImGui::PopFont();
                // What it belongs to, or -- selected and unable to run -- why.
                pushFont(FontWeight::Regular, uiFonts().size * 0.85f);
                const char* aside = sel && c.why ? c.why : c.group;
                dl->AddText(ImVec2(at.x + 34.0f + nw + 10.0f, at.y + (rowH - ImGui::GetTextLineHeight()) * 0.5f),
                            u32(sel && c.why ? palette::kWarn : palette::kTextFaint), aside);
                if (c.keys) {
                    const ImVec2 ks = ImGui::CalcTextSize(c.keys);
                    const ImVec2 k0(at.x + w - ks.x - 18.0f, at.y + (rowH - ks.y) * 0.5f - 3.0f);
                    dl->AddRectFilled(k0, ImVec2(k0.x + ks.x + 12.0f, k0.y + ks.y + 6.0f), u32(palette::kRaised), 4.0f);
                    dl->AddText(ImVec2(k0.x + 6.0f, k0.y + 3.0f), u32(palette::kTextDim), c.keys);
                }
                ImGui::PopFont();
                if (clicked && !c.why) chosen = &c;
                ImGui::PopID();
            }
        }
        if (!hits.empty()) ImGui::EndChild();

        // Out of it by clicking anywhere else.
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
            !ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows | ImGuiHoveredFlags_AllowWhenBlockedByActiveItem))
            close = true;
    }
    ImGui::End();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar(3);

    if (chosen) {
        chosen->run();
        close = true;
    }
    if (close) g_open = false;
}

} // namespace tg::ui
