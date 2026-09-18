// Tangent - the bar along the top.
//
// The logo, then the tools in four groups -- File, Create, Modify, Inspect --
// each a row of pictures with its name under it. The name is a menu: it opens
// the group's full list, with every command the pictures stand for and the
// ones there was no room for, each with its key. So the bar is the short way
// to the common things and the menu under it is the long way to all of them,
// and a person who knows one can find the other.
//
// When the application draws its own window frame the bar is also the title
// bar: empty parts of it drag the window, and the right-hand end holds the
// window's own three buttons.
#include "ui/panels.h"
#include "ui/glyph.h"
#include "ui/icons.h"
#include "ui/png.h"
#include "ui/theme.h"
#include "ui/widgets.h"

#include "core/palette.h"
#include "geom/brep.h"

#include <epoxy/gl.h>

#include "imgui.h"
#include "imgui_internal.h"

#include <algorithm>
#include <cstdio>

namespace tg {
namespace {

using ui::im;
using ui::u32;

GLuint g_logoTex = 0;
int    g_logoW = 0, g_logoH = 0;

constexpr float kIconPx   = 20.0f;
constexpr float kBarPadX  = 14.0f;
constexpr float kGroupGap = 26.0f;

// Records the item just laid out as a place the window must not drag from.
void keepFromDragging(UiContext& ctx) {
    const ImVec2 lo = ImGui::GetItemRectMin(), hi = ImGui::GetItemRectMax();
    const ImVec2 origin = ImGui::GetMainViewport()->Pos;
    ctx.frame.noDrag.push_back({lo.x - origin.x, lo.y - origin.y, hi.x - origin.x, hi.y - origin.y});
}

bool barButton(UiContext& ctx, const char* id, Glyph g, const char* tip, bool enabled = true,
               bool active = false, ImU32 tint = 0) {
    const bool clicked = glyphButton(id, g, kIconPx, tip, active, enabled, tint);
    keepFromDragging(ctx);
    return clicked;
}

// A small chevron in the corner of the item just drawn: it opens something.
void dropMark() {
    const ImVec2 hi = ImGui::GetItemRectMax();
    drawGlyph(ImGui::GetWindowDrawList(), Glyph::ChevronDown, ImVec2(hi.x - 5.0f, hi.y - 5.0f),
              8.0f, u32(palette::kTextDim), 1.2f);
}

// ---- the menus under the groups ---------------------------------------------
void fileMenu(UiContext& ctx) {
    using namespace ui;
    if (menuEntry(Glyph::New,  "New",        "Ctrl+N")) ctx.actions.newProject = true;
    if (menuEntry(Glyph::Open, "Open...",    "Ctrl+O")) ctx.actions.openProject = true;
    if (menuEntry(Glyph::Save, "Save",       "Ctrl+S")) ctx.actions.saveProject = true;
    if (menuEntry(Glyph::Save, "Save As...", nullptr))  ctx.actions.saveProjectAs = true;
    menuGap();
    if (menuEntry(Glyph::Import, "Import STEP...")) ctx.actions.importStep = true;
    menuNote("the surfaces themselves, not a mesh of them");
    if (menuEntry(Glyph::Mesh, "Import Mesh...")) ctx.actions.importMesh = true;
    menuNote(".stl or .obj, as triangles");
    menuGap();
    if (menuEntry(Glyph::Export, "Export STEP...")) ctx.actions.exportStep = true;
    menuNote("exact; what another CAD package wants");
    if (menuEntry(Glyph::Export, "Export 3MF...", "Ctrl+Shift+E")) ctx.actions.export3mf = true;
    menuNote("for a slicer: millimetres, each part named");
    if (menuEntry(Glyph::Export, "Export STL...", "Ctrl+E")) ctx.actions.exportStl = true;
    menuNote("loose triangles; what every slicer reads");
    menuGap();
    if (menuEntry(Glyph::Undo, "Undo", "Ctrl+Z", ctx.canUndo)) ctx.actions.undo = true;
    if (menuEntry(Glyph::Redo, "Redo", "Ctrl+Shift+Z", ctx.canRedo)) ctx.actions.redo = true;
    menuGap();
    if (menuEntry(Glyph::Close, "Quit", "Ctrl+Q")) ctx.actions.quit = true;
}

void createMenu(UiContext& ctx) {
    using namespace ui;
    menuHeader("Shapes");
    drawAddMenuItems(ctx);
    menuGap();
    const bool has = !ctx.scene->selection().empty();
    if (menuEntry(Glyph::Plus, "Duplicate", "Shift+D", has)) ctx.actions.duplicateSelected = true;
    if (menuEntry(Glyph::Trash, "Delete", "X", has)) ctx.actions.deleteSelected = true;
    menuGap();
    if (menuEntry(Glyph::Select, "Select All", "A")) ctx.scene->selectAll();
    if (menuEntry(Glyph::Count, "Deselect All", "Alt+A")) ctx.scene->clearSelection();
}

void modifyMenu(UiContext& ctx) {
    using namespace ui;
    Scene& scene = *ctx.scene;
    const ObjectId ctxObj = scene.contextObject();
    const bool hasObject = ctxObj != kNoObject;
    const bool hasSel = !scene.selection().empty();
    const size_t selFaces = scene.selectedFaces(ctxObj).size();
    const size_t selEdges = scene.selectedEdges(ctxObj).size();
    const bool pair = scene.selection().size() == 2;

    menuHeader("Body");
    if (menuEntry(Glyph::Move,   "Move",   "G", hasSel)) ctx.actions.moveObject = true;
    if (menuEntry(Glyph::Rotate, "Rotate", "R", hasSel)) ctx.actions.rotateObject = true;
    if (menuEntry(Glyph::Scale,  "Scale",  "S", hasSel)) ctx.actions.scaleObject = true;
    menuNote("with a face selected these move the face instead");

    menuHeader("Face");
    if (menuEntry(Glyph::PushPull, "Push / Pull Face", "G", selFaces > 0)) ctx.actions.pushPull = true;
    if (menuEntry(Glyph::Extrude, "Extrude Face", "E", selFaces > 0)) ctx.actions.extrude = true;
    menuNote("keeps the outline of the boss; Shift+E cuts inward");
    if (menuEntry(Glyph::RotateFace, "Rotate Face", "R", selFaces > 0)) ctx.actions.rotateFace = true;
    if (menuEntry(Glyph::ScaleFace, "Scale Face", "S", selFaces > 0)) ctx.actions.scaleFace = true;
    if (menuEntry(Glyph::Inset, "Inset Face", nullptr, selFaces > 0)) ctx.actions.inset = true;
    {
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 34.0f);
        ImGui::SetNextItemWidth(120.0f);
        ImGui::DragScalarN("##inset", ImGuiDataType_Double, &ctx.view->insetAmount, 1, 0.05f,
                           nullptr, nullptr, "%.2f mm");
        ImGui::SameLine();
        ImGui::TextColored(im(palette::kTextFaint), "in from its edge");
    }

    menuHeader("Edges");
    if (menuEntry(Glyph::Fillet, "Fillet / Chamfer", "F", selEdges > 0 || selFaces > 0)) ctx.actions.fillet = true;
    menuNote(selEdges > 0 ? "the selected edges" : selFaces > 0 ? "the edges around the selected faces"
                                                                 : "select an edge or a face first");
    if (menuEntry(Glyph::Fillet, "Round All Edges", "Ctrl+B", hasObject)) ctx.actions.bevel = true;
    if (menuEntry(Glyph::Divide, "Divide Across an Edge", "K", selEdges > 0)) ctx.actions.divide = true;
    if (menuEntry(Glyph::Merge, "Merge Faces", nullptr, hasObject)) ctx.actions.mergeFaces = true;
    menuNote("drops every division that does not define the shape");

    menuHeader("Whole body");
    if (menuEntry(Glyph::Shell, "Shell", nullptr, hasObject)) ctx.actions.shell = true;
    {
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 34.0f);
        ImGui::SetNextItemWidth(120.0f);
        ImGui::DragScalarN("##wall", ImGuiDataType_Double, &ctx.view->shellThickness, 1, 0.05f,
                           nullptr, nullptr, "%.2f mm");
        ImGui::SameLine();
        ImGui::TextColored(im(palette::kTextFaint),
                           selFaces > 0 ? "wall; selected faces left open" : "wall; sealed, no face open");
    }
    if (menuEntry(Glyph::Pattern, "Pattern...", "P", hasObject)) ctx.actions.pattern = true;
    if (menuEntry(Glyph::Mirror,  "Mirror...",  "M", hasObject)) ctx.actions.mirror = true;
    if (menuEntry(Glyph::Split,   "Split Body", nullptr, hasObject)) ctx.actions.split = true;
    menuNote("by a face's plane, a tool plane, or into its shells");

    menuHeader("Combine two bodies");
    const char* note = pair ? "the first selected is kept, the second is the tool"
                            : "select two bodies first";
    if (menuEntry(Glyph::Union, "Join", "Ctrl+Shift+U", pair)) {
        ctx.actions.booleanRequested = true; ctx.actions.booleanOp = BooleanOp::Union;
    }
    if (menuEntry(Glyph::Difference, "Cut", "Ctrl+Shift+D", pair)) {
        ctx.actions.booleanRequested = true; ctx.actions.booleanOp = BooleanOp::Difference;
    }
    if (menuEntry(Glyph::Intersect, "Intersect", "Ctrl+Shift+I", pair)) {
        ctx.actions.booleanRequested = true; ctx.actions.booleanOp = BooleanOp::Intersection;
    }
    menuNote(note);

    const SceneObject* o = scene.find(ctxObj);
    const bool isMesh = o && !o->body.empty() && o->body.isMesh();
    menuHeader("Mesh");
    if (menuEntry(Glyph::Reduce, "Reduce Mesh...", nullptr, isMesh)) ctx.actions.reduceMesh = true;
    if (menuEntry(Glyph::Convert, "Convert to Solid", nullptr, isMesh)) ctx.actions.convertToSolid = true;
    menuNote(isMesh ? "sews the triangles and merges the flat ones"
             : o    ? "this body is already exact"
                    : "for an imported mesh");
}

void inspectMenu(UiContext& ctx) {
    using namespace ui;
    if (menuEntry(Glyph::Measure, "Measure", "D", true, ctx.measuring)) ctx.actions.toggleMeasure = true;
    menuNote("one thing for its size, two for the distance between");
    menuGap();
    menuToggle(Glyph::Alert, "Print Problems", &ctx.view->showPrintIssues);
    menuNote("red: thinner than the nozzle can lay");
    menuToggle(Glyph::Grid, "Grid", &ctx.view->showGrid);
    menuToggle(Glyph::Wire, "Wireframe", &ctx.view->showWireframe, "Z");
    menuToggle(Glyph::Frame, "Selection Box", &ctx.view->showSelectionBox);
    menuToggle(Glyph::Cube, "Backface Cull", &ctx.view->backfaceCulling);
    menuGap();
    menuHeader("View");
    if (menuEntry(Glyph::Frame, "Frame Selected", "Numpad .")) ctx.actions.frameSelected = true;
    if (menuEntry(Glyph::Frame, "Frame All", "Home")) ctx.actions.frameAll = true;
    bool ortho = ctx.camera->orthographic;
    if (menuToggle(Glyph::Camera, "Orthographic", &ortho, "Numpad 5")) ctx.camera->setOrthographic(ortho);
    menuToggle(Glyph::Rotate, "Invert Orbit X", &ctx.camera->invertOrbitX);
    menuToggle(Glyph::Rotate, "Invert Orbit Y", &ctx.camera->invertOrbitY);
    menuGap();
    char timing[64];
    std::snprintf(timing, sizeof timing, "%.1f fps   %.2f ms",
                  ctx.stats.frameMs > 0.0f ? 1000.0f / ctx.stats.frameMs : 0.0f, ctx.stats.frameMs);
    menuNote(timing);
}

// The caption under a group, which is also the handle of its menu.
void groupCaption(UiContext& ctx, const char* name, const char* popupId, float x0, float x1,
                  void (*menu)(UiContext&)) {
    ImGui::PushID(popupId);
    pushFont(FontWeight::Medium, uiFonts().size * 0.74f);
    const float h = ImGui::GetTextLineHeight() + 4.0f;
    const ImVec2 at(x0, ImGui::GetCursorScreenPos().y);
    ImGui::SetCursorScreenPos(at);
    const bool clicked = ImGui::InvisibleButton("##cap", ImVec2(std::max(20.0f, x1 - x0), h));
    const bool hovered = ImGui::IsItemHovered();
    keepFromDragging(ctx);

    // The word, spaced out, with a chevron after it, centred under the group.
    ImDrawList* dl = ImGui::GetWindowDrawList();
    float width = 0.0f;
    const float space = 1.3f;
    for (const char* p = name; *p; ++p) { char c[2] = {*p, 0}; width += ImGui::CalcTextSize(c).x + space; }
    width += 12.0f;
    float x = at.x + (x1 - x0 - width) * 0.5f;
    const ImU32 col = hovered ? u32(palette::kText) : u32(palette::kTextDim);
    for (const char* p = name; *p; ++p) {
        char c[2] = {*p, 0};
        dl->AddText(ImVec2(x, at.y + 2.0f), col, c);
        x += ImGui::CalcTextSize(c).x + space;
    }
    drawGlyph(dl, Glyph::ChevronDown, ImVec2(x + 5.0f, at.y + h * 0.5f), 9.0f, col, 1.2f);
    ImGui::PopFont();

    if (clicked) ImGui::OpenPopup(popupId);
    ImGui::SetNextWindowPos(ImVec2(at.x, at.y + h + 4.0f));
    if (ui::beginMenuPopup(popupId)) {
        ctx.frame.popupOpen = true;
        menu(ctx);
        ImGui::EndPopup();
    }
    ImGui::PopID();
}

} // namespace

bool loadBrandAssets(const std::string& assetDir) {
    unloadBrandAssets();
    std::vector<unsigned char> px;
    if (!readPng(assetDir + "/logo.png", g_logoW, g_logoH, px)) {
        std::fprintf(stderr, "[ui] no logo at %s/logo.png; the bar shows the wordmark in text\n",
                     assetDir.c_str());
        return false;
    }
    glGenTextures(1, &g_logoTex);
    glBindTexture(GL_TEXTURE_2D, g_logoTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, g_logoW, g_logoH, 0, GL_RGBA, GL_UNSIGNED_BYTE, px.data());
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
    return true;
}

void unloadBrandAssets() {
    if (g_logoTex) glDeleteTextures(1, &g_logoTex);
    g_logoTex = 0;
    g_logoW = g_logoH = 0;
}

float drawTopBar(UiContext& ctx) {
    Scene& scene = *ctx.scene;
    const ObjectId ctxObj = scene.contextObject();
    const bool hasObject = ctxObj != kNoObject;
    const bool hasSel = !scene.selection().empty();
    const bool pair = scene.selection().size() == 2;
    const size_t edges = scene.selectedEdges(ctxObj).size();
    const size_t faces = scene.selectedFaces(ctxObj).size();

    ctx.frame.noDrag.clear();
    ctx.frame.popupOpen = false;

    const ImGuiStyle& st = ImGui::GetStyle();
    const float buttonH = kIconPx + st.FramePadding.y * 2.0f;
    const float captionH = uiFonts().size * 0.74f + 4.0f + 3.0f;
    const float height = 6.0f + buttonH + 2.0f + captionH + 6.0f;
    ctx.frame.barHeight = height;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(kBarPadX, 6.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(2.0f, 2.0f));
    ImGui::PushStyleColor(ImGuiCol_ChildBg, im(palette::kTopBar));
    ImGui::BeginChild("##topbar", ImVec2(0.0f, height), ImGuiChildFlags_AlwaysUseWindowPadding,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 barMin = ImGui::GetWindowPos();
    const ImVec2 barMax(barMin.x + ImGui::GetWindowWidth(), barMin.y + height);
    dl->AddLine(ImVec2(barMin.x, barMax.y - 0.5f), ImVec2(barMax.x, barMax.y - 0.5f), u32(palette::kBorder));

    // ---- the logo ---------------------------------------------------------
    const float rowY = ImGui::GetCursorPosY();
    {
        const float logoH = 26.0f;
        if (g_logoTex && g_logoH > 0) {
            const float logoW = logoH * static_cast<float>(g_logoW) / static_cast<float>(g_logoH);
            ImGui::SetCursorPosY(rowY + (buttonH + captionH - logoH) * 0.5f);
            ImGui::Image(static_cast<ImTextureID>(g_logoTex), ImVec2(logoW, logoH));
        } else {
            pushFont(FontWeight::Bold, uiFonts().size * 1.55f);
            ImGui::SetCursorPosY(rowY + (buttonH + captionH - ImGui::GetTextLineHeight()) * 0.5f);
            ImGui::TextColored(im(palette::kBrand), "tangent");
            ImGui::PopFont();
        }
        // The project's name, small, beside it: the one thing a title bar
        // is for.
        if (!ctx.projectName.empty()) {
            ImGui::SameLine(0.0f, 14.0f);
            pushFont(FontWeight::Regular, uiFonts().size * 0.86f);
            ImGui::SetCursorPosY(rowY + (buttonH + captionH - ImGui::GetTextLineHeight()) * 0.5f);
            ImGui::TextColored(im(palette::kTextDim), "%s%s", ctx.projectName.c_str(),
                               ctx.dirty ? " *" : "");
            ImGui::PopFont();
        }
    }

    // ---- the groups -------------------------------------------------------
    struct Group { const char* name; const char* popup; void (*menu)(UiContext&); float x0, x1; };
    Group groups[4] = {{"FILE", "##m_file", fileMenu, 0, 0},
                       {"CREATE", "##m_create", createMenu, 0, 0},
                       {"MODIFY", "##m_modify", modifyMenu, 0, 0},
                       {"INSPECT", "##m_inspect", inspectMenu, 0, 0}};

    float x = std::max(ImGui::GetCursorPosX() + 20.0f, 150.0f);
    ImGui::SetCursorPos(ImVec2(x, rowY));

    auto beginGroup = [&](int i) {
        ImGui::SetCursorPos(ImVec2(x, rowY));
        groups[i].x0 = ImGui::GetCursorScreenPos().x;
    };
    auto endGroup = [&](int i) {
        groups[i].x1 = ImGui::GetItemRectMax().x;
        x = groups[i].x1 - ImGui::GetWindowPos().x + kGroupGap;
    };

    // File
    beginGroup(0);
    if (barButton(ctx, "new", Glyph::New, "New project  (Ctrl+N)")) ctx.actions.newProject = true;
    ImGui::SameLine();
    if (barButton(ctx, "open", Glyph::Open, "Open...  (Ctrl+O)")) ctx.actions.openProject = true;
    ImGui::SameLine();
    if (barButton(ctx, "save", Glyph::Save, "Save  (Ctrl+S)")) ctx.actions.saveProject = true;
    ImGui::SameLine();
    if (barButton(ctx, "settings", Glyph::Settings, "View and display settings"))
        ImGui::OpenPopup("##m_settings");
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetItemRectMin().x, ImGui::GetItemRectMax().y + 4.0f));
    if (ui::beginMenuPopup("##m_settings")) {
        ctx.frame.popupOpen = true;
        inspectMenu(ctx);
        ImGui::EndPopup();
    }
    endGroup(0);

    // Create
    beginGroup(1);
    {
        struct Shape { Icon icon; Glyph glyph; PrimitiveKind kind; const char* name; };
        static const Shape kShapes[] = {
            {Icon::Box,      Glyph::Box,      PrimitiveKind::Box,      "Box"},
            {Icon::Cylinder, Glyph::Cylinder, PrimitiveKind::Cylinder, "Cylinder"},
            {Icon::Sphere,   Glyph::Sphere,   PrimitiveKind::Sphere,   "Sphere"},
            {Icon::Cone,     Glyph::Cone,     PrimitiveKind::Cone,     "Cone"},
            {Icon::Torus,    Glyph::Torus,    PrimitiveKind::Torus,    "Torus"},
        };
        static int lastShape = 0;
        if (barButton(ctx, "shape", kShapes[lastShape].glyph, "Create a shape  (Shift+A)"))
            ImGui::OpenPopup("##createobject");
        dropMark();
        ImGui::SetNextWindowPos(ImVec2(ImGui::GetItemRectMin().x, ImGui::GetItemRectMax().y + 4.0f));
        if (ui::beginMenuPopup("##createobject")) {
            ctx.frame.popupOpen = true;
            ui::menuHeader("Start from");
            const float line = ImGui::GetTextLineHeight() + 6.0f;
            for (int i = 0; i < static_cast<int>(sizeof kShapes / sizeof kShapes[0]); ++i) {
                ImGui::PushID(i);
                // The baked render: a picture of the actual shape, which is
                // what a choice between shapes wants.
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 8.0f);
                iconImage(kShapes[i].icon, line);
                ImGui::SameLine(0.0f, 8.0f);
                ImGui::AlignTextToFramePadding();
                if (ImGui::Selectable(kShapes[i].name, false, 0, ImVec2(150.0f, line))) {
                    ctx.actions.addRequested = true;
                    ctx.actions.addKind = kShapes[i].kind;
                    lastShape = i;
                }
                ImGui::PopID();
            }
            ImGui::EndPopup();
        }
    }
    ImGui::SameLine();
    if (barButton(ctx, "sketch", Glyph::Sketch,
                  brep::available() ? "Sketch  (Shift+S): lines, circles and arcs on a plane, then extrude"
                                    : "Sketching needs the exact kernel, which this build does not have",
                  brep::available()))
        ctx.actions.sketch = true;
    ImGui::SameLine();
    if (barButton(ctx, "plane", Glyph::Plane, "A flat plate to start from")) {
        ctx.actions.addRequested = true;
        ctx.actions.addKind = PrimitiveKind::Plane;
    }
    ImGui::SameLine();
    if (barButton(ctx, "import", Glyph::Import, "Import a STEP file or a mesh"))
        ImGui::OpenPopup("##m_import");
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetItemRectMin().x, ImGui::GetItemRectMax().y + 4.0f));
    if (ui::beginMenuPopup("##m_import")) {
        ctx.frame.popupOpen = true;
        if (ui::menuEntry(Glyph::Import, "Import STEP...")) ctx.actions.importStep = true;
        ui::menuNote("exact surfaces from another CAD package");
        if (ui::menuEntry(Glyph::Mesh, "Import Mesh...")) ctx.actions.importMesh = true;
        ui::menuNote(".stl or .obj");
        ImGui::EndPopup();
    }
    endGroup(1);

    // Modify
    beginGroup(2);
    if (barButton(ctx, "move", Glyph::Move,
                  faces ? "Push / pull the face  (G)" : hasSel ? "Move  (G)" : "Move - select something first",
                  hasSel || faces)) {
        if (faces) ctx.actions.pushPull = true;
        else       ctx.actions.moveObject = true;
    }
    ImGui::SameLine();
    {
        const char* firstName = "the first";
        const char* secondName = "the second";
        if (pair) {
            if (const SceneObject* a = scene.find(scene.selection()[0])) firstName = a->name.c_str();
            if (const SceneObject* b = scene.find(scene.selection()[1])) secondName = b->name.c_str();
        }
        char tip[160];
        if (pair) std::snprintf(tip, sizeof tip, "Combine %s with %s", firstName, secondName);
        else      std::snprintf(tip, sizeof tip, "Combine - select two bodies (%zu selected)",
                                scene.selection().size());
        if (barButton(ctx, "boolean", Glyph::Boolean, tip, pair)) ImGui::OpenPopup("##m_boolean");
        dropMark();
        ImGui::SetNextWindowPos(ImVec2(ImGui::GetItemRectMin().x, ImGui::GetItemRectMax().y + 4.0f));
        if (ui::beginMenuPopup("##m_boolean")) {
            ctx.frame.popupOpen = true;
            char line[192];
            std::snprintf(line, sizeof line, "Join:  %s + %s", firstName, secondName);
            if (ui::menuEntry(Glyph::Union, line, "Ctrl+Shift+U")) {
                ctx.actions.booleanRequested = true; ctx.actions.booleanOp = BooleanOp::Union;
            }
            std::snprintf(line, sizeof line, "Cut:  %s minus %s", firstName, secondName);
            if (ui::menuEntry(Glyph::Difference, line, "Ctrl+Shift+D")) {
                ctx.actions.booleanRequested = true; ctx.actions.booleanOp = BooleanOp::Difference;
            }
            std::snprintf(line, sizeof line, "Intersect:  %s with %s", firstName, secondName);
            if (ui::menuEntry(Glyph::Intersect, line, "Ctrl+Shift+I")) {
                ctx.actions.booleanRequested = true; ctx.actions.booleanOp = BooleanOp::Intersection;
            }
            ImGui::EndPopup();
        }
    }
    ImGui::SameLine();
    if (barButton(ctx, "extrude", Glyph::Extrude,
                  faces ? "Extrude the face: a boss with its own outline  (E)"
                        : "Extrude - select a face first", faces > 0))
        ctx.actions.extrude = true;
    ImGui::SameLine();
    if (barButton(ctx, "fillet", Glyph::Fillet,
                  edges || faces ? "Fillet or chamfer the selected edges  (F)"
                                 : "Fillet - select an edge or a face first", edges > 0 || faces > 0))
        ctx.actions.fillet = true;
    ImGui::SameLine();
    if (barButton(ctx, "shell", Glyph::Shell,
                  hasObject ? "Shell: hollow the body, selected faces left open"
                            : "Shell - select a body first", hasObject))
        ctx.actions.shell = true;
    ImGui::SameLine();
    if (barButton(ctx, "pattern", Glyph::Pattern,
                  hasObject ? "Pattern: repeat in a row or around an axis  (P)"
                            : "Pattern - select a body first", hasObject))
        ctx.actions.pattern = true;
    ImGui::SameLine();
    if (barButton(ctx, "mirror", Glyph::Mirror,
                  hasObject ? "Mirror across a plane  (M)" : "Mirror - select a body first", hasObject))
        ctx.actions.mirror = true;
    ImGui::SameLine();
    if (barButton(ctx, "merge", Glyph::Merge,
                  hasObject ? "Merge faces: drop every division that does not define the shape"
                            : "Merge faces - select a body first", hasObject))
        ctx.actions.mergeFaces = true;
    endGroup(2);

    // Inspect
    beginGroup(3);
    if (barButton(ctx, "measure", Glyph::Measure, "Measure  (D)", true, false,
                  ctx.measuring ? u32(palette::kBrand) : 0))
        ctx.actions.toggleMeasure = true;
    ImGui::SameLine();
    // A standing toggle rather than a command, so it shows its state quietly:
    // the picture takes the brand colour while it is on.
    if (barButton(ctx, "print", Glyph::Alert,
                  ctx.view->showPrintIssues ? "Print problems are drawn on the model. Click to hide them."
                                            : "Print problems are hidden. Click to draw them on the model.",
                  true, false, ctx.view->showPrintIssues ? u32(palette::kBrand) : 0))
        ctx.view->showPrintIssues = !ctx.view->showPrintIssues;
    endGroup(3);

    // The captions, on the line under the pictures.
    for (Group& g : groups) {
        ImGui::SetCursorPos(ImVec2(0.0f, rowY + buttonH + 2.0f));
        groupCaption(ctx, g.name, g.popup, g.x0, g.x1, g.menu);
    }

    // ---- the right-hand end -----------------------------------------------
    {
        const float controlW = 44.0f, controlH = 30.0f;
        const float right = barMax.x;
        float xr = right - (ctx.frame.customFrame ? controlW * 3.0f + 10.0f : 12.0f);

        // Undo and redo, quiet, before the window's own buttons.
        xr -= 2.0f * (kIconPx + st.FramePadding.x * 2.0f) + 14.0f;
        ImGui::SetCursorScreenPos(ImVec2(xr, barMin.y + 6.0f + (buttonH + captionH - buttonH) * 0.5f));
        if (barButton(ctx, "undo", Glyph::Undo, "Undo  (Ctrl+Z)", ctx.canUndo)) ctx.actions.undo = true;
        ImGui::SameLine();
        if (barButton(ctx, "redo", Glyph::Redo, "Redo  (Ctrl+Shift+Z)", ctx.canRedo)) ctx.actions.redo = true;

        if (ctx.frame.customFrame) {
            struct Control { const char* id; Glyph g; const char* tip; };
            const Control controls[3] = {
                {"minimize", Glyph::Minimize, "Minimize"},
                {"maximize", ctx.frame.maximized ? Glyph::Restore : Glyph::Maximize,
                 ctx.frame.maximized ? "Restore" : "Maximize"},
                {"close", Glyph::Close, "Close"},
            };
            float cx = right - controlW * 3.0f;
            for (int i = 0; i < 3; ++i) {
                ImGui::SetCursorScreenPos(ImVec2(cx, barMin.y));
                ImGui::PushID(controls[i].id);
                const bool clicked = ImGui::InvisibleButton("##wc", ImVec2(controlW, controlH));
                const bool hovered = ImGui::IsItemHovered();
                keepFromDragging(ctx);
                const ImVec2 lo = ImGui::GetItemRectMin(), hi = ImGui::GetItemRectMax();
                if (hovered)
                    dl->AddRectFilled(lo, hi, i == 2 ? u32(palette::kBrand) : u32(palette::kHover));
                drawGlyph(dl, controls[i].g, ImVec2((lo.x + hi.x) * 0.5f, (lo.y + hi.y) * 0.5f), 14.0f,
                          hovered ? IM_COL32(255, 255, 255, 255) : u32(palette::kTextDim), 1.3f);
                if (clicked) {
                    if (i == 0) ctx.frame.wantMinimize = true;
                    if (i == 1) ctx.frame.wantToggleMaximize = true;
                    if (i == 2) ctx.frame.wantClose = true;
                }
                ImGui::PopID();
                cx += controlW;
            }
        }
    }

    // A double-click on the bar itself, away from its controls, maximises.
    if (ctx.frame.customFrame && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) &&
        !ctx.frame.popupOpen && ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows)) {
        const ImVec2 m = ImGui::GetIO().MousePos;
        const ImVec2 origin = ImGui::GetMainViewport()->Pos;
        bool onControl = false;
        for (const FrameState::Rect& r : ctx.frame.noDrag)
            if (m.x - origin.x >= r.x0 && m.x - origin.x <= r.x1 &&
                m.y - origin.y >= r.y0 && m.y - origin.y <= r.y1)
                onControl = true;
        if (!onControl) ctx.frame.wantToggleMaximize = true;
    }

    ImGui::EndChild();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(2);
    return height;
}

} // namespace tg
