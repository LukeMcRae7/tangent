#include "ui/panels.h"
#include "ui/theme.h"

#include "core/palette.h"

#include "imgui.h"
#include "imgui_internal.h"

#include <cstdio>

namespace tg {
namespace {

const ImVec4 kAccent(palette::kBrand.r, palette::kBrand.g, palette::kBrand.b, 1.0f);
const ImVec4 kDim(palette::kTextDim.r, palette::kTextDim.g, palette::kTextDim.b, 1.0f);

void sectionLabel(const char* text) {
    ImGui::PushStyleColor(ImGuiCol_Text, kDim);
    ImGui::SeparatorText(text);
    ImGui::PopStyleColor();
}

// Labelled row with the field stretched to the panel width; used everywhere so
// the inspector columns line up regardless of label length.
bool labeledDrag3(const char* label, Vec3& v, float speed, const char* fmt) {
    ImGui::PushID(label);
    ImGui::TextUnformatted(label);
    ImGui::SameLine(78.0f);
    ImGui::SetNextItemWidth(-1.0f);
    const bool changed = ImGui::DragFloat3("##v", &v.x, speed, 0.0f, 0.0f, fmt);
    ImGui::PopID();
    return changed;
}

bool labeledDrag(const char* label, float& v, float speed, float lo, float hi,
                 const char* fmt = "%.2f mm") {
    ImGui::PushID(label);
    ImGui::TextUnformatted(label);
    ImGui::SameLine(78.0f);
    ImGui::SetNextItemWidth(-1.0f);
    const bool changed = ImGui::DragFloat("##v", &v, speed, lo, hi, fmt);
    ImGui::PopID();
    return changed;
}

bool labeledInt(const char* label, int& v, int lo, int hi) {
    ImGui::PushID(label);
    ImGui::TextUnformatted(label);
    ImGui::SameLine(78.0f);
    ImGui::SetNextItemWidth(-1.0f);
    bool changed = ImGui::DragInt("##v", &v, 0.25f, lo, hi);
    if (changed) v = v < lo ? lo : (v > hi ? hi : v);
    ImGui::PopID();
    return changed;
}

// Parametric controls for whichever primitive the object was created from.
// Editing any of these regenerates the mesh, which is the first real piece of
// the parametric workflow.
bool drawPrimitiveParams(SceneObject& obj) {
    bool changed = false;
    switch (obj.spec.kind) {
        case PrimitiveKind::Box:
            changed |= labeledDrag("Width",  obj.spec.box.width,  0.1f, 0.01f, 10000.0f);
            changed |= labeledDrag("Depth",  obj.spec.box.depth,  0.1f, 0.01f, 10000.0f);
            changed |= labeledDrag("Height", obj.spec.box.height, 0.1f, 0.01f, 10000.0f);
            break;
        case PrimitiveKind::Cylinder:
            changed |= labeledDrag("Radius", obj.spec.cylinder.radius, 0.1f, 0.01f, 10000.0f);
            changed |= labeledDrag("Height", obj.spec.cylinder.height, 0.1f, 0.01f, 10000.0f);
            changed |= labeledInt ("Sides",  obj.spec.cylinder.segments, 3, 512);
            break;
        case PrimitiveKind::Sphere:
            changed |= labeledDrag("Radius",   obj.spec.sphere.radius, 0.1f, 0.01f, 10000.0f);
            changed |= labeledInt ("Segments", obj.spec.sphere.segments, 3, 512);
            changed |= labeledInt ("Rings",    obj.spec.sphere.rings, 2, 256);
            break;
        case PrimitiveKind::Cone:
            changed |= labeledDrag("Base R",  obj.spec.cone.bottomRadius, 0.1f, 0.01f, 10000.0f);
            changed |= labeledDrag("Top R",   obj.spec.cone.topRadius, 0.1f, 0.0f, 10000.0f);
            changed |= labeledDrag("Height",  obj.spec.cone.height, 0.1f, 0.01f, 10000.0f);
            changed |= labeledInt ("Sides",   obj.spec.cone.segments, 3, 512);
            break;
        case PrimitiveKind::Torus:
            changed |= labeledDrag("Major R", obj.spec.torus.majorRadius, 0.1f, 0.02f, 10000.0f);
            changed |= labeledDrag("Minor R", obj.spec.torus.minorRadius, 0.1f, 0.01f, 10000.0f);
            changed |= labeledInt ("Major",   obj.spec.torus.majorSegments, 3, 512);
            changed |= labeledInt ("Minor",   obj.spec.torus.minorSegments, 3, 256);
            // The generator rejects a minor radius that would self-intersect,
            // so clamp here instead of letting the rebuild silently no-op.
            if (obj.spec.torus.minorRadius >= obj.spec.torus.majorRadius)
                obj.spec.torus.minorRadius = obj.spec.torus.majorRadius * 0.98f;
            break;
        case PrimitiveKind::Plane:
            changed |= labeledDrag("Width", obj.spec.plane.width, 0.1f, 0.01f, 10000.0f);
            changed |= labeledDrag("Depth", obj.spec.plane.depth, 0.1f, 0.01f, 10000.0f);
            break;
        case PrimitiveKind::Custom:
            ImGui::TextColored(kDim, "Edited mesh - no parameters");
            break;
    }
    return changed;
}

} // namespace

// ---------------------------------------------------------------------------
void drawAddMenuItems(UiContext& ctx) {
    struct Entry { const char* label; PrimitiveKind kind; };
    static const Entry kEntries[] = {
        {"Box",      PrimitiveKind::Box},
        {"Cylinder", PrimitiveKind::Cylinder},
        {"Sphere",   PrimitiveKind::Sphere},
        {"Cone",     PrimitiveKind::Cone},
        {"Torus",    PrimitiveKind::Torus},
        {"Plane",    PrimitiveKind::Plane},
    };
    for (const Entry& e : kEntries) {
        if (ImGui::MenuItem(e.label)) {
            ctx.actions.addRequested = true;
            ctx.actions.addKind = e.kind;
        }
    }
}

void drawMenuBar(UiContext& ctx) {
    if (!ImGui::BeginMainMenuBar()) return;

    ImGui::PushStyleColor(ImGuiCol_Text, kAccent);
    // Lowercase wordmark: the identity is friendly and unfussy, not a
    // shouty enterprise logotype.
    ImGui::TextUnformatted("tangent");
    ImGui::PopStyleColor();
    ImGui::Spacing();

    if (ImGui::BeginMenu("File")) {
        ImGui::MenuItem("New", "Ctrl+N", false, false);
        ImGui::MenuItem("Open...", "Ctrl+O", false, false);
        ImGui::MenuItem("Save", "Ctrl+S", false, false);
        ImGui::MenuItem("Export STL...", "Ctrl+E", false, false);
        ImGui::Separator();
        if (ImGui::MenuItem("Quit", "Ctrl+Q")) ctx.actions.quit = true;
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Edit")) {
        if (ImGui::MenuItem("Undo", "Ctrl+Z", false, ctx.canUndo)) ctx.actions.undo = true;
        if (ImGui::MenuItem("Redo", "Ctrl+Shift+Z", false, ctx.canRedo)) ctx.actions.redo = true;
        ImGui::Separator();
        const bool has = !ctx.scene->selection().empty();
        if (ImGui::MenuItem("Duplicate", "Shift+D", false, has))
            ctx.actions.duplicateSelected = true;
        if (ImGui::MenuItem("Delete", "X", false, has))
            ctx.actions.deleteSelected = true;
        ImGui::Separator();
        if (ImGui::MenuItem("Select All", "A")) ctx.scene->selectAll();
        if (ImGui::MenuItem("Deselect All", "Alt+A")) ctx.scene->clearSelection();
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Add")) { drawAddMenuItems(ctx); ImGui::EndMenu(); }

    if (ImGui::BeginMenu("Transform")) {
        const bool has = !ctx.scene->selection().empty();
        ImGui::MenuItem("Move", "G", false, has);
        ImGui::MenuItem("Rotate", "R", false, has);
        ImGui::MenuItem("Scale", "S", false, has);
        ImGui::Separator();
        ImGui::TextColored(kDim, "During a transform:");
        ImGui::TextColored(kDim, "  X / Y / Z      constrain to an axis");
        ImGui::TextColored(kDim, "  Shift + axis   constrain to a plane");
        ImGui::TextColored(kDim, "  type a number  exact value");
        ImGui::TextColored(kDim, "  Ctrl           snap to increments");
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("View")) {
        if (ImGui::MenuItem("Frame Selected", "Numpad .")) ctx.actions.frameSelected = true;
        if (ImGui::MenuItem("Frame All", "Home"))          ctx.actions.frameAll = true;
        ImGui::Separator();
        if (ImGui::MenuItem("Front",  "Numpad 1"))       ctx.camera->setStandardView(StandardView::Front);
        if (ImGui::MenuItem("Back",   "Ctrl+Numpad 1"))  ctx.camera->setStandardView(StandardView::Back);
        if (ImGui::MenuItem("Right",  "Numpad 3"))       ctx.camera->setStandardView(StandardView::Right);
        if (ImGui::MenuItem("Left",   "Ctrl+Numpad 3"))  ctx.camera->setStandardView(StandardView::Left);
        if (ImGui::MenuItem("Top",    "Numpad 7"))       ctx.camera->setStandardView(StandardView::Top);
        if (ImGui::MenuItem("Bottom", "Ctrl+Numpad 7"))  ctx.camera->setStandardView(StandardView::Bottom);
        ImGui::Separator();
        ImGui::MenuItem("Orthographic", "Numpad 5", &ctx.camera->orthographic);
        ImGui::Separator();
        ImGui::MenuItem("Invert Orbit X", nullptr, &ctx.camera->invertOrbitX);
        ImGui::MenuItem("Invert Orbit Y", nullptr, &ctx.camera->invertOrbitY);
        ImGui::Separator();
        ImGui::MenuItem("Grid",           nullptr, &ctx.view->showGrid);
        ImGui::MenuItem("Wireframe",      "Z",     &ctx.view->showWireframe);
        ImGui::MenuItem("Selection Box",  nullptr, &ctx.view->showSelectionBox);
        ImGui::MenuItem("Backface Cull",  nullptr, &ctx.view->backfaceCulling);
        ImGui::EndMenu();
    }

    // Right-aligned frame timing.
    char timing[64];
    std::snprintf(timing, sizeof(timing), "%.1f fps   %.2f ms",
                  ctx.stats.frameMs > 0.0f ? 1000.0f / ctx.stats.frameMs : 0.0f,
                  ctx.stats.frameMs);
    const float tw = ImGui::CalcTextSize(timing).x;
    ImGui::SameLine(ImGui::GetWindowWidth() - tw - 16.0f);
    ImGui::TextColored(kDim, "%s", timing);

    ImGui::EndMainMenuBar();
}

// ---------------------------------------------------------------------------
void drawOutliner(UiContext& ctx) {
    if (!ImGui::Begin("Outliner")) { ImGui::End(); return; }

    Scene& scene = *ctx.scene;
    if (scene.objectCount() == 0) {
        ImGui::TextColored(kDim, "Empty scene");
        ImGui::Spacing();
        ImGui::TextColored(kDim, "Shift+A to add an object");
    }

    for (const auto& obj : scene.objects()) {
        ImGui::PushID(static_cast<int>(obj->id));

        // Visibility toggle, then the selectable name row.
        bool visible = obj->visible;
        if (ImGui::Checkbox("##vis", &visible)) obj->visible = visible;
        ImGui::SameLine();

        const bool selected = scene.isSelected(obj->id);
        if (ImGui::Selectable(obj->name.c_str(), selected,
                              ImGuiSelectableFlags_AllowDoubleClick)) {
            const bool additive = ImGui::GetIO().KeyShift || ImGui::GetIO().KeyCtrl;
            if (additive) scene.toggleSelect(obj->id);
            else          scene.select(obj->id);
            if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                ctx.actions.frameSelected = true;
        }

        ImGui::SameLine();
        ImGui::TextColored(kDim, "%s", primitiveName(obj->spec.kind));
        ImGui::PopID();
    }

    ImGui::End();
}

// ---------------------------------------------------------------------------
void drawInspector(UiContext& ctx) {
    if (!ImGui::Begin("Inspector")) { ImGui::End(); return; }

    Scene& scene = *ctx.scene;
    SceneObject* obj = scene.find(scene.activeObject());
    if (!obj) {
        ImGui::TextColored(kDim, "Nothing selected");
        ImGui::End();
        return;
    }

    // Snapshot before any widget runs, so a changed value can be paired with
    // what it replaced and pushed onto the undo stack.
    const Transform  transformBefore = obj->transform;
    const PrimitiveSpec specBefore = obj->spec;

    // Name.
    char nameBuf[128];
    std::snprintf(nameBuf, sizeof(nameBuf), "%s", obj->name.c_str());
    ImGui::SetNextItemWidth(-1.0f);
    if (ImGui::InputText("##name", nameBuf, sizeof(nameBuf))) obj->name = nameBuf;

    sectionLabel("TRANSFORM");
    labeledDrag3("Position", obj->transform.position, 0.1f, "%.2f");

    // Euler angles are a display convention only; the transform stores a
    // quaternion, so the conversion round-trips through it on every edit.
    Vec3 euler = toEuler(obj->transform.rotation);
    euler = {degrees(euler.x), degrees(euler.y), degrees(euler.z)};
    if (labeledDrag3("Rotation", euler, 0.5f, "%.1f")) {
        obj->transform.rotation = normalize(Quat::fromEuler(
            {radians(euler.x), radians(euler.y), radians(euler.z)}));
    }

    labeledDrag3("Scale", obj->transform.scale, 0.01f, "%.3f");
    if (ImGui::SmallButton("Reset Transform")) obj->transform = Transform{};

    sectionLabel("GEOMETRY");
    if (drawPrimitiveParams(*obj)) {
        ctx.actions.rebuildObject = obj->id;
        ctx.actions.specBefore = specBefore;
    }

    // Any transform field that moved becomes one undo entry per drag.
    const Transform& tNow = obj->transform;
    if (tNow.position != transformBefore.position ||
        tNow.scale != transformBefore.scale ||
        tNow.rotation.x != transformBefore.rotation.x ||
        tNow.rotation.y != transformBefore.rotation.y ||
        tNow.rotation.z != transformBefore.rotation.z ||
        tNow.rotation.w != transformBefore.rotation.w) {
        ctx.actions.transformEdited = obj->id;
        ctx.actions.transformBefore = transformBefore;
    }

    sectionLabel("STATISTICS");
    const AABB b = obj->localBounds;
    const Vec3 size = b.valid() ? b.size() : Vec3{};
    ImGui::TextColored(kDim, "Size    %.2f x %.2f x %.2f mm", size.x, size.y, size.z);
    ImGui::TextColored(kDim, "Verts   %d", obj->mesh.vertexCount());
    ImGui::TextColored(kDim, "Faces   %d", obj->mesh.faceCount());
    ImGui::TextColored(kDim, "Tris    %zu", obj->render.triangles.size() / 3);

    ImGui::End();
}

// ---------------------------------------------------------------------------
void drawStatusBar(UiContext& ctx) {
    ImGuiViewport* vp = ImGui::GetMainViewport();
    const float h = ImGui::GetFrameHeight();

    ImGui::SetNextWindowPos(ImVec2(vp->Pos.x, vp->Pos.y + vp->Size.y - h));
    ImGui::SetNextWindowSize(ImVec2(vp->Size.x, h));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10, 3));

    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoDocking |
        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNav;

    if (ImGui::Begin("##statusbar", nullptr, flags)) {
        if (!ctx.toolStatus.empty()) {
            // While a modal transform runs, the readout is the only thing that
            // matters -- show it in the accent colour and drop the scene stats.
            ImGui::TextColored(kAccent, "%s", ctx.toolStatus.c_str());
            const char* keys = "X/Y/Z axis   Shift+axis plane   type a number   "
                               "Ctrl snap   Enter confirm   Esc cancel";
            const float kw = ImGui::CalcTextSize(keys).x;
            ImGui::SameLine(ImGui::GetWindowWidth() - kw - 14.0f);
            ImGui::TextColored(kDim, "%s", keys);
            ImGui::End();
            ImGui::PopStyleVar(2);
            return;
        }

        const Scene& scene = *ctx.scene;
        ImGui::TextColored(kDim, "%zu object%s", scene.objectCount(),
                           scene.objectCount() == 1 ? "" : "s");
        ImGui::SameLine(0, 18);
        ImGui::TextColored(kDim, "%zu selected", scene.selection().size());
        ImGui::SameLine(0, 18);
        ImGui::TextColored(kDim, "%zu tris", ctx.stats.triangles);
        ImGui::SameLine(0, 18);
        ImGui::TextColored(kDim, "mm");

        const char* hint = "MMB orbit   Shift+MMB pan   Wheel zoom   G move   R rotate   S scale   Shift+A add";
        const float tw = ImGui::CalcTextSize(hint).x;
        ImGui::SameLine(ImGui::GetWindowWidth() - tw - 14.0f);
        ImGui::TextColored(kDim, "%s", hint);
    }
    ImGui::End();
    ImGui::PopStyleVar(2);
}

// ---------------------------------------------------------------------------
void drawViewportOverlay(UiContext& ctx, float x, float y, float w, float h) {
    (void)w;
    ImGui::SetNextWindowPos(ImVec2(x + 12.0f, y + 10.0f));
    ImGui::SetNextWindowBgAlpha(0.0f);
    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoDocking |
        ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoInputs;

    if (ImGui::Begin("##vpoverlay", nullptr, flags)) {
        const Camera& cam = *ctx.camera;

        // Name the view only when it is squarely on axis; otherwise "User".
        const float p = degrees(cam.pitch);
        const float yw = degrees(cam.yaw);
        auto near = [](float a, float b) { return std::fabs(a - b) < 0.5f; };
        const char* name = "User";
        if (near(p, 90.0f))       name = "Top";
        else if (near(p, -90.0f)) name = "Bottom";
        else if (near(p, 0.0f)) {
            const float ny = std::fmod(std::fmod(yw, 360.0f) + 360.0f, 360.0f);
            if      (near(ny, 0.0f))   name = "Front";
            else if (near(ny, 180.0f)) name = "Back";
            else if (near(ny, 90.0f))  name = "Right";
            else if (near(ny, 270.0f)) name = "Left";
        }
        ImGui::TextColored(ImVec4(0.78f, 0.80f, 0.83f, 0.9f), "%s %s",
                           name, cam.orthographic ? "Ortho" : "Persp");
    }
    ImGui::End();
    (void)h;
}

} // namespace tg
