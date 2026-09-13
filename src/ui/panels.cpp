#include "ui/panels.h"
#include "ui/command_panel.h"
#include "ui/icons.h"
#include "ui/theme.h"

#include "core/palette.h"

#include "imgui.h"
#include "imgui_internal.h"

#include <cstdio>

namespace tg {
namespace {

inline ImVec4 im(Rgb c, float a = 1.0f) {
    return ImVec4(static_cast<float>(c.r), static_cast<float>(c.g),
                  static_cast<float>(c.b), a);
}

const ImVec4 kAccent(palette::kBrand.r, palette::kBrand.g, palette::kBrand.b, 1.0f);
const ImVec4 kDim(palette::kTextDim.r, palette::kTextDim.g, palette::kTextDim.b, 1.0f);

// Which picture stands for a shape, and for a step of a history.
Icon iconFor(PrimitiveKind kind) {
    switch (kind) {
        case PrimitiveKind::Cylinder: return Icon::Cylinder;
        case PrimitiveKind::Sphere:   return Icon::Sphere;
        case PrimitiveKind::Cone:     return Icon::Cone;
        case PrimitiveKind::Torus:    return Icon::Torus;
        default:                      return Icon::Box;
    }
}

Icon iconFor(const Feature& f) {
    switch (f.kind) {
        case FeatureKind::Primitive: return iconFor(f.primitive.kind);
        case FeatureKind::Extrude:   return Icon::Extrude;
        case FeatureKind::Bevel:     return Icon::Fillet;
        case FeatureKind::Shell:      return Icon::Shell;
        case FeatureKind::FaceRotate: return Icon::Chamfer;
        case FeatureKind::FaceScale:  return Icon::Cone;
        case FeatureKind::Divide:     return Icon::Inset;
        case FeatureKind::Pattern:
            return f.patternMode == PatternMode::Mirror ? Icon::Difference
                                                        : Icon::Intersection;
        case FeatureKind::Inset:     return Icon::Inset;
        case FeatureKind::Boolean:
            return f.booleanOp == BooleanOp::Union        ? Icon::Union
                 : f.booleanOp == BooleanOp::Intersection ? Icon::Intersection
                                                          : Icon::Difference;
        default: return Icon::Box;
    }
}

void sectionLabel(const char* text) {
    ImGui::PushStyleColor(ImGuiCol_Text, kDim);
    ImGui::SeparatorText(text);
    ImGui::PopStyleColor();
}

// Labelled row with the field stretched to the panel width; used everywhere so
// the inspector columns line up regardless of label length.
// Geometry is double, so these bind ImGui's double scalar path rather than
// round-tripping through float and quietly losing digits in the fields the
// user types exact dimensions into.
// Three fields, tinted by axis.
//
// A row of three identical boxes says nothing about which is which, and the
// answer is needed on every glance: X, Y and Z are the same colours here as
// they are on the grid and the transform gizmo, so the mapping is learned once.
// The tint is on the field's background rather than on its text, which stays
// legible.
bool labeledDrag3(const char* label, Vec3& v, float speed, const char* fmt) {
    static const ImVec4 kAxisTint[3] = {
        ImVec4(0.46f, 0.20f, 0.17f, 0.55f),
        ImVec4(0.24f, 0.38f, 0.17f, 0.55f),
        ImVec4(0.18f, 0.29f, 0.47f, 0.55f),
    };
    ImGui::PushID(label);
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(label);
    ImGui::SameLine(78.0f);

    const ImGuiStyle& st = ImGui::GetStyle();
    const float avail = ImGui::GetContentRegionAvail().x;
    const float each = (avail - st.ItemInnerSpacing.x * 2.0f) / 3.0f;

    bool changed = false;
    for (int i = 0; i < 3; ++i) {
        if (i) ImGui::SameLine(0.0f, st.ItemInnerSpacing.x);
        ImGui::PushID(i);
        ImGui::PushStyleColor(ImGuiCol_FrameBg, kAxisTint[i]);
        ImGui::SetNextItemWidth(each);
        changed |= ImGui::DragScalar("##v", ImGuiDataType_Double, &(&v.x)[i], speed,
                                     nullptr, nullptr, fmt);
        ImGui::PopStyleColor();
        ImGui::PopID();
    }
    ImGui::PopID();
    return changed;
}

bool labeledDrag(const char* label, Real& v, float speed, Real lo, Real hi,
                 const char* fmt = "%.2f mm") {
    ImGui::PushID(label);
    ImGui::TextUnformatted(label);
    ImGui::SameLine(78.0f);
    ImGui::SetNextItemWidth(-1.0f);
    const bool changed = ImGui::DragScalarN("##v", ImGuiDataType_Double, &v, 1,
                                            speed, &lo, &hi, fmt);
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
        if (ImGui::MenuItem("New", "Ctrl+N"))          ctx.actions.newProject = true;
        if (ImGui::MenuItem("Open...", "Ctrl+O"))      ctx.actions.openProject = true;
        if (ImGui::MenuItem("Save", "Ctrl+S"))         ctx.actions.saveProject = true;
        if (ImGui::MenuItem("Save As..."))             ctx.actions.saveProjectAs = true;
        ImGui::Separator();
        if (ImGui::MenuItem("Import STEP...")) ctx.actions.importStep = true;
        ImGui::TextColored(kDim, "  brings in the surfaces, not a mesh of them");
        ImGui::Separator();
        if (ImGui::MenuItem("Export STEP...")) ctx.actions.exportStep = true;
        ImGui::TextColored(kDim, "  exact; what another CAD package wants");
        if (ImGui::MenuItem("Export STL...", "Ctrl+E")) ctx.actions.exportStl = true;
        ImGui::TextColored(kDim, "  triangles; what a slicer wants");
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

    if (ImGui::BeginMenu("Modify")) {
        const bool hasFaces = !ctx.scene->elementSelection().empty();
        const size_t selFaces = ctx.scene->selectedFaces(ctx.scene->contextObject()).size();
        const size_t selEdges = ctx.scene->selectedEdges(ctx.scene->contextObject()).size();
        const bool hasObject = ctx.scene->contextObject() != kNoObject;

        // Everything that acts on a face. These had a key and a toolbar button
        // and no menu entry, which meant the only way to find out they existed
        // was to read the source.
        if (ImGui::MenuItem("Move Face (Push / Pull)", "G", false, selFaces > 0))
            ctx.actions.pushPull = true;
        if (ImGui::MenuItem("Extrude Faces", "E", false, hasFaces))
            ctx.actions.extrude = true;
        ImGui::TextColored(kDim, "  Shift+E cuts inward");
        if (ImGui::MenuItem("Rotate Face", "R", false, selFaces > 0))
            ctx.actions.rotateFace = true;
        if (ImGui::MenuItem("Scale Face", "S", false, selFaces > 0))
            ctx.actions.scaleFace = true;
        ImGui::SetNextItemWidth(140.0f);
        ImGui::DragScalarN("Inset", ImGuiDataType_Double, &ctx.view->insetAmount, 1,
                           0.05f, nullptr, nullptr, "%.2f mm");
        if (ImGui::MenuItem("Inset Face", nullptr, false, selFaces > 0))
            ctx.actions.inset = true;
        ImGui::TextColored(kDim, "  a smaller face inside the one selected");
        ImGui::Separator();
        if (ImGui::MenuItem("Divide Across an Edge", "K", false, selEdges > 0))
            ctx.actions.divide = true;
        if (ImGui::MenuItem("Merge Faces", nullptr, false, hasObject))
            ctx.actions.mergeFaces = true;
        ImGui::Separator();
        if (ImGui::MenuItem("Pattern...", "P", false, hasObject))
            ctx.actions.pattern = true;
        if (ImGui::MenuItem("Mirror...", "M", false, hasObject))
            ctx.actions.mirror = true;
        ImGui::Separator();
        ImGui::SetNextItemWidth(140.0f);
        ImGui::DragScalarN("Width", ImGuiDataType_Double, &ctx.view->bevelWidth, 1,
                           0.05f, nullptr, nullptr, "%.2f mm");
        const size_t edgeCount = ctx.scene->selectedEdges(ctx.scene->contextObject()).size();
        const size_t faceCount = ctx.scene->selectedFaces(ctx.scene->contextObject()).size();
        const bool canFillet = (edgeCount > 0 || faceCount > 0);
        if (ImGui::MenuItem("Fillet Selected Edges / Faces", "F", false, canFillet))
            ctx.actions.fillet = true;
        if (edgeCount > 0)
            ImGui::TextColored(kDim, "  %zu edge%s selected", edgeCount,
                               edgeCount == 1 ? "" : "s");
        else if (faceCount > 0)
            ImGui::TextColored(kDim, "  %zu face%s selected (boundary edges)", faceCount,
                               faceCount == 1 ? "" : "s");
        else
            ImGui::TextColored(kDim, "  click an edge or face in viewport first");

        if (ImGui::MenuItem("Round All Edges", "Ctrl+B", false,
                            ctx.scene->contextObject() != kNoObject))
            ctx.actions.bevel = true;
        ImGui::TextColored(kDim, "  every edge of the body at once");

        ImGui::Separator();
        ImGui::SetNextItemWidth(140.0f);
        ImGui::DragScalarN("Wall", ImGuiDataType_Double, &ctx.view->shellThickness, 1,
                           0.05f, nullptr, nullptr, "%.2f mm");
        if (ImGui::MenuItem("Shell (Hollow Out)", nullptr, false,
                            ctx.scene->contextObject() != kNoObject))
            ctx.actions.shell = true;
        if (faceCount > 0)
            ImGui::TextColored(kDim, "  %zu selected face%s left open", faceCount,
                               faceCount == 1 ? "" : "s");
        else
            ImGui::TextColored(kDim, "  no faces selected: a sealed cavity");
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Combine")) {
        const size_t n = ctx.scene->selection().size();
        const bool pair = n == 2;

        if (ImGui::MenuItem("Union", "Ctrl+Shift+U", false, pair)) {
            ctx.actions.booleanRequested = true;
            ctx.actions.booleanOp = BooleanOp::Union;
        }
        if (ImGui::MenuItem("Difference", "Ctrl+Shift+D", false, pair)) {
            ctx.actions.booleanRequested = true;
            ctx.actions.booleanOp = BooleanOp::Difference;
        }
        if (ImGui::MenuItem("Intersect", "Ctrl+Shift+I", false, pair)) {
            ctx.actions.booleanRequested = true;
            ctx.actions.booleanOp = BooleanOp::Intersection;
        }
        if (!pair)
            ImGui::TextColored(kDim, "  Select two objects (%zu selected)", n);
        else
            ImGui::TextColored(kDim, "  First selected is kept, second is the tool");

        ImGui::Separator();
        if (ImGui::MenuItem("Split Body", nullptr, false,
                            ctx.scene->contextObject() != kNoObject))
            ctx.actions.split = true;
        ImGui::TextColored(kDim, "  Splits by face plane, tool plane, or shells");
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Measure")) {
        ImGui::MenuItem("Measure", "D", ctx.measuring);
        ImGui::Separator();
        ImGui::TextColored(kDim, "Click one entity for its own size,");
        ImGui::TextColored(kDim, "two for the distance between them.");
        ImGui::TextColored(kDim, "Distances are true minimums.");
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Transform")) {
        const bool has = !ctx.scene->selection().empty();
        ImGui::MenuItem("Move", "G", false, has);
        ImGui::MenuItem("Rotate", "R", false, has);
        ImGui::TextColored(kDim, "  with a face selected these move the face");
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
        bool ortho = ctx.camera->orthographic;
        if (ImGui::MenuItem("Orthographic", "Numpad 5", &ortho))
            ctx.camera->setOrthographic(ortho);
        ImGui::Separator();
        ImGui::MenuItem("Invert Orbit X", nullptr, &ctx.camera->invertOrbitX);
        ImGui::MenuItem("Invert Orbit Y", nullptr, &ctx.camera->invertOrbitY);
        ImGui::Separator();
        ImGui::MenuItem("Grid",           nullptr, &ctx.view->showGrid);
        ImGui::MenuItem("Wireframe",      "Z",     &ctx.view->showWireframe);
        ImGui::MenuItem("Selection Box",  nullptr, &ctx.view->showSelectionBox);
        ImGui::MenuItem("Print Problems", nullptr, &ctx.view->showPrintIssues);
        ImGui::TextColored(kDim, "  red: thinner than the nozzle can lay");
        ImGui::TextColored(kDim, "  amber: leans too far to hold itself up");
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
float drawToolbar(UiContext& ctx) {
    const Scene& scene = *ctx.scene;
    const ObjectId ctxObj = scene.contextObject();
    const bool hasObject = ctxObj != kNoObject;
    const bool pair = scene.selection().size() == 2;
    const size_t edges = scene.selectedEdges(ctxObj).size();
    const size_t faces = scene.selectedFaces(ctxObj).size();

    // Sized so the baked icons are read rather than guessed at: below about
    // twenty pixels a filleted cube and a chamfered one are the same picture.
    const float icon = 24.0f;
    const ImGuiStyle& st = ImGui::GetStyle();
    const float height = icon + st.FramePadding.y * 4.0f + st.ItemSpacing.y;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 5.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4.0f, 4.0f));
    ImGui::BeginChild("##toolbar", ImVec2(0.0f, height), ImGuiChildFlags_None,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    // One button for all five shapes.
    //
    // Which shape you start from is a choice made once at the beginning of a
    // part; the operations beside it are used over and over. Giving them equal
    // room on the bar made the row read as ten things of equal weight, and put
    // the five that matter further from the hand.
    struct Shape { Icon icon; PrimitiveKind kind; const char* name; };
    static const Shape kShapes[] = {
        {Icon::Box,      PrimitiveKind::Box,      "Box"},
        {Icon::Cylinder, PrimitiveKind::Cylinder, "Cylinder"},
        {Icon::Sphere,   PrimitiveKind::Sphere,   "Sphere"},
        {Icon::Cone,     PrimitiveKind::Cone,     "Cone"},
        {Icon::Torus,    PrimitiveKind::Torus,    "Torus"},
    };
    // The last one used, so the button keeps its face and the common case is
    // the same picture every time.
    static int lastShape = 0;

    if (iconButton(kShapes[lastShape].icon, "create", icon, "Create object  (Shift+A)"))
        ImGui::OpenPopup("##createobject");

    // A corner mark, so it reads as a button that opens rather than one that
    // acts. Drawn over the button just laid out.
    {
        const ImVec2 lo = ImGui::GetItemRectMin(), hi = ImGui::GetItemRectMax();
        const float d = 4.5f;
        ImGui::GetWindowDrawList()->AddTriangleFilled(
            ImVec2(hi.x - 2.0f, hi.y - 2.0f), ImVec2(hi.x - 2.0f - d, hi.y - 2.0f),
            ImVec2(hi.x - 2.0f, hi.y - 2.0f - d),
            ImGui::GetColorU32(ImGuiCol_Text, 0.65f));
        (void)lo;
    }

    if (ImGui::BeginPopup("##createobject")) {
        ImGui::TextDisabled("Create object");
        ImGui::Separator();
        const float line = ImGui::GetTextLineHeight();
        for (int i = 0; i < static_cast<int>(sizeof kShapes / sizeof kShapes[0]); ++i) {
            ImGui::PushID(i);
            iconImage(kShapes[i].icon, line);
            ImGui::SameLine();
            if (ImGui::Selectable(kShapes[i].name)) {
                ctx.actions.addRequested = true;
                ctx.actions.addKind = kShapes[i].kind;
                lastShape = i;
            }
            ImGui::PopID();
        }
        ImGui::EndPopup();
    }
    ImGui::SameLine();

    auto gap = [&] {
        ImGui::SameLine(0.0f, 10.0f);
        const ImVec2 p = ImGui::GetCursorScreenPos();
        ImGui::GetWindowDrawList()->AddLine(
            ImVec2(p.x, p.y + 3.0f), ImVec2(p.x, p.y + icon + st.FramePadding.y * 2.0f - 3.0f),
            ImGui::GetColorU32(ImGuiCol_Separator));
        ImGui::SameLine(0.0f, 10.0f);
    };
    gap();

    // Named operands, not just the verb. A boolean is the one operation where
    // which body survives is decided by selection order, and a tooltip that
    // says "Difference" leaves the user to remember the rule; one that says
    // "Bracket minus Bore" does not.
    const char* firstName = "the first";
    const char* secondName = "the second";
    if (pair) {
        if (const SceneObject* a2 = scene.find(scene.selection()[0])) firstName = a2->name.c_str();
        if (const SceneObject* b2 = scene.find(scene.selection()[1])) secondName = b2->name.c_str();
    }
    auto combine = [&](Icon ic, BooleanOp op, const char* name, const char* joiner) {
        char tip[192];
        if (pair) std::snprintf(tip, sizeof tip, "%s:  %s %s %s", name, firstName, joiner, secondName);
        else      std::snprintf(tip, sizeof tip, "%s - select two objects (%zu selected)",
                                name, scene.selection().size());
        if (iconButton(ic, name, icon, tip, false, pair)) {
            ctx.actions.booleanRequested = true;
            ctx.actions.booleanOp = op;
        }
        ImGui::SameLine();
    };
    combine(Icon::Union,        BooleanOp::Union,        "Union",     "+");
    combine(Icon::Difference,   BooleanOp::Difference,   "Difference", "minus");
    combine(Icon::Intersection, BooleanOp::Intersection, "Intersect", "with");
    gap();

    // The modifiers each want something different selected, and saying which
    // in the tooltip is the difference between a greyed-out button that
    // teaches and one that just refuses.
    if (iconButton(Icon::Fillet, "Fillet", icon,
                   edges || faces ? "Fillet the selected edges (F)"
                                  : "Fillet - select an edge or a face first",
                   false, edges || faces))
        ctx.actions.fillet = true;
    ImGui::SameLine();

    if (iconButton(Icon::Chamfer, "Bevel", icon,
                   hasObject ? "Bevel every edge (Ctrl+B)"
                             : "Bevel - select an object first",
                   false, hasObject))
        ctx.actions.bevel = true;
    ImGui::SameLine();

    if (iconButton(Icon::Shell, "Shell", icon,
                   hasObject ? "Shell - selected faces are left open"
                             : "Shell - select an object first",
                   false, hasObject))
        ctx.actions.shell = true;
    ImGui::SameLine();

    if (iconButton(Icon::Extrude, "PushPull", icon,
                   faces ? "Move face: the body follows  (G)"
                         : "Move face - select a face first",
                   false, faces > 0))
        ctx.actions.pushPull = true;
    ImGui::SameLine();

    if (iconButton(Icon::Union, "Extrude", icon,
                   faces ? "Extrude: grow a boss off the face, outline kept  (E)"
                         : "Extrude - select a face first",
                   false, faces > 0))
        ctx.actions.extrude = true;
    ImGui::SameLine();

    if (iconButton(Icon::Chamfer, "RotateFace", icon,
                   faces ? "Rotate face: tip it about an edge  (R)"
                         : "Rotate face - select one first",
                   false, faces > 0))
        ctx.actions.rotateFace = true;
    ImGui::SameLine();

    // A cone, because a tapered solid is what scaling a face makes, and
    // because the divide beside it already has the inset.
    if (iconButton(Icon::Cone, "ScaleFace", icon,
                   faces ? "Scale face: grow or shrink it in its own plane  (S)"
                         : "Scale face - select one first",
                   false, faces > 0))
        ctx.actions.scaleFace = true;
    ImGui::SameLine();

    if (iconButton(Icon::Inset, "Divide", icon,
                   edges ? "Divide the faces along an edge  (K)"
                         : "Divide - select an edge for the cut to run across",
                   false, edges > 0))
        ctx.actions.divide = true;
    ImGui::SameLine();

    // The opposite of divide, and the reason it is its own button: an
    // operation that tidied up after itself would take these with it.
    // A pattern of the last cut, or of the body. Which one it will be is
    // decided when the gesture starts, and said in its panel.
    if (iconButton(Icon::Intersection, "Pattern", icon,
                   hasObject ? "Pattern: repeat in a row or around an axis  (P)"
                             : "Pattern - select an object first",
                   false, hasObject))
        ctx.actions.pattern = true;
    ImGui::SameLine();

    if (iconButton(Icon::Difference, "Mirror", icon,
                   hasObject ? "Mirror: reflect across a plane  (M)"
                             : "Mirror - select an object first",
                   false, hasObject))
        ctx.actions.mirror = true;
    ImGui::SameLine();

    if (iconButton(Icon::Union, "MergeFaces", icon,
                   hasObject ? "Merge faces: drop every division that does not "
                               "define the shape"
                             : "Merge faces - select an object first",
                   false, hasObject))
        ctx.actions.mergeFaces = true;

    ImGui::EndChild();
    ImGui::PopStyleVar(2);
    return height;
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

        // Visibility toggle, then the shape, then the selectable name row.
        bool visible = obj->visible;
        if (ImGui::Checkbox("##vis", &visible)) obj->visible = visible;
        ImGui::SameLine();

        const float line = ImGui::GetTextLineHeight();
        ImGui::AlignTextToFramePadding();
        iconImage(iconFor(obj->spec.kind), line, visible ? 1.0f : 0.35f);
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

        // The kind, but only when it is not already the name. Every new object
        // is called after its shape, so the default row used to read "Box Box".
        const char* kind = primitiveName(obj->spec.kind);
        if (obj->name != kind) {
            ImGui::SameLine();
            ImGui::TextColored(kDim, "%s", kind);
        }
        ImGui::PopID();
    }

    ImGui::End();
}

// ---------------------------------------------------------------------------
void drawInspector(UiContext& ctx) {
    if (!ImGui::Begin("Inspector")) { ImGui::End(); return; }

    Scene& scene = *ctx.scene;
    SceneObject* obj = scene.find(scene.contextObject());
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

    // Volume lived under a PRINTABILITY heading that reported whether the body
    // was a watertight solid. On an exact body it always is -- the kernel
    // guarantees it and an edit that would break it is refused before it lands
    // -- so the heading spent its time saying "ready to print" about a part
    // with nothing checked against a printer. What a printer will actually
    // struggle with is drawn on the model instead; see app/printability.h.
    if (obj->healthVersion == obj->meshVersion)
        ImGui::TextColored(kDim, "Volume  %.2f cm3", obj->health.volume / 1000.0);
    ImGui::TextColored(kDim, "Verts   %d", obj->body.vertexCount());
    ImGui::TextColored(kDim, "Faces   %d", obj->body.faceCount());
    ImGui::TextColored(kDim, "Tris    %zu", obj->render.triangles.size() / 3);

    ImGui::End();
}

// ---------------------------------------------------------------------------
void drawHistory(UiContext& ctx) {
    if (!ImGui::Begin("History")) { ImGui::End(); return; }

    Scene& scene = *ctx.scene;
    SceneObject* obj = scene.find(scene.contextObject());
    if (!obj) {
        ImGui::TextColored(kDim, "Select an object to see its history");
        ImGui::End();
        return;
    }

    const std::vector<Feature> before = obj->features;
    bool changed = false;

    for (size_t i = 0; i < obj->features.size(); ++i) {
        Feature& f = obj->features[i];
        ImGui::PushID(static_cast<int>(i));

        bool enabled = f.enabled;
        if (ImGui::Checkbox("##on", &enabled)) { f.enabled = enabled; changed = true; }
        ImGui::SameLine();

        // The operation, as a picture. A chain of ten steps is read down this
        // column rather than along the summaries, which is the whole reason
        // for an icon here rather than a word.
        ImGui::AlignTextToFramePadding();
        iconImage(iconFor(f), ImGui::GetTextLineHeight(),
                  f.errored ? 0.4f : (f.enabled ? 1.0f : 0.35f));
        ImGui::SameLine();

        const bool open = ImGui::TreeNodeEx("##row", ImGuiTreeNodeFlags_SpanAvailWidth,
                                            "%s", f.summary().c_str());
        if (f.errored) {
            ImGui::SameLine();
            ImGui::TextColored(kAccent, "  failed: %s", f.error.c_str());
        } else if (!f.enabled) {
            ImGui::SameLine();
            ImGui::TextColored(kDim, "  off");
        }

        if (open) {
            switch (f.kind) {
            case FeatureKind::Primitive:
                ImGui::TextColored(kDim, "Edit dimensions in the Inspector");
                break;
            case FeatureKind::Merge:
                ImGui::TextColored(kDim, "Drops every division that does not");
                ImGui::TextColored(kDim, "define the shape.");
                break;
            case FeatureKind::FaceScale: {
                Real pct = (f.scale - 1.0) * 100.0;
                if (labeledDrag("Change", pct, 0.5f, -95.0f, 1000.0f, "%.1f %%")) {
                    f.scale = 1.0 + pct / 100.0;
                    changed = true;
                }
                ImGui::TextColored(kDim, "%.3g x its size", f.scale);
                break;
            }
            case FeatureKind::FaceRotate: {
                Real deg = degrees(f.angle);
                if (labeledDrag("Angle", deg, 0.2f, -89.0f, 89.0f, "%.1f deg")) {
                    f.angle = radians(deg);
                    changed = true;
                }
                ImGui::TextColored(kDim, "about %.2f, %.2f, %.2f",
                                   f.axisPoint.x, f.axisPoint.y, f.axisPoint.z);
                break;
            }
            case FeatureKind::Pattern: {
                // Everything the pattern panel offers, offered again -- a
                // pattern made three steps ago is still a count and a spacing,
                // and the panel that made it is long gone.
                const char* const layouts[] = {"Row", "Ring", "Mirror"};
                int layout = static_cast<int>(f.patternMode);
                ImGui::AlignTextToFramePadding();
                ImGui::TextUnformatted("Layout");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::Combo("##playout", &layout, layouts, 3)) {
                    f.patternMode = static_cast<PatternMode>(layout);
                    changed = true;
                }

                if (f.patternMode != PatternMode::Mirror) {
                    int n = f.patternCount;
                    ImGui::AlignTextToFramePadding();
                    ImGui::TextUnformatted("Copies");
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(-1.0f);
                    if (ImGui::DragInt("##pcount", &n, 0.1f, 2, 256)) {
                        f.patternCount = n < 2 ? 2 : (n > 256 ? 256 : n);
                        changed = true;
                    }
                    if (f.patternMode == PatternMode::Linear) {
                        changed |= labeledDrag("Spacing", f.distance, 0.1f, 0.05f,
                                               10000.0f, "%.2f mm");
                    } else {
                        Real deg = degrees(f.angle);
                        if (labeledDrag("Turn", deg, 0.25f, -360.0f, 360.0f, "%.1f deg")) {
                            f.angle = radians(deg);
                            changed = true;
                        }
                        ImGui::SameLine();
                        if (ImGui::SmallButton("Full turn")) {
                            f.angle = radians(360.0 / (f.patternCount < 2 ? 2 : f.patternCount));
                            changed = true;
                        }
                    }
                }

                // Which way it goes, as the axis it was built on rather than as
                // three numbers: a pattern is nearly always along one of them.
                const char* const axes[] = {"X", "Y", "Z"};
                int axis = std::fabs(f.axisDir.x) >= std::fabs(f.axisDir.y) &&
                                   std::fabs(f.axisDir.x) >= std::fabs(f.axisDir.z) ? 0
                         : std::fabs(f.axisDir.y) >= std::fabs(f.axisDir.z)         ? 1
                                                                                    : 2;
                ImGui::AlignTextToFramePadding();
                ImGui::TextUnformatted(f.patternMode == PatternMode::Mirror ? "Plane"
                                                                            : "Axis");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::Combo("##paxis", &axis, axes, 3)) {
                    f.axisDir = Vec3{axis == 0 ? 1.0 : 0.0, axis == 1 ? 1.0 : 0.0,
                                     axis == 2 ? 1.0 : 0.0};
                    changed = true;
                }

                ImGui::TextColored(kDim, "%s",
                                   f.bakedBody.empty() ? "Repeats the body."
                                                       : "Repeats the cut it replaced.");
                break;
            }
            case FeatureKind::Divide:
                ImGui::TextColored(kDim, "Cuts at %.2f, %.2f, %.2f",
                                   f.axisPoint.x, f.axisPoint.y, f.axisPoint.z);
                ImGui::TextColored(kDim, "The body stays whole; the faces divide.");
                break;
            case FeatureKind::Extrude: {
                changed |= labeledDrag("Distance", f.distance, 0.1f, -10000.0f, 10000.0f);
                const char* const opNames[] = {"Auto", "Join", "Cut", "Intersect"};
                int currentOp = static_cast<int>(f.extrudeOp);
                if (currentOp > 3) currentOp = 0;
                ImGui::AlignTextToFramePadding();
                ImGui::TextUnformatted("Operation");
                ImGui::SameLine();
                if (ImGui::Combo("##ExtrudeOp", &currentOp, opNames, 4)) {
                    f.extrudeOp = static_cast<ExtrudeOp>(currentOp);
                    changed = true;
                }
                ImGui::TextColored(kDim, "%s", f.faces.describe("face").c_str());
                break;
            }
            case FeatureKind::Inset:
                changed |= labeledDrag("Amount", f.amount, 0.05f, 0.01f, 10000.0f);
                break;
            case FeatureKind::Shell:
                changed |= labeledDrag("Wall", f.thickness, 0.05f, 0.01f, 10000.0f);
                ImGui::TextColored(kDim, "%s", f.faces.empty()
                                                   ? "sealed: no face left open"
                                                   : f.faces.describe("face").c_str());
                break;
            case FeatureKind::Bevel: {
                // A segment count is a mesh idea: an exact fillet is a surface
                // rather than an approximation of one, and the kernel ignores
                // the number outright. Shown only where it still does something.
                const bool isMesh = obj->body.isMesh();
                const bool wasChamfer = f.segments == 1;
                if (labeledDrag("Radius", f.width, 0.05f, 0.01f, 10000.0f)) {
                    // Editing the feature radius restates every edge's, which
                    // is what a user dragging one number expects. Per-edge
                    // radii come from picking edges one at a time.
                    f.radii.assign(f.edges.count(), f.width);
                    changed = true;
                }
                if (isMesh) {
                    changed |= labeledInt("Segments", f.segments, 1, 32);
                    if (wasChamfer != (f.segments == 1))
                        ImGui::TextColored(kDim, f.segments == 1 ? "flat cut" : "rounded");
                }
                ImGui::TextColored(kDim, "%s", f.edges.describe("edge").c_str());
                break;
            }
            case FeatureKind::Boolean: {
                // The tool body is baked into the feature, so its shape is not
                // editable here -- but which way it combines is, and redoing a
                // cut from scratch because it should have been a join is the
                // sort of thing a history exists to avoid.
                const char* const opNames[] = {"Union", "Difference", "Intersection"};
                int currentOp = static_cast<int>(f.booleanOp);
                if (currentOp < 0 || currentOp > 2) currentOp = 0;
                ImGui::AlignTextToFramePadding();
                ImGui::TextUnformatted("Operation");
                ImGui::SameLine();
                if (ImGui::Combo("##BooleanOp", &currentOp, opNames, 3)) {
                    f.booleanOp = static_cast<BooleanOp>(currentOp);
                    changed = true;
                }
                ImGui::TextColored(kDim, "Tool body: %d faces", f.bakedBody.faceCount());
                break;
            }
            case FeatureKind::BaseMesh:
                ImGui::TextColored(kDim, "Imported geometry, %d faces", f.bakedBody.faceCount());
                ImGui::TextColored(kDim, "Not parametric: edit it with the mesh tools");
                break;
            case FeatureKind::VertexEdit:
                ImGui::TextColored(kDim, "Free-form edit of %zu vertices",
                                   f.verts.size());
                break;
            }

            // The base primitive is what the chain starts from, so it cannot be
            // removed without leaving the rest with nothing to act on.
            if (f.kind != FeatureKind::Primitive) {
                if (ImGui::SmallButton("Delete")) {
                    obj->features.erase(obj->features.begin() + static_cast<long>(i));
                    changed = true;
                    ImGui::TreePop();
                    ImGui::PopID();
                    break;
                }
            }
            ImGui::TreePop();
        }
        ImGui::PopID();
    }

    if (changed) {
        ctx.actions.featuresEdited = obj->id;
        ctx.actions.featuresBefore = before;
    }

    ImGui::Spacing();
    ImGui::TextColored(kDim, "%zu feature%s", obj->features.size(),
                       obj->features.size() == 1 ? "" : "s");
    ImGui::End();
}

// ---------------------------------------------------------------------------
void drawMeasurePanel(UiContext& ctx) {
    if (!ctx.measuring) return;

    ImGuiViewport* vp = ImGui::GetMainViewport();
    // The same box every other operation runs in. A measurement is a reading
    // rather than a change, so it has no commit -- but a panel that looks
    // different for no reason is a panel the user has to learn twice.
    if (!ui::beginCommand("##measure", "Measure", Icon::Count,
                          vp->WorkPos.x + 16.0f, vp->WorkPos.y + 56.0f))
        return;

    const MeasureResult& m = ctx.measurement;
    auto value = [](const char* label, const char* fmt, double v, bool lead) {
        ui::commandRow(label);
        if (lead) ImGui::TextColored(kAccent, fmt, v);
        else      ImGui::Text(fmt, v);
    };

    if (!m.valid) {
        ui::commandHint("Click a vertex, edge or face. Click a second to measure between them.");
    } else {
        // A round thing leads with what it is. On an exact body the diameter is
        // a fact about the geometry rather than a measurement taken across it,
        // and it is the number someone is after.
        if (m.hasDiameter) {
            value("Diameter",  "%.4f mm", m.diameter, true);
            value("Radius",    "%.4f mm", m.diameter * 0.5, false);
            ui::commandRow("Centre");
            ImGui::Text("%.3f, %.3f, %.3f", m.centre.x, m.centre.y, m.centre.z);
        }
        if (m.hasLength)
            value(m.hasDiameter ? "Around" : "Length", "%.4f mm", m.length, true);
        if (m.hasArea) {
            value("Area",      "%.4f mm2", m.area, true);
            value("Perimeter", "%.4f mm",  m.perimeter, false);
        }
        if (ctx.measurePicks == 2 || m.hasLength) {
            if (ctx.measurePicks == 2) value("Distance", "%.4f mm", m.distance, true);
            value("dX", "%.4f mm", m.delta.x, false);
            value("dY", "%.4f mm", m.delta.y, false);
            value("dZ", "%.4f mm", m.delta.z, false);
        }
        if (m.hasAngle) value("Angle", "%.3f deg", m.angleDeg, true);

        ui::commandHint("Esc clears   D exits");
    }
    ui::endCommand();
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
        if (!ctx.notice.empty() && ctx.toolStatus.empty()) {
            ImGui::TextColored(kAccent, "%s", ctx.notice.c_str());
            ImGui::End();
            ImGui::PopStyleVar(2);
            return;
        }
        if (!ctx.toolStatus.empty()) {
            // While a modal transform runs, the readout is the only thing that
            // matters -- show it in the accent colour and drop the scene stats.
            ImGui::TextColored(kAccent, "%s", ctx.toolStatus.c_str());
            const char* keys = "X/Y/Z axis   Shift+axis plane   type a number   "
                               "Ctrl free (snap is on)   Enter confirm   Esc cancel";
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
        if (!scene.elementSelection().empty()) {
            const ElementRef& e = scene.elementSelection().front();
            ImGui::TextColored(kAccent, "%zu %s%s selected",
                               scene.elementSelection().size(),
                               elementKindName(e.kind),
                               scene.elementSelection().size() == 1 ? "" : "s");
        } else {
            ImGui::TextColored(kDim, "%zu selected", scene.selection().size());
        }
        ImGui::SameLine(0, 18);
        ImGui::TextColored(kDim, "%zu tris", ctx.stats.triangles);
        if (const SceneObject* ctxObj = scene.find(scene.contextObject())) {
            ImGui::SameLine(0, 18);
            if (ctxObj->healthVersion != ctxObj->meshVersion)
                ImGui::TextColored(kDim, "checking");
            else if (ctxObj->health.solid())
                ImGui::TextColored(im(palette::kValid), "solid");
            else
                ImGui::TextColored(kAccent, "not solid");
        }
        ImGui::SameLine(0, 18);
        ImGui::TextColored(kDim, "mm");

        const char* hint = "click edge/face   Ctrl+click object   G/R/S transform   E extrude   Ctrl snap";
        const float tw = ImGui::CalcTextSize(hint).x;
        ImGui::SameLine(ImGui::GetWindowWidth() - tw - 14.0f);
        ImGui::TextColored(kDim, "%s", hint);
    }
    ImGui::End();
    ImGui::PopStyleVar(2);
}

} // namespace tg
