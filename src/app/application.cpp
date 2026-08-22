#include "app/application.h"
#include "ui/theme.h"

#include "core/palette.h"

#include <SDL3/SDL.h>
#include <epoxy/gl.h>

#include "imgui.h"
#include "imgui_internal.h"
#include "backends/imgui_impl_sdl3.h"
#include "backends/imgui_impl_opengl3.h"

#include <cstdio>
#include <algorithm>
#include <cstdlib>
#include <vector>

namespace tg {
namespace {

constexpr int kGlMajor = 3;
constexpr int kGlMinor = 3;

// Resolve the shader directory relative to the executable so the app runs from
// any working directory.
std::string resolveShaderDir() {
    if (const char* env = SDL_getenv("TANGENT_SHADER_DIR")) return env;
    if (const char* base = SDL_GetBasePath()) return std::string(base) + "shaders";
    return "shaders";
}

} // namespace

// ---------------------------------------------------------------------------
bool Application::init() {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::fprintf(stderr, "[app] SDL_Init failed: %s\n", SDL_GetError());
        return false;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, kGlMajor);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, kGlMinor);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

    window_ = SDL_CreateWindow("Tangent", 1600, 950,
                               SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE |
                               SDL_WINDOW_HIGH_PIXEL_DENSITY);
    if (!window_) {
        std::fprintf(stderr, "[app] SDL_CreateWindow failed: %s\n", SDL_GetError());
        return false;
    }

    glCtx_ = SDL_GL_CreateContext(window_);
    if (!glCtx_) {
        std::fprintf(stderr, "[app] GL context creation failed: %s\n", SDL_GetError());
        return false;
    }
    SDL_GL_MakeCurrent(window_, static_cast<SDL_GLContext>(glCtx_));
    SDL_GL_SetSwapInterval(1);

    std::fprintf(stderr, "[app] GL %s | %s\n",
                 reinterpret_cast<const char*>(glGetString(GL_VERSION)),
                 reinterpret_cast<const char*>(glGetString(GL_RENDERER)));

    glEnable(GL_MULTISAMPLE);

    // ---- ImGui ------------------------------------------------------------
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigWindowsMoveFromTitleBarOnly = true;
    io.IniFilename = "tangent.ini";

    loadFonts(15.0f);
    applyDarkTheme();

    if (!ImGui_ImplSDL3_InitForOpenGL(window_, glCtx_)) {
        std::fprintf(stderr, "[app] ImGui SDL3 backend failed\n");
        return false;
    }
    if (!ImGui_ImplOpenGL3_Init("#version 330 core")) {
        std::fprintf(stderr, "[app] ImGui OpenGL3 backend failed\n");
        return false;
    }

    // ---- Renderer ---------------------------------------------------------
    shaderDir_ = resolveShaderDir();
    if (!renderer_.init(shaderDir_)) return false;

    ui_.scene  = &scene_;
    ui_.camera = &camera_;
    ui_.view   = &view_;

    // Start with a cube on the origin, so the viewport is never a blank void.
    if (!startEmpty_) {
        const ObjectId startup = scene_.addPrimitive(PrimitiveKind::Box);
        placeOnBuildPlate(startup);
        scene_.select(startup);
    }
    if (pickFace_ >= 0 && !scene_.objects().empty()) {
        const ObjectId id = scene_.objects().front()->id;
        scene_.clearSelection();
        scene_.selectElement({id, ElementKind::Face, static_cast<Index>(pickFace_)});
        if (autoExtrude_) {
            // Drive the real interactive path: extrude, type a distance, commit.
            extrudeSelection();
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%g", static_cast<double>(autoExtrudeMm_));
            for (const char* p = buf; *p; ++p) tool_.typeCharacter(*p);
            tool_.update(scene_, camera_, Vec2{0.0f, 0.0f}, false);
            commitTransform();
        }
    }

    if (measureDemo_ && !scene_.objects().empty()) {
        const ObjectId id = scene_.objects().front()->id;
        const Mesh& m = scene_.find(id)->mesh;
        Index top = 0, bottom = 0;
        for (Index f = 0; f < m.faceCount(); ++f) {
            if (dot(m.faceNormal(f), Vec3{0, 0, 1}) > 0.99) top = f;
            if (dot(m.faceNormal(f), Vec3{0, 0, -1}) > 0.99) bottom = f;
        }
        measure_.begin();
        measure_.pick({id, ElementKind::Face, top});
        measure_.pick({id, ElementKind::Face, bottom});
    }

    if (!fixedCamera_) {
        camera_.frame(scene_.bounds());
        camera_.snapToGoal();
    }

    return true;
}

void Application::shutdown() {
    renderer_.shutdown();
    if (ImGui::GetCurrentContext()) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplSDL3_Shutdown();
        ImGui::DestroyContext();
    }
    if (glCtx_) SDL_GL_DestroyContext(static_cast<SDL_GLContext>(glCtx_));
    if (window_) SDL_DestroyWindow(window_);
    SDL_Quit();
}

// ---------------------------------------------------------------------------
void Application::handleEvent(const SDL_Event& e) {
    ImGui_ImplSDL3_ProcessEvent(&e);

    switch (e.type) {
        case SDL_EVENT_QUIT:
            running_ = false;
            break;
        case SDL_EVENT_WINDOW_CLOSE_REQUESTED:
            if (e.window.windowID == SDL_GetWindowID(window_)) running_ = false;
            break;
        default:
            break;
    }
}

Vec2 Application::mouseInViewport() const {
    const ImVec2 m = ImGui::GetIO().MousePos;
    return {m.x - viewRect_.x, m.y - viewRect_.y};
}

void Application::beginTransform(TransformMode mode) {
    measure_.end();

    if (const SceneObject* o = scene_.find(scene_.contextObject()))
        preEditSolid_ = o->healthVersion == o->meshVersion && o->health.solid();
    else
        preEditSolid_ = false;

    // begin() declines when nothing is selected; there is simply no transform
    // to start, so this is not an error worth reporting.
    tool_.begin(mode, scene_, camera_, mouseInViewport());
}

// Blender-style navigation. ImGui gets first refusal on every input, so
// dragging a slider never also orbits the camera.
void Application::handleViewportMouse() {
    ImGuiIO& io = ImGui::GetIO();

    const bool overViewport =
        io.MousePos.x >= viewRect_.x && io.MousePos.x < viewRect_.x + viewRect_.w &&
        io.MousePos.y >= viewRect_.y && io.MousePos.y < viewRect_.y + viewRect_.h;

    // Middle-drag navigation: capture continues even if the cursor leaves the
    // viewport, which is what makes a long orbit feel unbounded.
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Middle) && overViewport && !io.WantCaptureMouse)
        navigating_ = true;
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Middle))
        navigating_ = false;

    if (navigating_) {
        const ImVec2 d = io.MouseDelta;
        if (d.x != 0.0f || d.y != 0.0f) {
            if (io.KeyShift) camera_.pan(d.x, d.y);
            else             camera_.orbit(d.x, d.y);
        }
    }

    // Navigation stays live during a transform (orbiting mid-move is useful),
    // but clicks mean commit/cancel rather than select.
    if (tool_.active()) {
        // Snapping is the default, not the modifier. A CAD part is designed in
        // round numbers; free positioning is the exception, so Ctrl releases
        // the snap rather than engaging it.
        tool_.update(scene_, camera_, mouseInViewport(), !io.KeyCtrl);
        if (!io.WantCaptureMouse) {
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))       commitTransform();
            else if (ImGui::IsMouseClicked(ImGuiMouseButton_Right))  abortTransform();
        }
        return;
    }

    if (io.WantCaptureMouse || !overViewport) return;

    if (io.MouseWheel != 0.0f) camera_.dolly(io.MouseWheel);

    // A click that ends a drag is ignored, so an orbit or a future box-select
    // gesture does not also fire a pick.
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
        !ImGui::IsMouseDragging(ImGuiMouseButton_Left, 4.0f)) {
        handleViewportClick(io.KeyShift, io.KeyCtrl);
    }
}

// Plain click picks the specific edge, face or vertex under the cursor, the way
// a CAD tool does. Ctrl+click takes the whole object instead, matching what
// clicking its row in the outliner does. Shift extends either.
void Application::handleViewportClick(bool shift, bool ctrl) {
    const Vec2 cursor = mouseInViewport();
    const Ray ray = camera_.rayThroughPixel(cursor.x, cursor.y);

    // While measuring, a click chooses what to measure rather than what to
    // edit, so the current selection is left alone.
    if (measure_.active()) {
        const ElementHit hit = scene_.pickElement(ray, camera_.viewProjection(),
                                                  camera_.viewportW, camera_.viewportH,
                                                  cursor);
        measure_.pick(hit.ref);
        return;
    }

    if (ctrl) {
        const RayHit hit = scene_.raycast(ray);
        scene_.clearElementSelection();
        if (hit.hit()) {
            if (shift) scene_.toggleSelect(hit.object);
            else       scene_.select(hit.object);
        } else if (!shift) {
            scene_.clearSelection();
        }
        return;
    }

    const ElementHit pick = scene_.pickElement(ray, camera_.viewProjection(),
                                               camera_.viewportW, camera_.viewportH,
                                               cursor);
    if (pick.hit()) {
        // Picking a sub-element takes the object selection out of play, so a
        // following G/R/S cannot silently move the whole body instead.
        scene_.clearSelection();
        if (shift) scene_.toggleElement(pick.ref);
        else       scene_.selectElement(pick.ref);
    } else if (!shift) {
        scene_.clearElementSelection();
        scene_.clearSelection();
    }
}

void Application::drawMeasureLabel() {
    if (!measure_.active() || !measureResult_.valid) return;

    // Anchored at the midpoint of the measured span, in the foreground list so
    // it is never hidden by geometry.
    const Vec3 mid = (measureResult_.from + measureResult_.to) * 0.5;
    Vec2 px;
    if (!camera_.projectToPixel(mid, px)) return;

    const ImVec2 at(static_cast<float>(px.x) + viewRect_.x + 14.0f,
                    static_cast<float>(px.y) + viewRect_.y - 10.0f);
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    const ImVec2 size = ImGui::CalcTextSize(measureResult_.summary.c_str());
    // Colours come from the palette, not from literals, so the label follows
    // a theme change like everything else.
    auto u8 = [](Real v) { return static_cast<int>(clampf(v, 0.0, 1.0) * 255.0 + 0.5); };
    const Rgb& bg = palette::kMenuBar;
    const Rgb& fg = palette::kBrand;
    dl->AddRectFilled(ImVec2(at.x - 6, at.y - 4),
                      ImVec2(at.x + size.x + 6, at.y + size.y + 4),
                      IM_COL32(u8(bg.r), u8(bg.g), u8(bg.b), 224), 4.0f);
    dl->AddText(at, IM_COL32(u8(fg.r), u8(fg.g), u8(fg.b), 255),
                measureResult_.summary.c_str());
}

void Application::drawSelectionHighlights() {
    const Vec4 faceTint = toVec4(palette::kBrand, 0.30f);
    const Vec4 edgeCol  = toVec4(palette::kBrand, 1.0f);

    for (const ElementRef& e : scene_.elementSelection()) {
        const SceneObject* o = scene_.find(e.object);
        if (!o) continue;
        const Mat4 model = o->modelMatrix();

        // Nudge toward the eye by a fixed number of pixels' worth of world
        // distance, so the highlight sits on the surface at any zoom instead of
        // z-fighting with it.
        auto lift = [&](Vec3 p) {
            const Vec3 toEye = camera_.eye() - p;
            const float len = length(toEye);
            if (len < 1e-6f) return p;
            return p + toEye * (camera_.pixelWorldSize(p) * 2.0f / len);
        };

        switch (e.kind) {
        case ElementKind::Face: {
            if (e.index >= o->mesh.faceCount()) break;
            const RenderMesh& rm = o->render;
            for (size_t i = 0; i < rm.triangleFace.size(); ++i) {
                if (rm.triangleFace[i] != e.index) continue;
                renderer_.addTriangle(
                    lift(transformPoint(model, rm.positions[rm.triangles[i * 3 + 0]])),
                    lift(transformPoint(model, rm.positions[rm.triangles[i * 3 + 1]])),
                    lift(transformPoint(model, rm.positions[rm.triangles[i * 3 + 2]])),
                    faceTint);
            }
            // Outline it too, so a face on a busy mesh still reads clearly.
            const Index start = o->mesh.faces[e.index].halfedge;
            Index h = start;
            do {
                renderer_.addLine(
                    lift(transformPoint(model, o->mesh.verts[o->mesh.fromVertex(h)].position)),
                    lift(transformPoint(model, o->mesh.verts[o->mesh.halfedges[h].vertex].position)),
                    edgeCol);
                h = o->mesh.halfedges[h].next;
            } while (h != start);
            break;
        }
        case ElementKind::Edge: {
            if (e.index >= o->mesh.halfedgeCount()) break;
            const Vec3 a = o->mesh.verts[o->mesh.fromVertex(e.index)].position;
            const Vec3 b = o->mesh.verts[o->mesh.halfedges[e.index].vertex].position;
            renderer_.addLine(lift(transformPoint(model, a)),
                              lift(transformPoint(model, b)), edgeCol);
            break;
        }
        case ElementKind::Vertex: {
            if (e.index >= o->mesh.vertexCount()) break;
            const Vec3 p = lift(transformPoint(model, o->mesh.verts[e.index].position));
            const float s = camera_.pixelWorldSize(p) * 4.0f;
            renderer_.addLine(p - Vec3{s, 0, 0}, p + Vec3{s, 0, 0}, edgeCol);
            renderer_.addLine(p - Vec3{0, s, 0}, p + Vec3{0, s, 0}, edgeCol);
            renderer_.addLine(p - Vec3{0, 0, s}, p + Vec3{0, 0, s}, edgeCol);
            break;
        }
        case ElementKind::None:
            break;
        }
    }
}

void Application::handleTransformKeys() {
    ImGuiIO& io = ImGui::GetIO();

    if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) { abortTransform(); return; }
    if (ImGui::IsKeyPressed(ImGuiKey_Enter, false) ||
        ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false)) {
        commitTransform();
        return;
    }

    // Shift picks the plane perpendicular to the axis instead of the axis.
    const bool plane = io.KeyShift;
    if (ImGui::IsKeyPressed(ImGuiKey_X, false))
        tool_.setConstraint(plane ? Constraint::PlaneX : Constraint::AxisX);
    if (ImGui::IsKeyPressed(ImGuiKey_Y, false))
        tool_.setConstraint(plane ? Constraint::PlaneY : Constraint::AxisY);
    if (ImGui::IsKeyPressed(ImGuiKey_Z, false))
        tool_.setConstraint(plane ? Constraint::PlaneZ : Constraint::AxisZ);

    // Exact numeric entry.
    for (int d = 0; d <= 9; ++d) {
        if (ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_0 + d), false) ||
            ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_Keypad0 + d), false))
            tool_.typeCharacter(static_cast<char>('0' + d));
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Period, false) ||
        ImGui::IsKeyPressed(ImGuiKey_KeypadDecimal, false)) tool_.typeCharacter('.');
    if (ImGui::IsKeyPressed(ImGuiKey_Minus, false) ||
        ImGui::IsKeyPressed(ImGuiKey_KeypadSubtract, false)) tool_.typeCharacter('-');
    if (ImGui::IsKeyPressed(ImGuiKey_Backspace, false)) tool_.backspace();
}

void Application::handleShortcuts() {
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureKeyboard) return;   // a text field has focus

    // A running transform owns the keyboard: X must constrain to an axis, not
    // delete the thing being moved.
    if (tool_.active()) { handleTransformKeys(); return; }

    const bool ctrl  = io.KeyCtrl;
    const bool shift = io.KeyShift;
    const bool alt   = io.KeyAlt;

    if (ctrl && ImGui::IsKeyPressed(ImGuiKey_Q, false)) ui_.actions.quit = true;

    if (ctrl && ImGui::IsKeyPressed(ImGuiKey_Z, false)) {
        if (shift) ui_.actions.redo = true;
        else       ui_.actions.undo = true;
    }

    // Modal transforms, Blender's G / R / S.
    if (!ctrl && !alt) {
        if (ImGui::IsKeyPressed(ImGuiKey_G, false)) beginTransform(TransformMode::Translate);
        if (ImGui::IsKeyPressed(ImGuiKey_R, false)) beginTransform(TransformMode::Rotate);
        if (ImGui::IsKeyPressed(ImGuiKey_S, false) && !shift)
            beginTransform(TransformMode::Scale);
    }

    // Add menu at the cursor.
    if (shift && ImGui::IsKeyPressed(ImGuiKey_A, false)) openAddMenu_ = true;
    else if (alt && ImGui::IsKeyPressed(ImGuiKey_A, false)) scene_.clearSelection();
    else if (!shift && !ctrl && !alt && ImGui::IsKeyPressed(ImGuiKey_A, false)) scene_.selectAll();

    if (ImGui::IsKeyPressed(ImGuiKey_X, false) || ImGui::IsKeyPressed(ImGuiKey_Delete, false))
        ui_.actions.deleteSelected = true;
    if (shift && ImGui::IsKeyPressed(ImGuiKey_D, false))
        ui_.actions.duplicateSelected = true;
    if (ImGui::IsKeyPressed(ImGuiKey_Z, false) && !ctrl && !shift)
        view_.showWireframe = !view_.showWireframe;

    // Measure. D for distance, and reachable without moving the left hand.
    if (!ctrl && !alt && !shift && ImGui::IsKeyPressed(ImGuiKey_D, false)) {
        if (measure_.active()) measure_.end();
        else                   measure_.begin();
    }
    if (measure_.active() && ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
        // First Escape clears the picks, a second leaves the tool.
        if (measure_.picks().empty()) measure_.end();
        else                          measure_.clearPicks();
    }

    // Mesh edits act on the selected faces. Shift+E cuts inward.
    if (!ctrl && !alt && ImGui::IsKeyPressed(ImGuiKey_E, false)) ui_.actions.extrude = true;
    if (ctrl && ImGui::IsKeyPressed(ImGuiKey_B, false))          ui_.actions.bevel = true;

    // Numpad view shortcuts; Ctrl gives the opposite side, as in Blender.
    if (ImGui::IsKeyPressed(ImGuiKey_Keypad1, false))
        camera_.setStandardView(ctrl ? StandardView::Back : StandardView::Front);
    if (ImGui::IsKeyPressed(ImGuiKey_Keypad3, false))
        camera_.setStandardView(ctrl ? StandardView::Left : StandardView::Right);
    if (ImGui::IsKeyPressed(ImGuiKey_Keypad7, false))
        camera_.setStandardView(ctrl ? StandardView::Bottom : StandardView::Top);
    if (ImGui::IsKeyPressed(ImGuiKey_Keypad5, false))
        camera_.orthographic = !camera_.orthographic;
    if (ImGui::IsKeyPressed(ImGuiKey_KeypadDecimal, false))
        ui_.actions.frameSelected = true;
    if (ImGui::IsKeyPressed(ImGuiKey_Home, false))
        ui_.actions.frameAll = true;

    // Orbit from the keyboard, matching Blender's numpad 4/6/8/2. Camera::orbit
    // speaks in pixels, so convert a 15-degree step through its pixel rate.
    const float px = radians(15.0f) / 0.010f;
    // Routed through orbit() so they honour the invert settings too.
    if (ImGui::IsKeyPressed(ImGuiKey_Keypad4, true)) camera_.orbit( px, 0.0f);
    if (ImGui::IsKeyPressed(ImGuiKey_Keypad6, true)) camera_.orbit(-px, 0.0f);
    if (ImGui::IsKeyPressed(ImGuiKey_Keypad8, true)) camera_.orbit(0.0f,  px);
    if (ImGui::IsKeyPressed(ImGuiKey_Keypad2, true)) camera_.orbit(0.0f, -px);
}

// ---------------------------------------------------------------------------
void Application::placeOnBuildPlate(ObjectId id) {
    SceneObject* o = scene_.find(id);
    if (!o || !o->localBounds.valid()) return;
    o->transform.position.z = -o->localBounds.min.z * o->transform.scale.z;
}

void Application::addPrimitiveAtCursor(PrimitiveKind kind) {
    // Blender drops new objects at the 3D cursor; with no cursor yet, the world
    // origin is the predictable choice.
    PrimitiveSpec spec;
    spec.kind = kind;
    const ObjectId id = scene_.addPrimitive(kind, spec, Vec3{0.0f, 0.0f, 0.0f});
    if (id != kNoObject) {
        placeOnBuildPlate(id);
        scene_.select(id);
    }
}

// Runs a mesh operation on the active object and records it for undo. The
// operation is given a scratch copy, so a rejected edit (a bevel too wide for
// the geometry, an inset that would invert a face) leaves the object alone.
// Extrudes the selected faces and immediately hands the user a drag along the
// new faces' own normal, rather than committing a fixed distance. The extrusion
// starts at a hair above zero so the side walls are valid geometry from the
// first frame; the drag supplies the real height.
void Application::extrudeSelection() {
    if (tool_.active()) return;

    if (const SceneObject* o = scene_.find(scene_.contextObject()))
        preEditSolid_ = o->healthVersion == o->meshVersion && o->health.solid();

    const ObjectId id = scene_.elementSelection().empty()
                        ? kNoObject : scene_.elementSelection().front().object;
    SceneObject* obj = scene_.find(id);
    if (!obj) return;

    const std::vector<Index> faces = scene_.selectedFaces(id);
    if (faces.empty()) return;

    // Direction to push, taken before the mesh changes underneath us.
    Vec3 normal{};
    for (Index f : faces) normal += obj->mesh.faceNormal(f) * obj->mesh.faceArea(f);
    if (lengthSq(normal) < 1e-12f) return;
    normal = normalize(normal);
    const Vec3 worldNormal = normalize(transformVector(normalMatrix(obj->modelMatrix()), normal));

    Mesh before = obj->mesh;
    std::vector<Feature> chainBefore = obj->features;

    constexpr float kSeed = 0.01f;   // mm
    Mesh next = obj->mesh;
    std::vector<Index> newFaces;
    if (!extrudeFaces(next, faces, kSeed, &newFaces)) return;

    // The drag edits the mesh directly for immediate feedback; on commit the
    // whole gesture is replaced by one Extrude feature, so the history stays
    // the authority rather than accumulating baked-in geometry.
    obj->mesh = std::move(next);
    obj->refreshDerived();

    // The moved faces stay selected, so the drag acts on them and so the user
    // can extrude again straight away.
    scene_.clearElementSelection();
    for (Index f : newFaces) scene_.selectElement({id, ElementKind::Face, f}, true);

    if (!tool_.begin(TransformMode::Translate, scene_, camera_, mouseInViewport())) {
        obj->mesh = std::move(before);
        obj->refreshDerived();
        return;
    }
    tool_.setCustomAxis(worldNormal, "N");

    pendingMeshObject_ = id;
    pendingMeshBefore_ = std::move(before);
    pendingChainBefore_ = std::move(chainBefore);
    pendingExtrudeFaces_ = faces;
    pendingNewFaces_ = newFaces;
    pendingLocalNormal_ = normal;
    pendingLabel_ = "Extrude";
}

bool Application::editKeepsSolid(ObjectId id) {
    SceneObject* obj = scene_.find(id);
    if (!obj) return true;

    const MeshHealth after = checkHealth(obj->mesh);
    obj->health = after;
    obj->healthVersion = obj->meshVersion;

    // Only refuse an edit that broke something that was previously sound. If
    // the model was already open or self-intersecting, blocking further edits
    // would leave the user unable to fix it.
    if (preEditSolid_ && !after.solid()) return false;
    return true;
}

void Application::commitTransform() {
    std::unique_ptr<Command> cmd = tool_.confirm(scene_);

    if (pendingMeshObject_ != kNoObject) {
        const ObjectId id = pendingMeshObject_;
        pendingMeshObject_ = kNoObject;

        SceneObject* obj = scene_.find(id);
        if (obj && !pendingNewFaces_.empty() && !pendingExtrudeFaces_.empty()) {
            // Recover the distance from the geometry itself rather than from
            // the drag: the drag happened in world space, and the feature needs
            // an object-space distance along the face normal, which differ as
            // soon as the object carries a scale.
            const Vec3 fromC = pendingMeshBefore_.faceCentroid(pendingExtrudeFaces_[0]);
            const Vec3 toC   = obj->mesh.faceCentroid(pendingNewFaces_[0]);
            const float distance = dot(toC - fromC, pendingLocalNormal_);

            Feature f;
            f.kind = FeatureKind::Extrude;
            f.faces = pendingExtrudeFaces_;
            f.distance = distance;

            // Roll back to the pre-extrude chain, then let the feature produce
            // the result, so the mesh is always something the history can
            // reproduce.
            obj->features = pendingChainBefore_;
            if (scene_.addFeature(id, f) && editKeepsSolid(id)) {
                undo_.push(std::make_unique<FeatureCommand>(
                    id, pendingChainBefore_, obj->features, pendingLabel_));
            } else {
                obj->features = pendingChainBefore_;
                scene_.reevaluate(id);
                setNotice("Extrude refused: it would make the model self-intersect");
            }
        }
        pendingMeshBefore_ = Mesh{};
        pendingChainBefore_.clear();
        pendingExtrudeFaces_.clear();
        pendingNewFaces_.clear();
        return;
    }

    // A free-form drag of vertices is recorded as a feature too. It is not
    // parametric, but it has to live in the chain: otherwise re-evaluating an
    // earlier feature would silently discard it.
    if (auto* vc = dynamic_cast<VertexCommand*>(cmd.get())) {
        if (!editKeepsSolid(vc->object())) {
            // Put it back. This is the CAD contract: an edit either produces
            // valid geometry or it does not happen.
            vc->undo(scene_);
            setNotice("Edit refused: it would make the model self-intersect");
            return;
        }
        SceneObject* obj = scene_.find(vc->object());
        if (obj) {
            Feature f;
            f.kind = FeatureKind::VertexEdit;
            f.verts = vc->vertices();
            for (size_t i = 0; i < f.verts.size(); ++i)
                f.offsets.push_back(vc->afterPositions()[i] - vc->beforePositions()[i]);

            std::vector<Feature> chainBefore = obj->features;
            if (scene_.addFeature(vc->object(), f)) {
                undo_.push(std::make_unique<FeatureCommand>(
                    vc->object(), std::move(chainBefore), obj->features, "Edit Vertices"));
                return;
            }
        }
    }

    undo_.push(std::move(cmd));
}

void Application::abortTransform() {
    tool_.cancel(scene_);

    if (pendingMeshObject_ != kNoObject) {
        if (SceneObject* obj = scene_.find(pendingMeshObject_)) {
            obj->features = pendingChainBefore_;
            obj->mesh = std::move(pendingMeshBefore_);
            obj->refreshDerived();
            scene_.clearElementSelection();
        }
        pendingMeshObject_ = kNoObject;
        pendingMeshBefore_ = Mesh{};
        pendingChainBefore_.clear();
        pendingExtrudeFaces_.clear();
        pendingNewFaces_.clear();
    }
}

void Application::bevelActiveObject() {
    const ObjectId target = scene_.contextObject();
    SceneObject* obj = scene_.find(target);
    if (!obj) return;

    // Clamp to what the geometry can actually take, so the slider cannot ask
    // for a bevel that inverts a face.
    const float limit = maxBevelWidth(obj->mesh);
    const Real width = std::min(view_.bevelWidth, limit * Real(0.95));
    if (width <= 1e-4f) return;

    Feature f;
    f.kind = FeatureKind::Bevel;
    f.width = width;
    f.segments = view_.bevelSegments;

    std::vector<Feature> chainBefore = obj->features;
    if (!scene_.addFeature(target, f)) return;
    undo_.push(std::make_unique<FeatureCommand>(target, std::move(chainBefore),
                                                obj->features, "Bevel"));
}

void Application::applyActions() {
    UiActions& a = ui_.actions;

    if (a.quit) running_ = false;

    if (a.undo) undo_.undo(scene_);
    if (a.redo) undo_.redo(scene_);

    if (a.addRequested) {
        const size_t before = scene_.objectCount();
        addPrimitiveAtCursor(a.addKind);
        if (scene_.objectCount() > before)
            undo_.push(ExistenceCommand::forCreate(scene_, {scene_.activeObject()}));
    }

    if (a.duplicateSelected) {
        const std::vector<ObjectId> sel = scene_.selection();
        std::vector<ObjectId> copies;
        scene_.clearSelection();
        for (ObjectId id : sel) {
            const ObjectId copy = scene_.duplicateObject(id);
            if (copy != kNoObject) { copies.push_back(copy); scene_.select(copy, true); }
        }
        if (!copies.empty()) undo_.push(ExistenceCommand::forCreate(scene_, copies));
    }

    if (a.deleteSelected) {
        const std::vector<ObjectId> sel = scene_.selection();
        if (!sel.empty()) {
            for (ObjectId id : sel) renderer_.forget(id);
            // forDelete does the removal itself, so the objects survive inside
            // the command and can be restored intact.
            undo_.push(ExistenceCommand::forDelete(scene_, sel));
            scene_.clearSelection();
        }
    }

    if (a.featuresEdited != kNoObject) {
        SceneObject* o = scene_.find(a.featuresEdited);
        if (o) {
            if (scene_.reevaluate(a.featuresEdited)) {
                undo_.push(std::make_unique<FeatureCommand>(
                    a.featuresEdited, std::move(a.featuresBefore), o->features,
                    "Edit History"), /*merge=*/true);
            } else {
                // The chain no longer produces anything; put it back.
                o->features = a.featuresBefore;
                scene_.reevaluate(a.featuresEdited);
            }
        }
    }

    if (a.extrude) extrudeSelection();
    if (a.bevel)   bevelActiveObject();

    if (a.rebuildObject != kNoObject) {
        SceneObject* o = scene_.find(a.rebuildObject);
        if (o) {
            std::vector<Feature> chainBefore = o->features;
            for (Feature& f : chainBefore)
                if (f.kind == FeatureKind::Primitive) { f.primitive = a.specBefore; break; }

            // Editing the base parameters re-runs every later operation, which
            // is the whole point of the history.
            scene_.rebuild(a.rebuildObject);
            // Merged so that dragging a parameter slider is one undo step
            // rather than one per frame.
            undo_.push(std::make_unique<FeatureCommand>(a.rebuildObject,
                                                        std::move(chainBefore),
                                                        o->features, "Change Parameters"),
                       /*merge=*/true);
        }
    }

    if (a.transformEdited != kNoObject) {
        SceneObject* o = scene_.find(a.transformEdited);
        if (o) {
            std::vector<TransformCommand::Entry> e{
                {a.transformEdited, a.transformBefore, o->transform}};
            undo_.push(std::make_unique<TransformCommand>(std::move(e), "Transform"),
                       /*merge=*/true);
        }
    }

    // A released mouse button ends any inspector drag, so the next one starts
    // its own undo entry.
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) undo_.breakMergeChain();

    if (a.frameSelected) {
        const AABB b = scene_.selection().empty() ? scene_.bounds() : scene_.selectionBounds();
        camera_.frame(b);
    }
    if (a.frameAll)   camera_.frame(scene_.bounds());
    if (a.resetView)  camera_.frame(scene_.bounds());

    a = UiActions{};
}

// ---------------------------------------------------------------------------
void Application::buildUi() {
    ImGuiViewport* vp = ImGui::GetMainViewport();
    const float statusH = ImGui::GetFrameHeight();

    // Dockspace host, inset to leave room for the status bar.
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x, vp->WorkSize.y - statusH));
    ImGui::SetNextWindowViewport(vp->ID);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));

    const ImGuiWindowFlags hostFlags =
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
        ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoBackground;

    ImGui::Begin("##DockHost", nullptr, hostFlags);
    ImGui::PopStyleVar(3);

    const ImGuiID dockId = ImGui::GetID("TangentDockspace");
    // PassthruCentralNode leaves the central node unpainted, so the GL scene
    // drawn underneath shows through instead of needing a render target.
    ImGui::DockSpace(dockId, ImVec2(0, 0), ImGuiDockNodeFlags_PassthruCentralNode);

    const ImGuiDockNode* node = ImGui::DockBuilderGetNode(dockId);
    if (firstLayout_ && node && !node->IsSplitNode()) {
        ImGui::DockBuilderRemoveNode(dockId);
        // DockSpace lives in ImGui's private flag enum, so the two have to be
        // combined as plain integers.
        ImGui::DockBuilderAddNode(dockId,
            static_cast<ImGuiDockNodeFlags>(ImGuiDockNodeFlags_PassthruCentralNode) |
            static_cast<ImGuiDockNodeFlags>(ImGuiDockNodeFlags_DockSpace));
        ImGui::DockBuilderSetNodeSize(dockId, ImVec2(vp->WorkSize.x, vp->WorkSize.y - statusH));

        ImGuiID centre = dockId;
        ImGuiID right  = ImGui::DockBuilderSplitNode(centre, ImGuiDir_Right, 0.21f, nullptr, &centre);
        ImGuiID lower  = ImGui::DockBuilderSplitNode(right, ImGuiDir_Down, 0.62f, nullptr, &right);

        ImGui::DockBuilderDockWindow("Outliner", right);
        ImGui::DockBuilderDockWindow("Inspector", lower);
        ImGui::DockBuilderDockWindow("History", lower);
        ImGui::DockBuilderFinish(dockId);
    }
    firstLayout_ = false;
    ImGui::End();

    // The central node is the 3D viewport's rectangle.
    if (const ImGuiDockNode* central = ImGui::DockBuilderGetCentralNode(dockId)) {
        viewRect_ = {central->Pos.x - vp->Pos.x, central->Pos.y - vp->Pos.y,
                     central->Size.x, central->Size.y};
    } else {
        viewRect_ = {vp->WorkPos.x - vp->Pos.x, vp->WorkPos.y - vp->Pos.y,
                     vp->WorkSize.x, vp->WorkSize.y - statusH};
    }

    drawMenuBar(ui_);
    drawOutliner(ui_);
    drawInspector(ui_);
    drawHistory(ui_);
    drawStatusBar(ui_);
    drawViewportOverlay(ui_, viewRect_.x, viewRect_.y, viewRect_.w, viewRect_.h);
    drawMeasurePanel(ui_);
    drawMeasureLabel();

    if (openAddMenu_) {
        ImGui::OpenPopup("##addmenu");
        openAddMenu_ = false;
    }
    if (ImGui::BeginPopup("##addmenu")) {
        ImGui::TextColored(ImVec4(palette::kTextDim.r, palette::kTextDim.g,
                          palette::kTextDim.b, 1.0f), "ADD");
        ImGui::Separator();
        drawAddMenuItems(ui_);
        ImGui::EndPopup();
    }
}

void Application::drawFrame() {
    int logicalW = 0, logicalH = 0, pixelW = 0, pixelH = 0;
    SDL_GetWindowSize(window_, &logicalW, &logicalH);
    SDL_GetWindowSizeInPixels(window_, &pixelW, &pixelH);
    pixelScaleX_ = logicalW > 0 ? static_cast<float>(pixelW) / logicalW : 1.0f;
    pixelScaleY_ = logicalH > 0 ? static_cast<float>(pixelH) / logicalH : 1.0f;

    // The camera works in logical points, matching ImGui's mouse coordinates;
    // only the GL viewport is expressed in physical pixels.
    camera_.viewportW = static_cast<int>(viewRect_.w);
    camera_.viewportH = static_cast<int>(viewRect_.h);

    PixelRect rect;
    rect.x = static_cast<int>(viewRect_.x * pixelScaleX_);
    rect.w = static_cast<int>(viewRect_.w * pixelScaleX_);
    rect.h = static_cast<int>(viewRect_.h * pixelScaleY_);
    // GL's origin is bottom-left, ImGui's is top-left.
    rect.y = pixelH - static_cast<int>(viewRect_.y * pixelScaleY_) - rect.h;

    lastViewportPx_ = rect;
    renderer_.render(scene_, camera_, view_, rect, pixelW, pixelH);
}

void Application::readViewport(std::vector<unsigned char>& out) const {
    const PixelRect& r = lastViewportPx_;
    if (!r.valid()) { out.clear(); return; }
    out.resize(static_cast<size_t>(r.w) * r.h * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_BACK);
    glReadPixels(r.x, r.y, r.w, r.h, GL_RGB, GL_UNSIGNED_BYTE, out.data());
}

// Luminance at a world point, read back from the rendered viewport.
// Returns -1 if the point is behind the camera or outside the viewport.
static float sampleAt(const std::vector<unsigned char>& px, const PixelRect& r,
                      const Camera& cam, float sx, float sy, Vec3 world) {
    Vec2 logical;
    if (!cam.projectToPixel(world, logical)) return -1.0f;

    const int cx = static_cast<int>(logical.x * sx);
    // projectToPixel measures Y downward from the top; the readback rows run
    // upward from the bottom of the rectangle.
    const int cy = r.h - 1 - static_cast<int>(logical.y * sy);

    // Brightest pixel in a 3x3 window. The projected position is rounded to a
    // whole pixel and a grid line is only about a pixel wide, so sampling a
    // single pixel misses the line whenever the rounding goes the wrong way --
    // which depends on sub-pixel phase and therefore looks like a failure at
    // scattered, arbitrary angles.
    float best = -1.0f;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            const int fx = cx + dx, fy = cy + dy;
            if (fx < 0 || fx >= r.w || fy < 0 || fy >= r.h) continue;
            const size_t i = (static_cast<size_t>(fy) * r.w + fx) * 3;
            if (i + 2 >= px.size()) continue;
            const float lum = 0.2126f * px[i] + 0.7152f * px[i + 1] + 0.0722f * px[i + 2];
            best = std::max(best, lum);
        }
    }
    return best;
}

void Application::sampleGridAlignment(float& onLine, float& offLine) const {
    onLine = offLine = 0.0f;
    const PixelRect& r = lastViewportPx_;
    if (!r.valid()) return;

    std::vector<unsigned char> px;
    readViewport(px);

    // Points on the x = k*10 and y = k*10 major lines, sampled away from any
    // perpendicular line so only the line under test contributes. Controls sit
    // half a millimetre off, the furthest possible from every line at the
    // finest level the grid draws.
    double on = 0.0, off = 0.0;
    int onN = 0, offN = 0;
    for (int k = -4; k <= 4; ++k) {
        const float g = static_cast<float>(k) * 10.0f;
        const Vec3 probes[4] = {{g, 3.7f, 0.0f}, {3.7f, g, 0.0f},
                                {g, -6.3f, 0.0f}, {-6.3f, g, 0.0f}};
        const Vec3 ctrls[4]  = {{g + 0.5f, 3.5f, 0.0f}, {3.5f, g + 0.5f, 0.0f},
                                {g + 0.5f, -6.5f, 0.0f}, {-6.5f, g + 0.5f, 0.0f}};
        for (int i = 0; i < 4; ++i) {
            const float a = sampleAt(px, r, camera_, pixelScaleX_, pixelScaleY_, probes[i]);
            if (a >= 0.0f) { on += a; ++onN; }
            const float b = sampleAt(px, r, camera_, pixelScaleX_, pixelScaleY_, ctrls[i]);
            if (b >= 0.0f) { off += b; ++offN; }
        }
    }
    if (onN)  onLine  = static_cast<float>(on / onN);
    if (offN) offLine = static_cast<float>(off / offN);
}

double Application::meanViewportLuminance() const {
    const PixelRect& r = lastViewportPx_;
    if (!r.valid()) return 0.0;

    std::vector<unsigned char> px(static_cast<size_t>(r.w) * r.h * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_BACK);
    glReadPixels(r.x, r.y, r.w, r.h, GL_RGB, GL_UNSIGNED_BYTE, px.data());

    // Rec. 709 luma, averaged. Any change in how much ink the grid lays down
    // moves this number, which is what makes it a usable stability signal.
    double sum = 0.0;
    for (size_t i = 0; i < px.size(); i += 3)
        sum += 0.2126 * px[i] + 0.7152 * px[i + 1] + 0.0722 * px[i + 2];
    return sum / (static_cast<double>(r.w) * r.h);
}

void Application::captureFramebuffer(int width, int height) const {
    if (width <= 0 || height <= 0) return;

    // Must run before SwapWindow: after the swap the back buffer's contents
    // are undefined.
    std::vector<unsigned char> pixels(static_cast<size_t>(width) * height * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_BACK);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

    FILE* f = std::fopen(screenshotPath_.c_str(), "wb");
    if (!f) {
        std::fprintf(stderr, "[app] cannot write %s\n", screenshotPath_.c_str());
        return;
    }
    std::fprintf(f, "P6\n%d %d\n255\n", width, height);
    // GL's first row is the bottom of the image; PPM's is the top.
    for (int y = height - 1; y >= 0; --y)
        std::fwrite(pixels.data() + static_cast<size_t>(y) * width * 3, 1,
                    static_cast<size_t>(width) * 3, f);
    std::fclose(f);
    std::fprintf(stderr, "[app] wrote %s (%dx%d)\n", screenshotPath_.c_str(), width, height);
}

// ---------------------------------------------------------------------------
int Application::run() {
    uint64_t previous = SDL_GetPerformanceCounter();
    const double freq = static_cast<double>(SDL_GetPerformanceFrequency());
    int frame = 0;

    while (running_) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) handleEvent(e);

        const uint64_t now = SDL_GetPerformanceCounter();
        const float dt = static_cast<float>((now - previous) / freq);
        previous = now;
        frameMs_ = frameMs_ * 0.9f + dt * 1000.0f * 0.1f;   // smoothed readout
        lastDt_ = dt;

        if (probeActive_) {
            const float u = static_cast<float>(probeIndex_) /
                            static_cast<float>(probeSteps_ - 1);
            camera_.yaw = radians(probeYaw0_ + (probeYaw1_ - probeYaw0_) * u);
            camera_.snapToGoal();
        }

        renderer_.reloadShadersIfChanged();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        // Stats are gathered before the panels that display them.
        ui_.stats = UiStats{};
        ui_.stats.frameMs = frameMs_;
        // Refresh the printability report only once the geometry has settled.
        // Self-intersection testing costs hundreds of milliseconds on a heavy
        // mesh; running it per frame during a drag would make editing
        // unusable, and the answer mid-drag is not interesting anyway.
        if (SceneObject* ctxObj = scene_.find(scene_.contextObject())) {
            if (ctxObj->healthVersion != ctxObj->meshVersion) {
                const bool interacting = tool_.active() ||
                                         ImGui::IsMouseDown(ImGuiMouseButton_Left);
                healthIdle_ = interacting ? 0.0f : healthIdle_ + lastDt_;
                if (healthIdle_ > 0.25f) {
                    ctxObj->health = checkHealth(ctxObj->mesh);
                    ctxObj->healthVersion = ctxObj->meshVersion;
                    healthIdle_ = 0.0f;
                }
            }
        }

        noticeAge_ += lastDt_;
        if (noticeAge_ > 4.0f) notice_.clear();
        ui_.notice = notice_;

        ui_.measuring = measure_.active();
        ui_.measurement = measureResult_;
        ui_.measurePicks = measure_.picks().size();

        ui_.toolStatus = tool_.statusText();
        if (measure_.active() && ui_.toolStatus.empty()) {
            const size_t n = measure_.picks().size();
            ui_.toolStatus = n == 0 ? "Measure: click a vertex, edge or face"
                           : n == 1 ? "Measure: " + measureResult_.summary +
                                      "   -   click another to measure between them"
                                    : "Measure: " + measureResult_.summary;
        }
        ui_.canUndo = undo_.canUndo();
        ui_.canRedo = undo_.canRedo();
        for (const auto& o : scene_.objects()) {
            ui_.stats.triangles += o->render.triangles.size() / 3;
            ui_.stats.vertices  += static_cast<size_t>(o->mesh.vertexCount());
        }

        buildUi();
        handleViewportMouse();
        handleShortcuts();
        applyActions();

        // Queued before the frame is drawn; the renderer flushes overlay lines
        // at the end of its pass.
        drawSelectionHighlights();
        measureResult_ = measure_.active() ? measure_.compute(scene_) : MeasureResult{};
        measure_.drawOverlay(renderer_, camera_, measureResult_);
        tool_.drawOverlay(renderer_, camera_);

        camera_.update(dt);

        ImGui::Render();
        drawFrame();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        if (probeActive_) {
            // Let the first couple of frames settle before sampling.
            // Render each angle twice and sample the second. The first render
            // after a camera change can carry a single-frame transient from
            // the swap chain, which shows up as an isolated pair of large
            // differences that standalone renders of the very same angles do
            // not reproduce. Sampling steady state measures the shading rather
            // than the buffer.
            if (frame >= 2 && ++probeSettle_ >= 2) {
                probeSettle_ = 0;
                std::vector<unsigned char> cur;
                readViewport(cur);

                // Mean absolute per-pixel change since the previous step. With
                // a sub-pixel rotation between steps, a stable image barely
                // changes; lines that breathe in width or levels that pop
                // produce a much larger difference.
                double diff = 0.0;
                if (probePrev_.size() == cur.size() && !cur.empty()) {
                    long long acc = 0;
                    for (size_t i = 0; i < cur.size(); ++i)
                        acc += std::abs(static_cast<int>(cur[i]) -
                                        static_cast<int>(probePrev_[i]));
                    diff = static_cast<double>(acc) / static_cast<double>(cur.size());
                }
                probePrev_ = std::move(cur);

                const float yawNow = probeYaw0_ + (probeYaw1_ - probeYaw0_) *
                                     (static_cast<float>(probeIndex_) /
                                      static_cast<float>(probeSteps_ - 1));
                if (alignProbe_) {
                    float on = 0.0f, off = 0.0f;
                    sampleGridAlignment(on, off);
                    std::printf("%.5f %.4f %.4f\n", yawNow, on, off);
                } else {
                    std::printf("%.5f %.6f %.6f\n", yawNow, meanViewportLuminance(), diff);
                }
                if (++probeIndex_ >= probeSteps_) { std::fflush(stdout); running_ = false; }
            }
        }

        if (screenshotFrame_ >= 0 && frame >= screenshotFrame_ && !screenshotPath_.empty()) {
            int pw = 0, ph = 0;
            SDL_GetWindowSizeInPixels(window_, &pw, &ph);
            captureFramebuffer(pw, ph);
            screenshotPath_.clear();
        }

        SDL_GL_SwapWindow(window_);

        ++frame;
        if (!probeActive_ && smokeFrames_ > 0 && frame >= smokeFrames_) {
            std::fprintf(stderr, "[app] smoke test: %d frames rendered cleanly\n", frame);
            running_ = false;
        }
    }
    return 0;
}

} // namespace tg
