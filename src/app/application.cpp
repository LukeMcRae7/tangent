#include "app/application.h"
#include "geom/kernel_guard.h"
#include "render/lod.h"
#include "ui/command_panel.h"
#include "ui/icons.h"
#include "ui/view_cube.h"
#include "ui/theme.h"

#include "core/palette.h"

#include <SDL3/SDL.h>
#include <epoxy/gl.h>

#include "imgui.h"
#include "imgui_internal.h"
#include "backends/imgui_impl_sdl3.h"
#include "backends/imgui_impl_opengl3.h"

#include <chrono>
#include <thread>
#include <cstdio>
#include <algorithm>
#include <cstdlib>
#include <vector>
#include <set>

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

std::string resolveAssetDir() {
    if (const char* env = SDL_getenv("TANGENT_ASSET_DIR")) return env;
    if (const char* base = SDL_GetBasePath()) return std::string(base) + "assets";
    return "assets";
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

    // Not fatal: without them the interface falls back to its words, which is
    // what it had before there were icons at all.
    loadIcons(resolveAssetDir() + "/icons");

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
            // The real interactive path: push the face, type a distance, commit.
            beginFaceMove(FaceOp::Move);
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%g", static_cast<double>(autoExtrudeMm_));
            faceTool_.typedValue = buf;
            updateFaceMove(false);
            while (faceTool_.preview.busy()) updateFaceMove(false);
            updateFaceMove(false);
            // Held open so the live readout can be captured mid-gesture.
            if (!holdTransform_) commitFaceMove();
        }
    }

    if (filletDemoSegments_ > 0 && !scene_.objects().empty()) {
        const ObjectId id = scene_.objects().front()->id;
        const Body& m = scene_.find(id)->body;
        // Edges of the top face, so the demo covers both a lone edge and a
        // loop whose corners have to be patched.
        Index top = 0;
        for (Index f = 0; f < m.faceCount(); ++f)
            if (dot(m.faceNormal(f), Vec3{0, 0, 1}) > 0.99) top = f;
        scene_.clearElementSelection();
        if (filletDemoEdges_ <= 0) {
            // Every edge, not a walk around one face -- which only ever
            // reaches that face's own edges.
            std::vector<EdgeId> all;
            m.allEdges(all);
            for (EdgeId e : all) scene_.selectElement({id, ElementKind::Edge, e}, true);
        } else {
            std::vector<EdgeId> fe;
            m.faceEdges(top, fe);
            for (int i = 0; i < filletDemoEdges_ && i < static_cast<int>(fe.size()); ++i)
                scene_.selectElement({id, ElementKind::Edge, fe[i]}, true);
        }
        view_.bevelWidth = 4.0;
        view_.bevelSegments = filletDemoSegments_;
        filletSelectedEdges();
        scene_.clearElementSelection();
        scene_.select(id);
    }

    if (booleanDemo_ >= 0 && !scene_.objects().empty()) {
        // Drop a cylinder through the startup box and combine them, exercising
        // the same path the menu uses.
        const ObjectId target = scene_.objects().front()->id;
        PrimitiveSpec spec;
        spec.kind = PrimitiveKind::Cylinder;
        spec.cylinder.radius = 6.0;
        spec.cylinder.height = 40.0;
        spec.cylinder.segments = 32;
        // Offset clear of tangency: at radius 6 an offset of 4 would put the
        // cylinder exactly against the box's x = 10 and y = 10 faces, which is
        // the degenerate case the boolean refuses.
        const ObjectId tool = scene_.addPrimitive(PrimitiveKind::Cylinder, spec,
                                                  Vec3{3, 3, 10});
        scene_.clearSelection();
        scene_.select(target);
        scene_.select(tool, true);
        applyBoolean(static_cast<BooleanOp>(booleanDemo_));
    }

    if (filletDemo_ > 0.0f && !scene_.objects().empty()) {
        // Round one edge, then another, each through beginFillet/commitFillet
        // the way F does -- and check the result against rounding both in a
        // single operation, which is unambiguous.
        //
        // Every pair, because which pairs go wrong depends on how the preview
        // renumbered the edges, and guessing one pair is how this was missed.
        // The second fillet goes through extendLastFillet, where a handle
        // picked before the preview was read against the body the preview had
        // replaced: if that stale handle happened to name a real edge, the
        // fillet landed on it instead of the one the user chose.
        const ObjectId id = scene_.objects().front()->id;
        const std::vector<Feature> chain0 = scene_.find(id)->features;
        const Real r = filletDemo_;

        // The edges are remembered by where they are, not by their handles.
        // A handle is only good until the next edit, and the whole point of the
        // sequence being tested is that there is an edit in the middle of it --
        // a harness that reused handles would be reproducing its own bug and
        // blaming the program.
        std::vector<Vec3> mids;
        {
            const Body& b0 = scene_.find(id)->body;
            std::vector<EdgeId> all0;
            b0.allEdges(all0);
            for (EdgeId e : all0) mids.push_back(b0.edgeMidpoint(e));
        }
        const size_t edgeCount = mids.size();

        // Whichever edge of the body as it stands now sits at that point.
        // Rounding an edge shortens the ones that meet it by the same amount at
        // each end, so their midpoints do not move.
        auto edgeAtMidpoint = [&](Vec3 at) -> EdgeId {
            const Body& b = scene_.find(id)->body;
            std::vector<EdgeId> es;
            b.allEdges(es);
            EdgeId best = kInvalid;
            Real bestD = 1e-4;
            for (EdgeId e : es) {
                const Real d = length(b.edgeMidpoint(e) - at);
                if (d < bestD) { bestD = d; best = e; }
            }
            return best;
        };

        auto restore = [&] {
            SceneObject* o = scene_.find(id);
            o->features = chain0;
            scene_.reevaluate(id);
            scene_.clearElementSelection();
        };

        // Rounding both at once: the answer the sequence has to match.
        auto together = [&](Vec3 am, Vec3 bm) -> Real {
            restore();
            SceneObject* o = scene_.find(id);
            const EdgeId a = edgeAtMidpoint(am), b = edgeAtMidpoint(bm);
            if (a == kInvalid || b == kInvalid) return -1.0;
            Feature f;
            f.kind = FeatureKind::Bevel;
            f.edges = nameEdges(o->body, {a, b});
            f.radii.assign(2, r);
            f.width = r;
            f.segments = 4;
            if (!scene_.addFeature(id, std::move(f))) return -1.0;
            return scene_.find(id)->body.health(false).volume;
        };

        // One after the other, through the tool. Reports the volume and how
        // many features the chain ended up with, because the two cases are only
        // comparable when the second fillet was folded into the first: two
        // separate fillet features are a different solid by design -- the
        // second cuts into the first one's surface rather than sharing a corner
        // with it -- and that difference is not this bug.
        int features = 0;
        auto inSequence = [&](Vec3 am, Vec3 bm) -> Real {
            restore();
            view_.bevelSegments = 4;
            for (Vec3 at : {am, bm}) {
                const EdgeId e = edgeAtMidpoint(at);   // resolved against the body as it is now
                if (e == kInvalid) return -1.0;
                scene_.clearElementSelection();
                scene_.selectElement({id, ElementKind::Edge, e}, true);
                beginFillet();
                if (!filletTool_.active) return -1.0;
                filletTool_.typedValue = std::to_string(r);
                updateFillet(false);
                commitFillet();
            }
            features = static_cast<int>(scene_.find(id)->features.size());
            return scene_.find(id)->body.health(false).volume;
        };

        int pairs = 0, wrong = 0, refused = 0, separate = 0;
        for (size_t i = 0; i < edgeCount; ++i) {
            for (size_t j = i + 1; j < edgeCount; ++j) {
                const Real want = together(mids[i], mids[j]);
                if (want < 0.0) continue;              // not a pair this body can round
                const Real got = inSequence(mids[i], mids[j]);
                ++pairs;
                if (got < 0.0) { ++refused; continue; }
                if (features != 2) { ++separate; continue; }   // not folded in; not comparable
                if (std::fabs(got - want) > 1e-6) {
                    ++wrong;
                    if (wrong <= 3)
                        std::fprintf(stderr,
                                     "[fillet] edges %zu+%zu: one at a time gives %.3f mm3, "
                                     "both at once gives %.3f mm3\n",
                                     i, j, got, want);
                }
            }
        }
        restore();
        std::fprintf(stderr,
                     "[fillet] %d pairs at %.2f mm: %d folded into one fillet and %d of those "
                     "landed somewhere else; %d added a second fillet, %d refused (%s)\n",
                     pairs, static_cast<double>(r), pairs - separate - refused, wrong,
                     separate, refused, wrong == 0 ? "all correct" : "WRONG EDGE");
    }

    // A negative distance means the same sequence but selecting every up-facing
    // face -- the rim and the cavity floor together -- which is a second way a
    // person lands here.
    if (shellExtrudeDemo_ != 0.0f && !scene_.objects().empty()) {
        const ObjectId id = scene_.objects().front()->id;
        // Shell the top, the way the menu does.
        {
            SceneObject* o = scene_.find(id);
            std::vector<FaceId> faces;
            o->body.allFaces(faces);
            scene_.clearElementSelection();
            for (FaceId f : faces)
                if (dot(o->body.faceNormal(f), Vec3{0, 0, 1}) > 0.99)
                    scene_.selectElement({id, ElementKind::Face, f}, true);
            view_.shellThickness = 2.0;
            shellActiveObject();
        }
        // Then select what is left of that face -- the rim around the opening
        // -- and extrude it, as a person would by clicking it.
        {
            SceneObject* o = scene_.find(id);
            std::vector<FaceId> faces;
            o->body.allFaces(faces);
            const AABB bb = o->body.bounds();
            scene_.clearElementSelection();
            int picked = 0;
            for (FaceId f : faces) {
                if (dot(o->body.faceNormal(f), Vec3{0, 0, 1}) < 0.99) continue;
                // A negative distance means "every face that points up",
                // which after a shell is the rim *and* the cavity floor --
                // two faces at different heights pushed in one gesture.
                if (shellExtrudeDemo_ > 0.0f &&
                    std::fabs(o->body.faceCentroid(f).z - bb.max.z) > 1e-6) continue;
                scene_.selectElement({id, ElementKind::Face, f}, true);
                ++picked;
            }
            std::fprintf(stderr, "[shell-extrude] rim faces selected: %d\n", picked);
            beginFaceMove(FaceOp::Move);
            char buf[32];
            std::snprintf(buf, sizeof buf, "%g", static_cast<double>(std::fabs(shellExtrudeDemo_)));
            faceTool_.typedValue = buf;
            updateFaceMove(false);
            while (faceTool_.preview.busy()) updateFaceMove(false);
            updateFaceMove(false);
            commitFaceMove();
            const SceneObject* after = scene_.find(id);
            std::fprintf(stderr, "[shell-extrude] %d features, %d faces, %.1f mm3, valid=%d\n",
                         static_cast<int>(after->features.size()), after->body.faceCount(),
                         after->body.health(false).volume, (int)after->body.validate());
        }
    }

    if (snapDemo_ > 0) {
        // A plate with a hole off to one side: the part every snapping feature
        // exists for. The hole gives a centre to be level with, the plate gives
        // corners, and the two together give a crossing.
        scene_.clear();
        PrimitiveSpec plate;
        plate.kind = PrimitiveKind::Box;
        plate.box = {80, 80, 10};
        const ObjectId id = scene_.addPrimitive(PrimitiveKind::Box, plate);

        PrimitiveSpec bore;
        bore.kind = PrimitiveKind::Cylinder;
        bore.cylinder.radius = 9;
        bore.cylinder.height = 40;
        Body tool;
        if (makePrimitive(bore, tool, scene_.defaultBackend())) {
            tool.transform(translate({20, 0, 0}));
            Body out;
            SceneObject* o = scene_.find(id);
            if (booleanOp(o->body, tool, BooleanOp::Difference, out, 77, false, nullptr)) {
                o->body = std::move(out);
                o->refreshDerived();
            }
        }

        camera_.yaw = 0.9f;
        camera_.pitch = 0.95f;
        camera_.distance = 190.0f;
        camera_.target = {0, 0, 0};
        camera_.snapToGoal();

        createTool_.start(PrimitiveKind::Box);
        createTool_.setHoveredPlane(PlaneChoice::Face, {0, 0, 5}, {0, 0, 1}, id, 0);
        createTool_.commitPlaneSelection(camera_);
        camera_.snapToGoal();

        // The cursor is placed per frame, in stepSnapDemo: the viewport has no
        // size until the first frame has been laid out, so a pixel worked out
        // here would point somewhere else by the time anything was drawn.
    }

    if (shellFilletDemo_ && !scene_.objects().empty()) {
        const ObjectId id = scene_.objects().front()->id;
        // The ordinary case first, on solid material: one edge of the untouched
        // cube, where the bound is half the body and not a wall thickness.
        {
            SceneObject* o = scene_.find(id);
            std::vector<EdgeId> es;
            o->body.allEdges(es);
            scene_.clearElementSelection();
            if (!es.empty()) scene_.selectElement({id, ElementKind::Edge, es.front()}, true);
            const auto s0 = std::chrono::steady_clock::now();
            beginFillet();
            const double beganMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - s0).count();
            int frames = 0;
            while (filletTool_.active && filletTool_.search.active && frames < 400) {
                stepFilletLimitSearch();
                ++frames;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            std::fprintf(stderr,
                         "[shell-fillet] solid cube edge: begin %.1f ms  floor %.3f  "
                         "max %.3f  track %.3f\n",
                         beganMs, filletTool_.axis.baseValue, filletTool_.maxRadius,
                         filletTool_.axis.spanValue);
            abortFillet();
            scene_.clearElementSelection();
        }
        {
            SceneObject* o = scene_.find(id);
            std::vector<FaceId> faces;
            o->body.allFaces(faces);
            scene_.clearElementSelection();
            for (FaceId f : faces)
                if (dot(o->body.faceNormal(f), Vec3{0, 0, 1}) > 0.99)
                    scene_.selectElement({id, ElementKind::Face, f}, true);
            view_.shellThickness = 2.0;
            shellActiveObject();
            std::fprintf(stderr, "[shell-fillet] shelled: %d faces\n",
                         scene_.find(id)->body.faceCount());
        }
        {
            SceneObject* o = scene_.find(id);
            std::vector<FaceId> faces;
            o->body.allFaces(faces);
            scene_.clearElementSelection();
            int picked = 0;
            for (FaceId f : faces) {
                if (dot(o->body.faceNormal(f), Vec3{0, -1, 0}) < 0.99) continue;
                scene_.selectElement({id, ElementKind::Face, f}, true);
                ++picked;
            }
            std::fprintf(stderr, "[shell-fillet] side faces selected: %d\n", picked);
            // Where the time actually goes, before anything is changed.
            {
                SceneObject* o2 = scene_.find(id);
                std::vector<EdgeId> es = scene_.selectedEdges(id);
                if (es.empty()) {
                    std::set<EdgeId> set;
                    for (FaceId f : scene_.selectedFaces(id)) {
                        std::vector<EdgeId> fe;
                        o2->body.faceEdges(f, fe);
                        set.insert(fe.begin(), fe.end());
                    }
                    es.assign(set.begin(), set.end());
                }
                auto once = [&](Real r) {
                    Body test = o2->body;
                    FilletSpec spec;
                    spec.segments = 4;
                    for (EdgeId e : es) spec.edges.push_back({e, r});
                    return filletEdges(test, spec);
                };
                const auto a1 = std::chrono::steady_clock::now();
                once(0.5);
                const double direct = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - a1).count();
                const auto a2 = std::chrono::steady_clock::now();
                tryInChild([&] { return once(0.5); });
                const double guarded = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - a2).count();
                const auto a3 = std::chrono::steady_clock::now();
                tryInChild([] { return true; });
                const double bare = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - a3).count();
                std::fprintf(stderr,
                             "[shell-fillet] %zu edges: one fillet %.1f ms, guarded %.1f ms, "
                             "fork alone %.1f ms\n", es.size(), direct, guarded, bare);
            }

            const auto t0 = std::chrono::steady_clock::now();
            beginFillet();
            const double beginMs =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            // A drag, not twenty frames of standing still: the cursor moves a
            // few pixels each frame, which is what decides how often the value
            // actually changes and therefore how often the body is rebuilt.
            double worstUpdate = 0.0, totalUpdate = 0.0;
            int rebuilt = 0, settledOn = -1;
            const int kFrames = 60;
            Real lastR = filletTool_.currentRadius;
            for (int i = 0; i < kFrames && filletTool_.active; ++i) {
                const auto frameStart = std::chrono::steady_clock::now();
                mouseOverride_ = Vec2{700.0 + i * 6.0, 460.0 - i * 4.0};
                updateFillet(true);
                stepFilletLimitSearch();
                const double ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - frameStart).count();
                if (settledOn < 0 && !filletTool_.search.active) settledOn = i + 1;
                if (std::fabs(filletTool_.currentRadius - lastR) > 1e-9) ++rebuilt;
                lastR = filletTool_.currentRadius;
                worstUpdate = std::max(worstUpdate, ms);
                totalUpdate += ms;
                // The rest of a 60 Hz frame. Without it the loop spins far
                // faster than the display and no forked trial can ever finish,
                // which made the limit look like it never moved.
                const double left = 16.6 - ms;
                if (left > 0.0)
                    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(left));
            }
            mouseOverride_ = Vec2{-1, -1};
            std::fprintf(stderr,
                         "[shell-fillet] begin %.1f ms; %d drag frames: %.2f ms each, "
                         "worst %.2f, %d rebuilt, limit settled on frame %d\n",
                         beginMs, kFrames, totalUpdate / kFrames, worstUpdate, rebuilt, settledOn);
            std::fprintf(stderr,
                         "[shell-fillet] began: %d  floor %.3f  max %.3f  track %.3f  notice: %s\n",
                         (int)filletTool_.active, filletTool_.axis.baseValue,
                         filletTool_.maxRadius, filletTool_.axis.spanValue, notice_.c_str());
            if (filletTool_.active) {
                for (int step = 1; step <= 5; ++step)
                    updateFillet(true);
                commitFillet();
                std::fprintf(stderr, "[shell-fillet] committed: %d features, %d faces\n",
                             static_cast<int>(scene_.find(id)->features.size()),
                             scene_.find(id)->body.faceCount());
            }
        }
    }

    if (shellDemo_ > 0.0f && !scene_.objects().empty()) {
        // Drives the menu's own path -- select the top face, then shell -- so
        // this exercises the action and the feature rather than the kernel call.
        const ObjectId id = scene_.objects().front()->id;
        std::vector<FaceId> faces;
        scene_.find(id)->body.allFaces(faces);
        for (FaceId f : faces)
            if (dot(scene_.find(id)->body.faceNormal(f), Vec3{0, 0, 1}) > 0.99)
                scene_.selectElement({id, ElementKind::Face, f}, true);
        view_.shellThickness = shellDemo_;
        shellActiveObject();
        if (!notice_.empty()) std::fprintf(stderr, "[app] %s\n", notice_.c_str());
        scene_.select(id);
    }

    if (measureDemo_ && !scene_.objects().empty()) {
        const ObjectId id = scene_.objects().front()->id;
        const Body& m = scene_.find(id)->body;
        Index top = 0, bottom = 0;
        for (Index f = 0; f < m.faceCount(); ++f) {
            if (dot(m.faceNormal(f), Vec3{0, 0, 1}) > 0.99) top = f;
            if (dot(m.faceNormal(f), Vec3{0, 0, -1}) > 0.99) bottom = f;
        }
        measure_.begin();
        measure_.pick({id, ElementKind::Face, top});
        measure_.pick({id, ElementKind::Face, bottom});
    }

    if (!headlessExport_.empty()) {
        runFileOperation(FileMode::ExportStl, headlessExport_);
        std::fprintf(stderr, "[app] %s\n", notice_.c_str());
    }
    if (fileDemo_ >= 0) beginFilePrompt(static_cast<FileMode>(fileDemo_));

    if (!fixedCamera_) {
        camera_.frame(scene_.bounds());
        camera_.snapToGoal();
    }

    return true;
}

void Application::shutdown() {
    unloadIcons();
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
            ui_.actions.quit = true;
            break;
        case SDL_EVENT_WINDOW_CLOSE_REQUESTED:
            // Routed through the action so closing the window gets the same
            // unsaved-work check as Ctrl+Q.
            if (e.window.windowID == SDL_GetWindowID(window_)) ui_.actions.quit = true;
            break;
        default:
            break;
    }
}

Vec2 Application::mouseInViewport() const {
    // The demos drive gestures with no mouse attached, and a gesture that
    // measures nothing measures nothing useful. Only ever set by them.
    if (mouseOverride_.x >= 0.0) return mouseOverride_;
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

    stepProfileDemo();
    stepFilletOpenDemo();
    stepFaceDemo();
    stepFaceStress();

    // Interactive Object Creation & Sketching Tool:
    if (createTool_.active()) {
        stepSnapDemo();

        // The view stays yours while you draw on it. Orbit and pan are already
        // live above; the wheel was not, because it is handled below a return
        // this branch never reaches -- so zooming in to place a point on a
        // small feature meant cancelling the tool and starting again.
        if (io.MouseWheel != 0.0f && overViewport && !io.WantCaptureMouse)
            camera_.dolly(io.MouseWheel);

        // Held still while the pointer is on the dialog, for the same reason
        // the fillet is: the way to a button is across the screen, and the
        // profile must not follow the pointer there.
        if (!io.WantCaptureMouse)
            createTool_.update(scene_, camera_, mouseInViewport(), !io.KeyCtrl);
        if (!io.WantCaptureMouse) {
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                createTool_.handleMouseDown(mouseInViewport(), scene_, camera_, undo_);
                if (!createTool_.active()) justFinishedModal_ = true;
            } else if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                createTool_.handleMouseUp(mouseInViewport(), camera_);
            } else if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
                createTool_.handleRightClick(camera_);
                if (!createTool_.active()) justFinishedModal_ = true;
            }
        }
        return;
    }

    // Modal face move, and the divide that makes a face to move.
    if (faceTool_.active) {
        updateFaceMove(!io.KeyCtrl, /*follow=*/!io.WantCaptureMouse);
        if (!io.WantCaptureMouse) {
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))       commitFaceMove();
            else if (ImGui::IsMouseClicked(ImGuiMouseButton_Right))  abortFaceMove();
        }
        return;
    }
    if (divideTool_.active) {
        updateDivide(!io.KeyCtrl, /*follow=*/!io.WantCaptureMouse);
        if (!io.WantCaptureMouse) {
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))       commitDivide();
            else if (ImGui::IsMouseClicked(ImGuiMouseButton_Right))  abortDivide();
        }
        return;
    }

    // Modal Fillet tool:
    if (filletTool_.active) {
        updateFillet(!io.KeyCtrl, /*follow=*/!io.WantCaptureMouse);
        if (!io.WantCaptureMouse) {
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))      commitFillet();
            else if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) abortFillet();
        }
        return;
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
    // gesture does not also fire a pick. Finalizing a modal action does not deselect.
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        if (justFinishedModal_) {
            justFinishedModal_ = false;
        } else if (!ImGui::IsMouseDragging(ImGuiMouseButton_Left, 4.0f)) {
            handleViewportClick(io.KeyShift, io.KeyCtrl);
        }
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

void Application::drawReadout(const std::string& text, float px, float py,
                              bool emphasise) {
    if (text.empty()) return;

    auto u8 = [](Real v) { return static_cast<int>(clampf(v, 0.0, 1.0) * 255.0 + 0.5); };
    const Rgb& bg = palette::kMenuBar;
    const Rgb& border = emphasise ? palette::kBrand : palette::kBorder;
    const Rgb& fg = emphasise ? palette::kBrand : palette::kText;

    ImDrawList* dl = ImGui::GetForegroundDrawList();
    const ImVec2 size = ImGui::CalcTextSize(text.c_str());
    const ImVec2 a(px, py);
    const ImVec2 b(px + size.x + 14.0f, py + size.y + 8.0f);

    dl->AddRectFilled(a, b, IM_COL32(u8(bg.r), u8(bg.g), u8(bg.b), 236), 4.0f);
    dl->AddRect(a, b, IM_COL32(u8(border.r), u8(border.g), u8(border.b), 255), 4.0f);
    dl->AddText(ImVec2(px + 7.0f, py + 4.0f),
                IM_COL32(u8(fg.r), u8(fg.g), u8(fg.b), 255), text.c_str());
}

// The fillet's own dialog, in the same shape as the create tool's.
//
// The gesture already says what the radius is -- the arrow, and the number by
// the cursor -- but a gesture cannot say how many segments are being used, how
// many edges were caught, or how to commit without a keyboard. An operation
// with parameters gets a panel; that is the rule the create tool follows and
// there is no reason for this one to be different.
void Application::drawFilletPanel() {
    if (!filletTool_.active) return;

    if (!ui::beginCommand("##fillet", "Fillet", Icon::Fillet,
                          viewRect_.x + 16.0f, viewRect_.y + 16.0f))
        return;

    if (ui::commandNumber("Radius", filletTool_.currentRadius, "mm",
                          !filletTool_.typedValue.empty(), !filletTool_.typedValue.empty(),
                          filletTool_.typedValue.c_str())) {
        // Clicking the field is a way in for the mouse: it clears whatever was
        // typed and hands the radius back to the pointer.
        filletTool_.typedValue.clear();
    }

    char edges[64];
    std::snprintf(edges, sizeof edges, "%zu edge%s", filletTool_.edges.size(),
                  filletTool_.edges.size() == 1 ? "" : "s");
    ui::commandValue("Selection", edges);

    // While the limit is still being found there is no honest number to show
    // for it, and a maximum that grows as you read it is worse than none.
    if (!filletTool_.search.active && filletTool_.maxRadius > 0.0) {
        char limit[48];
        std::snprintf(limit, sizeof limit, "%.2f mm", filletTool_.maxRadius);
        ui::commandValue("Largest", limit);
    }

    ui::commandHint("Pull along the arrow, or type a radius.  Wheel changes the segments.");

    const int footer = ui::commandFooter("OK  (Click)");
    ui::endCommand();

    if (footer > 0)      commitFillet();
    else if (footer < 0) abortFillet();
}

void Application::drawFacePanel() {
    if (!faceTool_.active) return;
    const bool rotate  = faceTool_.op == FaceOp::Rotate;
    const bool extrude = faceTool_.op == FaceOp::Extrude;

    if (!ui::beginCommand("##faceop",
                          rotate ? "Rotate Face" : extrude ? "Extrude" : "Move Face",
                          rotate ? Icon::Chamfer : Icon::Extrude,
                          viewRect_.x + 16.0f, viewRect_.y + 16.0f))
        return;

    if (ui::commandNumber(rotate ? "Angle" : "Distance", faceTool_.value,
                          rotate ? "deg" : "mm", !faceTool_.typedValue.empty(),
                          !faceTool_.typedValue.empty(), faceTool_.typedValue.c_str()))
        faceTool_.typedValue.clear();

    char sel[64];
    std::snprintf(sel, sizeof sel, "%zu face%s", faceTool_.faces.size(),
                  faceTool_.faces.size() == 1 ? "" : "s");
    ui::commandValue("Selection", sel);

    // Which way it goes. The face's own normal unless an axis key says
    // otherwise -- for a rotate, which way round it turns.
    ui::commandRow(rotate ? "Pivot" : "Along");
    static const char* kAxisName[3] = {"X", "Y", "Z"};
    {
        const bool on = faceTool_.lockedAxis < 0;
        if (on) ImGui::PushStyleColor(ImGuiCol_Button,
                                      ImVec4(palette::kBrand.r, palette::kBrand.g,
                                             palette::kBrand.b, 0.85f));
        if (ImGui::Button(rotate ? "Edge" : "Normal")) setFaceAxis(-1);
        if (on) ImGui::PopStyleColor();
    }
    for (int a = 0; a < 3; ++a) {
        ImGui::SameLine();
        const bool on = faceTool_.lockedAxis == a;
        if (on) ImGui::PushStyleColor(ImGuiCol_Button,
                                      ImVec4(palette::kBrand.r, palette::kBrand.g,
                                             palette::kBrand.b, 0.85f));
        if (ImGui::Button(kAxisName[a])) setFaceAxis(a);
        if (on) ImGui::PopStyleColor();
    }

    // What the number is doing to the body, said plainly. Not a choice: moving
    // a face out adds material and moving it in takes some away, and offering
    // to override that would be offering to make the tool lie.
    if (!rotate) {
        ui::commandRow("Result");
        const bool cutting = faceTool_.value < 0.0;
        if (std::fabs(faceTool_.value) < 1e-6)
            ImGui::TextDisabled("unchanged");
        else
            ImGui::TextColored(cutting ? ImVec4(0.95f, 0.35f, 0.25f, 1.0f)
                                       : ImVec4(palette::kBrand.r, palette::kBrand.g,
                                                palette::kBrand.b, 1.0f),
                               "%s", cutting ? "takes material away" : "adds material");
    }

    // Only when it has actually run into something. Fusion asks at this point
    // and not before, and for the same reason: until the material meets
    // another body there is no decision to make.
    if (!rotate && faceTool_.meets != kNoObject) {
        const SceneObject* other = scene_.find(faceTool_.meets);
        ui::commandRow("Meets");
        ImGui::TextColored(ImVec4(palette::kBrand.r, palette::kBrand.g,
                                  palette::kBrand.b, 1.0f),
                           "%s", other ? other->name.c_str() : "another body");

        ui::commandRow("");
        {
            const bool on = !faceTool_.combineWithMeet;
            if (on) ImGui::PushStyleColor(ImGuiCol_Button,
                                          ImVec4(palette::kBrand.r, palette::kBrand.g,
                                                 palette::kBrand.b, 0.85f));
            if (ImGui::Button("Leave")) faceTool_.combineWithMeet = false;
            if (on) ImGui::PopStyleColor();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Two bodies that overlap, left as they are");
        }
        const float ic = ImGui::GetTextLineHeight() * 1.4f;
        ImGui::SameLine();
        if (iconButton(Icon::Union, "mjoin", ic, "Join them into one",
                       faceTool_.combineWithMeet && faceTool_.meetOp == BooleanOp::Union)) {
            faceTool_.combineWithMeet = true;
            faceTool_.meetOp = BooleanOp::Union;
        }
        ImGui::SameLine();
        if (iconButton(Icon::Difference, "mcut", ic, "Cut this one out of it",
                       faceTool_.combineWithMeet && faceTool_.meetOp == BooleanOp::Difference)) {
            faceTool_.combineWithMeet = true;
            faceTool_.meetOp = BooleanOp::Difference;
        }
    }

    ui::commandHint(rotate
        ? "Pull either way across the pivot, or type an angle. X / Y / Z choose "
          "which way it turns."
        : extrude
        ? "Grows a boss off the face and leaves its outline, so you can take "
          "hold of it afterwards. Negative cuts in."
        : "Moves the face; the body follows. X / Y / Z move it along a world "
          "axis instead of its own.");

    const int footer = ui::commandFooter("OK  (Click)");
    ui::endCommand();
    if (footer > 0)      commitFaceMove();
    else if (footer < 0) abortFaceMove();
}

void Application::drawDividePanel() {
    if (!divideTool_.active) return;

    if (!ui::beginCommand("##divide", "Divide", Icon::Inset,
                          viewRect_.x + 16.0f, viewRect_.y + 16.0f))
        return;

    const Real len = length(divideTool_.dir);
    if (ui::commandNumber("Along", divideTool_.t * len, "mm",
                          !divideTool_.typedValue.empty(),
                          !divideTool_.typedValue.empty(),
                          divideTool_.typedValue.c_str()))
        divideTool_.typedValue.clear();

    char of[48];
    std::snprintf(of, sizeof of, "%.2f mm", len);
    ui::commandValue("Edge", of);

    ui::commandHint("The cut runs square across the edge you chose and slides "
                    "along it. The body stays whole.");

    const int footer = ui::commandFooter("OK  (Click)");
    ui::endCommand();
    if (footer > 0)      commitDivide();
    else if (footer < 0) abortDivide();
}

// The live value sits next to the cursor rather than only in the status bar.// The live value sits next to the cursor rather than only in the status bar.
// During a drag the eye is on the geometry, and a number at the bottom of the
// window is somewhere the user is not looking.
void Application::drawTransformReadout() {
    if (filletTool_.active) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "Fillet  %.2f mm", filletTool_.currentRadius);
        ImVec2 at(viewRect_.x + viewRect_.w * 0.5f, viewRect_.y + viewRect_.h * 0.5f);
        if (ImGui::IsMousePosValid()) {
            const ImVec2 m = ImGui::GetIO().MousePos;
            at = ImVec2(m.x + 20.0f, m.y - 34.0f);
        }
        drawReadout(buf, at.x, at.y, /*emphasise=*/true);
        return;
    }


    if (!tool_.active()) return;
    const std::string text = tool_.statusText();
    if (text.empty()) return;

    // If the cursor has never entered the window ImGui reports a sentinel
    // position, and the box would be drawn off-screen. Fall back to the
    // viewport centre so the value is never simply missing.
    ImVec2 at(viewRect_.x + viewRect_.w * 0.5f, viewRect_.y + viewRect_.h * 0.5f);
    if (ImGui::IsMousePosValid()) {
        const ImVec2 m = ImGui::GetIO().MousePos;
        at = ImVec2(m.x + 20.0f, m.y - 34.0f);
    }
    drawReadout(text, at.x, at.y, /*emphasise=*/true);
}

void Application::drawMeasureLabel() {
    if (!measure_.active() || !measureResult_.valid) return;

    // Anchored to the measured span rather than the cursor: a measurement
    // belongs to the geometry it describes, and stays readable once the mouse
    // has moved away.
    const Vec3 mid = (measureResult_.from + measureResult_.to) * 0.5;
    Vec2 px;
    if (!camera_.projectToPixel(mid, px)) return;

    drawReadout(measureResult_.summary,
                static_cast<float>(px.x) + viewRect_.x + 14.0f,
                static_cast<float>(px.y) + viewRect_.y - 12.0f, /*emphasise=*/true);
}

void Application::drawSelectionHighlights() {
    // While a fillet is being dragged, the object's body is the preview -- the
    // edges that were selected have been rounded away and their handles now
    // point at whatever inherited the numbers. Highlighting them draws a red
    // line along an edge nobody chose, which is what made the fillet look like
    // it was about to act on the wrong one. The preview is the feedback here.
    if (filletTool_.active) return;

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

        // An edge is drawn along its curve, not across it. A rim's two ends are
        // the same point, or nearly, so the chord between them runs through the
        // hole instead of around it -- and a fillet or a bore reads as a
        // polygon. Sampled to half a pixel, which is the tolerance the
        // wireframe underneath it already uses.
        std::vector<Vec3> pts;
        auto outlineEdge = [&](EdgeId edge) {
            const Vec3 midW = transformPoint(model, o->body.edgeMidpoint(edge));
            o->body.edgePolyline(edge, camera_.pixelWorldSize(midW) * 0.5, pts);
            for (size_t k = 1; k < pts.size(); ++k)
                renderer_.addLine(lift(transformPoint(model, pts[k - 1])),
                                  lift(transformPoint(model, pts[k])), edgeCol);
        };

        switch (e.kind) {
        case ElementKind::Face: {
            if (!o->body.hasFace(e.index)) break;
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
            std::vector<EdgeId> fe;
            o->body.faceEdges(e.index, fe);
            for (EdgeId edge : fe) {
                // A bridge edge is not an edge of the part: it exists only
                // because a mesh face cannot hold a hole, so it runs from the
                // outline across to the rim. Outlining it draws a line over
                // the opening. A B-rep body has none and answers false.
                if (o->body.isBridgeEdge(edge)) continue;
                outlineEdge(edge);
            }
            break;
        }
        case ElementKind::Edge: {
            if (!o->body.hasEdge(e.index)) break;
            outlineEdge(e.index);
            break;
        }
        case ElementKind::Vertex: {
            if (!o->body.hasVertex(e.index)) break;
            const Vec3 p = lift(transformPoint(model, o->body.vertexPosition(e.index)));
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

    // The two face tools take a number the same way the fillet does. One
    // helper, so a digit means the same thing in all three.
    auto typedInto = [&](std::string& buffer, auto&& refresh) {
        for (int d = 0; d <= 9; ++d) {
            if (ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_0 + d), false) ||
                ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_Keypad0 + d), false)) {
                buffer += static_cast<char>('0' + d);
                refresh();
            }
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Period, false) ||
            ImGui::IsKeyPressed(ImGuiKey_KeypadDecimal, false)) {
            buffer += '.';
            refresh();
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Minus, false) && buffer.empty()) {
            buffer += '-';                       // a negative distance cuts
            refresh();
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Backspace, false) && !buffer.empty()) {
            buffer.pop_back();
            refresh();
        }
    };

    if (faceTool_.active) {
        if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) { abortFaceMove(); return; }
        if (ImGui::IsKeyPressed(ImGuiKey_Enter, false) ||
            ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false)) { commitFaceMove(); return; }
        if (ImGui::IsKeyPressed(ImGuiKey_X, false)) setFaceAxis(0);
        if (ImGui::IsKeyPressed(ImGuiKey_Y, false)) setFaceAxis(1);
        if (ImGui::IsKeyPressed(ImGuiKey_Z, false)) setFaceAxis(2);
        typedInto(faceTool_.typedValue, [&] { updateFaceMove(true); });
        return;
    }

    if (divideTool_.active) {
        if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) { abortDivide(); return; }
        if (ImGui::IsKeyPressed(ImGuiKey_Enter, false) ||
            ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false)) { commitDivide(); return; }
        typedInto(divideTool_.typedValue, [&] { updateDivide(true); });
        return;
    }

    if (filletTool_.active) {
        if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) { abortFillet(); return; }
        if (ImGui::IsKeyPressed(ImGuiKey_Enter, false) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false)) {
            commitFillet();
            return;
        }
        typedInto(filletTool_.typedValue, [&] { updateFillet(true); });
        return;
    }

    if (createTool_.active()) {
        // Escape goes through handleKey rather than straight to cancel: with a
        // number half typed it takes back the mouse, and only then the tool.
        if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
            createTool_.handleKey(27, io.KeyShift, io.KeyCtrl, camera_, scene_, undo_);
            return;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Enter, false) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false) ||
            ImGui::IsKeyPressed(ImGuiKey_Space, false) || ImGui::IsKeyPressed(ImGuiKey_E, false)) {
            createTool_.handleKey('E', io.KeyShift, io.KeyCtrl, camera_, scene_, undo_);
            return;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_F, false)) {
            createTool_.handleKey('F', io.KeyShift, io.KeyCtrl, camera_, scene_, undo_);
            return;
        }
        // Operation, while the depth is being set. handleKey declines these in
        // any other stage, so the key falls through rather than being eaten.
        for (const auto& [imKey, ch] : {std::pair{ImGuiKey_A, 'A'}, std::pair{ImGuiKey_J, 'J'},
                                        std::pair{ImGuiKey_D, 'D'}, std::pair{ImGuiKey_N, 'N'}}) {
            if (ImGui::IsKeyPressed(imKey, false) &&
                createTool_.handleKey(ch, io.KeyShift, io.KeyCtrl, camera_, scene_, undo_))
                return;
        }
        // Digits, and the rest of what a number is made of. The tool decides
        // what each one means: in SelectPlane 1, 3 and 7 pick a plane, and
        // everywhere else a digit is the start of a dimension.
        for (int d = 0; d <= 9; ++d) {
            if ((ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_0 + d), false) ||
                 ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_Keypad0 + d), false)) &&
                createTool_.handleKey('0' + d, io.KeyShift, io.KeyCtrl, camera_, scene_, undo_))
                return;
        }
        for (const auto& [imKey, ch] : {std::pair{ImGuiKey_Period, '.'},
                                        std::pair{ImGuiKey_KeypadDecimal, '.'},
                                        std::pair{ImGuiKey_Minus, '-'},
                                        std::pair{ImGuiKey_KeypadSubtract, '-'},
                                        std::pair{ImGuiKey_Backspace, static_cast<char>(8)},
                                        std::pair{ImGuiKey_Tab, static_cast<char>(9)}}) {
            if (ImGui::IsKeyPressed(imKey, false) &&
                createTool_.handleKey(ch, io.KeyShift, io.KeyCtrl, camera_, scene_, undo_))
                return;
        }
        return;
    }

    const bool ctrl  = io.KeyCtrl;
    const bool shift = io.KeyShift;
    const bool alt   = io.KeyAlt;

    if (ctrl && ImGui::IsKeyPressed(ImGuiKey_Q, false)) ui_.actions.quit = true;

    if (ctrl && !shift) {
        if (ImGui::IsKeyPressed(ImGuiKey_N, false)) ui_.actions.newProject = true;
        if (ImGui::IsKeyPressed(ImGuiKey_O, false)) ui_.actions.openProject = true;
        if (ImGui::IsKeyPressed(ImGuiKey_S, false)) ui_.actions.saveProject = true;
        if (ImGui::IsKeyPressed(ImGuiKey_E, false)) ui_.actions.exportStl = true;
    }

    if (ctrl && ImGui::IsKeyPressed(ImGuiKey_Z, false)) {
        if (shift) ui_.actions.redo = true;
        else       ui_.actions.undo = true;
    }

    // Modal transforms, Blender's G / R / S.
    if (!ctrl && !alt) {
        // The same two keys, and the same two words, acting on whatever is
        // selected. A face is a thing that can be moved and turned just as a
        // body is, and giving those their own letters made the user remember
        // which of two names meant the same operation on a different noun.
        const bool onFace = !scene_.selectedFaces(scene_.contextObject()).empty();
        if (ImGui::IsKeyPressed(ImGuiKey_G, false)) {
            if (onFace) beginFaceMove(FaceOp::Move);
            else        beginTransform(TransformMode::Translate);
        }
        if (ImGui::IsKeyPressed(ImGuiKey_R, false)) {
            if (onFace) beginFaceMove(FaceOp::Rotate);
            else        beginTransform(TransformMode::Rotate);
        }
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
    if (!ctrl && !alt && ImGui::IsKeyPressed(ImGuiKey_K, false)) ui_.actions.divide = true;
    if (ctrl && !shift && ImGui::IsKeyPressed(ImGuiKey_B, false)) ui_.actions.bevel = true;

    // Fillet the selected edges. F, as in Fusion, and reachable by the left
    // hand next to the other edit keys.
    if (!ctrl && !alt && !shift && ImGui::IsKeyPressed(ImGuiKey_F, false))
        ui_.actions.fillet = true;

    // Booleans. Ctrl+Shift keeps them clear of the single-key edit commands.
    if (ctrl && shift) {
        if (ImGui::IsKeyPressed(ImGuiKey_U, false)) {
            ui_.actions.booleanRequested = true;
            ui_.actions.booleanOp = BooleanOp::Union;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_D, false)) {
            ui_.actions.booleanRequested = true;
            ui_.actions.booleanOp = BooleanOp::Difference;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_I, false)) {
            ui_.actions.booleanRequested = true;
            ui_.actions.booleanOp = BooleanOp::Intersection;
        }
    }

    // Numpad view shortcuts; Ctrl gives the opposite side, as in Blender.
    if (ImGui::IsKeyPressed(ImGuiKey_Keypad1, false))
        camera_.setStandardView(ctrl ? StandardView::Back : StandardView::Front);
    if (ImGui::IsKeyPressed(ImGuiKey_Keypad3, false))
        camera_.setStandardView(ctrl ? StandardView::Left : StandardView::Right);
    if (ImGui::IsKeyPressed(ImGuiKey_Keypad7, false))
        camera_.setStandardView(ctrl ? StandardView::Bottom : StandardView::Top);
    if (ImGui::IsKeyPressed(ImGuiKey_Keypad5, false))
        camera_.toggleProjection();
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
bool Application::editKeepsSolid(ObjectId id) {
    SceneObject* obj = scene_.find(id);
    if (!obj) return true;

    const MeshHealth after = obj->body.health();
    obj->health = after;
    obj->healthVersion = obj->meshVersion;

    // Only refuse an edit that broke something that was previously sound. If
    // the model was already open or self-intersecting, blocking further edits
    // would leave the user unable to fix it.
    if (preEditSolid_ && !after.solid()) return false;
    return true;
}

void Application::commitTransform() {
    justFinishedModal_ = true;
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
            const Vec3 toC   = obj->body.faceCentroid(pendingNewFaces_[0]);
            const float distance = dot(toC - fromC, pendingLocalNormal_);

            Feature f;
            f.kind = FeatureKind::Extrude;
            f.faces = nameFaces(pendingMeshBefore_, pendingExtrudeFaces_);
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
        pendingMeshBefore_ = Body{};
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
            f.verts = nameVertices(obj->body, vc->vertices());
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
    justFinishedModal_ = true;
    tool_.cancel(scene_);

    if (pendingMeshObject_ != kNoObject) {
        if (SceneObject* obj = scene_.find(pendingMeshObject_)) {
            obj->features = pendingChainBefore_;
            obj->body = std::move(pendingMeshBefore_);
            obj->refreshDerived();
            scene_.clearElementSelection();
        }
        pendingMeshObject_ = kNoObject;
        pendingMeshBefore_ = Body{};
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
    const float limit = static_cast<float>(maxFilletRadius(obj->body));
    const Real width = std::min(view_.bevelWidth, limit * Real(0.95));
    if (width <= 1e-4f) return;

    Feature f;
    f.kind = FeatureKind::Bevel;
    f.edges.kind = ElementRefs::Kind::All;   // whole-part rounding
    f.width = width;
    f.segments = view_.bevelSegments;

    std::vector<Feature> chainBefore = obj->features;
    if (!scene_.addFeature(target, f)) return;
    undo_.push(std::make_unique<FeatureCommand>(target, std::move(chainBefore),
                                                obj->features, "Bevel"));
}

void Application::shellActiveObject() {
    const ObjectId target = scene_.contextObject();
    SceneObject* obj = scene_.find(target);
    if (!obj) return;

    Feature f;
    f.kind = FeatureKind::Shell;
    f.thickness = view_.shellThickness;

    // Whatever faces are selected are the ones left open. None is not an error:
    // it is a sealed cavity, and the summary says so.
    const std::vector<FaceId> open = scene_.selectedFaces(target);
    if (!open.empty()) f.faces = nameFaces(obj->body, open);

    std::vector<Feature> chainBefore = obj->features;
    std::string why;
    if (!scene_.addFeature(target, f, &why)) {
        notice_ = why.empty() ? "the shell could not be built" : "Shell: " + why;
        return;
    }
    scene_.clearElementSelection();
    undo_.push(std::make_unique<FeatureCommand>(target, std::move(chainBefore),
                                                obj->features, "Shell"));
}

namespace {

// The edge of `mesh` that the segment a-b lies along.
//
// An edge the user picks after an earlier fillet is a shortened piece of the
// edge that fillet was applied to, so matching endpoints does not find it and
// matching the line it lies on does. Ambiguity is reported rather than guessed
// at: two candidates means the caller should not merge.
EdgeId edgeAlongSegment(const Body& body, Vec3 a, Vec3 b) {
    const Vec3 ab = b - a;
    const Real span = length(ab);
    if (span < 1e-9) return kInvalid;
    const Vec3 dir = ab / span;

    EdgeId found = kInvalid;
    std::vector<EdgeId> all;
    body.allEdges(all);
    for (EdgeId h : all) {
        Vec3 p, q;
        body.edgePositions(h, p, q);

        auto covers = [&](Vec3 x) {
            const Real t = dot(x - p, normalize(q - p));
            const Real len = length(q - p);
            return t > -1e-6 && t < len + 1e-6 &&
                   lengthSq(x - (p + normalize(q - p) * t)) < 1e-12;
        };
        // Same line, and it contains the segment the user picked.
        if (std::fabs(std::fabs(dot(normalize(q - p), dir)) - 1.0) > 1e-9) continue;
        if (!covers(a) || !covers(b)) continue;

        if (found != kInvalid) return kInvalid;   // ambiguous
        found = h;
    }
    return found;
}

} // namespace

void Application::filletSelectedEdges() {
    beginFillet();
}

// How much material lies behind the faces a fillet would run along.
//
// This is the bound that actually decides how big a round can be, and nothing
// cheaper finds it: a shelled box is twenty millimetres across and two thick,
// its faces are twenty on a side, and every edge length in the neighbourhood
// says twenty. The wall only shows up as a distance between two faces. So the
// question is asked directly -- stand on the face, look straight into it, and
// see how far it is to the other side.
//
// Rays go against the tessellation rather than the exact shape: it is already
// built, it is what the pointer is picking against anyway, and a chord
// tolerance of a few microns is far below the precision this bound needs.
static Real materialBehindEdges(const SceneObject& obj,
                                const std::vector<Index>& edges, Real fallback) {
    const RenderMesh& rm = obj.render;
    if (rm.triangles.empty()) return fallback;

    Real thinnest = fallback;
    for (Index e : edges) {
        FaceId fa = kInvalid, fb = kInvalid;
        obj.body.edgeFaces(e, fa, fb);
        const Vec3 mid = obj.body.edgeMidpoint(e);
        for (FaceId f : {fa, fb}) {
            if (f == kInvalid) continue;
            const Vec3 n = obj.body.faceNormal(f);
            if (length(n) < 1e-9) continue;

            // Started a hair inside so the face the ray leaves from is not the
            // face it hits, and biased towards the face's own middle so an edge
            // shared with a neighbour does not sight along the seam.
            const Vec3 into = normalize(n) * Real(-1.0);
            const Ray r{mid + into * Real(1e-3), into};

            Real nearest = fallback;
            for (size_t i = 0; i + 2 < rm.triangles.size(); i += 3) {
                Real t = 0.0;
                if (!rayTriangle(r, rm.positions[rm.triangles[i + 0]],
                                 rm.positions[rm.triangles[i + 1]],
                                 rm.positions[rm.triangles[i + 2]], t))
                    continue;
                if (t > 1e-3 && t < nearest) nearest = t;
            }
            thinnest = std::min(thinnest, nearest);
        }
    }
    return std::max(thinnest, Real(0.05));
}

// ---------------------------------------------------------------------------
// Moving a face
// ---------------------------------------------------------------------------

void Application::beginFaceMove(FaceOp op) {
    if (tool_.active() || filletTool_.active || createTool_.active() ||
        faceTool_.active || divideTool_.active)
        return;

    const ObjectId id = scene_.contextObject();
    SceneObject* obj = scene_.find(id);
    if (!obj) { setNotice("Select an object first"); return; }

    const std::vector<Index> faces = scene_.selectedFaces(id);
    if (faces.empty()) { setNotice("Select a face to move"); return; }
    if (obj->body.isMesh() && op == FaceOp::Rotate) {
        setNotice("Rotating a face needs the exact kernel");
        return;
    }

    faceTool_.reset();
    faceTool_.active = true;
    faceTool_.op = op;
    faceTool_.objectId = id;
    faceTool_.faces = faces;
    faceTool_.names = nameFaces(obj->body, faces);
    faceTool_.before = obj->body;
    faceTool_.chainBefore = obj->features;
    faceTool_.value = 0.0;
    preEditSolid_ = obj->healthVersion == obj->meshVersion && obj->health.solid();

    const Mat4 model = obj->modelMatrix();
    const Vec2 at = mouseInViewport();

    // Where the material goes. The sum of the selected faces' normals, so two
    // faces at right angles push out along their bisector rather than along
    // whichever happened to be listed first.
    Vec3 dir{};
    Vec3 anchor{};
    for (FaceId f : faces) {
        dir += normalize(transformVector(normalMatrix(model), obj->body.faceNormal(f)));
        anchor += transformPoint(model, obj->body.faceCentroid(f));
    }
    anchor *= 1.0 / static_cast<Real>(faces.size());
    if (lengthSq(dir) < 1e-12) { faceTool_.active = false; return; }
    dir = normalize(dir);

    const Vec3 extent = obj->body.bounds().size();
    const Real span = std::max(std::min({extent.x, extent.y, extent.z}), Real(1.0));
    faceTool_.direction = dir;

    if (op == FaceOp::Rotate) {
        // The hinge is the edge of the face nearest the cursor: you tip a face
        // about the side you are standing on.
        std::vector<EdgeId> edges;
        obj->body.faceEdges(faces.front(), edges);
        // The first edge unless one is nearer the cursor. Starting from
        // "nothing found" makes the gesture depend on the pointer having a
        // position at all, and a pointer that has never entered the window
        // reports a sentinel that loses every comparison.
        if (edges.empty()) { faceTool_.active = false; return; }
        EdgeId best = edges.front();
        Real bestPx = 1e30;
        for (EdgeId e : edges) {
            Vec2 sp{};
            if (!camera_.projectToPixel(transformPoint(model, obj->body.edgeMidpoint(e)), sp))
                continue;
            const Real d = length(sp - at);
            if (std::isfinite(d) && d < bestPx) { bestPx = d; best = e; }
        }

        Vec3 p, q;
        obj->body.edgePositions(best, p, q);
        faceTool_.hingePoint = p;
        faceTool_.hingeDir = normalize(q - p);

        // The gesture runs across the hinge, in the plane of the face: that is
        // the way the far edge of the face travels as it tips.
        const Vec3 worldHinge = normalize(transformVector(model, q - p));
        Vec3 across = cross(dir, worldHinge);
        if (lengthSq(across) < 1e-12) { faceTool_.active = false; return; }
        faceTool_.axis.origin = anchor;
        faceTool_.axis.direction = normalize(across);
        faceTool_.axis.valid = true;
        faceTool_.axis.signedRange = true;
        faceTool_.axis.baseValue = 0.0;
        faceTool_.axis.spanValue = 60.0;      // degrees for a track's travel
    } else {
        faceTool_.axis.origin = anchor;
        faceTool_.axis.direction = dir;
        faceTool_.axis.valid = true;
        faceTool_.axis.signedRange = true;
        faceTool_.axis.baseValue = 0.0;
        faceTool_.axis.spanValue = span;      // millimetres for a track's travel
    }

    // The track starts under the pointer, as the fillet's does, so the gesture
    // is "how far have I pulled from where I started".
    if (faceTool_.axis.valid) {
        const Real outPx = faceTool_.axis.offsetPx(camera_, at);
        const Real px = static_cast<Real>(camera_.pixelWorldSize(faceTool_.axis.origin));
        faceTool_.axis.origin = faceTool_.axis.origin +
                                faceTool_.axis.direction * (outPx * px);
    }
}

// Points the gesture along a world axis, or back at the face's own normal.
//
// Pressing the same key twice returns to the normal, which is the behaviour
// every axis constraint in the program already has: a modifier you cannot take
// off is a mode, and modes are what this is trying not to be.
void Application::setFaceAxis(int axis) {
    if (!faceTool_.active) return;
    const SceneObject* obj = scene_.find(faceTool_.objectId);
    if (!obj || faceTool_.faces.empty()) return;

    faceTool_.lockedAxis = faceTool_.lockedAxis == axis ? -1 : axis;
    faceTool_.requested = 1e30;          // whatever was built is about the old way
    faceTool_.previewValid = false;
    faceTool_.reachedMax = 1e30;
    faceTool_.reachedMin = -1e30;

    const Mat4 model = obj->modelMatrix();
    Vec3 normal{};
    for (FaceId f : faceTool_.faces)
        normal += normalize(transformVector(normalMatrix(model), faceTool_.before.faceNormal(f)));
    if (lengthSq(normal) < 1e-12) return;
    normal = normalize(normal);

    if (faceTool_.op == FaceOp::Rotate) {
        // For a rotation the axis names the line the face turns about.
        if (faceTool_.lockedAxis >= 0) {
            Vec3 pivot{};
            (&pivot.x)[faceTool_.lockedAxis] = 1.0;
            // It has to lie in the face, or there is nothing to pivot on.
            if (std::fabs(dot(pivot, faceTool_.before.faceNormal(faceTool_.faces.front()))) > 0.99) {
                faceTool_.lockedAxis = -1;
                setNotice("That axis points straight out of the face");
            } else {
                faceTool_.hingeDir = normalize(pivot);
                faceTool_.hingePoint = faceTool_.before.faceCentroid(faceTool_.faces.front());
            }
        }
        const Vec3 worldHinge = normalize(transformVector(model, faceTool_.hingeDir));
        Vec3 across = cross(normal, worldHinge);
        if (lengthSq(across) > 1e-12) faceTool_.axis.direction = normalize(across);
        return;
    }

    Vec3 dir = normal;
    if (faceTool_.lockedAxis >= 0) {
        Vec3 pick{};
        (&pick.x)[faceTool_.lockedAxis] = 1.0;
        // Whichever way along that axis is the way out of the face, so a
        // positive number still means "outward".
        if (dot(pick, normal) < 0.0) pick = pick * Real(-1);
        if (std::fabs(dot(pick, normal)) < 1e-3) {
            faceTool_.lockedAxis = -1;
            setNotice("That axis runs along the face, not into it");
        } else {
            dir = pick;
        }
    }
    faceTool_.direction = dir;
    faceTool_.axis.direction = dir;
}

// Finds the faces this began on in whatever the preview last built.
void Application::refreshFaceSelection() {
    SceneObject* obj = scene_.find(faceTool_.objectId);
    if (!obj || faceTool_.names.empty()) return;

    std::vector<FaceId> now;
    if (!faceTool_.names.resolveFaces(obj->body, now) || now.empty()) return;

    scene_.clearElementSelection();
    for (FaceId f : now) scene_.selectElement({faceTool_.objectId, ElementKind::Face, f}, true);
}

void Application::updateFaceMove(bool snap, bool follow) {
    if (!faceTool_.active) return;
    SceneObject* obj = scene_.find(faceTool_.objectId);
    if (!obj) { abortFaceMove(); return; }

    {
        Body built;
        bool failed = false;
        if (faceTool_.preview.take(built, &failed)) {
            obj->body = std::move(built);
            obj->refreshDerived();
            faceTool_.previewValid = true;
            // The faces are found again by name, so the highlight stays on the
            // ones the gesture began with rather than wandering onto their
            // neighbours as the shape changes under it.
            refreshFaceSelection();

            // Has it grown into anything?
            //
            // Against the material this gesture added, not against the whole
            // body: two bodies standing next to each other have overlapping
            // bounds without touching, and a tool that offered to combine them
            // every time would be asking a question whose answer is almost
            // always no. What is tested is the ground the faces swept through.
            faceTool_.meets = kNoObject;
            if (faceTool_.value > 0.0) {
                const Mat4 model = obj->modelMatrix();
                AABB swept;
                for (FaceId f : faceTool_.faces) {
                    std::vector<VertexId> fv;
                    faceTool_.before.faceVertices(f, fv);
                    for (VertexId v : fv) {
                        const Vec3 p = transformPoint(model, faceTool_.before.vertexPosition(v));
                        swept.expand(p);
                        swept.expand(p + faceTool_.direction * faceTool_.value);
                    }
                }
                for (const auto& other : scene_.objects()) {
                    if (other->id == faceTool_.objectId || !other->visible) continue;
                    const AABB b = other->worldBounds();
                    if (!b.valid() || !swept.valid()) continue;
                    const bool apart = swept.max.x < b.min.x || swept.min.x > b.max.x ||
                                       swept.max.y < b.min.y || swept.min.y > b.max.y ||
                                       swept.max.z < b.min.z || swept.min.z > b.max.z;
                    if (!apart) { faceTool_.meets = other->id; break; }
                }
            }
            if (faceTool_.meets == kNoObject) faceTool_.combineWithMeet = false;
        } else if (failed) {
            // The kernel would not build that far. Where the travel stops is
            // then a fact rather than a guess, and the gesture stops there.
            if (faceTool_.requested > 0.0) faceTool_.reachedMax = faceTool_.requested;
            else                           faceTool_.reachedMin = faceTool_.requested;
            faceTool_.value = clampf(faceTool_.value, faceTool_.reachedMin,
                                     faceTool_.reachedMax);
        }
    }
    if (!follow) return;

    Real want = faceTool_.value;
    if (!faceTool_.typedValue.empty()) {
        try { want = std::stod(faceTool_.typedValue); } catch (...) {}
    } else if (faceTool_.axis.valid) {
        const Vec2 cur = mouseInViewport();
        if (faceTool_.axis.facingCamera(camera_)) {
            want = faceTool_.axis.valueAt(camera_, cur);
        }
        if (snap) {
            const Real step = faceTool_.op == FaceOp::Rotate
                                  ? 5.0
                                  : static_cast<Real>(camera_.snapStep(faceTool_.axis.origin));
            if (step > 0.0) want = std::round(want / step) * step;
        }
    }
    // Never past what the kernel has shown it can do.
    want = clampf(want, faceTool_.reachedMin, faceTool_.reachedMax);
    faceTool_.value = want;

    if (std::fabs(want - faceTool_.requested) < 1e-9 && faceTool_.previewValid) return;
    if (std::fabs(want) < 1e-6) {
        // Back at the start: show the body as it was rather than asking the
        // kernel for a zero-sized operation it will refuse.
        obj->body = faceTool_.before;
        obj->refreshDerived();
        faceTool_.requested = want;
        faceTool_.previewValid = true;
        return;
    }

    faceTool_.requested = want;
    const std::vector<FaceId> faces = faceTool_.faces;
    if (faceTool_.op == FaceOp::Rotate) {
        const Real angle = radians(want);
        const Vec3 hp = faceTool_.hingePoint, hd = faceTool_.hingeDir;
        faceTool_.preview.request(faceTool_.before, [faces, angle, hp, hd](Body& b) {
            return rotateFaces(b, faces, angle, hp, hd, 7001);
        });
    } else {
        // The sign is the operation. Out adds, in cuts; there is nothing else
        // a moved face can mean.
        const bool merge = faceTool_.op == FaceOp::Move;
        const Vec3 along = faceTool_.direction;
        faceTool_.preview.request(faceTool_.before, [faces, want, merge, along](Body& b) {
            return extrudeFaces(b, faces, want, nullptr, 7002, ExtrudeOp::Auto, nullptr,
                                merge, along);
        });
    }
    faceTool_.previewValid = false;
}

void Application::commitFaceMove() {
    if (!faceTool_.active) return;
    const ObjectId id = faceTool_.objectId;
    justFinishedModal_ = true;
    faceTool_.active = false;
    faceTool_.preview.cancel();

    SceneObject* obj = scene_.find(id);
    if (!obj) return;

    std::vector<Feature> chainBefore = faceTool_.chainBefore;
    obj->features = faceTool_.chainBefore;
    obj->body = faceTool_.before;

    if (std::fabs(faceTool_.value) < 1e-6) { obj->refreshDerived(); return; }

    Feature f;
    if (faceTool_.op == FaceOp::Rotate) {
        f.kind = FeatureKind::FaceRotate;
        f.angle = radians(faceTool_.value);
        f.axisPoint = faceTool_.hingePoint;
        f.axisDir = faceTool_.hingeDir;
    } else {
        f.kind = FeatureKind::Extrude;
        f.distance = faceTool_.value;
        f.extrudeOp = ExtrudeOp::Auto;
        f.mergeFlush = faceTool_.op == FaceOp::Move;
        f.axisDir = faceTool_.direction;
    }
    f.faces = nameFaces(faceTool_.before, faceTool_.faces);

    const char* label = faceTool_.op == FaceOp::Rotate    ? "Rotate Face"
                      : faceTool_.op == FaceOp::Extrude   ? "Extrude"
                                                          : "Move Face";
    std::string why;
    if (scene_.addFeature(id, std::move(f), &why) && editKeepsSolid(id)) {
        // If it grew into another body and the user said to combine, that is
        // one more step on the same chain and one more part of the same undo.
        std::vector<std::unique_ptr<Command>> parts;
        parts.push_back(std::make_unique<FeatureCommand>(id, std::move(chainBefore),
                                                         obj->features, label));

        const ObjectId meets = faceTool_.meets;
        if (faceTool_.combineWithMeet && meets != kNoObject) {
            if (SceneObject* other = scene_.find(meets)) {
                std::vector<Feature> beforeBool = obj->features;
                Body baked = other->body;
                baked.transform(inverse(obj->modelMatrix()) * other->modelMatrix());

                Feature b;
                b.kind = FeatureKind::Boolean;
                b.booleanOp = faceTool_.meetOp;
                b.bakedBody = std::move(baked);
                if (scene_.addFeature(id, std::move(b), &why)) {
                    parts.push_back(std::make_unique<FeatureCommand>(
                        id, std::move(beforeBool), obj->features,
                        booleanOpName(faceTool_.meetOp)));
                    renderer_.forget(meets);
                    parts.push_back(ExistenceCommand::forDelete(scene_, {meets}));
                } else {
                    setNotice(why.empty() ? "They could not be combined"
                                          : "Not combined: " + why);
                }
            }
        }

        if (parts.size() == 1) undo_.push(std::move(parts.front()));
        else undo_.push(std::make_unique<CompositeCommand>(std::move(parts), label));
    } else {
        obj->features = std::move(chainBefore);
        obj->body = std::move(faceTool_.before);
        obj->refreshDerived();
        setNotice(why.empty() ? "The face could not be moved" : "Refused: " + why);
    }
}

void Application::abortFaceMove() {
    if (!faceTool_.active) return;
    justFinishedModal_ = true;
    faceTool_.active = false;
    faceTool_.preview.cancel();
    if (SceneObject* obj = scene_.find(faceTool_.objectId)) {
        obj->features = std::move(faceTool_.chainBefore);
        obj->body = std::move(faceTool_.before);
        obj->refreshDerived();
    }
}

// ---------------------------------------------------------------------------
// Dividing a face
// ---------------------------------------------------------------------------

// Drops every division that does not define the shape.
//
// Not a gesture: there is nothing to drag and nothing to choose, so it happens
// and says how much it removed. One step on the chain like any other, so it can
// be undone and so a later edit rebuilds through it.
void Application::mergeSelected() {
    const ObjectId id = scene_.contextObject();
    SceneObject* obj = scene_.find(id);
    if (!obj) { setNotice("Select an object first"); return; }
    if (obj->body.isMesh()) { setNotice("Merging faces needs the exact kernel"); return; }

    const int before = obj->body.faceCount();
    std::vector<Feature> chainBefore = obj->features;
    preEditSolid_ = obj->healthVersion == obj->meshVersion && obj->health.solid();

    Feature f;
    f.kind = FeatureKind::Merge;

    std::string why;
    if (!scene_.addFeature(id, std::move(f), &why) || !editKeepsSolid(id)) {
        obj->features = std::move(chainBefore);
        scene_.reevaluate(id);
        setNotice(why.empty() ? "Nothing to merge" : "Not merged: " + why);
        return;
    }

    scene_.clearElementSelection();
    undo_.push(std::make_unique<FeatureCommand>(id, std::move(chainBefore),
                                                obj->features, "Merge Faces"));
    const int after = obj->body.faceCount();
    char msg[96];
    std::snprintf(msg, sizeof msg, "Merged %d face%s into %d", before,
                  before == 1 ? "" : "s", after);
    setNotice(msg);
}

void Application::beginDivide() {
    if (tool_.active() || filletTool_.active || createTool_.active() ||
        faceTool_.active || divideTool_.active)
        return;

    const ObjectId id = scene_.contextObject();
    SceneObject* obj = scene_.find(id);
    if (!obj) { setNotice("Select an object first"); return; }
    if (obj->body.isMesh()) { setNotice("Dividing a face needs the exact kernel"); return; }

    // The cut runs square across a chosen edge and slides along it, which is
    // what a loop cut is: pick the direction by pointing at an edge.
    std::vector<Index> edges = scene_.selectedEdges(id);
    if (edges.empty()) {
        setNotice("Select an edge for the cut to run across");
        return;
    }

    const Mat4 model = obj->modelMatrix();
    Vec3 p, q;
    obj->body.edgePositions(edges.front(), p, q);
    if (lengthSq(q - p) < 1e-12) { setNotice("That edge has no length"); return; }

    divideTool_.reset();
    divideTool_.active = true;
    divideTool_.objectId = id;
    divideTool_.along = edges.front();
    divideTool_.from = p;
    divideTool_.dir = q - p;
    divideTool_.t = 0.5;
    divideTool_.before = obj->body;
    divideTool_.chainBefore = obj->features;
    preEditSolid_ = obj->healthVersion == obj->meshVersion && obj->health.solid();

    divideTool_.axis.origin = transformPoint(model, p);
    divideTool_.axis.direction = normalize(transformVector(model, q - p));
    divideTool_.axis.valid = true;
    divideTool_.axis.baseValue = 0.0;
    divideTool_.axis.spanValue = length(q - p);
}

void Application::updateDivide(bool snap, bool follow) {
    if (!divideTool_.active) return;
    SceneObject* obj = scene_.find(divideTool_.objectId);
    if (!obj) { abortDivide(); return; }

    {
        Body built;
        if (divideTool_.preview.take(built)) {
            obj->body = std::move(built);
            obj->refreshDerived();
            divideTool_.previewValid = true;
        }
    }
    if (!follow) return;

    const Real len = length(divideTool_.dir);
    Real along = divideTool_.t * len;
    if (!divideTool_.typedValue.empty()) {
        try { along = std::stod(divideTool_.typedValue); } catch (...) {}
    } else if (divideTool_.axis.valid && divideTool_.axis.facingCamera(camera_)) {
        along = divideTool_.axis.valueAt(camera_, mouseInViewport());
    }
    if (snap) {
        const Real step = static_cast<Real>(camera_.snapStep(divideTool_.axis.origin));
        if (step > 0.0) along = std::round(along / step) * step;
    }
    // Never on top of either end: a cut through a corner divides nothing and
    // the kernel will refuse it.
    along = clampf(along, len * 0.02, len * 0.98);
    divideTool_.t = along / len;

    if (std::fabs(along - divideTool_.requested) < 1e-9 && divideTool_.previewValid) return;
    divideTool_.requested = along;
    divideTool_.previewValid = false;

    const Vec3 at = divideTool_.from + normalize(divideTool_.dir) * along;
    const Vec3 n = normalize(divideTool_.dir);
    divideTool_.preview.request(divideTool_.before, [at, n](Body& b) {
        return divideBody(b, at, n, 7003);
    });
}

void Application::commitDivide() {
    if (!divideTool_.active) return;
    const ObjectId id = divideTool_.objectId;
    justFinishedModal_ = true;
    divideTool_.active = false;
    divideTool_.preview.cancel();

    SceneObject* obj = scene_.find(id);
    if (!obj) return;

    std::vector<Feature> chainBefore = divideTool_.chainBefore;
    obj->features = divideTool_.chainBefore;
    obj->body = divideTool_.before;

    Feature f;
    f.kind = FeatureKind::Divide;
    f.axisPoint = divideTool_.from + normalize(divideTool_.dir) *
                  (divideTool_.t * length(divideTool_.dir));
    f.axisDir = normalize(divideTool_.dir);

    std::string why;
    if (scene_.addFeature(id, std::move(f), &why) && editKeepsSolid(id)) {
        undo_.push(std::make_unique<FeatureCommand>(id, std::move(chainBefore),
                                                    obj->features, "Divide"));
        scene_.clearElementSelection();
    } else {
        obj->features = std::move(chainBefore);
        obj->body = std::move(divideTool_.before);
        obj->refreshDerived();
        setNotice(why.empty() ? "The divide could not be made" : "Refused: " + why);
    }
}

void Application::abortDivide() {
    if (!divideTool_.active) return;
    justFinishedModal_ = true;
    divideTool_.active = false;
    divideTool_.preview.cancel();
    if (SceneObject* obj = scene_.find(divideTool_.objectId)) {
        obj->features = std::move(divideTool_.chainBefore);
        obj->body = std::move(divideTool_.before);
        obj->refreshDerived();
    }
}

void Application::beginFillet() {
    if (tool_.active() || filletTool_.active || createTool_.active()) return;

    const ObjectId id = scene_.contextObject();
    SceneObject* obj = scene_.find(id);
    if (!obj) {
        setNotice("Select an object to fillet");
        return;
    }

    std::vector<Index> edges = scene_.selectedEdges(id);
    if (edges.empty()) {
        // Requirement 1: Clicking F (or edge action) while a face is selected
        // should perform the action on all edges connected to the selected face.
        const std::vector<Index> faces = scene_.selectedFaces(id);
        if (!faces.empty()) {
            std::set<Index> faceEdges;
            for (Index f : faces) {
                if (f < obj->body.faceCount()) {
                    std::vector<EdgeId> fe;
                    obj->body.faceEdges(f, fe);
                    faceEdges.insert(fe.begin(), fe.end());
                }
            }
            edges.assign(faceEdges.begin(), faceEdges.end());
        }
    }

    if (edges.empty()) {
        setNotice("Select edges or faces to fillet");
        return;
    }

    // Requirement 2: If a fillet is attempted on an edge composed of multiple sections,
    // extend selection to remaining sections and seamlessly act in unison.
    edges = extendTangentChain(obj->body, edges);

    // Nothing about the geometry is decided here. Every trial -- the floor the
    // gesture starts from and the limit it can reach -- runs in another
    // process, a frame at a time, because OpenCASCADE does not always refuse:
    // a fillet whose radius is exactly the wall thickness it is rounding takes
    // the process with it, and a modelling tool may decline an operation but
    // may not lose the model. See geom/kernel_guard.h.
    //
    // Running the first of those trials on the click cost fifty milliseconds of
    // dead frames before the guide appeared. Now the guide appears at once and
    // the floor arrives three frames later, which is the difference between a
    // tool that opens and one that hesitates.
    const Real minRadius = kFilletFloorLadder[0];
    const Real initialWidth = minRadius;

    // How far this gesture can go is bracketed rather than hunted for. No
    // fillet can be larger than half the smallest dimension of the body it is
    // cut into -- there is not the material -- so the answer is between the
    // smallest radius that builds and that. Doubling upward from 0.05 to find
    // the same bracket cost six trials before the bisection even started.
    //
    // The bisection itself runs a trial per frame in another process, so the
    // gesture begins immediately and the limit narrows over the next few
    // frames. Until it lands, the limit is the largest radius actually
    // verified, so the track never offers travel that has not been checked.
    // How far this can possibly go, bounded twice and tightly.
    //
    // Half the smallest dimension of the whole body is a true bound but a very
    // loose one: on a shelled box it says ten millimetres where the wall gives
    // out at two, and a track scaled to ten puts the entire usable range in the
    // first forty pixels of travel. The shortest edge of any face the fillet
    // runs into is the local bound, and it is the one that bites -- it is the
    // wall thickness, the width of the rib, the flat the round has to fit on.
    const Vec3 extent = obj->body.bounds().size();
    const Real smallest = std::max(std::min({extent.x, extent.y, extent.z}), Real(0.1));

    const Real nearby = materialBehindEdges(*obj, edges, smallest);
    const Real ceiling = std::max(std::min(smallest * 0.5, nearby), Real(0.05));
    const Real maxRadius = 0.0;   // nothing verified yet

    // Select all extended edges in the scene so the highlight displays them
    scene_.clearElementSelection();
    for (Index e : edges) {
        scene_.selectElement({id, ElementKind::Edge, e}, true);
    }

    filletTool_.active = true;
    filletTool_.objectId = id;
    filletTool_.edges = edges;
    // Only a fallback now: the first updateFillet replaces it with the distance
    // from the cursor to the edge, so the preview starts where the pointer is
    // rather than jumping to an arbitrary width.
    filletTool_.baseRadius = initialWidth;
    filletTool_.currentRadius = initialWidth;
    filletTool_.maxRadius = maxRadius;
    filletTool_.search.active = true;
    filletTool_.search.floorPhase = true;
    filletTool_.search.floorIndex = 0;
    filletTool_.search.ceiling = ceiling;
    filletTool_.search.hardCeiling = std::max(smallest * 0.5, ceiling);
    filletTool_.search.good = minRadius;
    filletTool_.search.bad = std::max(ceiling, minRadius * 4.0);
    filletTool_.search.stepsLeft = 8;
    filletTool_.search.pending = 0.0;
    filletTool_.search.testedTop = false;
    filletTool_.previewValid = false;

    filletTool_.requestedRadius = -1.0;

    filletTool_.meshBefore = obj->body;
    filletTool_.chainBefore = obj->features;
    filletTool_.typedValue.clear();

    preEditSolid_ = obj->healthVersion == obj->meshVersion && obj->health.solid();

    // The guide is fixed once, now, and does not move again for the rest of
    // the gesture.
    {
        const Vec2 at = mouseInViewport();
        const Mat4 model = obj->modelMatrix();

        // A point on the selection to aim from: whichever part of it the
        // pointer is nearest, in world space at the cursor's own depth.
        Vec3 nearest{};
        Real bestPx = -1.0;
        for (EdgeId e : edges) {
            if (!filletTool_.meshBefore.hasEdge(e)) continue;
            Vec3 aL, bL;
            filletTool_.meshBefore.edgePositions(e, aL, bL);
            const Vec3 aW = transformPoint(model, aL);
            const Vec3 bW = transformPoint(model, bL);
            Vec2 aPx, bPx;
            if (!camera_.projectToPixel(aW, aPx) || !camera_.projectToPixel(bW, bPx)) continue;
            const Vec2 ab = bPx - aPx;
            const Real len2 = lengthSq(ab);
            const Real t = len2 > 1e-9 ? clampf(dot(at - aPx, ab) / len2, 0.0, 1.0) : 0.0;
            const Real d = length(at - (aPx + ab * t));
            if (bestPx < 0.0 || d < bestPx) { bestPx = d; nearest = lerp(aW, bW, t); }
        }

        filletTool_.axis = filletAxis(filletTool_.meshBefore, model, edges, nearest);
        if (filletTool_.axis.valid) {
            // Slide the origin out to sit under the pointer, so the track
            // starts in the hand and the gesture is "how far have I pulled
            // from where I started" -- which cannot invert, and does not depend
            // on where along the edge the click landed. Measured in pixels and
            // converted, because the track itself is a screen-space length.
            const Real outPx = std::max(Real(0), filletTool_.axis.offsetPx(camera_, at));
            const Real px = camera_.pixelWorldSize(filletTool_.axis.origin);
            filletTool_.axis.origin =
                filletTool_.axis.origin + filletTool_.axis.direction * (outPx * px);

            // The track's range is fixed for the whole gesture, from the
            // smallest fillet this selection will take to the most the body
            // could possibly hold. It must not move while the search narrows:
            // a track that grew under the cursor would inflate the radius
            // while the hand was still.
            //
            // What the search changes is how far along that track the gesture
            // may go -- the arrow stops at the largest radius actually
            // verified, and the cap marks where the shape gives up.
            filletTool_.axis.baseValue = minRadius;
            filletTool_.axis.spanValue = std::max(ceiling, minRadius * 4.0);
        }
    }

    updateFillet(false);
}

// The smallest fillets worth offering, in the order they are tried. The floor
// belongs to the shape, not to the session: starting from the width last used
// made a 2mm fillet the smallest you could ask for next time, and the arrow had
// nowhere to go but out.
const Real Application::kFilletFloorLadder[6] = {0.05, 0.1, 0.25, 0.5, 1.0, 2.5};

// Runs one trial of the floor ladder. Returns once a trial is in flight.
void Application::stepFilletFloorSearch() {
    FilletToolState::LimitSearch& s = filletTool_.search;

    if (s.trial.running() || s.trial.finished()) {
        if (!s.trial.poll()) return;                 // still working
        const Real tried = kFilletFloorLadder[s.floorIndex];
        if (s.trial.result() == Attempt::Ok) {
            // The gesture has a floor. The track starts there, and the arrow
            // may go exactly that far until the limit search says otherwise.
            s.floorPhase = false;
            filletTool_.baseRadius = tried;
            filletTool_.maxRadius = tried;
            filletTool_.axis.baseValue = tried;
            filletTool_.axis.spanValue = std::max(s.ceiling, tried * 4.0);
            filletTool_.currentRadius = std::max(filletTool_.currentRadius, tried);
            s.good = tried;
            s.bad = std::max(s.ceiling, tried * 4.0);
            s.testedTop = false;
            s.stepsLeft = 8;
            return;
        }

        ++s.floorIndex;
        if (s.floorIndex >= static_cast<int>(std::size(kFilletFloorLadder))) {
            // Nothing on the ladder builds. Only now is it worth paying for a
            // reason: the radius is re-run in this process to collect one, and
            // only because the guarded trial proved it is not fatal.
            std::string why;
            if (s.trial.result() == Attempt::Refused) {
                Body test = filletTool_.meshBefore;
                FilletSpec spec;
                for (Index e : filletTool_.edges) spec.edges.push_back({e, tried});
                filletEdges(test, spec, &why);
            }
            abortFillet();
            setNotice(why.empty() ? "No room for a fillet on selected edges"
                                  : "Cannot fillet these edges: " + why);
            return;
        }
    }

    s.pending = kFilletFloorLadder[s.floorIndex];
    startFilletTrial(s.pending);
}

// Puts one guarded fillet at `radius` in flight.
void Application::startFilletTrial(Real radius) {
    const Body start = filletTool_.meshBefore;
    const std::vector<Index> edges = filletTool_.edges;
    filletTool_.search.trial.start([start, edges, radius] {
        Body test = start;
        FilletSpec spec;
        for (Index e : edges) spec.edges.push_back({e, radius});
        return filletEdges(test, spec);
    });
}

// Where the demo stands, in the sketch plane's own coordinates, and how far off
// the thing it is meant to catch.
void Application::stepSnapDemo() {
    if (snapDemo_ <= 0 || !createTool_.active()) return;

    const Vec2 targets[5] = {{0, 0}, {20.6, -31.0}, {20.5, 39.4}, {10.4, 20.3}, {20.0, 0.0}};
    const Vec2 uv = targets[std::min(snapDemo_, 4)];
    Vec2 px{};
    if (!camera_.projectToPixel(Vec3{uv.x, uv.y, 5.0}, px)) return;
    if (snapDemo_ == 4) px += Vec2{4.0, 3.0};
    mouseOverride_ = px;

    static int reported = -1;
    if (reported != snapDemo_) {
        reported = snapDemo_;
        createTool_.update(scene_, camera_, px, true);
        const PlaneSnap& hit = createTool_.activeSnap();

        // What one frame of sketching costs. This runs on every mouse move, so
        // it has to stay beneath the frame it is drawn in.
        const auto t0 = std::chrono::steady_clock::now();
        constexpr int kReps = 50;
        for (int i = 0; i < kReps; ++i)
            createTool_.update(scene_, camera_, px + Vec2{i * 0.01, 0.0}, true);
        const double each = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - t0).count() / kReps;

        std::fprintf(stderr, "[snap-demo] at %.2f,%.2f -> %s  (%.3f, %.3f)  refs %d  %.3f ms/frame\n",
                     uv.x, uv.y, hit.valid() ? describeSnap(hit).c_str() : "nothing",
                     hit.uv.x, hit.uv.y, hit.refCount, each);
    }
}

// Drives the create tool through its steps so the interface can be looked at.
// Runs once, on the first frame the viewport has a size: a pixel worked out
// before then points somewhere else by the time anything is drawn.
void Application::stepProfileDemo() {
    if (profileDemo_ <= 0 || profileDemoDone_ || viewRect_.w <= 0) return;
    profileDemoDone_ = true;

    const ObjectId id = scene_.objects().empty() ? kNoObject : scene_.objects().front()->id;
    if (id == kNoObject) return;
    const SceneObject* o = scene_.find(id);
    const Real top = o->worldBounds().max.z;   // the face you can see, not local z

    camera_.yaw = 0.55f;
    camera_.pitch = 0.85f;
    camera_.distance = 90.0f;
    camera_.target = {0, 0, 0};
    camera_.snapToGoal();

    createTool_.start(PrimitiveKind::Box);
    if (profileDemo_ == 4) return;             // stop at the plane question
    createTool_.setHoveredPlane(PlaneChoice::Face, {0, 0, top}, {0, 0, 1}, id, 0);
    createTool_.commitPlaneSelection(camera_);
    camera_.snapToGoal();

    auto px = [&](Vec2 uv) {
        Vec2 p{};
        camera_.projectToPixel(createTool_.plane().toWorld(uv), p);
        return p;
    };
    createTool_.update(scene_, camera_, px({-6, -5}), false);
    createTool_.handleMouseDown(px({-6, -5}), scene_, camera_, undo_);
    createTool_.update(scene_, camera_, px({6, 5}), false);
    createTool_.handleMouseDown(px({6, 5}), scene_, camera_, undo_);
    camera_.snapToGoal();

    if (profileDemo_ >= 2) {
        createTool_.update(scene_, camera_, px({6, 5}), false);
        createTool_.handleKey('F', false, false, camera_, scene_, undo_);
        createTool_.update(scene_, camera_, px({6, 2.5}), false);
        // Held, or every frame after this one re-runs the update from wherever
        // the real pointer happens to be -- which on a machine with nobody at
        // it is the corner of the window.
        mouseOverride_ = px({6, 2.5});
    } else {
        mouseOverride_ = px({9, 5});        // off the profile, nothing hovered
    }
    if (profileDemo_ >= 3) {
        createTool_.handleKey(13, false, false, camera_, scene_, undo_);
        createTool_.handleKey(13, false, false, camera_, scene_, undo_);
    }
    std::fprintf(stderr, "[profile-demo] stage %d, %.2f x %.2f, rounds %.2f\n",
                 static_cast<int>(createTool_.stage()),
                 createTool_.profileMax().x - createTool_.profileMin().x,
                 createTool_.profileMax().y - createTool_.profileMin().y,
                 createTool_.uniformCornerRadius());
}

void Application::stepFilletOpenDemo() {
    if (!filletOpen_ || filletOpenDone_ || viewRect_.w <= 0) return;
    if (scene_.objects().empty()) return;
    filletOpenDone_ = true;

    const ObjectId id = scene_.objects().front()->id;
    const SceneObject* o = scene_.find(id);
    std::vector<EdgeId> es;
    o->body.allEdges(es);
    if (es.empty()) return;

    camera_.yaw = 0.6f;
    camera_.pitch = 0.55f;
    camera_.distance = 80.0f;
    camera_.snapToGoal();

    // A top edge, so the arrow is drawn where it can be seen.
    EdgeId pick = es.front();
    Real best = -1e30;
    for (EdgeId e : es) {
        Vec3 a2, b2;
        o->body.edgePositions(e, a2, b2);
        const Real z = (a2.z + b2.z) * 0.5;
        if (z > best) { best = z; pick = e; }
    }
    scene_.select(id);
    scene_.clearElementSelection();
    scene_.selectElement({id, ElementKind::Edge, pick}, true);

    Vec2 px{};
    camera_.projectToPixel(o->body.edgeMidpoint(pick), px);
    mouseOverride_ = px;
    beginFillet();
    mouseOverride_ = px + Vec2{40.0, -40.0};
}

void Application::stepFaceDemo() {
    if (faceDemo_ <= 0 || faceDemoDone_ || viewRect_.w <= 0) return;
    if (scene_.objects().empty()) return;
    faceDemoDone_ = true;

    const ObjectId id = scene_.objects().front()->id;
    camera_.yaw = 0.6f;
    camera_.pitch = 0.5f;
    camera_.distance = 95.0f;
    camera_.snapToGoal();
    scene_.select(id);

    auto topFace = [&] {
        const SceneObject* o = scene_.find(id);
        std::vector<FaceId> fs;
        o->body.allFaces(fs);
        FaceId best = kInvalid;
        Real bestDot = -1e30;
        for (FaceId f : fs) {
            const Real d = dot(o->body.faceNormal(f), Vec3{0, 0, 1});
            if (d > bestDot) { bestDot = d; best = f; }
        }
        return best;
    };

    auto report = [&](const char* what) {
        const SceneObject* o = scene_.find(id);
        std::fprintf(stderr,
                     "[face-demo] %s: %d faces, %.1f mm3, valid=%d, %zu features, "
                     "%zu selected\n",
                     what, o->body.faceCount(), o->body.health(false).volume,
                     (int)o->body.validate(), o->features.size(),
                     scene_.selectedFaces(id).size());
    };

    if (faceDemo_ == 10) {
        // Cut a line, move an unrelated face, then ask for the line to go.
        const SceneObject* o0 = scene_.find(id);
        std::vector<EdgeId> es;
        o0->body.allEdges(es);
        EdgeId along = kInvalid;
        for (EdgeId e : es) {
            Vec3 p, q;
            o0->body.edgePositions(e, p, q);
            if (std::fabs((q - p).z) > 1e-6) { along = e; break; }
        }
        if (along == kInvalid) return;
        scene_.clearElementSelection();
        scene_.selectElement({id, ElementKind::Edge, along}, true);
        beginDivide();
        divideTool_.typedValue = "10";
        updateDivide(false);
        while (divideTool_.preview.busy()) updateDivide(false);
        updateDivide(false);
        commitDivide();
        report("divided");

        const SceneObject* o = scene_.find(id);
        std::vector<FaceId> fs;
        o->body.allFaces(fs);
        FaceId end = fs.front();
        Real best = -1e30;
        for (FaceId f : fs) {
            const Real d = dot(o->body.faceNormal(f), Vec3{0, 1, 0});
            if (d > best) { best = d; end = f; }
        }
        scene_.clearElementSelection();
        scene_.selectElement({id, ElementKind::Face, end}, true);
        beginFaceMove(FaceOp::Move);
        faceTool_.typedValue = "4";
        updateFaceMove(false);
        while (faceTool_.preview.busy()) updateFaceMove(false);
        updateFaceMove(false);
        commitFaceMove();
        report("moved an unrelated face");

        scene_.select(id);
        mergeSelected();
        report("merged");
        return;
    }

    if (faceDemo_ == 9) {
        // A second body in the way, so the extrude runs into it and the tool
        // has something to ask about.
        PrimitiveSpec spec;
        spec.kind = PrimitiveKind::Box;
        spec.box = {10, 10, 10};
        const ObjectId other = scene_.addPrimitive(PrimitiveKind::Box, spec, {0, 0, 40});
        scene_.select(id);
        scene_.clearElementSelection();
        scene_.selectElement({id, ElementKind::Face, topFace()}, true);
        beginFaceMove(FaceOp::Extrude);
        faceTool_.typedValue = "3";
        updateFaceMove(false);
        while (faceTool_.preview.busy()) updateFaceMove(false);
        updateFaceMove(false);
        std::fprintf(stderr, "[face-demo] short extrude meets: %s\n",
                     faceTool_.meets == kNoObject ? "nothing" : "something");
        faceTool_.typedValue = "20";
        updateFaceMove(false);
        while (faceTool_.preview.busy()) updateFaceMove(false);
        updateFaceMove(false);
        std::fprintf(stderr, "[face-demo] long extrude meets: %s\n",
                     faceTool_.meets == other ? "the other body" : "nothing");
        faceTool_.combineWithMeet = true;
        faceTool_.meetOp = BooleanOp::Union;
        commitFaceMove();
        std::fprintf(stderr, "[face-demo] after joining: %zu objects\n",
                     scene_.objects().size());
        report("joined on the way");
        return;
    }

    if (faceDemo_ == 7) {
        // A move along a world axis rather than the face's own normal.
        scene_.clearElementSelection();
        scene_.selectElement({id, ElementKind::Face, topFace()}, true);
        beginFaceMove(FaceOp::Move);
        setFaceAxis(2);
        faceTool_.typedValue = "5";
        updateFaceMove(false);
        while (faceTool_.preview.busy()) updateFaceMove(false);
        updateFaceMove(false);
        report("moved along Z");
        commitFaceMove();
        return;
    }

    if (faceDemo_ == 8) {
        // A rotation the other way, which the signed range has to allow.
        scene_.clearElementSelection();
        scene_.selectElement({id, ElementKind::Face, topFace()}, true);
        beginFaceMove(FaceOp::Rotate);
        faceTool_.typedValue = "-15";
        updateFaceMove(false);
        while (faceTool_.preview.busy()) updateFaceMove(false);
        updateFaceMove(false);
        report("rotated the other way");
        commitFaceMove();
        return;
    }

    if (faceDemo_ == 6) {
        // The same pull, as an extrude: the boss keeps its outline.
        scene_.clearElementSelection();
        scene_.selectElement({id, ElementKind::Face, topFace()}, true);
        beginFaceMove(FaceOp::Extrude);
        faceTool_.typedValue = "6";
        updateFaceMove(false);
        while (faceTool_.preview.busy()) updateFaceMove(false);
        updateFaceMove(false);
        report("extruded");
        commitFaceMove();
        return;
    }

    if (faceDemo_ >= 4 && faceDemo_ <= 5) {
        // A loop cut across the top, then push one half of it up.
        const SceneObject* o = scene_.find(id);
        std::vector<EdgeId> es;
        o->body.allEdges(es);
        EdgeId along = kInvalid;
        for (EdgeId e : es) {
            Vec3 p, q;
            o->body.edgePositions(e, p, q);
            if (std::fabs((q - p).x) > 1e-6 && std::fabs((q - p).y) < 1e-6 &&
                std::fabs((q - p).z) < 1e-6) { along = e; break; }
        }
        if (along == kInvalid) return;
        scene_.clearElementSelection();
        scene_.selectElement({id, ElementKind::Edge, along}, true);
        beginDivide();
        divideTool_.typedValue = "10";
        updateDivide(false);
        while (divideTool_.preview.busy()) updateDivide(false);
        updateDivide(false);
        commitDivide();
        report("divided");

        if (faceDemo_ >= 5) {
            const FaceId half = topFace();
            scene_.clearElementSelection();
            scene_.selectElement({id, ElementKind::Face, half}, true);
            beginFaceMove(FaceOp::Move);
            faceTool_.typedValue = "4";
            updateFaceMove(false);
            while (faceTool_.preview.busy()) updateFaceMove(false);
            updateFaceMove(false);
            commitFaceMove();
            report("pushed one half");
        }
        return;
    }

    scene_.clearElementSelection();
    scene_.selectElement({id, ElementKind::Face, topFace()}, true);

    if (faceDemo_ == 3) {
        beginFaceMove(FaceOp::Rotate);
        faceTool_.typedValue = "15";

    } else {
        beginFaceMove(FaceOp::Move);
        // Several values in turn, the way a drag arrives at one: the selection
        // has to survive every rebuild, not just the last.
        for (const char* v : {"1", "3", "5"}) {
            faceTool_.typedValue = v;
            updateFaceMove(false);
            while (faceTool_.preview.busy()) updateFaceMove(false);
            updateFaceMove(false);
            {
                const SceneObject* o2 = scene_.find(id);
                const std::vector<Index> sel = scene_.selectedFaces(id);
                const Vec3 c = sel.empty() ? Vec3{} : o2->body.faceCentroid(sel.front());
                const Vec3 n = sel.empty() ? Vec3{} : o2->body.faceNormal(sel.front());
                std::fprintf(stderr,
                             "[face-demo]   at %s mm: %zu selected, centre z %.2f, "
                             "normal z %.2f\n",
                             v, sel.size(), c.z, n.z);
            }
        }
        faceTool_.typedValue = faceDemo_ == 2 ? "-5" : "6";
    }
    updateFaceMove(false);
    while (faceTool_.preview.busy()) updateFaceMove(false);
    updateFaceMove(false);
    report(faceDemo_ == 3 ? "rotated" : (faceDemo_ == 2 ? "pulled in" : "pushed out"));
    if (faceDemo_ != 3) commitFaceMove();     // leave the rotate open, to be seen
}

// A fast, wandering drag, spread over real frames.
//
// In one loop it proves nothing: the builds never finish, so nothing is ever
// taken back and the frame loop never tessellates against them. What broke in
// practice needed both -- a preview being built on one thread while the shape
// it came from was being meshed on the other -- and that only happens frame by
// frame, most reliably as the drag passes back through its start.
void Application::stepFaceStress() {
    if (!faceStress_ || viewRect_.w <= 0 || scene_.objects().empty()) return;

    const ObjectId id = scene_.objects().front()->id;
    if (!faceStressDone_) {
        faceStressDone_ = true;
        camera_.yaw = 0.6f;
        camera_.pitch = 0.5f;
        camera_.distance = 95.0f;
        camera_.snapToGoal();
        scene_.select(id);

        const SceneObject* o = scene_.find(id);
        std::vector<FaceId> fs;
        o->body.allFaces(fs);
        FaceId top = fs.front();
        Real best = -1e30;
        for (FaceId f : fs) {
            const Real d = dot(o->body.faceNormal(f), Vec3{0, 0, 1});
            if (d > best) { best = d; top = f; }
        }
        scene_.clearElementSelection();
        scene_.selectElement({id, ElementKind::Face, top}, true);

        mouseOverride_ = Vec2{viewRect_.w * 0.5, viewRect_.h * 0.5};
        beginFaceMove(FaceOp::Extrude);
        if (!faceTool_.active) std::fprintf(stderr, "[stress] would not start\n");
        return;
    }

    if (!faceTool_.active) return;
    ++faceStressFrame_;

    // Straight through the start every other frame. At zero the body on screen
    // becomes the one the gesture began with and is tessellated, while the
    // build asked for on the frame before is very likely still running off the
    // same shape. That is the window, and hitting it by waving the mouse is a
    // matter of luck; this hits it on purpose.
    const int phase = faceStressFrame_ % 4;
    static const char* kPattern[4] = {"6", "0", "9", "0"};
    faceTool_.typedValue = kPattern[phase];

    if (std::fabs(faceTool_.value) < 0.5) ++faceStressCrossings_;
    if (!std::isfinite(faceTool_.value)) ++faceStressBad_;
    faceStressWorst_ = std::max(faceStressWorst_, std::fabs(faceTool_.value));

    if (faceStressFrame_ >= 400) {
        std::fprintf(stderr,
                     "[stress] %d frames: %d near the start, worst %.1f mm, %d not finite\n",
                     faceStressFrame_, faceStressCrossings_, faceStressWorst_, faceStressBad_);
        abortFaceMove();
        mouseOverride_ = Vec2{-1, -1};
        faceStress_ = false;
        std::fprintf(stderr, "[stress] survived\n");
    }
}

void Application::stepFilletLimitSearch() {
    FilletToolState::LimitSearch& s = filletTool_.search;
    if (!filletTool_.active || !s.active) return;

    if (s.floorPhase) {
        stepFilletFloorSearch();
        return;
    }

    // Collect whatever the last trial concluded. A crash counts as a refusal:
    // OpenCASCADE does not always decline politely, and a radius that ends its
    // process is one this gesture must not offer.
    if (s.trial.running() || s.trial.finished()) {
        if (!s.trial.poll()) return;                 // still working; not our turn
        if (s.trial.result() == Attempt::Ok) {
            s.good = s.pending;
            filletTool_.maxRadius = s.good;          // how far the arrow may go
            if (s.pending >= s.bad * 0.999) {
                // The top of the bracket built, so the estimate was low. The
                // shortest nearby edge is the bound that usually bites, but a
                // face can carry a small edge far from anything being rounded,
                // and that must not become the limit. Double upward instead,
                // as far as the one bound that is always true.
                if (s.bad < s.hardCeiling * 0.999) {
                    s.good = s.bad;
                    s.bad = std::min(s.bad * 2.0, s.hardCeiling);
                    s.testedTop = false;
                } else {
                    s.active = false;
                    return;
                }
            }
        } else {
            s.bad = s.pending;
        }
        --s.stepsLeft;
    }

    if (s.stepsLeft <= 0 || s.bad <= s.good * 1.001) {
        s.active = false;
        return;
    }

    // The top of the bracket first. On an ordinary part it builds -- half the
    // smallest dimension really is available -- and the whole search is one
    // trial rather than eight converging on a number already known.
    if (!s.testedTop) {
        s.testedTop = true;
        s.pending = s.bad;
    } else {
        s.pending = 0.5 * (s.good + s.bad);
    }
    startFilletTrial(s.pending);
}

void Application::updateFillet(bool snap, bool follow) {
    if (!filletTool_.active) return;
    SceneObject* obj = scene_.find(filletTool_.objectId);
    if (!obj) { abortFillet(); return; }

    // Whatever the kernel finished while the last few frames were drawn.
    {
        Body built;
        if (filletTool_.preview.take(built)) {
            obj->body = std::move(built);
            obj->refreshDerived();
            filletTool_.previewValid = true;
        }
    }

    // Reaching for the panel is not a change of mind about the radius.
    //
    // The dialog has buttons on it, and the way to a button is across the
    // screen: without this, moving to press OK pulled the fillet out to
    // whatever radius the pointer passed through on the way, and pressed OK on
    // that. The build already in flight still has to be collected, which is why
    // this returns here and not at the top.
    if (!follow) return;

    Real newR = filletTool_.baseRadius;
    if (!filletTool_.typedValue.empty()) {
        try {
            newR = std::max(Real(0.01), Real(std::stod(filletTool_.typedValue)));
        } catch (...) {}
    } else {
        // The radius is how far the cursor is from the edge being rounded.
        //
        // It used to be an accumulated screen delta times 0.04 -- a number with
        // no relation to the model, the zoom, or where the pointer actually was,
        // so the same drag gave a different radius at every zoom level and the
        // cursor told you nothing about the result. Measuring the gap to the
        // edge is the same gesture the profile fillet in the create tool uses:
        // sit on the edge for nothing, pull away for more, and the distance you
        // see is the radius you get.
        //
        // Measured on screen and converted to millimetres at the edge's own
        // depth, rather than by unprojecting onto a plane through the edge,
        // because that plane degenerates when you happen to be sighting along
        // the edge -- which is a normal thing to be doing.
        const Vec2 curMouse = mouseInViewport();
        const Mat4 model = obj->modelMatrix();
        const Body& m = filletTool_.meshBefore;

        // Along the axis fixed when the gesture began. Off to the side changes
        // nothing, and the line on screen says which way is more -- the whole
        // difference between aiming and discovering.
        if (filletTool_.axis.valid) {
            const Real step = DragAxis::stepFor(camera_, filletTool_.axis.origin,
                                                filletTool_.maxRadius);
            if (filletTool_.axis.facingCamera(camera_)) {
                newR = filletTool_.axis.valueAt(camera_, curMouse);
            } else {
                // The axis points near the eye, where a pixel of movement is
                // worth an unbounded amount. Distance from the anchor on screen
                // is cruder but steady.
                Vec2 anchorPx{};
                if (camera_.projectToPixel(filletTool_.axis.origin, anchorPx))
                    newR = filletTool_.axis.baseValue +
                           length(curMouse - anchorPx) *
                               camera_.pixelWorldSize(filletTool_.axis.origin);
            }

            if (snap && step > 0.0) newR = std::round(newR / step) * step;
            newR = std::max(newR, filletTool_.axis.baseValue);
        }
        (void)m;
        (void)model;
        newR = std::max(Real(0.05), newR);
    }

    // Held inside the travel found when the gesture began. Bisecting for the
    // limit here instead -- which is what this did -- made the preview flicker
    // between two answers as the cursor approached it, and the number jitter
    // with it. The limit belongs to the geometry, not to the pointer.
    if (filletTool_.maxRadius > 0.0) newR = std::min(newR, filletTool_.maxRadius);

    // The number and the guide are the pointer's own, so they move now. The
    // geometry is the kernel's and arrives when it arrives.
    filletTool_.currentRadius = newR;
    view_.bevelWidth = newR;

    // Nothing has been verified yet -- the floor trial is still in flight -- so
    // there is no radius it would be safe to hand a worker thread. A preview
    // built past the limit is the one that takes the process with it.
    if (filletTool_.maxRadius <= 0.0) return;

    // A drag that has not changed the answer has no work to ask for. With
    // snapping on the radius only moves when the cursor crosses a tick, so most
    // frames of a drag land here -- and the ones that do not now cost the price
    // of handing a request to another thread rather than a full rebuild of the
    // body, which on an eight-edge fillet is twenty to fifty milliseconds.
    if (std::fabs(newR - filletTool_.requestedRadius) < 1e-9 && filletTool_.previewValid) {
        return;
    }

    filletTool_.requestedRadius = newR;
    {
        const std::vector<Index> edges = filletTool_.edges;
        filletTool_.preview.request(filletTool_.meshBefore, [edges, newR](Body& b) {
            FilletSpec spec;
            for (Index e : edges) spec.edges.push_back({e, newR});
            return filletEdges(b, spec);
        });
    }
    // A refusal inside the travel means the segment count changed under the
    // limit that was measured for it. The last good preview stands rather than
    // the tool hunting for a new one mid-gesture.
}

void Application::commitFillet() {
    if (!filletTool_.active) return;

    // A click rather than a drag can land here before the floor trial has come
    // back, and there is nothing to apply until it has. This is the one moment
    // in the gesture where waiting is right: it is a few tens of milliseconds,
    // once, on a deliberate action, against committing a radius nothing has
    // checked.
    while (filletTool_.active && filletTool_.search.active &&
           filletTool_.search.floorPhase) {
        stepFilletFloorSearch();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (!filletTool_.active) return;   // the ladder ran out and aborted for us

    filletTool_.search.active = false;
    filletTool_.search.trial.abandon();
    filletTool_.preview.cancel();
    const ObjectId id = filletTool_.objectId;
    SceneObject* obj = scene_.find(id);
    justFinishedModal_ = true;
    filletTool_.active = false;

    if (!obj) return;

    // The handles were picked on the body as it stood before the preview
    // started rounding it, so that is the body they have to be read against.
    if (extendLastFillet(*obj, filletTool_.meshBefore, filletTool_.edges,
                         filletTool_.currentRadius)) {
        return;
    }

    Feature f;
    f.kind = FeatureKind::Bevel;
    f.edges = nameEdges(filletTool_.meshBefore, filletTool_.edges);
    f.radii.assign(f.edges.count(), filletTool_.currentRadius);
    f.width = filletTool_.currentRadius;

    std::vector<Feature> chainBefore = filletTool_.chainBefore;
    obj->features = filletTool_.chainBefore;
    std::string why;
    if (scene_.addFeature(id, std::move(f), &why) && editKeepsSolid(id)) {
        undo_.push(std::make_unique<FeatureCommand>(id, std::move(chainBefore),
                                                    obj->features, "Fillet"));
    } else {
        obj->features = std::move(chainBefore);
        obj->body = std::move(filletTool_.meshBefore);
        obj->refreshDerived();
        // An empty reason means the feature built and editKeepsSolid turned it
        // down, which is the one case where the mesh itself is the problem.
        setNotice(why.empty() ? "Fillet refused: it would make the model unprintable"
                              : "Fillet refused: " + why);
    }
}

void Application::abortFillet() {
    if (!filletTool_.active) return;
    // Whatever was being checked is no longer wanted, and the process doing it
    // must not outlive the gesture.
    filletTool_.search.active = false;
    filletTool_.search.trial.abandon();
    filletTool_.preview.cancel();
    const ObjectId id = filletTool_.objectId;
    justFinishedModal_ = true;
    filletTool_.active = false;

    SceneObject* obj = scene_.find(id);
    if (obj) {
        obj->features = std::move(filletTool_.chainBefore);
        obj->body = std::move(filletTool_.meshBefore);
        obj->refreshDerived();
    }
}

void Application::beginAddPrimitivePrompt(PrimitiveKind kind) {
    if (tool_.active() || filletTool_.active || createTool_.active()) return;
    createTool_.start(kind);
}

// Folds `edges`, picked on the current mesh, into the fillet at the end of the
// chain. Returns false if there is nothing to fold them into, if any of them
// cannot be traced back to an edge of the mesh that fillet saw, or if the
// merged fillet does not evaluate -- in which case the caller adds a new
// feature and the object is left exactly as it was.
bool Application::extendLastFillet(SceneObject& obj, const Body& picked,
                                   const std::vector<Index>& edges, Real radius) {
    if (obj.features.empty()) return false;
    const size_t last = obj.features.size() - 1;
    Feature& fillet = obj.features[last];
    if (fillet.kind != FeatureKind::Bevel || fillet.edges.empty() || !fillet.enabled)
        return false;
    if (fillet.segments != view_.bevelSegments) return false;

    // The mesh as it stood before that fillet ran, which is what its edge
    // indices are numbered against.
    if (last == 0 || last > obj.featureCache.size()) return false;
    const Body& before = obj.featureCache[last - 1];
    if (before.empty()) return false;

    // Only a list of edges can have one added to it. A rim selected as a
    // face's boundary already means "all of them", and adding to it would be
    // saying something different.
    if (fillet.edges.kind != ElementRefs::Kind::Explicit) return false;

    std::vector<Index> merged;
    if (!fillet.edges.resolveEdges(before, merged)) return false;

    std::vector<Real> radii;
    radii.reserve(merged.size());
    for (size_t i = 0; i < merged.size(); ++i) radii.push_back(fillet.radiusFor(i));

    for (Index e : edges) {
        // `picked`, not obj.body. By the time this runs the preview has already
        // replaced the object's body with a rounded one, where that same handle
        // is a different edge -- and reading the endpoints from there mapped
        // the fillet onto whichever edge had inherited the number. The preview
        // was right and the committed result was not, which is the worst way
        // for this to be wrong.
        if (!picked.hasEdge(e)) return false;
        Vec3 a, b;
        picked.edgePositions(e, a, b);
        const EdgeId mapped = edgeAlongSegment(before, a, b);
        if (mapped == kInvalid) return false;

        // Handles are canonical, so one comparison settles it -- there is no
        // longer a twin that could name the same edge.
        bool already = false;
        for (size_t i = 0; i < merged.size(); ++i)
            if (merged[i] == mapped) {
                radii[i] = radius;   // re-picking an edge restates its radius
                already = true;
            }
        if (!already) { merged.push_back(mapped); radii.push_back(radius); }
    }

    std::vector<Feature> chainBefore = obj.features;
    const ElementRefs edgesBefore = fillet.edges;
    const std::vector<Real> radiiBefore = fillet.radii;

    fillet.edges = nameEdges(before, merged);
    fillet.radii = std::move(radii);
    if (!scene_.reevaluateFrom(obj.id, last)) {
        fillet.edges = edgesBefore;
        fillet.radii = radiiBefore;
        scene_.reevaluateFrom(obj.id, last);
        return false;
    }
    if (obj.features[last].errored) {
        fillet.edges = edgesBefore;
        fillet.radii = radiiBefore;
        scene_.reevaluateFrom(obj.id, last);
        return false;
    }

    setNotice("Added to the fillet above");
    undo_.push(std::make_unique<FeatureCommand>(obj.id, std::move(chainBefore),
                                                obj.features, "Fillet"));
    return true;
}

void Application::applyBoolean(BooleanOp op) {
    const std::vector<ObjectId>& sel = scene_.selection();
    if (sel.size() != 2) {
        setNotice("Select two objects: Ctrl+click one, then Shift+Ctrl+click the other");
        return;
    }

    // First picked is the target, last picked is the tool. That ordering is
    // what makes difference mean what the user expects.
    const ObjectId targetId = sel.front();
    const ObjectId toolId = sel.back();
    SceneObject* target = scene_.find(targetId);
    SceneObject* tool = scene_.find(toolId);
    if (!target || !tool || targetId == toolId) return;

    // Bake the tool into the target's local space. The two objects have their
    // own transforms, and the boolean is defined on geometry, so they have to
    // be brought into one frame first.
    const Mat4 toLocal = inverse(target->modelMatrix()) * tool->modelMatrix();
    Body baked = tool->body;
    baked.transform(toLocal);

    Feature f;
    f.kind = FeatureKind::Boolean;
    f.booleanOp = op;
    f.bakedBody = std::move(baked);

    std::vector<Feature> chainBefore = target->features;
    if (!scene_.addFeature(targetId, std::move(f))) {
        setNotice(std::string(booleanOpName(op)) + " produced no valid solid");
        return;
    }

    // One undo entry covering both halves.
    std::vector<std::unique_ptr<Command>> parts;
    parts.push_back(std::make_unique<FeatureCommand>(
        targetId, std::move(chainBefore), target->features, booleanOpName(op)));
    renderer_.forget(toolId);
    parts.push_back(ExistenceCommand::forDelete(scene_, {toolId}));

    undo_.push(std::make_unique<CompositeCommand>(std::move(parts), booleanOpName(op)));
    scene_.select(targetId);
}

void Application::splitActiveObject() {
    const ObjectId id = scene_.contextObject();
    SceneObject* obj = scene_.find(id);
    if (!obj) {
        setNotice("Select an object to split");
        return;
    }

    // Check if a face is selected on the object to act as the splitting plane (Fusion 360 style)
    const std::vector<Index> selFaces = scene_.selectedFaces(id);
    if (!selFaces.empty()) {
        const Index f = selFaces.front();
        if (f < obj->body.faceCount()) {
            const Vec3 norm = obj->body.faceNormal(f);
            std::vector<VertexId> fv;
            obj->body.faceVertices(f, fv);
            const Vec3 pt = fv.empty() ? Vec3{0, 0, 0} : obj->body.vertexPosition(fv.front());

            Body piece1, piece2;
            if (splitByPlane(obj->body, pt, norm, piece1, piece2)) {
                std::vector<Feature> chainBefore = obj->features;

                std::vector<Feature> chain1;
                Feature base1;
                base1.kind = FeatureKind::BaseMesh;
                base1.bakedBody = std::move(piece1);
                chain1.push_back(std::move(base1));
                obj->features = std::move(chain1);
                scene_.reevaluate(id);

                const ObjectId copy = scene_.duplicateObject(id);
                if (copy != kNoObject) {
                    SceneObject* pieceObj = scene_.find(copy);
                    if (pieceObj) {
                        pieceObj->name = obj->name + " (Body 2)";
                        pieceObj->features.front().bakedBody = std::move(piece2);
                        scene_.reevaluate(copy);
                    }
                }

                std::vector<std::unique_ptr<Command>> parts;
                parts.push_back(std::make_unique<FeatureCommand>(
                    id, std::move(chainBefore), obj->features, "Split Body"));
                if (copy != kNoObject)
                    parts.push_back(ExistenceCommand::forCreate(scene_, {copy}));

                undo_.push(std::make_unique<CompositeCommand>(std::move(parts), "Split Body"));
                setNotice("Split " + obj->name + " into 2 bodies along face plane");
                return;
            }
        }
    }

    // If 2 objects are selected: Target body + Splitting plane / tool body
    if (scene_.selection().size() == 2) {
        const ObjectId targetId = scene_.selection().front();
        const ObjectId toolId = scene_.selection().back();
        SceneObject* targetObj = scene_.find(targetId);
        SceneObject* toolObj = scene_.find(toolId);
        if (targetObj && toolObj && targetId != toolId) {
            const Mat4 toTargetLocal = inverse(targetObj->modelMatrix()) * toolObj->modelMatrix();
            const Vec3 toolCenter = transformPoint(toTargetLocal, toolObj->body.bounds().center());
            const Vec3 toolNorm = normalize(transformVector(normalMatrix(toTargetLocal), Vec3{0, 0, 1}));

            Body piece1, piece2;
            if (splitByPlane(targetObj->body, toolCenter, toolNorm, piece1, piece2)) {
                std::vector<Feature> chainBefore = targetObj->features;

                std::vector<Feature> chain1;
                Feature base1;
                base1.kind = FeatureKind::BaseMesh;
                base1.bakedBody = std::move(piece1);
                chain1.push_back(std::move(base1));
                targetObj->features = std::move(chain1);
                scene_.reevaluate(targetId);

                const ObjectId copy = scene_.duplicateObject(targetId);
                if (copy != kNoObject) {
                    SceneObject* pieceObj = scene_.find(copy);
                    if (pieceObj) {
                        pieceObj->name = targetObj->name + " (Split 2)";
                        pieceObj->features.front().bakedBody = std::move(piece2);
                        scene_.reevaluate(copy);
                    }
                }

                std::vector<std::unique_ptr<Command>> parts;
                parts.push_back(std::make_unique<FeatureCommand>(
                    targetId, std::move(chainBefore), targetObj->features, "Split Body"));
                if (copy != kNoObject)
                    parts.push_back(ExistenceCommand::forCreate(scene_, {copy}));

                undo_.push(std::make_unique<CompositeCommand>(std::move(parts), "Split Body"));
                setNotice("Split " + targetObj->name + " into 2 bodies with tool plane");
                return;
            }
        }
    }

    // Check if body has disconnected shells
    std::vector<Body> bodies;
    if (splitBodies(obj->body, bodies) >= 2) {
        std::vector<Feature> chainBefore = obj->features;

        std::vector<Feature> chain;
        Feature base;
        base.kind = FeatureKind::BaseMesh;
        base.bakedBody = bodies.front();
        chain.push_back(std::move(base));
        obj->features = std::move(chain);
        scene_.reevaluate(id);

        std::vector<ObjectId> created;
        for (size_t i = 1; i < bodies.size(); ++i) {
            const ObjectId copy = scene_.duplicateObject(id);
            if (copy == kNoObject) continue;
            SceneObject* piece = scene_.find(copy);
            piece->features.front().bakedBody = bodies[i];
            scene_.reevaluate(copy);
            created.push_back(copy);
        }

        std::vector<std::unique_ptr<Command>> parts;
        parts.push_back(std::make_unique<FeatureCommand>(
            id, std::move(chainBefore), obj->features, "Split"));
        if (!created.empty())
            parts.push_back(ExistenceCommand::forCreate(scene_, created));

        undo_.push(std::make_unique<CompositeCommand>(std::move(parts), "Split"));
        setNotice("Split into " + std::to_string(bodies.size()) + " bodies");
        return;
    }

    // Single solid with no face selected: bisect through object center along XY plane
    const Vec3 center = obj->body.bounds().center();
    Body piece1, piece2;
    if (splitByPlane(obj->body, center, Vec3{0, 0, 1}, piece1, piece2)) {
        std::vector<Feature> chainBefore = obj->features;

        std::vector<Feature> chain1;
        Feature base1;
        base1.kind = FeatureKind::BaseMesh;
        base1.bakedBody = std::move(piece1);
        chain1.push_back(std::move(base1));
        obj->features = std::move(chain1);
        scene_.reevaluate(id);

        const ObjectId copy = scene_.duplicateObject(id);
        if (copy != kNoObject) {
            SceneObject* pieceObj = scene_.find(copy);
            if (pieceObj) {
                pieceObj->name = obj->name + " (Body 2)";
                pieceObj->features.front().bakedBody = std::move(piece2);
                scene_.reevaluate(copy);
            }
        }

        std::vector<std::unique_ptr<Command>> parts;
        parts.push_back(std::make_unique<FeatureCommand>(
            id, std::move(chainBefore), obj->features, "Split Body"));
        if (copy != kNoObject)
            parts.push_back(ExistenceCommand::forCreate(scene_, {copy}));

        undo_.push(std::make_unique<CompositeCommand>(std::move(parts), "Split Body"));
        setNotice("Split " + obj->name + " into 2 bodies at center");
        return;
    }

    setNotice("Split Body could not cut this object");
}

// Returns true when the caller may proceed immediately. Otherwise a prompt is
// raised and the action is replayed once the user answers.
bool Application::confirmDiscard(PendingAction next) {
    if (!dirty()) return true;
    pending_ = next;
    return false;
}

void Application::drawUnsavedPrompt() {
    if (pending_ == PendingAction::None) return;

    ImGui::OpenPopup("Unsaved Changes");
    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->GetCenter().x, vp->GetCenter().y),
                            ImGuiCond_Always, ImVec2(0.5f, 0.5f));

    if (ImGui::BeginPopupModal("Unsaved Changes", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("This project has unsaved changes.");
        ImGui::Spacing();

        const PendingAction next = pending_;
        auto finish = [&](bool proceed) {
            pending_ = PendingAction::None;
            ImGui::CloseCurrentPopup();
            if (!proceed) return;
            // Treat it as saved so the replayed action is not blocked again.
            savedRevision_ = undo_.revision();
            switch (next) {
                case PendingAction::New:  newProject(); break;
                case PendingAction::Open: beginFilePrompt(FileMode::Open); break;
                case PendingAction::Quit: running_ = false; break;
                case PendingAction::None: break;
            }
        };

        if (ImGui::Button("Save First", ImVec2(110, 0))) {
            pending_ = PendingAction::None;
            ImGui::CloseCurrentPopup();
            if (projectPath_.empty()) beginFilePrompt(FileMode::Save);
            else                      runFileOperation(FileMode::Save, projectPath_);
        }
        ImGui::SameLine();
        if (ImGui::Button("Discard", ImVec2(110, 0))) finish(true);
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(110, 0)) ||
            ImGui::IsKeyPressed(ImGuiKey_Escape, false)) finish(false);

        ImGui::EndPopup();
    }
}

void Application::newProject() {
    scene_.clear();
    undo_.clear();
    projectPath_.clear();
    savedRevision_ = undo_.revision();
    const ObjectId startup = scene_.addPrimitive(PrimitiveKind::Box);
    placeOnBuildPlate(startup);
    scene_.select(startup);
    camera_.frame(scene_.bounds());
    setNotice("New project");
}

void Application::beginFilePrompt(FileMode mode) {
    fileMode_ = mode;

    // Seed with something sensible: the current project, or a default name
    // beside it, so the common case is one keystroke.
    std::string seed = projectPath_;
    if (mode == FileMode::ExportStl) {
        if (seed.empty()) seed = "model.stl";
        else {
            const size_t dot = seed.find_last_of('.');
            seed = (dot == std::string::npos ? seed : seed.substr(0, dot)) + ".stl";
        }
    } else if (seed.empty()) {
        seed = "untitled.tangent";
    }
    std::snprintf(pathField_, sizeof(pathField_), "%s", seed.c_str());
}

void Application::runFileOperation(FileMode mode, const std::string& path) {
    if (path.empty()) return;

    switch (mode) {
    case FileMode::Save: {
        const ProjectResult r = saveProject(scene_, path);
        if (r.ok) {
            projectPath_ = path;
            savedRevision_ = undo_.revision();
            setNotice("Saved " + path);
        }
        else      setNotice("Save failed: " + r.error);
        break;
    }
    case FileMode::Open: {
        const ProjectResult r = loadProject(scene_, path);
        if (r.ok) {
            projectPath_ = path;
            // History from the previous project cannot apply to this one.
            undo_.clear();
            savedRevision_ = undo_.revision();
            camera_.frame(scene_.bounds());
            setNotice("Opened " + path + " (" + std::to_string(r.objects) + " objects)");
        } else {
            setNotice("Open failed: " + r.error);
        }
        break;
    }
    case FileMode::ExportStl: {
        StlOptions opt;
        opt.binary = exportBinaryStl_;
        opt.selectionOnly = exportSelectionOnly_;
        opt.deviationMm = exportDeviationMm_;
        const StlResult r = exportStl(scene_, path, opt);
        if (r.ok) {
            std::string note = "Exported " + std::to_string(r.triangles) + " triangles to " + path;
            // A mesh body cannot honour a tolerance -- its resolution was fixed
            // when it was made -- and saying so is better than letting the
            // number on the dialog imply otherwise.
            if (r.meshBodies > 0)
                note += "  (" + std::to_string(r.meshBodies) +
                        (r.meshBodies == 1 ? " mesh body written at its own resolution)"
                                           : " mesh bodies written at their own resolution)");
            setNotice(note);
        } else {
            setNotice("Export failed: " + r.error);
        }
        break;
    }
    case FileMode::None:
        break;
    }
}

void Application::drawFilePrompt() {
    if (fileMode_ == FileMode::None) return;

    const char* title = fileMode_ == FileMode::Open   ? "Open Project"
                      : fileMode_ == FileMode::Save   ? "Save Project"
                                                      : "Export STL";
    ImGui::OpenPopup(title);

    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->GetCenter().x, vp->GetCenter().y),
                            ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(460.0f, 0.0f));

    if (ImGui::BeginPopupModal(title, nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("Path");
        ImGui::SetNextItemWidth(-1.0f);
        // Focused on open, and Enter confirms, so the whole thing is keyboard
        // driven without reaching for the mouse.
        if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
        const bool entered = ImGui::InputText("##path", pathField_, sizeof(pathField_),
                                              ImGuiInputTextFlags_EnterReturnsTrue);

        if (fileMode_ == FileMode::ExportStl) {
            ImGui::Spacing();
            ImGui::Checkbox("Binary", &exportBinaryStl_);
            ImGui::SameLine();
            ImGui::Checkbox("Selection only", &exportSelectionOnly_);

            ImGui::SetNextItemWidth(140.0f);
            ImGui::DragFloat("Tolerance", &exportDeviationMm_, 0.001f, 0.001f, 0.5f,
                             "%.3f mm");
            ImGui::SameLine();
            ImGui::TextDisabled("how far a triangle may sit from the surface");
        }

        ImGui::Spacing();
        const bool confirm = ImGui::Button("OK", ImVec2(90, 0)) || entered;
        ImGui::SameLine();
        const bool cancel = ImGui::Button("Cancel", ImVec2(90, 0)) ||
                            ImGui::IsKeyPressed(ImGuiKey_Escape, false);

        if (confirm) {
            const FileMode mode = fileMode_;
            fileMode_ = FileMode::None;
            ImGui::CloseCurrentPopup();
            runFileOperation(mode, pathField_);
        } else if (cancel) {
            fileMode_ = FileMode::None;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

void Application::applyActions() {
    UiActions& a = ui_.actions;

    if (a.quit && confirmDiscard(PendingAction::Quit)) running_ = false;

    if (a.newProject && confirmDiscard(PendingAction::New))   newProject();
    if (a.openProject && confirmDiscard(PendingAction::Open)) beginFilePrompt(FileMode::Open);
    if (a.saveProjectAs) beginFilePrompt(FileMode::Save);
    if (a.exportStl)     beginFilePrompt(FileMode::ExportStl);
    if (a.saveProject) {
        // Save straight over the current file; prompt only the first time.
        if (projectPath_.empty()) beginFilePrompt(FileMode::Save);
        else                      runFileOperation(FileMode::Save, projectPath_);
    }

    if (a.undo) undo_.undo(scene_);
    if (a.redo) undo_.redo(scene_);

    if (a.addRequested) {
        beginAddPrimitivePrompt(a.addKind);
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
            // A radius typed or dragged in the timeline has never been tried,
            // and a fillet can take the process with it rather than refusing --
            // see geom/kernel_guard.h. So the edited chain is evaluated once
            // where a crash is survivable before it is evaluated for real.
            //
            // Only for chains that round something: everything else has no way
            // to fault, and a fork on every keystroke in the timeline would be
            // paid by edits that never needed it.
            bool rounds = false;
            for (const Feature& f : o->features)
                if (f.kind == FeatureKind::Bevel && f.enabled) rounds = true;

            if (rounds) {
                std::vector<Feature> trial = o->features;
                const Attempt attempt = tryInChild([&] {
                    Body out;
                    return evaluateFeatures(trial, out);
                });
                if (attempt == Attempt::Crashed) {
                    o->features = a.featuresBefore;
                    scene_.reevaluate(a.featuresEdited);
                    setNotice("That radius cannot be built on this shape");
                    a.featuresEdited = kNoObject;
                }
            }
        }
        if (a.featuresEdited != kNoObject && o) {
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

    if (a.pushPull)   beginFaceMove(FaceOp::Move);
    if (a.extrude)    beginFaceMove(FaceOp::Extrude);
    if (a.rotateFace) beginFaceMove(FaceOp::Rotate);
    if (a.divide) beginDivide();
    if (a.mergeFaces) mergeSelected();
    if (a.bevel)   bevelActiveObject();
    if (a.split)   splitActiveObject();
    if (a.fillet)  beginFillet();
    if (a.shell)   shellActiveObject();
    if (a.booleanRequested) applyBoolean(a.booleanOp);

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

    // The operations, above the panels and the viewport both.
    drawToolbar(ui_);

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
    // The cube carries the view's name and its projection, so the corner
    // readout that used to say both is gone: two places telling you the same
    // thing is one place too many, and the cube is where the eye already goes
    // to find out which way it is looking.
    drawViewCube(ui_, viewRect_.x, viewRect_.y, viewRect_.w, viewRect_.h);
    drawFilePrompt();
    drawUnsavedPrompt();
    drawMeasurePanel(ui_);
    drawMeasureLabel();
    drawTransformReadout();
    drawFilletPanel();
    drawFacePanel();
    drawDividePanel();
    if (createTool_.active()) {
        // Under the toolbar, in the corner of the viewport opposite the view
        // cube: a dialog over the middle of the model is a dialog in the way of
        // the thing being made.
        createTool_.setHudOrigin(viewRect_.x + 16.0f, viewRect_.y + 16.0f);
        bool finished = false;
        createTool_.drawHud(scene_, camera_, undo_, finished);
        if (finished) justFinishedModal_ = true;
    }

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
        // A cursor for a machine with nobody at it. Hover states are half of
        // what a widget is, and a screenshot of one taken with the pointer
        // parked at the origin shows the other half.
        //
        // Queued as events before NewFrame, not written into io afterwards:
        // the click, the release and how long the button was held are all
        // derived during NewFrame, so a MouseDown set after it is a button
        // that is down but was never pressed, and nothing fires.
        if (uiMouse_.x >= 0.0f) {
            ImGuiIO& in = ImGui::GetIO();
            in.AddMousePosEvent(uiMouse_.x, uiMouse_.y);
            // Pressed on one frame and released on the next, because a button
            // that fires on release never fires for a pointer never lifted.
            in.AddMouseButtonEvent(0, uiMouseDown_ && frame >= 2 && frame < 4);
        }

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
                    ctxObj->health = ctxObj->body.health();
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

        if (faceTool_.active) {
            char buf[160];
            std::snprintf(buf, sizeof buf,
                          faceTool_.op == FaceOp::Rotate
                              ? "Rotate face  %.1f deg   type a number   Click confirm   Esc cancel"
                              : "Push / pull  %.2f mm   A auto  J join  D cut   type a number   Click confirm   Esc cancel",
                          faceTool_.value);
            ui_.toolStatus = buf;
        } else if (divideTool_.active) {
            char buf[128];
            std::snprintf(buf, sizeof buf,
                          "Divide  %.2f mm along the edge   type a number   Click confirm   Esc cancel",
                          divideTool_.t * length(divideTool_.dir));
            ui_.toolStatus = buf;
        } else if (filletTool_.active) {
            char buf[128];
            std::snprintf(buf, sizeof(buf), "Fillet  %.2f mm   type a number   Click confirm   Esc cancel",
                          filletTool_.currentRadius);
            ui_.toolStatus = buf;
        } else if (createTool_.active()) {
            if (createTool_.stage() == CreateStage::SelectPlane) {
                ui_.toolStatus = "Select Plane: Click origin tile or object face [1: XZ, 3: YZ, 7: XY] (Esc cancel)";
            } else if (createTool_.stage() == CreateStage::DrawProfile_Pt1) {
                ui_.toolStatus = "Step 1: Click first point/center on plane (Esc cancel)";
            } else if (createTool_.stage() == CreateStage::DrawProfile_Pt2) {
                ui_.toolStatus = "Step 2: Move mouse to set dimensions, click to confirm (Esc cancel)";
            } else if (createTool_.stage() == CreateStage::AdjustProfile) {
                ui_.toolStatus = "Adjust: Drag edge handles or corner fillet handles, press Enter/OK to extrude (Esc cancel)";
            } else if (createTool_.stage() == CreateStage::ExtrudeDepth) {
                ui_.toolStatus =
                    createTool_.hasTargetBody()
                        ? std::string("Depth: move to set, ") + createOpName(createTool_.resolvedOp()) +
                          "   A auto  J join  D cut  N new body   Ctrl free (snap on)   click finish   Esc cancel"
                        : "Depth: move to set, click to finish   Ctrl free (snap on)   Esc cancel";
            }
            // What the cursor has caught replaces the step's own prompt: it is
            // the more specific thing to say, and a snap nobody is told about
            // is indistinguishable from the tool being imprecise -- or from a
            // snap to the wrong thing.
            if (const PlaneSnap& hit = createTool_.activeSnap(); hit.valid()) {
                std::string what = describeSnap(hit);
                if (hit.radius > 0.0) {
                    char buf[48];
                    std::snprintf(buf, sizeof buf, "  (\u00D8 %.3f mm)", hit.radius * 2.0);
                    what += buf;
                }
                ui_.toolStatus = what + "   Ctrl for free placement";
            }
        } else if (tool_.active()) {
            ui_.toolStatus = tool_.statusText();
        } else {
            ui_.toolStatus.clear();
        }

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
            ui_.stats.vertices  += static_cast<size_t>(o->body.vertexCount());
        }

        buildUi();
        handleViewportMouse();
        handleShortcuts();

        // The create tool can refuse from any of the three above -- the HUD's
        // Finish button, a click in the viewport, or the E shortcut -- so it is
        // drained once here rather than at each of them.
        if (std::string createErr = createTool_.takeError(); !createErr.empty())
            setNotice(createErr);

        applyActions();

        // A feature that dropped out of the chain during any of the above.
        // Drained here, after the actions have run, because a re-evaluation is
        // triggered from a dozen places -- an inspector nudge, a history
        // toggle, an undo -- and none of them should have to remember to say so.
        if (std::string chainErr = scene_.takeChainNotice(); !chainErr.empty())
            setNotice(chainErr);

        // Queued before the frame is drawn; the renderer flushes overlay lines
        // at the end of its pass.
        drawSelectionHighlights();
        if (createTool_.active()) createTool_.drawOverlay(scene_, camera_, renderer_);
        measureResult_ = measure_.active() ? measure_.compute(scene_) : MeasureResult{};
        measure_.drawOverlay(renderer_, camera_, measureResult_);
        tool_.drawOverlay(renderer_, camera_);

        // The line that says which way makes the fillet bigger, with ticks at
        // the step so the cost of a step is visible. Drawn after the preview so
        // it sits on top of the geometry it is about.
        if (filletTool_.active && filletTool_.axis.valid) {
            const SceneObject* o = scene_.find(filletTool_.objectId);
            const Real step = o ? DragAxis::stepFor(camera_, filletTool_.axis.origin,
                                                   filletTool_.maxRadius)
                                : 0.0;
            filletTool_.axis.drawGuide(renderer_, camera_, filletTool_.currentRadius, step,
                                       filletTool_.maxRadius);
        }

        // The same arrow for the face tools, since the gesture is the same one:
        // pull along a line and watch the number.
        if (faceTool_.active && faceTool_.axis.valid) {
            const Real step = faceTool_.op == FaceOp::Rotate
                                  ? 5.0
                                  : DragAxis::stepFor(camera_, faceTool_.axis.origin,
                                                      faceTool_.axis.spanValue);
            faceTool_.axis.drawGuide(renderer_, camera_, faceTool_.value, step, 0.0);
        }
        if (divideTool_.active && divideTool_.axis.valid) {
            const Real step = DragAxis::stepFor(camera_, divideTool_.axis.origin,
                                                divideTool_.axis.spanValue);
            divideTool_.axis.drawGuide(renderer_, camera_,
                                       divideTool_.t * length(divideTool_.dir), step,
                                       divideTool_.axis.spanValue);
        }

        camera_.update(dt);

        // One trial of the fillet limit search, if one is due. It runs in
        // another process, so this is a poll and a fork rather than a wait.
        stepFilletLimitSearch();

        // Bring a body or two up to the tolerance this view wants. Bounded per
        // frame on purpose: re-tessellating is tens of milliseconds on a heavy
        // part, so doing every body that wants it at once would turn a smooth
        // zoom into a series of stalls. See render/lod.h.
        refreshTessellation(scene_, camera_);

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
