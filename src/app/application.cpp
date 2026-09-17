#include "app/application.h"

#include "mesh/export_3mf.h"
#include "mesh/import_mesh.h"
#include "geom/kernel_guard.h"
#include "scene/trials.h"
#include "render/lod.h"
#include "ui/command_panel.h"
#include "app/printability.h"
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
#include <filesystem>
#include <unordered_set>
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

    if (filletEdgesDemo_ && !scene_.objects().empty()) {
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
        filletSelectedEdges();
        scene_.clearElementSelection();
        scene_.select(id);
    }

    if (roundAllDemo_ && !scene_.objects().empty()) {
        const ObjectId id = scene_.objects().front()->id;
        scene_.select(id);
        const Real before = scene_.find(id)->body.health(false).volume;
        roundAllEdges();
        const bool opened = filletTool_.active;
        const size_t picked = filletTool_.edges.size();
        if (opened) {
            for (int i = 0; i < 400 && filletTool_.search.active; ++i) {
                stepFilletLimitSearch();
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            filletTool_.typedValue = "2";
            updateFillet(false);
            while (filletTool_.preview.busy()) updateFillet(false);
            updateFillet(false);
            commitFillet();
        }
        const SceneObject* r = scene_.find(id);
        const bool rounded = r->features.size() == 2 && r->features[1].kind == FeatureKind::Bevel &&
                             !r->features[1].errored && r->features[1].edges.count() == picked;
        std::fprintf(stderr, "[round-all] opened=%d edges=%zu rounded=%d volume %.1f -> %.1f solid=%d\n",
                     (int)opened, picked, (int)rounded, before, r->body.health(false).volume,
                     (int)r->body.health(false).solid());

        Mesh cube;
        makeBox(cube);
        const ObjectId meshId = scene_.addBody(Body(std::move(cube)), {40, 0, 10}, "Mesh cube");
        scene_.select(meshId);
        notice_.clear();
        roundAllEdges();
        std::fprintf(stderr, "[round-all] on a mesh: opened=%d notice \"%s\"\n",
                     (int)filletTool_.active, notice_.c_str());
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

    if (sketchDemo_ > 0) {
        // A plate with a bore, drawn the way a person would: a rectangle, a
        // circle in the middle of it, and the circle's radius picked to edit.
        scene_.clear();
        camera_.yaw = 0.7f;
        camera_.pitch = 0.6f;
        camera_.distance = 150.0f;
        camera_.target = {0, 0, 0};
        camera_.snapToGoal();

        sketchTool_.start();
        sketchTool_.choosePlane(PlaneChoice::XY, camera_);
        sketchTool_.setMode(SketchMode::Rectangle);
        sketchTool_.clickAt({-30, -20});
        sketchTool_.clickAt({30, 20});
        sketchTool_.setMode(SketchMode::Circle);
        sketchTool_.clickAt({0, 0});
        sketchTool_.clickAt({10, 0});
        sketchTool_.setMode(SketchMode::Line);
        sketchTool_.clickAt({-20, 25});
        sketchTool_.clickAt({20, 25.3});
        sketchTool_.clearPending();
        sketchTool_.setMode(SketchMode::Dimension);
        for (const SketchEntity& e : sketchTool_.sketch().entities)
            if (e.curve == SketchCurve::Circle) sketchTool_.dimensionEntity(e.id);
        camera_.snapToGoal();

        if (sketchDemo_ >= 2) {
            // The plate, which is the region with the bore as its hole.
            if (sketchTool_.beginExtrude(&camera_)) {
                for (const SketchProfile& r : sketchTool_.regions())
                    if (r.holes.size() == 1) sketchTool_.toggleRegion(r.key);
            }
            camera_.snapToGoal();
        }
        if (sketchDemo_ >= 3) {
            sketchTool_.beginDepth();
            sketchTool_.typeKey('1');
            sketchTool_.typeKey('5');
        }
        if (sketchDemo_ >= 4) {
            sketchTool_.finish(scene_, camera_, undo_, true);
            if (!scene_.objects().empty()) {
                const SceneObject* o = scene_.objects().front().get();
                std::fprintf(stderr, "[sketch-demo] %s: %zu features, %d faces, %.3f mm3\n",
                             o->name.c_str(), o->features.size(), o->body.faceCount(),
                             o->body.health(false).volume);
                sketchTool_.startEdit(scene_, o->id, o->features.front().uid, camera_);
            }
            camera_.snapToGoal();
        }
        if (std::string e = sketchTool_.takeError(); !e.empty())
            std::fprintf(stderr, "[sketch-demo] %s\n", e.c_str());
        std::fprintf(stderr, "[sketch-demo] stage %d, %zu entities, %zu regions, %d free\n",
                     static_cast<int>(sketchTool_.stage()), sketchTool_.sketch().entities.size(),
                     sketchTool_.regions().size(), sketchTool_.solveState().freedoms);
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
    if (!headlessExport3mf_.empty()) {
        runFileOperation(FileMode::Export3mf, headlessExport3mf_);
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

bool Application::pointerDrives() const {
    return mouseOverride_.x >= 0.0 || ImGui::IsMousePosValid();
}

Vec2 Application::mouseInViewport() const {
    // The demos drive gestures with no mouse attached, and a gesture that
    // measures nothing measures nothing useful. Only ever set by them.
    if (mouseOverride_.x >= 0.0) return mouseOverride_;
    const ImVec2 m = ImGui::GetIO().MousePos;
    return {m.x - viewRect_.x, m.y - viewRect_.y};
}

bool Application::editToolActive() const {
    return filletTool_.active || faceTool_.active || divideTool_.active || patternTool_.active ||
           reduceTool_.active;
}

bool Application::refuseMeshEdit(const SceneObject& obj, const char* what) {
    if (!obj.body.isMesh()) return false;
    // Said once, here, so every command that edits part of a shape tells a
    // person the same thing -- and the thing that gets them unstuck.
    setNotice(std::string(what) + (brep::available()
        ? " needs a solid, and this is a mesh: Modify > Convert to Solid first"
        : " needs the exact kernel, which this build does not have"));
    return true;
}

void Application::beginTransform(TransformMode mode) {
    // Not over another operation. Keys are held by whichever tool is running,
    // but the menu's Move, Rotate and Scale reached here regardless and started
    // a transform on top of it.
    if (editToolActive()) { setNotice("Finish the current operation first"); return; }
    dismissSettled();
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
    stepPatternDemo();
    stepStepDemo();
    stepDialogDemo();
    stepMeshBench();
    stepReduceDemo();
    stepFaceStress();
    stepPrintDemo();
    stepPreviewCheck();

    // Sketching. The same arrangement as the create tool below: the wheel still
    // zooms, and the pointer on the dialog leaves the drawing alone.
    if (sketchTool_.active()) {
        if (io.MouseWheel != 0.0f && overViewport && !io.WantCaptureMouse)
            camera_.dolly(io.MouseWheel);
        if (!io.WantCaptureMouse) {
            sketchTool_.update(scene_, camera_, mouseInViewport(), !io.KeyCtrl);
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && overViewport) {
                sketchTool_.handleMouseDown(scene_, camera_, undo_);
                if (!sketchTool_.active()) justFinishedModal_ = true;
            } else if (ImGui::IsMouseClicked(ImGuiMouseButton_Right) && overViewport) {
                sketchTool_.handleRightClick(camera_);
                if (!sketchTool_.active()) justFinishedModal_ = true;
            }
        }
        return;
    }

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
    if (reduceTool_.active) {
        // Nothing follows the pointer, so a click in the viewport confirms
        // nothing; the right button cancels, as it does everywhere else.
        updateReduce();
        if (!io.WantCaptureMouse && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) abortReduce();
        return;
    }
    if (patternTool_.active) {
        updatePattern(!io.KeyCtrl, /*follow=*/!io.WantCaptureMouse);
        if (!io.WantCaptureMouse) {
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))       commitPattern();
            else if (ImGui::IsMouseClicked(ImGuiMouseButton_Right))  abortPattern();
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
    // Clicking in the viewport is the start of the next thing, whatever it
    // turns out to be. The panel of the last operation has had its chance.
    dismissSettled();

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
    const bool settled = settledIs(Settled::Fillet);
    if (!filletTool_.active && !settled) return;

    auto signature = [&] {
        char b[96];
        std::snprintf(b, sizeof b, "%d|%.9g|%.9g", (int)filletTool_.chamfer,
                      filletTool_.currentRadius, filletTool_.endRadius);
        return std::string(b);
    };
    const std::string was = settled ? signature() : std::string();

    if (!ui::beginCommand("##fillet", filletTool_.chamfer ? "Chamfer" : "Fillet",
                          filletTool_.chamfer ? Icon::Chamfer : Icon::Fillet,
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

    // A flat cut or a round, and whether the round holds its size along the
    // edge. Both change what gets built, so both re-plan and re-preview.
    ui::commandRow("Cut");
    {
        const float ic = ImGui::GetTextLineHeight() * 1.4f;
        if (iconButton(Icon::Fillet, "asround", ic, "Round  (R)", !filletTool_.chamfer) &&
            filletTool_.chamfer) {
            filletTool_.chamfer = false;
            filletTool_.requestedRadius = -1.0;
            filletTool_.previewValid = false;
        }
        ImGui::SameLine();
        if (iconButton(Icon::Chamfer, "asflat", ic, "Flat  (C)", filletTool_.chamfer) &&
            !filletTool_.chamfer) {
            filletTool_.chamfer = true;
            filletTool_.endRadius = -1.0;         // a flat cut does not taper
            filletTool_.requestedRadius = -1.0;
            filletTool_.previewValid = false;
        }
    }

    if (!filletTool_.chamfer) {
        const bool tapering = filletTool_.endRadius > 0.0;
        ui::commandRow("Taper");
        if (ImGui::Button(tapering ? "Even" : "Taper to...")) {
            filletTool_.endRadius = tapering ? -1.0
                                             : std::max(filletTool_.currentRadius * 0.25, 0.1);
            filletTool_.requestedRadius = -1.0;
            filletTool_.previewValid = false;
        }
        if (tapering) {
            ui::commandRow("Ends at");
            ImGui::SetNextItemWidth(-1.0f);
            if (ImGui::DragScalar("##end", ImGuiDataType_Double, &filletTool_.endRadius,
                                  0.05f, nullptr, nullptr, "%.2f mm")) {
                filletTool_.endRadius = std::max(filletTool_.endRadius, Real(0.05));
                filletTool_.requestedRadius = -1.0;
                filletTool_.previewValid = false;
            }
        }
    }

    if (settled) ui::commandApplied(filletTool_.chamfer ? "Chamfer" : "Fillet");
    ui::commandHint(filletTool_.chamfer
        ? "Pull along the arrow, or type a distance. The cut is the same from both faces."
        : "Pull along the arrow, or type a radius.");

    const int footer = settled ? ui::commandFooter("Done", true, nullptr)
                               : ui::commandFooter("OK  (Click)");
    ui::endCommand();

    if (settled) {
        if (footer > 0)             dismissSettled();
        else if (signature() != was) recommitSettled();
        return;
    }
    if (footer > 0)      commitFillet();
    else if (footer < 0) abortFillet();
}

void Application::drawFacePanel() {
    const bool settled = settledIs(Settled::Face);
    if (!faceTool_.active && !settled) return;

    auto signature = [&] {
        char b[96];
        std::snprintf(b, sizeof b, "%d|%.9g|%.9g|%.9g|%.9g", (int)faceTool_.op,
                      faceTool_.value, faceTool_.direction.x,
                      faceTool_.direction.y, faceTool_.direction.z);
        return std::string(b);
    };
    const std::string was = settled ? signature() : std::string();

    const bool rotate  = faceTool_.op == FaceOp::Rotate;
    const bool scale   = faceTool_.op == FaceOp::Scale;
    const bool extrude = faceTool_.op == FaceOp::Extrude;

    if (!ui::beginCommand("##faceop",
                          rotate ? "Rotate Face" : scale ? "Scale Face"
                                 : extrude ? "Extrude" : "Move Face",
                          rotate ? Icon::Chamfer : scale ? Icon::Cone : Icon::Extrude,
                          viewRect_.x + 16.0f, viewRect_.y + 16.0f))
        return;

    if (ui::commandNumber(rotate ? "Angle" : scale ? "Change" : "Distance",
                          faceTool_.value, rotate ? "deg" : scale ? "%" : "mm",
                          !faceTool_.typedValue.empty(),
                          !faceTool_.typedValue.empty(), faceTool_.typedValue.c_str()))
        faceTool_.typedValue.clear();

    // The multiple the percentage comes to, since that is the number a person
    // thinks in when they say "half again as big".
    if (scale) {
        char mult[32];
        std::snprintf(mult, sizeof mult, "%.3g x", 1.0 + faceTool_.value / 100.0);
        ui::commandValue("Size", mult);
    }

    char sel[64];
    std::snprintf(sel, sizeof sel, "%zu face%s", faceTool_.faces.size(),
                  faceTool_.faces.size() == 1 ? "" : "s");
    ui::commandValue("Selection", sel);

    // Which way it goes. The face's own normal unless an axis key says
    // otherwise -- for a rotate, which way round it turns. A scale goes every
    // way at once, so there is nothing to point.
    if (!scale) {
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
    }

    // What the number is doing to the body, said plainly. Not a choice: moving
    // a face out adds material and moving it in takes some away, and offering
    // to override that would be offering to make the tool lie.
    if (!rotate && !scale) {
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

    ui::commandHint(scale
        ? "Pull out from the middle of the face to grow it, in to shrink it. The "
          "faces around it slant to follow."
        : rotate
        ? "Pull either way across the pivot, or type an angle. X / Y / Z choose "
          "which way it turns."
        : extrude
        ? "Grows a boss off the face and leaves its outline, so you can take "
          "hold of it afterwards. Negative cuts in."
        : "Moves the face; the body follows. X / Y / Z move it along a world "
          "axis instead of its own.");

    if (settled)
        ui::commandApplied(rotate ? "Rotation" : scale ? "Scale"
                                  : extrude ? "Extrude" : "Move");

    const int footer = settled ? ui::commandFooter("Done", true, nullptr)
                               : ui::commandFooter("OK  (Click)");
    ui::endCommand();
    if (settled) {
        if (footer > 0)             dismissSettled();
        else if (signature() != was) recommitSettled();
        return;
    }
    if (footer > 0)      commitFaceMove();
    else if (footer < 0) abortFaceMove();
}

void Application::drawPatternPanel() {
    const bool settled = settledIs(Settled::Pattern);
    if (!patternTool_.active && !settled) return;

    // What the panel is showing, so that a control moved while the operation is
    // already applied re-applies it. Comparing one signature across the frame
    // beats hanging a call off every widget: a control added later is covered
    // by having been drawn, not by being remembered.
    auto signature = [&] {
        char b[96];
        std::snprintf(b, sizeof b, "%d|%d|%d|%d|%.9g", (int)patternTool_.mode,
                      patternTool_.count, patternTool_.axisIndex,
                      (int)patternTool_.useTool, patternTool_.dragged());
        return std::string(b);
    };
    const std::string was = settled ? signature() : std::string();

    const bool mirror = patternTool_.mode == PatternMode::Mirror;
    if (!ui::beginCommand("##pattern", mirror ? "Mirror" : "Pattern",
                          mirror ? Icon::Difference : Icon::Intersection,
                          viewRect_.x + 16.0f, viewRect_.y + 16.0f))
        return;

    // How the copies are laid out. Three answers to one question, so three
    // buttons rather than a dropdown.
    ui::commandRow("Layout");
    {
        // Words rather than icons: none of the baked pictures depicts a row or
        // a ring, and a picture that has to be explained by its tooltip is a
        // worse label than the word it was standing in for.
        struct Choice { const char* label; PatternMode mode; const char* tip; };
        static const Choice kChoices[3] = {
            {"Row",    PatternMode::Linear,   "Along a direction  (L)"},
            {"Ring",   PatternMode::Circular, "Around an axis  (C)"},
            {"Mirror", PatternMode::Mirror,   "Reflected across a plane  (M)"},
        };
        const float w = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x * 2.0f) / 3.0f;
        for (int i = 0; i < 3; ++i) {
            if (i) ImGui::SameLine();
            const bool on = patternTool_.mode == kChoices[i].mode;
            if (on) ImGui::PushStyleColor(ImGuiCol_Button,
                                          ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
            if (ImGui::Button(kChoices[i].label, ImVec2(w, 0)) && !on) {
                if (kChoices[i].mode == PatternMode::Circular) patternTool_.axisIndex = 2;
                setPatternMode(kChoices[i].mode);
            }
            if (on) ImGui::PopStyleColor();
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", kChoices[i].tip);
        }
    }

    // The axis, or the plane's normal. Same three keys the transform tools use.
    ui::commandRow(mirror ? "Plane" : "Axis");
    {
        static const char* kAxis[3] = {"X", "Y", "Z"};
        for (int i = 0; i < 3; ++i) {
            if (i) ImGui::SameLine();
            const bool on = patternTool_.axisIndex == i;
            if (on) ImGui::PushStyleColor(ImGuiCol_Button,
                                          ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
            if (ImGui::Button(kAxis[i], ImVec2(ImGui::GetTextLineHeight() * 1.8f, 0)) && !on) {
                patternTool_.axisIndex = i;
                setPatternMode(patternTool_.mode);
            }
            if (on) ImGui::PopStyleColor();
        }
    }

    if (mirror) {
        if (ui::commandNumber("Plane at", patternTool_.offset, "mm",
                              !patternTool_.typedValue.empty(),
                              !patternTool_.typedValue.empty(),
                              patternTool_.typedValue.c_str()))
            patternTool_.typedValue.clear();
    } else {
        if (ui::commandNumber(patternTool_.mode == PatternMode::Circular ? "Turn" : "Spacing",
                              patternTool_.dragged(),
                              patternTool_.mode == PatternMode::Circular ? "deg" : "mm",
                              !patternTool_.typedValue.empty(),
                              !patternTool_.typedValue.empty(),
                              patternTool_.typedValue.c_str()))
            patternTool_.typedValue.clear();

        ui::commandRow("Copies");
        // A full turn is what a ring pattern is nearly always for, and working
        // out 360 over the count by hand is not modelling. It shares the row,
        // so the count field leaves room for it.
        const bool ring = patternTool_.mode == PatternMode::Circular;
        const float turnW = ring ? ImGui::CalcTextSize("Full turn").x +
                                       ImGui::GetStyle().FramePadding.x * 2.0f +
                                       ImGui::GetStyle().ItemSpacing.x
                                 : 0.0f;
        ImGui::SetNextItemWidth(-1.0f - turnW);
        int n = patternTool_.count;
        if (ImGui::DragInt("##count", &n, 0.1f, 2, 256, "%d")) {
            patternTool_.count = n < 2 ? 2 : (n > 256 ? 256 : n);
            patternTool_.previewValid = false;
        }
        if (ring) {
            ImGui::SameLine();
            if (ImGui::Button("Full turn")) {
                patternTool_.stepAngle = radians(360.0 / std::max(2, patternTool_.count));
                patternTool_.typedValue.clear();
                patternTool_.previewValid = false;   // the next frame rebuilds
            }
        }
    }

    // What is being repeated. A boolean at the end of the chain leaves a tool
    // behind that can be repeated instead of the whole body, and repeating that
    // is what a person means by "pattern this hole".
    if (patternTool_.toolAvailable) {
        ui::commandRow("Repeat");
        if (ImGui::Button(patternTool_.useTool ? "The cut" : "The body")) {
            patternTool_.useTool = !patternTool_.useTool;
            patternTool_.previewValid = false;
            // A plane that was right for one of these is a no-op for the other.
            if (mirror) setPatternMode(PatternMode::Mirror);
        }
    }

    if (settled) ui::commandApplied(mirror ? "Mirror" : "Pattern");
    ui::commandHint(mirror
        ? (patternTool_.useTool
               ? "The last cut is reflected across the plane and made again."
               : "The body is reflected across the plane, and the two halves fuse.")
        : (patternTool_.useTool
               ? "The last cut is repeated. Its first copy is where it already is."
               : "The body is repeated, and the copies fuse where they meet."));

    const int footer = settled ? ui::commandFooter("Done", true, nullptr)
                               : ui::commandFooter("OK  (Click)");
    ui::endCommand();
    if (settled) {
        if (footer > 0)            dismissSettled();
        else if (signature() != was) recommitSettled();
        return;
    }
    if (footer > 0)      commitPattern();
    else if (footer < 0) abortPattern();
}

void Application::drawDividePanel() {
    const bool settled = settledIs(Settled::Divide);
    if (!divideTool_.active && !settled) return;

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

    if (settled) ui::commandApplied("Divide");
    ui::commandHint("The cut runs square across the edge you chose and slides "
                    "along it. The body stays whole.");

    const int footer = settled ? ui::commandFooter("Done", true, nullptr)
                               : ui::commandFooter("OK  (Click)");
    ui::endCommand();
    if (settled) {
        if (footer > 0) dismissSettled();
        return;
    }
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

// Tints the faces a printer will struggle with.
//
// Under the model rather than over it, and quiet: this is a standing report on
// the whole part, not a thing the user is doing right now, and it must not
// compete with a selection or a preview. Only thin walls are drawn: a slicer
// drops a wall it cannot lay and the part comes off the bed with a hole in it,
// where supports are the slicer's own decision and it makes them regardless.
Application::PrintJobResult Application::runPrintCheck(const Body& body, const RenderMesh& rm,
                                                       const PrintProfile& profile) {
    PrintJobResult out;
    out.report = checkPrintability(body, rm, profile);

    std::unordered_set<FaceId> flagged;
    for (const PrintFinding& bad : out.report.findings)
        if (bad.face != kInvalid && bad.issue == PrintIssue::ThinWall) flagged.insert(bad.face);
    if (flagged.empty()) return out;

    // One pass over the triangles, however many faces were flagged.
    for (size_t i = 0; i < rm.triangleFace.size(); ++i) {
        if (!flagged.count(rm.triangleFace[i])) continue;
        for (int k = 0; k < 3; ++k) out.triangles.push_back(rm.positions[rm.triangles[i * 3 + k]]);
    }
    return out;
}

void Application::retirePrintJob(ObjectId id) {
    auto it = printJobs_.find(id);
    if (it == printJobs_.end()) return;
    if (it->second.result.valid()) retiredPrintJobs_.push_back(std::move(it->second.result));
    printJobs_.erase(it);
}

void Application::refreshPrintCheck(SceneObject& o, const PrintProfile& profile) {
    PrintJobResult r = runPrintCheck(o.body, o.render, profile);
    o.printCheck = std::move(r.report);
    o.printTriangles = std::move(r.triangles);
    o.printVersion = o.meshVersion;
    retirePrintJob(o.id);            // anything still running is now stale
}

void Application::drawPrintIssues() {
    // Let go of stale jobs that have finished. Never waits on one that has not.
    retiredPrintJobs_.erase(
        std::remove_if(retiredPrintJobs_.begin(), retiredPrintJobs_.end(),
                       [](std::future<PrintJobResult>& f) {
                           return f.wait_for(std::chrono::seconds(0)) ==
                                  std::future_status::ready;
                       }),
        retiredPrintJobs_.end());

    if (!view_.showPrintIssues) return;
    if (filletTool_.active || faceTool_.active || divideTool_.active ||
        patternTool_.active || reduceTool_.active || createTool_.active() || sketchTool_.active() || tool_.active())
        return;                       // a gesture owns the model while it runs

    const Vec4 thin{0.95f, 0.30f, 0.22f, 0.34f};

    for (const auto& obj : scene_.objects()) {
        SceneObject* o = obj.get();
        if (!o->visible || o->body.empty()) continue;

        // Worked out once per change of geometry and kept, together with the
        // triangles it flagged -- so a frame costs the flagged triangles and
        // nothing else, however large the body behind them is.
        if (o->printVersion != o->meshVersion) {
            auto it = printJobs_.find(o->id);
            const bool current = it != printJobs_.end() && it->second.meshVersion == o->meshVersion;

            if (current && it->second.result.wait_for(std::chrono::seconds(0)) ==
                               std::future_status::ready) {
                PrintJobResult r = it->second.result.get();
                o->printCheck = std::move(r.report);
                o->printTriangles = std::move(r.triangles);
                o->printVersion = o->meshVersion;
                printJobs_.erase(it);
            } else if (!current) {
                // A snapshot, detached: an exact body's triangulation is written
                // into the shape it belongs to, so a worker reading the live one
                // while the frame thread tessellates it is a data race. A stale
                // job for an older version is simply replaced; its future is
                // left to finish and be thrown away.
                if (it != printJobs_.end()) retirePrintJob(o->id);
                PrintJob job;
                job.meshVersion = o->meshVersion;
                job.result = std::async(std::launch::async,
                                        [body = o->body.detached(), rm = o->render]() {
                                            return runPrintCheck(body, rm, PrintProfile{});
                                        });
                printJobs_.emplace(o->id, std::move(job));
            }
            // Until the answer arrives, draw the last one only if it was for this
            // geometry -- which it was not, or we would not be here.
            continue;
        }
        if (o->printTriangles.empty()) continue;

        const Mat4 model = o->modelMatrix();
        const std::vector<Vec3>& t = o->printTriangles;
        for (size_t i = 0; i + 2 < t.size(); i += 3)
            renderer_.addTriangle(transformPoint(model, t[i]), transformPoint(model, t[i + 1]),
                                  transformPoint(model, t[i + 2]), thin);
    }
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

    // A panel that is only waiting to be dismissed. Escape and Enter both put
    // it away and keep what it made: there is nothing left to cancel here, and
    // an Escape that quietly undid a finished operation would be a trap.
    // Ctrl+Z is still the way to take it back, and it dismisses too.
    if (settled_ != Settled::None) {
        if (ImGui::IsKeyPressed(ImGuiKey_Escape, false) ||
            ImGui::IsKeyPressed(ImGuiKey_Enter, false) ||
            ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false)) {
            dismissSettled();
            return;
        }
    }

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

    if (reduceTool_.active) {
        if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) { abortReduce(); return; }
        if (ImGui::IsKeyPressed(ImGuiKey_Enter, false) ||
            ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false)) { commitReduce(); return; }
        typedInto(reduceTool_.typedValue, [&] {
            try {
                const Real v = std::stod(reduceTool_.typedValue);
                if (v > 0 && v != reduceTool_.tolerance) {
                    reduceTool_.tolerance = v;
                    requestReducePreview();
                }
            } catch (...) {}
        });
        return;
    }

    if (patternTool_.active) {
        if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) { abortPattern(); return; }
        if (ImGui::IsKeyPressed(ImGuiKey_Enter, false) ||
            ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false)) { commitPattern(); return; }
        // The axis, and the mode, without reaching for the panel.
        int wantAxis = -1;
        if (ImGui::IsKeyPressed(ImGuiKey_X, false)) wantAxis = 0;
        if (ImGui::IsKeyPressed(ImGuiKey_Y, false)) wantAxis = 1;
        if (ImGui::IsKeyPressed(ImGuiKey_Z, false)) wantAxis = 2;
        if (wantAxis >= 0 && wantAxis != patternTool_.axisIndex) {
            patternTool_.axisIndex = wantAxis;
            setPatternMode(patternTool_.mode);
            return;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_L, false) &&
            patternTool_.mode != PatternMode::Linear) {
            setPatternMode(PatternMode::Linear); return;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_C, false) &&
            patternTool_.mode != PatternMode::Circular) {
            patternTool_.axisIndex = 2;
            setPatternMode(PatternMode::Circular); return;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_M, false) &&
            patternTool_.mode != PatternMode::Mirror) {
            setPatternMode(PatternMode::Mirror); return;
        }
        typedInto(patternTool_.typedValue, [&] { updatePattern(true); });
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
        // Round or flat, without reaching for the panel.
        if (ImGui::IsKeyPressed(ImGuiKey_C, false) || ImGui::IsKeyPressed(ImGuiKey_R, false)) {
            const bool wantFlat = ImGui::IsKeyPressed(ImGuiKey_C, false);
            if (filletTool_.chamfer != wantFlat) {
                filletTool_.chamfer = wantFlat;
                if (wantFlat) filletTool_.endRadius = -1.0;
                filletTool_.requestedRadius = -1.0;
                filletTool_.previewValid = false;
                // The largest that will build is a different number for a flat
                // cut than for a round, so it is found again.
                filletTool_.search.active = true;
                filletTool_.search.floorPhase = true;
                filletTool_.search.floorIndex = 0;
                filletTool_.search.testedTop = false;
                filletTool_.search.trial.abandon();
                filletTool_.maxRadius = 0.0;
            }
            return;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Enter, false) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false)) {
            commitFillet();
            return;
        }
        typedInto(filletTool_.typedValue, [&] { updateFillet(true); });
        return;
    }

    if (sketchTool_.active()) {
        auto send = [&](int key) {
            return sketchTool_.handleKey(key, io.KeyShift, io.KeyCtrl, scene_, camera_, undo_);
        };
        if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) { send(27); return; }
        if (ImGui::IsKeyPressed(ImGuiKey_Enter, false) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter, false)) {
            send(13);
            return;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Delete, false)) { send(127); return; }
        for (const auto& [imKey, ch] : {
                 std::pair{ImGuiKey_L, 'L'}, std::pair{ImGuiKey_R, 'R'}, std::pair{ImGuiKey_C, 'C'},
                 std::pair{ImGuiKey_A, 'A'}, std::pair{ImGuiKey_D, 'D'}, std::pair{ImGuiKey_Q, 'Q'},
                 std::pair{ImGuiKey_X, 'X'}, std::pair{ImGuiKey_E, 'E'}, std::pair{ImGuiKey_J, 'J'},
                 std::pair{ImGuiKey_N, 'N'}, std::pair{ImGuiKey_Z, 'Z'}}) {
            if (ImGui::IsKeyPressed(imKey, false)) { send(ch); return; }
        }
        for (int d = 0; d <= 9; ++d) {
            if (ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_0 + d), false) ||
                ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_Keypad0 + d), false)) {
                send('0' + d);
                return;
            }
        }
        for (const auto& [imKey, ch] : {std::pair{ImGuiKey_Period, '.'},
                                        std::pair{ImGuiKey_KeypadDecimal, '.'},
                                        std::pair{ImGuiKey_Minus, '-'},
                                        std::pair{ImGuiKey_KeypadSubtract, '-'},
                                        std::pair{ImGuiKey_Backspace, static_cast<char>(8)},
                                        std::pair{ImGuiKey_Tab, static_cast<char>(9)}}) {
            if (ImGui::IsKeyPressed(imKey, false)) { send(ch); return; }
        }
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
        if (ImGui::IsKeyPressed(ImGuiKey_S, false) && !shift) {
            if (onFace) beginFaceMove(FaceOp::Scale);
            else        beginTransform(TransformMode::Scale);
        }
    }

    // Sketch. Shift+S, beside Shift+A for the shapes it stands in for.
    if (shift && !ctrl && !alt && ImGui::IsKeyPressed(ImGuiKey_S, false)) ui_.actions.sketch = true;

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
    if (!ctrl && !alt && !shift && ImGui::IsKeyPressed(ImGuiKey_P, false))
        ui_.actions.pattern = true;
    if (!ctrl && !alt && !shift && ImGui::IsKeyPressed(ImGuiKey_M, false))
        ui_.actions.mirror = true;
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
        if (ImGui::IsKeyPressed(ImGuiKey_E, false)) ui_.actions.export3mf = true;
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
}

// Every edge of the body, through the same gesture as a picked edge. That
// gesture is what finds how large a round the part can take, and it does so in
// another process -- a round the size of a wall can take OpenCASCADE down with
// it, and committing a width straight from the menu gave it that chance.
void Application::roundAllEdges() {
    // Checked before the selection is touched: beginFillet would decline too,
    // but only after this had replaced what the user had picked.
    if (tool_.active() || createTool_.active() || sketchTool_.active() || editToolActive()) return;
    const ObjectId id = scene_.contextObject();
    SceneObject* obj = scene_.find(id);
    if (!obj || obj->body.empty()) { setNotice("Select an object to round"); return; }
    if (refuseMeshEdit(*obj, "Rounding edges")) return;

    std::vector<EdgeId> all;
    obj->body.allEdges(all);
    if (all.empty()) { setNotice("That body has no edges to round"); return; }
    scene_.select(id);
    scene_.clearElementSelection();
    for (EdgeId e : all) scene_.selectElement({id, ElementKind::Edge, e}, true);
    beginFillet();
}

// ---------------------------------------------------------------------------
// Reducing a mesh.

void Application::beginReduce() {
    dismissSettled();
    if (tool_.active() || filletTool_.active || createTool_.active() || sketchTool_.active() || faceTool_.active ||
        divideTool_.active || patternTool_.active || reduceTool_.active)
        return;
    const ObjectId id = scene_.contextObject();
    SceneObject* obj = scene_.find(id);
    if (!obj) { setNotice("Select a mesh to reduce"); return; }
    if (!obj->body.isMesh()) {
        setNotice("That body is already exact: reducing is for meshes");
        return;
    }

    reduceTool_.reset();
    reduceTool_.active = true;
    reduceTool_.objectId = id;
    reduceTool_.before = obj->body;
    reduceTool_.chainBefore = obj->features;
    reduceTool_.uid = scene_.takeFeatureUid();

    // A starting tolerance in proportion to the part: a thousandth of its size,
    // rounded to a figure a person would type. A 60mm part starts at 0.05mm.
    const Real size = obj->localBounds.valid() ? length(obj->localBounds.size()) : 50.0;
    const Real steps[] = {0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0};
    Real start = steps[0];
    for (Real st : steps)
        if (st <= size * 0.001) start = st;
    reduceTool_.tolerance = start;
    reduceTool_.target = 0;
    requestReducePreview();
}

void Application::requestReducePreview() {
    auto job = std::make_shared<ReduceJob>();
    job->tolerance = reduceTool_.tolerance;
    job->target = reduceTool_.target;
    job->loosen = reduceTool_.loosen;

    // Whatever is building is now for parameters nobody wants; tell it to stop.
    if (reduceTool_.running) reduceTool_.running->cancel = true;
    const bool building = reduceTool_.preview.busy();

    const ElementId uid = reduceTool_.uid;
    reduceTool_.preview.request(reduceTool_.before, [job, uid](Body& b) {
        ReduceOptions options;
        options.toleranceMm = job->tolerance;
        options.targetTriangles = job->target > 0 ? static_cast<size_t>(job->target) : 0;
        options.loosenToReachTarget = job->loosen;
        options.cancel = &job->cancel;
        if (!reduceBody(b, options, uid, job->result)) return false;
        job->solidFaces = predictSolidFaces(b);
        return true;
    });
    if (building) reduceTool_.pending = job;
    else          reduceTool_.running = job;
}

void Application::updateReduce() {
    if (!reduceTool_.active) return;
    SceneObject* obj = scene_.find(reduceTool_.objectId);
    if (!obj) { abortReduce(); return; }

    Body built;
    bool failed = false;
    const bool finished = reduceTool_.preview.take(built, &failed);
    if (finished || failed) {
        const std::shared_ptr<ReduceJob> done = reduceTool_.running;
        // take() starts whatever was waiting, which is now the one running.
        reduceTool_.running = reduceTool_.pending;
        reduceTool_.pending.reset();
        if (finished && done) {
            reduceTool_.shown = done;
            reduceTool_.shownBody = built;
            obj->body = std::move(built);
            obj->refreshDerived();
        } else if (failed && done && !done->cancel) {
            reduceTool_.shown = done;             // a real refusal: say why
        }
    }
}

void Application::commitReduce() {
    if (!reduceTool_.active || !reduceTool_.ready()) return;
    const ObjectId id = reduceTool_.objectId;
    SceneObject* obj = scene_.find(id);
    if (!obj) { abortReduce(); return; }

    Feature f;
    f.kind = FeatureKind::Reduce;
    f.uid = reduceTool_.uid;
    f.reduceTolerance = reduceTool_.tolerance;
    f.reduceTarget = reduceTool_.target;
    f.reduceLoosen = reduceTool_.loosen;

    const ReduceResult r = reduceTool_.shown->result;
    std::vector<Feature> chainBefore = reduceTool_.chainBefore;
    obj->features = reduceTool_.chainBefore;
    obj->body = reduceTool_.before;
    // The preview already built exactly this, from this body, with this uid.
    scene_.addFeatureWithResult(id, std::move(f), reduceTool_.shownBody);
    undo_.push(std::make_unique<FeatureCommand>(id, std::move(chainBefore), obj->features,
                                                "Reduce Mesh"));

    char buf[200];
    std::snprintf(buf, sizeof buf, "Reduced %zu triangles to %zu, within %.3g mm (measured %.3g)",
                  r.trianglesBefore, r.trianglesAfter, static_cast<double>(reduceTool_.tolerance),
                  static_cast<double>(r.deviationMm));
    setNotice(buf);
    justFinishedModal_ = true;
    reduceTool_.active = false;
    reduceTool_.reset();
}

void Application::abortReduce() {
    if (!reduceTool_.active) return;
    if (SceneObject* obj = scene_.find(reduceTool_.objectId)) {
        obj->features = reduceTool_.chainBefore;
        obj->body = reduceTool_.before;
        obj->refreshDerived();
    }
    justFinishedModal_ = true;
    reduceTool_.active = false;
    reduceTool_.reset();
}

void Application::drawReducePanel() {
    if (!reduceTool_.active) return;
    if (!ui::beginCommand("##reduce", "Reduce Mesh", Icon::Count, viewRect_.x + 16.0f,
                          viewRect_.y + 16.0f))
        return;

    const Real tolBefore = reduceTool_.tolerance;
    const int targetBefore = reduceTool_.target;
    const bool loosenBefore = reduceTool_.loosen;

    if (ui::commandNumber("Within", reduceTool_.tolerance, "mm", !reduceTool_.typedValue.empty(),
                          !reduceTool_.typedValue.empty(), reduceTool_.typedValue.c_str()))
        reduceTool_.typedValue.clear();

    // The tolerances people actually use, one click each.
    ui::commandRow("");
    {
        // Wrapped to the panel rather than run off its edge: the last one used
        // to be cut away where the panel ended.
        const Real presets[] = {0.01, 0.02, 0.05, 0.1, 0.25};
        const float column = ImGui::GetCursorPosX();
        const float right = ImGui::GetWindowContentRegionMax().x;
        for (int i = 0; i < 5; ++i) {
            char label[16];
            std::snprintf(label, sizeof label, "%g", presets[i]);
            if (i) {
                const ImGuiStyle& st = ImGui::GetStyle();
                const float need = ImGui::CalcTextSize(label).x + st.FramePadding.x * 2;
                ImGui::SameLine();
                if (ImGui::GetCursorPosX() + need > right) {
                    ImGui::NewLine();
                    ImGui::SetCursorPosX(column);
                }
            }
            const bool on = std::fabs(reduceTool_.tolerance - presets[i]) < 1e-12;
            if (on) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
            if (ImGui::SmallButton(label)) {
                reduceTool_.tolerance = presets[i];
                reduceTool_.typedValue.clear();
            }
            if (on) ImGui::PopStyleColor();
        }
    }

    ui::commandRow("Stop at");
    {
        // Few enough triangles to convert, however far the tolerance has to
        // loosen to get there -- which is only as far as the last few collapses
        // need. What it came to is shown under "Moved".
        const bool fit = reduceTool_.target == kSolidifyFaceLimit && reduceTool_.loosen;
        if (ImGui::Button(fit ? "Fit for conversion  (on)" : "Fit for conversion")) {
            reduceTool_.target = fit ? 0 : kSolidifyFaceLimit;
            reduceTool_.loosen = !fit;
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Reduce until there are few enough triangles to convert to a solid.\n"
                              "The tolerance is loosened only as far as that needs.");
    }

    // What it gives.
    const ReduceJob* shown = reduceTool_.shown.get();
    const bool stale = !reduceTool_.ready();
    char text[160];
    if (reduceTool_.preview.busy()) {
        ui::commandValue("Triangles", "reducing...");
    } else if (shown && !shown->result.ok) {
        ui::commandValue("Triangles", shown->result.error.c_str());
    } else if (shown) {
        std::snprintf(text, sizeof text, "%zu  ->  %zu", shown->result.trianglesBefore,
                      shown->result.trianglesAfter);
        ui::commandValue("Triangles", text);
    }
    if (shown && shown->result.ok) {
        std::snprintf(text, sizeof text, "%.3g mm, measured%s", static_cast<double>(shown->result.deviationMm),
                      shown->result.withinTolerance ? "" : " -- over");
        ui::commandValue("Moved", text);
        if (shown->result.toleranceUsedMm > shown->tolerance * 1.0001) {
            std::snprintf(text, sizeof text, "loosened to %.3g mm", static_cast<double>(shown->result.toleranceUsedMm));
            ui::commandValue("To fit", text);
        }
        if (shown->solidFaces <= kSolidifyFaceLimit)
            std::snprintf(text, sizeof text, "%d faces: will convert", shown->solidFaces);
        else
            std::snprintf(text, sizeof text, "%d faces: too many to convert", shown->solidFaces);
        ui::commandValue("As a solid", text);
    }

    ui::commandHint("Flat faces reduce to almost nothing; curved ones as far as the tolerance "
                    "allows. Edges, corners and holes stay within the tolerance, and the result "
                    "is measured before it is shown.");

    const int footer = ui::commandFooter(stale ? "OK  (wait)" : "OK  (Enter)", !stale);
    ui::endCommand();

    if (reduceTool_.tolerance != tolBefore || reduceTool_.target != targetBefore ||
        reduceTool_.loosen != loosenBefore)
        requestReducePreview();
    if (footer > 0)      commitReduce();
    else if (footer < 0) abortReduce();
}

void Application::convertSelectedToSolid() {
    dismissSettled();
    const ObjectId id = scene_.contextObject();
    SceneObject* obj = scene_.find(id);
    if (!obj) { setNotice("Select a mesh to convert"); return; }
    if (!obj->body.isMesh()) { setNotice("That body is already exact"); return; }

    Body converted = obj->body;
    const SolidifyResult r = toSolid(converted, scene_.nextImportSalt());
    if (!r.ok) {
        setNotice(r.error.empty() ? "It could not be made solid" : "Not converted: " + r.error);
        return;
    }

    // The chain becomes its root alone, holding the solid. What is converted
    // is the body as it now stands -- after a reduction, typically, which is
    // what makes a large mesh convertible at all -- so the steps that got it
    // there are in the solid already. Kept after it they would run again on the
    // exact body, where every one of them refuses: they are operations on
    // triangles, and there are no triangles any more.
    std::vector<Feature> chainBefore = obj->features;
    if (obj->features.empty() || obj->features.front().kind != FeatureKind::BaseMesh) {
        setNotice("That body did not come from an import");
        return;
    }
    std::vector<Feature> next(1, obj->features.front());
    next.front().bakedBody = converted;
    next.front().backend = Backend::Brep;
    const size_t baked = obj->features.size() - 1;

    std::string why;
    if (!scene_.setFeatures(id, std::move(next), &why)) {
        setNotice(why.empty() ? "It could not be made solid"
                              : "Not converted: " + why);
        return;
    }
    undo_.push(std::make_unique<FeatureCommand>(id, std::move(chainBefore),
                                                obj->features, "Convert to Solid"));

    char buf[240];
    std::snprintf(buf, sizeof buf,
                  "Converted: %d triangles became %d faces%s%s", r.facesBefore,
                  r.facesAfter,
                  r.facesAfter < r.facesBefore / 2
                      ? "" : "  (little merged: the mesh may not have been CAD)",
                  baked > 0 ? "  -- the steps before it are part of the solid now" : "");
    setNotice(buf);
}

void Application::insetSelectedFaces() {
    dismissSettled();
    const ObjectId target = scene_.contextObject();
    SceneObject* obj = scene_.find(target);
    if (!obj) { setNotice("Select an object first"); return; }
    if (refuseMeshEdit(*obj, "Insetting a face")) return;

    const std::vector<FaceId> faces = scene_.selectedFaces(target);
    if (faces.empty()) { setNotice("Select a face to inset"); return; }

    Feature f;
    f.kind = FeatureKind::Inset;
    f.amount = view_.insetAmount;
    f.faces = nameFaces(obj->body, faces);

    std::vector<Feature> chainBefore = obj->features;
    std::string why;
    if (!scene_.addFeature(target, f, &why)) {
        setNotice(why.empty() ? "the inset could not be built" : "Inset: " + why);
        return;
    }
    scene_.clearElementSelection();
    undo_.push(std::make_unique<FeatureCommand>(target, std::move(chainBefore),
                                                obj->features, "Inset"));
}

void Application::shellActiveObject() {
    const ObjectId target = scene_.contextObject();
    SceneObject* obj = scene_.find(target);
    if (!obj) return;
    if (refuseMeshEdit(*obj, "Shelling")) return;

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
    dismissSettled();
    if (tool_.active() || filletTool_.active || createTool_.active() || sketchTool_.active() ||
        faceTool_.active || divideTool_.active || patternTool_.active ||
        reduceTool_.active)
        return;

    const ObjectId id = scene_.contextObject();
    SceneObject* obj = scene_.find(id);
    if (!obj) { setNotice("Select an object first"); return; }
    {
        const char* what = op == FaceOp::Rotate  ? "Rotating a face"
                         : op == FaceOp::Scale   ? "Scaling a face"
                         : op == FaceOp::Extrude ? "Extruding a face"
                                                 : "Moving a face";
        if (refuseMeshEdit(*obj, what)) return;
    }

    const std::vector<Index> faces = scene_.selectedFaces(id);
    if (faces.empty()) { setNotice("Select a face to move"); return; }

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

    if (op == FaceOp::Scale) {
        // Out from the middle of the face, in its own plane: the way its
        // boundary travels as it grows.
        Vec3 across = anchor - transformPoint(model, obj->body.faceCentroid(faces.front()));
        across = across - dir * dot(across, dir);
        if (lengthSq(across) < 1e-9) {
            // Standing on the centre, so any direction in the plane will do.
            across = cross(dir, camera_.up());
            if (lengthSq(across) < 1e-9) across = cross(dir, camera_.right());
        }
        if (lengthSq(across) < 1e-12) { faceTool_.active = false; return; }
        faceTool_.axis.origin = anchor;
        faceTool_.axis.direction = normalize(across);
        faceTool_.axis.valid = true;
        faceTool_.axis.signedRange = true;
        faceTool_.axis.baseValue = 0.0;
        faceTool_.axis.spanValue = 100.0;     // per cent for a track's travel
        faceTool_.reachedMin = -95.0;         // a face cannot shrink to nothing
    } else if (op == FaceOp::Rotate) {
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
        if (pointerDrives() && faceTool_.axis.facingCamera(camera_)) {
            want = faceTool_.axis.valueAt(camera_, cur);
        }
        if (snap) {
            const Real step = faceTool_.op == FaceOp::Rotate ? 5.0
                            : faceTool_.op == FaceOp::Scale  ? 5.0
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
    if (faceTool_.op == FaceOp::Scale) {
        const Real factor = 1.0 + want / 100.0;
        faceTool_.preview.request(faceTool_.before, [faces, factor](Body& b) {
            return scaleFaces(b, faces, factor, 7004);
        });
    } else if (faceTool_.op == FaceOp::Rotate) {
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
    if (faceTool_.op == FaceOp::Scale) {
        f.kind = FeatureKind::FaceScale;
        f.scale = 1.0 + faceTool_.value / 100.0;
    } else if (faceTool_.op == FaceOp::Rotate) {
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
                      : faceTool_.op == FaceOp::Scale     ? "Scale Face"
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

        if (parts.size() == 1) {
            undo_.push(std::move(parts.front()), /*merge=*/recommitting_);
            settleCommand(Settled::Face, id);
        } else {
            // The move ran into another body and absorbed it. That other body
            // is gone, so there is nothing left to adjust this against and the
            // panel does not stay.
            undo_.push(std::make_unique<CompositeCommand>(std::move(parts), label));
        }
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
    if (refuseMeshEdit(*obj, "Merging faces")) return;

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

// A panel that has been applied and is waiting to be dismissed.
//
// The tool's own state is left exactly as it was, which is what lets the panel
// keep drawing its values and lets an adjustment re-run the same commit from
// the same starting chain.
void Application::settleCommand(Settled kind, ObjectId id) {
    ++settleSerial_;
    if (!recommitting_) {                   // a fresh operation, not a redo of one
        settled_ = kind;
        settledObject_ = id;
    }
    if (const SceneObject* o = scene_.find(id)) settledAfter_ = o->features;
}

void Application::dismissSettled() {
    if (settled_ == Settled::None) return;
    settled_ = Settled::None;
    settledObject_ = kNoObject;
    settledAfter_.clear();
    // The next edit is its own undo step, not more of this one.
    undo_.breakMergeChain();
    filletTool_.preview.cancel();
    divideTool_.preview.cancel();
    faceTool_.preview.cancel();
    patternTool_.preview.cancel();
}

void Application::recommitSettled() {
    if (settled_ == Settled::None) return;
    const Settled kind = settled_;
    const ObjectId id = settledObject_;

    const size_t applied = settleSerial_;
    recommitting_ = true;
    switch (kind) {
        case Settled::Fillet:  filletTool_.active = true;  commitFillet();   break;
        case Settled::Divide:  divideTool_.active = true;  commitDivide();   break;
        case Settled::Face:    faceTool_.active = true;    commitFaceMove(); break;
        case Settled::Pattern: patternTool_.active = true; commitPattern();  break;
        case Settled::None:    break;
    }
    recommitting_ = false;

    SceneObject* obj = scene_.find(id);
    if (!obj) { dismissSettled(); return; }

    // An adjustment the kernel refused leaves the chain where the commit put it
    // back -- before the operation entirely. The undo entry still says the
    // operation happened, so the model goes back to agreeing with it, and the
    // notice the commit already set says why the new value was not taken.
    if (settleSerial_ == applied) {
        obj->features = settledAfter_;
        scene_.reevaluate(id);
    }
}

// ---------------------------------------------------------------------------
// Pattern and mirror.

void Application::beginPattern(PatternMode mode) {
    dismissSettled();
    if (tool_.active() || filletTool_.active || createTool_.active() || sketchTool_.active() ||
        faceTool_.active || divideTool_.active || patternTool_.active ||
        reduceTool_.active)
        return;

    const ObjectId id = scene_.contextObject();
    SceneObject* obj = scene_.find(id);
    if (!obj) { setNotice("Select an object first"); return; }
    if (refuseMeshEdit(*obj, "Patterning")) return;

    patternTool_.reset();
    patternTool_.active = true;
    patternTool_.objectId = id;
    patternTool_.mode = mode;
    patternTool_.before = obj->body;
    patternTool_.chainBefore = obj->features;
    preEditSolid_ = obj->healthVersion == obj->meshVersion && obj->health.solid();

    // What the chain ends with decides what there is to repeat. A boolean left
    // a tool behind -- the cylinder that made the hole, the block that made the
    // boss -- and repeating that is what turns one hole into a bolt circle.
    // Anything else, and the only thing there is to repeat is the body.
    for (size_t i = obj->features.size(); i-- > 0;) {
        const Feature& f = obj->features[i];
        if (!f.enabled) continue;
        if (f.kind == FeatureKind::Boolean && !f.bakedBody.empty() && !f.errored) {
            patternTool_.tool = f.bakedBody;
            patternTool_.op = f.booleanOp;
            patternTool_.toolAvailable = true;
            patternTool_.useTool = true;
            patternTool_.replacing = i;     // the pattern stands where it stood
        }
        break;                              // only the last step, not a search
    }

    // Where the copies go from. A pattern of the body walks off its own
    // bounding box; a pattern of a tool turns about the body's centre, which is
    // where a bolt circle's axis nearly always is.
    const AABB b = obj->localBounds;
    patternTool_.origin = b.valid() ? b.center() : Vec3{};
    patternTool_.count = 4;
    patternTool_.axisIndex = mode == PatternMode::Circular ? 2 : 0;
    patternTool_.step = b.valid() ? std::max<Real>(1.0, b.size().x) : 10.0;
    patternTool_.stepAngle = radians(360.0 / 4.0);
    patternTool_.offset = 0.0;
    setPatternMode(mode);
}

// Switching mode rebuilds the drag track, because the three modes do not drag
// the same quantity: a distance, an angle, and where a plane sits.
void Application::setPatternMode(PatternMode mode) {
    SceneObject* obj = scene_.find(patternTool_.objectId);
    if (!obj) return;
    patternTool_.mode = mode;
    patternTool_.previewValid = false;
    patternTool_.requested = 1e30;
    patternTool_.typedValue.clear();
    if (mode == PatternMode::Circular && patternTool_.stepAngle == 0.0)
        patternTool_.stepAngle = radians(360.0 / std::max(2, patternTool_.count));

    const Mat4 model = obj->modelMatrix();
    const Vec3 dir{patternTool_.axisIndex == 0 ? 1.0 : 0.0,
                   patternTool_.axisIndex == 1 ? 1.0 : 0.0,
                   patternTool_.axisIndex == 2 ? 1.0 : 0.0};
    const AABB b = obj->localBounds;

    // Where a mirror's plane starts. Repeating a tool, the body's own centre is
    // right: the cut is off to one side of it and the reflection lands on the
    // other. Repeating the body, that same plane is the body's symmetry plane
    // and reflecting across it gives back exactly what was there -- so the
    // plane starts at the near face instead, where a mirror doubles the part.
    // Which is what someone who modelled half of something is asking for.
    if (mode == PatternMode::Mirror) {
        const Vec3 lo = b.valid() ? b.min : Vec3{};
        patternTool_.offset = patternTool_.useTool
                                  ? 0.0
                                  : (patternTool_.axisIndex == 0 ? lo.x
                                     : patternTool_.axisIndex == 1 ? lo.y : lo.z);
    }
    const Vec3 from = mode == PatternMode::Mirror ? dir * patternTool_.offset
                                                  : patternTool_.origin;

    patternTool_.axis.origin = transformPoint(model, from);
    patternTool_.axis.direction = normalize(transformVector(model, dir));
    patternTool_.axis.valid = true;
    patternTool_.axis.baseValue = mode == PatternMode::Mirror ? patternTool_.offset : 0.0;
    // An angle drags on a track scaled so a body-width of travel is a full
    // turn; a distance and a plane offset drag in millimetres, either way.
    const Real span = b.valid() ? std::max<Real>(1.0, length(b.size())) : 20.0;
    patternTool_.axis.spanValue = mode == PatternMode::Circular ? 360.0 : span;
    patternTool_.axis.signedRange = mode != PatternMode::Linear;
}

void Application::updatePattern(bool snap, bool follow) {
    if (!patternTool_.active) return;
    SceneObject* obj = scene_.find(patternTool_.objectId);
    if (!obj) { abortPattern(); return; }

    {
        Body built;
        if (patternTool_.preview.take(built)) {
            obj->body = std::move(built);
            obj->refreshDerived();
            patternTool_.previewValid = true;
        }
    }
    // Reaching for the panel is not a change of mind about the spacing -- the
    // way to a button is across the screen, and without this the copies would
    // spread out to wherever the pointer passed through on the way to OK.
    //
    // But the panel's own controls change what gets built: the count, the
    // layout, the axis, whether it is the cut or the body being repeated. So
    // only the reading of the pointer stops here. The rebuild below still runs,
    // and a button pressed with the pointer sitting on the panel is seen in the
    // viewport straight away rather than when the pointer wanders off it.
    Real v = patternTool_.dragged();
    if (follow) {
        if (!patternTool_.typedValue.empty()) {
            try { v = std::stod(patternTool_.typedValue); } catch (...) {}
        } else if (patternTool_.axis.valid && pointerDrives() &&
                   patternTool_.axis.facingCamera(camera_)) {
            v = patternTool_.axis.valueAt(camera_, mouseInViewport());
        }
    }
    if (follow && snap) {
        if (patternTool_.mode == PatternMode::Circular) {
            v = std::round(v / 5.0) * 5.0;
        } else {
            const Real step = static_cast<Real>(camera_.snapStep(patternTool_.axis.origin));
            if (step > 0.0) v = std::round(v / step) * step;
        }
    }
    // Copies on top of each other are not a pattern, and a turn of nothing is
    // not one either.
    if (patternTool_.mode == PatternMode::Linear) v = std::max<Real>(v, 0.05);
    if (patternTool_.mode == PatternMode::Circular) {
        // Past a full turn the copies land back on top of each other, so there
        // is nothing beyond it to ask for -- and an unbounded track pointed
        // nearly at the camera can otherwise report an absurd number.
        if (std::fabs(v) < 0.5) v = v < 0.0 ? -0.5 : 0.5;
        v = clampf(v, -360.0, 360.0);
    }
    patternTool_.setDragged(v);

    // The count is part of what is being previewed, so a changed count has to
    // invalidate the same way a changed distance does.
    const Real key = v + patternTool_.count * 1e6 +
                     (patternTool_.useTool ? 1e5 : 0.0);
    if (std::fabs(key - patternTool_.requested) < 1e-9 && patternTool_.previewValid) return;
    patternTool_.requested = key;
    patternTool_.previewValid = false;

    const PatternSpec spec = patternTool_.spec();
    Body tool = patternTool_.useTool ? patternTool_.tool : Body();
    Body base = patternTool_.before;
    // A tool pattern stands where the boolean stood, so the preview builds on
    // the body as it was *before* that boolean rather than on top of its result.
    if (patternTool_.useTool) {
        std::vector<Feature> upto(patternTool_.chainBefore.begin(),
                                  patternTool_.chainBefore.begin() +
                                      static_cast<long>(patternTool_.replacing));
        Body partial;
        if (evaluateFeatures(upto, partial)) base = std::move(partial);
    }
    patternTool_.preview.request(base, [spec, tool](Body& b) {
        return patternBody(b, tool, spec, 7101, nullptr);
    });
}

void Application::commitPattern() {
    if (!patternTool_.active) return;
    const ObjectId id = patternTool_.objectId;
    justFinishedModal_ = true;
    patternTool_.active = false;
    patternTool_.preview.cancel();

    SceneObject* obj = scene_.find(id);
    if (!obj) return;

    std::vector<Feature> chainBefore = patternTool_.chainBefore;
    obj->features = patternTool_.chainBefore;
    obj->body = patternTool_.before;

    Feature f;
    f.kind = FeatureKind::Pattern;
    const PatternSpec spec = patternTool_.spec();
    f.patternMode = spec.mode;
    f.patternCount = spec.count;
    f.axisPoint = spec.origin;
    f.axisDir = spec.dir;
    f.distance = spec.step;
    f.angle = spec.stepAngle;
    f.booleanOp = patternTool_.useTool ? patternTool_.op : BooleanOp::Union;
    if (patternTool_.useTool) f.bakedBody = patternTool_.tool;

    const char* label = spec.mode == PatternMode::Mirror ? "Mirror" : "Pattern";
    std::string why;
    bool ok = false;
    if (patternTool_.useTool) {
        // The boolean is not kept and then patterned on top of: it is replaced,
        // because the pattern's first copy is that boolean. Keeping both would
        // cut the first hole twice.
        std::vector<Feature> next(patternTool_.chainBefore.begin(),
                                  patternTool_.chainBefore.begin() +
                                      static_cast<long>(patternTool_.replacing));
        next.push_back(std::move(f));
        for (size_t i = patternTool_.replacing + 1; i < patternTool_.chainBefore.size(); ++i)
            next.push_back(patternTool_.chainBefore[i]);
        ok = scene_.setFeatures(id, std::move(next), &why);
    } else {
        ok = scene_.addFeature(id, std::move(f), &why);
    }

    if (ok && editKeepsSolid(id)) {
        undo_.push(std::make_unique<FeatureCommand>(id, std::move(chainBefore),
                                                    obj->features, label),
                   /*merge=*/recommitting_);
        settleCommand(Settled::Pattern, id);
        scene_.clearElementSelection();
    } else {
        obj->features = std::move(chainBefore);
        obj->body = std::move(patternTool_.before);
        obj->refreshDerived();
        setNotice(why.empty() ? std::string("The ") + label + " could not be made"
                              : "Refused: " + why);
    }
}

void Application::abortPattern() {
    if (!patternTool_.active) return;
    justFinishedModal_ = true;
    patternTool_.active = false;
    patternTool_.preview.cancel();
    if (SceneObject* obj = scene_.find(patternTool_.objectId)) {
        obj->features = std::move(patternTool_.chainBefore);
        obj->body = std::move(patternTool_.before);
        obj->refreshDerived();
    }
}

void Application::beginDivide() {
    dismissSettled();
    if (tool_.active() || filletTool_.active || createTool_.active() || sketchTool_.active() ||
        faceTool_.active || divideTool_.active || patternTool_.active ||
        reduceTool_.active)
        return;

    const ObjectId id = scene_.contextObject();
    SceneObject* obj = scene_.find(id);
    if (!obj) { setNotice("Select an object first"); return; }
    if (refuseMeshEdit(*obj, "Dividing a face")) return;

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
    } else if (divideTool_.axis.valid && pointerDrives() &&
               divideTool_.axis.facingCamera(camera_)) {
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
                                                    obj->features, "Divide"),
                   /*merge=*/recommitting_);
        settleCommand(Settled::Divide, id);
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

// Works out what this gesture will build, before it builds any of it.
//
// Either the new edges fold into the fillet above -- one operation against the
// body that one ran on -- or they are a fillet of their own on top of what is
// there. Deciding here rather than at the click is the whole point: whatever is
// previewed, searched for a limit, and finally committed is then the same
// operation, and the preview cannot promise a shape the commit will not make.
void Application::planFillet(SceneObject& obj) {
    FilletToolState& t = filletTool_;
    t.folding = false;
    t.buildBase = t.meshBefore;
    t.buildEdges = t.edges;
    t.fixedRadii.assign(t.edges.size(), -1.0);   // all of them take the drag

    if (obj.features.empty()) return;
    const size_t last = obj.features.size() - 1;
    const Feature& fillet = obj.features[last];
    if (fillet.kind != FeatureKind::Bevel || fillet.edges.empty() || !fillet.enabled)
        return;
    if (fillet.edges.kind != ElementRefs::Kind::Explicit) return;
    if (last == 0 || last > obj.featureCache.size()) return;

    const Body& before = obj.featureCache[last - 1];
    if (before.empty()) return;

    std::vector<Index> merged;
    if (!fillet.edges.resolveEdges(before, merged)) return;

    std::vector<Real> radii;
    radii.reserve(merged.size());
    for (size_t i = 0; i < merged.size(); ++i) radii.push_back(fillet.radiusFor(i));

    // The new edges were picked on the body as it stands, which is the rounded
    // one; they have to be found again on the body the fillet above ran on.
    for (Index e : t.edges) {
        if (!t.meshBefore.hasEdge(e)) return;
        Vec3 a, b;
        t.meshBefore.edgePositions(e, a, b);
        const EdgeId mapped = edgeAlongSegment(before, a, b);
        if (mapped == kInvalid) return;

        bool already = false;
        for (size_t i = 0; i < merged.size(); ++i)
            if (merged[i] == mapped) {
                radii[i] = -1.0;          // re-picking one restates its radius
                already = true;
            }
        if (!already) { merged.push_back(mapped); radii.push_back(-1.0); }
    }

    t.folding = true;
    t.foldAt = last;
    t.buildBase = before;
    t.buildEdges = std::move(merged);
    t.fixedRadii = std::move(radii);
}

// The radius each planned edge takes, in the order the edges are planned in.
std::vector<Real> Application::filletRadiiAt(Real radius) const {
    const FilletToolState& t = filletTool_;
    std::vector<Real> out;
    out.reserve(t.buildEdges.size());
    for (size_t i = 0; i < t.buildEdges.size(); ++i)
        out.push_back(i < t.fixedRadii.size() && t.fixedRadii[i] >= 0.0
                          ? t.fixedRadii[i] : radius);
    return out;
}

bool Application::filletUniform(const std::vector<Real>& radii) {
    for (size_t i = 1; i < radii.size(); ++i)
        if (std::fabs(radii[i] - radii[0]) > 1e-9) return false;
    return true;
}

FilletSpec Application::filletSpecAt(Real radius) const {
    const FilletToolState& t = filletTool_;
    FilletSpec spec;
    spec.chamfer = t.chamfer;
    spec.edges.reserve(t.buildEdges.size());
    for (size_t i = 0; i < t.buildEdges.size(); ++i) {
        const Real r = i < t.fixedRadii.size() && t.fixedRadii[i] >= 0.0
                           ? t.fixedRadii[i] : radius;
        // Only the edges this gesture is setting taper; ones folded in from the
        // fillet above keep whatever they already had.
        const bool mine = !(i < t.fixedRadii.size() && t.fixedRadii[i] >= 0.0);
        spec.edges.push_back({t.buildEdges[i], r, mine ? t.endRadius : Real(-1)});
    }
    return spec;
}

void Application::beginFillet() {
    dismissSettled();
    if (tool_.active() || filletTool_.active || createTool_.active() || sketchTool_.active() || reduceTool_.active) return;

    const ObjectId id = scene_.contextObject();
    SceneObject* obj = scene_.find(id);
    if (!obj) {
        setNotice("Select an object to fillet");
        return;
    }
    if (refuseMeshEdit(*obj, "Filleting an edge")) return;

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
    planFillet(*obj);

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
                Body test = filletTool_.buildBase;
                FilletSpec spec = filletSpecAt(tried);
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
    // The same operation the preview and the commit will run, so the largest
    // radius found is the largest radius of the thing being made.
    filletTool_.search.trial.start(filletTrial(filletTool_.buildBase, filletSpecAt(radius)));
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

// A plate drilled with a grid of holes, exported fine as an STL, for the
// benchmarks and demos to work on without a file of the user's. It is the other
// kind of large mesh: one that was CAD before it was triangles, where the
// triangle count is high and the faces behind it are few.
std::string Application::writeDrilledPlate() {
    scene_.clear();
    PrimitiveSpec ps;
    ps.kind = PrimitiveKind::Box;
    ps.box = {120, 120, 8};
    const ObjectId pid = scene_.addPrimitive(PrimitiveKind::Box, ps);
    PrimitiveSpec hs;
    hs.kind = PrimitiveKind::Cylinder;
    hs.cylinder = {2.5, 30, 48};
    Body holes;
    makePrimitive(hs, holes, Backend::Brep);
    for (int ix = 0; ix < 8; ++ix)
        for (int iy = 0; iy < 8; ++iy) {
            Body h;
            makePrimitive(hs, h, Backend::Brep);
            h.transform(translate({-49.0 + ix * 14.0, -49.0 + iy * 14.0, 0}));
            if (ix == 0 && iy == 0) { holes = h; continue; }
            Body u;
            if (booleanOp(holes, h, BooleanOp::Union, u, 900 + ix * 8 + iy, false, nullptr))
                holes = std::move(u);
        }
    Feature cut;
    cut.kind = FeatureKind::Boolean;
    cut.booleanOp = BooleanOp::Difference;
    cut.bakedBody = std::move(holes);
    scene_.addFeature(pid, std::move(cut), nullptr);
    StlOptions opt;
    opt.binary = true;
    opt.deviationMm = 0.002;
    const std::string out = (std::filesystem::temp_directory_path() / "tangent_plate_bench.stl").string();
    const StlResult sr = exportStl(scene_, out, opt);
    std::fprintf(stderr, "[demo] built a drilled plate: %d faces, %zu triangles out\n",
                 scene_.objects().front()->body.faceCount(), sr.triangles);
    scene_.clear();
    return out;
}

// Opens a mesh and the Reduce Mesh panel on it, with Fit for conversion on, and
// waits for the preview so that a screenshot shows real numbers. `:plate` uses
// the drilled plate above.
void Application::stepReduceDemo() {
    if (reduceDemo_.empty() || reduceDemoDone_ || viewRect_.w <= 0) return;
    reduceDemoDone_ = true;
    const std::string path = reduceDemo_ == ":plate" ? writeDrilledPlate() : reduceDemo_;
    scene_.clear();
    runFileOperation(FileMode::ImportMesh, path);
    if (scene_.objects().empty()) { std::fprintf(stderr, "[reduce-demo] nothing imported\n"); return; }
    beginReduce();
    if (!reduceTool_.active) { std::fprintf(stderr, "[reduce-demo] the panel did not open\n"); return; }
    reduceTool_.target = kSolidifyFaceLimit;
    reduceTool_.loosen = true;
    requestReducePreview();
    while (reduceTool_.preview.busy()) {
        updateReduce();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    updateReduce();
    const ReduceJob* shown = reduceTool_.shown.get();
    if (!shown) { std::fprintf(stderr, "[reduce-demo] no preview came back\n"); return; }
    std::fprintf(stderr, "[reduce-demo] ready=%d ok=%d %zu -> %zu, used %.4f, measured %.4f, solid faces %d %s\n",
                 (int)reduceTool_.ready(), (int)shown->result.ok, shown->result.trianglesBefore,
                 shown->result.trianglesAfter, shown->result.toleranceUsedMm, shown->result.deviationMm,
                 shown->solidFaces, shown->result.error.c_str());
    const SceneObject* o = scene_.find(reduceTool_.objectId);
    if (o) camera_.frame(o->worldBounds());
    camera_.snapToGoal();
    view_.showWireframe = true;          // the triangles are the point
}

// Times every stage a mesh goes through between a file and a drawn frame, so
// that "importing is slow" becomes a number attached to a particular step.
void Application::stepMeshBench() {
    if (meshBench_.empty() || meshBenchDone_ || viewRect_.w <= 0) return;
    meshBenchDone_ = true;

    using Clock = std::chrono::steady_clock;
    auto ms = [](Clock::time_point a, Clock::time_point b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };
    auto say = [&](const char* what, double t) {
        std::fprintf(stderr, "[mesh-bench] %-26s %8.1f ms\n", what, t);
    };

    scene_.clear();

    // ":plate" builds the other kind of mesh -- one that was CAD before it was
    // triangles. A plate drilled with a grid of holes and exported fine, so the
    // triangle count is large but the faces behind it are few, and conversion
    // is allowed through. That is the case the prediction does not stop.
    if (meshBench_.rfind(":plate", 0) == 0) meshBench_ = writeDrilledPlate();

    // --- reading ---
    Body body;
    auto t0 = Clock::now();
    const MeshImport r = readMesh(meshBench_, body);
    auto t1 = Clock::now();
    if (!r.ok) {
        std::fprintf(stderr, "[mesh-bench] read failed: %s\n", r.error.c_str());
        return;
    }
    std::fprintf(stderr, "[mesh-bench] %zu triangles, %d verts, closed=%d\n",
                 r.triangles, (int)body.mesh().verts.size(), (int)r.closed);
    say("read + weld + build", ms(t0, t1));

    // --- what the import then does ---
    auto t2 = Clock::now();
    RenderMesh rm;
    body.tessellate(rm);
    auto t3 = Clock::now();
    say("tessellate for drawing", ms(t2, t3));

    auto t4 = Clock::now();
    const MeshHealth h = body.health(false);
    auto t5 = Clock::now();
    say("health check", ms(t4, t5));
    std::fprintf(stderr, "[mesh-bench] volume %.1f mm3, watertight=%d\n",
                 h.volume, (int)h.watertight);

    auto t6 = Clock::now();
    const ObjectId id = scene_.addImportedBody(body, "bench");
    auto t7 = Clock::now();
    say("addImportedBody", ms(t6, t7));

    // --- the per-frame costs ---
    SceneObject* o = scene_.find(id);
    if (o) {
        auto t8 = Clock::now();
        const PrintReport pr = checkPrintability(o->body, o->render);
        auto t9 = Clock::now();
        say("printability check", ms(t8, t9));
        std::fprintf(stderr, "[mesh-bench] %zu findings\n", pr.findings.size());

        auto ta = Clock::now();
        o->refreshDerived();
        auto tb = Clock::now();
        say("refreshDerived", ms(ta, tb));
    }

    // --- the path a large mesh takes: reduce, convert, then what it is for ---
    //
    // A boolean and a split, timed on the converted solid, because those are
    // the reasons a mesh is converted at all. Every step is checked to have
    // made what it should, not only timed.
    if (brep::available() && r.closed) {
        Body work = body;
        ReduceOptions options;
        options.toleranceMm = 0.05;
        options.targetTriangles = kSolidifyFaceLimit;
        options.loosenToReachTarget = true;
        ReduceResult rr;
        auto t0 = Clock::now();
        const bool reduced = reduceBody(work, options, 5151, rr);
        say("reduce (0.05mm, fit)", ms(t0, Clock::now()));
        std::fprintf(stderr, "[mesh-bench] reduce ok=%d %zu -> %zu, %d pass%s, measured %.4f of %.4f used, within=%d %s\n",
                     (int)reduced, rr.trianglesBefore, rr.trianglesAfter, rr.passes,
                     rr.passes == 1 ? "" : "es", rr.deviationMm, rr.toleranceUsedMm,
                     (int)rr.withinTolerance, rr.error.c_str());

        t0 = Clock::now();
        const SolidifyResult sc = toSolid(work, 4242);
        say("convert", ms(t0, Clock::now()));
        std::fprintf(stderr, "[mesh-bench] convert ok=%d, %d triangles -> %d faces, closed=%d %s\n",
                     (int)sc.ok, sc.facesBefore, sc.facesAfter,
                     (int)(sc.ok && brep::closedShell(work.brep())), sc.error.c_str());

        if (sc.ok) {
            const AABB wb = work.bounds();
            PrimitiveSpec cyl;
            cyl.kind = PrimitiveKind::Cylinder;
            cyl.cylinder = {std::max<Real>(0.5, length(wb.size()) * 0.05), length(wb.size()) * 2, 48};
            Body tool;
            makePrimitive(cyl, tool, Backend::Brep);
            tool.transform(translate(wb.center()));
            Body drilled;
            std::string why;
            t0 = Clock::now();
            const bool bored = booleanOp(work, tool, BooleanOp::Difference, drilled, 5252, false, &why);
            say("boolean (bore a hole)", ms(t0, Clock::now()));
            std::fprintf(stderr, "[mesh-bench] boolean ok=%d closed=%d, %.1f -> %.1f mm3 %s\n", (int)bored,
                         (int)(bored && brep::closedShell(drilled.brep())), work.health(false).volume,
                         bored ? drilled.health(false).volume : 0.0, why.c_str());

            const Body& toSplit = bored ? drilled : work;
            Body top, bottom;
            t0 = Clock::now();
            const bool cut = splitByPlane(toSplit, toSplit.bounds().center(), {0, 0, 1}, top, bottom);
            say("split (through the middle)", ms(t0, Clock::now()));
            if (cut)
                std::fprintf(stderr, "[mesh-bench] split ok, %.1f + %.1f = %.1f of %.1f mm3, closed=%d/%d\n",
                             top.health(false).volume, bottom.health(false).volume,
                             top.health(false).volume + bottom.health(false).volume,
                             toSplit.health(false).volume, (int)brep::closedShell(top.brep()),
                             (int)brep::closedShell(bottom.brep()));
            else
                std::fprintf(stderr, "[mesh-bench] split REFUSED\n");
        }
    }

    // --- and the whole thing as a user does it: one menu action ---
    scene_.clear();
    auto tg0 = Clock::now();
    runFileOperation(FileMode::ImportMesh, meshBench_);
    auto tg1 = Clock::now();
    say("IMPORT, end to end", ms(tg0, tg1));
    std::fprintf(stderr, "[mesh-bench] %s\n", notice_.c_str());

    // The frames straight after it, which is where the print check used to
    // land. The worst of them is the stall a user would feel.
    {
        double worst = 0.0;
        int framesUntilReport = -1;
        for (int i = 0; i < 400; ++i) {
            auto f0 = Clock::now();
            drawPrintIssues();
            auto f1 = Clock::now();
            worst = std::max(worst, ms(f0, f1));
            const SceneObject* so = scene_.objects().empty() ? nullptr
                                                            : scene_.objects().front().get();
            if (so && so->printVersion == so->meshVersion) { framesUntilReport = i; break; }
            SDL_Delay(1);
        }
        std::fprintf(stderr, "[mesh-bench] %-26s %8.2f ms\n", "worst frame after import", worst);
        std::fprintf(stderr, "[mesh-bench] print report arrived after %d frames\n",
                     framesUntilReport);
    }

    // --- frames ---
    auto te = Clock::now();
    const int frames = 30;
    for (int i = 0; i < frames; ++i) { drawFrame(); }
    auto tf = Clock::now();
    std::fprintf(stderr, "[mesh-bench] %-26s %8.1f ms/frame\n", "steady-state frame",
                 ms(te, tf) / frames);

    // And with the print overlay full. A wall limit far thicker than this model
    // flags as many faces as the report will keep, which is the worst the
    // overlay can be asked to draw.
    if (!scene_.objects().empty()) {
        SceneObject* obj = scene_.objects().front().get();
        PrintProfile thick;
        thick.minWallMm = 50.0;
        auto tr0 = Clock::now();
        refreshPrintCheck(*obj, thick);
        auto tr1 = Clock::now();
        say("check + gather, overlay full", ms(tr0, tr1));
        std::fprintf(stderr, "[mesh-bench] %d thin faces counted, %zu kept, %zu triangles drawn\n",
                     obj->printCheck.thinWalls, obj->printCheck.findings.size(),
                     obj->printTriangles.size() / 3);
        // The overlay's own CPU cost per frame, timed directly: a whole frame's
        // wall clock mostly measures handing work to the GPU, and the thing this
        // is watching for is the old search through every triangle per flagged
        // face, which was entirely on this side.
        view_.showPrintIssues = true;
        auto tg = Clock::now();
        for (int i = 0; i < frames; ++i) drawPrintIssues();
        auto th = Clock::now();
        std::fprintf(stderr, "[mesh-bench] %-26s %8.3f ms/frame\n", "overlay draw, full",
                     ms(tg, th) / frames);
    }
}

// 1 drives the typed-path fallback, which is what a system with no chooser
// gets and is the half of this that can be tested without a person to click.
// 2 asks for a real chooser and reports whether one opened -- it puts a window
// on screen, so it is not in the sweep.
void Application::stepDialogDemo() {
    if (dialogDemo_ <= 0 || dialogDemoDone_ || viewRect_.w <= 0) return;
    dialogDemoDone_ = true;

    if (dialogDemo_ == 1) {
        typePathInstead_ = true;          // as if no chooser could be shown

        scene_.clear();
        PrimitiveSpec s;
        s.kind = PrimitiveKind::Box;
        s.box = {12, 14, 16};
        scene_.addPrimitive(PrimitiveKind::Box, s);

        beginFilePrompt(FileMode::ExportStep);
        std::fprintf(stderr, "[dialog-demo] fallback: popup=%d, chooser waiting=%d, "
                             "seeded path \"%s\"\n",
                     (int)(fileMode_ != FileMode::None), (int)fileDialog_.waiting(),
                     pathField_);

        // What the popup's OK button does, without needing the popup drawn.
        const std::string out = std::string(stepDemo_.empty() ? "/tmp/tg_dialog_demo.step"
                                                              : stepDemo_);
        const FileMode mode = fileMode_;
        fileMode_ = FileMode::None;
        runFileOperation(mode, out);
        std::fprintf(stderr, "[dialog-demo] typed path ran: %s\n", notice_.c_str());
        std::remove(out.c_str());
        return;
    }

    // A real chooser. Asked for, then given a moment to answer; nobody is here
    // to click it, so None after the wait means it opened and is sitting there.
    beginFilePrompt(FileMode::ImportMesh);
    std::fprintf(stderr, "[dialog-demo] asked for a chooser, waiting=%d\n",
                 (int)fileDialog_.waiting());
    for (int i = 0; i < 120 && fileDialog_.waiting(); ++i) {
        SDL_PumpEvents();               // the portal needs an event loop
        pollFileDialog();
        SDL_Delay(25);
    }
    std::fprintf(stderr, "[dialog-demo] after 3s: waiting=%d, fell back=%d%s%s\n",
                 (int)fileDialog_.waiting(), (int)typePathInstead_,
                 notice_.empty() ? "" : ", notice: ", notice_.c_str());
}

// Exports the scene to STEP and reads it straight back in, through the same
// menu path a person uses. The round trip is checked in test_step; what this
// checks is everything around it -- the object transforms folded into the
// geometry, the objects that come back, and whether what returns can still be
// modelled on.
void Application::stepStepDemo() {
    if (stepDemo_.empty() || stepDemoDone_ || viewRect_.w <= 0) return;
    stepDemoDone_ = true;

    // Something with a curved face and a modelling operation on it, so that a
    // tessellated round trip would be obvious.
    scene_.clear();
    PrimitiveSpec base;
    base.kind = PrimitiveKind::Cylinder;
    base.cylinder = {15, 30, 64};
    const ObjectId id = scene_.addPrimitive(PrimitiveKind::Cylinder, base);
    SceneObject* o = scene_.find(id);
    o->transform.position = {12, 5, 0};        // off the origin, to prove it travels

    {
        PrimitiveSpec ts;
        ts.kind = PrimitiveKind::Cylinder;
        ts.cylinder = {5, 50, 48};
        Body tool;
        makePrimitive(ts, tool, Backend::Brep);
        Feature cut;
        cut.kind = FeatureKind::Boolean;
        cut.booleanOp = BooleanOp::Difference;
        cut.bakedBody = std::move(tool);
        std::string why;
        scene_.addFeature(id, std::move(cut), &why);
    }
    o = scene_.find(id);
    const Real before = o->body.health(false).volume;
    const int facesBefore = o->body.faceCount();
    std::fprintf(stderr, "[step-demo] made: %d faces, %.1f mm3, at %.1f,%.1f,%.1f\n",
                 facesBefore, before, o->transform.position.x,
                 o->transform.position.y, o->transform.position.z);

    // Out as STL and back in as triangles, then converted -- the other half of
    // the exchange story, and the one the mesh backend now exists for.
    {
        const std::string stl = stepDemo_ + ".stl";
        StlOptions opt;
        opt.binary = true;
        opt.deviationMm = 0.05;
        const StlResult sr = exportStl(scene_, stl, opt);
        std::fprintf(stderr, "[step-demo] stl out: ok=%d, %zu triangles\n",
                     (int)sr.ok, sr.triangles);

        Body mesh;
        const MeshImport mi = readMesh(stl, mesh);
        std::fprintf(stderr, "[step-demo] stl in: ok=%d, %zu triangles, closed=%d%s\n",
                     (int)mi.ok, mi.triangles, (int)mi.closed, mi.error.c_str());
        if (mi.ok) {
            std::fprintf(stderr, "[step-demo] as a mesh: %.1f mm3\n",
                         mesh.health(false).volume);
            const SolidifyResult sc = toSolid(mesh, 777);
            std::fprintf(stderr, "[step-demo] convert: ok=%d, %d tris -> %d faces%s\n",
                         (int)sc.ok, sc.facesBefore, sc.facesAfter, sc.error.c_str());
            if (sc.ok) {
                // Flat faces come back; the curved wall does not, and the
                // volume says by how much.
                std::fprintf(stderr, "[step-demo] converted: %.1f mm3, valid=%d\n",
                             mesh.health(false).volume, (int)mesh.validate());
            }
        }
        std::remove(stl.c_str());
    }

    runFileOperation(FileMode::ExportStep, stepDemo_);
    std::fprintf(stderr, "[step-demo] export: %s\n", notice_.c_str());

    scene_.clear();
    runFileOperation(FileMode::ImportStep, stepDemo_);
    std::fprintf(stderr, "[step-demo] import: %s\n", notice_.c_str());

    if (scene_.objects().empty()) {
        std::fprintf(stderr, "[step-demo] nothing came back\n");
        return;
    }
    SceneObject* back = scene_.objects().front().get();
    const AABB b = back->localBounds;
    std::fprintf(stderr,
                 "[step-demo] back: \"%s\", %d faces, %.1f mm3, valid=%d, "
                 "%zu features, centred at %.1f,%.1f,%.1f\n",
                 back->name.c_str(), back->body.faceCount(),
                 back->body.health(false).volume, (int)back->body.validate(),
                 back->features.size(), b.center().x, b.center().y, b.center().z);
    std::fprintf(stderr, "[step-demo] faces %s, volume %s\n",
                 back->body.faceCount() == facesBefore ? "SAME" : "DIFFERENT",
                 std::fabs(back->body.health(false).volume - before) < 0.5
                     ? "SAME" : "DIFFERENT");

    // And the real question: can it still be modelled on? A fillet needs the
    // edges to be edges of real surfaces, which a tessellated import would not
    // have given us.
    std::vector<EdgeId> edges;
    back->body.allEdges(edges);
    if (!edges.empty()) {
        FilletSpec sp;
        sp.edges.push_back({edges.front(), 1.0});
        sp.salt = 5150;
        Body copy = back->body;
        std::string why;
        const bool ok = filletEdges(copy, sp, &why);
        std::fprintf(stderr, "[step-demo] filleting an imported edge: %s%s\n",
                     ok ? "built" : "REFUSED ", ok ? "" : why.c_str());
    }
}

// Drives the pattern tool the way a person does: begin the gesture, set the
// numbers, let the preview land, commit. 1 a row of holes, 2 a bolt circle,
// 3 a mirrored body, 4 a mirrored cut, 5 the panel left open to be looked at.
void Application::stepPatternDemo() {
    if (patternDemo_ <= 0 || patternDemoDone_ || viewRect_.w <= 0) return;
    if (scene_.objects().empty()) return;
    patternDemoDone_ = true;

    const int mode = patternDemo_;
    camera_.yaw = 0.7f;
    camera_.pitch = 0.75f;
    camera_.distance = 150.0f;
    camera_.snapToGoal();

    // The starting body, and for the tool cases the cut that will be repeated.
    scene_.clear();
    PrimitiveSpec base;
    ObjectId id = kNoObject;
    if (mode == 1 || mode == 4) {
        base.kind = PrimitiveKind::Box;
        base.box = {100, 20, 10};
        id = scene_.addPrimitive(PrimitiveKind::Box, base);
    } else if (mode == 2 || mode == 5 || mode == 6 || mode == 7) {
        base.kind = PrimitiveKind::Cylinder;
        base.cylinder = {30, 6, 64};
        id = scene_.addPrimitive(PrimitiveKind::Cylinder, base);
    } else {
        base.kind = PrimitiveKind::Box;
        base.box = {20, 30, 12};
        id = scene_.addPrimitive(PrimitiveKind::Box, base, Vec3{10, 0, 0});
    }
    scene_.select(id);

    auto report = [&](const char* what) {
        const SceneObject* o = scene_.find(id);
        std::fprintf(stderr, "[pattern-demo] %s: %d faces, %.1f mm3, valid=%d, "
                             "%zu features, last=%s\n",
                     what, o->body.faceCount(), o->body.health(false).volume,
                     (int)o->body.validate(), o->features.size(),
                     o->features.back().summary().c_str());
    };

    // A boolean first, where the pattern is meant to repeat one.
    if (mode != 3) {
        PrimitiveSpec ts;
        ts.kind = PrimitiveKind::Cylinder;
        ts.cylinder = {mode == 4 ? Real(4) : Real(3), 40, 48};
        Body tool;
        makePrimitive(ts, tool, Backend::Brep);
        tool.transform(translate(mode == 1 ? Vec3{-30, 0, 0}
                               : mode == 4 ? Vec3{20, 0, 0}
                                           : Vec3{20, 0, 0}));
        Feature cut;
        cut.kind = FeatureKind::Boolean;
        cut.booleanOp = BooleanOp::Difference;
        cut.bakedBody = std::move(tool);
        std::string why;
        if (!scene_.addFeature(id, std::move(cut), &why))
            std::fprintf(stderr, "[pattern-demo] the first cut failed: %s\n", why.c_str());
        report("one cut");
    }

    beginPattern(mode == 3 || mode == 4 ? PatternMode::Mirror : PatternMode::Linear);
    if (!patternTool_.active) {
        std::fprintf(stderr, "[pattern-demo] the tool would not start\n");
        return;
    }

    if (mode == 1) {
        patternTool_.typedValue = "15";
        patternTool_.count = 5;
    } else if (mode == 2 || mode == 5 || mode == 6 || mode == 7) {
        patternTool_.axisIndex = 2;
        setPatternMode(PatternMode::Circular);
        patternTool_.count = 8;
        patternTool_.typedValue = "45";
    } else {
        patternTool_.axisIndex = 0;
        setPatternMode(PatternMode::Mirror);
    }

    updatePattern(false);
    while (patternTool_.preview.busy()) updatePattern(false, /*follow=*/false);
    updatePattern(false, /*follow=*/false);
    report("previewed");

    if (mode == 5) return;          // left open mid-gesture, to be seen
    std::fprintf(stderr, "[pattern-demo] plane/axis %d at %.2f, useTool=%d\n",
                 patternTool_.axisIndex, patternTool_.offset, (int)patternTool_.useTool);
    commitPattern();
    report("committed");
    if (!ui_.notice.empty())
        std::fprintf(stderr, "[pattern-demo] notice: %s\n", ui_.notice.c_str());

    // 6 goes on to do what the panel is there for: the operation is applied,
    // the pointer is free, and the count is changed from a control that used to
    // be unreachable without dragging the whole gesture across the viewport.
    if (mode == 6) {
        std::fprintf(stderr, "[pattern-demo] settled=%d  undo=\"%s\"\n",
                     (int)(settled_ == Settled::Pattern), undo_.undoLabel().c_str());
        for (int n : {6, 12, 3}) {
            patternTool_.count = n;
            patternTool_.stepAngle = radians(360.0 / n);
            recommitSettled();
            const SceneObject* o = scene_.find(id);
            std::fprintf(stderr, "[pattern-demo] adjusted to %d: %d faces, %.1f mm3, "
                                 "%zu features, undo depth %zu\n",
                         n, o->body.faceCount(), o->body.health(false).volume,
                         o->features.size(), undo_.depth());
        }
        // And one the kernel will not take, to see the model and the undo entry
        // stay in step rather than drifting apart. A count of one is refused
        // outright; the panel cannot ask for it, but a refusal from any cause
        // comes back through here the same way.
        patternTool_.count = 1;
        recommitSettled();
        const SceneObject* o = scene_.find(id);
        std::fprintf(stderr, "[pattern-demo] after a refused adjustment: %d faces, "
                             "%.1f mm3, %zu features, undo depth %zu\n",
                     o->body.faceCount(), o->body.health(false).volume,
                     o->features.size(), undo_.depth());
        dismissSettled();
        std::fprintf(stderr, "[pattern-demo] dismissed: settled=%d\n",
                     (int)(settled_ == Settled::Pattern));
    }
    // 7 stops with the panel settled, which is the state this is all for: the
    // ring is cut, the pointer is free, and the count is still there to change.
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

    if (faceDemo_ == 11 || faceDemo_ == 12) {
        scene_.clearElementSelection();
        scene_.selectElement({id, ElementKind::Face, topFace()}, true);
        beginFaceMove(FaceOp::Scale);
        faceTool_.typedValue = faceDemo_ == 11 ? "50" : "-40";
        updateFaceMove(false);
        while (faceTool_.preview.busy()) updateFaceMove(false);
        updateFaceMove(false);
        report(faceDemo_ == 11 ? "grown by half" : "shrunk to three fifths");
        if (faceDemo_ == 12) commitFaceMove();     // 11 stays open, to be seen
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
// What the preview showed against what the chain rebuilt.
//
// The preview fillets the body the gesture started from, by edge index. The
// commit writes a feature that names those edges and lets the chain resolve
// them again on its own rebuild. Those are two different routes to the same
// answer, and when they disagree the user sees one shape and gets another.
void Application::stepPreviewCheck() {
    if (previewCheck_ <= 0 || previewCheckDone_ || viewRect_.w <= 0) return;
    if (scene_.objects().empty()) return;
    previewCheckDone_ = true;

    const ObjectId id = scene_.objects().front()->id;
    const SceneObject* o = scene_.find(id);
    scene_.select(id);
    scene_.clearElementSelection();

    // Mode 3: round two edges first, then come back and round the rest. That
    // second gesture is the one that folds into the first, and the one where
    // what was shown and what was built used to be different operations.
    if (previewCheck_ == 3) {
        std::vector<EdgeId> es;
        o->body.allEdges(es);
        int taken = 0;
        for (EdgeId e : es) {
            Vec3 a, b;
            o->body.edgePositions(e, a, b);
            if (std::fabs(a.z - 10.0) > 1e-6 || std::fabs(b.z - 10.0) > 1e-6) continue;
            if (std::fabs((b - a).x) < 1e-6) continue;      // only the two along X
            scene_.selectElement({id, ElementKind::Edge, e}, true);
            if (++taken == 2) break;
        }
        beginFillet();
        for (int i = 0; i < 400 && filletTool_.search.active; ++i) {
            stepFilletLimitSearch();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        filletTool_.typedValue = "4";
        updateFillet(false);
        while (filletTool_.preview.busy()) updateFillet(false);
        updateFillet(false);
        commitFillet();
        const SceneObject* r = scene_.find(id);
        std::fprintf(stderr, "[preview] first round: %d faces, %.1f mm3\n",
                     r->body.faceCount(), r->body.health(false).volume);

        // Now the two top edges still sharp -- which existed before that first
        // fillet ran, so they can be folded into it.
        scene_.clearElementSelection();
        const SceneObject* o2 = scene_.find(id);
        std::vector<EdgeId> es2;
        o2->body.allEdges(es2);
        int more = 0;
        for (EdgeId e : es2) {
            Vec3 a, b;
            o2->body.edgePositions(e, a, b);
            if (std::fabs(a.z - 10.0) > 1e-6 || std::fabs(b.z - 10.0) > 1e-6) continue;
            if (std::fabs((b - a).y) < 1e-6) continue;      // the ones along Y
            scene_.selectElement({id, ElementKind::Edge, e}, true);
            ++more;
        }
        std::fprintf(stderr, "[preview] second gesture picks %d edges\n", more);
    } else if (previewCheck_ == 1) {
        // The top face, whose boundary edges the fillet takes.
        std::vector<FaceId> fs;
        o->body.allFaces(fs);
        for (FaceId f : fs)
            if (dot(o->body.faceNormal(f), Vec3{0, 0, 1}) > 0.99)
                scene_.selectElement({id, ElementKind::Face, f}, true);
    } else {
        // Every edge of the body.
        std::vector<EdgeId> es;
        o->body.allEdges(es);
        for (EdgeId e : es) scene_.selectElement({id, ElementKind::Edge, e}, true);
    }

    beginFillet();
    if (!filletTool_.active) { std::fprintf(stderr, "[preview] would not start\n"); return; }
    std::fprintf(stderr, "[preview] %zu edges caught\n", filletTool_.edges.size());

    // The floor and the limit are found in another process a trial at a time,
    // the way the frame loop does it. Nothing previews until they land.
    for (int i = 0; i < 400 && filletTool_.search.active; ++i) {
        stepFilletLimitSearch();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::fprintf(stderr, "[preview] floor %.2f, largest %.2f, %s %zu edges\n",
                 filletTool_.axis.baseValue, filletTool_.maxRadius,
                 filletTool_.folding ? "folded into the fillet above:" : "on its own:",
                 filletTool_.buildEdges.size());

    if (previewCheck_ == 4) filletTool_.chamfer = true;
    if (previewCheck_ == 5) filletTool_.endRadius = 2.0;
    filletTool_.typedValue = "9";
    updateFillet(false);
    while (filletTool_.preview.busy()) updateFillet(false);
    updateFillet(false);
    std::fprintf(stderr, "[preview] asked for 9, radius is %.2f\n",
                 filletTool_.currentRadius);

    const SceneObject* p1 = scene_.find(id);
    const int previewFaces = p1->body.faceCount();
    const Real previewVolume = p1->body.health(false).volume;
    std::fprintf(stderr, "[preview] shown:    %d faces, %.1f mm3\n",
                 previewFaces, previewVolume);

    commitFillet();
    const SceneObject* p2 = scene_.find(id);
    std::fprintf(stderr, "[preview] committed: %d faces, %.1f mm3, %zu features%s\n",
                 p2->body.faceCount(), p2->body.health(false).volume,
                 p2->features.size(), notice_.empty() ? "" : ("  notice: " + notice_).c_str());
    std::fprintf(stderr, "[preview] %s\n",
                 (p2->body.faceCount() == previewFaces &&
                  std::fabs(p2->body.health(false).volume - previewVolume) < 1e-3)
                     ? "SAME" : "DIFFERENT -- the preview lied");

    // 6 carries on into the panel that is still up: the fillet is cut and the
    // pointer is free, so Round/Flat and the taper are finally reachable
    // without dragging the radius across the viewport to get to them.
    if (previewCheck_ == 6) {
        std::fprintf(stderr, "[preview] settled=%d  undo=\"%s\"  depth %zu\n",
                     (int)(settled_ == Settled::Fillet), undo_.undoLabel().c_str(),
                     undo_.depth());
        auto say = [&](const char* what) {
            const SceneObject* o = scene_.find(id);
            std::fprintf(stderr, "[preview] %s: %d faces, %.1f mm3, %zu features, "
                                 "last=%s, undo depth %zu\n",
                         what, o->body.faceCount(), o->body.health(false).volume,
                         o->features.size(), o->features.back().summary().c_str(),
                         undo_.depth());
        };
        filletTool_.chamfer = true;
        recommitSettled();
        say("switched to a flat cut");

        filletTool_.chamfer = false;
        filletTool_.endRadius = 2.0;
        recommitSettled();
        say("switched to a taper");

        // Something the kernel will not take, to see the last good result stand.
        filletTool_.endRadius = -1.0;
        filletTool_.currentRadius = 1e6;
        recommitSettled();
        say("after a refused adjustment");

        dismissSettled();
        std::fprintf(stderr, "[preview] dismissed: settled=%d\n",
                     (int)(settled_ == Settled::Fillet));
    }
}

void Application::stepPrintDemo() {
    if (printDemo_ <= 0 || printDemoDone_ || viewRect_.w <= 0) return;
    if (scene_.objects().empty()) return;
    printDemoDone_ = true;

    const ObjectId id = scene_.objects().front()->id;
    SceneObject* o = scene_.find(id);
    std::string why;
    auto topFace = [&] {
        std::vector<FaceId> fs;
        o->body.allFaces(fs);
        FaceId best = fs.front();
        Real bd = -1e30;
        for (FaceId f : fs) {
            const Real d = dot(o->body.faceNormal(f), Vec3{0, 0, 1});
            if (d > bd) { bd = d; best = f; }
        }
        return best;
    };

    if (printDemo_ == 1 || printDemo_ == 3) {
        Body b = o->body;
        if (shellBody(b, {topFace()}, 0.35, 600, &why)) {
            o->body = std::move(b);
            o->refreshDerived();
        } else {
            std::fprintf(stderr, "[print-demo] shell refused: %s\n", why.c_str());
        }
    }
    if (printDemo_ == 2 || printDemo_ == 3) {
        Body b = o->body;
        if (scaleFaces(b, {topFace()}, 4.5, 601, &why)) {
            o->body = std::move(b);
            o->refreshDerived();
        } else {
            std::fprintf(stderr, "[print-demo] taper refused: %s\n", why.c_str());
        }
    }

    // Framed after the shape is final, so the whole of it is in view.
    camera_.yaw = 0.7f;
    camera_.pitch = printDemo_ == 2 ? 0.16f : 0.42f;
    camera_.frame(o->worldBounds());
    camera_.snapToGoal();

    refreshPrintCheck(*o);
    std::fprintf(stderr, "[print-demo] %d thin: %s\n",
                 o->printCheck.thinWalls,
                 summarise(o->printCheck).empty() ? "nothing to report"
                                                  : summarise(o->printCheck).c_str());
}

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
            if (pointerDrives() && filletTool_.axis.facingCamera(camera_)) {
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
        // What the commit will build, not something that resembles it.
        const FilletSpec spec = filletSpecAt(newR);
        filletTool_.preview.request(filletTool_.buildBase, [spec](Body& b) {
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

    // Exactly what was planned when the gesture began, and exactly what has
    // been on screen the whole time.
    if (filletTool_.folding && extendLastFillet(*obj, filletTool_.currentRadius))
        return;

    Feature f;
    f.kind = FeatureKind::Bevel;
    f.radii = filletRadiiAt(filletTool_.currentRadius);
    f.width = filletTool_.currentRadius;
    f.chamfer = filletTool_.chamfer;
    f.endWidth = filletTool_.endRadius;

    // A rim is the sturdier way to name a set of edges, but it comes back in
    // the body's order rather than the one it went in, so it is only safe when
    // every edge has the same radius. With one radius there is nothing for the
    // order to get wrong, and `width` carries it.
    const bool uniform = filletUniform(f.radii);
    f.edges = nameEdges(filletTool_.buildBase, filletTool_.buildEdges, uniform);
    if (uniform) f.radii.clear();

    std::vector<Feature> chainBefore = filletTool_.chainBefore;
    obj->features = filletTool_.chainBefore;
    std::string why;
    if (scene_.addFeature(id, std::move(f), &why) && editKeepsSolid(id)) {
        undo_.push(std::make_unique<FeatureCommand>(id, std::move(chainBefore),
                                                    obj->features, "Fillet"),
                   /*merge=*/recommitting_);
        settleCommand(Settled::Fillet, id);
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
    if (tool_.active() || createTool_.active() || sketchTool_.active()) return;
    if (editToolActive()) { setNotice("Finish the current operation first"); return; }
    createTool_.start(kind);
}

void Application::beginSketch() {
    if (tool_.active() || createTool_.active() || sketchTool_.active()) return;
    if (editToolActive()) { setNotice("Finish the current operation first"); return; }
    if (!brep::available()) {
        setNotice("Sketching needs the exact kernel, which this build does not have");
        return;
    }
    measure_.end();
    sketchTool_.start();
}

void Application::beginEditSketch(ObjectId object, ElementId sketchUid) {
    if (tool_.active() || createTool_.active() || sketchTool_.active()) return;
    if (editToolActive()) { setNotice("Finish the current operation first"); return; }
    measure_.end();
    if (!sketchTool_.startEdit(scene_, object, sketchUid, camera_))
        setNotice(sketchTool_.takeError());
}

// Folds `edges`, picked on the current mesh, into the fillet at the end of the
// chain. Returns false if there is nothing to fold them into, if any of them
// cannot be traced back to an edge of the mesh that fillet saw, or if the
// merged fillet does not evaluate -- in which case the caller adds a new
// feature and the object is left exactly as it was.
// Writes the planned fillet into the feature above, which is the one it was
// planned against.
//
// The work of deciding -- whether a fold is possible at all, which edges it
// covers and at what radii -- happened in planFillet before anything was drawn.
// All that is left here is to record it and rebuild.
bool Application::extendLastFillet(SceneObject& obj, Real radius) {
    if (!filletTool_.folding) return false;
    if (filletTool_.foldAt >= obj.features.size()) return false;

    Feature& fillet = obj.features[filletTool_.foldAt];
    if (fillet.kind != FeatureKind::Bevel) return false;

    std::vector<Feature> chainBefore = obj.features;
    const ElementRefs edgesBefore = fillet.edges;
    const std::vector<Real> radiiBefore = fillet.radii;

    fillet.radii = filletRadiiAt(radius);
    fillet.chamfer = filletTool_.chamfer;
    fillet.endWidth = filletTool_.endRadius;
    const bool uniform = filletUniform(fillet.radii);
    fillet.edges = nameEdges(filletTool_.buildBase, filletTool_.buildEdges, uniform);
    if (uniform) {
        fillet.width = radius;
        fillet.radii.clear();
    }

    auto putBack = [&] {
        fillet.edges = edgesBefore;
        fillet.radii = radiiBefore;
        scene_.reevaluateFrom(obj.id, filletTool_.foldAt);
    };

    if (!scene_.reevaluateFrom(obj.id, filletTool_.foldAt)) { putBack(); return false; }
    if (obj.features[filletTool_.foldAt].errored) { putBack(); return false; }

    setNotice("Added to the fillet above");
    undo_.push(std::make_unique<FeatureCommand>(obj.id, std::move(chainBefore),
                                                obj.features, "Fillet"),
               /*merge=*/recommitting_);
    settleCommand(Settled::Fillet, obj.id);
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
    if (refuseMeshEdit(*target, "Combining bodies") || refuseMeshEdit(*tool, "Combining bodies"))
        return;

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
    std::string why;
    if (!scene_.addFeature(targetId, std::move(f), &why)) {
        setNotice(std::string(booleanOpName(op)) +
                  (why.empty() ? " produced no valid solid" : " refused: " + why));
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
                base1.backend = piece1.isMesh() ? Backend::Mesh : Backend::Brep;
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
                base1.backend = piece1.isMesh() ? Backend::Mesh : Backend::Brep;
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
        base.backend = bodies.front().isMesh() ? Backend::Mesh : Backend::Brep;
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
        base1.backend = piece1.isMesh() ? Backend::Mesh : Backend::Brep;
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

    // A mesh can come apart into the pieces it already is, above, but is only
    // cut by a plane once it is a solid.
    if (refuseMeshEdit(*obj, "Cutting a body in two")) return;
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

// What the chooser should offer, and what it should be called. The filters are
// a courtesy on platforms that honour them and ignored on those that do not,
// which is why every list ends with everything.
namespace {
struct ChooserSpec {
    FileDialog::Kind kind;
    const char* title;
    std::vector<FileDialog::Filter> filters;
};

ChooserSpec chooserFor(int mode) {
    using K = FileDialog::Kind;
    switch (mode) {
        case 0: return {K::Open, "Open Project",
                        {{"Tangent project", "tangent"}, {"All files", "*"}}};
        case 1: return {K::Save, "Save Project",
                        {{"Tangent project", "tangent"}, {"All files", "*"}}};
        case 2: return {K::Save, "Export STL",
                        {{"STL mesh", "stl"}, {"All files", "*"}}};
        case 3: return {K::Save, "Export STEP",
                        {{"STEP file", "step;stp"}, {"All files", "*"}}};
        case 4: return {K::Open, "Import STEP",
                        {{"STEP file", "step;stp"}, {"All files", "*"}}};
        case 5: return {K::Open, "Import Mesh",
                        {{"Mesh", "stl;obj"}, {"STL", "stl"}, {"OBJ", "obj"},
                         {"All files", "*"}}};
        case 6: return {K::Save, "Export 3MF",
                        {{"3MF package", "3mf"}, {"All files", "*"}}};
    }
    // Only reached by a mode added without a line above.
    return {K::Open, "Choose a File", {{"All files", "*"}}};
}
} // namespace

void Application::showFileChooser(FileMode mode) {
    fileMode_ = mode;
    const ChooserSpec spec = chooserFor(static_cast<int>(mode) - 1);

    // Somewhere sensible to start: beside the current project if there is one,
    // and with the suggested name already filled in for a save. beginFilePrompt
    // has worked that out and left it in pathField_.
    fileDialog_.show(spec.kind, window_, spec.filters, pathField_);
}

void Application::pollFileDialog() {
    if (!fileDialog_.waiting()) return;

    std::string answer;
    switch (fileDialog_.take(answer)) {
        case FileDialog::Result::None:
            return;
        case FileDialog::Result::Cancelled:
            fileMode_ = FileMode::None;
            return;
        case FileDialog::Result::Chosen: {
            const FileMode mode = fileMode_;
            fileMode_ = FileMode::None;
            runFileOperation(mode, answer);
            return;
        }
        case FileDialog::Result::Failed:
            // No chooser on this system. Say so once, fall back to typing the
            // path, and do not try again this session -- a dialog that failed
            // for want of a portal will fail the same way every time.
            typePathInstead_ = true;
            setNotice("No file chooser is available here (" + answer +
                      "), so paths are typed instead");
            return;                      // fileMode_ stays set: the popup opens
    }
}

void Application::beginFilePrompt(FileMode mode) {
    fileMode_ = mode;

    // Seed with something sensible: the current project, or a default name
    // beside it, so the common case is one keystroke.
    std::string seed = projectPath_;
    auto withSuffix = [&](const char* fallback, const char* ext) {
        if (seed.empty()) { seed = fallback; return; }
        const size_t dot = seed.find_last_of('.');
        seed = (dot == std::string::npos ? seed : seed.substr(0, dot)) + ext;
    };
    if (mode == FileMode::ExportStl) {
        withSuffix("model.stl", ".stl");
    } else if (mode == FileMode::Export3mf) {
        withSuffix("model.3mf", ".3mf");
    } else if (mode == FileMode::ExportStep) {
        withSuffix("model.step", ".step");
    } else if (mode == FileMode::ImportStep || mode == FileMode::ImportMesh) {
        seed = "";                       // there is no sensible guess at a name
    } else if (seed.empty()) {
        seed = "untitled.tangent";
    }
    std::snprintf(pathField_, sizeof(pathField_), "%s", seed.c_str());

    // Mesh export has choices a native chooser cannot carry, so they are asked
    // first and the chooser comes after. Everything else goes straight to it.
    if (mode == FileMode::ExportStl || mode == FileMode::Export3mf) {
        exportOptionsOpen_ = true;
        return;
    }
    if (!typePathInstead_) showFileChooser(mode);
}

namespace {
// "parts/bracket.step" -> "bracket". What an imported body gets called, which
// beats "Object 4" when three files are open at once.
std::string fileStem(const std::string& path) {
    size_t a = path.find_last_of("/\\");
    a = (a == std::string::npos) ? 0 : a + 1;
    const size_t b = path.find_last_of('.');
    const std::string name = (b == std::string::npos || b <= a)
                                 ? path.substr(a)
                                 : path.substr(a, b - a);
    return name.empty() ? std::string("Imported") : name;
}
} // namespace

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
    case FileMode::ImportMesh: {
        Body body;
        const MeshImport r = readMesh(path, body);
        if (!r.ok) {
            setNotice(r.error.empty() ? "Import failed" : "Import failed: " + r.error);
            break;
        }
        const ObjectId id = scene_.addImportedBody(std::move(body), fileStem(path));
        if (id == kNoObject) { setNotice("That mesh could not be brought in"); break; }

        undo_.push(ExistenceCommand::forCreate(scene_, {id}));
        scene_.clearSelection();
        scene_.select(id);
        camera_.frame(scene_.bounds());

        // What Convert to Solid would do, said now rather than left to be found
        // out by trying it. Being closed is only half the question: a scanned
        // mesh is closed and still has nothing worth converting, because every
        // triangle sits on its own plane.
        std::string note = "Imported " + std::to_string(r.triangles) + " triangles from " + path;
        if (!r.closed) {
            note += "  (open surface: it cannot become a solid)";
        } else if (const SceneObject* o = scene_.find(id)) {
            const int faces = predictSolidFaces(o->body);
            char tail[120];
            if (faces > 0 && faces <= kSolidifyFaceLimit)
                std::snprintf(tail, sizeof tail,
                              "  (Convert to Solid would give %d faces)", faces);
            else
                std::snprintf(tail, sizeof tail,
                              "  (would be %d faces: Modify > Reduce Mesh before converting)",
                              faces);
            note += tail;
        }
        setNotice(note);
        break;
    }
    case FileMode::ExportStep: {
        // What goes out is every visible body, each in its own place: the
        // shapes are moved into world space first, because a STEP file has no
        // notion of an object transform sitting outside the geometry.
        std::vector<Body> placed;
        std::vector<const BrepShape*> shapes;
        for (const auto& o : scene_.objects()) {
            if (!o->visible || o->body.empty() || o->body.isMesh()) continue;
            Body b = o->body;
            b.transform(o->modelMatrix());
            if (!b.empty()) placed.push_back(std::move(b));
        }
        for (const Body& b : placed) shapes.push_back(&b.brep());

        const size_t meshes = std::count_if(
            scene_.objects().begin(), scene_.objects().end(),
            [](const std::unique_ptr<SceneObject>& o) {
                return o->visible && !o->body.empty() && o->body.isMesh();
            });

        std::string why;
        if (shapes.empty()) {
            setNotice(meshes > 0 ? "STEP holds surfaces, and every visible body is a mesh"
                                 : "Nothing to export");
        } else if (brep::writeStep(shapes, path, &why)) {
            std::string note = "Exported " + std::to_string(shapes.size()) +
                               (shapes.size() == 1 ? " body to " : " bodies to ") + path;
            // Said rather than silently dropped: a mesh has no surfaces to
            // write, and a file that is quietly missing a part is worse than
            // one that is missing a part you were told about.
            if (meshes > 0)
                note += "  (" + std::to_string(meshes) + " mesh " +
                        (meshes == 1 ? "body" : "bodies") + " left out)";
            setNotice(note);
        } else {
            setNotice(why.empty() ? "STEP export failed" : "STEP export failed: " + why);
        }
        break;
    }
    case FileMode::ImportStep: {
        std::vector<BrepRef> solids;
        std::string why;
        if (!brep::readStep(path, scene_.nextImportSalt(), solids, &why)) {
            setNotice(why.empty() ? "STEP import failed" : "Import failed: " + why);
            break;
        }

        // Each solid arrives as its own object with a BaseMesh root -- the
        // chain root for geometry that has no parameters behind it. It can be
        // modelled on from here like anything else; what it cannot do is tell
        // you how it was made, because the file does not know.
        std::vector<ObjectId> added;
        const std::string stem = fileStem(path);
        for (size_t i = 0; i < solids.size(); ++i) {
            Body b(std::move(solids[i]));
            std::string name = stem;
            if (solids.size() > 1) name += " " + std::to_string(i + 1);
            const ObjectId id = scene_.addImportedBody(std::move(b), name);
            if (id != kNoObject) added.push_back(id);
        }
        if (added.empty()) { setNotice("Nothing in that file could be brought in"); break; }

        undo_.push(ExistenceCommand::forCreate(scene_, added));
        scene_.clearSelection();
        for (ObjectId id : added) scene_.select(id, /*additive=*/true);
        camera_.frame(scene_.bounds());
        setNotice("Imported " + std::to_string(added.size()) +
                  (added.size() == 1 ? " body from " : " bodies from ") + path);
        break;
    }
    case FileMode::Export3mf: {
        ThreeMfOptions opt;
        opt.selectionOnly = exportSelectionOnly_;
        opt.deviationMm = exportDeviationMm_;
        const ThreeMfResult r = export3mf(scene_, path, opt);
        if (!r.ok) {
            setNotice("Export failed: " + r.error);
            break;
        }
        std::string note = "Exported " + std::to_string(r.objects) +
                           (r.objects == 1 ? " object, " : " objects, ") +
                           std::to_string(r.triangles) + " triangles, to " + path;
        if (r.meshBodies > 0)
            note += "  (" + std::to_string(r.meshBodies) +
                    (r.meshBodies == 1 ? " mesh body at its own resolution)"
                                       : " mesh bodies at their own resolution)");
        // Written, because a slicer can often mend it, but not quietly.
        if (r.openObjects > 0)
            note += "  (" + std::to_string(r.openObjects) +
                    (r.openObjects == 1 ? " object is not closed)" : " objects are not closed)");
        setNotice(note);
        break;
    }
    case FileMode::ExportStl: {
        StlOptions opt;
        opt.binary = exportBinaryStl_;
        opt.selectionOnly = exportSelectionOnly_;
        opt.separateFiles = exportSeparateStl_;
        opt.deviationMm = exportDeviationMm_;
        const StlResult r = exportStl(scene_, path, opt);
        if (r.ok) {
            std::string note = r.files.size() > 1
                ? "Exported " + std::to_string(r.files.size()) + " files, " +
                      std::to_string(r.triangles) + " triangles, beside " + path
                : "Exported " + std::to_string(r.triangles) + " triangles to " + r.files.front();
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
    // Two jobs, and which one depends on why we are here.
    //
    //   The mesh export options, always: selection or everything, the
    //   tolerance, and for STL binary or ASCII and one file or one per object.
    //   A native chooser has nowhere to put these, so they are asked first and
    //   the chooser follows the button.
    //
    //   A typed path, only where there is no chooser to be had. That is the
    //   whole of this dialog's former job and is now the fallback for a system
    //   with no XDG portal, no zenity and no kdialog.
    const bool typing = typePathInstead_ && fileMode_ != FileMode::None;
    if (!exportOptionsOpen_ && !typing) return;

    const char* title = fileMode_ == FileMode::Export3mf  ? "Export 3MF"
                      : exportOptionsOpen_                ? "Export STL"
                      : fileMode_ == FileMode::Open       ? "Open Project"
                      : fileMode_ == FileMode::Save       ? "Save Project"
                      : fileMode_ == FileMode::ExportStep ? "Export STEP"
                      : fileMode_ == FileMode::ImportStep ? "Import STEP"
                      : fileMode_ == FileMode::ImportMesh ? "Import Mesh"
                                                          : "Export STL";
    ImGui::OpenPopup(title);

    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->GetCenter().x, vp->GetCenter().y),
                            ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(460.0f, 0.0f));

    if (!ImGui::BeginPopupModal(title, nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        return;

    bool entered = false;
    if (typing) {
        ImGui::TextUnformatted("Path");
        ImGui::SetNextItemWidth(-1.0f);
        // Focused on open, and Enter confirms, so the whole thing is keyboard
        // driven without reaching for the mouse.
        if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
        entered = ImGui::InputText("##path", pathField_, sizeof(pathField_),
                                   ImGuiInputTextFlags_EnterReturnsTrue);
    }

    if (exportOptionsOpen_) {
        ImGui::Checkbox("Selection only", &exportSelectionOnly_);
        if (fileMode_ == FileMode::ExportStl) {
            ImGui::SameLine();
            ImGui::Checkbox("Binary", &exportBinaryStl_);
            ImGui::SameLine();
            ImGui::Checkbox("One file per object", &exportSeparateStl_);
        } else {
            ImGui::TextDisabled("each object is kept separate and named in the file");
        }

        ImGui::SetNextItemWidth(140.0f);
        ImGui::DragFloat("Tolerance", &exportDeviationMm_, 0.001f, 0.001f, 0.5f,
                         "%.3f mm");
        ImGui::SameLine();
        ImGui::TextDisabled("how far a triangle may sit from the surface");
        ImGui::Spacing();
    }

    ImGui::Spacing();
    // The chooser is the way out when there is one; typing is the way out when
    // there is not. Only ever one of them, so the button says which.
    const bool chooseInstead = exportOptionsOpen_ && !typePathInstead_;
    const bool confirm =
        ImGui::Button(chooseInstead ? "Choose File..." : "OK", ImVec2(130, 0)) || entered;
    ImGui::SameLine();
    const bool cancel = ImGui::Button("Cancel", ImVec2(90, 0)) ||
                        ImGui::IsKeyPressed(ImGuiKey_Escape, false);

    if (confirm) {
        const FileMode mode = fileMode_;
        const bool toChooser = chooseInstead;
        exportOptionsOpen_ = false;
        ImGui::CloseCurrentPopup();
        if (toChooser) {
            showFileChooser(mode);       // the options are set; now pick a file
        } else {
            fileMode_ = FileMode::None;
            runFileOperation(mode, pathField_);
        }
    } else if (cancel) {
        exportOptionsOpen_ = false;
        fileMode_ = FileMode::None;
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
}

void Application::applyActions() {
    UiActions& a = ui_.actions;

    // Anything that changes the model, or what the model is, is the start of
    // the next thing. The gestures put their own panel away in begin*; this
    // catches the commands that are not gestures.
    if (a.addRequested || a.sketch || a.editSketchObject != kNoObject ||
        a.deleteSelected || a.duplicateSelected || a.mergeFaces ||
        a.booleanRequested || a.split || a.shell || a.inset || a.undo || a.redo ||
        a.importStep || a.importMesh || a.convertToSolid || a.reduceMesh ||
        a.newProject || a.openProject || a.rebuildObject != kNoObject ||
        a.transformEdited != kNoObject || a.featuresEdited != kNoObject)
        dismissSettled();

    if (a.quit && confirmDiscard(PendingAction::Quit)) running_ = false;

    if (a.newProject && confirmDiscard(PendingAction::New))   newProject();
    if (a.openProject && confirmDiscard(PendingAction::Open)) beginFilePrompt(FileMode::Open);
    if (a.saveProjectAs) beginFilePrompt(FileMode::Save);
    if (a.exportStl)     beginFilePrompt(FileMode::ExportStl);
    if (a.export3mf)     beginFilePrompt(FileMode::Export3mf);
    if (a.exportStep)    beginFilePrompt(FileMode::ExportStep);
    if (a.importStep)    beginFilePrompt(FileMode::ImportStep);
    if (a.importMesh)    beginFilePrompt(FileMode::ImportMesh);
    if (a.convertToSolid) convertSelectedToSolid();
    if (a.reduceMesh) beginReduce();
    if (a.saveProject) {
        // Save straight over the current file; prompt only the first time.
        if (projectPath_.empty()) beginFilePrompt(FileMode::Save);
        else                      runFileOperation(FileMode::Save, projectPath_);
    }

    if (a.undo) { dismissSettled(); undo_.undo(scene_); }
    if (a.redo) { dismissSettled(); undo_.redo(scene_); }

    if (a.addRequested) {
        beginAddPrimitivePrompt(a.addKind);
    }
    if (a.sketch) beginSketch();
    if (a.editSketchObject != kNoObject) beginEditSketch(a.editSketchObject, a.editSketchUid);

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
                const Attempt attempt = tryIsolated(chainTrial(o->features));
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
    if (a.scaleFace)  beginFaceMove(FaceOp::Scale);
    if (a.divide) beginDivide();
    if (a.pattern) beginPattern(PatternMode::Linear);
    if (a.mirror) beginPattern(PatternMode::Mirror);
    if (a.mergeFaces) mergeSelected();
    if (a.bevel)   roundAllEdges();
    if (a.split)   splitActiveObject();
    if (a.fillet)  beginFillet();
    if (a.shell)   shellActiveObject();
    if (a.inset)   insetSelectedFaces();
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
    pollFileDialog();
    drawFilePrompt();
    drawUnsavedPrompt();
    drawMeasurePanel(ui_);
    drawMeasureLabel();
    drawTransformReadout();
    drawFilletPanel();
    drawFacePanel();
    drawDividePanel();
    drawPatternPanel();
    drawReducePanel();
    if (sketchTool_.active()) {
        sketchTool_.setHudOrigin(viewRect_.x + 16.0f, viewRect_.y + 16.0f);
        sketchTool_.setViewportOrigin(viewRect_.x, viewRect_.y);
        bool finished = false;
        sketchTool_.drawHud(scene_, camera_, undo_, finished);
        if (finished) justFinishedModal_ = true;
    }
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
        } else if (reduceTool_.active) {
            char buf[160];
            std::snprintf(buf, sizeof buf, "Reduce Mesh  within %.3g mm%s   type a number   Enter confirm   Esc cancel",
                          static_cast<double>(reduceTool_.tolerance),
                          reduceTool_.preview.busy() ? "   reducing..." : "");
            ui_.toolStatus = buf;
        } else if (patternTool_.active) {
            char buf[160];
            if (patternTool_.mode == PatternMode::Mirror)
                std::snprintf(buf, sizeof buf,
                              "Mirror  plane at %.2f mm   X Y Z plane   Click confirm   Esc cancel",
                              patternTool_.offset);
            else
                std::snprintf(buf, sizeof buf,
                              "%s  %d x  %.2f %s apart   Click confirm   Esc cancel",
                              patternModeName(patternTool_.mode), patternTool_.count,
                              patternTool_.dragged(),
                              patternTool_.mode == PatternMode::Circular ? "deg" : "mm");
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
        } else if (sketchTool_.active()) {
            switch (sketchTool_.stage()) {
            case SketchStage::SelectPlane:
                ui_.toolStatus = "Sketch: click an origin plane or a face  [7 top, 1 front, 3 right]   Esc cancel";
                break;
            case SketchStage::Draw:
                ui_.toolStatus = std::string("Sketch: ") + sketchModeName(sketchTool_.mode()) +
                                 "   L R C A D tools   X delete   E extrude   Esc cancel";
                break;
            case SketchStage::Regions:
                ui_.toolStatus = "Extrude: click the regions to sweep, Enter for the depth   Esc back";
                break;
            case SketchStage::Depth:
                ui_.toolStatus = std::string("Extrude: move to set the depth, ") +
                                 createOpName(sketchTool_.resolvedOp()) +
                                 "   A auto  J join  D cut  N new body   click finish   Esc back";
                break;
            case SketchStage::None:
                break;
            }
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

        // What a printer would make of the part, when nothing else is being
        // said. Only the object in hand: a report on everything at once is a
        // report nobody reads.
        if (ui_.toolStatus.empty() && view_.showPrintIssues) {
            if (const SceneObject* o = scene_.find(scene_.contextObject()))
                if (o->printVersion == o->meshVersion)
                    ui_.toolStatus = summarise(o->printCheck);
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
        if (std::string sketchErr = sketchTool_.takeError(); !sketchErr.empty())
            setNotice(sketchErr);

        applyActions();

        // A feature that dropped out of the chain during any of the above.
        // Drained here, after the actions have run, because a re-evaluation is
        // triggered from a dozen places -- an inspector nudge, a history
        // toggle, an undo -- and none of them should have to remember to say so.
        if (std::string chainErr = scene_.takeChainNotice(); !chainErr.empty())
            setNotice(chainErr);

        // Queued before the frame is drawn; the renderer flushes overlay lines
        // at the end of its pass.
        drawPrintIssues();
        drawSelectionHighlights();
        if (createTool_.active()) createTool_.drawOverlay(scene_, camera_, renderer_);
        if (sketchTool_.active()) sketchTool_.drawOverlay(scene_, camera_, renderer_);
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
        if (patternTool_.active && patternTool_.axis.valid) {
            const Real step = DragAxis::stepFor(camera_, patternTool_.axis.origin,
                                                patternTool_.axis.spanValue);
            patternTool_.axis.drawGuide(renderer_, camera_, patternTool_.dragged(), step,
                                        patternTool_.axis.spanValue);
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
