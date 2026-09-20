// Tangent - application shell: window, event loop, input and tool dispatch.
#pragma once

#include "app/camera.h"
#include "app/drag_axis.h"
#include "app/async_build.h"
#include "mesh/decimate.h"
#include "geom/kernel_guard.h"
#include "app/create_tool.h"
#include "app/extrude_ops.h"
#include "app/sketch_tool.h"
#include "app/file_dialog.h"
#include "app/printability.h"
#include "app/measure.h"
#include "mesh/export_stl.h"
#include "scene/serialize.h"
#include "app/transform_tool.h"
#include "geom/operations.h"
#include "app/undo.h"
#include "render/renderer.h"
#include "scene/scene.h"
#include "ui/panels.h"

#include <atomic>
#include <future>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

struct SDL_Window;
union SDL_Event;

namespace tg {

class Application {
public:
    bool init();
    int  run();
    void shutdown();

    // Renders `frames` and exits; used to smoke-test startup non-interactively.
    void setSmokeTest(int frames) { smokeFrames_ = frames; }

    // Keeps the operating system's own title bar and borders instead of the
    // bar the application draws. For a desktop that cannot move or resize a
    // window from inside it, or for anyone who prefers the system's frame.
    void setNativeFrame() { nativeFrame_ = true; }

    // Where a point of the window is, for the window manager: a handle to
    // drag the window by, an edge to resize it from, or an ordinary spot. Only
    // consulted when the application draws its own frame.
    int frameHitTest(int x, int y) const;

    // Places the camera explicitly. Makes captures reproducible, which is what
    // lets grid behaviour be compared across viewing angles.
    void setCamera(float yawDeg, float pitchDeg, float dist) {
        camera_.yaw = radians(yawDeg);
        camera_.pitch = radians(pitchDeg);
        if (dist > 0.0f) camera_.distance = dist;
        camera_.snapToGoal();
        fixedCamera_ = true;
    }

    // Starts with no objects, so a measurement sees only the grid.
    void setStartEmpty() { startEmpty_ = true; }
    void setNoGrid() { view_.showGrid = false; }

    // Sweeps the camera through a yaw range, printing the mean luminance of
    // the viewport at each step. A grid that is stable under rotation produces
    // a smooth curve; popping or breathing lines show up as high-frequency
    // steps, which tests/grid_stability.py checks for numerically.
    void setGridProbe(float yaw0Deg, float yaw1Deg, int steps) {
        probeYaw0_ = yaw0Deg;
        probeYaw1_ = yaw1Deg;
        probeSteps_ = steps > 1 ? steps : 2;
        probeActive_ = true;
    }

    // Checks that grid lines are drawn at the world coordinates they belong to.
    // Samples the rendered image at points known to lie exactly on a major grid
    // line, and at control points deliberately off every line, for each yaw in
    // a sweep. A grid anchored to the world keeps the on-line samples bright at
    // every angle; one whose phase drifts with the camera does not.
    void setGridAlign(float yaw0Deg, float yaw1Deg, int steps) {
        probeYaw0_ = yaw0Deg;
        probeYaw1_ = yaw1Deg;
        probeSteps_ = steps > 1 ? steps : 2;
        probeActive_ = true;
        alignProbe_ = true;
    }

    // Pre-selects a face of the first object. Diagnostic only: it makes the
    // selection highlight and the mesh operations reproducible in a capture,
    // which a click cannot be.
    void setPickFace(int index) { pickFace_ = index; }
    void setMeasureDemo() { measureDemo_ = true; }
    void setFileDemo(int mode) { fileDemo_ = mode; }
    void setHeadlessExport(const std::string& p) { headlessExport_ = p; }
    void setHeadlessExport3mf(const std::string& p) { headlessExport3mf_ = p; }
    // Opens the Combine dialog on the startup box and a cylinder through it:
    // 0 join, 1 cut, 2 intersect. 3 to 5 the same, finished, so the applied
    // panel shows.
    void setBooleanDemo(int op) { booleanDemo_ = op; }

    // The reported case and what came of it: a cylinder stretched from the
    // inspector and cut from the box, then the box moved, turned and scaled --
    // every one of them a row in its history.
    void setHistoryDemo() { historyDemo_ = true; }

    // A second box beside the startup box with its top in the same plane, and
    // the place where the two tops overlap clicked `clicks` times: each click
    // takes the next of the coplanar faces, and the one selected is drawn in
    // front.
    void setCoplanarDemo(int clicks) { coplanarDemo_ = clicks; }
    void setFilletEdgesDemo(int edges) { filletEdgesDemo_ = true; filletDemoEdges_ = edges; }
    // Round All Edges on the startup box, committed at 2mm, then the same
    // command on a mesh, which has to refuse and say how to get past it.
    void setRoundAllDemo() { roundAllDemo_ = true; }
    void setAutoExtrude(float mm) { autoExtrude_ = true; autoExtrudeMm_ = mm; }
    void setShellDemo(float wallMm) { shellDemo_ = wallMm; }
    void setInsetDemo(float mm) { insetDemo_ = mm; }
    // 1 cuts through the middle, 2 moves the plane, 3 cuts by another axis.
    void setSplitDemo(int step) { splitDemo_ = step; }

    // Shell, then extrude what is left of the face that was opened -- the
    // sequence that crashed. Kept because it crossed three subsystems that had
    // each been tested on their own.
    void setShellExtrudeDemo(float mm) { shellExtrudeDemo_ = mm; }

    // Shell the top, then press fillet with a side face selected -- the second
    // way the crash was reached.
    void setShellFilletDemo(bool on) { shellFilletDemo_ = on; }

    // Drives the create tool to a position that should catch a snap, so the
    // indicator and the reason can be seen rather than argued about.
    // `mode`: 0 nothing, 1 in line with the hole, 2 where two lines cross,
    // 3 on a grid crossing, 4 on the hole's centre.
    void setSnapDemo(int mode) { snapDemo_ = mode; }

    // Drives the sketch tool to a step so it can be looked at: 1 drawing, with
    // a line half-drawn, 2 choosing regions, 3 setting the depth, 4 the part it
    // made, re-opened for editing.
    void setSketchDemo(int step) { sketchDemo_ = step; }
    // Turns a profile about an axis, the whole way through the tool: 1 a ring
    // as a new part, 2 the same as a part turn, 3 a groove cut round a
    // cylinder. Each prints the volume it made, against the one arithmetic
    // says it should be.
    void setRevolveDemo(int step) { revolveDemo_ = step; }
    // Drills a hole the way the tool drills one: 1 an M3 clearance hole
    // through a plate, 2 an M4 counterbore adjusted to M5 in its panel,
    // 3 a blind tapped hole whose face then moves under it.
    void setHoleDemo(int step) { holeDemo_ = step; }
    // Leans the walls of a box: 1 three degrees off the bed, 2 the same
    // adjusted to six in the panel, 3 widest in the middle.
    void setDraftDemo(int step) { draftDemo_ = step; }
    // An SVG onto the top plane: 1 placed in the sketch, 2 its filled regions
    // picked, 3 extruded 3 mm, 4 cut 3 mm into a plate under it, 5 with the
    // busiest face of the result selected -- what the highlight costs to draw.
    void setSvgDemo(const std::string& path, int step) { svgDemo_ = path; svgDemoStep_ = step; }
    // Print where each frame went, every `frames` frames. For finding a stall,
    // and for showing one is gone.
    void setFrameProbe(int frames);

    // Drives the create tool to a given step so the handles and the dimension
    // row can be looked at: 1 adjusting a profile, 2 rounding a corner,
    // 3 setting the depth.
    void setProfileDemo(int mode) { profileDemo_ = mode; }

    // Starts a fillet and leaves it running, so the panel and the arrow can be
    // looked at rather than only the result.
    void setFilletOpen() { filletOpen_ = true; }

    // Drives the face tools: 1 push out, 2 pull in, 3 rotate, 4 divide,
    // 5 divide then push one half -- the sequence the divide exists for -- and
    // 6 the same pull as an extrude, which keeps the boss's outline,
    // 11 a face grown by half and 12 one shrunk,
    // 7 a move along a world axis, 8 a rotation the other way, 9 an extrude
    // that runs into another body, and 10 divide, move, then merge.
    void setFaceDemo(int mode) { faceDemo_ = mode; }
    void setPatternDemo(int mode) { patternDemo_ = mode; }
    void setStepDemo(const std::string& path) { stepDemo_ = path; }
    void setDialogDemo(int mode) { dialogDemo_ = mode; }
    void setMeshBench(const std::string& path) { meshBench_ = path; }
    void setReduceDemo(const std::string& path) { reduceDemo_ = path; }

    // Throws a fast, wandering drag at the extrude, including the places a
    // hand actually goes: the corners of the window, and straight through the
    // start. Looking for the value the gesture computes, not for a pretty
    // picture.
    void setFaceStress() { faceStress_ = true; }

    // A part with something wrong with it, printing-wise: 1 a wall too thin,
    // 2 a face leaning too far, 3 both.
    void setPrintDemo(int mode) { printDemo_ = mode; }

    // Rounds a selection, then compares what the preview showed against what
    // the chain rebuilt. They are meant to be the same body.
    void setPreviewCheck(int mode) { previewCheck_ = mode; }

    // Parks the interface's pointer somewhere, for screenshots of hover states.
    void setUiMouse(float x, float y, bool down) {
        uiMouse_ = {x, y};
        uiMouseDown_ = down;
    }

    // Rounds one edge of a box and then the edge opposite it, through the same
    // path the keyboard takes, and reports whether the part came out symmetric.
    // Two opposite edges rounded to the same radius can only give a symmetric
    // body, so asymmetry means the second fillet landed somewhere else.
    void setFilletDemo(float radiusMm) { filletDemo_ = radiusMm; }
    void setHoldTransform() { holdTransform_ = true; }

    // Writes the viewport to a PPM after `afterFrames` frames. Reads back this
    // process's own GL framebuffer rather than going through the compositor, so
    // it captures only Tangent and works regardless of what else is on screen.
    void setScreenshot(const std::string& path, int afterFrames) {
        screenshotPath_ = path;
        screenshotFrame_ = afterFrames;
    }

private:
    void handleEvent(const SDL_Event& e);
    void handleViewportMouse();
    void handleShortcuts();
    void handleTransformKeys();
    Vec2 mouseInViewport() const;

    // Set only by the headless demos, which have no pointer of their own.
    Vec2 mouseOverride_{-1.0, -1.0};
    int  snapDemo_ = 0;
    int  sketchDemo_ = 0;
    int  revolveDemo_ = 0;
    int  holeDemo_ = 0;
    int  draftDemo_ = 0;
    std::string svgDemo_;
    int  svgDemoStep_ = 1;
    int  svgDemoFrames_ = 0;
    double svgDemoOverlayMs_ = 0.0;
    double svgDemoUpdateMs_ = 0.0;
    double svgDemoOverlaySum_ = 0.0;
    int  profileDemo_ = 0;
    bool profileDemoDone_ = false;
    bool filletOpen_ = false;
    bool filletOpenDone_ = false;
    int  faceDemo_ = 0;
    int  patternDemo_ = 0;
    std::string stepDemo_;
    bool stepDemoDone_ = false;
    int  dialogDemo_ = 0;
    bool dialogDemoDone_ = false;
    std::string meshBench_;
    bool meshBenchDone_ = false;
    void stepMeshBench();
    std::string reduceDemo_;
    bool reduceDemoDone_ = false;
    void stepReduceDemo();
    std::string writeDrilledPlate();

    // Runs the print check on one object and gathers the triangles it flagged,
    // so drawing them later costs those triangles and nothing else. Synchronous;
    // the demos and the benchmark want the answer before the next line.
    void refreshPrintCheck(SceneObject& o, const PrintProfile& profile = {});

    // The same work, off the frame thread. On a 62,000-triangle import the check
    // is most of a tenth of a second, and on the frame thread that is a stall
    // the moment the model appears. Here the model appears, and the red faces
    // follow a few frames later -- a report on the part is not something anyone
    // is waiting on to keep working.
    struct PrintJobResult {
        PrintReport       report;
        std::vector<Vec3> triangles;
    };
    struct PrintJob {
        uint32_t meshVersion = 0;
        std::future<PrintJobResult> result;
    };
    std::unordered_map<ObjectId, PrintJob> printJobs_;

    // Jobs for geometry that has since changed. Not destroyed on the spot: the
    // future std::async returns blocks in its destructor until its task is
    // done, so throwing one away would stall the frame thread on exactly the
    // work that was moved off it. They wait here and are dropped once finished.
    std::vector<std::future<PrintJobResult>> retiredPrintJobs_;
    void retirePrintJob(ObjectId id);

    // Checking the body is closed and does not cross itself, off the frame
    // thread: a fifth of a second on a part of a few thousand faces, which on
    // the frame thread is a hitch after every edit. The panel says "checking"
    // until the answer lands.
    struct HealthJob {
        uint32_t geometryVersion = 0;
        std::future<MeshHealth> result;
    };
    std::unordered_map<ObjectId, HealthJob> healthJobs_;
    std::vector<std::future<MeshHealth>> retiredHealthJobs_;
    void stepHealthCheck();

    // What the selection highlight draws, in each body's own space: the tinted
    // triangles of a selected face and the lines along its edges. Gathering it
    // asks the kernel for a polyline per edge and searches the body's triangles
    // per face -- on a face with two thousand edges that was seconds a frame,
    // so it is gathered when the selection, the geometry or the zoom changes
    // and drawn from here until one of them does.
    struct Highlight {
        ObjectId object = kNoObject;
        std::vector<Vec3> lines;   // pairs
        std::vector<Vec3> tris;    // triples
    };
    std::vector<Highlight> highlights_;
    uint64_t highlightKey_ = 0;
    void refreshHighlights();

    // Drawing a body finer, off the frame thread, for the same reason. Meshing
    // a body of a few thousand faces is hundreds of milliseconds: done on the
    // frame thread, every step of a zoom froze the view for half a second.
    // The body keeps the triangles it has until the finer ones arrive.
    struct MeshJobResult {
        RenderMesh mesh;
        Real deviation = 0.0;
    };
    struct MeshJob {
        uint32_t geometryVersion = 0;
        Real deviation = 0.0;
        std::future<MeshJobResult> result;
    };
    std::unordered_map<ObjectId, MeshJob> meshJobs_;
    std::vector<std::future<MeshJobResult>> retiredMeshJobs_;
    // Collects finished ones and starts what the view now wants.
    void stepTessellation();

    // Pure: everything it reads is passed in, which is what lets it run on a
    // worker against a snapshot while the scene carries on changing.
    static PrintJobResult runPrintCheck(const Body& body, const RenderMesh& rm,
                                        const PrintProfile& profile);
    void stepDialogDemo();
    void stepStepDemo();
    bool patternDemoDone_ = false;
    void stepPatternDemo();
    bool faceDemoDone_ = false;
    int  printDemo_ = 0;
    bool printDemoDone_ = false;
    int  previewCheck_ = 0;
    bool previewCheckDone_ = false;
    bool faceStress_ = false;
    bool faceStressDone_ = false;
    int  faceStressFrame_ = 0;
    int  faceStressCrossings_ = 0;
    int  faceStressBad_ = 0;
    Real faceStressWorst_ = 0.0;
    Vec2 uiMouse_{-1.0f, -1.0f};
    bool uiMouseDown_ = false;
    void beginTransform(TransformMode mode);
    void handleViewportClick(bool shift, bool ctrl);
    void drawPrintIssues();
    void drawSelectionHighlights();
    // A small value box drawn in the foreground, at a position given in
    // window pixels. Both the transform readout and the measure label use it,
    // so they always look like the same piece of interface.
    void drawReadout(const std::string& text, float px, float py, bool emphasise);
    void drawMeasureLabel();
    void drawTransformReadout();

    // The arrows the gestures are pulled along, drawn on the screen.
    void drawDragGuides();

    // File handling. There is no native dialog to call on Wayland without
    // taking a dependency, so the prompt is an in-app path field.
    // The values are what --file-prompt takes, so a new mode goes at the end.
    enum class FileMode { None, Open, Save, ExportStl, ExportStep, ImportStep,
                          ImportMesh, Export3mf, ImportSvg };

    // Turns the selected mesh body into an exact one. Its own command rather
    // than something the import does on its own: a mesh that will not convert
    // is still worth having on screen, and a mesh that converts badly is worth
    // seeing before it replaces what you had.
    void convertSelectedToSolid();
    void drawFilePrompt();
    void beginFilePrompt(FileMode mode);
    void runFileOperation(FileMode mode, const std::string& path);
    void newProject();

    // What to do once the user has answered the unsaved-work prompt.
    enum class PendingAction { None, New, Open, Quit };
    void drawUnsavedPrompt();
    bool confirmDiscard(PendingAction next);

    PendingAction pending_ = PendingAction::None;
    size_t        savedRevision_ = 0;
    bool          dirty() const { return undo_.revision() != savedRevision_; }

    FileMode    fileMode_ = FileMode::None;

    // The operating system's chooser, and what is needed to fall back to a
    // typed path when there is not one. SDL gives no way to ask in advance
    // whether a chooser can be shown -- the failure comes back through the
    // same callback as a chosen file -- so the first refusal is what tells us,
    // and after it the typed prompt is used for the rest of the session.
    FileDialog  fileDialog_;
    bool        typePathInstead_ = false;

    // Set while the mesh export options are up, for STL or 3MF. Export has
    // choices a native chooser cannot carry -- the tolerance, what to include,
    // how to split it into files -- so they are asked first and the chooser
    // follows.
    bool        exportOptionsOpen_ = false;

    // Starts the OS chooser for `mode`. Returns having asked; the answer is
    // collected by pollFileDialog on a later frame.
    void showFileChooser(FileMode mode);
    void pollFileDialog();
    std::string projectPath_;
    char        pathField_[512] = {};
    bool        exportBinaryStl_ = true;
    bool        exportSelectionOnly_ = false;
    bool        exportSeparateStl_ = false;

    // The chord tolerance the exported triangles must stay within. Finer than
    // any filament printer resolves, and the number a user should be able to
    // argue with -- so it is offered rather than assumed.
    float       exportDeviationMm_ = 0.01f;
    void shellActiveObject();

    // Insets the selected faces: a smaller copy of the face in its own plane,
    // with a ring of new faces around it. Implemented on both backends since
    // the first milestone and, until now, callable from nowhere.
    void insetSelectedFaces();

    // Rounds the selected edges only, the way F does in Fusion.
    void filletSelectedEdges();
    // `picked` is the body the edge handles were taken from, which during a
    // fillet is the body as it was *before* the preview replaced it. Passing it
    // rather than reading obj.body is the whole of the fix for a fillet landing
    // on a different edge than the one previewed: handles do not survive an
    // edit, and the preview is an edit.
    bool extendLastFillet(SceneObject& obj, Real radius);

    // Modal interactive fillet: the radius is pulled out along an axis.
    //
    // No segment count. A fillet on an exact body is a surface, not an
    // approximation of one, and the kernel was ignoring the number outright --
    // a control that moved and changed nothing.
    struct FilletToolState {
        bool active = false;
        ObjectId objectId = kNoObject;
        std::vector<Index> edges;
        Real baseRadius = 1.0;
        Real currentRadius = 1.0;

        // The direction the radius grows along, and the edge it is anchored
        // to. Rebuilt as the cursor moves between edges of a chain so the guide
        // follows the pointer rather than sitting on whichever edge came first.
        DragAxis axis;

        // The largest radius that builds, found once when the gesture starts.
        //
        // It used to be rediscovered every frame by bisecting whenever the
        // asked-for radius failed, so near the limit the preview flickered
        // between two answers and the number under the cursor jittered. The
        // limit is a property of the geometry, not of where the pointer is:
        // finding it once costs a dozen builds at the start and nothing after.
        Real maxRadius = 0.0;

        // The search for that limit, running in another process while this one
        // keeps drawing. Until it finishes, maxRadius is the largest radius
        // that has actually been verified, so the track only ever offers travel
        // that is known to work -- it grows as the answer narrows rather than
        // being guessed at and taken back.
        struct LimitSearch {
            bool active = false;

            // The floor comes first: the smallest radius this selection will
            // take at all, walked up a ladder. Until it lands there is nothing
            // verified to preview, so the gesture draws its guide and waits --
            // three frames, against the fifty milliseconds the same trial cost
            // when it ran on the click.
            bool floorPhase = false;
            int  floorIndex = 0;

            Real good = 0.0;      // verified to build
            Real bad = 0.0;       // verified not to, or assumed so
            Real ceiling = 0.0;   // the material behind the faces it runs along
            Real hardCeiling = 0.0;  // the size of the body; nothing beats it
            int  stepsLeft = 0;
            Real pending = 0.0;
            bool testedTop = false;
            AsyncTrial trial;
        };
        LimitSearch search;

        // What the preview currently shows, so a frame that asks for the same
        // thing again can be skipped rather than rebuilt.
        bool previewValid = false;

        // What the kernel has been asked for, which runs ahead of what it has
        // finished: the number and the guide follow the cursor every frame and
        // the geometry catches up a build later.
        Real requestedRadius = -1.0;
        AsyncBuild preview;

        // What the gesture will actually build, decided once when it starts.
        //
        // Two edges rounded in one operation blend their shared corner against
        // the original faces; rounded one after the other, the second has to
        // cut into the first one's surface, which is a harder problem and often
        // refused. So when the step before this one is a fillet, the new edges
        // are folded into it and the whole lot is rebuilt from the body that
        // fillet ran on.
        //
        // That is a different operation from rounding on top of what is on
        // screen -- a different shape, and one that can fail where the other
        // succeeds. It used to be chosen at the moment of committing, so the
        // preview showed one thing and the click produced another. Now it is
        // chosen first and everything -- the preview, the search for the
        // largest radius, the commit -- builds from it.
        bool folding = false;
        size_t foldAt = 0;                   // the feature being extended
        Body buildBase;                      // what the fillet is applied to
        std::vector<Index> buildEdges;       // indices into buildBase
        std::vector<Real> fixedRadii;        // < 0 means "whatever is dragged"

        // A flat cut instead of a round, and a radius that tapers along the
        // edge. Both are the same gesture with a different surface at the end
        // of it, so they belong to the tool rather than to a tool of their own.
        bool chamfer = false;
        Real endRadius = -1.0;         // negative: the radius holds all the way

        Body meshBefore;
        std::vector<Feature> chainBefore;
        std::string typedValue;
    };
    FilletToolState filletTool_;

    // ---- Moving a face -----------------------------------------------------
    //
    // Direct modelling: the body stays one body and only the face the user
    // pointed at moves, with the ones around it stretching to follow. Push and
    // pull along the normal is what "extrude" means on an existing face, and
    // rotate tips the face about one of its own edges.
    // Three, and the first two are not the same operation said twice.
    //
    //   Move      moves the face, and the body follows. Along its own normal
    //             unless X, Y or Z says otherwise. Out adds material and in
    //             takes it, which is not a choice to be made but a description
    //             of what moving a face does. This is what G means when a face
    //             is selected.
    //
    //   Extrude   grows a boss off the face and leaves its outline drawn, so
    //             the new part is something you can point at and act on
    //             afterwards. E.
    //
    //   Rotate    tips the face about an edge, either way. R when a face is
    //             selected.
    //
    //   Scale     grows or shrinks the face in its own plane and the faces
    //             around it slant to follow. S when a face is selected. Its
    //             number is a percentage rather than a multiple so that the
    //             gesture is the same as the other two: nothing at the start,
    //             either way from there.
    enum class FaceOp { Move, Extrude, Rotate, Scale };

    struct FaceToolState {
        bool     active = false;
        FaceOp   op = FaceOp::Move;
        ObjectId objectId = kNoObject;
        std::vector<FaceId> faces;

        Real value = 0.0;            // millimetres, or degrees for a rotate
        Real requested = 1e30;       // what the kernel was last asked for
        bool previewValid = false;

        // The furthest the kernel has actually managed, either way. A rotation
        // runs out long before ninety degrees on most shapes, and the honest
        // limit is the one that was reached rather than one guessed at: a
        // refused build pins the travel where it last worked.
        Real reachedMax = 1e30, reachedMin = -1e30;

        DragAxis axis;
        Vec3 direction{};                // what a move sweeps along
        int  lockedAxis = -1;            // 0/1/2 for X/Y/Z, -1 for the normal
        Vec3 hingePoint{}, hingeDir{};   // rotate only

        // The faces this began on, by name, so they can be found again in each
        // rebuilt preview and stay selected throughout.
        ElementRefs names;

        // The other body this one has grown into, if any, and what to do about
        // it. Nothing is asked until the material actually meets something:
        // a tool that offers to combine on every extrude is asking a question
        // the answer to which is almost always "no".
        ObjectId  meets = kNoObject;
        BooleanOp meetOp = BooleanOp::Union;
        bool      combineWithMeet = false;

        // Extrude only: what it does, and to which bodies -- see
        // app/extrude_ops.h. The swept solid is kept, in the world, for the
        // bodies it reaches and for drawing when it is not part of this body.
        ExtrudeChoice choice;
        ExtrudeReach  reach;
        std::string   toolKey;
        Body          tool;
        RenderMesh    toolMesh;

        Body before;
        std::vector<Feature> chainBefore;
        std::string typedValue;
        AsyncBuild preview;

        // The worker owns a thread, so it cannot be copied or moved: a fresh
        // gesture clears the fields around it rather than replacing the whole
        // state wholesale.
        void reset() {
            preview.cancel();
            objectId = kNoObject;
            faces.clear();
            names = ElementRefs{};
            value = 0.0;
            requested = 1e30;
            previewValid = false;
            reachedMax = 1e30;
            reachedMin = -1e30;
            meets = kNoObject;
            meetOp = BooleanOp::Union;
            combineWithMeet = false;
            choice.reset();
            reach.clear();
            toolKey.clear();
            tool = Body();
            toolMesh.clear();
            axis = DragAxis{};
            direction = hingePoint = hingeDir = Vec3{};
            lockedAxis = -1;
            before = Body();
            chainBefore.clear();
            typedValue.clear();
        }
    };
    FaceToolState faceTool_;

    void beginFaceMove(FaceOp op);

    // Points the gesture along a world axis instead of the face's own normal,
    // or back at the normal when the same key is pressed twice.
    void setFaceAxis(int axis);
    // The way a face move goes, in the body's own space: the axis the panel
    // locked, or a zero vector for the face's own normal. The preview, the
    // swept tool and the feature all have to be given the same answer, or what
    // is shown is not what is built.
    Vec3 faceAlongLocal(const SceneObject& obj) const;

    // Re-finds the faces the gesture began on in whatever the preview last
    // built, and leaves them selected. Without it the highlight wanders onto
    // the neighbours as the shape changes under it.
    void refreshFaceSelection();
    void updateFaceMove(bool snap, bool follow = true);
    void commitFaceMove();
    void abortFaceMove();
    void drawFacePanel();

    // ---- Combining bodies -----------------------------------------------------
    //
    // Fusion's Combine: one body to keep -- the target -- and any number of
    // tools, joined to it, cut from it, or intersected with it. It opens on
    // whatever is selected, the first body picked as the target, and bodies
    // clicked in the view while it is open go in or come out as tools. Each
    // tool is its own step in the target's history, so the history says what
    // cut what; the tools go unless Keep Tools says they stay.
    struct CombineToolState {
        bool active = false;
        ObjectId target = kNoObject;
        std::vector<ObjectId> tools;
        std::vector<std::string> toolNames;      // for the panel once they are gone
        std::string targetName;
        BooleanOp op = BooleanOp::Union;
        bool keepTools = false;

        // The target as it was, so the preview can be shown on it and put back.
        Body before;
        std::vector<Feature> chainBefore;
        std::vector<ObjectId> hidden;            // tools hidden while the result is shown
        std::string previewKey;
        std::string previewError;
        AsyncBuild preview;

        void reset() {
            preview.cancel();
            active = false;
            target = kNoObject;
            tools.clear();
            toolNames.clear();
            targetName.clear();
            op = BooleanOp::Union;
            keepTools = false;
            before = Body();
            chainBefore.clear();
            hidden.clear();
            previewKey.clear();
            previewError.clear();
        }
    };
    CombineToolState combineTool_;

    void beginCombine(BooleanOp op);
    void updateCombine();
    // Adds a body as a tool, or takes it out; the first body clicked with no
    // target becomes the target.
    void toggleCombineBody(ObjectId id);
    void setCombineTarget(ObjectId id);
    // Makes the combine from the tool's current state. False, with a notice,
    // when any of it cannot be built -- in which case nothing has changed.
    bool commitCombine();
    void finishCombine();
    void abortCombine();
    void drawCombinePanel();
    // Puts the target back as it was before the preview, and the tools back
    // in view.
    void restoreCombinePreview();
    void drawCombineOverlay();

    // ---- Dividing a face ---------------------------------------------------
    //
    // A loop cut: the plane rides along a chosen edge and splits every face it
    // crosses, leaving the body whole.
    struct DivideToolState {
        bool     active = false;
        ObjectId objectId = kNoObject;
        EdgeId   along = kInvalid;       // the edge the cut slides down
        Vec3     from{}, dir{};          // that edge, in world
        Real     t = 0.5;                // where along it, 0..1
        Real     requested = -1.0;
        bool     previewValid = false;
        DragAxis axis;
        Body before;
        std::vector<Feature> chainBefore;
        std::string typedValue;
        AsyncBuild preview;

        void reset() {
            preview.cancel();
            objectId = kNoObject;
            along = kInvalid;
            from = dir = Vec3{};
            t = 0.5;
            requested = -1.0;
            previewValid = false;
            axis = DragAxis{};
            before = Body();
            chainBefore.clear();
            typedValue.clear();
        }
    };
    DivideToolState divideTool_;

    // Repeating something. Two things wear this one tool: a pattern of the cut
    // or boss the last feature made -- which is how one hole becomes a bolt
    // circle -- and a pattern of the body itself, which is what a mirror
    // usually is. Which one it is, is decided when the gesture starts by
    // looking at what the chain ends with, and can be changed in the panel.
    struct PatternToolState {
        bool        active = false;
        ObjectId    objectId = kNoObject;
        PatternMode mode = PatternMode::Linear;
        int         count = 4;
        int         axisIndex = 0;            // 0 x, 1 y, 2 z, in the body's own space
        Vec3        origin{};                 // local: the axis, or the mirror plane
        Real        step = 10.0;              // mm between copies, Linear
        Real        stepAngle = 0.0;          // radians between copies, Circular
        Real        offset = 0.0;             // Mirror: the plane, along its normal

        // The tool the pattern repeats, and the feature it came from. Empty
        // means the pattern repeats the body.
        Body     tool;
        BooleanOp op = BooleanOp::Union;
        bool     toolAvailable = false;
        bool     useTool = false;
        size_t   replacing = 0;               // features kept, when repeating a tool

        Real     requested = 1e30;
        bool     previewValid = false;
        DragAxis axis;
        Body     before;
        std::vector<Feature> chainBefore;
        std::string typedValue;
        AsyncBuild preview;

        void reset() {
            preview.cancel();
            objectId = kNoObject;
            mode = PatternMode::Linear;
            count = 4;
            axisIndex = 0;
            origin = Vec3{};
            step = 10.0;
            stepAngle = 0.0;
            offset = 0.0;
            tool = Body();
            op = BooleanOp::Union;
            toolAvailable = useTool = false;
            replacing = 0;
            requested = 1e30;
            previewValid = false;
            axis = DragAxis{};
            before = Body();
            chainBefore.clear();
            typedValue.clear();
        }

        // The value the drag and the number field both mean, which is a
        // different quantity in each mode.
        Real dragged() const {
            return mode == PatternMode::Linear   ? step
                 : mode == PatternMode::Circular ? degrees(stepAngle)
                                                 : offset;
        }
        void setDragged(Real v) {
            if (mode == PatternMode::Linear)        step = v;
            else if (mode == PatternMode::Circular) stepAngle = radians(v);
            else                                    offset = v;
        }

        PatternSpec spec() const {
            PatternSpec s;
            s.mode = mode;
            s.count = count;
            s.dir = Vec3{axisIndex == 0 ? 1.0 : 0.0, axisIndex == 1 ? 1.0 : 0.0,
                         axisIndex == 2 ? 1.0 : 0.0};
            s.origin = mode == PatternMode::Mirror ? s.dir * offset : origin;
            s.step = step;
            s.stepAngle = stepAngle;
            s.op = op;
            return s;
        }
    };
    PatternToolState patternTool_;

    // Reducing a mesh. A panel with a live preview rather than a gesture: the
    // pointer drives nothing, a reduction takes seconds on a large mesh, and
    // what someone needs while choosing a tolerance is to see what it gives --
    // the triangles left, how far the surface moved, and whether the result
    // will convert.
    //
    // Each preview is a job with its own parameters, cancel flag and result.
    // When the tolerance changes mid-build the running job is told to stop, so
    // the one asked for next starts within a batch rather than seconds later.
    struct ReduceJob {
        Real tolerance = 0;
        int  target = 0;
        bool loosen = false;
        std::atomic<bool> cancel{false};
        ReduceResult result;
        int  solidFaces = 0;           // what Convert to Solid would make of it
    };
    struct ReduceToolState {
        bool     active = false;
        ObjectId objectId = kNoObject;
        Real     tolerance = 0.05;
        int      target = 0;
        bool     loosen = false;
        std::string typedValue;

        // The uid the committed feature will carry, taken before the first
        // preview so the preview names its faces exactly as evaluating the
        // feature later will.
        ElementId uid = 0;

        Body before;
        std::vector<Feature> chainBefore;
        AsyncBuild preview;
        std::shared_ptr<ReduceJob> running, pending, shown;
        Body shownBody;

        void reset() {
            if (running) running->cancel = true;
            preview.cancel();
            objectId = kNoObject;
            typedValue.clear();
            uid = 0;
            before = Body();
            chainBefore.clear();
            running.reset();
            pending.reset();
            shown.reset();
            shownBody = Body();
        }

        // Whether what is on screen is a finished, verified result for the
        // parameters as they stand -- the only thing OK will commit.
        bool ready() const {
            return shown && shown->tolerance == tolerance && shown->target == target &&
                   shown->loosen == loosen &&
                   shown->result.ok && shown->result.withinTolerance;
        }
    };
    ReduceToolState reduceTool_;

    // Whether a modal editing operation is running -- one that owns the model
    // until it is confirmed or cancelled.
    bool editToolActive() const;

    // True, with a notice saying how to get past it, when `obj` is a mesh: an
    // edit to part of a shape needs the exact kernel, and a mesh has to be
    // converted before it can have one.
    bool refuseMeshEdit(const SceneObject& obj, const char* what);

    void roundAllEdges();
    void beginReduce();
    void requestReducePreview();
    void updateReduce();
    void commitReduce();
    void abortReduce();
    void drawReducePanel();

    // Whether the pointer is somewhere a gesture can read a value from.
    //
    // ImGui reports -FLT_MAX for the mouse position when it does not have one --
    // the pointer left the window, or there never was one. Fed to a drag track
    // that reports a number a thousand million million million million million
    // times larger than anything a person could mean, and a gesture that was
    // waiting to be confirmed would commit it. A gesture whose pointer has gone
    // holds the value it had instead.
    bool pointerDrives() const;

    void beginPattern(PatternMode mode);
    void updatePattern(bool snap, bool follow = true);
    void setPatternMode(PatternMode mode);
    void commitPattern();
    void abortPattern();
    void drawPatternPanel();

    void mergeSelected();
    // Inset and Shell. Neither is a gesture: the faces say where, and the one
    // number is the operation. They are made as soon as they are asked for and
    // stay adjustable in their panel until Done, so the number is chosen by
    // looking at the result rather than typed into a menu beforehand.
    struct AmountToolState {
        ObjectId objectId = kNoObject;
        std::vector<FaceId> faces;          // inset: what to inset; shell: what to open
        Real amount = 1.0;
        Body before;
        std::vector<Feature> chainBefore;
        std::string typedValue;
        // Open, with nothing applied: the number it was asked for was refused,
        // so the panel stays and the next number is tried. Better than a
        // notice and no way to pick a number that works.
        bool pending = false;
        std::string refusal;
        bool active = false;                // only while it is being made

        void reset() {
            objectId = kNoObject;
            faces.clear();
            before = Body();
            chainBefore.clear();
            typedValue.clear();
            pending = false;
            refusal.clear();
            active = false;
        }
    };
    // Hole: where it goes, and what it is for.
    //
    // Placed by pointing at a face -- the hole follows the pointer and goes in
    // square to whatever is under it -- and made on the click, after which the
    // panel adjusts it until Done, the way Inset and Shell do. What is
    // adjusted there is mostly not a number but a choice: which screw, how
    // freely it passes, and what the head sits in. See geom/fasteners.h.
    struct HoleToolState {
        ObjectId objectId = kNoObject;
        FaceId face = kInvalid;          // the face it goes into, as it stands
        Vec3 at{0, 0, 0};                // its mouth, in the body's own space
        Vec3 into{0, 0, -1};             // the way it goes in, likewise
        HoleCut cut;
        int fastener = 2;                // M3, the one most printed parts use
        HoleFit fit = HoleFit::Normal;
        Body before;
        std::vector<Feature> chainBefore;
        std::string typedValue;
        bool placing = false;            // following the pointer, not yet made
        bool pending = false;            // refused: the panel stays, to try again
        std::string refusal;
        bool active = false;             // only while it is being made

        void reset() {
            objectId = kNoObject;
            face = kInvalid;
            before = Body();
            chainBefore.clear();
            typedValue.clear();
            placing = false;
            pending = false;
            refusal.clear();
            active = false;
        }
    };
    HoleToolState holeTool_;

    // Draft: how far the walls lean, which way the part comes off, and where
    // it is widest. Made as soon as it is asked for and adjusted in its panel
    // until Done, as Inset and Shell are.
    struct DraftToolState {
        ObjectId objectId = kNoObject;
        std::vector<FaceId> faces;
        Real angle = 3.0 * kDeg2Rad;
        int  axis = 2;                   // 0 X, 1 Y, 2 Z: the way it is pulled
        enum class Widest { Bottom, Top, Middle };
        Widest widest = Widest::Bottom;  // the bed, for a printed part
        Body before;
        std::vector<Feature> chainBefore;
        std::string typedValue;
        bool pending = false;
        std::string refusal;
        bool active = false;

        void reset() {
            objectId = kNoObject;
            faces.clear();
            before = Body();
            chainBefore.clear();
            typedValue.clear();
            pending = false;
            refusal.clear();
            active = false;
        }
    };
    DraftToolState draftTool_;

    // Split: what to cut with, and where. It used to decide for itself from
    // what happened to be selected -- a face's plane, a second body's, or the
    // pieces it was already in, and a cut through the middle when none of
    // those applied -- with no way to see which it had chosen or to move it.
    struct SplitToolState {
        enum class By { Face, Tool, X, Y, Z, Pieces };
        ObjectId objectId = kNoObject;
        ObjectId toolObject = kNoObject;   // a second selected body, whose plane can cut
        FaceId   face = kInvalid;          // a selected face, whose plane can cut
        By       by = By::Z;
        Real     offset = 0.0;             // where along the normal, in world
        int      pieces = 0;               // what the last cut made
        bool     canPieces = false;        // it is already in more than one piece
        Body before;
        std::vector<Feature> chainBefore;
        std::string typedValue;
        bool pending = false;               // open with nothing cut yet
        std::string refusal;
        bool active = false;

        void reset() {
            pending = false;
            refusal.clear();
            objectId = toolObject = kNoObject;
            face = kInvalid;
            by = By::Z;
            offset = 0.0;
            pieces = 0;
            canPieces = false;
            before = Body();
            chainBefore.clear();
            typedValue.clear();
            active = false;
        }
    };
    SplitToolState splitTool_;
    void beginSplit();
    void commitSplit();
    void drawSplitPanel();
    // Where the cut goes, in the body's own space: false when that choice has
    // nothing to cut with.
    bool splitPlane(Vec3& point, Vec3& normal) const;
    // Replaces `id` with the first piece and makes an object of each of the
    // rest, as one undo entry.
    bool applySplitPieces(ObjectId id, std::vector<Body> pieces, std::vector<Feature> chainBefore);

    AmountToolState insetTool_;
    AmountToolState shellTool_;
    // Placing a hole, making it, and the panel that adjusts it.
    void beginDraft();
    void commitDraft();
    void drawDraftPanel();
    // The pull direction and the point on the neutral plane the choices come
    // to, in the body's own space.
    void draftPlane(Vec3& neutralPoint, Vec3& pull) const;

    void beginHole();
    void updateHole();
    void commitHole();
    void abortHole();
    void drawHolePanel();
    void drawHoleOverlay();
    // The cut the current choices come to: the table's numbers for the
    // fastener, or what was typed when there is none.
    HoleCut holeCutNow() const;

    void beginInset();
    void commitInset();
    void drawInsetPanel();
    void beginShell();
    void commitShell();
    void drawShellPanel();

    void beginDivide();
    void updateDivide(bool snap, bool follow = true);
    void commitDivide();
    void abortDivide();
    void drawDividePanel();

    void beginFillet();

    // Works out what the gesture will build, before it builds any of it.
    void planFillet(SceneObject& obj);

    // The operation as planned, at this radius.
    FilletSpec filletSpecAt(Real radius) const;
    std::vector<Real> filletRadiiAt(Real radius) const;
    static bool filletUniform(const std::vector<Real>& radii);

    // Advances the search for the largest fillet this gesture can make, one
    // trial per frame, without waiting for any of them.
    void stepFilletLimitSearch();
    void stepSnapDemo();
    void stepProfileDemo();
    void stepFilletOpenDemo();
    void stepFaceDemo();
    void stepFaceStress();
    void stepPrintDemo();
    void stepPreviewCheck();
    void stepFilletFloorSearch();
    void startFilletTrial(Real radius);
    static const Real kFilletFloorLadder[6];
    // `follow` false keeps the radius where it is while still collecting a
    // preview that is already building: the pointer is on the dialog, not on
    // the model.
    void updateFillet(bool snap, bool follow = true);
    void commitFillet();
    void abortFillet();
    void drawFilletPanel();

    CreateTool createTool_;
    SketchTool sketchTool_;

    void beginAddPrimitivePrompt(PrimitiveKind kind);
    void beginSketch();
    // Import SVG: asks for a file. Into the sketch being drawn when there is
    // one; otherwise a new sketch, whose plane is chosen before it lands.
    void beginImportSvg();
    void importSvg(const std::string& path);
    void beginEditSketch(ObjectId object, ElementId sketchUid);
    void drawSceneSketches();

    bool justFinishedModal_ = false;

    // The panel of an operation that has been applied and not yet dismissed.
    //
    // A gesture is confirmed with a click in the viewport, which meant its
    // panel vanished at the moment the pointer was free to reach it -- so every
    // control on it was only reachable by first dragging the value across the
    // whole viewport to get there, which is to say not reachable at all. The
    // panel now stays up after the click. The operation is applied, the pointer
    // no longer drives anything, and the controls adjust what was just made.
    //
    // Blender calls this Adjust Last Operation and Plasticity leaves the same
    // panel floating; Fusion and Onshape sidestep it by never confirming on a
    // viewport click in the first place. This is the first of those, because
    // this app does confirm on a click.
    enum class Settled { None, Fillet, Divide, Face, Pattern, Create, Sketch, Combine, Inset, Shell,
                         Split, Hole, Draft };
    Settled  settled_ = Settled::None;
    ObjectId settledObject_ = kNoObject;

    // The chain as the panel left it. An adjustment that will not build puts
    // this back, so the model and the undo entry never disagree.
    std::vector<Feature> settledAfter_;

    // Set while an adjustment is being applied, so the commit folds into the
    // undo entry it is adjusting rather than stacking a new one per keystroke.
    bool recommitting_ = false;

    // Bumped every time an operation is successfully applied. A refusal is not
    // visible any other way from out here: a commit that fails puts the chain
    // back to before the operation, and for a pattern that replaced a boolean
    // that chain is the same length as the one that succeeded.
    size_t settleSerial_ = 0;

    // The undo stack as the settled operation left it. The panels that act on
    // more than one body adjust by taking their own undo entry back and making
    // the operation again; this is how they know that entry is still the top
    // one, and not something done since.
    size_t settledRevision_ = 0;

    // Keeps settled_ in step with the create and sketch tools, which say for
    // themselves when they have been applied and when they are done.
    void syncToolSettled();

    // The face extrude's swept solid, and the bodies it reaches, for its value
    // now. Drawn when it is not simply part of the body it came from.
    void refreshFaceReach();
    void drawFaceToolOverlay();

    // Applies the panel's current values again, over the top of the last
    // application rather than after it.
    void recommitSettled();

    // Records that an operation has been applied and its panel should stay.
    void settleCommand(Settled kind, ObjectId id);

    // Puts the panel away, keeping what it made.
    void dismissSettled();

    bool settledIs(Settled k) const { return settled_ == k; }


    // Breaks the active object into its separate bodies or splits by a plane/face.
    void splitActiveObject();

    // Single exit points for a modal transform, so the extrude-drag's extra
    // bookkeeping cannot be forgotten at one of the several call sites.
    void commitTransform();
    void abortTransform();
    void applyActions();
    void buildUi();
    void drawFrame();

    // Where the 3D view lives inside the window, in ImGui's logical points.
    struct ViewRect { float x = 0, y = 0, w = 0, h = 0; };

    void addPrimitiveAtCursor(PrimitiveKind kind);

    // Drops an object so its lowest point rests on z = 0. For print design the
    // ground plane is the build plate, so parts belong on it rather than
    // centred through it -- which also stops the grid drawing across them.
    void placeOnBuildPlate(ObjectId id);

    // Picking. See handleViewportClick.
    size_t nextInCycle(size_t count, bool additive, const std::function<bool(size_t)>& isSelected) const;
    void sayWhichOfCoincident(size_t at, size_t count, ObjectId on, const char* what);
    void pickWholeObject(ObjectId id, bool additive);

    // The inspector's transform fields, as Move, Rotate and Scale steps.
    void recordInspectorTransform(ObjectId id, const Transform& before);
    void bakePendingScale();
    void resetObjectTransform(ObjectId id);
    ObjectId pendingScale_ = kNoObject;

    SDL_Window* window_  = nullptr;
    void*       glCtx_   = nullptr;
    std::string shaderDir_;

    Scene         scene_;
    UndoStack     undo_;
    TransformTool tool_;
    MeasureTool   measure_;
    MeasureResult measureResult_;
    Camera      camera_;
    Renderer    renderer_;
    ViewOptions view_;
    UiContext   ui_;

    ViewRect viewRect_;
    float    pixelScaleX_ = 1.0f, pixelScaleY_ = 1.0f;

    bool  running_     = true;
    bool  firstLayout_ = true;
    int   smokeFrames_ = 0;
    float frameMs_     = 0.0f;
    float lastDt_      = 0.0f;

    // Seconds the geometry has been unchanged. The printability check is far
    // too expensive to run mid-drag, so it waits for things to settle.
    float healthIdle_  = 0.0f;

    // Was the object a clean solid when the current gesture started? An edit
    // is only refused for breaking something that was not already broken.
    bool  preEditSolid_ = false;

    // Transient message shown in the status bar, e.g. a refused edit.
    std::string notice_;
    float       noticeAge_ = 0.0f;
    void setNotice(const std::string& text) { notice_ = text; noticeAge_ = 0.0f; }

    // Reverts a just-applied edit that would leave the model unprintable.
    bool editKeepsSolid(ObjectId id);

    // Set for one frame when Shift+A asks for the add menu at the cursor.
    bool  openAddMenu_ = false;

    // True while a middle-drag navigation gesture is in progress.
    bool  navigating_ = false;

    // The window's frame: the system's, or the bar the application draws.
    // `hitFrame_` is the bar as it was last drawn, kept apart from the one
    // being drawn so the window manager's question can be answered at any
    // moment.
    bool       nativeFrame_ = false;
    FrameState hitFrame_;

    void captureFramebuffer(int width, int height) const;
    std::string screenshotPath_;
    int         screenshotFrame_ = -1;
    bool        fixedCamera_ = false;
    bool        startEmpty_ = false;

    int         pickFace_ = -1;
    bool        measureDemo_ = false;
    int         fileDemo_ = -1;
    std::string headlessExport_;
    std::string headlessExport3mf_;
    int         booleanDemo_ = -1;
    bool        historyDemo_ = false;
    int         coplanarDemo_ = 0;
    bool        coplanarDemoDone_ = false;
    void stepCoplanarDemo();
    bool        filletEdgesDemo_ = false;
    bool        roundAllDemo_ = false;
    int         filletDemoEdges_ = 1;
    bool        autoExtrude_ = false;
    float       autoExtrudeMm_ = 10.0f;
    float       filletDemo_ = 0.0f;     // radius in mm; 0 means do not
    float       shellExtrudeDemo_ = 0.0f;
    bool        shellFilletDemo_ = false;
    int         splitDemo_ = 0;
    float       insetDemo_ = 0.0f;      // distance in mm; 0 means do not
    float       shellDemo_ = 0.0f;      // wall in mm; 0 means do not
    bool        holdTransform_ = false;

    double meanViewportLuminance() const;
    void   readViewport(std::vector<unsigned char>& out) const;
    std::vector<unsigned char> probePrev_;
    PixelRect lastViewportPx_;
    bool  probeActive_ = false;
    float probeYaw0_ = 0.0f, probeYaw1_ = 90.0f;
    int   probeSteps_ = 2, probeIndex_ = 0, probeSettle_ = 0;
    bool  alignProbe_ = false;
    void  sampleGridAlignment(float& onLine, float& offLine) const;
};

} // namespace tg
