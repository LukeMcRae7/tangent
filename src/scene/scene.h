// Tangent - scene graph, selection and picking.
//
// Objects keep the parameters they were created from, not just their triangles,
// so a primitive can be re-evaluated when those parameters change. That is the
// seed of the parametric history in the later milestones; `meshVersion` is what
// tells the renderer its cached buffers went stale.
#pragma once

#include "app/printability.h"
#include "mesh/health.h"
#include "scene/assembly.h"
#include "scene/feature.h"

#include <memory>
#include <string>
#include <vector>

namespace tg {

using ObjectId = uint32_t;
inline constexpr ObjectId kNoObject = 0;

struct Transform {
    Vec3 position{0.0f, 0.0f, 0.0f};
    Quat rotation{};
    Vec3 scale{1.0f, 1.0f, 1.0f};

    Mat4 matrix() const {
        return translate(position) * toMat4(rotation) * scaleMat(scale);
    }
};

// Where the history puts an object that started at `base`: every Move and
// Rotate in it, in order, the ones turned off left out.
Transform placementOf(const Transform& base, const std::vector<Feature>& features);

// How much bigger the history has made the body along each of its own axes:
// every Scale step, multiplied together. What the inspector calls its scale.
Vec3 scaleOf(const std::vector<Feature>& features);

struct SceneObject {
    ObjectId      id = kNoObject;
    std::string   name;

    // The group it is in, in the outliner, or kNoGroup. See scene/assembly.h:
    // a group places nothing, but it is the unit a joint moves.
    GroupId       group = kNoGroup;

    // Its colour, when one has been chosen; until then it is drawn in the
    // theme's grey, like every other part. Saved, and written into a 3MF so
    // a slicer shows the parts apart.
    bool          coloured = false;
    Vec3          colour{0.74, 0.74, 0.75};

    // Where the object is. Derived, not set: `base` is where it was made --
    // the point a box was drawn at, the plane a part was drawn on -- and the
    // Move and Rotate steps in its history take it on from there. Scene keeps
    // the two in step every time the chain changes; anything that sets
    // `transform` directly is showing something, like a gesture in progress,
    // and the next evaluation puts it back where the history says.
    //
    // The scale is always one, but for the moment a Scale gesture is being
    // dragged: a scale is a change of shape, so it is a step in the chain.
    Transform     transform;
    Transform     base;
    PrimitiveSpec spec;

    // The chain the mesh is evaluated from. The first entry is the base
    // primitive; later entries are operations applied in order.
    std::vector<Feature> features;

    // The steps after the rollback marker: kept, and not run. What the view
    // shows and what the next operation builds on is `features` alone, so an
    // operation done while rolled back goes in at the marker -- and moving the
    // marker down again runs these on top of it, each finding what it names
    // on the geometry as it now is. Empty when the marker is at the end.
    std::vector<Feature> ahead;
    // What those waiting steps built when they last ran, and its fingerprints:
    // rolled forward again with nothing changed below them, they need not run.
    std::vector<Body> aheadCache;
    std::vector<uint64_t> aheadKeys;

    // featureCache[i] is the body as it stood after feature i, so an edit only
    // has to re-run from the feature it touched.
    std::vector<Body> featureCache;
    // featureKeys[i] is the fingerprint (featureKey) of the step the cache holds
    // the result of at i -- how a re-run knows where the history first differs
    // from what was built, and starts there. Shorter than the cache when not
    // known, which only costs a longer re-run.
    std::vector<uint64_t> featureKeys;

    Body       body;
    RenderMesh render;
    AABB       localBounds;
    bool       visible = true;

    // Bumped whenever `render` changes, for any reason; the renderer re-uploads
    // when it differs from the version it last saw.
    uint32_t meshVersion = 1;

    // Bumped when the body itself changes -- not when the same body is merely
    // drawn at a different tolerance. What is worked out from the geometry, and
    // costs real time, hangs off this: the printability check and the health
    // report are about the part, and a zoom does not change the part.
    uint32_t geometryVersion = 1;

    // The chord tolerance `render` was built at, or 0 for "whatever the backend
    // chose". An exact body is re-tessellated as the view changes -- see
    // render/lod.h -- and this is what that decision is made against.
    Real renderDeviation = 0.0;

    // Cached printability report. Self-intersection testing is too costly to
    // repeat every frame, so it is refreshed only when the geometry changes.
    MeshHealth health;
    uint32_t   healthVersion = 0;

    // What a printer would make of it, worked out when the geometry moves and
    // kept until it moves again. A ray per face against the triangles is a few
    // milliseconds on a small part and tens on a large one -- affordable once
    // per edit, not once per frame.
    PrintReport printCheck;
    // The flagged triangles themselves, in the body's own space, gathered once
    // when the check runs. Drawing them used to search every triangle of the
    // body for every flagged face on every frame -- invisible on a part with a
    // few hundred triangles and twenty-five million comparisons a frame on an
    // imported mesh with walls to complain about.
    std::vector<Vec3> printTriangles;
    uint32_t    printVersion = 0;

    Mat4 modelMatrix() const { return transform.matrix(); }

    // Recomputes everything derived from `mesh` and marks the GPU copy stale.
    // Any code that edits vertex positions must call this.
    void refreshDerived() {
        body.tessellate(render);
        // The backend chose the tolerance, so the view has not had its say yet.
        renderDeviation = 0.0;
        localBounds = body.bounds();
        // An object that is only a sketch has no body to be measured, but it is
        // somewhere and it has a size: without this, framing the view on one or
        // drawing a box round it would have nothing to work from.
        if (body.empty()) {
            for (const Feature& f : features) {
                if (f.kind != FeatureKind::Sketch || !f.sketchShown || !f.enabled) continue;
                for (const SketchEntity& e : f.sketch.entities)
                    for (Vec2 p : sketchEntityPoints(f.sketch, e))
                        localBounds.expand(f.sketch.plane.toWorld(p));
            }
        }
        ++meshVersion;
        ++geometryVersion;
    }
    AABB worldBounds() const;
    void markMeshChanged() { ++meshVersion; }
};

// What a click resolved to. Edges are identified by the lower of their two
// half-edge indices so both directions name the same edge.
enum class ElementKind { None, Vertex, Edge, Face };

const char* elementKindName(ElementKind k);

struct ElementRef {
    ObjectId    object = kNoObject;
    ElementKind kind   = ElementKind::None;
    Index       index  = kInvalid;

    bool valid() const { return object != kNoObject && kind != ElementKind::None; }
    bool operator==(const ElementRef& o) const {
        return object == o.object && kind == o.kind && index == o.index;
    }
    bool operator!=(const ElementRef& o) const { return !(*this == o); }
};

struct ElementHit {
    ElementRef ref;
    float      t = 0.0f;
    Vec3       point;
    bool hit() const { return ref.valid(); }
};

// A section view: the model drawn cut by a plane, with everything on the side
// the normal points to taken away -- for looking inside a part without changing
// it. A view, not an edit: nothing about any body is touched. It lives on the
// scene only because picking has to honour it too: a click on what is not
// drawn must not select it, and a click on the cut face must not reach through
// the cut to the wall behind it.
struct SectionCut {
    bool on = false;
    Vec3 normal{0.0, 0.0, 1.0};     // unit; points into the side taken away
    Real offset = 0.0;              // the plane is dot(normal, p) == offset

    // Whether `p` is on the side taken away. A hair of tolerance keeps what
    // lies in the plane itself -- a face the plane was put on -- drawn and
    // pickable.
    bool removes(Vec3 p) const { return on && dot(normal, p) > offset + 1e-6 * (1.0 + std::fabs(offset)); }
};

// A measurement kept on the model: what it was taken between, by the names
// the geometry keeps through edits, so it reads the part as it is now and not
// as it was when it was taken. Drawn in the view and listed in the outliner.
struct KeptMeasure {
    struct End {
        ObjectId    object = kNoObject;
        ElementKind kind = ElementKind::None;
        ElementId   name = 0;
    };
    uint32_t id = 0;
    End      ends[2];
    int      count = 0;          // one end or two
    bool     visible = true;
};

struct RayHit {
    ObjectId object = kNoObject;
    Index    face   = kInvalid;
    float    t      = 0.0f;
    Vec3     point;
    Vec3     normal;
    bool hit() const { return object != kNoObject; }
};

class Scene {
public:
    // ---- Contents --------------------------------------------------------
    // Adds a body that came from outside -- a STEP import, or a mesh read from
    // a file. Its chain root is BaseMesh: geometry with no parameters behind
    // it, because the file does not carry how it was made. Everything after
    // that root behaves like any other feature.
    ObjectId addImportedBody(Body body, const std::string& name);

    // A salt for the next import, so two imports of the same file do not name
    // their faces identically and a feature written against one does not
    // silently attach to the other.
    ElementId nextImportSalt() { return nextFeatureUid_++; }

    ObjectId addPrimitive(PrimitiveKind kind, const PrimitiveSpec& spec = {},
                          Vec3 position = {});

    // A sketch standing on its own that has just been built into its first
    // solid is a part now, and is called one -- unless it was named by hand.
    void nameAsPart(ObjectId id) {
        SceneObject* o = find(id);
        if (o && !o->body.empty() && o->name.rfind("Sketch", 0) == 0) o->name = uniqueName("Part");
    }

    // Which kernel new bodies are built with. Exact where the build has it,
    // and a mesh otherwise, so the same code path serves both -- and so a build
    // without OpenCASCADE behaves exactly as it did before the backend existed.
    //
    // Per scene rather than per call: it is a mode, not a parameter, and the
    // mesh side comes back as a mode too when imported geometry gets its own
    // place in the interface.
    Backend defaultBackend() const { return defaultBackend_; }
    void setDefaultBackend(Backend b) { defaultBackend_ = b; }
    ObjectId addBody(Body body, Vec3 position = {}, const std::string& name = "Object");

    // Adds an object made entirely by its history -- a part that begins as a
    // sketch and a region swept out of it, with no primitive underneath. The
    // chain is evaluated first, and nothing is added unless every step of it
    // succeeds; `error` then says which step did not.
    //
    // A chain of nothing but sketches is allowed and leaves the object with no
    // body: a sketch is a thing in the scene before anything is built from it.
    ObjectId addFeatureChain(std::vector<Feature> features, const std::string& name,
                             std::string* error = nullptr);

    // The same, but the object is kept whatever the chain does. For a file: a
    // step that no longer evaluates -- a sketch extruded in a build that has no
    // exact kernel -- is marked and skipped, and dropping the object instead
    // would lose the work rather than report it.
    ObjectId addChainAsIs(std::vector<Feature> features, const std::string& name);
    bool     removeObject(ObjectId id);
    ObjectId duplicateObject(ObjectId id);

    // Detach / re-attach preserving the object's id. Undo needs an object to
    // come back as the same object -- selections, and later feature references,
    // are held by id, so re-adding under a fresh id would silently break them.
    std::unique_ptr<SceneObject> takeObject(ObjectId id);
    void insertObject(std::unique_ptr<SceneObject> obj);

    SceneObject*       find(ObjectId id);
    const SceneObject* find(ObjectId id) const;

    const std::vector<std::unique_ptr<SceneObject>>& objects() const { return objects_; }
    size_t objectCount() const { return objects_.size(); }
    void clear();

    // Re-evaluates an object's feature chain. Returns false and leaves the
    // previous mesh in place if the chain cannot produce geometry at all.
    bool rebuild(ObjectId id);

    // Appends a feature and re-evaluates. On failure the feature is removed
    // again, so a rejected operation cannot leave a dead entry in the timeline.
    //
    // `error` gets the rejected feature's own reason, which is otherwise
    // discarded along with the feature and leaves the caller with nothing to
    // tell the user beyond "refused".
    bool addFeature(ObjectId id, Feature feature, std::string* error = nullptr);

    // Swaps an object's whole chain for another. For an edit that is not an
    // append: a pattern of the last boolean stands *where that boolean stood*
    // rather than after it, because the pattern's first copy is that boolean
    // and keeping both would cut the same hole twice. All or nothing -- a chain
    // that will not evaluate leaves the object as it was.
    bool setFeatures(ObjectId id, std::vector<Feature> features,
                     std::string* error = nullptr);

    // Appends a feature whose result has already been built -- by a preview on
    // another thread, from this object's current body -- instead of building it
    // again. For a step that takes seconds, like reducing a large mesh, which a
    // user has just watched finish and should not wait for twice.
    //
    // The caller vouches that `result` is exactly what evaluating `feature` on
    // the current body gives, which for a deterministic operation means: the
    // same body, the same parameters, and the same uid, since the uid names what
    // it makes. The feature must arrive with that uid already set.
    void addFeatureWithResult(ObjectId id, Feature feature, Body result);

    // Sets where an object was made, and puts it where its history then says.
    // For creation -- a box drawn at a point, a part on a plane -- and for a
    // file, never for moving something the user has placed: that is a Move.
    void setBasePlacement(ObjectId id, const Transform& base);

    // Moving, turning and scaling a whole object, as steps in its history.
    //
    // Each folds into the step before it when that is the same kind of step --
    // and, for a turn or a scale, about the same point -- so nudging an object
    // about leaves one Move behind rather than one per nudge, the way a fillet
    // added to next to another becomes part of it. A step folded back to
    // nothing is removed. Placement is cheap: a move or a turn changes no
    // geometry, so nothing is rebuilt. A scale is a change of shape and is
    // built like any other; false, with the reason, if it cannot be.
    bool recordMove(ObjectId id, Vec3 by);
    bool recordRotate(ObjectId id, Quat turn, Vec3 aboutWorld);
    bool recordScale(ObjectId id, Vec3 factors, Vec3 aboutLocal, std::string* error = nullptr);

    // Puts an object where its base and its history say it is.
    void place(SceneObject& obj) const { obj.transform = placementOf(obj.base, obj.features); }

    // For serialisation, which has to preserve the counter alongside the
    // features it has already handed numbers to.
    uint64_t nextFeatureUid() const { return nextFeatureUid_; }
    void setNextFeatureUid(uint64_t v) { if (v > nextFeatureUid_) nextFeatureUid_ = v; }
    ElementId takeFeatureUid() { return nextFeatureUid_++; }

    // Re-runs an object's chain as it stands. rebuild() pushes the inspector's
    // spec into the base feature first; this does not, which is what undo
    // needs when restoring a whole chain.
    // Re-runs from the first step that differs from what the cache was built
    // from -- nothing, if nothing does.
    bool reevaluate(ObjectId id);
    // Every step, from the root, whatever the cache holds.
    bool reevaluateAll(ObjectId id) { return reevaluateFrom(id, 0); }

    // Moves the rollback marker so that the first `active` steps of the whole
    // history run and the rest wait after it -- see SceneObject::ahead. False
    // when a step that now runs fails; it is marked, as any failing step is.
    bool rollTo(ObjectId id, size_t active);
    // Every step, run or waiting.
    size_t historyLength(ObjectId id) const;

    // Re-runs only from `fromFeature` onward, reusing the cached intermediate
    // before it. Pass 0 to rebuild everything.
    bool reevaluateFrom(ObjectId id, size_t fromFeature);

    // A feature that failed during the last re-evaluation, phrased for the
    // status bar, or empty. Reading it clears it.
    //
    // The History panel has always marked a failed step, but a chain is re-run
    // by edits that have nothing to do with the step that breaks: nudging a
    // base dimension, undoing, toggling something earlier. The fillet then
    // quietly disappears from the viewport and the only sign of it is a panel
    // the user may not have open. Only a step that has *newly* failed is
    // reported, so a chain carrying a known-bad step does not repeat itself on
    // every frame of a slider drag.
    std::string takeChainNotice() { std::string s; s.swap(chainNotice_); return s; }

    // ---- Groups and joints ----------------------------------------------------
    // See scene/assembly.h, which holds the operations on them.
    const Assembly& assembly() const { return assembly_; }
    Assembly&       assembly()       { return assembly_; }

    // ---- Kept measurements -------------------------------------------------------
    std::vector<KeptMeasure>&       measures()       { return measures_; }
    const std::vector<KeptMeasure>& measures() const { return measures_; }
    uint32_t takeMeasureId() { return nextMeasure_++; }
    // Where an end is now: its face, edge or corner found again by name. An
    // invalid ref when that is gone.
    ElementRef resolve(const KeptMeasure::End& e) const {
        const SceneObject* o = find(e.object);
        if (!o || e.name == 0) return {};
        Index at = kInvalid;
        switch (e.kind) {
            case ElementKind::Face:   at = o->body.findFace(e.name); break;
            case ElementKind::Edge:   at = o->body.findEdge(e.name); break;
            case ElementKind::Vertex: at = o->body.findVertex(e.name); break;
            case ElementKind::None:   break;
        }
        return at == kInvalid ? ElementRef{} : ElementRef{e.object, e.kind, at};
    }

    // ---- Section view ---------------------------------------------------------
    const SectionCut& section() const { return section_; }
    void setSection(const SectionCut& cut) { section_ = cut; }

    // ---- Selection -------------------------------------------------------
    const std::vector<ObjectId>& selection() const { return selection_; }
    bool isSelected(ObjectId id) const;
    void clearSelection();
    void select(ObjectId id, bool additive = false);
    void toggleSelect(ObjectId id);
    void selectAll();
    ObjectId activeObject() const { return selection_.empty() ? kNoObject : selection_.back(); }

    // The object the side panels should describe. Picking a face deliberately
    // clears the object selection, but the panels should still follow the body
    // that face belongs to rather than going blank.
    ObjectId contextObject() const {
        if (!selection_.empty()) return selection_.back();
        if (!elements_.empty()) return elements_.front().object;
        if (sketch_.valid()) return sketch_.object;
        return kNoObject;
    }

    // ---- A sketch, picked as a thing of its own ----------------------------
    // A sketch is in the model, not only in a history: it can be pointed at in
    // the view or in the outliner, and then edited or built from. Selecting it
    // takes the object and element selection out of play, and selecting
    // anything else drops it.
    struct SketchRef {
        ObjectId object = kNoObject;
        ElementId uid = 0;
        bool valid() const { return object != kNoObject && uid != 0; }
        bool operator==(const SketchRef& o) const { return object == o.object && uid == o.uid; }
    };
    void selectSketch(SketchRef ref);
    void clearSketchSelection() { sketch_ = SketchRef{}; }
    // The selected sketch, or an invalid ref when there is none or it has gone.
    SketchRef selectedSketch() const;
    // The Sketch feature `ref` names, or null.
    const Feature* sketchFeature(SketchRef ref) const;

    // ---- Queries ---------------------------------------------------------
    AABB bounds() const;
    AABB selectionBounds() const;
    Vec3 selectionCenter() const;

    // Nearest surface hit along the ray, in world space. Under a section view,
    // what the section takes away is not hit, and the cut face is: a ray whose
    // nearest surface is the inside of a wall crossed the plane inside the
    // part, so it meets the cut face first -- it misses, and `capped`, when
    // given, says so.
    RayHit raycast(const Ray& ray, bool* capped = nullptr) const;

    // Every surface the ray meets at the nearest depth, one hit per body.
    //
    // Usually one. More where faces of different bodies lie in the same plane
    // -- two parts flush on the build plate, a copy left where the original
    // is -- and fight over the same pixels, so that whichever the depth buffer
    // happens to draw is no guide to which a click means. The caller cycles
    // through them. In scene order, so the cycle is the same every time.
    std::vector<RayHit> raycastCoincident(const Ray& ray) const;

    // Resolves a click to the specific vertex, edge or face under the cursor,
    // the way a CAD tool does: whatever is nearest in *screen* space wins, with
    // vertices beating edges beating the face behind them. Tolerances are in
    // pixels so the pick feels the same at any zoom.
    //
    // Takes the view-projection and viewport size rather than a Camera, so the
    // scene layer stays independent of the application layer.
    ElementHit pickElement(const Ray& ray, const Mat4& viewProj,
                           int viewportW, int viewportH, Vec2 cursorPx,
                           float vertexTolPx = 10.0f, float edgeTolPx = 7.0f) const;

    // Everything a click could mean there: what pickElement would choose on
    // each body raycastCoincident finds, each once. The first is what
    // pickElement returns.
    std::vector<ElementHit> pickElements(const Ray& ray, const Mat4& viewProj,
                                         int viewportW, int viewportH, Vec2 cursorPx,
                                         float vertexTolPx = 10.0f, float edgeTolPx = 7.0f) const;

    // ---- Sub-object selection --------------------------------------------
    const std::vector<ElementRef>& elementSelection() const { return elements_; }
    bool isElementSelected(const ElementRef& e) const;
    // Every piece of the face `e` names: itself, plus anything it was split
    // from because a face cannot hold a hole. Anything else is just `e`.
    std::vector<ElementRef> faceGroup(const ElementRef& e) const;

    void selectElement(const ElementRef& e, bool additive = false);
    void toggleElement(const ElementRef& e);
    void clearElementSelection() { elements_.clear(); }

    // Faces currently selected on one object, for feeding the operations.
    std::vector<FaceId> selectedFaces(ObjectId id) const;

    // Selected edges on one object, as canonical edge handles.
    std::vector<EdgeId> selectedEdges(ObjectId id) const;

    // Drops any element selection referring to geometry that no longer exists.
    // Mesh edits renumber faces wholesale, so stale refs must not survive one.
    void pruneElementSelection();

private:
    std::string uniqueName(const std::string& base) const;

    std::vector<std::unique_ptr<SceneObject>> objects_;
    Assembly assembly_;
    SectionCut section_;
    std::vector<KeptMeasure> measures_;
    uint32_t nextMeasure_ = 1;
    std::vector<ObjectId> selection_;
    std::vector<ElementRef> elements_;
    // The persistent name of each selected element, taken when it was
    // selected. Indices are renumbered by any edit; the name is what finds the
    // same face, edge or point again afterwards, so a selection kept across a
    // change stays on what was picked instead of landing on whatever now has
    // its number.
    struct NamedRef { ElementRef ref; ElementId name = 0; };
    std::vector<NamedRef> elementNames_;
    void rememberName(const ElementRef& e);
    SketchRef sketch_;
    ObjectId nextId_ = 1;

    // Handed to each new feature so it has an identity independent of where it
    // sits in the chain. Saved with the project: reloading and then adding a
    // feature must not reissue a number an existing feature already holds.
    // Fills chainNotice_ from whatever the last evaluation marked, ignoring
    // steps that were already failing when it started.
    void noteNewFailures(const SceneObject& obj, const std::vector<ElementId>& wasBroken);

    Backend defaultBackend_ = brep::available() ? Backend::Brep : Backend::Mesh;
    uint64_t nextFeatureUid_ = 1;
    std::string chainNotice_;
};

} // namespace tg
