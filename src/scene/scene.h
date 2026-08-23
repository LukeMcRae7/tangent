// Tangent - scene graph, selection and picking.
//
// Objects keep the parameters they were created from, not just their triangles,
// so a primitive can be re-evaluated when those parameters change. That is the
// seed of the parametric history in the later milestones; `meshVersion` is what
// tells the renderer its cached buffers went stale.
#pragma once

#include "mesh/health.h"
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

struct SceneObject {
    ObjectId      id = kNoObject;
    std::string   name;
    Transform     transform;
    PrimitiveSpec spec;

    // The chain the mesh is evaluated from. The first entry is the base
    // primitive; later entries are operations applied in order.
    std::vector<Feature> features;

    // featureCache[i] is the mesh as it stood after feature i, so an edit only
    // has to re-run from the feature it touched.
    std::vector<Mesh> featureCache;

    Mesh       mesh;
    RenderMesh render;
    AABB       localBounds;
    bool       visible = true;

    // Bumped on every geometry change; the renderer re-uploads when it differs
    // from the version it last saw.
    uint32_t meshVersion = 1;

    // Cached printability report. Self-intersection testing is too costly to
    // repeat every frame, so it is refreshed only when the geometry changes.
    MeshHealth health;
    uint32_t   healthVersion = 0;

    Mat4 modelMatrix() const { return transform.matrix(); }

    // Recomputes everything derived from `mesh` and marks the GPU copy stale.
    // Any code that edits vertex positions must call this.
    void refreshDerived() {
        mesh.buildRenderMesh(render);
        localBounds = mesh.bounds();
        ++meshVersion;
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
    ObjectId addPrimitive(PrimitiveKind kind, const PrimitiveSpec& spec = {},
                          Vec3 position = {});
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
    bool addFeature(ObjectId id, Feature feature);

    // For serialisation, which has to preserve the counter alongside the
    // features it has already handed numbers to.
    uint64_t nextFeatureUid() const { return nextFeatureUid_; }
    void setNextFeatureUid(uint64_t v) { if (v > nextFeatureUid_) nextFeatureUid_ = v; }
    ElementId takeFeatureUid() { return nextFeatureUid_++; }

    // Re-runs an object's chain as it stands. rebuild() pushes the inspector's
    // spec into the base feature first; this does not, which is what undo
    // needs when restoring a whole chain.
    bool reevaluate(ObjectId id) { return reevaluateFrom(id, 0); }

    // Re-runs only from `fromFeature` onward, reusing the cached intermediate
    // before it. Pass 0 to rebuild everything.
    bool reevaluateFrom(ObjectId id, size_t fromFeature);

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
        return kNoObject;
    }

    // ---- Queries ---------------------------------------------------------
    AABB bounds() const;
    AABB selectionBounds() const;
    Vec3 selectionCenter() const;

    // Nearest surface hit along the ray, in world space.
    RayHit raycast(const Ray& ray) const;

    // Resolves a click to the specific vertex, edge or face under the cursor,
    // the way a CAD tool does: whatever is nearest in *screen* space wins, with
    // vertices beating edges beating the face behind them. Tolerances are in
    // pixels so the pick feels the same at any zoom.
    //
    // Takes the view-projection and viewport size rather than a Camera, so the
    // scene layer stays independent of the application layer.
    ElementHit pickElement(const Ray& ray, const Mat4& viewProj,
                           int viewportW, int viewportH, Vec2 cursorPx,
                           float vertexTolPx = 9.0f, float edgeTolPx = 7.0f) const;

    // ---- Sub-object selection --------------------------------------------
    const std::vector<ElementRef>& elementSelection() const { return elements_; }
    bool isElementSelected(const ElementRef& e) const;
    void selectElement(const ElementRef& e, bool additive = false);
    void toggleElement(const ElementRef& e);
    void clearElementSelection() { elements_.clear(); }

    // Faces currently selected on one object, for feeding the mesh operations.
    std::vector<Index> selectedFaces(ObjectId id) const;

    // Selected edges on one object, named by canonical half-edge.
    std::vector<Index> selectedEdges(ObjectId id) const;

    // Drops any element selection referring to geometry that no longer exists.
    // Mesh edits renumber faces wholesale, so stale refs must not survive one.
    void pruneElementSelection();

private:
    std::string uniqueName(const std::string& base) const;

    std::vector<std::unique_ptr<SceneObject>> objects_;
    std::vector<ObjectId> selection_;
    std::vector<ElementRef> elements_;
    ObjectId nextId_ = 1;

    // Handed to each new feature so it has an identity independent of where it
    // sits in the chain. Saved with the project: reloading and then adding a
    // feature must not reissue a number an existing feature already holds.
    uint64_t nextFeatureUid_ = 1;
};

} // namespace tg
