#include "app/undo.h"

#include <algorithm>

namespace tg {

// ---------------------------------------------------------------------------
void TransformCommand::undo(Scene& scene) {
    for (const Entry& e : entries_)
        if (SceneObject* o = scene.find(e.id)) o->transform = e.before;
}

void TransformCommand::redo(Scene& scene) {
    for (const Entry& e : entries_)
        if (SceneObject* o = scene.find(e.id)) o->transform = e.after;
}

bool TransformCommand::mergeWith(const Command& other) {
    const auto* rhs = dynamic_cast<const TransformCommand*>(&other);
    if (!rhs || rhs->what_ != what_ || rhs->entries_.size() != entries_.size()) return false;

    // Only merge when both steps moved exactly the same objects; otherwise the
    // combined command would not fully describe either edit.
    for (size_t i = 0; i < entries_.size(); ++i)
        if (entries_[i].id != rhs->entries_[i].id) return false;

    // Keep our `before` (the start of the gesture), take their `after`.
    for (size_t i = 0; i < entries_.size(); ++i)
        entries_[i].after = rhs->entries_[i].after;
    return true;
}

// ---------------------------------------------------------------------------
std::unique_ptr<ExistenceCommand> ExistenceCommand::forCreate(
        const Scene& scene, const std::vector<ObjectId>& ids) {
    auto cmd = std::make_unique<ExistenceCommand>();
    cmd->created_ = true;
    cmd->ids_ = ids;
    // The objects are already in the scene; undo will lift them out and park
    // them here, so nothing needs copying now.
    (void)scene;
    return cmd;
}

std::unique_ptr<ExistenceCommand> ExistenceCommand::forDelete(
        Scene& scene, const std::vector<ObjectId>& ids) {
    auto cmd = std::make_unique<ExistenceCommand>();
    cmd->created_ = false;
    cmd->ids_ = ids;
    for (ObjectId id : ids)
        if (auto obj = scene.takeObject(id)) cmd->objects_.push_back(std::move(obj));
    return cmd;
}

void ExistenceCommand::add(Scene& scene) {
    for (auto& o : objects_) scene.insertObject(std::move(o));
    objects_.clear();
}

void ExistenceCommand::remove(Scene& scene) {
    objects_.clear();
    for (ObjectId id : ids_)
        if (auto obj = scene.takeObject(id)) objects_.push_back(std::move(obj));
}

void ExistenceCommand::undo(Scene& scene) {
    if (created_) remove(scene);   // undoing a create removes
    else          add(scene);      // undoing a delete restores
}

void ExistenceCommand::redo(Scene& scene) {
    if (created_) add(scene);
    else          remove(scene);
}

// ---------------------------------------------------------------------------
void ParameterCommand::undo(Scene& scene) {
    if (SceneObject* o = scene.find(id_)) { o->spec = before_; scene.rebuild(id_); }
}

void ParameterCommand::redo(Scene& scene) {
    if (SceneObject* o = scene.find(id_)) { o->spec = after_; scene.rebuild(id_); }
}

bool ParameterCommand::mergeWith(const Command& other) {
    const auto* rhs = dynamic_cast<const ParameterCommand*>(&other);
    if (!rhs || rhs->id_ != id_) return false;
    after_ = rhs->after_;
    return true;
}

// ---------------------------------------------------------------------------
void VertexCommand::apply(Scene& scene, const std::vector<Vec3>& positions) {
    SceneObject* o = scene.find(id_);
    if (!o) return;
    for (size_t i = 0; i < verts_.size() && i < positions.size(); ++i)
        if (verts_[i] < o->mesh.vertexCount()) o->mesh.verts[verts_[i]].position = positions[i];
    o->refreshDerived();
}

bool VertexCommand::mergeWith(const Command& other) {
    const auto* rhs = dynamic_cast<const VertexCommand*>(&other);
    if (!rhs || rhs->id_ != id_ || rhs->verts_ != verts_) return false;
    after_ = rhs->after_;
    return true;
}

// ---------------------------------------------------------------------------
void MeshCommand::apply(Scene& scene, const Mesh& mesh, const PrimitiveSpec& spec) {
    SceneObject* o = scene.find(id_);
    if (!o) return;
    o->mesh = mesh;
    o->mesh.buildRenderMesh(o->render);
    o->localBounds = o->mesh.bounds();
    o->spec = spec;
    o->markMeshChanged();
    // Face and vertex numbering does not survive a mesh edit, so anything the
    // user had picked must be dropped rather than left pointing at whatever
    // now happens to occupy that index.
    scene.pruneElementSelection();
    scene.clearElementSelection();
}

void MeshCommand::undo(Scene& scene) { apply(scene, before_, specBefore_); }
void MeshCommand::redo(Scene& scene) { apply(scene, after_, specAfter_); }

// ---------------------------------------------------------------------------
void UndoStack::push(std::unique_ptr<Command> cmd, bool merge) {
    if (!cmd) return;

    if (merge && !mergeBarrier_ && !done_.empty() && done_.back()->mergeWith(*cmd)) {
        // Folded into the previous step; a new branch still invalidates redo.
        undone_.clear();
        return;
    }

    done_.push_back(std::move(cmd));
    undone_.clear();   // a new edit abandons the redo branch
    mergeBarrier_ = false;

    if (done_.size() > kMaxDepth)
        done_.erase(done_.begin(), done_.begin() + (done_.size() - kMaxDepth));
}

bool UndoStack::undo(Scene& scene) {
    // Stepping through history ends any gesture in progress.
    mergeBarrier_ = true;
    if (done_.empty()) return false;
    std::unique_ptr<Command> cmd = std::move(done_.back());
    done_.pop_back();
    cmd->undo(scene);
    undone_.push_back(std::move(cmd));
    return true;
}

bool UndoStack::redo(Scene& scene) {
    mergeBarrier_ = true;
    if (undone_.empty()) return false;
    std::unique_ptr<Command> cmd = std::move(undone_.back());
    undone_.pop_back();
    cmd->redo(scene);
    done_.push_back(std::move(cmd));
    return true;
}

} // namespace tg
