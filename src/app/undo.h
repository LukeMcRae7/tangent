// Tangent - undo/redo.
//
// Command-based rather than snapshot-based: a scene holds full meshes, so
// snapshotting every edit would copy megabytes for a one-millimetre nudge.
// Each command carries only what it needs to reverse itself.
#pragma once

#include "scene/scene.h"

#include <memory>
#include <string>
#include <vector>

namespace tg {

class Command {
public:
    virtual ~Command() = default;
    virtual void undo(Scene& scene) = 0;
    virtual void redo(Scene& scene) = 0;
    virtual std::string label() const = 0;

    // Lets a run of small edits of the same kind (dragging one slider, nudging
    // one object) collapse into a single undo step instead of flooding the
    // stack with one entry per frame.
    virtual bool mergeWith(const Command&) { return false; }
};

// Moves, rotates or scales any number of objects.
class TransformCommand : public Command {
public:
    struct Entry {
        ObjectId  id;
        Transform before;
        Transform after;
    };

    TransformCommand(std::vector<Entry> entries, std::string what)
        : entries_(std::move(entries)), what_(std::move(what)) {}

    void undo(Scene& scene) override;
    void redo(Scene& scene) override;
    std::string label() const override { return what_; }
    bool mergeWith(const Command& other) override;

private:
    std::vector<Entry> entries_;
    std::string what_;
};

// Creating and deleting are the same operation viewed from opposite ends, so
// they share one implementation and differ only in which direction "do" runs.
class ExistenceCommand : public Command {
public:
    static std::unique_ptr<ExistenceCommand> forCreate(const Scene& scene,
                                                       const std::vector<ObjectId>& ids);
    static std::unique_ptr<ExistenceCommand> forDelete(Scene& scene,
                                                       const std::vector<ObjectId>& ids);

    void undo(Scene& scene) override;
    void redo(Scene& scene) override;
    std::string label() const override { return created_ ? "Create" : "Delete"; }

private:
    void add(Scene& scene);
    void remove(Scene& scene);

    std::vector<std::unique_ptr<SceneObject>> objects_;
    std::vector<ObjectId> ids_;
    bool created_ = false;
};

// Changing a primitive's parameters, re-evaluating the mesh either way.
class ParameterCommand : public Command {
public:
    ParameterCommand(ObjectId id, PrimitiveSpec before, PrimitiveSpec after)
        : id_(id), before_(before), after_(after) {}

    void undo(Scene& scene) override;
    void redo(Scene& scene) override;
    std::string label() const override { return "Change Parameters"; }
    bool mergeWith(const Command& other) override;

private:
    ObjectId      id_;
    PrimitiveSpec before_, after_;
};

// A mesh edit (extrude, inset, bevel...). Holds full before/after meshes:
// operations rewrite connectivity wholesale, so there is no compact delta to
// store, and a print-design mesh is small enough for this to be cheap.
class MeshCommand : public Command {
public:
    MeshCommand(ObjectId id, Mesh before, Mesh after,
                PrimitiveSpec specBefore, PrimitiveSpec specAfter, std::string what)
        : id_(id), before_(std::move(before)), after_(std::move(after)),
          specBefore_(specBefore), specAfter_(specAfter), what_(std::move(what)) {}

    void undo(Scene& scene) override;
    void redo(Scene& scene) override;
    std::string label() const override { return what_; }

private:
    void apply(Scene& scene, const Mesh& mesh, const PrimitiveSpec& spec);

    ObjectId      id_;
    Mesh          before_, after_;
    PrimitiveSpec specBefore_, specAfter_;
    std::string   what_;
};

// Moving vertices, as opposed to changing topology. Stores only the positions
// that moved, so dragging a face on a heavy mesh costs a handful of vectors
// rather than two full copies of it.
class VertexCommand : public Command {
public:
    VertexCommand(ObjectId id, std::vector<Index> verts,
                  std::vector<Vec3> before, std::vector<Vec3> after, std::string what)
        : id_(id), verts_(std::move(verts)), before_(std::move(before)),
          after_(std::move(after)), what_(std::move(what)) {}

    void undo(Scene& scene) override { apply(scene, before_); }
    void redo(Scene& scene) override { apply(scene, after_); }
    std::string label() const override { return what_; }
    bool mergeWith(const Command& other) override;

    // Lets the application turn a completed drag into a history entry.
    ObjectId object() const { return id_; }
    const std::vector<Index>& vertices() const { return verts_; }
    const std::vector<Vec3>& beforePositions() const { return before_; }
    const std::vector<Vec3>& afterPositions() const { return after_; }

private:
    void apply(Scene& scene, const std::vector<Vec3>& positions);

    ObjectId           id_;
    std::vector<Index> verts_;
    std::vector<Vec3>  before_, after_;
    std::string        what_;
};

// Any change to an object's feature chain: adding an operation, toggling one
// off, editing a parameter. Stores the chain either side and re-evaluates,
// which is cheap next to storing meshes and is the only representation that
// stays correct when a later edit re-runs the whole thing.
class FeatureCommand : public Command {
public:
    FeatureCommand(ObjectId id, std::vector<Feature> before,
                   std::vector<Feature> after, std::string what)
        : id_(id), before_(std::move(before)), after_(std::move(after)),
          what_(std::move(what)) {}

    void undo(Scene& scene) override { apply(scene, before_); }
    void redo(Scene& scene) override { apply(scene, after_); }
    std::string label() const override { return what_; }
    bool mergeWith(const Command& other) override;

private:
    void apply(Scene& scene, const std::vector<Feature>& chain);

    ObjectId             id_;
    std::vector<Feature> before_, after_;
    std::string          what_;
};

// Several commands that must move together. A boolean edits one object's chain
// and removes another; undoing half of that would leave the scene inconsistent.
class CompositeCommand : public Command {
public:
    CompositeCommand(std::vector<std::unique_ptr<Command>> parts, std::string what)
        : parts_(std::move(parts)), what_(std::move(what)) {}

    void undo(Scene& scene) override {
        // Reverse order: the last thing done is the first thing undone.
        for (auto it = parts_.rbegin(); it != parts_.rend(); ++it) (*it)->undo(scene);
    }
    void redo(Scene& scene) override {
        for (auto& p : parts_) p->redo(scene);
    }
    std::string label() const override { return what_; }

private:
    std::vector<std::unique_ptr<Command>> parts_;
    std::string what_;
};

class UndoStack {
public:
    // `merge` collapses this command into the previous one when they represent
    // one continuous gesture.
    void push(std::unique_ptr<Command> cmd, bool merge = false);

    // Ends the current gesture, so the next push starts a fresh entry even if
    // it asks to merge. Without this, releasing a slider and dragging it again
    // would fold both drags into one undo step -- and an edit made after an
    // undo could merge into a command from before it.
    void breakMergeChain() { mergeBarrier_ = true; }

    bool undo(Scene& scene);
    bool redo(Scene& scene);

    // Bumped by anything that changes the model, so the application can tell
    // whether there is unsaved work without threading a dirty flag through
    // every edit path.
    size_t revision() const { return revision_; }

    bool canUndo() const { return !done_.empty(); }
    bool canRedo() const { return !undone_.empty(); }
    std::string undoLabel() const { return done_.empty() ? "" : done_.back()->label(); }
    std::string redoLabel() const { return undone_.empty() ? "" : undone_.back()->label(); }
    void clear() { done_.clear(); undone_.clear(); }

private:
    static constexpr size_t kMaxDepth = 200;

    std::vector<std::unique_ptr<Command>> done_;
    std::vector<std::unique_ptr<Command>> undone_;
    bool mergeBarrier_ = false;
    size_t revision_ = 0;
};

} // namespace tg
