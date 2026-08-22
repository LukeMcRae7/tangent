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
};

} // namespace tg
