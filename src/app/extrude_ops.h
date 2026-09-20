// Tangent - what an extrusion does to the bodies it reaches.
//
// Three tools extrude: a face pulled off a body (E), the create tool's profile,
// and a region of a sketch. They answer the same two questions the same way, so
// the answers live here rather than three times over.
//
// Which operation. Join, Cut, Intersect or a New Body -- and no "Auto" among
// them. Until the user picks one, the drag decides: out of a body joins, into
// one cuts, and out of nothing is a body of its own. The panel shows which it
// has decided on, and picking one ends the deciding.
//
// Which bodies. An extrusion acts on every body it reaches, not only the one it
// started from -- a cut drawn across two parts cuts both -- and the panel lists
// them so any can be taken out.
#pragma once

#include "app/undo.h"
#include "geom/op_types.h"
#include "scene/scene.h"

#include <memory>
#include <string>
#include <vector>

namespace tg {

struct ExtrudeChoice {
    ExtrudeOp op = ExtrudeOp::Join;
    bool automatic = true;

    // While nothing has been picked: into a body cuts, out of one joins, and
    // out of nothing makes a new one.
    void follow(Real depth, bool fromBody) {
        if (!automatic) return;
        op = depth < 0.0 ? ExtrudeOp::Cut : fromBody ? ExtrudeOp::Join : ExtrudeOp::NewBody;
    }
    void pick(ExtrudeOp o) { op = o; automatic = false; }
    void reset() { op = ExtrudeOp::Join; automatic = true; }
};

// The key that picks each operation, shared by every tool that offers them.
// Returns false for any other key.
bool extrudeOpForKey(int key, ExtrudeOp& out);

// A verb for a sentence: "joins", "cuts", ...
const char* extrudeOpVerb(ExtrudeOp op);

struct ReachedBody {
    ObjectId id = kNoObject;
    bool included = true;
};

// The bodies an extrusion reaches, and which of them it is to act on.
//
// Reached means touching or passing through, of every visible exact body: a
// mesh cannot be combined with anything, and a sketch is not material. The body
// the extrusion starts from -- `owner` -- counts only on the side the operation
// acts on it: a join grows out of it, a cut or an intersect goes into it, and
// the other way it is not touched however close it is. A body the user has
// taken out stays out for as long as it is still reached.
class ExtrudeReach {
public:
    void clear() { bodies_.clear(); excluded_.clear(); key_.clear(); }

    // `toolWorld` is the swept solid, in the world. Cheap to call every frame:
    // nothing is measured again unless the tool, the operation or the bodies
    // around it have changed.
    void refresh(const Scene& scene, const Body& toolWorld, ObjectId owner, ExtrudeOp op, Real depth,
                 const std::string& toolKey);

    const std::vector<ReachedBody>& bodies() const { return bodies_; }
    void toggle(ObjectId id);
    bool includes(ObjectId id) const;

    // The ones to act on, the owner first when it is one of them.
    std::vector<ObjectId> included() const;

private:
    std::vector<ReachedBody> bodies_;
    std::vector<ObjectId> excluded_;
    std::string key_;
};

// Applies a swept solid, in the world, to `bodies` -- as steps in their
// histories, collected into `parts` for one undo entry.
//
// `ownerDone` is a body whose own history has already taken the extrusion in,
// the way a face extrude or a sketch region does, parametrically; it is left
// alone here. A join merges every body it reaches into the first of them, as
// one body is what joining two makes; `joinInto` names that first body. A cut
// and an intersect act on each body on its own. A cut that turns out not to
// touch a body's material is dropped rather than recorded as a step that does
// nothing. All or nothing: on a refusal everything done so far is undone and
// `error` says which body refused and why.
bool applyExtrude(Scene& scene, const Body& toolWorld, ExtrudeOp op,
                  const std::vector<ObjectId>& bodies, ObjectId ownerDone,
                  const std::string& toolName, const char* label,
                  std::vector<std::unique_ptr<Command>>& parts, std::string& error);

// Undoes what `parts` did, last first, and empties it.
void unwind(Scene& scene, std::vector<std::unique_ptr<Command>>& parts);

// What the panel says about the bodies, and the row that lets them be taken
// out. Returns the body clicked, or kNoObject.
ObjectId drawReachedBodies(const Scene& scene, const ExtrudeReach& reach, ExtrudeOp op,
                           ObjectId owner);

// The operation tiles. Returns true if one was picked.
bool drawExtrudeChoice(ExtrudeChoice& choice, bool allowNewBody = true);

} // namespace tg
