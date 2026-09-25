// Tangent - choosing the plane something is drawn on.
//
// The same for every tool that draws on a plane -- a sketch, or the profile of a
// new shape -- so that the first thing both ask looks and answers the same way:
// the three origin planes as tiles, a face of a body under the pointer, and two
// ways of making a plane where none stands yet:
//
//   Three points   click three corners of the model; the plane passes through
//                  them, the first two giving its direction across
//   Along an edge  click an edge; the plane stands square to it at the end
//                  nearer the click -- where a sweep's profile is drawn
//
// Once chosen, a plane can stand off where it was put by a distance and be
// tilted about its own horizontal: offsetFrame() says where that puts it.
#pragma once

#include "app/camera.h"
#include "app/snap.h"
#include "render/renderer.h"
#include "scene/scene.h"

#include <string>
#include <vector>

namespace tg {

// Which plane: an origin plane, or one made some other way -- on a face,
// through points, at an edge.
enum class PlaneChoice { None, XY, XZ, YZ, Face };

enum class PlaneMethod { Surface, ThreePoints, AlongEdge };

// A plane's frame from its origin and normal, the directions across it chosen
// the way every tool here chooses them: X across when the plane faces up or
// forward, Y across when it faces sideways.
PlaneFrame planeFrameFor(Vec3 origin, Vec3 normal);

// `base` moved `offset` along its normal and turned `tiltRad` about its own
// horizontal (u), through its origin.
PlaneFrame offsetFrame(const PlaneFrame& base, Real offset, Real tiltRad);

class PlanePicker {
public:
    void reset();

    PlaneMethod method() const { return method_; }
    void setMethod(PlaneMethod m);

    // What a click would choose, for the pointer where it is.
    void update(const Scene& scene, const Camera& camera, Vec2 mousePx);
    // A click. True when it decided the plane: frame() and the rest describe it.
    bool click(const Scene& scene, const Camera& camera);
    // One of the origin planes, straight away.
    bool choose(PlaneChoice choice);

    PlaneChoice choice() const { return choice_; }
    const PlaneFrame& frame() const { return frame_; }
    ObjectId faceObject() const { return faceObject_; }
    Index faceIndex() const { return faceIndex_; }

    // The tiles, what the pointer is on, and the points picked so far.
    void drawOverlay(const Scene& scene, const Camera& camera, Renderer& renderer) const;
    // The panel rows. True when a plane was decided from the panel.
    bool drawRows(const char* hint);
    // What the next click does, in a few words.
    std::string prompt() const;
    // 7, 1, 3 for the origin planes. True when the key decided the plane.
    bool handleKey(int key);

private:
    PlaneMethod method_ = PlaneMethod::Surface;
    PlaneChoice choice_ = PlaneChoice::XY;
    PlaneFrame frame_;
    ObjectId faceObject_ = kNoObject;
    Index faceIndex_ = kInvalid;

    // Three points: those clicked, and the one under the pointer.
    std::vector<Vec3> points_;
    bool hoverOk_ = false;
    Vec3 hoverPoint_{};
    // Along an edge: the edge under the pointer, drawn, and the plane it gives.
    std::vector<Vec3> hoverEdge_;
    PlaneFrame edgeFrame_;
};

} // namespace tg
