#pragma once

#include "geom/body.h"
#include "geom/operations.h"

#include <atomic>
#include <thread>
#include <vector>

namespace tg {

// Builds fillet previews off the frame thread.
//
// The exact kernel takes twenty to fifty milliseconds to round eight edges,
// which is three frames at sixty hertz. Doing it inline meant the drag stopped
// dead every time the cursor crossed a tick: the number and the guide froze
// along with the geometry, so the gesture felt like it was fighting back.
//
// So the frame thread asks for a radius and carries on drawing. One build runs
// at a time; asking again while one is in flight replaces what will be built
// next, so a fast drag skips the radii it passed through instead of queueing
// them. The geometry trails the pointer by one build and the pointer never
// waits.
//
// The body is a shared_ptr to an immutable shape, so handing a copy to the
// worker shares it rather than duplicating it, and nothing the worker touches
// is visible to the frame thread until take() hands it back.
class FilletPreview {
public:
    FilletPreview() = default;
    ~FilletPreview() { cancel(); }

    FilletPreview(const FilletPreview&) = delete;
    FilletPreview& operator=(const FilletPreview&) = delete;

    // Asks for this radius. Returns immediately.
    void request(const Body& base, const std::vector<EdgeId>& edges,
                 Real radius, int segments);

    // Hands back a finished build, and starts the next one if the pointer has
    // moved on. Call once a frame. Returns false when there is nothing new,
    // which is most frames.
    bool take(Body& out, Real& radius, int& segments);

    bool busy() const { return running_ || hasPending_; }

    // Waits for the build in flight and drops it. The worker holds references
    // into bodies this object owns, so it has to be finished with before any
    // of them go away.
    void cancel();

private:
    void startPending();

    std::thread worker_;
    std::atomic<bool> done_{false};
    bool running_ = false;

    bool hasPending_ = false;
    Body pendingBase_;
    std::vector<EdgeId> pendingEdges_;
    Real pendingRadius_ = 0.0;
    int  pendingSegments_ = 0;

    // What the worker is reading, and what it wrote. Touched by the frame
    // thread only when `running_` is false or `done_` is true.
    Body inFlightInput_;
    std::vector<EdgeId> inFlightEdges_;
    Real inFlightRadius_ = 0.0;
    int  inFlightSegments_ = 0;
    Body result_;
    bool ok_ = false;
};

} // namespace tg
