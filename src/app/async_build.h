// Tangent - a body built off the frame thread.
//
// The exact kernel takes twenty to fifty milliseconds to round eight edges or
// to sweep a face and combine it, which is three frames at sixty hertz. Doing
// that inline made a drag stop dead every time the cursor crossed a tick: the
// number and the guide froze along with the geometry, so the gesture felt like
// it was fighting back.
//
// So the frame thread asks for a shape and carries on drawing. One build runs
// at a time; asking again while one is in flight replaces what will be built
// next, so a fast drag skips the values it passed through instead of queueing
// them. The geometry trails the pointer by one build and the pointer never
// waits.
//
// A Body is a shared_ptr to an immutable shape, so handing a copy to the worker
// shares it rather than duplicating it, and nothing the worker touches is
// visible to the frame thread until take() hands it back.
#pragma once

#include "geom/body.h"

#include <atomic>
#include <functional>
#include <thread>

namespace tg {

class AsyncBuild {
public:
    AsyncBuild() = default;
    ~AsyncBuild() { cancel(); }

    AsyncBuild(const AsyncBuild&) = delete;
    AsyncBuild& operator=(const AsyncBuild&) = delete;

    // Asks for a shape: `build` is handed a copy of `base` to work on and
    // returns whether it managed it. Returns immediately.
    void request(const Body& base, std::function<bool(Body&)> build);

    // Hands back a finished build, and starts the next one if the pointer has
    // moved on. Call once a frame. Returns false when there is nothing new,
    // which is most frames.
    // `failed`, when given, says that a build finished and did not manage it
    // -- which is different from there being nothing new, and is how a caller
    // learns where its range runs out.
    bool take(Body& out, bool* failed = nullptr);

    bool busy() const { return running_ || hasPending_; }

    // Waits for the build in flight and drops it. The worker holds a body this
    // object owns, so it has to be finished with before that goes away.
    void cancel();

private:
    void startPending();

    std::thread worker_;
    std::atomic<bool> done_{false};
    bool running_ = false;

    bool hasPending_ = false;
    Body pendingBase_;
    std::function<bool(Body&)> pendingWork_;

    // What the worker is reading, and what it wrote. Touched by the frame
    // thread only when `running_` is false or `done_` is true.
    Body inFlightInput_;
    std::function<bool(Body&)> inFlightWork_;
    Body result_;
    bool ok_ = false;
};

} // namespace tg
