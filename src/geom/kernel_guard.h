// Tangent - running a kernel call that might not come back.
//
// OpenCASCADE does not always refuse. On some inputs it dereferences a null
// pointer and the process is gone -- no exception, nothing to catch. The one
// found first is a fillet whose radius is exactly the thickness of the wall it
// is rounding: 1.9mm builds, 3.0mm refuses politely, 2.0mm segfaults. That is
// not an exotic number, it is the default fillet width.
//
// A modelling tool may refuse an operation. It may not lose the user's work
// because a library walked off the end of something.
//
// So an operation that has not been proven safe is tried in a forked child
// first. The child does the work and exits with a code; the parent waits. A
// crash there costs a process that owns nothing -- no window, no file, no
// scene -- and reads to the caller as one more way for an operation to be
// refused. Only a radius that survived the child is run for real.
//
// Windows has no fork. There the call runs directly, as it always has, and the
// comment on Attempt::Crashed says what that means.
#pragma once

#include <functional>

namespace tg {

enum class Attempt {
    Ok,        // it built
    Refused,   // it declined, in the ordinary way, with a reason
    Crashed,   // it took its process down; treat as a refusal that cannot explain itself
};

// Runs `work` where a crash is survivable, if the platform allows it.
//
// `work` must be self-contained: it runs in a child that shares the parent's
// memory at the moment of the fork and whose side effects are discarded. Use it
// to find out *whether* something works, then do it again for real.
Attempt tryInChild(const std::function<bool()>& work);

// The same isolation, without stopping to wait for it.
//
// A search for how far an operation can go is a dozen trials, and a dozen
// trials done one after another in the frame loop is most of a second with
// nothing on screen moving. Started and polled instead, the work happens in
// another process while this one keeps drawing, and the answer is picked up
// whenever it is ready.
class AsyncTrial {
public:
    AsyncTrial() = default;
    ~AsyncTrial();
    AsyncTrial(const AsyncTrial&) = delete;
    AsyncTrial& operator=(const AsyncTrial&) = delete;

    // Begins `work` elsewhere. Where that is not possible the work runs here
    // and finishes before this returns, which is correct but not free.
    void start(const std::function<bool()>& work);

    bool running() const { return pid_ >= 0; }
    bool finished() const { return done_; }

    // Collects the answer if there is one yet. Never blocks.
    bool poll();

    Attempt result() const { return result_; }

    // Gives up on a trial still in flight, without waiting for it.
    void abandon();

private:
    int     pid_ = -1;
    bool    done_ = false;
    Attempt result_ = Attempt::Refused;
};

// Whether tryInChild actually isolates. False on platforms without fork, where
// a crash in `work` is still fatal and callers should probe less adventurously.
bool childIsolationAvailable();

} // namespace tg
