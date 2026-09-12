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

// Whether tryInChild actually isolates. False on platforms without fork, where
// a crash in `work` is still fatal and callers should probe less adventurously.
bool childIsolationAvailable();

} // namespace tg
