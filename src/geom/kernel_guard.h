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
// So an operation that has not been proven safe is tried in another process
// first, and a crash there costs a process that owns nothing -- no window, no
// file, no scene -- and reads to the caller as one more way for an operation
// to be refused. Only what survived is run for real.
//
// Two ways to get another process:
//
//   Fork, where there is one. The child is a copy of this process at that
//   moment, so the work is simply called there; nothing has to be described.
//
//   A worker, where there is not -- Windows. tangent_trial runs beside the
//   application and is handed the work as bytes, which is why guarded work
//   carries a way to write itself down as well as a way to run. The worker
//   stays up between trials, because starting a process and loading the kernel
//   costs far more than any one trial; when a trial takes it down, the next one
//   starts another.
//
// TANGENT_ISOLATION=worker picks the worker where fork is available too, which
// is how that path is tested on Linux; =none runs guarded work directly.
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace tg {

enum class Attempt {
    Ok,        // it built
    Refused,   // it declined, in the ordinary way, with a reason
    Crashed,   // it took its process down; treat as a refusal that cannot explain itself
};

enum class Isolation { Fork, Worker, None };

// How guarded work is run in this process, decided once.
Isolation isolation();

// Whether a crash in guarded work is survivable here.
bool childIsolationAvailable();

// Whether tryInChild itself isolates, which is only where there is fork.
//
// Not the same question as the one above. On Windows guarded work is
// survivable -- through the worker -- but a bare closure cannot be handed to a
// worker, so tryInChild runs it in this process and a crash in it is fatal.
// A probe built on tryInChild asks this; one built on tryIsolated asks
// childIsolationAvailable.
bool forkIsolationAvailable();

// Work to be run where a crash is survivable.
//
// `run` is the work. `kind` and `encode` are the same work written down, for a
// worker in another process to rebuild: `encode` is only called when a worker
// is used, so describing the work costs nothing where fork is enough. The
// worker's side is `runEncodedTrial` in scene/trials.h, which knows every kind.
struct GuardedWork {
    std::function<bool()> run;
    uint32_t kind = 0;
    std::function<std::string()> encode;
};

// Runs `work` where a crash is survivable, and waits for the answer.
Attempt tryIsolated(const GuardedWork& work);

// Runs a closure in a forked child, where there is fork, and directly where
// there is not. For tests and probes that only make sense on a platform that
// can fork; the application uses tryIsolated, which is guarded everywhere.
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
    AsyncTrial();
    ~AsyncTrial();
    AsyncTrial(const AsyncTrial&) = delete;
    AsyncTrial& operator=(const AsyncTrial&) = delete;

    // Begins `work` elsewhere. Where that is not possible the work runs here
    // and finishes before this returns, which is correct but not free.
    void start(const GuardedWork& work);

    bool running() const;
    bool finished() const { return done_; }

    // Collects the answer if there is one yet. Never blocks.
    bool poll();

    // Waits for the answer, however long it takes.
    Attempt wait();

    Attempt result() const { return result_; }

    // Gives up on a trial still in flight, without waiting for it.
    void abandon();

private:
    struct Worker;
    int     pid_ = -1;                     // a forked child, while one runs
    std::unique_ptr<Worker> worker_;       // kept between trials
    bool    waitingOnWorker_ = false;
    bool    done_ = false;
    Attempt result_ = Attempt::Refused;
};

// True in a process forked to run kernel work in isolation.
//
// Such a child has exactly one thread -- the one that forked -- but inherits
// the bookkeeping of every thread pool its parent had started, OpenCASCADE's
// included. A parallel boolean run in it waits for workers that were never
// copied across, and waits forever: turning on OCCT's parallel mode hung both
// tests that exercise isolation, parent and child at zero CPU. Anything that
// would spread work across threads asks this first and stays on one.
//
// A worker process is not such a child. It started as itself, its thread
// pools are its own, and it runs in parallel like anything else.
bool inIsolatedChild();

// The worker's main loop: reads work from standard input, runs it with `run`,
// answers on standard output, and returns when its input closes. What
// tangent_trial's main is.
using EncodedRunner = bool (*)(uint32_t kind, const std::string& payload);
int serveTrials(EncodedRunner run);

} // namespace tg
