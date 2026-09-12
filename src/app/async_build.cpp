#include "app/async_build.h"

namespace tg {

void AsyncBuild::request(const Body& base, std::function<bool(Body&)> build) {
    pendingBase_ = base;
    pendingWork_ = std::move(build);
    hasPending_ = true;
    if (!running_) startPending();
}

void AsyncBuild::startPending() {
    inFlightInput_ = std::move(pendingBase_);
    inFlightWork_ = std::move(pendingWork_);
    hasPending_ = false;
    pendingBase_ = Body();
    pendingWork_ = nullptr;

    done_.store(false, std::memory_order_release);
    running_ = true;

    // Everything the worker reads was fixed before it started and is not
    // touched again until `done_` is seen, which is what makes the handover
    // safe without a lock.
    worker_ = std::thread([this] {
        Body scratch = inFlightInput_;
        const bool ok = inFlightWork_ && inFlightWork_(scratch);
        if (ok) result_ = std::move(scratch);
        ok_ = ok;
        done_.store(true, std::memory_order_release);
    });
}

bool AsyncBuild::take(Body& out, bool* failed) {
    if (failed) *failed = false;
    if (!running_) {
        if (hasPending_) startPending();
        return false;
    }
    if (!done_.load(std::memory_order_acquire)) return false;

    worker_.join();
    running_ = false;

    const bool ok = ok_;
    if (failed) *failed = !ok;
    if (ok) out = std::move(result_);
    result_ = Body();
    inFlightInput_ = Body();
    inFlightWork_ = nullptr;

    // The pointer kept moving while that was building, so the next build starts
    // now rather than waiting for another request.
    if (hasPending_) startPending();
    return ok;
}

void AsyncBuild::cancel() {
    if (running_) {
        worker_.join();
        running_ = false;
    }
    hasPending_ = false;
    result_ = Body();
    inFlightInput_ = Body();
    pendingBase_ = Body();
    inFlightWork_ = nullptr;
    pendingWork_ = nullptr;
}

} // namespace tg
