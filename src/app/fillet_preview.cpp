#include "app/fillet_preview.h"

namespace tg {

void FilletPreview::request(const Body& base, const std::vector<EdgeId>& edges,
                            Real radius, int segments) {
    pendingBase_ = base;
    pendingEdges_ = edges;
    pendingRadius_ = radius;
    pendingSegments_ = segments;
    hasPending_ = true;
    if (!running_) startPending();
}

void FilletPreview::startPending() {
    inFlightInput_ = std::move(pendingBase_);
    inFlightEdges_ = std::move(pendingEdges_);
    inFlightRadius_ = pendingRadius_;
    inFlightSegments_ = pendingSegments_;
    hasPending_ = false;
    pendingBase_ = Body();

    done_.store(false, std::memory_order_release);
    running_ = true;

    // Everything the worker reads was fixed before it started and is not
    // touched again until `done_` is seen, which is what makes the handover
    // safe without a lock.
    worker_ = std::thread([this] {
        Body scratch = inFlightInput_;
        FilletSpec spec;
        spec.segments = inFlightSegments_;
        for (EdgeId e : inFlightEdges_) spec.edges.push_back({e, inFlightRadius_});
        const bool ok = filletEdges(scratch, spec);
        if (ok) result_ = std::move(scratch);
        ok_ = ok;
        done_.store(true, std::memory_order_release);
    });
}

bool FilletPreview::take(Body& out, Real& radius, int& segments) {
    if (!running_) {
        if (hasPending_) startPending();
        return false;
    }
    if (!done_.load(std::memory_order_acquire)) return false;

    worker_.join();
    running_ = false;

    const bool ok = ok_;
    if (ok) {
        out = std::move(result_);
        radius = inFlightRadius_;
        segments = inFlightSegments_;
    }
    result_ = Body();
    inFlightInput_ = Body();
    inFlightEdges_.clear();

    // The pointer kept moving while that was building, so the next build starts
    // now rather than waiting for another request.
    if (hasPending_) startPending();
    return ok;
}

void FilletPreview::cancel() {
    if (running_) {
        worker_.join();
        running_ = false;
    }
    hasPending_ = false;
    result_ = Body();
    inFlightInput_ = Body();
    pendingBase_ = Body();
    inFlightEdges_.clear();
    pendingEdges_.clear();
}

} // namespace tg
