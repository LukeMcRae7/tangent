// Tangent - measurement, in the shape Fusion uses.
//
// Pick one entity to read its own dimensions (a vertex's position, an edge's
// length, a face's area and perimeter). Pick two to read the distance between
// them, broken down per axis, plus the angle where both have a direction.
//
// "Distance" means the true minimum distance between the two entities, not the
// distance between their midpoints. For a print part the useful question is
// almost always a clearance or a wall thickness, and midpoints answer neither.
#pragma once

#include "app/camera.h"
#include "render/renderer.h"
#include "scene/scene.h"

#include <string>
#include <vector>

namespace tg {

struct MeasureResult {
    bool valid = false;

    // Where the measurement is taken, in world space. These are the two ends
    // of the line the viewport draws.
    Vec3 from, to;

    Real distance = 0.0;   // minimum distance between the picks
    Vec3 delta;            // to - from, per axis

    bool hasAngle = false;
    Real angleDeg = 0.0;

    // Single-entity readings.
    bool hasLength = false;
    Real length = 0.0;
    bool hasArea = false;
    Real area = 0.0;
    Real perimeter = 0.0;

    std::string summary;   // one line for the viewport label
};

class MeasureTool {
public:
    bool active() const { return active_; }
    void begin() { active_ = true; picks_.clear(); }
    void end() { active_ = false; picks_.clear(); }
    void clearPicks() { picks_.clear(); }

    // Adds a pick, or removes it if it was already chosen. A third pick starts
    // a fresh measurement rather than silently ignoring the click.
    void pick(const ElementRef& e);

    const std::vector<ElementRef>& picks() const { return picks_; }

    MeasureResult compute(const Scene& scene) const;
    void drawOverlay(Renderer& renderer, const Camera& camera,
                     const MeasureResult& result) const;

private:
    bool active_ = false;
    std::vector<ElementRef> picks_;
};

} // namespace tg
