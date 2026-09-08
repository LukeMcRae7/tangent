// Tangent - STL output.
//
// STL is the format slicers actually take. It stores loose triangles in float32
// with no shared vertices and no units, so the writer's job is to triangulate,
// transform into world space, and narrow.
//
// For an exact body the triangulation is the export's own, not the one on
// screen. That is the whole difference between a print-first tool and one that
// exports whatever the viewport happened to be showing: the tolerance is a
// number the user sets in millimetres, matched to their printer, and it is the
// only thing that decides how round a hole comes out.
#pragma once

#include "scene/scene.h"

#include <string>

namespace tg {

struct StlOptions {
    bool binary = true;         // ASCII is larger and slower; useful for diffing
    bool selectionOnly = false; // otherwise every visible object
    std::string solidName = "tangent";

    // Chord deviation in millimetres: how far the triangles may sit from the
    // surface they stand in for. 0.01mm is finer than any filament printer
    // resolves and is a reasonable default to ship; a resin printer might want
    // 0.005, and a draft print 0.05.
    //
    // Only an exact body can honour it. A mesh body is already triangles and
    // goes out as it is -- its resolution was decided when it was made, which
    // is exactly the limitation the exact kernel exists to remove.
    Real deviationMm = 0.01;
};

struct StlResult {
    bool   ok = false;
    size_t triangles = 0;
    size_t objects = 0;
    size_t meshBodies = 0;   // written at whatever resolution they already had
    std::string error;
};

// Writes the scene to `path`. Objects are emitted in world space, so an
// object's transform is baked in -- a slicer has no concept of one.
StlResult exportStl(const Scene& scene, const std::string& path,
                    const StlOptions& options = {});

} // namespace tg
