// Tangent - STL output.
//
// STL is the format slicers actually take, and it is the reason this is a mesh
// modeller: the geometry goes out without conversion. It stores loose
// triangles in float32 with no shared vertices and no units, so the writer's
// job is to triangulate, transform into world space, and narrow.
#pragma once

#include "scene/scene.h"

#include <string>

namespace tg {

struct StlOptions {
    bool binary = true;         // ASCII is larger and slower; useful for diffing
    bool selectionOnly = false; // otherwise every visible object
    std::string solidName = "tangent";
};

struct StlResult {
    bool   ok = false;
    size_t triangles = 0;
    size_t objects = 0;
    std::string error;
};

// Writes the scene to `path`. Objects are emitted in world space, so an
// object's transform is baked in -- a slicer has no concept of one.
StlResult exportStl(const Scene& scene, const std::string& path,
                    const StlOptions& options = {});

} // namespace tg
