// Tangent - project files.
//
// The chain is what gets written, not the evaluated mesh: a project is the
// recipe, and reopening it re-runs that recipe. Baked geometry (a boolean's
// tool body, a split body) is stored too, because nothing else describes it.
//
// Binary, because those baked meshes make a text format both large and slow,
// and little-endian fixed-width so a file written on one machine reads on
// another. Every file starts with a version; a reader refuses anything it does
// not recognise rather than guessing at the layout.
#pragma once

#include "scene/scene.h"

#include <string>

namespace tg {

inline constexpr uint32_t kProjectVersion = 3;   // 3: features name what they act on

struct ProjectResult {
    bool ok = false;
    size_t objects = 0;
    std::string error;
};

ProjectResult saveProject(const Scene& scene, const std::string& path);

// Replaces the scene's contents on success; leaves it untouched on failure.
ProjectResult loadProject(Scene& scene, const std::string& path);

} // namespace tg
