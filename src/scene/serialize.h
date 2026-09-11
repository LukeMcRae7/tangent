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

inline constexpr uint32_t kProjectVersion = 6;   // 3: features name what they act on
                                                //  4: geometry is a tagged Body
                                                //  5: bodies may be exact, and
                                                //     say which kernel built them
                                                //  6: a feature may be a shell,
                                                //     which carries a wall thickness

// The oldest a file may be and still open. Reading an older format costs a
// branch or two and keeps someone's work openable; writing one does not, so
// saving always writes the current version.
inline constexpr uint32_t kMinReadableVersion = 4;

struct ProjectResult {
    bool ok = false;
    size_t objects = 0;
    std::string error;
};

ProjectResult saveProject(const Scene& scene, const std::string& path);

// Replaces the scene's contents on success; leaves it untouched on failure.
ProjectResult loadProject(Scene& scene, const std::string& path);

} // namespace tg
