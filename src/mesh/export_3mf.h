// Tangent - 3MF output.
//
// The format a print-first tool should be handing a slicer. Where STL is loose
// float32 triangles with no units and no idea how many parts it holds, 3MF says
// millimetres, keeps each object separate and named, and stores an indexed mesh
// -- shared vertices, so a closed surface is closed in the file rather than
// only after the slicer has welded it back together.
//
// Each object is written in world space, as STL is, with an exact body
// tessellated to the tolerance the user sets. Its triangles are welded into
// shared vertices, and every object is checked for being closed before it goes
// out; one that is not is still written -- a slicer can often repair it -- and
// counted in the result so the user is told.
#pragma once

#include "scene/scene.h"

#include <string>
#include <vector>

namespace tg {

struct ThreeMfOptions {
    bool selectionOnly = false; // otherwise every visible object
    Real deviationMm = 0.01;    // see StlOptions::deviationMm
};

struct ThreeMfResult {
    bool   ok = false;
    size_t objects = 0;
    size_t triangles = 0;
    size_t vertices = 0;
    size_t meshBodies = 0;      // written at whatever resolution they already had
    size_t openObjects = 0;     // written, but not closed; see above
    std::string error;
};

ThreeMfResult export3mf(const Scene& scene, const std::string& path,
                        const ThreeMfOptions& options = {});

// The indexed mesh an object goes out as, in world space. Exposed so a test
// can hold the file's contents against what was meant to be in it.
struct ExportMesh {
    std::vector<Vec3> vertices;
    std::vector<uint32_t> triangles;    // three per triangle, wound outward
    bool closed = false;                // every edge used once each way
};
void exportMeshOf(const SceneObject& obj, Real deviationMm, ExportMesh& out);

} // namespace tg
