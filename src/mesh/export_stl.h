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
#include <vector>

namespace tg {

struct StlOptions {
    bool binary = true;         // ASCII is larger and slower; useful for diffing
    bool selectionOnly = false; // otherwise every visible object

    // A file for each object rather than one for all of them, named from the
    // path given and the object's name: "plate.stl" and "Bracket" make
    // "plate - Bracket.stl". A multi-part print is loaded a part at a time.
    bool separateFiles = false;
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
    std::vector<std::string> files;   // every file written, in order
    std::string error;
};

// Writes the scene to `path`. Objects are emitted in world space, so an
// object's transform is baked in -- a slicer has no concept of one.
StlResult exportStl(const Scene& scene, const std::string& path,
                    const StlOptions& options = {});

// The triangles `obj` is exported as, in its own space: an exact body
// tessellated afresh to `deviationMm` into `scratch`, or -- for a mesh, or a
// tolerance of zero -- the triangles already drawn. Shared by every mesh format
// so they all go out at the same resolution.
const RenderMesh& exportTriangles(const SceneObject& obj, Real deviationMm, RenderMesh& scratch);

// The objects an export takes: the selected ones, or every visible one, and
// never an empty body.
std::vector<const SceneObject*> exportedObjects(const Scene& scene, bool selectionOnly);

// Whether a transform reflects, so that triangles have to be wound the other
// way for the surface to keep facing outward.
bool mirrors(const Mat4& m);

// The file one object goes to when each has its own: `path` with the object's
// name worked into it, never one already in `taken`, which it is added to.
std::string exportFileFor(const std::string& path, const std::string& objectName,
                          std::vector<std::string>& taken);

} // namespace tg
