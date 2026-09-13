// Tangent - reading triangles from a file.
//
// The counterpart to export_stl.h, and the reason the mesh side of this program
// still exists at all. Modelling happens on exact bodies; meshes are what
// arrives from outside -- a part someone sent you as an STL, a scan, a model
// downloaded to measure against or build around.
//
// Two steps, kept separate on purpose:
//
//   readMesh   turns a file into triangles. Always works, however large or
//              however broken the file, because a triangle is a triangle.
//
//   toSolid    turns those triangles into an exact body. Sometimes works, and
//              says clearly when it does not -- a scan with holes in it is not
//              a solid and no amount of asking will make it one.
//
// Keeping them apart means a file you cannot convert is still a file you can
// open, look at, measure and export.
#pragma once

#include "geom/body.h"

#include <string>

namespace tg {

enum class MeshFormat { Unknown, Stl, Obj };

// From the extension. The file's contents are not sniffed: a .stl that is
// really an OBJ is a mislabelled file, and guessing around that hides the
// mistake rather than fixing it.
MeshFormat meshFormatOf(const std::string& path);

struct MeshImport {
    bool ok = false;
    std::string error;
    size_t triangles = 0;

    // True if the triangles form a closed surface. Only a closed one can
    // become a solid, and knowing before trying is what lets the caller say
    // something useful instead of just failing.
    bool closed = false;
};

// Reads `path` into `out` as a mesh body.
MeshImport readMesh(const std::string& path, Body& out);

struct SolidifyResult {
    bool ok = false;
    std::string error;

    int facesBefore = 0;    // triangles that went in
    int facesAfter = 0;     // faces that came out, after coplanar ones merged
};

// Turns a mesh body into an exact one.
//
// Every triangle becomes a planar face, the faces are sewn into a shell, and
// the shell is closed into a solid. Then the coplanar faces are merged back
// together, which is the step that makes this worth doing at all: a box
// exported as twelve triangles comes back as six faces, and a part that was
// CAD before someone turned it into an STL gets its flat surfaces returned.
//
// What does not come back is curvature. A cylinder that left as sixty-four
// facets returns as sixty-four narrow planar faces, because the information
// that they approximate a circle was thrown away by whatever wrote the file.
// This is recovery, not reconstruction, and the difference is worth being
// plain about.
//
// `maxFaces` refuses outright above a triangle count where the result would be
// unusable rather than grinding for minutes to produce it.
SolidifyResult toSolid(Body& body, ElementId salt, int maxFaces = 60000);

} // namespace tg
