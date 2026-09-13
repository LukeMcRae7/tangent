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

    // Whether the fast route built it -- regions traced from the mesh -- or it
    // fell back to a face per triangle, sewn and merged by the kernel. Both
    // give the same solid; one takes a fraction of a second and the other
    // can take many, so which ran is worth being able to see.
    bool viaRegions = false;
};

// How many faces converting `body` would produce, without converting it.
//
// Every triangle lying on the same plane becomes one face, so counting the
// distinct planes counts the answer. It is a linear pass over the triangles
// and it is what makes the refusal below instant rather than something you
// find out after a minute of work.
//
// The two cases it separates are the whole point. A part that was CAD before
// somebody exported it has a few dozen planes and thousands of triangles, and
// converting it gives back the model. A scanned or sculpted mesh has nearly as
// many planes as triangles, and converting it gives a B-rep with sixty thousand
// faces that is slower than the mesh in every way and no more editable.
int predictSolidFaces(const Body& body);

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
// The most faces a conversion is allowed to produce. Not a limit on triangles:
// a million that are really a bracket should convert, and sixty thousand that
// are really a sculpture should not. Five thousand faces is already a large
// B-rep to be working with.
inline constexpr int kSolidifyFaceLimit = 5000;

SolidifyResult toSolid(Body& body, ElementId salt, int maxFaces = kSolidifyFaceLimit);

} // namespace tg
