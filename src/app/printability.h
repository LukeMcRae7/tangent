// Tangent - what a printer will struggle with.
//
// The panel this replaces asked whether the body was a watertight solid with no
// degenerate faces. On an exact body that is true by construction, and an edit
// that would break it is refused before it lands, so the answer was always yes
// -- "Solid: ready to print" over a part with 0.2mm walls and eighty-degree
// overhangs. It was checking the model against itself rather than against a
// machine.
//
// These are the things that actually go wrong, in the order they cost you:
//
//   A wall thinner than the nozzle       does not get printed at all. The
//                                        slicer drops it, silently, and the
//                                        part comes off the bed with a hole.
//   A face leaning further than the      needs support, or it droops. Whether
//   printer can bridge                   that is acceptable is the user's
//                                        call; whether it is happening is not
//                                        a matter of opinion.
//   A feature smaller than the nozzle    comes out as a blob or not at all.
//
// None of these is a refusal. A part may be deliberately thin, and supports
// exist. So this reports rather than judges, and the report is drawn on the
// model, where the eye is, and not as a line of text in a panel.
#pragma once

#include "geom/body.h"

#include <vector>

namespace tg {

// The machine the part is being checked against. Defaults are a 0.4mm nozzle
// at a 0.2mm layer, which is what most printers in the world are set to.
struct PrintProfile {
    Real nozzleMm = 0.4;

    // Two perimeters, which is the thinnest wall worth printing: one perimeter
    // is fragile and slicers often drop it.
    Real minWallMm = 0.8;

    // Which way is up. The bed is the XY plane here, as it is in the viewport.
    Vec3 up{0, 0, 1};
};

enum class PrintIssue {
    None,
    ThinWall,     // less material behind the face than the nozzle can lay down
    NotSolid,     // the slicer will not know what is inside
};

// How many offending faces are kept for drawing. Every one is counted; only
// this many are remembered. Ten thousand red faces on an imported model is a
// colour rather than information, and holding them all costs memory to say
// nothing that the count does not say better.
inline constexpr size_t kMaxDrawnFindings = 400;

// One face, and what is wrong with it.
struct PrintFinding {
    FaceId      face = kInvalid;
    PrintIssue  issue = PrintIssue::None;
    Real        value = 0.0;      // millimetres of wall, or degrees of lean
};

struct PrintReport {
    std::vector<PrintFinding> findings;

    // Every offending face, however many were kept to draw.
    int  thinWalls = 0;
    bool solid = true;

    // The worst of them, for saying something short about the whole part.
    Real thinnestWallMm = 0.0;

    bool clean() const { return thinWalls == 0 && solid; }
};

// Looks over a body and says what a printer would make of it.
//
// Two things, and deliberately not three. A wall too thin to lay down, and a
// body a slicer cannot tell the inside of.
//
// Overhangs used to be here and are not. The slicer decides about supports, it
// decides better because it knows the machine, and it decides whether you ask
// or not -- so a second opinion here bought nothing but a viewport full of
// amber. Wall thickness is the opposite case: nothing warns you, the slicer
// quietly drops the wall, and the part comes off the bed with a hole in it.
//
// Costs a ray cast per face against a tree built over the tessellation. Not
// free, and not meant for every frame: the caller runs it when the geometry
// changes and keeps the answer.
PrintReport checkPrintability(const Body& body, const RenderMesh& render,
                              const PrintProfile& profile = {});

// A short sentence for the status bar.
std::string summarise(const PrintReport& report);

} // namespace tg
