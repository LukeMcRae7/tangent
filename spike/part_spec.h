// Tangent - Stage 0 spike: the part catalogue, written once for both kernels.
//
// The point of the spike is a comparison, so the parts have to be the same
// part. Describing them declaratively and letting each backend realise the
// description is the only way to be sure of that: if the OCCT bracket were
// written by hand next to a mesh bracket written by hand, every difference in
// the results would be arguable.
//
// Everything is in millimetres, +Z up, base sitting on z = 0, matching the
// conventions in src/mesh/primitives.h.
#pragma once

#include <string>
#include <vector>

namespace spike {

struct Boss {    // cylinder standing on the plate, fused
    double x, y, dia, height;
};

struct Bore {    // cylinder through everything, cut
    double x, y, dia;
};

struct Pocket {  // rectangular slot cut down from the top face
    double x, y, w, d, depth;
};

// Which edges the fillet stage is aimed at. Both backends resolve these
// geometrically -- by position, not by index -- so neither gets to rely on the
// order its own boolean happened to produce.
enum class FilletTarget {
    None,
    BoreRims,      // the top rim of every bore: the case that fails today
    TopOuterEdges, // the four edges around the top face of the plate
    AllTopEdges,   // both of the above at once
};

struct PartSpec {
    std::string name;
    std::string what;         // one line, for the report

    double plateW = 100.0;    // X
    double plateD = 100.0;    // Y
    double plateH = 10.0;     // Z, from z = 0 up

    // Upstand along the -Y edge, making an L-section bracket. Zero for a plate.
    double wallH = 0.0;
    double wallT = 8.0;

    std::vector<Boss>   bosses;
    std::vector<Bore>   bores;
    std::vector<Pocket> pockets;

    FilletTarget target = FilletTarget::None;
    double filletRadius = 0.0;
};

// ---- The catalogue --------------------------------------------------------
// Three parts the plan names, plus the two that the mesh kernel found hardest
// during the review, so the comparison covers what is known to be difficult
// rather than only what is known to be easy.
std::vector<PartSpec> catalogue();

} // namespace spike
