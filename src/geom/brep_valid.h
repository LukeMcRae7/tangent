// Tangent - whether a B-rep is valid, answered without the quadratic part.
//
// OpenCASCADE's BRepCheck_Analyzer is the arbiter of whether a kernel result
// may be handed back, and on every ordinary part it is fast. It has one
// quadratic step: within a face, every wire is checked against every other,
// edge by edge. A face drilled with 64 holes of a hundred segments each -- what
// a converted STL looks like -- is 2,080 wire pairs of a hundred edges apiece,
// and that took twelve seconds after every edit.
//
// Most of those pairs cannot touch: holes on a grid have bounding boxes that do
// not overlap. So a heavy face is checked in pieces, with OCCT's own face check
// doing every piece -- each hole against the outer boundary, and hole against
// hole only where their boxes overlap -- and the checks that the whole-shape
// pass would have made across faces are made directly: every edge used once in
// each direction.
//
// Shapes with no heavy face take the full analyzer exactly as before, so every
// result that was accepted or refused before still is.
#pragma once

#ifdef TG_HAVE_OCCT
#include <TopoDS_Shape.hxx>

#include <cstddef>

namespace tg::brep {

// A face with more edges than this, spread over more than one wire, is checked
// in pieces. Ordinary modelled faces sit far below it; a converted mesh's
// drilled faces sit far above.
inline constexpr size_t kPrunedCheckEdges = 256;

bool shapeIsValid(const TopoDS_Shape& shape, size_t prunedAbove = kPrunedCheckEdges);

// The whole-shape analyzer, for tests to hold the pruned check against.
bool fullAnalyzerValid(const TopoDS_Shape& shape);

} // namespace tg::brep
#endif
