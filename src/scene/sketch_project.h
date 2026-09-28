// Tangent - bringing a body's edges into a sketch.
//
// An edge projected onto a sketch's plane becomes sketch geometry: a straight
// edge a line, a circle or an arc square to the plane a circle or an arc, and
// anything else the lines of its shape. Its points are held by Fix rules --
// it is a reference, not something to drag -- and ends that meet share one
// point, so projecting a face's whole boundary gives a region that closes.
//
// A line, a circle or an arc projected from the body the sketch is in keeps
// the edge's name, and follows it: the history moves it to wherever the edge
// has gone before the sketch is solved, so a sketch drawn round a hole in a
// face still goes round it after the hole is moved.
#pragma once

#include "core/math.h"
#include "geom/body.h"
#include "sketch/sketch.h"

#include <string>
#include <vector>

namespace tg {

// Adds the projection of `edge` of `body` to `sketch`. `toSketch` takes the
// body's coordinates into the frame the sketch's plane is in. `link` records
// the edge's name on what it makes, so refreshProjections can move it. The
// entities added are appended to `added`. False, with the reason, when the
// edge stands straight out of the plane and has nothing to leave on it.
bool projectEdge(Sketch& sketch, const Body& body, const Mat4& toSketch, EdgeId edge, bool link,
                 std::vector<SketchId>* added, std::string* why);

// Every edge around `face`, the same way.
bool projectFace(Sketch& sketch, const Body& body, const Mat4& toSketch, FaceId face, bool link,
                 std::vector<SketchId>* added, std::string* why);

// Moves every linked entity of `sketch` to where its edge of `body` now is,
// re-stating the Fix rules and radii that hold it. False, with the reason,
// when an edge it follows is gone or has become another kind of curve.
bool refreshProjections(Sketch& sketch, const Body& body, std::string* why);

} // namespace tg
