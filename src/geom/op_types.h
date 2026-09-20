// Tangent - what an operation is asked to do.
//
// The few words the kernel interface (brep.h) and the operations over bodies
// (operations.h) both need, kept apart from either so that neither has to
// include the other to say "a union".
#pragma once

#include <cstdint>

namespace tg {

enum class BooleanOp { Union, Difference, Intersection };

const char* booleanOpName(BooleanOp op);

// How a pushed face meets the body it came from. The exact kernel acts on Join
// and Cut; the other two are treated as Auto until something offers them.
enum class ExtrudeOp : uint32_t {
    Auto = 0,      // decided by the sign: pushing out joins, pushing in cuts
    Join = 1,      // always combined with the body
    Cut = 2,       // always taken away from the body
    Intersect = 3, // what the sweep and the body share
    NewBody = 4    // a separate body, leaving this one alone
};

const char* extrudeOpName(ExtrudeOp op);

// What a hole looks like where it meets the face it is drilled into.
enum class HoleKind : uint32_t {
    Simple = 0,       // one diameter all the way
    Counterbore = 1,  // a flat-bottomed pocket for a cap head
    Countersink = 2,  // a cone for a flat head
};

const char* holeKindName(HoleKind k);

// A hole, as the kernel is asked for it: every number in millimetres, already
// decided. Which fastener it was chosen for, and what a printed one has to
// allow for, are the panel's business -- see geom/fasteners.h -- because the
// body only ever has a diameter cut out of it.
struct HoleCut {
    HoleKind kind = HoleKind::Simple;
    double diameter = 3.4;
    double depth = 10.0;        // from the face, when it does not go through
    bool   through = true;      // all the way out the other side
    // A drill leaves a cone at the bottom of a blind hole, and a printed part
    // wants one too: a flat ceiling over a hole is the one overhang a slicer
    // cannot help with. The angle is the drill's, included.
    bool   drillPoint = true;
    double pointAngle = 118.0 * 3.14159265358979323846 / 180.0;
    // Counterbore: how wide and how deep. Countersink: how wide at the face,
    // and the cone's included angle.
    double headDiameter = 6.0;
    double headDepth = 3.3;
    double sinkAngle = 90.0 * 3.14159265358979323846 / 180.0;
};

} // namespace tg
