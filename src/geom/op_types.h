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

} // namespace tg
