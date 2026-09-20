// Tangent - the sizes a hole is actually chosen from.
//
// Nobody drilling a hole for an M4 cap screw wants to type 4.5 and 8.0 and
// 4.4: they want "M4, clearance, cap head". The numbers are here, from the
// standards a screw is made to -- ISO 273 for clearance, ISO 4762 for the cap
// head it takes, ISO 7046 for the countersunk one -- and the panel turns a
// choice into a HoleCut.
//
// What is different here from a machinist's table is the allowance. A printed
// hole comes out smaller than it was drawn: the nozzle's path is a circle of
// finite width and the inside corner of it is where the plastic goes. So a
// printed clearance hole is cut oversize by `printedAllowance()`, and a tapped
// one -- a hole a screw cuts its own thread in, which is how a printed part
// takes a machine screw -- is not, because the thread needs the material.
//
// The allowance is a default, not a measurement of the machine in front of
// you. It is one number in one place so that a calibration print can replace
// it, which is where phase 5 takes it.
#pragma once

#include "core/math.h"
#include "geom/op_types.h"

namespace tg {

// How freely the screw passes through.
enum class HoleFit : uint32_t {
    Close = 0,   // ISO 273 fine: located by the hole
    Normal = 1,  // ISO 273 medium: the everyday one
    Loose = 2,   // ISO 273 coarse: room to move
    Tapped = 3,  // no clearance at all: the screw cuts its own thread
};

const char* holeFitName(HoleFit fit);

// One metric screw, and the holes a part takes it in.
struct Fastener {
    const char* name;      // "M4"
    Real nominal;          // 4.0: the thread's outside diameter
    Real close, normal, loose;   // ISO 273 clearance holes
    Real tap;              // the hole a thread is cut in (ISO 2306 tapping drill)
    Real capHead;          // ISO 4762 socket head: diameter, then height
    Real capHeight;
    Real sinkHead;         // ISO 7046 countersunk head: diameter at the surface
};

// M2 to M10, which is the range a printed part uses. Ordered by size.
int fastenerCount();
const Fastener& fastenerAt(int i);
// The index of `name`, or -1. Used when a saved hole names its size.
int fastenerNamed(const char* name);

// How much wider a printed hole is cut than the screw needs, in millimetres of
// diameter. Nothing is added to a tapped hole: the thread is cut in material
// that has to be there.
Real printedAllowance(HoleFit fit);

// The hole for `i` at `fit`, as a printed part wants it: the clearance from
// the table plus the allowance.
Real holeDiameter(int i, HoleFit fit);

// A whole hole for that screw and that head, `depth` deep (or through), ready
// for the kernel. `head` says what the mouth looks like; the head's own
// numbers come from the table.
HoleCut holeFor(int i, HoleFit fit, HoleKind head, Real depth, bool through);

} // namespace tg
