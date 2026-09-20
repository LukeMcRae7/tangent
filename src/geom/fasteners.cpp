#include "geom/fasteners.h"

#include <cstring>

namespace tg {

const char* holeFitName(HoleFit fit) {
    switch (fit) {
        case HoleFit::Close:  return "Close";
        case HoleFit::Normal: return "Normal";
        case HoleFit::Loose:  return "Loose";
        case HoleFit::Tapped: return "Tapped";
    }
    return "Normal";
}

namespace {

// name  nominal  close  normal  loose   tap   capHead capHeight sinkHead
const Fastener kFasteners[] = {
    {"M2",   2.0,  2.2,  2.4,  2.6,  1.6,  3.8,  2.0,  4.4},
    {"M2.5", 2.5,  2.7,  2.9,  3.1,  2.05, 4.5,  2.5,  5.5},
    {"M3",   3.0,  3.2,  3.4,  3.6,  2.5,  5.5,  3.0,  6.3},
    {"M4",   4.0,  4.3,  4.5,  4.8,  3.3,  7.0,  4.0,  9.4},
    {"M5",   5.0,  5.3,  5.5,  5.8,  4.2,  8.5,  5.0, 10.4},
    {"M6",   6.0,  6.4,  6.6,  7.0,  5.0, 10.0,  6.0, 12.6},
    {"M8",   8.0,  8.4,  9.0, 10.0,  6.8, 13.0,  8.0, 17.3},
    {"M10", 10.0, 10.5, 11.0, 12.0,  8.5, 16.0, 10.0, 20.0},
};

} // namespace

int fastenerCount() { return static_cast<int>(sizeof(kFasteners) / sizeof(kFasteners[0])); }

const Fastener& fastenerAt(int i) {
    if (i < 0) i = 0;
    if (i >= fastenerCount()) i = fastenerCount() - 1;
    return kFasteners[i];
}

int fastenerNamed(const char* name) {
    if (!name) return -1;
    for (int i = 0; i < fastenerCount(); ++i)
        if (std::strcmp(kFasteners[i].name, name) == 0) return i;
    return -1;
}

Real printedAllowance(HoleFit fit) {
    // Two tenths on the diameter, which is about what an 0.4 nozzle at 0.2
    // layers takes off a vertical hole of this size. A tapped hole gets none:
    // the thread is cut in material that has to be there, and an oversize one
    // strips.
    return fit == HoleFit::Tapped ? Real(0) : Real(0.2);
}

Real holeDiameter(int i, HoleFit fit) {
    const Fastener& f = fastenerAt(i);
    const Real base = fit == HoleFit::Close  ? f.close
                    : fit == HoleFit::Loose  ? f.loose
                    : fit == HoleFit::Tapped ? f.tap
                                             : f.normal;
    return base + printedAllowance(fit);
}

HoleCut holeFor(int i, HoleFit fit, HoleKind head, Real depth, bool through) {
    const Fastener& f = fastenerAt(i);
    HoleCut cut;
    cut.kind = head;
    cut.diameter = holeDiameter(i, fit);
    cut.depth = depth;
    cut.through = through;
    // The head sits in a pocket the width of the head plus the same allowance,
    // and one deep enough for the head to end up level with the face.
    cut.headDiameter = f.capHead + printedAllowance(fit);
    cut.headDepth = f.capHeight + Real(0.2);
    if (head == HoleKind::Countersink) {
        cut.headDiameter = f.sinkHead + printedAllowance(fit);
        cut.sinkAngle = Real(90.0) * kDeg2Rad;
    }
    return cut;
}

} // namespace tg
