// Tangent - Stage 0 spike: what each backend has to report about a part.
#pragma once

#include "part_spec.h"

#include <chrono>
#include <string>

namespace spike {

inline double nowMs() {
    using namespace std::chrono;
    return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

struct Result {
    // Construction: every boolean needed to reach the unfilleted part.
    bool   built = false;
    double buildMs = 0.0;
    std::string buildNote;

    // The fillet stage, kept separate because it is the operation that decides
    // the gate. `filletNote` carries the refusal when there is one; a backend
    // that cannot say why it refused is a backend the interface cannot explain.
    bool   filletOk = false;
    double filletMs = 0.0;
    std::string filletNote;
    int    filletEdges = 0;

    // The part as it stands after both stages.
    int    faces = 0;
    // Health of whatever came out, whether or not the fillet stage ran. Kept
    // apart from `filletOk` on purpose: "the boolean was fine and the fillet
    // refused" and "the boolean produced rubbish" are different findings.
    bool   valid = false;
    std::string validNote;
    double volumeMm3 = 0.0;

    // Display cost, at a chord deviation tied to the part's size.
    double tessMs = 0.0;
    int    triangles = 0;
    double deviationMm = 0.0;
};

Result runOcct(const PartSpec& spec);
Result runMesh(const PartSpec& spec, int segments);

} // namespace spike
