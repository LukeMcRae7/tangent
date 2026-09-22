// Tangent - the kernel work the application tries before it does it.
//
// Each of these is guarded work (geom/kernel_guard.h): the work itself, and the
// same work written down, so that it can be tried in a forked child where there
// is fork and in the tangent_trial worker where there is not. Everything the
// work needs goes into the bytes -- a worker shares nothing with the
// application but the build it came from.
#pragma once

#include "geom/kernel_guard.h"
#include "geom/operations.h"
#include "scene/feature.h"

#include <string>
#include <vector>

namespace tg {

enum class TrialKind : uint32_t {
    SelfTest = 1,   // for the tests: behaves as asked
    Fillet   = 2,   // one fillet on one body
    Chain    = 3,   // a whole feature chain, evaluated
};

// What a self-test trial does, so a test can see every answer come back
// through the same path real work takes. Chatter prints to standard output
// before it builds, as the kernel sometimes does, to show that nothing printed
// can be mistaken for an answer.
enum class SelfTest : uint32_t { Build, Refuse, Throw, Crash, Chatter };

GuardedWork selfTestTrial(SelfTest behaviour);

// Rounds `edges` of `body` as `spec` says. The body goes across as the kernel's
// own text, and the edges by the handles `spec` holds, which a decoded body
// numbers the same way as the one it was written from.
GuardedWork filletTrial(Body body, FilletSpec spec);

// Evaluates `features` from the start, as a history edit would.
GuardedWork chainTrial(std::vector<Feature> features);

// The worker's side: rebuilds the work from `payload` and runs it. False for a
// kind it does not know or bytes it cannot read, which the application sees as
// a refusal.
bool runEncodedTrial(uint32_t kind, const std::string& payload);

} // namespace tg
