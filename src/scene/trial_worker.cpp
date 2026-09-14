// tangent_trial: where kernel work is tried when there is no fork to try it in.
//
// Started by the application, beside it, the first time guarded work needs a
// process of its own. It reads work from its input and answers on its output
// until the application closes the pipe -- or until the work takes it down,
// which is the point of it being here rather than there.
#include "scene/trials.h"

int main() {
    return tg::serveTrials(&tg::runEncodedTrial);
}
