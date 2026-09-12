#include "geom/kernel_guard.h"

#if defined(__unix__) || defined(__APPLE__)
#  define TG_HAVE_FORK 1
#  include <sys/wait.h>
#  include <unistd.h>
#else
#  define TG_HAVE_FORK 0
#endif

namespace tg {

bool childIsolationAvailable() { return TG_HAVE_FORK != 0; }

Attempt tryInChild(const std::function<bool()>& work) {
#if TG_HAVE_FORK
    const pid_t pid = fork();
    if (pid == 0) {
        // The child. Anything it touches is thrown away with it, so the only
        // thing that has to be right is the exit code.
        int code = 2;
        try {
            code = work() ? 0 : 1;
        } catch (...) {
            code = 1;
        }
        // _exit, not exit: the parent's atexit handlers, its GL context and its
        // open files are shared until now and must not be run down twice.
        _exit(code);
    }
    if (pid < 0) {
        // Out of processes. Better to answer from the call itself than to
        // refuse work because the machine is busy.
        return work() ? Attempt::Ok : Attempt::Refused;
    }

    int status = 0;
    while (waitpid(pid, &status, 0) < 0) { /* interrupted; wait again */ }
    if (WIFSIGNALED(status)) return Attempt::Crashed;
    if (WIFEXITED(status)) {
        switch (WEXITSTATUS(status)) {
            case 0:  return Attempt::Ok;
            case 1:  return Attempt::Refused;
            default: return Attempt::Crashed;
        }
    }
    return Attempt::Crashed;
#else
    // No isolation here: if this faults, it takes the program. Callers use
    // childIsolationAvailable() to decide how far to push.
    return work() ? Attempt::Ok : Attempt::Refused;
#endif
}

} // namespace tg
