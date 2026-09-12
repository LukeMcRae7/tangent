#include "geom/kernel_guard.h"

#if defined(__unix__) || defined(__APPLE__)
#  define TG_HAVE_FORK 1
#  include <csignal>
#  include <sys/wait.h>
#  include <unistd.h>
#else
#  define TG_HAVE_FORK 0
#endif

#include <cerrno>

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

AsyncTrial::~AsyncTrial() { abandon(); }

void AsyncTrial::start(const std::function<bool()>& work) {
    abandon();
    done_ = false;
    result_ = Attempt::Refused;

#if TG_HAVE_FORK
    const pid_t pid = fork();
    if (pid == 0) {
        int code = 2;
        try {
            code = work() ? 0 : 1;
        } catch (...) {
            code = 1;
        }
        _exit(code);
    }
    if (pid > 0) {
        pid_ = static_cast<int>(pid);
        return;
    }
#endif
    // No fork here, or none to be had: answer now rather than not at all.
    result_ = tryInChild(work);
    done_ = true;
}

bool AsyncTrial::poll() {
    if (done_) return true;
#if TG_HAVE_FORK
    if (pid_ < 0) return false;
    int status = 0;
    const pid_t got = waitpid(static_cast<pid_t>(pid_), &status, WNOHANG);
    if (got == 0) return false;              // still working
    pid_ = -1;
    done_ = true;
    if (got < 0) { result_ = Attempt::Crashed; return true; }
    if (WIFSIGNALED(status)) { result_ = Attempt::Crashed; return true; }
    if (WIFEXITED(status)) {
        switch (WEXITSTATUS(status)) {
            case 0:  result_ = Attempt::Ok; break;
            case 1:  result_ = Attempt::Refused; break;
            default: result_ = Attempt::Crashed; break;
        }
        return true;
    }
    result_ = Attempt::Crashed;
#endif
    return done_;
}

void AsyncTrial::abandon() {
#if TG_HAVE_FORK
    if (pid_ >= 0) {
        ::kill(static_cast<pid_t>(pid_), SIGKILL);
        int status = 0;
        // Only a signal is worth retrying for. Looping on any error would spin
        // forever against a child that is already gone.
        while (waitpid(static_cast<pid_t>(pid_), &status, 0) < 0 && errno == EINTR) {}
        pid_ = -1;
    }
#endif
    done_ = false;
}

} // namespace tg
