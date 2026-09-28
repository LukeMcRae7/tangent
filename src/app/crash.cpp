#include "app/crash.h"

#include "core/crashlog.h"

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#else
#include <execinfo.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace tg::crash {

namespace {

// Everything the handler needs, worked out beforehand: after the fault there
// is no allocating, no formatting through the heap.
char g_reportPath[1024] = {};
char g_pendingPath[1024] = {};
char g_header[512] = {};
Saved (*g_save)(void*) = nullptr;
void* g_saveContext = nullptr;
volatile std::sig_atomic_t g_handling = 0;
#if !defined(_WIN32)
pid_t g_mainPid = 0;
#endif

int openReport() {
#if defined(_WIN32)
    return _open(g_reportPath, _O_WRONLY | _O_CREAT | _O_TRUNC, 0644);
#else
    return ::open(g_reportPath, O_WRONLY | O_CREAT | O_TRUNC, 0644);
#endif
}

void put(int fd, const char* s) {
#if defined(_WIN32)
    (void)_write(fd, s, static_cast<unsigned>(std::strlen(s)));
#else
    (void)!::write(fd, s, std::strlen(s));
#endif
}

void closeFd(int fd) {
#if defined(_WIN32)
    _close(fd);
#else
    ::close(fd);
#endif
}

const char* signalName(int sig) {
    switch (sig) {
        case SIGSEGV: return "SIGSEGV (a bad memory access)";
        case SIGABRT: return "SIGABRT (aborted)";
        case SIGFPE:  return "SIGFPE (an arithmetic fault)";
        case SIGILL:  return "SIGILL (an illegal instruction)";
#if !defined(_WIN32)
        case SIGBUS:  return "SIGBUS (a bad memory access)";
#endif
        default:      return "a fatal signal";
    }
}

// The report, the save, and a note for the next start that there was one.
void report(const char* what) {
    const int fd = openReport();
    if (fd < 0) return;
    put(fd, g_header);
    put(fd, "What: ");
    put(fd, what);
    put(fd, "\n\nThe last things done, oldest first:\n");
    crashlog::writeTo(fd);
    put(fd, "\nWhere:\n");
#if defined(_WIN32)
    void* frames[64];
    const USHORT n = CaptureStackBackTrace(0, 64, frames, nullptr);
    char line[64];
    for (USHORT i = 0; i < n; ++i) {
        std::snprintf(line, sizeof line, "  %p\n", frames[i]);
        put(fd, line);
    }
#else
    void* frames[64];
    const int n = backtrace(frames, 64);
    backtrace_symbols_fd(frames, n, fd);
#endif

    // The work, saved aside. In a copy of the process, which may hang on a
    // lock the fault left held: it gets five seconds.
    if (g_save) {
        Saved saved = Saved::Failed;
#if defined(_WIN32)
        saved = g_save(g_saveContext);
#else
        const pid_t child = fork();
        if (child == 0) {
            alarm(5);
            _exit(static_cast<int>(g_save(g_saveContext)));
        }
        if (child > 0) {
            for (int i = 0; i < 60; ++i) {
                int status = 0;
                const pid_t got = waitpid(child, &status, WNOHANG);
                if (got == child) {
                    if (WIFEXITED(status) && WEXITSTATUS(status) <= 2) saved = static_cast<Saved>(WEXITSTATUS(status));
                    break;
                }
                const timespec wait{0, 100 * 1000 * 1000};
                nanosleep(&wait, nullptr);
                if (i == 59) kill(child, SIGKILL);
            }
        }
#endif
        put(fd, saved == Saved::Done    ? "\nThe work was saved aside, to be offered back.\n"
              : saved == Saved::Nothing ? "\nThere was nothing unsaved.\n"
                                        : "\nThe work could not be saved aside.\n");
    }
    closeFd(fd);

    // Which report is waiting to be mentioned.
#if defined(_WIN32)
    const int p = _open(g_pendingPath, _O_WRONLY | _O_CREAT | _O_TRUNC, 0644);
#else
    const int p = ::open(g_pendingPath, O_WRONLY | O_CREAT | O_TRUNC, 0644);
#endif
    if (p >= 0) {
        put(p, g_reportPath);
        closeFd(p);
    }
}

void onSignal(int sig) {
#if !defined(_WIN32)
    // A kernel trial's crash is its parent's to read, not a crash of Tangent.
    if (getpid() != g_mainPid) {
        std::signal(sig, SIG_DFL);
        std::raise(sig);
        return;
    }
#endif
    if (!g_handling) {
        g_handling = 1;
        report(signalName(sig));
    }
    std::signal(sig, SIG_DFL);
    std::raise(sig);
}

void onTerminate() {
    // An exception nobody caught: say which, then go the way abort goes.
    if (std::exception_ptr e = std::current_exception()) {
        try {
            std::rethrow_exception(e);
        } catch (const std::exception& ex) {
            crashlog::note("uncaught exception: %s", ex.what());
        } catch (...) {
            crashlog::note("uncaught exception of an unknown kind");
        }
    }
    std::abort();
}

} // namespace

void install(const std::string& dir, const std::string& build, Saved (*save)(void*), void* context) {
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    const std::time_t now = std::time(nullptr);
    char stamp[32];
    std::strftime(stamp, sizeof stamp, "%Y%m%d-%H%M%S", std::localtime(&now));
    std::snprintf(g_reportPath, sizeof g_reportPath, "%s/crash-%s.txt", dir.c_str(), stamp);
    std::snprintf(g_pendingPath, sizeof g_pendingPath, "%s/pending", dir.c_str());
    char started[64];
    std::strftime(started, sizeof started, "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    std::snprintf(g_header, sizeof g_header, "Tangent crashed.\nBuild: %s\nSession started: %s\n", build.c_str(),
                  started);
    g_save = save;
    g_saveContext = context;
#if !defined(_WIN32)
    g_mainPid = getpid();
#endif
    for (int sig : {SIGSEGV, SIGABRT, SIGFPE, SIGILL
#if !defined(_WIN32)
                    , SIGBUS
#endif
         })
        std::signal(sig, onSignal);
    std::set_terminate(onTerminate);
}

std::string pendingReport(const std::string& dir) {
    std::ifstream in(dir + "/pending");
    std::string path;
    if (!in || !std::getline(in, path)) return {};
    std::error_code ec;
    return std::filesystem::exists(path, ec) ? path : std::string();
}

void acknowledge(const std::string& dir) {
    std::error_code ec;
    std::filesystem::remove(dir + "/pending", ec);
}

void crashNow() {
    crashlog::note("crashing on purpose, for the test");
    volatile int* nothing = nullptr;
    *nothing = 1;
    std::abort();
}

} // namespace tg::crash
