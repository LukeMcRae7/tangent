#include "geom/kernel_guard.h"

#if defined(_WIN32)
#  define TG_WINDOWS 1
#  define TG_HAVE_FORK 0
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
#  include <fcntl.h>
#  include <io.h>
#else
#  define TG_WINDOWS 0
#  define TG_HAVE_FORK 1
#  include <csignal>
#  include <fcntl.h>
#  include <poll.h>
#  include <spawn.h>
#  include <sys/wait.h>
#  include <unistd.h>
extern char** environ;
#endif

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

namespace tg {

namespace {

// Written once, in a forked child, before it runs anything; read from then on
// by that single thread. No lock is needed and none would be safe to take.
bool gIsolatedChild = false;

// ---- the wire -------------------------------------------------------------
//
// A request is a header and the work's bytes; an answer is a header and a
// verdict. Both are little-endian, and both ends are the same build, so there
// is nothing to negotiate.
constexpr uint32_t kRequest = 0x51475454;       // "TTGQ"
constexpr uint32_t kAnswer  = 0x41475454;       // "TTGA"
constexpr uint64_t kMaxPayload = 1ull << 31;    // far past any body; a guard against garbage

void put32(unsigned char* at, uint32_t v) {
    for (int i = 0; i < 4; ++i) at[i] = static_cast<unsigned char>((v >> (8 * i)) & 0xFF);
}
void put64(unsigned char* at, uint64_t v) {
    for (int i = 0; i < 8; ++i) at[i] = static_cast<unsigned char>((v >> (8 * i)) & 0xFF);
}
uint32_t get32(const unsigned char* at) {
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i) v |= static_cast<uint32_t>(at[i]) << (8 * i);
    return v;
}
uint64_t get64(const unsigned char* at) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v |= static_cast<uint64_t>(at[i]) << (8 * i);
    return v;
}

// Where tangent_trial should be: beside whatever executable is running, which
// is the application or a test, both built into the same directory.
std::filesystem::path workerPath() {
#if TG_WINDOWS
    std::wstring buf(MAX_PATH, L'\0');
    for (;;) {
        const DWORD n = GetModuleFileNameW(nullptr, buf.data(), static_cast<DWORD>(buf.size()));
        if (n == 0) return {};
        if (n < buf.size()) { buf.resize(n); break; }
        buf.resize(buf.size() * 2);
    }
    return std::filesystem::path(buf).parent_path() / L"tangent_trial.exe";
#elif defined(__linux__)
    std::error_code ec;
    const std::filesystem::path self = std::filesystem::read_symlink("/proc/self/exe", ec);
    if (ec) return {};
    return self.parent_path() / "tangent_trial";
#else
    return {};                                   // fork is used here; no worker is looked for
#endif
}

Attempt runHere(const std::function<bool()>& work) {
    try {
        return work() ? Attempt::Ok : Attempt::Refused;
    } catch (...) {
        return Attempt::Refused;
    }
}

} // namespace

bool inIsolatedChild() { return gIsolatedChild; }

Isolation isolation() {
    static const Isolation decided = [] {
        const char* env = std::getenv("TANGENT_ISOLATION");
        const std::string want = env ? env : "";
        if (want == "none") return Isolation::None;
#if TG_HAVE_FORK
        if (want != "worker") return Isolation::Fork;
#endif
        std::error_code ec;
        const std::filesystem::path worker = workerPath();
        if (!worker.empty() && std::filesystem::exists(worker, ec)) return Isolation::Worker;
        // Said once, where someone running it can see: without the worker, a
        // kernel fault here ends the application.
        std::fprintf(stderr, "[isolation] no tangent_trial beside this executable; "
                             "guarded kernel work runs unguarded\n");
        return Isolation::None;
    }();
    return decided;
}

bool childIsolationAvailable() { return isolation() != Isolation::None; }

bool forkIsolationAvailable() { return TG_HAVE_FORK != 0; }

// ---- the worker process, from the application's side ----------------------

struct AsyncTrial::Worker {
    enum class Sent { Ok, CouldNotStart, Died };

#if TG_WINDOWS
    HANDLE process = nullptr;
    HANDLE toChild = nullptr;
    HANDLE fromChild = nullptr;

    bool alive() const {
        return process && WaitForSingleObject(process, 0) == WAIT_TIMEOUT;
    }

    bool spawn() {
        const std::filesystem::path exe = workerPath();
        if (exe.empty()) return false;

        SECURITY_ATTRIBUTES sa{};
        sa.nLength = sizeof sa;
        sa.bInheritHandle = TRUE;
        HANDLE inRead = nullptr, inWrite = nullptr, outRead = nullptr, outWrite = nullptr;
        if (!CreatePipe(&inRead, &inWrite, &sa, 1 << 20)) return false;
        if (!CreatePipe(&outRead, &outWrite, &sa, 1 << 16)) {
            CloseHandle(inRead);
            CloseHandle(inWrite);
            return false;
        }
        // Only the child's ends are inherited. Were ours inherited too, the
        // worker would hold its own input open and never see it close.
        SetHandleInformation(inWrite, HANDLE_FLAG_INHERIT, 0);
        SetHandleInformation(outRead, HANDLE_FLAG_INHERIT, 0);

        STARTUPINFOW si{};
        si.cb = sizeof si;
        si.dwFlags = STARTF_USESTDHANDLES;
        si.hStdInput = inRead;
        si.hStdOutput = outWrite;
        si.hStdError = GetStdHandle(STD_ERROR_HANDLE);
        PROCESS_INFORMATION pi{};
        std::wstring command = L"\"" + exe.wstring() + L"\"";
        const BOOL made = CreateProcessW(exe.c_str(), command.data(), nullptr, nullptr, TRUE,
                                         CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi);
        CloseHandle(inRead);
        CloseHandle(outWrite);
        if (!made) {
            CloseHandle(inWrite);
            CloseHandle(outRead);
            return false;
        }
        CloseHandle(pi.hThread);
        process = pi.hProcess;
        toChild = inWrite;
        fromChild = outRead;
        return true;
    }

    bool writeAll(const void* bytes, size_t n) {
        const auto* p = static_cast<const char*>(bytes);
        while (n > 0) {
            const DWORD chunk = static_cast<DWORD>(std::min<size_t>(n, 1u << 30));
            DWORD wrote = 0;
            if (!WriteFile(toChild, p, chunk, &wrote, nullptr) || wrote == 0) return false;
            p += wrote;
            n -= wrote;
        }
        return true;
    }

    // 1 with an answer, 0 still working, -1 gone.
    int readAnswer(Attempt& out, bool block) {
        if (!block) {
            DWORD available = 0;
            if (!PeekNamedPipe(fromChild, nullptr, 0, nullptr, &available, nullptr)) return -1;
            if (available < 8) return 0;
        }
        unsigned char answer[8];
        DWORD got = 0;
        size_t have = 0;
        while (have < sizeof answer) {
            if (!ReadFile(fromChild, answer + have, static_cast<DWORD>(sizeof answer - have), &got,
                          nullptr) || got == 0)
                return -1;
            have += got;
        }
        if (get32(answer) != kAnswer) return -1;
        out = get32(answer + 4) == 0 ? Attempt::Ok : Attempt::Refused;
        return 1;
    }

    void stop() {
        if (process) {
            TerminateProcess(process, 1);
            WaitForSingleObject(process, 5000);
            CloseHandle(process);
        }
        if (toChild) CloseHandle(toChild);
        if (fromChild) CloseHandle(fromChild);
        process = toChild = fromChild = nullptr;
    }
#else
    pid_t pid = -1;
    int toChild = -1;
    int fromChild = -1;

    bool alive() {
        if (pid <= 0) return false;
        int status = 0;
        const pid_t got = waitpid(pid, &status, WNOHANG);
        if (got == 0) return true;
        // Reaped just now, or not ours to reap: either way the number may be
        // handed to another process, and must not be signalled again.
        if (got == pid || (got < 0 && errno == ECHILD)) pid = -1;
        return false;
    }

    bool spawn() {
        const std::filesystem::path exe = workerPath();
        if (exe.empty()) return false;

        // A write to a worker that has just died raises SIGPIPE, whose default
        // is to end this process -- the very thing the worker is for. Ignored,
        // the write fails instead and is read as the crash it was.
        static std::once_flag ignorePipe;
        std::call_once(ignorePipe, [] { std::signal(SIGPIPE, SIG_IGN); });

        int in[2] = {-1, -1}, out[2] = {-1, -1};
        if (pipe(in) != 0) return false;
        if (pipe(out) != 0) {
            close(in[0]);
            close(in[1]);
            return false;
        }
        for (int fd : {in[0], in[1], out[0], out[1]}) fcntl(fd, F_SETFD, FD_CLOEXEC);

        posix_spawn_file_actions_t actions;
        posix_spawn_file_actions_init(&actions);
        posix_spawn_file_actions_adddup2(&actions, in[0], STDIN_FILENO);
        posix_spawn_file_actions_adddup2(&actions, out[1], STDOUT_FILENO);
        const std::string path = exe.string();
        char* argv[] = {const_cast<char*>(path.c_str()), nullptr};
        const int rc = posix_spawn(&pid, path.c_str(), &actions, nullptr, argv, environ);
        posix_spawn_file_actions_destroy(&actions);
        close(in[0]);
        close(out[1]);
        if (rc != 0) {
            close(in[1]);
            close(out[0]);
            pid = -1;
            return false;
        }
        toChild = in[1];
        fromChild = out[0];
        return true;
    }

    bool writeAll(const void* bytes, size_t n) {
        const auto* p = static_cast<const char*>(bytes);
        while (n > 0) {
            const ssize_t wrote = ::write(toChild, p, n);
            if (wrote < 0 && errno == EINTR) continue;
            if (wrote <= 0) return false;
            p += wrote;
            n -= static_cast<size_t>(wrote);
        }
        return true;
    }

    int readAnswer(Attempt& out, bool block) {
        if (!block) {
            pollfd p{fromChild, POLLIN, 0};
            int ready = 0;
            do { ready = ::poll(&p, 1, 0); } while (ready < 0 && errno == EINTR);
            if (ready < 0) return -1;
            if (ready == 0) return 0;
        }
        unsigned char answer[8];
        size_t have = 0;
        while (have < sizeof answer) {
            const ssize_t got = ::read(fromChild, answer + have, sizeof answer - have);
            if (got < 0 && errno == EINTR) continue;
            if (got <= 0) return -1;              // closed: the worker is gone
            have += static_cast<size_t>(got);
        }
        if (get32(answer) != kAnswer) return -1;
        out = get32(answer + 4) == 0 ? Attempt::Ok : Attempt::Refused;
        return 1;
    }

    void stop() {
        if (toChild >= 0) close(toChild);
        if (fromChild >= 0) close(fromChild);
        toChild = fromChild = -1;
        if (pid > 0) {
            ::kill(pid, SIGKILL);
            int status = 0;
            while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
        }
        pid = -1;
    }
#endif

    Sent send(uint32_t kind, const std::string& payload) {
        if (!alive()) {
            stop();                               // reap what is left of the last one
            if (!spawn()) return Sent::CouldNotStart;
        }
        unsigned char header[16];
        put32(header, kRequest);
        put32(header + 4, kind);
        put64(header + 8, payload.size());
        if (!writeAll(header, sizeof header) || !writeAll(payload.data(), payload.size()))
            return Sent::Died;
        return Sent::Ok;
    }

    ~Worker() { stop(); }
};

// ---- guarded work ---------------------------------------------------------

Attempt tryInChild(const std::function<bool()>& work) {
#if TG_HAVE_FORK
    const pid_t pid = fork();
    if (pid == 0) {
        gIsolatedChild = true;
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
        return runHere(work);
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
    // No fork here, so a closure cannot be isolated; if this faults, it takes
    // the program. Guarded work goes through tryIsolated instead.
    return runHere(work);
#endif
}

Attempt tryIsolated(const GuardedWork& work) {
    switch (isolation()) {
        case Isolation::Fork:
            return tryInChild(work.run);
        case Isolation::None:
            return runHere(work.run);
        case Isolation::Worker: {
            // One worker for every blocking trial, kept warm between them.
            static std::mutex lock;
            const std::lock_guard<std::mutex> hold(lock);
            static AsyncTrial trial;
            trial.start(work);
            return trial.wait();
        }
    }
    return runHere(work.run);
}

AsyncTrial::AsyncTrial() = default;

AsyncTrial::~AsyncTrial() { abandon(); }

bool AsyncTrial::running() const { return pid_ >= 0 || waitingOnWorker_; }

void AsyncTrial::start(const GuardedWork& work) {
    abandon();
    done_ = false;
    result_ = Attempt::Refused;

    switch (isolation()) {
        case Isolation::Fork: {
#if TG_HAVE_FORK
            const pid_t pid = fork();
            if (pid == 0) {
                gIsolatedChild = true;
                int code = 2;
                try {
                    code = work.run() ? 0 : 1;
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
            break;                               // no process to be had: answer here
        }
        case Isolation::Worker: {
            if (!work.encode) break;             // nothing a worker could rebuild it from
            if (!worker_) worker_ = std::make_unique<Worker>();
            switch (worker_->send(work.kind, work.encode())) {
                case Worker::Sent::Ok:
                    waitingOnWorker_ = true;
                    return;
                case Worker::Sent::Died:
                    worker_->stop();
                    result_ = Attempt::Crashed;
                    done_ = true;
                    return;
                case Worker::Sent::CouldNotStart:
                    break;                       // answer here rather than not at all
            }
            break;
        }
        case Isolation::None:
            break;
    }
    result_ = runHere(work.run);
    done_ = true;
}

bool AsyncTrial::poll() {
    if (done_) return true;
    if (waitingOnWorker_) {
        Attempt answer = Attempt::Refused;
        const int state = worker_->readAnswer(answer, /*block=*/false);
        if (state == 0) return false;
        waitingOnWorker_ = false;
        done_ = true;
        if (state < 0) {
            worker_->stop();
            result_ = Attempt::Crashed;
        } else {
            result_ = answer;
        }
        return true;
    }
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

Attempt AsyncTrial::wait() {
    if (done_) return result_;
    if (waitingOnWorker_) {
        Attempt answer = Attempt::Refused;
        const int state = worker_->readAnswer(answer, /*block=*/true);
        waitingOnWorker_ = false;
        done_ = true;
        if (state < 0) {
            worker_->stop();
            result_ = Attempt::Crashed;
        } else {
            result_ = answer;
        }
        return result_;
    }
#if TG_HAVE_FORK
    if (pid_ >= 0) {
        int status = 0;
        pid_t got = 0;
        while ((got = waitpid(static_cast<pid_t>(pid_), &status, 0)) < 0 && errno == EINTR) {}
        pid_ = -1;
        done_ = true;
        if (got < 0 || WIFSIGNALED(status) || !WIFEXITED(status)) result_ = Attempt::Crashed;
        else if (WEXITSTATUS(status) == 0) result_ = Attempt::Ok;
        else if (WEXITSTATUS(status) == 1) result_ = Attempt::Refused;
        else result_ = Attempt::Crashed;
    }
#endif
    return result_;
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
    // A worker in the middle of a trial cannot be told to stop, so it is
    // ended, and the next trial starts another. An idle one is kept.
    if (waitingOnWorker_) {
        worker_->stop();
        waitingOnWorker_ = false;
    }
    done_ = false;
}

// ---- the worker process, from its own side ---------------------------------

namespace {

#if TG_WINDOWS
LONG WINAPI endOnFault(EXCEPTION_POINTERS*) {
    // No dialog, no report, no debugger prompt: a fault is an answer, and the
    // application reads it from the pipe closing.
    TerminateProcess(GetCurrentProcess(), 3);
    return EXCEPTION_EXECUTE_HANDLER;
}
#endif

} // namespace

int serveTrials(EncodedRunner run) {
#if TG_WINDOWS
    SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX | SEM_NOOPENFILEERRORBOX);
    SetUnhandledExceptionFilter(endOnFault);
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);

    const HANDLE in = GetStdHandle(STD_INPUT_HANDLE);
    // The answers get a handle of their own, and standard output is pointed at
    // nothing, so whatever the kernel prints cannot land in the middle of one.
    // Duplicated first: reopening stdout closes the handle it had.
    HANDLE out = nullptr;
    if (!DuplicateHandle(GetCurrentProcess(), GetStdHandle(STD_OUTPUT_HANDLE),
                         GetCurrentProcess(), &out, 0, FALSE, DUPLICATE_SAME_ACCESS))
        return 2;
    FILE* discarded = nullptr;
    freopen_s(&discarded, "NUL", "w", stdout);
    SetStdHandle(STD_OUTPUT_HANDLE, INVALID_HANDLE_VALUE);

    auto readAll = [&](void* bytes, size_t n) {
        auto* p = static_cast<char*>(bytes);
        while (n > 0) {
            DWORD got = 0;
            const DWORD chunk = static_cast<DWORD>(std::min<size_t>(n, 1u << 30));
            if (!ReadFile(in, p, chunk, &got, nullptr) || got == 0) return false;
            p += got;
            n -= got;
        }
        return true;
    };
    auto writeAll = [&](const void* bytes, size_t n) {
        DWORD wrote = 0;
        return WriteFile(out, bytes, static_cast<DWORD>(n), &wrote, nullptr) && wrote == n;
    };
#else
    const int in = STDIN_FILENO;
    // The answers get a descriptor of their own, and anything printed goes to
    // the error stream -- or nowhere, if there is none -- never between them.
    const int out = dup(STDOUT_FILENO);
    if (out < 0) return 2;
    fcntl(out, F_SETFD, FD_CLOEXEC);
    if (dup2(STDERR_FILENO, STDOUT_FILENO) < 0) {
        const int nothing = open("/dev/null", O_WRONLY);
        if (nothing >= 0) {
            dup2(nothing, STDOUT_FILENO);
            close(nothing);
        }
    }

    auto readAll = [&](void* bytes, size_t n) {
        auto* p = static_cast<char*>(bytes);
        while (n > 0) {
            const ssize_t got = ::read(in, p, n);
            if (got < 0 && errno == EINTR) continue;
            if (got <= 0) return false;
            p += got;
            n -= static_cast<size_t>(got);
        }
        return true;
    };
    auto writeAll = [&](const void* bytes, size_t n) {
        const auto* p = static_cast<const char*>(bytes);
        while (n > 0) {
            const ssize_t wrote = ::write(out, p, n);
            if (wrote < 0 && errno == EINTR) continue;
            if (wrote <= 0) return false;
            p += wrote;
            n -= static_cast<size_t>(wrote);
        }
        return true;
    };
#endif

    std::string payload;
    for (;;) {
        unsigned char header[16];
        if (!readAll(header, sizeof header)) return 0;       // the application has gone
        if (get32(header) != kRequest) return 2;
        const uint32_t kind = get32(header + 4);
        const uint64_t size = get64(header + 8);
        if (size > kMaxPayload) return 2;
        payload.resize(static_cast<size_t>(size));
        if (size > 0 && !readAll(payload.data(), payload.size())) return 0;

        bool ok = false;
        try {
            ok = run(kind, payload);
        } catch (...) {
            ok = false;
        }
        unsigned char answer[8];
        put32(answer, kAnswer);
        put32(answer + 4, ok ? 0u : 1u);
        if (!writeAll(answer, sizeof answer)) return 0;
    }
}

} // namespace tg
