// Tangent - what happens when it crashes.
//
// A fault outside the kernel's guarded trials used to take the session with it
// and leave nothing behind. Now, before the process goes:
//
//   - a report is written: the signal, where it was (a backtrace), the build,
//     and the last things done (core/crashlog.h);
//   - the work is saved aside, where the next start offers it back -- tried in
//     a forked copy of the process, with a time limit, because the copy
//     inherits whatever state the fault left and may hang; if it does, it is
//     killed and the report still stands;
//   - and the next start says what happened and where the report is.
//
// The handler acts only in the process that installed it: a guarded kernel
// trial is a forked child, its crash is expected and is its parent's to read.
#pragma once

#include <string>

namespace tg::crash {

// What saving the work aside came to.
enum class Saved { Nothing = 0, Done = 1, Failed = 2 };   // nothing unsaved / saved / could not

// `dir` is where reports go (created if it is not there); `build` names the
// build in them. `save` is called, in a forked copy of the crashed process, to
// save the work aside.
void install(const std::string& dir, const std::string& build, Saved (*save)(void* context), void* context);

// The report the last session left, if it crashed and it has not been seen
// yet; empty otherwise.
std::string pendingReport(const std::string& dir);
// It has been seen: the next start does not mention it again.
void acknowledge(const std::string& dir);

// For the test: fault on purpose, as a bug would.
[[noreturn]] void crashNow();

} // namespace tg::crash
