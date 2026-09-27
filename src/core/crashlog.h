// Tangent - the last few things done, for a crash report.
//
// A crash report that only says where the program was is half an answer; what
// the person had just done is the other half, and it is usually what makes the
// fault reproducible. So the application notes what it does -- a step made, a
// file opened, a refusal said -- into a small fixed ring, and the crash handler
// writes the ring out.
//
// Fixed and preallocated on purpose: the handler runs after the fault, when
// the heap may be what broke, so reading the ring must not allocate. Writing
// to it happens in ordinary code and may format freely.
#pragma once

namespace tg::crashlog {

// One line, printf-style. Cheap: a format into a fixed slot.
void note(const char* fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 1, 2)))
#endif
    ;

// The ring, oldest first, one line each, written to a file descriptor with
// nothing but write(): safe inside a signal handler.
void writeTo(int fd);

} // namespace tg::crashlog
