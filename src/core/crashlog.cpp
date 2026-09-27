#include "core/crashlog.h"

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#if defined(_WIN32)
#include <io.h>
#define TG_WRITE _write
#else
#include <unistd.h>
#define TG_WRITE ::write
#endif

namespace tg::crashlog {

namespace {
constexpr int kLines = 40;
constexpr int kWidth = 200;
char g_lines[kLines][kWidth] = {};
std::atomic<unsigned> g_next{0};

void put(int fd, const char* s) {
    const size_t n = std::strlen(s);
    if (n) (void)!TG_WRITE(fd, s, static_cast<unsigned>(n));
}
} // namespace

void note(const char* fmt, ...) {
    char line[kWidth];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(line, sizeof line, fmt, args);
    va_end(args);
    const unsigned slot = g_next.fetch_add(1) % kLines;
    std::memcpy(g_lines[slot], line, sizeof line);
}

void writeTo(int fd) {
    const unsigned next = g_next.load();
    const unsigned count = next < kLines ? next : kLines;
    for (unsigned i = 0; i < count; ++i) {
        const unsigned slot = (next - count + i) % kLines;
        put(fd, "  ");
        put(fd, g_lines[slot]);
        put(fd, "\n");
    }
    if (count == 0) put(fd, "  (nothing yet)\n");
}

} // namespace tg::crashlog
