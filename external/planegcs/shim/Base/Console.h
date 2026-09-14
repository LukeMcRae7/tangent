// Tangent shim: FreeCAD's console, which planegcs writes solver diagnostics to.
// Tangent reports solver results through its own interface, so these messages
// go nowhere.
#pragma once

namespace Base {

struct ConsoleShim {
    template <typename... Args> void log(Args&&...) {}
    template <typename... Args> void warning(Args&&...) {}
    template <typename... Args> void error(Args&&...) {}
    template <typename... Args> void message(Args&&...) {}
};

inline ConsoleShim& Console() {
    static ConsoleShim console;
    return console;
}

} // namespace Base
