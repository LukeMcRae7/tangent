// Tangent - the operating system's file chooser.
//
// Typing a path into a text box works, and is what this had. It is also the
// only part of the program that asks the user to know something the computer
// already knows: where their files are. SDL ships a native chooser on every
// platform this targets, so there is no reason to keep asking.
//
// Two awkward facts shape this wrapper, both from SDL's contract:
//
//   The answer arrives later, possibly on another thread. So nothing is done
//   with it there -- the result is parked behind a mutex and the frame loop
//   collects it, on the thread that owns the scene.
//
//   There is no way to ask whether a chooser can be shown. On Linux it needs
//   an XDG desktop portal or zenity or kdialog, and if none is there the
//   failure arrives through the same callback as everything else. So the
//   caller cannot check first; it starts a dialog, and keeps a typed path as
//   the fallback for when one comes back unavailable.
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

struct SDL_Window;

namespace tg {

class FileDialog {
public:
    enum class Kind { Open, Save };

    struct Filter {
        const char* name;      // "STEP file"
        const char* pattern;   // "step;stp", or "*" for everything
    };

    enum class Result {
        None,       // nothing has come back yet
        Chosen,     // a path, in `path`
        Cancelled,  // the user closed it
        Failed,     // no chooser could be shown; `path` says why
    };

    // Asks for a chooser. Returns immediately. `filters` is copied, so the
    // caller need not keep it alive -- SDL requires that the array outlive the
    // call, and this owns a copy for exactly that reason.
    //
    // `startAt` may be a directory or a suggested file; empty means wherever
    // the platform would go on its own.
    void show(Kind kind, SDL_Window* window, const std::vector<Filter>& filters,
              const std::string& startAt);

    // True between show() and the answer arriving.
    bool waiting() const;

    // Collects the answer, if there is one. Call it once a frame.
    Result take(std::string& path);

private:
    // Held by shared_ptr so that a chooser still open when this object goes
    // away has somewhere valid to write its answer.
    struct Pending {
        std::mutex mutex;
        Result result = Result::None;
        std::string path;
        bool collected = false;

        // Kept alive for as long as the dialog is up, because SDL reads the
        // strings out of it rather than copying them.
        std::vector<std::string> filterText;
        std::vector<void*> filterArray;   // SDL_DialogFileFilter, opaque here
    };

    std::shared_ptr<Pending> pending_;
    bool waiting_ = false;
};

} // namespace tg
