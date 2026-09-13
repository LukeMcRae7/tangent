#include "app/file_dialog.h"

#include <SDL3/SDL.h>

#include <cstring>

namespace tg {
namespace {

// SDL hands the callback a heap pointer we chose. What we choose is a
// deliberately leaked copy of the shared_ptr, so the state survives even if the
// FileDialog that started this is gone by the time the user picks a file. The
// callback takes that copy back and lets it go.
void SDLCALL onAnswer(void* userdata, const char* const* filelist, int /*filter*/) {
    std::unique_ptr<std::shared_ptr<void>> holder(
        static_cast<std::shared_ptr<void>*>(userdata));
    if (!holder || !*holder) return;

    // The type is erased in the handoff so the header does not have to expose
    // Pending. This is the only place that puts it back, and the only place
    // that made it in the first place.
    struct Slot {
        std::mutex mutex;
        FileDialog::Result result;
        std::string path;
        bool collected;
    };
    auto* slot = static_cast<Slot*>(holder->get());

    std::lock_guard<std::mutex> lock(slot->mutex);
    if (!filelist) {
        // No chooser could be shown at all -- no portal, no zenity, no
        // kdialog. SDL_GetError says which, and the caller needs to know
        // because it has to offer something else instead.
        slot->result = FileDialog::Result::Failed;
        const char* err = SDL_GetError();
        slot->path = (err && *err) ? err : "no file chooser is available";
    } else if (!filelist[0]) {
        slot->result = FileDialog::Result::Cancelled;
        slot->path.clear();
    } else {
        slot->result = FileDialog::Result::Chosen;
        slot->path = filelist[0];
    }
}

} // namespace

void FileDialog::show(Kind kind, SDL_Window* window, const std::vector<Filter>& filters,
                      const std::string& startAt) {
    // A second request replaces the first. The old state stays alive on its own
    // shared_ptr until its callback fires, so abandoning it here is safe.
    pending_ = std::make_shared<Pending>();
    waiting_ = true;

    // SDL reads the filter strings while the dialog is up rather than copying
    // them, so both the strings and the array of pointers into them have to
    // live somewhere that outlasts this function.
    pending_->filterText.reserve(filters.size() * 2);
    auto* sdlFilters = new SDL_DialogFileFilter[filters.size() ? filters.size() : 1];
    for (size_t i = 0; i < filters.size(); ++i) {
        pending_->filterText.push_back(filters[i].name ? filters[i].name : "");
        pending_->filterText.push_back(filters[i].pattern ? filters[i].pattern : "*");
    }
    for (size_t i = 0; i < filters.size(); ++i) {
        sdlFilters[i].name = pending_->filterText[i * 2].c_str();
        sdlFilters[i].pattern = pending_->filterText[i * 2 + 1].c_str();
    }
    pending_->filterArray.push_back(sdlFilters);

    // Type-erased so the header need not include SDL. Freed by the callback.
    auto* handoff = new std::shared_ptr<void>(pending_, pending_.get());

    const char* location = startAt.empty() ? nullptr : startAt.c_str();
    const int n = static_cast<int>(filters.size());
    if (kind == Kind::Open)
        SDL_ShowOpenFileDialog(onAnswer, handoff, window, sdlFilters, n, location,
                               /*allow_many=*/false);
    else
        SDL_ShowSaveFileDialog(onAnswer, handoff, window, sdlFilters, n, location);
}

bool FileDialog::waiting() const { return waiting_; }

FileDialog::Result FileDialog::take(std::string& path) {
    if (!pending_) return Result::None;

    Result got = Result::None;
    {
        std::lock_guard<std::mutex> lock(pending_->mutex);
        if (pending_->result == Result::None || pending_->collected) return Result::None;
        pending_->collected = true;
        got = pending_->result;
        path = pending_->path;
    }

    waiting_ = false;
    // The filters were needed only while the chooser was up.
    for (void* f : pending_->filterArray)
        delete[] static_cast<SDL_DialogFileFilter*>(f);
    pending_->filterArray.clear();
    pending_.reset();
    return got;
}

} // namespace tg
