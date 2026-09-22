// Tangent - where a test may write a file.
//
// One place, because this has now been got wrong five times. A hardcoded
// "/tmp/x" is a path on Linux and a path on the *current drive's root* on
// Windows -- C:\tmp\x -- which does not exist, so every write fails. The
// failures do not read as path failures either: a save reports "cannot open",
// and a read-back of the file that was never written reports whatever the
// reader makes of an empty file, several checks later.
//
// std::filesystem::temp_directory_path() honours TMPDIR on Linux and TEMP on
// Windows, so it is the same answer this had on Linux and a working one
// everywhere else.
#pragma once

#include <filesystem>
#include <string>

namespace tg {

inline std::string tempPath(const std::string& name) {
    return (std::filesystem::temp_directory_path() / ("tangent_test_" + name)).string();
}

} // namespace tg
