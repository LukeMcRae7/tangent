// Tangent - writing a ZIP archive.
//
// 3MF is a ZIP of a few XML parts, so exporting one needs a ZIP writer and
// nothing more of the format: no reading, no ZIP64, no encryption, no streaming
// of unknown sizes. What it does write is plain and widely readable -- deflate,
// or stored where deflate would not help, local headers carrying the real sizes
// and CRCs rather than trailing data descriptors, and a central directory at
// the end.
//
// Each part is compressed and written as it is added, so only one part's
// compressed bytes are held at a time.
#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include <string_view>
#include <vector>

namespace tg {

class ZipWriter {
public:
    ZipWriter() = default;
    ~ZipWriter();
    ZipWriter(const ZipWriter&) = delete;
    ZipWriter& operator=(const ZipWriter&) = delete;

    // Creates or truncates the file.
    bool open(const std::string& path, std::string* error);

    // Adds one part, by its path inside the archive ("3D/3dmodel.model").
    bool add(const std::string& name, std::string_view data, std::string* error);

    // Writes the central directory and closes the file. Without this the
    // archive is not readable, so the caller should treat a false here as the
    // export having failed.
    bool finish(std::string* error);

private:
    struct Entry {
        std::string name;
        uint32_t crc = 0;
        uint32_t size = 0;
        uint32_t packed = 0;
        uint16_t method = 0;
        uint32_t offset = 0;
    };

    bool put(const void* bytes, size_t n);

    FILE* file_ = nullptr;
    uint64_t written_ = 0;
    uint16_t dosTime_ = 0;
    uint16_t dosDate_ = 0;
    bool failed_ = false;
    std::vector<Entry> entries_;
};

} // namespace tg
