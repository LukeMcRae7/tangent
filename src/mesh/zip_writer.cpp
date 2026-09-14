#include "mesh/zip_writer.h"

#include <zlib.h>

#include <ctime>
#include <limits>

namespace tg {
namespace {

constexpr uint32_t kLocalHeader   = 0x04034b50;
constexpr uint32_t kCentralHeader = 0x02014b50;
constexpr uint32_t kEndOfCentral  = 0x06054b50;
constexpr uint16_t kVersion       = 20;          // 2.0: deflate, folders
constexpr uint16_t kStored        = 0;
constexpr uint16_t kDeflated      = 8;
constexpr uint64_t kLimit         = std::numeric_limits<uint32_t>::max();

void le16(std::string& out, uint16_t v) {
    out.push_back(static_cast<char>(v & 0xFF));
    out.push_back(static_cast<char>((v >> 8) & 0xFF));
}

void le32(std::string& out, uint32_t v) {
    le16(out, static_cast<uint16_t>(v & 0xFFFF));
    le16(out, static_cast<uint16_t>((v >> 16) & 0xFFFF));
}

// Raw deflate -- no zlib header or trailer, which is what a ZIP entry holds.
bool deflateRaw(std::string_view in, std::string& out) {
    z_stream zs{};
    if (deflateInit2(&zs, Z_DEFAULT_COMPRESSION, Z_DEFLATED, -15, 8, Z_DEFAULT_STRATEGY) != Z_OK)
        return false;
    out.resize(deflateBound(&zs, static_cast<uLong>(in.size())));
    zs.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(in.data()));
    zs.avail_in = static_cast<uInt>(in.size());
    zs.next_out = reinterpret_cast<Bytef*>(out.data());
    zs.avail_out = static_cast<uInt>(out.size());
    const int rc = deflate(&zs, Z_FINISH);
    const size_t produced = out.size() - zs.avail_out;
    deflateEnd(&zs);
    if (rc != Z_STREAM_END) return false;
    out.resize(produced);
    return true;
}

} // namespace

ZipWriter::~ZipWriter() {
    if (file_) std::fclose(file_);
}

bool ZipWriter::open(const std::string& path, std::string* error) {
    if (file_) std::fclose(file_);
    entries_.clear();
    written_ = 0;
    failed_ = false;
    file_ = std::fopen(path.c_str(), "wb");
    if (!file_) {
        if (error) *error = "cannot open " + path;
        return false;
    }

    // Stamped with the time of writing, in the local time DOS dates are in.
    const std::time_t now = std::time(nullptr);
    std::tm t{};
#ifdef _WIN32
    localtime_s(&t, &now);
#else
    localtime_r(&now, &t);
#endif
    const int year = t.tm_year + 1900;
    if (year >= 1980) {
        dosTime_ = static_cast<uint16_t>((t.tm_hour << 11) | (t.tm_min << 5) | (t.tm_sec / 2));
        dosDate_ = static_cast<uint16_t>(((year - 1980) << 9) | ((t.tm_mon + 1) << 5) | t.tm_mday);
    } else {
        dosTime_ = 0;
        dosDate_ = (1 << 5) | 1;                    // 1980-01-01, the earliest there is
    }
    return true;
}

bool ZipWriter::put(const void* bytes, size_t n) {
    if (failed_ || !file_) return false;
    if (n > 0 && std::fwrite(bytes, 1, n, file_) != n) {
        failed_ = true;
        return false;
    }
    written_ += n;
    return true;
}

bool ZipWriter::add(const std::string& name, std::string_view data, std::string* error) {
    if (!file_) {
        if (error) *error = "the archive is not open";
        return false;
    }
    // Neither the part nor where it starts may pass what 32 bits can say; past
    // that a ZIP needs the 64-bit extension, which this does not write.
    if (data.size() >= kLimit || written_ >= kLimit || name.size() > 0xFFFF) {
        if (error) *error = "the file would be larger than 4 GB, which this writer does not do";
        return false;
    }

    Entry e;
    e.name = name;
    e.offset = static_cast<uint32_t>(written_);
    e.size = static_cast<uint32_t>(data.size());
    e.crc = static_cast<uint32_t>(
        crc32(crc32(0L, Z_NULL, 0), reinterpret_cast<const Bytef*>(data.data()),
              static_cast<uInt>(data.size())));

    std::string packed;
    std::string_view body = data;
    e.method = kStored;
    if (!data.empty() && deflateRaw(data, packed) && packed.size() < data.size()) {
        e.method = kDeflated;
        body = packed;
    }
    e.packed = static_cast<uint32_t>(body.size());

    std::string header;
    header.reserve(30 + name.size());
    le32(header, kLocalHeader);
    le16(header, kVersion);
    le16(header, 0);                                // flags: sizes are known up front
    le16(header, e.method);
    le16(header, dosTime_);
    le16(header, dosDate_);
    le32(header, e.crc);
    le32(header, e.packed);
    le32(header, e.size);
    le16(header, static_cast<uint16_t>(name.size()));
    le16(header, 0);                                // no extra field
    header += name;

    if (!put(header.data(), header.size()) || !put(body.data(), body.size())) {
        if (error) *error = "the file could not be written";
        return false;
    }
    entries_.push_back(std::move(e));
    return true;
}

bool ZipWriter::finish(std::string* error) {
    if (!file_) {
        if (error) *error = "the archive is not open";
        return false;
    }
    const uint64_t directoryAt = written_;
    std::string dir;
    for (const Entry& e : entries_) {
        le32(dir, kCentralHeader);
        le16(dir, kVersion);                        // made by: MS-DOS attributes, 2.0
        le16(dir, kVersion);                        // needed to extract
        le16(dir, 0);
        le16(dir, e.method);
        le16(dir, dosTime_);
        le16(dir, dosDate_);
        le32(dir, e.crc);
        le32(dir, e.packed);
        le32(dir, e.size);
        le16(dir, static_cast<uint16_t>(e.name.size()));
        le16(dir, 0);                               // extra
        le16(dir, 0);                               // comment
        le16(dir, 0);                               // disk
        le16(dir, 0);                               // internal attributes
        le32(dir, 0);                               // external attributes
        le32(dir, e.offset);
        dir += e.name;
    }
    const bool fits = directoryAt < kLimit && dir.size() < kLimit && entries_.size() <= 0xFFFF;

    std::string end;
    le32(end, kEndOfCentral);
    le16(end, 0);                                   // this disk
    le16(end, 0);                                   // the disk the directory starts on
    le16(end, static_cast<uint16_t>(entries_.size()));
    le16(end, static_cast<uint16_t>(entries_.size()));
    le32(end, static_cast<uint32_t>(dir.size()));
    le32(end, static_cast<uint32_t>(directoryAt));
    le16(end, 0);                                   // no comment

    const bool wrote = fits && put(dir.data(), dir.size()) && put(end.data(), end.size());
    const bool closed = std::fclose(file_) == 0;
    file_ = nullptr;
    if (!fits) {
        if (error) *error = "the file would be larger than 4 GB, which this writer does not do";
        return false;
    }
    if (!wrote || !closed) {
        if (error) *error = "the file could not be written";
        return false;
    }
    return true;
}

} // namespace tg
