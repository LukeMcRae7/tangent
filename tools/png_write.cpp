// Tangent - the PNG the icons are written as.
//
// Split from the rasteriser because it shares nothing with it: this knows about
// bytes and CRCs, that one knows about triangles. Keeping them apart is what
// lets the reader in src/ui/icons.cpp be tested against the writer without
// dragging the mesh kernel into the test.
#include "icon_raster.h"

#include <algorithm>
#include <cstdio>
#include <cstdint>
#include <vector>

namespace tg::icons {

// ---- PNG -------------------------------------------------------------------
namespace {

uint32_t crc32Of(const uint8_t* data, size_t n, uint32_t crc = 0xFFFFFFFFu) {
    static uint32_t table[256];
    static bool built = false;
    if (!built) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            table[i] = c;
        }
        built = true;
    }
    for (size_t i = 0; i < n; ++i) crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    return crc;
}

void be32(std::vector<uint8_t>& out, uint32_t v) {
    out.push_back(static_cast<uint8_t>(v >> 24));
    out.push_back(static_cast<uint8_t>(v >> 16));
    out.push_back(static_cast<uint8_t>(v >> 8));
    out.push_back(static_cast<uint8_t>(v));
}

void chunk(std::vector<uint8_t>& out, const char tag[5], const std::vector<uint8_t>& body) {
    be32(out, static_cast<uint32_t>(body.size()));
    const size_t start = out.size();
    out.insert(out.end(), tag, tag + 4);
    out.insert(out.end(), body.begin(), body.end());
    be32(out, crc32Of(out.data() + start, out.size() - start) ^ 0xFFFFFFFFu);
}

}  // namespace

bool writePng(const Image& image, const std::string& path) {
    // Raw scanlines, each with a zero filter byte.
    std::vector<uint8_t> raw;
    raw.reserve(static_cast<size_t>(image.height) * (image.width * 4 + 1));
    for (int y = 0; y < image.height; ++y) {
        raw.push_back(0);
        for (int x = 0; x < image.width; ++x) {
            const Rgba p = image.at(x, y);
            raw.push_back(p.r); raw.push_back(p.g); raw.push_back(p.b); raw.push_back(p.a);
        }
    }

    // zlib around stored deflate blocks: no compressor needed, and an icon is a
    // few kilobytes either way.
    std::vector<uint8_t> z;
    z.push_back(0x78); z.push_back(0x01);
    size_t at = 0;
    while (at < raw.size()) {
        const size_t n = std::min<size_t>(65535, raw.size() - at);
        z.push_back(at + n >= raw.size() ? 1 : 0);
        z.push_back(static_cast<uint8_t>(n & 0xFF));
        z.push_back(static_cast<uint8_t>(n >> 8));
        z.push_back(static_cast<uint8_t>(~n & 0xFF));
        z.push_back(static_cast<uint8_t>((~n >> 8) & 0xFF));
        z.insert(z.end(), raw.begin() + at, raw.begin() + at + n);
        at += n;
    }
    uint32_t s1 = 1, s2 = 0;
    for (uint8_t byte : raw) { s1 = (s1 + byte) % 65521; s2 = (s2 + s1) % 65521; }
    be32(z, (s2 << 16) | s1);

    std::vector<uint8_t> png = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    std::vector<uint8_t> ihdr;
    be32(ihdr, static_cast<uint32_t>(image.width));
    be32(ihdr, static_cast<uint32_t>(image.height));
    ihdr.push_back(8);      // bit depth
    ihdr.push_back(6);      // RGBA
    ihdr.push_back(0); ihdr.push_back(0); ihdr.push_back(0);
    chunk(png, "IHDR", ihdr);
    chunk(png, "IDAT", z);
    chunk(png, "IEND", {});

    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    const bool ok = std::fwrite(png.data(), 1, png.size(), f) == png.size();
    std::fclose(f);
    return ok;
}

} // namespace tg::icons
