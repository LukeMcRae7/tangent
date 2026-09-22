#include "ui/png.h"

#include <zlib.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>

namespace tg {
namespace {

uint32_t beU32(const uint8_t* p) {
    return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3];
}

int paeth(int a, int b, int c) {
    const int p = a + b - c;
    const int pa = std::abs(p - a), pb = std::abs(p - b), pc = std::abs(p - c);
    if (pa <= pb && pa <= pc) return a;
    if (pb <= pc) return b;
    return c;
}

} // namespace

bool readPng(const std::string& path, int& w, int& h, std::vector<unsigned char>& rgba) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return false;
    const std::streamsize size = f.tellg();
    if (size < 8) return false;
    f.seekg(0);
    std::vector<uint8_t> d(static_cast<size_t>(size));
    if (!f.read(reinterpret_cast<char*>(d.data()), size)) return false;

    static const uint8_t kMagic[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    if (std::memcmp(d.data(), kMagic, 8) != 0) return false;

    std::vector<uint8_t> idat;
    int depth = 0, colour = 0, interlace = 0;
    w = h = 0;
    size_t at = 8;
    while (at + 8 <= d.size()) {
        const uint32_t len = beU32(&d[at]);
        const char* type = reinterpret_cast<const char*>(&d[at + 4]);
        const size_t body = at + 8;
        if (body + len + 4 > d.size()) return false;
        if (std::memcmp(type, "IHDR", 4) == 0) {
            if (len < 13) return false;
            w = static_cast<int>(beU32(&d[body]));
            h = static_cast<int>(beU32(&d[body + 4]));
            depth = d[body + 8];
            colour = d[body + 9];
            interlace = d[body + 12];
        } else if (std::memcmp(type, "IDAT", 4) == 0) {
            idat.insert(idat.end(), d.begin() + body, d.begin() + body + len);
        } else if (std::memcmp(type, "IEND", 4) == 0) {
            break;
        }
        at = body + len + 4;
    }
    if (w <= 0 || h <= 0 || idat.empty()) return false;
    if (depth != 8 || interlace != 0) {
        std::fprintf(stderr, "[ui] %s: only 8-bit non-interlaced PNGs are read\n", path.c_str());
        return false;
    }
    int channels = 0;
    switch (colour) {
        case 0: channels = 1; break;
        case 2: channels = 3; break;
        case 4: channels = 2; break;
        case 6: channels = 4; break;
        default:
            std::fprintf(stderr, "[ui] %s: palette PNGs are not read\n", path.c_str());
            return false;
    }

    const size_t stride = static_cast<size_t>(w) * channels;
    std::vector<uint8_t> raw((stride + 1) * static_cast<size_t>(h));
    uLongf rawLen = static_cast<uLongf>(raw.size());
    if (uncompress(raw.data(), &rawLen, idat.data(), static_cast<uLong>(idat.size())) != Z_OK ||
        rawLen != raw.size()) {
        std::fprintf(stderr, "[ui] %s: the image data did not inflate\n", path.c_str());
        return false;
    }

    // Unfilter in place, a row at a time, each against the row above.
    std::vector<uint8_t> px(stride * static_cast<size_t>(h));
    for (int y = 0; y < h; ++y) {
        const uint8_t filter = raw[static_cast<size_t>(y) * (stride + 1)];
        const uint8_t* src = &raw[static_cast<size_t>(y) * (stride + 1) + 1];
        uint8_t* dst = &px[static_cast<size_t>(y) * stride];
        const uint8_t* up = y > 0 ? &px[static_cast<size_t>(y - 1) * stride] : nullptr;
        for (size_t i = 0; i < stride; ++i) {
            const int a = i >= static_cast<size_t>(channels) ? dst[i - channels] : 0;
            const int b = up ? up[i] : 0;
            const int c = (up && i >= static_cast<size_t>(channels)) ? up[i - channels] : 0;
            int v = src[i];
            switch (filter) {
                case 0: break;
                case 1: v += a; break;
                case 2: v += b; break;
                case 3: v += (a + b) / 2; break;
                case 4: v += paeth(a, b, c); break;
                default: return false;
            }
            dst[i] = static_cast<uint8_t>(v & 0xFF);
        }
    }

    rgba.assign(static_cast<size_t>(w) * static_cast<size_t>(h) * 4, 255);
    for (size_t i = 0; i < static_cast<size_t>(w) * static_cast<size_t>(h); ++i) {
        const uint8_t* s = &px[i * channels];
        uint8_t* o = &rgba[i * 4];
        switch (channels) {
            case 1: o[0] = o[1] = o[2] = s[0]; break;
            case 2: o[0] = o[1] = o[2] = s[0]; o[3] = s[1]; break;
            case 3: o[0] = s[0]; o[1] = s[1]; o[2] = s[2]; break;
            default: o[0] = s[0]; o[1] = s[1]; o[2] = s[2]; o[3] = s[3]; break;
        }
    }
    return true;
}

} // namespace tg
