#include "ui/icons.h"

#include "core/palette.h"

#include <epoxy/gl.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>

namespace tg {
namespace {

constexpr int kIconPx = 64;          // what the baker writes
constexpr int kGrid   = 4;           // 4 x 4 holds the thirteen with room to spare
constexpr int kAtlasPx = kIconPx * kGrid;

const char* kFiles[static_cast<int>(Icon::Count)] = {
    "box.png", "cylinder.png", "sphere.png", "cone.png", "torus.png",
    "union.png", "difference.png", "intersection.png",
    "extrude.png", "fillet.png", "chamfer.png", "shell.png", "inset.png",
};

GLuint texture = 0;
bool   ready = false;

uint32_t beU32(const uint8_t* p) {
    return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3];
}

// Reads exactly what tools/icon_raster.cpp writes, and nothing else.
//
// That is: eight-bit RGBA, no interlacing, every scanline filtered with zero,
// and the zlib stream built from stored -- uncompressed -- deflate blocks. It
// is not a PNG decoder and does not pretend to be one; anything else is
// refused by name so a re-baked file that needs a real inflate says so rather
// than rendering as noise.
bool readBakedPngImpl(const std::string& path, int& w, int& h, std::vector<uint8_t>& rgba) {
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
    size_t at = 8;
    w = h = 0;
    while (at + 8 <= d.size()) {
        const uint32_t len = beU32(&d[at]);
        const char* type = reinterpret_cast<const char*>(&d[at + 4]);
        const size_t body = at + 8;
        if (body + len + 4 > d.size()) return false;

        if (std::memcmp(type, "IHDR", 4) == 0) {
            if (len < 13) return false;
            w = static_cast<int>(beU32(&d[body]));
            h = static_cast<int>(beU32(&d[body + 4]));
            if (d[body + 8] != 8 || d[body + 9] != 6 || d[body + 12] != 0) {
                std::fprintf(stderr, "[ui] %s is not 8-bit RGBA without interlacing\n",
                             path.c_str());
                return false;
            }
        } else if (std::memcmp(type, "IDAT", 4) == 0) {
            idat.insert(idat.end(), d.begin() + body, d.begin() + body + len);
        } else if (std::memcmp(type, "IEND", 4) == 0) {
            break;
        }
        at = body + len + 4;
    }
    if (w <= 0 || h <= 0 || idat.size() < 2) return false;

    // Past the two-byte zlib header, then the stored blocks.
    std::vector<uint8_t> raw;
    raw.reserve(static_cast<size_t>(h) * (static_cast<size_t>(w) * 4 + 1));
    size_t p = 2;
    while (p + 5 <= idat.size()) {
        const uint8_t header = idat[p];
        if ((header >> 1) & 0x3) {
            std::fprintf(stderr, "[ui] %s is compressed; this reader only takes "
                                 "the stored blocks the baker writes\n", path.c_str());
            return false;
        }
        const size_t n = static_cast<size_t>(idat[p + 1]) | (static_cast<size_t>(idat[p + 2]) << 8);
        if (p + 5 + n > idat.size()) return false;
        raw.insert(raw.end(), idat.begin() + p + 5, idat.begin() + p + 5 + n);
        p += 5 + n;
        if (header & 1) break;                       // final block
    }

    const size_t stride = static_cast<size_t>(w) * 4 + 1;
    if (raw.size() < stride * static_cast<size_t>(h)) return false;

    rgba.assign(static_cast<size_t>(w) * static_cast<size_t>(h) * 4, 0);
    for (int y = 0; y < h; ++y) {
        if (raw[y * stride] != 0) return false;       // only the zero filter
        std::memcpy(&rgba[static_cast<size_t>(y) * w * 4], &raw[y * stride + 1],
                    static_cast<size_t>(w) * 4);
    }
    return true;
}

ImVec2 uvMin(Icon icon) {
    const int i = static_cast<int>(icon);
    return {static_cast<float>(i % kGrid) / kGrid, static_cast<float>(i / kGrid) / kGrid};
}
ImVec2 uvMax(Icon icon) {
    const ImVec2 lo = uvMin(icon);
    return {lo.x + 1.0f / kGrid, lo.y + 1.0f / kGrid};
}

} // namespace

bool readIconImage(const std::string& path, int& w, int& h,
                   std::vector<unsigned char>& rgba) {
    return readBakedPngImpl(path, w, h, rgba);
}

bool loadIcons(const std::string& assetDir) {
    unloadIcons();

    std::vector<uint8_t> atlas(static_cast<size_t>(kAtlasPx) * kAtlasPx * 4, 0);
    int loaded = 0;
    for (int i = 0; i < static_cast<int>(Icon::Count); ++i) {
        int w = 0, h = 0;
        std::vector<uint8_t> px;
        if (!readBakedPngImpl(assetDir + "/" + kFiles[i], w, h, px)) continue;
        if (w != kIconPx || h != kIconPx) {
            std::fprintf(stderr, "[ui] %s is %dx%d, expected %dx%d\n", kFiles[i], w, h,
                         kIconPx, kIconPx);
            continue;
        }
        const int cx = (i % kGrid) * kIconPx, cy = (i / kGrid) * kIconPx;
        for (int y = 0; y < kIconPx; ++y)
            std::memcpy(&atlas[(static_cast<size_t>(cy + y) * kAtlasPx + cx) * 4],
                        &px[static_cast<size_t>(y) * kIconPx * 4],
                        static_cast<size_t>(kIconPx) * 4);
        ++loaded;
    }

    if (loaded == 0) {
        std::fprintf(stderr, "[ui] no icons found in %s; going without them\n",
                     assetDir.c_str());
        return false;
    }

    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, kAtlasPx, kAtlasPx, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, atlas.data());
    // Linear with mipmaps: a 64px icon is most often drawn at 16 or 18, and
    // point sampling at that ratio turns a rounded edge into a staircase.
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    ready = true;
    std::fprintf(stderr, "[ui] icons: %d of %d\n", loaded,
                 static_cast<int>(Icon::Count));
    return true;
}

void unloadIcons() {
    if (texture) glDeleteTextures(1, &texture);
    texture = 0;
    ready = false;
}

bool iconsReady() { return ready; }

void iconImage(Icon icon, float sizePx, float alpha) {
    if (!ready) {
        ImGui::Dummy(ImVec2(sizePx, sizePx));
        return;
    }
    ImGui::ImageWithBg(static_cast<ImTextureID>(texture), ImVec2(sizePx, sizePx),
                       uvMin(icon), uvMax(icon), ImVec4(0, 0, 0, 0),
                       ImVec4(1, 1, 1, alpha));
}

bool iconButton(Icon icon, const char* id, float sizePx, const char* tooltip,
                bool active, bool enabled) {
    ImGui::PushID(id);
    ImGui::BeginDisabled(!enabled);

    if (active) {
        ImGui::PushStyleColor(ImGuiCol_Button,
                              ImVec4(palette::kBrand.r, palette::kBrand.g,
                                     palette::kBrand.b, 0.30f));
        ImGui::PushStyleColor(ImGuiCol_Border,
                              ImVec4(palette::kBrand.r, palette::kBrand.g,
                                     palette::kBrand.b, 0.9f));
    }

    const float pad = ImGui::GetStyle().FramePadding.x;
    bool clicked = false;
    if (ready) {
        clicked = ImGui::ImageButton(id, static_cast<ImTextureID>(texture),
                                     ImVec2(sizePx, sizePx), uvMin(icon), uvMax(icon));
    } else {
        clicked = ImGui::Button(tooltip ? tooltip : id,
                                ImVec2(sizePx + pad * 2, sizePx + pad * 2));
    }

    if (active) ImGui::PopStyleColor(2);
    ImGui::EndDisabled();
    // Not while a menu is open: a button that opens one is still hovered while
    // it is open, and its tooltip lands exactly on top of what it opened.
    if (tooltip && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled) &&
        !ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId | ImGuiPopupFlags_AnyPopupLevel))
        ImGui::SetTooltip("%s", tooltip);
    ImGui::PopID();
    return clicked;
}

} // namespace tg
