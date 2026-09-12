// The icons, and the one part of them that can silently be wrong.
//
// They are written by tools/icon_raster.cpp and read by src/ui/icons.cpp, and
// the two have to agree about a format neither of them got from a library. A
// reader that is slightly wrong does not fail: it renders noise, or an icon
// shifted by a scanline, which is the kind of thing that survives review
// because nobody looks closely at a 24-pixel picture.
#include "ui/icons.h"
#include "icon_raster.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}

int main(int argc, char** argv) {
    const std::string assets = argc > 1 ? argv[1] : "assets/icons";

    std::printf("--- what the baker writes is what the reader reads ---\n");
    {
        // Every value in every channel, so a swapped byte order or a dropped
        // filter byte cannot pass.
        icons::Image img;
        img.width = 17;
        img.height = 11;
        img.pixels.resize(17 * 11);
        for (int y = 0; y < img.height; ++y)
            for (int x = 0; x < img.width; ++x)
                img.at(x, y) = icons::Rgba{static_cast<uint8_t>(x * 13 + y),
                                           static_cast<uint8_t>(255 - x * 7),
                                           static_cast<uint8_t>(x ^ (y * 3)),
                                           static_cast<uint8_t>(y * 23 + x)};

        const std::string path = std::string(std::getenv("TMPDIR") ? std::getenv("TMPDIR") : "/tmp")
                               + "/tangent-icon-roundtrip.png";
        check(icons::writePng(img, path), "wrote the test image");

        int w = 0, h = 0;
        std::vector<unsigned char> rgba;
        check(readIconImage(path, w, h, rgba), "read it back");
        check(w == 17 && h == 11, "same size");
        check(rgba.size() == 17u * 11u * 4u, "same number of bytes");

        bool same = rgba.size() == 17u * 11u * 4u;
        for (int y = 0; same && y < 11; ++y)
            for (int x = 0; same && x < 17; ++x) {
                const icons::Rgba want = img.at(x, y);
                const unsigned char* got = &rgba[(static_cast<size_t>(y) * 17 + x) * 4];
                same = got[0] == want.r && got[1] == want.g &&
                       got[2] == want.b && got[3] == want.a;
            }
        check(same, "every pixel came back exactly as it went in");
        std::remove(path.c_str());
    }

    std::printf("--- a wide image crosses a block boundary ---\n");
    {
        // The baker breaks the stream into 64KB stored blocks. An image with
        // more bytes than that exercises the second block, which is where a
        // reader that stops at the first one gives up half an icon.
        icons::Image img;                        // 200*4+1 = 801 bytes a row
        img.width = 200;
        img.height = 120;
        img.pixels.resize(200 * 120);
        for (int y = 0; y < img.height; ++y)
            for (int x = 0; x < img.width; ++x)
                img.at(x, y) = icons::Rgba{static_cast<uint8_t>(x), static_cast<uint8_t>(y),
                                           static_cast<uint8_t>(x + y), 255};
        const std::string path = std::string(std::getenv("TMPDIR") ? std::getenv("TMPDIR") : "/tmp")
                               + "/tangent-icon-blocks.png";
        check(icons::writePng(img, path), "wrote a 96KB image");

        int w = 0, h = 0;
        std::vector<unsigned char> rgba;
        check(readIconImage(path, w, h, rgba) && w == 200 && h == 120, "read it back whole");
        const unsigned char* last = &rgba[(119u * 200 + 199) * 4];
        check(last[0] == 199 && last[1] == 119, "including the very last pixel");
        std::remove(path.c_str());
    }

    std::printf("--- the icons that ship ---\n");
    {
        static const char* kNames[] = {
            "box", "cylinder", "sphere", "cone", "torus",
            "union", "difference", "intersection",
            "extrude", "fillet", "chamfer", "shell", "inset",
        };
        int found = 0;
        for (const char* name : kNames) {
            int w = 0, h = 0;
            std::vector<unsigned char> rgba;
            if (!readIconImage(assets + "/" + name + ".png", w, h, rgba)) continue;
            ++found;
            check(w == 64 && h == 64, std::string(name) + " is 64 x 64");

            // A picture, not an empty square: something has to be opaque, and
            // it must not cover the whole tile either, or it is a solid block.
            size_t opaque = 0;
            for (size_t i = 3; i < rgba.size(); i += 4)
                if (rgba[i] > 128) ++opaque;
            const size_t total = rgba.size() / 4;
            check(opaque > total / 20, std::string(name) + " has something in it");
            check(opaque < total * 9 / 10, std::string(name) + " is not a filled square");
        }
        std::printf("  %d icons read\n", found);
        check(found == 13, "all thirteen are there");
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
