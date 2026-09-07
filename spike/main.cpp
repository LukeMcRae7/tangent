// Tangent - Stage 0 spike: run the catalogue on both kernels and print what
// happened. One row per kernel per part; the OCCT volume is the reference the
// mesh rows are measured against, since it is the exact one.
#include "backends.h"

#include <cstdio>
#include <string>
#include <vector>

using namespace spike;

static void header() {
    std::printf("\n%-15s %-9s %8s %8s %8s  %6s %7s %6s %6s %9s %8s\n",
                "part", "kernel", "build", "fillet", "tess", "faces", "tris",
                "fillet", "solid", "volume", "err");
    std::printf("%s\n", std::string(104, '-').c_str());
}

static void row(const char* part, const char* kernel, const Result& r, double refVolume) {
    char err[16] = "  -  ";
    if (refVolume > 0.0 && r.volumeMm3 > 0.0)
        std::snprintf(err, sizeof err, "%+7.3f%%", (r.volumeMm3 - refVolume) / refVolume * 100.0);

    std::printf("%-15s %-9s %7.1fms %7.1fms %7.1fms  %6d %7d %6s %6s %9.1f %8s\n",
                part, kernel, r.buildMs, r.filletMs, r.tessMs, r.faces, r.triangles,
                !r.built ? "-" : (r.filletOk ? "ok" : "NO"),
                !r.built ? "-" : (r.valid ? "yes" : "NO"),
                r.volumeMm3, err);

    if (!r.built)         std::printf("%27s build refused: %s\n", "", r.buildNote.c_str());
    if (r.built && !r.filletOk)
                          std::printf("%27s fillet refused (%d edges asked): %s\n", "",
                                      r.filletEdges, r.filletNote.c_str());
    if (r.built && !r.valid)
                          std::printf("%27s not a solid: %s\n", "", r.validNote.c_str());
}

int main(int argc, char** argv) {
    std::vector<std::string> only;
    for (int i = 1; i < argc; ++i) only.push_back(argv[i]);

    header();
    for (const PartSpec& p : catalogue()) {
        if (!only.empty()) {
            bool wanted = false;
            for (const std::string& n : only) wanted = wanted || n == p.name;
            if (!wanted) continue;
        }

        const Result occt = runOcct(p);
        row(p.name.c_str(), "occt", occt, occt.volumeMm3);
        for (int seg : {32, 64}) {
            const Result m = runMesh(p, seg);
            row("", (seg == 32 ? "mesh/32" : "mesh/64"), m, occt.volumeMm3);
        }
        std::printf("%-15s %s (fillet r=%.2f on %d edges, chord dev %.4f mm)\n\n",
                    "", p.what.c_str(), p.filletRadius, occt.filletEdges, occt.deviationMm);
    }
    return 0;
}
