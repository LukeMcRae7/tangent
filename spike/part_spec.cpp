#include "part_spec.h"

#include <cmath>

namespace spike {

std::vector<PartSpec> catalogue() {
    std::vector<PartSpec> parts;

    // 1. The bracket the plan names: an L-section with bosses, bores through
    //    them, and two fixing holes in the base. Nothing exotic; it is here
    //    because it is the shape most printed parts actually are.
    {
        PartSpec p;
        p.name = "bracket";
        p.what = "L-section, 2 bosses bored through, 2 fixing holes";
        p.plateW = 80; p.plateD = 60; p.plateH = 8;
        p.wallH = 40;  p.wallT = 8;
        p.bosses = {{-20, 15, 16, 10}, {20, 15, 16, 10}};
        p.bores  = {{-20, 15, 8}, {20, 15, 8}, {-30, -15, 6}, {30, -15, 6}};
        p.target = FilletTarget::BoreRims;
        p.filletRadius = 1.5;
        parts.push_back(p);
    }

    // 2. The bolt circle. Eight holes on a pitch circle and one in the middle,
    //    filleted in a single operation -- the exit criterion that says the
    //    fillet works on a real fixing pattern rather than on one hole.
    {
        PartSpec p;
        p.name = "bolt-circle";
        p.what = "100mm plate, 8 holes on a 70mm PCD + 20mm centre bore";
        p.plateW = 100; p.plateD = 100; p.plateH = 10;
        for (int i = 0; i < 8; ++i) {
            const double a = 3.14159265358979 * 2.0 * i / 8.0;
            p.bores.push_back({35.0 * std::cos(a), 35.0 * std::sin(a), 6.6});
        }
        p.bores.push_back({0, 0, 20});
        p.target = FilletTarget::BoreRims;
        p.filletRadius = 1.5;   // wider than a 32-segment facet on a 6.6 hole
        parts.push_back(p);
    }

    // 3 and 4. The counts the plan calls out as failing. Kept as two parts
    //    rather than one because the interesting question is how the cost and
    //    the failure rate move with hole count, not whether one number works.
    for (int n : {3, 4}) {
        PartSpec p;
        p.name = (n == 3 ? "nine-bores" : "sixteen-bores");
        p.what = (n == 3 ? "60mm plate, 3x3 grid of 8mm holes"
                         : "80mm plate, 4x4 grid of 8mm holes");
        p.plateW = p.plateD = (n == 3 ? 60 : 80);
        p.plateH = 8;
        const double pitch = 18.0;
        const double first = -pitch * (n - 1) / 2.0;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                p.bores.push_back({first + pitch * i, first + pitch * j, 8});
        p.target = FilletTarget::BoreRims;
        p.filletRadius = 1.0;
        parts.push_back(p);
    }

    // 5. Slots rather than holes. This is the case that produced 176 faces on
    //    the mesh kernel before the coplanar rebuild landed, so it is the one
    //    that says most about face count -- the thing the user sees as stray
    //    lines across a flat surface.
    {
        PartSpec p;
        p.name = "slotted-plate";
        p.what = "80mm plate, 4 through slots, outer top edges filleted";
        p.plateW = 80; p.plateD = 80; p.plateH = 10;
        p.pockets = {{-20, 0, 10, 40, 10}, {20, 0, 10, 40, 10},
                     {0, -20, 40, 10, 10}, {0, 20, 40, 10, 10}};
        p.target = FilletTarget::TopOuterEdges;
        p.filletRadius = 3.0;
        parts.push_back(p);
    }

    return parts;
}

} // namespace spike
