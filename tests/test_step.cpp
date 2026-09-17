// STEP in and out.
//
// The claim this file has to justify is that a STEP round trip is an exchange
// of *surfaces* and not of triangles. The check for that is not the volume --
// a fine enough tessellation matches the volume too -- it is that a cylinder
// comes back as a cylinder: the same small number of faces, and a curved one
// among them. A tessellated import would arrive with sixty-four flat sides and
// would still measure correctly.
#include "geom/brep.h"
#include "geom/operations.h"
#include "temp_path.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near1(Real got, Real want) {
    return std::fabs(got - want) < std::fabs(want) * 1e-3 + 1e-6;
}

static Body prim(PrimitiveKind k, PrimitiveSpec spec) {
    spec.kind = k;
    Body b;
    makePrimitive(spec, b, Backend::Brep);
    return b;
}

static bool writeOne(const Body& b, const std::string& path, std::string* why) {
    std::vector<const BrepShape*> shapes{&b.brep()};
    return brep::writeStep(shapes, path, why);
}

int main() {
    if (!brep::available()) {
        std::printf("exact kernel not built; STEP needs one\n");
        return 0;
    }
    std::printf("step\n");
    const std::string path = tempPath("step.stp");

    // --- a box goes out and comes back a box -------------------------------
    {
        PrimitiveSpec s; s.box = {20, 30, 40};
        Body b = prim(PrimitiveKind::Box, s);
        const Real vol = b.health(false).volume;

        std::string why;
        check(writeOne(b, path, &why), "the box writes: " + why);

        std::vector<BrepRef> back;
        check(brep::readStep(path, 7, back, &why), "and reads back: " + why);
        check(back.size() == 1, "as one solid");
        if (back.size() == 1) {
            Body r(std::move(back[0]));
            check(r.faceCount() == 6, "with six faces");
            check(near1(r.health(false).volume, vol), "and the same volume");
            check(r.validate(), "and it is a valid solid");
        }
    }

    // --- the one that matters: a cylinder stays curved ---------------------
    {
        PrimitiveSpec s; s.cylinder = {10, 25, 64};
        Body b = prim(PrimitiveKind::Cylinder, s);
        const Real vol = b.health(false).volume;

        std::string why;
        check(writeOne(b, path, &why), "the cylinder writes: " + why);

        std::vector<BrepRef> back;
        check(brep::readStep(path, 8, back, &why), "and reads back: " + why);
        if (back.size() == 1) {
            Body r(std::move(back[0]));
            // Three faces: two flat caps and one cylindrical side. A file that
            // had been tessellated on the way through would have sixty-six.
            check(r.faceCount() == 3, "as three faces, not sixty-six");
            if (r.faceCount() != 3) std::printf("    %d faces\n", r.faceCount());
            check(near1(r.health(false).volume, vol), "and the same volume");

            // And the volume is the true one rather than a polygon's. A 64-gon
            // inscribed in a circle of radius 10 encloses about 0.2% less.
            const Real exact = kPi * 100.0 * 25.0;
            check(near1(r.health(false).volume, exact),
                  "which is the volume of a circle, not of a 64-gon");
            if (!near1(r.health(false).volume, exact))
                std::printf("    %.2f against %.2f exact\n",
                            r.health(false).volume, exact);
        }
    }

    // --- a modelled body, not just a primitive -----------------------------
    // A filleted, shelled box: the case where the surfaces are worth keeping.
    {
        PrimitiveSpec s; s.box = {40, 40, 20};
        Body b = prim(PrimitiveKind::Box, s);
        std::vector<FaceId> fs;
        b.allFaces(fs);
        FaceId top = fs.front();
        Real best = -1e30;
        for (FaceId f : fs) {
            const Real d = dot(b.faceNormal(f), Vec3{0, 0, 1});
            if (d > best) { best = d; top = f; }
        }
        std::string why;
        check(shellBody(b, {top}, 2.0, 11, &why), "the box shells: " + why);
        const Real vol = b.health(false).volume;
        const int faces = b.faceCount();

        check(writeOne(b, path, &why), "the shelled box writes: " + why);
        std::vector<BrepRef> back;
        check(brep::readStep(path, 9, back, &why), "and reads back: " + why);
        if (back.size() == 1) {
            Body r(std::move(back[0]));
            check(r.faceCount() == faces, "with the same faces");
            check(near1(r.health(false).volume, vol), "and the same volume");
            check(r.validate(), "and it is still a solid");
            if (r.faceCount() != faces)
                std::printf("    %d faces against %d\n", r.faceCount(), faces);
        }
    }

    // --- two bodies in one file --------------------------------------------
    {
        PrimitiveSpec a; a.box = {10, 10, 10};
        PrimitiveSpec c; c.cylinder = {5, 10, 32};
        Body b1 = prim(PrimitiveKind::Box, a);
        Body b2 = prim(PrimitiveKind::Cylinder, c);
        b2.transform(translate({30, 0, 0}));

        std::vector<const BrepShape*> shapes{&b1.brep(), &b2.brep()};
        std::string why;
        check(brep::writeStep(shapes, path, &why), "two bodies write: " + why);

        std::vector<BrepRef> back;
        check(brep::readStep(path, 10, back, &why), "and read back: " + why);
        check(back.size() == 2, "as two solids");
        if (back.size() == 2) {
            Real total = 0;
            for (BrepRef& r : back) { Body t(std::move(r)); total += t.health(false).volume; }
            check(near1(total, 1000.0 + kPi * 25.0 * 10.0), "with both volumes");
        }
    }

    // --- names are minted, and are the same on a second read ---------------
    // The file cannot say what Tangent called anything, so the names come from
    // the faces. Two reads of the same file must agree, or a feature written
    // against an imported body would not survive reopening the project.
    {
        PrimitiveSpec s; s.cylinder = {8, 12, 32};
        Body b = prim(PrimitiveKind::Cylinder, s);
        std::string why;
        check(writeOne(b, path, &why), "written for the naming check: " + why);

        std::vector<BrepRef> one, two;
        brep::readStep(path, 42, one, &why);
        brep::readStep(path, 42, two, &why);
        check(one.size() == 1 && two.size() == 1, "both reads give one solid");
        if (one.size() == 1 && two.size() == 1) {
            Body a(std::move(one[0])), c(std::move(two[0]));
            std::vector<FaceId> fa, fc;
            a.allFaces(fa); c.allFaces(fc);
            bool same = fa.size() == fc.size();
            for (size_t i = 0; same && i < fa.size(); ++i)
                same = a.faceName(fa[i]) == c.faceName(fc[i]);
            check(same, "and name their faces identically");
        }
    }

    // --- refusals ----------------------------------------------------------
    {
        std::string why;
        std::vector<BrepRef> back;
        check(!brep::readStep(tempPath("definitely_not_here.stp"), 1, back, &why),
              "a missing file is refused");
        check(!why.empty(), "and says why");

        check(!brep::writeStep({}, path, &why), "writing nothing is refused");
    }

    std::remove(path.c_str());
    std::printf("%s\n", failures ? "FAILED" : "ok");
    return failures ? 1 : 0;
}
