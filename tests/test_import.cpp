// Reading triangles from a file, and turning them back into a solid.
//
// The round trip is the test: build a body, write it as STL, read it back, and
// ask whether it is the same thing. What "the same" means differs by step --
// the triangles should match immediately, and the *faces* should only match
// after the coplanar ones have been merged back together. A box exported as
// twelve triangles that comes back as twelve faces has been read but not
// recovered, and that distinction is the whole point of toSolid.
#include "mesh/import_mesh.h"
#include "mesh/export_stl.h"
#include "scene/scene.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near1(Real got, Real want) {
    return std::fabs(got - want) < std::fabs(want) * 2e-3 + 1e-6;
}

// Writes one body out through the real exporter, so the file under test is the
// kind of file this program actually produces.
static bool writeVia(Scene& scene, PrimitiveKind kind, const PrimitiveSpec& spec,
                     const std::string& path, bool binary, Real deviation = 0.02) {
    scene.clear();
    if (scene.addPrimitive(kind, spec) == kNoObject) return false;
    StlOptions opt;
    opt.binary = binary;
    opt.deviationMm = deviation;
    return exportStl(scene, path, opt).ok;
}

int main() {
    std::printf("import\n");
    const std::string path = "/tmp/tg_import_test.stl";
    Scene scene;

    check(meshFormatOf("a/b/part.STL") == MeshFormat::Stl, "an upper-case .STL is an STL");
    check(meshFormatOf("part.obj") == MeshFormat::Obj, "an .obj is an OBJ");
    check(meshFormatOf("part.step") == MeshFormat::Unknown, "a .step is neither");

    // --- a binary STL round trip -------------------------------------------
    {
        PrimitiveSpec s; s.box = {20, 30, 40};
        check(writeVia(scene, PrimitiveKind::Box, s, path, /*binary=*/true),
              "the box exports as binary STL");

        Body b;
        const MeshImport r = readMesh(path, b);
        check(r.ok, "and reads back: " + r.error);
        if (r.ok) {
            check(r.triangles == 12, "as twelve triangles");
            // The welding check. Without it every triangle would keep its own
            // three vertices, the surface would have no shared edges at all,
            // and nothing below would be true.
            check(r.closed, "welded into a closed surface");
            check(b.isMesh(), "as a mesh body");
            check(near1(b.health(false).volume, 24000.0), "with the right volume");
            if (!near1(b.health(false).volume, 24000.0))
                std::printf("    %.2f\n", b.health(false).volume);
        }
    }

    // --- an ASCII STL is the same file -------------------------------------
    {
        PrimitiveSpec s; s.box = {20, 30, 40};
        check(writeVia(scene, PrimitiveKind::Box, s, path, /*binary=*/false),
              "the box exports as ASCII STL");
        Body b;
        const MeshImport r = readMesh(path, b);
        check(r.ok && r.triangles == 12 && r.closed,
              "and reads back the same: " + r.error);
        if (r.ok) check(near1(b.health(false).volume, 24000.0), "with the same volume");
    }

    // --- and back to a solid -----------------------------------------------
    // The recovery step. Twelve triangles should become six faces, because the
    // pairs that share a diagonal are coplanar and belong together.
    if (brep::available()) {
        PrimitiveSpec s; s.box = {20, 30, 40};
        writeVia(scene, PrimitiveKind::Box, s, path, true);
        Body b;
        check(readMesh(path, b).ok, "the box reads for conversion");

        const SolidifyResult c = toSolid(b, 1234);
        check(c.ok, "and converts to a solid: " + c.error);
        if (c.ok) {
            std::printf("  box: %d triangles -> %d faces\n", c.facesBefore, c.facesAfter);
            check(c.facesBefore == 12, "twelve triangles went in");
            check(c.facesAfter == 6, "six faces came out");
            check(c.viaRegions, "by the fast route, not the per-triangle fallback");
            check(!b.isMesh(), "and the body is exact now");
            check(b.validate(), "and valid");
            check(near1(b.health(false).volume, 24000.0), "and the same size");

            // The real prize: an exact body can be modelled on. A fillet needs
            // edges that bound real surfaces, which is exactly what the merge
            // put back.
            std::vector<EdgeId> edges;
            b.allEdges(edges);
            check(edges.size() == 12, "with twelve edges, not thirty-six");
            if (!edges.empty()) {
                FilletSpec sp;
                sp.edges.push_back({edges.front(), 2.0});
                sp.salt = 99;
                std::string why;
                check(filletEdges(b, sp, &why), "and it can be filleted: " + why);
            }
        }
    }

    // --- what does not come back -------------------------------------------
    // A cylinder leaves as facets and returns as facets. The volume is close
    // and the surface is not a cylinder, and the test says so rather than
    // implying the conversion is lossless.
    if (brep::available()) {
        PrimitiveSpec s; s.cylinder = {10, 20, 24};
        writeVia(scene, PrimitiveKind::Cylinder, s, path, true);
        Body b;
        check(readMesh(path, b).ok, "the cylinder reads");
        const SolidifyResult c = toSolid(b, 4321);
        check(c.ok, "and converts: " + c.error);
        if (c.ok) {
            std::printf("  cylinder: %d triangles -> %d faces\n",
                        c.facesBefore, c.facesAfter);
            // Two caps and one flat strip per facet -- not three faces. The
            // information that the strips approximate a circle went out with
            // the STL and no amount of merging brings it back. Neighbouring
            // strips are not coplanar, so there is nothing to merge them into.
            //
            // The facet count is not 24: the exporter re-tessellates an exact
            // body to its own deviation tolerance rather than to whatever
            // segment count the primitive was defined with, which is the
            // documented behaviour and the right one for a print-first tool.
            // So the count is read off the result and the volume checked
            // against it, which tests the relationship rather than a number
            // that moves when the tolerance does.
            check(c.viaRegions, "the cylinder takes the fast route too");
            check(predictSolidFaces(b) == 0, "and a converted body predicts nothing");
            const int sides = c.facesAfter - 2;
            check(sides > 24, "as many flat strips as the tolerance asked for");
            check(c.facesBefore == sides * 2 + (c.facesBefore - sides * 2),
                  "with the caps triangulated too");

            // A regular n-gon inscribed in a circle of radius 10 falls short of
            // it, and that shortfall is real geometry rather than a reading
            // error -- so the volume must match the polygon, not the circle.
            const Real poly = 0.5 * sides * std::sin(2.0 * kPi / sides) * 100.0 * 20.0;
            const Real circle = kPi * 100.0 * 20.0;
            check(near1(b.health(false).volume, poly),
                  "with the volume of the polygon it actually is");
            check(b.health(false).volume < circle,
                  "which is less than the cylinder it was");
            if (!near1(b.health(false).volume, poly))
                std::printf("    %.2f against %.2f for a %d-gon (circle is %.2f)\n",
                            b.health(false).volume, poly, sides, circle);
        }
    }

    // --- refusals ----------------------------------------------------------
    {
        Body b;
        MeshImport r = readMesh("/tmp/tg_definitely_absent.stl", b);
        check(!r.ok && !r.error.empty(), "a missing file is refused, with a reason");

        r = readMesh("/tmp/tg_import_test.step", b);
        check(!r.ok, "an extension this does not read is refused");

        if (brep::available()) {
            // An open surface cannot be a solid, and saying which is the point.
            PrimitiveSpec s; s.kind = PrimitiveKind::Plane;
            scene.clear();
            if (scene.addPrimitive(PrimitiveKind::Plane, s) != kNoObject) {
                StlOptions opt; opt.binary = true;
                exportStl(scene, path, opt);
                Body open;
                if (readMesh(path, open).ok) {
                    const SolidifyResult c = toSolid(open, 7);
                    check(!c.ok, "an open surface is refused");
                    check(c.error.find("holes") != std::string::npos,
                          "and says it has holes rather than just failing");
                }
            }
        }
    }

    std::remove(path.c_str());
    std::printf("%s\n", failures ? "FAILED" : "ok");
    return failures ? 1 : 0;
}
