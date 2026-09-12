// Tangent - the toolbar icons, built by the kernel that does the operations.
//
// No icon set has a fillet in it. Every CAD package draws its own, because a
// fillet icon is a cube with one edge rounded and nobody making a general icon
// set has ever needed one.
//
// This project has a kernel that rounds edges and a renderer that draws solids,
// so the icons are made rather than drawn: each one is a real body, built by
// the real operation, rendered from the angle every CAD icon set uses. The
// fillet icon is a fillet. The shell icon is a shelled box. They cannot drift
// from what the operations do, because they are what the operations did.
//
// The part the operation touched is picked out in the accent -- which is what
// makes them readable at a glance, and it comes for free: our operations name
// what they create, so the faces to colour can be asked for rather than
// guessed at.
#include "icon_raster.h"

#include "geom/operations.h"

#include <cstdio>
#include <functional>
#include <string>
#include <vector>

using namespace tg;
using namespace tg::icons;

namespace {

Body primitive(PrimitiveKind kind, const PrimitiveSpec& spec) {
    Body b;
    makePrimitive(spec, b, Backend::Brep);
    (void)kind;
    return b;
}

PrimitiveSpec boxSpec(Real w, Real d, Real h) {
    PrimitiveSpec s;
    s.kind = PrimitiveKind::Box;
    s.box = {w, d, h};
    return s;
}

PrimitiveSpec cylSpec(Real r, Real h) {
    PrimitiveSpec s;
    s.kind = PrimitiveKind::Cylinder;
    s.cylinder.radius = r;
    s.cylinder.height = h;
    return s;
}

// Faces whose names this operation created, so the accent lands on exactly what
// the operation made.
std::vector<FaceId> facesNotIn(const Body& after, const Body& before) {
    std::vector<FaceId> fresh;
    std::vector<FaceId> all;
    after.allFaces(all);
    for (FaceId f : all)
        if (before.findFace(after.faceName(f)) == kInvalid) fresh.push_back(f);
    return fresh;
}

struct Icon {
    std::string name;
    Body body;
    std::vector<FaceId> accent;
    Vec3 eye{1.0, -1.25, 0.85};
};

std::vector<Icon> build() {
    std::vector<Icon> icons;
    std::string why;

    // ---- Primitives: themselves, with no accent. They are not operations on
    // something, they are the something.
    icons.push_back({"box", primitive(PrimitiveKind::Box, boxSpec(20, 20, 20)), {}, {}});
    icons.push_back({"cylinder", primitive(PrimitiveKind::Cylinder, cylSpec(10, 22)), {}, {}});
    {
        PrimitiveSpec s;
        s.kind = PrimitiveKind::Sphere;
        s.sphere.radius = 11;
        icons.push_back({"sphere", primitive(PrimitiveKind::Sphere, s), {}, {}});
    }
    {
        PrimitiveSpec s;
        s.kind = PrimitiveKind::Cone;
        s.cone.bottomRadius = 11;
        s.cone.topRadius = 0;
        s.cone.height = 22;
        icons.push_back({"cone", primitive(PrimitiveKind::Cone, s), {}, {}});
    }
    {
        PrimitiveSpec s;
        s.kind = PrimitiveKind::Torus;
        s.torus.majorRadius = 9;
        s.torus.minorRadius = 3.4;
        icons.push_back({"torus", primitive(PrimitiveKind::Torus, s), {}, {}});
    }

    // ---- Fillet: a cube with one edge rounded, and that round face in accent.
    {
        Body cube = primitive(PrimitiveKind::Box, boxSpec(20, 20, 20));
        const Body before = cube;
        EdgeId pick = kInvalid;
        std::vector<EdgeId> edges;
        cube.allEdges(edges);
        for (EdgeId e : edges) {
            Vec3 a, b;
            cube.edgePositions(e, a, b);
            if (std::fabs(a.z - 10) < 1e-6 && std::fabs(b.z - 10) < 1e-6 &&
                std::fabs(a.y + 10) < 1e-6 && std::fabs(b.y + 10) < 1e-6)
                pick = e;
        }
        FilletSpec spec;
        spec.salt = 1;
        spec.segments = 4;
        spec.edges.push_back({pick, 6.0});
        if (filletEdges(cube, spec, &why))
            icons.push_back({"fillet", cube, facesNotIn(cube, before), {}});
        else
            std::fprintf(stderr, "fillet icon: %s\n", why.c_str());
    }

    // ---- Chamfer: the same edge, cut flat. One segment is a chamfer.
    {
        Body cube = primitive(PrimitiveKind::Box, boxSpec(20, 20, 20));
        const Body before = cube;
        EdgeId pick = kInvalid;
        std::vector<EdgeId> edges;
        cube.allEdges(edges);
        for (EdgeId e : edges) {
            Vec3 a, b;
            cube.edgePositions(e, a, b);
            if (std::fabs(a.z - 10) < 1e-6 && std::fabs(b.z - 10) < 1e-6 &&
                std::fabs(a.y + 10) < 1e-6 && std::fabs(b.y + 10) < 1e-6)
                pick = e;
        }
        FilletSpec spec;
        spec.salt = 2;
        spec.segments = 1;
        spec.edges.push_back({pick, 6.0});
        if (filletEdges(cube, spec, &why))
            icons.push_back({"chamfer", cube, facesNotIn(cube, before), {}});
    }

    // ---- Shell: hollowed with the top open, seen from above so the cavity is
    // the thing you look into.
    {
        Body cube = primitive(PrimitiveKind::Box, boxSpec(20, 20, 16));
        const Body before = cube;
        FaceId top = kNoFace;
        std::vector<FaceId> all;
        cube.allFaces(all);
        for (FaceId f : all)
            if (dot(cube.faceNormal(f), Vec3{0, 0, 1}) > 0.99) top = f;
        if (shellBody(cube, {top}, 2.5, 3, &why)) {
            Icon icon{"shell", cube, facesNotIn(cube, before), Vec3{0.85, -1.0, 1.15}};
            icons.push_back(icon);
        } else {
            std::fprintf(stderr, "shell icon: %s\n", why.c_str());
        }
    }

    // ---- Extrude: a boss pushed out of the top face, the boss in accent.
    {
        Body plate = primitive(PrimitiveKind::Box, boxSpec(22, 22, 7));
        const Body before = plate;
        FaceId top = kNoFace;
        std::vector<FaceId> all;
        plate.allFaces(all);
        for (FaceId f : all)
            if (dot(plate.faceNormal(f), Vec3{0, 0, 1}) > 0.99) top = f;

        // A smaller face to push, so the icon reads as "this part came up".
        std::vector<FaceId> inner;
        if (insetFaces(plate, {top}, 5.5, &inner, 4, &why) && !inner.empty()) {
            const Body beforePush = plate;
            if (extrudeFaces(plate, inner, 9.0, nullptr, 5, ExtrudeOp::Join, &why)) {
                std::vector<FaceId> accent = facesNotIn(plate, beforePush);
                for (FaceId f : facesNotIn(plate, before)) accent.push_back(f);
                icons.push_back({"extrude", plate, accent, {}});
            } else {
                std::fprintf(stderr, "extrude icon: %s\n", why.c_str());
            }
        } else {
            std::fprintf(stderr, "extrude icon inset: %s\n", why.c_str());
        }
    }

    // ---- Inset: the same plate, split but not moved. The inner face accented.
    {
        Body plate = primitive(PrimitiveKind::Box, boxSpec(22, 22, 8));
        FaceId top = kNoFace;
        std::vector<FaceId> all;
        plate.allFaces(all);
        for (FaceId f : all)
            if (dot(plate.faceNormal(f), Vec3{0, 0, 1}) > 0.99) top = f;
        std::vector<FaceId> inner;
        if (insetFaces(plate, {top}, 5.0, &inner, 6, &why))
            icons.push_back({"inset", plate, inner, Vec3{0.85, -1.0, 1.15}});
    }

    // ---- The booleans: the same two shapes every time, so the three icons
    // differ only by what the operation did to them.
    //
    // The cylinder is short and set into a corner rather than run through the
    // middle: a tall one sticking out of both ends is most of the picture and
    // the box behind it cannot be read, which is what the first attempt looked
    // like. Sunk into a corner, all three results have the box's silhouette in
    // them and the difference between them is the thing being shown.
    {
        auto pair = [&](BooleanOp op, const char* name, bool accentAll) {
            Body a = primitive(PrimitiveKind::Box, boxSpec(18, 18, 14));
            Body b = primitive(PrimitiveKind::Cylinder, cylSpec(6.5, 20));
            b.transform(translate({6, -6, 2}));
            const Body before = a;
            Body out;
            if (!booleanOp(a, b, op, out, 7, false, &why)) {
                std::fprintf(stderr, "%s icon: %s\n", name, why.c_str());
                return;
            }
            // An intersection is only the overlap, so the whole result is the
            // answer and all of it is accented. A union and a difference are
            // mostly the original shape, so only what changed is.
            std::vector<FaceId> accent;
            if (accentAll) {
                out.allFaces(accent);
            } else {
                accent = facesNotIn(out, before);
            }
            icons.push_back({name, out, accent, {}});
        };
        pair(BooleanOp::Union, "union", false);
        pair(BooleanOp::Difference, "difference", false);
        pair(BooleanOp::Intersection, "intersection", true);
    }

    // Measure is deliberately not here. It is not an operation on a solid --
    // there is no body it produces -- and a plate with a hole in it says
    // "hole", not "measure". A ruler from the icon font is the honest glyph for
    // it, and the same goes for the rest of the chrome: open, save, undo, hide.
    // This file is for the operations that have a shape of their own.

    return icons;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string outDir = argc > 1 ? argv[1] : "assets/icons";
    const int size = argc > 2 ? std::atoi(argv[2]) : 64;

    if (!brep::available()) {
        std::fprintf(stderr, "icons are built by the exact kernel; configure with TANGENT_BREP=ON\n");
        return 1;
    }

    const std::vector<Icon> icons = build();
    Style style;

    int written = 0;
    for (const Icon& icon : icons) {
        Subject subject;
        subject.body = &icon.body;
        subject.accentFaces = icon.accent;
        if (lengthSq(icon.eye) > 1e-9) subject.eye = icon.eye;

        const Image img = render(subject, size, style);
        const std::string path = outDir + "/" + icon.name + ".png";
        if (writePng(img, path)) {
            std::printf("%-14s %d faces, %zu accented -> %s\n", icon.name.c_str(),
                        icon.body.faceCount(), icon.accent.size(), path.c_str());
            ++written;
        } else {
            std::fprintf(stderr, "could not write %s\n", path.c_str());
        }
    }
    std::printf("%d of %zu icons written at %dpx\n", written, icons.size(), size);
    return written == static_cast<int>(icons.size()) ? 0 : 1;
}
