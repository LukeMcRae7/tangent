// Tangent - Stage 0 spike: does a name survive a boolean and a fillet?
//
// The chain is the smallest one that is still a real history: plate, bore,
// fillet the rim, second bore. Every reference is made by name -- the rim to
// fillet is looked up, not searched for geometrically -- and the whole chain is
// then replayed with a changed base parameter, which is what a feature history
// has to survive and what the mesh kernel's naming was tested against in
// Stage 1.
#include "naming.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>

#include <cstdio>
#include <string>
#include <vector>

using namespace spike;

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    std::printf("  %-58s %s\n", what.c_str(), ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}

struct Run {
    TopoDS_Shape shape;
    NameMap names;
    Dropped dropped;
    int unnamed = 0;
    int rimEdges = 0;
    int topPieces = 0;
    bool filletOk = false;
};

// One replay of the chain. `plateH` and `boreDia` are the parameters a user
// would edit; everything else is fixed so that what changes between runs is
// only what is meant to.
Run buildChain(double plateH, double boreDia, bool verbose) {
    Run r;

    const TopoDS_Shape plate = BRepPrimAPI_MakeBox(gp_Pnt(-30, -30, 0), 60, 60, plateH).Shape();
    r.names.seedSolid(plate, "plate");
    if (verbose) std::printf("  plate seeded: %d faces, %zu named edges\n",
                             r.names.faceCount(), r.names.edgeNames().size());

    // --- bore 1 -------------------------------------------------------------
    const TopoDS_Shape tool1 =
        BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -1), gp_Dir(0, 0, 1)), boreDia / 2, plateH + 2).Shape();
    r.names.seedSolid(tool1, "bore1");
    BRepAlgoAPI_Cut cut1(plate, tool1);
    cut1.Build();
    r.names.update(cut1, plate, {tool1}, cut1.Shape(), "cut1", &r.dropped);
    r.shape = cut1.Shape();
    if (verbose) std::printf("  after the bore: %d faces named\n", r.names.faceCount());

    // --- fillet the rim, found by name and not by looking for a circle -------
    const Name rim = "bore1.wall|plate.top";
    const std::vector<TopoDS_Shape> rimEdges = r.names.findAll(rim);
    r.rimEdges = static_cast<int>(rimEdges.size());
    if (verbose) std::printf("  \"%s\" resolves to %d edge(s)\n", rim.c_str(), r.rimEdges);

    if (!rimEdges.empty()) {
        BRepFilletAPI_MakeFillet fil(r.shape);
        for (const TopoDS_Shape& e : rimEdges) fil.Add(1.5, TopoDS::Edge(e));
        fil.Build();
        if (fil.IsDone()) {
            const TopoDS_Shape before = r.shape;
            r.names.update(fil, before, {}, fil.Shape(), "fillet1", &r.dropped);
            r.shape = fil.Shape();
            r.filletOk = true;
        }
    }

    // --- bore 2, so the first bore's names have to survive a second edit -----
    const TopoDS_Shape tool2 =
        BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(20, 0, -1), gp_Dir(0, 0, 1)), 5, plateH + 2).Shape();
    r.names.seedSolid(tool2, "bore2");
    const TopoDS_Shape beforeCut2 = r.shape;
    BRepAlgoAPI_Cut cut2(beforeCut2, tool2);
    cut2.Build();
    r.names.update(cut2, beforeCut2, {tool2}, cut2.Shape(), "cut2", &r.dropped);
    r.shape = cut2.Shape();

    // --- a slot straight across, which splits the top face in two ----------
    // The case every naming scheme is judged on: after this, "plate.top" is two
    // faces, and a feature that referred to it must still find both.
    const TopoDS_Shape slot =
        BRepPrimAPI_MakeBox(gp_Pnt(-40, -4, plateH - 3), 80, 8, 5).Shape();
    r.names.seedSolid(slot, "slot");
    const TopoDS_Shape beforeSlot = r.shape;
    BRepAlgoAPI_Cut cut3(beforeSlot, slot);
    cut3.Build();
    r.names.update(cut3, beforeSlot, {slot}, cut3.Shape(), "slot1", &r.dropped);
    r.shape = cut3.Shape();
    r.topPieces = static_cast<int>(r.names.findAll("plate.top").size());
    if (verbose) std::printf("  after a slot across it, plate.top is %d face(s)\n", r.topPieces);

    for (const Name& n : r.names.faceNames())
        if (n.find(".unnamed") != std::string::npos) ++r.unnamed;
    return r;
}

} // namespace

int main() {
    std::printf("\n=== a name through a chain: plate, bore, fillet the rim, second bore ===\n");
    const Run a = buildChain(10.0, 12.0, true);

    std::printf("\n  faces (%d)\n", a.names.faceCount());
    for (const Name& n : a.names.faceNames()) std::printf("      %s\n", n.c_str());

    std::printf("\n=== what the mechanism has to guarantee ===\n");
    check(a.filletOk, "the fillet ran on an edge found by name alone");
    check(a.rimEdges > 0, "the rim resolved from \"bore1.wall|plate.top\"");
    check(a.dropped.fromBody.empty(), "no face of the body lost its name");
    for (const Name& n : a.dropped.fromBody) std::printf("      lost from body: %s\n", n.c_str());
    check(a.unnamed == 0, "every face in the result has a name from provenance");

    check(!a.names.findAll("plate.top").empty(), "the top face still answers to plate.top");
    check(!a.names.findAll("bore1.wall").empty(), "the first hole's wall still answers to bore1.wall");
    check(!a.names.findAll("bore2.wall").empty(), "the second hole's wall is named from its tool");
    bool filletNamed = false;
    for (const Name& n : a.names.faceNames())
        if (n.rfind("fillet1.from(", 0) == 0) filletNamed = true;
    check(filletNamed, "the fillet surface is named after the edge it came from");
    check(a.topPieces >= 2, "a face split by a later cut still answers to its own name");

    std::printf("\n  tool faces with no successor (expected: the caps outside the plate): %zu\n",
                a.dropped.fromTools.size());

    std::printf("\n=== replay ===\n");
    const Run b = buildChain(10.0, 12.0, false);
    const Run c = buildChain(14.0, 12.0, false);   // thicker plate
    const Run d = buildChain(10.0, 16.0, false);   // wider bore

    check(a.names.allNames() == b.names.allNames(), "replaying the chain gives exactly the same names");
    check(a.names.allNames() == c.names.allNames(), "changing the plate thickness changes no name");
    check(a.names.allNames() == d.names.allNames(), "changing the bore diameter changes no name");
    check(c.filletOk && d.filletOk, "the fillet still resolves its edge after a parameter change");

    std::printf("\n%s (%d failure%s)\n", failures ? "NOT SOUND" : "sound", failures,
                failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
