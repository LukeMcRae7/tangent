// Tangent - Stage 0 spike: persistent names on a B-rep, carried through
// operations by provenance rather than by position.
//
// This is the piece the whole migration hangs on. Today a face keeps its name
// because the name is derived from the vertices it is made of (element_id.h);
// on a B-rep there are no vertices to derive from, so the name has to be
// carried across each operation by asking the operation what became of what.
// OCCT answers that through three questions on every BRepBuilderAPI_MakeShape:
//
//   IsDeleted(S)   this shape is gone
//   Modified(S)    these shapes are what it became
//   Generated(S)   these shapes appeared because of it
//
// Two rules, and the second is what keeps the first small:
//
//   1. Faces are named, and their names are carried across each operation.
//   2. Edges are not carried. An edge is named by the two faces that meet at
//      it, derived on demand -- exactly the rule the mesh kernel already uses,
//      and for the same reason: it costs nothing, it is order-independent, and
//      an edge that survives keeps its name without the operation having to
//      know that it did. It also gives the right answer for edges that did not
//      exist before, which is most of the edges a boolean produces.
#pragma once

#include <BRepBuilderAPI_MakeShape.hxx>
#include <TopTools_DataMapOfShapeInteger.hxx>
#include <TopoDS_Shape.hxx>

#include <cstdint>
#include <string>
#include <vector>

namespace spike {

using Name = std::string;

// What an operation dropped. The two are different findings and must not be
// added together: a cutting tool's far end legitimately has no successor,
// while a face of the body losing its name is the mechanism leaking.
struct Dropped {
    std::vector<Name> fromBody;
    std::vector<Name> fromTools;
};

class NameMap {
public:
    // Names every face of a freshly built primitive by its role -- which way it
    // faces, whether it is a wall or a cap -- so that rebuilding the same
    // primitive with different parameters produces the same names. Naming by
    // face index would not survive a parameter change; this does.
    void seedSolid(const TopoDS_Shape& shape, const std::string& prefix);

    // Carries face names across one operation and adopts the shape it produced.
    void update(BRepBuilderAPI_MakeShape& op,
                const TopoDS_Shape& body,
                const std::vector<TopoDS_Shape>& tools,
                const TopoDS_Shape& after,
                const std::string& opPrefix,
                Dropped* dropped = nullptr);

    // Every shape currently answering to a name. A name can stand for more than
    // one: a boolean routinely splits a cylindrical wall into two halves and a
    // rim into two arcs, and both halves are still "the wall". Resolving to a
    // set is what ElementRefs already does on the mesh side.
    std::vector<TopoDS_Shape> findAll(const Name& n) const;

    Name faceName(const TopoDS_Shape& face) const;
    Name edgeName(const TopoDS_Shape& edge) const;   // derived from its two faces

    std::vector<Name> faceNames() const;
    std::vector<Name> edgeNames() const;
    std::vector<Name> allNames() const;
    int faceCount() const;

private:
    struct Entry { Name name; TopoDS_Shape shape; };

    void adopt(const TopoDS_Shape& shape);   // recomputes the edge -> faces index

    std::vector<Entry> entries_;                     // faces only
    TopTools_DataMapOfShapeInteger index_;           // face -> entries_ slot
    TopoDS_Shape current_;
    TopTools_DataMapOfShapeInteger edgeIndex_;       // edge -> edgeNames_ slot
    std::vector<Name> edgeNames_;
};

} // namespace spike
