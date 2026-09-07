#include "naming.h"

#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>

#include <algorithm>
#include <cmath>

namespace spike {
namespace {

gp_Pnt centroid(const TopoDS_Shape& s) {
    GProp_GProps p;
    BRepGProp::SurfaceProperties(s, p);
    return p.CentreOfMass();
}

// A deterministic order for shapes that are otherwise indistinguishable -- the
// several pieces one face was split into, say. Position is a legitimate
// tie-break *within* one operation's output; it is never used to find a name
// again later, which is the thing that must not depend on geometry.
bool before(const TopoDS_Shape& a, const TopoDS_Shape& b) {
    const gp_Pnt pa = centroid(a), pb = centroid(b);
    if (std::fabs(pa.X() - pb.X()) > 1e-9) return pa.X() < pb.X();
    if (std::fabs(pa.Y() - pb.Y()) > 1e-9) return pa.Y() < pb.Y();
    return pa.Z() < pb.Z();
}

std::string axisRole(const gp_Dir& d) {
    const double x = d.X(), y = d.Y(), z = d.Z();
    if (std::fabs(z) > 0.9) return z > 0 ? "top" : "bottom";
    if (std::fabs(x) > 0.9) return x > 0 ? "east" : "west";
    if (std::fabs(y) > 0.9) return y > 0 ? "north" : "south";
    return "angled";
}

std::string faceRole(const TopoDS_Face& f) {
    BRepAdaptor_Surface s(f);
    switch (s.GetType()) {
        case GeomAbs_Plane: {
            gp_Dir n = s.Plane().Axis().Direction();
            if (f.Orientation() == TopAbs_REVERSED) n.Reverse();
            return axisRole(n);
        }
        case GeomAbs_Cylinder: return "wall";
        case GeomAbs_Cone:     return "cone";
        case GeomAbs_Sphere:   return "dome";
        case GeomAbs_Torus:    return "ring";
        default:               return "surface";
    }
}

} // namespace

void NameMap::seedSolid(const TopoDS_Shape& shape, const std::string& prefix) {
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(shape, TopAbs_FACE, faces);

    // Group by role first, so two faces with the same role get a stable suffix
    // rather than whichever number OCCT's face order happens to imply.
    std::vector<std::pair<std::string, TopoDS_Face>> byRole;
    for (int i = 1; i <= faces.Extent(); ++i)
        byRole.push_back({faceRole(TopoDS::Face(faces(i))), TopoDS::Face(faces(i))});
    std::stable_sort(byRole.begin(), byRole.end(), [](const auto& a, const auto& b) {
        if (a.first != b.first) return a.first < b.first;
        return before(a.second, b.second);
    });

    std::string lastRole;
    int k = 0;
    for (auto& [role, f] : byRole) {
        if (role != lastRole) { lastRole = role; k = 0; } else { ++k; }
        const Name n = prefix + "." + role + (k ? "#" + std::to_string(k) : "");
        if (!index_.IsBound(f)) {
            entries_.push_back({n, f});
            index_.Bind(f, static_cast<int>(entries_.size()) - 1);
        }
    }
    if (current_.IsNull()) adopt(shape);
    else adopt(current_);   // seeding a tool must not change what the body is
}

void NameMap::adopt(const TopoDS_Shape& shape) {
    current_ = shape;
    edgeIndex_.Clear();
    edgeNames_.clear();

    TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeFaces);
    for (int i = 1; i <= edgeFaces.Extent(); ++i) {
        std::vector<Name> ns;
        for (TopTools_ListOfShape::Iterator it(edgeFaces(i)); it.More(); it.Next()) {
            const Name n = faceName(it.Value());
            if (!n.empty()) ns.push_back(n);
        }
        if (ns.size() < 2) continue;          // a seam or a free edge: nothing to name it with
        std::sort(ns.begin(), ns.end());
        edgeNames_.push_back(ns[0] + "|" + ns[1]);
        edgeIndex_.Bind(edgeFaces.FindKey(i), static_cast<int>(edgeNames_.size()) - 1);
    }
}

void NameMap::update(BRepBuilderAPI_MakeShape& op,
                     const TopoDS_Shape& body,
                     const std::vector<TopoDS_Shape>& tools,
                     const TopoDS_Shape& after,
                     const std::string& opPrefix,
                     Dropped* dropped) {
    // Everything the result actually contains, so "it came through untouched"
    // can be checked rather than assumed.
    TopTools_IndexedMapOfShape live;
    TopExp::MapShapes(after, TopAbs_FACE, live);

    TopTools_IndexedMapOfShape bodyFaces, toolFaces;
    TopExp::MapShapes(body, TopAbs_FACE, bodyFaces);
    for (const TopoDS_Shape& t : tools) TopExp::MapShapes(t, TopAbs_FACE, toolFaces);

    // Names an operation made. Asked before the carry, because the answers are
    // about the shapes as they were: a fillet surface is Generated from the
    // edge it rounds, and that edge is about to stop existing.
    std::vector<std::pair<Name, TopoDS_Shape>> made;
    auto collectGenerated = [&](const TopoDS_Shape& from, const Name& name) {
        if (name.empty()) return;
        for (TopTools_ListOfShape::Iterator it(op.Generated(from)); it.More(); it.Next())
            made.push_back({opPrefix + ".from(" + name + ")", it.Value()});
    };
    for (const Entry& e : entries_) collectGenerated(e.shape, e.name);
    for (const TopoDS_Shape& src : {body}) {
        TopTools_IndexedMapOfShape edges;
        TopExp::MapShapes(src, TopAbs_EDGE, edges);
        for (int i = 1; i <= edges.Extent(); ++i) collectGenerated(edges(i), edgeName(edges(i)));
    }

    std::vector<Entry> carried;
    carried.reserve(entries_.size());
    for (const Entry& e : entries_) {
        const bool isBody = bodyFaces.Contains(e.shape);
        const bool isTool = toolFaces.Contains(e.shape);
        if (!isBody && !isTool) continue;      // belonged to an earlier state; already superseded

        auto drop = [&] {
            if (!dropped) return;
            (isBody ? dropped->fromBody : dropped->fromTools).push_back(e.name);
        };

        if (op.IsDeleted(e.shape)) { drop(); continue; }

        std::vector<TopoDS_Shape> succ;
        for (TopTools_ListOfShape::Iterator it(op.Modified(e.shape)); it.More(); it.Next())
            if (live.Contains(it.Value())) succ.push_back(it.Value());

        if (succ.empty()) {
            if (live.Contains(e.shape)) carried.push_back({e.name, e.shape});
            else drop();
            continue;
        }
        if (succ.size() == 1) { carried.push_back({e.name, succ[0]}); continue; }

        // One face became several, and every piece keeps the parent's name.
        //
        // Not a suffix per piece: a feature that referred to the face has to go
        // on referring to all of it, which is the same decision already taken
        // on the mesh side -- selecting a bored face selects the whole face,
        // not the fragment under the cursor. A name therefore stands for a set,
        // and it is findAll, not find, that callers use.
        std::sort(succ.begin(), succ.end(), before);
        for (const TopoDS_Shape& piece : succ) carried.push_back({e.name, piece});
    }

    entries_ = std::move(carried);
    index_.Clear();
    for (size_t i = 0; i < entries_.size(); ++i) index_.Bind(entries_[i].shape, static_cast<int>(i));

    std::sort(made.begin(), made.end(), [](const auto& a, const auto& b) {
        if (a.first != b.first) return a.first < b.first;
        return before(a.second, b.second);
    });
    for (auto& [n, s] : made) {
        if (!live.Contains(s) || index_.IsBound(s)) continue;
        entries_.push_back({n, s});          // several faces from one source share its name
        index_.Bind(s, static_cast<int>(entries_.size()) - 1);
    }

    // Anything left is a hole in the mechanism, and the count of it is the
    // number that decides whether this approach works.
    std::vector<TopoDS_Shape> orphans;
    for (int i = 1; i <= live.Extent(); ++i)
        if (!index_.IsBound(live(i))) orphans.push_back(live(i));
    std::sort(orphans.begin(), orphans.end(), before);
    for (size_t i = 0; i < orphans.size(); ++i) {
        entries_.push_back({opPrefix + ".unnamed" + std::to_string(i), orphans[i]});
        index_.Bind(orphans[i], static_cast<int>(entries_.size()) - 1);
    }

    adopt(after);
}

Name NameMap::faceName(const TopoDS_Shape& s) const {
    return index_.IsBound(s) ? entries_[index_(s)].name : Name();
}

Name NameMap::edgeName(const TopoDS_Shape& e) const {
    return edgeIndex_.IsBound(e) ? edgeNames_[edgeIndex_(e)] : Name();
}

std::vector<TopoDS_Shape> NameMap::findAll(const Name& n) const {
    std::vector<TopoDS_Shape> out;
    for (const Entry& e : entries_) if (e.name == n) out.push_back(e.shape);
    if (!out.empty()) return out;

    TopTools_IndexedMapOfShape edges;
    TopExp::MapShapes(current_, TopAbs_EDGE, edges);
    for (int i = 1; i <= edges.Extent(); ++i)
        if (edgeName(edges(i)) == n) out.push_back(edges(i));
    return out;
}

std::vector<Name> NameMap::faceNames() const {
    std::vector<Name> out;
    for (const Entry& e : entries_) out.push_back(e.name);
    std::sort(out.begin(), out.end());
    return out;
}

std::vector<Name> NameMap::edgeNames() const {
    std::vector<Name> out = edgeNames_;
    std::sort(out.begin(), out.end());
    return out;
}

std::vector<Name> NameMap::allNames() const {
    std::vector<Name> out = faceNames();
    for (const Name& n : edgeNames_) out.push_back(n);
    std::sort(out.begin(), out.end());
    return out;
}

int NameMap::faceCount() const { return static_cast<int>(entries_.size()); }

} // namespace spike
