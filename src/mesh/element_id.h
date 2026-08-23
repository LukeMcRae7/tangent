// Tangent - stable names for mesh elements.
//
// A feature history is only parametric if a step can still find what it acts
// on after the steps before it change. Naming faces and edges by index cannot
// do that: indices are positions in an array that every operation rebuilds. The
// failure is not loud, either. Raise a cylinder's segment count under a rim
// fillet and the indices the fillet stored still resolve -- to different edges.
// The model comes back valid, watertight, and wrong.
//
// So elements carry a name instead. Names are 64-bit, and the rules are:
//
//   * An element that survives an operation keeps its name.
//   * An element an operation creates is named from the operation and from the
//     names of what it was made out of -- never from a counter, and never from
//     where the operation sits in the chain.
//
// The second rule is what makes re-evaluation reproducible: running the same
// chain twice produces the same names, so a stored reference still resolves.
// It is also what makes reordering possible, since a feature's salt is its own
// identity rather than its position.
//
// Names are derived rather than allocated, so two elements can in principle
// collide. At 64 bits, a model would need on the order of a billion elements
// before that became as likely as a cosmic ray flipping the answer, and
// `Mesh::validate` checks for it regardless.
#pragma once

#include <cstdint>

namespace tg {

using ElementId = uint64_t;

// Not a name. Means "this element has never been named", which is what a mesh
// built from raw geometry -- an import, a boolean's output before it is
// labelled -- starts out as.
inline constexpr ElementId kNoId = 0;

// Which part an element plays in the operation that made it. Kept explicit so
// that two elements built from the same source, in the same operation, but
// serving different purposes cannot collide.
enum class IdRole : uint32_t {
    Vertex = 1,
    Edge,
    Face,
    // Primitives
    Cap,
    Side,
    Rim,
    Seam,
    Apex,
    // Operations
    Wall,
    Top,
    Cross,
    Ring,
    Strip,
    Patch,
    Split,
};

inline ElementId mix64(ElementId x) {
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}

// `salt` identifies the operation -- a feature's own identity, not its place in
// the chain. `a` and `b` are the names of what this element was made from, and
// `k` separates a run of siblings such as the points along one fillet section.
inline ElementId nameId(ElementId salt, IdRole role, ElementId a,
                        ElementId b = 0, uint64_t k = 0) {
    ElementId h = mix64(salt + 0x243F6A8885A308D3ull);
    h = mix64(h ^ (static_cast<ElementId>(role) * 0x100000001B3ull));
    h = mix64(h ^ a);
    h = mix64(h ^ (b + 0x9E3779B9ull));
    h = mix64(h ^ (k + 0x7F4A7C15ull));
    return h == kNoId ? 1 : h;
}

// An edge is named from its two endpoints where nothing better is available.
// Order must not matter: the two half-edges have to agree.
inline ElementId edgeNameFrom(ElementId salt, ElementId a, ElementId b) {
    return nameId(salt, IdRole::Edge, a < b ? a : b, a < b ? b : a);
}

} // namespace tg
