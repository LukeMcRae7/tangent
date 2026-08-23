#include "mesh/primitives.h"

namespace tg {

const char* primitiveName(PrimitiveKind k) {
    switch (k) {
        case PrimitiveKind::Box:      return "Box";
        case PrimitiveKind::Cylinder: return "Cylinder";
        case PrimitiveKind::Sphere:   return "Sphere";
        case PrimitiveKind::Cone:     return "Cone";
        case PrimitiveKind::Torus:    return "Torus";
        case PrimitiveKind::Plane:    return "Plane";
        case PrimitiveKind::Custom:   return "Mesh";
    }
    return "Object";
}

namespace {

// Small accumulator so each generator reads as "emit vertices, emit faces"
// without repeating the parallel-array bookkeeping build() expects.
struct SoupBuilder {
    std::vector<Vec3>     positions;
    std::vector<uint32_t> faceSizes;
    std::vector<uint32_t> faceIndices;

    uint32_t vertex(Vec3 p) {
        positions.push_back(p);
        return static_cast<uint32_t>(positions.size() - 1);
    }
    void tri(uint32_t a, uint32_t b, uint32_t c) {
        faceSizes.push_back(3);
        faceIndices.insert(faceIndices.end(), {a, b, c});
    }
    void quad(uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
        faceSizes.push_back(4);
        faceIndices.insert(faceIndices.end(), {a, b, c, d});
    }
    void ngon(const std::vector<uint32_t>& loop) {
        faceSizes.push_back(static_cast<uint32_t>(loop.size()));
        faceIndices.insert(faceIndices.end(), loop.begin(), loop.end());
    }

    // Names the face just emitted after what it *is* rather than where it came
    // in the generation order. A cylinder's end cap is the end cap whether the
    // wall around it has sixteen facets or twenty-four, and a feature that
    // refers to it should survive that change.
    std::vector<std::pair<size_t, ElementId>> structural;
    void nameLastFace(IdRole role, uint64_t k = 0) {
        structural.emplace_back(faceSizes.size() - 1,
                                nameId(0xCA9E, role, static_cast<ElementId>(k)));
    }
    // Every primitive is named the same way, from the order it generates in.
    // That is stable under a parameter change that leaves the topology alone --
    // a 20mm box and a 30mm box name their corners identically -- which is the
    // case that matters, since it is what a user editing dimensions does.
    //
    // It cannot be stable when the topology itself changes: a cylinder taken
    // from 16 segments to 24 has eight rim edges that did not exist before, and
    // no naming scheme conjures a name for something that was not there. That
    // is what the face-boundary selector is for; see feature.h.
    //
    // The kind is salted in so that switching a primitive from one shape to
    // another does not hand the new shape the old one's names.
    // `topo` carries whatever decides how many elements there are -- segment
    // and ring counts. Folding it into the salt means a shape taken from
    // sixteen segments to twenty-four shares no vertex names with its old self,
    // so a stored reference to one of its edges fails to resolve instead of
    // resolving to whichever edge now happens to sit at that ordinal. Loud is
    // the whole point: silently filleting the wrong edge is the bug this file
    // exists to prevent.
    //
    // Faces named structurally are exempt, so a reference to "the top cap"
    // still lands, and a feature that selected a rim as that cap's boundary
    // re-selects the whole rim at its new segment count.
    bool commit(Mesh& out, PrimitiveKind kind, uint64_t topo = 0) const {
        const ElementId base = static_cast<ElementId>(kind) + 1;
        const ElementId salt = nameId(base, IdRole::Vertex, topo);

        Mesh::Names names;
        names.vertices.reserve(positions.size());
        for (size_t i = 0; i < positions.size(); ++i)
            names.vertices.push_back(nameId(salt, IdRole::Vertex, i));

        names.faces.assign(faceSizes.size(), kNoId);
        for (size_t f = 0; f < faceSizes.size(); ++f)
            names.faces[f] = nameId(salt, IdRole::Face, f);
        for (const auto& [at, id] : structural)
            if (at < names.faces.size()) names.faces[at] = nameId(base, IdRole::Face, id);

        return out.build(positions, faceSizes, faceIndices, &names);
    }
};

} // namespace

// ---------------------------------------------------------------------------
bool makeBox(Mesh& out, const BoxParams& p) {
    if (p.width <= 0 || p.depth <= 0 || p.height <= 0) return false;
    const Real x = p.width * 0.5f, y = p.depth * 0.5f, z = p.height * 0.5f;

    SoupBuilder s;
    s.vertex({-x, -y, -z}); s.vertex({ x, -y, -z});   // 0 1
    s.vertex({ x,  y, -z}); s.vertex({-x,  y, -z});   // 2 3
    s.vertex({-x, -y,  z}); s.vertex({ x, -y,  z});   // 4 5
    s.vertex({ x,  y,  z}); s.vertex({-x,  y,  z});   // 6 7

    s.quad(0, 3, 2, 1);   // -Z
    s.quad(4, 5, 6, 7);   // +Z
    s.quad(0, 1, 5, 4);   // -Y
    s.quad(1, 2, 6, 5);   // +X
    s.quad(2, 3, 7, 6);   // +Y
    s.quad(3, 0, 4, 7);   // -X
    return s.commit(out, PrimitiveKind::Box);
}

// ---------------------------------------------------------------------------
bool makeCylinder(Mesh& out, const CylinderParams& p) {
    if (p.radius <= 0 || p.height <= 0 || p.segments < 3) return false;
    const int n = p.segments;
    const Real hz = p.height * 0.5f;

    SoupBuilder s;
    for (int i = 0; i < n; ++i) {
        const Real a = kTwoPi * static_cast<Real>(i) / static_cast<Real>(n);
        s.vertex({p.radius * std::cos(a), p.radius * std::sin(a), -hz});
    }
    for (int i = 0; i < n; ++i) {
        const Real a = kTwoPi * static_cast<Real>(i) / static_cast<Real>(n);
        s.vertex({p.radius * std::cos(a), p.radius * std::sin(a),  hz});
    }

    for (int i = 0; i < n; ++i) {
        const uint32_t b0 = i, b1 = (i + 1) % n;
        s.quad(b0, b1, b1 + n, b0 + n);
    }

    std::vector<uint32_t> cap;
    for (int i = 0; i < n; ++i) cap.push_back(static_cast<uint32_t>(i + n));
    s.ngon(cap);                                       // +Z
    s.nameLastFace(IdRole::Cap, 1);
    cap.clear();
    for (int i = n - 1; i >= 0; --i) cap.push_back(static_cast<uint32_t>(i));
    s.ngon(cap);                                       // -Z
    s.nameLastFace(IdRole::Cap, 0);
    return s.commit(out, PrimitiveKind::Cylinder, static_cast<uint64_t>(n));
}

// ---------------------------------------------------------------------------
bool makeSphere(Mesh& out, const SphereParams& p) {
    if (p.radius <= 0 || p.segments < 3 || p.rings < 2) return false;
    const int n = p.segments, r = p.rings;

    SoupBuilder s;
    const uint32_t north = s.vertex({0, 0, p.radius});

    // Interior latitude bands only; the poles are single vertices.
    for (int j = 1; j < r; ++j) {
        const Real phi = kPi * static_cast<Real>(j) / static_cast<Real>(r);
        const Real rz  = p.radius * std::cos(phi);
        const Real rr  = p.radius * std::sin(phi);
        for (int i = 0; i < n; ++i) {
            const Real a = kTwoPi * static_cast<Real>(i) / static_cast<Real>(n);
            s.vertex({rr * std::cos(a), rr * std::sin(a), rz});
        }
    }
    const uint32_t south = s.vertex({0, 0, -p.radius});

    auto ring = [&](int j, int i) -> uint32_t {   // j in [1, r-1]
        return static_cast<uint32_t>(1 + (j - 1) * n + (i % n));
    };

    for (int i = 0; i < n; ++i) s.tri(north, ring(1, i), ring(1, i + 1));
    for (int j = 1; j < r - 1; ++j)
        for (int i = 0; i < n; ++i)
            s.quad(ring(j, i), ring(j + 1, i), ring(j + 1, i + 1), ring(j, i + 1));
    for (int i = 0; i < n; ++i) s.tri(south, ring(r - 1, i + 1), ring(r - 1, i));

    return s.commit(out, PrimitiveKind::Sphere,
                    static_cast<uint64_t>(p.segments) * 1024 + static_cast<uint64_t>(p.rings));
}

// ---------------------------------------------------------------------------
bool makeCone(Mesh& out, const ConeParams& p) {
    if (p.bottomRadius <= 0 || p.topRadius < 0 || p.height <= 0 || p.segments < 3)
        return false;
    const int n = p.segments;
    const Real hz = p.height * 0.5f;
    const bool pointed = p.topRadius < 1e-5f;

    SoupBuilder s;
    for (int i = 0; i < n; ++i) {
        const Real a = kTwoPi * static_cast<Real>(i) / static_cast<Real>(n);
        s.vertex({p.bottomRadius * std::cos(a), p.bottomRadius * std::sin(a), -hz});
    }

    if (pointed) {
        const uint32_t apex = s.vertex({0, 0, hz});
        for (int i = 0; i < n; ++i) s.tri(i, (i + 1) % n, apex);
    } else {
        for (int i = 0; i < n; ++i) {
            const Real a = kTwoPi * static_cast<Real>(i) / static_cast<Real>(n);
            s.vertex({p.topRadius * std::cos(a), p.topRadius * std::sin(a), hz});
        }
        for (int i = 0; i < n; ++i) {
            const uint32_t b0 = i, b1 = (i + 1) % n;
            s.quad(b0, b1, b1 + n, b0 + n);
        }
        std::vector<uint32_t> cap;
        for (int i = 0; i < n; ++i) cap.push_back(static_cast<uint32_t>(i + n));
        s.ngon(cap);
        s.nameLastFace(IdRole::Cap, 1);
    }

    std::vector<uint32_t> base;
    for (int i = n - 1; i >= 0; --i) base.push_back(static_cast<uint32_t>(i));
    s.ngon(base);
    s.nameLastFace(IdRole::Cap, 0);
    return s.commit(out, PrimitiveKind::Cone,
                    static_cast<uint64_t>(n) * 2 + (pointed ? 1 : 0));
}

// ---------------------------------------------------------------------------
bool makeTorus(Mesh& out, const TorusParams& p) {
    if (p.minorRadius <= 0 || p.majorRadius <= p.minorRadius ||
        p.majorSegments < 3 || p.minorSegments < 3) return false;
    const int N = p.majorSegments, M = p.minorSegments;

    SoupBuilder s;
    for (int i = 0; i < N; ++i) {
        const Real u = kTwoPi * static_cast<Real>(i) / static_cast<Real>(N);
        const Vec3 dir{std::cos(u), std::sin(u), 0.0f};
        for (int j = 0; j < M; ++j) {
            const Real v = kTwoPi * static_cast<Real>(j) / static_cast<Real>(M);
            const Vec3 c = dir * p.majorRadius;
            s.vertex(c + dir * (p.minorRadius * std::cos(v)) +
                     Vec3{0, 0, p.minorRadius * std::sin(v)});
        }
    }

    auto at = [&](int i, int j) -> uint32_t {
        return static_cast<uint32_t>((i % N) * M + (j % M));
    };
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < M; ++j)
            s.quad(at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1));

    return s.commit(out, PrimitiveKind::Torus,
                    static_cast<uint64_t>(p.majorSegments) * 1024 + static_cast<uint64_t>(p.minorSegments));
}

// ---------------------------------------------------------------------------
bool makePlane(Mesh& out, const PlaneParams& p) {
    if (p.width <= 0 || p.depth <= 0) return false;
    const Real x = p.width * 0.5f, y = p.depth * 0.5f;

    SoupBuilder s;
    s.vertex({-x, -y, 0}); s.vertex({ x, -y, 0});
    s.vertex({ x,  y, 0}); s.vertex({-x,  y, 0});
    s.quad(0, 1, 2, 3);
    return s.commit(out, PrimitiveKind::Plane);
}

} // namespace tg
