// Tangent - a bounding volume hierarchy over triangles.
//
// Exists because of one measurement. The wall-thickness check casts a ray from
// every face and asked every triangle whether it was hit; on a 62,000-triangle
// import that is nearly four billion tests and took forty seconds. The same
// work through this takes milliseconds, and nothing else about the check had to
// change.
//
// Median split on the widest axis rather than a surface-area heuristic. SAH
// builds a better tree and costs more to build, and the trees here are built
// once per mesh change and then queried tens of thousands of times in a burst
// -- but the burst is over in a few milliseconds either way, and a build that
// is linear and obviously correct is worth more here than a tree that is
// fifteen per cent better to walk.
#pragma once

#include "core/math.h"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace tg {

class TriangleBvh {
public:
    // `triangles` is three indices into `positions` per triangle.
    void build(const std::vector<Vec3>& positions, const std::vector<uint32_t>& triangles) {
        pos_ = &positions;
        tris_ = &triangles;
        nodes_.clear();
        order_.clear();
        const size_t count = triangles.size() / 3;
        if (count == 0) return;

        order_.resize(count);
        for (size_t i = 0; i < count; ++i) order_[i] = static_cast<uint32_t>(i);

        centroids_.resize(count);
        for (size_t i = 0; i < count; ++i) {
            const Vec3 a = positions[triangles[i * 3 + 0]];
            const Vec3 b = positions[triangles[i * 3 + 1]];
            const Vec3 c = positions[triangles[i * 3 + 2]];
            centroids_[i] = (a + b + c) * (Real(1) / Real(3));
        }

        nodes_.reserve(count * 2);
        buildRange(0, count);
        centroids_.clear();
        centroids_.shrink_to_fit();
    }

    bool empty() const { return nodes_.empty(); }

    // Nearest hit along `dir` from `origin`, within [tMin, tMax]. Returns the
    // distance, or `tMax` if nothing was hit.
    Real nearestHit(Vec3 origin, Vec3 dir, Real tMin, Real tMax) const {
        if (nodes_.empty()) return tMax;

        // Reciprocals once rather than per slab test. An axis the ray is
        // parallel to gives an infinity here, which the slab test handles: the
        // comparisons come out as no-overlap rather than as a division by zero.
        const Vec3 inv{Real(1) / dir.x, Real(1) / dir.y, Real(1) / dir.z};

        Real best = tMax;
        uint32_t stack[64];
        int top = 0;
        stack[top++] = 0;

        while (top > 0) {
            const uint32_t at = stack[--top];
            const Node& n = nodes_[at];
            if (!hitsBox(n.min, n.max, origin, inv, tMin, best)) continue;

            if (n.count > 0) {
                for (uint32_t i = 0; i < n.count; ++i) {
                    const uint32_t t = order_[n.first + i];
                    Real hit = 0;
                    if (rayTri((*pos_)[(*tris_)[t * 3 + 0]], (*pos_)[(*tris_)[t * 3 + 1]],
                               (*pos_)[(*tris_)[t * 3 + 2]], origin, dir, hit) &&
                        hit > tMin && hit < best)
                        best = hit;
                }
            } else {
                // The left child is always the next node; the right one is
                // wherever the left subtree ended, which is what `first` holds.
                stack[top++] = at + 1;
                stack[top++] = n.first;
            }
        }
        return best;
    }

private:
    struct Node {
        Vec3 min, max;
        uint32_t first = 0;   // leaf: first index into order_; inner: right child
        uint32_t count = 0;   // 0 means inner; an inner node's left child is the next node
    };

    static constexpr size_t kLeafSize = 8;

    // Returns the index of the node it built.
    uint32_t buildRange(size_t begin, size_t end) {
        const uint32_t self = static_cast<uint32_t>(nodes_.size());
        nodes_.push_back({});

        Vec3 lo{1e30, 1e30, 1e30}, hi{-1e30, -1e30, -1e30};
        for (size_t i = begin; i < end; ++i) {
            const uint32_t t = order_[i];
            for (int k = 0; k < 3; ++k) {
                const Vec3& p = (*pos_)[(*tris_)[t * 3 + k]];
                lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
                hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
            }
        }
        nodes_[self].min = lo;
        nodes_[self].max = hi;

        const size_t count = end - begin;
        if (count <= kLeafSize) {
            nodes_[self].first = static_cast<uint32_t>(begin);
            nodes_[self].count = static_cast<uint32_t>(count);
            return self;
        }

        // Widest axis of the centroids, split at the median. nth_element is
        // linear, which keeps the whole build O(n log n) without a sort.
        Vec3 clo{1e30, 1e30, 1e30}, chi{-1e30, -1e30, -1e30};
        for (size_t i = begin; i < end; ++i) {
            const Vec3& c = centroids_[order_[i]];
            clo = {std::min(clo.x, c.x), std::min(clo.y, c.y), std::min(clo.z, c.z)};
            chi = {std::max(chi.x, c.x), std::max(chi.y, c.y), std::max(chi.z, c.z)};
        }
        const Vec3 span = chi - clo;
        const int axis = span.x >= span.y && span.x >= span.z ? 0 : (span.y >= span.z ? 1 : 2);

        const size_t mid = begin + count / 2;
        std::nth_element(order_.begin() + static_cast<long>(begin),
                         order_.begin() + static_cast<long>(mid),
                         order_.begin() + static_cast<long>(end),
                         [&](uint32_t a, uint32_t b) {
                             const Vec3& ca = centroids_[a];
                             const Vec3& cb = centroids_[b];
                             return (axis == 0 ? ca.x : axis == 1 ? ca.y : ca.z) <
                                    (axis == 0 ? cb.x : axis == 1 ? cb.y : cb.z);
                         });

        // Every centroid identical -- a degenerate cluster. Split down the
        // middle anyway; the tree is poor there and still correct, where
        // recursing on the same range forever would not be.
        // Left subtree first, so it starts at self + 1 and needs no index kept.
        // The right one starts wherever that subtree happened to end, and that
        // is the index that has to be remembered. This used to store self + 1
        // and look for the right child beside it, which is the left child's own
        // first child: every right half of the tree went unvisited, and the
        // wall check reported four thin faces on a mesh where nearly all of
        // them were.
        buildRange(begin, mid);
        const uint32_t right = buildRange(mid, end);
        nodes_[self].first = right;
        nodes_[self].count = 0;
        return self;
    }

    static bool hitsBox(Vec3 lo, Vec3 hi, Vec3 o, Vec3 inv, Real tMin, Real tMax) {
        Real t0 = tMin, t1 = tMax;
        for (int k = 0; k < 3; ++k) {
            const Real oc = k == 0 ? o.x : k == 1 ? o.y : o.z;
            const Real ic = k == 0 ? inv.x : k == 1 ? inv.y : inv.z;
            const Real lc = k == 0 ? lo.x : k == 1 ? lo.y : lo.z;
            const Real hc = k == 0 ? hi.x : k == 1 ? hi.y : hi.z;
            Real near = (lc - oc) * ic;
            Real far = (hc - oc) * ic;
            if (near > far) std::swap(near, far);
            t0 = near > t0 ? near : t0;
            t1 = far < t1 ? far : t1;
            if (t0 > t1) return false;
        }
        return true;
    }

    // Möller–Trumbore, without the back-face cull: material behind a face is
    // material whichever way the triangle in front of it happens to be wound.
    static bool rayTri(Vec3 a, Vec3 b, Vec3 c, Vec3 o, Vec3 d, Real& t) {
        const Vec3 e1 = b - a, e2 = c - a;
        const Vec3 p = cross(d, e2);
        const Real det = dot(e1, p);
        if (std::fabs(det) < 1e-12) return false;
        const Real invDet = Real(1) / det;
        const Vec3 s = o - a;
        const Real u = dot(s, p) * invDet;
        if (u < -1e-9 || u > 1 + 1e-9) return false;
        const Vec3 q = cross(s, e1);
        const Real v = dot(d, q) * invDet;
        if (v < -1e-9 || u + v > 1 + 1e-9) return false;
        t = dot(e2, q) * invDet;
        return true;
    }

    const std::vector<Vec3>* pos_ = nullptr;
    const std::vector<uint32_t>* tris_ = nullptr;
    std::vector<Node> nodes_;
    std::vector<uint32_t> order_;
    std::vector<Vec3> centroids_;
};

} // namespace tg
