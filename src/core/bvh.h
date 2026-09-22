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
#include <cmath>
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

    // The nearest point of any triangle to `p`, searching no further than
    // `maxDist`. Returns the distance, or `maxDist` if nothing is that close;
    // `triangle` receives the index of the one found, or stays untouched.
    //
    // Nearer children are walked first so the bound tightens early, and a box
    // further away than the best so far is never opened -- which is what makes
    // this cheap when the answer is small, as it is for a surface being checked
    // against itself to within a tenth of a millimetre.
    //
    // `hint`, when given, is a triangle to measure first -- the answer for a
    // nearby point, typically -- which starts the search with a tight bound
    // instead of `maxDist` and lets it prune most of the tree at once. It never
    // changes the answer, only how quickly it is found.
    Real nearestPoint(Vec3 p, Real maxDist, uint32_t* triangle = nullptr,
                      uint32_t hint = 0xffffffffu) const {
        if (nodes_.empty()) return maxDist;
        Real best2 = maxDist * maxDist;
        if (hint < tris_->size() / 3) {
            const Real d2 = pointTriangleDistance2(p, (*pos_)[(*tris_)[hint * 3 + 0]],
                                                   (*pos_)[(*tris_)[hint * 3 + 1]],
                                                   (*pos_)[(*tris_)[hint * 3 + 2]]);
            if (d2 < best2) {
                best2 = d2;
                if (triangle) *triangle = hint;
            }
        }
        uint32_t stack[64];
        int top = 0;
        stack[top++] = 0;

        while (top > 0) {
            const uint32_t at = stack[--top];
            const Node& n = nodes_[at];
            if (boxDistance2(n.min, n.max, p) >= best2) continue;

            if (n.count > 0) {
                for (uint32_t i = 0; i < n.count; ++i) {
                    const uint32_t t = order_[n.first + i];
                    const Real d2 = pointTriangleDistance2(p, (*pos_)[(*tris_)[t * 3 + 0]],
                                                           (*pos_)[(*tris_)[t * 3 + 1]],
                                                           (*pos_)[(*tris_)[t * 3 + 2]]);
                    if (d2 < best2) {
                        best2 = d2;
                        if (triangle) *triangle = t;
                    }
                }
            } else {
                // Push the further child first so the nearer one is walked next.
                const uint32_t left = at + 1, right = n.first;
                const Real dl = boxDistance2(nodes_[left].min, nodes_[left].max, p);
                const Real dr = boxDistance2(nodes_[right].min, nodes_[right].max, p);
                if (dl <= dr) {
                    if (dr < best2) stack[top++] = right;
                    if (dl < best2) stack[top++] = left;
                } else {
                    if (dl < best2) stack[top++] = left;
                    if (dr < best2) stack[top++] = right;
                }
            }
        }
        return best2 < maxDist * maxDist ? std::sqrt(best2) : maxDist;
    }

    // Squared distance from `p` to the triangle (a, b, c), by the region of the
    // triangle's plane the point projects into. Ericson, Real-Time Collision
    // Detection, 5.1.5 -- public because the simplifier measures the same thing
    // against a handful of triangles where a tree would be overhead.
    static Real pointTriangleDistance2(Vec3 p, Vec3 a, Vec3 b, Vec3 c) {
        const Vec3 ab = b - a, ac = c - a, ap = p - a;
        const Real d1 = dot(ab, ap), d2 = dot(ac, ap);
        if (d1 <= 0 && d2 <= 0) return lengthSq(ap);

        const Vec3 bp = p - b;
        const Real d3 = dot(ab, bp), d4 = dot(ac, bp);
        if (d3 >= 0 && d4 <= d3) return lengthSq(bp);

        const Real vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0) {
            const Real v = d1 / (d1 - d3);
            return lengthSq(p - (a + ab * v));
        }

        const Vec3 cp = p - c;
        const Real d5 = dot(ab, cp), d6 = dot(ac, cp);
        if (d6 >= 0 && d5 <= d6) return lengthSq(cp);

        const Real vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0) {
            const Real w = d2 / (d2 - d6);
            return lengthSq(p - (a + ac * w));
        }

        const Real va = d3 * d6 - d5 * d4;
        if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
            const Real w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return lengthSq(p - (b + (c - b) * w));
        }

        // Inside the face. A degenerate triangle has denom zero; the edge and
        // vertex regions above have already caught every point near one, and
        // what reaches here is measured against the plane of what is left.
        const Real sum = va + vb + vc;
        if (std::fabs(sum) < 1e-300) return std::min({lengthSq(ap), lengthSq(bp), lengthSq(cp)});
        const Real denom = Real(1) / sum;
        const Real v = vb * denom, w = vc * denom;
        return lengthSq(p - (a + ab * v + ac * w));
    }

private:
    static Real boxDistance2(Vec3 lo, Vec3 hi, Vec3 p) {
        Real d = 0;
        auto axis = [&](Real v, Real l, Real h) {
            if (v < l) d += (l - v) * (l - v);
            else if (v > h) d += (v - h) * (v - h);
        };
        axis(p.x, lo.x, hi.x);
        axis(p.y, lo.y, hi.y);
        axis(p.z, lo.z, hi.z);
        return d;
    }

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
