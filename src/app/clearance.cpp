#include "app/clearance.h"

#include "core/bvh.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace tg {
namespace {

// The point of triangle (a, b, c) nearest `p`. Ericson, Real-Time Collision
// Detection, 5.1.5: the region of the triangle's plane the point falls in.
Vec3 closestOnTriangle(Vec3 p, Vec3 a, Vec3 b, Vec3 c) {
    const Vec3 ab = b - a, ac = c - a, ap = p - a;
    const Real d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return a;
    const Vec3 bp = p - b;
    const Real d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return b;
    const Real vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return a + ab * (d1 / (d1 - d3));
    const Vec3 cp = p - c;
    const Real d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return c;
    const Real vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return a + ac * (d2 / (d2 - d6));
    const Real va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    const Real sum = va + vb + vc;
    if (std::fabs(sum) < 1e-300) return a;
    return a + ab * (vb / sum) + ac * (vc / sum);
}

// The nearest points of segments p1-q1 and p2-q2. Ericson 5.1.9.
Real closestSegments(Vec3 p1, Vec3 q1, Vec3 p2, Vec3 q2, Vec3& c1, Vec3& c2) {
    const Vec3 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    const Real a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
    Real s = 0, t = 0;
    const Real eps = 1e-18;
    if (a <= eps && e <= eps) {
        c1 = p1;
        c2 = p2;
        return length(c1 - c2);
    }
    if (a <= eps) {
        t = std::clamp(f / e, Real(0), Real(1));
    } else {
        const Real c = dot(d1, r);
        if (e <= eps) {
            s = std::clamp(-c / a, Real(0), Real(1));
        } else {
            const Real b = dot(d1, d2);
            const Real denom = a * e - b * b;
            s = denom > eps ? std::clamp((b * f - c * e) / denom, Real(0), Real(1)) : Real(0);
            t = (b * s + f) / e;
            if (t < 0) {
                t = 0;
                s = std::clamp(-c / a, Real(0), Real(1));
            } else if (t > 1) {
                t = 1;
                s = std::clamp((b - c) / a, Real(0), Real(1));
            }
        }
    }
    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
    return length(c1 - c2);
}

// Whether segment p-q passes through triangle (a, b, c), and where.
bool segmentThroughTriangle(Vec3 p, Vec3 q, Vec3 a, Vec3 b, Vec3 c, Vec3& at) {
    const Vec3 dir = q - p;
    const Vec3 e1 = b - a, e2 = c - a;
    const Vec3 h = cross(dir, e2);
    const Real det = dot(e1, h);
    if (std::fabs(det) < 1e-18) return false;
    const Real inv = Real(1) / det;
    const Vec3 s = p - a;
    const Real u = dot(s, h) * inv;
    if (u < 0 || u > 1) return false;
    const Vec3 qq = cross(s, e1);
    const Real v = dot(dir, qq) * inv;
    if (v < 0 || u + v > 1) return false;
    const Real t = dot(e2, qq) * inv;
    if (t < 0 || t > 1) return false;
    at = p + dir * t;
    return true;
}

// Whether two triangles that meet actually run into each other: an edge of one
// passes through the inside of the other, its two ends clearly on either side.
// Not merely each across the other's plane -- two wall facets that end on the
// plane where two parts sit, one from below and one from above, are across
// each other's planes and meet at one point on it, and that is touching.
bool edgeThrough(Vec3 p, Vec3 q, const Vec3 t[3], Real eps) {
    const Vec3 n = cross(t[1] - t[0], t[2] - t[0]);
    const Real len = length(n);
    if (len < 1e-18) return false;
    const Vec3 u = n / len;
    const Real dp = dot(p - t[0], u), dq = dot(q - t[0], u);
    if (!((dp > eps && dq < -eps) || (dp < -eps && dq > eps))) return false;
    const Vec3 x = p + (q - p) * (dp / (dp - dq));
    // Inside the triangle by more than eps from each of its sides.
    for (int i = 0; i < 3; ++i) {
        const Vec3 a = t[i], b = t[(i + 1) % 3];
        const Vec3 inward = cross(u, b - a);
        const Real l = length(inward);
        if (l < 1e-18) return false;
        if (dot(x - a, inward) / l <= eps) return false;
    }
    return true;
}

bool crossing(const Vec3 a[3], const Vec3 b[3], Real eps) {
    for (int i = 0; i < 3; ++i)
        if (edgeThrough(a[i], a[(i + 1) % 3], b, eps) || edgeThrough(b[i], b[(i + 1) % 3], a, eps)) return true;
    return false;
}

// One part, meshed and placed. Not to be moved once placed: its tree holds the
// address of `world`.
struct Meshed {
    ObjectId id = 0;
    std::shared_ptr<const RenderMesh> local;
    std::vector<Vec3> world;        // the mesh's positions, placed
    TriangleBvh tree;
    AABB box;
    bool moving = false;
    std::vector<float> nearest;     // per triangle: the nearest another part came
    std::vector<uint8_t> crosses;   // per triangle: runs into another part
    void place(const Mat4& m) {
        world.resize(local->positions.size());
        box = AABB{};
        for (size_t i = 0; i < world.size(); ++i) {
            world[i] = transformPoint(m, local->positions[i]);
            box.expand(world[i]);
        }
        tree.build(world, local->triangles);
    }
    void triangle(uint32_t t, Vec3 out[3]) const {
        for (int k = 0; k < 3; ++k) out[k] = world[local->triangles[t * 3 + k]];
    }
};

// Whether any point of `inner` is inside `outer`: a part wholly inside another
// has no surface crossing it, and would otherwise read as clear.
// A point on the other surface is neither in nor out -- a ray from it may count
// the face it lies on or not -- and two parts sat face to face are full of
// such points. Within a tenth of a micron it is on it: touching, not inside.
constexpr Real kOnSurface = 1e-4;

bool pointInside(Vec3 p, const Meshed& outer) {
    if (p.x < outer.box.min.x || p.x > outer.box.max.x || p.y < outer.box.min.y || p.y > outer.box.max.y ||
        p.z < outer.box.min.z || p.z > outer.box.max.z)
        return false;
    if (outer.tree.nearestPoint(p, kOnSurface * 2.0) <= kOnSurface) return false;
    // Three odd directions, and the answer most of them give: a ray that
    // grazes an edge or runs along a face can miscount, and two such rays
    // from one point hardly ever do.
    static const Vec3 kDirs[3] = {normalize(Vec3{0.5773, 0.5779, 0.5765}), normalize(Vec3{-0.6123, 0.3217, 0.7224}),
                                  normalize(Vec3{0.2811, -0.8467, 0.4517})};
    int votes = 0;
    for (const Vec3& d : kDirs) votes += outer.tree.countHits(p, d) & 1;
    return votes >= 2;
}

// Whether any of `inner` is inside `outer`: a part wholly inside another has no
// surface crossing it, and two parts overlapping with their faces in line --
// boxes of one height pushed into each other -- have none that crosses
// either. Asked at the middles of its triangles as well as its corners, since
// every corner of such a part can lie on the other's surface.
bool inside(const Meshed& inner, const Meshed& outer) {
    if (inner.world.empty() || outer.local->triangles.empty()) return false;
    const size_t tris = inner.local->triangles.size() / 3;
    const size_t stride = std::max<size_t>(1, tris / 96);
    int asked = 0;
    for (size_t t = 0; t < tris; t += stride) {
        Vec3 c{};
        for (int k = 0; k < 3; ++k) c += inner.world[inner.local->triangles[t * 3 + k]];
        c = c * (Real(1) / Real(3));
        if (outer.tree.nearestPoint(c, kOnSurface * 2.0) <= kOnSurface) continue;
        if (pointInside(c, outer)) return true;
        if (++asked >= 96) break;
    }
    return false;
}

constexpr size_t kPairBudget = 40'000'000;
// Enough to draw a band round every hole in a busy assembly, and few enough
// to upload once and draw every frame for nothing.
constexpr size_t kMarkBudget = 60'000;

// Cut at the middle of its longest side, into two: a sliver cut into four
// stays a sliver, and a face triangulated as a fan is all slivers, so cutting
// in quarters covered a whole face in cells to find the edge of a region.
void bisect(Vec3 a, Vec3 b, Vec3 c, Vec3 out[2][3]) {
    const Real ab = lengthSq(b - a), bc = lengthSq(c - b), ca = lengthSq(a - c);
    if (ab >= bc && ab >= ca) {
        const Vec3 m = (a + b) * 0.5;
        out[0][0] = a; out[0][1] = m; out[0][2] = c;
        out[1][0] = m; out[1][1] = b; out[1][2] = c;
    } else if (bc >= ca) {
        const Vec3 m = (b + c) * 0.5;
        out[0][0] = a; out[0][1] = b; out[0][2] = m;
        out[1][0] = a; out[1][1] = m; out[1][2] = c;
    } else {
        const Vec3 m = (c + a) * 0.5;
        out[0][0] = a; out[0][1] = b; out[0][2] = m;
        out[1][0] = m; out[1][1] = b; out[1][2] = c;
    }
}

// The parts of a triangle that come within `required` of another part: the
// triangle cut into quarters where it might, and dropped where it cannot, down
// to cells of about `cell`. A flat face is two triangles, and marking the whole
// of a face because one corner of it is near a pin would say the wrong thing
// about where the pin is too close.
void markNear(Vec3 a, Vec3 b, Vec3 c, const std::vector<const Meshed*>& others, Real required, Real cell,
              int depth, std::vector<Vec3>& out) {
    if (out.size() >= kMarkBudget * 3) return;
    const Vec3 m = (a + b + c) * (Real(1) / Real(3));
    const Real r = std::max({length(a - m), length(b - m), length(c - m)});
    Real d = r + required + 1.0;
    for (const Meshed* o : others) d = std::min(d, o->tree.nearestPoint(m, d));
    if (d >= r + required) return;                      // all of it is further than asked
    // All of it nearer than asked, as far as its corners and middle say: a
    // wall running alongside another, marked whole rather than cut up to find
    // what is already known.
    auto near = [&](Vec3 p) {
        Real q = required;
        for (const Meshed* o : others) q = std::min(q, o->tree.nearestPoint(p, q));
        return q < required;
    };
    if (d < required && near(a) && near(b) && near(c)) {
        out.push_back(a);
        out.push_back(b);
        out.push_back(c);
        return;
    }
    if (r <= cell || depth >= 28) {
        out.push_back(a);
        out.push_back(b);
        out.push_back(c);
        return;
    }
    Vec3 half[2][3];
    bisect(a, b, c, half);
    markNear(half[0][0], half[0][1], half[0][2], others, required, cell, depth + 1, out);
    markNear(half[1][0], half[1][1], half[1][2], others, required, cell, depth + 1, out);
}

// The parts of a triangle inside another part, the same way.
void markInside(Vec3 a, Vec3 b, Vec3 c, const std::vector<const Meshed*>& others, Real cell, int depth,
                std::vector<Vec3>& out) {
    if (out.size() >= kMarkBudget * 3) return;
    const Vec3 m = (a + b + c) * (Real(1) / Real(3));
    const Real r = std::max({length(a - m), length(b - m), length(c - m)});
    bool straddles = false, in = false;
    for (const Meshed* o : others) {
        if (o->tree.nearestPoint(m, r * 1.001 + 1e-9) <= r) straddles = true;
        if (pointInside(m, *o)) in = true;
    }
    if (!straddles || r <= cell || depth >= 28) {
        if (in) {
            out.push_back(a);
            out.push_back(b);
            out.push_back(c);
        }
        return;
    }
    Vec3 half[2][3];
    bisect(a, b, c, half);
    markInside(half[0][0], half[0][1], half[0][2], others, cell, depth + 1, out);
    markInside(half[1][0], half[1][1], half[1][2], others, cell, depth + 1, out);
}

} // namespace

Real clearanceDeviation(Real required) { return std::clamp(required * 0.1, 0.002, 0.02); }

Real triangleDistance(const Vec3 a[3], const Vec3 b[3], Vec3& pa, Vec3& pb) {
    Vec3 at;
    for (int i = 0; i < 3; ++i) {
        if (segmentThroughTriangle(a[i], a[(i + 1) % 3], b[0], b[1], b[2], at) ||
            segmentThroughTriangle(b[i], b[(i + 1) % 3], a[0], a[1], a[2], at)) {
            pa = pb = at;
            return 0.0;
        }
    }
    Real best = std::numeric_limits<Real>::max();
    for (int i = 0; i < 3; ++i) {
        const Vec3 qb = closestOnTriangle(a[i], b[0], b[1], b[2]);
        if (const Real d = length(a[i] - qb); d < best) { best = d; pa = a[i]; pb = qb; }
        const Vec3 qa = closestOnTriangle(b[i], a[0], a[1], a[2]);
        if (const Real d = length(b[i] - qa); d < best) { best = d; pa = qa; pb = b[i]; }
    }
    if (best <= 0.0) return 0.0;        // a corner lying on the other: nothing is nearer
    for (int i = 0; i < 3; ++i)
        for (int k = 0; k < 3; ++k) {
            Vec3 c1, c2;
            const Real d = closestSegments(a[i], a[(i + 1) % 3], b[k], b[(k + 1) % 3], c1, c2);
            if (d < best) { best = d; pa = c1; pb = c2; }
        }
    return best;
}

ClearanceResult checkClearance(ClearanceRequest req, const std::atomic<bool>* cancel) {
    const auto t0 = std::chrono::steady_clock::now();
    ClearanceResult out;
    out.required = req.required;
    out.deviation = req.deviation;
    out.sampleValues = req.sampleValues;
    // Pairs further apart than this are only said to be clear by more than it:
    // a few times the gap asked for, and at least a couple of millimetres, so a
    // gap a little over the requirement is still given as a number.
    out.limit = std::max<Real>(2.0, req.required * 4.0);
    auto stop = [&] { return cancel && cancel->load(std::memory_order_relaxed); };

    std::vector<Meshed> parts;
    parts.reserve(req.bodies.size());
    for (ClearanceBody& b : req.bodies) {
        if (stop()) { out.cancelled = true; return out; }
        Meshed m;
        m.id = b.id;
        m.moving = b.moving;
        if (b.mesh) {
            m.local = b.mesh;
        } else {
            auto rm = std::make_shared<RenderMesh>();
            TessellationQuality q;
            q.deviationMm = req.deviation;
            q.angleRad = 0.2;
            q.independent = true;
            b.body.tessellate(*rm, q);
            m.local = rm;
            out.meshes.emplace_back(b.id, m.local);
        }
        if (m.local->triangles.empty()) continue;
        m.nearest.assign(m.local->triangles.size() / 3, std::numeric_limits<float>::max());
        m.crosses.assign(m.local->triangles.size() / 3, 0);
        // Placed where it will stay: the tree keeps the address of the
        // positions it was built over, so it is built after the move, in a
        // list reserved not to move again.
        parts.push_back(std::move(m));
        parts.back().place(b.model);
    }

    const bool sweeping = !req.samples.empty();
    std::vector<Mat4> baseModel;
    for (const ClearanceBody& b : req.bodies)
        if (std::any_of(parts.begin(), parts.end(), [&](const Meshed& m) { return m.id == b.id; }))
            baseModel.push_back(b.model);

    // Measures one pair as they stand, adding to its answer.
    auto measure = [&](Meshed& A, Meshed& B, ClearancePair& pair, bool mark) {
        const Real limit = out.limit;
        const Real cross = std::max<Real>(req.deviation * 0.05, 1e-6);
        AABB grown = A.box;
        grown.min -= Vec3{limit, limit, limit};
        grown.max += Vec3{limit, limit, limit};
        if (grown.max.x < B.box.min.x || grown.min.x > B.box.max.x || grown.max.y < B.box.min.y ||
            grown.min.y > B.box.max.y || grown.max.z < B.box.min.z || grown.min.z > B.box.max.z)
            return;
        Vec3 ta[3], tb[3];
        A.tree.pairsWithin(B.tree, limit, [&](uint32_t i, uint32_t k) {
            if (++out.trianglePairs > kPairBudget) { out.partial = true; return false; }
            if ((out.trianglePairs & 0xffff) == 0 && stop()) return false;
            A.triangle(i, ta);
            B.triangle(k, tb);
            // Two triangles each already known to touch the other part, in a
            // pair already known to touch: no distance can come of them that
            // is not already known. Only whether they cross is still news --
            // the whole cost of two parts sitting face to face.
            if (mark && pair.gap <= 0.0 && A.nearest[i] <= 0.0f && B.nearest[k] <= 0.0f) {
                if (!A.crosses[i] || !B.crosses[k]) {
                    Vec3 pa, pb;
                    if (crossing(ta, tb, cross) && triangleDistance(ta, tb, pa, pb) <= 1e-9) {
                        pair.overlap = true;
                        A.crosses[i] = B.crosses[k] = 1;
                    }
                }
                return true;
            }
            Vec3 pa, pb;
            const Real d = triangleDistance(ta, tb, pa, pb);
            if (d < pair.gap) { pair.gap = d; pair.pa = pa; pair.pb = pb; }
            if (d <= 1e-9) {
                if (crossing(ta, tb, cross)) {
                    pair.overlap = true;
                    if (mark) { A.crosses[i] = B.crosses[k] = 1; }
                } else {
                    pair.touching = true;
                }
            }
            if (mark) {
                A.nearest[i] = std::min(A.nearest[i], static_cast<float>(d));
                B.nearest[k] = std::min(B.nearest[k], static_cast<float>(d));
            }
            return true;
        });
        if (!pair.overlap && (inside(A, B) || inside(B, A))) {
            pair.overlap = true;
            pair.gap = 0.0;
        }
        // Where they run into each other: every triangle of either that lies
        // inside the other. Crossing triangles alone miss it when faces line
        // up -- two boxes of one height overlapping share their tops, and no
        // triangle of one passes through the other.
        if (pair.overlap && mark) {
            auto within = [&](Meshed& M, const Meshed& O) {
                Vec3 t[3];
                for (size_t i = 0; i < M.crosses.size(); ++i) {
                    M.triangle(static_cast<uint32_t>(i), t);
                    const Vec3 c = (t[0] + t[1] + t[2]) * (Real(1) / Real(3));
                    if (pointInside(c, O)) M.crosses[i] = 1;
                }
            };
            within(A, B);
            within(B, A);
        }
    };

    auto newPair = [&](const Meshed& A, const Meshed& B) {
        ClearancePair p;
        p.a = A.id;
        p.b = B.id;
        p.gap = std::numeric_limits<Real>::max();
        return p;
    };

    if (!sweeping) {
        for (size_t i = 0; i < parts.size(); ++i)
            for (size_t k = i + 1; k < parts.size(); ++k) {
                if (stop() || out.partial) break;
                ClearancePair p = newPair(parts[i], parts[k]);
                measure(parts[i], parts[k], p, true);
                if (p.gap < out.limit || p.overlap) out.pairs.push_back(p);
            }
    } else {
        // Each sample moves the moving parts and measures them against the
        // rest; the pair keeps its tightest moment, and the marks are the ones
        // made there.
        std::vector<ClearancePair> tightest;
        for (size_t s = 0; s < req.samples.size(); ++s) {
            if (stop() || out.partial) break;
            const Mat4 d = translate(req.samples[s].t) * toMat4(req.samples[s].q);
            for (size_t i = 0; i < parts.size(); ++i)
                if (parts[i].moving) parts[i].place(d * baseModel[i]);
            Real worst = std::numeric_limits<Real>::max();
            for (size_t i = 0; i < parts.size(); ++i) {
                if (!parts[i].moving) continue;
                for (size_t k = 0; k < parts.size(); ++k) {
                    if (parts[k].moving) continue;
                    ClearancePair p = newPair(parts[i], parts[k]);
                    measure(parts[i], parts[k], p, false);
                    if (!(p.gap < out.limit || p.overlap)) continue;
                    p.sample = static_cast<int>(s);
                    // Running into something is worse than touching it, which
                    // is worse than any gap: a lid resting on its box at the
                    // start of its swing is not where the swing goes wrong.
                    worst = std::min(worst, p.overlap ? -1.0 : p.gap);
                    auto it = std::find_if(tightest.begin(), tightest.end(), [&](const ClearancePair& q) {
                        return q.a == p.a && q.b == p.b;
                    });
                    const auto rank = [](const ClearancePair& q) { return q.overlap ? -1.0 : q.gap; };
                    if (it == tightest.end()) tightest.push_back(p);
                    else if (rank(p) < rank(*it)) *it = p;
                }
            }
            out.sampleGaps.push_back(worst);
            if (worst < std::numeric_limits<Real>::max() &&
                (out.worstSample < 0 || worst < out.sampleGaps[static_cast<size_t>(out.worstSample)]))
                out.worstSample = static_cast<int>(s);
        }
        out.pairs = tightest;
        // The marks, where it was tightest.
        if (out.worstSample >= 0) out.worstMove = req.samples[static_cast<size_t>(out.worstSample)];
        if (out.worstSample >= 0 && !stop()) {
            const Mat4 d = translate(req.samples[static_cast<size_t>(out.worstSample)].t) *
                           toMat4(req.samples[static_cast<size_t>(out.worstSample)].q);
            for (size_t i = 0; i < parts.size(); ++i)
                if (parts[i].moving) parts[i].place(d * baseModel[i]);
            for (size_t i = 0; i < parts.size(); ++i)
                for (size_t k = 0; k < parts.size(); ++k)
                    if (parts[i].moving && !parts[k].moving) {
                        ClearancePair p = newPair(parts[i], parts[k]);
                        measure(parts[i], parts[k], p, true);
                    }
        }
    }
    if (stop()) { out.cancelled = true; return out; }

    // The regions to draw, refined to where they are. Cells of about a
    // millimetre, or finer for a finer gap: the band a pin is too close in is
    // a ring round its hole, not the face the hole is in.
    //
    // Only against parts it keeps some gap from. Two parts that touch are
    // listed as touching; painting where they touch -- whole faces sat on each
    // other, and a band along every wall beside them, which is truly within
    // the gap of the other part -- would bury the one mark that means
    // something under a hundred that do not.
    const Real cell = std::max<Real>(0.25, std::min<Real>(1.0, req.required * 4.0));
    auto touches = [&](ObjectId a, ObjectId b) {
        for (const ClearancePair& p : out.pairs)
            if (((p.a == a && p.b == b) || (p.a == b && p.b == a)) && p.touching && !p.overlap) return true;
        return false;
    };
    for (const Meshed& m : parts) {
        ClearanceResult::Marks marks;
        marks.id = m.id;
        std::vector<const Meshed*> others;
        std::vector<const Meshed*> overlapping;
        for (const Meshed& o : parts) {
            if (&o == &m || (sweeping && o.moving == m.moving)) continue;
            AABB grown = m.box;
            grown.min -= Vec3{out.limit, out.limit, out.limit};
            grown.max += Vec3{out.limit, out.limit, out.limit};
            if (grown.max.x < o.box.min.x || grown.min.x > o.box.max.x || grown.max.y < o.box.min.y ||
                grown.min.y > o.box.max.y || grown.max.z < o.box.min.z || grown.min.z > o.box.max.z)
                continue;
            overlapping.push_back(&o);
            if (!touches(m.id, o.id)) others.push_back(&o);
        }
        Vec3 t3[3];
        for (size_t t = 0; t < m.nearest.size(); ++t) {
            if (stop()) break;
            m.triangle(static_cast<uint32_t>(t), t3);
            if (m.crosses[t]) markInside(t3[0], t3[1], t3[2], overlapping, cell, 0, marks.overlap);
            else if (m.nearest[t] < req.required && !others.empty())
                markNear(t3[0], t3[1], t3[2], others, req.required, cell, 0, marks.close);
        }
        if (!marks.close.empty() || !marks.overlap.empty()) out.marks.push_back(std::move(marks));
    }
    // Tightest first: that is the one to look at.
    std::sort(out.pairs.begin(), out.pairs.end(), [](const ClearancePair& x, const ClearancePair& y) {
        if (x.overlap != y.overlap) return x.overlap;
        return x.gap < y.gap;
    });
    out.ok = true;
    out.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return out;
}

} // namespace tg
