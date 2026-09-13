// The triangle tree against the thing it replaces.
//
// A tree that misses hits does not crash or slow down -- it gets faster and
// quietly wrong, which is exactly what the first one did: it skipped the right
// half of every node and the wall check went from finding thin walls to
// finding almost none. So the test is not "does it hit" but "does it agree
// with asking every triangle", over many rays and a mesh large enough for the
// tree to be deep.
#include "core/bvh.h"

#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}

// Möller–Trumbore, both-sided, written again rather than borrowed so the
// reference does not share a bug with the thing it checks.
static bool refHit(Vec3 a, Vec3 b, Vec3 c, Vec3 o, Vec3 d, Real& t) {
    const Vec3 e1 = b - a, e2 = c - a, p = cross(d, e2);
    const Real det = dot(e1, p);
    if (std::fabs(det) < 1e-12) return false;
    const Vec3 s = o - a;
    const Real u = dot(s, p) / det;
    if (u < -1e-9 || u > 1 + 1e-9) return false;
    const Vec3 q = cross(s, e1);
    const Real v = dot(d, q) / det;
    if (v < -1e-9 || u + v > 1 + 1e-9) return false;
    t = dot(e2, q) / det;
    return true;
}

int main() {
    std::printf("bvh\n");

    // A lumpy sphere: thousands of triangles, every direction covered, so the
    // tree splits on all three axes and grows deep enough to have right halves.
    std::vector<Vec3> pos;
    std::vector<uint32_t> tris;
    const int rings = 60, segs = 90;
    for (int i = 0; i <= rings; ++i) {
        const Real th = kPi * i / rings;
        for (int j = 0; j < segs; ++j) {
            const Real ph = 2 * kPi * j / segs;
            const Real r = 10.0 + 1.5 * std::sin(5 * th) * std::cos(7 * ph);
            pos.push_back({r * std::sin(th) * std::cos(ph), r * std::sin(th) * std::sin(ph),
                           r * std::cos(th)});
        }
    }
    for (int i = 0; i < rings; ++i)
        for (int j = 0; j < segs; ++j) {
            const uint32_t a = i * segs + j, b = i * segs + (j + 1) % segs;
            const uint32_t c = (i + 1) * segs + j, d = (i + 1) * segs + (j + 1) % segs;
            tris.insert(tris.end(), {a, c, b, b, c, d});
        }

    TriangleBvh bvh;
    bvh.build(pos, tris);
    check(!bvh.empty(), "the tree builds");

    std::mt19937 rng(1234);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    int disagreements = 0, hits = 0;
    const int rays = 3000;
    for (int r = 0; r < rays; ++r) {
        Vec3 o{u(rng) * 14, u(rng) * 14, u(rng) * 14};
        Vec3 d{u(rng), u(rng), u(rng)};
        if (lengthSq(d) < 1e-6) continue;
        d = normalize(d);

        Real want = 100.0;
        for (size_t t = 0; t + 2 < tris.size(); t += 3) {
            Real h = 0;
            if (refHit(pos[tris[t]], pos[tris[t + 1]], pos[tris[t + 2]], o, d, h) &&
                h > 1e-6 && h < want)
                want = h;
        }
        const Real got = bvh.nearestHit(o, d, 1e-6, 100.0);
        if (want < 100.0) ++hits;
        if (std::fabs(got - want) > 1e-6) ++disagreements;
    }
    std::printf("  %d rays, %d hit something, %d disagreements\n", rays, hits, disagreements);
    check(hits > rays / 3, "enough rays hit for the comparison to mean something");
    check(disagreements == 0, "the tree agrees with asking every triangle");

    // Rays along an axis are the case the slab test has to survive: a
    // direction component of zero makes a reciprocal of infinity.
    for (int axis = 0; axis < 3; ++axis) {
        Vec3 d{axis == 0 ? 1.0 : 0.0, axis == 1 ? 1.0 : 0.0, axis == 2 ? 1.0 : 0.0};
        const Real got = bvh.nearestHit({0, 0, 0}, d, 1e-6, 100.0);
        check(got > 8.0 && got < 12.0, "an axis-aligned ray from the centre hits the shell");
    }

    std::printf("%s\n", failures ? "FAILED" : "ok");
    return failures ? 1 : 0;
}
