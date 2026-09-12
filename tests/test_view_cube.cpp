// The view cube, without a window.
//
// What is worth testing here is not the drawing: it is the mapping from a pixel
// to one of twenty-six directions, and from a direction back to a camera. Those
// two have to be inverses of each other, or clicking a corner of the cube sends
// the view somewhere the cube was not pointing -- which is the one failure a
// user cannot work around, because the widget's whole job is to be trusted.
#include "ui/view_cube.h"

#include <cmath>
#include <cstdio>
#include <set>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}

static Camera viewFrom(Vec3 dir) {
    Camera c;
    c.viewportW = 1600;
    c.viewportH = 900;
    c.setViewDirection(dir);
    c.snapToGoal();
    return c;
}

static std::string key(Vec3 z) {
    auto sign = [](Real v) { return v > 0.3 ? '+' : (v < -0.3 ? '-' : '0'); };
    return std::string{sign(z.x), sign(z.y), sign(z.z)};
}

int main() {
    constexpr float kSize = 104.0f;

    std::printf("--- every zone is a view, and every view is that zone ---\n");
    {
        // The round trip. Twenty-six directions, each asked for and then read
        // back off the camera it produced.
        int n = 0;
        for (int i = -1; i <= 1; ++i)
            for (int j = -1; j <= 1; ++j)
                for (int k = -1; k <= 1; ++k) {
                    if (!i && !j && !k) continue;
                    ++n;
                    const Vec3 want = normalize(Vec3{Real(i), Real(j), Real(k)});
                    const Camera c = viewFrom(want);
                    const Vec3 got = -c.forward();
                    // Top and bottom stop a hundredth of a degree short of the
                    // pole, which is where the turntable's basis gives out.
                    check(length(got - want) < 2e-3,
                          "round trip for " + key(want) + " gave " + key(got));
                }
        check(n == 26, "twenty-six of them");
    }

    std::printf("--- the centre of the cube is the view you are in ---\n");
    {
        const Vec3 dirs[4] = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1},
                              normalize(Vec3{1, -1, 1})};
        for (Vec3 d : dirs) {
            const Camera c = viewFrom(d);
            const Vec3 z = viewCubeZoneAt(c, {0, 0}, kSize);
            check(length(z - d) < 1e-3,
                  "looking from " + key(d) + ", the middle of the cube is " + key(z));
        }
    }

    std::printf("--- the bands around a face are its edges and corners ---\n");
    {
        // Head-on at the front face. The cube's projected half-width is the
        // scale, so these offsets are a known fraction across it.
        const Camera c = viewFrom({0, -1, 0});
        const float scale = kSize * 0.5f / 1.82f;

        const Vec3 mid = viewCubeZoneAt(c, {0, 0}, kSize);
        check(key(mid) == "0-0", "the middle is the face");

        // Right of centre, past the band, is the front-right edge.
        const Vec3 e = viewCubeZoneAt(c, {scale * 0.88f, 0}, kSize);
        check(key(e) == "+-0", std::string("a side band is an edge, got ") + key(e));

        // Up and right is the corner.
        const Vec3 cn = viewCubeZoneAt(c, {scale * 0.88f, -scale * 0.88f}, kSize);
        check(key(cn) == "+-+", std::string("a corner is a corner, got ") + key(cn));

        // And off the cube entirely is nothing at all.
        const Vec3 off = viewCubeZoneAt(c, {scale * 3.0f, 0}, kSize);
        check(lengthSq(off) < 0.5, "outside the cube is not a zone");
    }

    std::printf("--- a click lands where the cursor was ---\n");
    {
        // The two halves together: hit a corner of the cube, take the view it
        // names, and the cube drawn from that view has the same corner facing
        // the eye. This is what makes the widget honest.
        const Camera c = viewFrom({0, -1, 0});
        const float scale = kSize * 0.5f / 1.82f;
        const Vec3 zone = viewCubeZoneAt(c, {scale * 0.88f, -scale * 0.88f}, kSize);

        Camera after = c;
        after.setViewDirection(zone);
        after.snapToGoal();
        check(length(-after.forward() - zone) < 2e-3, "the view faces the zone clicked");
        check(key(viewCubeZoneAt(after, {0, 0}, kSize)) == key(zone),
              "and the cube now shows it in the middle");
    }

    std::printf("--- from a corner, three faces and their shared corner ---\n");
    {
        // The isometric view: whatever else is reachable, the three faces that
        // are turned towards the eye must be, or the commonest views on the
        // cube would be the ones you cannot click.
        const Camera c = viewFrom(normalize(Vec3{1, -1, 1}));
        const float scale = kSize * 0.5f / 1.82f;
        std::set<std::string> found;
        for (int i = -30; i <= 30; ++i)
            for (int j = -30; j <= 30; ++j) {
                const Vec3 z = viewCubeZoneAt(
                    c, {i * scale / 20.0f, j * scale / 20.0f}, kSize);
                if (lengthSq(z) > 0.5) found.insert(key(z));
            }
        for (const char* want : {"+00", "0-0", "00+", "+-+"})
            check(found.count(want) == 1, std::string("can reach ") + want);
        std::printf("  %zu zones reachable from the isometric view\n", found.size());
    }

    std::printf("--- the names are the ones the views have ---\n");
    {
        check(viewCubeZoneName({0, -1, 0}) == "Front", "front");
        check(viewCubeZoneName({0, 0, -1}) == "Bottom", "bottom");
        check(viewCubeZoneName(normalize(Vec3{1, -1, 1})) == "Front Top Right",
              "corners read in the order a person says them");
        check(viewCubeZoneName(normalize(Vec3{-1, 1, 0})) == "Back Left", "edges too");
    }

    std::printf("--- orbiting leaves orthographic, clicking the cube comes back ---\n");
    {
        Camera c = viewFrom({0, -1, 0});
        check(c.orthographic, "a named view is square-on");
        check(c.preferOrtho, "and that is the preference");

        c.orbit(40.0f, 12.0f);
        check(!c.orthographic, "an orbit is a look, not a measurement");
        check(c.preferOrtho, "but it does not change what was asked for");

        c.setViewDirection({1, 0, 0});
        check(c.orthographic, "so a view from the cube comes home to square");

        // Until the preference itself is changed, which then sticks.
        c.toggleProjection();
        check(!c.orthographic && !c.preferOrtho, "the toggle changes both");
        c.orbit(10.0f, 0.0f);
        c.setViewDirection({0, 0, 1});
        check(!c.orthographic, "and a named view respects it");

        // Toggling is about what is on screen, not about the preference: after
        // an orbit has left ortho, asking for the other of what you see has to
        // mean ortho, or the menu item does nothing at all.
        Camera d = viewFrom({0, -1, 0});
        d.orbit(30.0f, 0.0f);
        d.toggleProjection();
        check(d.orthographic && d.preferOrtho, "toggle after an orbit gives ortho");
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
