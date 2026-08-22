// Scene-level behaviour: creation, naming, picking, selection, parametric
// rebuild. No GL context is involved, so this runs headless.
#include "scene/scene.h"

#include <cstdio>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}

int main() {
    // ---- Creation and naming ---------------------------------------------
    {
        Scene s;
        const ObjectId a = s.addPrimitive(PrimitiveKind::Box);
        const ObjectId b = s.addPrimitive(PrimitiveKind::Box);
        const ObjectId c = s.addPrimitive(PrimitiveKind::Cylinder);
        check(a && b && c, "primitives were created");
        check(s.objectCount() == 3, "three objects present");
        check(s.find(a)->name == "Box", "first box keeps the plain name");
        check(s.find(b)->name == "Box.001", "second box gets a numeric suffix");
        check(s.find(c)->name == "Cylinder", "cylinder named independently");

        // Duplicating Box.001 must not produce Box.001.001.
        const ObjectId d = s.duplicateObject(b);
        check(s.find(d)->name == "Box.002", "duplicate strips the existing suffix");
        std::printf("[naming] %s, %s, %s\n", s.find(a)->name.c_str(),
                    s.find(b)->name.c_str(), s.find(d)->name.c_str());
    }

    // ---- Every primitive kind builds a usable object -----------------------
    {
        Scene s;
        const PrimitiveKind kinds[] = {
            PrimitiveKind::Box, PrimitiveKind::Cylinder, PrimitiveKind::Sphere,
            PrimitiveKind::Cone, PrimitiveKind::Torus, PrimitiveKind::Plane};
        for (PrimitiveKind k : kinds) {
            const ObjectId id = s.addPrimitive(k);
            check(id != kNoObject, std::string("created ") + primitiveName(k));
            const SceneObject* o = s.find(id);
            check(o && !o->render.triangles.empty(),
                  std::string(primitiveName(k)) + " has triangles");
            check(o && o->localBounds.valid(),
                  std::string(primitiveName(k)) + " has valid bounds");
        }
        std::printf("[kinds] all %zu primitive kinds build\n", s.objectCount());
    }

    // ---- Picking ----------------------------------------------------------
    {
        Scene s;
        const ObjectId box = s.addPrimitive(PrimitiveKind::Box);   // 20mm, centred

        // Straight down the -Z axis: must hit the top face at z = +10.
        RayHit hit = s.raycast(Ray{{0, 0, 100}, {0, 0, -1}});
        check(hit.hit() && hit.object == box, "ray hits the box");
        check(std::fabs(hit.point.z - 10.0f) < 1e-3f, "hit lands on the top face");
        check(hit.normal.z > 0.9f, "top face normal points up");
        std::printf("[pick] t=%.3f point=(%.2f,%.2f,%.2f) n=(%.2f,%.2f,%.2f)\n",
                    hit.t, hit.point.x, hit.point.y, hit.point.z,
                    hit.normal.x, hit.normal.y, hit.normal.z);

        // A ray that misses entirely.
        check(!s.raycast(Ray{{500, 500, 100}, {0, 0, -1}}).hit(), "ray misses cleanly");

        // Picking must respect the object transform, not just the local mesh.
        s.find(box)->transform.position = {50, 0, 0};
        check(!s.raycast(Ray{{0, 0, 100}, {0, 0, -1}}).hit(), "moved box no longer under the old ray");
        check(s.raycast(Ray{{50, 0, 100}, {0, 0, -1}}).hit(), "moved box is hit at its new position");

        // ... including non-uniform scale.
        s.find(box)->transform.scale = {1, 1, 4};
        hit = s.raycast(Ray{{50, 0, 100}, {0, 0, -1}});
        check(hit.hit() && std::fabs(hit.point.z - 40.0f) < 1e-3f,
              "non-uniform scale is applied to the hit point");

        // Hidden objects are not pickable.
        s.find(box)->visible = false;
        check(!s.raycast(Ray{{50, 0, 100}, {0, 0, -1}}).hit(), "hidden objects are skipped");
    }

    // ---- Nearest-hit ordering ---------------------------------------------
    {
        Scene s;
        const ObjectId far_  = s.addPrimitive(PrimitiveKind::Box, {}, Vec3{0, 0, 0});
        const ObjectId near_ = s.addPrimitive(PrimitiveKind::Box, {}, Vec3{0, 0, 60});
        (void)far_;
        const RayHit hit = s.raycast(Ray{{0, 0, 200}, {0, 0, -1}});
        check(hit.object == near_, "the nearer of two boxes wins");
    }

    // ---- Selection --------------------------------------------------------
    {
        Scene s;
        const ObjectId a = s.addPrimitive(PrimitiveKind::Box);
        const ObjectId b = s.addPrimitive(PrimitiveKind::Sphere);

        s.select(a);
        check(s.isSelected(a) && !s.isSelected(b), "plain select replaces");
        check(s.activeObject() == a, "selected object is active");

        s.select(b, /*additive=*/true);
        check(s.isSelected(a) && s.isSelected(b), "additive select keeps both");
        check(s.activeObject() == b, "most recent selection is active");

        s.select(a, true);
        check(s.activeObject() == a, "re-selecting promotes to active");
        check(s.selection().size() == 2, "re-selecting does not duplicate the entry");

        s.toggleSelect(a);
        check(!s.isSelected(a) && s.isSelected(b), "toggle removes");

        s.selectAll();
        check(s.selection().size() == 2, "select all covers both");
        s.clearSelection();
        check(s.selection().empty(), "clear empties the selection");

        // Deleting must not leave a dangling selection entry.
        s.select(a);
        s.removeObject(a);
        check(!s.isSelected(a), "removing an object drops it from the selection");
    }

    // ---- Parametric rebuild -----------------------------------------------
    {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        SceneObject* o = s.find(id);
        const uint32_t v0 = o->meshVersion;

        o->spec.box.width = 50.0f;
        check(s.rebuild(id), "rebuild succeeds");
        check(std::fabs(o->localBounds.size().x - 50.0f) < 1e-3f, "width change took effect");
        check(o->meshVersion != v0, "mesh version bumped so the renderer re-uploads");

        // Degenerate parameters must leave the previous geometry intact rather
        // than blanking the object.
        const AABB before = o->localBounds;
        o->spec.box.width = -5.0f;
        check(!s.rebuild(id), "degenerate parameters are rejected");
        check(o->localBounds.max.x == before.max.x, "geometry survives a rejected rebuild");
        check(!o->render.triangles.empty(), "object still renders after a rejected rebuild");
        std::printf("[rebuild] bounds preserved at %.1f mm after invalid input\n",
                    o->localBounds.size().x);
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
