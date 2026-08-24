// Scene-level behaviour: creation, naming, picking, selection, parametric
// rebuild. No GL context is involved, so this runs headless.
#include "scene/scene.h"
#include "app/camera.h"

#include <cstdio>
#include <string>

using namespace tg;

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) { std::printf("  FAIL: %s\n", what.c_str()); ++failures; }
}
static bool near(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) < eps; }

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

    // ---- Element picking ---------------------------------------------------
    {
        Scene s;
        const ObjectId box = s.addPrimitive(PrimitiveKind::Box);   // 20mm, centred

        // Straight down the -Z axis onto the top face. A viewProj built the
        // same way the camera builds it, looking down from +Z.
        const int W = 800, H = 800;
        const Mat4 view = lookAt({0, 0, 200}, {0, 0, 0}, {0, 1, 0});
        const Mat4 proj = perspective(radians(45.0f), 1.0f, 0.1f, 1000.0f);
        const Mat4 vp = proj * view;

        auto pixelOf = [&](Vec3 world) {
            const Vec4 clip = vp * Vec4(world, 1.0f);
            const Vec3 ndc = clip.xyz() / clip.w;
            return Vec2{(ndc.x * 0.5f + 0.5f) * W, (1.0f - (ndc.y * 0.5f + 0.5f)) * H};
        };

        // Middle of the top face: nothing else is near, so the face wins.
        {
            const Vec2 px = pixelOf({0, 0, 10});
            const ElementHit h = s.pickElement(Ray{{0, 0, 200}, {0, 0, -1}}, vp, W, H, px);
            check(h.hit() && h.ref.kind == ElementKind::Face, "centre of a face picks the face");
            check(h.ref.object == box, "and reports the right object");
        }

        // Over an edge midpoint: the edge beats the face behind it.
        {
            const Vec3 mid{10.0f, 0.0f, 10.0f};       // middle of a top-face edge
            const Vec2 px = pixelOf(mid);
            const Ray ray{{mid.x, mid.y, 200.0f}, {0, 0, -1}};
            const ElementHit h = s.pickElement(ray, vp, W, H, px);
            check(h.hit() && h.ref.kind == ElementKind::Edge, "over an edge picks the edge");
        }

        // Over a corner: the vertex beats the edge.
        {
            const Vec3 corner{10.0f, 10.0f, 10.0f};
            const Vec2 px = pixelOf(corner);
            const Ray ray{{corner.x - 0.01f, corner.y - 0.01f, 200.0f}, {0, 0, -1}};
            const ElementHit h = s.pickElement(ray, vp, W, H, px);
            check(h.hit() && h.ref.kind == ElementKind::Vertex, "over a corner picks the vertex");
        }

        // Off-silhouette picking: cursor 5 pixels outside the boundary edge
        // still picks the edge generously even when raycast misses the surface.
        {
            const Vec3 mid{10.0f, 0.0f, 10.0f};
            const Vec2 px = pixelOf(mid) + Vec2{5.0f, 0.0f}; // 5px off-silhouette into empty space
            const Ray ray{{500.0f, 500.0f, 200.0f}, {0, 0, -1}}; // ray misses surface
            const ElementHit h = s.pickElement(ray, vp, W, H, px);
            check(h.hit() && h.ref.kind == ElementKind::Edge, "off-silhouette click within tolerance picks edge");
            check(h.ref.object == box, "reports the correct box object");
        }

        // Missing the object entirely (far outside tolerance).
        {
            const ElementHit h = s.pickElement(Ray{{500, 500, 200}, {0, 0, -1}}, vp, W, H,
                                               Vec2{0, 0});
            check(!h.hit(), "a distant miss picks nothing");
        }
        std::printf("[pick] vertex/edge/face priority and generous picking ok\n");
    }

    // ---- Element selection bookkeeping -------------------------------------
    {
        Scene s;
        const ObjectId a = s.addPrimitive(PrimitiveKind::Box);
        const ElementRef f0{a, ElementKind::Face, 0};
        const ElementRef f1{a, ElementKind::Face, 1};

        s.selectElement(f0);
        check(s.isElementSelected(f0), "element selected");
        s.selectElement(f1, true);
        check(s.elementSelection().size() == 2, "additive keeps both");
        check(s.selectedFaces(a).size() == 2, "both reported as faces");

        s.toggleElement(f0);
        check(!s.isElementSelected(f0) && s.isElementSelected(f1), "toggle removes");

        // Stale references must not survive a mesh edit that renumbers faces.
        s.selectElement(ElementRef{a, ElementKind::Face, 999}, true);
        s.pruneElementSelection();
        check(s.elementSelection().size() == 1, "out-of-range element pruned");

        // Nor outlive the object itself.
        s.removeObject(a);
        check(s.elementSelection().empty(), "removing the object clears its elements");
        std::printf("[pick] selection bookkeeping ok\n");
    }

    // ---- Zoom-adaptive snap ladder -----------------------------------------
    {
        check(near(niceStep(0.7f), 0.5f), "0.7 -> 0.5");
        check(near(niceStep(1.0f), 1.0f), "1.0 -> 1");
        check(near(niceStep(1.2f), 1.0f), "1.2 -> 1");
        check(near(niceStep(4.0f), 5.0f), "4 -> 5");
        check(near(niceStep(12.0f), 10.0f), "12 -> 10");
        check(near(niceStep(0.06f), 0.05f), "0.06 -> 0.05");

        // Every step must be a 1, 2 or 5 times a power of ten, and the ladder
        // must never go backwards as the requested size grows.
        float prev = 0.0f;
        for (float v = 0.01f; v < 500.0f; v *= 1.05f) {
            const float s = niceStep(v);
            check(s >= prev, "ladder is monotonic");
            prev = s;
            const float m = s / std::pow(10.0f, std::floor(std::log10(s)));
            check(near(m, 1.0f, 1e-3f) || near(m, 2.0f, 1e-3f) || near(m, 5.0f, 1e-3f),
                  "step is a round number");
        }

        // One step should stay about the same size on screen as the camera
        // moves: 42 pixels' worth, rounded.
        Camera cam;
        cam.viewportW = 1000; cam.viewportH = 800;
        for (float d : {20.0f, 100.0f, 500.0f}) {
            cam.distance = d;
            const float step = niceStep(cam.pixelWorldSize({0, 0, 0}) * 42.0f);
            const float px = step / cam.pixelWorldSize({0, 0, 0});
            check(px > 20.0f && px < 90.0f, "one snap step stays a sane screen size");
        }
        std::printf("[snap] ladder and zoom scaling ok\n");
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
