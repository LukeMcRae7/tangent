// Scene-level behaviour: creation, naming, picking, selection, parametric
// rebuild. No GL context is involved, so this runs headless.
#include "scene/scene.h"
#include "app/camera.h"
#include "mesh/primitives.h"

#include <cmath>
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

        // Faces in one plane fight over the same pixels. A taller box beside
        // this one, its top at the same height where they overlap, and a copy
        // of this box left exactly where it is: every face under the cursor
        // there has to be a candidate, or only one of them can ever be picked.
        {
            PrimitiveSpec tall;
            tall.kind = PrimitiveKind::Box;
            tall.box.width = tall.box.depth = 10.0;
            tall.box.height = 20.0;
            const ObjectId beside = s.addPrimitive(PrimitiveKind::Box, tall, {5, 5, 0});
            const ObjectId copy = s.duplicateObject(box);
            const Vec2 px = pixelOf({4, 4, 10});
            const std::vector<ElementHit> all =
                s.pickElements(Ray{{4, 4, 200}, {0, 0, -1}}, vp, W, H, px);
            check(all.size() == 3, "three coplanar top faces under the cursor, three candidates");
            bool haveBox = false, haveBeside = false, haveCopy = false;
            for (const ElementHit& h : all) {
                check(h.ref.kind == ElementKind::Face, "each of them a face");
                haveBox |= h.ref.object == box;
                haveBeside |= h.ref.object == beside;
                haveCopy |= h.ref.object == copy;
            }
            check(haveBox && haveBeside && haveCopy, "one from each body");
            check(s.pickElement(Ray{{4, 4, 200}, {0, 0, -1}}, vp, W, H, px).ref == all.front().ref,
                  "the single pick is the first candidate");
            check(s.raycastCoincident(Ray{{4, 4, 200}, {0, 0, -1}}).size() == 3, "and three bodies");

            // Where only the box and its copy are under the cursor, only they
            // are: a candidate has to be as near as the nearest.
            const Vec2 px2 = pixelOf({-5, -5, 10});
            check(s.pickElements(Ray{{-5, -5, 200}, {0, 0, -1}}, vp, W, H, px2).size() == 2,
                  "away from the taller box, two");
            s.removeObject(beside);
            s.removeObject(copy);
            std::printf("[pick] coplanar faces: %zu candidates\n", all.size());
        }
        std::printf("[pick] vertex/edge/face priority and generous picking ok\n");
    }

    // ---- A round rim is clickable all the way round ------------------------
    // Picking measured the cursor against the chord between an edge's two ends.
    // A closed rim's ends are the same point, so the chord collapsed and only
    // the handful of pixels around its seam could ever be clicked.
    if (brep::available()) {
        Scene s;
        PrimitiveSpec plateSpec;
        plateSpec.kind = PrimitiveKind::Box;
        plateSpec.box.width = plateSpec.box.depth = 60;
        plateSpec.box.height = 10;
        PrimitiveSpec drillSpec;
        drillSpec.kind = PrimitiveKind::Cylinder;
        drillSpec.cylinder.radius = 10;
        drillSpec.cylinder.height = 40;

        Body plate, drill, bored;
        check(makePrimitive(plateSpec, plate, Backend::Brep), "exact plate built");
        check(makePrimitive(drillSpec, drill, Backend::Brep), "exact drill built");
        drill.transform(translate({0, 0, -15}));
        std::string why;
        check(booleanOp(plate, drill, BooleanOp::Difference, bored, 5, false, &why),
              std::string("bored the plate: ") + why);

        // The upper rim, and where it sits.
        std::vector<EdgeId> all;
        bored.allEdges(all);
        EdgeId rim = kInvalid;
        Vec3 centre{};
        Real radius = 0;
        for (EdgeId e : all) {
            Vec3 c, ax;
            Real r = 0;
            if (!bored.edgeCircle(e, c, ax, r)) continue;
            if (rim == kInvalid || c.z > centre.z) { rim = e; centre = c; radius = r; }
        }
        check(rim != kInvalid, "found a circular rim");

        const ObjectId id = s.addBody(bored, {}, "Plate");
        const Body& body = s.find(id)->body;

        const int W = 800, H = 800;
        const Mat4 view = lookAt({0, 0, 200}, {0, 0, 0}, {0, 1, 0});
        const Mat4 proj = perspective(radians(45.0f), 1.0f, 0.1f, 1000.0f);
        const Mat4 vp = proj * view;
        auto pixelOf = [&](Vec3 world) {
            const Vec4 clip = vp * Vec4(world, 1.0f);
            const Vec3 ndc = clip.xyz() / clip.w;
            return Vec2{(ndc.x * 0.5f + 0.5f) * W, (1.0f - (ndc.y * 0.5f + 0.5f)) * H};
        };

        const int probes = 8;
        int onRim = 0;
        for (int i = 0; i < probes; ++i) {
            const Real a = 2.0 * kPi * i / probes;
            const Vec3 on{centre.x + radius * std::cos(a),
                          centre.y + radius * std::sin(a), centre.z};
            // Aim the ray just outside the rim so it lands on the top face and
            // the cursor sits right on the drawn curve.
            const Vec3 aim{centre.x + (radius + 0.4) * std::cos(a),
                           centre.y + (radius + 0.4) * std::sin(a), centre.z};
            const ElementHit h = s.pickElement(Ray{{static_cast<float>(aim.x),
                                                    static_cast<float>(aim.y), 200.0f},
                                                   {0, 0, -1}},
                                               vp, W, H, pixelOf(on));
            if (h.hit() && h.ref.kind == ElementKind::Edge &&
                body.edgeKind(h.ref.index) == CurveKind::Circle) ++onRim;
        }
        check(onRim == probes, "a rim picks all the way round, not only at its seam");
        std::printf("[pick] rim clickable at %d of %d points around it\n", onRim, probes);
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
        check(near(niceStep(0.23f), 0.25f), "0.23 -> 0.25");
        check(near(niceStep(2.4f), 2.5f), "2.4 -> 2.5");

        // Rounding to the nearest is wrong when the value is a bound. Below,
        // for a step that must not exceed a limit; above, for a tick spacing
        // that must clear what the eye can separate.
        check(near(niceStepBelow(1.75f), 1.0f), "below 1.75 -> 1");
        check(near(niceStepAbove(1.75f), 2.5f), "above 1.75 -> 2.5");
        check(near(niceStepBelow(2.5f), 2.5f), "a nice number is its own bound, below");
        check(near(niceStepAbove(2.5f), 2.5f), "and above");
        check(near(niceStepAbove(11.25f), 25.0f), "above 11.25 -> 25");
        check(near(niceStepBelow(0.09f), 0.05f), "below 0.09 -> 0.05");
        for (float v = 0.003f; v < 900.0f; v *= 1.07f) {
            check(niceStepBelow(v) <= v * 1.0001f, "below never exceeds");
            check(niceStepAbove(v) >= v * 0.9999f, "above never falls short");
        }

        // Every step is 1, 2.5 or 5 times a power of ten -- halving, which is
        // how a person divides a measurement they are looking at -- and the
        // ladder never goes backwards as the requested size grows.
        float prev = 0.0f;
        for (float v = 0.01f; v < 500.0f; v *= 1.05f) {
            const float s = niceStep(v);
            check(s >= prev, "ladder is monotonic");
            prev = s;
            const float m = s / std::pow(10.0f, std::floor(std::log10(s)));
            check(near(m, 1.0f, 1e-3f) || near(m, 2.5f, 1e-3f) || near(m, 5.0f, 1e-3f),
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

    // ---- Picking a face that had to be split ------------------------------
    //
    // A face here holds one boundary loop, so a bored face has to be cut into
    // pieces. Those cuts are not drawn and cannot be clicked, so picking any one
    // piece has to reach the whole surface -- otherwise the user selects two
    // thirds of a face with nothing on screen to explain why.
    //
    // The other half of the rule matters just as much: a split the *user* made
    // is a face they meant to have. Both are checked here, because widening the
    // selection would be easy to get right for one and wrong for the other.
    {
        // A 40 mm square with a 12 mm square hole, cut into the two pieces a
        // face with a hole has to be here: joined along both cuts, from each
        // inner corner to the outer corner behind it.
        Scene s;
        const std::vector<Vec3> pos = {
            {-20, -20, 5}, {20, -20, 5}, {20, 20, 5}, {-20, 20, 5},   // outer 0..3
            {-6, -6, 5},   {6, -6, 5},   {6, 6, 5},   {-6, 6, 5},     // inner 4..7
        };
        Mesh bored;
        check(bored.build(pos, {6, 6}, {0, 1, 2, 6, 5, 4,
                                        2, 3, 0, 4, 7, 6}),
              "the bored face builds as two pieces");

        const ObjectId id = s.addBody(Body(bored));
        const SceneObject* o = s.find(id);
        check(o != nullptr, "bored plate is in the scene");

        std::vector<Index> top;
        for (Index f = 0; f < o->body.faceCount(); ++f)
            if (o->body.faceNormal(f).z > 0.99 &&
                std::fabs(o->body.faceCentroid(f).z - 5.0) < 1e-3) top.push_back(f);
        check(top.size() == 2, "the bored face came back as two pieces");
        if (top.size() != 2) return 1;

        s.selectElement({id, ElementKind::Face, top[0]});
        check(s.selectedFaces(id).size() == 2, "clicking one piece selects the whole face");

        // The area has to add up to the real face, not to one piece.
        double area = 0.0;
        for (Index f : s.selectedFaces(id)) area += o->body.faceArea(f);
        const double whole = 40.0 * 40.0 - 12.0 * 12.0;
        check(std::fabs(area - whole) < 1e-6, "and the whole of its area");

        // Shift-clicking takes the whole thing back out again.
        s.toggleElement({id, ElementKind::Face, top[1]});
        check(s.selectedFaces(id).empty(), "toggling removes the group together");
        std::printf("[select] bored face: one click selects %zu pieces, %.1f mm2\n",
                    (size_t)2, area);
    }

    if (brep::available()) {
        // A line the user put there divides a face into two on purpose. Picking
        // one must pick one.
        //
        // This used to raise the whole top of a box and take the banding that
        // left down the sides as its two coplanar faces. That banding was never
        // wanted -- a taller box is a box, and drawing a seam across a wall
        // where nothing intersects it is the complaint this tool started from;
        // extrudeFaces merges them now. A divide is the operation that makes
        // such a pair deliberately, so it is the one this asks about.
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box);
        SceneObject* o = s.find(id);
        std::string why;
        check(divideBody(o->body, {0, 0, 0}, {0, 0, 1}, 91, &why),
              "cut a line round the box: " + why);
        o->refreshDerived();

        // The two halves of a side wall: coplanar, sharing one edge.
        Index lower = kInvalid, upper = kInvalid;
        for (Index f = 0; f < o->body.faceCount(); ++f) {
            if (dot(o->body.faceNormal(f), Vec3{1, 0, 0}) < 0.99) continue;
            if (o->body.faceCentroid(f).z < 0.0) lower = f; else upper = f;
        }
        check(lower != kInvalid && upper != kInvalid, "found both halves of the wall");

        s.selectElement({id, ElementKind::Face, lower});
        check(s.selectedFaces(id).size() == 1, "a user's section line still divides the face");
        check(s.selectedFaces(id).front() == lower, "and the piece picked is the one selected");
        std::printf("[select] divided wall: one click selects 1 piece\n");
    } else {
        std::printf("[select] no exact kernel, so nothing to divide\n");
    }

    // ---- Moving, turning and scaling are steps in the history ------------
    {
        Scene s;
        const ObjectId id = s.addPrimitive(PrimitiveKind::Box, {}, {10, 0, 0});
        const size_t root = s.find(id)->features.size();

        // Two nudges are one Move, and the object is where both put it.
        check(s.recordMove(id, {5, 0, 0}) && s.recordMove(id, {0, 3, 0}), "moved twice");
        const SceneObject* o = s.find(id);
        check(o->features.size() == root + 1, "two moves fold into one step");
        check(near(o->transform.position.x, 15) && near(o->transform.position.y, 3), "and it went there");
        check(near(o->base.position.x, 10), "where it was made is unchanged");

        // Turned off in the history, the move takes the object back.
        s.find(id)->features.back().enabled = false;
        s.reevaluate(id);
        check(near(s.find(id)->transform.position.x, 10) && near(s.find(id)->transform.position.y, 0),
              "a move turned off puts the object back");
        s.find(id)->features.back().enabled = true;
        s.reevaluate(id);

        // A move folded back to nothing leaves no step behind.
        check(s.recordMove(id, {-5, -3, 0}), "moved back");
        check(s.find(id)->features.size() == root, "a move folded to nothing is removed");

        // A turn about a point turns the object and carries it round.
        check(s.recordRotate(id, Quat::fromAxisAngle({0, 0, 1}, radians(90.0)), {0, 0, 0}), "turned");
        o = s.find(id);
        check(near(o->transform.position.x, 0, 1e-4f) && near(o->transform.position.y, 10, 1e-4f),
              "turned a quarter about the origin, it stands on +Y");
        check(o->features.back().kind == FeatureKind::Rotate, "as a Rotate step");
        std::printf("[history] move and rotate are steps; placement follows them\n");
    }

    if (brep::available()) {
        // The reported bug: a tool scaled from the side panel went into a
        // boolean as though it had never been scaled. A scale is now a change
        // of shape, so what is baked is what is on the screen.
        Scene s;
        const ObjectId target = s.addPrimitive(PrimitiveKind::Box);                 // 20mm, centred
        const ObjectId tool = s.addPrimitive(PrimitiveKind::Box, {}, {10, 0, 0});
        std::string why;
        check(s.recordScale(tool, {0.5, 1, 1}, {0, 0, 0}, &why), "tool scaled: " + why);
        const SceneObject* t = s.find(tool);
        check(near(t->localBounds.size().x, 10) && near(t->localBounds.size().y, 20),
              "the tool's shape is half as wide");
        check(t->transform.scale == Vec3(1, 1, 1), "and the transform carries no scale");
        check(t->body.faceKind(0) == SurfaceKind::Plane, "a stretched box keeps flat faces");

        Body baked = t->body;
        baked.transform(inverse(s.find(target)->modelMatrix()) * t->modelMatrix());
        Feature cut;
        cut.kind = FeatureKind::Boolean;
        cut.booleanOp = BooleanOp::Difference;
        cut.bakedBody = std::move(baked);
        check(s.addFeature(target, std::move(cut), &why), "cut: " + why);
        const double vol = s.find(target)->body.health(false).volume;
        check(std::fabs(vol - 6000.0) < 1e-3, "the cut takes the scaled tool's volume, not the unscaled one");
        std::printf("[history] scaled tool cuts %.1f mm3 (8000 - 2000)\n", vol);

        // Two scales about the same point fold, and the inspector's number is
        // their product.
        check(s.recordScale(tool, {2, 1, 1}, {0, 0, 0}, &why), "scaled back");
        check(s.find(tool)->features.size() == 1, "a scale folded back to nothing is removed");
        check(s.recordScale(tool, {1, 1, 3}, {0, 0, 0}, &why), "taller");
        check(near(scaleOf(s.find(tool)->features).z, 3), "the overall scale reads back");

        // A negative scale is a mirror, and a sketch has nothing to scale.
        check(!s.recordScale(tool, {-1, 1, 1}, {0, 0, 0}, &why), "a negative scale is refused");
        std::printf("[history] scale is a step: %s\n", why.c_str());
    }

    if (brep::available()) {
        // A round tool stretched into an oval, cut from a box that is then
        // moved, turned and stretched itself: the sequence that once took the
        // kernel down, since the oval's edges were more than the exact stretch
        // knew and the general one faulted on what a boolean leaves behind.
        Scene s;
        const ObjectId box = s.addPrimitive(PrimitiveKind::Box, {}, {0, 0, 10});
        PrimitiveSpec spec;
        spec.kind = PrimitiveKind::Cylinder;
        spec.cylinder.radius = 4.0;
        spec.cylinder.height = 40.0;
        const ObjectId tool = s.addPrimitive(PrimitiveKind::Cylinder, spec, {0, 0, 10});
        std::string why;
        check(s.recordScale(tool, {2.2, 1, 1}, {0, 0, 0}, &why), "the tool is stretched into an oval: " + why);
        Body baked = s.find(tool)->body;
        baked.transform(inverse(s.find(box)->modelMatrix()) * s.find(tool)->modelMatrix());
        Feature cut;
        cut.kind = FeatureKind::Boolean;
        cut.booleanOp = BooleanOp::Difference;
        cut.bakedBody = std::move(baked);
        check(s.addFeature(box, std::move(cut), &why), "the oval is cut: " + why);
        s.recordMove(box, {6, 4, 0});
        s.recordRotate(box, Quat::fromAxisAngle({0, 0, 1}, radians(20.0)), s.find(box)->transform.position);
        check(s.recordScale(box, {1, 1, 0.6}, {0, 0, 0}, &why), "and the box made shorter: " + why);
        check(near(s.find(box)->localBounds.size().z, 12.0f, 1e-3f), "twelve tall");
        std::printf("[history] a cut box with an oval hole stretches: %d faces\n", s.find(box)->body.faceCount());
    }

    std::printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
    return failures ? 1 : 0;
}
