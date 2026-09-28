#include "app/plane_pick.h"

#include "core/palette.h"
#include "ui/command_panel.h"

#include <imgui.h>

#include <algorithm>
#include <cmath>

namespace tg {

PlaneFrame planeFrameFor(Vec3 origin, Vec3 normal) {
    PlaneFrame f;
    f.origin = origin;
    f.normal = normalize(normal);
    if (std::fabs(f.normal.z) > 0.9 || std::fabs(f.normal.y) > 0.9) {
        f.u = Vec3{1, 0, 0};
        f.v = normalize(cross(f.normal, f.u));
        if (dot(cross(f.u, f.v), f.normal) < 0.0) f.v = -f.v;
    } else if (std::fabs(f.normal.x) > 0.9) {
        f.u = Vec3{0, 1, 0};
        f.v = normalize(cross(f.normal, f.u));
        if (dot(cross(f.u, f.v), f.normal) < 0.0) f.v = -f.v;
    } else {
        f.u = normalize(cross(Vec3{0, 0, 1}, f.normal));
        if (lengthSq(f.u) < 1e-4) f.u = normalize(cross(Vec3{0, 1, 0}, f.normal));
        f.v = normalize(cross(f.normal, f.u));
    }
    return f;
}

PlaneFrame offsetFrame(const PlaneFrame& base, Real offset, Real tiltRad) {
    PlaneFrame f = base;
    const Real c = std::cos(tiltRad), s = std::sin(tiltRad);
    f.v = base.v * c + base.normal * s;
    f.normal = normalize(cross(f.u, f.v));
    f.origin = base.origin + base.normal * offset;
    return f;
}

namespace {

// The origin planes: which way each faces, and what is across it.
struct Origin { PlaneChoice choice; Vec3 normal, a, b; Vec4 tint; };
const Origin kOrigins[3] = {
    {PlaneChoice::XY, {0, 0, 1},  {1, 0, 0}, {0, 1, 0}, {0.2f, 0.6f, 0.9f, 0.4f}},
    {PlaneChoice::XZ, {0, -1, 0}, {1, 0, 0}, {0, 0, 1}, {0.3f, 0.8f, 0.4f, 0.4f}},
    {PlaneChoice::YZ, {1, 0, 0},  {0, 1, 0}, {0, 0, 1}, {0.9f, 0.4f, 0.3f, 0.4f}},
};

Real tileSize(const Camera& camera) { return std::max<Real>(camera.distance * 0.35, 25.0); }

bool exact(const SceneObject* o) { return o && !o->body.empty() && !o->body.isMesh(); }

} // namespace

void PlanePicker::reset() { *this = PlanePicker{}; frame_ = planeFrameFor({0, 0, 0}, {0, 0, 1}); }

void PlanePicker::setMethod(PlaneMethod m) {
    method_ = m;
    points_.clear();
    hoverOk_ = false;
    hoverEdge_.clear();
}

bool PlanePicker::choose(PlaneChoice choice) {
    for (const Origin& o : kOrigins) {
        if (o.choice != choice) continue;
        choice_ = choice;
        frame_ = planeFrameFor({0, 0, 0}, o.normal);
        faceObject_ = kNoObject;
        faceIndex_ = kInvalid;
        return true;
    }
    return false;
}

bool PlanePicker::handleKey(int key) {
    if (key == '7') return choose(PlaneChoice::XY);
    if (key == '1') return choose(PlaneChoice::XZ);
    if (key == '3') return choose(PlaneChoice::YZ);
    return false;
}

void PlanePicker::update(const Scene& scene, const Camera& camera, Vec2 mousePx) {
    const Ray ray = camera.rayThroughPixel(static_cast<float>(mousePx.x), static_cast<float>(mousePx.y));

    if (method_ == PlaneMethod::ThreePoints) {
        // A corner of a body: the nearest vertex the pointer is on.
        hoverOk_ = false;
        const ElementHit hit = scene.pickElement(ray, camera.viewProjection(), camera.viewportW,
                                                 camera.viewportH, mousePx, 16.0f, 0.0f);
        if (hit.hit() && hit.ref.kind == ElementKind::Vertex) {
            const SceneObject* o = scene.find(hit.ref.object);
            if (o) {
                hoverPoint_ = transformPoint(o->modelMatrix(), o->body.vertexPosition(hit.ref.index));
                hoverOk_ = true;
            }
        }
        return;
    }

    if (method_ == PlaneMethod::AlongEdge) {
        hoverEdge_.clear();
        const ElementHit hit = scene.pickElement(ray, camera.viewProjection(), camera.viewportW,
                                                 camera.viewportH, mousePx, 0.0f, 12.0f);
        if (!hit.hit() || hit.ref.kind != ElementKind::Edge) return;
        const SceneObject* o = scene.find(hit.ref.object);
        if (!exact(o)) return;
        std::vector<Vec3> pts;
        o->body.edgePolyline(hit.ref.index, 0.02, pts);
        if (pts.size() < 2) return;
        for (Vec3& p : pts) p = transformPoint(o->modelMatrix(), p);
        hoverEdge_ = pts;
        // The end nearer the pointer, and the way the edge runs there.
        const bool atEnd = lengthSq(hit.point - pts.back()) < lengthSq(hit.point - pts.front());
        const Vec3 at = atEnd ? pts.back() : pts.front();
        const Vec3 along = atEnd ? pts.back() - pts[pts.size() - 2] : pts[1] - pts.front();
        if (lengthSq(along) < 1e-18) return;
        edgeFrame_ = planeFrameFor(at, along);
        return;
    }

    // A face of a body first: drawing on the part is the common case.
    const RayHit hit = scene.raycast(ray);
    if (hit.hit() && hit.face != kInvalid) {
        if (const SceneObject* o = scene.find(hit.object)) {
            const Mat4 model = o->modelMatrix();
            const Vec3 n = normalize(transformVector(normalMatrix(model), o->body.faceNormal(hit.face)));
            std::vector<VertexId> fv;
            o->body.faceVertices(hit.face, fv);
            Vec3 centre{0, 0, 0};
            for (VertexId v : fv) centre += o->body.vertexPosition(v);
            if (!fv.empty()) centre *= 1.0 / static_cast<Real>(fv.size());
            choice_ = PlaneChoice::Face;
            frame_ = planeFrameFor(transformPoint(model, centre), n);
            faceObject_ = hit.object;
            faceIndex_ = hit.face;
            return;
        }
    }
    // The origin planes, nearest first; the top plane when the pointer is on
    // none of them.
    const Real tile = tileSize(camera);
    Real bestT = 1e300;
    PlaneChoice best = PlaneChoice::XY;
    for (const Origin& o : kOrigins) {
        const Real denom = dot(o.normal, ray.dir);
        if (std::fabs(denom) < 1e-9) continue;
        const Real t = dot(-ray.origin, o.normal) / denom;
        if (t < 0.0 || t >= bestT) continue;
        const Vec3 p = ray.origin + ray.dir * t;
        if (std::fabs(dot(p, o.a)) > tile || std::fabs(dot(p, o.b)) > tile) continue;
        bestT = t;
        best = o.choice;
    }
    choose(best);
}

bool PlanePicker::click(const Scene& scene, const Camera& camera) {
    (void)scene;
    if (method_ == PlaneMethod::Surface) return true;
    if (method_ == PlaneMethod::AlongEdge) {
        if (hoverEdge_.empty()) return false;
        frame_ = edgeFrame_;
        choice_ = PlaneChoice::Face;
        faceObject_ = kNoObject;
        faceIndex_ = kInvalid;
        hoverEdge_.clear();
        return true;
    }
    // Three points.
    if (!hoverOk_) return false;
    for (const Vec3& p : points_)
        if (lengthSq(p - hoverPoint_) < 1e-12) return false;
    points_.push_back(hoverPoint_);
    if (points_.size() < 3) return false;
    const Vec3 a = points_[0], b = points_[1], c = points_[2];
    Vec3 n = cross(b - a, c - a);
    if (lengthSq(n) < 1e-12) {
        // In a line: they make no plane. The third is taken back, to be
        // picked again somewhere that does.
        points_.pop_back();
        return false;
    }
    n = normalize(n);
    // Facing the viewer, so what is drawn on it is drawn the right way round.
    if (dot(n, camera.forward()) > 0.0) n = -n;
    PlaneFrame f;
    f.origin = a;
    f.normal = n;
    f.u = normalize(b - a);
    f.v = normalize(cross(n, f.u));
    frame_ = f;
    choice_ = PlaneChoice::Face;
    faceObject_ = kNoObject;
    faceIndex_ = kInvalid;
    points_.clear();
    return true;
}

std::string PlanePicker::prompt() const {
    switch (method_) {
        case PlaneMethod::Surface:
            return "Click a face of a body or an origin plane";
        case PlaneMethod::ThreePoints:
            return "Click corner " + std::to_string(points_.size() + 1) + " of 3 the plane passes through";
        case PlaneMethod::AlongEdge:
            return "Click an edge: the plane stands square to it, at the nearer end";
    }
    return "";
}

void PlanePicker::drawOverlay(const Scene& scene, const Camera& camera, Renderer& renderer) const {
    const Vec4 brand = toVec4(palette::kBrand, 0.95f);
    const Vec4 lit{1.0f, 0.82f, 0.35f, 1.0f};
    if (method_ == PlaneMethod::Surface) {
        const Real sz = tileSize(camera);
        for (const Origin& o : kOrigins) {
            const Vec3 c[4] = {o.a * -sz + o.b * -sz, o.a * sz + o.b * -sz, o.a * sz + o.b * sz, o.a * -sz + o.b * sz};
            const Vec4 edge = choice_ == o.choice ? brand : o.tint;
            for (int i = 0; i < 4; ++i) renderer.addLine(c[i], c[(i + 1) % 4], edge);
            const Vec4 fill{o.tint.x, o.tint.y, o.tint.z, choice_ == o.choice ? 0.16f : 0.08f};
            renderer.addTriangle(c[0], c[1], c[2], fill);
            renderer.addTriangle(c[0], c[2], c[3], fill);
        }
        if (choice_ == PlaneChoice::Face && faceObject_ != kNoObject) {
            if (const SceneObject* o = scene.find(faceObject_)) {
                const RenderMesh& rm = o->render;
                const Mat4 model = o->modelMatrix();
                for (size_t i = 0; i < rm.triangleFace.size(); ++i) {
                    if (rm.triangleFace[i] != faceIndex_) continue;
                    renderer.addTriangle(transformPoint(model, rm.positions[rm.triangles[i * 3]]),
                                         transformPoint(model, rm.positions[rm.triangles[i * 3 + 1]]),
                                         transformPoint(model, rm.positions[rm.triangles[i * 3 + 2]]),
                                         Vec4{palette::kBrand.r, palette::kBrand.g, palette::kBrand.b, 0.45f});
                }
                renderer.addLine(frame_.origin, frame_.origin + frame_.normal * 15.0, brand);
            }
        }
        return;
    }

    // The plane a click would make, as a square: seen before it is taken.
    auto square = [&](const PlaneFrame& f, const Vec4& c) {
        const Real s = tileSize(camera) * 0.5;
        const Vec3 p[4] = {f.toWorld({-s, -s}), f.toWorld({s, -s}), f.toWorld({s, s}), f.toWorld({-s, s})};
        for (int i = 0; i < 4; ++i) renderer.addFrontLine(camera, p[i], p[(i + 1) % 4], c, 1.6);
        renderer.addTriangle(p[0], p[1], p[2], Vec4{c.x, c.y, c.z, 0.10f});
        renderer.addTriangle(p[0], p[2], p[3], Vec4{c.x, c.y, c.z, 0.10f});
    };
    if (method_ == PlaneMethod::AlongEdge) {
        if (hoverEdge_.size() >= 2) {
            for (size_t i = 0; i + 1 < hoverEdge_.size(); ++i)
                renderer.addFrontLine(camera, hoverEdge_[i], hoverEdge_[i + 1], lit, 3.0);
            square(edgeFrame_, brand);
        }
        return;
    }
    // Three points: those picked, the one under the pointer, and the plane
    // once two are down and the third is being pointed at.
    auto mark = [&](Vec3 p, const Vec4& c) {
        const Real r = camera.distance * 0.012;
        renderer.addFrontLine(camera, p - Vec3{r, 0, 0}, p + Vec3{r, 0, 0}, c, 3.0);
        renderer.addFrontLine(camera, p - Vec3{0, r, 0}, p + Vec3{0, r, 0}, c, 3.0);
        renderer.addFrontLine(camera, p - Vec3{0, 0, r}, p + Vec3{0, 0, r}, c, 3.0);
    };
    for (const Vec3& p : points_) mark(p, brand);
    if (hoverOk_) mark(hoverPoint_, lit);
    for (size_t i = 0; i + 1 < points_.size(); ++i) renderer.addFrontLine(camera, points_[i], points_[i + 1], brand, 1.6);
    if (points_.size() == 2 && hoverOk_) {
        const Vec3 n = cross(points_[1] - points_[0], hoverPoint_ - points_[0]);
        if (lengthSq(n) > 1e-12) {
            PlaneFrame f;
            f.origin = points_[0];
            f.normal = normalize(n);
            f.u = normalize(points_[1] - points_[0]);
            f.v = normalize(cross(f.normal, f.u));
            square(f, brand);
        }
    }
}

bool PlanePicker::drawRows(const char* hint) {
    static const ui::Choice kPlanes[3] = {
        {Glyph::Plane, "Top",   "7", "The XY plane, looking down  (7)"},
        {Glyph::Plane, "Front", "1", "The XZ plane, looking from the front  (1)"},
        {Glyph::Plane, "Right", "3", "The YZ plane, looking from the right  (3)"},
    };
    const int on = method_ != PlaneMethod::Surface ? -1
                 : choice_ == PlaneChoice::XY ? 0 : choice_ == PlaneChoice::XZ ? 1
                 : choice_ == PlaneChoice::YZ ? 2 : -1;
    bool decided = false;
    const int pick = ui::commandChoices("Plane", kPlanes, 3, on);
    if (pick >= 0) {
        setMethod(PlaneMethod::Surface);
        decided = choose(kOrigins[pick].choice);
    }
    static const ui::Choice kMethods[3] = {
        {Glyph::Select,      "Face",         nullptr, "A face of a body, or an origin plane, clicked in the view"},
        {Glyph::ThreePoints, "Three points", nullptr, "Through three corners of the model, clicked in turn"},
        {Glyph::AlongEdge,   "At an edge",   nullptr, "Square to an edge, at its nearer end: where a sweep's profile goes"},
    };
    const int m = ui::commandChoices("Or", kMethods, 3, static_cast<int>(method_), true);
    if (m >= 0 && m != static_cast<int>(method_)) setMethod(static_cast<PlaneMethod>(m));
    ui::commandHint(hint && *hint && method_ == PlaneMethod::Surface ? hint : prompt().c_str());
    return decided;
}

} // namespace tg
