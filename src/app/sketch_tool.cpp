#include "app/sketch_tool.h"

#include "app/overlay_shapes.h"
#include "app/snap_overlay.h"
#include "core/palette.h"
#include "geom/brep.h"
#include "ui/command_panel.h"
#include "ui/widgets.h"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace tg {

const char* sketchModeName(SketchMode mode) {
    switch (mode) {
        case SketchMode::Select:    return "Select";
        case SketchMode::Line:      return "Line";
        case SketchMode::Rectangle: return "Rectangle";
        case SketchMode::Circle:    return "Circle";
        case SketchMode::Arc:       return "Arc";
        case SketchMode::Dimension: return "Dimension";
    }
    return "Line";
}

namespace {

// Within this many degrees of level or plumb, a line is taken to mean it.
constexpr Real kLockDegrees = 3.0;
constexpr Real kPointPickPx = 10.0;
constexpr Real kEntityPickPx = 7.0;

const ImVec4 kAccentIm(palette::kBrand.r, palette::kBrand.g, palette::kBrand.b, 1.0f);

// Blue for what can still move, green for what cannot. The same two words a
// person would use, and the same way round as every other sketcher.
constexpr Vec4 kFree(0.38f, 0.66f, 0.95f, 0.95f);
constexpr Vec4 kFixed(0.42f, 0.85f, 0.55f, 0.95f);
const ImVec4 kFreeIm(palette::kInfo.r, palette::kInfo.g, palette::kInfo.b, 1.0f);
const ImVec4 kFixedIm(palette::kValid.r, palette::kValid.g, palette::kValid.b, 1.0f);

// The same axes the create tool puts on a plane, so a sketch and a profile
// drawn on the same face agree about which way is x.
PlaneFrame frameFor(Vec3 origin, Vec3 normal) {
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

bool rayOntoPlane(const Ray& ray, const PlaneFrame& p, Vec3& hit) {
    const Real denom = dot(p.normal, ray.dir);
    if (std::fabs(denom) < 1e-9) return false;
    const Real t = dot(p.origin - ray.origin, p.normal) / denom;
    if (t < 0.0) return false;
    hit = ray.origin + ray.dir * t;
    return true;
}

Real distanceToSegment(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const Real len2 = lengthSq(ab);
    const Real t = len2 > 1e-18 ? clampf(dot(p - a, ab) / len2, 0.0, 1.0) : 0.0;
    return length(p - (a + ab * t));
}

// Pixels per millimetre on the plane where it is being drawn.
Real pixelsPerMm(const Camera& camera, const PlaneFrame& plane) {
    Vec2 a{}, b{};
    if (!camera.projectToPixel(plane.origin, a) ||
        !camera.projectToPixel(plane.origin + plane.u * 10.0, b))
        return 1.0;
    return std::max(length(b - a) / 10.0, 1e-6);
}

bool parseNumber(const std::string& s, Real& out) {
    if (s.empty() || s == "-" || s == "." || s == "-.") return false;
    char* end = nullptr;
    const double v = std::strtod(s.c_str(), &end);
    if (!end || *end != '\0') return false;
    out = v;
    return true;
}

bool contains(const std::vector<SketchId>& v, SketchId id) {
    return std::find(v.begin(), v.end(), id) != v.end();
}

// For the solver's lists, which are sorted: an imported drawing has thousands
// of points, asked about every frame.
bool sortedHas(const std::vector<SketchId>& v, SketchId id) {
    return std::binary_search(v.begin(), v.end(), id);
}


// Diagonal strokes across a region, clipped to it -- holes included -- so a
// region reads as a region whatever shape it is, without triangulating it.
// Two points a stroke, in plane coordinates.
std::vector<Vec2> hatchStrokes(std::vector<std::vector<Vec2>> loops, Real spacingMm) {
    std::vector<Vec2> out;
    // Turned 45 degrees, so the strokes are horizontal in the turned frame.
    const Real c = std::sqrt(0.5);
    auto turn = [c](Vec2 p) { return Vec2{c * (p.x + p.y), c * (p.y - p.x)}; };
    auto back = [c](Vec2 q) { return Vec2{c * (q.x - q.y), c * (q.x + q.y)}; };

    Real lo = 1e300, hi = -1e300;
    for (auto& loop : loops)
        for (Vec2& p : loop) {
            p = turn(p);
            lo = std::min(lo, p.y);
            hi = std::max(hi, p.y);
        }
    if (!(hi > lo) || spacingMm <= 0.0) return out;
    const int count = static_cast<int>((hi - lo) / spacingMm);
    if (count > 600) spacingMm = (hi - lo) / 600.0;

    std::vector<Real> xs;
    for (Real y = lo + spacingMm * 0.5; y < hi; y += spacingMm) {
        xs.clear();
        for (const auto& loop : loops) {
            for (size_t i = 0, n = loop.size(); i < n; ++i) {
                const Vec2 a = loop[i], b = loop[(i + 1) % n];
                if ((a.y > y) == (b.y > y)) continue;
                xs.push_back(a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y));
            }
        }
        std::sort(xs.begin(), xs.end());
        for (size_t i = 0; i + 1 < xs.size(); i += 2) {
            out.push_back(back({xs[i], y}));
            out.push_back(back({xs[i + 1], y}));
        }
    }
    return out;
}

bool insidePolygon(Vec2 p, const std::vector<Vec2>& poly) {
    bool in = false;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const Vec2& a = poly[i];
        const Vec2& b = poly[j];
        if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) in = !in;
    }
    return in;
}

} // namespace

// ---------------------------------------------------------------------------
// Frames
// ---------------------------------------------------------------------------

SketchPlane SketchTool::planeToLocal(const SketchPlane& world, const Mat4& model) {
    const Mat4 inv = inverse(model);
    SketchPlane p;
    p.origin = transformPoint(inv, world.origin);
    p.xAxis = normalize(transformVector(inv, world.xAxis));
    p.yAxis = normalize(transformVector(inv, world.yAxis));
    return p;
}

SketchPlane SketchTool::planeToWorld(const SketchPlane& local, const Mat4& model) {
    SketchPlane p;
    p.origin = transformPoint(model, local.origin);
    p.xAxis = normalize(transformVector(model, local.xAxis));
    p.yAxis = normalize(transformVector(model, local.yAxis));
    return p;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void SketchTool::resetDrawing() {
    // Before the sketch goes: the solver behind a drag is built on this one.
    drag_.reset();
    dragBefore_ = Sketch{};
    dragPoint_ = dragEntity_ = dragDim_ = kNoSketchId;
    dragMoved_ = false;
    sketch_ = Sketch{};
    solved_ = SketchSolve{};
    solved_.solved = true;
    history_.clear();
    regions_.clear();
    chosen_.clear();
    clicks_.clear();
    clickPoints_.clear();
    typed_.clear();
    typedField_ = 0;
    fixed_[0] = fixed_[1] = false;
    activeDim_ = kNoSketchId;
    hoverPoint_ = hoverEntity_ = hoverRegion_ = kNoSketchId;
    handleCurve_ = kNoSketchId;
    lock_ = Lock::None;
    snap_ = PlaneSnap{};
    depth_ = depthBase_ = 10.0;
    depthTyped_ = false;
    choice_.reset();
    reach_.clear();
    adjusted_ = false;
    reachToolKey_.clear();
    reachTool_ = Body{};
    escapeArmed_ = false;
    mode_ = SketchMode::Line;
    svg_ = SvgInsert{};
    svgPlace_ = SvgPlacement{};
    svgName_.clear();
    importRequested_ = false;
}

void SketchTool::start() {
    resetDrawing();
    error_.clear();
    editObject_ = kNoObject;
    editUid_ = 0;
    faceObject_ = kNoObject;
    cameraSaved_ = false;
    hoveredChoice_ = PlaneChoice::XY;
    plane_ = frameFor({0, 0, 0}, {0, 0, 1});
    stage_ = SketchStage::SelectPlane;
    pendingSvg_ = SvgDrawing{};
    pendingSvgName_.clear();
}

void SketchTool::startImport(SvgDrawing drawing, std::string name) {
    start();
    pendingSvg_ = std::move(drawing);
    pendingSvgName_ = std::move(name);
}

bool SketchTool::importSvg(SvgDrawing drawing, std::string name) {
    if (stage_ != SketchStage::Draw) {
        error_ = "An SVG goes into a sketch while it is being drawn";
        return false;
    }
    if (!drawing.ok || drawing.empty()) {
        error_ = drawing.error.empty() ? "There is nothing in that SVG to import" : drawing.error;
        return false;
    }
    endDrag();
    clearPending();
    Sketch before = sketch_;
    SvgPlacement at;
    const SvgInsert added = insertSvg(sketch_, drawing, at);
    Sketch kept = before;
    if (!settle(std::move(before), "The drawing")) return false;
    // settle() ends any placement, which is right for what came before this
    // one: this one is what can now be placed.
    svg_ = added;
    svgPlace_ = at;
    svgName_ = std::move(name);
    svgDrawing_ = std::move(drawing);
    svgBefore_ = std::move(kept);
    mode_ = SketchMode::Select;
    return true;
}

bool SketchTool::setPlacement(const SvgPlacement& placement) {
    if (svg_.empty() || stage_ != SketchStage::Draw) return false;
    if (!(placement.scale > 1e-9) || !std::isfinite(placement.scale)) {
        error_ = "The drawing has to have some size";
        return false;
    }
    svgPlace_ = placement;
    placeSvg(sketch_, svg_, svgPlace_);
    solved_ = solveNow();
    refreshRegions();
    return true;
}

bool SketchTool::setInkColours(const std::vector<bool>& ink) {
    if (svg_.empty() || stage_ != SketchStage::Draw || !svgDrawing_.source) return false;
    SvgDrawing again = recolourSvg(svgDrawing_, ink);
    if (!again.ok) {
        error_ = again.error.empty() ? "Nothing would be left" : again.error;
        return false;
    }
    // Back to the sketch before the drawing, and the drawing in again, at the
    // same size and place. The step back from here is still to before it.
    sketch_ = svgBefore_;
    svg_ = insertSvg(sketch_, again, svgPlace_);
    svgDrawing_ = std::move(again);
    solved_ = solveNow();
    chosen_.clear();
    refreshRegions();
    return true;
}

void SketchTool::chooseAllRegions() {
    chosen_.clear();
    for (const SketchProfile& r : regions_) chosen_.push_back(r.key);
}

void SketchTool::chooseFilledRegions() {
    chosen_ = sketchFilledProfiles(sketch_, regions_);
}

bool SketchTool::startEdit(const Scene& scene, ObjectId object, ElementId sketchUid,
                           Camera& camera) {
    const SceneObject* obj = scene.find(object);
    const Feature* source = nullptr;
    if (obj)
        for (const Feature& f : obj->features)
            if (f.kind == FeatureKind::Sketch && f.uid == sketchUid) source = &f;
    if (!source) {
        error_ = "That sketch is no longer in the history";
        return false;
    }

    resetDrawing();
    error_.clear();
    editObject_ = object;
    editUid_ = sketchUid;
    faceObject_ = object;
    cameraSaved_ = false;

    sketch_ = source->sketch;
    sketch_.plane = planeToWorld(source->sketch.plane, obj->modelMatrix());
    plane_.origin = sketch_.plane.origin;
    plane_.u = sketch_.plane.xAxis;
    plane_.v = sketch_.plane.yAxis;
    plane_.normal = sketch_.plane.normal();

    solved_ = solveNow();
    refreshRegions();
    stage_ = SketchStage::Draw;
    // Editing a sketch that already exists is mostly moving and resizing what
    // is in it, so that is the tool it opens with.
    mode_ = SketchMode::Select;
    squareUp(camera);
    return true;
}

void SketchTool::cancel(Camera& camera) {
    restoreCamera(camera);
    stage_ = SketchStage::None;
    resetDrawing();
    editObject_ = kNoObject;
    faceObject_ = kNoObject;
}

void SketchTool::squareUp(Camera& camera) {
    if (!cameraSaved_) {
        savedCamera_.target = camera.target;
        savedCamera_.distance = camera.distance;
        savedCamera_.yaw = camera.yaw;
        savedCamera_.pitch = camera.pitch;
        savedCamera_.orthographic = camera.orthographic;
        cameraSaved_ = true;
    }
    float y = 0.0f, p = 0.0f;
    Camera::anglesFor(plane_.normal, y, p);
    camera.animateTo(plane_.origin, camera.distance, y, p);
    camera.orthographic = true;
}

// Back to the view the tool found, for choosing what to sweep and how far: a
// depth is judged in perspective, not head-on.
void SketchTool::restoreCamera(Camera& camera) {
    if (!cameraSaved_) return;
    camera.animateTo(savedCamera_.target, savedCamera_.distance,
                     savedCamera_.yaw, savedCamera_.pitch);
    camera.orthographic = savedCamera_.orthographic;
}

// ---------------------------------------------------------------------------
// Plane
// ---------------------------------------------------------------------------

void SketchTool::choosePlane(PlaneChoice choice, Camera& camera) {
    if (stage_ != SketchStage::SelectPlane) return;
    switch (choice) {
        case PlaneChoice::XY: setPlane(frameFor({0, 0, 0}, {0, 0, 1}), kNoObject, &camera); break;
        case PlaneChoice::XZ: setPlane(frameFor({0, 0, 0}, {0, -1, 0}), kNoObject, &camera); break;
        case PlaneChoice::YZ: setPlane(frameFor({0, 0, 0}, {1, 0, 0}), kNoObject, &camera); break;
        case PlaneChoice::Face:
        case PlaneChoice::None: break;
    }
}

void SketchTool::setPlane(const PlaneFrame& frame, ObjectId faceObject, Camera* camera) {
    resetDrawing();
    plane_ = frame;
    faceObject_ = faceObject;
    sketch_.plane.origin = frame.origin;
    sketch_.plane.xAxis = frame.u;
    sketch_.plane.yAxis = frame.v;
    stage_ = SketchStage::Draw;
    if (pendingSvg_.ok) {
        SvgDrawing d = std::move(pendingSvg_);
        pendingSvg_ = SvgDrawing{};
        importSvg(std::move(d), std::move(pendingSvgName_));
        pendingSvgName_.clear();
    }
    if (camera) {
        squareUp(*camera);
        if (placing()) frameDrawing(*camera);
    }
}

void SketchTool::frameDrawing(Camera& camera) {
    if (svg_.empty()) return;
    const Vec2 size = svg_.size * svgPlace_.scale;
    // Into the upper half of the view, which is what the sketch's panel
    // leaves uncovered: the view at least twice the drawing high and a tenth
    // wider than it, with the drawing's middle a fifth of the view above the
    // view's. The view's height is 2 d tan(fov / 2).
    const Real aspect = std::max<Real>(camera.aspect(), 0.1);
    const Real viewHeight = std::max({size.y * 2.0, size.x * 1.1 / aspect, 1.0});
    const float dist = static_cast<float>(viewHeight / (2.0 * std::tan(camera.fovY * 0.5)));
    float y = 0.0f, p = 0.0f;
    Camera::anglesFor(plane_.normal, y, p);
    const Vec2 target = svgPlace_.centre - Vec2{0.0, viewHeight * 0.2};
    camera.animateTo(plane_.toWorld(target), std::max(dist, 10.0f), y, p);
}

// ---------------------------------------------------------------------------
// The sketch
// ---------------------------------------------------------------------------

SketchSolve SketchTool::solveNow() {
    // Nothing drawn is nothing to solve, and is not a failure to.
    if (sketch_.entities.empty() && sketch_.constraints.empty()) {
        SketchSolve s;
        s.solved = true;
        return s;
    }
    return solveSketch(sketch_);
}

bool SketchTool::settle(Sketch before, const char* what) {
    SketchSolve s = solveNow();
    if (!s.solved) {
        sketch_ = std::move(before);
        error_ = std::string(what) + " was not added: " + s.reason;
        solved_ = solveNow();
        refreshRegions();
        return false;
    }
    solved_ = std::move(s);
    history_.push_back(std::move(before));
    escapeArmed_ = false;
    // Something else has been drawn: the import is ordinary geometry now, and
    // resizing it would undo whatever was just done to it.
    endPlacing();
    refreshRegions();
    return true;
}

void SketchTool::refreshRegions() {
    regions_ = sketchProfiles(sketch_);
    chosen_.erase(std::remove_if(chosen_.begin(), chosen_.end(), [&](SketchId key) {
        return std::none_of(regions_.begin(), regions_.end(),
                            [key](const SketchProfile& p) { return p.key == key; });
    }), chosen_.end());
}

Vec2 SketchTool::pointUV(SketchId id) const {
    const SketchPoint* p = sketch_.point(id);
    return p ? p->at : Vec2{};
}

SketchId SketchTool::nearestPoint(Vec2 uv, Real radius) const {
    SketchId best = kNoSketchId;
    Real bestD = radius;
    for (const SketchPoint& p : sketch_.points) {
        const Real d = length(p.at - uv);
        if (d <= bestD) { bestD = d; best = p.id; }
    }
    return best;
}

SketchId SketchTool::pointAt(Vec2 uv, SketchId reuse) {
    return reuse != kNoSketchId ? reuse : sketch_.addPoint(uv);
}

void SketchTool::setMode(SketchMode mode) {
    clearPending();
    mode_ = mode;
    if (mode != SketchMode::Dimension) activeDim_ = kNoSketchId;
}

void SketchTool::clearPending() {
    clicks_.clear();
    clickPoints_.clear();
    typed_.clear();
    typedField_ = 0;
    fixed_[0] = fixed_[1] = false;
    lock_ = Lock::None;
}

Vec2 SketchTool::previewEnd() const {
    if (clicks_.empty()) return cursor_;
    const Vec2 s = clicks_.front();
    Vec2 toward = cursor_ - s;
    const Real len = length(toward);
    const Vec2 dir = len > 1e-12 ? toward / len : Vec2{1, 0};

    switch (mode_) {
    case SketchMode::Line: {
        if (fixed_[0]) {
            Vec2 d = dir;
            if (lock_ == Lock::Horizontal) d = {dir.x < 0 ? -1.0 : 1.0, 0.0};
            if (lock_ == Lock::Vertical)   d = {0.0, dir.y < 0 ? -1.0 : 1.0};
            return s + d * fixedValue_[0];
        }
        if (hoverPoint_ != kNoSketchId) return cursor_;
        if (lock_ == Lock::Horizontal) return {cursor_.x, s.y};
        if (lock_ == Lock::Vertical)   return {s.x, cursor_.y};
        return cursor_;
    }
    case SketchMode::Rectangle: {
        const Real w = fixed_[0] ? fixedValue_[0] : std::fabs(cursor_.x - s.x);
        const Real h = fixed_[1] ? fixedValue_[1] : std::fabs(cursor_.y - s.y);
        return {s.x + (cursor_.x < s.x ? -w : w), s.y + (cursor_.y < s.y ? -h : h)};
    }
    case SketchMode::Circle:
        return s + dir * (fixed_[0] ? fixedValue_[0] : len);
    case SketchMode::Arc:
        if (clicks_.size() == 1) {
            if (!fixed_[0] && hoverPoint_ != kNoSketchId) return cursor_;
            return s + dir * (fixed_[0] ? fixedValue_[0] : len);
        }
        if (hoverPoint_ != kNoSketchId) return cursor_;
        return s + dir * length(clicks_[1] - s);
    case SketchMode::Select:
    case SketchMode::Dimension:
        break;
    }
    return cursor_;
}

void SketchTool::clickAt(Vec2 uv) {
    if (stage_ != SketchStage::Draw) return;
    const SketchId on = nearestPoint(uv, pickMm_);
    cursor_ = on != kNoSketchId ? pointUV(on) : uv;
    cursorValid_ = true;
    hoverPoint_ = on;

    if (mode_ == SketchMode::Dimension) {
        SketchId best = kNoSketchId;
        Real bestD = pickMm_;
        for (const SketchEntity& e : sketch_.entities) {
            const std::vector<Vec2> pts = sketchEntityPoints(sketch_, e);
            for (size_t i = 0; i + 1 < pts.size(); ++i) {
                const Real d = distanceToSegment(uv, pts[i], pts[i + 1]);
                if (d <= bestD) { bestD = d; best = e.id; }
            }
        }
        if (best != kNoSketchId) dimensionEntity(best);
        return;
    }

    // Level or plumb, when that is plainly what was meant.
    lock_ = Lock::None;
    if (mode_ == SketchMode::Line && clicks_.size() == 1) {
        const Vec2 d = cursor_ - clicks_.front();
        if (on != kNoSketchId) {
            if (std::fabs(d.y) < 1e-9 && std::fabs(d.x) > 1e-9) lock_ = Lock::Horizontal;
            else if (std::fabs(d.x) < 1e-9 && std::fabs(d.y) > 1e-9) lock_ = Lock::Vertical;
        } else if (length(d) > 1e-9) {
            const Real deg = std::atan2(std::fabs(d.y), std::fabs(d.x)) * kRad2Deg;
            if (deg < kLockDegrees)              lock_ = Lock::Horizontal;
            else if (deg > 90.0 - kLockDegrees)  lock_ = Lock::Vertical;
        }
    }
    commitPending();
}

// One click's worth of drawing, at the cursor as it was last aimed.
// ---------------------------------------------------------------------------
// Dragging
// ---------------------------------------------------------------------------

bool SketchTool::beginDrag(SketchId point) {
    if (stage_ != SketchStage::Draw || drag_ || !sketch_.point(point)) return false;
    dragBefore_ = sketch_;
    drag_ = std::make_unique<SketchSolver>(sketch_);
    if (!drag_->beginDrag(point)) {
        drag_.reset();
        return false;
    }
    dragPoint_ = point;
    dragEntity_ = dragDim_ = kNoSketchId;
    dragMoved_ = false;
    return true;
}

bool SketchTool::beginRadiusDrag(SketchId entity) {
    const SketchEntity* e = sketch_.entity(entity);
    if (stage_ != SketchStage::Draw || drag_ || !e) return false;
    if (e->curve != SketchCurve::Circle && e->curve != SketchCurve::Arc) return false;
    dragBefore_ = sketch_;
    drag_ = std::make_unique<SketchSolver>(sketch_);

    // A circle drawn here carries its radius as a dimension, which is what
    // holds it still -- so pulling on the rim drives that number rather than
    // being refused by it. The number in the panel moves with the pointer, and
    // what the sketch says about itself stays true.
    dragDim_ = kNoSketchId;
    const SketchId dim = existingDimension(entity);
    const SketchConstraint* k = sketch_.constraint(dim);
    if (k && k->rule == SketchRule::Radius) {
        dragDim_ = dim;
        activeDim_ = dim;
    } else if (!drag_->beginRadiusDrag(entity)) {
        drag_.reset();
        return false;
    }
    dragEntity_ = entity;
    dragPoint_ = kNoSketchId;
    dragMoved_ = false;
    return true;
}

bool SketchTool::dragTo(Vec2 at) {
    if (!drag_) return false;
    bool moved = false;
    if (dragPoint_ != kNoSketchId) {
        moved = drag_->dragTo(at);
    } else if (const SketchEntity* e = sketch_.entity(dragEntity_)) {
        const SketchPoint* centre = sketch_.point(e->a);
        const Real r = centre ? length(at - centre->at) : 0.0;
        if (!centre || r < 1e-6) {
            moved = false;
        } else if (dragDim_ != kNoSketchId) {
            moved = drag_->setDimension(dragDim_, r) && drag_->solve().solved;
        } else {
            moved = drag_->dragRadiusTo(r);
        }
    }
    if (moved) dragMoved_ = true;
    return moved;
}

void SketchTool::endDrag() {
    if (!drag_) return;
    drag_->endDrag();
    drag_.reset();
    dragPoint_ = dragEntity_ = kNoSketchId;
    dragDim_ = kNoSketchId;
    // One drag is one step back, and only if it actually moved something: a
    // click that missed everything must not fill the undo history.
    if (dragMoved_) {
        solved_ = solveNow();
        history_.push_back(std::move(dragBefore_));
        endPlacing();
        refreshRegions();
    }
    dragBefore_ = Sketch{};
    dragMoved_ = false;
}

bool SketchTool::commitPending() {
    if (stage_ != SketchStage::Draw || mode_ == SketchMode::Dimension ||
        mode_ == SketchMode::Select || !cursorValid_)
        return false;
    escapeArmed_ = false;

    if (clicks_.empty()) {
        clicks_.push_back(cursor_);
        clickPoints_.push_back(hoverPoint_);
        typed_.clear();
        fixed_[0] = fixed_[1] = false;
        return true;
    }

    const Vec2 start = clicks_.front();
    const Vec2 end = previewEnd();
    Sketch before = sketch_;

    switch (mode_) {
    case SketchMode::Line: {
        const bool reuseEnd = hoverPoint_ != kNoSketchId && !fixed_[0];
        if (length(end - start) < 1e-6 || (reuseEnd && hoverPoint_ == clickPoints_.front()))
            return false;
        const SketchId a = pointAt(start, clickPoints_.front());
        const SketchId b = pointAt(end, reuseEnd ? hoverPoint_ : kNoSketchId);
        const SketchId line = sketch_.addLine(a, b);
        if (lock_ == Lock::Horizontal) sketch_.constrain(SketchRule::Horizontal, line);
        if (lock_ == Lock::Vertical)   sketch_.constrain(SketchRule::Vertical, line);
        if (fixed_[0]) sketch_.constrain(SketchRule::Distance, a, b, fixedValue_[0]);
        if (!settle(std::move(before), "That line")) return false;
        // A chain goes on from where the line ended, unless it ended on
        // something already there -- closing a loop is the end of drawing it.
        clearPending();
        if (!reuseEnd) {
            clicks_.push_back(pointUV(b));
            clickPoints_.push_back(b);
        }
        return true;
    }
    case SketchMode::Rectangle: {
        const Vec2 lo{std::min(start.x, end.x), std::min(start.y, end.y)};
        const Real w = std::fabs(end.x - start.x), h = std::fabs(end.y - start.y);
        if (w < 1e-6 || h < 1e-6) return false;
        sketch_.addRectangle(lo, w, h);
        clearPending();
        return settle(std::move(before), "That rectangle");
    }
    case SketchMode::Circle: {
        const Real r = length(end - start);
        if (r < 1e-6) return false;
        const bool newCentre = clickPoints_.front() == kNoSketchId;
        const SketchId centre = pointAt(start, clickPoints_.front());
        // A new centre is held where it was put, the way a rectangle's corner
        // is, so the circle is fully sized by the one number it carries.
        if (newCentre) sketch_.constrain(SketchRule::Fix, centre, kNoSketchId, start.x, start.y);
        const SketchId circle = sketch_.addCircle(centre, r);
        sketch_.constrain(SketchRule::Radius, circle, kNoSketchId, r);
        clearPending();
        return settle(std::move(before), "That circle");
    }
    case SketchMode::Arc: {
        if (clicks_.size() == 1) {
            if (length(end - start) < 1e-6) return false;
            const SketchId reuse = !fixed_[0] ? hoverPoint_ : kNoSketchId;
            clicks_.push_back(end);
            clickPoints_.push_back(reuse);
            typed_.clear();
            fixed_[0] = fixed_[1] = false;
            return true;
        }
        const Vec2 from = clicks_[1];
        if (length(end - from) < 1e-6) return false;
        const SketchId centre = pointAt(start, clickPoints_[0]);
        SketchId s = pointAt(from, clickPoints_[1]);
        SketchId e = pointAt(end, hoverPoint_);
        if (s == e) return false;
        // The short way round, whichever order the ends were clicked in.
        const Vec2 ds = from - start, de = end - start;
        if (ds.x * de.y - ds.y * de.x < 0.0) std::swap(s, e);
        sketch_.addArc(centre, s, e);
        clearPending();
        return settle(std::move(before), "That arc");
    }
    case SketchMode::Select:
    case SketchMode::Dimension:
        break;
    }
    return false;
}

bool SketchTool::typeKey(int key) {
    enum class Target { None, Size, Dimension, Depth } target = Target::None;
    if (stage_ == SketchStage::Depth) target = Target::Depth;
    else if (stage_ == SketchStage::Draw && activeDim_ != kNoSketchId &&
             (mode_ == SketchMode::Dimension || mode_ == SketchMode::Select))
        target = Target::Dimension;
    else if (stage_ == SketchStage::Draw && !clicks_.empty() &&
             !(mode_ == SketchMode::Arc && clicks_.size() == 2))
        target = Target::Size;
    if (target == Target::None) return false;

    const bool digit = key >= '0' && key <= '9';
    if (digit || key == '.') {
        typed_ += static_cast<char>(key);
    } else if (key == '-') {
        if (target != Target::Depth || !typed_.empty()) return true;
        typed_ += '-';
    } else if (key == 8) {
        if (typed_.empty()) return false;
        typed_.pop_back();
    } else if (key == 9) {
        if (target == Target::Size && mode_ == SketchMode::Rectangle) {
            typed_.clear();
            typedField_ = 1 - typedField_;
        }
        return true;
    } else {
        return false;
    }

    Real v = 0.0;
    const bool ok = parseNumber(typed_, v);
    switch (target) {
    case Target::Depth:
        depthTyped_ = ok;
        if (ok) depth_ = v;
        break;
    case Target::Size:
        fixed_[typedField_] = ok && v > 0.0;
        if (fixed_[typedField_]) fixedValue_[typedField_] = v;
        break;
    case Target::Dimension:
    case Target::None:
        break;   // a dimension takes its number on Enter, not per keystroke
    }
    return true;
}

SketchId SketchTool::existingDimension(SketchId entity) const {
    const SketchEntity* e = sketch_.entity(entity);
    if (!e) return kNoSketchId;
    for (const SketchConstraint& k : sketch_.constraints) {
        const bool same =
            (e->curve == SketchCurve::Line && k.rule == SketchRule::Distance &&
             ((k.first == e->a && k.second == e->b) || (k.first == e->b && k.second == e->a))) ||
            ((e->curve == SketchCurve::Circle || e->curve == SketchCurve::Arc) &&
             k.rule == SketchRule::Radius && k.first == e->id);
        if (same) return k.id;
    }
    return kNoSketchId;
}

SketchId SketchTool::dimensionEntity(SketchId entity) {
    const SketchEntity* e = sketch_.entity(entity);
    if (!e || stage_ != SketchStage::Draw) return kNoSketchId;

    // A size already given is the one to edit, not a second one to add.
    if (const SketchId have = existingDimension(entity); have != kNoSketchId) {
        activeDim_ = have;
        typed_.clear();
        return have;
    }

    Sketch before = sketch_;
    SketchId id = kNoSketchId;
    switch (e->curve) {
    case SketchCurve::Line:
        id = sketch_.constrain(SketchRule::Distance, e->a, e->b, length(pointUV(e->b) - pointUV(e->a)));
        break;
    case SketchCurve::Circle:
        id = sketch_.constrain(SketchRule::Radius, e->id, kNoSketchId, e->radius);
        break;
    case SketchCurve::Arc:
        id = sketch_.constrain(SketchRule::Radius, e->id, kNoSketchId,
                               length(pointUV(e->b) - pointUV(e->a)));
        break;
    case SketchCurve::Bezier:
        error_ = "A curve has no single size to dimension";
        return kNoSketchId;
    }

    // A size the rest of the sketch already decides would only be a second
    // place for the same number to disagree with.
    SketchSolve s = solveNow();
    if (!s.solved || s.redundant.size() > solved_.redundant.size() || contains(s.redundant, id)) {
        error_ = s.solved ? "That size is already set by the rest of the sketch"
                          : "That dimension was not added: " + s.reason;
        sketch_ = std::move(before);
        return kNoSketchId;
    }
    solved_ = std::move(s);
    history_.push_back(std::move(before));
    endPlacing();
    refreshRegions();
    activeDim_ = id;
    typed_.clear();
    return id;
}

bool SketchTool::setDimension(SketchId constraint, Real value) {
    SketchConstraint* k = sketch_.constraint(constraint);
    if (!k || !isDimension(k->rule)) return false;
    if (k->rule != SketchRule::Angle && value <= 0.0) {
        error_ = "A length or a radius has to be more than zero";
        return false;
    }
    if (k->value == value) return true;
    Sketch before = sketch_;
    k->value = value;
    SketchSolve s = solveNow();
    if (!s.solved) {
        sketch_ = std::move(before);
        error_ = "The sketch cannot take that size: " + s.reason;
        return false;
    }
    solved_ = std::move(s);
    history_.push_back(std::move(before));
    endPlacing();
    refreshRegions();
    return true;
}

bool SketchTool::deleteEntity(SketchId entity) {
    const SketchEntity* found = sketch_.entity(entity);
    if (!found || stage_ != SketchStage::Draw) return false;
    const SketchEntity gone = *found;
    Sketch before = sketch_;

    sketch_.entities.erase(std::remove_if(sketch_.entities.begin(), sketch_.entities.end(),
        [&](const SketchEntity& e) { return e.id == entity; }), sketch_.entities.end());

    std::vector<SketchId> removed{entity};
    for (SketchId p : {gone.a, gone.b, gone.c, gone.d}) {
        if (p == kNoSketchId) continue;
        const bool used = std::any_of(sketch_.entities.begin(), sketch_.entities.end(),
            [p](const SketchEntity& e) { return e.a == p || e.b == p || e.c == p || e.d == p; });
        if (used) continue;
        removed.push_back(p);
        sketch_.points.erase(std::remove_if(sketch_.points.begin(), sketch_.points.end(),
            [p](const SketchPoint& q) { return q.id == p; }), sketch_.points.end());
    }
    sketch_.constraints.erase(std::remove_if(sketch_.constraints.begin(), sketch_.constraints.end(),
        [&](const SketchConstraint& k) {
            if (contains(removed, k.first) || contains(removed, k.second)) return true;
            // Every length this tool makes runs along a line -- a rectangle's
            // width is its bottom side's. With the line gone the number would
            // still bind the solver and be drawn nowhere.
            if (k.rule != SketchRule::Distance) return false;
            return std::none_of(sketch_.entities.begin(), sketch_.entities.end(),
                [&](const SketchEntity& e) {
                    return e.curve == SketchCurve::Line &&
                           ((e.a == k.first && e.b == k.second) || (e.a == k.second && e.b == k.first));
                });
        }), sketch_.constraints.end());

    if (activeDim_ != kNoSketchId && !sketch_.constraint(activeDim_)) activeDim_ = kNoSketchId;
    clearPending();
    return settle(std::move(before), "Deleting that");
}

bool SketchTool::toggleConstruction(SketchId entity) {
    if (!sketch_.entity(entity) || stage_ != SketchStage::Draw) return false;
    Sketch before = sketch_;
    SketchEntity* e = sketch_.entity(entity);
    e->construction = !e->construction;
    return settle(std::move(before), "That change");
}

bool SketchTool::undoEdit() {
    if (history_.empty()) return false;
    endPlacing();
    sketch_ = std::move(history_.back());
    history_.pop_back();
    solved_ = solveNow();
    if (activeDim_ != kNoSketchId && !sketch_.constraint(activeDim_)) activeDim_ = kNoSketchId;
    clearPending();
    refreshRegions();
    return true;
}

// ---------------------------------------------------------------------------
// Extruding
// ---------------------------------------------------------------------------

bool SketchTool::beginExtrude(Camera* camera) {
    if (stage_ != SketchStage::Draw) return false;
    endDrag();
    clearPending();
    refreshRegions();
    if (regions_.empty()) {
        error_ = "Nothing in the sketch closes yet: a region needs a loop that ends where it began";
        return false;
    }
    // Nothing picked yet: the filled regions, which for one region is that
    // one, for a plate with a bore is the plate, and for an imported logo is
    // every letter with its counters left open.
    if (chosen_.empty()) chooseFilledRegions();
    stage_ = SketchStage::Regions;
    if (camera) restoreCamera(*camera);
    return true;
}

void SketchTool::toggleRegion(SketchId key) {
    auto it = std::find(chosen_.begin(), chosen_.end(), key);
    if (it != chosen_.end()) { chosen_.erase(it); return; }
    if (std::any_of(regions_.begin(), regions_.end(),
                    [key](const SketchProfile& p) { return p.key == key; }))
        chosen_.push_back(key);
}

bool SketchTool::beginDepth() {
    if (stage_ != SketchStage::Regions) return false;
    if (chosen_.empty()) {
        error_ = "Pick a region to extrude";
        return false;
    }
    typed_.clear();
    depthTyped_ = false;
    depthBase_ = depth_;
    stage_ = SketchStage::Depth;
    return true;
}


Vec2 SketchTool::chosenCentre() const {
    Vec2 sum{0, 0};
    int n = 0;
    for (const SketchProfile& r : regions_) {
        if (!contains(chosen_, r.key)) continue;
        for (Vec2 p : sketchLoopPoints(sketch_, r.outer)) { sum += p; ++n; }
    }
    return n ? sum / static_cast<Real>(n) : Vec2{0, 0};
}

bool SketchTool::finish(Scene& scene, Camera& camera, UndoStack& undo, bool extrude) {
    if (stage_ != SketchStage::Draw && stage_ != SketchStage::Regions &&
        stage_ != SketchStage::Depth)
        return false;
    endDrag();
    clearPending();
    if (sketch_.empty()) {
        error_ = "The sketch is empty";
        return false;
    }
    if (!solved_.solved) {
        error_ = "The sketch does not solve: " + solved_.reason;
        return false;
    }
    if (extrude) {
        if (!brep::available()) {
            error_ = "Extruding a sketch needs the exact kernel, which this build does not have";
            return false;
        }
        if (chosen_.empty()) {
            error_ = "Pick a region to extrude";
            return false;
        }
        if (std::fabs(depth_) < 1e-6) {
            error_ = "The depth is zero";
            return false;
        }
        // A refusal leaves the tool where it was, so the operation or the
        // bodies can be changed and Finish tried again.
        if (!commitExtrusion(scene, undo)) return false;
        restoreCamera(camera);
        cameraSaved_ = false;
        stage_ = SketchStage::Applied;
        return true;
    }

    // Kept as a sketch: in the part it was drawn on or is being edited in, or,
    // drawn on a plane of its own, as its own object in the outliner, to be
    // extruded whenever.
    const ObjectId target = owner();
    if (target == kNoObject) {
        std::vector<Feature> chain;
        Feature s;
        s.kind = FeatureKind::Sketch;
        s.uid = scene.takeFeatureUid();
        s.sketch = sketch_;
        chain.push_back(s);
        std::string why;
        const ObjectId id = scene.addFeatureChain(std::move(chain), "Sketch", &why);
        if (id == kNoObject) {
            error_ = "The sketch was not kept: " + (why.empty() ? std::string("it would not evaluate") : why);
            return false;
        }
        undo.push(ExistenceCommand::forCreate(scene, {id}));
        scene.select(id);
    } else {
        SceneObject* obj = scene.find(target);
        if (!obj) {
            error_ = "The part this sketch belongs to is gone";
            return false;
        }
        const std::vector<Feature> before = obj->features;
        std::vector<Feature> chain = obj->features;
        Sketch local = sketch_;
        local.plane = planeToLocal(sketch_.plane, obj->modelMatrix());
        if (editing()) {
            auto it = std::find_if(chain.begin(), chain.end(), [&](const Feature& f) {
                return f.kind == FeatureKind::Sketch && f.uid == editUid_;
            });
            if (it == chain.end()) {
                error_ = "That sketch is no longer in the history";
                return false;
            }
            it->sketch = std::move(local);
        } else {
            Feature s;
            s.kind = FeatureKind::Sketch;
            s.uid = scene.takeFeatureUid();
            s.sketch = std::move(local);
            chain.push_back(std::move(s));
        }
        std::string why;
        if (!scene.setFeatures(target, std::move(chain), &why)) {
            error_ = std::string(editing() ? "The edit was not kept: " : "The sketch was not kept: ") +
                     (why.empty() ? std::string("the history would not evaluate") : why);
            return false;
        }
        undo.push(std::make_unique<FeatureCommand>(target, before, scene.find(target)->features,
                                                   editing() ? "Edit Sketch" : "Sketch"));
        scene.select(target);
    }

    restoreCamera(camera);
    cameraSaved_ = false;
    stage_ = SketchStage::None;
    resetDrawing();
    editObject_ = kNoObject;
    faceObject_ = kNoObject;
    return true;
}

bool SketchTool::recommit(Scene& scene, UndoStack& undo) {
    if (stage_ != SketchStage::Applied) return false;
    return commitExtrusion(scene, undo);
}

void SketchTool::dismissApplied() {
    if (stage_ != SketchStage::Applied) return;
    stage_ = SketchStage::None;
    resetDrawing();
    editObject_ = kNoObject;
    faceObject_ = kNoObject;
}

Body SketchTool::sweptRegions(std::string* why) const {
    if (chosen_.empty()) return Body{};
    BrepRef all = brep::sketchSolids(sketch_, regions_, chosen_, 0.0, depth_, 0, why);
    return all ? Body(std::move(all)) : Body{};
}

void SketchTool::refreshReach(const Scene& scene, bool now) {
    if (choice_.op == ExtrudeOp::NewBody || !brep::available() || chosen_.empty()) {
        reach_.refresh(scene, Body{}, owner(), choice_.op, depth_, "none");
        return;
    }

    // Where the sweep could possibly go: the box around the chosen regions,
    // from the plane to the depth. With no body near that box there is
    // nothing to reach, and nothing needs sweeping to know it -- the usual
    // case for a drawing brought in to be a part of its own.
    AABB swept;
    {
        std::vector<SketchId> chosen = chosen_;
        std::sort(chosen.begin(), chosen.end());
        Vec2 lo{1e300, 1e300}, hi{-1e300, -1e300};
        for (const SketchProfile& r : regions_) {
            if (!std::binary_search(chosen.begin(), chosen.end(), r.key)) continue;
            for (SketchId id : r.outer.entities) {
                const SketchEntity* e = sketch_.entity(id);
                if (!e) continue;
                for (SketchId p : {e->a, e->b, e->c, e->d}) {
                    if (p == kNoSketchId) continue;
                    const Vec2 q = pointUV(p);
                    const Real grow = e->curve == SketchCurve::Circle || e->curve == SketchCurve::Arc
                                          ? std::max(e->radius, length(pointUV(e->b) - pointUV(e->a)))
                                          : 0.0;
                    lo = {std::min(lo.x, q.x - grow), std::min(lo.y, q.y - grow)};
                    hi = {std::max(hi.x, q.x + grow), std::max(hi.y, q.y + grow)};
                }
            }
        }
        for (Vec2 c : {lo, hi, Vec2{lo.x, hi.y}, Vec2{hi.x, lo.y}}) {
            swept.expand(plane_.toWorld(c));
            swept.expand(plane_.toWorld(c) + plane_.normal * depth_);
        }
    }
    const bool near = std::any_of(scene.objects().begin(), scene.objects().end(), [&](const auto& o) {
        return o->visible && !o->body.empty() && !o->body.isMesh() && swept.overlaps(o->worldBounds(), 1e-3);
    });
    if (!near) {
        reach_.refresh(scene, Body{}, owner(), choice_.op, depth_, "clear");
        return;
    }

    // The same regions at the same depth are the same tool, and the bodies it
    // reaches need not be measured again.
    std::string key;
    char b[96];
    std::snprintf(b, sizeof b, "%.9g|%.6g,%.6g,%.6g", depth_, plane_.origin.x, plane_.origin.y,
                  plane_.origin.z);
    key = b;
    for (SketchId k : chosen_) key += "|" + std::to_string(k);
    if (reachToolKey_ != key) {
        // A sweep that takes a noticeable time is not made again until four
        // times that has passed: a depth dragged across a drawing of hundreds
        // of letters would otherwise sweep all of them on every frame. What it
        // reaches is shown a moment late; what is built is measured now.
        const auto t = std::chrono::steady_clock::now();
        const double since = std::chrono::duration<double, std::milli>(t - reachBuiltAt_).count();
        if (!now && !reachToolKey_.empty() && reachBuildMs_ > 20.0 && since < 4.0 * reachBuildMs_) return;
        reachTool_ = sweptRegions(nullptr);
        reachToolKey_ = key;
        reachBuiltAt_ = std::chrono::steady_clock::now();
        reachBuildMs_ = std::chrono::duration<double, std::milli>(reachBuiltAt_ - t).count();
    }
    reach_.refresh(scene, reachTool_, owner(), choice_.op, depth_, key);
}

bool SketchTool::commitExtrusion(Scene& scene, UndoStack& undo) {
    // Measured now rather than trusted from the last frame: this is what the
    // extrusion is about to act on.
    choice_.follow(depth_, owner() != kNoObject);
    refreshReach(scene, true);
    ExtrudeOp op = choice_.op;
    std::vector<ObjectId> bodies = reach_.included();

    // A sketch that is an object of its own, with no body yet, becomes a part
    // when it is joined or made a body: its first solid is these regions, in
    // its own history, with anything else the join reaches merged into it.
    const ObjectId belongs = owner();
    const SceneObject* belongsTo = scene.find(belongs);
    const bool firstSolid = belongsTo && belongsTo->body.empty() &&
                            (op == ExtrudeOp::Join || op == ExtrudeOp::NewBody);
    if (firstSolid) {
        if (op == ExtrudeOp::NewBody) bodies.clear();
        op = ExtrudeOp::Join;
        bodies.insert(bodies.begin(), belongs);
    }

    // A join that reaches nothing has nothing to join, and is a body of its
    // own -- as it is in Fusion.
    const bool asNewBody = op == ExtrudeOp::NewBody || (op == ExtrudeOp::Join && bodies.empty());
    if (asNewBody && editing()) {
        error_ = "A sketch already in a part extrudes into that part: leave the part in, or pick Cut or Intersect";
        return false;
    }
    if (!asNewBody && bodies.empty()) {
        error_ = op == ExtrudeOp::Cut ? "Nothing there to cut into" : "Nothing there to intersect with";
        return false;
    }

    // Every chosen region in one step: one sweep and one boolean, however
    // many letters a drawing has.
    std::vector<SketchId> keys = chosen_;
    std::sort(keys.begin(), keys.end());
    auto sweep = [&](ElementId sketchUid, ExtrudeOp how) {
        Feature f;
        f.kind = FeatureKind::ExtrudeProfile;
        f.uid = scene.takeFeatureUid();
        f.sketchUid = sketchUid;
        f.profileKeys = keys;
        f.distance = depth_;
        f.extrudeOp = how;
        return f;
    };

    if (asNewBody) {
        // A part of its own: the sketch where it was drawn, and the regions
        // swept out of it. Once a solid stands where the sketch is, the
        // drawing is scaffolding and is hidden.
        std::vector<Feature> chain;
        Feature s;
        s.kind = FeatureKind::Sketch;
        s.uid = scene.takeFeatureUid();
        s.sketch = sketch_;
        s.sketchShown = false;
        chain.push_back(s);
        chain.push_back(sweep(s.uid, ExtrudeOp::Join));
        std::string why;
        const ObjectId id = scene.addFeatureChain(std::move(chain), "Part", &why);
        if (id == kNoObject) {
            error_ = "Extrude failed: " + (why.empty() ? std::string("no solid came out of it") : why);
            return false;
        }
        undo.push(ExistenceCommand::forCreate(scene, {id}));
        scene.select(id);
        return true;
    }

    // Where the sketch lives: the part it belongs to, or -- drawn on a plane
    // of its own -- the first body it acts on, where it stays editable and
    // re-runs with that part's history. The regions are swept there as steps
    // of that history; every other body reached takes the swept solid.
    const ObjectId home = editing() || firstSolid ? belongs
                        : belongs != kNoObject && reach_.includes(belongs) ? belongs
                                                                           : bodies.front();
    const bool homeActs = std::find(bodies.begin(), bodies.end(), home) != bodies.end();
    SceneObject* obj = scene.find(home);
    if (!obj) {
        error_ = "The part this sketch belongs to is gone";
        return false;
    }
    // An object with no body yet is a sketch waiting to be built from, not a
    // mesh: Body calls itself a mesh until a kernel shape is put in it.
    if (!obj->body.empty() && obj->body.isMesh()) {
        error_ = "Sketching onto a mesh needs a solid: Modify > Convert to Solid first";
        return false;
    }

    const char* label = op == ExtrudeOp::Join ? "Extrude Join"
                      : op == ExtrudeOp::Cut  ? "Extrude Cut" : "Extrude Intersect";
    const std::vector<Feature> before = obj->features;
    std::vector<Feature> chain = obj->features;
    Sketch local = sketch_;
    local.plane = planeToLocal(sketch_.plane, obj->modelMatrix());
    ElementId sketchUid = 0;
    if (editing()) {
        auto it = std::find_if(chain.begin(), chain.end(), [&](const Feature& f) {
            return f.kind == FeatureKind::Sketch && f.uid == editUid_;
        });
        if (it == chain.end()) {
            error_ = "That sketch is no longer in the history";
            return false;
        }
        it->sketch = std::move(local);
        it->sketchShown = false;
        sketchUid = editUid_;
    } else {
        Feature s;
        s.kind = FeatureKind::Sketch;
        s.uid = scene.takeFeatureUid();
        s.sketch = std::move(local);
        s.sketchShown = false;
        sketchUid = s.uid;
        chain.push_back(std::move(s));
    }
    if (homeActs) chain.push_back(sweep(sketchUid, op));

    std::string why;
    if (!scene.setFeatures(home, std::move(chain), &why)) {
        error_ = std::string(editing() ? "The edit was not kept: " : "Extrude failed: ") +
                 (why.empty() ? std::string("the history would not evaluate") : why);
        return false;
    }
    std::vector<std::unique_ptr<Command>> parts;
    parts.push_back(std::make_unique<FeatureCommand>(home, before, scene.find(home)->features,
                                                     editing() ? "Edit Sketch" : label));

    if (bodies.size() > (homeActs ? 1u : 0u)) {
        const Body tool = sweptRegions(&why);
        std::string error;
        if (tool.empty()) {
            unwind(scene, parts);
            error_ = "Extrude failed: " + (why.empty() ? std::string("the regions could not be swept") : why);
            return false;
        }
        if (!applyExtrude(scene, tool, op, bodies, homeActs ? home : kNoObject, "Sketch", label, parts,
                          error)) {
            unwind(scene, parts);
            error_ = error;
            return false;
        }
    }
    if (parts.size() == 1) undo.push(std::move(parts.front()));
    else                   undo.push(std::make_unique<CompositeCommand>(std::move(parts), label));
    if (scene.find(home)) scene.select(home);
    return true;
}

// ---------------------------------------------------------------------------
// Per frame
// ---------------------------------------------------------------------------

bool SketchTool::unproject(const Camera& camera, Vec2 mousePx, Vec2& uv) const {
    Vec3 hit{};
    if (!rayOntoPlane(camera.rayThroughPixel(static_cast<float>(mousePx.x),
                                             static_cast<float>(mousePx.y)), plane_, hit))
        return false;
    uv = plane_.toUV(hit);
    return true;
}

// Where along the plane's normal the pointer is: the closest approach between
// the pointer's ray and the normal through the middle of what is being swept.
Real SketchTool::depthFromPointer(const Camera& camera, Vec2 mousePx) const {
    const Vec3 centre = plane_.toWorld(chosenCentre());
    const Ray ray = camera.rayThroughPixel(static_cast<float>(mousePx.x),
                                           static_cast<float>(mousePx.y));
    const Vec3 w0 = centre - ray.origin;
    const Real b = dot(plane_.normal, ray.dir);
    const Real c = dot(ray.dir, ray.dir);
    const Real d = dot(plane_.normal, w0);
    const Real e = dot(ray.dir, w0);
    const Real denom = c - b * b;
    if (std::fabs(denom) < 1e-6) return depth_;
    return (b * e - c * d) / denom;
}

void SketchTool::update(const Scene& scene, const Camera& camera, Vec2 mousePx, bool snap) {
    if (stage_ == SketchStage::None || stage_ == SketchStage::Applied) return;

    if (stage_ == SketchStage::SelectPlane) {
        const Ray ray = camera.rayThroughPixel(static_cast<float>(mousePx.x),
                                               static_cast<float>(mousePx.y));
        const RayHit hit = scene.raycast(ray);
        if (hit.hit() && hit.face != kInvalid) {
            if (const SceneObject* o = scene.find(hit.object)) {
                const Mat4 model = o->modelMatrix();
                const Vec3 n = normalize(transformVector(normalMatrix(model),
                                                         o->body.faceNormal(hit.face)));
                std::vector<VertexId> fv;
                o->body.faceVertices(hit.face, fv);
                Vec3 centre{0, 0, 0};
                for (VertexId v : fv) centre += o->body.vertexPosition(v);
                if (!fv.empty()) centre *= 1.0 / static_cast<Real>(fv.size());
                plane_ = frameFor(transformPoint(model, centre), n);
                hoveredChoice_ = PlaneChoice::Face;
                faceObject_ = hit.object;
                return;
            }
        }
        // The origin planes, nearest first; the top plane when the pointer is
        // on none of them.
        const Real tile = std::max(camera.distance * 0.35, 25.0);
        struct Option { PlaneChoice choice; Vec3 normal; Vec3 a, b; };
        const Option options[3] = {
            {PlaneChoice::XY, {0, 0, 1}, {1, 0, 0}, {0, 1, 0}},
            {PlaneChoice::XZ, {0, -1, 0}, {1, 0, 0}, {0, 0, 1}},
            {PlaneChoice::YZ, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
        };
        Real bestT = 1e300;
        hoveredChoice_ = PlaneChoice::XY;
        for (const Option& op : options) {
            const Real denom = dot(op.normal, ray.dir);
            if (std::fabs(denom) < 1e-9) continue;
            const Real t = dot(-ray.origin, op.normal) / denom;
            if (t < 0.0 || t >= bestT) continue;
            const Vec3 p = ray.origin + ray.dir * t;
            if (std::fabs(dot(p, op.a)) > tile || std::fabs(dot(p, op.b)) > tile) continue;
            bestT = t;
            hoveredChoice_ = op.choice;
        }
        const Vec3 n = hoveredChoice_ == PlaneChoice::XZ ? Vec3{0, -1, 0}
                     : hoveredChoice_ == PlaneChoice::YZ ? Vec3{1, 0, 0}
                                                         : Vec3{0, 0, 1};
        plane_ = frameFor({0, 0, 0}, n);
        faceObject_ = kNoObject;
        return;
    }

    pickMm_ = kPointPickPx / pixelsPerMm(camera, plane_);
    Vec2 raw{};
    cursorValid_ = unproject(camera, mousePx, raw);
    // What the pointer is measured against below is what is drawn: kept
    // between frames, and only worked out again for what changed.
    if (stage_ == SketchStage::Draw || stage_ == SketchStage::Regions) refreshDrawCache(camera);

    if (stage_ == SketchStage::Draw) {
        // A drag owns the pointer while it lasts: the solver decides where the
        // geometry goes, so there is nothing to hover and nothing to snap to
        // except the grid the drag is being read off.
        if (drag_) {
            if (!cursorValid_) return;
            Vec2 at = raw;
            if (snap) {
                const Real step = static_cast<Real>(camera.snapStep(plane_.toWorld(raw)));
                if (step > 0.0) at = {std::round(at.x / step) * step, std::round(at.y / step) * step};
            }
            cursor_ = at;
            dragTo(at);
            return;
        }

        hoverPoint_ = hoverEntity_ = kNoSketchId;
        snap_ = PlaneSnap{};
        lock_ = Lock::None;
        if (!cursorValid_) return;

        // The sketch's own points first: a corner that is meant to close a
        // loop has to win over the grid line under it. Measured on the plane,
        // which the view is square to while drawing, so a millimetre there is
        // a fixed number of pixels -- and ten thousand points cost nothing to
        // look through. A curve's handle counts only where it is drawn.
        const Real mmPerPx = drawMmPerPx_;
        std::vector<SketchId> hiddenHandles;
        {
            size_t curves = 0;
            for (const SketchEntity& e : sketch_.entities) curves += e.curve == SketchCurve::Bezier;
            if (curves > 64)
                for (const SketchEntity& e : sketch_.entities)
                    if (e.curve == SketchCurve::Bezier && e.id != handleCurve_) {
                        hiddenHandles.push_back(e.b);
                        hiddenHandles.push_back(e.c);
                    }
            std::sort(hiddenHandles.begin(), hiddenHandles.end());
        }
        Real bestMm = kPointPickPx * mmPerPx;
        for (const SketchPoint& p : sketch_.points) {
            const Vec2 d = p.at - raw;
            if (std::fabs(d.x) > bestMm || std::fabs(d.y) > bestMm) continue;
            const Real dist = length(d);
            if (dist > bestMm) continue;
            if (std::binary_search(hiddenHandles.begin(), hiddenHandles.end(), p.id)) continue;
            bestMm = dist;
            hoverPoint_ = p.id;
        }

        Vec2 uv = raw;
        if (hoverPoint_ != kNoSketchId) {
            uv = pointUV(hoverPoint_);
        } else if (snap) {
            // Only the points near enough to be snapped to: the snapper looks
            // at each one it is handed, and a drawing hands it thousands.
            std::vector<SnapPoint> extra;
            const Real reach = 4.0 * kPointPickPx * mmPerPx;
            for (const SketchPoint& p : sketch_.points)
                if (std::fabs(p.at.x - raw.x) <= reach && std::fabs(p.at.y - raw.y) <= reach)
                    extra.push_back({plane_.toWorld(p.at), p.at, SnapKind::Vertex, kNoObject, 0.0});
            for (Vec2 c : clicks_)
                extra.push_back({plane_.toWorld(c), c, SnapKind::Vertex, kNoObject, 0.0});
            snap_ = snapOnPlane(scene, camera, plane_, mousePx, raw, {}, extra);
            if (snap_.valid()) uv = snap_.uv;
        }
        cursor_ = uv;

        if (mode_ == SketchMode::Line && clicks_.size() == 1) {
            const Vec2 d = cursor_ - clicks_.front();
            if (hoverPoint_ != kNoSketchId) {
                if (std::fabs(d.y) < 1e-9 && std::fabs(d.x) > 1e-9) lock_ = Lock::Horizontal;
                else if (std::fabs(d.x) < 1e-9 && std::fabs(d.y) > 1e-9) lock_ = Lock::Vertical;
            } else if (length(d) > 1e-9) {
                const Real deg = std::atan2(std::fabs(d.y), std::fabs(d.x)) * kRad2Deg;
                if (deg < kLockDegrees)              lock_ = Lock::Horizontal;
                else if (deg > 90.0 - kLockDegrees)  lock_ = Lock::Vertical;
            }
        }

        // A radius lands on a number someone would choose, unless the rim has
        // caught something real.
        if (snap && hoverPoint_ == kNoSketchId && !clicks_.empty() && !fixed_[0] &&
            (mode_ == SketchMode::Circle || (mode_ == SketchMode::Arc && clicks_.size() == 1)) &&
            (!snap_.valid() || snap_.kind == SnapKind::GridPoint || snap_.kind == SnapKind::GridLine)) {
            const Real step = static_cast<Real>(camera.snapStep(plane_.toWorld(clicks_.front())));
            const Vec2 d = cursor_ - clicks_.front();
            const Real r = length(d);
            if (step > 0.0 && r > 1e-9) {
                const Real q = std::max(std::round(r / step) * step, step);
                cursor_ = clicks_.front() + d * (q / r);
            }
        }

        // What the pointer is over, for dimensioning, deleting, construction:
        // the drawn line of each curve, skipped whole when its box is not
        // within reach.
        Real bestEntity = kEntityPickPx * mmPerPx;
        for (const EntityDraw& d : entityDraw_) {
            if (raw.x < d.lo.x - bestEntity || raw.x > d.hi.x + bestEntity ||
                raw.y < d.lo.y - bestEntity || raw.y > d.hi.y + bestEntity)
                continue;
            for (size_t i = 0; i + 1 < d.line.size(); ++i) {
                const Real dist = distanceToSegment(raw, d.line[i], d.line[i + 1]);
                if (dist <= bestEntity) { bestEntity = dist; hoverEntity_ = d.id; }
            }
        }
        if (hoverEntity_ != kNoSketchId) handleCurve_ = hoverEntity_;
        return;
    }

    if (stage_ == SketchStage::Regions) {
        hoverRegion_ = kNoSketchId;
        if (!cursorValid_) return;
        // The innermost region under the pointer: a boss drawn inside a hole
        // is its own region, and is the one being pointed at.
        Real smallest = 1e300;
        for (const SketchProfile& r : regions_) {
            if (r.area < smallest && regionHas(r, raw)) {
                smallest = r.area;
                hoverRegion_ = r.key;
            }
        }
        return;
    }

    if (stage_ == SketchStage::Depth) {
        if (!depthTyped_) {
            Real d = depthFromPointer(camera, mousePx);
            if (snap) {
                const Real step = static_cast<Real>(camera.snapStep(plane_.toWorld(chosenCentre())));
                if (step > 0.0) d = std::round(d / step) * step;
            }
            if (std::fabs(d) > 1e-4) depth_ = d;
        }
        choice_.follow(depth_, owner() != kNoObject);
        refreshReach(scene);
    }
}

void SketchTool::handleMouseDown(Scene& scene, Camera& camera, UndoStack& undo) {
    switch (stage_) {
    case SketchStage::SelectPlane:
        setPlane(plane_, hoveredChoice_ == PlaneChoice::Face ? faceObject_ : kNoObject, &camera);
        break;
    case SketchStage::Draw:
        if (mode_ == SketchMode::Select) {
            // Take hold of whatever is under the pointer: a point moves, the
            // rim of a circle or an arc resizes it. Clicking a line picks the
            // number that sizes it, if it has one, so it can be typed over.
            if (hoverPoint_ != kNoSketchId) {
                beginDrag(hoverPoint_);
            } else if (const SketchEntity* e = sketch_.entity(hoverEntity_)) {
                if (e->curve == SketchCurve::Circle || e->curve == SketchCurve::Arc)
                    beginRadiusDrag(e->id);
                activeDim_ = existingDimension(e->id);
            } else {
                activeDim_ = kNoSketchId;
            }
        } else if (mode_ == SketchMode::Dimension) {
            if (hoverEntity_ != kNoSketchId) dimensionEntity(hoverEntity_);
            else activeDim_ = kNoSketchId;
        } else {
            commitPending();
        }
        break;
    case SketchStage::Regions:
        if (hoverRegion_ != kNoSketchId) toggleRegion(hoverRegion_);
        break;
    case SketchStage::Depth:
        finish(scene, camera, undo, true);
        break;
    case SketchStage::Applied:
    case SketchStage::None:
        break;
    }
}

void SketchTool::handleMouseUp() {
    endDrag();
}

void SketchTool::handleRightClick(Camera& camera) {
    // A drag let go of by the other button still ends where it stands: the
    // constraints decided every position it passed through, so there is no
    // half-applied state to take back.
    endDrag();
    switch (stage_) {
    case SketchStage::Draw:
        clearPending();
        break;
    case SketchStage::Regions:
        stage_ = SketchStage::Draw;
        squareUp(camera);
        break;
    case SketchStage::Depth:
        stage_ = SketchStage::Regions;
        break;
    case SketchStage::SelectPlane:
        cancel(camera);
        break;
    case SketchStage::Applied:
    case SketchStage::None:
        break;
    }
}

bool SketchTool::handleKey(int key, bool shift, bool ctrl, Scene& scene, Camera& camera,
                           UndoStack& undo) {
    (void)shift;
    if (stage_ == SketchStage::None) return false;
    // Anything typed ends a drag where it stands, so no key can edit the sketch
    // out from under the solver holding it.
    endDrag();

    if (key == 27) {
        if (!typed_.empty()) {
            typed_.clear();
            fixed_[0] = fixed_[1] = false;
            depthTyped_ = false;
            return true;
        }
        switch (stage_) {
        case SketchStage::SelectPlane:
            cancel(camera);
            break;
        case SketchStage::Draw:
            // A sketch is more work than a profile, so one stray Escape does not
            // throw it away: the first ends what is being drawn, and leaving
            // with drawing in hand takes a second.
            if (pending()) clearPending();
            else if (activeDim_ != kNoSketchId) activeDim_ = kNoSketchId;
            else if (sketch_.empty() || escapeArmed_) cancel(camera);
            else {
                escapeArmed_ = true;
                error_ = editing() ? "Press Esc again to leave without keeping the changes"
                                   : "Press Esc again to discard this sketch";
            }
            break;
        case SketchStage::Regions:
            stage_ = SketchStage::Draw;
            squareUp(camera);
            break;
        case SketchStage::Depth:
            stage_ = SketchStage::Regions;
            break;
        case SketchStage::Applied:
        case SketchStage::None:
            break;
        }
        return true;
    }

    switch (stage_) {
    case SketchStage::SelectPlane:
        if (key == '7') { choosePlane(PlaneChoice::XY, camera); return true; }
        if (key == '1') { choosePlane(PlaneChoice::XZ, camera); return true; }
        if (key == '3') { choosePlane(PlaneChoice::YZ, camera); return true; }
        return false;

    case SketchStage::Draw:
        if (ctrl && key == 'Z') { undoEdit(); return true; }
        if (ctrl) return false;
        if (key == 'S') { setMode(SketchMode::Select); return true; }
        if (key == 'L') { setMode(SketchMode::Line); return true; }
        if (key == 'R') { setMode(SketchMode::Rectangle); return true; }
        if (key == 'C') { setMode(SketchMode::Circle); return true; }
        if (key == 'A') { setMode(SketchMode::Arc); return true; }
        if (key == 'D') { setMode(SketchMode::Dimension); return true; }
        if (key == 'Q') { toggleConstruction(hoverEntity_); return true; }
        if (key == 'K') { finish(scene, camera, undo, false); return true; }
        if (key == 127 || key == 'X') { deleteEntity(hoverEntity_); return true; }
        if (key == 13) {
            if (mode_ == SketchMode::Dimension && activeDim_ != kNoSketchId && !typed_.empty()) {
                Real v = 0.0;
                if (parseNumber(typed_, v)) {
                    const SketchConstraint* k = sketch_.constraint(activeDim_);
                    setDimension(activeDim_, k && k->rule == SketchRule::Angle ? v * kDeg2Rad : v);
                }
                typed_.clear();
                return true;
            }
            if (pending()) {
                // Enter on a line chain with nothing typed ends the chain.
                if (mode_ == SketchMode::Line && typed_.empty()) clearPending();
                else commitPending();
                return true;
            }
            if (editing()) finish(scene, camera, undo, false);
            else beginExtrude(&camera);
            return true;
        }
        if (key == 'E') {
            beginExtrude(&camera);
            return true;
        }
        return typeKey(key);

    case SketchStage::Regions:
        if (key == 13 || key == 'E') { beginDepth(); return true; }
        return false;

    case SketchStage::Depth:
        if (key == 13 || key == 'E') { finish(scene, camera, undo, true); return true; }
        {
            // Picking one ends the depth's say in it. A sketch already in a
            // part extrudes into that part, so it has no New Body.
            ExtrudeOp picked;
            if (extrudeOpForKey(key, picked) && !(picked == ExtrudeOp::NewBody && editing())) {
                choice_.pick(picked);
                refreshReach(scene);
                return true;
            }
        }
        return typeKey(key);

    case SketchStage::Applied:
    case SketchStage::None:
        break;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Drawing it
// ---------------------------------------------------------------------------

// Whether a disagreement runs through this entity: either it is named by a
// conflicting constraint, or one of its points is.
bool SketchTool::inConflict(const SketchEntity& e) const {
    if (solved_.conflicting.empty()) return false;
    for (SketchId id : solved_.conflicting) {
        const SketchConstraint* k = sketch_.constraint(id);
        if (!k) continue;
        for (SketchId named : {k->first, k->second}) {
            if (named == kNoSketchId) continue;
            if (named == e.id) return true;
            if (named == e.a || named == e.b || named == e.c || named == e.d) return true;
        }
    }
    return false;
}

void SketchTool::refreshDrawCache(const Camera& camera) const {
    const Real ppm = pixelsPerMm(camera, plane_);
    drawMmPerPx_ = 1.0 / ppm;
    // Half-octave steps of zoom: fine enough that a curve never shows its
    // corners, coarse enough that zooming does not re-sample every frame.
    drawBucket_ = static_cast<int>(std::lround(2.0 * std::log2(std::max(ppm, 1e-9))));
    const Real bucketPpm = std::pow(2.0, drawBucket_ * 0.5);

    std::vector<EntityDraw> next;
    next.reserve(sketch_.entities.size());
    std::unordered_map<SketchId, size_t> at;
    at.reserve(sketch_.entities.size());
    for (const SketchEntity& e : sketch_.entities) {
        EntityDraw d;
        d.id = e.id;
        d.curve = e.curve;
        d.radius = e.radius;
        const SketchId ids[4] = {e.a, e.b, e.c, e.d};
        for (int i = 0; i < 4; ++i) d.ctrl[i] = ids[i] != kNoSketchId ? pointUV(ids[i]) : Vec2{};

        const auto old = entityDrawAt_.find(e.id);
        bool same = false;
        if (old != entityDrawAt_.end()) {
            const EntityDraw& was = entityDraw_[old->second];
            same = was.curve == d.curve && was.radius == d.radius &&
                   (d.curve == SketchCurve::Line || was.bucket == drawBucket_);
            for (int i = 0; i < 4 && same; ++i)
                same = was.ctrl[i].x == d.ctrl[i].x && was.ctrl[i].y == d.ctrl[i].y;
        }
        if (same) {
            next.push_back(std::move(entityDraw_[old->second]));
        } else {
            // As many pieces as it needs on screen: about one per five pixels
            // of its length, a line one.
            Real extent = 0;
            switch (e.curve) {
            case SketchCurve::Line:   break;
            case SketchCurve::Circle: extent = kTwoPi * e.radius; break;
            case SketchCurve::Arc:    extent = kTwoPi * length(d.ctrl[1] - d.ctrl[0]); break;
            case SketchCurve::Bezier:
                extent = length(d.ctrl[1] - d.ctrl[0]) + length(d.ctrl[2] - d.ctrl[1]) +
                         length(d.ctrl[3] - d.ctrl[2]);
                break;
            }
            const int steps = static_cast<int>(std::clamp(std::ceil(extent * bucketPpm / 5.0), 4.0, 64.0));
            d.line = sketchEntityPoints(sketch_, e, steps);
            d.lo = {1e300, 1e300};
            d.hi = {-1e300, -1e300};
            for (Vec2 p : d.line) {
                d.lo = {std::min(d.lo.x, p.x), std::min(d.lo.y, p.y)};
                d.hi = {std::max(d.hi.x, p.x), std::max(d.hi.y, p.y)};
            }
            d.bucket = drawBucket_;
            d.version = ++drawVersion_;
            next.push_back(std::move(d));
        }
        at.emplace(e.id, next.size() - 1);
    }
    entityDraw_.swap(next);
    entityDrawAt_.swap(at);
    if (regionDraw_.size() > regions_.size() * 2 + 16) regionDraw_.clear();
}

const SketchTool::EntityDraw* SketchTool::drawOf(SketchId entity) const {
    const auto it = entityDrawAt_.find(entity);
    return it == entityDrawAt_.end() ? nullptr : &entityDraw_[it->second];
}

const SketchTool::RegionDraw& SketchTool::regionDrawOf(const SketchProfile& region) const {
    // Which curves, in which order, at which versions: anything else the same
    // is the same outline.
    uint64_t sig = 1469598103934665603ull;
    auto mix = [&sig](uint64_t v) { sig = (sig ^ v) * 1099511628211ull; };
    std::vector<const SketchLoop*> loops{&region.outer};
    for (const SketchLoop& h : region.holes) loops.push_back(&h);
    for (const SketchLoop* loop : loops) {
        mix(0xABCDu);
        for (size_t k = 0; k < loop->entities.size(); ++k) {
            const EntityDraw* d = drawOf(loop->entities[k]);
            mix(loop->entities[k]);
            mix(d ? d->version : 0u);
            mix(k < loop->reversed.size() && loop->reversed[k] ? 1u : 0u);
        }
    }
    RegionDraw& r = regionDraw_[region.key];
    if (r.signature == sig && !r.loops.empty()) return r;

    r = RegionDraw{};
    r.signature = sig;
    r.lo = {1e300, 1e300};
    r.hi = {-1e300, -1e300};
    for (const SketchLoop* loop : loops) {
        std::vector<Vec2> poly;
        for (size_t k = 0; k < loop->entities.size(); ++k) {
            const EntityDraw* d = drawOf(loop->entities[k]);
            if (!d || d->line.empty()) continue;
            const bool rev = k < loop->reversed.size() && loop->reversed[k];
            const size_t n = d->line.size();
            for (size_t i = poly.empty() ? 0 : 1; i < n; ++i) poly.push_back(d->line[rev ? n - 1 - i : i]);
        }
        if (poly.size() > 1 && lengthSq(poly.front() - poly.back()) < 1e-18) poly.pop_back();
        for (Vec2 p : poly) {
            r.lo = {std::min(r.lo.x, p.x), std::min(r.lo.y, p.y)};
            r.hi = {std::max(r.hi.x, p.x), std::max(r.hi.y, p.y)};
        }
        r.loops.push_back(std::move(poly));
    }
    return r;
}

const std::vector<Vec2>& SketchTool::hatchOf(const SketchProfile& region, Real spacingPx, bool fine) const {
    RegionDraw& r = const_cast<RegionDraw&>(regionDrawOf(region));
    // Spaced by the zoom step rather than the zoom itself, so it holds still
    // between steps instead of being worked out again on every frame of a zoom.
    const Real mm = spacingPx / std::pow(2.0, drawBucket_ * 0.5);
    Real& have = fine ? r.fineMm : r.coarseMm;
    std::vector<Vec2>& strokes = fine ? r.fine : r.coarse;
    if (have != mm) {
        strokes = hatchStrokes(r.loops, mm);
        have = mm;
    }
    return strokes;
}

bool SketchTool::regionHas(const SketchProfile& region, Vec2 uv) const {
    const RegionDraw& r = regionDrawOf(region);
    if (r.loops.empty() || uv.x < r.lo.x || uv.x > r.hi.x || uv.y < r.lo.y || uv.y > r.hi.y) return false;
    if (!insidePolygon(uv, r.loops.front())) return false;
    for (size_t i = 1; i < r.loops.size(); ++i)
        if (insidePolygon(uv, r.loops[i])) return false;
    return true;
}

void SketchTool::drawOverlay(const Scene& scene, const Camera& camera, Renderer& renderer) const {
    if (stage_ == SketchStage::None || stage_ == SketchStage::Applied) return;
    const Vec4 brand = toVec4(palette::kBrand, 0.95f);
    const Vec4 lit{1.0f, 0.82f, 0.35f, 1.0f};

    if (stage_ == SketchStage::SelectPlane) {
        const Real sz = std::max(camera.distance * 0.35, 25.0);
        struct Tile { PlaneChoice choice; Vec3 a, b; Vec4 tint; };
        const Tile tiles[3] = {
            {PlaneChoice::XY, {1, 0, 0}, {0, 1, 0}, {0.2f, 0.6f, 0.9f, 0.4f}},
            {PlaneChoice::XZ, {1, 0, 0}, {0, 0, 1}, {0.3f, 0.8f, 0.4f, 0.4f}},
            {PlaneChoice::YZ, {0, 1, 0}, {0, 0, 1}, {0.9f, 0.4f, 0.3f, 0.4f}},
        };
        for (const Tile& t : tiles) {
            const Vec4 edge = hoveredChoice_ == t.choice ? brand : t.tint;
            const Vec3 c[4] = {(-t.a - t.b) * sz, (t.a - t.b) * sz, (t.a + t.b) * sz, (t.b - t.a) * sz};
            for (int i = 0; i < 4; ++i) renderer.addLine(c[i], c[(i + 1) % 4], edge);
            const Vec4 fill{t.tint.x, t.tint.y, t.tint.z, 0.08f};
            renderer.addTriangle(c[0], c[1], c[2], fill);
            renderer.addTriangle(c[0], c[2], c[3], fill);
        }
        if (hoveredChoice_ == PlaneChoice::Face && faceObject_ != kNoObject) {
            const Real s = std::max(camera.distance * 0.08, 4.0);
            const Vec3 c[4] = {plane_.origin + (-plane_.u - plane_.v) * s,
                               plane_.origin + (plane_.u - plane_.v) * s,
                               plane_.origin + (plane_.u + plane_.v) * s,
                               plane_.origin + (plane_.v - plane_.u) * s};
            for (int i = 0; i < 4; ++i) renderer.addFrontLine(camera, c[i], c[(i + 1) % 4], brand, 2.0);
            renderer.addFrontLine(camera, plane_.origin, plane_.origin + plane_.normal * s * 1.5, brand, 2.0);
        }
        (void)scene;
        return;
    }

    // The grid the pointer lands on, as the create tool draws it.
    const Real span = std::max<Real>(camera.distance * 1.6, 20.0);
    const GridLevels levels = gridLevelsAt(camera, plane_.origin);
    auto drawLevel = [&](Real step, Vec4 col) {
        if (step <= 0.0) return false;
        const int n = static_cast<int>(span / step);
        if (n > 240) return false;
        for (int i = -n; i <= n; ++i) {
            const Real t = static_cast<Real>(i) * step;
            renderer.addLine(plane_.origin + plane_.u * t - plane_.v * span,
                             plane_.origin + plane_.u * t + plane_.v * span, col);
            renderer.addLine(plane_.origin - plane_.u * span + plane_.v * t,
                             plane_.origin + plane_.u * span + plane_.v * t, col);
        }
        return true;
    };
    if (stage_ == SketchStage::Draw) {
        const Real ladder[3] = {levels.main, levels.major, levels.major * 10.0};
        for (int i = 0; i < 2; ++i)
            if (drawLevel(ladder[i], Vec4{0.45f, 0.55f, 0.65f, 0.13f})) {
                drawLevel(ladder[i + 1], Vec4{0.50f, 0.60f, 0.70f, 0.28f});
                break;
            }
        renderer.addLine(plane_.origin - plane_.u * span, plane_.origin + plane_.u * span,
                         Vec4{0.9f, 0.3f, 0.3f, 0.4f});
        renderer.addLine(plane_.origin - plane_.v * span, plane_.origin + plane_.v * span,
                         Vec4{0.3f, 0.8f, 0.3f, 0.4f});
        drawSnapIndicator(renderer, camera, snap_);
    }

    refreshDrawCache(camera);
    const Real mmPerPx = drawMmPerPx_;

    // Lines and points lie in the sketch's plane, so they are drawn as flat
    // strips and hexagons in it: a width in pixels is a width in millimetres
    // here, with no per-segment turn towards the eye. That is what lets a
    // drawing of thousands of curves be drawn every frame.
    auto flatLine = [&](Vec2 a, Vec2 b, Vec4 col, Real widthPx) {
        const Vec2 d = b - a;
        const Real len = length(d);
        if (len < 1e-12) return;
        const Vec2 n = Vec2{-d.y, d.x} * (widthPx * 0.5 * mmPerPx / len);
        const Vec3 a0 = plane_.toWorld(a - n), a1 = plane_.toWorld(a + n);
        const Vec3 b0 = plane_.toWorld(b - n), b1 = plane_.toWorld(b + n);
        renderer.addFrontTriangle(a0, b0, b1, col);
        renderer.addFrontTriangle(a0, b1, a1, col);
    };
    auto flatDashes = [&](Vec2 a, Vec2 b, Vec4 col, Real widthPx, Real dashPx) {
        const Vec2 d = b - a;
        const Real len = length(d);
        const Real period = 2.0 * dashPx * mmPerPx;
        if (len < 1e-12 || period <= 0.0) return;
        const int count = std::min(static_cast<int>(len / period) + 1, 64);
        for (int i = 0; i < count; ++i) {
            const Real s0 = i * period, s1 = std::min(s0 + period * 0.5, len);
            if (s1 > s0) flatLine(a + d * (s0 / len), a + d * (s1 / len), col, widthPx);
        }
    };
    static const Vec2 kHex[6] = {{1, 0}, {0.5, 0.8660254}, {-0.5, 0.8660254},
                                 {-1, 0}, {-0.5, -0.8660254}, {0.5, -0.8660254}};
    auto flatDot = [&](Vec2 c, Real radiusPx, Vec4 col) {
        const Real r = radiusPx * mmPerPx;
        const Vec3 mid = plane_.toWorld(c);
        for (int i = 0; i < 6; ++i)
            renderer.addFrontTriangle(mid, plane_.toWorld(c + kHex[i] * r),
                                      plane_.toWorld(c + kHex[(i + 1) % 6] * r), col);
    };
    auto hatch = [&](const SketchProfile& r, Real spacingPx, bool fine, Vec4 col) {
        const std::vector<Vec2>& strokes = hatchOf(r, spacingPx, fine);
        for (size_t i = 0; i + 1 < strokes.size(); i += 2)
            renderer.addLine(plane_.toWorld(strokes[i]), plane_.toWorld(strokes[i + 1]), col);
    };

    // Regions: every one faintly while choosing, the chosen ones plainly.
    if (stage_ == SketchStage::Regions || stage_ == SketchStage::Depth) {
        std::vector<SketchId> chosen = chosen_;
        std::sort(chosen.begin(), chosen.end());
        for (const SketchProfile& r : regions_) {
            const bool on = std::binary_search(chosen.begin(), chosen.end(), r.key);
            const bool hovered = stage_ == SketchStage::Regions && hoverRegion_ == r.key;
            if (!on && !hovered && stage_ == SketchStage::Depth) continue;
            Vec4 col = on ? Vec4{0.20f, 0.60f, 0.95f, 0.55f} : Vec4{0.6f, 0.65f, 0.7f, 0.25f};
            if (hovered) col = Vec4{lit.x, lit.y, lit.z, 0.6f};
            const bool fine = on || hovered;
            hatch(r, fine ? 6.0 : 12.0, fine, col);
        }
    } else if (stage_ == SketchStage::Draw) {
        for (const SketchProfile& r : regions_) hatch(r, 14.0, false, Vec4{0.20f, 0.60f, 0.95f, 0.16f});
    }

    // The geometry itself, in the colour of what is still free to move: blue
    // for geometry a dimension could still shift, green for geometry that is
    // pinned down, and the brand red for geometry a disagreement runs through.
    // Editing a sketch is largely deciding which of those you are looking at.
    for (const SketchEntity& e : sketch_.entities) {
        const EntityDraw* d = drawOf(e.id);
        if (!d) continue;
        const bool hovered = stage_ == SketchStage::Draw && hoverEntity_ == e.id;
        Vec4 col = sortedHas(solved_.freeEntities, e.id) ? kFree : kFixed;
        if (inConflict(e)) col = brand;
        if (e.construction) col = Vec4{col.x * 0.75f, col.y * 0.75f, col.z * 0.8f, 0.75f};
        if (hovered) col = lit;
        for (size_t i = 0; i + 1 < d->line.size(); ++i) {
            if (e.construction) flatDashes(d->line[i], d->line[i + 1], col, hovered ? 2.4 : 1.6, 4.0);
            else                flatLine(d->line[i], d->line[i + 1], col, hovered ? 3.0 : 2.0);
        }
    }
    if (stage_ == SketchStage::Draw) {
        // A curve's handles, as every drawing program shows them: a thin line
        // from each end to the point that steers it there. With a few curves
        // every one shows them; in an imported drawing of hundreds only the
        // curve under the pointer does, or the thicket would hide the drawing.
        size_t curves = 0;
        for (const SketchEntity& e : sketch_.entities) curves += e.curve == SketchCurve::Bezier;
        const bool allHandles = curves <= 64;
        std::vector<SketchId> handles, shown;
        for (const SketchEntity& e : sketch_.entities) {
            if (e.curve != SketchCurve::Bezier) continue;
            handles.push_back(e.b);
            handles.push_back(e.c);
            const bool near = allHandles || hoverEntity_ == e.id || handleCurve_ == e.id ||
                              (hoverPoint_ != kNoSketchId &&
                               (hoverPoint_ == e.a || hoverPoint_ == e.b || hoverPoint_ == e.c || hoverPoint_ == e.d)) ||
                              (dragPoint_ != kNoSketchId && (dragPoint_ == e.b || dragPoint_ == e.c));
            if (!near) continue;
            shown.push_back(e.b);
            shown.push_back(e.c);
            const Vec4 col{kFree.x, kFree.y, kFree.z, 0.45f};
            flatDashes(pointUV(e.a), pointUV(e.b), col, 1.0, 2.0);
            flatDashes(pointUV(e.d), pointUV(e.c), col, 1.0, 2.0);
        }
        std::sort(handles.begin(), handles.end());
        std::sort(shown.begin(), shown.end());
        // A point is drawn where there is room to tell it from its neighbours:
        // on a curve at least a dozen pixels across. Zoomed out on a drawing of
        // hundreds of letters, every letter would otherwise be a blot of dots.
        std::vector<SketchId> roomy;
        for (const SketchEntity& e : sketch_.entities) {
            const EntityDraw* d = drawOf(e.id);
            if (!d || length(d->hi - d->lo) < 12.0 * mmPerPx) continue;
            for (SketchId id : {e.a, e.b, e.c, e.d})
                if (id != kNoSketchId) roomy.push_back(id);
        }
        std::sort(roomy.begin(), roomy.end());
        for (const SketchPoint& p : sketch_.points) {
            const bool on = hoverPoint_ == p.id || dragPoint_ == p.id;
            const bool handle = std::binary_search(handles.begin(), handles.end(), p.id);
            if (handle && !on && !std::binary_search(shown.begin(), shown.end(), p.id)) continue;
            if (!on && !std::binary_search(roomy.begin(), roomy.end(), p.id)) continue;
            const bool free = sortedHas(solved_.freePoints, p.id);
            flatDot(p.at, on ? 4.5 : handle ? 1.8 : 2.6, on ? lit : (free ? kFree : kFixed));
        }
    }

    // What is being drawn.
    if (stage_ == SketchStage::Draw && !clicks_.empty() && cursorValid_) {
        const Vec4 ghost{brand.x, brand.y, brand.z, 0.7f};
        const Vec2 s = clicks_.front();
        const Vec2 e = previewEnd();
        auto line = [&](Vec2 a, Vec2 b) {
            renderer.addFrontLine(camera, plane_.toWorld(a), plane_.toWorld(b), ghost, 2.0);
        };
        auto circle = [&](Vec2 c, Real r, bool dashed) {
            const int n = 72;
            for (int i = 0; i < n; ++i) {
                const Real t0 = kTwoPi * i / n, t1 = kTwoPi * (i + 1) / n;
                const Vec3 a = plane_.toWorld({c.x + r * std::cos(t0), c.y + r * std::sin(t0)});
                const Vec3 b = plane_.toWorld({c.x + r * std::cos(t1), c.y + r * std::sin(t1)});
                if (dashed) renderer.addFrontDashes(camera, a, b, ghost, 1.2, 3.0, 3.0);
                else        renderer.addFrontLine(camera, a, b, ghost, 2.0);
            }
        };
        switch (mode_) {
        case SketchMode::Line:
            line(s, e);
            if (lock_ != Lock::None) {
                const Vec3 a = plane_.toWorld(s);
                const Vec3 dir = lock_ == Lock::Horizontal ? plane_.u : plane_.v;
                renderer.addFrontDashes(camera, a - dir * span, a + dir * span,
                                        Vec4{lit.x, lit.y, lit.z, 0.45f}, 1.2);
            }
            break;
        case SketchMode::Rectangle:
            line(s, {e.x, s.y});
            line({e.x, s.y}, e);
            line(e, {s.x, e.y});
            line({s.x, e.y}, s);
            break;
        case SketchMode::Circle:
            circle(s, length(e - s), false);
            line(s, e);
            break;
        case SketchMode::Arc:
            if (clicks_.size() == 1) {
                circle(s, length(e - s), true);
                line(s, e);
            } else {
                const Vec2 from = clicks_[1];
                const Real r = length(from - s);
                Real a0 = std::atan2(from.y - s.y, from.x - s.x);
                Real a1 = std::atan2(e.y - s.y, e.x - s.x);
                const Vec2 ds = from - s, de = e - s;
                if (ds.x * de.y - ds.y * de.x < 0.0) std::swap(a0, a1);
                if (a1 <= a0) a1 += kTwoPi;
                const int n = 48;
                for (int i = 0; i < n; ++i) {
                    const Real t0 = a0 + (a1 - a0) * i / n, t1 = a0 + (a1 - a0) * (i + 1) / n;
                    line({s.x + r * std::cos(t0), s.y + r * std::sin(t0)},
                         {s.x + r * std::cos(t1), s.y + r * std::sin(t1)});
                }
            }
            break;
        case SketchMode::Select:
        case SketchMode::Dimension:
            break;
        }
        for (Vec2 c : clicks_) {
            const Vec3 at = plane_.toWorld(c);
            overlay::disc(renderer, at, overlay::frameAt(camera, at), 3.2, ghost);
        }
    }

    // How far it goes.
    if (stage_ == SketchStage::Depth && std::fabs(depth_) > 1e-4) {
        const Vec3 lift = plane_.normal * depth_;
        const bool removes = choice_.op == ExtrudeOp::Cut || choice_.op == ExtrudeOp::Intersect;
        const Vec4 col = removes ? Vec4{1.0f, 0.4f, 0.3f, 0.9f} : brand;
        // Thick lines for a few regions; thin ones, which cost nothing to
        // place, for a drawing's worth.
        const bool many = chosen_.size() > 24;
        auto edge = [&](Vec3 a, Vec3 b, Real w) {
            if (many) renderer.addLine(a, b, col);
            else      renderer.addFrontLine(camera, a, b, col, w);
        };
        for (const SketchProfile& r : regions_) {
            if (!contains(chosen_, r.key)) continue;
            std::vector<const SketchLoop*> loops{&r.outer};
            for (const SketchLoop& h : r.holes) loops.push_back(&h);
            for (const SketchLoop* loop : loops) {
                for (SketchId id : loop->entities) {
                    const EntityDraw* d = drawOf(id);
                    if (!d) continue;
                    const std::vector<Vec2>& pts = d->line;
                    const size_t every = std::max<size_t>(pts.size() / 4, 1);
                    for (size_t i = 0; i < pts.size(); ++i) {
                        const Vec3 a = plane_.toWorld(pts[i]);
                        if (i + 1 < pts.size()) edge(a + lift, plane_.toWorld(pts[i + 1]) + lift, 2.0);
                        if (i == 0 || (d->curve != SketchCurve::Line && i % every == 0 && i + 1 < pts.size()))
                            edge(a, a + lift, 1.4);
                    }
                }
            }
        }
    }
}

// The drawing just imported: how big it is, where its middle is, which way
// round. Sizes are of the whole drawing, and keep its proportions.
void SketchTool::drawPlacementRows() {
    SvgPlacement at = svgPlace_;
    bool changed = false;
    const bool across = svgPlace_.quarterTurns % 2 != 0;
    const Vec2 base = across ? Vec2{svg_.size.y, svg_.size.x} : svg_.size;

    ui::commandValue("Drawing", svgName_.empty() ? "SVG" : svgName_.c_str());
    auto sizeRow = [&](const char* label, Real natural, const char* id) {
        if (natural < 1e-9) return;
        ui::commandRow(label);
        double v = natural * at.scale;
        ImGui::SetNextItemWidth(-1.0f);
        ImGui::InputDouble(id, &v, 0.0, 0.0, "%.2f mm");
        if (ImGui::IsItemDeactivatedAfterEdit() && v > 1e-6) {
            at.scale = v / natural;
            changed = true;
        }
    };
    sizeRow("Width", base.x, "##svgw");
    sizeRow("Height", base.y, "##svgh");

    // Which colours become the part. A pill each, lit when it is ink; the
    // swatch beside it is the colour itself.
    if (svgDrawing_.colours.size() > 1) {
        ui::commandRow("Colours");
        std::vector<bool> ink;
        for (const SvgColour& c : svgDrawing_.colours) ink.push_back(c.ink);
        int toggled = -1;
        const float right = ImGui::GetWindowContentRegionMax().x;
        for (size_t i = 0; i < svgDrawing_.colours.size(); ++i) {
            const SvgColour& c = svgDrawing_.colours[i];
            const float w = ImGui::CalcTextSize(c.css.c_str()).x + 44.0f;
            if (i > 0) {
                ImGui::SameLine(0.0f, 6.0f);
                if (ImGui::GetCursorPosX() + w > right) {
                    ImGui::NewLine();
                    ImGui::SetCursorPosX(ui::commandLabelWidth());
                }
            }
            ImGui::PushID(static_cast<int>(i));
            ImGui::ColorButton("##swatch",
                               ImVec4(static_cast<float>(c.rgb.x), static_cast<float>(c.rgb.y),
                                      static_cast<float>(c.rgb.z), 1.0f),
                               ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoPicker |
                                   ImGuiColorEditFlags_NoDragDrop,
                               ImVec2(ImGui::GetFrameHeight() * 0.6f, ImGui::GetFrameHeight()));
            ImGui::SameLine(0.0f, 2.0f);
            if (ui::pillButton(c.css.c_str(), c.ink)) toggled = static_cast<int>(i);
            ui::hoverTip(c.ink ? "Ink: part of the part. Click to make it paper."
                               : "Paper: painted in it wipes ink out. Click to make it ink.");
            ImGui::PopID();
        }
        if (toggled >= 0) {
            ink[static_cast<size_t>(toggled)] = !ink[static_cast<size_t>(toggled)];
            setInkColours(ink);
        }
    }

    ui::commandRow("Centre");
    double c[2] = {at.centre.x, at.centre.y};
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::InputScalarN("##svgc", ImGuiDataType_Double, c, 2, nullptr, nullptr, "%.2f");
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        at.centre = {c[0], c[1]};
        changed = true;
    }

    ui::commandRow("");
    if (ui::quietButton("Turn 90\xC2\xB0")) {
        at.quarterTurns = (at.quarterTurns + 1) % 4;
        changed = true;
    }
    ui::hoverTip("A quarter turn anticlockwise, about its centre");
    ImGui::SameLine();
    if (ui::quietButton("Actual size")) {
        at.scale = 1.0;
        changed = true;
    }
    ui::hoverTip("The size the file says it is");
    ImGui::SameLine();
    if (ui::quietButton("Leave it")) endPlacing();
    ui::hoverTip("Stop placing it: it is ordinary sketch geometry from here on");

    if (changed) setPlacement(at);
}

void SketchTool::drawHud(Scene& scene, Camera& camera, UndoStack& undo, bool& finished) {
    finished = false;
    if (stage_ == SketchStage::None) return;

    const char* title = editing() ? "Edit Sketch" : "Sketch";
    if (!ui::beginCommand("##sketch", title, Glyph::Sketch)) return;

    // A number pulled on the panel goes in the way the keyboard would have put
    // it, one character at a time, so the two cannot disagree about what a
    // number means at this step.
    // How far a bar reaches: the view's own height, which is what the sketch
    // is being drawn in and does not move while the bar is pulled.
    const double extent = niceStepAbove(static_cast<double>(camera.orthoHeight()) * 0.5);

    auto pulled = [&](const ui::NumberEdit& e, int field) {
        if (e.dragged) {
            char b[48];
            std::snprintf(b, sizeof b, "%.6g", e.value);
            typed_.clear();
            typedField_ = field;
            for (const char* p = b; *p; ++p) typeKey(*p);
        } else if (e.clicked) {
            typedField_ = field;
            typed_.clear();
        }
    };

    int footer = 0;
    switch (stage_) {
    case SketchStage::SelectPlane: {
        ui::commandRow("Plane");
        if (ImGui::Button("Top")) choosePlane(PlaneChoice::XY, camera);
        ImGui::SameLine();
        if (ImGui::Button("Front")) choosePlane(PlaneChoice::XZ, camera);
        ImGui::SameLine();
        if (ImGui::Button("Right")) choosePlane(PlaneChoice::YZ, camera);
        if (importPending()) {
            char b[160];
            std::snprintf(b, sizeof b, "Where %s goes: a plane, or click a face to put it on.  %.1f x %.1f mm.",
                          pendingSvgName_.c_str(), pendingSvg_.size().x, pendingSvg_.size().y);
            ui::commandHint(b);
        } else {
            ui::commandHint("Or click a face of an object to sketch on it.  7 / 1 / 3 pick a plane.");
        }
        footer = ui::commandFooter(nullptr);
        break;
    }

    case SketchStage::Draw: {
        static const SketchMode kModeOf[6] = {
            SketchMode::Select, SketchMode::Line, SketchMode::Rectangle,
            SketchMode::Circle, SketchMode::Arc,  SketchMode::Dimension};
        static const ui::Choice kModes[6] = {
            {Glyph::Select,    "Select", "S", "Drag a point, or a rim, and the constraints hold  (S)"},
            {Glyph::Line,      "Line",   "L", "Click point to point; Enter ends the chain  (L)"},
            {Glyph::Rect,      "Rect",   "R", "Two opposite corners  (R)"},
            {Glyph::Circle,    "Circle", "C", "Centre, then a point on the rim  (C)"},
            {Glyph::Arc,       "Arc",    "A", "Centre, start, end  (A)"},
            {Glyph::Dimension, "Size",   "D", "Click a line or a circle to size it  (D)"},
        };
        int on = 0;
        for (int i = 0; i < 6; ++i) if (kModeOf[i] == mode_) on = i;
        const int pick = ui::commandChoices("", kModes, 6, on);
        if (pick >= 0) setMode(kModeOf[pick]);

        // The size of what is being drawn, the way the create tool shows it.
        if (!clicks_.empty() && !(mode_ == SketchMode::Arc && clicks_.size() == 2)) {
            const Vec2 s = clicks_.front(), e = previewEnd();
            if (mode_ == SketchMode::Rectangle) {
                const Real vals[2] = {std::fabs(e.x - s.x), std::fabs(e.y - s.y)};
                const char* names[2] = {"Width", "Height"};
                for (int f = 0; f < 2; ++f)
                    pulled(ui::commandNumber(names[f], vals[f], "mm", fixed_[f],
                                             f == typedField_ && !typed_.empty(), typed_.c_str(),
                                             0.0, std::max(extent, vals[f])), f);
            } else {
                const Real v = length(e - s);
                pulled(ui::commandNumber(mode_ == SketchMode::Line ? "Length" : "Radius", v, "mm",
                                         fixed_[0], !typed_.empty(), typed_.c_str(),
                                         0.0, std::max(extent, v)), 0);
            }
        }

        if (placing()) drawPlacementRows();

        // What the two colours in the viewport mean, said in the colours
        // themselves: a legend in grey text would need reading twice.
        ui::commandRow("State");
        if (sketch_.empty()) {
            ImGui::TextUnformatted("empty");
        } else if (solved_.freedoms == 0 && solved_.solved) {
            ImGui::TextColored(kFixedIm, "fully constrained");
        } else {
            ImGui::TextColored(kFreeIm, "%d %s free", solved_.freedoms,
                               solved_.freedoms == 1 ? "freedom" : "freedoms");
            if (!solved_.freeEntities.empty()) {
                ImGui::SameLine(0.0f, 6.0f);
                ImGui::TextDisabled("(%zu of %zu)", solved_.freeEntities.size(),
                                    sketch_.entities.size());
            }
        }

        // A disagreement, named. "Over-constrained" is not something a person
        // can act on; "the length and the radius disagree" is.
        if (!solved_.conflicting.empty()) {
            ui::commandRow("Disagree");
            std::string names;
            for (SketchId id : solved_.conflicting) {
                const SketchConstraint* k = sketch_.constraint(id);
                if (!names.empty()) names += " and ";
                names += std::string(k ? sketchRuleName(k->rule) : "constraint") + " #" +
                         std::to_string(id);
            }
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.35f, 0.25f, 1.0f));
            ImGui::TextWrapped("%s cannot both hold", names.c_str());
            ImGui::PopStyleColor();
        } else if (!solved_.redundant.empty()) {
            ui::commandRow("Repeated");
            std::string names;
            for (SketchId id : solved_.redundant) {
                if (!names.empty()) names += ", ";
                names += "#" + std::to_string(id);
            }
            ImGui::TextDisabled("%s say what the rest already does", names.c_str());
        }

        char regions[64];
        std::snprintf(regions, sizeof regions, "%zu closed", regions_.size());
        ui::commandValue("Regions", regions);

        // Every number that sizes the sketch, editable in place.
        bool anyDim = false;
        for (const SketchConstraint& k : sketch_.constraints) {
            if (!isDimension(k.rule)) continue;
            anyDim = true;
            char label[32];
            std::snprintf(label, sizeof label, "%s #%u",
                          k.rule == SketchRule::Radius ? "Radius"
                          : k.rule == SketchRule::Angle ? "Angle" : "Length", k.id);
            ui::commandRow(label);
            ImGui::PushID(static_cast<int>(k.id));
            const bool active = activeDim_ == k.id;
            const bool bad = contains(solved_.conflicting, k.id);
            const bool repeated = contains(solved_.redundant, k.id);
            if (bad)           ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.55f, 0.16f, 0.10f, 0.65f));
            else if (repeated) ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.45f, 0.35f, 0.10f, 0.55f));
            else if (active)   ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.20f, 0.40f, 0.62f, 0.55f));
            double v = k.rule == SketchRule::Angle ? k.value * kRad2Deg : k.value;
            ImGui::SetNextItemWidth(-1.0f);
            ImGui::InputDouble("##dim", &v, 0.0, 0.0,
                               k.rule == SketchRule::Angle ? "%.2f deg" : "%.3f mm");
            const SketchId kid = k.id;
            const SketchRule rule = k.rule;
            if (bad || repeated || active) ImGui::PopStyleColor();
            if (ImGui::IsItemActivated()) activeDim_ = kid;
            const bool edited = ImGui::IsItemDeactivatedAfterEdit();
            ImGui::PopID();
            if (edited) {
                setDimension(kid, rule == SketchRule::Angle ? v * kDeg2Rad : v);
                break;   // the constraints may have been replaced; draw them next frame
            }
        }
        if (!anyDim && !sketch_.empty())
            ui::commandHint("No dimensions yet: D, then click a line or a circle to size it.");

        const char* hint = "";
        switch (mode_) {
            case SketchMode::Select:    hint = "Drag a point, or the rim of a circle, and the constraints hold it."; break;
            case SketchMode::Line:      hint = "Click point to point. End on a point to close the loop; Enter ends the chain."; break;
            case SketchMode::Rectangle: hint = "Click two opposite corners. Type to fix a side, Tab for the other."; break;
            case SketchMode::Circle:    hint = "Click the centre, then the rim. Type a radius."; break;
            case SketchMode::Arc:       hint = "Click the centre, the start, then the end."; break;
            case SketchMode::Dimension: hint = activeDim_ != kNoSketchId
                                                ? "Type the new value and press Enter."
                                                : "Click a line or a circle to size it."; break;
        }
        ui::commandHint(hint);
        ui::commandHint("Hover and press X to delete, Q for construction.  Ctrl+Z steps back.");

        if (ui::quietButton("Import SVG...")) importRequested_ = true;
        ui::hoverTip("Bring the outlines of an SVG drawing into this sketch");
        ImGui::SameLine();
        if (editing()) {
            if (ui::quietButton("Extrude a region")) beginExtrude(&camera);
            ui::hoverTip("Sweep one of the closed regions into the part  (E)");
            footer = ui::commandFooter("Done", !sketch_.empty() && solved_.solved);
        } else {
            // A sketch is worth keeping before anything is built from it: it
            // becomes its own item in the outliner, to extrude whenever.
            if (ui::quietButton(faceObject_ != kNoObject ? "Keep the sketch only" : "Keep as a sketch")) {
                if (finish(scene, camera, undo, false)) {
                    finished = true;
                    ui::endCommand();
                    return;
                }
            }
            ui::hoverTip("Keep the drawing without building anything from it  (K)");
            footer = ui::commandFooter("Extrude", !regions_.empty());
        }
        break;
    }

    case SketchStage::Regions: {
        char chosen[64];
        std::snprintf(chosen, sizeof chosen, "%zu of %zu", chosen_.size(), regions_.size());
        ui::commandValue("Regions", chosen);
        ui::commandRow("Pick");
        if (ui::quietButton("Filled")) chooseFilledRegions();
        ui::hoverTip("The regions a drawing program would fill: a letter, not the hole in it");
        ImGui::SameLine();
        if (ui::quietButton("All")) chooseAllRegions();
        ImGui::SameLine();
        if (ui::quietButton("None")) chooseNoRegions();
        ui::commandHint("Click a region to add it or take it out.");
        footer = ui::commandFooter("Next", !chosen_.empty(), "Back");
        break;
    }

    case SketchStage::Depth:
    case SketchStage::Applied: {
        const bool applied = stage_ == SketchStage::Applied;
        {
            const double span = std::max(extent, std::fabs(depth_));
            const ui::NumberEdit e = ui::commandNumber("Depth", depth_, "mm", depthTyped_,
                                                       !applied && !typed_.empty(), typed_.c_str(),
                                                       -span, span, true);
            if (applied) {
                // Nothing follows the pointer now: the bar sets the depth
                // itself, and the extrusion is made again at it.
                if (e.dragged && std::fabs(e.value) > 1e-6 && e.value != depth_) {
                    depth_ = e.value;
                    adjusted_ = true;
                }
            } else {
                pulled(e, 0);
            }
        }
        // A sketch already in a part extrudes into that part: no New Body.
        if (drawExtrudeChoice(choice_, !editing())) {
            refreshReach(scene);
            if (applied) adjusted_ = true;
        }
        if (applied) refreshReach(scene);
        const ObjectId toggled = drawReachedBodies(scene, reach_, choice_.op, owner());
        if (toggled != kNoObject) {
            reach_.toggle(toggled);
            if (applied) adjusted_ = true;
        }
        if (applied) ui::commandApplied("Extrude");
        ui::commandHint(applied ? "Change the depth, the operation or the bodies, and it is made again."
                                : "Move to set the depth, drag the bar, or type one.  Click a body above to leave it out.");
        footer = applied ? ui::commandFooter("Done", true, nullptr) : ui::commandFooter("Finish", true, "Back");
        break;
    }

    case SketchStage::None:
        break;
    }

    if (footer > 0) {
        // What the button says, rather than what Enter would do: Enter part-way
        // through a chain of lines ends the chain, and a button labelled
        // Extrude has to extrude.
        switch (stage_) {
        case SketchStage::Draw:
            if (editing())            finish(scene, camera, undo, false);
            else                      beginExtrude(&camera);
            break;
        case SketchStage::Regions: beginDepth(); break;
        case SketchStage::Depth:   finish(scene, camera, undo, true); break;
        case SketchStage::Applied: dismissApplied(); break;
        case SketchStage::SelectPlane:
        case SketchStage::None:    break;
        }
        if (stage_ == SketchStage::None) finished = true;
    } else if (footer < 0) {
        if (stage_ == SketchStage::Regions || stage_ == SketchStage::Depth) {
            handleKey(27, false, false, scene, camera, undo);
        } else {
            cancel(camera);
            finished = true;
        }
    }
    ui::endCommand();

    // The numbers on the geometry they size.
    if (stage_ == SketchStage::Draw) {
        ImDrawList* dl = ImGui::GetForegroundDrawList();
        for (const SketchConstraint& k : sketch_.constraints) {
            if (!isDimension(k.rule)) continue;
            Vec2 anchor{};
            char text[48];
            if (k.rule == SketchRule::Distance) {
                const SketchPoint* a = sketch_.point(k.first);
                const SketchPoint* b = sketch_.point(k.second);
                if (!a || !b) continue;
                anchor = (a->at + b->at) * 0.5;
                std::snprintf(text, sizeof text, "%.2f", k.value);
            } else if (k.rule == SketchRule::Radius) {
                const SketchEntity* e = sketch_.entity(k.first);
                const SketchPoint* c = e ? sketch_.point(e->a) : nullptr;
                if (!c) continue;
                const Real r = e->curve == SketchCurve::Arc && sketch_.point(e->b)
                                   ? length(sketch_.point(e->b)->at - c->at)
                                   : e->radius;
                anchor = c->at + Vec2{r, r} * std::sqrt(0.5);
                std::snprintf(text, sizeof text, "R %.2f", k.value);
            } else {
                continue;
            }
            Vec2 px{};
            if (!camera.projectToPixel(plane_.toWorld(anchor), px)) continue;
            const bool active = activeDim_ == k.id;
            const ImVec2 size = ImGui::CalcTextSize(text);
            const ImVec2 at(static_cast<float>(px.x) + viewX_ - size.x * 0.5f,
                            static_cast<float>(px.y) + viewY_ - size.y - 6.0f);
            dl->AddRectFilled(ImVec2(at.x - 4, at.y - 2), ImVec2(at.x + size.x + 4, at.y + size.y + 2),
                              IM_COL32(28, 28, 32, 220), 3.0f);
            if (active)
                dl->AddRect(ImVec2(at.x - 4, at.y - 2), ImVec2(at.x + size.x + 4, at.y + size.y + 2),
                            IM_COL32(243, 68, 37, 255), 3.0f);
            dl->AddText(at, active ? IM_COL32(243, 68, 37, 255) : IM_COL32(235, 235, 240, 255), text);
        }
    }
}

} // namespace tg
