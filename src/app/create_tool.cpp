#include "app/create_tool.h"

#include "app/overlay_shapes.h"
#include "ui/command_panel.h"
#include "ui/widgets.h"
#include "app/snap_overlay.h"

#include "app/camera.h"
#include "app/undo.h"
#include "core/palette.h"
#include "geom/operations.h"
#include "render/renderer.h"
#include "scene/scene.h"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace tg {

namespace {

constexpr Vec4 kCreateCol(0.20f, 0.60f, 0.95f, 0.70f);
constexpr Vec4 kCutCol(0.95f, 0.35f, 0.20f, 0.70f);
const ImVec4 kAccentIm(palette::kBrand.r, palette::kBrand.g, palette::kBrand.b, 1.0f);

bool intersectRayPlane(const Ray& ray, Vec3 p0, Vec3 normal, float& outT, Vec3& outPt) {
    const float denom = dot(normal, ray.dir);
    if (std::fabs(denom) < 1e-6f) return false;
    const float t = dot(p0 - ray.origin, normal) / denom;
    if (t < 0.0f) return false;
    outT = t;
    outPt = ray.origin + ray.dir * t;
    return true;
}

bool pointInTile(Vec3 p, Vec3 center, Vec3 u, Vec3 v, float halfSize) {
    const Vec3 d = p - center;
    const float du = std::fabs(dot(d, u));
    const float dv = std::fabs(dot(d, v));
    return (du <= halfSize) && (dv <= halfSize);
}

} // namespace

// ---------------------------------------------------------------------------
// 2D Polygon & 3D Prism Generators
// ---------------------------------------------------------------------------

std::vector<Vec2> CreateTool::makeRectPolygon(Vec2 p1, Vec2 p2, const Real cornerRadii[4], int arcSegments) {
    const Real uMin = std::min(p1.x, p2.x);
    const Real uMax = std::max(p1.x, p2.x);
    const Real vMin = std::min(p1.y, p2.y);
    const Real vMax = std::max(p1.y, p2.y);

    const Real w = uMax - uMin;
    const Real h = vMax - vMin;
    if (w < 1e-4 || h < 1e-4) return {};

    Real r[4];
    for (int i = 0; i < 4; ++i) r[i] = std::max(cornerRadii[i], Real(0.0));

    // Clamp adjacent pairs to fit within width and height
    Real scale = 1.0;
    if (r[0] + r[3] > w) scale = std::min(scale, w / (r[0] + r[3]));
    if (r[1] + r[2] > w) scale = std::min(scale, w / (r[1] + r[2]));
    if (r[0] + r[1] > h) scale = std::min(scale, h / (r[0] + r[1]));
    if (r[2] + r[3] > h) scale = std::min(scale, h / (r[2] + r[3]));

    const Real maxR = 0.499f * std::min(w, h);
    for (int i = 0; i < 4; ++i) {
        r[i] = clampf(r[i] * scale, 0.0f, maxR);
    }

    std::vector<Vec2> out;

    // Corner 0 (Bottom-Right: uMax, vMin): center (uMax - r0, vMin + r0), angle -pi/2 to 0
    if (r[0] > 1e-4 && arcSegments > 0) {
        const Vec2 c{uMax - r[0], vMin + r[0]};
        for (int i = 0; i <= arcSegments; ++i) {
            const float t = static_cast<float>(i) / static_cast<float>(arcSegments);
            const float a = -kHalfPi + kHalfPi * t;
            out.push_back({c.x + r[0] * std::cos(a), c.y + r[0] * std::sin(a)});
        }
    } else {
        out.push_back({uMax, vMin});
    }

    // Corner 1 (Top-Right: uMax, vMax): center (uMax - r1, vMax - r1), angle 0 to pi/2
    if (r[1] > 1e-4 && arcSegments > 0) {
        const Vec2 c{uMax - r[1], vMax - r[1]};
        for (int i = 0; i <= arcSegments; ++i) {
            const float t = static_cast<float>(i) / static_cast<float>(arcSegments);
            const float a = 0.0f + kHalfPi * t;
            out.push_back({c.x + r[1] * std::cos(a), c.y + r[1] * std::sin(a)});
        }
    } else {
        out.push_back({uMax, vMax});
    }

    // Corner 2 (Top-Left: uMin, vMax): center (uMin + r2, vMax - r2), angle pi/2 to pi
    if (r[2] > 1e-4 && arcSegments > 0) {
        const Vec2 c{uMin + r[2], vMax - r[2]};
        for (int i = 0; i <= arcSegments; ++i) {
            const float t = static_cast<float>(i) / static_cast<float>(arcSegments);
            const float a = kHalfPi + kHalfPi * t;
            out.push_back({c.x + r[2] * std::cos(a), c.y + r[2] * std::sin(a)});
        }
    } else {
        out.push_back({uMin, vMax});
    }

    // Corner 3 (Bottom-Left: uMin, vMin): center (uMin + r3, vMin + r3), angle pi to 3pi/2
    if (r[3] > 1e-4 && arcSegments > 0) {
        const Vec2 c{uMin + r[3], vMin + r[3]};
        for (int i = 0; i <= arcSegments; ++i) {
            const float t = static_cast<float>(i) / static_cast<float>(arcSegments);
            const float a = kPi + kHalfPi * t;
            out.push_back({c.x + r[3] * std::cos(a), c.y + r[3] * std::sin(a)});
        }
    } else {
        out.push_back({uMin, vMin});
    }

    // Remove any duplicate consecutive points
    std::vector<Vec2> cleaned;
    for (size_t i = 0; i < out.size(); ++i) {
        if (cleaned.empty() || lengthSq(out[i] - cleaned.back()) > 1e-8) {
            cleaned.push_back(out[i]);
        }
    }
    if (cleaned.size() > 1 && lengthSq(cleaned.front() - cleaned.back()) < 1e-8) {
        cleaned.pop_back();
    }
    return cleaned;
}

std::vector<Vec2> CreateTool::makeRectPolygon(Vec2 p1, Vec2 p2, Real cornerRadius, int arcSegments) {
    const Real r[4] = {cornerRadius, cornerRadius, cornerRadius, cornerRadius};
    return makeRectPolygon(p1, p2, r, arcSegments);
}

std::vector<Vec2> CreateTool::makeCirclePolygon(Vec2 center, Real radius, int segments) {
    if (radius < 1e-4 || segments < 3) return {};
    std::vector<Vec2> out;
    out.reserve(segments);
    for (int i = 0; i < segments; ++i) {
        const float a = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(segments);
        out.push_back({center.x + radius * std::cos(a), center.y + radius * std::sin(a)});
    }
    return out;
}

bool CreateTool::makePrismMesh(const std::vector<Vec2>& poly2D,
                              Vec3 planeOrigin, Vec3 planeU, Vec3 planeV, Vec3 planeN,
                              Real z0, Real z1, Mesh& out) {
    const size_t n = poly2D.size();
    if (n < 3) return false;

    const Real zBot = std::min(z0, z1);
    const Real zTop = std::max(z0, z1);
    if (std::fabs(zTop - zBot) < 1e-6) return false;

    std::vector<Vec3> positions;
    positions.reserve(n * 2);

    // Bottom ring: 0 .. n-1
    for (size_t i = 0; i < n; ++i) {
        const Vec2 p = poly2D[i];
        positions.push_back(planeOrigin + planeU * p.x + planeV * p.y + planeN * zBot);
    }
    // Top ring: n .. 2n-1
    for (size_t i = 0; i < n; ++i) {
        const Vec2 p = poly2D[i];
        positions.push_back(planeOrigin + planeU * p.x + planeV * p.y + planeN * zTop);
    }

    std::vector<uint32_t> faceSizes;
    std::vector<uint32_t> faceIndices;

    // Bottom cap: normal points along -planeN (reversed winding of CCW poly)
    faceSizes.push_back(static_cast<uint32_t>(n));
    for (size_t i = n; i > 0; --i) {
        faceIndices.push_back(static_cast<uint32_t>(i - 1));
    }

    // Top cap: normal points along +planeN (CCW winding)
    faceSizes.push_back(static_cast<uint32_t>(n));
    for (size_t i = 0; i < n; ++i) {
        faceIndices.push_back(static_cast<uint32_t>(n + i));
    }

    // Side quads: CCW around each prism side
    for (size_t i = 0; i < n; ++i) {
        const size_t next = (i + 1) % n;
        faceSizes.push_back(4);
        faceIndices.push_back(static_cast<uint32_t>(i));
        faceIndices.push_back(static_cast<uint32_t>(next));
        faceIndices.push_back(static_cast<uint32_t>(n + next));
        faceIndices.push_back(static_cast<uint32_t>(n + i));
    }

    return out.build(positions, faceSizes, faceIndices, nullptr);
}

// ---------------------------------------------------------------------------
// Plane Geometry & Projection
// ---------------------------------------------------------------------------

void CreateTool::computePlaneBasis(Vec3 normal) {
    planeNormal_ = normalize(normal);
    if (std::fabs(planeNormal_.z) > 0.9f) {
        // Top / Bottom plane (+Z or -Z):
        planeU_ = Vec3{1, 0, 0};
        planeV_ = normalize(cross(planeNormal_, planeU_));
        if (dot(cross(planeU_, planeV_), planeNormal_) < 0.0f) planeV_ = -planeV_;
    } else if (std::fabs(planeNormal_.y) > 0.9f) {
        // Front / Back plane (+Y or -Y):
        planeU_ = Vec3{1, 0, 0};
        planeV_ = normalize(cross(planeNormal_, planeU_));
        if (dot(cross(planeU_, planeV_), planeNormal_) < 0.0f) planeV_ = -planeV_;
    } else if (std::fabs(planeNormal_.x) > 0.9f) {
        // Left / Right plane (+X or -X):
        planeU_ = Vec3{0, 1, 0};
        planeV_ = normalize(cross(planeNormal_, planeU_));
        if (dot(cross(planeU_, planeV_), planeNormal_) < 0.0f) planeV_ = -planeV_;
    } else {
        // Arbitrary angled face
        const Vec3 up{0, 0, 1};
        planeU_ = normalize(cross(up, planeNormal_));
        if (lengthSq(planeU_) < 1e-4f) {
            planeU_ = normalize(cross(Vec3{0, 1, 0}, planeNormal_));
        }
        planeV_ = normalize(cross(planeNormal_, planeU_));
    }
}

bool CreateTool::unprojectToPlane(const Camera& camera, Vec2 mousePx, Vec2& outUV) const {
    const Ray ray = camera.rayThroughPixel(mousePx.x, mousePx.y);
    float t = 0.0f;
    Vec3 pt{};
    if (!intersectRayPlane(ray, planeOrigin_, planeNormal_, t, pt)) return false;
    const Vec3 d = pt - planeOrigin_;
    outUV = Vec2{dot(d, planeU_), dot(d, planeV_)};
    return true;
}

Real CreateTool::rayPlaneExtrudeDepth(const Camera& camera, Vec2 mousePx) const {
    const Vec2 centerUV = (pt1_ + pt2_) * 0.5f;
    const Vec3 pCenter = planeOrigin_ + planeU_ * centerUV.x + planeV_ * centerUV.y;
    const Ray ray = camera.rayThroughPixel(mousePx.x, mousePx.y);

    const Vec3 w0 = pCenter - ray.origin;
    const float a = dot(planeNormal_, planeNormal_);
    const float b = dot(planeNormal_, ray.dir);
    const float c = dot(ray.dir, ray.dir);
    const float d = dot(planeNormal_, w0);
    const float e = dot(ray.dir, w0);

    const float denom = a * c - b * b;
    if (std::fabs(denom) < 1e-6f) {
        Vec2 s0{}, s1{};
        if (camera.projectToPixel(pCenter, s0) && camera.projectToPixel(pCenter + planeNormal_ * 10.0f, s1)) {
            const Vec2 dir = normalize(s1 - s0);
            const float deltaPx = dot(mousePx - extrudeStartMouse_, dir);
            return extrudeBaseDepth_ + deltaPx * 0.2f;
        }
        return extrudeBaseDepth_;
    }

    const float t = (b * e - c * d) / denom;
    return static_cast<Real>(t);
}

std::vector<Vec2> CreateTool::getCurrentProfile() const {
    if (kind_ == PrimitiveKind::Cylinder) {
        return makeCirclePolygon(pt1_, currentRadius_, 32);
    } else {
        return makeRectPolygon(pt1_, pt2_, cornerRadii_, 6);
    }
}

void CreateTool::getCurrentProfileArcs(std::vector<Vec3>& points, std::vector<Real>& arcs) const {
    points.clear();
    arcs.clear();

    // The sagitta of a quarter circle: how far the arc stands off the middle of
    // its chord. Every corner here is a quarter, whether it is a rounded corner
    // or one of the four spans that make a circle.
    auto quarter = [](Real r) { return r * (1.0 - std::sqrt(2.0) * 0.5); };

    std::vector<Vec2> uv;
    std::vector<Real> bulge;

    if (kind_ == PrimitiveKind::Cylinder) {
        const Real r = currentRadius_;
        if (r <= 1e-9) return;
        for (int i = 0; i < 4; ++i) {
            const Real a = kHalfPi * i;
            uv.push_back({pt1_.x + r * std::cos(a), pt1_.y + r * std::sin(a)});
            bulge.push_back(quarter(r));
        }
    } else {
        const Real uMin = std::min(pt1_.x, pt2_.x), uMax = std::max(pt1_.x, pt2_.x);
        const Real vMin = std::min(pt1_.y, pt2_.y), vMax = std::max(pt1_.y, pt2_.y);
        if (uMax - uMin < 1e-9 || vMax - vMin < 1e-9) return;

        // Clamped the same way the polygon path clamps: a corner cannot eat
        // more than half the side it sits on.
        const Real lim = std::min(uMax - uMin, vMax - vMin) * 0.5;
        Real r[4];
        for (int i = 0; i < 4; ++i) r[i] = clampf(cornerRadii_[i], 0.0f, static_cast<float>(lim));

        // Counter-clockwise from the bottom-right corner, matching the corner
        // order the tool stores: 0 BR, 1 TR, 2 TL, 3 BL.
        struct Corner { Vec2 in, out, centre; Real radius; };
        const Corner corners[4] = {
            {{uMax - r[0], vMin}, {uMax, vMin + r[0]}, {uMax - r[0], vMin + r[0]}, r[0]},
            {{uMax, vMax - r[1]}, {uMax - r[1], vMax}, {uMax - r[1], vMax - r[1]}, r[1]},
            {{uMin + r[2], vMax}, {uMin, vMax - r[2]}, {uMin + r[2], vMax - r[2]}, r[2]},
            {{uMin, vMin + r[3]}, {uMin + r[3], vMin}, {uMin + r[3], vMin + r[3]}, r[3]},
        };
        for (const Corner& c : corners) {
            uv.push_back(c.in);
            if (c.radius > 1e-9) {
                bulge.push_back(quarter(c.radius));   // the corner arc
                uv.push_back(c.out);
                bulge.push_back(0.0);                 // the straight side after it
            } else {
                bulge.push_back(0.0);                 // a sharp corner: no arc at all
            }
        }
    }
    if (uv.size() < 3) return;

    // Which way the profile winds decides which side of a chord its arcs bulge
    // towards. Measuring it is cheaper than reasoning about whether the plane's
    // basis came out right-handed.
    Real area = 0.0;
    for (size_t i = 0; i < uv.size(); ++i) {
        const Vec2 a = uv[i], b = uv[(i + 1) % uv.size()];
        area += a.x * b.y - b.x * a.y;
    }
    const Real sign = area > 0 ? -1.0 : 1.0;

    points.reserve(uv.size());
    for (size_t i = 0; i < uv.size(); ++i) {
        points.push_back(planeOrigin_ + planeU_ * uv[i].x + planeV_ * uv[i].y);
        arcs.push_back(bulge[i] * sign);
    }
}

Body CreateTool::buildCurrentSolid(Real depth, Backend backend) const {
    if (std::fabs(depth) < 1e-4) return Body{};
    const Real z0 = depth > 0.0 ? 0.0 : depth;
    const Real z1 = depth > 0.0 ? depth : 0.0;

    if (backend == Backend::Brep) {
        std::vector<Vec3> points;
        std::vector<Real> arcs;
        getCurrentProfileArcs(points, arcs);
        Body out;
        std::string why;
        if (makeProfileSolid(points, arcs, planeNormal_, z0, z1, out, 0, &why)) return out;
        // Falling through to a mesh would put a body in the scene that cannot
        // be combined with the exact one it was drawn on. Better to hand back
        // nothing and let the caller say so.
        return Body{};
    }

    const std::vector<Vec2> prof = getCurrentProfile();
    if (prof.empty()) return Body{};
    Mesh out;
    makePrismMesh(prof, planeOrigin_, planeU_, planeV_, planeNormal_, z0, z1, out);
    return Body(std::move(out));
}

// ---------------------------------------------------------------------------
// Tool State Lifecycle
// ---------------------------------------------------------------------------

void CreateTool::start(PrimitiveKind kind) {
    lastError_.clear();
    choice_.reset();
    reach_.clear();
    adjusted_ = false;
    kind_ = kind;
    stage_ = CreateStage::SelectPlane;
    hoveredPlane_ = PlaneChoice::XY;
    picker_.reset();
    planeOffset_ = 0.0;
    planeTilt_ = 0.0;
    selectedPlane_ = PlaneChoice::None;
    planeOrigin_ = Vec3{0, 0, 0};
    planeNormal_ = Vec3{0, 0, 1};
    computePlaneBasis(planeNormal_);
    faceObject_ = kNoObject;
    faceIndex_ = kInvalid;

    pt1_ = Vec2{0, 0};
    pt2_ = Vec2{20, 20};
    for (int i = 0; i < 4; ++i) cornerRadii_[i] = 0.0;
    currentRadius_ = 10.0;
    currentWidth_ = 20.0;
    currentDepth_ = 20.0;
    extrudeDepth_ = 20.0;

    hoveredHandle_ = HandleId::None;
    activeHandle_ = HandleId::None;
    selectedElement_ = HandleId::None;
    isMouseDown_ = false;
    isDragging_ = false;
    isFilleting_ = false;
    activeFilletCorners_.clear();
    typedValue_.clear();
    typedField_ = 0;
}

void CreateTool::cancel(Camera& camera) {
    if (stage_ == CreateStage::DrawProfile_Pt1 || stage_ == CreateStage::DrawProfile_Pt2) {
        // Restore perspective camera if canceling from orthographic
        restoreCamera(camera);
    }
    stage_ = CreateStage::None;
    activeHandle_ = HandleId::None;
    hoveredHandle_ = HandleId::None;
    selectedElement_ = HandleId::None;
    isMouseDown_ = false;
    isDragging_ = false;
    isFilleting_ = false;
    typedValue_.clear();
    typedField_ = 0;
}

void CreateTool::setHoveredPlane(PlaneChoice choice, Vec3 point, Vec3 normal,
                                ObjectId faceObj, Index faceIdx) {
    if (stage_ != CreateStage::SelectPlane) return;
    hoveredPlane_ = choice;
    planeOrigin_ = point;
    planeNormal_ = normalize(normal);
    computePlaneBasis(planeNormal_);
    faceObject_ = faceObj;
    faceIndex_ = faceIdx;
}

Vec2 CreateTool::handlePointUV(HandleId id) const {
    const Real uMin = pt1_.x, uMax = pt2_.x;
    const Real vMin = pt1_.y, vMax = pt2_.y;
    switch (id) {
        case HandleId::Corner0:    return {uMax, vMin};
        case HandleId::Corner1:    return {uMax, vMax};
        case HandleId::Corner2:    return {uMin, vMax};
        case HandleId::Corner3:    return {uMin, vMin};
        case HandleId::EdgeLeft:   return {uMin, (vMin + vMax) * 0.5};
        case HandleId::EdgeRight:  return {uMax, (vMin + vMax) * 0.5};
        case HandleId::EdgeBottom: return {(uMin + uMax) * 0.5, vMin};
        case HandleId::EdgeTop:    return {(uMin + uMax) * 0.5, vMax};
        case HandleId::FaceCenter: return (pt1_ + pt2_) * 0.5;
        case HandleId::RadiusHandle: return {pt1_.x + currentRadius_, pt1_.y};
        case HandleId::None:       break;
    }
    return pt1_;
}

void CreateTool::clampCornerRadii() {
    const Real maxR = std::min(currentWidth_, currentDepth_) * 0.499;
    for (int i = 0; i < 4; ++i) cornerRadii_[i] = clampf(cornerRadii_[i], 0.0, maxR);
}

// Puts the view back where the tool found it.
//
// Over the animation rather than in one frame, the same as going in: the view
// is being handed back to the user, and a camera that teleports leaves them to
// work out for themselves where their model went.
void CreateTool::restoreCamera(Camera& camera) {
    camera.animateTo(savedCamera_.target, savedCamera_.distance,
                     savedCamera_.yaw, savedCamera_.pitch);
    camera.orthographic = savedCamera_.orthographic;
}

void CreateTool::commitPlaneSelection(Camera& camera) {
    if (stage_ != CreateStage::SelectPlane) return;
    selectedPlane_ = hoveredPlane_;
    planeBase_ = plane();
    planeOffset_ = 0.0;
    planeTilt_ = 0.0;

    // Save camera perspective before switching to orthographic head-on
    savedCamera_.target = camera.target;
    savedCamera_.distance = camera.distance;
    savedCamera_.yaw = camera.yaw;
    savedCamera_.pitch = camera.pitch;
    savedCamera_.orthographic = camera.orthographic;

    // Square up to the plane: the pivot on it, the eye straight out along its
    // normal, and orthographic, because a profile drawn in perspective is drawn
    // against a picture of itself rather than against its dimensions.
    float y = 0.0f, p = 0.0f;
    Camera::anglesFor(planeNormal_, y, p);
    camera.animateTo(planeOrigin_, camera.distance, y, p);
    camera.orthographic = true;

    clearFields();   // each step asks its own questions
    stage_ = CreateStage::DrawProfile_Pt1;
    pt1_ = Vec2{0, 0};
    pt2_ = Vec2{0, 0};
}

void CreateTool::applyPlaneShift() {
    const PlaneFrame f = offsetFrame(planeBase_, planeOffset_, planeTilt_);
    planeOrigin_ = f.origin;
    planeU_ = f.u;
    planeV_ = f.v;
    planeNormal_ = f.normal;
}

void CreateTool::adoptPickedPlane() {
    const PlaneFrame& f = picker_.frame();
    setHoveredPlane(picker_.choice(), f.origin, f.normal, picker_.faceObject(), picker_.faceIndex());
}

void CreateTool::choosePlane(PlaneChoice choice, Camera& camera, const Scene& scene) {
    (void)scene;
    if (stage_ != CreateStage::SelectPlane) return;
    switch (choice) {
        case PlaneChoice::XY:
            setHoveredPlane(PlaneChoice::XY, {0, 0, 0}, {0, 0, 1});
            break;
        case PlaneChoice::XZ:
            setHoveredPlane(PlaneChoice::XZ, {0, 0, 0}, {0, -1, 0});
            break;
        case PlaneChoice::YZ:
            setHoveredPlane(PlaneChoice::YZ, {0, 0, 0}, {1, 0, 0});
            break;
        case PlaneChoice::Face:
        case PlaneChoice::None:
            break;
    }
    commitPlaneSelection(camera);
}

// ---------------------------------------------------------------------------
// Per-Frame Update
// ---------------------------------------------------------------------------

void CreateTool::update(const Scene& scene, const Camera& camera, Vec2 mousePx, bool snap) {
    if (stage_ == CreateStage::None) return;

    // Lengths and positions want different ladders, and conflating them is why
    // this used to carry three fixed steps that were all wrong at some zoom.
    //
    //   a position lands on a line you can see -- the decimal grid, handled by
    //   snapOnPlane, because a point that sits where there is no line leaves the
    //   user holding a number that disagrees with the screen;
    //
    //   a length is a number you would choose -- 0.25, 0.5, 1, 2.5, 5 -- which
    //   is finer than the decades and is what a depth or a radius is actually
    //   typed as.
    //
    // Ten pixels a step, so a steady hand can reach every value on the ladder.
    const Real step = snap ? static_cast<Real>(camera.snapStep(planeOrigin_)) : 0.0;
    auto quantise = [step](Real v) {
        return step > 0.0 ? std::round(v / step) * step : v;
    };

    // Where the point goes, and why. Not only what is under the cursor: being
    // level with a hole on the far side of the part is as much a place as the
    // hole itself, and is most of what makes a sketch land where it was meant.
    activeSnap_ = PlaneSnap{};
    auto placePoint = [&](Vec2 freeUV, bool useStartPoint) {
        if (!snap) return freeUV;
        std::vector<SnapPoint> extra;
        if (useStartPoint) {
            const Vec3 world = planeOrigin_ + planeU_ * pt1_.x + planeV_ * pt1_.y;
            extra.push_back({world, pt1_, SnapKind::Vertex, kNoObject, 0.0});
        }
        activeSnap_ = snapOnPlane(scene, camera, plane(), mousePx, freeUV, {}, extra);
        return activeSnap_.valid() ? activeSnap_.uv : freeUV;
    };

    if (stage_ == CreateStage::SelectPlane) {
        // The same picker the sketch tool asks with: a face, an origin plane,
        // three points or an edge.
        picker_.update(scene, camera, mousePx);
        if (picker_.method() == PlaneMethod::Surface) adoptPickedPlane();
        return;
    }

    if (stage_ == CreateStage::DrawProfile_Pt1) {
        Vec2 uv{0, 0};
        if (unprojectToPlane(camera, mousePx, uv)) {
            uv = placePoint(uv, false);
            pt1_ = uv;
            pt2_ = uv;
        }
        return;
    }

    if (stage_ == CreateStage::DrawProfile_Pt2) {
        Vec2 uv{0, 0};
        if (unprojectToPlane(camera, mousePx, uv)) {
            // The point already placed is a reference in its own right: keeping
            // the second corner level with the first is the commonest thing
            // anyone does here, and the scene knows nothing about it.
            uv = placePoint(uv, true);

            if (kind_ == PrimitiveKind::Cylinder) {
                if (fieldFixed_[0]) {
                    // Typed. The cursor still says which way round, and nothing
                    // more: the radius is the user's.
                    const Vec2 dir = uv - pt1_;
                    const Real len = length(dir);
                    pt2_ = len > 1e-9 ? pt1_ + dir * (fieldValue_[0] / len)
                                      : pt1_ + Vec2{fieldValue_[0], 0};
                    currentRadius_ = fieldValue_[0];
                } else {
                    // A circle is drawn by its radius, so that is what has to
                    // land on a round number -- snapping the point on the rim
                    // leaves the radius at whatever the diagonal happened to
                    // be. When the rim has caught something real, the point
                    // wins: matching a hole exactly is worth more than a tidy
                    // number.
                    pt2_ = uv;
                    const bool onFeature =
                        activeSnap_.valid() && activeSnap_.kind != SnapKind::GridPoint &&
                        activeSnap_.kind != SnapKind::GridLine;
                    Real r = std::max(length(pt2_ - pt1_), Real(1.0));
                    if (!onFeature && snap) {
                        r = std::max(quantise(r), Real(0.1));
                        const Vec2 dir = pt2_ - pt1_;
                        const Real len = length(dir);
                        pt2_ = len > 1e-9 ? pt1_ + dir * (r / len) : pt1_ + Vec2{r, 0};
                    }
                    currentRadius_ = r;
                }
            } else {
                // Each side independently: a width that has been typed stays
                // typed while the depth is swept out by hand. Which side of the
                // first corner the rectangle goes is always the cursor's, even
                // for a dimension that is fixed -- that is direction, not size.
                const Real w = fieldFixed_[0] ? fieldValue_[0]
                                              : std::max(std::fabs(uv.x - pt1_.x), Real(1.0));
                const Real d = fieldFixed_[1] ? fieldValue_[1]
                                              : std::max(std::fabs(uv.y - pt1_.y), Real(1.0));
                pt2_ = {pt1_.x + (uv.x < pt1_.x ? -w : w),
                        pt1_.y + (uv.y < pt1_.y ? -d : d)};
                currentWidth_ = w;
                currentDepth_ = d;

                // An alignment along an axis the cursor no longer controls is
                // not what put the corner there, so it is not claimed.
                if (activeSnap_.kind == SnapKind::Alignment &&
                    (activeSnap_.refs[0].alongU ? fieldFixed_[0] : fieldFixed_[1]))
                    activeSnap_ = PlaneSnap{};
                else if (activeSnap_.valid() && fieldFixed_[0] && fieldFixed_[1])
                    activeSnap_ = PlaneSnap{};
            }
        }
        return;
    }

    if (stage_ == CreateStage::AdjustProfile) {
        Vec2 currentUV{};
        unprojectToPlane(camera, mousePx, currentUV);

        // Rounding a corner is a mode now, not a drag.
        //
        // It used to be what a corner handle did when you pulled on it, which
        // meant the profile could not be resized from its corners at all -- the
        // one thing a rectangle's corners are for in every other tool that has
        // them. So dragging moves, and a fillet is asked for by name: hover the
        // corner and press F.
        if (isFilleting_) {
            if (!fieldFixed_[0]) {
                // Sit on the corner for nothing, pull away for more. The same
                // gesture the main fillet tool uses, and the same ladder.
                const Real dist = quantise(length(currentUV - filletRefUV_));
                const Real maxR = std::min(currentWidth_, currentDepth_) * 0.499;
                const Real r = clampf(dist, 0.0, maxR);
                for (int c : activeFilletCorners_) cornerRadii_[c] = r;
            }
            return;
        }

        if (isDragging_ && activeHandle_ != HandleId::None) {
            // The handle is what moves, so the handle is what snaps: the
            // cursor's own position is an accident of where inside the grab
            // circle the press landed. A corner brought level with a hole below
            // has to be the corner that got there, not the pointer.
            const Vec2 placed = placePoint(currentUV + dragGrabOffset_, false);

            // An alignment on an axis this handle does not move says nothing
            // about where it went, so it is not shown.
            const bool movesU = activeHandle_ != HandleId::EdgeBottom &&
                                activeHandle_ != HandleId::EdgeTop;
            const bool movesV = activeHandle_ != HandleId::EdgeLeft &&
                                activeHandle_ != HandleId::EdgeRight;
            if (activeSnap_.kind == SnapKind::Alignment &&
                (activeSnap_.refs[0].alongU ? !movesU : !movesV))
                activeSnap_ = PlaneSnap{};

            // Never inside out, and never smaller than something that can still
            // be grabbed by its own handles.
            constexpr Real kMin = 0.5;
            switch (activeHandle_) {
                case HandleId::RadiusHandle:
                    currentRadius_ = std::max(quantise(length(currentUV - pt1_)), Real(0.1));
                    pt2_ = pt1_ + Vec2{currentRadius_, 0};
                    break;

                // The whole profile follows, keeping its size.
                case HandleId::FaceCenter: {
                    const Vec2 half = (pt2_ - pt1_) * 0.5;
                    pt1_ = placed - half;
                    pt2_ = placed + half;
                    break;
                }

                case HandleId::Corner0:
                    pt2_.x = std::max(placed.x, pt1_.x + kMin);
                    pt1_.y = std::min(placed.y, pt2_.y - kMin);
                    break;
                case HandleId::Corner1:
                    pt2_.x = std::max(placed.x, pt1_.x + kMin);
                    pt2_.y = std::max(placed.y, pt1_.y + kMin);
                    break;
                case HandleId::Corner2:
                    pt1_.x = std::min(placed.x, pt2_.x - kMin);
                    pt2_.y = std::max(placed.y, pt1_.y + kMin);
                    break;
                case HandleId::Corner3:
                    pt1_.x = std::min(placed.x, pt2_.x - kMin);
                    pt1_.y = std::min(placed.y, pt2_.y - kMin);
                    break;

                case HandleId::EdgeLeft:   pt1_.x = std::min(placed.x, pt2_.x - kMin); break;
                case HandleId::EdgeRight:  pt2_.x = std::max(placed.x, pt1_.x + kMin); break;
                case HandleId::EdgeBottom: pt1_.y = std::min(placed.y, pt2_.y - kMin); break;
                case HandleId::EdgeTop:    pt2_.y = std::max(placed.y, pt1_.y + kMin); break;
                case HandleId::None:       break;
            }
            if (kind_ != PrimitiveKind::Cylinder) {
                currentWidth_ = pt2_.x - pt1_.x;
                currentDepth_ = pt2_.y - pt1_.y;
                clampCornerRadii();
            }
            return;
        }

        // Hover test for handles in screen space
        hoveredHandle_ = HandleId::None;
        auto testScreenHandle = [&](Vec3 worldPos, HandleId id, float radiusPx = 14.0f) {
            Vec2 sp{};
            if (camera.projectToPixel(worldPos, sp)) {
                if (length(sp - mousePx) < radiusPx) {
                    hoveredHandle_ = id;
                    return true;
                }
            }
            return false;
        };

        if (kind_ == PrimitiveKind::Cylinder) {
            const Vec3 rPoint = planeOrigin_ + planeU_ * (pt1_.x + currentRadius_) + planeV_ * pt1_.y;
            testScreenHandle(rPoint, HandleId::RadiusHandle);
        } else {
            const Real uMin = pt1_.x, uMax = pt2_.x;
            const Real vMin = pt1_.y, vMax = pt2_.y;

            const Vec3 c0 = planeOrigin_ + planeU_ * uMax + planeV_ * vMin; // BR
            const Vec3 c1 = planeOrigin_ + planeU_ * uMax + planeV_ * vMax; // TR
            const Vec3 c2 = planeOrigin_ + planeU_ * uMin + planeV_ * vMax; // TL
            const Vec3 c3 = planeOrigin_ + planeU_ * uMin + planeV_ * vMin; // BL

            const Vec3 mBot   = (c0 + c3) * 0.5f;
            const Vec3 mRight = (c0 + c1) * 0.5f;
            const Vec3 mTop   = (c1 + c2) * 0.5f;
            const Vec3 mLeft  = (c2 + c3) * 0.5f;
            const Vec3 mFace  = (c0 + c2) * 0.5f;

            if (!testScreenHandle(c0, HandleId::Corner0))
                if (!testScreenHandle(c1, HandleId::Corner1))
                    if (!testScreenHandle(c2, HandleId::Corner2))
                        if (!testScreenHandle(c3, HandleId::Corner3))
                            if (!testScreenHandle(mLeft, HandleId::EdgeLeft))
                                if (!testScreenHandle(mRight, HandleId::EdgeRight))
                                    if (!testScreenHandle(mBot, HandleId::EdgeBottom))
                                        if (!testScreenHandle(mTop, HandleId::EdgeTop))
                                            testScreenHandle(mFace, HandleId::FaceCenter, 10.0f);
        }
        return;
    }

    if (stage_ == CreateStage::ExtrudeDepth) {
        if (!fieldFixed_[0]) {                  // typed, so the mouse is out of it
            const Real d = quantise(rayPlaneExtrudeDepth(camera, mousePx));
            if (std::fabs(d) > 1e-4) extrudeDepth_ = d;
        }
        choice_.follow(extrudeDepth_, faceObject_ != kNoObject);
        refreshReach(scene);
        return;
    }
}

// ---------------------------------------------------------------------------
// Mouse Handlers (Press-and-Hold to Drag, Release to Commit)
// ---------------------------------------------------------------------------

void CreateTool::handleMouseDown(Vec2 mousePx, Scene& scene, Camera& camera, UndoStack& undo) {
    isMouseDown_ = true;

    if (stage_ == CreateStage::SelectPlane) {
        // Three points or an edge take more than one click, or a click on
        // the right thing: the picker says when the plane is decided.
        if (picker_.method() != PlaneMethod::Surface) {
            if (!picker_.click(scene, camera)) return;
            adoptPickedPlane();
        }
        commitPlaneSelection(camera);
        return;
    }

    // Both point stages commit what update() placed this frame, which is the
    // snapped point. Re-unprojecting the cursor here is what the click used to
    // do, and it threw the snap away at the one moment it mattered: the
    // indicator said "centre", the dotted line said which centre, and the point
    // landed a third of a millimetre off it. update() runs before input for
    // exactly this reason.
    if (stage_ == CreateStage::DrawProfile_Pt1) {
        pt2_ = pt1_;
        clearFields();   // each step asks its own questions
        stage_ = CreateStage::DrawProfile_Pt2;
        return;
    }

    if (stage_ == CreateStage::DrawProfile_Pt2) {
        if (kind_ != PrimitiveKind::Cylinder) {
            const Real uMin = std::min(pt1_.x, pt2_.x);
            const Real uMax = std::max(pt1_.x, pt2_.x);
            const Real vMin = std::min(pt1_.y, pt2_.y);
            const Real vMax = std::max(pt1_.y, pt2_.y);
            pt1_ = {uMin, vMin};
            pt2_ = {uMax, vMax};
        }

        // Restore perspective camera
        restoreCamera(camera);

        clearFields();   // each step asks its own questions
        stage_ = CreateStage::AdjustProfile;
        return;
    }

    if (stage_ == CreateStage::AdjustProfile) {
        // A click while rounding a corner is what confirms it, the same as it
        // is for the fillet tool proper.
        if (isFilleting_) {
            isFilleting_ = false;
            activeFilletCorners_.clear();
            activeHandle_ = HandleId::None;
            return;
        }

        if (hoveredHandle_ != HandleId::None) {
            activeHandle_ = hoveredHandle_;
            selectedElement_ = hoveredHandle_;
            isDragging_ = true;
            dragStartMouse_ = mousePx;
            dragStartPt1_ = pt1_;
            dragStartPt2_ = pt2_;
            for (int i = 0; i < 4; ++i) dragStartFillets_[i] = cornerRadii_[i];

            Vec2 currentUV{};
            unprojectToPlane(camera, mousePx, currentUV);
            dragStartMouseUV_ = currentUV;
            dragGrabOffset_ = handlePointUV(activeHandle_) - currentUV;

            // A handle grabbed by hand is being taken back from the keyboard:
            // a dimension cannot be both pinned to a typed number and dragged.
            clearLocks(activeHandle_);
        }
        return;
    }

    if (stage_ == CreateStage::ExtrudeDepth) {
        finishCreation(scene, camera, undo);
        return;
    }
}

void CreateTool::handleMouseUp(Vec2 mousePx, Camera& camera) {
    (void)mousePx;
    (void)camera;
    isMouseDown_ = false;
    if (stage_ == CreateStage::AdjustProfile) {
        // Only the drag ends here. Rounding a corner is a mode that a press
        // began and a press ends, so letting go of the button that started it
        // must not also finish it.
        if (!isFilleting_) {
            isDragging_ = false;
            activeHandle_ = HandleId::None;
        }
    }
}

void CreateTool::handleLeftClick(Scene& scene, Camera& camera, UndoStack& undo) {
    handleMouseDown(dragStartMouse_, scene, camera, undo);
}

void CreateTool::handleRightClick(Camera& camera) {
    if (isDragging_ || activeHandle_ != HandleId::None) {
        isDragging_ = false;
        isFilleting_ = false;
        activeHandle_ = HandleId::None;
        return;
    }
    cancel(camera);
}

bool CreateTool::handleTypedKey(int key) {
    // Only where there is a dimension for a number to mean something. In
    // SelectPlane the digits are already spoken for: 1, 3 and 7 pick a plane.
    if (stage_ != CreateStage::DrawProfile_Pt2 && stage_ != CreateStage::AdjustProfile &&
        stage_ != CreateStage::ExtrudeDepth)
        return false;

    // Every accepted keystroke re-reads the buffer into the dimension, so the
    // profile follows the digits rather than waiting for Enter. Backspacing a
    // field back to empty hands it to the mouse again.
    if (key == 8 || key == 127) {                       // backspace
        if (typedValue_.empty()) return false;
        typedValue_.pop_back();
        syncTypedField();
        return true;
    }
    if (key >= '0' && key <= '9') {
        typedValue_.push_back(static_cast<char>(key));
        syncTypedField();
        return true;
    }
    if (key == '.' && typedValue_.find('.') == std::string::npos) {
        typedValue_.push_back('.');
        syncTypedField();
        return true;
    }
    // A leading minus only, and only where a negative reads as something: a
    // depth of -5 is a cut, a width of -5 is nothing.
    if (key == '-' && typedValue_.empty() && stage_ == CreateStage::ExtrudeDepth) {
        typedValue_.push_back('-');
        return true;
    }
    return false;
}

int CreateTool::fieldCount() const {
    if (stage_ == CreateStage::ExtrudeDepth) return 1;
    if (stage_ == CreateStage::AdjustProfile && isFilleting_) return 1;
    return kind_ == PrimitiveKind::Cylinder ? 1 : 2;
}

const char* CreateTool::fieldName(int field) const {
    if (stage_ == CreateStage::ExtrudeDepth) return "Depth";
    if (stage_ == CreateStage::AdjustProfile && isFilleting_) return "Radius";
    if (kind_ == PrimitiveKind::Cylinder) return "Radius";
    return field == 0 ? "Width" : "Depth";
}

Real CreateTool::fieldDisplay(int field) const {
    if (stage_ == CreateStage::ExtrudeDepth) return extrudeDepth_;
    if (stage_ == CreateStage::AdjustProfile && isFilleting_)
        return activeFilletCorners_.empty() ? 0.0 : cornerRadii_[activeFilletCorners_.front()];
    if (kind_ == PrimitiveKind::Cylinder) return currentRadius_;
    return field == 0 ? currentWidth_ : currentDepth_;
}

void CreateTool::setField(int field, Real v) {
    if (field < 0 || field > 1) return;

    auto fix = [&](Real applied) {
        fieldFixed_[field] = true;
        fieldValue_[field] = applied;
    };

    if (stage_ == CreateStage::ExtrudeDepth) {
        extrudeDepth_ = v;                      // a cut is a negative depth
        fix(v);
        return;
    }

    if (stage_ == CreateStage::AdjustProfile && isFilleting_) {
        const Real maxR = std::min(currentWidth_, currentDepth_) * 0.499;
        const Real r = clampf(v, 0.0, maxR);
        for (int c : activeFilletCorners_) cornerRadii_[c] = r;
        fix(r);
        return;
    }

    if (v <= 0.0) return;                       // a size has to be positive

    if (kind_ == PrimitiveKind::Cylinder) {
        currentRadius_ = v;
        pt2_ = pt1_ + Vec2{currentRadius_, 0};
        fix(v);
        return;
    }

    // Keep the corner the user started from where it is and move the opposite
    // one, in whichever direction the rectangle is already being drawn.
    if (field == 0) {
        currentWidth_ = v;
        pt2_.x = pt1_.x + (pt2_.x < pt1_.x ? -v : v);
    } else {
        currentDepth_ = v;
        pt2_.y = pt1_.y + (pt2_.y < pt1_.y ? -v : v);
    }
    if (stage_ == CreateStage::AdjustProfile) clampCornerRadii();
    fix(v);
}

void CreateTool::syncTypedField() {
    if (typedValue_.empty()) {
        // Backspaced away to nothing: the mouse has it again.
        if (typedField_ >= 0 && typedField_ < 2) fieldFixed_[typedField_] = false;
        return;
    }
    Real v = 0.0;
    try {
        size_t used = 0;
        v = std::stod(typedValue_, &used);
        if (used != typedValue_.size()) return;   // "12." part-way through
    } catch (...) {
        return;                                   // "-" or "." on their own
    }
    setField(typedField_, v);
}

void CreateTool::clearLocks(HandleId id) {
    switch (id) {
        case HandleId::EdgeLeft:
        case HandleId::EdgeRight:
        case HandleId::RadiusHandle:
            fieldFixed_[0] = false;
            break;
        case HandleId::EdgeBottom:
        case HandleId::EdgeTop:
            fieldFixed_[1] = false;
            break;
        case HandleId::Corner0:
        case HandleId::Corner1:
        case HandleId::Corner2:
        case HandleId::Corner3:
            fieldFixed_[0] = fieldFixed_[1] = false;
            break;
        // Moving the profile does not resize it, so nothing typed is disturbed.
        case HandleId::FaceCenter:
        case HandleId::None:
            break;
    }
    typedValue_.clear();
}

bool CreateTool::applyTypedValue() {
    if (typedValue_.empty()) return false;
    syncTypedField();
    typedValue_.clear();
    return true;
}

bool CreateTool::handleKey(int key, bool shift, bool ctrl, Camera& camera, Scene& scene, UndoStack& undo) {
    (void)shift;
    (void)ctrl;
    if (stage_ == CreateStage::None) return false;

    // Esc undoes the smallest thing it can: what is half-typed, then the
    // dimension it was pinning, then the fillet being adjusted, and only then
    // the tool itself. A modal key that always cancels everything is one users
    // learn not to press.
    if (key == 27) {
        if (typing()) {
            typedValue_.clear();
            syncTypedField();               // releases the field to the mouse
            return true;
        }
        if (typedField_ < 2 && fieldFixed_[typedField_]) {
            fieldFixed_[typedField_] = false;
            return true;
        }
        if (isFilleting_) {
            for (int i = 0; i < 4; ++i) cornerRadii_[i] = dragStartFillets_[i];
            isFilleting_ = false;
            activeFilletCorners_.clear();
            activeHandle_ = HandleId::None;
            clearFields();
            return true;
        }
        cancel(camera);
        return true;
    }

    // Tab moves between the dimensions this stage has, fixing whatever was
    // typed for the one being left.
    if (key == 9 && fieldCount() > 1 &&
        (stage_ == CreateStage::DrawProfile_Pt2 || stage_ == CreateStage::AdjustProfile)) {
        applyTypedValue();
        typedField_ = (typedField_ + 1) % fieldCount();
        return true;
    }

    // Digits, a point, a leading minus, backspace.
    if (handleTypedKey(key)) return true;

    // E = Extrude, Enter, Space. Anything typed is committed before the stage
    // moves on, so "25 Enter" sets the width and advances in one gesture.
    if (key == 'E' || key == 'e' || key == 13 || key == 32) {
        applyTypedValue();
        if (stage_ == CreateStage::SelectPlane) {
            commitPlaneSelection(camera);
            return true;
        }
        if (stage_ == CreateStage::DrawProfile_Pt1) {
            clearFields();   // each step asks its own questions
            stage_ = CreateStage::DrawProfile_Pt2;
            return true;
        }
        if (stage_ == CreateStage::DrawProfile_Pt2) {
            if (kind_ != PrimitiveKind::Cylinder) {
                const Real uMin = std::min(pt1_.x, pt2_.x);
                const Real uMax = std::max(pt1_.x, pt2_.x);
                const Real vMin = std::min(pt1_.y, pt2_.y);
                const Real vMax = std::max(pt1_.y, pt2_.y);
                pt1_ = {uMin, vMin};
                pt2_ = {uMax, vMax};
            }
            restoreCamera(camera);
            clearFields();   // each step asks its own questions
            stage_ = CreateStage::AdjustProfile;
            return true;
        }
        if (stage_ == CreateStage::AdjustProfile) {
            // Enter finishes what is in front of you. While a corner is being
            // rounded that is the round, not the extrude: moving on before the
            // fillet is settled is never what was meant.
            if (isFilleting_) {
                isFilleting_ = false;
                activeFilletCorners_.clear();
                activeHandle_ = HandleId::None;
                clearFields();
                return true;
            }
            clearFields();   // each step asks its own questions
            stage_ = CreateStage::ExtrudeDepth;
            extrudeBaseDepth_ = 20.0;
            extrudeDepth_ = 20.0;
            return true;
        }
        if (stage_ == CreateStage::ExtrudeDepth) {
            finishCreation(scene, camera, undo);
            return true;
        }
    }

    // F = Fillet
    if (key == 'F' || key == 'f') {
        if (stage_ == CreateStage::AdjustProfile && kind_ != PrimitiveKind::Cylinder) {
            HandleId target = (hoveredHandle_ != HandleId::None) ? hoveredHandle_ : selectedElement_;
            const Real uMin = pt1_.x, uMax = pt2_.x;
            const Real vMin = pt1_.y, vMax = pt2_.y;

            // A mode rather than a drag: nothing is being held down, so the
            // radius follows the pointer until a click or Enter confirms it,
            // and Esc puts back what was there.
            isFilleting_ = true;
            isDragging_ = false;
            activeHandle_ = target;
            selectedElement_ = target;
            for (int i = 0; i < 4; ++i) dragStartFillets_[i] = cornerRadii_[i];
            clearFields();

            if (target == HandleId::Corner0) {
                filletRefUV_ = {uMax, vMin};
                activeFilletCorners_ = {0};
            } else if (target == HandleId::Corner1) {
                filletRefUV_ = {uMax, vMax};
                activeFilletCorners_ = {1};
            } else if (target == HandleId::Corner2) {
                filletRefUV_ = {uMin, vMax};
                activeFilletCorners_ = {2};
            } else if (target == HandleId::Corner3) {
                filletRefUV_ = {uMin, vMin};
                activeFilletCorners_ = {3};
            } else if (target == HandleId::EdgeRight) {
                filletRefUV_ = {uMax, (vMin + vMax) * 0.5f};
                activeFilletCorners_ = {0, 1}; // 2 corners on right edge
            } else if (target == HandleId::EdgeTop) {
                filletRefUV_ = {(uMin + uMax) * 0.5f, vMax};
                activeFilletCorners_ = {1, 2}; // 2 corners on top edge
            } else if (target == HandleId::EdgeLeft) {
                filletRefUV_ = {uMin, (vMin + vMax) * 0.5f};
                activeFilletCorners_ = {2, 3}; // 2 corners on left edge
            } else if (target == HandleId::EdgeBottom) {
                filletRefUV_ = {(uMin + uMax) * 0.5f, vMin};
                activeFilletCorners_ = {3, 0}; // 2 corners on bottom edge
            } else {
                // Face / all corners
                filletRefUV_ = (pt1_ + pt2_) * 0.5f;
                activeFilletCorners_ = {0, 1, 2, 3};
            }
            return true;
        }
    }

    // Operation, while the depth is being set. Picking one ends the depth's
    // say in it.
    if (stage_ == CreateStage::ExtrudeDepth) {
        ExtrudeOp picked;
        if (extrudeOpForKey(key, picked)) {
            choice_.pick(picked);
            refreshReach(scene);
            return true;
        }
    }

    // Number keys for direct plane selection in SelectPlane
    if (stage_ == CreateStage::SelectPlane) {
        if (key == '1') { setHoveredPlane(PlaneChoice::XZ, {0, 0, 0}, {0, -1, 0}); commitPlaneSelection(camera); return true; }
        if (key == '3') { setHoveredPlane(PlaneChoice::YZ, {0, 0, 0}, {1, 0, 0});  commitPlaneSelection(camera); return true; }
        if (key == '7') { setHoveredPlane(PlaneChoice::XY, {0, 0, 0}, {0, 0, 1});  commitPlaneSelection(camera); return true; }
    }

    return false;
}

bool CreateTool::handleKey(int key, bool shift, bool ctrl, Camera& camera) {
    Scene dummyScene;
    UndoStack dummyUndo;
    return handleKey(key, shift, ctrl, camera, dummyScene, dummyUndo);
}

// ---------------------------------------------------------------------------
// Final Creation & Boolean Cut Execution
// ---------------------------------------------------------------------------

bool CreateTool::finishCreation(Scene& scene, Camera& camera, UndoStack& undo) {
    if (stage_ != CreateStage::ExtrudeDepth && stage_ != CreateStage::AdjustProfile) return false;
    if (std::fabs(extrudeDepth_) < 1e-4) {
        cancel(camera);
        return false;
    }
    // A refusal leaves the tool where it was, so the operation or the bodies
    // can be changed and Finish tried again.
    if (!commitExtrusion(scene, undo)) return false;
    stage_ = CreateStage::Applied;
    return true;
}

bool CreateTool::recommit(Scene& scene, Camera& camera, UndoStack& undo) {
    (void)camera;
    if (stage_ != CreateStage::Applied) return false;
    return commitExtrusion(scene, undo);
}

void CreateTool::refreshReach(const Scene& scene) {
    if (choice_.op == ExtrudeOp::NewBody || scene.defaultBackend() != Backend::Brep) {
        reach_.refresh(scene, Body{}, faceObject_, choice_.op, extrudeDepth_, "none");
        return;
    }
    // The tool, and what it is made from: the same profile at the same depth
    // is the same tool, and the bodies it reaches need not be measured again.
    char key[256];
    std::snprintf(key, sizeof key, "%d|%.6g,%.6g,%.6g,%.6g|%.6g,%.6g,%.6g,%.6g|%.6g|%.6g|%.6g,%.6g,%.6g|%.6g,%.6g,%.6g",
                  static_cast<int>(kind_), pt1_.x, pt1_.y, pt2_.x, pt2_.y, cornerRadii_[0],
                  cornerRadii_[1], cornerRadii_[2], cornerRadii_[3], currentRadius_, extrudeDepth_,
                  planeOrigin_.x, planeOrigin_.y, planeOrigin_.z, planeNormal_.x, planeNormal_.y,
                  planeNormal_.z);
    if (reachToolKey_ != key) {
        reachTool_ = buildCurrentSolid(extrudeDepth_, Backend::Brep);
        reachToolKey_ = key;
    }
    reach_.refresh(scene, reachTool_, faceObject_, choice_.op, extrudeDepth_, key);
}

bool CreateTool::commitExtrusion(Scene& scene, UndoStack& undo) {
    const Real depth = extrudeDepth_;
    if (std::fabs(depth) < 1e-4) {
        lastError_ = "The depth is zero";
        return false;
    }
    const Body solid = buildCurrentSolid(depth, scene.defaultBackend());
    if (solid.empty()) {
        lastError_ = "That profile could not be turned into a solid";
        return false;
    }

    // Measured now rather than trusted from the last frame: this is what the
    // extrusion is about to act on.
    choice_.follow(depth, faceObject_ != kNoObject);
    refreshReach(scene);
    const ExtrudeOp op = choice_.op;
    const std::vector<ObjectId> bodies = reach_.included();
    // A join that reaches nothing has nothing to join, and is a body of its
    // own -- as it is in Fusion.
    const bool asNewBody = op == ExtrudeOp::NewBody || (op == ExtrudeOp::Join && bodies.empty());

    if (!asNewBody) {
        const SceneObject* drawnOn = scene.find(faceObject_);
        if (scene.defaultBackend() != Backend::Brep) {
            lastError_ = std::string(extrudeOpName(op)) +
                         " needs the exact kernel, which this build does not have";
            return false;
        }
        if (bodies.empty()) {
            lastError_ = drawnOn && drawnOn->body.isMesh()
                             ? std::string(extrudeOpName(op)) + " needs a solid, and this is a mesh: "
                                                                "Modify > Convert to Solid first"
                         : op == ExtrudeOp::Cut ? "Nothing there to cut into"
                                                : "Nothing there to intersect with";
            return false;
        }

        std::vector<std::unique_ptr<Command>> parts;
        const char* label = op == ExtrudeOp::Join ? "Extrude Join"
                          : op == ExtrudeOp::Cut  ? "Extrude Cut" : "Extrude Intersect";
        ObjectId ownerDone = kNoObject;

        // Into the face it was drawn on, a cut starts a little outside that
        // face, so its entry is a clean crossing rather than a coincident
        // plane. Only for that body: pushed past it, the overshoot would nick
        // whatever sits against the face.
        if (op == ExtrudeOp::Cut && faceObject_ != kNoObject && reach_.includes(faceObject_)) {
            SceneObject* target = scene.find(faceObject_);
            const Real overshoot = std::max(std::fabs(depth) * 0.05, Real(1.0));
            const Real cz0 = depth < 0.0 ? depth : -overshoot;
            const Real cz1 = depth < 0.0 ? overshoot : depth;
            Body cutter;
            std::vector<Vec3> points;
            std::vector<Real> arcs;
            getCurrentProfileArcs(points, arcs);
            std::string why;
            if (!makeProfileSolid(points, arcs, planeNormal_, cz0, cz1, cutter, 0, &why)) {
                lastError_ = why.empty() ? "The cutter could not be built" : "Cut failed: " + why;
                return false;
            }
            // Into the target's own space, where its chain lives -- and into
            // the chain, not over the body: a cut written straight onto the
            // body was thrown away by the next thing that re-ran the history.
            cutter.transform(inverse(target->modelMatrix()));
            std::vector<Feature> chainBefore = target->features;
            Feature f;
            f.kind = FeatureKind::Boolean;
            f.booleanOp = BooleanOp::Difference;
            f.bakedBody = std::move(cutter);
            f.toolName = primitiveName(kind_);
            if (!scene.addFeature(faceObject_, std::move(f), &why)) {
                lastError_ = why.empty() ? "Cut failed: no valid solid came out of it" : "Cut failed: " + why;
                return false;
            }
            parts.push_back(std::make_unique<FeatureCommand>(faceObject_, std::move(chainBefore),
                                                             target->features, label));
            ownerDone = faceObject_;
        }

        std::string error;
        if (!applyExtrude(scene, solid, op, bodies, ownerDone, primitiveName(kind_), label, parts, error)) {
            unwind(scene, parts);
            lastError_ = error;
            return false;
        }
        if (parts.empty()) {
            lastError_ = std::string(extrudeOpName(op)) + " changed nothing: it does not reach into those bodies";
            return false;
        }
        if (parts.size() == 1) undo.push(std::move(parts.front()));
        else                   undo.push(std::make_unique<CompositeCommand>(std::move(parts), label));
        if (scene.find(bodies.front())) scene.select(bodies.front());
        return true;
    }

    // Add as new standalone object in scene
    ObjectId id = kNoObject;
    const bool isSharpBox = (kind_ == PrimitiveKind::Box &&
                             cornerRadii_[0] < 1e-4 && cornerRadii_[1] < 1e-4 &&
                             cornerRadii_[2] < 1e-4 && cornerRadii_[3] < 1e-4);

    // The plane's own axes, as a rotation. A box drawn on the front plane is
    // still a box: it used to be baked into a mesh, and lose its dimensions
    // from the Inspector, purely because nothing recorded which way it faced.
    const Quat orientation = Quat::fromFrame(planeU_, planeV_, planeNormal_);

    if (isSharpBox) {
        PrimitiveSpec spec;
        spec.kind = PrimitiveKind::Box;
        spec.box.width = currentWidth_;
        spec.box.depth = currentDepth_;
        spec.box.height = std::fabs(depth);
        const Vec2 centerUV = (pt1_ + pt2_) * 0.5f;
        const Vec3 pos = planeOrigin_ + planeU_ * centerUV.x + planeV_ * centerUV.y +
                         planeNormal_ * (depth * 0.5f);
        id = scene.addPrimitive(PrimitiveKind::Box, spec, pos);
        scene.setBasePlacement(id, Transform{pos, orientation, {1, 1, 1}});
    } else if (kind_ == PrimitiveKind::Cylinder) {
        PrimitiveSpec spec;
        spec.kind = PrimitiveKind::Cylinder;
        spec.cylinder.radius = currentRadius_;
        spec.cylinder.height = std::fabs(depth);
        const Vec3 pos = planeOrigin_ + planeU_ * pt1_.x + planeV_ * pt1_.y +
                         planeNormal_ * (depth * 0.5f);
        id = scene.addPrimitive(PrimitiveKind::Cylinder, spec, pos);
        scene.setBasePlacement(id, Transform{pos, orientation, {1, 1, 1}});
    } else {
        // Any rounded rectangle with fillet, or arbitrary plane, or custom profile
        id = scene.addBody(solid, {0, 0, 0}, kind_ == PrimitiveKind::Box ? "Box" : "Cylinder");
    }

    if (id == kNoObject) {
        lastError_ = "The new body could not be made";
        return false;
    }
    undo.push(ExistenceCommand::forCreate(scene, {id}));
    scene.select(id);
    return true;
}

// ---------------------------------------------------------------------------
// 3D Viewport Overlays
// ---------------------------------------------------------------------------

void CreateTool::drawOverlay(const Scene& scene, const Camera& camera, Renderer& renderer) const {
    if (stage_ == CreateStage::None || stage_ == CreateStage::Applied) return;

    // What the cursor has caught, and what it was inferred from. See
    // app/snap_overlay.h for the shapes and why they are those shapes.
    drawSnapIndicator(renderer, camera, activeSnap_);

    if (stage_ == CreateStage::SelectPlane) {
        picker_.drawOverlay(scene, camera, renderer);
        return;
    }

    // The sketch grid, at the levels the snapper pulls to.
    //
    // It used to be a fixed 10mm step across a fixed 160mm square, which was
    // the wrong grid at every zoom but one and -- worse -- was not the grid the
    // point actually landed on. A line you can see that the cursor passes
    // straight through teaches the user not to trust the grid at all.
    const Real gridSpan = std::max<Real>(camera.distance * 1.6, 20.0);
    const GridLevels levels = gridLevelsAt(camera, planeOrigin_);
    auto drawLevel = [&](Real stepMm, Vec4 col) {
        if (stepMm <= 0.0) return false;
        const int n = static_cast<int>(gridSpan / stepMm);
        if (n > 240) return false;          // denser than it can be read; leave it out
        for (int i = -n; i <= n; ++i) {
            const Real t = static_cast<Real>(i) * stepMm;
            renderer.addLine(planeOrigin_ + planeU_ * t - planeV_ * gridSpan,
                             planeOrigin_ + planeU_ * t + planeV_ * gridSpan, col);
            renderer.addLine(planeOrigin_ - planeU_ * gridSpan + planeV_ * t,
                             planeOrigin_ + planeU_ * gridSpan + planeV_ * t, col);
        }
        return true;
    };
    // Exactly the lines the snapper will call a snap, and no others.
    //
    // gridLevelsAt picks `main` so its cells stay at least twelve pixels wide,
    // which is the coarsest thing a person can still land on by hand; anything
    // finer the snapper steps through without claiming, so drawing it would
    // promise a line that means nothing. One level to land on and the decade
    // above it to count by.
    const Real ladder[3] = {levels.main, levels.major, levels.major * 10.0};
    for (int i = 0; i < 2; ++i) {
        if (drawLevel(ladder[i], Vec4{0.45f, 0.55f, 0.65f, 0.13f})) {
            drawLevel(ladder[i + 1], Vec4{0.50f, 0.60f, 0.70f, 0.28f});
            break;
        }
    }
    // Main U/V axes on the plane
    renderer.addLine(planeOrigin_ - planeU_ * gridSpan, planeOrigin_ + planeU_ * gridSpan, Vec4{0.9f, 0.3f, 0.3f, 0.4f});
    renderer.addLine(planeOrigin_ - planeV_ * gridSpan, planeOrigin_ + planeV_ * gridSpan, Vec4{0.3f, 0.8f, 0.3f, 0.4f});

    // Draw 2D Profile outline & fill
    const std::vector<Vec2> prof = getCurrentProfile();
    if (prof.size() >= 3) {
        const Vec4 outlineCol = toVec4(palette::kBrand, 0.95f);
        const Vec4 fillCol = (stage_ == CreateStage::ExtrudeDepth &&
                               (choice_.op == ExtrudeOp::Cut || choice_.op == ExtrudeOp::Intersect))
                                  ? kCutCol : kCreateCol;

        // Boundary lines
        for (size_t i = 0; i < prof.size(); ++i) {
            const size_t next = (i + 1) % prof.size();
            const Vec3 pA = planeOrigin_ + planeU_ * prof[i].x + planeV_ * prof[i].y;
            const Vec3 pB = planeOrigin_ + planeU_ * prof[next].x + planeV_ * prof[next].y;
            renderer.addLine(pA, pB, outlineCol);
        }

        // 2D fan fill on plane
        const Vec3 pCenter = planeOrigin_ + planeU_ * prof[0].x + planeV_ * prof[0].y;
        for (size_t i = 1; i + 1 < prof.size(); ++i) {
            const Vec3 pA = planeOrigin_ + planeU_ * prof[i].x + planeV_ * prof[i].y;
            const Vec3 pB = planeOrigin_ + planeU_ * prof[i + 1].x + planeV_ * prof[i + 1].y;
            renderer.addTriangle(pCenter, pA, pB, Vec4{fillCol.x, fillCol.y, fillCol.z, 0.15f});
        }
    }

    // Handles in AdjustProfile stage.
    //
    // Each shape says what pulling on it does, because they no longer all do
    // the same thing: a square corner resizes both ways, a bar moves the side
    // it lies along, and the cross in the middle slides the whole profile. They
    // used to be nine identical crosses, four of which quietly rounded a corner
    // instead of moving it.
    if (stage_ == CreateStage::AdjustProfile) {
        using overlay::frameAt;
        const Vec4 idle = toVec4(palette::kBrand, 0.92f);
        const Vec4 lit{1.0f, 0.82f, 0.35f, 1.0f};
        const Vec4 dark{0.10f, 0.10f, 0.11f, 0.95f};

        auto state = [&](HandleId id) {
            return hoveredHandle_ == id || activeHandle_ == id;
        };

        // Filled, with a dark rim: a profile is drawn over a face that may be
        // any colour, and an outline alone disappears against a light one.
        auto corner = [&](Vec3 pos, HandleId id) {
            const bool on = state(id);
            overlay::square(renderer, camera, pos, frameAt(camera, pos),
                            on ? 5.6 : 4.4, on ? lit : idle, dark, 1.6);
        };

        // A bar lying along the edge it moves, so which side it belongs to is
        // never in doubt.
        auto edge = [&](Vec3 pos, HandleId id, bool horizontal) {
            const bool on = state(id);
            const Real half = on ? 11.0 : 9.0, thick = on ? 3.0 : 2.4;
            const Real ax = horizontal ? half : thick;
            const Real ay = horizontal ? thick : half;
            const Vec2 pts[4] = {{-ax, -ay}, {ax, -ay}, {ax, ay}, {-ax, ay}};
            overlay::filled(renderer, pos, frameAt(camera, pos), pts, 4, on ? lit : idle);
        };

        if (kind_ == PrimitiveKind::Cylinder) {
            const Vec3 rPt = planeOrigin_ + planeU_ * (pt1_.x + currentRadius_) + planeV_ * pt1_.y;
            const bool on = state(HandleId::RadiusHandle);
            overlay::disc(renderer, rPt, frameAt(camera, rPt), on ? 5.0 : 3.8, on ? lit : idle);
        } else {
            const Real uMin = pt1_.x, uMax = pt2_.x;
            const Real vMin = pt1_.y, vMax = pt2_.y;

            const Vec3 c0 = planeOrigin_ + planeU_ * uMax + planeV_ * vMin; // BR
            const Vec3 c1 = planeOrigin_ + planeU_ * uMax + planeV_ * vMax; // TR
            const Vec3 c2 = planeOrigin_ + planeU_ * uMin + planeV_ * vMax; // TL
            const Vec3 c3 = planeOrigin_ + planeU_ * uMin + planeV_ * vMin; // BL

            corner(c0, HandleId::Corner0);
            corner(c1, HandleId::Corner1);
            corner(c2, HandleId::Corner2);
            corner(c3, HandleId::Corner3);

            edge((c0 + c3) * 0.5f, HandleId::EdgeBottom, true);
            edge((c0 + c1) * 0.5f, HandleId::EdgeRight,  false);
            edge((c1 + c2) * 0.5f, HandleId::EdgeTop,    true);
            edge((c2 + c3) * 0.5f, HandleId::EdgeLeft,   false);

            // The four-way cross every tool uses for "move this".
            const Vec3 mFace = (c0 + c2) * 0.5f;
            const bool onFace = state(HandleId::FaceCenter);
            const overlay::ScreenFrame ff = frameAt(camera, mFace);
            const Real arm = onFace ? 11.0 : 9.0;
            const Vec4 fc = onFace ? lit : idle;
            renderer.addFrontLine(camera, mFace - ff.right * (arm - 3.0),
                                  mFace + ff.right * (arm - 3.0), fc, 2.6);
            renderer.addFrontLine(camera, mFace - ff.up * (arm - 3.0),
                                  mFace + ff.up * (arm - 3.0), fc, 2.6);
            for (int k = 0; k < 4; ++k) {
                const Vec3 dir = (k & 1) ? ff.up : ff.right;
                const Real sgn = (k & 2) ? -1.0 : 1.0;
                const Vec3 across = (k & 1) ? ff.right : ff.up;
                const Vec3 tip = mFace + dir * (arm * sgn);
                renderer.addFrontTriangle(tip, tip - dir * (sgn * 4.5) + across * 3.6,
                                          tip - dir * (sgn * 4.5) - across * 3.6, fc);
            }
        }

        // While a corner is being rounded, a ring on the corner that is
        // listening. Not the arc as well: the profile outline already draws
        // exactly that curve, and the same line in two colours reads as a bug.
        if (isFilleting_) {
            const Vec3 ref = planeOrigin_ + planeU_ * filletRefUV_.x + planeV_ * filletRefUV_.y;
            overlay::ring(renderer, camera, ref, overlay::frameAt(camera, ref), 8.0,
                          Vec4{1.0f, 0.82f, 0.35f, 1.0f}, 2.2);
        }
    }

    // 3D Extrusion Solid Preview
    if (stage_ == CreateStage::ExtrudeDepth && std::fabs(extrudeDepth_) > 0.05) {
        // The preview is drawn every frame, so it is built the cheap way even
        // when the scene is exact: what it shows is the same shape, and the
        // solid that gets committed is built exactly. If the exact path ever
        // becomes cheap enough to run per frame, this is the only line that
        // has to change.
        const Body solid = buildCurrentSolid(extrudeDepth_, Backend::Mesh);
        if (!solid.empty()) {
            // Red for what takes material away, the brand colour for what adds it.
            const bool removes = choice_.op == ExtrudeOp::Cut || choice_.op == ExtrudeOp::Intersect;
            const Vec4 wireCol = removes ? Vec4{1.0f, 0.4f, 0.3f, 0.9f} : toVec4(palette::kBrand, 0.9f);
            const Vec4 faceTint = removes ? kCutCol : kCreateCol;

            std::vector<EdgeId> edges;
            solid.allEdges(edges);
            for (EdgeId e : edges) {
                Vec3 pA, pB;
                solid.edgePositions(e, pA, pB);
                renderer.addLine(pA, pB, wireCol);
            }

            std::vector<FaceId> faces;
            std::vector<VertexId> fv;
            solid.allFaces(faces);
            for (FaceId f : faces) {
                solid.faceVertices(f, fv);
                if (fv.size() < 3) continue;
                const Vec3 v0 = solid.vertexPosition(fv[0]);
                for (size_t i = 1; i + 1 < fv.size(); ++i)
                    renderer.addTriangle(v0, solid.vertexPosition(fv[i]),
                                         solid.vertexPosition(fv[i + 1]), faceTint);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// ImGui 2D HUD Overlays
// ---------------------------------------------------------------------------

// One row, whatever the stage is asking for.
//
// A dimension the keyboard is holding is shown in brackets and in the accent
// colour; one the mouse still drives is plain. Without that the two are
// indistinguishable, and a user who typed a width has no way of knowing why it
// stopped following the pointer.
void CreateTool::drawDimensionFields() {
    const int n = fieldCount();
    for (int f = 0; f < n; ++f) {
        if (f) ImGui::SameLine(0.0f, 16.0f);
        ImGui::TextDisabled("%s", fieldName(f));
        ImGui::SameLine(0.0f, 6.0f);

        if (f == typedField_ && typing()) {
            ImGui::TextColored(kAccentIm, "%s_", typedValue_.c_str());
        } else if (fieldFixed_[f]) {
            ImGui::TextColored(kAccentIm, "[%.2f]", fieldDisplay(f));
        } else {
            ImGui::Text("%.2f", fieldDisplay(f));
        }
    }
}

bool CreateTool::drawHud(Scene& scene, Camera& camera, UndoStack& undo, bool& outFinished) {
    if (stage_ == CreateStage::None) return false;
    outFinished = false;

    // The panel says what is being done now, which during a round is not the
    // same as what the command is called.
    const bool rounding = stage_ == CreateStage::AdjustProfile && isFilleting_;
    const Glyph icon = rounding                           ? Glyph::Fillet
                     : kind_ == PrimitiveKind::Cylinder   ? Glyph::Cylinder
                     : kind_ == PrimitiveKind::Sphere     ? Glyph::Sphere
                     : kind_ == PrimitiveKind::Cone       ? Glyph::Cone
                     : kind_ == PrimitiveKind::Torus      ? Glyph::Torus
                                                          : Glyph::Box;
    char title[64];
    if (rounding)
        std::snprintf(title, sizeof title, "Round %s",
                      activeFilletCorners_.size() > 1 ? "Corners" : "Corner");
    else
        std::snprintf(title, sizeof title, "Create %s", primitiveName(kind_));

    if (!ui::beginCommand("##create", title, icon)) return true;

    // How far a bar reaches. What is being drawn is being drawn in this view,
    // so the view's own height is the natural extent for a side or a depth:
    // it does not move while the bar is pulled, and it grows when the user
    // zooms out to draw something bigger. A corner round is bounded by the
    // profile it sits in, the same bound setField applies.
    const double extent = niceStepAbove(static_cast<double>(camera.orthoHeight()) * 0.5);
    const double fieldMax = (stage_ == CreateStage::AdjustProfile && isFilleting_)
                                ? std::min(currentWidth_, currentDepth_) * 0.499
                                : extent;

    // A number pulled on the panel is a number typed: the bar fixes the field
    // the way the keyboard would, and the pointer lets go of it.
    auto pulled = [&](int f, const ui::NumberEdit& e) {
        if (e.dragged) {
            char b[48];
            std::snprintf(b, sizeof b, "%.6g", e.value);
            typedField_ = f;
            typedValue_ = b;
            syncTypedField();
            typedValue_.clear();
        } else if (e.clicked) {
            typedField_ = f;
        }
    };

    int footer = 0;
    switch (stage_) {
    // -----------------------------------------------------------------------
    case CreateStage::SelectPlane: {
        if (picker_.drawRows("Click a face of a body to draw on it, or an origin plane."))
            choosePlane(picker_.choice(), camera, scene);
        footer = ui::commandFooter(nullptr);
        break;
    }

    // -----------------------------------------------------------------------
    case CreateStage::DrawProfile_Pt1: {
        char at[64];
        std::snprintf(at, sizeof at, "%.2f, %.2f", pt1_.x, pt1_.y);
        ui::commandValue(kind_ == PrimitiveKind::Cylinder ? "Centre" : "Corner", at);
        // Where the plane stands and how it leans: the same two rows a
        // sketch's plane has, before anything is drawn on it.
        {
            const double span = std::max(extent, std::fabs(planeOffset_));
            const ui::NumberEdit e = ui::commandNumber("Offset", planeOffset_, "mm", false, false, nullptr,
                                                       -span, span, true);
            if (e.dragged) { planeOffset_ = e.value; applyPlaneShift(); }
            const ui::NumberEdit t = ui::commandNumber("Tilt", planeTilt_ * kRad2Deg, "\xC2\xB0", false, false,
                                                       nullptr, -90.0, 90.0, true);
            if (t.dragged) { planeTilt_ = std::clamp(t.value, -90.0, 90.0) * kDeg2Rad; applyPlaneShift(); }
            if (e.released || t.released) {
                float y = 0.0f, p = 0.0f;
                Camera::anglesFor(planeNormal_, y, p);
                camera.animateTo(planeOrigin_, camera.distance, y, p);
            }
        }
        ui::commandHint("Click to place it.  Ctrl for free placement.");
        footer = ui::commandFooter(nullptr);
        break;
    }

    // -----------------------------------------------------------------------
    case CreateStage::DrawProfile_Pt2: {
        for (int f = 0; f < fieldCount(); ++f)
            pulled(f, ui::commandNumber(fieldName(f), fieldDisplay(f), "mm", fieldFixed_[f],
                                        f == typedField_ && typing(), typedValue_.c_str(),
                                        0.0, fieldMax));
        ui::commandHint(fieldCount() > 1
            ? "Type a number to fix a side; the other still follows the mouse.  Tab next, Enter confirm."
            : "Type a number to fix the radius.  Enter confirms.");
        footer = ui::commandFooter(nullptr);
        break;
    }

    // -----------------------------------------------------------------------
    case CreateStage::AdjustProfile: {
        for (int f = 0; f < fieldCount(); ++f)
            pulled(f, ui::commandNumber(fieldName(f), fieldDisplay(f), "mm", fieldFixed_[f],
                                        f == typedField_ && typing(), typedValue_.c_str(),
                                        0.0, fieldMax));

        if (kind_ != PrimitiveKind::Cylinder && !isFilleting_) {
            ui::commandRow("Corners");
            char r[32];
            std::snprintf(r, sizeof r, "%.2f mm", uniformCornerRadius());
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(r);
            ImGui::SameLine();
            if (ui::pillButton("Round all", false)) {
                isFilleting_ = true;
                isDragging_ = false;
                activeHandle_ = HandleId::FaceCenter;
                filletRefUV_ = (pt1_ + pt2_) * 0.5;
                activeFilletCorners_ = {0, 1, 2, 3};
                for (int i = 0; i < 4; ++i) dragStartFillets_[i] = cornerRadii_[i];
                clearFields();
            }
        }

        ui::commandHint(isFilleting_
            ? "Move away from the corner to open it out.  Click or Enter confirms, Esc puts it back."
            : "Drag a handle to move it.  Hover one and press F to round that corner.");
        footer = ui::commandFooter(isFilleting_ ? "Done" : "Extrude");
        break;
    }

    // -----------------------------------------------------------------------
    case CreateStage::ExtrudeDepth:
    case CreateStage::Applied: {
        const bool applied = stage_ == CreateStage::Applied;
        {
            // Either way. As far as the view is tall.
            const double span = std::max(extent, std::fabs(extrudeDepth_));
            const ui::NumberEdit e = ui::commandNumber("Depth", extrudeDepth_, "mm", fieldFixed_[0],
                                                       !applied && typing(), typedValue_.c_str(),
                                                       -span, span, true);
            if (applied) {
                // Nothing follows the pointer now: the bar sets the depth
                // itself, and the extrusion is made again at it.
                if (e.dragged && std::fabs(e.value) > 1e-4 && e.value != extrudeDepth_) {
                    extrudeDepth_ = e.value;
                    adjusted_ = true;
                }
            } else {
                pulled(0, e);
            }
        }

        if (drawExtrudeChoice(choice_)) {
            refreshReach(scene);
            if (applied) adjusted_ = true;
        }
        if (applied) refreshReach(scene);
        const ObjectId toggled = drawReachedBodies(scene, reach_, choice_.op, faceObject_);
        if (toggled != kNoObject) {
            reach_.toggle(toggled);
            if (applied) adjusted_ = true;
        }

        if (applied) ui::commandApplied("Extrude");
        ui::commandHint(applied ? "Change the depth, the operation or the bodies, and it is made again."
                                : "Move to set the depth, drag the bar, or type one.  Click a body above to leave it out.");
        footer = applied ? ui::commandFooter("Done", true, nullptr) : ui::commandFooter("Finish");
        break;
    }

    case CreateStage::None:
        break;
    }

    if (footer > 0 && stage_ == CreateStage::Applied) {
        stage_ = CreateStage::None;
        outFinished = true;
    } else if (footer > 0) {
        // The same path the keys take, so there is one answer to what a step
        // means rather than two that can drift apart.
        handleKey(13, false, false, camera, scene, undo);
        if (!active()) outFinished = true;
    } else if (footer < 0) {
        cancel(camera);
    }

    ui::endCommand();
    return true;
}

} // namespace tg
