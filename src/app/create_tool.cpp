#include "app/create_tool.h"

#include "core/palette.h"
#include "imgui.h"
#include "mesh/boolean.h"
#include "mesh/health.h"
#include "mesh/operations.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace tg {

namespace {

inline Vec4 toV4(Rgb c, float a = 1.0f) {
    return Vec4{static_cast<float>(c.r), static_cast<float>(c.g),
                static_cast<float>(c.b), a};
}

const ImVec4 kAccentIm(palette::kBrand.r, palette::kBrand.g, palette::kBrand.b, 1.0f);
const Vec4 kAccentCol(palette::kBrand.r, palette::kBrand.g, palette::kBrand.b, 1.0f);
const Vec4 kCutCol(0.95f, 0.35f, 0.25f, 0.6f);
const Vec4 kCreateCol(palette::kBrand.r, palette::kBrand.g, palette::kBrand.b, 0.6f);

// Ray-Plane intersection: returns true and parameter t if ray intersects plane (P0, N)
bool intersectRayPlane(const Ray& ray, Vec3 p0, Vec3 n, float& outT, Vec3& outPt) {
    const float denom = dot(ray.dir, n);
    if (std::fabs(denom) < 1e-6f) return false;
    const float t = dot(p0 - ray.origin, n) / denom;
    if (t < 0.0f) return false;
    outT = t;
    outPt = ray.origin + ray.dir * t;
    return true;
}

// Check if a point P on the plane (in UV) lies within a square tile of size [-L, L]
bool pointInTile(Vec3 p, Vec3 p0, Vec3 u, Vec3 v, float halfSize) {
    const Vec3 d = p - p0;
    const float uCoord = dot(d, u);
    const float vCoord = dot(d, v);
    return std::fabs(uCoord) <= halfSize && std::fabs(vCoord) <= halfSize;
}

} // namespace

// ---------------------------------------------------------------------------
// 2D Polygon & 3D Prism Generators
// ---------------------------------------------------------------------------

std::vector<Vec2> CreateTool::makeRectPolygon(Vec2 p1, Vec2 p2, Real cornerRadius, int arcSegments) {
    const Real uMin = std::min(p1.x, p2.x);
    const Real uMax = std::max(p1.x, p2.x);
    const Real vMin = std::min(p1.y, p2.y);
    const Real vMax = std::max(p1.y, p2.y);

    const Real w = uMax - uMin;
    const Real h = vMax - vMin;
    if (w < 1e-4 || h < 1e-4) return {};

    const Real maxR = std::min(w, h) * 0.499f;
    const Real r = clampf(cornerRadius, 0.0f, maxR);

    std::vector<Vec2> out;
    if (r < 1e-4 || arcSegments < 1) {
        // Sharp rectangle (CCW)
        out.push_back({uMin, vMin});
        out.push_back({uMax, vMin});
        out.push_back({uMax, vMax});
        out.push_back({uMin, vMax});
        return out;
    }

    // 4 rounded corners:
    auto addCorner = [&](Vec2 center, float startAngle, float endAngle) {
        for (int i = 0; i <= arcSegments; ++i) {
            const float t = static_cast<float>(i) / static_cast<float>(arcSegments);
            const float a = startAngle + (endAngle - startAngle) * t;
            out.push_back({center.x + r * std::cos(a), center.y + r * std::sin(a)});
        }
    };

    // Bottom edge -> bottom-right corner
    addCorner({uMax - r, vMin + r}, -kHalfPi, 0.0f);
    // Right edge -> top-right corner
    addCorner({uMax - r, vMax - r}, 0.0f, kHalfPi);
    // Top edge -> top-left corner
    addCorner({uMin + r, vMax - r}, kHalfPi, kPi);
    // Left edge -> bottom-left corner
    addCorner({uMin + r, vMin + r}, kPi, 3.0f * kHalfPi);

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
    if (std::fabs(z1 - z0) < 1e-5) return false;

    const Real zBot = std::min(z0, z1);
    const Real zTop = std::max(z0, z1);

    std::vector<Vec3> positions;
    positions.reserve(n * 2);

    // Bottom ring: 0 .. n-1
    for (size_t i = 0; i < n; ++i) {
        positions.push_back(planeOrigin + planeU * poly2D[i].x + planeV * poly2D[i].y + planeN * zBot);
    }
    // Top ring: n .. 2*n-1
    for (size_t i = 0; i < n; ++i) {
        positions.push_back(planeOrigin + planeU * poly2D[i].x + planeV * poly2D[i].y + planeN * zTop);
    }

    std::vector<uint32_t> faceSizes;
    std::vector<uint32_t> faceIndices;

    // Bottom cap: outward normal is -N, so reverse winding (CW viewed from top)
    faceSizes.push_back(static_cast<int>(n));
    for (size_t i = n; i > 0; --i) {
        faceIndices.push_back(static_cast<uint32_t>(i - 1));
    }

    // Top cap: outward normal is +N, so CCW winding
    faceSizes.push_back(static_cast<int>(n));
    for (size_t i = 0; i < n; ++i) {
        faceIndices.push_back(static_cast<uint32_t>(n + i));
    }

    // Side wall quads around perimeter
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
// Plane Basis & Projections
// ---------------------------------------------------------------------------

void CreateTool::computePlaneBasis(Vec3 normal) {
    planeNormal_ = normalize(normal);
    // Pick an intuitive tangent U aligned with world axes
    if (std::fabs(planeNormal_.z) > 0.9f) {
        // Top or bottom plane: U = +X, V = +Y (or -Y)
        planeU_ = Vec3{1, 0, 0};
        planeV_ = normalize(cross(planeNormal_, planeU_));
    } else if (std::fabs(planeNormal_.y) > 0.9f) {
        // Front or back plane: U = +X, V = +Z
        planeU_ = Vec3{1, 0, 0};
        planeV_ = normalize(cross(planeNormal_, planeU_));
    } else if (std::fabs(planeNormal_.x) > 0.9f) {
        // Side plane: U = +Y, V = +Z
        planeU_ = Vec3{0, 1, 0};
        planeV_ = normalize(cross(planeNormal_, planeU_));
    } else {
        // Arbitrary angled face
        const Vec3 seed = std::fabs(planeNormal_.z) < 0.8f ? Vec3{0, 0, 1} : Vec3{1, 0, 0};
        planeU_ = normalize(cross(seed, planeNormal_));
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
    // Project mouse onto the extrusion axis line: P(t) = P_center + t * planeNormal_
    const Vec2 centerUV = (pt1_ + pt2_) * 0.5f;
    const Vec3 pCenter = planeOrigin_ + planeU_ * centerUV.x + planeV_ * centerUV.y;
    const Ray ray = camera.rayThroughPixel(mousePx.x, mousePx.y);

    // Ray-Line closest approach
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
        return makeRectPolygon(pt1_, pt2_, cornerRadius_, 6);
    }
}

Mesh CreateTool::buildCurrentSolid(Real depth) const {
    Mesh out;
    const std::vector<Vec2> prof = getCurrentProfile();
    if (prof.empty() || std::fabs(depth) < 1e-4) return out;

    if (depth > 0.0) {
        makePrismMesh(prof, planeOrigin_, planeU_, planeV_, planeNormal_, 0.0, depth, out);
    } else {
        makePrismMesh(prof, planeOrigin_, planeU_, planeV_, planeNormal_, depth, 0.0, out);
    }
    return out;
}

// ---------------------------------------------------------------------------
// Tool State Lifecycle
// ---------------------------------------------------------------------------

void CreateTool::start(PrimitiveKind kind) {
    kind_ = kind;
    stage_ = CreateStage::SelectPlane;
    hoveredPlane_ = PlaneChoice::XY;
    selectedPlane_ = PlaneChoice::None;
    planeOrigin_ = Vec3{0, 0, 0};
    planeNormal_ = Vec3{0, 0, 1};
    computePlaneBasis(planeNormal_);
    faceObject_ = kNoObject;
    faceIndex_ = kInvalid;

    pt1_ = Vec2{0, 0};
    pt2_ = Vec2{20, 20};
    cornerRadius_ = 0.0;
    currentRadius_ = 10.0;
    currentWidth_ = 20.0;
    currentDepth_ = 20.0;
    extrudeDepth_ = 20.0;
    hoveredHandle_ = HandleId::None;
    activeHandle_ = HandleId::None;
    typedValue_.clear();
}

void CreateTool::cancel(Camera& camera) {
    if (stage_ == CreateStage::DrawProfile_Pt1 || stage_ == CreateStage::DrawProfile_Pt2) {
        // Restore perspective camera if canceling from orthographic
        camera.target = savedCamera_.target;
        camera.distance = savedCamera_.distance;
        camera.yaw = savedCamera_.yaw;
        camera.pitch = savedCamera_.pitch;
        camera.orthographic = savedCamera_.orthographic;
        camera.snapToGoal();
    }
    stage_ = CreateStage::None;
    activeHandle_ = HandleId::None;
    hoveredHandle_ = HandleId::None;
    typedValue_.clear();
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

void CreateTool::commitPlaneSelection(Camera& camera) {
    if (stage_ != CreateStage::SelectPlane) return;
    selectedPlane_ = hoveredPlane_;

    // Save camera perspective before switching to orthographic head-on
    savedCamera_.target = camera.target;
    savedCamera_.distance = camera.distance;
    savedCamera_.yaw = camera.yaw;
    savedCamera_.pitch = camera.pitch;
    savedCamera_.orthographic = camera.orthographic;

    // Set camera target to plane center and face head-on
    camera.target = planeOrigin_;
    const float p = std::asin(clampf(planeNormal_.z, -1.0f, 1.0f));
    const float y = std::atan2(planeNormal_.x, -planeNormal_.y);
    camera.pitch = clampf(p, -1.5705f, 1.5705f);
    camera.yaw = y;
    camera.orthographic = true;
    camera.snapToGoal();

    stage_ = CreateStage::DrawProfile_Pt1;
    pt1_ = Vec2{0, 0};
    pt2_ = Vec2{0, 0};
}

void CreateTool::choosePlane(PlaneChoice choice, Camera& camera, const Scene& scene) {
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
            break;
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

    if (stage_ == CreateStage::SelectPlane) {
        // Raycast against scene objects first
        const Ray ray = camera.rayThroughPixel(mousePx.x, mousePx.y);
        const RayHit hit = scene.raycast(ray);
        if (hit.hit() && hit.face != kInvalid) {
            const SceneObject* o = scene.find(hit.object);
            if (o) {
                const Mat4 model = o->modelMatrix();
                const Vec3 localN = o->mesh.faceNormal(hit.face);
                const Vec3 worldN = normalize(transformVector(normalMatrix(model), localN));

                // Compute face center in local space and transform to world space
                std::vector<Index> fv;
                o->mesh.faceVertices(hit.face, fv);
                Vec3 localCenter{0, 0, 0};
                for (Index v : fv) localCenter += o->mesh.verts[v].position;
                if (!fv.empty()) localCenter *= (1.0f / static_cast<float>(fv.size()));
                const Vec3 worldCenter = transformPoint(model, localCenter);

                setHoveredPlane(PlaneChoice::Face, worldCenter, worldN, hit.object, hit.face);
                return;
            }
        }

        // Raycast against the 3 origin plane tiles (size 40mm)
        const float tileSize = std::max(camera.distance * 0.35f, 25.0f);
        float tXY = 1e9f, tXZ = 1e9f, tYZ = 1e9f;
        Vec3 pXY{}, pXZ{}, pYZ{};
        const bool hitXY = intersectRayPlane(ray, {0, 0, 0}, {0, 0, 1}, tXY, pXY) &&
                           pointInTile(pXY, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, tileSize);
        const bool hitXZ = intersectRayPlane(ray, {0, 0, 0}, {0, 1, 0}, tXZ, pXZ) &&
                           pointInTile(pXZ, {0, 0, 0}, {1, 0, 0}, {0, 0, 1}, tileSize);
        const bool hitYZ = intersectRayPlane(ray, {0, 0, 0}, {1, 0, 0}, tYZ, pYZ) &&
                           pointInTile(pYZ, {0, 0, 0}, {0, 1, 0}, {0, 0, 1}, tileSize);

        float bestT = 1e9f;
        PlaneChoice bestChoice = PlaneChoice::XY;
        Vec3 bestPt{0, 0, 0};
        Vec3 bestNorm{0, 0, 1};

        if (hitXY && tXY < bestT) { bestT = tXY; bestChoice = PlaneChoice::XY; bestPt = pXY; bestNorm = {0, 0, 1}; }
        if (hitXZ && tXZ < bestT) { bestT = tXZ; bestChoice = PlaneChoice::XZ; bestPt = pXZ; bestNorm = {0, -1, 0}; }
        if (hitYZ && tYZ < bestT) { bestT = tYZ; bestChoice = PlaneChoice::YZ; bestPt = pYZ; bestNorm = {1, 0, 0}; }

        if (bestT < 1e8f) {
            setHoveredPlane(bestChoice, {0, 0, 0}, bestNorm);
        } else {
            // Default to XY plane
            setHoveredPlane(PlaneChoice::XY, {0, 0, 0}, {0, 0, 1});
        }
        return;
    }

    if (stage_ == CreateStage::DrawProfile_Pt1) {
        Vec2 uv{0, 0};
        if (unprojectToPlane(camera, mousePx, uv)) {
            if (snap) {
                const Real gridStep = 5.0;
                uv.x = std::round(uv.x / gridStep) * gridStep;
                uv.y = std::round(uv.y / gridStep) * gridStep;
            }
            pt1_ = uv;
            pt2_ = uv;
        }
        return;
    }

    if (stage_ == CreateStage::DrawProfile_Pt2) {
        Vec2 uv{0, 0};
        if (unprojectToPlane(camera, mousePx, uv)) {
            if (snap) {
                const Real gridStep = 5.0;
                uv.x = std::round(uv.x / gridStep) * gridStep;
                uv.y = std::round(uv.y / gridStep) * gridStep;
            }
            pt2_ = uv;
            if (kind_ == PrimitiveKind::Cylinder) {
                currentRadius_ = std::max(length(pt2_ - pt1_), Real(1.0));
            } else {
                currentWidth_ = std::max(std::fabs(pt2_.x - pt1_.x), Real(1.0));
                currentDepth_ = std::max(std::fabs(pt2_.y - pt1_.y), Real(1.0));
            }
        }
        return;
    }

    if (stage_ == CreateStage::AdjustProfile) {
        const Real uMin = std::min(pt1_.x, pt2_.x);
        const Real uMax = std::max(pt1_.x, pt2_.x);
        const Real vMin = std::min(pt1_.y, pt2_.y);
        const Real vMax = std::max(pt1_.y, pt2_.y);

        Vec2 currentUV{};
        unprojectToPlane(camera, mousePx, currentUV);

        if (activeHandle_ != HandleId::None) {
            // Dragging active handle
            if (kind_ == PrimitiveKind::Cylinder) {
                currentRadius_ = std::max(length(currentUV - pt1_), Real(1.0));
                if (snap) currentRadius_ = std::round(currentRadius_ / 1.0) * 1.0;
                pt2_ = pt1_ + Vec2{currentRadius_, 0};
            } else {
                if (activeHandle_ == HandleId::EdgeLeft) {
                    pt1_.x = currentUV.x;
                } else if (activeHandle_ == HandleId::EdgeRight) {
                    pt2_.x = currentUV.x;
                } else if (activeHandle_ == HandleId::EdgeBottom) {
                    pt1_.y = currentUV.y;
                } else if (activeHandle_ == HandleId::EdgeTop) {
                    pt2_.y = currentUV.y;
                } else if (activeHandle_ >= HandleId::Corner0 && activeHandle_ <= HandleId::Corner3) {
                    const Real maxDim = std::min(std::fabs(pt2_.x - pt1_.x), std::fabs(pt2_.y - pt1_.y)) * 0.5;
                    cornerRadius_ = clampf(std::fabs(currentUV.x - uMin), 0.0, maxDim * 0.95);
                    if (snap) cornerRadius_ = std::round(cornerRadius_ / 1.0) * 1.0;
                }
                currentWidth_ = std::fabs(pt2_.x - pt1_.x);
                currentDepth_ = std::fabs(pt2_.y - pt1_.y);
            }
            return;
        }

        // Hover test for handles in screen space
        hoveredHandle_ = HandleId::None;
        auto testScreenHandle = [&](Vec3 worldPos, HandleId id) {
            Vec2 sp{};
            if (camera.projectToPixel(worldPos, sp)) {
                if (length(sp - mousePx) < 14.0f) {
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
            const Vec3 c0 = planeOrigin_ + planeU_ * uMin + planeV_ * vMin;
            const Vec3 c1 = planeOrigin_ + planeU_ * uMax + planeV_ * vMin;
            const Vec3 c2 = planeOrigin_ + planeU_ * uMax + planeV_ * vMax;
            const Vec3 c3 = planeOrigin_ + planeU_ * uMin + planeV_ * vMax;

            const Vec3 mBot   = (c0 + c1) * 0.5f;
            const Vec3 mRight = (c1 + c2) * 0.5f;
            const Vec3 mTop   = (c2 + c3) * 0.5f;
            const Vec3 mLeft  = (c3 + c0) * 0.5f;

            if (!testScreenHandle(c0, HandleId::Corner0))
                if (!testScreenHandle(c1, HandleId::Corner1))
                    if (!testScreenHandle(c2, HandleId::Corner2))
                        if (!testScreenHandle(c3, HandleId::Corner3))
                            if (!testScreenHandle(mLeft, HandleId::EdgeLeft))
                                if (!testScreenHandle(mRight, HandleId::EdgeRight))
                                    if (!testScreenHandle(mBot, HandleId::EdgeBottom))
                                        testScreenHandle(mTop, HandleId::EdgeTop);
        }
        return;
    }

    if (stage_ == CreateStage::ExtrudeDepth) {
        Real d = rayPlaneExtrudeDepth(camera, mousePx);
        if (snap) {
            d = std::round(d / 1.0) * 1.0;
        }
        if (std::fabs(d) > 0.1) extrudeDepth_ = d;
        return;
    }
}

// ---------------------------------------------------------------------------
// Mouse Clicks
// ---------------------------------------------------------------------------

void CreateTool::handleLeftClick(Scene& scene, Camera& camera, UndoStack& undo) {
    if (stage_ == CreateStage::SelectPlane) {
        commitPlaneSelection(camera);
        return;
    }

    if (stage_ == CreateStage::DrawProfile_Pt1) {
        stage_ = CreateStage::DrawProfile_Pt2;
        return;
    }

    if (stage_ == CreateStage::DrawProfile_Pt2) {
        // Point 2 set! Restore perspective camera and transition to AdjustProfile
        camera.target = savedCamera_.target;
        camera.distance = savedCamera_.distance;
        camera.yaw = savedCamera_.yaw;
        camera.pitch = savedCamera_.pitch;
        camera.orthographic = savedCamera_.orthographic;
        camera.snapToGoal();

        stage_ = CreateStage::AdjustProfile;
        return;
    }

    if (stage_ == CreateStage::AdjustProfile) {
        if (activeHandle_ != HandleId::None) {
            activeHandle_ = HandleId::None;
            return;
        }
        if (hoveredHandle_ != HandleId::None) {
            activeHandle_ = hoveredHandle_;
            return;
        }
        return;
    }

    if (stage_ == CreateStage::ExtrudeDepth) {
        finishCreation(scene, camera, undo);
        return;
    }
}

void CreateTool::handleRightClick(Camera& camera) {
    if (activeHandle_ != HandleId::None) {
        activeHandle_ = HandleId::None;
        return;
    }
    cancel(camera);
}

bool CreateTool::handleKey(int key, bool shift, bool ctrl, Camera& camera) {
    if (stage_ == CreateStage::None) return false;

    // Esc = Cancel
    if (key == 27) {
        cancel(camera);
        return true;
    }

    // Enter or Space = Advance / Commit
    if (key == 13 || key == 32) {
        if (stage_ == CreateStage::SelectPlane) {
            commitPlaneSelection(camera);
            return true;
        }
        if (stage_ == CreateStage::DrawProfile_Pt1) {
            stage_ = CreateStage::DrawProfile_Pt2;
            return true;
        }
        if (stage_ == CreateStage::DrawProfile_Pt2) {
            camera.target = savedCamera_.target;
            camera.distance = savedCamera_.distance;
            camera.yaw = savedCamera_.yaw;
            camera.pitch = savedCamera_.pitch;
            camera.orthographic = savedCamera_.orthographic;
            camera.snapToGoal();
            stage_ = CreateStage::AdjustProfile;
            return true;
        }
        if (stage_ == CreateStage::AdjustProfile) {
            stage_ = CreateStage::ExtrudeDepth;
            extrudeBaseDepth_ = 20.0;
            extrudeDepth_ = 20.0;
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

// ---------------------------------------------------------------------------
// Final Creation & Boolean Cut Execution
// ---------------------------------------------------------------------------

bool CreateTool::finishCreation(Scene& scene, Camera& camera, UndoStack& undo) {
    if (stage_ != CreateStage::ExtrudeDepth && stage_ != CreateStage::AdjustProfile) return false;

    const Real depth = extrudeDepth_;
    if (std::fabs(depth) < 1e-4) {
        cancel(camera);
        return false;
    }

    const Mesh solid = buildCurrentSolid(depth);
    if (solid.empty()) {
        cancel(camera);
        return false;
    }

    if (depth < 0.0) {
        // Negative Extrude = Boolean Difference Cut
        SceneObject* target = nullptr;
        if (faceObject_ != kNoObject) {
            target = scene.find(faceObject_);
        }
        if (!target) {
            // Find first intersecting body in scene
            const AABB solidBox = solid.bounds();
            for (const auto& obj : scene.objects()) {
                if (solidBox.overlaps(obj->worldBounds(), 1e-4)) {
                    target = obj.get();
                    break;
                }
            }
        }

        if (target) {
            Mesh meshBefore = target->mesh;
            PrimitiveSpec specBefore = target->spec;

            // Extend cutter slightly outside the entrance face to ensure clean, non-degenerate intersection
            Mesh cutter;
            const std::vector<Vec2> prof = getCurrentProfile();
            makePrismMesh(prof, planeOrigin_, planeU_, planeV_, planeNormal_, depth, 1.0, cutter);

            // Transform cutter into target object's local coordinate space
            const Mat4 toLocal = inverse(target->modelMatrix());
            for (MeshVertex& v : cutter.verts) v.position = transformPoint(toLocal, v.position);

            Mesh result;
            if (meshBoolean(target->mesh, cutter, BooleanOp::Difference, result)) {
                mergeCoplanarFaces(result);
                target->mesh = std::move(result);
                target->spec.kind = PrimitiveKind::Custom;
                target->refreshDerived();
                undo.push(std::make_unique<MeshCommand>(target->id, std::move(meshBefore),
                                                        target->mesh, specBefore, target->spec,
                                                        "Extrude Cut"));
                scene.select(target->id);
                stage_ = CreateStage::None;
                return true;
            }
        }
    }

    // Positive Extrude on an existing face (auto-join)
    if (depth > 0.0 && faceObject_ != kNoObject) {
        SceneObject* target = scene.find(faceObject_);
        if (target) {
            Mesh meshBefore = target->mesh;
            PrimitiveSpec specBefore = target->spec;

            // Transform solid into target object's local coordinate space
            const Mat4 toLocal = inverse(target->modelMatrix());
            Mesh localSolid = solid;
            for (MeshVertex& v : localSolid.verts) v.position = transformPoint(toLocal, v.position);

            Mesh unionResult;
            if (meshBoolean(target->mesh, localSolid, BooleanOp::Union, unionResult)) {
                mergeCoplanarFaces(unionResult);
                target->mesh = std::move(unionResult);
                target->spec.kind = PrimitiveKind::Custom;
                target->refreshDerived();
                undo.push(std::make_unique<MeshCommand>(target->id, std::move(meshBefore),
                                                        target->mesh, specBefore, target->spec,
                                                        "Extrude Join"));
                scene.select(target->id);
                stage_ = CreateStage::None;
                return true;
            }
        }
    }

    // Add as new standalone object in scene
    ObjectId id = kNoObject;
    if (kind_ == PrimitiveKind::Box && cornerRadius_ < 1e-4 &&
        selectedPlane_ == PlaneChoice::XY && faceObject_ == kNoObject) {
        PrimitiveSpec spec;
        spec.kind = PrimitiveKind::Box;
        spec.box.width = currentWidth_;
        spec.box.depth = currentDepth_;
        spec.box.height = std::fabs(depth);
        const Vec2 centerUV = (pt1_ + pt2_) * 0.5f;
        const Vec3 pos = planeOrigin_ + planeU_ * centerUV.x + planeV_ * centerUV.y +
                         planeNormal_ * (depth * 0.5f);
        id = scene.addPrimitive(PrimitiveKind::Box, spec, pos);
    } else if (kind_ == PrimitiveKind::Cylinder &&
               selectedPlane_ == PlaneChoice::XY && faceObject_ == kNoObject) {
        PrimitiveSpec spec;
        spec.kind = PrimitiveKind::Cylinder;
        spec.cylinder.radius = currentRadius_;
        spec.cylinder.height = std::fabs(depth);
        const Vec3 pos = planeOrigin_ + planeU_ * pt1_.x + planeV_ * pt1_.y +
                         planeNormal_ * (depth * 0.5f);
        id = scene.addPrimitive(PrimitiveKind::Cylinder, spec, pos);
    } else {
        // Any rounded rectangle with fillet, or arbitrary plane, or custom profile
        id = scene.addMesh(solid, {0, 0, 0}, kind_ == PrimitiveKind::Box ? "Box" : "Cylinder");
    }

    if (id != kNoObject) {
        undo.push(ExistenceCommand::forCreate(scene, {id}));
        scene.select(id);
    }

    stage_ = CreateStage::None;
    return true;
}

// ---------------------------------------------------------------------------
// 3D Viewport Overlays
// ---------------------------------------------------------------------------

void CreateTool::drawOverlay(const Scene& scene, const Camera& camera, Renderer& renderer) const {
    if (stage_ == CreateStage::None) return;

    if (stage_ == CreateStage::SelectPlane) {
        const float sz = std::max(camera.distance * 0.35f, 25.0f);
        const Vec4 activeBorder = toV4(palette::kBrand, 0.95f);

        // XY Plane Card (Top / Blue)
        const Vec4 colXY = (hoveredPlane_ == PlaneChoice::XY) ? activeBorder : Vec4{0.2f, 0.6f, 0.9f, 0.4f};
        renderer.addLine({-sz, -sz, 0}, {sz, -sz, 0}, colXY);
        renderer.addLine({sz, -sz, 0}, {sz, sz, 0}, colXY);
        renderer.addLine({sz, sz, 0}, {-sz, sz, 0}, colXY);
        renderer.addLine({-sz, sz, 0}, {-sz, -sz, 0}, colXY);
        renderer.addTriangle({-sz, -sz, 0}, {sz, -sz, 0}, {sz, sz, 0}, Vec4{0.2f, 0.6f, 0.9f, 0.08f});
        renderer.addTriangle({-sz, -sz, 0}, {sz, sz, 0}, {-sz, sz, 0}, Vec4{0.2f, 0.6f, 0.9f, 0.08f});

        // XZ Plane Card (Front / Green)
        const Vec4 colXZ = (hoveredPlane_ == PlaneChoice::XZ) ? activeBorder : Vec4{0.3f, 0.8f, 0.4f, 0.4f};
        renderer.addLine({-sz, 0, -sz}, {sz, 0, -sz}, colXZ);
        renderer.addLine({sz, 0, -sz}, {sz, 0, sz}, colXZ);
        renderer.addLine({sz, 0, sz}, {-sz, 0, sz}, colXZ);
        renderer.addLine({-sz, 0, sz}, {-sz, 0, -sz}, colXZ);
        renderer.addTriangle({-sz, 0, -sz}, {sz, 0, -sz}, {sz, 0, sz}, Vec4{0.3f, 0.8f, 0.4f, 0.08f});
        renderer.addTriangle({-sz, 0, -sz}, {sz, 0, sz}, {-sz, 0, sz}, Vec4{0.3f, 0.8f, 0.4f, 0.08f});

        // YZ Plane Card (Right / Red)
        const Vec4 colYZ = (hoveredPlane_ == PlaneChoice::YZ) ? activeBorder : Vec4{0.9f, 0.4f, 0.3f, 0.4f};
        renderer.addLine({0, -sz, -sz}, {0, sz, -sz}, colYZ);
        renderer.addLine({0, sz, -sz}, {0, sz, sz}, colYZ);
        renderer.addLine({0, sz, sz}, {0, -sz, sz}, colYZ);
        renderer.addLine({0, -sz, sz}, {0, -sz, -sz}, colYZ);
        renderer.addTriangle({0, -sz, -sz}, {0, sz, -sz}, {0, sz, sz}, Vec4{0.9f, 0.4f, 0.3f, 0.08f});
        renderer.addTriangle({0, -sz, -sz}, {0, sz, sz}, {0, -sz, sz}, Vec4{0.9f, 0.4f, 0.3f, 0.08f});

        // Object Face Highlight
        if (hoveredPlane_ == PlaneChoice::Face && faceObject_ != kNoObject) {
            const SceneObject* o = scene.find(faceObject_);
            if (o && faceIndex_ < o->mesh.faceCount()) {
                const RenderMesh& rm = o->render;
                const Mat4 model = o->modelMatrix();
                for (size_t i = 0; i < rm.triangleFace.size(); ++i) {
                    if (rm.triangleFace[i] != faceIndex_) continue;
                    renderer.addTriangle(
                        transformPoint(model, rm.positions[rm.triangles[i * 3 + 0]]),
                        transformPoint(model, rm.positions[rm.triangles[i * 3 + 1]]),
                        transformPoint(model, rm.positions[rm.triangles[i * 3 + 2]]),
                        Vec4{palette::kBrand.r, palette::kBrand.g, palette::kBrand.b, 0.45f});
                }
                renderer.addLine(planeOrigin_, planeOrigin_ + planeNormal_ * 15.0f, activeBorder);
            }
        }
        return;
    }

    // In 2D Profile & Extrude stages: draw plane grid and profile
    const float gridSpan = 80.0f;
    const float step = 10.0f;
    const Vec4 gridCol(0.4f, 0.5f, 0.6f, 0.12f);
    for (float i = -gridSpan; i <= gridSpan; i += step) {
        renderer.addLine(planeOrigin_ + planeU_ * i - planeV_ * gridSpan,
                         planeOrigin_ + planeU_ * i + planeV_ * gridSpan, gridCol);
        renderer.addLine(planeOrigin_ - planeU_ * gridSpan + planeV_ * i,
                         planeOrigin_ + planeU_ * gridSpan + planeV_ * i, gridCol);
    }
    // Main U/V axes on the plane
    renderer.addLine(planeOrigin_ - planeU_ * gridSpan, planeOrigin_ + planeU_ * gridSpan, Vec4{0.9f, 0.3f, 0.3f, 0.4f});
    renderer.addLine(planeOrigin_ - planeV_ * gridSpan, planeOrigin_ + planeV_ * gridSpan, Vec4{0.3f, 0.8f, 0.3f, 0.4f});

    // Draw 2D Profile outline & fill
    const std::vector<Vec2> prof = getCurrentProfile();
    if (prof.size() >= 3) {
        const Vec4 outlineCol = toV4(palette::kBrand, 0.95f);
        const Vec4 fillCol = (stage_ == CreateStage::ExtrudeDepth && extrudeDepth_ < 0.0) ? kCutCol : kCreateCol;

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

    // Handles in AdjustProfile stage
    if (stage_ == CreateStage::AdjustProfile) {
        const Real uMin = std::min(pt1_.x, pt2_.x);
        const Real uMax = std::max(pt1_.x, pt2_.x);
        const Real vMin = std::min(pt1_.y, pt2_.y);
        const Real vMax = std::max(pt1_.y, pt2_.y);

        auto drawDotHandle = [&](Vec3 pos, bool active) {
            const float r = active ? 2.5f : 1.5f;
            const Vec4 col = active ? Vec4{1.0f, 0.9f, 0.2f, 1.0f} : toV4(palette::kBrand, 0.9f);
            renderer.addLine(pos - planeU_ * r, pos + planeU_ * r, col);
            renderer.addLine(pos - planeV_ * r, pos + planeV_ * r, col);
            renderer.addLine(pos - planeNormal_ * r, pos + planeNormal_ * r, col);
        };

        if (kind_ == PrimitiveKind::Cylinder) {
            const Vec3 rPt = planeOrigin_ + planeU_ * (pt1_.x + currentRadius_) + planeV_ * pt1_.y;
            drawDotHandle(rPt, hoveredHandle_ == HandleId::RadiusHandle || activeHandle_ == HandleId::RadiusHandle);
        } else {
            const Vec3 c0 = planeOrigin_ + planeU_ * uMin + planeV_ * vMin;
            const Vec3 c1 = planeOrigin_ + planeU_ * uMax + planeV_ * vMin;
            const Vec3 c2 = planeOrigin_ + planeU_ * uMax + planeV_ * vMax;
            const Vec3 c3 = planeOrigin_ + planeU_ * uMin + planeV_ * vMax;

            drawDotHandle(c0, hoveredHandle_ == HandleId::Corner0);
            drawDotHandle(c1, hoveredHandle_ == HandleId::Corner1);
            drawDotHandle(c2, hoveredHandle_ == HandleId::Corner2);
            drawDotHandle(c3, hoveredHandle_ == HandleId::Corner3);

            drawDotHandle((c0 + c1) * 0.5f, hoveredHandle_ == HandleId::EdgeBottom);
            drawDotHandle((c1 + c2) * 0.5f, hoveredHandle_ == HandleId::EdgeRight);
            drawDotHandle((c2 + c3) * 0.5f, hoveredHandle_ == HandleId::EdgeTop);
            drawDotHandle((c3 + c0) * 0.5f, hoveredHandle_ == HandleId::EdgeLeft);
        }
    }

    // 3D Extrusion Solid Preview
    if (stage_ == CreateStage::ExtrudeDepth && std::fabs(extrudeDepth_) > 0.05) {
        const Mesh solid = buildCurrentSolid(extrudeDepth_);
        if (!solid.empty()) {
            const Vec4 wireCol = (extrudeDepth_ < 0.0) ? Vec4{1.0f, 0.4f, 0.3f, 0.9f} : toV4(palette::kBrand, 0.9f);
            const Vec4 faceTint = (extrudeDepth_ < 0.0) ? kCutCol : kCreateCol;

            // Draw wireframe of extrusion
            for (Index h = 0; h < solid.halfedgeCount(); ++h) {
                const Index tw = solid.halfedges[h].twin;
                if (tw != kInvalid && tw < h) continue;
                const Vec3 a = solid.verts[solid.fromVertex(h)].position;
                const Vec3 b = solid.verts[solid.halfedges[h].vertex].position;
                renderer.addLine(a, b, wireCol);
            }

            // Draw tinted faces of solid
            std::vector<Index> tris;
            for (Index f = 0; f < solid.faceCount(); ++f) {
                tris.clear();
                solid.triangulateFacePublic(f, tris);
                std::vector<Index> fv;
                solid.faceVertices(f, fv);
                for (size_t i = 0; i + 2 < tris.size(); i += 3) {
                    renderer.addTriangle(solid.verts[fv[tris[i]]].position,
                                         solid.verts[fv[tris[i + 1]]].position,
                                         solid.verts[fv[tris[i + 2]]].position,
                                         faceTint);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// ImGui 2D HUD Overlays
// ---------------------------------------------------------------------------

bool CreateTool::drawHud(Scene& scene, Camera& camera, UndoStack& undo, bool& outFinished) {
    if (stage_ == CreateStage::None) return false;
    outFinished = false;

    ImGuiIO& io = ImGui::GetIO();
    const ImVec2 displaySize = io.DisplaySize;

    // Top action banner
    ImGui::SetNextWindowPos(ImVec2(displaySize.x * 0.5f, 50.0f), ImGuiCond_Always, ImVec2(0.5f, 0.0f));
    ImGui::SetNextWindowBgAlpha(0.85f);
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                            ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                            ImGuiWindowFlags_NoNav;

    if (ImGui::Begin("##CreateToolHud", nullptr, flags)) {
        if (stage_ == CreateStage::SelectPlane) {
            ImGui::TextColored(kAccentIm, "Select Plane for %s", primitiveName(kind_));
            ImGui::SameLine();
            ImGui::TextDisabled("| Click origin tile or object face");
            ImGui::Spacing();
            if (ImGui::Button("Top (XY)"))   choosePlane(PlaneChoice::XY, camera, scene);
            ImGui::SameLine();
            if (ImGui::Button("Front (XZ)")) choosePlane(PlaneChoice::XZ, camera, scene);
            ImGui::SameLine();
            if (ImGui::Button("Right (YZ)")) choosePlane(PlaneChoice::YZ, camera, scene);
            ImGui::SameLine();
            if (ImGui::Button("Cancel (Esc)")) cancel(camera);
        } else if (stage_ == CreateStage::DrawProfile_Pt1) {
            ImGui::TextColored(kAccentIm, "Step 1: Click to set %s",
                              kind_ == PrimitiveKind::Cylinder ? "Center Point" : "First Corner");
            ImGui::SameLine();
            if (ImGui::Button("Cancel (Esc)")) cancel(camera);
        } else if (stage_ == CreateStage::DrawProfile_Pt2) {
            if (kind_ == PrimitiveKind::Cylinder) {
                ImGui::TextColored(kAccentIm, "Radius: %.2f mm", currentRadius_);
            } else {
                ImGui::TextColored(kAccentIm, "Width: %.2f mm  |  Depth: %.2f mm", currentWidth_, currentDepth_);
            }
            ImGui::SameLine();
            ImGui::TextDisabled("(Click to set)");
            ImGui::SameLine();
            if (ImGui::Button("Cancel (Esc)")) cancel(camera);
        } else if (stage_ == CreateStage::AdjustProfile) {
            ImGui::TextColored(kAccentIm, "Adjust Profile");
            ImGui::SameLine();
            ImGui::TextDisabled("| Drag handles or tweak below");
            ImGui::Separator();

            if (kind_ == PrimitiveKind::Cylinder) {
                ImGui::SetNextItemWidth(120.0f);
                if (ImGui::DragScalar("Radius", ImGuiDataType_Double, &currentRadius_, 0.1f, nullptr, nullptr, "%.2f mm")) {
                    pt2_ = pt1_ + Vec2{currentRadius_, 0};
                }
            } else {
                ImGui::SetNextItemWidth(100.0f);
                if (ImGui::DragScalar("Width", ImGuiDataType_Double, &currentWidth_, 0.1f, nullptr, nullptr, "%.2f mm")) {
                    pt2_.x = pt1_.x + currentWidth_;
                }
                ImGui::SameLine();
                ImGui::SetNextItemWidth(100.0f);
                if (ImGui::DragScalar("Depth", ImGuiDataType_Double, &currentDepth_, 0.1f, nullptr, nullptr, "%.2f mm")) {
                    pt2_.y = pt1_.y + currentDepth_;
                }
                ImGui::SameLine();
                ImGui::SetNextItemWidth(100.0f);
                ImGui::DragScalar("Fillet", ImGuiDataType_Double, &cornerRadius_, 0.1f, nullptr, nullptr, "%.2f mm");
            }

            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Button, kAccentIm);
            if (ImGui::Button("  OK / Extrude (Enter)  ")) {
                stage_ = CreateStage::ExtrudeDepth;
                extrudeBaseDepth_ = 20.0;
                extrudeDepth_ = 20.0;
            }
            ImGui::PopStyleColor();
            ImGui::SameLine();
            if (ImGui::Button("Cancel (Esc)")) cancel(camera);
        } else if (stage_ == CreateStage::ExtrudeDepth) {
            if (extrudeDepth_ >= 0.0) {
                ImGui::TextColored(kAccentIm, "Extrude: +%.2f mm (Solid / Join)", extrudeDepth_);
            } else {
                ImGui::TextColored(ImVec4(0.95f, 0.35f, 0.25f, 1.0f), "Extrude: %.2f mm (Boolean Cut)", extrudeDepth_);
            }
            ImGui::SameLine();
            ImGui::TextDisabled("| Move mouse, click to finalize");
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Button, kAccentIm);
            if (ImGui::Button("  Finish (Click/Enter)  ")) {
                finishCreation(scene, camera, undo);
                outFinished = true;
            }
            ImGui::PopStyleColor();
            ImGui::SameLine();
            if (ImGui::Button("Cancel (Esc)")) cancel(camera);
        }
    }
    ImGui::End();
    return true;
}

} // namespace tg
