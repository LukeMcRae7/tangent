// Tangent - viewport renderer.
//
// Owns every GL resource. GPU buffers are cached per object and refreshed only
// when SceneObject::meshVersion moves, so editing one object does not re-upload
// the whole scene.
#pragma once

#include "render/gpu_mesh.h"
#include "render/shader.h"
#include "scene/scene.h"
#include "app/camera.h"
#include "core/palette.h"

#include <unordered_map>

namespace tg {

struct ViewOptions {
    bool  showGrid       = true;
    bool  showWireframe  = true;
    bool  showSelectionBox = false;
    bool  backfaceCulling = true;
    float creaseAngleDeg = 35.0f;

    Vec3  background    = toVec3(palette::kViewport);
    Vec3  objectColor   = toVec3(palette::kSurface);
    Vec3  accentColor   = toVec3(palette::kSelection);
    Vec4  edgeColor     = toVec4(palette::kEdge, 0.85f);
    Vec4  selectedEdge  = toVec4(palette::kSelection, 1.0f);

    // Base cell size in mm. The grid subdivides and merges by powers of this
    // ratio as you zoom, so these are the finest cell and the step between
    // levels rather than two fixed line sets.
    float gridSpacing    = 1.0f;
    float gridSubdivide  = 10.0f;
};

// Sub-rectangle of the framebuffer to draw into, in physical pixels and in
// GL's convention (origin bottom-left). The 3D view occupies the dockspace's
// central node rather than the whole window, so it cannot be assumed fullscreen.
struct PixelRect {
    int x = 0, y = 0, w = 0, h = 0;
    bool valid() const { return w > 0 && h > 0; }
};

class Renderer {
public:
    bool init(const std::string& shaderDir);
    void shutdown();

    // `fbWidth`/`fbHeight` describe the whole framebuffer, which is cleared to
    // the background colour before `viewport` is scissored off for the scene.
    void render(const Scene& scene, const Camera& camera, const ViewOptions& opts,
                PixelRect viewport, int fbWidth, int fbHeight);

    // Picks up shader edits without a restart; cheap enough to poll each frame.
    void reloadShadersIfChanged();

    // Discards the cached buffers for an object that no longer exists.
    void forget(ObjectId id) { cache_.erase(id); }

    // Queued world-space line segments, flushed at the end of the frame.
    // Gizmos and measurement overlays draw through this.
    void addLine(Vec3 a, Vec3 b, Vec4 color);
    void addBox(const AABB& box, Vec4 color);

private:
    struct CacheEntry {
        GpuMesh  gpu;
        uint32_t version = 0;
    };
    struct LineVert { Vec3 pos; Vec4 color; };

    const GpuMesh& syncObject(const SceneObject& obj);
    void drawGrid(const Camera& camera, const ViewOptions& opts);
    void flushLines(const Camera& camera);

    Shader surfaceShader_, lineShader_, gridShader_, overlayShader_;
    uint32_t emptyVao_ = 0;                 // for attribute-less fullscreen draws
    uint32_t lineVao_ = 0, lineVbo_ = 0;
    std::vector<LineVert> lineVerts_;
    std::unordered_map<ObjectId, CacheEntry> cache_;
};

} // namespace tg
