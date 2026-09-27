#include "render/renderer.h"

#include <cstddef>

#include <algorithm>

#include <epoxy/gl.h>

#include <cstdio>

namespace tg {

bool Renderer::init(const std::string& dir) {
    const bool ok =
        surfaceShader_.load(dir + "/surface.vert", dir + "/surface.frag") &&
        lineShader_.load(dir + "/line.vert", dir + "/line.frag") &&
        gridShader_.load(dir + "/grid.vert", dir + "/grid.frag") &&
        overlayShader_.load(dir + "/overlay.vert", dir + "/overlay.frag") &&
        capShader_.load(dir + "/surface.vert", dir + "/cap.frag");
    if (!ok) {
        std::fprintf(stderr, "[renderer] shader initialisation failed (dir=%s)\n", dir.c_str());
        return false;
    }

    // Core profile forbids drawing with no VAO bound, even when the vertex
    // shader reads nothing but gl_VertexID.
    glGenVertexArrays(1, &emptyVao_);

    glGenVertexArrays(1, &lineVao_);
    glGenBuffers(1, &lineVbo_);
    glBindVertexArray(lineVao_);
    glBindBuffer(GL_ARRAY_BUFFER, lineVbo_);
    const GLsizei stride = sizeof(LineVert);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, stride,
                          reinterpret_cast<void*>(offsetof(LineVert, color)));
    glBindVertexArray(0);

    // The vertex format above must match LineVert exactly, or the GPU reads
    // nonsense. Checked here so a future change to the struct fails loudly.
    static_assert(sizeof(LineVert) == 7 * sizeof(float),
                  "LineVert must stay 7 tightly packed floats");
    static_assert(offsetof(LineVert, color) == 3 * sizeof(float),
                  "LineVert colour must follow three position floats");

    // Same vertex layout as the line batch, drawn as triangles.
    glGenVertexArrays(1, &triVao_);
    glGenBuffers(1, &triVbo_);
    glBindVertexArray(triVao_);
    glBindBuffer(GL_ARRAY_BUFFER, triVbo_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, stride,
                          reinterpret_cast<void*>(offsetof(LineVert, color)));
    glBindVertexArray(0);
    return true;
}

void Renderer::setStatic(int slot, uint64_t version, const std::vector<Vec3>& triangles, Vec4 colour) {
    if (slot < 0 || slot >= kStaticSlots) return;
    StaticBatch& b = statics_[slot];
    if (b.version == version && b.vao) return;
    b.version = version;
    if (!b.vao) {
        const GLsizei stride = sizeof(LineVert);
        glGenVertexArrays(1, &b.vao);
        glGenBuffers(1, &b.vbo);
        glBindVertexArray(b.vao);
        glBindBuffer(GL_ARRAY_BUFFER, b.vbo);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(offsetof(LineVert, color)));
        glBindVertexArray(0);
    }
    std::vector<LineVert> verts;
    verts.reserve(triangles.size());
    for (const Vec3& p : triangles) verts.push_back(makeVert(p, colour));
    glBindBuffer(GL_ARRAY_BUFFER, b.vbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(verts.size() * sizeof(LineVert)), verts.data(),
                 GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    b.count = static_cast<int>(verts.size());
}

void Renderer::flushStatics(const Camera& camera) {
    bool any = false;
    for (const StaticBatch& b : statics_) any = any || (b.show && b.count > 0);
    if (any) {
        overlayShader_.bind();
        overlayShader_.set("uViewProj", camera.viewProjection());
        overlayShader_.set("uClip", overlayClip_);
        if (clipOverlays_) glEnable(GL_CLIP_DISTANCE0);
        glDepthMask(GL_FALSE);
        glDisable(GL_CULL_FACE);
        for (const StaticBatch& b : statics_) {
            if (!b.show || b.count == 0) continue;
            glBindVertexArray(b.vao);
            glDrawArrays(GL_TRIANGLES, 0, b.count);
        }
        glBindVertexArray(0);
        glDepthMask(GL_TRUE);
        glDisable(GL_CLIP_DISTANCE0);
    }
    for (StaticBatch& b : statics_) b.show = false;
}

void Renderer::shutdown() {
    // Order matters: every GL object must be released while the context is
    // still current, before Application destroys it.
    for (StaticBatch& b : statics_) {
        if (b.vbo) glDeleteBuffers(1, &b.vbo);
        if (b.vao) glDeleteVertexArrays(1, &b.vao);
        b = StaticBatch{};
    }
    cache_.clear();
    triVerts_.clear();
    lineVerts_.clear();
    surfaceShader_.destroy();
    lineShader_.destroy();
    gridShader_.destroy();
    overlayShader_.destroy();
    capShader_.destroy();
    if (triVbo_) { glDeleteBuffers(1, &triVbo_); triVbo_ = 0; }
    if (triVao_) { glDeleteVertexArrays(1, &triVao_); triVao_ = 0; }
    if (lineVbo_) { glDeleteBuffers(1, &lineVbo_); lineVbo_ = 0; }
    if (lineVao_) { glDeleteVertexArrays(1, &lineVao_); lineVao_ = 0; }
    if (emptyVao_) { glDeleteVertexArrays(1, &emptyVao_); emptyVao_ = 0; }
}

void Renderer::reloadShadersIfChanged() {
    surfaceShader_.reloadIfChanged();
    lineShader_.reloadIfChanged();
    gridShader_.reloadIfChanged();
    overlayShader_.reloadIfChanged();
    capShader_.reloadIfChanged();
}

Vec4 Renderer::clipPlane(const SectionCut& cut, Real slack) {
    if (!cut.on) return {0.0, 0.0, 0.0, 1.0};
    return {cut.normal.x, cut.normal.y, cut.normal.z, cut.offset + slack};
}

const GpuMesh& Renderer::syncObject(const SceneObject& obj) {
    CacheEntry& e = cache_[obj.id];
    if (e.version != obj.meshVersion || !e.gpu.valid()) {
        e.gpu.upload(obj.render);
        e.version = obj.meshVersion;
    }
    e.lastSeen = frameIndex_;
    return e.gpu;
}

void Renderer::pruneCache() {
    // A generous grace period: an object hidden for a moment, or restored by
    // undo shortly after being removed, keeps its buffers and re-appears
    // without a re-upload.
    constexpr uint64_t kGraceFrames = 240;
    if (frameIndex_ < kGraceFrames) return;

    const uint64_t cutoff = frameIndex_ - kGraceFrames;
    for (auto it = cache_.begin(); it != cache_.end(); ) {
        if (it->second.lastSeen < cutoff) it = cache_.erase(it);
        else ++it;
    }
}

void Renderer::addLine(Vec3 a, Vec3 b, Vec4 color) {
    lineVerts_.push_back(makeVert(a, color));
    lineVerts_.push_back(makeVert(b, color));
}

void Renderer::addTriangle(Vec3 a, Vec3 b, Vec3 c, Vec4 color) {
    triVerts_.push_back(makeVert(a, color));
    triVerts_.push_back(makeVert(b, color));
    triVerts_.push_back(makeVert(c, color));
}

void Renderer::addFrontTriangle(Vec3 a, Vec3 b, Vec3 c, Vec4 color) {
    frontVerts_.push_back(makeVert(a, color));
    frontVerts_.push_back(makeVert(b, color));
    frontVerts_.push_back(makeVert(c, color));
}

void Renderer::addFrontLine(const Camera& camera, Vec3 a, Vec3 b, Vec4 color,
                            Real widthPx) {
    Vec3 along = b - a;
    const Real len = length(along);
    if (len < 1e-12) return;
    along = along / len;

    // Across the line and facing the eye, so the quad keeps its width whichever
    // way the line runs. Taken at the midpoint: a line long enough for the two
    // ends to disagree is already far enough away that either answer reads the
    // same.
    const Vec3 mid = (a + b) * 0.5;
    Vec3 across = cross(along, normalize(camera.eye() - mid));
    if (lengthSq(across) < 1e-12) return;      // end-on: nothing to draw
    across = normalize(across) * (static_cast<Real>(camera.pixelWorldSize(mid)) * widthPx * 0.5);

    addFrontTriangle(a - across, b - across, b + across, color);
    addFrontTriangle(a - across, b + across, a + across, color);
}

void Renderer::addFrontDashes(const Camera& camera, Vec3 a, Vec3 b, Vec4 color,
                              Real widthPx, Real dashPx, Real gapPx) {
    const Vec3 span = b - a;
    const Real len = length(span);
    if (len < 1e-12) return;

    const Real px = static_cast<Real>(camera.pixelWorldSize((a + b) * 0.5));
    const Real dash = std::max(dashPx * px, Real(1e-9));
    const Real gap = std::max(gapPx * px, Real(0));
    const Real period = dash + gap;
    if (period <= 0.0) return;

    // Bounded: a reference line can run the length of the model, and at a tight
    // zoom that is thousands of dashes nobody can tell apart.
    const int count = std::min(static_cast<int>(len / period) + 1, 256);
    const Vec3 dir = span / len;
    for (int i = 0; i < count; ++i) {
        const Real s0 = static_cast<Real>(i) * period;
        const Real s1 = std::min(s0 + dash, len);
        if (s1 <= s0) break;
        addFrontLine(camera, a + dir * s0, a + dir * s1, color, widthPx);
    }
}

void Renderer::addBox(const AABB& box, Vec4 color) {
    if (!box.valid()) return;
    Vec3 c[8];
    for (int i = 0; i < 8; ++i)
        c[i] = {(i & 1) ? box.max.x : box.min.x,
                (i & 2) ? box.max.y : box.min.y,
                (i & 4) ? box.max.z : box.min.z};
    // Corner bits are (x, y, z), so pairs differing in exactly one bit are edges.
    static const int kEdges[12][2] = {
        {0,1},{2,3},{4,5},{6,7},   // along X
        {0,2},{1,3},{4,6},{5,7},   // along Y
        {0,4},{1,5},{2,6},{3,7}};  // along Z
    for (const auto& e : kEdges) addLine(c[e[0]], c[e[1]], color);
}

void Renderer::drawGrid(const Camera& camera, const ViewOptions& opts) {
    const Mat4 viewProj = camera.viewProjection();

    gridShader_.bind();
    gridShader_.set("uViewProj", viewProj);
    gridShader_.set("uInvViewProj", inverse(viewProj));
    gridShader_.set("uCameraPos", camera.eye());
    gridShader_.set("uSpacing", opts.gridSpacing);
    gridShader_.set("uSubdivide", opts.gridSubdivide);
    gridShader_.set("uAxisXColor", toVec3(palette::kGridAxisX));
    gridShader_.set("uAxisYColor", toVec3(palette::kGridAxisY));
    gridShader_.set("uLineColor", toVec3(palette::kGridLine));
    // Fade with zoom so the grid always dissolves near the horizon rather than
    // at a fixed world radius. The far end is kept fairly tight because at
    // grazing angles the ground point runs to thousands of millimetres, where
    // fract() in the shader starts losing precision and the lines speckle.
    gridShader_.set("uFadeStart", camera.distance * 2.5f);
    gridShader_.set("uFadeEnd",   camera.distance * 9.0f);

    glBindVertexArray(emptyVao_);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);
}

void Renderer::flushLines(const Camera& camera) {
    if (lineVerts_.empty()) return;

    overlayShader_.bind();
    overlayShader_.set("uViewProj", camera.viewProjection());

    glBindVertexArray(lineVao_);
    glBindBuffer(GL_ARRAY_BUFFER, lineVbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(lineVerts_.size() * sizeof(LineVert)),
                 lineVerts_.data(), GL_STREAM_DRAW);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(lineVerts_.size()));
    glBindVertexArray(0);

    lineVerts_.clear();
}

void Renderer::flushTriangles(const Camera& camera) {
    if (triVerts_.empty()) return;

    overlayShader_.bind();
    overlayShader_.set("uViewProj", camera.viewProjection());
    overlayShader_.set("uClip", overlayClip_);
    if (clipOverlays_) glEnable(GL_CLIP_DISTANCE0);

    // Tint without occluding: the highlight must not hide the geometry under it
    // or write depth that later overlay lines would fail against.
    glDepthMask(GL_FALSE);
    glDisable(GL_CULL_FACE);

    glBindVertexArray(triVao_);
    glBindBuffer(GL_ARRAY_BUFFER, triVbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(triVerts_.size() * sizeof(LineVert)),
                 triVerts_.data(), GL_STREAM_DRAW);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(triVerts_.size()));
    glBindVertexArray(0);

    glDepthMask(GL_TRUE);
    glDisable(GL_CLIP_DISTANCE0);
    triVerts_.clear();
}

void Renderer::render(const Scene& scene, const Camera& camera, const ViewOptions& opts,
                      PixelRect vp, int fbWidth, int fbHeight) {
    // Clear the whole framebuffer first so the area behind the docked panels
    // is the background colour too, then confine the scene to the central node.
    glDisable(GL_SCISSOR_TEST);
    glViewport(0, 0, fbWidth, fbHeight);
    glClearColor(opts.background.x, opts.background.y, opts.background.z, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    if (!vp.valid()) return;
    glViewport(vp.x, vp.y, vp.w, vp.h);
    glEnable(GL_SCISSOR_TEST);
    glScissor(vp.x, vp.y, vp.w, vp.h);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_TRUE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);

    if (opts.backfaceCulling) {
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
    } else {
        glDisable(GL_CULL_FACE);
    }

    const Mat4 viewProj = camera.viewProjection();

    // A section view: everything past the plane is not drawn, and the inside
    // of each part it cuts is filled in the plane (the cap pass below).
    const SectionCut& cut = scene.section();
    const Vec4 clip = clipPlane(cut);
    // Which side of the plane an object's box is on: -1 wholly kept, +1
    // wholly taken away, 0 cut.
    auto sideOf = [&](const SceneObject& o) {
        if (!cut.on) return -1;
        const AABB b = o.worldBounds();
        if (!b.valid()) return -1;
        Real lo = 1e300, hi = -1e300;
        for (int k = 0; k < 8; ++k) {
            const Vec3 p{(k & 1) ? b.max.x : b.min.x, (k & 2) ? b.max.y : b.min.y, (k & 4) ? b.max.z : b.min.z};
            const Real d = dot(cut.normal, p) - cut.offset;
            lo = std::min(lo, d);
            hi = std::max(hi, d);
        }
        return hi <= 0.0 ? -1 : lo > 0.0 ? 1 : 0;
    };
    if (cut.on) {
        glEnable(GL_CLIP_DISTANCE0);
        // Only the outside of each wall is shaded; the inside, where it shows
        // through the cut, is the cap's.
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
    }

    // ---- Shaded surfaces --------------------------------------------------
    // Push fills a hair away from the viewer so the wireframe pass can win the
    // depth test on shared edges without a manual bias. Biasing the *lines*
    // forward instead would also pull hidden back-face edges through the
    // surface, which is exactly the artefact this avoids.
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);

    surfaceShader_.bind();
    surfaceShader_.set("uViewProj", viewProj);
    surfaceShader_.set("uCameraPos", camera.eye());
    surfaceShader_.set("uBaseColor", opts.objectColor);
    surfaceShader_.set("uAccent", opts.accentColor);
    surfaceShader_.set("uClip", clip);

    // Where two bodies have faces in one plane, the depth buffer cannot tell
    // them apart and draws a speckle of both. The body holding the selection
    // is pushed back less than the rest, so it wins: select a face there --
    // clicking again steps to the next -- and that face is the one shown.
    std::vector<ObjectId> holding;
    for (const ElementRef& e : scene.elementSelection())
        if (std::find(holding.begin(), holding.end(), e.object) == holding.end())
            holding.push_back(e.object);

    for (const auto& obj : scene.objects()) {
        if (!obj->visible || sideOf(*obj) > 0) continue;
        const GpuMesh& gpu = syncObject(*obj);
        const Mat4 model = obj->modelMatrix();
        const bool selected = scene.isSelected(obj->id);
        const bool inFront = selected ||
                             std::find(holding.begin(), holding.end(), obj->id) != holding.end();
        glPolygonOffset(inFront ? 1.0f : 2.0f, inFront ? 1.0f : 3.0f);

        surfaceShader_.bind();
        surfaceShader_.set("uModel", model);
        surfaceShader_.set("uNormalMat", normalMatrix(model));
        surfaceShader_.set("uSelected", selected ? 1.0f : 0.0f);
        gpu.drawTriangles();
    }

    glDisable(GL_POLYGON_OFFSET_FILL);

    // ---- Section caps -----------------------------------------------------
    // The inside of each wall the plane cuts, drawn as the cut face in the
    // plane: see shaders/cap.frag. Only for parts the plane goes through.
    //
    // In two steps a part. The cap is drawn at the plane, in front of the wall
    // it comes from, so it would pass the depth test even where that wall is
    // hidden -- looking down a hole the cut went past, the far side of the
    // hole would be capped over. So first the inside of the walls is drawn
    // where it really is, into the stencil only, which marks where it is the
    // nearest thing; then the cap is drawn there and nowhere else.
    if (cut.on) {
        glEnable(GL_STENCIL_TEST);
        glCullFace(GL_FRONT);
        auto markInside = [&](const SceneObject& obj, const GpuMesh& gpu) {
            surfaceShader_.bind();
            surfaceShader_.set("uModel", obj.modelMatrix());
            glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
            glDepthMask(GL_FALSE);
            glDepthFunc(GL_LEQUAL);
            // Pushed back as the outside was, so the two meet at the rim.
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(2.0f, 3.0f);
            glStencilFunc(GL_ALWAYS, 1, 0xFF);
            glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
            gpu.drawTriangles();
            glDisable(GL_POLYGON_OFFSET_FILL);
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
            glDepthMask(GL_TRUE);
            // Then only there, once a pixel, and the mark wiped as it goes.
            glDepthFunc(GL_ALWAYS);
            glStencilFunc(GL_EQUAL, 1, 0xFF);
            glStencilOp(GL_KEEP, GL_KEEP, GL_ZERO);
        };
        capShader_.bind();
        capShader_.set("uViewProj", viewProj);
        capShader_.set("uInvViewProj", inverse(viewProj));
        capShader_.set("uViewportPx", Vec4{static_cast<Real>(vp.x), static_cast<Real>(vp.y),
                                           static_cast<Real>(vp.w), static_cast<Real>(vp.h)});
        capShader_.set("uClip", clip);
        capShader_.set("uCameraPos", camera.eye());
        capShader_.set("uBaseColor", opts.objectColor);
        capShader_.set("uAccent", opts.accentColor);
        // Two directions in the plane, for the hatch.
        const Vec3 n = cut.normal;
        const Vec3 u = normalize(std::fabs(n.z) < 0.9 ? cross(n, Vec3{0.0, 0.0, 1.0}) : cross(n, Vec3{1.0, 0.0, 0.0}));
        capShader_.set("uHatchU", u);
        capShader_.set("uHatchV", cross(n, u));
        capShader_.set("uHatchSpacing", std::max(opts.sectionHatch, Real(1e-3)));
        int cutIndex = 0;
        for (const auto& obj : scene.objects()) {
            if (!obj->visible || sideOf(*obj) != 0) continue;
            const GpuMesh& gpu = syncObject(*obj);
            const Mat4 model = obj->modelMatrix();
            markInside(*obj, gpu);
            capShader_.bind();
            capShader_.set("uModel", model);
            capShader_.set("uNormalMat", normalMatrix(model));
            capShader_.set("uSelected", scene.isSelected(obj->id) ? 1.0f : 0.0f);
            // Neighbours lean opposite ways, as on a drawing.
            capShader_.set("uHatchSign", (cutIndex++ & 1) ? Real(-1.0) : Real(1.0));
            gpu.drawTriangles();
        }
        glDepthFunc(GL_LEQUAL);
        glDisable(GL_STENCIL_TEST);
        glCullFace(GL_BACK);
        if (!opts.backfaceCulling) glDisable(GL_CULL_FACE);
        glDisable(GL_CLIP_DISTANCE0);
    }

    // ---- Ground grid ------------------------------------------------------
    // After the surfaces so it blends against them, with depth writes off so
    // it never occludes anything itself.
    if (opts.showGrid) {
        glDepthMask(GL_FALSE);
        glDisable(GL_CULL_FACE);
        drawGrid(camera, opts);
        glDepthMask(GL_TRUE);
        if (opts.backfaceCulling) glEnable(GL_CULL_FACE);
    }

    // ---- Polygon wireframe ------------------------------------------------
    if (opts.showWireframe) {
        glDisable(GL_CULL_FACE);
        lineShader_.bind();
        lineShader_.set("uViewProj", viewProj);
        lineShader_.set("uClip", clip);
        if (cut.on) glEnable(GL_CLIP_DISTANCE0);

        for (const auto& obj : scene.objects()) {
            if (!obj->visible || sideOf(*obj) > 0) continue;
            const bool selected = scene.isSelected(obj->id);
            const GpuMesh& gpu = syncObject(*obj);

            lineShader_.set("uModel", obj->modelMatrix());
            lineShader_.set("uColor", selected ? opts.selectedEdge : opts.edgeColor);
            // No bias needed: the fills were already offset away above, so
            // visible edges pass the depth test and hidden ones still fail it.
            lineShader_.set("uDepthBias", 0.0f);
            glLineWidth(selected ? 1.8f : 1.0f);
            gpu.drawEdges();
        }
        glLineWidth(1.0f);
        glDisable(GL_CLIP_DISTANCE0);
        if (opts.backfaceCulling) glEnable(GL_CULL_FACE);
    }

    // ---- Overlays ---------------------------------------------------------
    if (opts.showSelectionBox) {
        const AABB sel = scene.selectionBounds();
        addBox(sel, {opts.accentColor.x, opts.accentColor.y, opts.accentColor.z, 0.45f});
    }

    // Tints lie on the surfaces, so they are cut with them -- but not what is
    // drawn in the plane itself, the section's own outline among it.
    overlayClip_ = clipPlane(cut, 1e-3 * (1.0 + std::fabs(cut.offset)));
    clipOverlays_ = cut.on;
    glDisable(GL_CULL_FACE);
    flushStatics(camera);
    flushTriangles(camera);
    flushLines(camera);

    // Last, and with the depth buffer ignored: whatever is in this layer is
    // there to be read, so nothing -- not the model, not the overlay's own
    // ticks -- may be drawn over it.
    if (!frontVerts_.empty()) {
        overlayShader_.bind();
        overlayShader_.set("uViewProj", camera.viewProjection());
        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        glBindVertexArray(triVao_);
        glBindBuffer(GL_ARRAY_BUFFER, triVbo_);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(frontVerts_.size() * sizeof(LineVert)),
                     frontVerts_.data(), GL_STREAM_DRAW);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(frontVerts_.size()));
        glBindVertexArray(0);
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        frontVerts_.clear();
    }

    glDisable(GL_BLEND);
    glDisable(GL_LINE_SMOOTH);
    glDisable(GL_SCISSOR_TEST);

    ++frameIndex_;
    if ((frameIndex_ % 120) == 0) pruneCache();
}

} // namespace tg
