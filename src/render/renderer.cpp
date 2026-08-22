#include "render/renderer.h"

#include <epoxy/gl.h>

#include <cstdio>

namespace tg {

bool Renderer::init(const std::string& dir) {
    const bool ok =
        surfaceShader_.load(dir + "/surface.vert", dir + "/surface.frag") &&
        lineShader_.load(dir + "/line.vert", dir + "/line.frag") &&
        gridShader_.load(dir + "/grid.vert", dir + "/grid.frag") &&
        overlayShader_.load(dir + "/overlay.vert", dir + "/overlay.frag");
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
    return true;
}

void Renderer::shutdown() {
    // Order matters: every GL object must be released while the context is
    // still current, before Application destroys it.
    cache_.clear();
    surfaceShader_.destroy();
    lineShader_.destroy();
    gridShader_.destroy();
    overlayShader_.destroy();
    if (lineVbo_) { glDeleteBuffers(1, &lineVbo_); lineVbo_ = 0; }
    if (lineVao_) { glDeleteVertexArrays(1, &lineVao_); lineVao_ = 0; }
    if (emptyVao_) { glDeleteVertexArrays(1, &emptyVao_); emptyVao_ = 0; }
}

void Renderer::reloadShadersIfChanged() {
    surfaceShader_.reloadIfChanged();
    lineShader_.reloadIfChanged();
    gridShader_.reloadIfChanged();
    overlayShader_.reloadIfChanged();
}

const GpuMesh& Renderer::syncObject(const SceneObject& obj) {
    CacheEntry& e = cache_[obj.id];
    if (e.version != obj.meshVersion || !e.gpu.valid()) {
        e.gpu.upload(obj.render);
        e.version = obj.meshVersion;
    }
    return e.gpu;
}

void Renderer::addLine(Vec3 a, Vec3 b, Vec4 color) {
    lineVerts_.push_back({a, color});
    lineVerts_.push_back({b, color});
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
    gridShader_.set("uLineColor", Vec3{0.30f, 0.30f, 0.32f});
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

void Renderer::render(const Scene& scene, const Camera& camera, const ViewOptions& opts,
                      PixelRect vp, int fbWidth, int fbHeight) {
    // Clear the whole framebuffer first so the area behind the docked panels
    // is the background colour too, then confine the scene to the central node.
    glDisable(GL_SCISSOR_TEST);
    glViewport(0, 0, fbWidth, fbHeight);
    glClearColor(opts.background.x, opts.background.y, opts.background.z, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

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

    for (const auto& obj : scene.objects()) {
        if (!obj->visible) continue;
        const GpuMesh& gpu = syncObject(*obj);
        const Mat4 model = obj->modelMatrix();

        surfaceShader_.bind();
        surfaceShader_.set("uModel", model);
        surfaceShader_.set("uNormalMat", normalMatrix(model));
        surfaceShader_.set("uSelected", scene.isSelected(obj->id) ? 1.0f : 0.0f);
        gpu.drawTriangles();
    }

    glDisable(GL_POLYGON_OFFSET_FILL);

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

        for (const auto& obj : scene.objects()) {
            if (!obj->visible) continue;
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
        if (opts.backfaceCulling) glEnable(GL_CULL_FACE);
    }

    // ---- Overlays ---------------------------------------------------------
    if (opts.showSelectionBox) {
        const AABB sel = scene.selectionBounds();
        addBox(sel, {opts.accentColor.x, opts.accentColor.y, opts.accentColor.z, 0.45f});
    }

    glDisable(GL_CULL_FACE);
    flushLines(camera);

    glDisable(GL_BLEND);
    glDisable(GL_LINE_SMOOTH);
    glDisable(GL_SCISSOR_TEST);
}

} // namespace tg
