#include "render/gpu_mesh.h"

#include <epoxy/gl.h>

#include <vector>

namespace tg {

GpuMesh::~GpuMesh() { release(); }

void GpuMesh::release() {
    if (edgeIbo_) glDeleteBuffers(1, &edgeIbo_);
    if (triIbo_)  glDeleteBuffers(1, &triIbo_);
    if (vbo_)     glDeleteBuffers(1, &vbo_);
    if (vao_)     glDeleteVertexArrays(1, &vao_);
    vao_ = vbo_ = triIbo_ = edgeIbo_ = 0;
    triCount_ = edgeCount_ = 0;
}

void GpuMesh::moveFrom(GpuMesh& o) {
    vao_ = o.vao_; vbo_ = o.vbo_; triIbo_ = o.triIbo_; edgeIbo_ = o.edgeIbo_;
    triCount_ = o.triCount_; edgeCount_ = o.edgeCount_;
    o.vao_ = o.vbo_ = o.triIbo_ = o.edgeIbo_ = 0;
    o.triCount_ = o.edgeCount_ = 0;
}

void GpuMesh::upload(const RenderMesh& mesh) {
    if (!vao_) {
        glGenVertexArrays(1, &vao_);
        glGenBuffers(1, &vbo_);
        glGenBuffers(1, &triIbo_);
        glGenBuffers(1, &edgeIbo_);
    }

    std::vector<float> interleaved;
    interleaved.reserve(mesh.positions.size() * 6);
    // Geometry is kept in double; the GPU takes float32. This and Shader::set
    // are the only places that narrowing happens.
    auto f = [](Real v) { return static_cast<float>(v); };
    for (size_t i = 0; i < mesh.positions.size(); ++i) {
        const Vec3& p = mesh.positions[i];
        const Vec3& n = mesh.normals[i];
        interleaved.insert(interleaved.end(),
                           {f(p.x), f(p.y), f(p.z), f(n.x), f(n.y), f(n.z)});
    }

    glBindVertexArray(vao_);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(interleaved.size() * sizeof(float)),
                 interleaved.data(), GL_STATIC_DRAW);

    const GLsizei stride = 6 * sizeof(float);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride,
                          reinterpret_cast<void*>(3 * sizeof(float)));

    // The element buffer bound when the VAO is unbound is the one it keeps, so
    // each draw call rebinds the index buffer it needs.
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, triIbo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(mesh.triangles.size() * sizeof(uint32_t)),
                 mesh.triangles.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, edgeIbo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(mesh.edgeLines.size() * sizeof(uint32_t)),
                 mesh.edgeLines.data(), GL_STATIC_DRAW);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    triCount_  = static_cast<int32_t>(mesh.triangles.size());
    edgeCount_ = static_cast<int32_t>(mesh.edgeLines.size());
}

void GpuMesh::drawTriangles() const {
    if (!vao_ || triCount_ == 0) return;
    glBindVertexArray(vao_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, triIbo_);
    glDrawElements(GL_TRIANGLES, triCount_, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

void GpuMesh::drawEdges() const {
    if (!vao_ || edgeCount_ == 0) return;
    glBindVertexArray(vao_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, edgeIbo_);
    glDrawElements(GL_LINES, edgeCount_, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

} // namespace tg
