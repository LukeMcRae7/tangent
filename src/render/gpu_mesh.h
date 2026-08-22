// Tangent - GPU-side mirror of a RenderMesh.
#pragma once

#include "mesh/halfedge.h"

#include <cstdint>

namespace tg {

// Interleaved position + normal, with separate index buffers for the shaded
// triangles and the polygon wireframe so both draw from one vertex stream.
class GpuMesh {
public:
    GpuMesh() = default;
    ~GpuMesh();
    GpuMesh(const GpuMesh&) = delete;
    GpuMesh& operator=(const GpuMesh&) = delete;
    GpuMesh(GpuMesh&& other) noexcept { moveFrom(other); }
    GpuMesh& operator=(GpuMesh&& other) noexcept {
        if (this != &other) { release(); moveFrom(other); }
        return *this;
    }

    void upload(const RenderMesh& mesh);
    void drawTriangles() const;
    void drawEdges() const;
    bool valid() const { return vao_ != 0; }

private:
    void release();
    void moveFrom(GpuMesh& other);

    uint32_t vao_ = 0, vbo_ = 0, triIbo_ = 0, edgeIbo_ = 0;
    int32_t  triCount_ = 0, edgeCount_ = 0;
};

} // namespace tg
