#version 330 core
// Fullscreen triangle. The ground plane is reconstructed per-fragment by
// unprojecting, which gives an infinite grid with correct perspective and
// per-pixel antialiasing -- no grid geometry and no extent to outrun.
out vec3 vNear;
out vec3 vFar;

uniform mat4 uInvViewProj;

vec3 unproject(vec2 ndc, float z) {
    vec4 p = uInvViewProj * vec4(ndc, z, 1.0);
    return p.xyz / p.w;
}

void main() {
    vec2 pos = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2) * 2.0 - 1.0;
    vNear = unproject(pos, -1.0);
    vFar  = unproject(pos,  1.0);
    gl_Position = vec4(pos, 0.0, 1.0);
}
