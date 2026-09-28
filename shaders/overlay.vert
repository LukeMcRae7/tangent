#version 330 core
// Per-vertex coloured world-space lines, used for gizmos and overlays.
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 aColor;

out vec4 vColor;
uniform mat4 uViewProj;
uniform vec4 uClip;         // a section view's plane; see surface.vert

void main() {
    vColor = aColor;
    gl_Position = uViewProj * vec4(aPos, 1.0);
    gl_ClipDistance[0] = uClip.w - dot(uClip.xyz, aPos);
}
