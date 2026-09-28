#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;   // unused; keeps one vertex layout

uniform mat4  uModel;
uniform mat4  uViewProj;
uniform float uDepthBias;   // pull lines toward the viewer to beat z-fighting
uniform vec4  uClip;        // a section view's plane; see surface.vert

void main() {
    vec4 world = uModel * vec4(aPos, 1.0);
    gl_ClipDistance[0] = uClip.w - dot(uClip.xyz, world.xyz);
    vec4 clip = uViewProj * world;
    clip.z -= uDepthBias * clip.w;
    gl_Position = clip;
}
