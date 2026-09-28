#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;

out vec3 vWorldPos;
out vec3 vNormal;

uniform mat4 uModel;
uniform mat4 uNormalMat;   // inverse-transpose, correct under non-uniform scale
uniform mat4 uViewProj;
// A section view's plane, (normal, offset): what lies beyond it -- the side the
// normal points to -- is not drawn. Read only while GL_CLIP_DISTANCE0 is on.
uniform vec4 uClip;

void main() {
    vec4 world = uModel * vec4(aPos, 1.0);
    vWorldPos  = world.xyz;
    vNormal    = mat3(uNormalMat) * aNormal;
    gl_Position = uViewProj * world;
    gl_ClipDistance[0] = uClip.w - dot(uClip.xyz, world.xyz);
}
