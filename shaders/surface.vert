#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;

out vec3 vWorldPos;
out vec3 vNormal;

uniform mat4 uModel;
uniform mat4 uNormalMat;   // inverse-transpose, correct under non-uniform scale
uniform mat4 uViewProj;

void main() {
    vec4 world = uModel * vec4(aPos, 1.0);
    vWorldPos  = world.xyz;
    vNormal    = mat3(uNormalMat) * aNormal;
    gl_Position = uViewProj * world;
}
