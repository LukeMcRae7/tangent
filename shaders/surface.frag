#version 330 core
in vec3 vWorldPos;
in vec3 vNormal;
out vec4 fragColor;

uniform vec3  uCameraPos;
uniform vec3  uBaseColor;
uniform float uSelected;    // 0 or 1
uniform vec3  uAccent;

// Studio three-point rig in view-independent world space. Keeping the lights
// fixed to the world (not the camera) means orbiting reads as the object
// turning under stable lighting, which is what makes form legible.
const vec3  kKeyDir   = normalize(vec3(-0.4, -0.7,  0.62));
const vec3  kFillDir  = normalize(vec3( 0.75, 0.25, 0.18));
const vec3  kRimDir   = normalize(vec3( 0.25, 0.85, -0.35));
const float kKeyInt   = 0.95;
const float kFillInt  = 0.28;
const float kRimInt   = 0.35;

void main() {
    vec3 N = normalize(vNormal);
    vec3 V = normalize(uCameraPos - vWorldPos);
    // Two-sided: interior walls seen through an opening still shade sensibly.
    if (dot(N, V) < 0.0) N = -N;

    // Only a whisper of accent on the surface: the orange outline is what
    // actually communicates selection, and tinting the whole body makes it
    // hard to read form and shading.
    vec3 base = mix(uBaseColor, mix(uBaseColor, uAccent, 0.10), uSelected);

    // Hemispheric ambient: sky above, cooler bounce below.
    vec3 sky    = vec3(0.34, 0.36, 0.40);
    vec3 ground = vec3(0.11, 0.11, 0.13);
    vec3 ambient = mix(ground, sky, N.z * 0.5 + 0.5);

    float key  = max(dot(N, kKeyDir),  0.0) * kKeyInt;
    float fill = max(dot(N, kFillDir), 0.0) * kFillInt;

    vec3 H = normalize(kKeyDir + V);
    float spec = pow(max(dot(N, H), 0.0), 48.0) * 0.22;

    float rim = pow(1.0 - max(dot(N, V), 0.0), 2.5)
              * max(dot(N, kRimDir), 0.0) * kRimInt;

    vec3 color = base * (ambient + key + fill) + vec3(spec) + vec3(rim) * 1.15;
    color += uAccent * uSelected * 0.015;

    // Approximate sRGB encode; the default framebuffer is not sRGB-aware here.
    fragColor = vec4(pow(max(color, 0.0), vec3(1.0 / 2.2)), 1.0);
}
