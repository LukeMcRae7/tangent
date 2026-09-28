#version 330 core
// The cut face of a section view.
//
// Drawn from the inside of each part's walls -- the triangles facing away from
// the eye -- which, with the near side of the part cut away, are exactly where
// the cut is and nowhere else. They are not where the cut is, though: they are
// behind it. So each fragment finds where its line of sight crosses the plane,
// is shaded as that point on a flat face lying in the plane, and writes that
// depth, so the cut face hides what is behind it and is hidden by what is in
// front, exactly as if the part had been sawn through.
//
// Hatched, as a drawing hatches a section, in the plane's own coordinates so
// the lines stay put as the view turns; neighbouring parts hatch the other way,
// so where two parts meet in the cut, the eye can tell which is which.
in vec3 vWorldPos;
in vec3 vNormal;
out vec4 fragColor;

uniform mat4  uViewProj;
uniform mat4  uInvViewProj;
uniform vec4  uViewportPx;      // the view's rectangle in framebuffer pixels: x, y, w, h
uniform vec4  uClip;            // normal, offset
uniform vec3  uCameraPos;
uniform vec3  uBaseColor;
uniform vec3  uAccent;
uniform float uSelected;
uniform vec3  uHatchU;          // two directions in the plane
uniform vec3  uHatchV;
uniform float uHatchSpacing;    // mm between lines
uniform float uHatchSign;       // +1 or -1: which way this part's lines lean

const vec3  kKeyDir   = normalize(vec3(-0.4, -0.7,  0.62));
const vec3  kFillDir  = normalize(vec3( 0.75, 0.25, 0.18));
const float kKeyInt   = 0.95;
const float kFillInt  = 0.28;

void main() {
    // The line of sight through this pixel, from the near plane inwards. The
    // second point is taken at mid-depth, not at the far plane, where a float
    // inverse has little precision left.
    vec2 ndc = (gl_FragCoord.xy - uViewportPx.xy) / uViewportPx.zw * 2.0 - 1.0;
    vec4 a = uInvViewProj * vec4(ndc, -1.0, 1.0);
    vec4 b = uInvViewProj * vec4(ndc,  0.0, 1.0);
    vec3 p0 = a.xyz / a.w;
    vec3 dir = b.xyz / b.w - p0;

    vec3  P = vWorldPos;
    float depth = gl_FragCoord.z;
    float denom = dot(uClip.xyz, dir);
    if (abs(denom) > 1e-12) {
        float t = (uClip.w - dot(uClip.xyz, p0)) / denom;
        vec3 q = p0 + dir * t;
        vec4 c = uViewProj * vec4(q, 1.0);
        float z = (c.z / c.w) * 0.5 + 0.5;
        // Only in front of the wall it came through: from inside a part, the
        // wall itself is the nearest thing there is.
        if (t >= 0.0 && z < depth) { depth = z; P = q; }
    }
    gl_FragDepth = depth;

    // Lit as a flat face in the plane, turned to the eye.
    vec3 N = uClip.xyz;
    vec3 V = normalize(uCameraPos - P);
    if (dot(N, V) < 0.0) N = -N;
    vec3 sky    = vec3(0.34, 0.36, 0.40);
    vec3 ground = vec3(0.11, 0.11, 0.13);
    vec3 ambient = mix(ground, sky, N.z * 0.5 + 0.5);
    float key  = max(dot(N, kKeyDir),  0.0) * kKeyInt;
    float fill = max(dot(N, kFillDir), 0.0) * kFillInt;

    // A shade warmer and flatter than the surface, so the cut reads as
    // material and not as another face of the part.
    vec3 base = mix(uBaseColor, vec3(0.93, 0.80, 0.62), 0.18);
    base = mix(base, mix(base, uAccent, 0.35), uSelected);
    vec3 color = base * (ambient + key + fill) * 0.92;

    // 45 degrees in the plane, a pixel and a bit wide at any zoom.
    float s = (dot(P, uHatchU) + uHatchSign * dot(P, uHatchV)) / (uHatchSpacing * 1.41421356);
    float f = fract(s);
    float dist = min(f, 1.0 - f);
    float w = max(fwidth(s), 1e-6);
    float line = 1.0 - smoothstep(0.6 * w, 1.6 * w, dist);
    // Lines closer than a few pixels apart would be a grey wash; they fade out.
    line *= 1.0 - smoothstep(0.12, 0.25, w);
    color = mix(color, color * 0.45, line * 0.8);

    fragColor = vec4(pow(max(color, 0.0), vec3(1.0 / 2.2)), 1.0);
}
