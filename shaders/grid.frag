#version 330 core
in vec3 vNear;
in vec3 vFar;
out vec4 fragColor;

uniform mat4  uViewProj;
uniform vec3  uCameraPos;
uniform float uFadeStart;
uniform float uFadeEnd;
uniform float uSpacing;      // finest cell, mm
uniform float uSubdivide;    // ratio between levels (10)
uniform vec3  uAxisXColor;
uniform vec3  uAxisYColor;
uniform vec3  uLineColor;

// Coverage of the nearest grid line, antialiased with screen-space
// derivatives. Per-axis on purpose: a line running along Y is thin in X, so
// its width must be measured in X.
float gridCoverage(vec2 p, float spacing, float width) {
    vec2 coord = p / spacing;
    vec2 d = fwidth(coord);
    vec2 g = abs(fract(coord - 0.5) - 0.5) / max(d, vec2(1e-8));
    return 1.0 - min(min(g.x, g.y) / width, 1.0);
}

void main() {
    // Intersect the view ray with the Z = 0 ground plane.
    float t = -vNear.z / (vFar.z - vNear.z);
    if (t <= 0.0 || t >= 1.0) discard;
    vec3 P = vNear + t * (vFar - vNear);

    // Depth-test the grid against scene geometry like real geometry.
    vec4 clip = uViewProj * vec4(P, 1.0);
    gl_FragDepth = (clip.z / clip.w) * 0.5 + 0.5;

    float dist = length(P - uCameraPos);
    float fade = 1.0 - smoothstep(uFadeStart, uFadeEnd, dist);
    if (fade <= 0.001) discard;

    // Pick the grid level from an *isotropic* measure of world-units-per-pixel.
    // Keying this off fwidth(P.x) alone makes the chosen level depend on which
    // way the camera happens to face, so the fine lines pop in and out as the
    // view orbits. Taking both axes keeps the level stable under rotation.
    float cell = max(fwidth(P.x), fwidth(P.y));

    // Continuous level, then split into an integer level plus a blend factor,
    // so zooming crossfades between decades instead of switching abruptly.
    const float kPixelsPerCell = 10.0;
    float lodF  = log2(max(cell * kPixelsPerCell / uSpacing, 1e-8)) / log2(uSubdivide);
    float lod   = max(floor(lodF), -1.0);          // allow one level finer than base
    float blend = clamp(lodF - lod, 0.0, 1.0);

    float fine   = uSpacing * pow(uSubdivide, lod);
    float coarse = fine * uSubdivide;

    // The fine level fades out exactly as the coarse level takes over.
    float covFine   = gridCoverage(P.xy, fine, 1.0) * (1.0 - blend);
    float covCoarse = gridCoverage(P.xy, coarse, 1.0);

    vec3  color = uLineColor;
    float alpha = covFine * 0.26;

    float coarseAlpha = covCoarse * 0.50;
    if (coarseAlpha > alpha) {
        color = uLineColor * 1.35;
        alpha = coarseAlpha;
    }

    // Coloured world axes, drawn over the grid lines.
    const float kAxisWidth = 1.4;
    float ax = 1.0 - min(abs(P.y) / (fwidth(P.y) * kAxisWidth), 1.0);
    float ay = 1.0 - min(abs(P.x) / (fwidth(P.x) * kAxisWidth), 1.0);
    if (ax > 0.0) { color = uAxisXColor; alpha = max(alpha, ax); }
    if (ay > 0.0) { color = uAxisYColor; alpha = max(alpha, ay); }

    fragColor = vec4(color, alpha * fade);
}
