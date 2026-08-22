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

// Coverage of the nearest grid line.
//
// `grad` carries the screen-space gradient magnitude of P.xy, computed once by
// the caller. Derivatives must NOT be taken in here: the spacing passed in
// depends on the LOD, which can differ between pixels of the same 2x2 quad,
// and dFdx across a quad whose pixels disagree is undefined.
float gridCoverage(vec2 p, float spacing, vec2 grad) {
    vec2 coord = p / spacing;
    vec2 d = max(grad / spacing, vec2(1e-8));
    vec2 g = abs(fract(coord - 0.5) - 0.5) / d;
    return 1.0 - min(min(g.x, g.y), 1.0);
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

    // Rotation-invariant line width.
    //
    // fwidth(v) is |dFdx(v)| + |dFdy(v)|, which is an L1 measure and therefore
    // depends on how the grid happens to be oriented on screen: a line running
    // along a screen axis measures thinner than the same line running
    // diagonally. Under orbit that makes every line breathe in width and
    // brightness as it sweeps through 45 degrees -- the jitter. The true
    // screen-space rate of change of a scalar field is the length of its
    // gradient, which is isotropic and so invariant under rotation.
    vec3 dPdx = dFdx(P);
    vec3 dPdy = dFdy(P);
    vec2 grad = vec2(length(vec2(dPdx.x, dPdy.x)),
                     length(vec2(dPdx.y, dPdy.y)));

    // Choose the level so the finest drawn cell stays at least kPixelsPerCell
    // wide. That means rounding the level *up* (ceil): floor would select the
    // next level finer, which is below the density we can actually resolve.
    const float kPixelsPerCell = 12.0;

    // The scale measure must not depend on which way the camera is facing.
    // max(grad.x, grad.y) does: those are the screen densities of the two line
    // families, and how they compare depends entirely on how the world axes
    // happen to lie relative to the view. Over one orbit that swings the
    // chosen level by up to sqrt(2), which fades the fine lines in and out as
    // the view turns -- the grid appears to change scale while the geometry
    // stands still.
    //
    // The area of a pixel's footprint on the ground plane has no such
    // dependence: orbiting rotates that footprint but does not resize it. Its
    // square root is the characteristic world length per pixel, and it is
    // invariant under rotation about the plane normal.
    float cell = sqrt(max(length(cross(dPdx, dPdy)), 1e-16));
    float lodF = log2(max(cell * kPixelsPerCell / uSpacing, 1e-8)) / log2(uSubdivide);
    float lod  = max(ceil(lodF), -1.0);
    float k    = clamp(lod - lodF, 0.0, 1.0);   // 0 = just resolvable, 1 = about to subdivide

    float sMain  = uSpacing * pow(uSubdivide, lod);
    float sFine  = sMain / uSubdivide;
    float sMajor = sMain * uSubdivide;

    // Weights must be continuous across a LOD step. Writing L(j) for the level
    // at spacing*subdivide^j, the step takes (fine, main, major) from
    // (L(m-1), L(m), L(m+1)) at k=0 to (L(m), L(m+1), L(m+2)) at k=1, so every
    // level's weight has to match across that hand-off:
    //
    //   L(m-1):  wFine(0)  = 0        (it is absent afterwards)
    //   L(m)  :  wMain(0)  = wFine(1)
    //   L(m+1):  wMajor(0) = wMain(1)
    //   L(m+2):  wMajor(1) = 0        (it was absent beforehand)
    //
    // That last one is the easy one to miss: leaving the major weight constant
    // makes an entire grid level appear at full strength the instant the LOD
    // steps, which is a whole-frame flash rather than a local artefact.
    const float kA = 0.22;   // weight a level carries when newly resolvable
    const float kB = 0.50;   // weight once it has become a major line

    // The fine level spans subdivide-to-one in density across a single fade
    // cycle -- with a ratio of 10 that is 1.2px up to 12px per cell. Fading it
    // linearly in `k` therefore draws it at heavy sub-pixel density through
    // most of the cycle, producing moire that slides violently under small
    // rotations. Gating on its actual screen density instead holds it at zero
    // until it is genuinely resolvable.
    //
    // This still satisfies the hand-off conditions above: the gate is 0 when
    // the fine level is at its densest (k=0) and 1 when it has grown to the
    // main level's density (k=1), which is exactly what wFine(0)=0 and
    // wFine(1)=kA require.
    float pxFine = sFine / max(cell, 1e-8);
    float wFine  = kA * smoothstep(2.0, kPixelsPerCell, pxFine);
    float wMain  = kA + (kB - kA) * k;
    float wMajor = kB * (1.0 - k);

    float aFine  = gridCoverage(P.xy, sFine,  grad) * wFine;
    float aMain  = gridCoverage(P.xy, sMain,  grad) * wMain;
    float aMajor = gridCoverage(P.xy, sMajor, grad) * wMajor;

    // Brightness follows the same continuity rule as the weights.
    vec3  color = uLineColor;
    float alpha = aFine;
    if (aMain > alpha)  { alpha = aMain;  color = uLineColor * mix(1.0, 1.5, k); }
    if (aMajor > alpha) { alpha = aMajor; color = uLineColor * 1.5; }

    // Coloured world axes, drawn over the grid lines.
    const float kAxisWidth = 1.4;
    float ax = 1.0 - min(abs(P.y) / max(grad.y * kAxisWidth, 1e-8), 1.0);
    float ay = 1.0 - min(abs(P.x) / max(grad.x * kAxisWidth, 1e-8), 1.0);
    if (ax > 0.0) { color = uAxisXColor; alpha = max(alpha, ax); }
    if (ay > 0.0) { color = uAxisYColor; alpha = max(alpha, ay); }

    fragColor = vec4(color, alpha * fade);
}
