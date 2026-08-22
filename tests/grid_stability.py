#!/usr/bin/env python3
"""Grid stability under orbit.

Rotates the camera in sub-pixel steps and measures how much the rendered image
changes between consecutive steps. Because the rotation is small enough that
genuine motion moves the far field by well under a pixel, a stable grid should
barely change at all. Lines that breathe in width, or levels that pop, produce
a far larger frame-to-frame difference -- which is exactly what the eye reads
as jitter.

An earlier version of this test measured mean viewport luminance instead. That
was useless: line-width breathing roughly conserves total ink, so an
anisotropic width bug scored the same as a correct one. Whole-image averages
hide local instability; per-pixel differencing does not.
"""
import os, subprocess, sys, statistics, pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
BIN = ROOT / "build" / "tangent"
# Point at the source shaders, not the copy staged into build/ at compile time.
# Otherwise editing a shader and re-running the test silently measures the old
# one, and a fix looks like it changed nothing.
SHADER_DIR = ROOT / "shaders"
# A square grid repeats every 90 degrees; sweeping a slice that straddles the
# 45-degree diagonal covers the orientation where an anisotropic width measure
# is worst. Steps are ~0.004 deg, which moves the far field by a fraction of a
# pixel, so any large per-pixel change is instability rather than motion.
SWEEPS = [("diagonal", "43,47,240"), ("axis-aligned", "-2,2,240")]
# Median absolute change per channel byte, 0-255.
#
# The median, not the max or p95. This harness reads the back buffer of a live
# swap chain, which contributes single-frame transients: rendering the exact
# angles the sweep flags reproduces ~0.29 difference standalone versus the ~3.1
# the sweep reports, so those outliers are the measurement rather than the
# shading. Their *count* also varies between runs, which makes p95 unstable
# too. The median is reproducible to within a few percent across runs and is
# what a systematic shading problem actually moves.
#
# Reference values on this harness:
#   correct shader                median 0.35 - 0.37
#   floor() LOD (levels popping)  median 1.14 - 1.26   caught
#   fwidth() anisotropic width    median 0.39 - 0.45   marginal
#
# The anisotropy regression sits close to the floor of what this can resolve;
# it is fixed on correctness grounds (gradient length is the right measure of a
# scalar field's screen-space rate of change) rather than because this test
# forced it. Catching level-popping, the severe and visually obvious failure,
# is what this test is for.
THRESHOLD = 0.60


def probe(sweep, extra_args):
    env = dict(os.environ)
    env.setdefault("TANGENT_SHADER_DIR", str(SHADER_DIR))
    out = subprocess.run(
        [str(BIN), "--empty", "--camera", "0,26,120", "--grid-probe", sweep, *extra_args],
        capture_output=True, text=True, timeout=300, env=env)
    if out.returncode != 0:
        print(out.stderr, file=sys.stderr)
        raise SystemExit(f"tangent exited {out.returncode}")

    samples = []
    for line in out.stdout.splitlines():
        parts = line.split()
        if len(parts) == 3:
            try:
                samples.append(tuple(float(p) for p in parts))
            except ValueError:
                pass
    return samples


def main():
    if not BIN.exists():
        raise SystemExit(f"{BIN} not built")

    failed = False
    for name, sweep in SWEEPS:
        samples = probe(sweep, sys.argv[1:])
        if len(samples) < 20:
            raise SystemExit(f"expected a full sweep, got {len(samples)} samples")
        if max(v for _, v, _ in samples) <= 0.0:
            raise SystemExit("viewport is black - the grid did not render")

        # The first sample has no predecessor to difference against.
        diffs = [d for _, _, d in samples[1:]]
        ordered = sorted(diffs)
        median = statistics.median(diffs)
        worst = max(diffs)
        worst_at = samples[1 + diffs.index(worst)][0]
        transients = sum(1 for d in diffs if d > 1.5)

        print(f"[{name}] {sweep}  n={len(diffs)}")
        print(f"  flicker  median {median:.4f}"
              f"  p90 {ordered[int(len(ordered) * 0.90)]:.4f}"
              f"  max {worst:.4f} at {worst_at:.2f} deg"
              f"  ({transients} harness transient(s) ignored)")
        if median > THRESHOLD:
            print(f"  FAIL: exceeds threshold {THRESHOLD}")
            failed = True
        else:
            print(f"  ok (threshold {THRESHOLD})")

    print("\nFAIL: grid is unstable under orbit" if failed
          else "\nPASS: grid is stable under orbit")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
