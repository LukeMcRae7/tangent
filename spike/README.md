# Stage 0 spike — OpenCASCADE

Throwaway. Nothing in here ships, and nothing outside `spike/` was touched:
the main build does not know this directory exists, and `src/` is unchanged, so
the spike can be deleted with `rm -rf spike` and `git rm` when the gate is
decided either way.

## What it answers

| Program  | Question |
| -------- | -------- |
| `occt_probe` | Does OCCT build, link and run at all? |
| `compare`    | The same five parts on both kernels: does OCCT do what the mesh kernel refuses, and what does it cost? |
| `naming`     | Does a persistent name survive a boolean and a fillet through OCCT's provenance, with no geometric fallback? |
| `tess`       | What does display cost, at a chord deviation tied to the part? |
| `tess2`      | Which tessellator parameters actually move that cost? |

`compare` links the shipping mesh kernel straight out of `../src`, so the
control is the code that runs today rather than a copy of it that could drift.

## Getting OCCT without installing it

The spike was run against Arch's `opencascade` 7.9.3 unpacked into a local
prefix — no root, nothing added to the system:

    curl -LO 'https://geo.mirror.pkgbuild.com/extra/os/x86_64/opencascade-1%3A7.9.3-3-x86_64.pkg.tar.zst'
    mkdir -p prefix && tar --zstd -xf opencascade-*.pkg.tar.zst -C prefix

Only the modelling libraries are linked — no visualization — which is what
keeps the dependency to 15 shared objects and about 32 MB, with nothing beneath
them but libc, libstdc++ and libm. Visualization is what would drag in vtk,
tcl/tk, freetype and X, and Tangent has its own renderer.

## Build and run

    cmake -S spike -B spike/build -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_PREFIX_PATH=<the prefix above>/usr
    cmake --build spike/build -j
    LD_LIBRARY_PATH=<the prefix above>/usr/lib ./spike/build/compare

## What was not answered

No Windows build was made. That is the one Stage 0 exit criterion still open,
and the plan is explicit that it should be settled before the gate rather than
discovered in Stage 6.
