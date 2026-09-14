<p align="center">
  <img src="assets/logo.png" alt="Project Logo" width="300" height="auto">
</p>
Meet tangent, a Linux-based 3D modeling software built for the ergonomics of Blender with the precision of Fusion360. Built for 3D printing purposes, it models on an exact kernel and exports to .stl natively, rejecting invalid operations immediately rather than handing back geometry that looks right and is not. It's fast, accurate, and just works. Tangent is C++ on OpenGL 3.3, targeting Linux/Wayland; the exact kernel is an optional dependency on OpenCASCADE, and without it the build has none beyond the system GL stack.<br><br>

**Status:** early development, not ready for use

## Build

Requires a C++20 compiler, CMake ≥ 3.24, SDL3, libepoxy and OpenCASCADE.

```sh
sudo pacman -S opencascade          # or your distribution's equivalent

cmake -S . -B build -G Ninja
cmake --build build
./build/tangent
```

Run the tests:

```sh
ctest --test-dir build --output-on-failure
```

### Either platform, either kernel

Presets cover the four combinations, and the same three commands drive all of
them:

|             | Exact (default) | Without kernel |
|-------------|-----------------|----------------|
| **Linux**   | `linux`         | `linux-mesh`   |
| **Windows** | `windows`       | `windows-mesh` |

```sh
cmake --preset windows
cmake --build --preset windows
ctest --preset windows
```

Linux takes the dependencies from the system. Windows takes them from vcpkg
through `vcpkg.json`, so it wants `VCPKG_ROOT` set and an x64 Native Tools
Command Prompt, which is where Ninja finds MSVC.

### Without the exact kernel

Modelling is done on an exact boundary representation, where a hole is a
cylinder rather than a thirty-two-sided prism. It needs OpenCASCADE; only the
modelling libraries are linked -- no visualization -- so it costs fifteen
shared objects and about 32 MB, with nothing beneath them but libc, libstdc++
and libm.

A build without it still compiles and runs, and can import, look at, measure,
reduce and export meshes. Every modelling command in it refuses and says the
kernel is missing:

```sh
cmake -S . -B build -G Ninja -DTANGENT_BREP=OFF
```

The test suite gains one suite with the exact kernel (28 rather than 27), and
the modelling sections of the others run only when it is there.

## Features

### Exact for modelling, meshes for exchange
Tangent models on an exact boundary representation. A hole is a cylinder, so
its diameter is 8mm rather than 7.994mm; a fillet is a real blend, so rounding
every rim of a bolt circle in one operation works rather than being refused;
and a face is one face, so selecting a bored surface selects the surface.

**Meshes** are the native object of `.stl` and `.obj` files, and importing one
needs no conversion to look at it. A mesh can be moved, measured, checked,
separated into its pieces, reduced to fewer triangles within a tolerance you
set, and exported as it is. Anything that edits part of a shape -- a boolean, a
split, a fillet -- asks for it to be converted first (Modify > Convert to
Solid), and a mesh too dense to convert usefully can be reduced until it is.

Either way, tangent rejects non-manifold edges, open surfaces, and other
invalid results that a slicer would also reject. The status bar indicates
`solid` or `not solid`, and the Inspector says which kernel a body is made of.

### Conventions

- **Millimetres**, **+Z up**
- New objects are placed **on the build plate** (z = 0), not centred through it
- The grid subdivides by powers of ten as you zoom, down to 0.1 mm.
- **Snapping is on by default**, and Ctrl releases it. A part is designed in
  round numbers; free positioning is the exception. The increment is relative
  to the viewport zoom such that you can make more precise edits when zoomed closer. It is shown in
  the status bar while you drag.
- **An edit either produces valid geometry or it does not happen.** A drag that
  would make the model self-intersect is refused and reverted.
- Numeric entry is always available during any transform.

### Default Controls

| Input | Action |
|-------|--------|
| MMB drag | Orbit |
| Shift + MMB | Pan |
| Wheel | Zoom |
| Click | Select the edge, face or vertex under the cursor |
| Ctrl + click | Select the whole object (as clicking its outliner row) |
| Shift + click | Extend either selection |
| E | Extrude selected faces, then drag to set the height |
| F | Fillet the selected edges |
| Ctrl + B | Bevel all edges of the active object |
| Ctrl + Shift + U / D / I | Union / difference / intersect the two selected objects |
| D | Measure — one entity for its own size, two for the distance between |
| Numpad 1 / 3 / 7 | Front / Right / Top (Ctrl for opposite) |
| Numpad 4 / 6 / 8 / 2 | Orbit in 15° steps |
| Numpad 5 | Perspective / orthographic |
| Numpad . / Home | Frame selection / frame all |
| G / R / S | Move / rotate / scale — the object, or the selected faces/edges/vertices |
| X / Y / Z | *(during a transform)* constrain to an axis |
| Shift + X/Y/Z | *(during a transform)* constrain to a plane |
| type a number | *(during a transform)* exact value |
| Ctrl | *(during a transform)* release the snap for free positioning |
| Enter or click | Confirm transform |
| Esc or right click | Cancel transform |
| Ctrl + Z / Ctrl + Shift + Z | Undo / redo |
| Shift + A | Add object |
| A / Alt + A | Select all / deselect all |
| Shift + D | Duplicate |
| X or Delete | Delete |
| Z | Toggle wireframe |
| Ctrl + N / O / S | New / open / save project |
| Ctrl + E | Export STL |
| Ctrl + Q | Quit |

## Project Structure

```
src/core/     math and colour palette (header only)
src/geom/     bodies, the exact kernel behind them, and every modelling operation
src/mesh/     half-edge meshes: primitives, import, reduction, printability checks
src/scene/    scene graph, feature history, selection, ray picking
src/render/   shader and buffer wrappers, viewport renderer
src/app/      SDL3 shell, orbit camera, input dispatch, transform tool, undo
src/ui/       theme and panels
shaders/      GLSL 330, hot-reloaded
tests/        headless kernel and scene tests
```

## Licence
GPL-3.0