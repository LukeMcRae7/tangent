# tangent

A 3D modelling tool built for 3D print design — direct, fast, and native.

Tangent is C++20 on OpenGL 3.3, targeting Linux/Wayland. It has no runtime
dependencies beyond the system GL stack, starts instantly, and holds a smooth
frame rate on modest hardware. Navigation follows Blender; editing shortcuts
follow Fusion 360.

**Status:** early. Viewport, parametric primitives, selection, modal
move/rotate/scale and undo all work — see [Roadmap](#roadmap).

## Build

Requires a C++20 compiler, CMake ≥ 3.24, SDL3 and libepoxy. Dear ImGui is
fetched automatically.

```sh
cmake -S . -B build -G Ninja
cmake --build build
./build/tangent
```

Run the tests (headless — no GL context required):

```sh
ctest --test-dir build --output-on-failure
```

`tests/grid_stability.py` is separate because it needs a GL context: it orbits
the camera in sub-pixel steps and measures how much the rendered image changes
between them, catching grid levels that pop or lines that breathe under
rotation. `--grid-align` is a companion diagnostic that checks grid lines are
drawn at the world coordinates they belong to.

```sh
python3 tests/grid_stability.py
```

### Command-line flags

| Flag | Purpose |
|------|---------|
| `--smoke-test N` | Render N frames and exit |
| `--screenshot out.ppm` | Capture the window to a PPM |
| `--camera yaw,pitch,dist` | Place the camera (degrees, mm) |
| `--empty` | Start with an empty scene |
| `--no-grid` | Hide the ground grid |
| `--grid-probe y0,y1,n` | Sweep yaw, printing viewport stability samples |

`TANGENT_FONT` overrides the UI font (or `default` for the built-in one) and
`TANGENT_SHADER_DIR` points at an alternate shader directory. Shaders reload on
save, so you can edit them while the app runs.

## Conventions

- **Millimetres**, **+Z up** — matching Fusion 360 and Blender.
- New objects are placed **on the build plate** (z = 0), not centred through it.
- The grid subdivides by powers of ten as you zoom, down to 0.1 mm.
- Snapping is relative to zoom: one step is always about the same distance on
  screen, rounded to a value you would actually pick (0.5, 1, 2, 5, 10 mm...).
  The active increment is shown in the status bar while you drag.

## Controls

| Input | Action |
|-------|--------|
| MMB drag | Orbit |
| Shift + MMB | Pan |
| Wheel | Zoom |
| Click | Select the edge, face or vertex under the cursor |
| Ctrl + click | Select the whole object (as clicking its outliner row) |
| Shift + click | Extend either selection |
| E / Shift + E | Extrude selected faces outward / inward |
| Ctrl + B | Bevel all edges of the active object |
| Numpad 1 / 3 / 7 | Front / Right / Top (Ctrl for opposite) |
| Numpad 4 / 6 / 8 / 2 | Orbit in 15° steps |
| Numpad 5 | Perspective / orthographic |
| Numpad . / Home | Frame selection / frame all |
| G / R / S | Move / rotate / scale the selection |
| X / Y / Z | *(during a transform)* constrain to an axis |
| Shift + X/Y/Z | *(during a transform)* constrain to a plane |
| type a number | *(during a transform)* exact value |
| Ctrl | *(during a transform)* snap to a round increment |
| Enter or click | Confirm transform |
| Esc or right click | Cancel transform |
| Ctrl + Z / Ctrl + Shift + Z | Undo / redo |
| Shift + A | Add object |
| A / Alt + A | Select all / deselect all |
| Shift + D | Duplicate |
| X or Delete | Delete |
| Z | Toggle wireframe |
| Ctrl + Q | Quit |

Orbit direction can be inverted per axis under **View → Invert Orbit**.

## Layout

```
src/core/     math and colour palette (header only)
src/mesh/     half-edge kernel, parametric primitive generators
src/scene/    scene graph, transforms, selection, ray picking
src/render/   shader and buffer wrappers, viewport renderer
src/app/      SDL3 shell, orbit camera, input dispatch, transform tool, undo
src/ui/       theme and panels
shaders/      GLSL 330, hot-reloaded
tests/        headless kernel and scene tests
```

Two choices shape everything above:

**Faces stay polygons.** The kernel keeps n-gons rather than triangulating on
creation, because extrude, inset, bevel and boolean are all defined on
polygonal faces. Triangulation happens only when building render buffers, and
every triangle remembers the face it came from, so picking resolves to a real
face.

**Geometry is derived, not authored.** An object stores the parameters it was
built from, so editing a value re-evaluates the mesh. This is the foundation
the parametric feature history will be built on.

## Theming

All colour lives in [`src/core/palette.h`](src/core/palette.h). Change
`kBrand` and the entire application follows — UI accents, selection
highlights, viewport outlines. Nothing else hardcodes a brand colour.

```cpp
inline constexpr Rgb kBrand = hex(0xFF4B33);
```

## Roadmap

1. ~~Viewport and object creation~~ — grid, orbit camera, six parametric
   primitives, selection, outliner and inspector
2. ~~Transforms~~ — modal move/rotate/scale with axis and plane constraints,
   exact numeric entry, snapping, and command-based undo/redo.
   *Draggable on-screen gizmo handles are not implemented yet; transforms are
   driven by the modal G/R/S grammar.*
3. **Face editing** — extrude, move face, fillet, full parametric history
   *(next)*
4. **Multi-object** — booleans, split
5. **Project files** — save/load, STL and 3MF export

## Licence

Not yet chosen.
