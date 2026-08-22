# tangent

A 3D modelling tool built for 3D print design — direct, fast, and native.

Tangent is C++20 on OpenGL 3.3, targeting Linux/Wayland. It has no runtime
dependencies beyond the system GL stack, starts instantly, and holds a smooth
frame rate on modest hardware. Navigation follows Blender; editing shortcuts
follow Fusion 360.

**Status:** early. The viewport, parametric primitives and selection work.
Transform tools are next — see [Roadmap](#roadmap).

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

### Command-line flags

| Flag | Purpose |
|------|---------|
| `--smoke-test N` | Render N frames and exit |
| `--screenshot out.ppm` | Capture the window to a PPM |
| `--camera yaw,pitch,dist` | Place the camera (degrees, mm) |

`TANGENT_FONT` overrides the UI font (or `default` for the built-in one) and
`TANGENT_SHADER_DIR` points at an alternate shader directory. Shaders reload on
save, so you can edit them while the app runs.

## Conventions

- **Millimetres**, **+Z up** — matching Fusion 360 and Blender.
- New objects are placed **on the build plate** (z = 0), not centred through it.
- The grid subdivides by powers of ten as you zoom, down to 0.1 mm.

## Controls

| Input | Action |
|-------|--------|
| MMB drag | Orbit |
| Shift + MMB | Pan |
| Wheel | Zoom |
| Click / Shift + click | Select / extend selection |
| Numpad 1 / 3 / 7 | Front / Right / Top (Ctrl for opposite) |
| Numpad 4 / 6 / 8 / 2 | Orbit in 15° steps |
| Numpad 5 | Perspective / orthographic |
| Numpad . / Home | Frame selection / frame all |
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
src/app/      SDL3 shell, orbit camera, input dispatch
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
2. **Transforms** — move/rotate/scale gizmos, G/R/S with axis constraints, undo
3. **Face editing** — extrude, move face, fillet, full parametric history
4. **Multi-object** — booleans, split
5. **Project files** — save/load, STL and 3MF export

## Licence

Not yet chosen.
