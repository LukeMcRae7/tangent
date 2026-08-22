# tangent

A 3D modelling tool built for 3D print design — direct, fast, and native.

Tangent is C++20 on OpenGL 3.3, targeting Linux/Wayland. It has no runtime
dependencies beyond the system GL stack, starts instantly, and holds a smooth
frame rate on modest hardware. Navigation follows Blender; editing shortcuts
follow Fusion 360.

**Status:** early. Viewport, parametric primitives, selection, modal
move/rotate/scale and undo all work — see [Roadmap](#roadmap).

## Performance

Editing stays interactive on heavy models. Measured on a 101,760-triangle mesh
(`cmake --build build --target bench && ./build/bench`):

| | before | now |
|---|---|---|
| Dragging a parameter slider | 122 ms | 27 ms |
| Half-edge construction | 23 ms | 10 ms |
| Full printability check | 409 ms | 77 ms |

Three things get that: the feature chain is evaluated **incrementally**, so
editing the last step re-runs that step rather than the whole history; twin
pairing uses per-vertex buckets instead of hashing every directed edge; and the
printability check is debounced, so it never runs mid-drag.

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

## Solids, not soup

The usual complaint about mesh modelling is that you end up with non-manifold
geometry and only find out in the slicer. That is a consequence of how
general-purpose modellers are built, not of meshes: they permit non-manifold
topology on purpose, because floating edges and open surfaces are useful while
modelling.

Tangent takes the opposite position, because the output is a printed part:

- `Mesh::build` **rejects** non-manifold input — an edge shared by more than
  two faces, or two surface sheets meeting at a single vertex. There is no way
  to construct such a mesh.
- Every operation is **transactional**: it rebuilds into a scratch mesh through
  that same check and only commits if it validates. A bevel too wide for the
  geometry leaves your model exactly as it was.
- What remains possible is geometry that is manifold but still not a solid —
  an open surface, a zero-area face, or a mesh that passes through itself.
  Direct edits are **validated on commit and refused** if they would break a
  model that was sound. The status bar shows `solid` / `not solid` and the
  Inspector lists exactly what is wrong.

## Precision

Geometry is computed in **double precision**. Booleans are the reason: their
robustness is dominated by how reliably a point can be classified against a
surface, and the hard cases are always near-degenerate — faces that are almost
coplanar, an edge passing almost exactly through a vertex. At float32 a 100 mm
part resolves to about 7.6e-06 mm, which is not enough headroom for those
decisions; in double it is 1.4e-14 mm.

Narrowing to float32 happens in exactly two places, both at the GPU boundary:
`Shader::set` for uniforms and `GpuMesh::upload` for vertex data.

## Conventions

- **Millimetres**, **+Z up** — matching Fusion 360 and Blender.
- New objects are placed **on the build plate** (z = 0), not centred through it.
- The grid subdivides by powers of ten as you zoom, down to 0.1 mm.
- **Snapping is on by default**, and Ctrl releases it. A part is designed in
  round numbers; free positioning is the exception. The increment is relative
  to zoom — one step is always about the same distance on screen — and rounded
  to a value you would actually pick (0.5, 1, 2, 5, 10 mm...). It is shown in
  the status bar while you drag.
- **An edit either produces valid geometry or it does not happen.** A drag that
  would make the model self-intersect is refused and reverted, not accepted and
  flagged. Numeric entry is always available during any transform.

## Controls

| Input | Action |
|-------|--------|
| MMB drag | Orbit |
| Shift + MMB | Pan |
| Wheel | Zoom |
| Click | Select the edge, face or vertex under the cursor |
| Ctrl + click | Select the whole object (as clicking its outliner row) |
| Shift + click | Extend either selection |
| E | Extrude selected faces, then drag to set the height |
| Ctrl + B | Bevel all edges of the active object |
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
| Ctrl + Q | Quit |

Orbit direction can be inverted per axis under **View → Invert Orbit**.

## Layout

```
src/core/     math and colour palette (header only)
src/mesh/     half-edge kernel, primitives, operations, printability checks
src/scene/    scene graph, feature history, selection, ray picking
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

**Geometry is derived, not authored.** An object is a *feature chain* — a base
primitive followed by operations — and its mesh is evaluated from that chain.
Editing any parameter re-runs everything after it. Widen the base box and an
extrusion added later re-applies to the wider box.

The known limit, stated plainly because it will bite: operations name faces by
index. Numbering is stable while earlier features are unchanged, so editing
dimensions works. Inserting or reordering a feature renumbers everything
downstream, and an index cannot follow that — the topological naming problem,
which needs identifiers that survive a remesh. Rather than silently acting on
the wrong face, a step whose references no longer resolve is marked failed,
skipped, and shown as failed in the History panel.

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
4. **Multi-object** *(in progress)* — the boolean kernel is done and tested
   (union, difference, intersection, with exact volume assertions). Still to
   do: wiring it to the UI as a two-object operation, and split.
5. **Project files** — save/load, STL and 3MF export

## Licence

Not yet chosen.
