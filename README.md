<p align="center">
  <img src="assets/logo.png" alt="Project Logo" width="300" height="auto">
</p>
Meet tangent, a Linux-based 3D modeling software built for the ergonomics of Blender with the precision of Fusion360. Built for 3D printing purposes, it models on an exact kernel and exports to .3mf and .stl natively, rejecting invalid operations immediately rather than handing back geometry that looks right and is not. It's fast, accurate, and just works. Tangent is C++ on OpenGL 3.3, targeting Linux/Wayland; the exact kernel is an optional dependency on OpenCASCADE, and without it the build needs nothing beyond the system GL stack and zlib.<br><br>

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

A kernel call that might crash is tried in another process first. Linux forks
for it; Windows has no fork, so the build makes `tangent_trial` for the purpose,
and it has to stay beside `tangent` wherever that is copied. Setting
`TANGENT_ISOLATION=worker` uses it on Linux too, which is how that path is
tested.

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

The test suite gains one suite with the exact kernel (30 rather than 29), and
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

**For the slicer**, a model goes out as 3MF -- in millimetres, each part kept
separate and named, every mesh closed -- or as STL, in one file or one per
part. Either way an exact body is tessellated to a tolerance you set rather
than whatever the viewport happened to be drawing.

Either way, tangent rejects non-manifold edges, open surfaces, and other
invalid results that a slicer would also reject. The corner of the viewport
says `solid` or `not solid` for the body in hand.

**Drawings come in as sketches.** File > Import SVG puts an SVG's outlines into
a sketch on a plane or on the face of a part, at the size the file says it is
and the right way up, then lets it be sized, moved and turned before anything
else is drawn. Lines stay lines, circles and circular arcs stay circles and
arcs (so they can be dimensioned), quadratic curves become the cubics they
exactly are, and only ellipses are approximated, as every drawing program does.
Text and images are left out and counted, not silently dropped: convert text to
paths first. Extruding picks the filled regions -- a letter and not the hole in
it -- and sweeps them as one step: a drawing of 240 letters is under a second,
not one boolean per letter.

**A hole is chosen, not measured.** Press H, point at the face it goes into
and click. What the dialog asks for is the screw -- M2 to M10, close, normal or
loose, or tapped for one that cuts its own thread -- and whether its head sits
in a counterbore or a countersink; the millimetres come from the standards, and
the panel says what it arrived at. A printed hole is cut two tenths over size,
because that is roughly what a nozzle takes back off it, and a tapped one is
not, because the thread needs the material. A blind hole ends in the cone a
drill leaves, which is also the shape a printer wants: a flat ceiling over a
hole has nothing to print onto. The hole belongs to its face, so when that face
moves the hole goes with it.

**A thread is cut, not drawn on.** Select a bore or a shaft, Modify > Thread,
and pick the screw: the helix is real geometry, because a printed part has no
second operation to cut it with. The face decides which kind it is -- a bore
takes an inside thread and a shaft an outside one -- and the fit allows for
printing by default, cutting an inside thread a tenth deeper and an outside one
a tenth shallower, so a steel screw turns into plastic instead of splitting it.

**A body can be grown or shrunk whole.** Modify > Offset moves every face along
its own normal, corners staying corners, so a 20 mm cube offset by 1 mm is a
22 mm cube rather than one with a round on every edge. It is how a clearance
copy of a part is made -- two tenths bigger all round to fit the pocket it sits
in -- and how the void a part needs is made from the part.

**A split can cut its own alignment pins.** A part too big for the bed is cut
in two, and the Split panel will put two, three or four pins across the cut
while it is there -- a pin on one half and a socket in the other, or a socket
in each for a dowel. The socket is cut wider than the pin by the same allowance
a printed hole gets, because a pin that has to be forced is a pin that splits
the part it is going into, and the panel says what the fit comes to. A cut with
nowhere to put a pin still cuts, and says why the halves came out bare.

**A face can be taken off.** Modify > Delete Face removes the faces you picked
and grows what is around them back over the gap: the hole fills in, the boss
goes, the fillet becomes the corner it rounded. It is the operation for a shape
nobody kept a history for -- a STEP file somebody else made -- and it is the
one place the tool edits geometry rather than a step. A gap that cannot be
closed is refused with the reason, rather than handing back the body unchanged
and calling it done.

**A curved face is pushed along itself.** Divide a cylinder twice and the band
of wall between the cuts is a face like any other: Push/Pull (G) moves it out
into a collar or in into a groove, Extrude (E) raises it keeping its outline,
and Scale (S) reads as what scaling a round face means -- 120 per cent of a
15 mm wall is an 18 mm one. There is no single direction to sweep a curved face
along, so what is built is the face moved along its own surface, and a bore
widened this way stays a cylinder rather than becoming a prism leaning off in
whichever direction the middle of it happened to face.

**Walls can be leant off the bed.** Modify > Draft takes the selected walls and
tips them a few degrees away from the direction the part is pulled, which for a
printed part is up off the bed and for a moulded one is out of its tool. Each
wall keeps the size it was drawn where it crosses the plane you name -- the
bottom, the top or half way -- and the panel says how far the far end has come
in, in millimetres, because that is the number that decides whether it prints.

**A sketch is drawn, finished, and then built from.** Sketch (Shift+S) asks for
a plane the way a new shape does -- a face of a body, an origin plane, a plane
through three corners, or one square to an edge at its nearer end -- and both
the sketch and the shape tool then let the plane stand off by a distance and
lean about its own horizontal. Lines, rectangles, circles and arcs are drawn
with the constraints the drawing implies; the pen (B) draws curves -- click for
a corner, press and drag for a smooth point, whose handles stay mirrored after,
and click the first point to close the shape. Project (P) brings an edge of a
body, or a face's whole outline, onto the plane, fixed where it is; projected
from the part the sketch is in, it follows that edge when the part changes.
Enter, or Finish, keeps the sketch -- it builds nothing -- and leaves it
selected.

A sketch is then a thing of its own: click one in the view or its row in the
outliner and a bar of what can be done with it opens over the view, and the
inspector says what it is. Edit opens it again (so does a double-click);
Extrude asks for its regions and a depth; Revolve, Sweep and Loft start with
its regions as their profile; Hide and Delete do what they say, and Delete
refuses while something is built from it.

**Revolve, sweep and loft, from sketches or from the part itself.** Each has a
button beside Sketch on the bar and an entry in the Create menu, and opens a
panel with a short list of things to point at, filled in order: Revolve takes a
profile and an axis, Sweep a profile and a path, Loft two or more outlines in
the order the solid runs through them. The row the next click fills is lit, the
pointer carries a line saying what it is for, and the solid it will make is
shown see-through as soon as there is enough to make it -- so a wrong axis or a
path running the wrong way is seen before Finish, not after.

A profile or an outline is a region of a sketch or a flat face of a body. An
axis is a line of a sketch, a straight edge, or X, Y or Z, with Reverse for
which way a part turn goes. A path is sketch curves -- one click takes the whole
run it is joined to -- or edges of a body, clicked one by one, and need not lie
in a plane: a bar bent round the rim of a can is the rim's edge. A sweep is held
square to its path and mitred at corners, and starts from whichever end is
nearer its profile; a loft's walls are smooth or straight. Faces and edges
selected before the command starts are taken as what they can be.

Whatever it was built from, it is one step in the part's history. Faces and
edges are named there, so they are found again when the steps before them
change; sketches are brought in before the step -- one that stood on its own in
the outliner moved, one in another part copied. With a sketch selected, the
panel opens with its regions picked. A sketch's plane can stand off the plane
or face it was put on by any distance, which is how the next outline of a loft
is drawn above the first.

**The history can be gone back into.** Every part keeps the steps that made
it, in order, in the inspector. The orange marker under them is where the model
stands: drag it up onto a step, or right-click a step for Roll back to here or
Insert before this, and the steps after it wait, dimmed, while the view shows
the part as it was there. Whatever is done then goes in at the marker; Step on
and To end run the waiting steps again on top of it, each finding the faces
and edges it names on the part as it now is -- push a cube's top up under a
rounded corner and the round rounds the taller edge. A step dragged to another
place runs from there, refused if it would come before the sketch it is built
from. Double-click a step to name it: "Bolt circle" reads better than what it
does, which stays in the tooltip. A step that no longer builds is marked with
what went wrong, and its Fix rolls back to just before it: select the faces or
edges it should act on, click Use the selected faces, and it and everything
after it run again.

### The interface

One dark surface with the model lit in the middle of it. Along the top, the
tools in four groups -- File, Create, Modify, Inspect -- each a row of pictures
with its name under it; the name opens the group's full menu, a list of
commands and their keys, each explaining itself on hover. The outliner down the
left lists the bodies, sketches and meshes there are, with an eye to hide each;
the inspector down the right holds the selected thing's name, transform, shape
and history, with its volume, vertex and face counts at the foot.

A running operation gets a panel at the bottom of the viewport: its name,
one bar per number (pull the bar, pull the arrow in the viewport, or type),
the choices among modes as a row of tiles with their keys, and Finish and
Cancel. The arrow the value is pulled along is drawn on the screen, where the
value is measured, so its head sits under the pointer.

Nothing in that panel explains itself in prose. What the operation does, and
what the keys are, is behind the **?** in its top right corner and comes out on
hover; the panel itself is the numbers and the choices, and the line at the top
left of the viewport says what the tool is doing and what its value is, and
nothing else.

The application draws its own window frame: the bar along the top drags the
window, double-clicking it maximises, and its right-hand end holds the
window's own buttons. `--native-frame` (or `TANGENT_NATIVE_FRAME=1`) keeps
the system's title bar and borders instead, for a desktop that cannot move
or resize a window from inside it.

The face is Space Grotesk, bundled in `assets/fonts` under the SIL Open Font
License, so the interface looks the same on every platform. `TANGENT_FONT`
names a different file, or `default` for ImGui's built-in bitmap face.

The icons are [Tabler Icons](https://tabler.io/icons), bundled as
`assets/fonts/tabler-icons.ttf` under the MIT License
(`assets/fonts/tabler-icons-LICENSE.txt`). Each one is chosen by name in
`src/ui/glyph.cpp`, so a new one is a codepoint from the webfont's
`tabler-icons.css`.

### Conventions

- **Millimetres**, **+Z up**
- New objects are placed **on the build plate** (z = 0), not centred through it
- The grid subdivides by powers of ten as you zoom, down to 0.1 mm.
- **Snapping is on by default**, and Ctrl releases it. A part is designed in
  round numbers; free positioning is the exception. The increment is relative
  to the viewport zoom such that you can make more precise edits when zoomed closer. The ticks
  along the arrow show it while you drag.
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
| E | Extrude selected faces, then drag to set the height (Shift + E starts it as a cut) |
| J / D / I / N | *(while extruding)* join, cut, intersect or a new body — until one is picked the drag decides: out joins, in cuts |
| F | Fillet the selected edges |
| H | Hole — point at the face it goes into and click; the size is chosen in the dialog |
| Ctrl + B | Bevel all edges of the active object |
| Ctrl + Shift + U / D / I | Combine, set to join / cut / intersect: the first selected body is the target, the rest are tools |
| D | Measure — one entity for its own size, two for the distance between |
| Numpad 1 / 3 / 7 | Front / Right / Top (Ctrl for opposite) |
| Numpad 4 / 6 / 8 / 2 | Orbit in 15° steps |
| Numpad 5 | Perspective / orthographic |
| Numpad . / Home | Frame selection / frame all |
| G / R / S | Move / rotate / scale — the object, or the selected faces/edges/vertices. An object's move, turn or scale is a step in its history |
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
| Ctrl + Shift + E | Export 3MF |
| Ctrl + Q | Quit |

## Project Structure

```
src/core/     math and colour palette (header only)
src/geom/     bodies, the exact kernel behind them, and every modelling operation
src/mesh/     half-edge meshes: primitives, import, reduction, printability checks
src/scene/    scene graph, feature history, selection, ray picking
src/sketch/   constrained 2D sketches, their regions, and SVG import
src/render/   shader and buffer wrappers, viewport renderer
src/app/      SDL3 shell, orbit camera, input dispatch, transform tool, undo
src/ui/       theme and panels
shaders/      GLSL 330, hot-reloaded
tests/        headless kernel and scene tests
```

## Licence
GPL-3.0