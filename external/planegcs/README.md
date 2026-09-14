# planegcs

FreeCAD's 2D geometric constraint solver, as used by its Sketcher. Tangent's
sketches are solved with it.

- **Origin:** `src/Mod/Sketcher/App/planegcs/` of FreeCAD, tag `1.1.3`
  (https://github.com/FreeCAD/FreeCAD/tree/1.1.3/src/Mod/Sketcher/App/planegcs)
- **Licence:** GNU Library General Public License, version 2 or (at your option)
  any later version, per the header of every file. FreeCAD's `LICENSE` is here
  alongside. Tangent is GPL-3.0, which that permits.
- **Author of the solver:** Konstantinos Poulios and the FreeCAD contributors.

## What is here

`src/Mod/Sketcher/App/planegcs/` holds FreeCAD's files **unmodified**, in
FreeCAD's own directory layout so that their relative includes resolve.
`SHA256SUMS` lists them as they were fetched.

planegcs expects a few things from the rest of FreeCAD. Those are supplied by
small shims written for Tangent rather than by editing the sources:

| Expected                              | Supplied by                                   |
|---------------------------------------|-----------------------------------------------|
| `SketcherExport` (`SketcherGlobal.h`) | `src/Mod/Sketcher/SketcherGlobal.h`, empty    |
| `Base::Console()` logging             | `shim/Base/Console.h`, discards messages      |
| `FCConfig.h`                          | `shim/FCConfig.h`, empty                      |
| Boost.Graph connected components      | `shim/boost_graph_adjacency_list.hpp`, a union-find numbering components as Boost does |
| Boost headers included but unused     | empty shims under `shim/boost/`               |
| FreeCAD's precompiled header          | `shim/planegcs_prelude.h`, force-included     |

Eigen, which planegcs uses throughout, is vendored in `external/eigen`.

## Updating

Replace the files under `src/Mod/Sketcher/App/planegcs/` with those from a newer
FreeCAD tag, regenerate `SHA256SUMS`, and build. A new include of something
outside planegcs will fail to compile, and needs a shim.
