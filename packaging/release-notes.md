The first release of tangent: exact 3D modelling for printing, on Windows and Linux.

## Download

| Platform | File | How to run |
|---|---|---|
| Windows 10 or 11, 64-bit | `tangent-…-windows-x64.zip` | Unzip anywhere and run `tangent.exe`. Nothing is installed. |
| Linux, x86-64 | `tangent-…-linux-x86_64.AppImage` | `chmod +x` the file, then run it. |

tangent needs a graphics driver with OpenGL 3.3. The Linux build runs on
distributions from the Ubuntu 24.04 generation onwards (glibc 2.39).

The Windows build is not code-signed, so SmartScreen may warn before the first
run: choose **More info**, then **Run anyway**.

## What it does

- Models on OpenCASCADE's exact boundary representation, with STEP in and out
- Holes chosen by the screw, M2 to M10, with the allowance a printer needs
- Real threads, inside and outside
- Rounds and chamfers, per edge
- Sketches with constraints, and SVG drawings imported at their real size
- Splits with alignment pins, offset, delete face, push and pull
- Assemblies with joints, clearance checks and sections
- Exports 3MF and STL, checked to be solid before they leave

## Known limits

tangent is young. Expect rough edges, and please report what you find at
https://github.com/LukeMcRae7/tangent/issues

## Licence

GPL-3.0. Each package carries the licence, and the notices for the projects it
includes, beside the program.
