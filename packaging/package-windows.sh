#!/usr/bin/env bash
# The Windows build as a portable folder, zipped: tangent.exe, the trial
# worker, shaders, assets, the DLLs vcpkg put beside them, and the Visual C++
# runtime, so nothing has to be installed first. Run in bash on the build
# machine, with the MSVC environment set (VCToolsRedistDir).
#   packaging/package-windows.sh <version>
set -euo pipefail
cd "$(dirname "$0")/.."
version=$1
name=tangent-$version-windows-x64
out=dist/$name

rm -rf dist && mkdir -p "$out"
cp build/tangent.exe build/tangent_trial.exe "$out/"
cp -r shaders "$out/"
mkdir "$out/assets"
cp -r assets/fonts assets/icons assets/icon.png assets/icon.svg "$out/assets/"
cp LICENSE packaging/NOTICE.txt "$out/"

# vcpkg copies each program's DLLs beside it as it builds.
cp build/*.dll "$out/"

# The C++ runtime, which Microsoft allows to travel with the program.
crt=$(cygpath -u "${VCToolsRedistDir:?run with the MSVC environment set}")/x64
crt=$(ls -d "$crt"/Microsoft.VC*.CRT | tail -1)
cp "$crt"/msvcp140*.dll "$crt"/vcruntime140*.dll "$out/"

# Each vcpkg package's licence, OpenCASCADE's LGPL among them.
mkdir "$out/licenses"
for dir in build/vcpkg_installed/x64-windows/share/*/; do
  pkg=$(basename "$dir")
  [ -f "$dir/copyright" ] && cp "$dir/copyright" "$out/licenses/$pkg.txt"
done
[ -f "$out/licenses/opencascade.txt" ] || { echo "OpenCASCADE's licence is missing"; exit 1; }
ls "$out"

(cd dist && 7z a -tzip -mx=9 "$name.zip" "$name" >/dev/null)
ls -l "dist/$name.zip"
