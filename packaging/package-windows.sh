#!/usr/bin/env bash
# The Windows build as a portable folder, zipped: tangent.exe, the trial
# worker, shaders, assets and every DLL they load that Windows does not have.
# Run inside an MSYS2 UCRT64 shell, from build/, with OpenCASCADE's bin/ on
# PATH.
#   packaging/package-windows.sh <version> [<OpenCASCADE prefix>]
set -euo pipefail
cd "$(dirname "$0")/.."
version=$1
occt=${2:-}
name=tangent-$version-windows-x64
out=dist/$name

rm -rf dist && mkdir -p "$out"
cp build/tangent.exe build/tangent_trial.exe "$out/"
strip "$out/tangent.exe" "$out/tangent_trial.exe"
cp -r shaders "$out/"
mkdir "$out/assets"
cp -r assets/fonts assets/icons assets/icon.png assets/icon.svg "$out/assets/"
cp LICENSE packaging/NOTICE.txt "$out/"

# Every DLL either program loads that is not part of Windows itself, and the
# licence each one's MSYS2 package installed.
mkdir "$out/licenses"
for dll in $( { ldd build/tangent.exe; ldd build/tangent_trial.exe; } \
              | awk '{print $3}' | grep -iv "^/c/windows/" | grep -i "\.dll$" | sort -u); do
  cp "$dll" "$out/"
  pkg=$(pacman -Qqo "$dll" 2>/dev/null || true)
  if [ -n "$pkg" ] && [ -d "$MINGW_PREFIX/share/licenses/${pkg#mingw-w64-ucrt-x86_64-}" ]; then
    cp -r "$MINGW_PREFIX/share/licenses/${pkg#mingw-w64-ucrt-x86_64-}" "$out/licenses/"
  fi
done
if [ -n "$occt" ]; then
  mkdir "$out/licenses/opencascade"
  find "$(cygpath "$occt")" -maxdepth 4 \( -name 'LICENSE_LGPL_21.txt' -o -name 'OCCT_LGPL_EXCEPTION.txt' \) \
    -exec cp {} "$out/licenses/opencascade/" \;
  [ -n "$(ls "$out/licenses/opencascade")" ] || { echo "OpenCASCADE's licence not found under $occt"; exit 1; }
fi
ls "$out"

(cd dist && 7z a -tzip -mx=9 "$name.zip" "$name" >/dev/null)
ls -l "dist/$name.zip"
