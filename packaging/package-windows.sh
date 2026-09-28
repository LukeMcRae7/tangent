#!/usr/bin/env bash
# The Windows build as a portable folder, zipped: tangent.exe, the trial
# worker, shaders, assets and every DLL they load from the MSYS2 prefix.
# Run inside an MSYS2 UCRT64 shell, from build/.
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

# Every DLL either program loads that comes from the MSYS2 prefix, not from
# Windows itself, and the licence each one's package installed.
mkdir "$out/licenses"
for dll in $( { ldd build/tangent.exe; ldd build/tangent_trial.exe; } \
              | awk '{print $3}' | grep -i "^$MINGW_PREFIX/bin/" | sort -u); do
  cp "$dll" "$out/"
  pkg=$(pacman -Qqo "$dll" 2>/dev/null || true)
  if [ -n "$pkg" ] && [ -d "$MINGW_PREFIX/share/licenses/${pkg#mingw-w64-ucrt-x86_64-}" ]; then
    cp -r "$MINGW_PREFIX/share/licenses/${pkg#mingw-w64-ucrt-x86_64-}" "$out/licenses/"
  fi
done
ls "$out"

(cd dist && 7z a -tzip -mx=9 "$name.zip" "$name" >/dev/null)
ls -l "dist/$name.zip"
