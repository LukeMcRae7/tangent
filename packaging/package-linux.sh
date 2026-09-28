#!/usr/bin/env bash
# The Linux build as one AppImage: tangent, the trial worker, shaders, assets
# and every library that is not part of the base system, from build/.
#   packaging/package-linux.sh <version> [<library prefix>]
set -euo pipefail
cd "$(dirname "$0")/.."
version=$1
prefix=${2:-/usr}

rm -rf AppDir && mkdir -p AppDir/usr/bin
cp build/tangent build/tangent_trial AppDir/usr/bin/
# The program finds these beside itself (SDL_GetBasePath).
cp -r shaders AppDir/usr/bin/
mkdir AppDir/usr/bin/assets
cp -r assets/fonts assets/icons assets/icon.png assets/icon.svg AppDir/usr/bin/assets/
cp LICENSE packaging/NOTICE.txt AppDir/
# OpenCASCADE's licence travels with its libraries, as the LGPL asks.
mkdir -p AppDir/licenses/opencascade
find "$prefix" -maxdepth 4 \( -name 'LICENSE_LGPL_21.txt' -o -name 'OCCT_LGPL_EXCEPTION.txt' \) \
  -exec cp {} AppDir/licenses/opencascade/ \;
[ -n "$(ls AppDir/licenses/opencascade)" ] || { echo "OpenCASCADE's licence not found under $prefix"; exit 1; }

tools=$PWD/.packaging-tools && mkdir -p "$tools"
[ -x "$tools/linuxdeploy" ] || {
  wget -q -O "$tools/linuxdeploy" \
    https://github.com/linuxdeploy/linuxdeploy/releases/download/continuous/linuxdeploy-x86_64.AppImage
  chmod +x "$tools/linuxdeploy"
}

# linuxdeploy takes only the standard icon sizes, and icon.png is 800px.
im=$(command -v magick || command -v convert)
"$im" assets/icon.png -resize 256x256 "$tools/tangent.png"

export APPIMAGE_EXTRACT_AND_RUN=1 ARCH=x86_64 VERSION=$version
export LD_LIBRARY_PATH=$prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export LDAI_OUTPUT=tangent-$version-linux-x86_64.AppImage
"$tools/linuxdeploy" --appdir AppDir \
  --executable AppDir/usr/bin/tangent --executable AppDir/usr/bin/tangent_trial \
  --desktop-file packaging/tangent.desktop \
  --icon-file "$tools/tangent.png" \
  --output appimage
ls -l "$LDAI_OUTPUT"
