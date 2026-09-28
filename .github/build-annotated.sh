#!/usr/bin/env bash
# cmake --build, with the compiler's errors raised as annotations: they show
# on the run's summary page, where the full log needs a signed-in viewer.
set -uo pipefail
cmake --build build -j"$(nproc)" -- -k 0 2>&1 | tee build.log
rc=${PIPESTATUS[0]}
if [ "$rc" -ne 0 ]; then
  grep -E "error:|error [A-Z]+[0-9]+|undefined reference|FAILED:" build.log | head -40 |
    while IFS= read -r line; do echo "::error::${line//%/%25}"; done
fi
exit "$rc"
