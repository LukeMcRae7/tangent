#!/usr/bin/env bash
# ctest, with each failing test's last words raised as annotations: they show
# on the run's summary page, where the full log needs a signed-in viewer.
set -uo pipefail
ctest --test-dir build --output-on-failure 2>&1 | tee test.log
rc=${PIPESTATUS[0]}
if [ "$rc" -ne 0 ] && [ -f build/Testing/Temporary/LastTestsFailed.log ]; then
  while IFS=: read -r _ name; do
    name=${name%$'\r'}   # native ctest on Windows writes CRLF
    tail=$(ctest --test-dir build -R "^${name}\$" --output-on-failure 2>&1 |
           grep -v "^Test project\|^ *Start\|tests passed\|Total Test\|^$" | tail -14)
    tail=${tail//%/%25}; tail=${tail//$'\r'/}; tail=${tail//$'\n'/%0A}
    echo "::error title=test ${name}::${tail}"
  done < build/Testing/Temporary/LastTestsFailed.log
fi
exit "$rc"
