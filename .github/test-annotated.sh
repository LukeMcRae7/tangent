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
    # A crash prints nothing useful on its own; where it happened does.
    if command -v gdb >/dev/null; then
      bt=$( (cd build && gdb -q -batch -ex run -ex bt "./test_${name}") 2>&1 |
            grep -E "signal|^#[0-9]" | head -18)
      bt=${bt//%/%25}; bt=${bt//$'\r'/}; bt=${bt//$'\n'/%0A}
      echo "::error title=backtrace ${name}::${bt}"
    fi
  done < build/Testing/Temporary/LastTestsFailed.log
fi
exit "$rc"
