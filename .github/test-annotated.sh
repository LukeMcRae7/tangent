#!/usr/bin/env bash
# ctest, with each failing test and the lines it printed about the failure
# raised as annotations: see build-annotated.sh.
set -uo pipefail
ctest --test-dir build --output-on-failure 2>&1 | tee test.log
rc=${PIPESTATUS[0]}
if [ "$rc" -ne 0 ]; then
  grep -E "\*\*\*|Failed|FAIL|Exception|SEGFAULT|Timeout|assert|expected|Error|error" test.log | head -45 |
    while IFS= read -r line; do echo "::error::${line//%/%25}"; done
fi
exit "$rc"
