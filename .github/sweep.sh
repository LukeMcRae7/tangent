#!/usr/bin/env bash
# Every demo mode, headless. Each drives a real gesture and must render clean.
set -uo pipefail
cd "$(dirname "$0")/.."

modes=(
  "--shell-fillet-demo" "--fillet-pair-demo 1.0" "--face-stress" "--fillet-open"
  "--print-demo 1" "--print-demo 2" "--print-demo 3"
  "--snap-demo 1" "--profile-demo 1"
  "--face-demo 5" "--face-demo 7" "--face-demo 8" "--face-demo 9"
  "--face-demo 10" "--face-demo 11" "--face-demo 12"
  "--preview-check 1" "--preview-check 2" "--preview-check 3"
  "--preview-check 4" "--preview-check 5" "--preview-check 6"
  "--pattern-demo 1" "--pattern-demo 2" "--pattern-demo 3" "--pattern-demo 4"
  "--pattern-demo 5" "--pattern-demo 6" "--pattern-demo 7"
  "--step-demo /tmp/tg_sweep.step"
)

fail=0
for m in "${modes[@]}"; do
  out=$(timeout 200 ./build/tangent $m --smoke-test 2>&1)
  if echo "$out" | grep -q "rendered cleanly"; then
    printf '  ok    %s\n' "$m"
  else
    printf '  FAIL  %s\n' "$m"
    echo "$out" | tail -20 | sed 's/^/        /'
    fail=1
  fi
done

# The preview must agree with what gets committed. A gesture that shows one
# thing and builds another is the worst kind of bug in a modeller, so the
# comparison is a build failure rather than a note.
for m in 1 2 3 4 5; do
  # Captured rather than piped into grep: `grep -q` exits on its first match and
  # SIGPIPEs the program upstream, which under `pipefail` reads as the pipeline
  # failing -- so every check reported a mismatch that had not happened.
  out=$(timeout 200 ./build/tangent --preview-check $m --smoke-test 30 2>&1)
  if ! echo "$out" | grep -q "SAME"; then
    printf '  FAIL  preview-check %s: the preview did not match the commit\n' "$m"
    fail=1
  fi
done

# A large mesh has to get all the way to being useful: reduced to fit, turned
# into a solid, bored, and split -- each step checked for what it made, not
# only for running. The plate is generated, so this needs no file on disk.
out=$(timeout 300 ./build/tangent --reduce-demo :plate --smoke-test 5 2>&1)
if ! echo "$out" | grep -q "\[reduce-demo\] ready=1 ok=1"; then
  printf '  FAIL  reduce-demo: the panel did not come back ready\n'
  echo "$out" | grep "reduce-demo" | sed 's/^/        /'
  fail=1
else
  printf '  ok    --reduce-demo :plate\n'
fi
out=$(timeout 300 ./build/tangent --mesh-bench :plate --smoke-test 5 2>&1)
if echo "$out" | grep -q "reduce ok=1.*within=1" &&
   echo "$out" | grep -q "convert ok=1.*closed=1" &&
   echo "$out" | grep -q "boolean ok=1 closed=1" &&
   echo "$out" | grep -q "split ok,.*closed=1/1"; then
  printf '  ok    --mesh-bench :plate\n'
else
  printf '  FAIL  mesh-bench: reduce, convert, boolean or split did not hold\n'
  echo "$out" | grep "mesh-bench" | sed 's/^/        /'
  fail=1
fi

[ $fail -eq 0 ] && echo "sweep clean" || echo "sweep FAILED"
exit $fail
