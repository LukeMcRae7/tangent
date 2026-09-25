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
  "--face-demo 15" "--face-demo 16" "--face-demo 17"
  "--preview-check 1" "--preview-check 2" "--preview-check 3"
  "--preview-check 4" "--preview-check 5" "--preview-check 6"
  "--preview-check 7"
  "--revolve-demo 1" "--revolve-demo 2" "--revolve-demo 3" "--revolve-demo 4" "--revolve-demo 5"
  "--sweep-demo 1" "--sweep-demo 2" "--sweep-demo 3" "--sweep-demo 4"
  "--loft-demo 1" "--loft-demo 2" "--loft-demo 3" "--loft-demo 4"
  "--pen-demo 1" "--pen-demo 2" "--project-demo 1" "--plane-demo 1"
  "--timeline-demo 1" "--timeline-demo 2"
  "--hole-demo 1" "--hole-demo 2" "--hole-demo 3"
  "--draft-demo 1" "--draft-demo 2" "--draft-demo 3"
  "--delete-face-demo 1" "--delete-face-demo 2" "--delete-face-demo 3"
  "--pattern-demo 1" "--pattern-demo 2" "--pattern-demo 3" "--pattern-demo 4"
  "--pattern-demo 5" "--pattern-demo 6" "--pattern-demo 7" "--pattern-demo 8"
  "--step-demo /tmp/tg_sweep.step"
  "--inset-demo 2" "--shell-demo 2" "--split-demo 3" "--offset-demo 1"
  "--thread-demo 1" "--thread-demo 2" "--thread-demo 3"
  "--split-demo 4" "--split-demo 5"
  "--perf-scene 1 --perf-size 6" "--perf-scene 2 --perf-size 6" "--perf-scene 3 --perf-size 6"
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
for m in 1 2 3 4 5 7; do
  # Captured rather than piped into grep: `grep -q` exits on its first match and
  # SIGPIPEs the program upstream, which under `pipefail` reads as the pipeline
  # failing -- so every check reported a mismatch that had not happened.
  out=$(timeout 200 ./build/tangent --preview-check $m --smoke-test 30 2>&1)
  if ! echo "$out" | grep -q "SAME"; then
    printf '  FAIL  preview-check %s: the preview did not match the commit\n' "$m"
    fail=1
  fi
done

# Revolve, sweep and loft have to make the volume arithmetic says -- Pappus for
# a turn, A times the centreline for a mitred bend, the frustum formula for a
# loft -- whether built from a sketch or from a box's own face and edge, and a
# sketch picked from has to end up inside the part rather than beside it.
for dm in "revolve 1" "revolve 2" "revolve 3" "revolve 4" "sweep 1" "sweep 2" "sweep 3" \
          "loft 1" "loft 2" "loft 3"; do
  set -- $dm
  out=$(timeout 200 ./build/tangent --$1-demo $2 --smoke-test 20 2>&1)
  line=$(echo "$out" | grep "\[$1-demo\]" | head -1)
  if ! echo "$line" | grep -q "agrees=1 tidy=1"; then
    printf '  FAIL  %s-demo %s: %s\n' "$1" "$2" "$line"
    fail=1
  else
    printf '  ok    --%s-demo %s\n' "$1" "$2"
  fi
done

# The pen's corners draw straight sides, so a triangle of them extruded has the
# prism's volume; and a sketch projected from a face goes round the face after
# the part is widened, not round where the face was; and a round rolled past a
# step put in under it finds its edge again and rounds all of it.
for dm in "pen 1" "project 1" "timeline 1"; do
  set -- $dm
  out=$(timeout 200 ./build/tangent --$1-demo $2 --smoke-test 20 2>&1)
  line=$(echo "$out" | grep "\[$1-demo\]" | head -1)
  if ! echo "$line" | grep -q "agrees=1"; then
    printf '  FAIL  %s-demo %s: %s\n' "$1" "$2" "$line"
    fail=1
  else
    printf '  ok    --%s-demo %s\n' "$1" "$2"
  fi
done

# A hole has to take away exactly the cylinder it says it does -- including
# after the panel changes its size, and after the face it was drilled into
# moves under it.
for m in 1 2 3; do
  out=$(timeout 200 ./build/tangent --hole-demo $m --smoke-test 20 2>&1)
  if echo "$out" | grep "\[hole-demo\]" | grep -q "agrees=0"; then
    printf '  FAIL  hole-demo %s\n' "$m"
    echo "$out" | grep "\[hole-demo\]" | sed 's/^/        /'
    fail=1
  else
    printf '  ok    --hole-demo %s\n' "$m"
  fi
done

# And a draft has to lean by exactly the angle it was asked for.
for m in 1 2 3; do
  out=$(timeout 200 ./build/tangent --draft-demo $m --smoke-test 20 2>&1)
  if echo "$out" | grep "\[draft-demo\]" | grep -q "agrees=0"; then
    printf '  FAIL  draft-demo %s\n' "$m"
    echo "$out" | grep "\[draft-demo\]" | sed 's/^/        /'
    fail=1
  else
    printf '  ok    --draft-demo %s\n' "$m"
  fi
done

# A curved face has no one direction to be swept along: pushing the band round
# a cylinder has to make a collar of exactly the ring between the two radii.
for m in 15 16 17; do
  out=$(timeout 200 ./build/tangent --face-demo $m --smoke-test 30 2>&1)
  if echo "$out" | grep "\[face-demo\]" | grep -q "agrees=0"; then
    printf '  FAIL  face-demo %s\n' "$m"
    echo "$out" | grep "\[face-demo\]" | sed 's/^/        /'
    fail=1
  else
    printf '  ok    --face-demo %s\n' "$m"
  fi
done

# Taking a face off has to put the body back exactly as it was before the
# feature was on it -- and refuse, rather than quietly do nothing, when the
# gap cannot be closed.
for m in 1 2 3; do
  out=$(timeout 200 ./build/tangent --delete-face-demo $m --smoke-test 30 2>&1)
  if echo "$out" | grep "\[delete-face-demo\]" | grep -q "agrees=0"; then
    printf '  FAIL  delete-face-demo %s\n' "$m"
    echo "$out" | grep "\[delete-face-demo\]" | sed 's/^/        /'
    fail=1
  else
    printf '  ok    --delete-face-demo %s\n' "$m"
  fi
done

# Pins across a split have to be exactly the cylinders they say they are: the
# halves gain a pin and lose a socket, or lose two sockets for a dowel.
for m in 4 5; do
  out=$(timeout 200 ./build/tangent --split-demo $m --smoke-test 30 2>&1)
  if echo "$out" | grep "\[split-demo\]" | grep -q "agrees=0"; then
    printf '  FAIL  split-demo %s\n' "$m"
    echo "$out" | grep "\[split-demo\]" | sed 's/^/        /'
    fail=1
  else
    printf '  ok    --split-demo %s\n' "$m"
  fi
done

# An offset has to be the size it says: a 20 mm cube grown by 1 is a 22 mm cube
# and not a 22 mm cube with a round on every edge.
out=$(timeout 200 ./build/tangent --offset-demo 1 --smoke-test 30 2>&1)
if echo "$out" | grep "\[offset-demo\]" | grep -q "agrees=0"; then
  printf '  FAIL  offset-demo\n'
  echo "$out" | grep "\[offset-demo\]" | sed 's/^/        /'
  fail=1
else
  printf '  ok    --offset-demo 1\n'
fi

# A thread has to come out the size it says: the crest and the root of what was
# cut, measured off the body, against the screw it was cut for.
for m in 1 2 3; do
  out=$(timeout 300 ./build/tangent --thread-demo $m --smoke-test 30 2>&1)
  if echo "$out" | grep "\[thread-demo\]" | grep -q "agrees=0"; then
    printf '  FAIL  thread-demo %s\n' "$m"
    echo "$out" | grep "\[thread-demo\]" | sed 's/^/        /'
    fail=1
  else
    printf '  ok    --thread-demo %s\n' "$m"
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

# Round All Edges goes through the fillet gesture, so it is checked for what
# it built -- all twelve edges of the startup cube at 2mm, to the volume -- and
# for refusing on a mesh with the way past it.
out=$(timeout 200 ./build/tangent --round-all-demo --smoke-test 5 2>&1)
if echo "$out" | grep -q "opened=1 edges=12 rounded=1 volume 8000.0 -> 7804.7 solid=1" &&
   echo "$out" | grep -q "on a mesh: opened=0 notice .*Convert to Solid"; then
  printf '  ok    --round-all-demo\n'
else
  printf '  FAIL  round-all-demo: the rounds, or the refusal on a mesh, were not right\n'
  echo "$out" | grep "round-all" | sed 's/^/        /'
  fail=1
fi

# A 3MF of a shelled, filleted part: written, one object, and closed -- the
# exporter checks every edge is used once each way and says when one is not.
rm -f /tmp/tg_sweep.3mf
out=$(timeout 200 ./build/tangent --shell-fillet-demo --export-3mf /tmp/tg_sweep.3mf --smoke-test 2 2>&1)
if echo "$out" | grep -q "Exported 1 object, .* triangles, to /tmp/tg_sweep.3mf" &&
   ! echo "$out" | grep -q "not closed" && [ -s /tmp/tg_sweep.3mf ]; then
  printf '  ok    --export-3mf\n'
else
  printf '  FAIL  export-3mf: not written, or not closed\n'
  echo "$out" | grep "\[app\]" | sed 's/^/        /'
  fail=1
fi

[ $fail -eq 0 ] && echo "sweep clean" || echo "sweep FAILED"
exit $fail
