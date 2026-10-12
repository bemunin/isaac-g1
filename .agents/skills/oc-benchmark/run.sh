#!/usr/bin/env bash
# Run the 6 warehouse benchmark scenarios one after another (never in parallel: they share the GPU), all into
# one session folder research/benchmark_output/<YYYYMMDD>_<HHMMSS>_warehouse/<scenario>/.
# Usage: run.sh [SECONDS] [CONTROLLER] [SESSION_DIR [SCENARIO ...]]
#   defaults: 60 sonic, a new session; DDS is the session's (`pixi run dds`).
#   SESSION_DIR + SCENARIO names reruns only those scenarios into an existing session.
SECONDS_=${1:-60}
CONTROLLER=${2:-sonic}
cd "$(dirname "$0")/../../.." || exit 1
SESSION=${3:-$PWD/research/benchmark_output/$(date +%Y%m%d_%H%M%S)_warehouse}
SESSION=$(mkdir -p "$SESSION" && cd "$SESSION" && pwd)
ONLY=("${@:4}")

DDS=$(pixi run dds 2>/dev/null | tail -1)
printf 'seconds=%s\ncontroller=%s\ndds=%s\n' "$SECONDS_" "$CONTROLLER" "$DDS" > "$SESSION/settings.txt"
echo "session=$SESSION"

run() {
  name=$1; shift
  if [ ${#ONLY[@]} -gt 0 ] && [[ ! " ${ONLY[*]} " == *" $name "* ]]; then return; fi
  d=$SESSION/$name
  rm -rf "$d" && mkdir -p "$d"
  pixi run warehouse --controller "$CONTROLLER" --benchmark "$SECONDS_" --benchmark-output "$d" "$@" > "$d/$name.log" 2>&1
  rc=$?
  echo "$name rc=$rc kpis=$(ls "$d"/kpis_warehouse_*.json 2>/dev/null | wc -l)"
}

run baseline1_newton_headless --physics newton --headless
run newton_ros_headless --physics newton --ros --headless
run newton_ros --physics newton --ros
run baseline2_physx_headless --physics physx --headless
run physx_ros_headless --physics physx --ros --headless
run physx_ros --physics physx --ros
