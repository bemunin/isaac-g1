#!/usr/bin/env bash
# Wait <seconds> of SIM time, read from /clock with the ros2 CLI. Run from the repo root.
# Usage: sim_sleep.sh <seconds>
# Exit 0: waited. 1: sim time stopped advancing for 5 s of wall time (paused sim). 2: no /clock message in 5 s.
set -euo pipefail

[ $# -eq 1 ] || { echo "usage: sim_sleep.sh <seconds>" >&2; exit 2; }
target=$1
stall_s=5  # wall seconds to wait for a first message, and for sim time to advance

# The echo runs in its own process group so the trap can stop pixi and the ros2 process under it.
coproc CLOCK { PYTHONUNBUFFERED=1 exec setsid pixi run ros2 topic echo /clock rosgraph_msgs/msg/Clock --csv --field clock 2>/dev/null; }
trap 'kill -- -"$CLOCK_PID" 2>/dev/null || true' EXIT

start=""
last=""
last_move=$SECONDS
while :; do
    if IFS=, read -r -t 1 sec nsec <&"${CLOCK[0]}" && [ -n "${nsec:-}" ]; then
        now=$(awk -v s="$sec" -v n="$nsec" 'BEGIN { printf "%.6f", s + n / 1e9 }')
        [ -n "$start" ] || start=$now
        if [ "$now" != "$last" ]; then
            last=$now
            last_move=$SECONDS
        fi
        if awk -v a="$now" -v b="$start" -v t="$target" 'BEGIN { exit !(a - b >= t) }'; then
            exit 0
        fi
    fi
    if (( SECONDS - last_move >= stall_s )); then
        if [ -z "$start" ]; then
            echo "no /clock: start the warehouse with ROS on: pixi run warehouse -a -r" >&2
            exit 2
        fi
        echo "sim time stalled for ${stall_s} s (sim paused?)" >&2
        exit 1
    fi
done
