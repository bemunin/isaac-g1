---
name: oc-sonic-cmd
description: Command the Unitree G1 with SONIC in a running warehouse over ROS 2. Use when asked to move, walk, run, turn, squat, kneel, crawl or stop the G1, to send a Locomotion command, to pick a SONIC mode, or to explain SONIC modes or the planner's inputs and ranges.
---

# oc-sonic-cmd

Send Locomotion commands to the G1 in a running warehouse with `pixi run ros2 topic pub`, from the repo root. Show the
user every command you run, so they can re-run it.

## 1. Check the sim is ready

Run `pixi run ros2 topic list`. It must include `/clock`, `/cmd_vel`, `/sonic/mode` and `/sonic/pelvis_height`; the
`/sonic` topics appear only after Play.

If any is missing, tell the user to start the warehouse with `pixi run warehouse -a -r` (and that `pixi run dds` must
match the sim's DDS), then stop. The user launches the sim.

## 2. Send commands

Send mode and pelvis height first, then velocity. `-w 1` waits for the sim's subscriber.
A non-zero exit from a pub call means the sim's subscriber is missing: treat it as "sim not ready" (§1).

```sh
pixi run ros2 topic pub --once -w 1 --max-wait-time-secs 5 /sonic/mode std_msgs/msg/Int32 "{data: 2}"
pixi run ros2 topic pub --once -w 1 --max-wait-time-secs 5 /sonic/pelvis_height std_msgs/msg/Float32 "{data: -1.0}"
pixi run ros2 topic pub --once -w 1 --max-wait-time-secs 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}"
```

Stop (idle, height -1, zero twist):

```sh
pixi run ros2 topic pub --once -w 1 --max-wait-time-secs 5 /sonic/mode std_msgs/msg/Int32 "{data: 0}"
pixi run ros2 topic pub --once -w 1 --max-wait-time-secs 5 /sonic/pelvis_height std_msgs/msg/Float32 "{data: -1.0}"
pixi run ros2 topic pub --once -w 1 --max-wait-time-secs 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

- **Open-ended request** ("start walking"): send the command and stop there. It holds until replaced; tell the user the
  robot keeps going and give them the stop block.
- **Bounded request** (a distance, an angle, "for 3 seconds"): the duration is `distance / speed` or `angle / wz`, in
  sim seconds. Send the command, run `.agents/skills/oc-sonic-cmd/scripts/sim_sleep.sh <seconds>`, then send the stop
  block. Send the stop block even when `sim_sleep.sh` fails, and report its exit code: 0 waited, 1 sim time stalled
  (paused sim), 2 no `/clock`. Tell the user the result is approximate because tracking is not exact.
- Keep speeds, turn rates and heights inside the ranges in `GUIDE.md` section 2 and 3.

## 3. Commanding each mode family

Mode ids in brackets. Set `pelvis_height` only for squat, kneel, lying and crawls; send -1 elsewhere. Return to a stand
with the stop block.

| Family (ids) | Command | Example |
|---|---|---|
| Standing locomotion (0-3) | `linear.x`/`linear.y` in the mode's speed range, turn with `angular.z`. | run: mode 3, `linear.x: 2.0` |
| Turn in place (0) | Mode 0 with `angular.z`; duration is angle / `wz`. Static modes replan when facing changes. | mode 0, `angular.z: 0.5` for 3.1 s turns 90 degrees left |
| Postures (4-6) | Static, velocity ignored, `angular.z` still turns. Height is required: squat 0.4-0.8, kneels 0.2-0.4. | squat: mode 4, height 0.5 |
| Lying (7) | Static, height required (0.2-0.8). Experimental. | mode 7, height 0.3 |
| Crawls (8, 14) | Moving, low: hand_crawl 0.4-1.0 m/s, elbow_crawl 0.7-1.0 m/s, height 0.2-0.8. Elbow crawl overheats the real robot. | hand_crawl: mode 8, `linear.x: 0.6`, height 0.4 |
| Boxing (9-13, 15-16) | boxing_idle is a static stance; the rest move at 0.7-1.5 m/s. Height -1. | boxing_walk: mode 10, `linear.x: 1.0` |
| Styled walks (17-26) | Moving at the clip's own speed; any positive `linear.x` sets the direction. happy_dance_walk walks forward only. Height -1. | zombie_walk: mode 24, `linear.x: 1.0` |

A moving mode with zero velocity may keep moving here. To stand still, send the stop block or a static mode.

Modes 7 and 9-26 are unverified in Isaac Sim; tell the user to watch the robot.

## 4. Guide

`GUIDE.md` (same folder) holds the mode table (ids, names, speed and pelvis-height ranges), the map from ROS 2 topics to
planner inputs, turn-rate advice and timing.

- When the user asks about modes, planner inputs, ranges or how commands map to the planner, show them `GUIDE.md` as a
  file with SendUserFile, then answer from it.
- When you choose a mode, speed, turn rate or height yourself, read `GUIDE.md` first.
