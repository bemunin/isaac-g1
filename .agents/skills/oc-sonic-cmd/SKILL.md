---
name: oc-sonic-cmd
description: Command the Unitree G1 with SONIC in a running warehouse, over ROS 2 or Python. Use when asked to move, walk, run, turn, squat, kneel, crawl or stop the G1, to send a Locomotion command, or to pick a SONIC mode.
---

# oc-sonic-cmd

Send Locomotion commands to the G1 in a running warehouse with `scripts/sonic_cmd.py`. The agent runs the CLI and
shows the user each command, so they can re-run it.

## 1. Pick the Command source

The Command source is where Locomotion commands come from: ROS 2 or a Python caller in Isaac Sim's process.

- If the request names one, or this conversation already settled it, use that.
- Otherwise ask with AskUserQuestion: ROS 2, or Python into the running sim. Reuse the answer for later commands.

| Source | `--source` | Warehouse launch |
|---|---|---|
| ROS 2 | `ros` | `pixi run warehouse -a -r` (the `pixi run dds` choice must match) |
| Python | `python` | `pixi run warehouse -a --vscode` (python_server on 127.0.0.1:8226) |

With both on, the sources race and the latest command wins.

## 2. Check the sim is ready

Run the CLI (§3). Exit 2 means the sim is not ready: tell the user the launch command it printed and stop. The
user launches the warehouse; the agent does not.

## 3. Send commands

```sh
pixi run python .agents/skills/oc-sonic-cmd/scripts/sonic_cmd.py --source ros|python [flags]
```

Run `--help` for the flags. Exit codes: 0 ok, 1 command failed, 2 sim not ready.

```sh
# walk forward 2 m: walk at 1.0 m/s for 2 s
... --source python --mode walk --vx 1.0 --duration 2
# turn left 90 degrees in place: slow walk at 0.5 rad/s for pi/2 / 0.5 = 3.1 s
... --source python --mode slow_walk --wz 0.5 --duration 3.1
# squat to 0.5 m, hold 3 s, stand
... --source ros --mode squat --pelvis-height 0.5 --duration 3
# stop
... --source ros --stop
```

- **Bounded request** (a distance, an angle, "for 3 seconds"): pass `--duration` = `distance / speed` or
  `angle / wz`, in sim seconds. The CLI ends in idle with zero velocity. Keep speeds inside the mode's range in the
  table, and tell the user the result is approximate because tracking is not exact.
- **Open-ended request** ("start walking"): omit `--duration`. The command holds until replaced; tell the user the
  robot keeps going and that `--stop` stops it.
- If the sim is paused the CLI gives up waiting after 5 s, sends idle and exits 1.

## 4. SONIC modes

SONIC's kinematic planner has 27 modes, ids 0–26. The weights from `scripts/sonic_download.py` are NVIDIA's V2
planner, and every id produces valid planner motion. `--mode` takes the CLI name or the id.

- **Moving** modes are driven by velocity and facing.
- **Static** modes hold in place and replan only when mode, facing or height changes.
- **Height** is the target pelvis height [m]; "–" means the mode ignores it.
- **Speed** "clip" means the planner uses the motion clip's own speed (NVIDIA sends `target_vel = -1`); a positive
  command is still tracked.

| id | CLI name | name | kind | speed (m/s) | height (m) |
|---|---|---|---|---|---|
| 0 | idle | idle | static | – | – |
| 1 | slow_walk | slow walk | moving | 0.2–0.8 | – |
| 2 | walk | walk | moving | 0.8–1.5 | – |
| 3 | run | run | moving | 1.5–3.0 | – |
| 4 | squat | squat | static | – | 0.4–0.8 |
| 5 | kneel_two_legs | kneel, two legs | static | – | 0.2–0.4 |
| 6 | kneel_one_leg | kneel, one leg | static | – | 0.2–0.4 |
| 7 | lying_face_down | lying face down | static | – | required |
| 8 | hand_crawl | hand crawl | moving | 0.4–1.0 | 0.2–0.8 |
| 9 | boxing_idle | boxing, idle stance | static | – | – |
| 10 | boxing_walk | boxing, walk | moving | 0.7–1.5 | – |
| 11 | boxing_left_jab | boxing, left jab | moving | 0.7–1.5 | – |
| 12 | boxing_right_jab | boxing, right jab | moving | 0.7–1.5 | – |
| 13 | boxing_random_punches | boxing, random punches | moving | 0.7–1.5 | – |
| 14 | elbow_crawl | elbow crawl (overheats on the real robot) | moving | 0.7–1.0 | 0.2–0.8 |
| 15 | boxing_left_hook | boxing, left hook | moving | 0.7–1.5 | – |
| 16 | boxing_right_hook | boxing, right hook | moving | 0.7–1.5 | – |
| 17 | happy_walk | happy walk (`FORWARD_JUMP` in NVIDIA's enum; it walks, it does not jump) | moving | clip | – |
| 18 | stealth_walk | stealth walk | moving | clip | – |
| 19 | injured_walk | injured walk | moving | clip | – |
| 20 | careful_walk | careful walk (`LEDGE_WALKING`) | moving | clip | – |
| 21 | object_carrying | object carrying | moving | clip | – |
| 22 | crouch_walk | crouch walk (`STEALTH_WALK_2`) | moving | clip | – |
| 23 | happy_dance_walk | happy dance walk (forward only) | moving | clip | – |
| 24 | zombie_walk | zombie walk | moving | clip | – |
| 25 | point_walk | point walk (`GUN_WALK`) | moving | clip | – |
| 26 | scared_walk | scared walk | moving | clip | – |

Ids 27 and above are not modes. The planner accepts them silently and NVIDIA's C++ replaces them with 0, so validate
ids to 0–26.

Unverified: whether the SONIC tracking policy follows modes 7 and 9–26 in Isaac Sim (planner output being sane does
not prove it), and whether the boxing and styled walks can side-step or walk backwards. Watch the robot when using them.

Sources: NVIDIA GR00T-WholeBodyControl `gear_sonic_deploy/.../include/localmotion_kplanner.hpp:78-115` (enum, static
modes), `docs/source/references/planner_onnx.md:105-150` (V2 names, heights), `input_interface/keyboard_handler.hpp:260-294`
(speed ranges).

## 5. Doing it yourself

Python, in VS Code ("Isaac Sim: Run", warehouse with `--vscode`). The CLI prints this snippet before it sends it:

```python
from oc.scenario.warehouse import get_sonic_controller

get_sonic_controller().command(2, vx=1.0)  # mode id 0-26; holds until replaced
get_sonic_controller().command(0)          # idle, zero velocity
```

ROS 2 (warehouse with `-r`), mode and pelvis height first, then velocity:

```sh
pixi run ros2 topic pub --once /sonic/mode std_msgs/msg/Int32 "{data: 2}"
pixi run ros2 topic pub --once /sonic/pelvis_height std_msgs/msg/Float32 "{data: -1.0}"
pixi run ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}"
```
