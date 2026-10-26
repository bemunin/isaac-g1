---
name: oc-sonic-cmd
description: Command the G1 SONIC whole-body controller (oc.control.g1_sonic) over ROS 2 or from Python in Isaac Sim. Use when switching planner modes (SonicMode), moving or turning the G1, or setting its pelvis height.
---

# oc-sonic-cmd

Every command picks a planner **mode** (an Id from [GUIDE.md](GUIDE.md), which lists each mode's group, speed
and height limits), a velocity and a pelvis height. A command holds until the next one replaces it. ROS 2 and
Python feed the same planner, so the latest command wins.

## ROS 2 command

The controller listens on these topics when it is built with `ros=True` (the default):

| Topic | Message | Fields | Planner input |
|---|---|---|---|
| `/sonic/mode` | `std_msgs/msg/Int32` | `data`: mode Id | `mode` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `linear.x`, `linear.y`: forward, left velocity [m/s]; `angular.z`: turn rate [rad/s] | `speed`, `movement_direction`, `facing_direction` |
| `/sonic/pelvis_height` | `std_msgs/msg/Float32` | `data`: pelvis height [m], `-1` = standing modes only (never in a ground pose) | `pelvis_height` |

Each topic sets its inputs independently, and the others keep their last values.

### Change motion mode

Publish a mode Id from [GUIDE.md](GUIDE.md) to `/sonic/mode`, e.g. `2` WALK or `4` SQUAT. To switch to WALK:
```sh
# reset velocity to zero
pixi run ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{}"

# change mode, then reset pelvis height
pixi run ros2 topic pub --once /sonic/mode std_msgs/msg/Int32 "{data: 2}"
pixi run ros2 topic pub --once /sonic/pelvis_height std_msgs/msg/Float32 "{data: -1.0}"
```

Velocity and pelvis height carry over to the new mode, so reset them: a zero `/cmd_vel` and, after the mode,
`pelvis_height` `-1`. To leave a ground pose for a standing mode, use [Get up](#get-up) instead of switching
directly. For a ground pose, publish its default pelvis height from GUIDE.md with the mode instead:

```sh
pixi run ros2 topic pub --once /sonic/pelvis_height std_msgs/msg/Float32 "{data: 0.4}"
pixi run ros2 topic pub --once /sonic/mode std_msgs/msg/Int32 "{data: 8}"  # HAND_CRAWL
```

### Control motion direction

`/cmd_vel` steers movable modes in the robot frame: `linear.x` forward/backward, `linear.y` left/right,
`angular.z` turn. Keep the speed within the mode's range in [GUIDE.md](GUIDE.md).

```sh
# forward at 1.0 m/s
pixi run ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"

# step left at 0.4 m/s
pixi run ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.4}}"

# forward while turning left
pixi run ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.3}}"

# stop
pixi run ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{}"
```

`/cmd_vel` publishes at 10 Hz until the user says stop. Keep exactly one `/cmd_vel` publisher running, in the
background, and manage it yourself: before starting a new one, and when the user says stop, kill the running one,
then publish the next command (a zero Twist for stop).

```sh
pkill -f "topic pub.*/cmd_vel"  # end the running /cmd_vel publisher
```

### Rotate (set facing direction)

`facing_direction` has no topic of its own; `angular.z` turns it. To turn by an angle, publish a rate for
angle ÷ rate seconds, then stop. Static modes turn in place.

```sh
# turn 90° left: 0.5 rad/s for 3.14 s
timeout 3.14 pixi run ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
pixi run ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{}"
```

The angle is approximate: the turn runs on sim time, and each `ros2` call adds start-up delay.

### Set pelvis height

Ground poses (Set 1 and LYING_FACE_DOWN) take 0.2-0.8 m; `-1` is for standing modes only; in a ground pose it breaks the plan.

```sh
pixi run ros2 topic pub --once /sonic/pelvis_height std_msgs/msg/Float32 "{data: 0.5}"   # lower to 0.5 m
pixi run ros2 topic pub --once /sonic/pelvis_height std_msgs/msg/Float32 "{data: 0.4}"   # back to 0.4 m
```

## Isaac Sim API command

Use this from the script or extension that runs the physics loop:

```python
from oc.control.g1_sonic import SonicController, SonicMode

controller = SonicController("/World/G1", ros=False)  # ros=True (default) also listens on the ROS 2 topics
controller.forward(dt)  # every physics step; the first call sets up, then the policy runs at 50 Hz
controller.reset()      # on Stop: back to IDLE, the next forward() restarts from the measured pose
controller.close()      # on shutdown
```

`command(mode, vx=0.0, vy=0.0, wz=0.0, pelvis_height=-1.0)` sets every input in one call:

| Argument | Meaning | Planner input |
|---|---|---|
| `mode` | `SonicMode` or an Id 0-26 | `mode` |
| `vx`, `vy` | forward, left velocity [m/s], robot frame | `speed`, `movement_direction` |
| `wz` | turn rate [rad/s] | `facing_direction` |
| `pelvis_height` | pelvis height [m], `-1` = standing modes only (never in a ground pose) | `pelvis_height` |

Arguments you leave out go back to their defaults, so each call is a complete command. A bad mode raises
`ValueError` and keeps the previous command.

### Change motion mode

```python
controller.command(SonicMode.WALK)  # WALK, standing still at the default height
controller.command(SonicMode.HAND_CRAWL, pelvis_height=0.4)  # a ground pose takes its height with the mode
```

Unlike ROS 2, nothing carries over from the previous command.

### Control motion direction

```python
controller.command(SonicMode.WALK, vx=1.0)          # forward at 1.0 m/s
controller.command(SonicMode.SLOW_WALK, vy=0.4)     # step left at 0.4 m/s
controller.command(SonicMode.WALK, vx=1.0, wz=0.3)  # forward while turning left
controller.command(SonicMode.WALK)                  # stop
```

### Rotate (set facing direction)

`wz` turns `facing_direction`; the angle is `wz` × sim time. Count sim time from the `forward(dt)` calls for an
exact turn.

```python
controller.command(SonicMode.IDLE, wz=0.5)  # turn left at 0.5 rad/s
# ... after 3.14 s of sim time (90°):
controller.command(SonicMode.IDLE)
```

### Set pelvis height

```python
controller.command(SonicMode.SQUAT, pelvis_height=0.5)  # lower to 0.5 m
controller.command(SonicMode.SQUAT, pelvis_height=0.4)  # back to the default 0.4 m
```

### Create python script

Run a motion chain inside the running warehouse from VS Code. Start the warehouse with the VS Code server on
(SONIC is on by default):

```sh
pixi run warehouse --vscode
```

The warehouse owns the controller and steps it; the script only sends commands. `asyncio.sleep` hands control
back to Isaac Sim between steps:

```python
import asyncio

from oc.control.g1_sonic import SonicMode
from oc.scenario.warehouse import get_sonic_controller

CHAIN = [  # (mode, command arguments, seconds)
    (SonicMode.WALK, {"vx": 1.0}, 5),
    (SonicMode.KNEEL_TWO_LEGS, {"pelvis_height": 0.4}, 5),
    (SonicMode.HAND_CRAWL, {"vx": 0.6, "pelvis_height": 0.4}, 5),
    (SonicMode.HAND_CRAWL, {"pelvis_height": 0.4}, 0.5),  # get up in stages (see Get up)
    (SonicMode.KNEEL_TWO_LEGS, {"pelvis_height": 0.4}, 0.5),
    (SonicMode.IDLE, {}, 5),
]


async def run_chain():
    controller = get_sonic_controller()
    if controller is None:
        print("SONIC is off in this warehouse")
        return
    for mode, arguments, seconds in CHAIN:
        controller.command(mode, **arguments)
        await asyncio.sleep(seconds)


asyncio.ensure_future(run_chain())
```

In the Isaac Sim VS Code Edition, run the file with **Run Remotely** (it connects to `127.0.0.1:8226`). Durations
are wall-clock seconds, so a slow sim covers less ground per step.

## Lie down and get up

SONIC has no get-up or fall-recovery mode. Get up in stages, like NVIDIA's gamepad crawl exit: hand crawl → kneel on two legs → IDLE, each held about 0.5 s. Going straight from a crawl or LYING_FACE_DOWN to a standing mode asks for a 0.6-1.5 s stand-up the robot often can't track, and falls.

### Lie face down

The robot can't spawn lying; SONIC lays itself down from standing: KNEEL_TWO_LEGS 0.4 → HAND_CRAWL 0.4 → LYING_FACE_DOWN 0.2, about 0.5 s each. Publish the height before the mode.

```sh
# hold still
pkill -f "topic pub.*/cmd_vel"
pixi run ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{}" &

# 1. kneel on two legs
pixi run ros2 topic pub --once /sonic/pelvis_height std_msgs/msg/Float32 "{data: 0.4}"
pixi run ros2 topic pub --once /sonic/mode std_msgs/msg/Int32 "{data: 5}"  # KNEEL_TWO_LEGS
sleep 0.5

# 2. hands down
pixi run ros2 topic pub --once /sonic/pelvis_height std_msgs/msg/Float32 "{data: 0.4}"
pixi run ros2 topic pub --once /sonic/mode std_msgs/msg/Int32 "{data: 8}"  # HAND_CRAWL
sleep 0.5

# 3. lie down
pixi run ros2 topic pub --once /sonic/pelvis_height std_msgs/msg/Float32 "{data: 0.2}"
pixi run ros2 topic pub --once /sonic/mode std_msgs/msg/Int32 "{data: 7}"  # LYING_FACE_DOWN
```

```python
controller.command(SonicMode.KNEEL_TWO_LEGS, pelvis_height=0.4)
# ... 0.5 s later:
controller.command(SonicMode.HAND_CRAWL, pelvis_height=0.4)
# ... 0.5 s later:
controller.command(SonicMode.LYING_FACE_DOWN, pelvis_height=0.2)
```

### Get up

From LYING_FACE_DOWN, HAND_CRAWL or any ground pose, with speed 0 throughout (keep the zero `/cmd_vel` publisher running):

| Step | Mode | Pelvis height | Hold |
|---|---|---|---|
| 1 | 8 HAND_CRAWL | 0.4 | 0.5 s |
| 2 | 5 KNEEL_TWO_LEGS | 0.4 | 0.5 s |
| 3 | 0 IDLE, then height -1 | -1 | at least 2 s |

```sh
# hold still
pkill -f "topic pub.*/cmd_vel"
pixi run ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{}" &

# 1. hand crawl at 0.4 m
pixi run ros2 topic pub --once /sonic/pelvis_height std_msgs/msg/Float32 "{data: 0.4}"
pixi run ros2 topic pub --once /sonic/mode std_msgs/msg/Int32 "{data: 8}"  # HAND_CRAWL
sleep 0.5

# 2. kneel on two legs (height stays 0.4)
pixi run ros2 topic pub --once /sonic/mode std_msgs/msg/Int32 "{data: 5}"  # KNEEL_TWO_LEGS
sleep 0.5

# 3. stand: IDLE first, then the standing height
pixi run ros2 topic pub --once /sonic/mode std_msgs/msg/Int32 "{data: 0}"  # IDLE
pixi run ros2 topic pub --once /sonic/pelvis_height std_msgs/msg/Float32 "{data: -1.0}"
```

```python
controller.command(SonicMode.HAND_CRAWL, pelvis_height=0.4)
# ... 0.5 s later:
controller.command(SonicMode.KNEEL_TWO_LEGS, pelvis_height=0.4)
# ... 0.5 s later:
controller.command(SonicMode.IDLE)
```

- Publish mode 0 before height -1; -1 in KNEEL_TWO_LEGS or LYING_FACE_DOWN plans the pelvis metres up. Skip step 1 when already in HAND_CRAWL; start at step 2 from SQUAT or a kneel.
- Optional, gentler but not NVIDIA's path: between steps 2 and 3, SQUAT (4) at 0.5 then 0.7 m, 0.5 s each.
- Holds are wall-clock; on a sim slower than real time, lengthen them, or wait until the measured pelvis height and tilt settle.
- Success: pelvis above 0.7 m and upright for 5 s after IDLE.
- Works from a deliberate ground pose only; there is no recovery from a fall, and LYING_FACE_DOWN is disabled upstream and unvalidated.

## Notes

### Clamp inputs

Check each requested value against the mode's row in [GUIDE.md](GUIDE.md) and clamp it into range:

- **Mode**: an Id 0-26; map a named motion ("crawl", "box") to its Id.
- **Speed**: scale `(vx, vy)` into the mode's Speed range, keeping the direction. A `fixed` speed can't be set:
  keep the request and say so.
- **Static modes** (Movable `no`): zero `vx` and `vy`; only `wz` applies.
- **HAPPY_DANCE_WALK**: forward only, `vy = 0` and `vx >= 0`.
- **Pelvis height**: 0.2-0.8 m for ground poses, sent with the mode (default from GUIDE.md when the user
  gives none); `-1` for every other mode, published after the mode when leaving a ground pose.

Tell the user each value you changed, e.g. "WALK speed 2.0 → 1.5 m/s".

When `SonicMode` changes (`controller/sonic_controller.py`), update GUIDE.md to match.
