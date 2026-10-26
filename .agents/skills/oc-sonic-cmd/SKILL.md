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
| `/sonic/pelvis_height` | `std_msgs/msg/Float32` | `data`: pelvis height [m], `-1` = the mode's default | `pelvis_height` |

Each topic sets its inputs independently, and the others keep their last values.

### Change motion mode

Publish a mode Id from [GUIDE.md](GUIDE.md) to `/sonic/mode`, e.g. `2` WALK or `4` SQUAT. To switch to WALK:
```sh
# reset velocity and pelvis height to defaults
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"                        
ros2 topic pub --once /sonic/pelvis_height std_msgs/msg/Float32 "{data: -1.0}"   

# change mode
ros2 topic pub --once /sonic/mode std_msgs/msg/Int32 "{data: 2}"
```

Velocity and pelvis height carry over to the new mode, so reset both first: a zero `/cmd_vel` and
`pelvis_height` `-1`. For a ground pose, publish the target height instead of `-1`.

### Control motion direction

`/cmd_vel` steers movable modes in the robot frame: `linear.x` forward/backward, `linear.y` left/right,
`angular.z` turn. Keep the speed within the mode's range in [GUIDE.md](GUIDE.md).

```sh
# forward at 1.0 m/s
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"

# step left at 0.4 m/s
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.4}}"

# forward while turning left
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.3}}"

# stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

The last command holds after Ctrl+C.

### Rotate (set facing direction)

`facing_direction` has no topic of its own; `angular.z` turns it. To turn by an angle, publish a rate for
angle ÷ rate seconds, then stop. Static modes turn in place.

```sh
# turn 90° left: 0.5 rad/s for 3.14 s
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
sleep 3.14
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

The angle is approximate: the turn runs on sim time, and each `ros2` call adds start-up delay.

### Set pelvis height

Ground poses (Set 1 and LYING_FACE_DOWN) take 0.2-0.8 m; `-1` restores the mode's default.

```sh
ros2 topic pub --once /sonic/pelvis_height std_msgs/msg/Float32 "{data: 0.5}"   # lower to 0.5 m
ros2 topic pub --once /sonic/pelvis_height std_msgs/msg/Float32 "{data: -1.0}"  # default height
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
| `pelvis_height` | pelvis height [m], `-1` = the mode's default | `pelvis_height` |

Arguments you leave out go back to their defaults, so each call is a complete command. A bad mode raises
`ValueError` and keeps the previous command.

### Change motion mode

```python
controller.command(SonicMode.WALK)  # WALK, standing still at the default height
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
controller.command(SonicMode.SQUAT)                     # default height
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
    (SonicMode.KNEEL_TWO_LEGS, {"pelvis_height": 0.5}, 5),
    (SonicMode.HAND_CRAWL, {"vx": 0.6, "pelvis_height": 0.4}, 5),
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

## Notes

Before sending a command, check every requested value against the target mode's row in [GUIDE.md](GUIDE.md)
and clamp it into range:

- **Mode**: an Id 0-26 from GUIDE.md; map a named motion ("crawl", "box") to its Id.
- **Speed** (the length of `(vx, vy)`): scale `(vx, vy)` into the mode's Speed range, keeping the direction.
  For a `fixed` speed, keep the request and tell the user the speed is not adjustable in that mode.
- **Static modes** (Movable `no`): send zero `vx` and `vy`; only `wz` (turning) applies.
- **HAPPY_DANCE_WALK**: forward only, so send `vy = 0` and `vx >= 0`.
- **Pelvis height**: clamp ground poses to 0.2-0.8 m; send `-1` for every other mode.

Tell the user each value you changed, e.g. "WALK speed 2.0 → 1.5 m/s".

When `SonicMode` changes (`controller/sonic_controller.py`), update GUIDE.md to match.
