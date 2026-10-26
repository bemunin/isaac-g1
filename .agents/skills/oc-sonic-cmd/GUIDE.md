# SONIC command guide

## Planner modes

The SONIC kinematic planner turns five inputs into the reference motion the policy tracks:

- **`mode`**: the planner mode, an **Id** from the tables below (e.g. `2` = WALK).
- **`speed`** (m/s): how fast to travel; `-1` uses the mode's default speed.
- **`movement_direction`**: where to travel, a unit vector `[x, y, 0]` in the planner frame (+x = the robot's heading when the planner starts); `[0, 0, 0]` stands still.
- **`facing_direction`**: the heading the body turns to (e.g. `[0.707, 0.707, 0]` = 45°, `[-0.5, -0.866, 0]` = -120°).
- **`pelvis_height`** (m): pelvis height for ground poses; `-1` keeps the default standing height.

Modes are grouped like the upstream keyboard teleop sets. **Movable** modes take `speed` and `movement_direction`; **static** modes hold a pose and take `facing_direction` only (plus `pelvis_height` for ground poses). A changed input triggers a replan, so the motion blends into the new command.

### Set 0: Standing

| Id | Name | Movable | Speed (m/s) | Pelvis height (m) |
|---|---|---|---|---|
| 1 | SLOW_WALK | yes | 0.2-0.8 | fixed |
| 2 | WALK | yes | 0.8-1.5 | fixed |
| 3 | RUN | yes | 1.5-3.0 | fixed |
| 17 | HAPPY_WALK | yes | fixed | fixed |
| 18 | STEALTH_WALK | yes | fixed | fixed |
| 19 | INJURED_WALK | yes | fixed | fixed |

- `movement_direction`: any direction (forward, backward, sideways, diagonal); keep sideways steps near 0.4 m/s to avoid foot collisions.
- `facing_direction`: any heading.
- `speed`: adjustable in SLOW_WALK, WALK and RUN; the other walks use their default.
- `pelvis_height`: fixed.

### Set 1: Squat / ground

| Id | Name | Movable | Speed (m/s) | Pelvis height (m) |
|---|---|---|---|---|
| 4 | SQUAT | no | - | 0.2-0.8 |
| 5 | KNEEL_TWO_LEGS | no | - | 0.2-0.8 |
| 6 | KNEEL_ONE_LEG | no | - | 0.2-0.8 |
| 8 | HAND_CRAWL | yes | 0.4-1.0 | 0.2-0.8 |
| 14 | ELBOW_CRAWL | yes | 0.7-1.0 | 0.2-0.8 |

- `movement_direction`: any direction for the crawls; `[0, 0, 0]` for SQUAT and the kneels.
- `facing_direction`: any heading; static poses turn in place.
- `speed`: adjustable for the crawls only.
- `pelvis_height`: adjustable in every mode.

### Set 2: Boxing

| Id | Name | Movable | Speed (m/s) | Pelvis height (m) |
|---|---|---|---|---|
| 9 | BOXING_IDLE | no | - | fixed |
| 10 | BOXING_WALK | yes | 0.7-1.5 | fixed |
| 11 | BOXING_LEFT_JAB | yes | 0.7-1.5 | fixed |
| 12 | BOXING_RIGHT_JAB | yes | 0.7-1.5 | fixed |
| 13 | BOXING_RANDOM_PUNCHES | yes | 0.7-1.5 | fixed |
| 15 | BOXING_LEFT_HOOK | yes | 0.7-1.5 | fixed |
| 16 | BOXING_RIGHT_HOOK | yes | 0.7-1.5 | fixed |

- `movement_direction`: any direction; `[0, 0, 0]` for BOXING_IDLE.
- `facing_direction`: any heading; BOXING_IDLE turns in place.
- `speed`: adjustable in every mode except BOXING_IDLE.
- `pelvis_height`: fixed.

### Set 3: Styled walking

| Id | Name | Movable | Speed (m/s) | Pelvis height (m) |
|---|---|---|---|---|
| 20 | CAREFUL_WALK | yes | fixed | fixed |
| 21 | OBJECT_CARRYING | yes | fixed | fixed |
| 22 | CROUCH_WALK | yes | fixed | fixed |
| 23 | HAPPY_DANCE_WALK | yes | fixed | fixed |
| 24 | ZOMBIE_WALK | yes | fixed | fixed |
| 25 | POINT_WALK | yes | fixed | fixed |
| 26 | SCARED_WALK | yes | fixed | fixed |

- `movement_direction`: any direction, except HAPPY_DANCE_WALK, which walks forward only (`movement_direction` = `facing_direction`).
- `facing_direction`: any heading.
- `speed`: fixed at each mode's default.
- `pelvis_height`: fixed.

### Ungrouped

Not in any teleop set; LYING_FACE_DOWN is disabled upstream.

| Id | Name | Movable | Speed (m/s) | Pelvis height (m) |
|---|---|---|---|---|
| 0 | IDLE | no | - | fixed |
| 7 | LYING_FACE_DOWN | no | - | 0.2-0.8 |

- `movement_direction`: `[0, 0, 0]` for both.
- `facing_direction`: any heading; both turn in place.
- `speed`: fixed.
- `pelvis_height`: fixed for IDLE; adjustable for LYING_FACE_DOWN.

## Get up

SONIC has no get-up mode: switch from a ground pose (SQUAT, a kneel, a crawl, LYING_FACE_DOWN) to IDLE with `pelvis_height` back at `-1`, and the planner plans a ~1.5 s stand-up.

```sh
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
ros2 topic pub --once /sonic/pelvis_height std_msgs/msg/Float32 "{data: -1.0}"
ros2 topic pub --once /sonic/mode std_msgs/msg/Int32 "{data: 0}"
```

```python
controller.command(SonicMode.IDLE)  # zero velocity, pelvis_height -1
```

- Works from a deliberate ground pose only; there is no recovery from a fall.
- Crawl → IDLE is unreliable in sim: the robot can lag the stand-up and fall. Going through a kneel or SQUAT first may help (untested).
