# The Kinematic planner is a stepped class that takes a command from ROS 2 and its context from its own previous plan

Status: Proposed (2026-10-25). Covers the Kinematic planner's (kplanner's) shape, inputs, model inference and output;
how the tracking policy consumes the reference is designed with the policy.

## Decision

The kplanner is a `KinematicPlanner` class in `oc.control.g1_sonic` that is stepped by its owner and registers no
physics callback or simulation manager of its own:

```python
class KinematicPlanner:
    def initialize(self, joint_positions): ...  # first context; ROS 2 graphs for mode and pelvis height
    def step(self, dt): ...                     # integrate facing_yaw, count steps, replan at 10 Hz

# the owner (the SONIC controller) registers the only physics callback
def _on_physics_step(self, dt):
    self._kplanner.step(dt)
    self._policy.step(dt, self._kplanner.reference())
```

The class holds the state that lasts between steps: the ONNX session, `facing_yaw`, the latest `/cmd_vel`, mode and
pelvis height, the stored plan and its playback frame, the last planned inputs (for the replan thresholds) and whether
it is initialized. ROS callbacks only store the latest values; `step(dt)` integrates and plans on sim time.

The kplanner runs at 10 Hz on these inputs:

| Name | Source | Type / shape | Meaning | Read via (status) |
|---|---|---|---|---|
| `mode` | ROS (mode topic) | int, 0–26 | Planner mode (0 idle, 1 slow walk, 2 walk, 3 run, others such as squat or kneel) | To do: a ROS 2 subscriber graph in `oc.control.g1_sonic`, created in the kplanner initialize phase |
| `movement_direction` | ROS (`Twist.linear.x/y`) | [3] unit vector `[cos, sin, 0]`, or zeros to stand | Direction to move in: `movement_yaw = atan2(vy, vx) + facing_yaw`, since `/cmd_vel` is body-relative | Done: oc.robot.g1 `/cmd_vel` graph, `subscribe_cmd_vel()`: `vx, vy` |
| `speed` (ONNX: `target_vel`) | ROS (`Twist.linear` magnitude) | float, m/s | Target speed `hypot(vx, vy)`; ≤ 0 means the mode's default | Done: oc.robot.g1 `/cmd_vel` graph, `subscribe_cmd_vel()`: `vx, vy` |
| `facing_direction` | ROS (`Twist.angular.z`) | [3] unit vector `[cos, sin, 0]` | Direction the body faces: `facing_yaw += wz * dt`, starting at 0 when the kplanner initializes | Done: oc.robot.g1 `/cmd_vel` graph, `subscribe_cmd_vel()`: `wz` |
| `pelvis_height` (ONNX: `height`) | ROS (optional) | float, m | Pelvis height for squat or kneel; -1 means the mode decides | To do: a ROS 2 subscriber graph in `oc.control.g1_sonic`, created in the kplanner initialize phase |
| `context_mujoco_qpos` | First plan: measured G1 joints. Replans: the stored plan | [4, 36], MuJoCo qpos layout | 4 frames at 30 Hz: root xyz, quaternion wxyz, 29 joints in MuJoCo order | To do: first plan, the Isaac Sim Articulation API once (`get_dof_positions`, the 29 body DOFs picked by name in MuJoCo order); replans, the stored plan |

The model also takes these inputs, all fixed:

| ONNX input | Shape | Value |
|---|---|---|
| `random_seed` | [1] int64 | `0` |
| `has_specific_target` | [1, 1] int64 | `0` |
| `specific_target_positions` | [1, 4, 3] float | zeros |
| `specific_target_headings` | [1, 4] float | zeros |
| `allowed_pred_num_tokens` | [1, 11] int64 | first 6 set to `1` |

`context_mujoco_qpos` keeps the ONNX input's name so it passes to `planner_sonic.onnx` unchanged, and `mujoco` marks
the MuJoCo qpos layout: a plain float32 array, joints in MuJoCo order (Unitree's motor order), not Isaac Sim's.

Its output comes in three layers. Frame spacing is the time between consecutive frames of one trajectory; one
inference returns a whole trajectory, at most 10 times a second.

| Layer | Field | Shape | Frame spacing | Updated when | Confirmed against |
|---|---|---|---|---|---|
| ONNX output | `mujoco_qpos` | [1, 64, 36] | 1/30 s | Each replan (at most 10 Hz) | Model file |
| ONNX output | `num_pred_frames` | [1] int32 | n/a | Each replan | Model file |
| Stored plan | Root position, root quaternion (wxyz), joint positions (IsaacLab order), joint velocities (finite difference × 50) | [N, 3], [N, 4], [N, 29], [N, 29] | 1/50 s | When a new plan is blended in (8-frame cross-fade) | NVIDIA code |
| Stored plan | Playback frame | int | n/a | Every policy step (50 Hz) | NVIDIA code |
| Policy reference | Joint positions, joint velocities, root quaternion | [10, 29], [10, 29], [10, 4] | 0.1 s (0–0.9 s ahead) | Read every policy step (50 Hz) | Earlier port only |

N is at most `floor(num_pred_frames / 30 × 50)` (106 for a full 64-frame output), plus the old frames kept by the
blend. The stored plan's root position is used only to build the next context, not by the policy.

## Why

- **Not a physics callback inside the kplanner**: within each 50 Hz step the plan in flight must be spliced in, then
  the policy reads the reference, then the playback frame advances. Separate callbacks for the kplanner and the policy
  run in an order nobody controls, and their step counts (10 Hz replans every 5th policy step, playback +1 per policy
  step) could drift apart. One owner calling `step(dt)` keeps the order and one clock, handles Play, Stop and reset in
  one place, and lets the class run in plain pytest without Isaac Sim.
- **Not a stateless function**: `facing_yaw`, the stored plan and the replan thresholds need state between steps.
- **The kplanner reads the robot only once**: as in NVIDIA's runtime (`gear_sonic_deploy`), the first context is the
  measured joints with the root at (0, 0, default height) and an identity quaternion (`g1_deploy_onnx_ref.cpp`
  `Planner()`, `localmotion_kplanner.hpp` `InitializeContext()`). Every replan samples 4 frames from the stored plan at
  the playback frame plus the look-ahead (`UpdateContextFromMotion()`). The stored plan holds kplanner output only; the
  SONIC policy's joint targets go to the motors and never feed back. The IMU and joint states close the loop in the
  50 Hz policy, not here.
- **The facing angle is integrated from `/cmd_vel`, not read from the IMU**: NVIDIA's ROS 2 input handler keeps its
  own angle (`ros2_input_handler.hpp`: `planner_facing_angle_ += ang_vel_z * 0.02`) and rotates the body-relative
  linear velocity by it. The IMU's measured heading would mix real-world drift into an open-loop plan.
- **`/cmd_vel` alone is not enough**: a Twist carries no mode, pelvis height or absolute facing, so mode and pelvis
  height need their own subscriber graph (a custom message, or `std_msgs` topics such as `Int32` mode and `Float32`
  height).

## Consequences

- oc.robot.g1 only reads `/cmd_vel` today (`subscribe_cmd_vel()`), and the `ROS2_CmdVel` graph is authored by the
  warehouse scenario, not the G1 asset.
- The first context reads the joints from the Articulation API, not `/joint_states`: no ROS dependency or publish
  lag, and selecting DOFs by name both reorders them to MuJoCo order and drops the hand joints. It needs the physics
  view, so it happens on the first physics step after Play.
- `subscribe_cmd_vel()` calls back only when a Twist arrives, so `wz` is stored and integrated in `step(dt)`: a turn
  keeps going if the publisher sends one message and stops.
- Until the policy exists, a test script may own a temporary physics callback that calls `step(dt)`; the class never
  does.
- The replan thresholds come from the earlier port and still need checking against the reference source.
