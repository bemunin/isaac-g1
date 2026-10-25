# SonicController is the extension's one controller: a Scenario calls forward(dt) from its physics callback

Status: Proposed (2026-10-25). Supersedes 0003-stepped-controller-off-thread-planning's `reset()` / `step(dt)` /
`close()` Protocol and the `KinematicPlannerRunner` physics callback in `runner.py`. Keeps 0003's Scenario-owned
physics callback and 0003-kinematic-planner-inputs' stepped `KinematicPlanner`.

## Decision

`oc.control.g1_sonic` exports one class, `SonicController` (`controller/sonic_controller.py`). It owns the
articulation, the drives, the decimation, the Kinematic planner and SONIC's encoder and decoder; the policy inference
is a private method of it, not a controller of its own.

```
Scenario physics callback (PHYSICS_PRE_STEP, 200 Hz)
  -> SonicController.forward(dt)                # sets up on the first call after construction or reset()
      every 4th call (50 Hz; between them the drives hold the last target):
      -> _read_state()                          # body q, dq (IsaacLab order), root wxyz, world angular velocity
      -> _infer(q, dq, q_root, w_world)         # one policy tick, NumPy
          -> kplanner.reference()               # frames now + 0.1 k: [10, 29] joints, velocities, [10, 4] root quats
          -> encoder -> decoder                 # model_encoder.onnx, model_decoder.onnx
          -> kplanner.step(CONTROL_DT)          # then advance: facing_yaw, playback frame, 10 Hz replans
      -> _write_targets(target)                 # set_dof_position_targets on the 29 body DOFs
```

### SonicController (`controller/sonic_controller.py`)

```python
class SonicController:
    def __init__(self, prim_path: str, *, ros: bool = True, directory: Path = SONIC_DIR,
                 providers: list[str] | None = None): ...
    def command(self, mode: int, vx=0.0, vy=0.0, wz=0.0, pelvis_height=-1.0) -> None: ...  # a Locomotion command
    def forward(self, dt: float) -> None: ...  # every physics step
    def reset(self) -> None: ...               # on Stop: the next forward() starts again from the measured pose
    def close(self) -> None: ...               # on shutdown
```

- **Construction** loads the encoder and decoder sessions and the `KinematicPlanner` (the heavy part), so the physics
  step stays fast. It does not touch the stage.
- **Set-up on the first `forward(dt)`** (a physics callback guarantees the physics view): `Articulation(prim_path)`,
  the 29 body DOFs picked by name (`ISAACLAB_JOINTS`, so the USD, IsaacLab and MuJoCo orders never meet by index;
  hands are left alone), the drives, the decimation `round(CONTROL_DT / dt)` (warning if not whole),
  `kplanner.initialize()` from the measured joints in MuJoCo order (with the prim path for its ROS 2 graphs when
  `ros=True`), and
  empty histories, zero last action and a re-armed heading capture.
- **The planner is stepped after the policy has read the reference**, as NVIDIA's tick does
  (`g1_deploy_onnx_ref.cpp:3883-4062`: observations from `current_frame`, then `CurrentFrameAdvancement`). This
  reverses the step-then-read order sketched in 0003-kinematic-planner-inputs.
- **Locomotion commands** come from one of two sources, picked at construction. With `ros=True` (the default),
  `/cmd_vel`, `/sonic/mode` and `/sonic/pelvis_height` reach the planner over ROS 2. With `ros=False`, no ROS 2 graph
  is made and a Python caller sends them with `command(mode, vx, vy, wz, pelvis_height)`: a SONIC mode (`SonicMode`
  or an id 0-26), the velocity in the robot's frame and the pelvis height (-1: the mode decides). `command()` also
  works with `ros=True`, but the latest of the two wins. A command holds until the next one replaces it, with no
  timeout, and turning is by rate only (facing is integrated from `wz`).
- **Robot state** is the articulation root's orientation and angular velocity (the pelvis is the root, so this is an
  ideal pelvis IMU) and the body joints' positions and velocities, as NumPy.
- **`reset()`** marks the controller for set-up and clears the command to idle, standing still, so the next Play
  doesn't resume an old velocity; the articulation and drives are rebuilt on the next `forward()`, because Stop
  restores the USD values. **`close()`** closes the planner (its worker and ROS 2
  subscriptions) and drops the sessions; `forward()` after `close()` raises.
- **Errors raise**: a bad decoder action raises `ValueError`. Guarding the physics callback is the Scenario's job.

### Inference inputs

Sources: NVlabs/GR00T-WholeBodyControl at 45c03fa. `CPP` is
`gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/src/g1_deploy_onnx_ref.cpp`; `TRN` is the training code `gear_sonic/`,
which agrees with the deploy code on every point below. Nothing is scaled; all joint arrays are IsaacLab order;
quaternions are wxyz.

**Encoder** `model_encoder.onnx`: `obs_dict [1, 1762] float32` -> `encoded_tokens [1, 64]`, run every tick. The
buffer is zeroed and mode `g1` (id 0) writes only its required terms (`CPP:2031,2096-2118`):

| Offset | Len | Term | Value |
|---|---|---|---|
| 0 | 4 | `encoder_mode_4` | `[0, 0, 0, 0]` (mode id 0 in element 0; `CPP:1691-1697`) |
| 4 | 290 | `motion_joint_positions_10frame_step5` | reference joints, absolute rad, frame-major `[f0 j0..j28, f1 ...]` (`CPP:772-781`) |
| 294 | 290 | `motion_joint_velocities_10frame_step5` | reference joint velocities, same layout (`CPP:849-858`) |
| 601 | 60 | `motion_anchor_orientation_10frame_step5` | 6 per frame, below |
| 584-600, 661-1761 | | 10 other terms (root z, lower body, VR, SMPL, wrists) | zero |

Frame k of the reference is `current_frame + 5 k` of the 50 Hz motion (0 to 0.9 s ahead, frame 0 is now, clamped to
the last frame; `CPP:654-661`). Velocities are the motion's 50 Hz forward difference.

Anchor orientation per frame (`CPP:617-695`, `TRN/envs/manager_env/mdp/commands.py:1956-1979`):
`d = conj(q_root) * (delta_heading * q_ref[k])`, `R = mat(d)`, written `[R00, R01, R10, R11, R20, R21]` (the first
two columns, row-major; identity is `[1, 0, 0, 1, 0, 0]`). `q_root` is the robot's full orientation now.
`delta_heading = yaw(q_root_at_init) * yaw(q_ref_at_init)^-1` maps the planner's frame (zero yaw at init) to the
world; it is captured once, on the first policy tick after a start, and replans don't recompute it
(`CPP:559-606,3329-3331`).

**Decoder** `model_decoder.onnx`: `obs_dict [1, 994] float32` -> `action [1, 29]`. Term-major; each term is 10 frames
at 50 Hz, oldest first, and the newest is this tick's state (`CPP:1454-1636`, `state_logger.cpp:180-233`):

| Offset | Len | Term | Value |
|---|---|---|---|
| 0 | 64 | `token_state` | this tick's encoder output |
| 64 | 30 | `his_base_angular_velocity_10frame_step1` | root angular velocity in the root (pelvis) frame, `R^T w_world` |
| 94 | 290 | `his_body_joint_positions_10frame_step1` | `q - DEFAULT_POSE` (`CPP:2862-2864`) |
| 384 | 290 | `his_body_joint_velocities_10frame_step1` | `dq` |
| 674 | 290 | `his_last_actions_10frame_step1` | raw decoder output; the newest entry is the previous tick's (`CPP:3160`) |
| 964 | 30 | `his_gravity_dir_10frame_step1` | `rotate(conj(q_root), [0, 0, -1])`; upright is `(0, 0, -1)` (`CPP:1626-1631`) |

`observation_config.yaml`'s header (436) describes 4-frame histories and is stale. On reset, the first observation
fills every history slot, as IsaacLab's `CircularBuffer` does in training; NVIDIA's deploy zero-pads instead but
starts from a settled pose. `last_action` starts at zero.

**Action** (`CPP:3137-3168`, `TRN/envs/manager_env/robots/g1.py:360-371`): `target = DEFAULT_POSE + action * ACTION_SCALE`
per joint, no clipping, velocity target and feed-forward torque zero. Scale is `0.25 * effort_limit / stiffness`.

**Drives** (`policy_parameters.hpp:39-65`, `TRN/envs/manager_env/robots/g1.py:238-357`): stiffness
`armature * (2 pi 10)^2`, damping `4 * armature * 2 pi 10` (rad units), with IsaacLab's per-joint armature, effort and
velocity limits; implicit PD at 200 Hz, decimation 4. SonicController sets them through the articulation API in rad, not
as USD drive attributes, which are per degree.

The per-joint values (default, scale, kp, kd, effort, velocity, armature) are constants in `controller/sonic_policy.py`, each table
citing its source line. Unit tests check them, the offsets (1762, 994) and a standing sanity check: a standing
reference with the robot at the default pose gives max `|action|` about 0.31. Known mistakes give 0.76 (gravity sign
flipped), 2.2 (reference joints in MuJoCo order) or 5.0 (reference joints minus default).

### Public API

`oc.control.g1_sonic` exports `SonicController` and `SonicMode` (`api.py`, re-exported by `__init__.py`):
`from oc.control.g1_sonic import SonicController, SonicMode`. `SonicMode` names all 27 SONIC modes, ids 0-26
(gaits, postures, crawls, boxing and styled walks; table in `.agents/skills/oc-sonic-cmd`). Whether the tracking policy
follows modes 7 and 9-26 in Isaac Sim is not yet checked.
`KinematicPlanner`, the ROS 2 command input and the helpers in `controller/sonic_policy.py` are internal.

### Scenario

`oc.scenario.warehouse`, with the setting `exts."oc.scenario.warehouse".sonic` (default on), creates the controller
after the G1 spawns and registers two SimulationManager callbacks after the stage opens (opening a stage clears them):

- `PHYSICS_PRE_STEP` calls `forward(dt)`, so targets are written before the step they apply to. An exception is
  logged once and stops the calls until the next Stop; the callback never raises.
- `SIMULATION_STOPPED` calls `reset()`.

It deregisters both and calls `close()` before re-opening the stage and on shutdown. If construction fails (for
example, missing weights), it logs the error and the scene opens without SONIC. The controller's `ros` follows the
warehouse's `ros` setting.

`oc.scenario.warehouse.get_sonic_controller()` returns the running controller (None when SONIC is off), so code
running in Isaac Sim's process, such as a snippet sent from VS Code through `isaacsim.code_editor.vscode`
(`--vscode` on the launch scripts, opt-in because it opens an unauthenticated local port), commands the same robot
instead of creating a second controller for it.

## Why

- **One class is the whole interface other extensions need**: a Scenario creates it and calls `forward(dt)`. An
  earlier draft split it into a runner plus an `isaacsim.robot_motion.experimental.motion_generation.BaseController`
  policy controller, after Isaac Sim's `RobotPolicyRunner` / `IsaacLabPolicyController`. That added `RobotState` and
  warp conversions at every tick, an experimental dependency and a second class without a second user.
- **Not `isaacsim.core.api.controllers.BaseController`**: it is in `extsDeprecated/`, and its `forward() ->
  ArticulationAction` returns an action instead of applying it.
- **The planner stepped inside the controller**: the reference must be read before the playback frame next advances.
  One `forward()` keeps that order and one clock.
- **No physics callback in the controller extension**: the Scenario keeps control of the order within a step, and a
  second callback would step the planner twice per tick (0003-kinematic-planner-inputs, Why).
- **Set-up on the first `forward()`**: the physics view exists inside the physics callback, so the Scenario needs no
  Play hook, and `reset()` on Stop is enough to start again from the restored pose.
- **Root state instead of an IMU sensor**: the policy reads the pelvis IMU (`CPP:2928-2929`; the torso IMU is only
  logged) and was trained on the pelvis' `root_ang_vel_b`. The pelvis is the root, so the root's orientation and
  body-frame angular rate are what an ideal pelvis IMU reads, with no sensor prim or ROS lag.
- **History filled with the first observation, not zeros**: it matches training; zero padding would also turn
  gravity's padded entries into `(0, 0, +1)`.

## Consequences

- `forward(dt)` applies the targets and returns nothing, unlike Isaac Sim's controller `forward()`s, which return an
  action.
- `KinematicPlanner` (`controller/kinematic_planner.py`) keeps the blended 50 Hz stored plan; `reference()` returns
  frames at now + 0.1 k on it, with 50 Hz forward-difference velocities and root quats in the planner's frame, and
  `step()` builds its replan context from it (`localmotion_kplanner.hpp:634-680`). Replans run on a worker thread and
  never block the policy tick (0005).
- To check when the G1 runs: that the articulation root is the pelvis, and that the root orientation read through
  `isaacsim.core.experimental.prims` is wxyz on both PhysX and Newton.
- The planner no longer reads the `Articulation`; its first context comes from `SonicController`'s set-up.
- Kit-free tests (`tests/test_sonic_policy.py`) cover the observation building, histories and action decoding, the
  real ONNX standing check, and `SonicController` against a stub planner and a fake articulation (set-up, decimation,
  order, reset, close).
- `scripts/warehouse_cmd_standalone.py` still imports the removed `G1Wbc` and needs rewriting on `SonicController`.

## Plan

1. Done: `controller/sonic_policy.py`: the policy constants (scales, offsets, action scale, PD gains, observation
   layout) with source citations, and the NumPy observation and action helpers.
2. Done: `controller/sonic_controller.py`: `SonicController` with `forward(dt)` / `reset()` / `close()` and the policy
   as `_infer()`; tested against a stub planner and the real ONNX models. It is the only export.
3. Done: `oc.scenario.warehouse`: the `sonic` setting, the PRE_STEP and STOPPED callbacks; headless run logs only the
   missing `reference()`.
4. Done: `controller/kinematic_planner.py`: `reference()` and the stored 50 Hz plan, replanned asynchronously (0005).
5. Done: remove `runner.py`.
6. Check in the warehouse that the G1 stands, then walks from `/cmd_vel` and from `command()`.
