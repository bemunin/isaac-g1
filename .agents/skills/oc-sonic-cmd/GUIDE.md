# Commanding the G1 with SONIC: planner inputs and ranges

How a ROS 2 command becomes the inputs of SONIC's kinematic planner, which values each of the 27 modes accepts, and what
the robot does between commands. It reads on its own; the commands to send are in `SKILL.md`.

Paths are relative to the repo root (`ADR3` = `docs/adr/0003-kinematic-planner-inputs.md`, `ADR5` =
`docs/adr/0005-kinematic-planner-never-blocks.md`). `KP` = `sim/exts/oc.control.g1_sonic/oc/control/g1_sonic/controller/kinematic_planner.py`.
`NV` = NVIDIA's GR00T-WholeBodyControl `gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/`.

## 1. How a command becomes planner inputs

Three ROS 2 topics feed the planner. It rebuilds its inputs from the latest values every 0.1 s of sim time.

| ROS 2 topic and field | Planner input (ONNX name) | Meaning |
|---|---|---|
| `/cmd_vel` `linear.x`, `linear.y` [m/s] | `movement_direction` and `speed` (`target_vel`) | Body-relative. Direction is `atan2(vy, vx) + facing_yaw`, speed is `hypot(vx, vy)`. Below 0.01 m/s, or in mode 0, the robot stands: direction zeros, speed -1 (`KP:36,283-289`). |
| `/cmd_vel` `angular.z` [rad/s] | `facing_direction` | Integrated: `facing_yaw += wz * dt`, starting at 0 on each Play (`KP:179,207,233`). The turn continues while `wz` is non-zero; nothing times it out. |
| `/sonic/mode` (`std_msgs/Int32`) | `mode` | Id 0-26. |
| `/sonic/pelvis_height` (`std_msgs/Float32`) | `height` [m] | -1 lets the mode decide. |

- `linear.z`, `angular.x` and `angular.y` are ignored (`sim/exts/oc.robot.g1/oc/robot/g1/ros/cmd_vel.py:52-53` reads `vx`, `vy`, `wz`).
- `speed` -1 (or any value <= 0) means the mode's default (`KP:58`).
- `/cmd_vel` is stored when a Twist arrives, so one message is enough; the planner keeps integrating `wz` between messages (ADR3 Consequences).
- The first plan after Play starts from the robot's measured joints with facing 0 (`KP:198-213`). Replans start from the planner's own stored plan (`KP:254-265`).
- A mode id of 27 or above is invalid: the ROS subscriber's setter raises (`KP:190-193`), and NVIDIA's C++ replaces it with 0.
- On Stop, `SonicController.reset()` returns the command to idle: mode 0, zero velocity and turn rate, height -1 (`controller/sonic_controller.py:100-104`). The next Play starts again at facing 0.

Fixed inputs the user never sets (`KP:43-49`): `random_seed` 0, `has_specific_target` 0, `specific_target_positions` and
`specific_target_headings` zeros, `allowed_pred_num_tokens` with the first 6 of 11 set to 1. `context_mujoco_qpos` (4 frames at
30 Hz) comes from the measured joints on the first plan and from the planner's stored plan afterwards (ADR3).

## 2. Mode reference

- **Moving** modes follow velocity and facing. **Static** modes hold in place and replan when mode, facing or height changes.
- **Speed** "clip" means the motion clip's own speed (`speed` -1); a positive command is still tracked.
- **Height** "-" means send -1; "required" means the mode needs a value.
- NVIDIA names in brackets are the C++ enum names where they differ (`NV localmotion_kplanner.hpp:78-107`).

| id | name | family | kind | speed (m/s) | pelvis height (m) | notes |
|---|---|---|---|---|---|---|
| 0 | idle | standing | static | - | - | Turn in place with `wz`. |
| 1 | slow_walk | standing | moving | 0.2-0.8 | - | |
| 2 | walk | standing | moving | 0.8-1.5 | - | NVIDIA's enum comment gives 0.8-2.5; its keyboard handler sends the default speed. 1.5 is this repo's suggestion. |
| 3 | run | standing | moving | 1.5-3.0 | - | NVIDIA's enum comment says 2.5-7.5; its keyboard clamp is 1.5-3.0. |
| 4 | squat | posture | static | - | 0.4-0.8 | |
| 5 | kneel_two_legs | posture | static | - | 0.2-0.4 | |
| 6 | kneel_one_leg | posture | static | - | 0.2-0.4 | |
| 7 | lying_face_down | posture | static | - | required | Experimental: NVIDIA left it out of its ground set until it has a better motion. Range: NVIDIA clamps ground modes to 0.2-0.8. |
| 8 | hand_crawl | crawl | moving | 0.4-1.0 | 0.2-0.8 | |
| 9 | boxing_idle | boxing | static | - | - | |
| 10 | boxing_walk | boxing | moving | 0.7-1.5 | - | |
| 11 | boxing_left_jab | boxing | moving | 0.7-1.5 | - | |
| 12 | boxing_right_jab | boxing | moving | 0.7-1.5 | - | |
| 13 | boxing_random_punches | boxing | moving | 0.7-1.5 | - | |
| 14 | elbow_crawl | crawl | moving | 0.7-1.0 | 0.2-0.8 | Overheats the real robot. |
| 15 | boxing_left_hook | boxing | moving | 0.7-1.5 | - | |
| 16 | boxing_right_hook | boxing | moving | 0.7-1.5 | - | |
| 17 | happy_walk | styled | moving | clip | - | `FORWARD_JUMP` in NVIDIA's enum; it walks. |
| 18 | stealth_walk | styled | moving | clip | - | |
| 19 | injured_walk | styled | moving | clip | - | |
| 20 | careful_walk | styled | moving | clip | - | `LEDGE_WALKING`. |
| 21 | object_carrying | styled | moving | clip | - | |
| 22 | crouch_walk | styled | moving | clip | - | `STEALTH_WALK_2`. |
| 23 | happy_dance_walk | styled | moving | clip | - | Forward only. |
| 24 | zombie_walk | styled | moving | clip | - | |
| 25 | point_walk | styled | moving | clip | - | `GUN_WALK`. |
| 26 | scared_walk | styled | moving | clip | - | `SCARE_WALK`. |

Verification status: the weights are NVIDIA's V2 planner and every id produces valid planner motion. Whether the tracking policy
follows modes 7 and 9-26 in Isaac Sim is unverified, and so is whether boxing and styled walks can side-step or walk backwards.
Watch the robot when using them.

## 3. Ranges and rules

**Pelvis height [m]**
- Standing, boxing and styled modes: -1. NVIDIA forces -1 for them (`NV input_interface/keyboard_handler.hpp:260-267`, `is_standing_motion_mode` at `localmotion_kplanner.hpp:123-145`).
- Squat, kneel, lying and both crawls: a value is required. NVIDIA clamps these to 0.2-0.8 (`keyboard_handler.hpp:260-267`); NVIDIA's docs give squat about 0.4-0.8 and kneels 0.2-0.4 (`docs/source/references/planner_onnx.md:95-150`). Stay inside those per-mode ranges.

**Speed [m/s]**
- Static modes ignore it (NVIDIA sends -1).
- Moving modes: the ranges in the table, from NVIDIA's keyboard clamps (`keyboard_handler.hpp:269-294`): slow_walk 0.2-0.8, run 1.5-3.0, hand_crawl 0.4-1.0, elbow_crawl 0.7-1.0, boxing walk, jabs, punches and hooks 0.7-1.5. Walk and styled walks have no clamp there; they run at the clip's speed.
- Direction (`vx`, `vy`) is free in the plane. Side-stepping with `vy` is unverified for boxing and styled walks.

**Turn rate [rad/s]**
- NVIDIA's ROS 2 handler sets no limit on `angular.z`; it adds `ang_vel_z * 0.02` to its facing angle when `|ang_vel_z| > 0.01` (`NV input_interface/ros2_input_handler.hpp:737-743`). Its other limit is on height, clamped to 0.1-0.88 (`:729`).
- Suggestion from this repo, not NVIDIA's: keep `|wz|` at or below 1.0 rad/s; a turn of angle A takes A / wz seconds.
- Static modes ignore velocity but replan when facing changes, so idle with `wz` turns in place (the body follows the facing direction).

**Holding still**
- A moving mode with zero velocity is sent speed -1 (mode default) here; NVIDIA sends 0 for crawls, boxing and styled walks when keys are released (`keyboard_handler.hpp:625-691`; boxing punches and hooks also keep the facing as movement direction). The robot may keep moving here. To stand still, send mode 0 or a static mode.
- NVIDIA's ROS 2 handler differs from this repo in what it sends: it bins the movement direction to 8 directions and picks the mode from the height command (`ros2_input_handler.hpp:753-801`). Here, mode, height and velocity are separate inputs and direction is continuous.

## 4. Timing

- Replans run every 0.1 s of sim time (`PLANNER_PERIOD`, `KP:30,236`), on one worker thread. The policy reads the stored 50 Hz motion every tick and never waits (ADR5).
- If a replan is still running at the next 0.1 s tick, that tick is skipped. A plan that lands late is cross-faded in over 8 frames (0.16 s) from the frame it was planned for. A command reaches the motion one inference (about 52 ms on CPU) after its replan tick (ADR5 Consequences).
- A command holds until replaced. A bounded move is a command, a wait on sim time, then an idle command.
- Results are approximate: two runs are not bit-identical because plans land at wall-clock-dependent frames.

## 5. Sources

- ADR3 (input and output tables), ADR5 (async planner).
- `KP`: `_build_inputs` `:282-292`, `step` `:228-243`, `FIXED_INPUTS` `:43-49`, `initialize` `:198-217`.
- `controller/ros.py:68-103` and `sim/exts/oc.robot.g1/oc/robot/g1/ros/cmd_vel.py` (ROS mapping), `controller/sonic_controller.py:100-104` (reset).
- NVIDIA: `docs/source/references/planner_onnx.md:95-150`; `NV input_interface/keyboard_handler.hpp:260-294,625-691`; `NV localmotion_kplanner.hpp:78-180`; `NV input_interface/ros2_input_handler.hpp:729-801`.
