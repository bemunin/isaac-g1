# G1 SONIC

Moves a Unitree G1 with NVIDIA SONIC whole-body control while the simulation plays. It runs SONIC's
kinematic planner and tracking policy in-process with onnxruntime (see
[ADR 0002](../../../../docs/adr/0002-sonic-in-process-onnx.md)). Its one input is a body-frame velocity,
Nav2's `cmd_vel`, and it always moves in SONIC's locomotion **walk** mode (planner mode 2, as key 2 in
NVIDIA's [keyboard tutorial](https://nvlabs.github.io/GR00T-WholeBodyControl/tutorials/keyboard.html)),
standing in idle mode between commands. `oc.scenario.warehouse` forwards `/cmd_vel` to it (see
[ADR 0003](../../../../docs/adr/0003-sonic-walk-from-nav2-cmd-vel.md)).
Lower-body ground moves (squat, kneel and crawl) are also available. Enabling the extension does not move any robot; a caller starts a `G1Sonic` and gives it commands.

Extension ID: `oc.control.g1_sonic`

## Setup

The weights are not in git. Download NVIDIA's default SONIC release (~865 MB, NVIDIA Open Model
License, `data/sonic/LICENSE`) once from the repository root:

```bash
pixi run sonic-download
```

It fetches `model_encoder.onnx`, `model_decoder.onnx`, `observation_config.yaml` and
`planner_sonic.onnx` from [nvidia/GEAR-SONIC](https://huggingface.co/nvidia/GEAR-SONIC) into `data/sonic/`.

## Run

```bash
pixi run warehouse --physics physx
pixi run ros2 run teleop_twist_keyboard teleop_twist_keyboard   # second terminal: drive /cmd_vel by hand
```

The warehouse G1 stands until something publishes `/cmd_vel` (Nav2, teleop, `ros2 topic pub`). Physics
should run at 200 Hz, as SONIC was trained; the warehouse sets this, and `G1Sonic` warns otherwise.

## API

```python
from oc.control.g1_sonic import G1Sonic, MovementStatus

robot = G1Sonic("/World/G1")  # IMU at /World/G1/pelvis/imu_sensor
robot.start()  # steps on every Play until stop(); stands until it gets a command

robot.command((0.6, 0.0, 0.3))  # vx m/s, vy m/s, wz rad/s, robot frame (cmd_vel); repeat within 0.5 s
robot.halt()                     # slow down and stand

# Lower-body ground moves (SONIC's ground modes; halt() or a drive stands back up):
robot.squat(0.6)             # hold a squat at pelvis height 0.4-0.8 m
robot.kneel(0.3)             # kneel on two legs, 0.2-0.4 m; kneel_one_leg(0.3) for one
robot.crawl_hands()          # enter hand crawl (mode 8): command() / cmd_vel then drives it at 0.4-1.0 m/s
robot.crawl_elbows()         # enter elbow crawl (mode 14), a lower crawl at 0.7-1.0 m/s

robot.status                     # MovementStatus.IDLE or RUNNING
```

- **Walk mode only.** A command walks in SONIC's walk mode (2); below 0.1 m/s of linear speed the robot
  stands in idle mode (0) and turns in place at `wz`. There is no slow walk or run.
- **Speeds.** A mostly forward or backward command (|vx| >= |vy|) walks at its speed clamped to 0.8-1.5 m/s,
  so 0.1-0.8 m/s is raised to 0.8. A mostly sideways one keeps its speed, up to 0.4 m/s, or the feet
  collide.
- **Limits.** -0.8 <= vx <= 1.5 m/s (backward up to 0.8), |vy| <= 0.4 m/s, |wz| <= 1.5 rad/s; larger values
  are clamped, with one warning.
- **Timeout.** The robot stands (status IDLE) when no new command arrives for 0.5 s of simulation time.
- **Nav2 tuning.** Keep the controller's speeds inside these limits: e.g. Regulated Pure Pursuit's
  `desired_linear_vel` 0.8-1.5 and `min_approach_linear_velocity` >= 0.8 (slower approaches overshoot,
  since they are raised to 0.8), `BackUp` at <= 0.8 m/s, velocity smoother `max_velocity` `[1.5, 0.4, 1.5]`.
- **Lower-body moves.** `squat(height)` [0.4-0.8 m], `kneel(height)` and `kneel_one_leg(height)` [0.2-0.4 m] hold a
  static ground posture in place until the next command. `crawl_hands()` and `crawl_elbows()` enter a low *moving*
  mode (SONIC modes 8 and 14) that `command()` then drives, forward only, at 0.4-1.0 / 0.7-1.0 m/s, holding the low
  pose when commands stop; `halt()` stands back up. Standing up directly from elbow crawl is unreliable — enter hand
  crawl first. An out-of-range height raises `ValueError`.

## Behavior

Each policy step (50 Hz, every 4th physics step at 200 Hz), `G1SonicController`:
1. Reads the robot's state. Orientation and body angular velocity come from the pelvis IMU. Position
   and joints come from the articulation (simulation ground truth, as in ADR 0001).
2. Takes the latest real-time Locomotion command, or stands if there is none.
3. Turns the command into planner inputs: walk or idle mode, a world movement direction (the command's
   direction from the measured yaw), a speed and a facing direction. The facing turns by `wz` each step
   and stays within 30° of the measured yaw.
4. Runs SONIC's kinematic planner (`planner_sonic.onnx`, 30 Hz output) when the command changes, at
   most at 10 Hz. It resamples the plan to 50 Hz and cross-fades it into the current reference motion
   over 8 frames.
5. Encodes the reference's next 0.9 s (10 frames, 0.1 s apart) into a motion token.
6. Decodes the token and 10 frames of state history into joint position targets for all 29 body joints.

On bind, the 29 joints get SONIC's PD gains, armatures and effort limits, and start in its standing
pose. Falls are not handled. Jumps are not supported, because SONIC's planner has no jump mode.

`G1SonicController` is a MotionGen controller (`isaacsim.robot_motion.experimental.motion_generation.BaseController`).
To step it directly: `command(LocomotionCommand(vx, vy, wz))` (from `locomotion_command.py`), then `reset(state, None, t)`
and `forward(state, None, t)` at 50 Hz.

## Tests

Extension Manager → G1 SONIC → Tests tab runs `oc.control.g1_sonic.tests`. They cover:
- Command limits, walk/idle mode selection, speeds, timeout and halt
- Joint orders against NVIDIA's
- ONNX input sizes, the planner, and the controller loop, without a robot

They need the weights.

## Sources

Ported from [NVlabs/GR00T-WholeBodyControl](https://github.com/NVlabs/GR00T-WholeBodyControl) (Apache-2.0),
`gear_sonic_deploy/src/g1/g1_deploy_onnx_ref`:
- `include/policy_parameters.hpp`: joint orders, gains, action scales, default pose
- `src/g1_deploy_onnx_ref.cpp`: observations, heading alignment, plan blending
- `include/localmotion_kplanner*.hpp` and `docs/source/references/planner_onnx.md`: the planner

Research notes: `research/reports/research-g1-sonic-wbc.md`.
