# G1 SONIC

Moves a Unitree G1 with NVIDIA SONIC whole-body control while the simulation plays. It runs SONIC's
kinematic planner and tracking policy in-process with onnxruntime (see
[ADR 0002](../../../../docs/adr/0002-sonic-in-process-onnx.md)). Commands come from a code-defined
Real-time control (Locomotion commands, cmd_vel-ready for Nav2): slow walk, walk and run, side steps and turns.
Lower-body ground moves (squat, kneel and crawl) are also available. Enabling the extension does not move any
robot; a caller starts a `G1Wbc` and gives it commands.

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
```

The warehouse G1 then stands until commanded (with `--ros`, by `/cmd_vel`; see `oc.scenario.warehouse`). Physics
should run at 200 Hz, as SONIC was trained; the warehouse sets this for `sonic`, and `G1Wbc` warns otherwise.

## API

```python
from oc.control.g1_sonic import G1Wbc, Gait, LocomotionCommand

robot = G1Wbc("/World/G1")  # IMU at /World/G1/pelvis/imu_sensor
robot.start()  # steps on every Play until stop(); stands until it gets a command

# Real-time control: each call cancels the running command or posture.
robot.command(LocomotionCommand(0.6, 0.0, 0.3, Gait.WALK))  # vx m/s, vy m/s, wz rad/s, robot frame; repeat within 0.5 s
robot.walk((0.6, 0.0, 0.3))  # the same; also slow_walk(cmd_vel) and run(cmd_vel)
robot.set_speed_limit(0.8)   # m/s cap on real-time commands, like Nav2's SpeedLimit: slow walk at most; None lifts it
robot.halt()                 # slow down and stand

# Lower-body ground moves (SONIC's ground modes; halt() or a drive stands back up):
robot.squat(0.6)             # hold a squat at pelvis height 0.4-0.8 m
robot.kneel(0.3)             # kneel on two legs, 0.2-0.4 m; kneel_one_leg(0.3) for one
robot.crawl_hands()          # enter hand crawl (mode 8): /cmd_vel or command() then drives it at 0.4-1.0 m/s
robot.crawl_elbows()         # enter elbow crawl (mode 14), a lower crawl at 0.7-1.0 m/s
```

- **Gait is a ceiling.** A Locomotion command's `gait` (`IDLE`, `SLOW_WALK`, `WALK`, `RUN`) is the fastest
  SONIC mode it may use; the speed picks the mode below it (slow walk up to 0.8 m/s, walk up to 1.5 m/s,
  run above). `gait=None` lets the speed pick any mode. So a run slowing down passes through walk and slow
  walk, and `run((0.5, 0, 0))` slow-walks at 0.5 m/s.
- **Limits.** Commands are forward only, up to the gait's top speed (0.8, 1.5 or 3.0 m/s), |vy| <= 0.4 m/s,
  |wz| <= 1.5 rad/s. Below 0.1 m/s the robot stands; forward speeds below 0.4 m/s are raised to 0.4, where
  SONIC's slow walk stops stepping in place.
- **Validation.** Speeds and posture heights must be in range, or `ValueError`.
- **Real-time commands** stop (the robot stands) when no new one arrives for 0.5 s. A command with no linear
  speed turns in place at `wz`. `set_speed_limit` scales their linear speed down to the limit, which also caps
  the gait (up to 0.8 m/s slow walk, up to 1.5 m/s walk).
- **Lower-body moves.** `squat(height)` [0.4-0.8 m], `kneel(height)` and `kneel_one_leg(height)` [0.2-0.4 m] hold a
  static ground posture in place until the next command. `crawl_hands()` and `crawl_elbows()` enter a low *moving*
  mode (SONIC modes 8 and 14) that real-time commands then drive at 0.4-1.0 / 0.7-1.0 m/s, holding the low pose when
  they stop; `halt()` stands back up. Standing up directly from elbow crawl is unreliable — enter hand crawl first.
  An out-of-range height raises `ValueError`.

## Behavior

The controller is layered: Real-time control -> Locomotion command -> Whole-body controller. Each policy step
(50 Hz, every 4th physics step at 200 Hz), `G1WbcController`:
1. Reads the robot's state. Orientation and body angular velocity come from the pelvis IMU, joints from the
   articulation. The position is not read.
2. Gets the latest real-time Locomotion command, or stands once it is 0.5 s old or when there is none. Nav2's own
   controllers come in through Real-time control (`cmd_vel`).
3. Turns the command into planner inputs: a gait mode, a world movement direction (the command's
   direction from the measured yaw), a speed and a facing direction. The facing turns by `wz` each step
   and stays within 30° of the measured yaw.
4. Runs SONIC's kinematic planner (`planner_sonic.onnx`, 30 Hz output) when the command changes, at
   most at 10 Hz. It resamples the plan to 50 Hz and cross-fades it into the current reference motion
   over 8 frames.
5. Encodes the reference's next 0.9 s (10 frames, 0.1 s apart) into a motion token.
6. Decodes the token and 10 frames of state history into joint position targets for all 29 body joints.

On bind, the 29 joints get SONIC's PD gains, armatures and effort limits, and start in its standing
pose. Falls are not handled. Jumps are not supported, because SONIC's planner has no jump mode.

`G1WbcController` is a MotionGen controller (`isaacsim.robot_motion.experimental.motion_generation.BaseController`).
To step it directly: `command(locomotion_command)`, then `reset(state, None, t)`
and `forward(state, None, t)` at 50 Hz.

## Tests

Extension Manager → G1 SONIC → Tests tab runs `oc.control.g1_sonic.tests`. They cover:
- Locomotion command limits, gait ceilings, the speed limit, timeout and halt
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
