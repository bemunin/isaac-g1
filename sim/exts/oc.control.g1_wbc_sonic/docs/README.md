# G1 WBC SONIC

Moves a Unitree G1 with NVIDIA SONIC whole-body control while the simulation plays. It runs SONIC's
kinematic planner and tracking policy in-process with onnxruntime (see
[ADR 0002](../../../../docs/adr/0002-sonic-in-process-onnx.md)). Commands are Movement primitives:
slow walk, walk, run, side move left/right, turn left/right and stop. They come from a code-defined
Movement plan or as Real-time control (Locomotion commands, cmd_vel-ready for nav2).
Enabling the extension does not move any robot; a caller starts a `G1Wbc` and gives it commands.

Extension ID: `oc.control.g1_wbc_sonic`

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
pixi run warehouse --controller wbc --physics physx
```

The warehouse G1 then follows `WAREHOUSE_MOVEMENT_PLAN` (see `oc.scenario.warehouse`). Physics should
run at 200 Hz, as SONIC was trained; the warehouse sets this for `wbc`, and `G1Wbc` warns otherwise. (A
lap also completed at 100 Hz.)

## API

```python
from oc.control.g1_wbc_sonic import G1Wbc, Gait, LocomotionCommand, MovementPlan, MovementStatus

robot = G1Wbc("/World/G1", on_done=lambda status: print(status))  # IMU at /World/G1/pelvis/imu_sensor
robot.start()  # steps on every Play until stop(); stands until it gets a command

# Movement plan: fixed in code, followed with pose feedback. Each primitive returns a new plan.
robot.execute(MovementPlan(loop=True).stop(2).run(10).turn_left(90).walk(3).side_move_right(1))

# Real-time control: each call cancels the running command or plan (status CANCELLED).
robot.command(LocomotionCommand(0.6, 0.0, 0.3, Gait.WALK))  # vx m/s, vy m/s, wz rad/s, robot frame; repeat within 0.5 s
robot.walk((0.6, 0.0, 0.3))  # the same; also slow_walk(cmd_vel) and run(cmd_vel)
robot.side_move_left(1.0)    # meters, runs to completion
robot.turn_right(90)         # degrees, in place
robot.halt()                 # slow down and stand; also ends a looping plan
robot.status                 # MovementStatus.IDLE, RUNNING, DONE, FAILED or CANCELLED
robot.failure                # MovementFailure(reason="timeout", index=segment) after FAILED
```

| Primitive | Movement plan | Gait |
| --- | --- | --- |
| `slow_walk` | meters forward, `speed=` 0.4-0.8 m/s (default 0.6) | slow walk |
| `walk` | meters forward, `speed=` 0.8-1.5 m/s (default 1.0) | walk |
| `run` | meters forward, `speed=` 1.5-3.0 m/s (default 2.0) | run |
| `side_move_left`, `side_move_right` | meters, `speed=` up to 0.4 m/s (default 0.3) | slow walk |
| `turn_left`, `turn_right` | degrees, in place | idle |
| `stop` | `stop(seconds)` stands, then goes on; `stop()` stands until the next command and ends the plan | idle |

- **Gait is a ceiling.** A Locomotion command's `gait` (`IDLE`, `SLOW_WALK`, `WALK`, `RUN`) is the fastest
  SONIC mode it may use; the speed picks the mode below it (slow walk up to 0.8 m/s, walk up to 1.5 m/s,
  run above). `gait=None` lets the speed pick any mode. So a run slowing down passes through walk and slow
  walk, and `run((0.5, 0, 0))` slow-walks at 0.5 m/s.
- **Limits.** Commands are forward only, up to the gait's top speed (0.8, 1.5 or 3.0 m/s), |vy| <= 0.4 m/s,
  |wz| <= 1.5 rad/s. Below 0.1 m/s the robot stands; forward speeds below 0.4 m/s are raised to 0.4, where
  SONIC's slow walk stops stepping in place.
- **Slowdowns are automatic.** A slow walk, walk or run carries its speed into a following one (the lower of the
  two) and otherwise slows to a stop before the target, at 1.0 m/s² (slow walk, walk) or 1.5 m/s² (run).
  Speeding up is left to SONIC's planner.
- **Validation.** Amounts must be positive and speeds in range, or `ValueError`. Nothing can follow `stop()`.
  Executing an empty plan raises `ValueError`.
- **Loops.** `MovementPlan(loop=True)` repeats from its first segment, measured from the same start pose,
  until `halt()` or another command. A plan ending in `stop()` doesn't repeat.
- **Real-time commands** stop (status IDLE) when no new one arrives for 0.5 s. A command with no linear
  speed turns in place at `wz`.
- **Callbacks.** `on_done` is called with DONE, FAILED or CANCELLED when a Movement plan ends.

## Behavior

The controller is layered: Movement plan -> Plan executor -> Path tracker -> Locomotion command ->
Whole-body controller. Each policy step (50 Hz, every 4th physics step at 200 Hz), `G1WbcController`:
1. Reads the robot's state. Orientation and body angular velocity come from the pelvis IMU. Position
   and joints come from the articulation (simulation ground truth, as in ADR 0001).
2. Gets a Locomotion command: the latest real-time one, or else the Plan executor's (`plan_executor.py`)
   active Segment, steered by the Path tracker (`path_tracker.py`) and stamped with the Segment's gait.
   - The executor anchors the plan at the robot's pose when it starts and moves on when a Segment ends:
     a forward or side move 0.15 m before its target (along the segment), a turn within 5°, a stop after
     its time. A Segment not done within 2 × its nominal time + 3 s fails the plan (FAILED, `failure`).
   - The tracker (`GoalLineTracker`) steers along the line from the Segment's start to its target, with a
     correction back onto the line, at the speed profile above; it turns toward the target heading at
     2 rad/s per rad of error (up to 1.5 rad/s). A different tracker (for example nav2's) can be passed in.
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
To step it directly: `execute(plan)` or `command(locomotion_command)`, then `reset(state, None, t)`
and `forward(state, None, t)` at 50 Hz.

## Tests

Extension Manager → G1 WBC SONIC → Tests tab runs `oc.control.g1_wbc_sonic.tests`. They cover:
- Movement plans, speed profiles, the Path tracker and the Plan executor
- Locomotion command limits, gait ceilings, timeout and preemption
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
