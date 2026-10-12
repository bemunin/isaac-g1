# Changelog

## [0.1.0] - 2026-10-08
### Added
- `G1Wbc`: moves a G1 with NVIDIA SONIC whole-body control (default weights, kinematic planner + tracking policy, onnxruntime on CPU) while the simulation plays, with IMU and pose feedback.
- `BehaviorSequence`: an unchangeable list of `Segment`s from behaviors named after Nav2's: `drive_on_heading(meters, speed, gait=None)` and its gait shortcuts `slow_walk`, `walk`, `run` (forward only), `side_step(±meters)`, `spin(±degrees)` (positive: left, REP-103) and `wait(seconds)` / `wait()`; drives slow down automatically before waits, spins and side steps.
- Layered control: `SequenceExecutor` (steering, sequencing, timeouts, halt; `MovementFailure`) and `LocomotionCommand` with a `Gait` ceiling (`IDLE`, `SLOW_WALK`, `WALK`, `RUN`), the controller's one input. `MovementStatus` ends in Nav2's `SUCCEEDED`, `ABORTED` or `CANCELED`.
- Real-time control: `command(LocomotionCommand)`, `slow_walk/walk/run(cmd_vel)` (limited, 0.5 s timeout), `side_move_left/right`, `turn_left/right`, `halt()`; each command cancels the one before. `set_speed_limit(m/s)` caps real-time speed and so the gait, like Nav2's `SpeedLimit`.
- `G1WbcController`: the MotionGen controller behind `G1Wbc`.
- `pixi run sonic-download` fetches the weights into `data/sonic/`.
