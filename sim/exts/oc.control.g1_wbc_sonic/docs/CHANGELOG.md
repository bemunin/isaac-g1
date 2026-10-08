# Changelog

## [0.1.0] - 2026-10-08
### Added
- `G1Wbc`: moves a G1 with NVIDIA SONIC whole-body control (default weights, kinematic planner + tracking policy, onnxruntime on CPU) while the simulation plays, with IMU and pose feedback.
- `MovementPlan`: an unchangeable list of `Segment`s from the Movement primitives `slow_walk`, `walk`, `run` (forward only), `side_move_left/right`, `turn_left/right` and `stop(seconds)` / `stop()`; forward moves slow down automatically before stops, turns and side moves.
- Layered control: `PlanExecutor` (sequencing, timeouts, halt; `MovementFailure`), `PathTracker` / `GoalLineTracker` (steering) and `LocomotionCommand` with a `Gait` ceiling (`IDLE`, `SLOW_WALK`, `WALK`, `RUN`), the controller's one input.
- Real-time control: `command(LocomotionCommand)`, `slow_walk/walk/run(cmd_vel)` (limited, 0.5 s timeout), `side_move_left/right`, `turn_left/right`, `halt()`; each command cancels the one before.
- `G1WbcController`: the MotionGen controller behind `G1Wbc`.
- `pixi run sonic-download` fetches the weights into `data/sonic/`.
