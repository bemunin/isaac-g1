# Changelog

## [0.3.0] - 2026-10-15
### Changed
- Renamed `G1Wbc` to `G1Sonic`, `G1WbcController` to `G1SonicController` (`controllers/sonic_controller.py`).
- The one input is a body-frame velocity (Nav2's `cmd_vel`): `G1Sonic.command((vx, vy, wz))` and `halt()`. It always moves in SONIC's walk mode (planner mode 2) and stands in idle mode below 0.1 m/s or 0.5 s after the last command. Forward and backward speeds are clamped to 0.8-1.5 m/s (backward up to 0.8); strafing stays up to 0.4 m/s. `MovementStatus` is `IDLE` or `RUNNING`.

### Kept
- The lower-body ground moves (`squat`, `kneel`, `kneel_one_leg`, `crawl_hands`, `crawl_elbows`) on `G1Sonic`; `/cmd_vel` drives a crawl.

### Removed
- `BehaviorSequence`, `Segment`, `SequenceExecutor`, `MovementFailure`, `execute`, `sequence`, `failure`, `on_done`, `side_move_left/right`, `turn_left/right`, `slow_walk/walk/run`, `set_speed_limit`, the `Gait` ceiling and the public `LocomotionCommand`: routes come from Nav2 over `/cmd_vel`.

## [0.1.2] - 2026-10-15
### Added
- Lower-body ground moves on `G1Wbc`: `squat(height)` (0.4-0.8 m), `kneel(height)` and `kneel_one_leg(height)` (0.2-0.4 m) hold a static ground posture until the next command; `crawl_hands()` and `crawl_elbows()` enter a low *moving* mode (SONIC modes 8 and 14) that real-time commands drive at 0.4-1.0 / 0.7-1.0 m/s and hold the low pose when they stop. `halt()` stands back up (from elbow crawl, enter hand crawl first). An out-of-range height raises `ValueError`.
- `Posture`, `Crawl` and their height/speed ranges in `locomotion_command.py`; `KinematicPlanner.command(..., height=-1.0)` threads SONIC's pelvis-height input (was hardcoded), and `G1WbcController` gains a `posture(mode, height)` hold and a `crawl(mode)` locomotion mode.

## [0.1.0] - 2026-10-08
### Added
- `G1Wbc`: moves a G1 with NVIDIA SONIC whole-body control (default weights, kinematic planner + tracking policy, onnxruntime on CPU) while the simulation plays, with IMU and pose feedback.
- `BehaviorSequence`: an unchangeable list of `Segment`s from behaviors named after Nav2's: `drive_on_heading(meters, speed, gait=None)` and its gait shortcuts `slow_walk`, `walk`, `run` (forward only), `side_step(±meters)`, `spin(±degrees)` (positive: left, REP-103) and `wait(seconds)` / `wait()`; drives slow down automatically before waits, spins and side steps.
- Layered control: `SequenceExecutor` (steering, sequencing, timeouts, halt; `MovementFailure`) and `LocomotionCommand` with a `Gait` ceiling (`IDLE`, `SLOW_WALK`, `WALK`, `RUN`), the controller's one input. `MovementStatus` ends in Nav2's `SUCCEEDED`, `ABORTED` or `CANCELED`.
- Real-time control: `command(LocomotionCommand)`, `slow_walk/walk/run(cmd_vel)` (limited, 0.5 s timeout), `side_move_left/right`, `turn_left/right`, `halt()`; each command cancels the one before. `set_speed_limit(m/s)` caps real-time speed and so the gait, like Nav2's `SpeedLimit`.
- `G1WbcController`: the MotionGen controller behind `G1Wbc`.
- `pixi run sonic-download` fetches the weights into `data/sonic/`.
