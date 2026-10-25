# Changelog

## [0.2.0] - 2026-10-25
### Added
- `SonicController`: the G1 SONIC whole-body controller, the extension's only public API (a Scenario drives it with `forward(dt)` from a physics callback).
- `SonicController.command(mode, vx, vy, wz, pelvis_height)`: sets the whole Locomotion command from Python; `reset()` clears it to idle.
- `SonicMode`: names for all 27 SONIC modes, ids 0-26 (IDLE, SLOW_WALK, WALK, RUN, SQUAT, ... SCARED_WALK).
- `ros` flag of `SonicController` (default on): `ros=False` creates no ROS 2 subscriptions.
- `KinematicPlanner` (internal): builds the planner inputs from `/cmd_vel`, `/sonic/mode` and `/sonic/pelvis_height` and runs `planner_sonic.onnx`.
- `KinematicPlanner` (internal) keeps a 50 Hz stored plan, gives the policy its reference from it and replans on a worker thread that never blocks the policy tick.

### Removed
- The scaffold `add()` import and its test.
- `KinematicPlannerRunner` (`runner.py`) and `ros.read_mujoco_joint_positions`.

## [0.1.0] - 2026-10-24
### Added
- Initial version.
