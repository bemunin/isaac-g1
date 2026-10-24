# Changelog

## [0.10.0] - 2026-10-20
### Removed
- `WAREHOUSE_SEQUENCE` and `warehouse_sequence.py`: the G1 no longer walks a loop. On Play it stands until a `/cmd_vel` arrives (with `--ros`). Benchmarks now measure a standing G1 and are not comparable with runs before 0.10.0.

## [0.9.0] - 2026-10-20
### Added
- With `exts."oc.scenario.warehouse".ros` (`pixi run warehouse --ros`), the G1 subscribes to `/cmd_vel` (`geometry_msgs/Twist`) through a new graph, `/World/G1/Graph/ROS2_CmdVel`, and drives with SONIC: each Twist is a Locomotion command that takes over from `WAREHOUSE_SEQUENCE` (Real-time control) until the next Play. `CMD_VEL_GRAPH` is its path.

## [0.8.0] - 2026-10-20
### Removed
- The `walk` controller (`oc.control.g1_walk`), the `exts."oc.scenario.warehouse".controller` setting, `controller()`, `CONTROLLERS` and `WAREHOUSE_WALK_TRAJECTORY`. The G1 always follows `WAREHOUSE_SEQUENCE` with SONIC, and physics always runs at `PHYSICS_RATE` (200 Hz).

## [0.7.0] - 2026-10-17
### Added
- `exts."oc.scenario.warehouse".obstacles` (`pixi run warehouse -o clutter,forklift`) shows the Scenario's hidden clutter and forklift. `obstacles()` reads it and `apply_obstacles(stage, kinds)` activates the `OBSTACLES` paths in the session layer; `open_scenario()` calls both.

## [0.6.0] - 2026-10-12
### Changed
- The default controller is now `sonic` (SONIC whole-body control); `exts."oc.scenario.warehouse".controller = "walk"` (`pixi run warehouse --controller walk`) selects the walking policy.

## [0.5.0] - 2026-10-12
### Changed
- The controller value `wbc` is renamed `sonic`: `exts."oc.scenario.warehouse".controller = "sonic"` (`pixi run warehouse --controller sonic`) selects SONIC whole-body control.

## [0.4.0] - 2026-10-08
### Changed
- The G1's ROS 2 topics are off by default: `open_scenario()` deactivates `/World/G1/Graph` (camera, lidar, TF, IMU, joint states, odometry, joint commands) unless the setting `exts."oc.scenario.warehouse".ros` is true (`pixi run warehouse --ros`). `ros()` reads it.

## [0.3.0] - 2026-10-08
### Added
- Setting `exts."oc.scenario.warehouse".controller` (`walk` or `wbc`) and `controller()`. With `wbc`, the G1 follows `WAREHOUSE_SEQUENCE` with SONIC whole-body control (`oc.control.g1_sonic`): it walks the warehouse loop and side-steps into aisle 4.
- `open_scenario()` sets the physics rate the controller needs (100 Hz walk, 200 Hz wbc).

## [0.2.0] - 2026-10-06
### Changed
- `WAREHOUSE_WALK_SEQUENCE` is now `WAREHOUSE_WALK_TRAJECTORY`, a looping Walk trajectory the G1 follows with feedback.

## [0.1.0] - 2026-10-05
### Added
- Initial version: `open_scenario()` opens `usd/warehouse_g1_scene.usda`, and the G1 walks
  `WAREHOUSE_WALK_SEQUENCE` on Play.
