# Changelog

## [0.8.0] - 2026-10-17
### Added
- `exts."oc.scenario.warehouse".obstacles` (`pixi run warehouse -o clutter,forklift`) shows the Scenario's hidden clutter and forklift. `obstacles()` reads it and `apply_obstacles(stage, kinds)` activates the `OBSTACLES` paths in the session layer; `open_scenario()` calls both.

## [0.7.0] - 2026-10-15
### Added
- `/World/G1/Graph/ROS2_CmdVel` (`ROS2SubscribeTwist` on `/cmd_vel` + a message `Counter`) and `CmdVelBridge`, which forwards each new `/cmd_vel` to `G1Sonic.command()`.

### Changed
- The G1 always uses SONIC (`oc.control.g1_sonic`'s `G1Sonic`) in walk mode at `/cmd_vel`, and physics always runs at 200 Hz. Without a `/cmd_vel` publisher it stands.
- ROS is on by default (`exts."oc.scenario.warehouse".ros = true`); `--ros` is replaced by `--no-ros`, with which the G1 stands.

### Removed
- `WAREHOUSE_SEQUENCE`, `WAREHOUSE_WALK_TRAJECTORY`, `controller()`, `CONTROLLERS`, the `controller` setting and `--controller`; `oc.control.g1_walk` is no longer a dependency.

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
