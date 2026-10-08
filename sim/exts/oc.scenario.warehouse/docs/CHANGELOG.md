# Changelog

## [0.4.0] - 2026-10-08
### Changed
- The G1's ROS 2 topics are off by default: `open_scenario()` deactivates `/World/G1/Graph` (camera, lidar, TF, IMU, joint states, odometry, joint commands) unless the setting `exts."oc.scenario.warehouse".ros` is true (`pixi run warehouse --ros`). `ros()` reads it.

## [0.3.0] - 2026-10-08
### Added
- Setting `exts."oc.scenario.warehouse".controller` (`walk` or `wbc`) and `controller()`. With `wbc`, the G1 follows `WAREHOUSE_MOVEMENT_PLAN` with SONIC whole-body control (`oc.control.g1_wbc_sonic`): it walks the warehouse loop and side-moves into aisle 4.
- `open_scenario()` sets the physics rate the controller needs (100 Hz walk, 200 Hz wbc).

## [0.2.0] - 2026-10-06
### Changed
- `WAREHOUSE_WALK_SEQUENCE` is now `WAREHOUSE_WALK_TRAJECTORY`, a looping Walk trajectory the G1 follows with feedback.

## [0.1.0] - 2026-10-05
### Added
- Initial version: `open_scenario()` opens `usd/warehouse_g1_scene.usda`, and the G1 walks
  `WAREHOUSE_WALK_SEQUENCE` on Play.
