# Changelog

## [0.2.0] - 2026-10-06
### Added
- `G1WalkController`: MotionGen controller that follows a Walk trajectory and runs the walking policy, with orientation and angular velocity from the IMU and position from the articulation root.
- `WalkPlan.to_trajectory()`: the `WalkTrajectory` of `WalkGoal`s that `G1Walker.execute()` follows; Walk primitives include `walk_to` for planners' world poses.
- `G1Walker.execute()`, `status`, `on_done`; Kit unit tests.

### Changed
- `to_trajectory()` replaces `to_walk_sequence()`; `G1Walker(prim_path)` no longer takes a sequence.
- `WalkPlan.stop(seconds)` is now `wait(seconds)`, and `abort()` is now `stop()`.
- Primitives take an optional `speed=` (m/s, deg/s for turns; default 0.5 m/s and 45°/s, up to the policy's 1 m/s and 1 rad/s), kept per Walk goal.
- The policy reads orientation and angular velocity from the IMU; the policy is no longer a separate controller.

### Removed
- Walk sequences: `command_at`.

## [0.1.0] - 2026-10-01
### Added
- G1 walking with Unitree's pretrained policy through a MotionGen controller, on a repeating square loop.
