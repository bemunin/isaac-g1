# Walk trajectories are followed with IMU heading and simulated root position

> Superseded: the Walk controller and Walk trajectories were removed; the G1 is driven only by Nav2's `cmd_vel`
> through SONIC ([ADR 0003](0003-sonic-walk-from-nav2-cmd-vel.md)). SONIC still reads the IMU and the simulated
> root position as described here.

## Decision

The Walk controller in `oc.control.g1_walk` reads the robot's state from two sources:

- **Orientation and body angular rate**: the pelvis IMU, as on the real robot. The walking Policy reads the same IMU.
- **Position (x, y)**: the articulation root, which is simulation ground truth.

## Why

- **Not IMU-only**: an IMU gives orientation and angular rate, but position from double-integrated
  acceleration drifts within seconds. Distance and sideways error could not be corrected, so loops would drift.
- **Not `/odom` over ROS 2**: it would match a real robot's stack, but couple the controller to the ROS
  bridge and to the DDS under Benchmark.

## Consequences

- Sim-to-real: on hardware the position source must be replaced by an odometry or localization estimate.
- The ROS 2 planner adapter (later) feeds Walk trajectories in; it does not feed pose back.
