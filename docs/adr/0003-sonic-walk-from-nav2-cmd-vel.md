# SONIC is the only controller, driven by Nav2's cmd_vel in walk mode

## Decision

The G1 has one controller, `oc.control.g1_sonic`, and one input: a body-frame velocity, Nav2's `/cmd_vel`. It
always moves in SONIC's locomotion walk mode (planner mode 2) and stands in idle mode (0) below 0.1 m/s or 0.5 s
after the last command. Forward and backward speeds are clamped to 0.8-1.5 m/s (backward up to 0.8); strafing
stays up to 0.4 m/s. The warehouse Scenario subscribes to `/cmd_vel` with an OmniGraph `ROS2SubscribeTwist`
(`/World/G1/Graph/ROS2_CmdVel`) and forwards each message to `G1Sonic.command()`.

## Why

- **Nav2 shapes the motion.** Routes, recoveries and speed come from Nav2 (or teleop) over the topic every ROS
  tool understands. The controller only has to walk the velocity it is given.
- **Not Motion sequences** (code-defined routes, first inside each controller, then as a ROS 2 node publishing
  `cmd_vel`): a second route system next to Nav2, with its own format, follower and tests.
- **Not several gaits** (slow walk, walk, run picked by speed or a gait ceiling): `cmd_vel` carries no gait, and
  mode switches at speed boundaries made Nav2's slowdowns change gait. Walk covers Nav2's useful speed range.
- **Not the walking-policy controller** (`oc.control.g1_walk`): it moved only the legs and duplicated the
  controller API; SONIC is the default and moves the whole body.
- **OmniGraph, not rclpy in Kit, for `/cmd_vel`**: the G1's other ROS 2 topics already use OmniGraph in the
  same ROS context, and a Twist needs no custom types or threads. Arrival is counted in the graph and stamped in
  the physics step, so the 0.5 s timeout runs on simulation time.

## Consequences

- No slow walk or run, and nothing below 0.8 m/s forward: Nav2's approach speeds must be at least 0.8 m/s or the
  robot overshoots (e.g. Regulated Pure Pursuit `min_approach_linear_velocity`).
- Backward walking (for Nav2's `BackUp`) relies on SONIC's walk mode moving opposite its facing, as the
  keyboard's S key does.
- The robot moves only with ROS on and a `/cmd_vel` publisher; otherwise it stands. Benchmarks now measure a
  standing G1, so they are not comparable with earlier runs.
- Other planner modes (squat, kneel, styled walks) need a new input beside `cmd_vel`; see
  `research/reports/research-g1-sonic-ros2-behaviors.md`.
