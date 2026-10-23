# oc_sonic

NVIDIA SONIC (kinematic planner + tracking policy) for the Unitree G1 as two composable C++ nodes, ported from the
`oc.control.g1_sonic` Python reference. Messages are in `oc_sonic_msgs`.

```bash
pixi run sonic-download   # weights -> ros_ws/src/oc_sonic/data/sonic (git-ignored)
pixi run build
ros2 launch oc_sonic sonic.launch.py   # model_dir:=... use_sim_time:=true
```

| Node | In | Out |
|------|----|-----|
| `PolicyNode` (`/policy`) | `/joint_states`, `/imu/data`, `planned_motion` | `/joint_commands` (50 Hz, MuJoCo joint order), `planner_context` |
| `PlannerNode` (`/planner`) | `/cmd_vel`, `/facing_direction` (Float64, world yaw; NaN clears), `planner_context`, `~/set_mode` | `planned_motion` |

Both run in one `component_container_mt` with intra-process comms, each in its own callback group. The planner's ONNX
model takes ~75 ms on the CPU, so the policy never waits for it: the policy owns the blended motion and an absolute
frame counter, publishes the context to plan from, and splices each plan in by its start frame (late plans skip frames).

`~/set_mode` (`oc_sonic_msgs/srv/SetMode`): SONIC planner mode 0 idle, 1 slow walk, 2 walk (default), 3 run, 4 squat,
5 kneel two legs, 6 kneel one leg, 8 hand crawl, 14 elbow crawl, with a pelvis height for 4-6. The walking and crawling
modes follow `/cmd_vel` (stopping after `cmd_vel_timeout`, 0.5 s).

## Tests

```bash
cd ros_ws && colcon test --packages-select oc_sonic --merge-install && colcon test-result --verbose
```

`test/data/*.f64` are goldens from the Python reference: regenerate with
`.pixi/envs/sim/bin/python ros_ws/src/oc_sonic/test/make_goldens.py`. The ONNX tests skip without the weights.
`test/fake_sim.py` stands in for Isaac Sim (`/clock`, `/joint_states`, `/imu/data` at 100 Hz) to try the nodes.

Planner replan rules, the 8-frame blend and the observation layouts follow NVlabs/GR00T-WholeBodyControl
`gear_sonic_deploy` (Apache-2.0).
