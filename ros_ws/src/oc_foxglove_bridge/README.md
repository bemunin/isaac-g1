# oc_foxglove_bridge

Starts the Foxglove bridge (`ws://localhost:8765`) and `robot_state_publisher` with the G1's URDF, both on
sim time. With a Scenario playing, Isaac Sim publishes `/joint_states` and `odom → pelvis`, and Foxglove's 3D
panel shows the G1 from `/robot_description`.

```bash
pixi run build
pixi run foxglove
```

## Vendored G1 description

`urdf/g1_29dof_with_hand_rev_1_0.urdf` and `meshes/` come from
[unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros/tree/5994d4faef0a9cadd3287f8de0199a67eeb2a259/robots/g1_description)
at `5994d4f`, under Unitree's BSD-3-Clause license (`LICENSE.unitree`). Only the meshes this URDF uses are
copied. Two changes from upstream: mesh paths are `package://oc_foxglove_bridge/meshes/...`, and mesh files
end in `.stl` instead of `.STL`, because `foxglove_bridge`'s default `asset_uri_allowlist` only matches
lowercase extensions.
