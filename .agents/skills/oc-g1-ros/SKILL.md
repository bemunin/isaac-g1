---
name: oc-g1-ros
description: Load and work with the Unitree G1 ROS 2 sample asset (G1_ROS.usd) in this repo. Use when adding the G1 to a stage, touching its ROS 2 graphs, topics or IMU, or adding sensors (D435 camera, MID-360 lidar) and setting their tick rates.
---

# oc-g1-ros

The repo's G1 is NVIDIA's `Samples/ROS2/Robots/G1_ROS.usd`: the Multiphysics 29-DOF G1 with hands,
plus one IMU and four ROS 2 OmniGraphs. Background and line-cited sources:
`docs/report-g1-isaacsim-assets.md`, `docs/research-g1-sensors-isaacsim.md`.

## 0. Match the Isaac Sim version

Every path, URL and API fact in this skill is written for **Isaac Sim 6.1** (the repo pins
`isaacsim==6.1.0.0` in `pixi.toml`). Before using any of them, find the version the user is on:
what they state, else the `isaacsim` pin in `pixi.toml`, else
`.pixi/envs/sim/lib/python3.12/site-packages/isaacsim/VERSION`. If it is not 6.1, rewrite the
version segment everywhere you use it, in code and in committed layers:

| Where | 6.1 form | Rewrite to |
|---|---|---|
| Asset URLs | `.../Assets/Isaac/6.1/Isaac/...` | `.../Assets/Isaac/<major.minor>/Isaac/...` (e.g. `6.0`) |
| Local mirror | `~/mnt/vault/nvsim/isaacsim_assets/Assets/Isaac/6.1/...` | same `<major.minor>` |
| Docs URLs | `docs.isaacsim.omniverse.nvidia.com/6.1.0/...` | `<major.minor.patch>` (e.g. `6.0.0`) |

Then confirm the asset exists at that version before referencing it:
`curl -sI <G1_ROS.usd URL> | head -1` must return `200`. `G1_ROS.usd` exists for 6.0 and 6.1 but
**not 5.1** (404); stop and tell the user if it is missing. Also re-check version-specific facts
against the installed package rather than trusting this skill: the RealSense catalog (§2), API
signatures (`RtxCamera`, `create_ros2_camera_graph`), and the node attributes in
`site-packages/isaacsim/exts/isaacsim.ros2.nodes/ogn/docs/`.

## 1. Use the G1_ROS.usd asset

### Where it lives

| Copy | Path |
|---|---|
| Remote (what stages reference) | `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/Isaac/Samples/ROS2/Robots/G1_ROS.usd` |
| Local mirror (read-only, for inspection) | `~/mnt/vault/nvsim/isaacsim_assets/Assets/Isaac/6.1/Isaac/Samples/ROS2/Robots/G1_ROS.usd` |

Reference the remote URL in committed layers, matching `sim/exts/oc.benchmark.ros_dds/usd/simple_g1_scenario.usda`. The file
is binary; to read it, export text with
`Sdf.Layer.FindOrOpen(path).ExportToString()` into the scratchpad.

### Add it to a stage

Reference the asset's defaultPrim (`g1_29dof_with_hand_rev_1_0`) onto `/World/G1`:

```usda
def Xform "World"
{
    def "G1" (
        prepend references = @https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/Isaac/Samples/ROS2/Robots/G1_ROS.usd@
    )
    {
    }
}
```

```python
from pxr import Usd
G1_ROS_URL = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/Isaac/Samples/ROS2/Robots/G1_ROS.usd"
prim = stage.DefinePrim("/World/G1", "Xform")
prim.GetReferences().AddReference(G1_ROS_URL)
```

The stage also needs a `PhysicsScene` stepping at 100 Hz for both engines
(`newton:timeStepsPerSecond` and `physxScene:timeStepsPerSecond`), since three graphs tick on
`OnPhysicsStep`. Copy the `PhysicsScene` prim from `simple_g1_scenario.usda`. The asset already
lifts the robot to standing height (root translate z ≈ 0.792 m), so leave `/World/G1` at the origin
unless placing it elsewhere.

### What the reference gives you

Links are siblings under the robot root, not nested: `/World/G1/pelvis`, `/World/G1/torso_link`,
`/World/G1/left_ankle_roll_link`. The articulation root is `pelvis`.

| Prim | What it is |
|---|---|
| `/World/G1/pelvis/imu_sensor` | `IsaacImuSensor` at the pelvis origin |
| `/World/G1/torso_link/d435_link`, `mid360_link`, `imu_in_torso`; `/World/G1/pelvis/imu_in_pelvis` | Plain Xform mount frames matching Unitree's URDF; parent new sensors here |
| `/World/G1/Graph/ROS2_JointStates` | `OnPhysicsStep` → publishes `/joint_states` (asset: `/isaac_sim_joint_states`) |
| `/World/G1/Graph/ROS2_JointCommands` | `OnPhysicsStep` → subscribes `/joint_commands` (asset: `/isaac_sim_joint_commands`) → `IsaacArticulationController` |
| `/World/G1/Graph/ROS2_IMU` | `OnPhysicsStep` → publishes `/imu/data` (asset: `/isaac_sim_imu`), `frame_id` `imu` |
| `/World/G1/Graph/OdometryGraph` | `OnPlaybackTick` → publishes `/odom` and `/tf` (`odom → pelvis` only) |

**Topic names.** The scenario renames the asset's `isaac_sim_*` topics to standard ROS names
through `over`s on each node's `inputs:topicName`; the `isaac_sim_` prefix is specific to this asset,
not an NVIDIA convention. Sensors keep their vendor drivers' default names for sim-to-real parity.
For multiple robots, namespace with `isaac:namespace` on the robot root rather than a prefix.
Rationale: `docs/research-g1-ros-topic-names.md`.

Absent from the asset: camera, lidar, contact sensors, `/clock`, and a link TF tree (the `imu` frame
is in no TF tree). The `Sensor = "sensors"` variant is an empty payload and adds none of these.

### Gotchas

- **`ArticulationController.inputs:robotPath` is a string**, `"/g1_29dof_with_hand_rev_1_0/pelvis"`.
  Referencing remaps relationships, not strings, so under `/World/G1` it points nowhere. Its
  `targetPrim` relationship is remapped correctly. Author an `over` setting `robotPath` to
  `/World/G1/pelvis` when `/joint_commands` must drive the robot.
- **`ROS2_JointCommands` competes with any in-sim controller** (e.g. the `oc.control.g1_walk`
  walker) as soon as something publishes `/joint_commands`. Keep one owner of the joint
  targets.
- **`ROS2_JointStates` logs** "Reading from targetPrim is deprecated". It is a warning; the topic still
  publishes.
- **Edit the asset through `over`s in the scenario layer**, never by saving into the asset. Changes to
  graph nodes target `/World/G1/Graph/<graph>/<node>`.

## 2. Sensors

Parent each new sensor under its asset mount frame (§1, "What the reference gives you") so it rides
the right link with Unitree's URDF offsets. Author sensors and their graphs while the timeline is
**stopped**, then save the stage so they land as `over`/`def` opinions in the scenario layer
(`sim/exts/oc.benchmark.ros_dds/usd/simple_g1_scenario.usda`). Put each sensor's graph beside the asset's, at
`/World/G1/Graph/ROS2_<Sensor>`.

### Multi-tick sensors

Isaac Sim 6.x renders each RTX sensor at its own rate ("multi-tick rendering", on by default via
`/rtx/hydra/supportMultiTickRate=true`). Source:
<https://docs.isaacsim.omniverse.nvidia.com/6.1.0/sensors/isaacsim_sensors_multitick_rendering.html>.

- **Rate lives on the sensor prim**: `omni:sensor:tickRate` (Hz) from `OmniSensorAPI`, on `Camera`
  and `OmniLidar` prims; `0` = autotrigger, render every frame. Set it with
  `RtxCamera(path, tick_rate=...)` / `Lidar.create(..., tick_rate=...)`.
- **ROS helpers inherit it.** `ROS2CameraHelper`, `ROS2CameraInfoHelper` and `ROS2RtxLidarHelper`
  read `tickRate` from the prim. Leave their `frameSkipCount` at `0`: it is deprecated and logs a
  warning when non-zero.
- **Sensors tick on physics time.** The helpers fire on `OnPlaybackTick`, but a sensor renders only
  when `/ExternalSimulationTime` (written on each physics step) has advanced. Timestamps come from
  `IsaacReadSimulationTime`, never `omni.timeline`.
- **In this scenario** physics runs at 100 Hz and the loop at 60 Hz (`timeCodesPerSecond`), so
  physics dt < loop dt and sensors tick on schedule. Pick rates that divide 60 (30, 20, 15, 10 Hz).
- **The rate is guaranteed in sim time, not wall time.** With fixed time stepping (the GUI default),
  a loop running below 60 Hz wall time plays in slow motion, and topics publish slower in wall time
  by the same ratio. Measure with `ros2 topic hz`; this matters for DDS Benchmarks.
- **Lidar**: `tickRate` must equal `omni:sensor:Core:scanRateBaseHz`. A mismatch gives silent
  per-frame partial scans.
- **Debug**: launch with `--/rtx/hydra/supportMultiTickRate=false` to render every sensor every frame.

### Head RGB-D camera (RealSense D435)

Isaac Sim 6.1 ships no D435 asset or config (only D455, D457, D555 under
`Isaac/Sensors/RealSense/`). Use a plain RTX camera with the D435 colour intrinsics; colour and depth
come from the same camera, so depth is aligned to colour by construction.

**Before authoring, ask the user** which resolution to publish: **640×360** or **1280×720**. Both are
16:9, so the optics are identical; only the render product size changes, and 1280×720 carries 4× the
image payload of 640×360.

| Setting | Value | Why |
|---|---|---|
| Camera prim | `/World/G1/torso_link/d435_link/camera` | under the asset's D435 mount (pitched 47.6° down) |
| Local orient (wxyz) | `(0.5, 0.5, -0.5, -0.5)` | maps the USD camera's −Z view axis to the link's +X, +Y up to +Z |
| `focalLength` / `horizontalAperture` / `verticalAperture` | `15.24` / `20.955` / `11.787` | 69° × 42° colour FOV; the FOV depends only on their ratios |
| `clippingRange` | `(0.1, 20)` | |
| `omni:sensor:tickRate` | `30` | D435 colour rate; divides the 60 Hz loop |
| Graph | `/World/G1/Graph/ROS2_Camera` | |
| rgb topic | `/camera/camera/color/image_raw` | `realsense2_camera` driver names, so ROS code runs unchanged on the robot |
| depth topic | `/camera/camera/aligned_depth_to_color/image_raw` | |
| camera_info topics | `/camera/camera/color/camera_info`, `/camera/camera/aligned_depth_to_color/camera_info` | same intrinsics; the driver publishes both and `depth_image_proc` wants info beside the depth image (`DepthCameraInfoPublish`) |
| `frame_id` | `camera_color_optical_frame` | |

```python
from isaacsim.sensors.experimental.rtx import RtxCamera
from isaacsim.ros2.nodes import Ros2CameraGraphConfig, create_ros2_camera_graph
import omni.graph.core as og

WIDTH, HEIGHT = 640, 360  # or 1280, 720 -- the user's choice

cam = RtxCamera("/World/G1/torso_link/d435_link/camera", tick_rate=30.0)
cam.set_local_poses(translations=[[0.0, 0.0, 0.0]], orientations=[[0.5, 0.5, -0.5, -0.5]])
cam.camera.set_focal_lengths(15.24)
cam.camera.set_apertures(horizontal_apertures=20.955, vertical_apertures=11.787)
cam.camera.set_clipping_ranges(0.1, 20.0)

graph = create_ros2_camera_graph(Ros2CameraGraphConfig(
    graph_path="/World/G1/Graph/ROS2_Camera",
    camera_prim="/World/G1/torso_link/d435_link/camera",
    frame_id="camera_color_optical_frame",
    rgb_topic="/camera/camera/color/image_raw",
    depth_topic="/camera/camera/aligned_depth_to_color/image_raw",
    camera_info_topic="/camera/camera/color/camera_info",
))
og.Controller.attribute(f"{graph}/RenderProduct.inputs:width").set(WIDTH)
og.Controller.attribute(f"{graph}/RenderProduct.inputs:height").set(HEIGHT)
```

Then save the stage. Done when, with the scenario playing, `ros2 topic list` shows the three topics
and `ros2 topic hz /camera/camera/color/image_raw` reads about 30 Hz (lower if the loop runs below
real time; see Multi-tick sensors).

Gotchas:

- **`RtxCamera(orientations=...)` is world-frame.** Set the mount-relative rotation with
  `set_local_poses`, as above.
- **`create_ros2_camera_graph` stops the timeline** and builds `OnPlaybackTick → RunOnce →
  RenderProduct (IsaacCreateRenderProduct) → helpers`. Its config has no resolution field; the
  render product defaults to 1280×720 until you set `width`/`height`.
- **Depth differs from the real driver**: Isaac publishes `32FC1` metres; `realsense2_camera`
  publishes `16UC1` millimetres on the same topic.
- **The camera frame comes from TF's name override.** `ROS2_TF` publishes the camera prim as
  `camera_color_optical_frame` (see TF tree); keep the helpers' `frameId` equal to it.

### Head lidar (Livox MID-360)

Isaac Sim 6.1 ships no Livox asset or config, so the scenario defines a custom rotary `OmniLidar` sized
to the [MID-360 spec](https://www.livoxtech.com/mid-360/specs): 360° × −7°…52°, 200 000 pts/s,
10 Hz, 0.1–40 m. Its rosette (non-repetitive) pattern is not modelled; the point budget, FOV and
message size match, which is what a DDS Benchmark measures.

| Setting | Value | Why |
|---|---|---|
| Lidar prim | `/World/G1/torso_link/mid360_link/lidar` (`OmniLidar` + `OmniSensorGenericLidarCoreAPI`) | |
| Local orient (wxyz) | `(0, 1, 0, 0)` | roll 180°: the real G1 mounts it upside down (Unitree URDF), so it sees the ground; the asset's `mid360_link` is upright |
| Emitters | 40, elevations evenly spaced −7°…52°, azimuth 0, `scanType` `ROTARY` | |
| `patternFiringRateHz` | `5000` | 500 azimuth steps/rev × 40 = 20 000 pts/frame |
| `scanRateBaseHz` / `omni:sensor:tickRate` | `10` / `10` | must be equal |
| `nearRangeM` / `farRangeM` / `maxReturns` | `0.1` / `40` / `1` | spec: 40 m @ 10 % reflectivity, first return |
| Graph | `/World/G1/Graph/ROS2_Lidar` | |
| topic | `/livox/lidar`, `PointCloud2`, xyz + intensity (16 B/point) | `livox_ros_driver2` default name |
| `frame_id` | `livox_frame` | `livox_ros_driver2` default |

```python
from isaacsim.ros2.nodes import Ros2RtxLidarGraphConfig, create_ros2_rtx_lidar_graph

create_ros2_rtx_lidar_graph(Ros2RtxLidarGraphConfig(
    graph_path="/World/G1/Graph/ROS2_Lidar",
    lidar_prim="/World/G1/torso_link/mid360_link/lidar",
    frame_id="livox_frame",
    publish_laser_scan=False,
    publish_point_cloud=True,
    point_cloud_topic="/livox/lidar",
    metadata={"Intensity"},
))
```

Done when, with the scenario playing, `ros2 topic echo /livox/lidar --no-arr --field header.stamp`
steps by 0.1 s and a message carries up to 20 000 points (rays that hit nothing are dropped, so
about 12 500 at the start pose).

Gotchas:

- **The rotary firing rate is `patternFiringRateHz` in 6.1.** NVIDIA's `Example_Rotary.usda` still
  authors the old `reportRateBaseHz`, which the schema no longer declares (it saves as `custom`) and
  the sensor ignores, falling back to 36 000 Hz.
- **The lidar validator wants an `OmniLidar` prim** with `OmniSensorGenericLidarCoreAPI`; a
  Camera-typed lidar is rejected.
- **Saving the stage from Kit adds default attributes** to untouched graph nodes (e.g. `ROS2_Clock`).
  Drop those hunks before committing.
- **Point clouds are in the lidar prim's frame**, roll included, which `ROS2_TF` publishes as
  `livox_frame`.

### TF tree

The asset publishes only `odom → pelvis`. The scenario adds the rest, so every sensor frame
resolves from `world`:

```
world ─(/tf_static)→ odom ─(/tf, asset OdometryGraph)→ pelvis ─(/tf, ROS2_TF)→ camera_color_optical_frame
                                                                               livox_frame
                                                                               imu
```

| Piece | How |
|---|---|
| Frame names | `isaac:nameOverride` on `/World` (`world`), `.../d435_link/camera`, `.../mid360_link/lidar`, `pelvis/imu_sensor` (`imu`); frame names are prim names otherwise |
| `world → odom` | `/World/G1/odom`, a plain Xform on the ground under the pelvis's start pose: start x, y and yaw, z = 0 (REP-120; Nav2 height filters assume it). Rationale: `docs/research-g1-odom-frame.md` |
| `odom → pelvis` height | `IsaacComputeOdometry` reports pose relative to the pelvis's start pose and has no offset input. `OdometryGraph/OdomOffset` (`omni.graph.nodes.Add`) adds `(0, 0, 0.7929)`, the pelvis start z, and feeds both `PublishOdom.position` and `PublishTF.translation` (scenario `delete`/`prepend` over the asset's connections), so `/odom` and `/tf` agree. The start tilt (~1e-5 rad) is ignored |
| Graph | `/World/G1/Graph/ROS2_TF`, `OnPlaybackTick`, sim-time stamps: `ComputeTF`/`PublishTF` (parent `pelvis`, three targets, `/tf`) and `ComputeStaticTF`/`PublishStaticTF` (parent `/World`, target `odom`, `/tf_static`, `staticPublisher = 1`) |

`IsaacComputeTransformTree` applies the camera's optical rotation itself, so the camera prim *is* the
optical frame. Sensor frames hang off `pelvis`, not their mount links (only the TF tree's sensor
frames, not the 30 links, travel on `/tf`).

Done when `world → odom` has z 0, `odom → pelvis` and `/odom` have z ≈ 0.76 m while walking, and `tf2_ros` resolves `world` to each of `camera_color_optical_frame` (z ≈ 1.23 m standing),
`livox_frame` (z ≈ 1.22 m) and `imu`.

Gotchas:

- **Set graph inputs in USD, not only through `og.Controller.attribute(...).set`.** Values set that
  way (a `targetPrims` list, `staticPublisher`) did not reach the saved layer; check the `.usda`.
- **`append_to_existing_tf_node` edits the publisher's `targetPrims`,** which the publisher ignores
  once `parentFrames` is connected. Add targets to the compute node instead.
- **One ROS domain per sim.** A second Isaac Sim on the same `ROS_DOMAIN_ID` interleaves `/clock` and
  `/odom` stamps and makes TF lookups fail with extrapolation errors. Test with a spare domain.
