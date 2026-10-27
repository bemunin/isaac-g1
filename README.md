# isaac-g1

A Pixi-based Isaac Sim project for the Unitree G1, built for agentic AI workflows.
Connects to ROS 2 Jazzy over the DDS of your choice.

![G1 walking in the Isaac Sim warehouse](docs/images/isaac_g1_walk.gif)

## Prerequisites

- Isaac Sim 6.2.0. The project targets the newest Isaac Sim release first.
- [Pixi](https://pixi.sh)
- Linux (Ubuntu 22.04 / 24.04) or Windows
- An NVIDIA RTX GPU that meets [Isaac Sim's requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)
- A coding agent such as Claude Code (for setup and the project skills)
- [Foxglove](https://foxglove.dev) (optional, to view topics)

## Setup

1. Install Pixi and clone this repo.
2. Open a coding agent in the repo root and initialize the project:
   - Claude Code: type `/init`.
   - Codex: type `$oc-init-proj`.

This installs the Pixi environments, downloads the SONIC weights, links the Isaac Sim package (`_isaacsim`), and loads the project's required skills from Isaac Sim Skills.

> [!NOTE]
> - In Claude Code, `/init` follows `AGENTS.md`'s Init section: it sets up the project without 
>   creating a `CLAUDE.md`. Howevery, Codex has no such hook, so run the skill directly.

## Getting started

Run the warehouse example to see the G1 stand in the hall, and inspect its topics in Foxglove.

1. Start the sim with ROS 2 on. Isaac Sim opens the warehouse and, with `-a`, plays it; the
   G1 stands until a `/cmd_vel` arrives.

   ```bash
   pixi run warehouse --ros
   ```

2. In a second terminal, build `ros_ws` (once, and after changing its packages) and start the
   Foxglove bridge with the G1's robot description:

   ```bash
   pixi run build
   pixi run foxglove
   ```

3. In Foxglove, open a connection to `ws://localhost:8765` to see the G1 model, lidar, camera, TF
   and IMU topics.

The sim uses the session's DDS (`pixi run dds` shows it). If it is `zenoh`, start the router with
`pixi run zenoh` before steps 1 and 2. See [DDS](#dds) and [Warehouse](#warehouse) for the options.

## Control

Our project use **SONIC** ([`oc.control.g1_sonic`](sim/exts/oc.control.g1_sonic/docs/README.md)) to move the G1 with
NVIDIA SONIC whole-body control: slow walk, walk, run, side steps and turns, plus lower-body ground
moves — `squat`, `kneel`, `kneel_one_leg` and `crawl_hands`/`crawl_elbows`. Drive it one real-time
command at a time (`command(LocomotionCommand)`, `walk(cmd_vel)`, ...).
Download its weights once with `pixi run sonic-download`.

## Command over ROS

SONIC listens on these topics:

| Topic | Message | Sets |
| --- | --- | --- |
| `/sonic/mode` | `std_msgs/Int32` | Motion mode Id 0–26, e.g. `2` WALK, `4` SQUAT, `8` hand crawl |
| `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` forward and `linear.y` left [m/s], `angular.z` turn [rad/s] |
| `/sonic/pelvis_height` | `std_msgs/Float32` | Pelvis height [m] for ground poses; `-1` keeps the mode's default |

Example: make the G1 walk forward at 1 m/s:

```bash
pixi run ros2 topic pub --once /sonic/mode std_msgs/msg/Int32 "{data: 2}"
pixi run ros2 topic pub -r 100 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"
```
The full ROS 2 command examples are in oc-sonic-cmd [SKILL.md](.agents/skills/oc-sonic-cmd/SKILL.md), and all 27 sonic motion mode details are in [GUIDE.md](.agents/skills/oc-sonic-cmd/GUIDE.md).

> [!IMPORTANT]
> Your coding agent can drive the G1 too: load the `oc-sonic-cmd` skill in your coding agent and start asking it to walk, turn, squat or crawl.

## Warehouse

The warehouse example can run in many configurations, such as a different physics engine, workflow or headless mode:

```bash
pixi run warehouse                   # Extension workflow, Newton, SONIC, with a window
pixi run warehouse -s                # Standalone workflow
pixi run warehouse --physics physx   # PhysX instead of Newton
pixi run warehouse --headless        # no window
pixi run warehouse --ros             # publish the G1's ROS 2 topics and accept /cmd_vel (off by default)
pixi run warehouse -o clutter,forklift # Show floor clutter (boxes, crates, cones, signs) and the forklift; both are hidden by default.
```

The G1 uses SONIC whole-body control by default; download its weights once with
`pixi run sonic-download` first. `--dds fast|cyclone|zenoh` overrides the session's DDS. See
[`sim/exts/oc.scenario.warehouse`](sim/exts/oc.scenario.warehouse/docs/README.md).

## DDS

Pick the ROS 2 middleware once; new Pixi sessions use it.

| Command | Does |
| --- | --- |
| `pixi run dds [fast\|cyclone\|zenoh]` | Select the DDS (no argument shows the current one) |
| `pixi run zenoh` | Start the Zenoh router (needed for `zenoh`) |
| `pixi run shell` | Open a ROS 2 shell in `ros_ws` |
| `pixi run build` | Build the ROS 2 packages in `ros_ws` |
| `pixi run foxglove` | Start the Foxglove bridge and the G1 robot description on `ws://localhost:8765` (`oc_foxglove_bridge`) |

![Foxglove showing the G1 lidar, camera, TF and IMU topics](docs/images/showcase_foxglove_panel.png)


## Skills

Ask your coding agent to use these (in Claude Code, `/<name>`):

- `oc-init-proj`: install the Pixi envs and create the project symlinks; `/init` in Claude Code, `$oc-init-proj` in Codex.
- `oc-add-ext`: scaffold a new Isaac Sim extension.
- `oc-g1-ros`: add the G1 to a stage, work with its ROS 2 graphs, or add sensors.
- `oc-sonic-cmd`: command the G1 with any of the 27 SONIC motion modes (walk, run, squat, kneel, crawl, ...) over ROS 2.

The other skills in `.agents/skills/` are third-party Isaac Sim references. Agent rules live in
[`AGENTS.md`](AGENTS.md); project terms in [`GLOSSARY.md`](GLOSSARY.md).