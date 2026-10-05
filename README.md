# isaac-g1

A Unitree G1 in Isaac Sim 6.1 talking ROS 2 Jazzy over a selectable DDS.

## Prerequisites

- [Pixi](https://pixi.sh)
- Linux (glibc ≥ 2.35, e.g. Ubuntu 22.04+) or Windows
- An NVIDIA RTX GPU that meets [Isaac Sim's requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)
- A coding agent such as Claude Code (for setup and the project skills)
- [Foxglove](https://foxglove.dev) (optional, to view topics)

## Setup

1. Install Pixi and clone this repo.
2. Open a coding agent in the repo root and run `/init`.

`/init` installs the Pixi environments and links the agent skills and the Isaac Sim package
(`.agents/skills/oc-init-proj/SKILL.md`).

## DDS

Pick the ROS 2 middleware once; new Pixi sessions use it.

| Command | Does |
| --- | --- |
| `pixi run dds [fast\|cyclone\|zenoh]` | Select the DDS (no argument shows the current one) |
| `pixi run zenoh` | Start the Zenoh router (needed for `zenoh`) |
| `pixi run foxglove` | Start the Foxglove bridge on `ws://localhost:8765` |
| `pixi run shell` | Open a ROS 2 shell in `ros_ws` |

## Warehouse

Opens the warehouse Scenario and plays it; the G1 walks a loop through the hall.

```bash
pixi run warehouse                   # Standalone workflow, Newton, with a window
pixi run warehouse --workflow ext    # Extension workflow
pixi run warehouse --physics physx   # PhysX instead of Newton
pixi run warehouse --headless        # no window
```

`--dds fast|cyclone|zenoh` overrides the session's DDS. See
[`sim/exts/oc.scenario.warehouse`](sim/exts/oc.scenario.warehouse/docs/README.md).

## Control

`oc.control.g1_walk` makes a G1 walk a Walk sequence with Unitree's walking Policy. A Scenario
starts a `G1Walker` with its own sequence, built from a `WalkPlan`; the warehouse's is in
`warehouse_walk_sequence.py`. See [`sim/exts/oc.control.g1_walk`](sim/exts/oc.control.g1_walk/docs/README.md).

## Skills

Ask your coding agent to use these (in Claude Code, `/<name>`):

- `oc-init-proj`: set up the repo (what `/init` runs).
- `oc-add-ext`: scaffold a new Isaac Sim extension.
- `oc-g1-ros`: add the G1 to a stage, work with its ROS 2 graphs, or add sensors.

The other skills in `.agents/skills/` are third-party Isaac Sim references. Agent rules live in
[`AGENTS.md`](AGENTS.md); project terms in [`GLOSSARY.md`](GLOSSARY.md).
