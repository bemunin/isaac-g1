# isaac-g1

Simulates the Unitree G1 in Isaac Sim with SONIC whole-body control, driven over ROS 2 across selectable middlewares, built by Omnicraft.

## Language

### Middleware

**DDS**:
The selectable ROS 2 middleware layer for a session: Fast DDS, Cyclone DDS, or Zenoh. Used as deliberate shorthand even though Zenoh is not a DDS.
_Avoid_: RMW (except when naming the `RMW_IMPLEMENTATION` value)

### Extensions

**Omnicraft**:
The organization that owns this project and its agent skills; abbreviated "oc" in skill names.

**Coding agent**:
An AI assistant that works in this repo by following `AGENTS.md` and its skills, e.g. Claude Code or Codex.
_Avoid_: LLM, AI, bot

**Extension**:
An Isaac Sim / Omniverse Kit extension: a folder with a `config/extension.toml` that Kit loads.
_Avoid_: plugin, add-on, ext (in prose)

**Extension ID**:
An Extension's name, `<organization>.<group>.<ext_name>`, e.g. `oc.scenario.warehouse`.
_Avoid_: brand.category.name

**Organization**:
The first segment of an Extension ID: who owns the Extension. Not fixed to Omnicraft.
_Avoid_: brand, company

**Group**:
The middle segment of an Extension ID: the project area the Extension belongs to.
_Avoid_: category, area

**Category**:
The Extension Manager category in an Extension's `extension.toml` (e.g. Simulation, Rendering). Unrelated to Group.

**Metadata**:
The descriptive fields of an Extension shown in the Extension Manager: title, description, category, keywords, authors, repository, license. Where the Extension lives is not Metadata.
_Avoid_: other fields, package info

### Robot motion

**MotionGen**:
Isaac Sim's experimental motion-generation library, whose controllers turn a robot's estimated state and a setpoint into a desired robot state.
_Avoid_: motion gen (as an unnamed generic), cuRobo MotionGen

**MotionGen controller**:
A controller built on MotionGen's controller interface. The Whole-body controller is not one.
_Avoid_: experiment motion controller, motion controller

**Policy**:
A pretrained neural network that maps a robot's observations to joint actions, e.g. NVIDIA's SONIC.
_Avoid_: model (alone), checkpoint

**Whole-body controller**:
The G1's SONIC-based controller: it turns Locomotion commands into joint targets for the whole body (legs, waist and arms).
_Avoid_: WBC (in prose), SONIC controller

**Locomotion command**:
The one input to the Whole-body controller's motion: a SONIC mode, a velocity in the robot's frame (forward, sideways, turn rate) and a pelvis height. Comes over ROS 2 or from a Python caller and, later, Nav2.
_Avoid_: cmd_vel (except for the ROS 2 message), twist, maneuver

**Command source**:
Where a run's Locomotion commands come from: ROS 2 or a Python caller in Isaac Sim's process.
_Avoid_: communication type, channel, interface

**SONIC mode**:
One of the 27 kinds of motion SONIC's kinematic planner produces, each either a Gait or a Posture.
_Avoid_: mode (alone), planner mode

**Gait**:
A SONIC mode that moves the robot at the command's velocity, e.g. slow walk, walk, run or a crawl.
_Avoid_: speed level

**Posture**:
A SONIC mode held in place, optionally at a pelvis height, e.g. idle (standing), squat or kneel.
_Avoid_: pose, stance

**Real-time control**:
Issuing Locomotion commands one at a time while the simulation runs. A new command replaces the current one.
_Avoid_: teleop, manual control, live control

### Robots

**Spawn**:
To add a robot asset to the current stage at a prim path.
_Avoid_: load, import, add (for robots)

**Multi-tick sensors**:
A robot's sensors, each running at its own tick rate instead of all updating every frame.
_Avoid_: multi tick sensor, sensor rate

### Scenarios and workflows

**Scenario**:
A USD stage holding the environment and the physics scene settings; its robots are Spawned into it when it opens.
_Avoid_: scene (for the whole stage), world

**Extension workflow**:
Running a Scenario by launching Isaac Sim with its Extension, which opens the Scenario and, with Autoplay, plays it. `ext` on the command line.
_Avoid_: ext workflow (in prose), app workflow

**Autoplay**:
Playing a Scenario's simulation as soon as the Scenario opens, instead of waiting for the user to press Play. On by default when launching from the command line; the user can opt out to press Play themselves.
_Avoid_: autostart (that opens the Scenario), play on start

**Standalone workflow**:
Running a Scenario from a Python script that starts Isaac Sim itself and owns the app loop. `standalone` on the command line.
_Avoid_: script workflow, headless workflow

**Physics engine**:
The simulator that steps physics for a session: PhysX or Newton. Each command sets its own default.
_Avoid_: engine (alone), backend
