# isaac-g1

Benchmarks Isaac Sim's ROS 2 communication across ROS 2 middlewares, built by Omnicraft.

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
An Extension's name, `<organization>.<group>.<ext_name>`, e.g. `oc.benchmark.ros_dds`.
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
A controller built on MotionGen's controller interface. The G1's Walk controller is one.
_Avoid_: experiment motion controller, motion controller

**Policy**:
A pretrained neural network that maps a robot's observations to joint actions, e.g. Unitree's G1 walking policy.
_Avoid_: model (alone), checkpoint

**Walk controller**:
The G1's MotionGen controller: it follows a Walk trajectory from the robot's measured state (orientation and body rates from its IMU, position from the simulation) and turns it into joint targets with the walking Policy.
_Avoid_: mid-level controller, locomotion controller, path follower

**Walk primitive**:
One walk command: move forward or backward, step left or right, turn left or right (each by an amount), wait (stand still for a time), all relative to the robot; or walk to a pose in the world; or stop (end the plan).
_Avoid_: action, move, waypoint

**Walk goal**:
A pose the robot must reach (position and heading) at a given speed, then hold for a given time.
_Avoid_: waypoint, target

**Walk plan**:
An ordered list of Walk primitives that generates a Walk trajectory.
_Avoid_: walk path, route, script

**Walk trajectory**:
An ordered list of Walk goals, built from Walk primitives, that a Walk controller follows, either relative to the robot's pose when it starts or in the world. Each goal is approached at its primitive's speed. A looping Walk trajectory repeats from its first goal, measured from the same start pose, so every lap is the same.
_Avoid_: walk path, walk sequence, route

### Robots

**Spawn**:
To add a robot asset to the current stage at a prim path.
_Avoid_: load, import, add (for robots)

**Multi-tick sensors**:
A robot's sensors, each running at its own tick rate instead of all updating every frame.
_Avoid_: multi tick sensor, sensor rate

### Benchmarks

**Benchmark**:
A named, repeatable run selected by name (e.g. `dds`) that opens a Scenario under one Physics engine and DDS.
_Avoid_: test, experiment

**Scenario**:
A USD stage a Benchmark opens, holding the environment, the physics scene settings and its robots already placed.
_Avoid_: scene (for the whole stage), world

**Extension workflow**:
Running a Benchmark or a Scenario by launching Isaac Sim with its Extension, which opens the Scenario and plays. `ext` on the command line.
_Avoid_: ext workflow (in prose), app workflow

**Standalone workflow**:
Running a Benchmark or a Scenario from a Python script that starts Isaac Sim itself and owns the app loop. `standalone` on the command line.
_Avoid_: script workflow, headless workflow

**Physics engine**:
The simulator that steps physics for a session: PhysX (default) or Newton.
_Avoid_: engine (alone), backend
