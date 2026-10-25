# Scenarios step the Whole-body controller; SONIC planning runs off-thread inside it

Status: Accepted (2026-10-25). Partly supersedes 0002: the controller no longer builds on MotionGen and no longer
registers its own physics callback.

## Decision

`oc.control.g1_sonic` is rewritten from scratch as a library that a Scenario steps:

- **The Scenario owns the physics callback.** It calls the controller's `step(dt)` on every physics step. Every
  controller a Scenario can plug in offers the same small interface (`reset()`, `step(dt)`, `close()`), which the
  Scenario defines as a Python `Protocol`. The Scenario imports the robot and controller extensions; they never import
  each other.
- **The controller keeps SONIC's timing and state inside `step(dt)`.** That covers the policy on every 4th physics step
  (50 Hz at 200 Hz physics), a replan check on every 5th policy tick (10 Hz of sim time), the cached 50 Hz reference
  motion and an absolute frame counter.
- **`kinematic_planner.py` holds only stateless functions.** The controller sends `plan()` to one worker thread with a
  copy of its inputs. Only the control thread swaps a finished plan in, blending by absolute frame, so a late plan skips
  the frames already played. A generation counter drops plans started before a reset. A synchronous mode waits for each
  plan.
- **Plain NumPy and onnxruntime**: no MotionGen and no `isaacsim.robot.policy.examples`.

## Why

- **Not synchronous planning**: a replan takes ~75 ms on CPU and stalls the 50 Hz control step.
- **Not a planner thread with its own timer**: a thread can't follow sim time. It would read the motion and the command
  while the control thread changes them, and it drifts when the simulation pauses or runs off real time. Counting
  physics steps follows sim time by construction.
- **Not a physics callback inside the controller extension**: the Scenario would lose control of the order within a
  step, and a Standalone script that steps physics would need each controller's own hooks.
- **Not the Scenario calling the planner functions**: every Scenario (warehouse, house, ...) would re-implement the
  schedule, the swap and the blend.
- **Not MotionGen**: its types force warp arrays at the controller's edges, and the controller uses none of its planning.

## Consequences

- A Scenario must call `step(dt)` on every physics step and `close()` on shutdown. The extension's `on_shutdown` also
  closes any controller left open, so a hot reload doesn't leak the worker thread or the ONNX sessions (~0.9 GB of
  weights).
- Plan arrival depends on wall time. When the simulation runs much faster than real time, plans land late and are
  spliced in later; deterministic runs use the synchronous mode.
- Files ported from NVIDIA's `gear_sonic_deploy` keep Apache-2.0 (SPDX `MIT AND Apache-2.0`, NVIDIA's copyright line, a
  modified-by note); the rest of the extension is MIT. See `research/reports/research-sonic-extension-mit-license.md`.
