# The Kinematic planner never blocks the policy tick; a late plan is blended in when it lands

Status: Accepted (2026-10-25). Supersedes 0003-kinematic-planner-async-sim-time, whose physics step waited for a late
plan.

## Decision

`KinematicPlanner` (`oc.control.g1_sonic` `controller/kinematic_planner.py`) keeps a 50 Hz stored motion and runs
every replan on one worker thread:

- **Stored motion**: each plan (30 Hz `mujoco_qpos`) is resampled to 50 Hz, and `SonicController` reads its reference
  from it every policy tick (`reference()`: frames now to +0.9 s, 0.1 s apart). The playback cursor advances with the
  sim time `step(dt)` is given.
- **Replans**: every 0.1 s of sim time `step()` builds the inputs on the calling thread (the context is 4 frames at
  30 Hz sampled from the stored motion 2 frames past the playback frame) and submits the inference to the worker. If
  the previous plan is still in flight, that tick's replan is skipped.
- **Never waits**: `reference()` and `step()` splice a plan in only once it has finished: an 8-frame cross-fade from
  the frame it was planned to start at. A plan that lands late skips the frames already played; one that lands
  entirely in the past is dropped. A worker error is raised at the splice.
- **`synchronous=True`** runs each replan inline and splices it at once, for deterministic runs and tests.

This follows NVIDIA's runtime (gear_sonic_deploy `localmotion_kplanner.hpp:395-519` resample, `:591-680` context,
`g1_deploy_onnx_ref.cpp:3184-3398` blend and frame advance), which also never waits for its planner thread.

## Why

- **Not waiting at a splice deadline** (0003-kinematic-planner-async-sim-time): with a 2-frame look-ahead the deadline
  is 40 ms after launch, and a CPU replan takes longer, so the 50 Hz policy tick, and the physics step it runs in,
  would stall on most replans.
- **Not synchronous planning**: the same stall on every replan.
- **A skipped replan, not a queue**: plans built from an older playback frame would only land later still.

## Consequences

- Where a plan lands depends on wall-clock inference time, so two runs are not bit-identical, and a slow machine
  replans less often instead of slowing the simulation. Reproducible runs and tests use `synchronous=True`.
- Measured: a replan takes about 52 ms (median of 20) on `CPUExecutionProvider`, longer than the 40 ms look-ahead,
  so on CPU most plans land late and every other 10 Hz tick is skipped. CUDA did not load in the sim env
  (`libcublasLt.so.12` missing), so onnxruntime fell back to CPU; making CUDA load would put plans back inside the
  look-ahead.
- A command change reaches the motion one inference later than its replan tick; `late_plans` and `skipped_replans`
  count how often plans miss their start frame or a tick.
- The worker only sees its own copy of the inputs and the ONNX session; the stored motion is touched only on the
  calling thread.
