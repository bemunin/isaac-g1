# The Kinematic planner infers on a worker thread, ticked on simulation time

Status: Superseded by 0005 (2026-10-25): a late plan no longer makes the physics step wait. Previously: Accepted (2026-10-25). Supersedes 0002's "steps them from a physics callback" for the Kinematic planner only.

## Decision

The Kinematic planner in `oc.control.g1_sonic` runs its inference on a worker thread and is ticked every 0.1 s of
simulation time from the physics step. When a replan is due it launches inference, and the physics step waits at the
splice deadline (the plan starts LOOK_AHEAD = 2 control frames ahead of the playback cursor) if inference is late.
Simulation-time results are therefore deterministic and identical to planning synchronously.

The execution provider defaults to CUDA with a CPU fallback.

## Why

- **Not NVIDIA's C++ late-plan offset**: it never waits, but where a late plan lands depends on wall-clock speed,
  so results change from run to run and machine to machine.
- **Not a 5-frame look-ahead**: it would rarely wait, but it departs from the reference behaviour and its motion
  quality is untested.

## Consequences

- A slow machine slows the simulation (the step waits) instead of changing the motion.
- A plan is blended in before the reference is next read, so the reference's look-ahead sets how long it can
  overlap with the physics step.
- The planner core has no Kit imports and is tested without Isaac Sim, including equivalence with synchronous planning.
