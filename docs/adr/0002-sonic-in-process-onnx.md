# The Whole-body controller runs SONIC in-process with onnxruntime

## Decision

`oc.control.g1_wbc_sonic` runs NVIDIA SONIC's three ONNX models (planner, encoder, decoder) with
onnxruntime inside the Kit extension. It steps them from a physics callback, as `G1Walker` does, and
re-implements in Python the parts of NVIDIA's C++ runtime that it needs: observation histories, planner
replanning, resampling and blending. It uses the default SONIC weights.

## Why

- **NVIDIA ships no Isaac Sim path.** The official runtime is C++ with TensorRT over Unitree DDS
  (`rt/lowstate`, `rt/lowcmd`, `rt/secondary_imu`). The only other paths are a MuJoCo sim2sim and Isaac
  Lab evaluation on Python 3.11. See `research/reports/research-g1-sonic-wbc.md`.
- **Not the C++ runtime over a DDS bridge**: it would need TensorRT pinned to 10.13, a C++ build, a Unitree
  DDS bridge and a second process. It would also bypass MotionGen and the Movement primitive API we want
  to call from Python and, later, from nav2.
- **Default weights, not v1.1**: v1.1 targets VR teleop heading and wrist tracking. We drive SONIC through
  its planner instead.

## Consequences

- We own the port. Joint-order or history mistakes fail silently, so unit tests check them against
  NVIDIA's constants.
- Physics runs at 200 Hz for this controller (SONIC's training rate); the warehouse sets it for `wbc` and
  keeps 100 Hz for `walk`. A lap also completed at 100 Hz.
- onnxruntime numerics are not checked against TensorRT. NVIDIA's MuJoCo sim2sim is the reference if
  motion looks wrong.
- Running the official runtime on hardware later is unaffected: the weights and the planner interface are the same.
