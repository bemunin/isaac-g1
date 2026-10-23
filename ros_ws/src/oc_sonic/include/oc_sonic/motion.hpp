// A whole-body reference motion at 50 Hz and the operations on it that the planner and policy share.
//
// Port of oc.control.g1_sonic controllers/kinematic_planner.py (Motion, _resample, _blend), which ports NVlabs/
// GR00T-WholeBodyControl (Apache-2.0) gear_sonic_deploy g1_deploy_onnx_ref.cpp CurrentFrameAdvancement and
// localmotion_kplanner.hpp.
#pragma once

#include <cstdint>
#include <vector>

#include "oc_sonic/sonic_common.hpp"

namespace oc_sonic
{

constexpr int BLEND_FRAMES = 8;

// Root position, root quaternion (wxyz) and joint positions and velocities (IsaacLab order) per 50 Hz frame, all in
// the planner's frame.
struct Motion
{
  std::vector<Eigen::Vector3d> positions;
  std::vector<Quat> quats;
  std::vector<Joints> joints;
  std::vector<Joints> joint_velocities;

  int size() const {return static_cast<int>(joints.size());}
};

// (root position, root quaternion, joints) at a fractional frame, clamped to the motion.
struct Sample
{
  Eigen::Vector3d position;
  Quat quat;
  Joints joints;
};
Sample sample(const Motion & motion, double frame);

// A 50 Hz Motion from the planner's 30 Hz MuJoCo qpos (rows of root xyz, wxyz, 29 joints).
Motion resample(const std::vector<Eigen::Matrix<double, 36, 1>> & qpos);

// The context the planner plans from: 4 frames at 30 Hz of motion from `frame` on (fractional index), MuJoCo qpos.
std::vector<Eigen::Matrix<double, 36, 1>> planner_context(const Motion & motion, int frame);

// `old` from its frame `current` on, cross-faded into `fresh`, which starts at frame `start` (both on the old motion's
// index, so start < current for a late plan). Empty if the plan arrived so late that nothing is left.
Motion blend(const Motion & old, const Motion & fresh, int64_t current, int64_t start);

}  // namespace oc_sonic
