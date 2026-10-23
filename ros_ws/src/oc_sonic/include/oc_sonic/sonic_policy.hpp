// NVIDIA SONIC's G1 tracking policy: an encoder turns a reference motion into a motion token, a decoder turns the token
// and the robot's state history into joint targets. Port of oc.control.g1_sonic controllers/sonic.py.
#pragma once

#include <array>
#include <deque>
#include <string>

#include "oc_sonic/sonic_common.hpp"

namespace oc_sonic
{

// The 10 future reference frames 0.1 s apart, joints in IsaacLab order.
struct Reference
{
  std::array<Joints, FUTURE_FRAMES> positions;
  std::array<Joints, FUTURE_FRAMES> velocities;
  std::array<Quat, FUTURE_FRAMES> quats;
};

class SonicPolicy
{
public:
  SonicPolicy(const std::string & model_dir, int threads);

  // Forgets the history, and aligns the reference's heading `reference_quat` with the robot's `base_quat`.
  void reset(const Quat & base_quat, const Quat & reference_quat);

  // Joint targets [rad, MuJoCo order] for joints q, dq [MuJoCo order], pelvis base_quat (world) and gyro (body frame),
  // tracking `reference`. Throws if the decoder returns an invalid action.
  Joints act(
    const Joints & q, const Joints & dq, const Quat & base_quat, const Eigen::Vector3d & gyro,
    const Reference & reference);

  // The 64-wide motion token of `reference` in the g1 encoder mode.
  Eigen::Matrix<float, TOKEN_SIZE, 1> encode(const Quat & base_quat, const Reference & reference);

private:
  OrtModel encoder_, decoder_;
  // Per observation term (gyro, q - default, dq, last action, gravity), the last HISTORY frames, oldest first.
  std::array<std::deque<Eigen::VectorXd>, 5> history_;
  Joints last_action_ = Joints::Zero();
  Quat heading_ = Quat(1.0, 0.0, 0.0, 0.0);
};

}  // namespace oc_sonic
