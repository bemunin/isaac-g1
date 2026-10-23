#include "oc_sonic/sonic_policy.hpp"

#include <stdexcept>

namespace oc_sonic
{

namespace
{
// Offsets in the encoder input (observation_config.yaml): the g1 mode (0) fills three terms, the rest stay zero.
constexpr int POSITIONS_AT = 4;
constexpr int VELOCITIES_AT = POSITIONS_AT + FUTURE_FRAMES * NUM_JOINTS;
constexpr int ANCHOR_AT = VELOCITIES_AT + FUTURE_FRAMES * NUM_JOINTS + 10 + 1 + 6;
}  // namespace

SonicPolicy::SonicPolicy(const std::string & model_dir, int threads)
: encoder_(model_dir + "/model_encoder.onnx", threads), decoder_(model_dir + "/model_decoder.onnx", threads)
{
}

void SonicPolicy::reset(const Quat & base_quat, const Quat & reference_quat)
{
  for (auto & h : history_) {
    h.clear();
  }
  last_action_.setZero();
  // g1_deploy_onnx_ref.cpp ComputeApplyDeltaHeading: robot heading at start x inverse reference heading at start.
  heading_ = quat_mul(yaw_quat(yaw_of(base_quat)), yaw_quat(-yaw_of(reference_quat)));
}

Joints SonicPolicy::act(
  const Joints & q, const Joints & dq, const Quat & base_quat, const Eigen::Vector3d & gyro, const Reference & reference)
{
  const Eigen::VectorXd frame[5] = {
    gyro,
    to_isaaclab(q - joint_tables().default_pose),
    to_isaaclab(dq),
    last_action_,
    quat_rotate(quat_conjugate(base_quat), Eigen::Vector3d(0.0, 0.0, -1.0))};
  for (int i = 0; i < 5; ++i) {
    if (history_[i].empty()) {
      history_[i].assign(HISTORY, frame[i]);
    } else {
      history_[i].pop_front();
      history_[i].push_back(frame[i]);
    }
  }

  const auto token = encode(base_quat, reference);
  std::vector<float> observation;
  observation.reserve(DECODER_SIZE);
  observation.insert(observation.end(), token.data(), token.data() + TOKEN_SIZE);
  for (const auto & h : history_) {
    for (const auto & v : h) {
      for (int i = 0; i < v.size(); ++i) {
        observation.push_back(static_cast<float>(v[i]));
      }
    }
  }
  std::vector<Ort::Value> inputs;
  inputs.push_back(make_tensor(observation.data(), {1, DECODER_SIZE}));
  const auto outputs = decoder_.run({"obs_dict"}, inputs);
  const float * action = outputs[0].GetTensorData<float>();
  Joints a;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (!std::isfinite(action[i])) {
      throw std::runtime_error("SonicPolicy: decoder returned an invalid action");
    }
    a[i] = action[i];
  }
  last_action_ = a;
  return joint_tables().default_pose + to_mujoco(a).cwiseProduct(joint_tables().action_scale);
}

Eigen::Matrix<float, TOKEN_SIZE, 1> SonicPolicy::encode(const Quat & base_quat, const Reference & reference)
{
  std::vector<float> observation(ENCODER_SIZE, 0.0f);
  const Quat inverse_base = quat_conjugate(base_quat);
  for (int f = 0; f < FUTURE_FRAMES; ++f) {
    for (int j = 0; j < NUM_JOINTS; ++j) {
      observation[POSITIONS_AT + f * NUM_JOINTS + j] = static_cast<float>(reference.positions[f][j]);
      observation[VELOCITIES_AT + f * NUM_JOINTS + j] = static_cast<float>(reference.velocities[f][j]);
    }
    const auto r = rotation_6d(quat_mul(inverse_base, quat_mul(heading_, reference.quats[f])));
    for (int i = 0; i < 6; ++i) {
      observation[ANCHOR_AT + f * 6 + i] = static_cast<float>(r[i]);
    }
  }
  std::vector<Ort::Value> inputs;
  inputs.push_back(make_tensor(observation.data(), {1, ENCODER_SIZE}));
  const auto outputs = encoder_.run({"obs_dict"}, inputs);
  return Eigen::Map<const Eigen::Matrix<float, TOKEN_SIZE, 1>>(outputs[0].GetTensorData<float>());
}

}  // namespace oc_sonic
