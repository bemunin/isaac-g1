#include "oc_sonic/kinematic_planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace oc_sonic
{

std::vector<Eigen::Matrix<double, 36, 1>> standing_context()
{
  Eigen::Matrix<double, 36, 1> row;
  row << 0.0, 0.0, DEFAULT_HEIGHT, 1.0, 0.0, 0.0, 0.0, joint_tables().default_pose;
  return std::vector<Eigen::Matrix<double, 36, 1>>(4, row);
}

KinematicPlanner::KinematicPlanner(const std::string & model_dir, int threads)
: model_(model_dir + "/planner_sonic.onnx", threads)
{
}

Motion KinematicPlanner::plan(const std::vector<Eigen::Matrix<double, 36, 1>> & context, const PlannerCommand & command)
{
  std::vector<float> qpos(4 * 36);
  for (int n = 0; n < 4; ++n) {
    for (int i = 0; i < 36; ++i) {
      qpos[n * 36 + i] = static_cast<float>(context[n][i]);
    }
  }
  std::vector<float> target_vel = {command.speed > 0 ? static_cast<float>(command.speed) : -1.0f};
  std::vector<int64_t> mode = {command.mode};
  std::vector<float> movement = {0.0f, 0.0f, 0.0f};
  if (command.moving) {
    movement = {static_cast<float>(std::cos(command.movement_yaw)), static_cast<float>(std::sin(command.movement_yaw)), 0.0f};
  }
  std::vector<float> facing = {
    static_cast<float>(std::cos(command.facing_yaw)), static_cast<float>(std::sin(command.facing_yaw)), 0.0f};
  std::vector<int64_t> seed = {0};
  std::vector<int64_t> has_target = {0};
  std::vector<float> target_positions(12, 0.0f);
  std::vector<float> target_headings(4, 0.0f);
  std::vector<int64_t> allowed_tokens(11, 0);
  std::fill(allowed_tokens.begin(), allowed_tokens.begin() + 6, 1);  // localmotion_kplanner_onnx.hpp
  std::vector<float> height = {static_cast<float>(command.height)};

  std::vector<Ort::Value> inputs;
  inputs.push_back(make_tensor(qpos.data(), {1, 4, 36}));
  inputs.push_back(make_tensor(target_vel.data(), {1}));
  inputs.push_back(make_tensor(mode.data(), {1}));
  inputs.push_back(make_tensor(movement.data(), {1, 3}));
  inputs.push_back(make_tensor(facing.data(), {1, 3}));
  inputs.push_back(make_tensor(seed.data(), {1}));
  inputs.push_back(make_tensor(has_target.data(), {1, 1}));
  inputs.push_back(make_tensor(target_positions.data(), {1, 4, 3}));
  inputs.push_back(make_tensor(target_headings.data(), {1, 4}));
  inputs.push_back(make_tensor(allowed_tokens.data(), {1, 11}));
  inputs.push_back(make_tensor(height.data(), {1}));
  const auto outputs = model_.run(
    {"context_mujoco_qpos", "target_vel", "mode", "movement_direction", "facing_direction", "random_seed",
      "has_specific_target", "specific_target_positions", "specific_target_headings", "allowed_pred_num_tokens", "height"},
    inputs);

  const float * out = outputs[0].GetTensorData<float>();
  const int count = outputs[1].GetTensorData<int32_t>()[0];
  const int capacity = static_cast<int>(outputs[0].GetTensorTypeAndShapeInfo().GetShape()[1]);
  if (count < 2 || count > capacity) {
    throw std::runtime_error("KinematicPlanner: the planner returned an invalid motion");
  }
  std::vector<Eigen::Matrix<double, 36, 1>> rows(count);
  for (int n = 0; n < count; ++n) {
    for (int i = 0; i < 36; ++i) {
      const float v = out[n * 36 + i];
      if (!std::isfinite(v)) {
        throw std::runtime_error("KinematicPlanner: the planner returned an invalid motion");
      }
      rows[n][i] = v;
    }
  }
  return resample(rows);
}

}  // namespace oc_sonic
