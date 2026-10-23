// Constants, joint maps, quaternion math and the onnxruntime helper shared by the SONIC planner and policy.
//
// Port of oc.control.g1_sonic controllers/sonic.py. Constants and observation layouts follow NVlabs/GR00T-WholeBodyControl
// (Apache-2.0) gear_sonic_deploy/src/g1/g1_deploy_onnx_ref: include/policy_parameters.hpp, src/g1_deploy_onnx_ref.cpp
// and policy/release/observation_config.yaml.
#pragma once

#include <onnxruntime/core/session/onnxruntime_cxx_api.h>

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace oc_sonic
{

constexpr int NUM_JOINTS = 29;
constexpr double CONTROL_RATE = 50.0;  // Hz: the policy runs at 50 Hz
constexpr double PLANNER_RATE = 30.0;  // Hz: the planner's qpos
constexpr int HISTORY = 10;  // frames of robot state history, oldest first
constexpr int FUTURE_FRAMES = 10;  // reference frames 0.1 s apart, 0.9 s ahead
constexpr int FUTURE_STEP = 5;
constexpr int LOOK_AHEAD = 2;  // control steps: a new plan starts this far ahead of the current frame
constexpr double DEFAULT_HEIGHT = 0.788740;  // m: the planner's standing pelvis height
constexpr int ENCODER_SIZE = 1762;
constexpr int TOKEN_SIZE = 64;
constexpr int DECODER_SIZE = TOKEN_SIZE + HISTORY * (3 + 29 + 29 + 29 + 3);  // 994

using Joints = Eigen::Matrix<double, NUM_JOINTS, 1>;
using Quat = Eigen::Vector4d;  // w, x, y, z

// Hardware (MuJoCo) joint order: motors, gains and the planner's qpos.
const std::array<std::string, NUM_JOINTS> & mujoco_joints();
// IsaacLab joint order (breadth-first): the policy's observations and actions.
const std::array<std::string, NUM_JOINTS> & isaaclab_joints();
const std::array<int, NUM_JOINTS> & mujoco_of_isaaclab();  // mujoco[mujoco_of_isaaclab()[i]] is isaaclab joint i
const std::array<int, NUM_JOINTS> & isaaclab_of_mujoco();  // isaaclab[isaaclab_of_mujoco()[i]] is mujoco joint i

// Per-joint motor constants, MuJoCo order. kp = armature w^2, kd = 2 zeta armature w, w = 10 Hz, zeta = 2.
struct JointTables
{
  Joints armature;  // kg m^2
  Joints effort_limit;  // N m
  Joints action_scale;  // 0.25 effort / kp of one motor
  Joints kp;
  Joints kd;
  Joints default_pose;  // rad
};
const JointTables & joint_tables();

// Reorders a vector between the two joint orders.
Joints to_isaaclab(const Joints & mujoco);
Joints to_mujoco(const Joints & isaaclab);

// Quaternions are [w, x, y, z].
Quat quat_mul(const Quat & a, const Quat & b);
Quat quat_conjugate(const Quat & q);
Eigen::Vector3d quat_rotate(const Quat & q, const Eigen::Vector3d & v);
Quat quat_slerp(const Quat & a, Quat b, double t);
double yaw_of(const Quat & q);
Quat yaw_quat(double yaw);
Eigen::Matrix<double, 6, 1> rotation_6d(const Quat & q);  // first two columns of the rotation matrix, row by row
double wrap(double angle);  // to [-pi, pi)
double angle_between(double a, double b);

// An onnxruntime CPU session on `threads` intra-op threads without spinning (spinning threads compete with the other
// nodes' threads and run several times slower).
class OrtModel
{
public:
  OrtModel(const std::string & path, int threads);
  // Runs with the given inputs, named in input_names; returns all outputs.
  std::vector<Ort::Value> run(const std::vector<const char *> & input_names, std::vector<Ort::Value> & inputs);

private:
  std::unique_ptr<Ort::Session> session_;
  std::vector<std::string> output_names_;
};

// Tensor over caller-owned data.
template<typename T>
Ort::Value make_tensor(T * data, const std::vector<int64_t> & shape)
{
  size_t count = 1;
  for (auto n : shape) {
    count *= static_cast<size_t>(n);
  }
  static const Ort::MemoryInfo info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  return Ort::Value::CreateTensor<T>(info, data, count, shape.data(), shape.size());
}

}  // namespace oc_sonic
