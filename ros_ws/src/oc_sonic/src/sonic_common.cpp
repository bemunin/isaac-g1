#include "oc_sonic/sonic_common.hpp"

#include <algorithm>
#include <filesystem>
#include <stdexcept>

namespace oc_sonic
{

namespace
{

std::array<std::string, NUM_JOINTS> build_mujoco_joints()
{
  std::array<std::string, NUM_JOINTS> names;
  size_t n = 0;
  for (const char * side : {"left", "right"}) {
    for (const char * joint : {"hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll"}) {
      names[n++] = std::string(side) + "_" + joint + "_joint";
    }
  }
  for (const char * joint : {"waist_yaw", "waist_roll", "waist_pitch"}) {
    names[n++] = std::string(joint) + "_joint";
  }
  for (const char * side : {"left", "right"}) {
    for (const char * joint :
      {"shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"})
    {
      names[n++] = std::string(side) + "_" + joint + "_joint";
    }
  }
  return names;
}

std::array<std::string, NUM_JOINTS> build_isaaclab_joints()
{
  const std::vector<std::vector<std::string>> groups = {
    {"left_hip_pitch", "right_hip_pitch", "waist_yaw"},
    {"left_hip_roll", "right_hip_roll", "waist_roll"},
    {"left_hip_yaw", "right_hip_yaw", "waist_pitch"},
    {"left_knee", "right_knee"},
    {"left_shoulder_pitch", "right_shoulder_pitch"},
    {"left_ankle_pitch", "right_ankle_pitch"},
    {"left_shoulder_roll", "right_shoulder_roll"},
    {"left_ankle_roll", "right_ankle_roll"},
    {"left_shoulder_yaw", "right_shoulder_yaw"},
    {"left_elbow", "right_elbow"},
    {"left_wrist_roll", "right_wrist_roll"},
    {"left_wrist_pitch", "right_wrist_pitch"},
    {"left_wrist_yaw", "right_wrist_yaw"},
  };
  std::array<std::string, NUM_JOINTS> names;
  size_t n = 0;
  for (const auto & group : groups) {
    for (const auto & joint : group) {
      names[n++] = joint + "_joint";
    }
  }
  return names;
}

struct Motor
{
  double armature;  // kg m^2
  double effort;  // N m
};

JointTables build_joint_tables()
{
  constexpr double omega = 10 * 2.0 * M_PI;
  const Motor m7520_22{0.025101925, 139.0}, m7520_14{0.010177520, 88.0}, m5020{0.003609725, 25.0}, m4010{0.00425, 5.0};
  // MuJoCo order; count: two motors in parallel.
  struct Entry
  {
    Motor motor;
    int count;
  };
  const Entry leg[6] = {{m7520_22, 1}, {m7520_22, 1}, {m7520_14, 1}, {m7520_22, 1}, {m5020, 2}, {m5020, 2}};
  const Entry arm[7] = {{m5020, 1}, {m5020, 1}, {m5020, 1}, {m5020, 1}, {m5020, 1}, {m4010, 1}, {m4010, 1}};
  std::vector<Entry> entries;
  for (int side = 0; side < 2; ++side) {
    entries.insert(entries.end(), leg, leg + 6);
  }
  entries.push_back({m7520_14, 1});
  entries.push_back({m5020, 2});
  entries.push_back({m5020, 2});
  for (int side = 0; side < 2; ++side) {
    entries.insert(entries.end(), arm, arm + 7);
  }

  JointTables t;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    const auto & [motor, count] = entries[i];
    t.armature[i] = count * motor.armature;
    t.effort_limit[i] = count * motor.effort;
    t.action_scale[i] = 0.25 * motor.effort / (motor.armature * omega * omega);
  }
  t.kp = t.armature * omega * omega;
  t.kd = 2.0 * 2.0 * t.armature * omega;
  const double leg_pose[6] = {-0.312, 0.0, 0.0, 0.669, -0.363, 0.0};
  const double left_arm[7] = {0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0};
  const double right_arm[7] = {0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0};
  t.default_pose.setZero();
  for (int i = 0; i < 6; ++i) {
    t.default_pose[i] = t.default_pose[6 + i] = leg_pose[i];
  }
  for (int i = 0; i < 7; ++i) {
    t.default_pose[15 + i] = left_arm[i];
    t.default_pose[22 + i] = right_arm[i];
  }
  return t;
}

}  // namespace

const std::array<std::string, NUM_JOINTS> & mujoco_joints()
{
  static const auto names = build_mujoco_joints();
  return names;
}

const std::array<std::string, NUM_JOINTS> & isaaclab_joints()
{
  static const auto names = build_isaaclab_joints();
  return names;
}

namespace
{

std::array<int, NUM_JOINTS> index_of(
  const std::array<std::string, NUM_JOINTS> & names, const std::array<std::string, NUM_JOINTS> & in)
{
  std::array<int, NUM_JOINTS> map;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    map[i] = static_cast<int>(std::find(in.begin(), in.end(), names[i]) - in.begin());
  }
  return map;
}

}  // namespace

const std::array<int, NUM_JOINTS> & mujoco_of_isaaclab()
{
  static const auto map = index_of(isaaclab_joints(), mujoco_joints());
  return map;
}

const std::array<int, NUM_JOINTS> & isaaclab_of_mujoco()
{
  static const auto map = index_of(mujoco_joints(), isaaclab_joints());
  return map;
}

const JointTables & joint_tables()
{
  static const auto tables = build_joint_tables();
  return tables;
}

Joints to_isaaclab(const Joints & mujoco)
{
  Joints out;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    out[i] = mujoco[mujoco_of_isaaclab()[i]];
  }
  return out;
}

Joints to_mujoco(const Joints & isaaclab)
{
  Joints out;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    out[i] = isaaclab[isaaclab_of_mujoco()[i]];
  }
  return out;
}

Quat quat_mul(const Quat & a, const Quat & b)
{
  return Quat(
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]);
}

Quat quat_conjugate(const Quat & q)
{
  return Quat(q[0], -q[1], -q[2], -q[3]);
}

Eigen::Vector3d quat_rotate(const Quat & q, const Eigen::Vector3d & v)
{
  const Eigen::Vector3d u = q.tail<3>();
  const Eigen::Vector3d t = 2.0 * u.cross(v);
  return v + q[0] * t + u.cross(t);
}

Quat quat_slerp(const Quat & a, Quat b, double t)
{
  double dot = a.dot(b);
  if (dot < 0.0) {
    b = -b;
    dot = -dot;
  }
  if (dot > 0.9995) {
    Quat q = a + t * (b - a);
    return q / q.norm();
  }
  const double theta = std::acos(dot);
  return (std::sin((1 - t) * theta) * a + std::sin(t * theta) * b) / std::sin(theta);
}

double yaw_of(const Quat & q)
{
  return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
}

Quat yaw_quat(double yaw)
{
  return Quat(std::cos(yaw / 2), 0.0, 0.0, std::sin(yaw / 2));
}

Eigen::Matrix<double, 6, 1> rotation_6d(const Quat & q)
{
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  Eigen::Matrix<double, 6, 1> r;
  r << 1 - 2 * (y * y + z * z), 2 * (x * y - w * z),
    2 * (x * y + w * z), 1 - 2 * (x * x + z * z),
    2 * (x * z - w * y), 2 * (y * z + w * x);
  return r;
}

double wrap(double angle)
{
  return std::fmod(std::fmod(angle + M_PI, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI) - M_PI;
}

double angle_between(double a, double b)
{
  return std::abs(wrap(a - b));
}

OrtModel::OrtModel(const std::string & path, int threads)
{
  if (!std::filesystem::is_regular_file(path)) {
    throw std::runtime_error("SONIC model missing: " + path + "; run `pixi run sonic-download`");
  }
  static Ort::Env env(ORT_LOGGING_LEVEL_ERROR, "oc_sonic");
  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(threads);
  options.SetInterOpNumThreads(1);
  options.AddConfigEntry("session.intra_op.allow_spinning", "0");
  session_ = std::make_unique<Ort::Session>(env, path.c_str(), options);
  Ort::AllocatorWithDefaultOptions allocator;
  for (size_t i = 0; i < session_->GetOutputCount(); ++i) {
    output_names_.emplace_back(session_->GetOutputNameAllocated(i, allocator).get());
  }
}

std::vector<Ort::Value> OrtModel::run(const std::vector<const char *> & input_names, std::vector<Ort::Value> & inputs)
{
  std::vector<const char *> outputs;
  for (const auto & name : output_names_) {
    outputs.push_back(name.c_str());
  }
  return session_->Run(
    Ort::RunOptions{nullptr}, input_names.data(), inputs.data(), inputs.size(), outputs.data(), outputs.size());
}

}  // namespace oc_sonic
