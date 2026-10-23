#include "oc_sonic/motion.hpp"

#include <algorithm>
#include <cmath>

namespace oc_sonic
{

Sample sample(const Motion & motion, double frame)
{
  const double last = motion.size() - 1.0;
  frame = std::min(std::max(frame, 0.0), last);
  const int f0 = static_cast<int>(std::floor(frame));
  const int f1 = std::min(f0 + 1, motion.size() - 1);
  const double w = frame - f0;
  return {
    (1 - w) * motion.positions[f0] + w * motion.positions[f1],
    quat_slerp(motion.quats[f0], motion.quats[f1], w),
    (1 - w) * motion.joints[f0] + w * motion.joints[f1]};
}

Motion resample(const std::vector<Eigen::Matrix<double, 36, 1>> & qpos)
{
  const int n = static_cast<int>(qpos.size());
  const int length = static_cast<int>(std::floor(n / PLANNER_RATE * CONTROL_RATE));
  Motion m;
  for (int i = 0; i < length; ++i) {
    const double t = i * PLANNER_RATE / CONTROL_RATE;
    const int f0 = static_cast<int>(std::floor(t));
    const int f1 = std::min(f0 + 1, n - 1);
    const double w = t - f0;
    const Eigen::Matrix<double, 36, 1> row = (1 - w) * qpos[f0] + w * qpos[f1];
    m.positions.push_back(row.head<3>());
    m.quats.push_back(quat_slerp(qpos[f0].segment<4>(3), qpos[f1].segment<4>(3), w));
    m.joints.push_back(to_isaaclab(row.tail<NUM_JOINTS>()));
  }
  m.joint_velocities.resize(length);
  for (int i = 0; i + 1 < length; ++i) {
    m.joint_velocities[i] = (m.joints[i + 1] - m.joints[i]) * CONTROL_RATE;
  }
  m.joint_velocities[length - 1] = m.joint_velocities[length - 2];
  return m;
}

std::vector<Eigen::Matrix<double, 36, 1>> planner_context(const Motion & motion, int frame)
{
  std::vector<Eigen::Matrix<double, 36, 1>> context(4);
  for (int n = 0; n < 4; ++n) {
    const Sample s = sample(motion, frame + n * CONTROL_RATE / PLANNER_RATE);
    context[n] << s.position, s.quat, to_mujoco(s.joints);
  }
  return context;
}

Motion blend(const Motion & old, const Motion & fresh, int64_t current, int64_t start)
{
  const int64_t length = start - current + fresh.size();
  Motion m;
  if (length <= 0) {
    return m;
  }
  const int64_t blend_start = std::max<int64_t>(0, start - current);
  for (int64_t f = 0; f < length; ++f) {
    const auto f_old = static_cast<int>(std::clamp<int64_t>(f + current, 0, old.size() - 1));
    const auto f_new = static_cast<int>(std::clamp<int64_t>(f + current - start, 0, fresh.size() - 1));
    const double w = std::clamp(static_cast<double>(f - blend_start) / BLEND_FRAMES, 0.0, 1.0);
    m.positions.push_back((1 - w) * old.positions[f_old] + w * fresh.positions[f_new]);
    m.quats.push_back(quat_slerp(old.quats[f_old], fresh.quats[f_new], w));
    m.joints.push_back((1 - w) * old.joints[f_old] + w * fresh.joints[f_new]);
    m.joint_velocities.push_back((1 - w) * old.joint_velocities[f_old] + w * fresh.joint_velocities[f_new]);
  }
  return m;
}

}  // namespace oc_sonic
