// Golden-data tests: the C++ port against the Python reference (test/make_goldens.py). The ONNX tests need the SONIC
// weights (`pixi run sonic-download`) and are skipped without them.
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "oc_sonic/kinematic_planner.hpp"
#include "oc_sonic/motion.hpp"
#include "oc_sonic/planner_command.hpp"
#include "oc_sonic/sonic_policy.hpp"

using namespace oc_sonic;

namespace
{

std::vector<double> golden(const std::string & name)
{
  std::ifstream file(std::string(GOLDEN_DIR) + "/" + name + ".f64", std::ios::binary | std::ios::ate);
  EXPECT_TRUE(file.good()) << name;
  std::vector<double> data(static_cast<size_t>(file.tellg()) / sizeof(double));
  file.seekg(0);
  file.read(reinterpret_cast<char *>(data.data()), data.size() * sizeof(double));
  return data;
}

// Cursor over a golden's concatenated arrays.
struct Reader
{
  const std::vector<double> & data;
  size_t at = 0;
  const double * take(size_t n)
  {
    const double * p = data.data() + at;
    at += n;
    return p;
  }
};

std::vector<Eigen::Matrix<double, 36, 1>> rows(const double * p, int n)
{
  std::vector<Eigen::Matrix<double, 36, 1>> out(n);
  for (int i = 0; i < n; ++i) {
    out[i] = Eigen::Map<const Eigen::Matrix<double, 36, 1>>(p + 36 * i);
  }
  return out;
}

void expect_close(const double * expected, const double * actual, size_t n, double tolerance, const std::string & what)
{
  for (size_t i = 0; i < n; ++i) {
    ASSERT_NEAR(expected[i], actual[i], tolerance) << what << "[" << i << "]";
  }
}

// The arrays of motion (positions, quats, joints, velocities) after a golden's reader position.
void expect_motion(Reader & r, const Motion & m, double tolerance, const std::string & what)
{
  const size_t n = m.size();
  for (size_t f = 0; f < n; ++f) {
    expect_close(r.take(3), m.positions[f].data(), 3, tolerance, what + " positions");
  }
  for (size_t f = 0; f < n; ++f) {
    expect_close(r.take(4), m.quats[f].data(), 4, tolerance, what + " quats");
  }
  for (size_t f = 0; f < n; ++f) {
    expect_close(r.take(NUM_JOINTS), m.joints[f].data(), NUM_JOINTS, tolerance, what + " joints");
  }
  for (size_t f = 0; f < n; ++f) {
    expect_close(r.take(NUM_JOINTS), m.joint_velocities[f].data(), NUM_JOINTS, tolerance, what + " velocities");
  }
}

// The motion in a golden that holds `n` qpos rows and then its 50 Hz resampling.
Motion resampled(const std::string & name, int n)
{
  const auto data = golden(name);
  return resample(rows(data.data(), n));
}

bool has_models()
{
  return std::filesystem::exists(std::string(MODEL_DIR) + "/planner_sonic.onnx");
}

}  // namespace

TEST(Golden, Resample)
{
  for (const auto & [name, n] : {std::pair<std::string, int>{"resample", 40}, {"resample_b", 50}}) {
    const auto data = golden(name);
    const Motion m = resample(rows(data.data(), n));
    ASSERT_EQ(m.size(), n * 5 / 3);
    Reader r{data, 36u * n};
    expect_motion(r, m, 1e-9, name);
  }
}

TEST(Golden, Blend)
{
  const Motion old = resampled("resample", 40), fresh = resampled("resample_b", 50);
  for (const auto & [name, current, start] : {std::tuple<std::string, int, int>{"blend_ontime", 10, 12}, {"blend_late", 10, 3}}) {
    const auto data = golden(name);
    const Motion m = blend(old, fresh, current, start);
    ASSERT_EQ(m.size(), start - current + fresh.size());
    Reader r{data};
    expect_motion(r, m, 1e-9, name);
  }
  EXPECT_EQ(blend(old, fresh, 100, 10).size(), 0);  // too late: nothing left
}

TEST(Golden, PlannerContext)
{
  const auto context = planner_context(resampled("resample", 40), 14);
  const auto data = golden("context");
  for (int n = 0; n < 4; ++n) {
    expect_close(data.data() + 36 * n, context[n].data(), 36, 1e-9, "context");
  }
}

TEST(Golden, Policy)
{
  if (!has_models()) {
    GTEST_SKIP() << "SONIC weights missing";
  }
  const Motion motion = resampled("resample", 40);
  const auto data = golden("policy");
  Reader r{data};
  SonicPolicy policy(MODEL_DIR, 2);
  const Quat base_yaw = yaw_quat(0.7);
  policy.reset(base_yaw, motion.quats[0]);
  const double * token = r.take(TOKEN_SIZE);
  bool first = true;
  while (r.at < data.size()) {
    const Joints q = Eigen::Map<const Joints>(r.take(NUM_JOINTS)), dq = Eigen::Map<const Joints>(r.take(NUM_JOINTS));
    const Quat base = Eigen::Map<const Quat>(r.take(4));
    const Eigen::Vector3d gyro = Eigen::Map<const Eigen::Vector3d>(r.take(3));
    Reference reference;
    for (int k = 0; k < FUTURE_FRAMES; ++k) {
      reference.positions[k] = Eigen::Map<const Joints>(r.take(NUM_JOINTS));
    }
    for (int k = 0; k < FUTURE_FRAMES; ++k) {
      reference.velocities[k] = Eigen::Map<const Joints>(r.take(NUM_JOINTS));
    }
    for (int k = 0; k < FUTURE_FRAMES; ++k) {
      reference.quats[k] = Eigen::Map<const Quat>(r.take(4));
    }
    if (first) {
      const auto encoded = policy.encode(base, reference);
      for (int i = 0; i < TOKEN_SIZE; ++i) {
        ASSERT_NEAR(token[i], encoded[i], 1e-3) << "token[" << i << "]";
      }
      first = false;
    }
    const Joints targets = policy.act(q, dq, base, gyro, reference);
    expect_close(r.take(NUM_JOINTS), targets.data(), NUM_JOINTS, 1e-3, "targets");
  }
}

TEST(Golden, Planner)
{
  if (!has_models()) {
    GTEST_SKIP() << "SONIC weights missing";
  }
  KinematicPlanner planner(MODEL_DIR, 2);
  PlannerCommand idle, walk;
  walk.mode = WALK;
  walk.moving = true;
  walk.movement_yaw = 0.3;
  walk.facing_yaw = 0.2;
  walk.speed = 0.9;
  for (const auto & [name, command] : {std::pair<std::string, PlannerCommand>{"plan_idle", idle}, {"plan_walk", walk}}) {
    const auto data = golden(name);
    const Motion m = planner.plan(standing_context(), command);
    ASSERT_EQ(data.size(), m.size() * (3 + 4 + 2 * NUM_JOINTS)) << name;
    Reader r{data};
    expect_motion(r, m, 1e-3, name);
  }
}

TEST(Tables, JointOrders)
{
  const auto & tables = joint_tables();
  EXPECT_NEAR(tables.kp[0], 99.098, 0.001);  // 7520_22 hip pitch: armature w^2
  EXPECT_NEAR(tables.default_pose[3], 0.669, 1e-12);
  for (int i = 0; i < NUM_JOINTS; ++i) {
    EXPECT_EQ(mujoco_joints()[mujoco_of_isaaclab()[i]], isaaclab_joints()[i]);
  }
  Joints j = Joints::LinSpaced(0, 28);
  EXPECT_EQ(to_mujoco(to_isaaclab(j)), j);
}

TEST(Command, LimitAndShape)
{
  EXPECT_EQ(limit({5.0, 2.0, 9.0}, WALK), (Twist{MAX_WALK_SPEED, MAX_SIDE_SPEED, MAX_TURN_RATE}));
  EXPECT_EQ(limit({1.0, 1.0, 0.5}, IDLE), (Twist{0.0, 0.0, 0.5}));

  CommandShaper shaper;  // walk by default
  shaper.start(0.0);
  auto c = shaper.shape({0.05, 0.0, 0.0}, 0.1, 0.0, 0.0);  // below the deadband: stand
  EXPECT_EQ(c.mode, IDLE);
  EXPECT_FALSE(c.moving);
  c = shaper.shape({0.2, 0.0, 0.0}, 0.1, 0.0, 0.0);  // slow forward: raised to the minimum moving speed
  EXPECT_EQ(c.mode, WALK);
  EXPECT_TRUE(c.moving);
  EXPECT_DOUBLE_EQ(c.speed, MIN_MOVING_SPEED);
  c = shaper.shape({1.0, 0.0, 0.0}, 0.1, 0.2, 0.25);  // world movement and facing into the planner frame
  EXPECT_NEAR(c.movement_yaw, -0.05, 1e-12);
  EXPECT_NEAR(c.facing_yaw, -0.25, 1e-12);

  shaper.set_mode(HAND_CRAWL, -1.0);  // a crawl holds its pose when stopped and floors its speed
  c = shaper.shape({0.0, 0.0, 0.0}, 0.1, 0.0, 0.0);
  EXPECT_EQ(c.mode, HAND_CRAWL);
  EXPECT_FALSE(c.moving);
  c = shaper.shape({0.2, 0.0, 0.0}, 0.1, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(c.speed, 0.4);
  shaper.set_facing(1.2);  // facing overrides wz
  EXPECT_DOUBLE_EQ(shaper.shape({0.0, 0.0, 1.0}, 0.1, 0.0, 0.0).facing_yaw, 1.2);
}

TEST(Command, FacingLead)
{
  CommandShaper shaper;
  shaper.start(0.0);
  double facing = 0.0;
  for (int i = 0; i < 100; ++i) {  // turning with a robot that doesn't follow: the facing stops at the lead
    facing = shaper.shape({0.0, 0.0, 1.5}, 0.1, 0.0, 0.0).facing_yaw;
  }
  EXPECT_NEAR(facing, MAX_FACING_LEAD, 1e-12);
}

TEST(Command, SetModeChecks)
{
  EXPECT_EQ(CommandShaper::check(WALK, -1.0), "");
  EXPECT_EQ(CommandShaper::check(SQUAT, 0.5), "");
  EXPECT_EQ(CommandShaper::check(SQUAT, -1.0), "");
  EXPECT_NE(CommandShaper::check(SQUAT, 0.3), "");
  EXPECT_NE(CommandShaper::check(KNEEL_ONE_LEG, 0.5), "");
  EXPECT_NE(CommandShaper::check(7, -1.0), "");
  EXPECT_NE(CommandShaper::check(200, -1.0), "");
}

TEST(Command, Replan)
{
  PlannerCommand planned, want;
  EXPECT_FALSE(needs_replan(want, planned, 5.0));  // idle never replans on the timer
  want.mode = SQUAT;
  want.height = 0.5;
  planned = want;
  EXPECT_FALSE(needs_replan(want, planned, 5.0));  // nor does a held posture (the old replan bug)
  want.height = 0.6;
  EXPECT_TRUE(needs_replan(want, planned, 0.0));

  planned = want = PlannerCommand{};
  want.mode = planned.mode = WALK;
  want.moving = planned.moving = true;
  want.speed = planned.speed = 1.0;
  EXPECT_FALSE(needs_replan(want, planned, 0.5));
  EXPECT_TRUE(needs_replan(want, planned, 1.0));  // timer
  want.mode = planned.mode = RUN;
  EXPECT_TRUE(needs_replan(want, planned, 0.1));
  want.speed = 1.2;
  EXPECT_TRUE(needs_replan(want, planned, 0.0));
}
