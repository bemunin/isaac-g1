// SONIC's kinematic planner (planner_sonic.onnx): turns a planner mode, a movement direction, a facing direction and a
// speed into a whole-body reference motion. Port of oc.control.g1_sonic controllers/kinematic_planner.py (_plan) and
// NVlabs/GR00T-WholeBodyControl (Apache-2.0) gear_sonic_deploy localmotion_kplanner.hpp.
#pragma once

#include <string>
#include <vector>

#include "oc_sonic/motion.hpp"

namespace oc_sonic
{

// SONIC planner modes (planner_onnx.md lists 27; these are the ones oc_sonic exposes).
enum Mode {IDLE = 0, SLOW_WALK = 1, WALK = 2, RUN = 3, SQUAT = 4, KNEEL_TWO_LEGS = 5, KNEEL_ONE_LEG = 6,
  HAND_CRAWL = 8, ELBOW_CRAWL = 14};

// What the planner is asked for; yaw angles are in the planner's frame, which starts at the origin facing +x.
struct PlannerCommand
{
  int mode = IDLE;
  bool moving = false;  // false: stand (no movement direction)
  double movement_yaw = 0.0;
  double facing_yaw = 0.0;
  double speed = 0.0;  // m/s, <= 0: the mode's default
  double height = -1.0;  // m, the ground postures' pelvis height; -1: the mode's default
};

// The standing context at the origin facing +x.
std::vector<Eigen::Matrix<double, 36, 1>> standing_context();

class KinematicPlanner
{
public:
  KinematicPlanner(const std::string & model_dir, int threads);

  // Plans the 50 Hz motion that starts at context. Throws if the planner returns an invalid motion.
  Motion plan(const std::vector<Eigen::Matrix<double, 36, 1>> & context, const PlannerCommand & command);

private:
  OrtModel model_;
};

}  // namespace oc_sonic
