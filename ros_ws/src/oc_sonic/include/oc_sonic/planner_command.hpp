// Command shaping for the planner: turns /cmd_vel, the commanded mode and the facing into a PlannerCommand, and decides
// when it needs a replan. Port of oc.control.g1_sonic wbc_controller.py (limit, planner_inputs) and
// kinematic_planner.py (_needs_replan); replan timers and the static-mode rule follow NVlabs/GR00T-WholeBodyControl
// (Apache-2.0) gear_sonic_deploy g1_deploy_onnx_ref.cpp Planner().
#pragma once

#include <optional>
#include <string>

#include "oc_sonic/kinematic_planner.hpp"

namespace oc_sonic
{

constexpr double DEADBAND = 0.1;  // m/s: slower commands stand
constexpr double MIN_MOVING_SPEED = 0.4;  // m/s: slower, SONIC's slow walk steps in place
constexpr double MAX_SLOW_WALK_SPEED = 0.8;  // m/s
constexpr double MAX_WALK_SPEED = 1.5;  // m/s
constexpr double MAX_RUN_SPEED = 3.0;  // m/s
constexpr double MAX_SIDE_SPEED = 0.4;  // m/s
constexpr double MAX_TURN_RATE = 1.5;  // rad/s
constexpr double MAX_FACING_LEAD = M_PI / 6.0;  // rad: the facing the planner is told stays this close to the measured yaw
constexpr double FACING_CHANGE = M_PI / 180.0;  // rad: a smaller facing change doesn't replan
constexpr double DIRECTION_CHANGE = 5.0 * M_PI / 180.0;  // rad
constexpr double SPEED_CHANGE = 0.05;  // m/s

struct Twist
{
  double vx = 0.0, vy = 0.0, wz = 0.0;
  bool operator==(const Twist & o) const {return vx == o.vx && vy == o.vy && wz == o.wz;}
};

bool is_posture(int mode);  // squat, kneel: held in place at a pelvis height
bool is_crawl(int mode);  // hand / elbow crawl: low moving modes
bool is_static(int mode);  // idle and the postures: replan only on a mode, facing or height change

// cmd within what the robot can do in mode: forward only, up to the mode's top speed, |vy| <= 0.4, |wz| <= 1.5;
// idle only turns.
Twist limit(const Twist & cmd, int mode);

// Seconds between periodic replans while moving in mode.
double replan_interval(int mode);

// Whether `want` differs enough from the last planned command, `since` seconds ago, to plan again.
bool needs_replan(const PlannerCommand & want, const PlannerCommand & planned, double since);

class CommandShaper
{
public:
  // Empty if (mode, pelvis_height) is accepted, else why not. A height of -1 is the mode's default; the non-posture
  // modes ignore it.
  static std::string check(int mode, double pelvis_height);

  void set_mode(int mode, double pelvis_height);
  int mode() const {return mode_;}

  // World yaw to face, overriding cmd_vel's wz; nullopt returns to integrating wz.
  void set_facing(std::optional<double> yaw) {override_ = yaw;}
  // Starts facing yaw (the measured world yaw).
  void start(double yaw) {facing_ = yaw;}

  // The planner command for cmd, dt seconds after the last one, with the pelvis at world yaw `yaw` and the planner's
  // frame rotated by `offset` (world yaw - planner yaw). Turns the facing by wz, keeping it within MAX_FACING_LEAD of yaw.
  PlannerCommand shape(const Twist & cmd, double dt, double yaw, double offset);

private:
  int mode_ = WALK;
  double height_ = -1.0;
  double facing_ = 0.0;  // world yaw the planner is told to face
  std::optional<double> override_;
};

}  // namespace oc_sonic
