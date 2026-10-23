#include "oc_sonic/planner_command.hpp"

#include <algorithm>
#include <cmath>

namespace oc_sonic
{

namespace
{

// Allowed pelvis-height range [m] of each posture (SONIC's keyframes; planner_onnx.md).
std::pair<double, double> height_range(int mode)
{
  return mode == SQUAT ? std::pair<double, double>{0.4, 0.8} : std::pair<double, double>{0.2, 0.4};
}

// Speed range [m/s] of each crawl mode (a command below the low bound is raised to it).
std::pair<double, double> crawl_speed_range(int mode)
{
  return mode == HAND_CRAWL ? std::pair<double, double>{0.4, 1.0} : std::pair<double, double>{0.7, 1.0};
}

}  // namespace

bool is_posture(int mode) {return mode == SQUAT || mode == KNEEL_TWO_LEGS || mode == KNEEL_ONE_LEG;}
bool is_crawl(int mode) {return mode == HAND_CRAWL || mode == ELBOW_CRAWL;}
bool is_static(int mode) {return mode == IDLE || is_posture(mode);}

Twist limit(const Twist & cmd, int mode)
{
  const double top = mode == IDLE ? 0.0 : mode == SLOW_WALK ? MAX_SLOW_WALK_SPEED : mode == WALK ? MAX_WALK_SPEED : MAX_RUN_SPEED;
  const double side = top > 0.0 ? MAX_SIDE_SPEED : 0.0;
  return {std::clamp(cmd.vx, 0.0, top), std::clamp(cmd.vy, -side, side), std::clamp(cmd.wz, -MAX_TURN_RATE, MAX_TURN_RATE)};
}

double replan_interval(int mode)
{
  return mode == RUN ? 0.1 : is_crawl(mode) ? 0.2 : 1.0;
}

bool needs_replan(const PlannerCommand & want, const PlannerCommand & planned, double since)
{
  if (
    want.mode != planned.mode || angle_between(want.facing_yaw, planned.facing_yaw) > FACING_CHANGE ||
    std::abs(want.height - planned.height) > 1e-6)
  {
    return true;
  }
  if (is_static(want.mode)) {
    return false;
  }
  if (
    std::abs(want.speed - planned.speed) > SPEED_CHANGE || want.moving != planned.moving ||
    (want.moving && angle_between(want.movement_yaw, planned.movement_yaw) > DIRECTION_CHANGE))
  {
    return true;
  }
  return want.moving && since >= replan_interval(want.mode);
}

std::string CommandShaper::check(int mode, double pelvis_height)
{
  if (!(mode == IDLE || mode == SLOW_WALK || mode == WALK || mode == RUN || is_posture(mode) || is_crawl(mode))) {
    return "unknown mode " + std::to_string(mode);
  }
  if (is_posture(mode) && pelvis_height != -1.0) {
    const auto [low, high] = height_range(mode);
    if (!(pelvis_height >= low && pelvis_height <= high)) {
      return "pelvis_height " + std::to_string(pelvis_height) + " is outside " + std::to_string(low) + " - " +
             std::to_string(high) + " m for mode " + std::to_string(mode);
    }
  }
  return "";
}

void CommandShaper::set_mode(int mode, double pelvis_height)
{
  mode_ = mode;
  height_ = is_posture(mode) ? pelvis_height : -1.0;
}

PlannerCommand CommandShaper::shape(const Twist & command, double dt, double yaw, double offset)
{
  const Twist cmd = limit(command, mode_);
  if (override_) {
    facing_ = wrap(*override_);
  } else if (!is_posture(mode_)) {  // a posture holds the facing it stood with
    const double lead = std::clamp(wrap(facing_ + cmd.wz * dt - yaw), -MAX_FACING_LEAD, MAX_FACING_LEAD);
    facing_ = wrap(yaw + lead);
  }

  PlannerCommand out;
  out.mode = mode_;
  out.facing_yaw = wrap(facing_ - offset);
  if (is_posture(mode_)) {
    out.speed = -1.0;
    out.height = height_;
    return out;
  }
  double speed = std::hypot(cmd.vx, cmd.vy);
  if (mode_ == IDLE || speed < DEADBAND) {
    out.mode = is_crawl(mode_) ? mode_ : IDLE;  // a crawl holds its low pose
    return out;
  }
  if (is_crawl(mode_)) {
    const auto [low, high] = crawl_speed_range(mode_);
    speed = std::clamp(speed, low, high);
  } else if (cmd.vx >= std::abs(cmd.vy)) {  // mostly forward; side steps are slow by design
    speed = std::max(speed, MIN_MOVING_SPEED);
  }
  out.moving = true;
  out.movement_yaw = wrap(wrap(yaw + std::atan2(cmd.vy, cmd.vx)) - offset);
  out.speed = speed;
  return out;
}

}  // namespace oc_sonic
