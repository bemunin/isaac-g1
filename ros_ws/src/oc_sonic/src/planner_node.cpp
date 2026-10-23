// The SONIC planner as a composable node: shapes /cmd_vel, the commanded mode and the facing into a planner command,
// replans when it changes (planner_command.hpp), and publishes the 50 Hz motion the policy node tracks. Planning runs
// onnxruntime on the CPU (~75 ms), so this node must not share a callback group with the policy.
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <oc_sonic_msgs/msg/planned_motion.hpp>
#include <oc_sonic_msgs/msg/planner_context.hpp>
#include <oc_sonic_msgs/srv/set_mode.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/float64.hpp>

#include "oc_sonic/planner_command.hpp"

namespace oc_sonic
{

using oc_sonic_msgs::msg::PlannedMotion;
using oc_sonic_msgs::msg::PlannerContext;
using oc_sonic_msgs::srv::SetMode;

class PlannerNode : public rclcpp::Node
{
public:
  explicit PlannerNode(const rclcpp::NodeOptions & options)
  : Node("planner", options)
  {
    const auto model_dir = declare_parameter<std::string>("model_dir", "");
    const auto threads = declare_parameter<int>("ort_threads", 4);
    timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.5);
    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto facing_topic = declare_parameter<std::string>("facing_topic", "/facing_direction");
    planner_ = std::make_unique<KinematicPlanner>(model_dir, threads);

    group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = group_;
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, rclcpp::QoS(1), [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {on_cmd_vel(*msg);}, sub_options);
    facing_sub_ = create_subscription<std_msgs::msg::Float64>(
      facing_topic, rclcpp::QoS(1), [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        shaper_.set_facing(std::isnan(msg->data) ? std::nullopt : std::optional<double>(msg->data));
      }, sub_options);
    context_sub_ = create_subscription<PlannerContext>(
      "planner_context", rclcpp::QoS(1), [this](PlannerContext::ConstSharedPtr msg) {on_context(msg);}, sub_options);
    // The latest plan is kept for a policy that starts later.
    motion_pub_ = create_publisher<PlannedMotion>("planned_motion", rclcpp::QoS(1).reliable().transient_local());
    service_ = create_service<SetMode>(
      "~/set_mode", [this](SetMode::Request::SharedPtr request, SetMode::Response::SharedPtr response) {
        const auto why = CommandShaper::check(request->mode, request->pelvis_height);
        response->accepted = why.empty();
        response->message = why;
        if (response->accepted) {
          shaper_.set_mode(request->mode, request->pelvis_height);
        }
      }, rclcpp::ServicesQoS(), group_);
    timer_ = create_timer(std::chrono::milliseconds(100), [this] {on_timer();}, group_);
  }

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist & msg)
  {
    const Twist cmd{msg.linear.x, msg.linear.y, msg.angular.z};
    if (!clamp_warned_ && !(limit(cmd, shaper_.mode()) == cmd)) {
      RCLCPP_WARN(
        get_logger(), "cmd_vel (%.2f, %.2f, %.2f) limited for mode %d", cmd.vx, cmd.vy, cmd.wz, shaper_.mode());
      clamp_warned_ = true;
    }
    cmd_ = cmd;
    cmd_time_ = now();
  }

  void on_context(PlannerContext::ConstSharedPtr msg)
  {
    if (msg->qpos.size() != 4 * 36) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "planner_context has %zu qpos", msg->qpos.size());
      return;
    }
    if (!context_) {
      shaper_.start(msg->measured_yaw);
    }
    context_ = msg;
  }

  void on_timer()
  {
    const auto stamp = now();
    try {
      if (!planned_) {  // start standing at the origin
        publish(standing_context(), PlannerCommand{}, 0);
        planned_ = PlannerCommand{};
        last_time_ = stamp;
        return;
      }
      if (!context_) {
        return;
      }
      const double dt = std::max(0.0, (stamp - last_time_).seconds());
      last_time_ = stamp;
      since_replan_ += dt;
      const bool stale = !cmd_time_ || (stamp - *cmd_time_).seconds() > timeout_;
      const PlannerCommand want = shaper_.shape(
        stale ? Twist{} : cmd_, dt, context_->measured_yaw, context_->heading_offset);
      if (!needs_replan(want, *planned_, since_replan_)) {
        return;
      }
      std::vector<Eigen::Matrix<double, 36, 1>> context(4);
      for (int n = 0; n < 4; ++n) {
        for (int i = 0; i < 36; ++i) {
          context[n][i] = context_->qpos[n * 36 + i];
        }
      }
      publish(context, want, context_->frame);
      planned_ = want;
      since_replan_ = 0.0;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "planning failed: %s", e.what());
    }
  }

  void publish(const std::vector<Eigen::Matrix<double, 36, 1>> & context, const PlannerCommand & command, uint64_t frame)
  {
    const Motion motion = planner_->plan(context, command);
    auto msg = std::make_unique<PlannedMotion>();
    msg->stamp = now();
    msg->start_frame = frame;
    msg->num_frames = motion.size();
    for (int f = 0; f < motion.size(); ++f) {
      for (int j = 0; j < NUM_JOINTS; ++j) {
        msg->joints.push_back(motion.joints[f][j]);
        msg->joint_velocities.push_back(motion.joint_velocities[f][j]);
      }
      for (int i = 0; i < 4; ++i) {
        msg->quats.push_back(motion.quats[f][i]);
      }
      for (int i = 0; i < 3; ++i) {
        msg->positions.push_back(motion.positions[f][i]);
      }
    }
    RCLCPP_INFO(
      get_logger(), "plan from frame %lu: mode %d, speed %.2f, %d frames", frame, command.mode, command.speed,
      motion.size());
    motion_pub_->publish(std::move(msg));
  }

  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr facing_sub_;
  rclcpp::Subscription<PlannerContext>::SharedPtr context_sub_;
  rclcpp::Publisher<PlannedMotion>::SharedPtr motion_pub_;
  rclcpp::Service<SetMode>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<KinematicPlanner> planner_;
  CommandShaper shaper_;
  PlannerContext::ConstSharedPtr context_;
  Twist cmd_;
  std::optional<rclcpp::Time> cmd_time_;
  double timeout_ = 0.5;
  bool clamp_warned_ = false;
  std::optional<PlannerCommand> planned_;  // the last planned command; unset until the first plan
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  double since_replan_ = 0.0;
};

}  // namespace oc_sonic

RCLCPP_COMPONENTS_REGISTER_NODE(oc_sonic::PlannerNode)
