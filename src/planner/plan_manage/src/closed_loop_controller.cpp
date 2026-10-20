#include <algorithm>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <std_msgs/msg/bool.hpp>
#include "plan_manage/tf2_compat.h"

#include "bspline_opt/uniform_bspline.h"
#include "plan_manage/tracking_checks.h"

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController() : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    odom_timeout_ = declare_parameter<double>("odom_timeout_sec", 0.25);
    trajectory_grace_ = declare_parameter<double>("trajectory_grace_sec", 0.25);
    external_execution_ = declare_parameter<bool>("external_execution_control", false);
    require_reference_ = declare_parameter<bool>("require_reference_path", false);
    validate_timestamps_ = declare_parameter<bool>("validate_timestamps", false);
    expected_frame_ = declare_parameter<std::string>("expected_frame", "");
    execution_enabled_ = !external_execution_;
    for (double value : {time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_,
                         max_vx_, max_vy_, max_vyaw_, finish_dist_, odom_timeout_})
      if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument("Controller limits and gains must be finite and positive");
    if (!std::isfinite(trajectory_grace_) || trajectory_grace_ < 0.0)
      throw std::invalid_argument("Trajectory grace must be finite and nonnegative");

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    tracking_valid_pub_ = create_publisher<std_msgs::msg::Bool>("planning/tracking_valid", 1);
    execution_sub_ = create_subscription<std_msgs::msg::Bool>(
        "execution_enabled", 1, [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
          execution_enabled_ = !external_execution_ || msg->data;
        });
    if (require_reference_)
      path_sub_ = create_subscription<nav_msgs::msg::Path>(
          "initial_path", rclcpp::QoS(1).reliable().transient_local(),
          [this](const nav_msgs::msg::Path::ConstSharedPtr msg) {
            reference_ready_ = msg->poses.size() >= 2 &&
                (expected_frame_.empty() || msg->header.frame_id == expected_frame_);
            reference_stamp_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            if (!reference_ready_ || trajectory_stamp_ < reference_stamp_)
              receive_traj_ = false;
          });
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_seconds_ = steadySeconds();
    RCLCPP_INFO(get_logger(), "Closed-loop controller ready");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

  static double steadySeconds()
  {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    cmd_vel_pub_->publish(cmd);
  }

  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    receive_traj_ = false;
    if (!validCubicKnots(msg->pos_pts.size(), msg->order, msg->knots))
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    if (!points.allFinite())
      return;
    trajectory_stamp_ = msg->start_time.sec + msg->start_time.nanosec * 1e-9;
    if (require_reference_ && trajectory_stamp_ < reference_stamp_)
      return;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);
    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    trajectory_lifetime_ = traj_duration_;
    if (validate_timestamps_)
    {
      const double age = now().seconds() - trajectory_stamp_;
      if (trajectory_stamp_ <= 0.0 || age < -0.05 || age >= traj_duration_)
        return;
      trajectory_lifetime_ -= std::max(0.0, age);
    }
    traj_id_ = msg->traj_id;
    exec_time_ = 0.0;
    trajectory_received_ = steadySeconds();
    last_update_seconds_ = trajectory_received_;
    receive_traj_ = true;
    RCLCPP_INFO(get_logger(), "Received trajectory %lld, duration %.3fs",
                static_cast<long long>(traj_id_), traj_duration_);
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    have_odom_ = false;
    const auto &q = msg->pose.pose.orientation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    const double age = now().seconds() - stamp;
    if (!std::isfinite(norm) || norm < 1e-12 ||
        (!expected_frame_.empty() && msg->header.frame_id != expected_frame_) ||
        (validate_timestamps_ && (stamp <= 0.0 || age < -0.05 || age > odom_timeout_)))
      return;
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    if (!odom_pos_.allFinite())
      return;
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    have_odom_ = std::isfinite(odom_yaw_);
    odom_received_ = steadySeconds();
  }

  void cmdCallback()
  {
    const double current_time = steadySeconds();
    const bool valid = receive_traj_ && have_odom_ &&
        (!require_reference_ || reference_ready_) &&
        trackingTimesValid(current_time, odom_received_, trajectory_received_,
                           trajectory_lifetime_, odom_timeout_, trajectory_grace_);
    std_msgs::msg::Bool tracking;
    tracking.data = valid;
    tracking_valid_pub_->publish(tracking);
    if (!valid)
    {
      publishExecutionFrozen(true);
      publishStop();
      last_update_seconds_ = current_time;
      return;
    }
    double dt = current_time - last_update_seconds_;
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    if (!execution_enabled_) dt = 0.0;
    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    const double yaw_error = normalizeAngle(estimateDesiredYaw(t_eval, pos_des) - odom_yaw_);
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    if (std::abs(yaw_error) > heading_error_threshold_)
    {
      publishExecutionFrozen(true);
      publishStop(yaw_command);
      last_update_seconds_ = current_time;
      return;
    }

    publishExecutionFrozen(!execution_enabled_);
    exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    last_update_seconds_ = current_time;
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    const Eigen::Vector2d vel_world = clampNorm(
        Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
        std::max(max_vx_, max_vy_));
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    command.angular.z = yaw_command;
    if (exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_)
      command = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tracking_valid_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execution_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  double trajectory_lifetime_{0.0};
  std::int64_t traj_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double exec_time_{0.0};
  double last_update_seconds_{0.0};
  double odom_received_{-std::numeric_limits<double>::infinity()};
  double trajectory_received_{-std::numeric_limits<double>::infinity()};
  double trajectory_stamp_{0.0}, reference_stamp_{0.0};
  double odom_timeout_, trajectory_grace_;
  bool external_execution_, require_reference_, validate_timestamps_;
  bool execution_enabled_{true}, reference_ready_{false};
  std::string expected_frame_;
  double time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
