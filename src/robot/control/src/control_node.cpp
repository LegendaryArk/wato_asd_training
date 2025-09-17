#include "control_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom_ = msg; });
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(dt), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
  if (!current_path_ || !odom_) {
    RCLCPP_WARN(this->get_logger(), "Waiting for path and odometry data...");
    return;
  }

  double dist = computeDist(odom_->pose.pose.position, current_path_->poses.back().pose.position);
  if (dist < goal_tolerance_) {
    RCLCPP_INFO(this->get_logger(), "Goal reached, stopping robot.");
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }
  auto lookahead_point = findLookaheadPoint();
  if (dist < ld_) {
    lookahead_point = current_path_->poses.back();
  } else if (!lookahead_point.has_value()) {
    RCLCPP_WARN(this->get_logger(), "No valid lookahead point found.");
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }

  auto cmd = computeVel(lookahead_point.value());

  cmd_pub_->publish(cmd);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  for (auto& pose : current_path_->poses) {
    double dist = computeDist(odom_->pose.pose.position, pose.pose.position);
    if (dist >= ld_) {
      return pose;
    }
  }
  return std::nullopt;
}

geometry_msgs::msg::Twist ControlNode::computeVel(const geometry_msgs::msg::PoseStamped& tgt) {
  double yaw = extractYaw(odom_->pose.pose.orientation);
  double dx = tgt.pose.position.x - odom_->pose.pose.position.x;
  double dy = tgt.pose.position.y - odom_->pose.pose.position.y;
  double tgt_angle = std::atan2(dy, dx);
  double angle_err = tgt_angle - yaw;

  while (angle_err > M_PI) angle_err -= 2 * M_PI;
  while (angle_err < -M_PI) angle_err += 2 * M_PI;

  double dist = computeDist(odom_->pose.pose.position, current_path_->poses.back().pose.position);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = std::min(linear_vel_, linear_kp * dist);
  cmd.angular.z = tgt == current_path_->poses.back() ? 0.0 : angular_kp * angle_err;
  return cmd;
}

double ControlNode::computeDist(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}
double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion& q) {
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
