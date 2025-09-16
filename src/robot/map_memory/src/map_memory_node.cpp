#include "map_memory_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(MAP_PUB_RATE), std::bind(&MapMemoryNode::timerCallback, this));

  global_map_ = nav_msgs::msg::OccupancyGrid();
  latest_costmap_ = nav_msgs::msg::OccupancyGrid();
  robot_x_ = 0;
  robot_y_ = 0;
  theta_ = 0.0;
  prev_x_ = 0;
  prev_y_ = 0;

  global_map_.info.resolution = MAP_RES;
  global_map_.info.width = MAP_WIDTH;
  global_map_.info.height = MAP_HEIGHT;
  global_map_.info.origin.position.x = -MAP_WIDTH * MAP_RES / 2.0;
  global_map_.info.origin.position.y = -MAP_HEIGHT * MAP_RES / 2.0;
  global_map_.data.assign(MAP_WIDTH * MAP_HEIGHT, 0);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double curr_x = msg->pose.pose.position.x;
  double curr_y = msg->pose.pose.position.y;

  double dist = std::hypot(curr_x - prev_x_, curr_y - prev_y_);
  if (dist < DIST_UPDATE) return;

  robot_x_ = curr_x;
  robot_y_ = curr_y;

  auto q_msg = msg->pose.pose.orientation;
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  theta_ = yaw;

  update_map_ = true;
}

void MapMemoryNode::timerCallback() {
  if (!update_map_ || !costmap_updated_) return;

  updateMap();
  global_map_.header.stamp = this->now();
  global_map_.header.frame_id = "sim_world";
  map_pub_->publish(global_map_);

  update_map_ = false;
  costmap_updated_ = false;
}

void MapMemoryNode::updateMap() {
  if (std::isnan(robot_x_) || std::isnan(robot_y_)) return;

  double l_res = latest_costmap_.info.resolution;
  int l_width = latest_costmap_.info.width;
  int l_height = latest_costmap_.info.height;
  auto& l_data = latest_costmap_.data;

  double g_res = global_map_.info.resolution;
  double g_origin_x = global_map_.info.origin.position.x;
  double g_origin_y = global_map_.info.origin.position.y;
  int g_width = global_map_.info.width;
  int g_height = global_map_.info.height;
  auto& g_data = global_map_.data;

  for (int y = 0; y < l_height; y++) {
    for (int x = 0; x < l_width; x++) {
      double l_x = (x - l_width / 2) * l_res;
      double l_y = (y - l_height / 2) * l_res;

      double g_x = robot_x_ + (l_x * std::cos(theta_) - l_y * std::sin(theta_));
      double g_y = robot_y_ + (l_x * std::sin(theta_) + l_y * std::cos(theta_));

      int idx_g_x = static_cast<int>(std::round(g_x / g_res + g_width / 2));
      int idx_g_y = static_cast<int>(std::round(g_y / g_res + g_height / 2));

      if (idx_g_x < 0 || idx_g_x >= g_width || idx_g_y < 0 || idx_g_y >= g_height) continue;

      int8_t& g_cost = g_data[idx_g_y * g_width + idx_g_x];
      g_cost = std::max(g_cost, l_data[y * l_width + x]);
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
