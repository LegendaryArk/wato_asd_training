#include "planner_node.hpp"
#include <queue>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(PATH_TIMEOUT), std::bind(&PlannerNode::timerCallback, this));

  state_ = State::WAITING_FOR_GOAL;

  current_map_ = nav_msgs::msg::OccupancyGrid();
  goal_ = geometry_msgs::msg::PointStamped();
  robot_pose_ = geometry_msgs::msg::Pose();
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  if (state_ == State::REACHING_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::REACHING_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
  if (state_ == State::REACHING_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal Reached!");
      state_ = State::WAITING_FOR_GOAL;
    } else {
      RCLCPP_INFO(this->get_logger(), "Replanning path...");
      planPath();
    }
  }
}

bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::hypot(dx, dy) < SETTLE_RADIUS;
}

void PlannerNode::planPath() {
  if (!goal_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal");
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = "sim_world";
  std::vector<CellIndex> path_cells;

  int width = current_map_.info.width;
  int height = current_map_.info.height;
  
  int init_x = static_cast<int>(std::round((robot_pose_.position.x - current_map_.info.origin.position.x) / current_map_.info.resolution));
  int init_y = static_cast<int>(std::round((robot_pose_.position.y - current_map_.info.origin.position.y) / current_map_.info.resolution));
  CellIndex init_idx(init_x, init_y);

  int goal_x = static_cast<int>(std::round((goal_.point.x - current_map_.info.origin.position.x) / current_map_.info.resolution));
  int goal_y = static_cast<int>(std::round((goal_.point.y - current_map_.info.origin.position.y) / current_map_.info.resolution));
  CellIndex goal_idx(goal_x, goal_y);

  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> f_scores;
  std::unordered_map<CellIndex, double, CellIndexHash> g_scores;

  if (init_x < 0 || init_x >= width || init_y < 0 || init_y >= height) {
    RCLCPP_WARN(this->get_logger(), "Robot position is outside map bounds: grid(%d, %d), map size(%d, %d)", init_x, init_y, width, height);
    return;
  }
  if (goal_x < 0 || goal_x >= width || goal_y < 0 || goal_y >= height) {
    RCLCPP_WARN(this->get_logger(), "Goal position is outside map bounds: grid(%d, %d), map size(%d, %d)", goal_x, goal_y, width, height);
    return;
  }

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> pq;
  pq.push(AStarNode(init_idx, 0));
  f_scores[init_idx] = std::hypot(init_idx.x - goal_x, init_idx.y - goal_y);
  g_scores[init_idx] = 0;
  while (!pq.empty()) {
    CellIndex idx = pq.top().idx;
    int x = idx.x, y = idx.y;
    pq.pop();

    if (idx == goal_idx) {
      RCLCPP_INFO(this->get_logger(), "Goal reached in A* search!");
      reconstructPath(came_from, idx, path_cells);
      break;
    }

    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;
        if (x + dx < 0 || x + dx >= width || y + dy < 0 || y + dy >= height) continue;

        CellIndex new_idx(x + dx, y + dy);

        int cell_value = current_map_.data[(y + dy) * width + (x + dx)];
        if (cell_value > 10) continue;

        double movement_cost = (dx == 0 || dy == 0) ? 1.0 : sqrt(2);
        double g = g_scores[idx] + movement_cost;
        
        if (g_scores.find(new_idx) == g_scores.end() || g < g_scores[new_idx]) {
          g_scores[new_idx] = g;
          double h = std::hypot((x + dx) - goal_x, (y + dy) - goal_y);
          f_scores[new_idx] = g + h;
          came_from[new_idx] = idx;
          pq.push(AStarNode(new_idx, f_scores[new_idx]));
        }
      }
    }
  }

  path.poses.clear();
  for (auto& cell : path_cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;

    double x = cell.x * current_map_.info.resolution + current_map_.info.origin.position.x;
    double y = cell.y * current_map_.info.resolution + current_map_.info.origin.position.y;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    path.poses.push_back(pose);
  }

  if (path_cells.empty()) {
    RCLCPP_WARN(this->get_logger(), "No path found to goal!");
    return;
  }

  path_pub_->publish(path);
}

void PlannerNode::reconstructPath(const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from, CellIndex current, std::vector<CellIndex>& path) {
  path.clear();
  path.push_back(current);
  
  while (came_from.find(current) != came_from.end()) {
    current = came_from.at(current);
    path.insert(path.begin(), current);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
