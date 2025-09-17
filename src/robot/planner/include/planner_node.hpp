#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "planner_core.hpp"

struct CellIndex {
  int x, y;

  CellIndex(int x, int y) : x(x), y(y) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex& other) const {
    return (x == other.x && y == other.y);
  }
  bool operator!=(const CellIndex& other) const {
    return (x != other.x || y != other.y);
  }
};

struct CellIndexHash {
  std::size_t operator()(const CellIndex& idx) const {
    return std::hash<int>()(idx.x) & (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode {
  CellIndex idx;
  double f;

  AStarNode(CellIndex idx, double f) : idx(idx), f(f) {}
};

struct CompareF {
  bool operator()(const AStarNode& a, const AStarNode& b) {
    return a.f > b.f;
  }
};

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    enum class State {
      WAITING_FOR_GOAL,
      REACHING_GOAL
    };
    State state_;

    robot::PlannerCore planner_;

    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;

    bool goal_received_ = false;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    static constexpr int PATH_TIMEOUT = 500;
    static constexpr double SETTLE_RADIUS = 0.5;
    static constexpr double COST_WEIGHTING = 0.1;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    bool goalReached();

    void planPath();
    void reconstructPath(const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from, CellIndex current, std::vector<CellIndex>& path);
};

#endif 
