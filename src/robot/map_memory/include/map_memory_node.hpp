#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void timerCallback();

    void updateMap();

  private:
    robot::MapMemoryCore map_memory_;

    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;

    double robot_x_, robot_y_;
    double theta_;
    double prev_x_, prev_y_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool update_map_ = false;
    bool costmap_updated_ = false;

    static constexpr uint32_t MAP_PUB_RATE = 1000;
    static constexpr double DIST_UPDATE = 1.5;
    static constexpr double MAP_RES = 0.1;
    static constexpr int MAP_WIDTH = 300;
    static constexpr int MAP_HEIGHT = 300;
    static constexpr double MAP_ORIGIN_X = 15;
    static constexpr double MAP_ORIGIN_Y = 15;
};

#endif 
