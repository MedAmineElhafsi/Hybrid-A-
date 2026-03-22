#ifndef HYBRID_ASTAR_CPP_PLANNER_NODE_HPP_
#define HYBRID_ASTAR_CPP_PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <mutex>

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode();

private:
    void gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void planPath();

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr footprint_pub_;

    std::mutex state_mutex_;
    nav_msgs::msg::OccupancyGrid current_grid_;
    geometry_msgs::msg::Pose start_pose_;
    geometry_msgs::msg::Pose goal_pose_;
    bool has_grid_ = false;
    bool has_start_ = false;
    bool has_goal_ = false;
    
    double xy_res_;
    double min_turning_radius_;
};

#endif // HYBRID_ASTAR_CPP_PLANNER_NODE_HPP_