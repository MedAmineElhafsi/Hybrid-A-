#include "hybrid_astar_cpp/planner_node.hpp"
#include "hybrid_astar_cpp/grid_collision.hpp"
#include "hybrid_astar_cpp/hybrid_astar.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <cmath>

PlannerNode::PlannerNode() : Node("hybrid_astar_cpp_node") {
    // 1. Declare Parameters
    this->declare_parameter("xy_resolution", 0.25);
    this->declare_parameter("min_turning_radius", 1.0);
    this->declare_parameter("car_length", 0.9);
    this->declare_parameter("car_width", 0.45);
    this->declare_parameter("plan_margin", 0.04);
    this->declare_parameter("hard_obstacle_margin", 0.0);
    this->declare_parameter("footprint_display_margin", 0.0);
    this->declare_parameter("footprint_spacing", 0.4);
    this->declare_parameter("planner_step_size", 0.2);
    this->declare_parameter("planner_wheelbase", 0.5);
    this->declare_parameter("soft_obstacle_margin", 0.15);
    this->declare_parameter("obstacle_clearance_weight", 3.0);
    this->declare_parameter("clearance_relaxation_radius", 1.0);
    
    xy_res_ = this->get_parameter("xy_resolution").as_double();
    min_turning_radius_ = this->get_parameter("min_turning_radius").as_double();

    // 2. Setup Publishers
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("astar_path", 10);
    footprint_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("astar_footprints", 10);

    // 3. Setup Subscribers
    grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "astar_grid", 10, std::bind(&PlannerNode::gridCallback, this, std::placeholders::_1));
        
    start_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10, std::bind(&PlannerNode::initialPoseCallback, this, std::placeholders::_1));
        
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&PlannerNode::goalPoseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "C++ Hybrid A* Backend Initialized. Waiting for Grid, Start, and Goal...");
}

void PlannerNode::gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    bool should_replan = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_grid_ = *msg;
        has_grid_ = true;
        should_replan = has_start_ && has_goal_;
    }

    if (should_replan) {
        planPath();
    }
}

void PlannerNode::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    bool should_replan = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        start_pose_ = msg->pose.pose;
        has_start_ = true;
        should_replan = has_grid_ && has_goal_;
    }
    RCLCPP_INFO(this->get_logger(), "Start pose received.");
    if (should_replan) {
        planPath();
    }
}

void PlannerNode::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    bool should_replan = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        goal_pose_ = msg->pose;
        has_goal_ = true;
        should_replan = has_grid_ && has_start_;
    }
    RCLCPP_INFO(this->get_logger(), "Goal pose received.");
    if (should_replan) {
        planPath();
    }
}

void PlannerNode::planPath() {
    nav_msgs::msg::OccupancyGrid grid_snapshot;
    geometry_msgs::msg::Pose start_pose_snapshot;
    geometry_msgs::msg::Pose goal_pose_snapshot;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!has_grid_ || !has_start_ || !has_goal_) {
            return;
        }
        grid_snapshot = current_grid_;
        start_pose_snapshot = start_pose_;
        goal_pose_snapshot = goal_pose_;
    }

    RCLCPP_INFO(this->get_logger(), "Starting C++ Hybrid A* planning...");
    auto start_time = std::chrono::high_resolution_clock::now();

    // 1. Initialize the Collision Checker
    double c_len = this->get_parameter("car_length").as_double();
    double c_wid = this->get_parameter("car_width").as_double();
    double c_mar = this->get_parameter("plan_margin").as_double();
    double hard_obstacle_margin = std::max(0.0, this->get_parameter("hard_obstacle_margin").as_double());
    double footprint_display_margin = std::max(0.0, this->get_parameter("footprint_display_margin").as_double());
    double footprint_spacing = std::max(0.05, this->get_parameter("footprint_spacing").as_double());
    double planner_step_size = std::max(0.05, this->get_parameter("planner_step_size").as_double());
    double planner_wheelbase = std::max(0.05, this->get_parameter("planner_wheelbase").as_double());
    double soft_obstacle_margin = std::max(0.0, this->get_parameter("soft_obstacle_margin").as_double());
    double obstacle_clearance_weight = std::max(0.0, this->get_parameter("obstacle_clearance_weight").as_double());
    double clearance_relaxation_radius = std::max(0.0, this->get_parameter("clearance_relaxation_radius").as_double());
    min_turning_radius_ = std::max(0.05, this->get_parameter("min_turning_radius").as_double());
    double planner_max_steer = std::atan(planner_wheelbase / min_turning_radius_);
    double collision_margin = c_mar + hard_obstacle_margin;
    double desired_clearance = (c_wid * 0.5) + collision_margin + soft_obstacle_margin;
    
    auto collision_checker = std::make_shared<GridCollision>(72, c_len, c_wid, collision_margin);
    collision_checker->updateGrid(grid_snapshot);

    // 2. Initialize the Planner
    HybridAStar planner(
        planner_step_size, planner_max_steer, 7, planner_wheelbase, xy_res_, 72,
        desired_clearance, obstacle_clearance_weight, clearance_relaxation_radius);

    // 3. Extract Poses
    Pose2D start_pose2d{start_pose_snapshot.position.x, start_pose_snapshot.position.y, tf2::getYaw(start_pose_snapshot.orientation)};
    Pose2D goal_pose2d{goal_pose_snapshot.position.x, goal_pose_snapshot.position.y, tf2::getYaw(goal_pose_snapshot.orientation)};
    collision_checker->computeDistanceMap(goal_pose2d.x, goal_pose2d.y);

    // 4. Run the Search
    std::vector<Pose2D> final_path;
    bool success = planner.plan(start_pose2d, goal_pose2d, collision_checker, final_path);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    // 5. Construct the ROS 2 Messages
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";

    visualization_msgs::msg::MarkerArray marker_array;

    if (success && !final_path.empty()) {
        RCLCPP_INFO(this->get_logger(), "Path found! Length: %zu nodes. Time: %ld ms", final_path.size(), duration.count());
        
        int id_counter = 0;
        double distance_since_last_footprint = 0.0;
        for (size_t i = 0; i < final_path.size(); ++i) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path_msg.header;
            ps.pose.position.x = final_path[i].x;
            ps.pose.position.y = final_path[i].y;
            ps.pose.position.z = 0.0;
            
            tf2::Quaternion q;
            q.setRPY(0, 0, final_path[i].yaw);
            ps.pose.orientation = tf2::toMsg(q);
            path_msg.poses.push_back(ps);

            if (i > 0) {
                distance_since_last_footprint += std::hypot(
                    final_path[i].x - final_path[i - 1].x,
                    final_path[i].y - final_path[i - 1].y);
            }

            const bool is_last_pose = (i == final_path.size() - 1);
            const bool should_publish_footprint =
                (i == 0) || (distance_since_last_footprint >= footprint_spacing) || is_last_pose;

            // Build footprint boxes at a fixed traveled-distance spacing so
            // dense analytic samples do not collapse into a solid ribbon in RViz.
            if (should_publish_footprint) {
                visualization_msgs::msg::Marker m;
                m.header = path_msg.header;
                m.ns = "car_footprints";
                m.id = id_counter++;
                m.type = visualization_msgs::msg::Marker::CUBE;
                m.action = visualization_msgs::msg::Marker::ADD;
                m.pose = ps.pose;
                m.pose.position.z = 0.05;
                m.scale.x = c_len + 2.0 * footprint_display_margin;
                m.scale.y = c_wid + 2.0 * footprint_display_margin;
                m.scale.z = 0.05;

                double dx = 0, dy = 0;
                if (i + 1 < final_path.size()) {
                    dx = final_path[i+1].x - final_path[i].x;
                    dy = final_path[i+1].y - final_path[i].y;
                } else if (i > 0) {
                    dx = final_path[i].x - final_path[i-1].x;
                    dy = final_path[i].y - final_path[i-1].y;
                }
                
                bool is_reversing = (dx * std::cos(final_path[i].yaw) + dy * std::sin(final_path[i].yaw)) < -0.01;
                if (is_reversing) {
                    m.color.r = 1.0; m.color.g = 0.35; m.color.b = 0.2; m.color.a = 0.40; 
                } else {
                    m.color.r = 0.1; m.color.g = 0.7; m.color.b = 1.0; m.color.a = 0.30; 
                }
                marker_array.markers.push_back(m);
                distance_since_last_footprint = 0.0;
            }
        }
        
        // Target specifically the old markers to delete them (solves the DELETEALL bug)
        for (int j = id_counter; j < id_counter + 300; ++j) {
            visualization_msgs::msg::Marker m_del;
            m_del.header = path_msg.header;
            m_del.ns = "car_footprints";
            m_del.id = j;
            m_del.action = visualization_msgs::msg::Marker::DELETE;
            marker_array.markers.push_back(m_del);
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Planner failed to find a valid path. Time: %ld ms", duration.count());
        for (int j = 0; j < 300; ++j) {
            visualization_msgs::msg::Marker m_del;
            m_del.header = path_msg.header;
            m_del.ns = "car_footprints";
            m_del.id = j;
            m_del.action = visualization_msgs::msg::Marker::DELETE;
            marker_array.markers.push_back(m_del);
        }
    }

    path_pub_->publish(path_msg);
    footprint_pub_->publish(marker_array);
}
