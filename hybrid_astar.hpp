#ifndef HYBRID_ASTAR_CPP_HYBRID_ASTAR_HPP_
#define HYBRID_ASTAR_CPP_HYBRID_ASTAR_HPP_

#include <vector>
#include <memory>
#include <queue>
#include <unordered_map>
#include "hybrid_astar_cpp/grid_collision.hpp"
#include "hybrid_astar_cpp/curves.hpp"

// 3D Grid Key for the Closed List
struct StateKey {
    int x_idx;
    int y_idx;
    int yaw_idx;

    bool operator==(const StateKey& other) const {
        return x_idx == other.x_idx && y_idx == other.y_idx && yaw_idx == other.yaw_idx;
    }
};

// Custom Hash for the 3D StateKey to make unordered_map lightning fast
struct StateKeyHash {
    std::size_t operator()(const StateKey& k) const {
        return ((std::hash<int>()(k.x_idx) ^ (std::hash<int>()(k.y_idx) << 1)) >> 1) ^ (std::hash<int>()(k.yaw_idx) << 1);
    }
};

// Represents a single node in the A* search tree
struct Node3D {
    double x, y, yaw;
    double g_cost;
    double f_cost;
    double steer;
    int direction;
    std::shared_ptr<Node3D> parent;

    Node3D(double x, double y, double yaw, double g, double f, double s, int dir, std::shared_ptr<Node3D> p)
        : x(x), y(y), yaw(yaw), g_cost(g), f_cost(f), steer(s), direction(dir), parent(p) {}
};

// Comparator for the Priority Queue (Lowest f_cost first)
struct NodeComparator {
    bool operator()(const std::shared_ptr<Node3D>& a, const std::shared_ptr<Node3D>& b) const {
        return a->f_cost > b->f_cost;
    }
};

class HybridAStar {
public:
    HybridAStar(double step_size, double max_steer, int steer_samples, double wheelbase, double xy_res,
                int yaw_bins, double clearance_distance, double clearance_weight,
                double clearance_relaxation_radius);

    // Main Planning Function
    bool plan(const Pose2D& start, const Pose2D& goal, 
              std::shared_ptr<GridCollision> collision_checker,
              std::vector<Pose2D>& out_path);

private:
    StateKey poseToKey(double x, double y, double yaw) const;
    double normalizeYaw(double yaw) const;

    double step_size_;
    double max_steer_;
    int steer_samples_;
    double wheelbase_;
    double xy_res_;
    int yaw_bins_;
    
    // Penalties
    double penalty_reverse_ = 0.2;
    double penalty_steer_ = 0.005;
    double penalty_steer_change_ = 0.04;
    double clearance_distance_;
    double clearance_weight_;
    double clearance_relaxation_radius_;
};

#endif // HYBRID_ASTAR_CPP_HYBRID_ASTAR_HPP_
