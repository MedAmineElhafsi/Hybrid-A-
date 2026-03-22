#ifndef HYBRID_ASTAR_CPP_SMOOTHER_HPP_
#define HYBRID_ASTAR_CPP_SMOOTHER_HPP_

#include <vector>
#include <memory>
#include "hybrid_astar_cpp/curves.hpp"
#include "hybrid_astar_cpp/grid_collision.hpp"

class PathSmoother {
public:
    PathSmoother(double weight_data, double weight_smooth, double weight_obstacle, double obstacle_threshold);

    void smoothPath(std::vector<Pose2D>& path, std::shared_ptr<GridCollision> collision_checker);

private:
    double w_data_;
    double w_smooth_;
    double w_obs_;
    double obs_thresh_;
    int max_iterations_ = 50;
};

#endif // HYBRID_ASTAR_CPP_SMOOTHER_HPP_