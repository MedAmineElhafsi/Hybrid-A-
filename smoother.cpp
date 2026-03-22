#include "hybrid_astar_cpp/smoother.hpp"
#include <cmath>

PathSmoother::PathSmoother(double weight_data, double weight_smooth, double weight_obstacle, double obstacle_threshold)
    : w_data_(weight_data), w_smooth_(weight_smooth), w_obs_(weight_obstacle), obs_thresh_(obstacle_threshold) {}

void PathSmoother::smoothPath(std::vector<Pose2D>& path, std::shared_ptr<GridCollision> collision_checker) {
    if (path.size() < 3) return;

    std::vector<Pose2D> new_path = path;
    double tolerance = 0.01;
    double change = tolerance;

    int iterations = 0;
    while (change >= tolerance && iterations < max_iterations_) {
        change = 0.0;
        
        // Don't modify the start and end points
        for (size_t i = 1; i < path.size() - 1; ++i) {
            Pose2D pt = new_path[i];
            Pose2D orig_pt = path[i];

            // 1. Data Term (Stay close to original path)
            double dx_data = w_data_ * (orig_pt.x - pt.x);
            double dy_data = w_data_ * (orig_pt.y - pt.y);

            // 2. Smoothness Term (Rubber band effect)
            double dx_smooth = w_smooth_ * (new_path[i-1].x + new_path[i+1].x - 2.0 * pt.x);
            double dy_smooth = w_smooth_ * (new_path[i-1].y + new_path[i+1].y - 2.0 * pt.y);

            // 3. Obstacle Term (Push away from walls using the Voronoi/Distance Map)
            double dx_obs = 0.0, dy_obs = 0.0;
            double dist = collision_checker->getHeuristicCost(pt.x, pt.y);
            
            if (dist < obs_thresh_ && dist > 0.01) {
                // Approximate gradient of distance field
                double dist_x = collision_checker->getHeuristicCost(pt.x + 0.1, pt.y);
                double dist_y = collision_checker->getHeuristicCost(pt.x, pt.y + 0.1);
                
                double grad_x = (dist_x - dist) / 0.1;
                double grad_y = (dist_y - dist) / 0.1;
                
                double push_force = w_obs_ * (dist - obs_thresh_);
                dx_obs = push_force * grad_x;
                dy_obs = push_force * grad_y;
            }

            // Apply updates
            double new_x = pt.x + dx_data + dx_smooth + dx_obs;
            double new_y = pt.y + dy_data + dy_smooth + dy_obs;

            // Recalculate yaw to face the next point
            double new_yaw = std::atan2(new_path[i+1].y - new_y, new_path[i+1].x - new_x);

            // Ensure we didn't push the path INTO a collision
            if (collision_checker->isCollisionFree(new_x, new_y, new_yaw)) {
                change += std::abs(pt.x - new_x) + std::abs(pt.y - new_y);
                new_path[i].x = new_x;
                new_path[i].y = new_y;
                new_path[i].yaw = new_yaw;
            }
        }
        iterations++;
    }

    path = new_path;
}