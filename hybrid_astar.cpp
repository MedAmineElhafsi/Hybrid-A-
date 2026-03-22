#include "hybrid_astar_cpp/hybrid_astar.hpp"
#include "hybrid_astar_cpp/curves.hpp"
#include <cmath>
#include <queue>
#include <unordered_map>
#include <algorithm>

namespace {
Pose2D integrateBicycleStep(const Pose2D& pose, double distance, double steer, double wheelbase) {
    Pose2D next = pose;
    const double curvature = std::tan(steer) / std::max(1e-6, wheelbase);

    if (std::abs(curvature) < 1e-9) {
        next.x += distance * std::cos(pose.yaw);
        next.y += distance * std::sin(pose.yaw);
    } else {
        const double next_yaw = AnalyticCurves::wrapAngle(pose.yaw + distance * curvature);
        next.x += (std::sin(next_yaw) - std::sin(pose.yaw)) / curvature;
        next.y += (-std::cos(next_yaw) + std::cos(pose.yaw)) / curvature;
        next.yaw = next_yaw;
    }

    next.yaw = AnalyticCurves::wrapAngle(next.yaw);
    return next;
}

bool rolloutMotion(const Pose2D& start, double distance, double steer, double wheelbase,
                   double collision_check_step,
                   const std::shared_ptr<GridCollision>& collision_checker,
                   Pose2D& end_pose) {
    const int rollout_steps = std::max(1, static_cast<int>(
        std::ceil(std::abs(distance) / std::max(1e-6, collision_check_step))));
    const double substep_distance = distance / static_cast<double>(rollout_steps);

    Pose2D pose = start;
    for (int i = 0; i < rollout_steps; ++i) {
        pose = integrateBicycleStep(pose, substep_distance, steer, wheelbase);
        if (!collision_checker->isCollisionFree(pose.x, pose.y, pose.yaw)) {
            return false;
        }
    }

    end_pose = pose;
    return true;
}
}  // namespace

HybridAStar::HybridAStar(double step_size, double max_steer, int steer_samples, double wheelbase, double xy_res,
                         int yaw_bins, double clearance_distance, double clearance_weight,
                         double clearance_relaxation_radius)
    : step_size_(step_size), max_steer_(max_steer), steer_samples_(steer_samples),
      wheelbase_(wheelbase), xy_res_(xy_res), yaw_bins_(yaw_bins),
      clearance_distance_(clearance_distance), clearance_weight_(clearance_weight),
      clearance_relaxation_radius_(clearance_relaxation_radius) {}

namespace {
double goalHeuristic(const std::shared_ptr<GridCollision>& collision_checker,
                     double x, double y, const Pose2D& goal) {
    const double grid_h = collision_checker->getHeuristicCost(x, y);
    if (std::isfinite(grid_h)) {
        return grid_h;
    }
    return std::hypot(x - goal.x, y - goal.y);
}

double clearancePenalty(const std::shared_ptr<GridCollision>& collision_checker,
                        double x, double y, double desired_clearance, double weight,
                        const Pose2D& start, const Pose2D& goal,
                        double relaxation_radius) {
    if (desired_clearance <= 0.0 || weight <= 0.0) {
        return 0.0;
    }

    const double obstacle_distance = collision_checker->getObstacleDistance(x, y);
    if (!std::isfinite(obstacle_distance) || obstacle_distance >= desired_clearance) {
        return 0.0;
    }

    const double deficit = desired_clearance - obstacle_distance;
    double relaxation_scale = 1.0;
    if (relaxation_radius > 0.0) {
        const double start_dist = std::hypot(x - start.x, y - start.y);
        const double goal_dist = std::hypot(x - goal.x, y - goal.y);
        const double terminal_dist = std::min(start_dist, goal_dist);
        relaxation_scale = std::min(1.0, terminal_dist / relaxation_radius);
    }

    return weight * deficit * deficit * relaxation_scale;
}
}  // namespace

StateKey HybridAStar::poseToKey(double x, double y, double yaw) const {
    StateKey key;
    key.x_idx = static_cast<int>(std::round(x / xy_res_));
    key.y_idx = static_cast<int>(std::round(y / xy_res_));
    key.yaw_idx = static_cast<int>(std::round((AnalyticCurves::wrapAngle(yaw) / (2.0 * M_PI)) * yaw_bins_)) % yaw_bins_;
    if (key.yaw_idx < 0) key.yaw_idx += yaw_bins_;
    return key;
}

bool HybridAStar::plan(const Pose2D& start, const Pose2D& goal, 
                       std::shared_ptr<GridCollision> collision_checker, 
                       std::vector<Pose2D>& final_path) {
    if (!collision_checker->isCollisionFree(start.x, start.y, start.yaw) ||
        !collision_checker->isCollisionFree(goal.x, goal.y, goal.yaw)) {
        return false;
    }
    
    // Priority queue comparing f_cost
    auto comp = [](const std::shared_ptr<Node3D>& a, const std::shared_ptr<Node3D>& b) {
        return a->f_cost > b->f_cost; 
    };
    std::priority_queue<std::shared_ptr<Node3D>, std::vector<std::shared_ptr<Node3D>>, decltype(comp)> open_set(comp);
    
    // Map to keep track of visited nodes using your custom StateKeyHash
    std::unordered_map<StateKey, std::shared_ptr<Node3D>, StateKeyHash> all_nodes;

    // Create the start node using your specific constructor: (x, y, yaw, g, f, steer, dir, parent)
    double start_h = goalHeuristic(collision_checker, start.x, start.y, goal);
    auto start_node = std::make_shared<Node3D>(start.x, start.y, start.yaw, 0.0, start_h, 0.0, 1, nullptr);
    
    StateKey start_key = poseToKey(start.x, start.y, start.yaw);
    all_nodes[start_key] = start_node;
    open_set.push(start_node);

    std::shared_ptr<Node3D> best_goal_node = nullptr;
    std::vector<Pose2D> analytic_suffix;
    int expansions = 0;

    std::vector<double> steer_angles;
    if (steer_samples_ > 1) {
        double d_steer = 2.0 * max_steer_ / (steer_samples_ - 1);
        for (int i = 0; i < steer_samples_; ++i) steer_angles.push_back(-max_steer_ + i * d_steer);
    } else {
        steer_angles.push_back(0.0);
    }

    const double collision_check_step = std::max(0.05, std::min(xy_res_, step_size_ / 4.0));

    while (!open_set.empty()) {
        auto current = open_set.top();
        open_set.pop();

        // Check if we've reached the goal threshold directly
        if (std::hypot(current->x - goal.x, current->y - goal.y) < 0.5 && 
            std::abs(AnalyticCurves::wrapAngle(current->yaw - goal.yaw)) < 0.2) {
            best_goal_node = current;
            break;
        }

        expansions++;

        // 4. Analytic Expansion (The "Shot" with Endpoint Sanity Check)
        if (expansions % 10 == 0 && std::hypot(current->x - goal.x, current->y - goal.y) < 8.0) {
            double min_turn_rad = wheelbase_ / std::max(1e-6, std::tan(max_steer_));
            
            std::vector<Pose2D> dubins_path;
            std::vector<Pose2D> rs_path;
            std::vector<int> rs_gears;
            
            bool dubins_valid = AnalyticCurves::getDubinsPath({current->x, current->y, current->yaw}, goal, min_turn_rad, 0.05, dubins_path);
            bool rs_valid = AnalyticCurves::getReedsSheppPath({current->x, current->y, current->yaw}, goal, min_turn_rad, 0.05, rs_path, rs_gears);

            auto get_path_length = [](const std::vector<Pose2D>& p) {
                double len = 0.0;
                for (size_t i = 1; i < p.size(); ++i) len += std::hypot(p[i].x - p[i-1].x, p[i].y - p[i-1].y);
                return len;
            };

            auto get_clearance_cost = [&](const std::vector<Pose2D>& p) {
                double cost = 0.0;
                for (const auto& pt : p) {
                    cost += clearancePenalty(
                        collision_checker, pt.x, pt.y, clearance_distance_, clearance_weight_,
                        start, goal, clearance_relaxation_radius_) * 0.05;
                }
                return cost;
            };

            auto is_endpoint_correct = [](const std::vector<Pose2D>& p, const Pose2D& g) {
                if (p.empty()) return false;
                return std::hypot(p.back().x - g.x, p.back().y - g.y) < 0.2; 
            };

            double dxy = std::hypot(current->x - goal.x, current->y - goal.y);
            double best_shot_cost = std::numeric_limits<double>::infinity();
            std::vector<Pose2D> winning_shot;

            // --- EVALUATE DUBINS ---
            if (dubins_valid && is_endpoint_correct(dubins_path, goal)) {
                double len = get_path_length(dubins_path);
                double clearance_cost = get_clearance_cost(dubins_path);
                if (!(dxy < 2.0 && len > 3.0 * std::max(dxy, 0.1))) {
                    bool safe = true;
                    for (const auto& pt : dubins_path) if (!collision_checker->isCollisionFree(pt.x, pt.y, pt.yaw)) { safe = false; break; }
                    if (safe && (len + clearance_cost) < best_shot_cost) {
                        best_shot_cost = len + clearance_cost;
                        winning_shot = dubins_path;
                    }
                }
            }

            // --- EVALUATE REEDS-SHEPP ---
            if (rs_valid && is_endpoint_correct(rs_path, goal)) {
                double len = get_path_length(rs_path);
                double cost = len * 1.1 + get_clearance_cost(rs_path);
                if (!(dxy < 2.0 && len > 3.0 * std::max(dxy, 0.1))) {
                    bool safe = true;
                    for (const auto& pt : rs_path) if (!collision_checker->isCollisionFree(pt.x, pt.y, pt.yaw)) { safe = false; break; }
                    if (safe && cost < best_shot_cost) { best_shot_cost = cost; winning_shot = rs_path; }
                }
            }

            // --- DECLARE THE WINNER ---
            if (!winning_shot.empty()) {
                best_goal_node = current;
                analytic_suffix = winning_shot;
                break; 
            }
        }

        // 5. Expand neighbors by rolling out a car-like arc instead of a
        // straight translation with a snapped heading update.
        int dirs[] = {1, -1};
        for (int dir : dirs) {
            for (double steer : steer_angles) {
                Pose2D next_pose;
                if (!rolloutMotion({current->x, current->y, current->yaw},
                                   dir * step_size_, steer, wheelbase_,
                                   collision_check_step, collision_checker, next_pose)) {
                    continue;
                }

                const double next_x = next_pose.x;
                const double next_y = next_pose.y;
                const double next_yaw = next_pose.yaw;

                StateKey key = poseToKey(next_x, next_y, next_yaw);
                
                double step_cost = step_size_;
                if (dir == -1) {
                    step_cost *= (1.0 + penalty_reverse_);
                }
                step_cost += penalty_steer_ * std::abs(steer);
                if (current->parent) {
                    step_cost += penalty_steer_change_ * std::abs(steer - current->steer);
                    if (current->direction != dir) {
                        step_cost += 0.5; // Penalty for changing directions
                    }
                }
                step_cost += clearancePenalty(
                    collision_checker, next_x, next_y, clearance_distance_, clearance_weight_,
                    start, goal, clearance_relaxation_radius_);

                double new_g = current->g_cost + step_cost;
                double h = goalHeuristic(collision_checker, next_x, next_y, goal);

                auto it = all_nodes.find(key);
                if (it == all_nodes.end() || new_g < it->second->g_cost) {
                    // Create using your constructor
                    auto next_node = std::make_shared<Node3D>(next_x, next_y, next_yaw, new_g, new_g + h, steer, dir, current);
                    
                    all_nodes[key] = next_node;
                    open_set.push(next_node);
                }
            }
        }
    }

    // 6. Reconstruct Path
    if (best_goal_node) {
        auto curr = best_goal_node;
        std::vector<Pose2D> path_prefix;
        while (curr) {
            path_prefix.push_back({curr->x, curr->y, curr->yaw});
            curr = curr->parent;
        }
        std::reverse(path_prefix.begin(), path_prefix.end());
        
        final_path = path_prefix;
        final_path.insert(final_path.end(), analytic_suffix.begin(), analytic_suffix.end());
        return true;
    }

    return false;
}
