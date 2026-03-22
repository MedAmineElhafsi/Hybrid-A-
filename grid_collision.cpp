#include "hybrid_astar_cpp/grid_collision.hpp"
#include <algorithm>
#include <queue>

namespace {
bool rotatedRectIntersectsCellSquare(double rect_cx, double rect_cy, double yaw,
                                     double half_l, double half_w,
                                     double cell_cx, double cell_cy,
                                     double half_cell) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double abs_c = std::abs(c);
    const double abs_s = std::abs(s);

    const double dx = cell_cx - rect_cx;
    const double dy = cell_cy - rect_cy;

    const double proj_rect_x = std::abs(dx * c + dy * s);
    const double proj_rect_y = std::abs(-dx * s + dy * c);

    if (proj_rect_x > half_l + half_cell * (abs_c + abs_s)) {
        return false;
    }
    if (proj_rect_y > half_w + half_cell * (abs_c + abs_s)) {
        return false;
    }
    if (std::abs(dx) > half_cell + half_l * abs_c + half_w * abs_s) {
        return false;
    }
    if (std::abs(dy) > half_cell + half_l * abs_s + half_w * abs_c) {
        return false;
    }

    return true;
}
}  // namespace

GridCollision::GridCollision(int yaw_bins, double car_length, double car_width, double margin)
    : yaw_bins_(yaw_bins), car_length_(car_length), car_width_(car_width), margin_(margin) {
    // We can't precompute the LUT until we have a grid resolution, 
    // so we will call precomputeFootprintLUT() inside updateGrid the first time.
}

void GridCollision::updateGrid(const nav_msgs::msg::OccupancyGrid& grid) {
    bool resolution_changed = (grid_.info.resolution != grid.info.resolution);
    grid_ = grid;
    
    if ((resolution_changed || footprint_lut_.empty()) && grid_.info.resolution > 0) {
        precomputeFootprintLUT();
    }
    if (grid_.info.resolution > 0) {
        computeObstacleDistanceMap();
    }
}

void GridCollision::precomputeFootprintLUT() {
    footprint_lut_.clear();
    footprint_lut_.resize(yaw_bins_);

    double res = grid_.info.resolution;
    double half_l = (car_length_ / 2.0) + margin_;
    double half_w = (car_width_ / 2.0) + margin_;
    const double cell_half = res * 0.5;

    for (int i = 0; i < yaw_bins_; ++i) {
        double yaw = (i * 2.0 * M_PI) / yaw_bins_;
        double cos_y = std::cos(yaw);
        double sin_y = std::sin(yaw);

        // Conservatively include any cell whose square can intersect the
        // rotated footprint by inflating the rectangle by half a cell.
        const double expanded_half_l = half_l + cell_half;
        const double expanded_half_w = half_w + cell_half;
        const int max_dx = static_cast<int>(std::ceil(
            (std::abs(cos_y) * expanded_half_l + std::abs(sin_y) * expanded_half_w) / res));
        const int max_dy = static_cast<int>(std::ceil(
            (std::abs(sin_y) * expanded_half_l + std::abs(cos_y) * expanded_half_w) / res));

        for (int dx = -max_dx; dx <= max_dx; ++dx) {
            for (int dy = -max_dy; dy <= max_dy; ++dy) {
                const double wx = static_cast<double>(dx) * res;
                const double wy = static_cast<double>(dy) * res;

                const double local_x = wx * cos_y + wy * sin_y;
                const double local_y = -wx * sin_y + wy * cos_y;

                if (std::abs(local_x) <= expanded_half_l &&
                    std::abs(local_y) <= expanded_half_w) {
                    footprint_lut_[i].push_back({dx, dy});
                }
            }
        }
    }
}

int GridCollision::getYawBin(double yaw) const {
    // Normalize yaw to [0, 2PI)
    yaw = std::fmod(yaw, 2.0 * M_PI);
    if (yaw < 0) yaw += 2.0 * M_PI;
    
    int bin = std::round((yaw / (2.0 * M_PI)) * yaw_bins_);
    return bin % yaw_bins_;
}

bool GridCollision::worldToGrid(double wx, double wy, int& gx, int& gy) const {
    if (grid_.info.resolution <= 0) return false;
    gx = std::floor((wx - grid_.info.origin.position.x) / grid_.info.resolution);
    gy = std::floor((wy - grid_.info.origin.position.y) / grid_.info.resolution);
    
    return (gx >= 0 && gx < (int)grid_.info.width && gy >= 0 && gy < (int)grid_.info.height);
}

int GridCollision::getIndex(int gx, int gy) const {
    return gy * grid_.info.width + gx;
}

bool GridCollision::isOccupiedCell(int gx, int gy) const {
    if (gx < 0 || gx >= static_cast<int>(grid_.info.width) ||
        gy < 0 || gy >= static_cast<int>(grid_.info.height)) {
        return true;
    }
    return grid_.data[getIndex(gx, gy)] > 50;
}

bool GridCollision::isCollisionFree(double wx, double wy, double yaw) const {
    int cx, cy;
    if (!worldToGrid(wx, wy, cx, cy)) return false; // Out of bounds

    const double res = grid_.info.resolution;
    const double half_cell = res * 0.5;
    const double half_l = (car_length_ * 0.5) + margin_;
    const double half_w = (car_width_ * 0.5) + margin_;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    const double bbox_half_x = std::abs(c) * half_l + std::abs(s) * half_w + half_cell;
    const double bbox_half_y = std::abs(s) * half_l + std::abs(c) * half_w + half_cell;

    const int min_gx = static_cast<int>(std::floor(
        (wx - bbox_half_x - grid_.info.origin.position.x) / res));
    const int max_gx = static_cast<int>(std::floor(
        (wx + bbox_half_x - grid_.info.origin.position.x) / res));
    const int min_gy = static_cast<int>(std::floor(
        (wy - bbox_half_y - grid_.info.origin.position.y) / res));
    const int max_gy = static_cast<int>(std::floor(
        (wy + bbox_half_y - grid_.info.origin.position.y) / res));

    if (min_gx < 0 || min_gy < 0 ||
        max_gx >= static_cast<int>(grid_.info.width) ||
        max_gy >= static_cast<int>(grid_.info.height)) {
        return false;
    }

    for (int gx = min_gx; gx <= max_gx; ++gx) {
        for (int gy = min_gy; gy <= max_gy; ++gy) {
            if (!isOccupiedCell(gx, gy)) {
                continue;
            }

            const double cell_cx = grid_.info.origin.position.x + (static_cast<double>(gx) + 0.5) * res;
            const double cell_cy = grid_.info.origin.position.y + (static_cast<double>(gy) + 0.5) * res;

            if (rotatedRectIntersectsCellSquare(
                    wx, wy, yaw, half_l, half_w, cell_cx, cell_cy, half_cell)) {
                return false;
            }
        }
    }

    return true;
}

void GridCollision::computeDistanceMap(double goal_wx, double goal_wy) {
    int w = grid_.info.width;
    int h = grid_.info.height;
    distance_map_.assign(w * h, std::numeric_limits<double>::infinity());

    int gx, gy;
    if (!worldToGrid(goal_wx, goal_wy, gx, gy)) return;

    // Standard BFS using a queue
    std::queue<Cell> q;
    q.push({gx, gy});
    distance_map_[getIndex(gx, gy)] = 0.0;

    int dirs[4][2] = {{1,0}, {-1,0}, {0,1}, {0,-1}};

    while (!q.empty()) {
        Cell curr = q.front();
        q.pop();

        int curr_idx = getIndex(curr.x, curr.y);
        double dist = distance_map_[curr_idx];

        for (auto& d : dirs) {
            int nx = curr.x + d[0];
            int ny = curr.y + d[1];

            if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                int n_idx = getIndex(nx, ny);
                if (!isOccupiedCell(nx, ny)) {
                    if (dist + 1.0 < distance_map_[n_idx]) {
                        distance_map_[n_idx] = dist + 1.0;
                        q.push({nx, ny});
                    }
                }
            }
        }
    }
}

double GridCollision::getHeuristicCost(double wx, double wy) const {
    int gx, gy;
    if (!worldToGrid(wx, wy, gx, gy)) return std::numeric_limits<double>::infinity();
    
    double grid_dist = distance_map_[getIndex(gx, gy)];
    return grid_dist * grid_.info.resolution; // Convert back to meters
}

void GridCollision::computeObstacleDistanceMap() {
    const int w = grid_.info.width;
    const int h = grid_.info.height;
    const double res = grid_.info.resolution;
    obstacle_distance_map_.assign(w * h, std::numeric_limits<double>::infinity());

    using QueueEntry = std::pair<double, Cell>;
    auto cmp = [](const QueueEntry& a, const QueueEntry& b) { return a.first > b.first; };
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, decltype(cmp)> open(cmp);

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (isOccupiedCell(x, y)) {
                const int idx = getIndex(x, y);
                obstacle_distance_map_[idx] = 0.0;
                open.push({0.0, {x, y}});
            }
        }
    }

    const int dirs[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };

    while (!open.empty()) {
        const auto [dist, cell] = open.top();
        open.pop();

        const int idx = getIndex(cell.x, cell.y);
        if (dist > obstacle_distance_map_[idx]) {
            continue;
        }

        for (const auto& dir : dirs) {
            const int nx = cell.x + dir[0];
            const int ny = cell.y + dir[1];
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) {
                continue;
            }

            const double step = ((dir[0] == 0 || dir[1] == 0) ? 1.0 : std::sqrt(2.0)) * res;
            const double candidate = dist + step;
            const int n_idx = getIndex(nx, ny);
            if (candidate < obstacle_distance_map_[n_idx]) {
                obstacle_distance_map_[n_idx] = candidate;
                open.push({candidate, {nx, ny}});
            }
        }
    }
}

double GridCollision::getObstacleDistance(double wx, double wy) const {
    int gx, gy;
    if (!worldToGrid(wx, wy, gx, gy)) {
        return 0.0;
    }
    return obstacle_distance_map_[getIndex(gx, gy)];
}
