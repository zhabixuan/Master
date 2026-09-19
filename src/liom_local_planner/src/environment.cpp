#include "liom_local_planner/environment.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include <bitset>
#include <cmath>

namespace liom_local_planner {

#define MY_CORRIDOR

Environment::Environment(std::shared_ptr<PlannerConfig> config, 
            const nav2_costmap_2d::Costmap2D* costmap)
            : config_(config) {
    setCostmap(costmap);
}
bool Environment::CheckBoxCollision(double time, const math::AABox2d &box) const {

    for (auto& polygon : polygons_) {
        if (polygon.HasOverlap(math::Box2d(box))) {
            return true;
        }
    }

    // Query the obstacle-cell R-tree with early exit. `bgi::intersects` already
    // performs the exact point-in-box test, so the iterator is non-end iff a
    // lethal cell center lies inside the query box — no result container and no
    // second IsPointIn pass are needed.
    const auto query_box = boost::geometry::model::box<Point>(
        Point(box.min_x(), box.min_y()),
        Point(box.max_x(), box.max_y()));
    const auto predicate = bgi::intersects(query_box);
    return obstacle_tree_.qbegin(predicate) != obstacle_tree_.qend();
}

bool Environment::CheckPoseCollision(double time, const math::Pose &pose) const {
    const auto &coeffs = config_->vehicle.disc_coefficients;
    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    const double wh = config_->vehicle.disc_radius * 2;
    for (int i = 0; i < config_->vehicle.n_disc; i++) {
        const math::Vec2d center(pose.x + coeffs[i] * c, pose.y + coeffs[i] * s);
        if (CheckBoxCollision(time, math::AABox2d(center, wh, wh))) {
            return true;
        }
    }

    return false;
}

void Environment::UpdateCostmapObstacles() {

    //points_.clear();
    obstacle_tree_.clear();
    for (size_t i = 0; i < costmap_->getSizeInCellsX(); i++) {
        for (size_t j = 0; j < costmap_->getSizeInCellsY(); j++) {
            if (costmap_->getCost(i, j) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
                double obs_x, obs_y;
                costmap_->mapToWorld(i, j, obs_x, obs_y);
                //points_.emplace_back(obs_x, obs_y);
                //obstacle_tree_.insert(std::make_pair(Point(obs_x, obs_y), points_.size() - 1));
                obstacle_tree_.insert(Point(obs_x, obs_y));
            }
        }
    }
}

void Environment::setCostmap(const nav2_costmap_2d::Costmap2D *costmap) {
    costmap_ = costmap;
    UpdateCostmapObstacles();
    double min_x = costmap_->getOriginX();
    double max_x = min_x + (costmap_->getSizeInCellsX() * costmap_->getResolution());
    double min_y = costmap_->getOriginY();
    double max_y = min_y + (costmap_->getSizeInCellsY() * costmap_->getResolution());
    XYbounds_ = {min_x, max_x, min_y, max_y};
}

// 使用point_而没有用rtree也出现了障碍物进入廊道，应该是rviz自己的显示问题，实际上应该没有障碍物进入廊道
bool Environment::GenerateCorridorBox(double time, double x, double y, double theta, double radius, math::AABox2d &result) const {
    double ri = radius;
    double d_ri = 2 * ri;
    math::AABox2d bound({x, y}, d_ri, d_ri);

    if (CheckBoxCollision(time, bound)) {
        // initial condition not satisfied, involute to find feasible box
        int inc = 4;
        double real_x, real_y;

        do {
            int iter = inc / 4;
            uint8_t edge = inc % 4;

            real_x = x;
            real_y = y;
            double offset = iter * config_->corridor_search_resolution;
            if (edge == 0) {
                real_x = x - offset;
            } else if (edge == 1) {
                real_x = x + offset;
            } else if (edge == 2) {
                real_y = y - offset;
            } else {
                real_y = y + offset;
            }

            inc++;
            bound = math::AABox2d({real_x - ri, real_y - ri}, {real_x + ri, real_y + ri});
        } while (CheckBoxCollision(time, bound) && inc <= config_->corridor_max_iter);
        if (inc > config_->corridor_max_iter) {
            return false;
        }

        x = real_x;
        y = real_y;
    }

    int inc = 4;
    std::bitset<4> blocked;
    //double incremental[4] = {0.0, 0.0, 0.0, 0.0}; // left, down, right, up
    std::array<double, 4> points = {x - ri, y - ri, x + ri, y + ri};
    int direction_index = int((theta + M_PI) / (M_PI / 4)) % 8;
    const int* direction_order = direction_set[direction_index];
    
    double step = radius * 0.2;

    do {
        //int iter = inc / 4;
        uint8_t edge = inc % 4;
        inc++;
        //int dir = edge;
        int dir = direction_order[edge];
        if (std::abs(points[dir] - (dir & 1 ? y : x)) + step >= config_->corridor_incremental_limit) {
            blocked[dir] = true;
            continue;
        }
        if (blocked[dir]) continue;
        math::Vec2d mid;
        double len, wid;
        if (dir == 0) {
            mid.set_x(points[0] - step / 2.0);  
            mid.set_y((points[1] + points[3]) / 2.0);
            len = step;
            wid = points[3] - points[1];
        } else if (dir == 1) {
            mid.set_x((points[2] + points[0]) / 2.0);
            mid.set_y(points[1] - step / 2.0);
            len = points[2] - points[0];
            wid = step;
        } else if (dir == 2) {
            mid.set_x(points[2] + step / 2.0);
            mid.set_y((points[3] + points[1]) / 2.0);
            len = step;
            wid = points[3] - points[1];
        } else {
            mid.set_x((points[2] + points[0]) / 2.0);
            mid.set_y(points[3] + step / 2.0);
            len = points[2] - points[0];
            wid = step;
        }
        math::AABox2d test(mid, len, wid);

        if (CheckBoxCollision(time, test)) {
            blocked[dir] = true;
        } else {
            points[dir] += (dir / 2 == 0 ? -step : step);
        }
    } while (!blocked.all() && inc <= config_->corridor_max_iter);
    if (inc > config_->corridor_max_iter) {
        return false;
    }

    // Shrink by ri on each side so the box bounds the disc *center* safe region
    result = {{points[0] + ri, points[1] + ri},
              {points[2] - ri, points[3] - ri}};
    return true;
}

}