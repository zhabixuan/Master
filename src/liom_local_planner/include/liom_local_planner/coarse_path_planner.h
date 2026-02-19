
#include "liom_local_planner/environment.h"
#include "liom_local_planner/planner_config.h"
#include "liom_local_planner/math/pose.h"

#include <array>
#include <vector>
#include <limits>
#include <unordered_map>

#include <assert.h>

#ifndef LIOM_LOCAL_PLANNER_COARSE_PATH_PLANNER_H
#define LIOM_LOCAL_PLANNER_COARSE_PATH_PLANNER_H

namespace liom_local_planner {

class CoarsePathPlanner {
public:
    CoarsePathPlanner(
        std::shared_ptr<PlannerConfig> config,
        std::shared_ptr<Environment> env) : config_(config), env_(env){
        }
    
    bool Plan(math::Pose start, math::Pose goal, std::vector<math::Pose>& result);

private:
    static constexpr double inf = std::numeric_limits<double>::max();
    static constexpr uint64_t undefined_index = std::numeric_limits<uint64_t>::max();
    
    struct Node3d {
        bool is_forward, is_closed = false;
        math::Pose pose;
        double steering;

        Node3d() = default;

        Node3d(math::Pose ps, math::Vec2d origin, const PlannerConfig& config) : pose(ps) {
            x_grid = static_cast<int32_t>(floor((ps.x() - origin.x()) / config.xy_resolution));
            y_grid = static_cast<int32_t>(floor((ps.y() - origin.y()) / config.xy_resolution));
            theta_grid = static_cast<int32_t>(floor((ps.theta() - (-M_PI)) / config.theta_resolution));

            assert(abs(x_grid) < 0x00FFFFFF && abs(y_grid) < 0x00FFFFFF && abs(theta_grid) < 0x00003FFF);

            // 1 (x_sign_bit) + 24 (x_grid_bit) + 1 (y_sign_bit) + 24 (y_grid_bit) + 14 (theta_grid_bit)
            index |= (static_cast<uint64_t>(x_grid) & 0x80000000) << 63;
            index |= (static_cast<uint64_t>(x_grid) & 0x00FFFFFF) << 39;
            index |= (static_cast<uint64_t>(y_grid) & 0x80000000) << 38;
            index |= (static_cast<uint64_t>(y_grid) & 0x00FFFFFF) << 14;
            index |= static_cast<uint64_t>(theta_grid) & 0x00003FFF;
        }

        inline void set_cost(double g, double h) {
            g_cost = g;
            f_cost = g + h;
        }

        int32_t x_grid, y_grid, theta_grid;
        uint64_t index = 0, pre_index = undefined_index;
        double g_cost = inf, f_cost = inf;
    };

    struct Node2d {
        int32_t x_grid, y_grid;
        uint64_t index = 0, pre_index = 0;
        double f_cost = inf;
        bool is_closed = false;

        Node2d(math::Pose ps, math::Vec2d origin, const PlannerConfig& config) {
            x_grid = static_cast<int32_t>(floor((ps.x() - origin.x()) / config.grid_xy_resolution));
            y_grid = static_cast<int32_t>(floor((ps.y() - origin.y()) / config.grid_xy_resolution));

            assert(abs(x_grid) < 0xFFFFFFFF && abs(y_grid) < 0xFFFFFFFF);
            index = (static_cast<uint64_t>(x_grid) << 32) | static_cast<uint32_t>(y_grid);
        }

        Node2d(int x_grd, int y_grd) : x_grid(x_grd), y_grid(y_grd) {
            index = (static_cast<uint64_t>(x_grd) << 32) | static_cast<uint32_t>(y_grd);
        }

        inline math::AABox2d GenerateBox(math:: Vec2d origin, const PlannerConfig& config) const {
            math::Vec2d corner(origin.x() + config.grid_xy_resolution * x_grid, origin.y() + config.grid_xy_resolution * y_grid);
            return { corner, config.vehicle.disc_radius * 2, config.vehicle.disc_radius * 2 };
        }
    };

    std::shared_ptr<PlannerConfig> config_;
    std::shared_ptr<Environment> env_;
    math::Vec2d origin_;
    bool is_forward_only_ = false;
    int forward_num_;

    struct cost_cmp {
        bool operator()(const std::pair<uint64_t, double>& left, const std::pair<uint64_t, double>& right) const {
            return left.second >= right.second;
        }
    };

    std::priority_queue<std::pair<uint64_t, double>, std::vector<std::pair<uint64_t, double>>, cost_cmp> open_pq_;
    std::unordered_map<uint64_t, Node3d> open_set_;

    std::priority_queue<std::pair<uint64_t, double>, std::vector<std::pair<uint64_t, double>>, cost_cmp> grid_open_pq_;
    std::unordered_map<uint64_t, Node2d> grid_open_set_;

    std::vector<math::Pose> TraversePath(uint64_t node_index);

    std::vector<math::Pose> GenerateKinematicPath(math::Pose pose, int is_forward, double steering) const;

    bool ExpandNextNode(const Node3d &node, int next_index, Node3d &next_node);

    double EvaluateExpandCost(const Node3d &parent, const Node3d &node);

    double EstimateHeuristicCost(const Node3d &node);

    double Calculate2DCost(const Node3d &node);

    bool CheckOneshotPath(const Node3d &node, const Node3d &goal, std::vector<math::Pose> &result);

    bool GenerateShortestPath(math::Pose start, math::Pose goal, std::vector<math::Pose> &result);

};
}

#endif //LIOM_LOCAL_PLANNER_COARSE_PATH_PLANNER_H