#pragma once

#include "liom_local_planner/environment.h"
#include "liom_local_planner/planner_config.h"
#include "common_math/pose.h"

#include <array>
#include <vector>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include <assert.h>

namespace liom_local_planner {

namespace math = common::math;

class PathPlanner {
public:
    PathPlanner(
        std::shared_ptr<PlannerConfig> config,
        std::shared_ptr<Environment> env);

    bool Plan(math::Pose start, math::Pose goal, std::vector<math::Pose>& result);

private:
    static constexpr double inf = std::numeric_limits<double>::max();
    
    struct Node3d {
        bool is_forward = true;
        // bool is_closed = false;
        math::Pose pose;
        double steering = 0.0;

        Node3d() = default;

        Node3d(math::Pose ps, const std::vector<double>& XYbounds, const PlannerConfig& config) : pose(ps) {
            x_grid = static_cast<int>((ps.x - XYbounds[0]) / config.xy_resolution);
            y_grid = static_cast<int>((ps.y - XYbounds[2]) / config.xy_resolution);
            theta_grid = static_cast<int>((ps.theta - (-M_PI)) / config.theta_resolution);

            // offset negative values into positive range for correct bit packing
            static constexpr int64_t kOffset = 1 << 23;
            assert(abs(x_grid) < kOffset && abs(y_grid) < kOffset && abs(theta_grid) < 0x0000FFFF);

            uint64_t ux = static_cast<uint64_t>(static_cast<int64_t>(x_grid) + kOffset);
            uint64_t uy = static_cast<uint64_t>(static_cast<int64_t>(y_grid) + kOffset);
            uint64_t ut = static_cast<uint64_t>(theta_grid);
            index = (ux & 0x00FFFFFF) << 40 | (uy & 0x00FFFFFF) << 16 | (ut & 0x0000FFFF);
        }

        inline void set_cost(double g, double h) {
            g_cost = g;
            f_cost = g + h;
        }

        int x_grid, y_grid, theta_grid;
        uint64_t index = 0;
        std::shared_ptr<Node3d> pre_node = nullptr;
        double g_cost = inf, f_cost = inf;
    };

    struct Node2d {
        int x_grid, y_grid;
        uint64_t index = 0;
        double f_cost = inf;

        static uint64_t GridIndex(int x_grid, int y_grid);
        static uint64_t GridIndex(math::Pose ps, const std::vector<double>& XYbounds, const PlannerConfig& config);
        static math::AABox2d GenerateBox(int x_grid, int y_grid, const std::vector<double>& XYbounds, const PlannerConfig& config);

        Node2d(math::Pose ps, const std::vector<double>& XYbounds, const PlannerConfig& config)
            : Node2d(static_cast<int>((ps.x - XYbounds[0]) / config.grid_xy_resolution),
                     static_cast<int>((ps.y - XYbounds[2]) / config.grid_xy_resolution)) {}

        Node2d(int x_grd, int y_grd) : x_grid(x_grd), y_grid(y_grd) {
            index = GridIndex(x_grd, y_grd);
        }
    };

    std::shared_ptr<PlannerConfig> config_;
    std::shared_ptr<Environment> env_;
    // double arc_length_ = 0.0;
    // bool is_forward_only_ = false;
    int forward_num_;
    // XYbounds with xmin, xmax, ymin, ymax
    std::vector<double> XYbounds_;
    double max_grid_x_ = 0.0;
    double max_grid_y_ = 0.0;

    struct cost_cmp {
        bool operator()(const std::pair<uint64_t, double>& left, const std::pair<uint64_t, double>& right) const {
            return left.second >= right.second;
        }
    };

    // std::priority_queue<std::pair<uint64_t, double>, std::vector<std::pair<uint64_t, double>>, cost_cmp> open_pq_;
    // std::unordered_map<uint64_t, std::shared_ptr<Node3d>> open_set_;
    std::priority_queue<std::pair<uint64_t, double>, std::vector<std::pair<uint64_t, double>>, cost_cmp> open_pq_;
    std::unordered_map<uint64_t, std::shared_ptr<Node3d>> open_set_;
    std::unordered_map<uint64_t, std::shared_ptr<Node3d>> closed_set_;

    std::priority_queue<std::pair<uint64_t, double>, std::vector<std::pair<uint64_t, double>>, cost_cmp> grid_open_pq_;
    std::unordered_map<uint64_t, std::shared_ptr<Node2d>> grid_open_set_;
    std::unordered_map<uint64_t, std::shared_ptr<Node2d>> grid_closed_set_;

    std::vector<math::Pose> TraversePath(std::shared_ptr<Node3d> node);

    std::vector<math::Pose> GenerateKinematicPath(math::Pose pose, bool is_forward, double steering) const;

    bool ExpandNextNode(std::shared_ptr<Node3d> node, int next_index, std::shared_ptr<Node3d>& next_node);

    double EvaluateExpandCost(std::shared_ptr<Node3d> current_node, std::shared_ptr<Node3d> next_node);

    double EstimateHeuristicCost(std::shared_ptr<Node3d> node);

    bool GridCellCollides(int x_grid, int y_grid) const;

    double Calculate2DCost(std::shared_ptr<Node3d> node);

    bool CheckOneshotPath(std::shared_ptr<Node3d> node, std::shared_ptr<Node3d> goal, std::vector<math::Pose> &result);

    bool GenerateShortestPath(math::Pose start, math::Pose goal, std::vector<math::Pose> &result);

};
}
