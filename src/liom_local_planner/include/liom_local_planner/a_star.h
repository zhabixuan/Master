#pragma once

#include <array>
#include <vector>
#include <limits>
#include <unordered_map>

#include <assert.h>
#include <iostream>

#include "liom_local_planner/planner_config.h"
#include "common_math/pose.h"
#include "liom_local_planner/environment.h"

namespace liom_local_planner {

namespace math = common::math;
static constexpr double inf = std::numeric_limits<double>::max();

class Node2d {
 public:
  Node2d(const double x, const double y, const double xy_resolution,
         const std::vector<double>& XYbounds) {
    // XYbounds with xmin, xmax, ymin, ymax
    grid_x_ = static_cast<int>((x - XYbounds[0]) / xy_resolution);
    grid_y_ = static_cast<int>((y - XYbounds[2]) / xy_resolution);
    index_ = ComputeIndex(grid_x_, grid_y_);
  }
  Node2d(const int grid_x, const int grid_y,
         const std::vector<double>& XYbounds) {
    grid_x_ = grid_x;
    grid_y_ = grid_y;
    index_ = ComputeIndex(grid_x_, grid_y_);
  }
  void SetPathCost(const double path_cost) {
    path_cost_ = path_cost;
    cost_ = path_cost_ + heuristic_;
  }
  void SetHeuristic(const double heuristic) {
    heuristic_ = heuristic;
    cost_ = path_cost_ + heuristic_;
  }
  void SetCost(const double cost) { cost_ = cost; }
  void SetDistanceToObstacle(const double dist) {
      distance_to_obstacle_ = dist;
  }
  void SetPreNode(std::shared_ptr<Node2d> pre_node) { pre_node_ = pre_node; }
  double GetGridX() const { return grid_x_; }
  double GetGridY() const { return grid_y_; }
  double GetPathCost() const { return path_cost_; }
  double GetHeuCost() const { return heuristic_; }
  double GetCost() const { return cost_; }
  double GetDistanceToObstacle() const {
      return distance_to_obstacle_;
  }
  const int64_t& GetIndex() const { return index_; }
  std::shared_ptr<Node2d> GetPreNode() const { return pre_node_; }
  static int64_t CalcIndex(const double x, const double y,
                           const double xy_resolution,
                           const std::vector<double>& XYbounds) {
    // XYbounds with xmin, xmax, ymin, ymax
    int grid_x = static_cast<int>((x - XYbounds[0]) / xy_resolution);
    int grid_y = static_cast<int>((y - XYbounds[2]) / xy_resolution);
    return ComputeIndex(grid_x, grid_y);
  }
  bool operator==(const Node2d& right) const {
    return right.GetIndex() == index_;
  }

  inline math::AABox2d GenerateBox(const std::vector<double>& XYbounds, const PlannerConfig& config) const {
        math::Vec2d corner(XYbounds[0] + config.grid_xy_resolution * grid_x_, XYbounds[2] + config.grid_xy_resolution * grid_y_);
        return { corner, config.vehicle.disc_radius * 2, config.vehicle.disc_radius * 2 };
    }

 private:
  static int64_t ComputeIndex(int x_grid, int y_grid) {
    assert(abs(x_grid) < 0xFFFFFFFF && abs(y_grid) < 0xFFFFFFFF);
    return (static_cast<int64_t>(x_grid) << 32) | static_cast<int64_t>(y_grid);
  }

 private:
  int grid_x_ = 0;
  int grid_y_ = 0;
  double path_cost_ = 0.0;
  double heuristic_ = 0.0;
  double cost_ = 0.0;
  double distance_to_obstacle_ = std::numeric_limits<double>::max();
  int64_t index_;
  std::shared_ptr<Node2d> pre_node_ = nullptr;
};

class AStar {
public:
  explicit AStar(std::shared_ptr<PlannerConfig> config,
        std::shared_ptr<Environment> env);
  virtual ~AStar() = default;
  bool GenerateDpMap(
          const double ex,
          const double ey,
          const std::vector<double>& XYbounds);
  double CheckDpMap(const double sx, const double sy);

private:
  std::vector<std::shared_ptr<Node2d>> GenerateNextNodes(
      std::shared_ptr<Node2d> node);
  bool CheckConstraints(std::shared_ptr<Node2d> node);
private:
  std::shared_ptr<PlannerConfig> planner_config_;
  std::shared_ptr<Environment> env_;
  double xy_grid_resolution_ = 0.0;
  double node_radius_ = 0.0;
  std::vector<double> XYbounds_;
  double max_grid_x_ = 0.0;
  double max_grid_y_ = 0.0;
  struct cmp {
      bool operator()(const std::pair<int64_t, double>& left,
                      const std::pair<int64_t, double>& right) const {
          return left.second >= right.second;
      }
  };
  std::unordered_map<int64_t, std::shared_ptr<Node2d>> dp_map_;
};
}