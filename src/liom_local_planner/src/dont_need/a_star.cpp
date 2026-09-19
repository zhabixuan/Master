#include "liom_local_planner/a_star.h"

namespace liom_local_planner {
AStar::AStar(std::shared_ptr<PlannerConfig> config,
        std::shared_ptr<Environment> env) : planner_config_(config), env_(env) {
    xy_grid_resolution_ = planner_config_->grid_xy_resolution;
    node_radius_ = planner_config_->vehicle.disc_radius;
}

bool AStar::CheckConstraints(std::shared_ptr<Node2d> node) {
  const double node_grid_x = node->GetGridX();
  const double node_grid_y = node->GetGridY();
  if (node_grid_x > max_grid_x_ ||
      node_grid_x < 0  ||
      node_grid_y > max_grid_y_ ||
      node_grid_y < 0) {
    return false;
  }
  if (env_->CheckBoxCollision(0.0, node->GenerateBox(XYbounds_, *planner_config_))) {
    return false;
  }
  // for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
  //   for (const common::math::LineSegment2d& linesegment :
  //        obstacle_linesegments) {
  //     if (linesegment.DistanceTo({node->GetGridX(), node->GetGridY()})
  //         < node_radius_) {
  //       return false;
  //     }
  //   }
  // }
  return true;
}

std::vector<std::shared_ptr<Node2d>> AStar::GenerateNextNodes(
    std::shared_ptr<Node2d> current_node) {
  double current_node_x = current_node->GetGridX();
  double current_node_y = current_node->GetGridY();
  double current_node_path_cost = current_node->GetPathCost();
  double diagonal_distance = std::sqrt(2.0);
  std::vector<std::shared_ptr<Node2d>> next_nodes;
  std::shared_ptr<Node2d> up =
      std::make_shared<Node2d>(current_node_x, current_node_y + 1.0, XYbounds_);
  up->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> up_right = std::make_shared<Node2d>(
      current_node_x + 1.0, current_node_y + 1.0, XYbounds_);
  up_right->SetPathCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> right =
      std::make_shared<Node2d>(current_node_x + 1.0, current_node_y, XYbounds_);
  right->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> down_right = std::make_shared<Node2d>(
      current_node_x + 1.0, current_node_y - 1.0, XYbounds_);
  down_right->SetPathCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> down =
      std::make_shared<Node2d>(current_node_x, current_node_y - 1.0, XYbounds_);
  down->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> down_left = std::make_shared<Node2d>(
      current_node_x - 1.0, current_node_y - 1.0, XYbounds_);
  down_left->SetPathCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> left =
      std::make_shared<Node2d>(current_node_x - 1.0, current_node_y, XYbounds_);
  left->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> up_left = std::make_shared<Node2d>(
      current_node_x - 1.0, current_node_y + 1.0, XYbounds_);
  up_left->SetPathCost(current_node_path_cost + diagonal_distance);

  next_nodes.emplace_back(up);
  next_nodes.emplace_back(up_right);
  next_nodes.emplace_back(right);
  next_nodes.emplace_back(down_right);
  next_nodes.emplace_back(down);
  next_nodes.emplace_back(down_left);
  next_nodes.emplace_back(left);
  next_nodes.emplace_back(up_left);
  return next_nodes;
}
bool AStar::GenerateDpMap(
        const double ex,
        const double ey,
        const std::vector<double>& XYbounds) {
  std::priority_queue<std::pair<int64_t, double>,
                      std::vector<std::pair<int64_t, double>>, cmp>
      open_pq;
  std::unordered_map<int64_t, std::shared_ptr<Node2d>> open_set;
  dp_map_ = decltype(dp_map_)();
  XYbounds_ = XYbounds;
  // XYbounds with xmin, xmax, ymin, ymax
  max_grid_y_ = std::round((XYbounds_[3] - XYbounds_[2]) / xy_grid_resolution_);
  max_grid_x_ = std::round((XYbounds_[1] - XYbounds_[0]) / xy_grid_resolution_);
  std::shared_ptr<Node2d> end_node =
      std::make_shared<Node2d>(ex, ey, xy_grid_resolution_, XYbounds_);
  open_set.emplace(end_node->GetIndex(), end_node);
  open_pq.emplace(end_node->GetIndex(), end_node->GetCost());

  // Grid a star begins
  size_t explored_node_num = 0;
  while (!open_pq.empty()) {
    const int64_t current_id = open_pq.top().first;
    open_pq.pop();
    std::shared_ptr<Node2d> current_node = open_set[current_id];
    dp_map_.emplace(current_node->GetIndex(), current_node);
    std::vector<std::shared_ptr<Node2d>> next_nodes =
        std::move(GenerateNextNodes(current_node));
    for (auto& next_node : next_nodes) {
      if (!CheckConstraints(next_node)) {
        continue;
      }
      if (dp_map_.find(next_node->GetIndex()) != dp_map_.end()) {
        continue;
      }
      if (open_set.find(next_node->GetIndex()) == open_set.end()) {
        ++explored_node_num;
        next_node->SetPreNode(current_node);
        open_set.emplace(next_node->GetIndex(), next_node);
        open_pq.emplace(next_node->GetIndex(), next_node->GetCost());
      } else {
        if (open_set[next_node->GetIndex()]->GetCost() > next_node->GetCost()) {
          open_set[next_node->GetIndex()]->SetCost(next_node->GetCost());
          open_set[next_node->GetIndex()]->SetPreNode(current_node);
        }
      }
    }
  }
  std::cout << "explored node num is " << explored_node_num;
  return true;
}
}