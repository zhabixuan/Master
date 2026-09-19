
#include "liom_local_planner/path_planner.h"
#include "common_math/math_utils.h"

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/ScopedState.h>

#include <algorithm>
#include <cstdlib>
#include <queue>

namespace liom_local_planner {

constexpr int min_oneshot_freq = 2, max_oneshot_freq = 100;

uint64_t PathPlanner::Node2d::GridIndex(int x_grid, int y_grid) {
    static constexpr int64_t kOffset = 1LL << 30;
    assert(std::abs(x_grid) < (1LL << 30) && std::abs(y_grid) < (1LL << 30));
    uint64_t ux = static_cast<uint64_t>(static_cast<int64_t>(x_grid) + kOffset);
    uint64_t uy = static_cast<uint64_t>(static_cast<int64_t>(y_grid) + kOffset);
    return (ux << 32) | uy;
}

uint64_t PathPlanner::Node2d::GridIndex(math::Pose ps, const std::vector<double>& XYbounds, const PlannerConfig& config) {
    return GridIndex(static_cast<int>((ps.x - XYbounds[0]) / config.grid_xy_resolution),
                     static_cast<int>((ps.y - XYbounds[2]) / config.grid_xy_resolution));
}

math::AABox2d PathPlanner::Node2d::GenerateBox(int x_grid, int y_grid, const std::vector<double>& XYbounds, const PlannerConfig& config) {
    math::Vec2d corner(XYbounds[0] + config.grid_xy_resolution * x_grid,
                       XYbounds[2] + config.grid_xy_resolution * y_grid);
    return { corner, config.vehicle.disc_radius * 2, config.vehicle.disc_radius * 2 };
}

PathPlanner::PathPlanner(
        std::shared_ptr<PlannerConfig> config,
        std::shared_ptr<Environment> env
    ) : config_(config), env_(env) {
      XYbounds_ = env_->XYbounds_;
    }

bool PathPlanner::Plan(math::Pose start, math::Pose goal, std::vector<math::Pose>& result) {
    // is_forward_only_ = config_->vehicle.min_velocity >= 0.0;
    // double arc_length = 
    //     config_->theta_resolution *
    //     config_->vehicle.wheel_base / 
    //     std::tan(config_->vehicle.phi_max * 2 / (config_->next_node_num / 2 - 1));
    double arc_length =
        config_->theta_resolution *
        config_->vehicle.wheel_base /
        std::tan(config_->vehicle.phi_max);
    if (arc_length < config_->xy_resolution * M_SQRT2) {
      arc_length = config_->xy_resolution * M_SQRT2;
    }
    forward_num_ = std::max(1, static_cast<int>(std::ceil(arc_length / config_->step_size)));

    // XYbounds_ = XYbounds;
    std::cout << "XYbounds: " << XYbounds_[0] << " " << XYbounds_[1] << " " << XYbounds_[2] << " " << XYbounds_[3] << std::endl;
    max_grid_y_ = std::round((XYbounds_[3] - XYbounds_[2]) / config_->grid_xy_resolution);
    max_grid_x_ = std::round((XYbounds_[1] - XYbounds_[0]) / config_->grid_xy_resolution);

    std::shared_ptr<Node3d> start_node = std::make_shared<Node3d>(start, XYbounds_, *config_);
    std::shared_ptr<Node3d> goal_node = std::make_shared<Node3d>(goal, XYbounds_, *config_);
    if (env_->CheckPoseCollision(0.0, start_node->pose)) {
        std::cout << "Start pose is in collision!" << std::endl;
        return false;
    }
    if (env_->CheckPoseCollision(0.0, goal_node->pose)) {
        std::cout << "Goal pose is in collision!" << std::endl;
        return false;
    }

    open_pq_ = decltype(open_pq_)();
    open_set_.clear();
    closed_set_.clear();

    grid_open_pq_ = decltype(grid_open_pq_)();
    grid_open_set_.clear();
    grid_closed_set_.clear();

    std::shared_ptr<Node2d> grid_goal_node = std::make_shared<Node2d>(goal, XYbounds_, *config_);
    grid_goal_node->f_cost = 0.0;
    grid_open_pq_.emplace(grid_goal_node->index, grid_goal_node->f_cost);
    grid_open_set_.emplace(grid_goal_node->index, grid_goal_node);
    
    start_node->set_cost(0, EstimateHeuristicCost(start_node));
    open_set_.emplace(start_node->index, start_node);
    open_pq_.emplace(start_node->index, start_node->f_cost);

    static constexpr int kMaxNodeNum = 200000;
    size_t explored_node_num = 0;
    size_t max_explored_num = 1000000;

    double dist_start_to_goal = start.DistanceTo(goal);
    std::shared_ptr<Node3d> oneshot_node = nullptr;
    std::vector<math::Pose> oneshot_path;
    std::shared_ptr<Node3d> final_node = nullptr;
    
    while (!open_pq_.empty() &&
           open_pq_.size() < kMaxNodeNum &&
           explored_node_num < max_explored_num) {
      uint64_t current_node_index = open_pq_.top().first;
      open_pq_.pop();
      std::shared_ptr<Node3d> current_node = open_set_[current_node_index];
      if (closed_set_.count(current_node->index) > 0) {
        continue;
      }
      closed_set_.emplace(current_node->index, current_node);
      explored_node_num++;
      double scaled_heu_cost = dist_start_to_goal > 1e-6
          ? (current_node->f_cost - current_node->g_cost) / dist_start_to_goal
          : 1.0;
      int oneshot_freq = static_cast<int>(std::clamp(
          min_oneshot_freq + scaled_heu_cost * (max_oneshot_freq - min_oneshot_freq),
          static_cast<double>(min_oneshot_freq), static_cast<double>(max_oneshot_freq)));
      if (explored_node_num % oneshot_freq == 0) {
        if (CheckOneshotPath(current_node, goal_node, oneshot_path)) {
          oneshot_node = current_node;
          break;
        }
      }
      if (current_node->index == goal_node->index) {
        final_node = current_node;
        break;
      }
      int next_node_num = config_->next_node_num;

      for (int i = 0; i < next_node_num; ++i) {
        std::shared_ptr<Node3d> next_node;
        if (!ExpandNextNode(current_node, i, next_node)) {
          continue;
        }
        if (closed_set_.count(next_node->index) > 0) {
          continue;
        }
        next_node->set_cost(
            current_node->g_cost + EvaluateExpandCost(current_node, next_node), 
            EstimateHeuristicCost(next_node));
        auto node_opened = open_set_.find(next_node->index);
        if (node_opened == open_set_.end()) {
          open_set_.emplace(next_node->index, next_node);
          open_pq_.emplace(next_node->index, next_node->f_cost);
        } else {
          if (next_node->g_cost < node_opened->second->g_cost) {
            node_opened->second->g_cost = next_node->g_cost;
            node_opened->second->f_cost = next_node->f_cost;
            node_opened->second->pre_node = current_node;
            node_opened->second->steering = next_node->steering;
            node_opened->second->is_forward = next_node->is_forward;
            open_pq_.emplace(next_node->index, next_node->f_cost);
          }
        }
      }
    }


  if (oneshot_node != nullptr) {
    result = TraversePath(oneshot_node);
    result.insert(result.end(), oneshot_path.begin(), oneshot_path.end());
  } else if (final_node != nullptr) {
    result = TraversePath(final_node);
  } else {
    return false;
  }

  std::cout << "walked node: " << explored_node_num << std::endl;

  return true;
}

std::vector<math::Pose> PathPlanner::TraversePath(std::shared_ptr<Node3d> current_node) {
  std::vector<math::Pose> result;
  if (!current_node) {
    return result;
  }
  while (current_node->pre_node)
  {
    std::shared_ptr<Node3d> pre_node = current_node->pre_node;
    auto path = GenerateKinematicPath(pre_node->pose, current_node->is_forward, current_node->steering);
    for (int i = path.size() - 1; i > 0; --i) {
      result.push_back(path[i]);
    }
    current_node = pre_node;
  }

  result.push_back(current_node->pose);
  std::reverse(result.begin(), result.end());
  return result;
}

double PathPlanner::EvaluateExpandCost(std::shared_ptr<Node3d> current_node, std::shared_ptr<Node3d> next_node) {
  double piecewise_cost = 0.0;
  if (next_node->is_forward) {
    piecewise_cost += static_cast<double>(forward_num_) * config_->step_size * config_->forward_penalty;
  } else {
    piecewise_cost += static_cast<double>(forward_num_) * config_->step_size * config_->backward_penalty;
  }
  if (current_node->is_forward != next_node->is_forward) {
    piecewise_cost += config_->gear_change_penalty;
  }
  piecewise_cost += config_->steering_penalty * std::abs(next_node->steering);
  piecewise_cost += config_->steering_change_penalty * std::abs(next_node->steering - current_node->steering);
  return piecewise_cost;
}

double PathPlanner::EstimateHeuristicCost(std::shared_ptr<Node3d> node) {
  return Calculate2DCost(node);
}

constexpr int grid_directions[8][2] = {
    {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}
};

constexpr double grid_direction_costs[8] = {
    1, M_SQRT2, 1, M_SQRT2, 1, M_SQRT2, 1, M_SQRT2
};

bool PathPlanner::GridCellCollides(int x_grid, int y_grid) const {
  if (x_grid < 0 || x_grid > max_grid_x_ || y_grid < 0 || y_grid > max_grid_y_) {
    return true;
  }
  return env_->CheckBoxCollision(0.0, Node2d::GenerateBox(x_grid, y_grid, XYbounds_, *config_));
}

double PathPlanner::Calculate2DCost(std::shared_ptr<Node3d> node_3d) {
  const uint64_t target_index = Node2d::GridIndex(node_3d->pose, XYbounds_, *config_);

  auto closed_node = grid_closed_set_.find(target_index);
  if (closed_node != grid_closed_set_.end()) {
    return closed_node->second->f_cost * config_->grid_xy_resolution;
  }

  // Incremental 2D Dijkstra propagating from the goal; continue where the
  // previous call left off until the target grid cell is settled.
  while (!grid_open_pq_.empty()) {
    const uint64_t current_index = grid_open_pq_.top().first;
    grid_open_pq_.pop();
    auto current_it = grid_open_set_.find(current_index);
    if (current_it == grid_open_set_.end()) {
      continue;  // stale entry already settled
    }
    std::shared_ptr<Node2d> current_node = current_it->second;
    grid_open_set_.erase(current_it);
    grid_closed_set_.emplace(current_index, current_node);

    const int current_node_x = current_node->x_grid;
    const int current_node_y = current_node->y_grid;
    const double current_node_f_cost = current_node->f_cost;

    for (int i = 0; i < 8; ++i) {
      const int next_x = current_node_x + grid_directions[i][0];
      const int next_y = current_node_y + grid_directions[i][1];
      if (next_x < 0 || next_x > max_grid_x_ || next_y < 0 || next_y > max_grid_y_) {
        continue;
      }
      const uint64_t next_index = Node2d::GridIndex(next_x, next_y);
      if (grid_closed_set_.count(next_index) > 0) {
        continue;
      }
      const double next_f_cost = current_node_f_cost + grid_direction_costs[i];
      auto opened_it = grid_open_set_.find(next_index);
      if (opened_it != grid_open_set_.end()) {
        if (opened_it->second->f_cost > next_f_cost) {
          opened_it->second->f_cost = next_f_cost;
          grid_open_pq_.emplace(next_index, next_f_cost);
        }
        continue;
      }
      if (GridCellCollides(next_x, next_y)) {
        continue;
      }
      auto next_node = std::make_shared<Node2d>(next_x, next_y);
      next_node->f_cost = next_f_cost;
      grid_open_set_.emplace(next_index, next_node);
      grid_open_pq_.emplace(next_index, next_f_cost);
    }

    if (current_index == target_index) {
      return current_node_f_cost * config_->grid_xy_resolution;
    }
  }

  return inf;
}

std::vector<math::Pose> PathPlanner::GenerateKinematicPath(math::Pose last_pose, bool is_forward, double steering) const {
  std::vector<math::Pose> path(forward_num_ + 1);
  const double step_size = is_forward ? config_->step_size : -config_->step_size;
  const double dtheta = step_size / config_->vehicle.wheel_base * std::tan(steering);

  path[0] = last_pose;
  for (int i = 0; i < forward_num_; i++) {
    const double last_theta = last_pose.theta;
    const double next_theta = last_theta + dtheta;
    const double mid_theta = (last_theta + next_theta) / 2.0;
    last_pose.x += step_size * std::cos(mid_theta);
    last_pose.y += step_size * std::sin(mid_theta);
    last_pose.theta = math::NormalizeAngle(next_theta);
    path[i + 1] = last_pose;
  }

  return path;
}

bool PathPlanner::ExpandNextNode(std::shared_ptr<Node3d> node, int next_index, std::shared_ptr<Node3d>& next_node) {
  double steering;
  bool is_forward = (next_index < static_cast<double>(config_->next_node_num) / 2);

  double node_res = (static_cast<double>(config_->next_node_num) / 2 - 1);
  if (is_forward) {
    steering = -config_->vehicle.phi_max + (2 * config_->vehicle.phi_max / node_res) * static_cast<double>(next_index);
  } else {
    int index = next_index - config_->next_node_num / 2;
    steering = -config_->vehicle.phi_max + (2 * config_->vehicle.phi_max / node_res) * static_cast<double>(index);
  }

  auto path = GenerateKinematicPath(node->pose, is_forward, steering);
  // check if the vehicle runs outside of XY boundary
  if (path.back().x > XYbounds_[1] ||
      path.back().x < XYbounds_[0] ||
      path.back().y > XYbounds_[3] ||
      path.back().y < XYbounds_[2]) {
    next_node = nullptr;
    return false;
  }

  for (auto &pose : path) {
    if (env_->CheckPoseCollision(0.0, pose)) {
      return false;
    }
  }

  next_node = std::make_shared<Node3d>(path.back(), XYbounds_, *config_);
  next_node->is_forward = is_forward;
  next_node->steering = steering;
  next_node->pre_node = node;
  return true;
}

bool PathPlanner::CheckOneshotPath(std::shared_ptr<Node3d> node, std::shared_ptr<Node3d> goal, std::vector<math::Pose> &result) {
  if(!GenerateShortestPath(node->pose, goal->pose, result)) {
    return false;
  }

  for(auto &pose: result) {
    if(env_->CheckPoseCollision(0.0, pose)) {
      return false;
    }
  }

  return true;
}

namespace ob = ompl::base;

bool PathPlanner::GenerateShortestPath(math::Pose start, math::Pose goal, std::vector<math::Pose> &result) {
  double min_turning_radius = config_->vehicle.wheel_base / std::tan(config_->vehicle.phi_max);
  std::shared_ptr<ob::SE2StateSpace> state_space;
  state_space = std::make_shared<ob::ReedsSheppStateSpace>(min_turning_radius);

  ob::ScopedState<ob::SE2StateSpace> rs_start(state_space), rs_goal(state_space);
  rs_start[0] = start.x;
  rs_start[1] = start.y;
  rs_start[2] = start.theta;
  rs_goal[0] = goal.x;
  rs_goal[1] = goal.y;
  rs_goal[2] = goal.theta;

  auto ss = std::static_pointer_cast<ob::ReedsSheppStateSpace>(state_space);
  auto path = ss->reedsShepp(rs_start.get(), rs_goal.get());
  ob::ScopedState<> state(state_space);
  bool firstTime = false;
  int sample_count = std::max(1, static_cast<int>(std::ceil(path.length() / config_->step_size)));
  result.resize(sample_count);

  for (int i = 1; i <= sample_count; ++i) {
    ss->interpolate(rs_start.get(), rs_goal.get(), static_cast<double>(i) / sample_count, firstTime, path, state.get());
    result[i-1].x = state[0];
    result[i-1].y = state[1];
    result[i-1].theta = state[2];
  }
  
  return true;
}


}// namespace liom_local_planner