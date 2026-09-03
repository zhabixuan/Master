// Copyright (c) 2021, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include "hybrid_Astar/collision_checker.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace hybrid_Astar
{

GridCollisionChecker::GridCollisionChecker(
  nav2_costmap_2d::Costmap2D * costmap,
  unsigned int num_quantizations,
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
: FootprintCollisionChecker(costmap)
{
  if (node) {
    clock_ = node->get_clock();
    logger_ = node->get_logger();
  }

  // Convert number of regular bins into angles
  float bin_size = 2 * M_PI / static_cast<float>(num_quantizations);
  angles_.reserve(num_quantizations);
  for (unsigned int i = 0; i != num_quantizations; i++) {
    angles_.push_back(bin_size * i);
  }
}

bool GridCollisionChecker::inCollision(
  const float & x,
  const float & y,
  const float & angle_bin,
  const bool & traverse_unknown)
{
  // Check to make sure cell is inside the map
  if (outsideRange(costmap_->getSizeInCellsX(), x) ||
    outsideRange(costmap_->getSizeInCellsY(), y))
  {
    return true;
  }

  // Assumes setFootprint already set
  double wx, wy;
  costmap_->mapToWorld(static_cast<double>(x), static_cast<double>(y), wx, wy);


  // if radius, then we can check the center of the cost assuming inflation is used
  float front_wx, front_wy, w_b = 2.80f;
  front_wx = wx + w_b * cos(angles_[angle_bin]);
  front_wy = wy + w_b * sin(angles_[angle_bin]);

  double cost1 = costmap_->getCost(
    static_cast<unsigned int>(x), static_cast<unsigned int>(y));

  unsigned int front_x, front_y;
  costmap_->worldToMap(front_wx, front_wy, front_x, front_y);
  double cost2 = costmap_->getCost(front_x, front_y);
  
  // footprint_cost_ = costmap_->getCost(
  //   static_cast<unsigned int>(x), static_cast<unsigned int>(y));
  footprint_cost_ = std::max(cost1, cost2);

  if (footprint_cost_ == UNKNOWN && traverse_unknown) {
    return false;
  }

  // if occupied or unknown and not to traverse unknown space
  return static_cast<double>(footprint_cost_) >= INSCRIBED;
}

float GridCollisionChecker::getCost()
{
  // Assumes inCollision called prior
  return static_cast<float>(footprint_cost_);
}

bool GridCollisionChecker::outsideRange(const unsigned int & max, const float & value)
{
  return value < 0.0f || value > max;
}

}  // namespace nav2_smac_planner
