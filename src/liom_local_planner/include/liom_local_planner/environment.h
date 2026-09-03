//
// Created by yenkn on 1/11/23.
//

#ifndef LIOM_LOCAL_PLANNER_ENVIRONMENT_H
#define LIOM_LOCAL_PLANNER_ENVIRONMENT_H

#include <vector>
#include <iostream>
//#include <costmap_2d/costmap_2d.h>
#include "nav2_costmap_2d/costmap_2d.hpp"
//#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "common_math/polygon2d.h"
#include "planner_config.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace liom_local_planner {

namespace math = common::math;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using Point = bg::model::point<double, 2, bg::cs::cartesian>; // 2D point

class Environment {
public:
  explicit Environment(std::shared_ptr<PlannerConfig> config, 
              const nav2_costmap_2d::Costmap2D* costmap);

  std::vector<common::math::Polygon2d> &polygons() { return polygons_; }

  bool CheckPoseCollision(double time, common::math::Pose pose) const;

  //bool GenerateCorridorBox(double time, double x, double y, double radius, math::AABox2d &result) const;
  bool GenerateCorridorBox(double time, double x, double y, double theta, double radius, common::math::AABox2d &result) const;

  bool CheckBoxCollision(double time, const common::math::AABox2d &box) const;

  void UpdateCostmapObstacles();
  
  void setCostmap(const nav2_costmap_2d::Costmap2D *costmap);
public:
  std::vector<double> XYbounds_;
private:
  //bgi::rtree<std::pair<Point, size_t>, bgi::quadratic<16>> obstacle_tree_;
  bgi::rtree<Point, bgi::quadratic<16>> obstacle_tree_;
  std::shared_ptr<PlannerConfig> config_;
  std::vector<common::math::Polygon2d> polygons_;
  const nav2_costmap_2d::Costmap2D* costmap_;
  //std::vector<math::Vec2d> points_;
  static constexpr int direction_set[8][4] = {
      {0, 1, 2, 3},   // -pi - 0.75pi
      {1, 0, 3, 2},   // -0.75pi - -0.5pi
      {1, 2, 3, 0},   // -0.5pi - -0.25pi
      {2, 1, 0, 3},   // -0.25pi - 0
      {2, 3, 0, 1},   // 0 - 0.25pi
      {3, 2, 1, 0},   // 0.25pi - 0.5pi
      {3, 0, 1, 2},   // 0.5pi - 0.75pi
      {0, 3, 2, 1}    // 0.75pi - pi
  };
};

}

#endif //SRC_ENVIRONMENT_H
