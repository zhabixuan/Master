//
// Created by yenkn on 2021/2/24.
//
//#pragma once
#ifndef COMMON__MATH__POSE_HPP_
#define COMMON__MATH__POSE_HPP_

#include "vec2d.h"

namespace common {
namespace math {

class Pose {
public:
  Pose() = default;
  Pose(double x_in, double y_in, double theta_in): x(x_in), y(y_in), theta(theta_in) {}

  inline bool operator==(const Pose & rhs)
  {
    return this->x == rhs.x && this->y == rhs.y && this->theta == rhs.theta;
  }

  inline bool operator!=(const Pose & rhs)
  {
    return !(*this == rhs);
  }

  inline operator Vec2d() const { return { x, y}; }

  inline Pose relativeTo(const Pose &coord) const {
    double dx = this->x - coord.x;
    double dy = this->y - coord.y;
    return {
      dx * cos(coord.theta) + dy * sin(coord.theta),
      -dx * sin(coord.theta) + dy * cos(coord.theta),
      theta - coord.theta
    };
  }

  inline Pose extend(double length) const {
    return transform({ length, 0, 0 });
  }

  inline Pose transform(const Pose &relative) const {
    return {
      this->x + relative.x * cos(theta) - relative.y * sin(theta),
      this->y + relative.x * sin(theta) + relative.y * cos(theta),
      this->theta + relative.theta
    };
  }

  inline double DistanceTo(const Vec2d &rhs) const {
    return hypot(this->x - rhs.x(), this->y - rhs.y());
  }

  double x, y, theta;
};

}
}

#endif
