#pragma once

#include "common_math/vec2d.h"
#include "common_math/polygon2d.h"

#include <mutex>
//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
//#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include "nav2_util/lifecycle_node.hpp"

#include "color.h"

namespace liom_local_planner {

namespace math = common::math;

namespace visualization {

using math::Vec2d;
using math::Polygon2d;

using Vector = std::vector<double>;

void Init(const rclcpp::Node::SharedPtr &node, const std::string &frame, const std::string &topic);
//void Init(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node, const std::string &frame, const std::string &topic);

// nav2_util::CallbackReturn on_activate();
// nav2_util::CallbackReturn on_deactivate();
nav2_util::CallbackReturn on_cleanup();

void Plot(const Vector &xs, const Vector &ys, double width = 0.1, Color color = Color(1, 1, 1),
          int id = -1, const std::string &ns = "");

void Plot(const Vector &xs, const Vector &ys, double width = 0.1, const std::vector<Color> &color = {},
          int id = -1, const std::string &ns = "");

void PlotPolygon(const Vector &xs, const Vector &ys, double width = 0.1, Color color = Color::White,
                 int id = -1, const std::string &ns = "");

void PlotPolygon(const Polygon2d &polygon, double width = 0.1, Color color = Color::White,
                 int id = -1, const std::string &ns = "");

void PlotTrajectory(const Vector &xs, const Vector &ys, const Vector &vs, double max_velocity = 10.0,
                    double width = 0.1, const Color &color = Color::Blue,
                    int id = -1, const std::string &ns = "");

void PlotPoints(const Vector &xs, const Vector &ys, double width = 0.1, const Color &color = Color::White,
                int id = -1, const std::string &ns = "");

void PlotPoints(const Vector &xs, const Vector &ys, const std::vector<Color> &colors, double width = 0.1,
                int id = -1, const std::string &ns = "");

void Trigger();

void Delete(int id, const std::string &ns);

void Clear(const std::string &ns);
}

}