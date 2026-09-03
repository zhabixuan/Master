/***********************************************************************************
 *  C++ Source Codes for "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method".
 ***********************************************************************************
 *  Copyright (C) 2022 Bai Li
 *  Users are suggested to cite the following article when they use the source codes.
 *  Bai Li et al., "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method",
 *  IEEE Transactions on Intelligent Transportation Systems, 2022.
 ***********************************************************************************/

#include "liom_local_planner/visualization/plot.h"

#include <geometry_msgs/msg/point.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace liom_local_planner {
namespace visualization {
namespace {
std::string frame_ = "map";

// 修改：ROS 2发布器类型（智能指针）
//ros::Publisher publisher_;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
//rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
// 修改：ROS 2 MarkerArray消息类型
//visualization_msgs::MarkerArray arr_;
visualization_msgs::msg::MarkerArray arr_;
}

// 修改：Node参数改为ROS 2的节点共享指针
void Init(const rclcpp::Node::SharedPtr &node, const std::string &frame, const std::string &topic) {
//void Init(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node, const std::string &frame, const std::string &topic) {
  frame_ = frame;
  // 修改：ROS 2创建发布器方式（替换advertise）
  //publisher_ = node.advertise<visualization_msgs::MarkerArray>(topic, 10, true);
  publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    topic, 
    rclcpp::QoS(10).transient_local()  // 对应ROS 1的latch=true
  );
}

// nav2_util::CallbackReturn on_activate() {
//   publisher_->on_activate();
//   return nav2_util::CallbackReturn::SUCCESS;
// }

// nav2_util::CallbackReturn on_deactivate() {
//   publisher_->on_deactivate();
//   return nav2_util::CallbackReturn::SUCCESS;
// }

nav2_util::CallbackReturn on_cleanup() {
  publisher_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

void
Plot(const Vector &xs, const Vector &ys, double width, Color color, int id, const std::string &ns) {
  // 修改：ROS 2 Marker消息类型
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = frame_;
  // 修改：ROS 2时间获取方式
  msg.header.stamp = rclcpp::Clock().now();
  msg.ns = ns;
  msg.id = id >= 0 ? id : static_cast<int>(arr_.markers.size());

  msg.action = visualization_msgs::msg::Marker::ADD;
  msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = width;
  msg.color = color.toColorRGBA();

  for (size_t i = 0; i < xs.size(); i++) {
    // 修改：ROS 2 Point消息类型
    geometry_msgs::msg::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    pt.z = 0.1 * id;
    msg.points.push_back(pt);
  }

  arr_.markers.push_back(msg);
}

void Plot(const Vector &xs, const Vector &ys, double width,
          const std::vector<Color> &color, int id, const std::string &ns) {
  assert(xs.size() == color.size());

  // 修改：ROS 2 Marker消息类型
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = frame_;
  // 修改：ROS 2时间获取方式
  msg.header.stamp = rclcpp::Clock().now();
  msg.ns = ns;
  msg.id = id >= 0 ? id : static_cast<int>(arr_.markers.size());

  msg.action = visualization_msgs::msg::Marker::ADD;
  msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = width;

  for (size_t i = 0; i < xs.size(); i++) {
    // 修改：ROS 2 Point消息类型
    geometry_msgs::msg::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    msg.points.push_back(pt);
    msg.colors.push_back(color[i].toColorRGBA());
  }

  arr_.markers.push_back(msg);
}


void PlotPolygon(const Vector &xs, const Vector &ys, double width, Color color, int id,
                 const std::string &ns) {
  if (xs.empty() || ys.empty()) {
    return;
  }
  auto xxs = xs;
  auto yys = ys;
  xxs.push_back(xxs[0]);
  yys.push_back(yys[0]);
  Plot(xxs, yys, width, color, id, ns);
}

void PlotPolygon(const Polygon2d &polygon, double width, Color color, int id,
                 const std::string &ns) {
  std::vector<double> xs, ys;
  for (auto &pt: polygon.points()) {
    xs.push_back(pt.x());
    ys.push_back(pt.y());
  }
  PlotPolygon(xs, ys, width, color, id, ns);
}

void PlotTrajectory(const Vector &xs, const Vector &ys, const Vector &vs, double max_velocity, double width,
                    const Color &color, int id, const std::string &ns) {
  std::vector<Color> colors(xs.size());
  float h, tmp;
  color.toHSV(h, tmp, tmp);

  for (size_t i = 0; i < xs.size(); i++) {
    double percent = (vs[i] / max_velocity);
    colors[i] = Color::fromHSV(static_cast<int>(h), percent, 1.0);  // 修改：显式类型转换
  }

  Plot(xs, ys, width, colors, id, ns);
}

void PlotPoints(const Vector &xs, const Vector &ys, double width, const Color &color, int id,
                const std::string &ns) {
  assert(xs.size() == ys.size());

  // 修改：ROS 2 Marker消息类型
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = frame_;
  // 修改：ROS 2时间获取方式
  msg.header.stamp = rclcpp::Clock().now();
  msg.ns = ns.empty() ? "Points" : ns;
  msg.id = id >= 0 ? id : static_cast<int>(arr_.markers.size());

  msg.action = visualization_msgs::msg::Marker::ADD;
  msg.type = visualization_msgs::msg::Marker::POINTS;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = msg.scale.y = width;
  msg.color = color.toColorRGBA();

  for (size_t i = 0; i < xs.size(); i++) {
    // 修改：ROS 2 Point消息类型
    geometry_msgs::msg::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    msg.points.push_back(pt);
  }

  arr_.markers.push_back(msg);
}

void PlotPoints(const Vector &xs, const Vector &ys, const std::vector<Color> &colors, double width, int id,
                const std::string &ns) {
  assert(xs.size() == ys.size());

  // 修改：ROS 2 Marker消息类型
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = frame_;
  // 修改：ROS 2时间获取方式
  msg.header.stamp = rclcpp::Clock().now();
  msg.ns = ns.empty() ? "Points" : ns;
  msg.id = id >= 0 ? id : static_cast<int>(arr_.markers.size());

  msg.action = visualization_msgs::msg::Marker::ADD;
  msg.type = visualization_msgs::msg::Marker::POINTS;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = msg.scale.y = width;

  for (size_t i = 0; i < xs.size(); i++) {
    // 修改：ROS 2 Point消息类型
    geometry_msgs::msg::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    msg.points.push_back(pt);
    msg.colors.push_back(colors[i].toColorRGBA());
  }

  arr_.markers.push_back(msg);
}

void Delete(int id, const std::string &ns) {
  // 修改：ROS 2 Marker消息类型
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = frame_;
  msg.ns = ns;
  msg.id = id;

  msg.action = visualization_msgs::msg::Marker::DELETE;
  arr_.markers.push_back(msg);
}

void Trigger() {
  if (!publisher_) {
    arr_.markers.clear();
    return;
  }
  publisher_->publish(arr_);
  arr_.markers.clear();
}

void Clear(const std::string &ns) {
  arr_.markers.clear();

  if (!publisher_) {
    return;
  }

  // 修改：ROS 2 MarkerArray和Marker消息类型
  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = frame_;
  msg.ns = ns;

  msg.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(msg);
  publisher_->publish(arr);  // ROS 2发布方式一致（指针调用）
}
}
}