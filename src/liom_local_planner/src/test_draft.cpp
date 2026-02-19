//
// Created by yenkn on 1/10/23.
//
#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "liom_local_planner/liom_local_planner.h"
#include "liom_local_planner/visualization/plot.h"

#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace liom_local_planner;

class LiomTestNode : public rclcpp::Node {
public:
  LiomTestNode() : Node("liom_test_node"), config_(std::make_shared<PlannerConfig>()) {
    config_->vehicle.InitializeDiscs();

    env_ = std::make_shared<Environment>(config_);
    planner_ = std::make_shared<LiomLocalPlanner>(config_, env_);

    // ROS 2 Publisher with QoS settings
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/liom_test_path",
      rclcpp::QoS(10).transient_local()  // 对应 ROS 1 的 latch=true
      //10  // 默认 QoS
    );

    // ROS 2 Interactive Marker Server
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      //"/liom_obstacle",
      "liom_obstacle",  // 修复：去掉前缀 /，避免marker名称重复导致交互异常
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_topics_interface(),
      this->get_node_services_interface()
    );

    // Initialize polygons
    polys_ = {
        math::Polygon2d({{-3, -3}, {-3, 3}, {3, 3}, {3, -3}}),
    };
    
    env_->polygons() = polys_;
    
    // Create interactive markers
    for(size_t i = 0; i < polys_.size(); i++) {
      auto marker = CreateMarker(static_cast<int>(i+1), polys_[i], 0.2, visualization::Color::Magenta);
      server_->insert(marker);
      // 修复：为每个marker单独绑定回调（原代码只绑定了Obstacle 1）
      server_->setCallback(marker.name, 
        std::bind(&LiomTestNode::interactiveCallback, this, std::placeholders::_1));
    }
    
    // server_->setCallback("/liom_obstacle/Obstacle 1", 
    //   std::bind(&LiomTestNode::interactiveCallback, this, std::placeholders::_1));
    
    server_->applyChanges();

    // Initialize visualization
    //visualization::Init(this->shared_from_this(), "map", "/liom_test_vis");

    // ROS 2 Subscriber
    start_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose",
      10,
      std::bind(&LiomTestNode::startCallback, this, std::placeholders::_1)
    );

    // Initialize start and goal
    start_.x = -20;
    start_.y = -20;
    start_.theta = 0;
    goal_.x = 20;
    goal_.y = 20;
    goal_.theta = M_PI;

    // Create timer for main loop
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 10 Hz
      std::bind(&LiomTestNode::timerCallback, this)
    );

    // 新增：添加初始化日志，方便调试
    RCLCPP_INFO(this->get_logger(), "Liom Test Node initialized successfully!");
  }

private:
  visualization_msgs::msg::InteractiveMarker CreateMarker(int i, const math::Polygon2d &polygon, double width, visualization::Color c) {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.name = "Obstacle " + std::to_string(i);
    
    // 计算多边形中心点
    double center_x = 0.0, center_y = 0.0;
    for (const auto& pt : polygon.points()) {
      center_x += pt.x();
      center_y += pt.y();
    }
    center_x /= polygon.num_points();
    center_y /= polygon.num_points();
    marker.pose.position.x = center_x;
    marker.pose.position.y = center_y;
    marker.pose.position.z = 0.0; // 确保Z轴为0
    marker.pose.orientation.w = 1.0;

    // 可视化控制
    visualization_msgs::msg::InteractiveMarkerControl visual_control;
    visual_control.always_visible = true;

    visualization_msgs::msg::Marker polygon_marker;
    polygon_marker.header.frame_id = marker.header.frame_id;
    polygon_marker.header.stamp = this->now();
    polygon_marker.ns = "Obstacles";
    polygon_marker.id = i;

    polygon_marker.action = visualization_msgs::msg::Marker::ADD;
    polygon_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    polygon_marker.pose.orientation.w = 1.0;
    polygon_marker.scale.x = width;
    polygon_marker.color = c.toColorRGBA();

    for (size_t j = 0; j < polygon.num_points(); j++) {
      geometry_msgs::msg::Point pt;
      // 使用局部坐标（相对于marker中心）
      pt.x = polygon.points().at(j).x() - center_x;
      pt.y = polygon.points().at(j).y() - center_y;
      pt.z = 0.0; // 确保Z轴为0
      polygon_marker.points.push_back(pt);
    }
    polygon_marker.points.push_back(polygon_marker.points.front());

    visual_control.markers.push_back(polygon_marker);
    marker.controls.push_back(visual_control);

    // 关键修改：只添加XY平面的移动控制，使用MOVE_PLANE模式
    visualization_msgs::msg::InteractiveMarkerControl move_control;
    move_control.name = "move_xy";
    move_control.orientation.w = 1.0;
    move_control.orientation.x = 0.0;
    move_control.orientation.y = 0.0;
    move_control.orientation.z = 0.0;
    move_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    // 设置平面法向量为Z轴，这样移动将限制在XY平面
    move_control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING;
    // 设置平面在XY平面，Z轴位置固定
    marker.controls.push_back(move_control);
    
    return marker;
  }

  void interactiveCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &msg) {
    if (msg->marker_name.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty marker name in feedback");
      return;
    }
    
    // 解析marker索引
    std::string marker_num_str = msg->marker_name.substr(msg->marker_name.find_last_of(' ') + 1);
    int idx = std::stoi(marker_num_str) - 1;
    if (idx < 0 || idx >= static_cast<int>(polys_.size())) {
      RCLCPP_WARN(this->get_logger(), "Invalid marker index: %d", idx);
      return;
    }

    // 直接使用反馈的位置，不进行Z轴复位
    // Interactive Marker Server已经确保移动在XY平面上
    math::Vec2d offset(msg->pose.position.x, msg->pose.position.y);
    
    auto& poly = polys_[idx];
    
    // 计算原始中心点
    double center_x = 0.0, center_y = 0.0;
    for (const auto& pt : poly.points()) {
      center_x += pt.x();
      center_y += pt.y();
    }
    center_x /= poly.num_points();
    center_y /= poly.num_points();
    
    // 计算偏移量
    math::Vec2d original_center(center_x, center_y);
    math::Vec2d delta = offset - original_center;
    
    // 移动多边形
    auto new_poly = poly;
    new_poly.Move(delta);
    polys_[idx] = new_poly;
    env_->polygons().at(idx) = new_poly;
    
    RCLCPP_INFO(this->get_logger(), "Moved obstacle %d to (%.2f, %.2f)", 
                idx+1, msg->pose.position.x, msg->pose.position.y);
  }

  void startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose) {
    double x = pose->pose.pose.position.x;
    double y = pose->pose.pose.position.y;

    double min_distance = DBL_MAX;
    int idx = -1;
    for(size_t i = 0; i < solution_.states.size(); i++) {
      double distance = hypot(solution_.states[i].x - x, solution_.states[i].y - y);
      if(distance < min_distance) {
        min_distance = distance;
        idx = static_cast<int>(i);
      }
    }

    if (idx >= 0) {
      start_ = solution_.states[idx];
      // 新增：添加日志，方便调试起点更新
      RCLCPP_INFO(this->get_logger(), "Updated start position to (%.2f, %.2f)", start_.x, start_.y);
    }
  }

  void timerCallback() {
    if(!planner_->Plan(solution_, start_, goal_, solution_)) {
      RCLCPP_ERROR(this->get_logger(), "Planning failed");
      return;
    }

    // Clear old footprints
    for(int i = 0; i < 1000; i++) {
      visualization::Delete(i, "Footprints");
    }
    visualization::Trigger();

    // Publish path
    nav_msgs::msg::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();
    
    for(size_t i = 0; i < solution_.states.size(); i++) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position.x = solution_.states[i].x;
      pose.pose.position.y = solution_.states[i].y;
      
      // Use tf2 to create quaternion from yaw
      tf2::Quaternion q;
      q.setRPY(0, 0, solution_.states[i].theta);
      pose.pose.orientation = tf2::toMsg(q);
      
      msg.poses.push_back(pose);

      // Plot vehicle footprints
      auto box = config_->vehicle.GenerateBox(solution_.states[i].pose());
      auto color = visualization::Color::White;
      color.set_alpha(0.4);
      visualization::PlotPolygon(math::Polygon2d(box), 0.05, color, static_cast<int>(i), "Footprints");
    }

    path_pub_->publish(msg);
    //RCLCPP_INFO(this->get_logger(), "Published path with %zu poses", msg.poses.size());
    // 修复：使用THROTTLE避免日志刷屏（原代码每秒输出10次日志）
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Published path with %zu poses", msg.poses.size());
    visualization::Trigger();
  }

  // ROS 2 Components
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  // Planning components
  std::shared_ptr<PlannerConfig> config_;
  std::shared_ptr<Environment> env_;
  std::shared_ptr<LiomLocalPlanner> planner_;
  
  std::vector<math::Polygon2d> polys_;
  TrajectoryPoint start_, goal_;
  FullStates solution_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<LiomTestNode>();

  visualization::Init(node, "map", "/liom_test_vis");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}
// ——————————————————————————————————————————————————————————————————————————————————————————————————————
// 为什么移动障碍物，规划的路径不会改变，因为回调函数interactiveCallback根本没有被调用
// 绑定的名称："/liom_obstacle/Obstacle 1"（带前缀），但 server 中实际存在的 marker 名称是"Obstacle 1"（无前缀），名称完全不匹配。
//
// Created by yenkn on 1/10/23.
//
#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "liom_local_planner/liom_local_planner.h"
#include "liom_local_planner/visualization/plot.h"

#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace liom_local_planner;

class LiomTestNode : public rclcpp::Node {
public:
  LiomTestNode() : Node("liom_test_node"), config_(std::make_shared<PlannerConfig>()) {
    config_->vehicle.InitializeDiscs();

    env_ = std::make_shared<Environment>(config_);
    planner_ = std::make_shared<LiomLocalPlanner>(config_, env_);

    // ROS 2 Publisher with QoS settings
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/liom_test_path",
      rclcpp::QoS(10).transient_local()  // 对应 ROS 1 的 latch=true
      //10  // 默认 QoS
    );

    // ROS 2 Interactive Marker Server
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "/liom_obstacle",
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_topics_interface(),
      this->get_node_services_interface()
    );

    // Initialize polygons
    polys_ = {
        math::Polygon2d({{-3, -3}, {-3, 3}, {3, 3}, {3, -3}}),
    };
    
    env_->polygons() = polys_;
    
    // Create interactive markers
    for(size_t i = 0; i < polys_.size(); i++) {
      auto marker = CreateMarker(static_cast<int>(i+1), polys_[i], 0.2, visualization::Color::Magenta);
      server_->insert(marker);
      // (*关键*) 修复：为每个marker单独绑定回调（原代码只绑定了Obstacle 1）
      server_->setCallback(marker.name, 
        std::bind(&LiomTestNode::interactiveCallback, this, std::placeholders::_1));
    }
    
    server_->setCallback("/liom_obstacle/Obstacle 1", 
      std::bind(&LiomTestNode::interactiveCallback, this, std::placeholders::_1));
    
    server_->applyChanges();

    // Initialize visualization
    //visualization::Init(this->shared_from_this(), "map", "/liom_test_vis");

    // ROS 2 Subscriber
    start_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose",
      10,
      std::bind(&LiomTestNode::startCallback, this, std::placeholders::_1)
    );

    // Initialize start and goal
    start_.x = -20;
    start_.y = -20;
    start_.theta = 0;
    goal_.x = 20;
    goal_.y = 20;
    goal_.theta = M_PI;

    // Create timer for main loop
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 10 Hz
      std::bind(&LiomTestNode::timerCallback, this)
    );
  }

private:
  visualization_msgs::msg::InteractiveMarker CreateMarker(int i, const math::Polygon2d &polygon, double width, visualization::Color c) {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.name = "Obstacle " + std::to_string(i);
    
    marker.pose.orientation.w = 1.0;

    visualization_msgs::msg::InteractiveMarkerControl box_control;
    box_control.always_visible = true;

    visualization_msgs::msg::Marker polygon_marker;
    polygon_marker.header.frame_id = marker.header.frame_id;
    polygon_marker.header.stamp = this->now();
    polygon_marker.ns = "Obstacles";
    polygon_marker.id = i;

    polygon_marker.action = visualization_msgs::msg::Marker::ADD;
    polygon_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    polygon_marker.pose.orientation.w = 1.0;
    polygon_marker.scale.x = width;
    polygon_marker.color = c.toColorRGBA();

    for (size_t i = 0; i < polygon.num_points(); i++) {
      geometry_msgs::msg::Point pt;
      pt.x = polygon.points().at(i).x();
      pt.y = polygon.points().at(i).y();
      polygon_marker.points.push_back(pt);
    }
    polygon_marker.points.push_back(polygon_marker.points.front());
  
    box_control.markers.push_back(polygon_marker);
    marker.controls.push_back(box_control);

    visualization_msgs::msg::InteractiveMarkerControl move_control;
    move_control.name = "move_x";
    move_control.orientation.w = 0.707107f;
    move_control.orientation.x = 0;
    move_control.orientation.y = 0.707107f;
    move_control.orientation.z = 0;
    move_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;

    marker.controls.push_back(move_control);
    return marker;
  }

  void interactiveCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &msg) {
    if (msg->marker_name.empty()) return;
    
    int idx = msg->marker_name.back() - '1';
    if (idx < 0 || idx >= static_cast<int>(polys_.size())) return;

    auto new_poly = polys_[idx];
    new_poly.Move({ msg->pose.position.x, msg->pose.position.y });
    env_->polygons().at(idx) = new_poly;
    
    // Update the marker position
    auto marker = CreateMarker(idx + 1, new_poly, 0.2, visualization::Color::Magenta);
    server_->insert(marker);
    server_->applyChanges();
  }

  void startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose) {
    double x = pose->pose.pose.position.x;
    double y = pose->pose.pose.position.y;

    double min_distance = DBL_MAX;
    int idx = -1;
    for(size_t i = 0; i < solution_.states.size(); i++) {
      double distance = hypot(solution_.states[i].x - x, solution_.states[i].y - y);
      if(distance < min_distance) {
        min_distance = distance;
        idx = static_cast<int>(i);
      }
    }

    if (idx >= 0) {
      start_ = solution_.states[idx];
    }
  }

  void timerCallback() {
    if(!planner_->Plan(solution_, start_, goal_, solution_)) {
      RCLCPP_ERROR(this->get_logger(), "Planning failed");
      return;
    }

    // Clear old footprints
    for(int i = 0; i < 1000; i++) {
      visualization::Delete(i, "Footprints");
    }
    visualization::Trigger();

    // Publish path
    nav_msgs::msg::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();
    
    for(size_t i = 0; i < solution_.states.size(); i++) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position.x = solution_.states[i].x;
      pose.pose.position.y = solution_.states[i].y;
      
      // Use tf2 to create quaternion from yaw
      tf2::Quaternion q;
      q.setRPY(0, 0, solution_.states[i].theta);
      pose.pose.orientation = tf2::toMsg(q);
      
      msg.poses.push_back(pose);

      // Plot vehicle footprints
      auto box = config_->vehicle.GenerateBox(solution_.states[i].pose());
      auto color = visualization::Color::White;
      color.set_alpha(0.4);
      visualization::PlotPolygon(math::Polygon2d(box), 0.05, color, static_cast<int>(i), "Footprints");
    }

    path_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published path with %zu poses", msg.poses.size());
    visualization::Trigger();
  }

  // ROS 2 Components
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  // Planning components
  std::shared_ptr<PlannerConfig> config_;
  std::shared_ptr<Environment> env_;
  std::shared_ptr<LiomLocalPlanner> planner_;
  
  std::vector<math::Polygon2d> polys_;
  TrajectoryPoint start_, goal_;
  FullStates solution_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<LiomTestNode>();

  visualization::Init(node, "map", "/liom_test_vis");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}