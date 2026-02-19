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
      // 修复：为每个marker单独绑定回调（原代码只绑定了Obstacle 1）
      server_->setCallback(marker.name, 
        std::bind(&LiomTestNode::interactiveCallback, this, std::placeholders::_1));
    }
    
    // server_->setCallback("/liom_obstacle/Obstacle 1", 
    //   std::bind(&LiomTestNode::interactiveCallback, this, std::placeholders::_1));
    // （*关键*）名称要和 CreateMarker 中的一致
    // server_->setCallback("Obstacle 1", 
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
  }

private:
  visualization_msgs::msg::InteractiveMarker CreateMarker(int i, const math::Polygon2d &polygon, double width, visualization::Color c) {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.name = "Obstacle " + std::to_string(i);

    // (*关键*)没有这段代码,每次移动完后,中心点会跳回原点,而多边形障碍物则在鼠标移动到的新位置,出现中心点和多边形错位
    // 新增：让Marker初始位姿与多边形中心一致
    marker.pose.position.x = polygon.center().x();
    marker.pose.position.y = polygon.center().y();
    
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
    // 默认YZ方向，黑色是X轴负方向？
    // move_control.orientation.w = 1;
    // move_control.orientation.x = 0;
    // move_control.orientation.y = 0;
    // move_control.orientation.z = 0;
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
    // // 1. 获取「当前多边形」和「当前中心」（基于本地最新polys_，已同步）
    // auto& current_poly = polys_[idx];
    // math::Vec2d current_center = current_poly.center();

    // // 2. 获取交互式标记的「目标绝对中心」（XY平面纯平移）
    // math::Vec2d target_center(msg->pose.position.x, msg->pose.position.y);

    // // 3. 计算「正确的相对偏移量」：目标中心 - 当前中心（核心修复）
    // math::Vec2d delta = target_center - current_center;

    // // 4. 执行相对移动：仅移动差值距离，无跳变
    // auto new_poly = current_poly;
    // new_poly.Move(delta);

    // // 5. 双向同步：环境（规划器感知） + 本地（后续拖动基础）
    // env_->polygons().at(idx) = new_poly;
    // polys_[idx] = new_poly;
    
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