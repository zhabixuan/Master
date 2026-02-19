#ifndef LIOM_LOCAL_PLANNER_LIOM_LOCAL_PLANNER_ROS_H
#define LIOM_LOCAL_PLANNER_LIOM_LOCAL_PLANNER_ROS_H

// ROS2 核心头文件
#include <memory>
#include <string>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

// Nav2 核心接口
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"

// 成本地图
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

// TF2
#include "tf2_ros/buffer.h"

// ROS2 消息
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
//#include "nav_msgs/msg/odometry.hpp"

// 您的算法核心
#include "liom_local_planner/liom_local_planner.h"
#include "liom_local_planner/math/math_utils.h"

namespace liom_local_planner {

struct LiomLocalPlannerROSConfig {
  double goal_xy_tolerance = 0.5;
  double goal_yaw_tolerance = 0.2;
  std::string odom_topic = "odom";
  std::string global_frame = "map";
};

class LiomLocalPlannerROS : public nav2_core::Controller {
public:
  LiomLocalPlannerROS() = default;
  ~LiomLocalPlannerROS() override = default;

  // ========== nav2_core::Controller 生命周期接口 ==========
  
  /**
   * @brief 配置插件 (替代 ROS1 的 initialize)
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief 清理资源
   */
  void cleanup() override;

  /**
   * @brief 激活插件
   */
  void activate() override;

  /**
   * @brief 停用插件
   */
  void deactivate() override;

  // ========== nav2_core::Controller 功能接口 ==========
  
  /**
   * @brief 设置全局路径 (替代 ROS1 的 setPlan)
   * @param path 全局路径 (ROS2 使用 nav_msgs::msg::Path)
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief 计算速度命令 (替代 ROS1 的 computeVelocityCommands)
   * @param pose 当前机器人位姿
   * @param velocity 当前机器人速度
   * @param goal_checker 目标检查器
   * @return 带时间戳的速度命令 (ROS2 使用 TwistStamped)
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief 设置速度限制 (可选实现)
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  // ========== 内部辅助方法 ==========
  
  /**
   * @brief 读取 ROS2 参数
   */
  void readParameters();
  
  
  /**
   * @brief 里程计回调函数 (替代 ROS1 的 odom_helper_)
   */
  //void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  
  /**
   * @brief 更新机器人状态信息
   */
  void updateRobotState(const geometry_msgs::msg::PoseStamped & pose, 
                        const geometry_msgs::msg::Twist & velocity);
  

  // ========== 成员变量 ==========
  
  // 配置
  LiomLocalPlannerROSConfig config_;
  //bool initialized_ = false;
  std::string name_;
  
  // ROS2 资源 (使用智能指针)
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("LiomLocalPlannerROS")};
  //rclcpp::Logger logger_ = rclcpp::get_logger("LiomLocalPlannerROS");
  
  // 订阅器和发布器
  //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> path_pub_;
  //std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>> marker_pub_;
  
  // 里程计数据
  geometry_msgs::msg::Twist current_velocity_;
  std::mutex velocity_mutex_;
  
  // 算法核心
  std::shared_ptr<PlannerConfig> planner_config_;
  std::shared_ptr<Environment> env_;
  std::shared_ptr<LiomLocalPlanner> planner_;
  
  // 状态与路径
  std::vector<math::Pose> global_path_;
  TrajectoryPoint start_state_, goal_state_;
  FullStates solution_;
};

}  // namespace liom_local_planner

#endif  // LIOM_LOCAL_PLANNER_LIOM_LOCAL_PLANNER_ROS_H