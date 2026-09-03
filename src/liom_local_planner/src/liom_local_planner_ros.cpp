#include "liom_local_planner/liom_local_planner_ros.h"
#include "liom_local_planner/visualization/plot.h"
#include "common_math/math_utils.h"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "nav2_util/node_utils.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace liom_local_planner {

// ========== readParameters() ==========
void LiomLocalPlannerROS::readParameters() {
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node in readParameters"};
    }
    
    std::string param_prefix = name_;
    
    // 声明并读取所有参数
    // 基本配置参数
    declare_parameter_if_not_declared(
        node, param_prefix + ".goal_xy_tolerance", rclcpp::ParameterValue(config_.goal_xy_tolerance));
    node->get_parameter(param_prefix + ".goal_xy_tolerance", config_.goal_xy_tolerance);

    declare_parameter_if_not_declared(
        node, param_prefix + ".goal_yaw_tolerance", rclcpp::ParameterValue(config_.goal_yaw_tolerance));
    node->get_parameter(param_prefix + ".goal_yaw_tolerance", config_.goal_yaw_tolerance);

    declare_parameter_if_not_declared(
        node, param_prefix + ".odom_topic", rclcpp::ParameterValue(config_.odom_topic));
    node->get_parameter(param_prefix + ".odom_topic", config_.odom_topic);

    declare_parameter_if_not_declared(
        node, param_prefix + ".global_frame", rclcpp::ParameterValue(config_.global_frame));
    node->get_parameter(param_prefix + ".global_frame", config_.global_frame);

    // 规划器配置参数 - 逐个读取（可靠）
    declare_parameter_if_not_declared(
        node, param_prefix + ".coarse_xy_resolution", rclcpp::ParameterValue(planner_config_->xy_resolution));
    node->get_parameter(param_prefix + ".coarse_xy_resolution", planner_config_->xy_resolution);

    declare_parameter_if_not_declared(
        node, param_prefix + ".coarse_theta_resolution", rclcpp::ParameterValue(planner_config_->theta_resolution));
    node->get_parameter(param_prefix + ".coarse_theta_resolution", planner_config_->theta_resolution);

    declare_parameter_if_not_declared(
        node, param_prefix + ".coarse_step_size", rclcpp::ParameterValue(planner_config_->step_size));
    node->get_parameter(param_prefix + ".coarse_step_size", planner_config_->step_size);

    declare_parameter_if_not_declared(
        node, param_prefix + ".coarse_next_node_num", rclcpp::ParameterValue(planner_config_->next_node_num));
    node->get_parameter(param_prefix + ".coarse_next_node_num", planner_config_->next_node_num);   
    declare_parameter_if_not_declared(
        node, param_prefix + ".coarse_grid_xy_resolution", rclcpp::ParameterValue(planner_config_->grid_xy_resolution));
    node->get_parameter(param_prefix + ".coarse_grid_xy_resolution", planner_config_->grid_xy_resolution);

    declare_parameter_if_not_declared(
        node, param_prefix + ".coarse_forward_penalty", rclcpp::ParameterValue(planner_config_->forward_penalty));
    node->get_parameter(param_prefix + ".coarse_forward_penalty", planner_config_->forward_penalty);

    declare_parameter_if_not_declared(
        node, param_prefix + ".coarse_backward_penalty", rclcpp::ParameterValue(planner_config_->backward_penalty));
    node->get_parameter(param_prefix + ".coarse_backward_penalty", planner_config_->backward_penalty);

    declare_parameter_if_not_declared(
        node, param_prefix + ".coarse_gear_change_penalty", rclcpp::ParameterValue(planner_config_->gear_change_penalty));
    node->get_parameter(param_prefix + ".coarse_gear_change_penalty", planner_config_->gear_change_penalty);

    declare_parameter_if_not_declared(
        node, param_prefix + ".coarse_steering_penalty", rclcpp::ParameterValue(planner_config_->steering_penalty));
    node->get_parameter(param_prefix + ".coarse_steering_penalty", planner_config_->steering_penalty);
    declare_parameter_if_not_declared(
        node, param_prefix + ".coarse_steering_change_penalty", rclcpp::ParameterValue(planner_config_->steering_change_penalty));
    node->get_parameter(param_prefix + ".coarse_steering_change_penalty", planner_config_->steering_change_penalty);

    declare_parameter_if_not_declared(
        node, param_prefix + ".min_waypoints", rclcpp::ParameterValue(planner_config_->min_nfe));
    node->get_parameter(param_prefix + ".min_waypoints", planner_config_->min_nfe);

    declare_parameter_if_not_declared(
        node, param_prefix + ".time_step", rclcpp::ParameterValue(planner_config_->time_step));
    node->get_parameter(param_prefix + ".time_step", planner_config_->time_step);

    declare_parameter_if_not_declared(
        node, param_prefix + ".corridor_max_iter", rclcpp::ParameterValue(planner_config_->corridor_max_iter));
    node->get_parameter(param_prefix + ".corridor_max_iter", planner_config_->corridor_max_iter);
    declare_parameter_if_not_declared(
        node, param_prefix + ".corridor_incremental_limit", rclcpp::ParameterValue(planner_config_->corridor_incremental_limit));
    node->get_parameter(param_prefix + ".corridor_incremental_limit", planner_config_->corridor_incremental_limit);

    declare_parameter_if_not_declared(
        node, param_prefix + ".weight_a", rclcpp::ParameterValue(planner_config_->opti_w_a));
    node->get_parameter(param_prefix + ".weight_a", planner_config_->opti_w_a);

    declare_parameter_if_not_declared(
        node, param_prefix + ".weight_omega", rclcpp::ParameterValue(planner_config_->opti_w_omega));
    node->get_parameter(param_prefix + ".weight_omega", planner_config_->opti_w_omega);

    declare_parameter_if_not_declared(
        node, param_prefix + ".max_iter", rclcpp::ParameterValue(planner_config_->opti_inner_iter_max));
    node->get_parameter(param_prefix + ".max_iter", planner_config_->opti_inner_iter_max);

    declare_parameter_if_not_declared(
        node, param_prefix + ".infeasible_penalty", rclcpp::ParameterValue(planner_config_->opti_w_penalty0));
    node->get_parameter(param_prefix + ".infeasible_penalty", planner_config_->opti_w_penalty0);
    declare_parameter_if_not_declared(
        node, param_prefix + ".infeasible_tolerance", rclcpp::ParameterValue(planner_config_->opti_varepsilon_tol));
    node->get_parameter(param_prefix + ".infeasible_tolerance", planner_config_->opti_varepsilon_tol);

    // 车辆参数
    declare_parameter_if_not_declared(
        node, param_prefix + ".front_hang_length", rclcpp::ParameterValue(planner_config_->vehicle.front_hang_length));
    node->get_parameter(param_prefix + ".front_hang_length", planner_config_->vehicle.front_hang_length);

    declare_parameter_if_not_declared(
        node, param_prefix + ".wheel_base", rclcpp::ParameterValue(planner_config_->vehicle.wheel_base));
    node->get_parameter(param_prefix + ".wheel_base", planner_config_->vehicle.wheel_base);

    declare_parameter_if_not_declared(
        node, param_prefix + ".rear_hang_length", rclcpp::ParameterValue(planner_config_->vehicle.rear_hang_length));
    node->get_parameter(param_prefix + ".rear_hang_length", planner_config_->vehicle.rear_hang_length);

    declare_parameter_if_not_declared(
        node, param_prefix + ".width", rclcpp::ParameterValue(planner_config_->vehicle.width));
    node->get_parameter(param_prefix + ".width", planner_config_->vehicle.width);

    declare_parameter_if_not_declared(
        node, param_prefix + ".max_velocity", rclcpp::ParameterValue(planner_config_->vehicle.max_velocity));
    node->get_parameter(param_prefix + ".max_velocity", planner_config_->vehicle.max_velocity);

    declare_parameter_if_not_declared(
        node, param_prefix + ".min_velocity", rclcpp::ParameterValue(planner_config_->vehicle.min_velocity));
    node->get_parameter(param_prefix + ".min_velocity", planner_config_->vehicle.min_velocity);

    declare_parameter_if_not_declared(
        node, param_prefix + ".max_acceleration", rclcpp::ParameterValue(planner_config_->vehicle.max_acceleration));
    node->get_parameter(param_prefix + ".max_acceleration", planner_config_->vehicle.max_acceleration);

    declare_parameter_if_not_declared(
        node, param_prefix + ".max_steering", rclcpp::ParameterValue(planner_config_->vehicle.phi_max));
    node->get_parameter(param_prefix + ".max_steering", planner_config_->vehicle.phi_max);

    declare_parameter_if_not_declared(
        node, param_prefix + ".max_steering_rate", rclcpp::ParameterValue(planner_config_->vehicle.omega_max));
    node->get_parameter(param_prefix + ".max_steering_rate", planner_config_->vehicle.omega_max);

    declare_parameter_if_not_declared(
        node, param_prefix + ".n_disc", rclcpp::ParameterValue(planner_config_->vehicle.n_disc));
    node->get_parameter(param_prefix + ".n_disc", planner_config_->vehicle.n_disc);
    
    RCLCPP_INFO(logger_, "Parameters loaded for %s", name_.c_str());
}

// ========== configure() ==========
void LiomLocalPlannerROS::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent;
    auto node = parent.lock();
    logger_ = node->get_logger();
    name_ = name;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    // 初始化算法核心对象
    planner_config_ = std::make_shared<PlannerConfig>();
    planner_config_->vehicle.InitializeDiscs();
    
    env_ = std::make_shared<Environment>(planner_config_);
    planner_ = std::make_shared<LiomLocalPlanner>(planner_config_, env_);

    // 读取参数
    readParameters();

    rclcpp::Node::SharedPtr vis_node = std::make_shared<rclcpp::Node>("liom_visualization_node");
    //marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(name_ + "/liom_markers", rclcpp::QoS(10).transient_local());
    visualization::Init(vis_node, "map", "liom_markers");

    path_pub_ = node->create_publisher<nav_msgs::msg::Path>(name_ + "/local_path", 10);

    RCLCPP_INFO(logger_, "Configuring %s of type LiomLocalPlannerROS", name.c_str());
}

void LiomLocalPlannerROS::activate() {
    path_pub_->on_activate();
    //visualization::on_activate();
}

void LiomLocalPlannerROS::deactivate() {
    path_pub_->on_deactivate();
    //visualization::on_deactivate();
}

void LiomLocalPlannerROS::cleanup() {
    path_pub_.reset();
    visualization::on_cleanup();
}

void LiomLocalPlannerROS::setPlan(const nav_msgs::msg::Path & path) {
    global_path_.clear();
    global_path_.reserve(path.poses.size());
    for (const auto & pose_stamped : path.poses) {
        double yaw = tf2::getYaw(pose_stamped.pose.orientation);
        global_path_.emplace_back(pose_stamped.pose.position.x, pose_stamped.pose.position.y, yaw);
    }

    planner_->set_global_path(global_path_);

    if (!path.poses.empty()) {
        const auto& goal_pose = path.poses.back();
        geometry_msgs::msg::PoseStamped trans_goal;
        if (goal_pose.header.frame_id != config_.global_frame) {
            tf_->transform(goal_pose, trans_goal, config_.global_frame);
        } else {
            trans_goal = goal_pose;
        }

        goal_state_.x = trans_goal.pose.position.x;
        goal_state_.y = trans_goal.pose.position.y;
        goal_state_.theta = math::NormalizeAngle(tf2::getYaw(trans_goal.pose.orientation));
    }

    auto node = node_.lock();
    if (node) {
        RCLCPP_INFO(node->get_logger(), "%s: Global path set with %zu poses", 
                   name_.c_str(), path.poses.size());
    }

}

void LiomLocalPlannerROS::updateRobotState(
    const geometry_msgs::msg::PoseStamped & pose, 
    const geometry_msgs::msg::Twist & velocity) 
{
    // 更新起始状态
    start_state_.x = pose.pose.position.x;
    start_state_.y = pose.pose.position.y;
    start_state_.theta = math::NormalizeAngle(tf2::getYaw(pose.pose.orientation));

    // 更新速度
    start_state_.v = velocity.linear.x;

    if (std::fabs(start_state_.v) > 1e-3) {
        double dtheta = velocity.angular.z;
        start_state_.phi = atan(dtheta / start_state_.v * planner_config_->vehicle.wheel_base);
    } else {
        start_state_.phi = 0.0;
    }
}

geometry_msgs::msg::TwistStamped LiomLocalPlannerROS::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) 
{
    geometry_msgs::msg::TwistStamped cmd_vel_stamped;

    auto node = node_.lock();
    if (node) {
        cmd_vel_stamped.header.stamp = node->now();
    } else {
        cmd_vel_stamped.header.stamp = rclcpp::Clock().now();
    }
    cmd_vel_stamped.header.frame_id = "base_link";

    if (goal_checker != nullptr) {
        geometry_msgs::msg::Pose goal_pose;
        goal_pose.position.x = goal_state_.x;
        goal_pose.position.y = goal_state_.y;
        goal_pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), goal_state_.theta));
        if (goal_checker->isGoalReached(pose.pose, goal_pose, velocity)) {
            RCLCPP_INFO(logger_, "Goal reached!");
            solution_ = decltype(solution_)();
            cmd_vel_stamped.twist.linear.x = 0.0;
            cmd_vel_stamped.twist.angular.z = 0.0;
            return cmd_vel_stamped; // 直接返回零速度
        }
    }

    env_->UpdateCostmapObstacles(costmap_ros_->getCostmap());
    updateRobotState(pose, velocity);
    
    // 从上一次解决方案中获取控制量
    if (solution_.states.size() > 1) {
        start_state_.a = solution_.states[1].a;
        start_state_.omega = solution_.states[1].omega;
    }

    bool plan_result = planner_->Plan(solution_, start_state_, goal_state_, solution_);
    if (!plan_result) {
        RCLCPP_WARN(logger_, "%s: Local plan failed", name_.c_str());
        return cmd_vel_stamped;
    }

    nav_msgs::msg::Path msg_path;
    msg_path.header.frame_id = config_.global_frame;
    msg_path.header.stamp = cmd_vel_stamped.header.stamp;
    for (const auto & state : solution_.states) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = msg_path.header;
        pose_stamped.pose.position.x = state.x;
        pose_stamped.pose.position.y = state.y;
        pose_stamped.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), state.theta));
        msg_path.poses.push_back(pose_stamped);
    }
    path_pub_->publish(msg_path);

    // 从解决方案中提取速度命令
    if (solution_.states.size() < 2) {
        cmd_vel_stamped.twist.linear.x = 0.0;
        cmd_vel_stamped.twist.angular.z = 0.0;
    } else {
        cmd_vel_stamped.twist.linear.x = solution_.states[1].v;
        double v = solution_.states[1].v;
        double phi = solution_.states[1].phi;
        double L = planner_config_->vehicle.wheel_base;
        cmd_vel_stamped.twist.angular.z = (v * tan(phi)) / L;
    }
    
    return cmd_vel_stamped;
}

void LiomLocalPlannerROS::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
    if (!planner_config_) {
        return;
    }

    if (percentage) {
        // 百分比方式
        double factor = std::max(0.0, std::min(speed_limit, 100.0)) / 100.0;
        planner_config_->vehicle.max_velocity = planner_config_->vehicle.max_velocity * factor;
    } else {
        // 绝对值方式
        planner_config_->vehicle.max_velocity = std::abs(speed_limit);
    }

    auto node = node_.lock();
    if (node) {
        RCLCPP_INFO(node->get_logger(), "%s: Speed limit set to %.2f %s", 
                   name_.c_str(), speed_limit, percentage ? "%" : "m/s");
    }
}

} // namespace liom_local_planner

#include <pluginlib/class_list_macros.hpp>
// 插件注册宏
PLUGINLIB_EXPORT_CLASS(liom_local_planner::LiomLocalPlannerROS, nav2_core::Controller)