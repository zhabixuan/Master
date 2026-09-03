#ifndef PLANNER__CUSTOM_PLANNER_HPP_
#define PLANNER__CUSTOM_PLANNER_HPP_

#include <memory>
#include <vector>
#include <string>

// #include "trajectory_planner/a_star.hpp"
// //#include "trajectory_planner/smoother.hpp"
// #include "trajectory_planner/utils.hpp"
// #include "trajectory_planner/costmap_downsampler.hpp"
#include "hybrid_Astar/a_star.hpp"
//#include "hybrid_Astar/smoother.hpp"
#include "hybrid_Astar/utils.hpp"
#include "hybrid_Astar/costmap_downsampler.hpp"

// liom
//#include "liom_local_planner/coarse_path_planner.h"
#include "common_math/math_utils.h"
#include "liom_local_planner/time.h"
#include "liom_local_planner/visualization/plot.h"
#include "liom_local_planner/environment.h"
#include "liom_local_planner/optimizer_interface.h"
#include "liom_local_planner/planner_config.h"
#include "liom_local_planner/liom_local_planner.h"
#include "liom_local_planner/lightweight_nlp_problem.h"

#include "liom_local_planner/path_planner.h"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"

namespace planner
{

class CustomPlanner : public nav2_core::GlobalPlanner
{
public:
  /**
   * @brief constructor
   */
  CustomPlanner();

  /**
   * @brief destructor
   */
  ~CustomPlanner();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros Costmap2DROS object
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup lifecycle node
   */
  void cleanup() override;

  /**
   * @brief Activate lifecycle node
   */
  void activate() override;

  /**
   * @brief Deactivate lifecycle node
   */
  void deactivate() override;

  /**
   * @brief Creating a plan from start and goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @return nav2_msgs::Path of the generated path
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

  bool coarsePlan(
    const common::math::Pose & start,
    const common::math::Pose & goal,
    std::vector<common::math::Pose> & path);

  bool liomPlan(const liom_local_planner::FullStates &prev_sol, 
  const liom_local_planner::TrajectoryPoint &start, 
  const liom_local_planner::TrajectoryPoint &goal, 
  liom_local_planner::FullStates &result);

  /**
   * @brief 基于弧长的路径插值
   * @param solution 优化后的轨迹（稀疏状态点）
   * @param dt 时间步长（秒），默认 0.01 秒
   * @return 稠密的路径点序列
   */
  void interpolatePathByTime(
    const liom_local_planner::FullStates& solution,
    std::vector<liom_local_planner::TrajectoryPoint>& interpolated_states,
    double dt = 0.05);

protected:
  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  std::unique_ptr<hybrid_Astar::AStarAlgorithm<hybrid_Astar::NodeHybrid>> _a_star;
  std::unique_ptr<liom_local_planner::LiomLocalPlanner> _liom_local_planner;
  //std::unique_ptr<liom_local_planner::LiomLocalPlanner> _liom_local_planner;
  hybrid_Astar::GridCollisionChecker _collision_checker;
  //std::unique_ptr<hybrid_Astar::Smoother> _smoother;
  rclcpp::Clock::SharedPtr _clock;
  rclcpp::Logger _logger{rclcpp::get_logger("SmacPlannerHybrid")};
  nav2_costmap_2d::Costmap2D * _costmap;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> _costmap_ros;
  std::unique_ptr<hybrid_Astar::CostmapDownsampler> _costmap_downsampler;
  std::string _global_frame, _name;
  float _lookup_table_dim;
  float _tolerance;
  bool _downsample_costmap;
  int _downsampling_factor;
  double _angle_bin_size;
  unsigned int _angle_quantizations;
  bool _allow_unknown;
  int _max_iterations;
  int _max_on_approach_iterations;
  hybrid_Astar::SearchInfo _search_info;
  double _max_planning_time;
  double _lookup_table_size;
  double _minimum_turning_radius_global_coords;
  std::string _motion_model_for_search;
  hybrid_Astar::MotionModel _motion_model;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr _raw_plan_publisher;
  std::mutex _mutex;
  rclcpp_lifecycle::LifecycleNode::WeakPtr _node;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _dyn_params_handler;


  std::shared_ptr<liom_local_planner::PlannerConfig> planner_config_;
  std::shared_ptr<liom_local_planner::Environment> env_;
};

}  // namespace planner

#endif   // PLANNER__CUSTOM_PLANNER_HPP_