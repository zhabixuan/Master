  // Copyright (c) 2020, Samsung Research America
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

  #include <string>
  #include <memory>
  #include <vector>
  #include <algorithm>
  #include <limits>

  #include "Eigen/Core"
  #include "planner/custom_planner.hpp"

  #include <ompl/base/spaces/ReedsSheppStateSpace.h>
  #include <ompl/base/spaces/DubinsStateSpace.h>
  #include <ompl/base/ScopedState.h>
  #include <queue>
  #include <tf2/utils.h>

  namespace planner
  {

  //#define ROS2_HYBRID_A
  //#define COARSE_PATH_PLANNER
  #define BENCHMARK_TESTING

  using namespace std::chrono;  // NOLINT
  using rcl_interfaces::msg::ParameterType;
  using std::placeholders::_1;

  namespace llp = liom_local_planner;
  using liom_local_planner::inf;

  CustomPlanner::CustomPlanner()
  : _a_star(nullptr),
    _collision_checker(nullptr, 1, nullptr),
    //_smoother(nullptr),
    _costmap(nullptr),
    _costmap_downsampler(nullptr)
  {
  }

  CustomPlanner::~CustomPlanner()
  {
    RCLCPP_INFO(
      _logger, "Destroying plugin %s of type CustomPlannerHybrid",
      _name.c_str());
  }

  void CustomPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    _node = parent;
    auto node = parent.lock();

    _logger = node->get_logger();
    _clock = node->get_clock();
    _costmap = costmap_ros->getCostmap();
    _costmap_ros = costmap_ros;
    _name = name;
    _global_frame = costmap_ros->getGlobalFrameID();

    RCLCPP_INFO(_logger, "Configuring %s of type CustomPlannerHybrid", name.c_str());

    int angle_quantizations;
    double analytic_expansion_max_length_m;
    bool smooth_path;

    // General planner params
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".downsample_costmap", rclcpp::ParameterValue(false));
    node->get_parameter(name + ".downsample_costmap", _downsample_costmap);
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".downsampling_factor", rclcpp::ParameterValue(1));
    node->get_parameter(name + ".downsampling_factor", _downsampling_factor);

    nav2_util::declare_parameter_if_not_declared(
      node, name + ".angle_quantization_bins", rclcpp::ParameterValue(72));
    node->get_parameter(name + ".angle_quantization_bins", angle_quantizations);
    _angle_bin_size = 2.0 * M_PI / angle_quantizations;
    _angle_quantizations = static_cast<unsigned int>(angle_quantizations);

    nav2_util::declare_parameter_if_not_declared(
      node, name + ".tolerance", rclcpp::ParameterValue(0.25));
    _tolerance = static_cast<float>(node->get_parameter(name + ".tolerance").as_double());
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".allow_unknown", rclcpp::ParameterValue(true));
    node->get_parameter(name + ".allow_unknown", _allow_unknown);
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".max_iterations", rclcpp::ParameterValue(1000000));
    node->get_parameter(name + ".max_iterations", _max_iterations);
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".max_on_approach_iterations", rclcpp::ParameterValue(1000));
    node->get_parameter(name + ".max_on_approach_iterations", _max_on_approach_iterations);
    // nav2_util::declare_parameter_if_not_declared(
    //   node, name + ".smooth_path", rclcpp::ParameterValue(true));
    // node->get_parameter(name + ".smooth_path", smooth_path);

    nav2_util::declare_parameter_if_not_declared(
      node, name + ".minimum_turning_radius", rclcpp::ParameterValue(0.4));
    node->get_parameter(name + ".minimum_turning_radius", _minimum_turning_radius_global_coords);
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".cache_obstacle_heuristic", rclcpp::ParameterValue(false));
    node->get_parameter(name + ".cache_obstacle_heuristic", _search_info.cache_obstacle_heuristic);
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".reverse_penalty", rclcpp::ParameterValue(2.0));
    node->get_parameter(name + ".reverse_penalty", _search_info.reverse_penalty);
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".change_penalty", rclcpp::ParameterValue(0.0));
    node->get_parameter(name + ".change_penalty", _search_info.change_penalty);
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".non_straight_penalty", rclcpp::ParameterValue(1.2));
    node->get_parameter(name + ".non_straight_penalty", _search_info.non_straight_penalty);
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".cost_penalty", rclcpp::ParameterValue(2.0));
    node->get_parameter(name + ".cost_penalty", _search_info.cost_penalty);
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".retrospective_penalty", rclcpp::ParameterValue(0.015));
    node->get_parameter(name + ".retrospective_penalty", _search_info.retrospective_penalty);
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".analytic_expansion_ratio", rclcpp::ParameterValue(3.5));
    node->get_parameter(name + ".analytic_expansion_ratio", _search_info.analytic_expansion_ratio);
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".analytic_expansion_max_length", rclcpp::ParameterValue(3.0));
    node->get_parameter(name + ".analytic_expansion_max_length", analytic_expansion_max_length_m);
    _search_info.analytic_expansion_max_length =
      analytic_expansion_max_length_m / _costmap->getResolution();

    nav2_util::declare_parameter_if_not_declared(
      node, name + ".max_planning_time", rclcpp::ParameterValue(5.0));
    node->get_parameter(name + ".max_planning_time", _max_planning_time);
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".lookup_table_size", rclcpp::ParameterValue(20.0));
    node->get_parameter(name + ".lookup_table_size", _lookup_table_size);

    nav2_util::declare_parameter_if_not_declared(
      node, name + ".motion_model_for_search", rclcpp::ParameterValue(std::string("DUBIN")));
    node->get_parameter(name + ".motion_model_for_search", _motion_model_for_search);
    _motion_model = hybrid_Astar::fromString(_motion_model_for_search);
    if (_motion_model == hybrid_Astar::MotionModel::UNKNOWN) {
      RCLCPP_WARN(
        _logger,
        "Unable to get MotionModel search type. Given '%s', "
        "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP, STATE_LATTICE.",
        _motion_model_for_search.c_str());
    }

    if (_max_on_approach_iterations <= 0) {
      RCLCPP_INFO(
        _logger, "On approach iteration selected as <= 0, "
        "disabling tolerance and on approach iterations.");
      _max_on_approach_iterations = std::numeric_limits<int>::max();
    }

    if (_max_iterations <= 0) {
      RCLCPP_INFO(
        _logger, "maximum iteration selected as <= 0, "
        "disabling maximum iterations.");
      _max_iterations = std::numeric_limits<int>::max();
    }

    // convert to grid coordinates
    if (!_downsample_costmap) {
      _downsampling_factor = 1;
    }
    _search_info.minimum_turning_radius =
      _minimum_turning_radius_global_coords / (_costmap->getResolution() * _downsampling_factor);
    _lookup_table_dim =
      static_cast<float>(_lookup_table_size) /
      static_cast<float>(_costmap->getResolution() * _downsampling_factor);

    // Make sure its a whole number
    _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

    // Make sure its an odd number
    if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
      RCLCPP_INFO(
        _logger,
        "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
        _lookup_table_dim);
      _lookup_table_dim += 1.0;
    }

    // visualization
    rclcpp::Node::SharedPtr visualization_node = std::make_shared<rclcpp::Node>("liom_visualization_node");
    llp::visualization::Init(visualization_node, "map", "liom_markers");
    //
    // planner_config
    planner_config_ = std::make_shared<llp::PlannerConfig>();
    // Initialize planner
    planner_config_->vehicle.InitializeDiscs();
    //
    env_ = std::make_shared<llp::Environment>(planner_config_, _costmap);

    // Initialize collision checker
    _collision_checker = hybrid_Astar::GridCollisionChecker(_costmap, _angle_quantizations, node);
    // _collision_checker.setFootprint(
    //   _costmap_ros->getRobotFootprint(),
    //   _costmap_ros->getUseRadius(),
    //   hybrid_Astar::findCircumscribedCost(_costmap_ros));


    // Initialize A* template
    _a_star = std::make_unique<hybrid_Astar::AStarAlgorithm<hybrid_Astar::NodeHybrid>>(_motion_model, _search_info);
    _a_star->initialize(
      _allow_unknown,
      _max_iterations,
      _max_on_approach_iterations,
      _max_planning_time,
      _lookup_table_dim,
      _angle_quantizations);

    // Initialize path smoother
    if (smooth_path) {
      hybrid_Astar::SmootherParams params;
      params.get(node, name);
      //_smoother = std::make_unique<hybrid_Astar::Smoother>(params);
      //_smoother->initialize(_minimum_turning_radius_global_coords);
    }

    // Initialize costmap downsampler
    if (_downsample_costmap && _downsampling_factor > 1) {
      _costmap_downsampler = std::make_unique<hybrid_Astar::CostmapDownsampler>();
      std::string topic_name = "downsampled_costmap";
      _costmap_downsampler->on_configure(
        node, _global_frame, topic_name, _costmap, _downsampling_factor);
    }

    _raw_plan_publisher = node->create_publisher<nav_msgs::msg::Path>("unsmoothed_plan", 1);

    RCLCPP_INFO(
      _logger, "Configured plugin %s of type CustomPlannerHybrid with "
      "maximum iterations %i, max on approach iterations %i, and %s. Tolerance %.2f."
      "Using motion model: %s.",
      _name.c_str(), _max_iterations, _max_on_approach_iterations,
      _allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
      _tolerance, toString(_motion_model).c_str());
  }

  void CustomPlanner::activate()
  {
    RCLCPP_INFO(
      _logger, "Activating plugin %s of type CustomPlannerHybrid",
      _name.c_str());
    _raw_plan_publisher->on_activate();
    if (_costmap_downsampler) {
      _costmap_downsampler->on_activate();
    }

    auto node = _node.lock();
    // Add callback for dynamic parameters
    _dyn_params_handler = node->add_on_set_parameters_callback(
      std::bind(&CustomPlanner::dynamicParametersCallback, this, _1));
    //llp::visualization::on_activate();
  }

  void CustomPlanner::deactivate()
  {
    RCLCPP_INFO(
      _logger, "Deactivating plugin %s of type CustomPlannerHybrid",
      _name.c_str());
    _raw_plan_publisher->on_deactivate();
    if (_costmap_downsampler) {
      _costmap_downsampler->on_deactivate();
    }
    _dyn_params_handler.reset();
    //llp::visualization::on_deactivate();
  }

  void CustomPlanner::cleanup()
  {
    RCLCPP_INFO(
      _logger, "Cleaning up plugin %s of type CustomPlannerHybrid",
      _name.c_str());
    _a_star.reset();
    //_smoother.reset();
    if (_costmap_downsampler) {
      _costmap_downsampler->on_cleanup();
      _costmap_downsampler.reset();
    }
    _raw_plan_publisher.reset();
    llp::visualization::on_cleanup();
  }

  #ifdef ROS2_HYBRID_A
  nav_msgs::msg::Path CustomPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    std::lock_guard<std::mutex> lock_reinit(_mutex);
    steady_clock::time_point a = steady_clock::now();

    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

    // Downsample costmap, if required
    nav2_costmap_2d::Costmap2D * costmap = _costmap;
    if (_costmap_downsampler) {
      costmap = _costmap_downsampler->downsample(_downsampling_factor);
      _collision_checker.setCostmap(costmap);
    }

    // Set collision checker and costmap information
    // _collision_checker.setFootprint(
    //   _costmap_ros->getRobotFootprint(),
    //   _costmap_ros->getUseRadius(),
    //   hybrid_Astar::findCircumscribedCost(_costmap_ros));
    _a_star->setCollisionChecker(&_collision_checker);

    // Set starting point, in A* bin search coordinates
    unsigned int mx, my;
    if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx, my)) {
      throw std::runtime_error("Start pose is out of costmap!");
    }

    double orientation_bin = std::round(tf2::getYaw(start.pose.orientation) / _angle_bin_size);
    while (orientation_bin < 0.0) {
      orientation_bin += static_cast<float>(_angle_quantizations);
    }
    // This is needed to handle precision issues
    if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
      orientation_bin -= static_cast<float>(_angle_quantizations);
    }
    _a_star->setStart(mx, my, static_cast<unsigned int>(orientation_bin));

    // Set goal point, in A* bin search coordinates
    if (!costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my)) {
      throw std::runtime_error("Goal pose is out of costmap!");
    }
    orientation_bin = std::round(tf2::getYaw(goal.pose.orientation) / _angle_bin_size);
    while (orientation_bin < 0.0) {
      orientation_bin += static_cast<float>(_angle_quantizations);
    }
    // This is needed to handle precision issues
    if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
      orientation_bin -= static_cast<float>(_angle_quantizations);
    }
    _a_star->setGoal(mx, my, static_cast<unsigned int>(orientation_bin));

    // Setup message
    nav_msgs::msg::Path plan;
    plan.header.stamp = _clock->now();
    plan.header.frame_id = _global_frame;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    // Compute plan
    hybrid_Astar::NodeHybrid::CoordinateVector path;
    int num_iterations = 0;
    std::string error;

    try {
      if (!_a_star->createPath(
          path, num_iterations, _tolerance / static_cast<float>(costmap->getResolution())))
      {
        RCLCPP_WARN(_logger, "Failed to create path.");
        if (num_iterations < _a_star->getMaxIterations()) {
          error = std::string("no valid path found");
        } else {
          error = std::string("exceeded maximum iterations");
        }
      }
    } catch (const std::runtime_error & e) {
      error = "invalid use: ";
      error += e.what();
    }

    if (!error.empty()) {
      RCLCPP_WARN(
        _logger,
        "%s: failed to create plan, %s.",
        _name.c_str(), error.c_str());
      return plan;
    }

    // Convert to world coordinates
    plan.poses.reserve(path.size());
    for (int i = path.size() - 1; i >= 0; --i) {
      pose.pose = hybrid_Astar::getWorldCoords(path[i].x, path[i].y, costmap);
      pose.pose.orientation = hybrid_Astar::getWorldOrientation(path[i].theta);
      plan.poses.push_back(pose);
    }

    // Publish raw path for debug
    if (_raw_plan_publisher->get_subscription_count() > 0) {
      _raw_plan_publisher->publish(plan);
    }

    // Find how much time we have left to do smoothing
    steady_clock::time_point b = steady_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(b - a);
    double time_remaining = _max_planning_time - static_cast<double>(time_span.count());

  #ifdef BENCHMARK_TESTING
    std::cout << "It took " << time_span.count() * 1000 <<
      " milliseconds with " << num_iterations << " iterations." << std::endl;
  #endif
    RCLCPP_INFO(_logger, "Final path size is %ld", plan.poses.size());
    return plan;
  }

  #elif defined(COARSE_PATH_PLANNER)
  nav_msgs::msg::Path CustomPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    steady_clock::time_point a = steady_clock::now(); // Start time
    namespace llp = liom_local_planner;
    llp::math::Pose start_pose, goal_pose;
    start_pose.x = (start.pose.position.x);
    start_pose.y = (start.pose.position.y);
    start_pose.theta = (tf2::getYaw(start.pose.orientation));
    goal_pose.x = (goal.pose.position.x);
    goal_pose.y = (goal.pose.position.y);
    goal_pose.theta = (tf2::getYaw(goal.pose.orientation));
    std::shared_ptr<llp::PlannerConfig> planner_config_;
    std::shared_ptr<llp::Environment> env_;
    planner_config_ = std::make_shared<llp::PlannerConfig>();
    // planner_config_->vehicle.InitializeDiscs();
    env_ = std::make_shared<llp::Environment>(planner_config_, _costmap);
    // env_->UpdateCostmapObstacles(_costmap);
    //llp::CoarsePathPlanner planner_(planner_config_, env_);
    llp::PathPlanner planner_(planner_config_, env_);
    std::vector<llp::math::Pose> result;
    RCLCPP_INFO(_logger, "radius is %f", planner_config_->vehicle.disc_radius);
    nav_msgs::msg::Path plan;
    plan.header.stamp = _clock->now();
    plan.header.frame_id = _global_frame;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    if (!planner_.Plan(start_pose, goal_pose, result)) {
      RCLCPP_WARN(
        _logger,
        "%s: failed to create plan using liom_local_planner.",
        _name.c_str());
      nav_msgs::msg::Path empty_plan;
      empty_plan.header.stamp = _clock->now();
      empty_plan.header.frame_id = _global_frame;
      return empty_plan;
    }
    for (const auto & p : result) {
      pose.pose.position.x = p.x;
      pose.pose.position.y = p.y;
      pose.pose.orientation = hybrid_Astar::getWorldOrientation(p.theta);
      plan.poses.push_back(pose);
    }
    steady_clock::time_point b = steady_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(b - a);
  #ifdef BENCHMARK_TESTING
    std::cout << "It took " << time_span.count() * 1000 <<
      " milliseconds." << std::endl;
  #endif
    RCLCPP_INFO(_logger, "path.size() is %ld", plan.poses.size());
    return plan;
  }

  #else 
  nav_msgs::msg::Path CustomPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    namespace llp = liom_local_planner;
    llp::FullStates solution_;
    llp::TrajectoryPoint llp_start, llp_goal;
    llp_start.x = start.pose.position.x;
    llp_start.y = start.pose.position.y;
    llp_start.theta = tf2::getYaw(start.pose.orientation);
    llp_goal.x = goal.pose.position.x;
    llp_goal.y = goal.pose.position.y;
    llp_goal.theta = tf2::getYaw(goal.pose.orientation);
    std::shared_ptr<liom_local_planner::PlannerConfig> planner_config_;
    std::shared_ptr<liom_local_planner::Environment> env_;
    planner_config_ = std::make_shared<liom_local_planner::PlannerConfig>();
    // planner_config_->vehicle.InitializeDiscs();
    env_ = std::make_shared<liom_local_planner::Environment>(planner_config_, _costmap);
    // env_->UpdateCostmapObstacles(_costmap);
    _liom_local_planner = std::make_unique<llp::LiomLocalPlanner>(planner_config_, env_);
    RCLCPP_INFO(_logger, "radius is %f", planner_config_->vehicle.disc_radius);
    bool success = true;
    for (int i = 0; i < 1; i++) {
      if (!_liom_local_planner->Plan(solution_,llp_start, llp_goal, solution_)) {
        success = false;
        break;
      }
    }
    if (success) {
      std::vector<llp::TrajectoryPoint> interpolated_states;
      interpolatePathByTime(solution_, interpolated_states, 0.1);
      
      nav_msgs::msg::Path plan;
      plan.header.stamp = _clock->now();
      plan.header.frame_id = _global_frame;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = plan.header;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;

      for (const auto & p : interpolated_states) {
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        pose.pose.orientation = hybrid_Astar::getWorldOrientation(p.theta);
        plan.poses.push_back(pose);
      }
      RCLCPP_INFO(_logger, "path.size() is %ld", plan.poses.size());
      return plan;
    } else {
      RCLCPP_WARN(
        _logger,
        "%s: failed to create plan using liom_local_planner.",
        _name.c_str());
      nav_msgs::msg::Path empty_plan;
      empty_plan.header.stamp = _clock->now();
      empty_plan.header.frame_id = _global_frame;
      return empty_plan;
    }

  }
  #endif
void CustomPlanner::interpolatePathByTime(
    const llp::FullStates& solution,
    std::vector<llp::TrajectoryPoint>& interpolated_states,
    double dt)
{
    if (solution.states.empty()) {
        return;
    }
    interpolated_states.clear();

    const size_t n = solution.states.size();
    const double total_time = solution.tf;
    const double wheel_base = _liom_local_planner->config_->vehicle.wheel_base;
    const double phi_max = _liom_local_planner->config_->vehicle.phi_max;
    const double max_accel = _liom_local_planner->config_->vehicle.max_acceleration;
    const double omega_max = _liom_local_planner->config_->vehicle.omega_max;

    if (n < 3) {
        interpolated_states = solution.states;
        return;
    }

    // Extract x, y, theta from original solution; unwrap theta for continuity
    std::vector<double> xs(n), ys(n), thetas(n);
    for (size_t i = 0; i < n; i++) {
        xs[i] = solution.states[i].x;
        ys[i] = solution.states[i].y;
        thetas[i] = solution.states[i].theta;
    }
    auto theta_cont = llp::math::ToContinuousAngle(thetas);

    // Compute B-spline control points
    auto cp_x = llp::math::CubicBSplineControlPoints(xs);
    auto cp_y = llp::math::CubicBSplineControlPoints(ys);
    auto cp_theta = llp::math::CubicBSplineControlPoints(theta_cont);

    // Parameter range: data point index [0, n-1]
    const double spline_max = static_cast<double>(n - 1);
    // dt_factor converts time -> spline parameter
    const double dt_factor = spline_max / total_time;

    int n_out = static_cast<int>(std::ceil(total_time / dt)) + 1;
    interpolated_states.reserve(n_out);

    for (int i = 0; i < n_out; i++) {
        double t = std::min(static_cast<double>(i) * dt, total_time);
        double s = t * dt_factor;  // map time to spline parameter [0, spline_max]

        llp::TrajectoryPoint pt;
        pt.x = llp::math::EvaluateCubicBSpline(s, cp_x);
        pt.y = llp::math::EvaluateCubicBSpline(s, cp_y);
        pt.theta = llp::math::NormalizeAngle(llp::math::EvaluateCubicBSpline(s, cp_theta));

        // Derivatives w.r.t. spline parameter; chain-rule to get time derivatives
        double dx_ds = llp::math::EvaluateCubicBSplineDerivative(s, cp_x);
        double dy_ds = llp::math::EvaluateCubicBSplineDerivative(s, cp_y);
        double dtheta_ds = llp::math::EvaluateCubicBSplineDerivative(s, cp_theta);

        double dx_dt = dx_ds * dt_factor;
        double dy_dt = dy_ds * dt_factor;
        double dtheta_dt = dtheta_ds * dt_factor;

        double speed = std::hypot(dx_dt, dy_dt);
        // Determine gear: velocity sign from alignment of heading with motion direction
        double heading_x = std::cos(pt.theta);
        double heading_y = std::sin(pt.theta);
        double v_sign = (heading_x * dx_dt + heading_y * dy_dt >= 0.0) ? 1.0 : -1.0;
        pt.v = v_sign * speed;

        // phi from bicycle model: tan(phi) = L * theta_dot / v
        if (std::abs(pt.v) > 0.01) {
            pt.phi = std::atan2(wheel_base * dtheta_dt, std::abs(pt.v));
        } else {
            pt.phi = 0.0;
        }
        pt.phi = llp::math::Clamp(pt.phi, -phi_max, phi_max);

        interpolated_states.push_back(pt);
    }

    // Compute a and omega by central / forward-backward differences
    for (size_t i = 0; i < interpolated_states.size(); i++) {
        double a_val, omega_val;
        if (i == 0 && interpolated_states.size() > 1) {
            a_val = (interpolated_states[1].v - interpolated_states[0].v) / dt;
            omega_val = (interpolated_states[1].phi - interpolated_states[0].phi) / dt;
        } else if (i == interpolated_states.size() - 1 && interpolated_states.size() > 1) {
            a_val = (interpolated_states[i].v - interpolated_states[i - 1].v) / dt;
            omega_val = (interpolated_states[i].phi - interpolated_states[i - 1].phi) / dt;
        } else if (interpolated_states.size() > 2) {
            a_val = (interpolated_states[i + 1].v - interpolated_states[i - 1].v) / (2.0 * dt);
            omega_val = (interpolated_states[i + 1].phi - interpolated_states[i - 1].phi) / (2.0 * dt);
        } else {
            a_val = 0.0;
            omega_val = 0.0;
        }
        interpolated_states[i].a = llp::math::Clamp(a_val, -max_accel, max_accel);
        interpolated_states[i].omega = llp::math::Clamp(omega_val, -omega_max, omega_max);
    }

    RCLCPP_INFO(_logger, "B-spline interpolated path has %zu states (original: %zu)",
                interpolated_states.size(), solution.states.size());
}

/*
  bool CustomPlanner::liomPlan(const llp::FullStates &prev_sol, 
    const llp::TrajectoryPoint &start, 
    const llp::TrajectoryPoint &goal, 
    llp::FullStates &result) 
  {
    rclcpp::Logger logger = rclcpp::get_logger("liom_local_planner");
    RCLCPP_INFO(logger, "liomPlan started");
    llp::FullStates guess = _liom_local_planner->StitchPreviousSolution(prev_sol, start);
    RCLCPP_INFO(logger, "Guess has %zu states", guess.states.size());
    if(!_liom_local_planner->CheckGuessFeasibility(guess)) {
      RCLCPP_INFO(logger, "Guess not feasible, generating coarse path");
      std::vector<llp::math::Pose> initial_path;
      double st = llp::GetCurrentTimestamp();
      if(!coarsePlan(start.pose(), goal.pose(), initial_path)) {
        RCLCPP_ERROR(logger, "re-plan coarse path failed!");
        return false;
      }
      RCLCPP_INFO(logger, "Coarse path generated with %zu poses", initial_path.size());
      guess = _liom_local_planner->GenerateGuessFromPath(initial_path, start);
      RCLCPP_INFO(logger, "coarse path generation time: %f", llp::GetCurrentTimestamp() - st);

      std::vector<double> xs, ys;
      for(auto &pose: initial_path) {
        xs.push_back(pose.x); ys.push_back(pose.y);
      }

      RCLCPP_INFO(logger, "Calling visualization::Plot...");
      llp::visualization::Plot(xs, ys, 0.1, llp::visualization::Color::Yellow, 1, "Coarse Path");
      RCLCPP_INFO(logger, "Calling visualization::Trigger...");
      llp::visualization::Trigger();
      RCLCPP_INFO(logger, "coarse path had published!---------------");
    }

    llp::Constraints constraints;
    constraints.start = start;
    constraints.goal = goal;

    int disc_nvar = _liom_local_planner->config_->vehicle.n_disc * 2;
    constraints.corridor_lb.setConstant(guess.states.size(), disc_nvar, -inf);
    constraints.corridor_ub.setConstant(guess.states.size(), disc_nvar, inf);
    for(size_t i = 0; i < guess.states.size(); i++) {
      auto disc_pos = _liom_local_planner->config_->vehicle.GetDiscPositions(guess.states[i].x, guess.states[i].y, guess.states[i].theta);

      for(int j = 0; j < _liom_local_planner->config_->vehicle.n_disc; j++) {
        llp::math::AABox2d box;
        //if (!env_->GenerateCorridorBox(0.0, disc_pos[j*2], disc_pos[j*2+1], config_->vehicle.disc_radius, box)) {
        if (!_liom_local_planner->env_->GenerateCorridorBox(0.0, disc_pos[j*2], disc_pos[j*2+1], guess.states[i].theta, _liom_local_planner->config_->vehicle.disc_radius, box)) {
          RCLCPP_ERROR(logger, "%d th corridor box indexed at %zu generation failed!", j, i);
          return false;
        }
        RCLCPP_INFO(logger, "%d th corridor box indexed at %zu generation had generated!", j, i);
        auto color = llp::visualization::Color::Green;
        color.set_alpha(0.1);
        llp::visualization::PlotPolygon(llp::math::Polygon2d(llp::math::Box2d(box)), 0.05, color, i, "Corridor " + std::to_string(j));

        constraints.corridor_lb(i, j*2) = box.min_x();
        constraints.corridor_lb(i, j*2+1) = box.min_y();
        constraints.corridor_ub(i, j*2) = box.max_x();
        constraints.corridor_ub(i, j*2+1) = box.max_y();
      }
    }

    llp::visualization::Trigger();

    RCLCPP_INFO(logger, "All corridor boxs has been pulished!");

    double infeasibility;
    if(!_liom_local_planner->problem_->Solve(_liom_local_planner->config_->opti_w_penalty0, constraints, guess, result, infeasibility)) {
      RCLCPP_ERROR(logger, "solver failed!");
      return false;
    }
    RCLCPP_INFO(logger, "solver successed and result has been generated");
    if(infeasibility > _liom_local_planner->config_->opti_varepsilon_tol) {
      RCLCPP_WARN(logger, "infeasibility = %.6f > %.6f, trajectory may not be feasible", infeasibility, _liom_local_planner->config_->opti_varepsilon_tol);
    }

    return true;
  }

  bool CustomPlanner::coarsePlan(
    const common::math::Pose & start,
    const common::math::Pose & goal,
    std::vector<common::math::Pose> & coarsePath)
  {
    RCLCPP_INFO(_logger, "coarsePlan started from (%.3f, %.3f, %.3f) to (%.3f, %.3f, %.3f)", 
                start.x, start.y, start.theta, goal.x, goal.y, goal.theta);
    
    steady_clock::time_point a = steady_clock::now();
    
    RCLCPP_INFO(_logger, "Already inside locked section");
    
    // Downsample costmap, if required
    nav2_costmap_2d::Costmap2D * costmap = _costmap;
    if (_costmap_downsampler) {
      costmap = _costmap_downsampler->downsample(_downsampling_factor);
      _collision_checker.setCostmap(costmap);
    }

    RCLCPP_INFO(_logger, "Costmap resolution: %.6f, size: %dx%d", 
                costmap->getResolution(), costmap->getSizeInCellsX(), costmap->getSizeInCellsY());

    // Set collision checker and costmap information
    // _collision_checker.setFootprint(
    //   _costmap_ros->getRobotFootprint(),
    //   _costmap_ros->getUseRadius(),
    //   hybrid_Astar::findCircumscribedCost(_costmap_ros));
    _a_star->setCollisionChecker(&_collision_checker);

    RCLCPP_INFO(_logger, "Collision checker set");

    // Set starting point, in A* bin search coordinates
    unsigned int mx, my;
    if (!costmap->worldToMap(start.x, start.y, mx, my)) {
      RCLCPP_ERROR(_logger, "Start pose (%.3f, %.3f) is out of costmap!", start.x, start.y);
      throw std::runtime_error("Start pose is out of costmap!");
    }
    RCLCPP_INFO(_logger, "Start point in grid: (%u, %u)", mx, my);

    double orientation_bin = std::round(start.theta / _angle_bin_size);
    while (orientation_bin < 0.0) {
      orientation_bin += static_cast<float>(_angle_quantizations);
    }
    // This is needed to handle precision issues
    if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
      orientation_bin -= static_cast<float>(_angle_quantizations);
    }
    _a_star->setStart(mx, my, static_cast<unsigned int>(orientation_bin));

    RCLCPP_INFO(_logger, "Start orientation bin: %u", static_cast<unsigned int>(orientation_bin));

    // Set goal point, in A* bin search coordinates
    if (!costmap->worldToMap(goal.x, goal.y, mx, my)) {
      RCLCPP_ERROR(_logger, "Goal pose (%.3f, %.3f) is out of costmap!", goal.x, goal.y);
      throw std::runtime_error("Goal pose is out of costmap!");
    }
    RCLCPP_INFO(_logger, "Goal point in grid: (%u, %u)", mx, my);
    orientation_bin = std::round(goal.theta / _angle_bin_size);
    while (orientation_bin < 0.0) {
      orientation_bin += static_cast<float>(_angle_quantizations);
    }
    // This is needed to handle precision issues
    if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
      orientation_bin -= static_cast<float>(_angle_quantizations);
    }
    _a_star->setGoal(mx, my, static_cast<unsigned int>(orientation_bin));
    RCLCPP_INFO(_logger, "Goal orientation bin: %u", static_cast<unsigned int>(orientation_bin));

    // Compute plan
    hybrid_Astar::NodeHybrid::CoordinateVector path;
    int num_iterations = 0;
    std::string error;

    RCLCPP_INFO(_logger, "Starting A* search...");
    try {
      if (!_a_star->createPath(
          path, num_iterations, _tolerance / static_cast<float>(costmap->getResolution())))
      {
        RCLCPP_WARN(_logger, "Failed to create path.");
        if (num_iterations < _a_star->getMaxIterations()) {
          error = std::string("no valid path found");
        } else {
          error = std::string("exceeded maximum iterations");
        }
      } else {
        RCLCPP_INFO(_logger, "A* search succeeded with %d iterations.", num_iterations);
      }
    } catch (const std::runtime_error & e) {
      error = "invalid use: ";
      error += e.what();
      RCLCPP_ERROR(_logger, "A* search exception: %s", error.c_str());
    }

    if (!error.empty()) {
      RCLCPP_WARN(
        _logger,
        "%s: failed to create plan, %s.",
        _name.c_str(), error.c_str());
      return false;
    }

    common::math::Pose pose;
    // Convert to world coordinates
    coarsePath.reserve(path.size());
    for (int i = path.size() - 1; i >= 0; --i) {
      pose.x = static_cast<float>(costmap->getOriginX()) + (path[i].x + 0.5) * costmap->getResolution();
      pose.y = static_cast<float>(costmap->getOriginY()) + (path[i].y + 0.5) * costmap->getResolution();
      pose.theta = common::math::NormalizeAngle(path[i].theta);
      coarsePath.push_back(pose);
    }

    RCLCPP_INFO(_logger, "coarse path size is %ld", coarsePath.size());
    RCLCPP_INFO(_logger, "Starting path planning from (%f, %f, %f) to (%f, %f, %f)", 
                coarsePath[0].x, coarsePath[0].y, coarsePath[0].theta,
                coarsePath[coarsePath.size() - 1].x, coarsePath[coarsePath.size() - 1].y, coarsePath[coarsePath.size() - 1].theta);
    return true;
  }
    */





  rcl_interfaces::msg::SetParametersResult
  CustomPlanner::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    std::lock_guard<std::mutex> lock_reinit(_mutex);

    bool reinit_collision_checker = false;
    bool reinit_a_star = false;
    bool reinit_downsampler = false;
    bool reinit_smoother = false;

    for (auto parameter : parameters) {
      const auto & type = parameter.get_type();
      const auto & name = parameter.get_name();

      if (type == ParameterType::PARAMETER_DOUBLE) {
        if (name == _name + ".max_planning_time") {
          reinit_a_star = true;
          _max_planning_time = parameter.as_double();
        } else if (name == _name + ".tolerance") {
          _tolerance = static_cast<float>(parameter.as_double());
        } else if (name == _name + ".lookup_table_size") {
          reinit_a_star = true;
          _lookup_table_size = parameter.as_double();
        } else if (name == _name + ".minimum_turning_radius") {
          reinit_a_star = true;
          // if (_smoother) {
          //   reinit_smoother = true;
          // }
          _minimum_turning_radius_global_coords = static_cast<float>(parameter.as_double());
        } else if (name == _name + ".reverse_penalty") {
          reinit_a_star = true;
          _search_info.reverse_penalty = static_cast<float>(parameter.as_double());
        } else if (name == _name + ".change_penalty") {
          reinit_a_star = true;
          _search_info.change_penalty = static_cast<float>(parameter.as_double());
        } else if (name == _name + ".non_straight_penalty") {
          reinit_a_star = true;
          _search_info.non_straight_penalty = static_cast<float>(parameter.as_double());
        } else if (name == _name + ".cost_penalty") {
          reinit_a_star = true;
          _search_info.cost_penalty = static_cast<float>(parameter.as_double());
        } else if (name == _name + ".analytic_expansion_ratio") {
          reinit_a_star = true;
          _search_info.analytic_expansion_ratio = static_cast<float>(parameter.as_double());
        } else if (name == _name + ".analytic_expansion_max_length") {
          reinit_a_star = true;
          _search_info.analytic_expansion_max_length =
            static_cast<float>(parameter.as_double()) / _costmap->getResolution();
        }
      } else if (type == ParameterType::PARAMETER_BOOL) {
        if (name == _name + ".downsample_costmap") {
          reinit_downsampler = true;
          _downsample_costmap = parameter.as_bool();
        } else if (name == _name + ".allow_unknown") {
          reinit_a_star = true;
          _allow_unknown = parameter.as_bool();
        } else if (name == _name + ".cache_obstacle_heuristic") {
          reinit_a_star = true;
          _search_info.cache_obstacle_heuristic = parameter.as_bool();
        } else if (name == _name + ".smooth_path") {
          // if (parameter.as_bool()) {
          //   reinit_smoother = true;
          // } else {
          //   _smoother.reset();
          // }
        }
      } else if (type == ParameterType::PARAMETER_INTEGER) {
        if (name == _name + ".downsampling_factor") {
          reinit_a_star = true;
          reinit_downsampler = true;
          _downsampling_factor = parameter.as_int();
        } else if (name == _name + ".max_iterations") {
          reinit_a_star = true;
          _max_iterations = parameter.as_int();
          if (_max_iterations <= 0) {
            RCLCPP_INFO(
              _logger, "maximum iteration selected as <= 0, "
              "disabling maximum iterations.");
            _max_iterations = std::numeric_limits<int>::max();
          }
        } else if (name == _name + ".max_on_approach_iterations") {
          reinit_a_star = true;
          _max_on_approach_iterations = parameter.as_int();
          if (_max_on_approach_iterations <= 0) {
            RCLCPP_INFO(
              _logger, "On approach iteration selected as <= 0, "
              "disabling tolerance and on approach iterations.");
            _max_on_approach_iterations = std::numeric_limits<int>::max();
          }
        } else if (name == _name + ".angle_quantization_bins") {
          reinit_collision_checker = true;
          reinit_a_star = true;
          int angle_quantizations = parameter.as_int();
          _angle_bin_size = 2.0 * M_PI / angle_quantizations;
          _angle_quantizations = static_cast<unsigned int>(angle_quantizations);
        }
      } else if (type == ParameterType::PARAMETER_STRING) {
        if (name == _name + ".motion_model_for_search") {
          reinit_a_star = true;
          _motion_model = hybrid_Astar::fromString(parameter.as_string());
          if (_motion_model == hybrid_Astar::MotionModel::UNKNOWN) {
            RCLCPP_WARN(
              _logger,
              "Unable to get MotionModel search type. Given '%s', "
              "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP.",
              _motion_model_for_search.c_str());
          }
        }
      }
    }

    // Re-init if needed with mutex lock (to avoid re-init while creating a plan)
    if (reinit_a_star || reinit_downsampler || reinit_collision_checker || reinit_smoother) {
      // convert to grid coordinates
      if (!_downsample_costmap) {
        _downsampling_factor = 1;
      }
      _search_info.minimum_turning_radius =
        _minimum_turning_radius_global_coords / (_costmap->getResolution() * _downsampling_factor);
      _lookup_table_dim =
        static_cast<float>(_lookup_table_size) /
        static_cast<float>(_costmap->getResolution() * _downsampling_factor);

      // Make sure its a whole number
      _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

      // Make sure its an odd number
      if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
        RCLCPP_INFO(
          _logger,
          "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
          _lookup_table_dim);
        _lookup_table_dim += 1.0;
      }

      auto node = _node.lock();

      // Re-Initialize A* template
      if (reinit_a_star) {
        _a_star = std::make_unique<hybrid_Astar::AStarAlgorithm<hybrid_Astar::NodeHybrid>>(_motion_model, _search_info);
        _a_star->initialize(
          _allow_unknown,
          _max_iterations,
          _max_on_approach_iterations,
          _max_planning_time,
          _lookup_table_dim,
          _angle_quantizations);
      }

      // Re-Initialize costmap downsampler
      if (reinit_downsampler) {
        if (_downsample_costmap && _downsampling_factor > 1) {
          std::string topic_name = "downsampled_costmap";
          _costmap_downsampler = std::make_unique<hybrid_Astar::CostmapDownsampler>();
          _costmap_downsampler->on_configure(
            node, _global_frame, topic_name, _costmap, _downsampling_factor);
        }
      }

      // Re-Initialize collision checker
      if (reinit_collision_checker) {
        _collision_checker = hybrid_Astar::GridCollisionChecker(_costmap, _angle_quantizations, node);
        // _collision_checker.setFootprint(
        //   _costmap_ros->getRobotFootprint(),
        //   _costmap_ros->getUseRadius(),
        //   hybrid_Astar::findCircumscribedCost(_costmap_ros));
      }

      // Re-Initialize smoother
      // if (reinit_smoother) {
      //   hybrid_Astar::SmootherParams params;
      //   params.get(node, _name);
      //   _smoother = std::make_unique<hybrid_Astar::Smoother>(params);
      //   _smoother->initialize(_minimum_turning_radius_global_coords);
      // }
    }
    result.successful = true;
    return result;
  }

  }  // namespace planner

  #include "pluginlib/class_list_macros.hpp"
  PLUGINLIB_EXPORT_CLASS(planner::CustomPlanner, nav2_core::GlobalPlanner)