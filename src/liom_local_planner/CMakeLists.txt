cmake_minimum_required(VERSION 3.8)
project(liom_local_planner)

# 强制C++标准（ROS2推荐显式指定）
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 编译类型（Release）
set(CMAKE_BUILD_TYPE Release)

# 预处理器宏定义（与原逻辑一致）
add_definitions(-DHAVE_CSTDDEF -D_USE_MATH_DEFINES)

# ========== 依赖查找 ==========
# ROS2 核心依赖（替换catkin）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)  # ROS2中costmap迁移到nav2栈
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)        # 替代ROS1的nav_core
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)        # 插件机制兼容，但需适配ROS2接口
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)  # ROS2需确保该包已安装（ros-humble-interactive-markers）

# 系统依赖（OMPL）
find_package(ompl REQUIRED)

# 可选：若使用IPOPT/ADOLC，需确保系统已安装并查找
find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)
pkg_check_modules(ADOLC REQUIRED adolc)

# ========== 库编译 ==========
# 声明C++库（替代catkin的add_library）
add_library(${PROJECT_NAME} SHARED
  src/liom_local_planner_ros.cpp
  src/lightweight_nlp_problem.cpp
  src/math/aabox2d.cpp
  src/math/box2d.cpp
  src/math/line_segment2d.cpp
  src/math/math_utils.cpp
  src/math/polygon2d.cpp
  src/math/vec2d.cpp
  src/visualization/color.cpp
  src/visualization/plot.cpp
  src/liom_local_planner.cpp
  src/coarse_path_planner.cpp
  src/environment.cpp
)

# 链接依赖（ROS2使用ament_target_dependencies替代catkin_LIBRARIES）
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav2_costmap_2d
  geometry_msgs
  nav2_core
  nav_msgs
  std_msgs
  pluginlib
  tf2
  tf2_ros
  visualization_msgs
  interactive_markers
)

# 链接系统库（OMPL/IPOPT/ADOLC）
target_link_libraries(${PROJECT_NAME}
  ${OMPL_LIBRARIES}
  ${IPOPT_LIBRARIES}
  ${ADOLC_LIBRARIES}
)

# 头文件路径
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OMPL_INCLUDE_DIRS}
    ${IPOPT_INCLUDE_DIRS}
    ${ADOLC_INCLUDE_DIRS}
)

add_executable(liom_test_node src/liom_test_node.cpp)
target_link_libraries(liom_test_node ${PROJECT_NAME})


# ========== 安装规则 ==========
# 安装库
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS liom_test_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY maps launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# 安装可执行文件
#install(TARGETS liom_test_node
#  RUNTIME DESTINATION lib/${PROJECT_NAME}
#)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 导出依赖（供其他包使用）
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  nav2_costmap_2d
  geometry_msgs
  nav2_core
  nav_msgs
  std_msgs
  pluginlib
  tf2
  tf2_ros
  visualization_msgs
  interactive_markers
  ompl
)

# ========== 测试（可选） ==========
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 排除不需要的lint检查（如copyright，若未配置）
  ament_lint_auto_find_test_dependencies()
  # 可选：添加gtest（ROS2版）
  # find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(${PROJECT_NAME}-test test/test_liom_local_planner.cpp)
  # target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

# 生成ament包配置
ament_package()