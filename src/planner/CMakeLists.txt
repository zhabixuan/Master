cmake_minimum_required(VERSION 3.8)
project(planner)

# ========== 关键修复1：显式指定 C++ 标准 ==========
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========== 关键修复2：指定 Release 编译类型 ==========
set(CMAKE_BUILD_TYPE Release)

# 依赖查找
find_package(ament_cmake REQUIRED)
find_package(path_searching REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(liom_local_planner REQUIRED)

# ========== 关键修复3：移除 Eigen3 从 ament 依赖列表 ==========
set(dependencies
  path_searching
  nav2_msgs
  nav2_core
  nav2_costmap_2d
  geometry_msgs
  nav2_util
  tf2
  liom_local_planner
)

# 编译库
add_library(planners SHARED
  src/custom_planner.cpp
)

# ========== 简化：移除冗余的 C 标准要求 ==========
target_compile_features(planners PUBLIC cxx_std_17)

# ========== 关键修复4：添加 Eigen3 头文件路径 ==========
target_include_directories(planners PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}  # 新增
)

# ========== 关键修复5：正确链接 Eigen3（头文件库） ==========
target_link_libraries(planners Eigen3::Eigen)

ament_target_dependencies(
  planners
  ${dependencies}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(planners PRIVATE "PLANNER_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(nav2_core planner_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS planners
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  FILES planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY params launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  planners
)
#ament_export_targets(
#  export_${PROJECT_NAME}
#)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)  # 补充 HAS_LIBRARY_TARGET（规范）
ament_export_dependencies(${dependencies})  # 新增：导出 ament 依赖列表

ament_package()