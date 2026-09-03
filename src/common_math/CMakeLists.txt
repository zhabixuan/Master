cmake_minimum_required(VERSION 3.5)
project(common_math)

set(CMAKE_BUILD_TYPE Release) #Debug, Release

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Boost REQUIRED)

# 可选：若使用IPOPT/ADOLC，需确保系统已安装并查找
find_package(PkgConfig REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
else()
  add_compile_options(-O3 -Wextra -Wdeprecated -fPIC)
endif()

include_directories(
  include
)


set(library_name common_math)

set(dependencies
  builtin_interfaces
  Boost
)

# Hybrid plugin
add_library(${library_name} SHARED
  src/aabox2d.cpp
  src/box2d.cpp
  src/line_segment2d.cpp
  src/math_utils.cpp
  src/polygon2d.cpp
  src/vec2d.cpp
)

target_link_libraries(${library_name}
  Boost::boost)
target_include_directories(${library_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

#install(DIRECTORY lattice_primitives/sample_primitives DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #set(AMENT_LINT_AUTO_FILE_EXCLUDE include/hybrid_Astar/thirdparty/robin_hood.h)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  #add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
