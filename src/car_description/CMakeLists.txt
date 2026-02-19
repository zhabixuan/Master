cmake_minimum_required(VERSION 3.8)
project(car_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 必须找到ament_cmake（和package.xml的buildtool_depend对应）
find_package(ament_cmake REQUIRED)

# 3. 安装URDF和launch文件夹（你的原代码已有，保留）
install(DIRECTORY launch urdf
    DESTINATION share/${PROJECT_NAME}
)

# 4. 测试相关（保留，可选）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 5. 关键！生成local_setup.bash等环境文件（必须加）
ament_package()