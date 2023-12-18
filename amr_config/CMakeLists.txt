cmake_minimum_required(VERSION 3.8)
project(amr_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

set(dependencies
ament_cmake
rclcpp
geometry_msgs
nav_msgs
tf2
)

# Add include directories
include_directories(
  include
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# install(DIRECTORY include/
#     DESTINATION include
# )

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

ament_package()

add_executable(amr_control src/amr_control.cpp)
ament_target_dependencies(amr_control ${dependencies})

add_executable(amr_odom src/amr_odom.cpp)
ament_target_dependencies(amr_odom ${dependencies})

install(TARGETS
  amr_control
  amr_odom
  DESTINATION lib/${PROJECT_NAME})
