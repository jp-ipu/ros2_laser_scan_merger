cmake_minimum_required(VERSION 3.5)
project(ros2_laser_scan_merger)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcutils REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
)

# Build testable core library (no ROS2 dependencies)
add_library(laser_scan_merger_core
  src/math_utils.cpp
  src/scan_processor.cpp
)
target_include_directories(laser_scan_merger_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Build ROS2 node executable
add_executable(ros2_laser_scan_merger src/main.cpp)
target_link_libraries(ros2_laser_scan_merger laser_scan_merger_core)
ament_target_dependencies(ros2_laser_scan_merger
  rclcpp
  std_msgs
  sensor_msgs
  rcl_interfaces
  rcutils
  tf2_geometry_msgs
  geometry_msgs
  tf2
  tf2_ros
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)


install(TARGETS
ros2_laser_scan_merger
DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Unit tests for core library (no ROS2 dependencies required)
  ament_add_gtest(test_math_utils test/test_math_utils.cpp)
  target_link_libraries(test_math_utils laser_scan_merger_core)

  ament_add_gtest(test_scan_processor test/test_scan_processor.cpp)
  target_link_libraries(test_scan_processor laser_scan_merger_core)
endif()

ament_package()
