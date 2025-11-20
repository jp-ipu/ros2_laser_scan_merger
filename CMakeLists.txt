cmake_minimum_required(VERSION 3.8)
project(ros2_laser_scan_merger)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
)

# =============================
# Configure
# =============================

set(package_dependencies
    rclcpp
    sensor_msgs
    std_msgs
    geometry_msgs
    rcl_interfaces
    rcutils
    tf2
    tf2_ros
    tf2_geometry_msgs
)

find_package(ament_cmake REQUIRED)

foreach (dependency IN LISTS package_dependencies)
    find_package(${dependency} REQUIRED)
endforeach ()

# =============================
# Core Library (Testable, Zero ROS2 Dependencies)
# =============================

add_library(laser_scan_merger_core
    src/math_utils.cpp
    src/scan_processor.cpp
)

target_include_directories(laser_scan_merger_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# =============================
# ROS2 Node Executable
# =============================

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/scan_merger_node.cpp
)

target_link_libraries(${PROJECT_NAME}
    laser_scan_merger_core
)

ament_target_dependencies(${PROJECT_NAME}
    ${package_dependencies}
)

# =============================
# Install
# =============================

install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

# =============================
# Testing
# =============================

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)

    # Linting
    # Skip copyright check - uncomment if no copyright in all files
    # set(ament_cmake_copyright_FOUND TRUE)
    # Skip cpplint - uncomment if not in a git repo
    # set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()

    # Unit tests for core library (no ROS2 dependencies)
    ament_add_gtest(test_math_utils
        test/test_math_utils.cpp
    )
    target_link_libraries(test_math_utils
        laser_scan_merger_core
    )

    ament_add_gtest(test_scan_processor
        test/test_scan_processor.cpp
    )
    target_link_libraries(test_scan_processor
        laser_scan_merger_core
    )
endif ()

# =============================
# Package Export
# =============================

ament_package()
