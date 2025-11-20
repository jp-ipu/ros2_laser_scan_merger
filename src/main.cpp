//
// ROS2 Laser Scan Merger - Entry Point
//
// Created by: Michael Jonathan (mich1342)
// github.com/mich1342
// 24/2/2022
//
// Modified to support N laser scans (dynamic number of inputs)
// Refactored for TF2-only approach with cached transforms for performance
// Optimized based on ira_laser_tools analysis
// Refactored for testability: core algorithms extracted to separate classes
// Removed PCL dependency: uses ROS2 sensor_msgs directly (zero external dependencies)
//

#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "laser_scan_merger/scan_merger_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<laser_scan_merger::ScanMerger>());
  rclcpp::shutdown();
  return 0;
}
