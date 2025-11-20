//
// ROS2 Laser Scan Merger Node
//
// Created by: Michael Jonathan (mich1342)
// Modified for modern C++20, TF2 caching, and testable architecture
//

#ifndef LASER_SCAN_MERGER_SCAN_MERGER_NODE_HPP_
#define LASER_SCAN_MERGER_SCAN_MERGER_NODE_HPP_

#include "laser_scan_merger/math_utils.hpp"
#include "laser_scan_merger/scan_processor.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace laser_scan_merger {

struct LaserConfig {
  // Topic configuration
  std::string topic;
  std::string frame_id;

  // Angle filtering (degrees)
  float angle_min{-181.0F};
  float angle_max{181.0F};

  // Visualization color (RGB 0-255)
  uint8_t r{255};
  uint8_t g{0};
  uint8_t b{0};

  // Processing flags
  bool show{true};      // Enable/disable this laser
  bool flip{false};     // Flip the scan data
  bool inverse{false};  // Inverse the angle filtering logic

  // Runtime state
  sensor_msgs::msg::LaserScan::SharedPtr last_scan;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber;
  rclcpp::Time last_update_time;
  bool data_received{false};  // For synchronization tracking

  // Cached transform (when use_fixed_transforms is enabled)
  math::Transform3D cached_transform;
};

class ScanMerger : public rclcpp::Node {
 public:
  ScanMerger();

 private:
  // Configuration parameters
  std::string cloud_topic_;
  std::string cloud_frame_id_;
  double publish_rate_{30.0};

  // Synchronization settings
  bool require_all_scans_{false};
  bool use_scan_triggering_{false};
  double max_scan_age_{1.0};
  bool skip_stale_scans_{false};

  // Transform settings
  bool use_fixed_transforms_{true};
  double tf_timeout_{1.0};

  // Global laser parameters (shared defaults)
  float global_angle_min_{-181.0F};
  float global_angle_max_{181.0F};
  bool global_flip_{false};
  bool global_inverse_{false};

  // Laser configuration and state
  std::vector<LaserConfig> lasers_;
  std::mutex lasers_mutex_;

  // ROS2 publishers and timers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // TF2 components
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Core processor (testable, zero ROS2 dependencies)
  LaserScanProcessor scan_processor_;

  // Initialization methods
  void initialize_params();
  void refresh_params();
  void setup_subscribers();

  // Parameter loading
  void load_laser_params(int laser_index);

  // Transform management
  bool cache_transform(size_t laser_idx);
  bool lookup_transform(size_t laser_idx, math::Transform3D& transform);

  // Callback handlers
  void scan_callback(size_t laser_index, sensor_msgs::msg::LaserScan::SharedPtr msg);

  // Publishing and processing
  void publish_merged_cloud();
  void process_laser_scan(const LaserConfig& laser, size_t laser_idx,
                          std::vector<ColoredPoint>& all_points);

};

}  // namespace laser_scan_merger

#endif  // LASER_SCAN_MERGER_SCAN_MERGER_NODE_HPP_
