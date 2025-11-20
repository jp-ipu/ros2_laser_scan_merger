//
//   created by: Michael Jonathan (mich1342)
//   github.com/mich1342
//   24/2/2022
//
//   Modified to support N laser scans (dynamic number of inputs)
//   Refactored for TF2-only approach with cached transforms for performance
//   Optimized based on ira_laser_tools analysis
//   Refactored for testability: core algorithms extracted to separate classes
//

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "laser_scan_merger/math_utils.hpp"
#include "laser_scan_merger/scan_processor.hpp"

namespace laser_scan_merger {

// Configuration for each laser scanner (ROS2-specific parts)
struct LaserConfig {
  std::string topic;
  // Note: source_frame is read from scan->header.frame_id, not configured
  std::string detected_frame_id;  // Frame ID detected from scan header
  float angle_min{-181.0f};  // Minimum angle to include (degrees)
  float angle_max{181.0f};   // Maximum angle to include (degrees)
  uint8_t r{255};
  uint8_t g{0};
  uint8_t b{0};
  bool show{true};     // Enable/disable this laser
  bool flip{false};    // Flip the scan data
  bool inverse{false};  // Inverse the angle filtering logic

  sensor_msgs::msg::LaserScan::SharedPtr last_scan;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber;
  rclcpp::Time last_update_time;
  bool data_received{false};  // For synchronization

  // Cached transform (for use_fixed_transforms mode)
  math::Transform3D cached_transform;  // Using testable type from math_utils
};

class ScanMerger : public rclcpp::Node {
 public:
  ScanMerger()
      : Node("ros2_laser_scan_merger"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    InitializeParams();
    RefreshParams();
    SetupSubscribers();

    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, rclcpp::SensorDataQoS());

    // Create timer for publishing
    if (!use_scan_triggering_) {
      publish_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
          std::bind(&ScanMerger::PublishMergedCloud, this));
    }

    RCLCPP_INFO(this->get_logger(),
                "Laser Scan Merger initialized with %zu laser(s)",
                lasers_.size());
    RCLCPP_INFO(this->get_logger(), "  Target frame: %s",
                cloud_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Fixed transforms: %s (TF caching %s)",
                use_fixed_transforms_ ? "true" : "false",
                use_fixed_transforms_ ? "ENABLED" : "disabled");
    RCLCPP_INFO(this->get_logger(), "  Scan synchronization: %s",
                require_all_scans_ ? "enabled (wait for all)" : "disabled");
    RCLCPP_INFO(this->get_logger(), "  Publishing mode: %s",
                use_scan_triggering_ ? "scan-triggered" : "timer-based");
    RCLCPP_INFO(this->get_logger(), "  Max scan age: %.2f seconds",
                max_scan_age_);

    for (size_t i = 0; i < lasers_.size(); i++) {
      RCLCPP_INFO(this->get_logger(),
                  "  Laser %zu: topic=%s, enabled=%s", i,
                  lasers_[i].topic.c_str(),
                  lasers_[i].show ? "true" : "false");
    }

    // Note: TF transforms will be cached on first scan reception if use_fixed_transforms is true
    if (use_fixed_transforms_) {
      RCLCPP_INFO(this->get_logger(),
                  "Fixed transform mode: will cache TF on first scan from each laser");
    }
  }

 private:
  void SetupSubscribers() {
    auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

    for (size_t i = 0; i < lasers_.size(); i++) {
      lasers_[i].last_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
      lasers_[i].subscriber =
          this->create_subscription<sensor_msgs::msg::LaserScan>(
              lasers_[i].topic, default_qos,
              [this, i](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                this->ScanCallback(i, msg);
              });
    }
  }

  bool CacheTransform(size_t laser_idx) {
    auto& laser = lasers_[laser_idx];

    if (laser.detected_frame_id.empty()) {
      return false;  // Frame not yet detected from scan
    }

    try {
      auto tf_msg = tf_buffer_.lookupTransform(
          cloud_frame_id_, laser.detected_frame_id, tf2::TimePointZero,
          tf2::durationFromSec(tf_timeout_));

      // Use testable math function to convert quaternion to transform
      laser.cached_transform = math::QuaternionToTransform(
          tf_msg.transform.rotation.x,
          tf_msg.transform.rotation.y,
          tf_msg.transform.rotation.z,
          tf_msg.transform.rotation.w,
          tf_msg.transform.translation.x,
          tf_msg.transform.translation.y,
          tf_msg.transform.translation.z);

      return true;

    } catch (const tf2::TransformException& ex) {
      laser.cached_transform.valid = false;
      return false;
    }
  }

  void ScanCallback(size_t laser_index,
                    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (laser_index >= lasers_.size()) {
      return;
    }

    std::lock_guard<std::mutex> lock(lasers_mutex_);
    auto& laser = lasers_[laser_index];

    // Detect frame_id from scan header on first reception
    if (laser.detected_frame_id.empty()) {
      laser.detected_frame_id = msg->header.frame_id;
      RCLCPP_INFO(this->get_logger(),
                  "Laser %zu: detected frame_id '%s' from scan header",
                  laser_index, laser.detected_frame_id.c_str());

      // Cache transform if using fixed mode
      if (use_fixed_transforms_ && laser.show) {
        if (CacheTransform(laser_index)) {
          RCLCPP_INFO(this->get_logger(),
                      "  ✓ Cached transform: %s -> %s",
                      laser.detected_frame_id.c_str(), cloud_frame_id_.c_str());
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "  ✗ Failed to cache transform: %s -> %s (will use dynamic lookup)",
                      laser.detected_frame_id.c_str(), cloud_frame_id_.c_str());
        }
      }
    }

    laser.last_scan = msg;
    laser.last_update_time = this->now();
    laser.data_received = true;

    // If using scan-triggered mode, check if we should publish
    if (use_scan_triggering_) {
      if (require_all_scans_) {
        // Check if all enabled lasers have received data
        bool all_received = true;
        for (const auto& laser : lasers_) {
          if (laser.show && !laser.data_received) {
            all_received = false;
            break;
          }
        }

        if (all_received) {
          PublishMergedCloud();
          // Reset flags for next cycle
          for (auto& laser : lasers_) {
            laser.data_received = false;
          }
        }
      } else {
        // Publish whenever any scan arrives
        PublishMergedCloud();
      }
    }
  }

  void PublishMergedCloud() {
    std::lock_guard<std::mutex> lock(lasers_mutex_);

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    rclcpp::Time latest_timestamp = this->now();
    rclcpp::Time oldest_timestamp = this->now();
    bool has_valid_scan = false;
    int stale_scan_count = 0;

    // Process each laser scanner
    for (size_t laser_idx = 0; laser_idx < lasers_.size(); laser_idx++) {
      const auto& laser = lasers_[laser_idx];

      if (!laser.show || !laser.last_scan ||
          laser.last_scan->ranges.empty()) {
        continue;
      }

      // Check scan age
      rclcpp::Duration scan_age = this->now() - laser.last_update_time;
      if (scan_age.seconds() > max_scan_age_) {
        stale_scan_count++;
        if (skip_stale_scans_) {
          RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 1000,
              "Skipping stale scan from laser %zu (age: %.3f s)", laser_idx,
              scan_age.seconds());
          continue;
        }
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Using stale scan from laser %zu (age: %.3f s)", laser_idx,
            scan_age.seconds());
      }

      has_valid_scan = true;

      // Track timestamps
      rclcpp::Time scan_time(laser.last_scan->header.stamp);
      if (scan_time > latest_timestamp) {
        latest_timestamp = scan_time;
      }
      if (!has_valid_scan || scan_time < oldest_timestamp) {
        oldest_timestamp = scan_time;
      }

      // Process scan with TF (cached or dynamic)
      ProcessLaserScan(laser, laser_idx, cloud);
    }

    if (!has_valid_scan) {
      return;  // No data to publish
    }

    // Warn if scans have large timestamp spread
    if (require_all_scans_) {
      rclcpp::Duration timestamp_spread = latest_timestamp - oldest_timestamp;
      if (timestamp_spread.seconds() > 0.1) {  // 100ms threshold
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Large timestamp spread in merged scans: %.3f seconds",
                             timestamp_spread.seconds());
      }
    }

    // Publish the merged point cloud
    auto pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud, *pc2_msg);
    pc2_msg->header.frame_id = cloud_frame_id_;
    pc2_msg->header.stamp = latest_timestamp;
    pc2_msg->is_dense = false;
    point_cloud_pub_->publish(*pc2_msg);

    if (stale_scan_count > 0) {
      RCLCPP_DEBUG(this->get_logger(), "Published with %d stale scans",
                   stale_scan_count);
    }
  }

  void ProcessLaserScan(const LaserConfig& laser, size_t laser_idx,
                        pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
    const auto& scan = laser.last_scan;

    // Get transform (cached or lookup)
    math::Transform3D transform;

    if (use_fixed_transforms_ && laser.cached_transform.valid) {
      // Use cached transform (fast path)
      transform = laser.cached_transform;
    } else {
      // Dynamic TF lookup
      if (!LookupTransform(laser_idx, transform)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "TF lookup failed for laser %zu (%s -> %s), skipping scan",
            laser_idx, laser.detected_frame_id.c_str(), cloud_frame_id_.c_str());
        return;
      }
    }

    // Convert ROS2 LaserScan to testable ScanData
    ScanData scan_data;
    scan_data.ranges = scan->ranges;
    scan_data.angle_min = scan->angle_min;
    scan_data.angle_max = scan->angle_max;
    scan_data.angle_increment = scan->angle_increment;

    // Configure processing
    ScanProcessingConfig config;
    config.angle_min_deg = laser.angle_min;
    config.angle_max_deg = laser.angle_max;
    config.flip = laser.flip;
    config.inverse = laser.inverse;
    config.r = laser.r;
    config.g = laser.g;
    config.b = laser.b;

    // Process scan using testable core algorithm
    std::vector<ColoredPoint> points = scan_processor_.ProcessScan(scan_data, transform, config);

    // Convert to PCL format
    for (const auto& point : points) {
      pcl::PointXYZRGB pcl_point;
      pcl_point.x = point.x;
      pcl_point.y = point.y;
      pcl_point.z = point.z;
      pcl_point.r = point.r;
      pcl_point.g = point.g;
      pcl_point.b = point.b;
      cloud.points.push_back(pcl_point);
    }
  }

  bool LookupTransform(size_t laser_idx, math::Transform3D& transform) {
    const auto& laser = lasers_[laser_idx];

    if (laser.detected_frame_id.empty()) {
      return false;  // Frame not yet detected from scan
    }

    try {
      auto tf_msg = tf_buffer_.lookupTransform(
          cloud_frame_id_, laser.detected_frame_id, tf2::TimePointZero,
          tf2::durationFromSec(tf_timeout_));

      // Use testable math function to convert quaternion to transform
      transform = math::QuaternionToTransform(
          tf_msg.transform.rotation.x,
          tf_msg.transform.rotation.y,
          tf_msg.transform.rotation.z,
          tf_msg.transform.rotation.w,
          tf_msg.transform.translation.x,
          tf_msg.transform.translation.y,
          tf_msg.transform.translation.z);

      return true;

    } catch (const tf2::TransformException& ex) {
      transform.valid = false;
      return false;
    }
  }

  void InitializeParams() {
    this->declare_parameter("pointCloudTopic", "cloud_in");
    this->declare_parameter("destination_frame", "laser");
    this->declare_parameter("num_lasers", 2);
    this->declare_parameter("publish_rate", 30.0);

    // Synchronization parameters
    this->declare_parameter("require_all_scans", false);
    this->declare_parameter("use_scan_triggering", false);
    this->declare_parameter("max_scan_age", 1.0);
    this->declare_parameter("skip_stale_scans", false);

    // TF parameters
    this->declare_parameter("use_fixed_transforms", true);
    this->declare_parameter("tf_timeout", 1.0);

    // Global laser parameters (shared by all lasers of the same type)
    this->declare_parameter("angle_min", -181.0);
    this->declare_parameter("angle_max", 181.0);
    this->declare_parameter("flip", false);
    this->declare_parameter("inverse", false);
  }

  void RefreshParams() {
    cloud_topic_ = this->get_parameter("pointCloudTopic").as_string();
    cloud_frame_id_ = this->get_parameter("destination_frame").as_string();
    int num_lasers = this->get_parameter("num_lasers").as_int();
    publish_rate_ = this->get_parameter("publish_rate").as_double();

    require_all_scans_ = this->get_parameter("require_all_scans").as_bool();
    use_scan_triggering_ = this->get_parameter("use_scan_triggering").as_bool();
    max_scan_age_ = this->get_parameter("max_scan_age").as_double();
    skip_stale_scans_ = this->get_parameter("skip_stale_scans").as_bool();

    use_fixed_transforms_ = this->get_parameter("use_fixed_transforms").as_bool();
    tf_timeout_ = this->get_parameter("tf_timeout").as_double();

    // Global laser parameters (shared by all lasers of the same type)
    global_angle_min_ = static_cast<float>(
        this->get_parameter("angle_min").as_double());
    global_angle_max_ = static_cast<float>(
        this->get_parameter("angle_max").as_double());
    global_flip_ = this->get_parameter("flip").as_bool();
    global_inverse_ = this->get_parameter("inverse").as_bool();

    // Resize laser vector if needed
    if (lasers_.size() != static_cast<size_t>(num_lasers)) {
      lasers_.resize(num_lasers);
    }

    // Load parameters for each laser
    for (int i = 0; i < num_lasers; i++) {
      LoadLaserParams(i);
    }
  }

  void LoadLaserParams(int laser_index) {
    std::string prefix = "laser" + std::to_string(laser_index);

    // Declare per-laser parameters if not already declared
    // Note: angle_min, angle_max, flip, inverse are now global parameters
    // Note: source_frame is detected from scan->header.frame_id, not configured
    if (!this->has_parameter(prefix + ".topic")) {
      this->declare_parameter(prefix + ".topic",
                              "/scan_" + std::to_string(laser_index));
      this->declare_parameter(prefix + ".r", 255);
      this->declare_parameter(prefix + ".g", 0);
      this->declare_parameter(prefix + ".b", 0);
      this->declare_parameter(prefix + ".show", true);
    }

    // Get per-laser parameters
    auto& laser = lasers_[laser_index];
    laser.topic = this->get_parameter(prefix + ".topic").as_string();

    // Use global parameters (shared by all lasers of the same type)
    laser.angle_min = global_angle_min_;
    laser.angle_max = global_angle_max_;
    laser.flip = global_flip_;
    laser.inverse = global_inverse_;

    int r_val = this->get_parameter(prefix + ".r").as_int();
    int g_val = this->get_parameter(prefix + ".g").as_int();
    int b_val = this->get_parameter(prefix + ".b").as_int();
    laser.r = static_cast<uint8_t>(r_val);
    laser.g = static_cast<uint8_t>(g_val);
    laser.b = static_cast<uint8_t>(b_val);

    laser.show = this->get_parameter(prefix + ".show").as_bool();
  }

  // Configuration
  std::string cloud_topic_;
  std::string cloud_frame_id_;
  double publish_rate_{30.0};
  bool require_all_scans_{false};
  bool use_scan_triggering_{false};
  double max_scan_age_{1.0};
  bool skip_stale_scans_{false};
  bool use_fixed_transforms_{true};
  double tf_timeout_{1.0};

  // Global laser parameters (shared by all lasers of the same type)
  float global_angle_min_{-181.0};
  float global_angle_max_{181.0};
  bool global_flip_{false};
  bool global_inverse_{false};

  // Laser configuration
  std::vector<LaserConfig> lasers_;
  std::mutex lasers_mutex_;

  // Publishers and timers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // TF2
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Core processor (testable, no ROS2 dependencies)
  LaserScanProcessor scan_processor_;
};

}  // namespace laser_scan_merger

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<laser_scan_merger::ScanMerger>());
  rclcpp::shutdown();
  return 0;
}
