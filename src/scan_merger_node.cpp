//
// ROS2 Laser Scan Merger Node - Implementation
//
// Created by: Michael Jonathan (mich1342)
// Modified for modern C++20, TF2 caching, and testable architecture
//

#include "laser_scan_merger/scan_merger_node.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <functional>

namespace laser_scan_merger {

// ============================================================================
// Constructor
// ============================================================================

ScanMerger::ScanMerger()
    : Node("ros2_laser_scan_merger"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_) {
  initialize_params();
  refresh_params();
  setup_subscribers();

  point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS());

  // Create timer for publishing (if not using scan-triggered mode)
  if (!use_scan_triggering_) {
    const auto period_ms = static_cast<int>(1000.0 / publish_rate_);
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&ScanMerger::publish_merged_cloud, this));
  }

  // Log initialization info
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

  for (size_t i = 0; i < lasers_.size(); ++i) {
    RCLCPP_INFO(this->get_logger(), "  Laser %zu: topic=%s, enabled=%s", i,
                lasers_[i].topic.c_str(),
                lasers_[i].show ? "true" : "false");
  }

  if (use_fixed_transforms_) {
    RCLCPP_INFO(
        this->get_logger(),
        "Fixed transform mode: will cache TF on first scan from each laser");
  }
}

// ============================================================================
// Initialization Methods
// ============================================================================

void ScanMerger::initialize_params() {
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

void ScanMerger::refresh_params() {
  cloud_topic_ = this->get_parameter("pointCloudTopic").as_string();
  cloud_frame_id_ = this->get_parameter("destination_frame").as_string();
  const int num_lasers = static_cast<int>(this->get_parameter("num_lasers").as_int());
  publish_rate_ = this->get_parameter("publish_rate").as_double();

  require_all_scans_ = this->get_parameter("require_all_scans").as_bool();
  use_scan_triggering_ = this->get_parameter("use_scan_triggering").as_bool();
  max_scan_age_ = this->get_parameter("max_scan_age").as_double();
  skip_stale_scans_ = this->get_parameter("skip_stale_scans").as_bool();

  use_fixed_transforms_ = this->get_parameter("use_fixed_transforms").as_bool();
  tf_timeout_ = this->get_parameter("tf_timeout").as_double();

  // Global laser parameters (shared by all lasers of the same type)
  global_angle_min_ =
      static_cast<float>(this->get_parameter("angle_min").as_double());
  global_angle_max_ =
      static_cast<float>(this->get_parameter("angle_max").as_double());
  global_flip_ = this->get_parameter("flip").as_bool();
  global_inverse_ = this->get_parameter("inverse").as_bool();

  // Resize laser vector if needed
  if (lasers_.size() != static_cast<size_t>(num_lasers)) {
    lasers_.resize(num_lasers);
  }

  // Load parameters for each laser
  for (int i = 0; i < num_lasers; ++i) {
    load_laser_params(i);
  }
}

void ScanMerger::setup_subscribers() {
  const auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

  for (size_t i = 0; i < lasers_.size(); ++i) {
    lasers_[i].last_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    lasers_[i].subscriber =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            lasers_[i].topic, default_qos,
            [this, i](sensor_msgs::msg::LaserScan::SharedPtr msg) {
              this->scan_callback(i, msg);
            });
  }
}

// ============================================================================
// Parameter Loading
// ============================================================================

void ScanMerger::load_laser_params(const int laser_index) {
  const std::string prefix = "laser" + std::to_string(laser_index);

  // Declare per-laser parameters if not already declared
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

  // Per-laser angle filtering (with fallback to global parameters)
  if (this->has_parameter(prefix + ".angle_min")) {
    laser.angle_min =
        static_cast<float>(this->get_parameter(prefix + ".angle_min").as_double());
  } else {
    laser.angle_min = global_angle_min_;
  }

  if (this->has_parameter(prefix + ".angle_max")) {
    laser.angle_max =
        static_cast<float>(this->get_parameter(prefix + ".angle_max").as_double());
  } else {
    laser.angle_max = global_angle_max_;
  }

  // flip and inverse use global parameters (shared by all lasers)
  laser.flip = global_flip_;
  laser.inverse = global_inverse_;

  // Color parameters
  const int r_val = static_cast<int>(this->get_parameter(prefix + ".r").as_int());
  const int g_val = static_cast<int>(this->get_parameter(prefix + ".g").as_int());
  const int b_val = static_cast<int>(this->get_parameter(prefix + ".b").as_int());
  laser.r = static_cast<uint8_t>(r_val);
  laser.g = static_cast<uint8_t>(g_val);
  laser.b = static_cast<uint8_t>(b_val);

  laser.show = this->get_parameter(prefix + ".show").as_bool();
}

// ============================================================================
// Transform Management
// ============================================================================

bool ScanMerger::cache_transform(const size_t laser_idx) {
  auto& laser = lasers_[laser_idx];

  if (laser.frame_id.empty()) {
    return false;  // Frame not yet detected from scan
  }

  try {
    const auto tf_msg = tf_buffer_.lookupTransform(
        cloud_frame_id_, laser.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_));

    // Convert quaternion to transform using testable math function
    laser.cached_transform = math::quaternion_to_transform(
        static_cast<float>(tf_msg.transform.rotation.x), static_cast<float>(tf_msg.transform.rotation.y),
        static_cast<float>(tf_msg.transform.rotation.z), static_cast<float>(tf_msg.transform.rotation.w),
        static_cast<float>(tf_msg.transform.translation.x), static_cast<float>(tf_msg.transform.translation.y),
        static_cast<float>(tf_msg.transform.translation.z));

    return true;
  } catch (const tf2::TransformException& ex) {
    laser.cached_transform.valid = false;
    return false;
  }
}

bool ScanMerger::lookup_transform(const size_t laser_idx,
                                 math::Transform3D& transform) {
  const auto& laser = lasers_[laser_idx];

  if (laser.frame_id.empty()) {
    return false;  // Frame not yet detected from scan
  }

  try {
    const auto tf_msg = tf_buffer_.lookupTransform(
        cloud_frame_id_, laser.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_));

    // Convert quaternion to transform using testable math function
    transform = math::quaternion_to_transform(
        static_cast<float>(tf_msg.transform.rotation.x), static_cast<float>(tf_msg.transform.rotation.y),
        static_cast<float>(tf_msg.transform.rotation.z), static_cast<float>(tf_msg.transform.rotation.w),
        static_cast<float>(tf_msg.transform.translation.x), static_cast<float>(tf_msg.transform.translation.y),
        static_cast<float>(tf_msg.transform.translation.z));

    return true;
  } catch (const tf2::TransformException& ex) {
    transform.valid = false;
    return false;
  }
}

// ============================================================================
// Callback Handlers
// ============================================================================

void ScanMerger::scan_callback(const size_t laser_index,
                               sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (laser_index >= lasers_.size()) {
    return;
  }

  std::lock_guard<std::mutex> lock(lasers_mutex_);
  auto& laser = lasers_[laser_index];

  // Detect frame_id from scan header on first reception
  if (laser.frame_id.empty()) {
    laser.frame_id = msg->header.frame_id;
    RCLCPP_INFO(this->get_logger(),
                "Laser %zu: detected frame_id '%s' from scan header",
                laser_index, laser.frame_id.c_str());

    // Cache transform if using fixed mode
    if (use_fixed_transforms_ && laser.show) {
      if (cache_transform(laser_index)) {
        RCLCPP_INFO(this->get_logger(), "  ✓ Cached transform: %s -> %s",
                    laser.frame_id.c_str(),
                    cloud_frame_id_.c_str());
      } else {
        RCLCPP_WARN(
            this->get_logger(),
            "  ✗ Failed to cache transform: %s -> %s (will use dynamic lookup)",
            laser.frame_id.c_str(), cloud_frame_id_.c_str());
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
      for (const auto& l : lasers_) {
        if (l.show && !l.data_received) {
          all_received = false;
          break;
        }
      }

      if (all_received) {
        publish_merged_cloud();
        // Reset flags for next cycle
        for (auto& l : lasers_) {
          l.data_received = false;
        }
      }
    } else {
      // Publish whenever any scan arrives
      publish_merged_cloud();
    }
  }
}

// ============================================================================
// Publishing and Processing
// ============================================================================

void ScanMerger::publish_merged_cloud() {
  std::lock_guard<std::mutex> lock(lasers_mutex_);

  std::vector<ColoredPoint> all_points;
  rclcpp::Time latest_timestamp = this->now();
  rclcpp::Time oldest_timestamp = this->now();
  bool has_valid_scan = false;
  int stale_scan_count = 0;

  // Process each laser scanner
  for (size_t laser_idx = 0; laser_idx < lasers_.size(); ++laser_idx) {
    const auto& laser = lasers_[laser_idx];

    if (!laser.show || !laser.last_scan || laser.last_scan->ranges.empty()) {
      continue;
    }

    // Check scan age
    const rclcpp::Duration scan_age = this->now() - laser.last_update_time;
    if (scan_age.seconds() > max_scan_age_) {
      ++stale_scan_count;
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
    const rclcpp::Time scan_time(laser.last_scan->header.stamp);
    if (scan_time > latest_timestamp) {
      latest_timestamp = scan_time;
    }
    if (!has_valid_scan || scan_time < oldest_timestamp) {
      oldest_timestamp = scan_time;
    }

    // Process scan with TF (cached or dynamic)
    process_laser_scan(laser, laser_idx, all_points);
  }

  if (!has_valid_scan) {
    return;  // No data to publish
  }

  // Warn if scans have large timestamp spread
  if (require_all_scans_) {
    const rclcpp::Duration timestamp_spread =
        latest_timestamp - oldest_timestamp;
    if (timestamp_spread.seconds() > 0.1) {  // 100ms threshold
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Large timestamp spread in merged scans: %.3f seconds",
          timestamp_spread.seconds());
    }
  }

  // Build PointCloud2 message directly (no PCL dependency)
  auto pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

  // Set header
  pc2_msg->header.frame_id = cloud_frame_id_;
  pc2_msg->header.stamp = latest_timestamp;
  pc2_msg->is_dense = false;

  // Set dimensions
  pc2_msg->height = 1;
  pc2_msg->width = all_points.size();

  // Define PointCloud2 fields (XYZRGB)
  sensor_msgs::PointCloud2Modifier modifier(*pc2_msg);
  modifier.setPointCloud2Fields(
      6, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "rgb", 1,
      sensor_msgs::msg::PointField::FLOAT32, "r", 1,
      sensor_msgs::msg::PointField::UINT8, "g", 1,
      sensor_msgs::msg::PointField::UINT8);
  modifier.resize(all_points.size());

  // Create iterators for efficient data population
  sensor_msgs::PointCloud2Iterator<float> iter_x(*pc2_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*pc2_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*pc2_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*pc2_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*pc2_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*pc2_msg, "b");

  // Copy points
  for (const auto& point : all_points) {
    *iter_x = point.x;
    *iter_y = point.y;
    *iter_z = point.z;
    *iter_r = point.r;
    *iter_g = point.g;
    *iter_b = point.b;

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_r;
    ++iter_g;
    ++iter_b;
  }

  point_cloud_pub_->publish(*pc2_msg);

  if (stale_scan_count > 0) {
    RCLCPP_DEBUG(this->get_logger(), "Published with %d stale scans",
                 stale_scan_count);
  }
}

void ScanMerger::process_laser_scan(const LaserConfig& laser,
                                   const size_t laser_idx,
                                   std::vector<ColoredPoint>& all_points) {
  const auto& scan = laser.last_scan;

  // Get transform (cached or lookup)
  math::Transform3D transform;

  if (use_fixed_transforms_ && laser.cached_transform.valid) {
    // Use cached transform (fast path)
    transform = laser.cached_transform;
  } else {
    // Dynamic TF lookup
    if (!lookup_transform(laser_idx, transform)) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "TF lookup failed for laser %zu (%s -> %s), skipping scan", laser_idx,
          laser.frame_id.c_str(), cloud_frame_id_.c_str());
      return;
    }
  }

  // Convert ROS2 LaserScan to testable ScanData
  ScanData scan_data{};
  scan_data.ranges = scan->ranges;
  scan_data.angle_min = scan->angle_min;
  scan_data.angle_max = scan->angle_max;
  scan_data.angle_increment = scan->angle_increment;

  // Configure processing
  ScanProcessingConfig config{};
  config.angle_min_deg = laser.angle_min;
  config.angle_max_deg = laser.angle_max;
  config.flip = laser.flip;
  config.inverse = laser.inverse;
  config.r = laser.r;
  config.g = laser.g;
  config.b = laser.b;

  // Process scan using testable core algorithm
  const std::vector<ColoredPoint> points =
      scan_processor_.process_scan(scan_data, transform, config);

  // Accumulate points from this laser
  all_points.insert(all_points.end(), points.begin(), points.end());
}

}  // namespace laser_scan_merger
