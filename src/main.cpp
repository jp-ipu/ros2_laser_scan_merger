//
//   created by: Michael Jonathan (mich1342)
//   github.com/mich1342
//   24/2/2022
//
//   Modified to support N laser scans (dynamic number of inputs)
//   Refactored for TF2-only approach with cached transforms for performance
//   Optimized based on ira_laser_tools analysis
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

namespace laser_scan_merger {

// Cached transform data for performance
struct CachedTransform {
  float r00, r01, r02, tx;  // Rotation matrix row 0 + translation x
  float r10, r11, r12, ty;  // Rotation matrix row 1 + translation y
  float r20, r21, r22, tz;  // Rotation matrix row 2 + translation z
  bool valid{false};
};

// Configuration for each laser scanner
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
  CachedTransform cached_transform;
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
      auto transform = tf_buffer_.lookupTransform(
          cloud_frame_id_, laser.detected_frame_id, tf2::TimePointZero,
          tf2::durationFromSec(tf_timeout_));

      // Extract translation
      laser.cached_transform.tx = transform.transform.translation.x;
      laser.cached_transform.ty = transform.transform.translation.y;
      laser.cached_transform.tz = transform.transform.translation.z;

      // Convert quaternion to rotation matrix
      float qx = transform.transform.rotation.x;
      float qy = transform.transform.rotation.y;
      float qz = transform.transform.rotation.z;
      float qw = transform.transform.rotation.w;

      laser.cached_transform.r00 = 1 - 2 * (qy * qy + qz * qz);
      laser.cached_transform.r01 = 2 * (qx * qy - qz * qw);
      laser.cached_transform.r02 = 2 * (qx * qz + qy * qw);
      laser.cached_transform.r10 = 2 * (qx * qy + qz * qw);
      laser.cached_transform.r11 = 1 - 2 * (qx * qx + qz * qz);
      laser.cached_transform.r12 = 2 * (qy * qz - qx * qw);
      laser.cached_transform.r20 = 2 * (qx * qz - qy * qw);
      laser.cached_transform.r21 = 2 * (qy * qz + qx * qw);
      laser.cached_transform.r22 = 1 - 2 * (qx * qx + qy * qy);

      laser.cached_transform.valid = true;
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
    CachedTransform transform;

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

    // Process scan points
    float angle_min = scan->angle_min;
    float angle_max = scan->angle_max;
    if (angle_min > angle_max) {
      std::swap(angle_min, angle_max);
    }

    const float filter_min_rad = laser.angle_min * M_PI / 180.0f;
    const float filter_max_rad = laser.angle_max * M_PI / 180.0f;

    size_t num_points = scan->ranges.size();
    float current_angle = angle_min;

    for (size_t i = 0; i < num_points; ++i) {
      size_t index = laser.flip ? (num_points - 1 - i) : i;
      float range = scan->ranges[index];

      if (!std::isfinite(range) || range <= 0.0f) {
        current_angle += scan->angle_increment;
        continue;
      }

      // Apply angle filtering
      bool outside_range =
          (current_angle < filter_min_rad) || (current_angle > filter_max_rad);
      bool include_point =
          (outside_range && laser.inverse) || (!outside_range && !laser.inverse);

      if (!include_point) {
        current_angle += scan->angle_increment;
        continue;
      }

      // Convert polar to Cartesian (in laser frame)
      float local_x = range * std::cos(current_angle);
      float local_y = range * std::sin(current_angle);
      float local_z = 0.0f;

      // Apply 3D TF transform
      pcl::PointXYZRGB point;
      point.x = transform.r00 * local_x + transform.r01 * local_y +
                transform.r02 * local_z + transform.tx;
      point.y = transform.r10 * local_x + transform.r11 * local_y +
                transform.r12 * local_z + transform.ty;
      point.z = transform.r20 * local_x + transform.r21 * local_y +
                transform.r22 * local_z + transform.tz;
      point.r = laser.r;
      point.g = laser.g;
      point.b = laser.b;

      cloud.points.push_back(point);
      current_angle += scan->angle_increment;
    }
  }

  bool LookupTransform(size_t laser_idx, CachedTransform& transform) {
    const auto& laser = lasers_[laser_idx];

    if (laser.detected_frame_id.empty()) {
      return false;  // Frame not yet detected from scan
    }

    try {
      auto tf_transform = tf_buffer_.lookupTransform(
          cloud_frame_id_, laser.detected_frame_id, tf2::TimePointZero,
          tf2::durationFromSec(tf_timeout_));

      // Extract translation
      transform.tx = tf_transform.transform.translation.x;
      transform.ty = tf_transform.transform.translation.y;
      transform.tz = tf_transform.transform.translation.z;

      // Convert quaternion to rotation matrix
      float qx = tf_transform.transform.rotation.x;
      float qy = tf_transform.transform.rotation.y;
      float qz = tf_transform.transform.rotation.z;
      float qw = tf_transform.transform.rotation.w;

      transform.r00 = 1 - 2 * (qy * qy + qz * qz);
      transform.r01 = 2 * (qx * qy - qz * qw);
      transform.r02 = 2 * (qx * qz + qy * qw);
      transform.r10 = 2 * (qx * qy + qz * qw);
      transform.r11 = 1 - 2 * (qx * qx + qz * qz);
      transform.r12 = 2 * (qy * qz - qx * qw);
      transform.r20 = 2 * (qx * qz - qy * qw);
      transform.r21 = 2 * (qy * qz + qx * qw);
      transform.r22 = 1 - 2 * (qx * qx + qy * qy);

      transform.valid = true;
      return true;

    } catch (const tf2::TransformException& ex) {
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
};

}  // namespace laser_scan_merger

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<laser_scan_merger::ScanMerger>());
  rclcpp::shutdown();
  return 0;
}
