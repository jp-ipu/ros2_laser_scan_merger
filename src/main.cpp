//
//   created by: Michael Jonathan (mich1342)
//   github.com/mich1342
//   24/2/2022
//
//   Modified to support N laser scans (dynamic number of inputs)
//   Refactored for best practices, thread safety, and proper scan synchronization
//

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace laser_scan_merger {

// Configuration for each laser scanner
struct LaserConfig {
  std::string topic;
  float x_offset{0.0f};
  float y_offset{0.0f};
  float z_offset{0.0f};
  float alpha{0.0f};        // Rotation angle in degrees
  float angle_min{-181.0f}; // Minimum angle to include (degrees)
  float angle_max{181.0f};  // Maximum angle to include (degrees)
  uint8_t r{255};
  uint8_t g{0};
  uint8_t b{0};
  bool show{true};    // Enable/disable this laser
  bool flip{false};   // Flip the scan data
  bool inverse{false}; // Inverse the angle filtering logic

  sensor_msgs::msg::LaserScan::SharedPtr last_scan;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber;
  rclcpp::Time last_update_time;
};

class ScanMerger : public rclcpp::Node {
 public:
  ScanMerger() : Node("ros2_laser_scan_merger") {
    InitializeParams();
    RefreshParams();
    SetupSubscribers();

    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, rclcpp::SensorDataQoS());

    // Create timer for synchronized publishing
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),  // ~30Hz default
        std::bind(&ScanMerger::PublishMergedCloud, this));

    RCLCPP_INFO(this->get_logger(), "Laser Scan Merger initialized with %zu laser(s)",
                lasers_.size());
    for (size_t i = 0; i < lasers_.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "  Laser %zu: topic=%s, enabled=%s", i,
                  lasers_[i].topic.c_str(), lasers_[i].show ? "true" : "false");
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

  void ScanCallback(size_t laser_index,
                    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (laser_index >= lasers_.size()) {
      return;
    }

    std::lock_guard<std::mutex> lock(lasers_mutex_);
    lasers_[laser_index].last_scan = msg;
    lasers_[laser_index].last_update_time = this->now();
  }

  void PublishMergedCloud() {
    std::lock_guard<std::mutex> lock(lasers_mutex_);

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    rclcpp::Time latest_timestamp = this->now();
    bool has_valid_scan = false;

    // Process each laser scanner
    for (size_t laser_idx = 0; laser_idx < lasers_.size(); laser_idx++) {
      const auto& laser = lasers_[laser_idx];

      if (!laser.show || !laser.last_scan ||
          laser.last_scan->ranges.empty()) {
        continue;
      }

      has_valid_scan = true;

      // Track the most recent scan timestamp
      rclcpp::Time scan_time(laser.last_scan->header.stamp);
      if (scan_time > latest_timestamp) {
        latest_timestamp = scan_time;
      }

      ProcessLaserScan(laser, cloud);
    }

    if (!has_valid_scan) {
      return; // No data to publish
    }

    // Publish the merged point cloud
    auto pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud, *pc2_msg);
    pc2_msg->header.frame_id = cloud_frame_id_;
    pc2_msg->header.stamp = latest_timestamp;
    pc2_msg->is_dense = false;
    point_cloud_pub_->publish(*pc2_msg);
  }

  void ProcessLaserScan(const LaserConfig& laser,
                        pcl::PointCloud<pcl::PointXYZRGB>& cloud) const {
    const auto& scan = laser.last_scan;

    float angle_min = scan->angle_min;
    float angle_max = scan->angle_max;

    // Handle reversed angle ranges
    if (angle_min > angle_max) {
      std::swap(angle_min, angle_max);
    }

    const float alpha_rad = laser.alpha * M_PI / 180.0f;
    const float cos_alpha = std::cos(alpha_rad);
    const float sin_alpha = std::sin(alpha_rad);
    const float filter_min_rad = laser.angle_min * M_PI / 180.0f;
    const float filter_max_rad = laser.angle_max * M_PI / 180.0f;

    size_t num_points = scan->ranges.size();
    float current_angle = angle_min;

    for (size_t i = 0; i < num_points; ++i) {
      size_t index = laser.flip ? (num_points - 1 - i) : i;
      float range = scan->ranges[index];

      // Skip invalid ranges
      if (!std::isfinite(range) || range <= 0.0f) {
        current_angle += scan->angle_increment;
        continue;
      }

      // Apply angle filtering (before rotation)
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

      // Apply 2D rotation and translation to output frame
      pcl::PointXYZRGB point;
      point.x = local_x * cos_alpha - local_y * sin_alpha + laser.x_offset;
      point.y = local_x * sin_alpha + local_y * cos_alpha + laser.y_offset;
      point.z = laser.z_offset;
      point.r = laser.r;
      point.g = laser.g;
      point.b = laser.b;

      cloud.points.push_back(point);
      current_angle += scan->angle_increment;
    }
  }

  void InitializeParams() {
    this->declare_parameter("pointCloudTopic", "cloud_in");
    this->declare_parameter("pointCloutFrameId", "laser");
    this->declare_parameter("num_lasers", 2);
    this->declare_parameter("publish_rate", 30.0);
  }

  void RefreshParams() {
    cloud_topic_ = this->get_parameter("pointCloudTopic").as_string();
    cloud_frame_id_ = this->get_parameter("pointCloutFrameId").as_string();
    int num_lasers = this->get_parameter("num_lasers").as_int();
    double publish_rate = this->get_parameter("publish_rate").as_double();

    // Update publish timer rate if changed
    if (publish_rate > 0.0) {
      int period_ms = static_cast<int>(1000.0 / publish_rate);
      publish_timer_->cancel();
      publish_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(period_ms),
          std::bind(&ScanMerger::PublishMergedCloud, this));
    }

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

    // Declare parameters if not already declared
    if (!this->has_parameter(prefix + ".topic")) {
      this->declare_parameter(prefix + ".topic",
                              "/scan_" + std::to_string(laser_index));
      this->declare_parameter(prefix + ".x_offset", 0.0);
      this->declare_parameter(prefix + ".y_offset", 0.0);
      this->declare_parameter(prefix + ".z_offset", 0.0);
      this->declare_parameter(prefix + ".alpha", 0.0);
      this->declare_parameter(prefix + ".angle_min", -181.0);
      this->declare_parameter(prefix + ".angle_max", 181.0);
      this->declare_parameter(prefix + ".r", 255);
      this->declare_parameter(prefix + ".g", 0);
      this->declare_parameter(prefix + ".b", 0);
      this->declare_parameter(prefix + ".show", true);
      this->declare_parameter(prefix + ".flip", false);
      this->declare_parameter(prefix + ".inverse", false);
    }

    // Get parameters
    auto& laser = lasers_[laser_index];
    laser.topic = this->get_parameter(prefix + ".topic").as_string();
    laser.x_offset = static_cast<float>(
        this->get_parameter(prefix + ".x_offset").as_double());
    laser.y_offset = static_cast<float>(
        this->get_parameter(prefix + ".y_offset").as_double());
    laser.z_offset = static_cast<float>(
        this->get_parameter(prefix + ".z_offset").as_double());
    laser.alpha =
        static_cast<float>(this->get_parameter(prefix + ".alpha").as_double());
    laser.angle_min = static_cast<float>(
        this->get_parameter(prefix + ".angle_min").as_double());
    laser.angle_max = static_cast<float>(
        this->get_parameter(prefix + ".angle_max").as_double());

    int r_val = this->get_parameter(prefix + ".r").as_int();
    int g_val = this->get_parameter(prefix + ".g").as_int();
    int b_val = this->get_parameter(prefix + ".b").as_int();
    laser.r = static_cast<uint8_t>(r_val);
    laser.g = static_cast<uint8_t>(g_val);
    laser.b = static_cast<uint8_t>(b_val);

    laser.show = this->get_parameter(prefix + ".show").as_bool();
    laser.flip = this->get_parameter(prefix + ".flip").as_bool();
    laser.inverse = this->get_parameter(prefix + ".inverse").as_bool();
  }

  std::string cloud_topic_;
  std::string cloud_frame_id_;
  std::vector<LaserConfig> lasers_;
  std::mutex lasers_mutex_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

} // namespace laser_scan_merger

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<laser_scan_merger::ScanMerger>());
  rclcpp::shutdown();
  return 0;
}
