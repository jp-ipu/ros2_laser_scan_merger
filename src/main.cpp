//
//   created by: Michael Jonathan (mich1342)
//   github.com/mich1342
//   24/2/2022
//
//   Modified to support N laser scans (dynamic number of inputs)
//

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

#include <string>
#include <vector>
#include <array>
#include <iostream>

// Struct to hold configuration for each laser scanner
struct LaserConfig
{
  std::string topic;
  float x_offset;
  float y_offset;
  float z_offset;
  float alpha;       // Rotation angle in degrees
  float angle_min;   // Minimum angle to include (degrees)
  float angle_max;   // Maximum angle to include (degrees)
  uint8_t r, g, b;   // RGB color for visualization
  bool show;         // Enable/disable this laser
  bool flip;         // Flip the scan data
  bool inverse;      // Inverse the angle filtering logic

  sensor_msgs::msg::LaserScan::SharedPtr last_scan;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber;
};

class scanMerger : public rclcpp::Node
{
public:
  scanMerger() : Node("ros2_laser_scan_merger")
  {
    initialize_params();
    refresh_params();
    setup_subscribers();

    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(), "Laser Scan Merger initialized with %zu laser(s)", lasers_.size());
    for (size_t i = 0; i < lasers_.size(); i++)
    {
      RCLCPP_INFO(this->get_logger(), "  Laser %zu: topic=%s, enabled=%s",
                  i, lasers_[i].topic.c_str(), lasers_[i].show ? "true" : "false");
    }
  }

private:
  void setup_subscribers()
  {
    auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

    for (size_t i = 0; i < lasers_.size(); i++)
    {
      lasers_[i].last_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
      lasers_[i].subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
          lasers_[i].topic,
          default_qos,
          [this, i](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            this->scan_callback(i, msg);
          });
    }
  }

  void scan_callback(size_t laser_index, const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (laser_index < lasers_.size())
    {
      lasers_[laser_index].last_scan = msg;
      update_point_cloud_rgb();
    }
  }

  void update_point_cloud_rgb()
  {
    refresh_params();
    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    std::vector<std::array<float, 2>> scan_data;
    float min_theta = 0;
    float max_theta = 0;
    bool first_point = true;

    // Process each laser scanner
    for (size_t laser_idx = 0; laser_idx < lasers_.size(); laser_idx++)
    {
      const auto& laser = lasers_[laser_idx];

      if (!laser.show || !laser.last_scan)
      {
        continue;
      }

      float temp_min, temp_max;
      if (laser.last_scan->angle_min < laser.last_scan->angle_max)
      {
        temp_min = laser.last_scan->angle_min;
        temp_max = laser.last_scan->angle_max;
      }
      else
      {
        temp_min = laser.last_scan->angle_max;
        temp_max = laser.last_scan->angle_min;
      }

      int count = 0;
      for (float i = temp_min; i <= temp_max && count < laser.last_scan->ranges.size();
           i += laser.last_scan->angle_increment)
      {
        pcl::PointXYZRGB pt;
        pt = pcl::PointXYZRGB(laser.r, laser.g, laser.b);

        int used_count = count;
        if (laser.flip)
        {
          used_count = (int)laser.last_scan->ranges.size() - 1 - count;
        }

        float range = laser.last_scan->ranges[used_count];
        if (!std::isfinite(range) || range <= 0.0f)
        {
          count++;
          continue;
        }

        float temp_x = range * std::cos(i);
        float temp_y = range * std::sin(i);

        // Apply rotation and translation
        float alpha_rad = laser.alpha * M_PI / 180.0;
        pt.x = temp_x * std::cos(alpha_rad) - temp_y * std::sin(alpha_rad) + laser.x_offset;
        pt.y = temp_x * std::sin(alpha_rad) + temp_y * std::cos(alpha_rad) + laser.y_offset;
        pt.z = laser.z_offset;

        // Apply angle filtering
        float angle_min_rad = laser.angle_min * M_PI / 180.0;
        float angle_max_rad = laser.angle_max * M_PI / 180.0;
        bool outside_range = (i < angle_min_rad) || (i > angle_max_rad);

        bool include_point = false;
        if (outside_range && laser.inverse)
        {
          include_point = true;
        }
        else if (!outside_range && !laser.inverse)
        {
          include_point = true;
        }

        if (include_point)
        {
          cloud_.points.push_back(pt);
          float r_ = GET_R(pt.x, pt.y);
          float theta_ = GET_THETA(pt.x, pt.y);
          std::array<float, 2> res_;
          res_[1] = r_;
          res_[0] = theta_;
          scan_data.push_back(res_);

          if (first_point)
          {
            min_theta = theta_;
            max_theta = theta_;
            first_point = false;
          }
          else
          {
            if (theta_ < min_theta)
            {
              min_theta = theta_;
            }
            if (theta_ > max_theta)
            {
              max_theta = theta_;
            }
          }
        }
        count++;
      }
    }

    // Publish the merged point cloud
    auto pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud_, *pc2_msg);
    pc2_msg->header.frame_id = cloud_frame_id_;
    pc2_msg->header.stamp = now();

    // Use timestamp from the first enabled laser
    for (const auto& laser : lasers_)
    {
      if (laser.show && laser.last_scan)
      {
        pc2_msg->header.stamp = laser.last_scan->header.stamp;
        break;
      }
    }

    pc2_msg->is_dense = false;
    point_cloud_pub_->publish(*pc2_msg);
  }

  float GET_R(float x, float y)
  {
    return sqrt(x * x + y * y);
  }

  float GET_THETA(float x, float y)
  {
    float temp_res;
    if ((x != 0))
    {
      temp_res = atan(y / x);
    }
    else
    {
      if (y >= 0)
      {
        temp_res = M_PI / 2;
      }
      else
      {
        temp_res = -M_PI / 2;
      }
    }
    if (temp_res > 0)
    {
      if (y < 0)
      {
        temp_res -= M_PI;
      }
    }
    else if (temp_res < 0)
    {
      if (x < 0)
      {
        temp_res += M_PI;
      }
    }
    return temp_res;
  }

  void initialize_params()
  {
    // Declare global parameters
    this->declare_parameter("pointCloudTopic", "cloud_in");
    this->declare_parameter("pointCloutFrameId", "laser");
    this->declare_parameter("num_lasers", 2);
  }

  void refresh_params()
  {
    // Get global parameters
    this->get_parameter_or<std::string>("pointCloudTopic", cloud_topic_, "cloud_in");
    this->get_parameter_or<std::string>("pointCloutFrameId", cloud_frame_id_, "laser");

    int num_lasers = 2;
    this->get_parameter_or<int>("num_lasers", num_lasers, 2);

    // Resize laser vector if needed
    if (lasers_.size() != static_cast<size_t>(num_lasers))
    {
      lasers_.resize(num_lasers);
    }

    // Load parameters for each laser
    for (int i = 0; i < num_lasers; i++)
    {
      std::string prefix = "laser" + std::to_string(i);

      // Declare parameters if not already declared
      if (!this->has_parameter(prefix + ".topic"))
      {
        this->declare_parameter(prefix + ".topic", "/scan_" + std::to_string(i));
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
      this->get_parameter_or<std::string>(prefix + ".topic", lasers_[i].topic, "/scan_" + std::to_string(i));
      this->get_parameter_or<float>(prefix + ".x_offset", lasers_[i].x_offset, 0.0);
      this->get_parameter_or<float>(prefix + ".y_offset", lasers_[i].y_offset, 0.0);
      this->get_parameter_or<float>(prefix + ".z_offset", lasers_[i].z_offset, 0.0);
      this->get_parameter_or<float>(prefix + ".alpha", lasers_[i].alpha, 0.0);
      this->get_parameter_or<float>(prefix + ".angle_min", lasers_[i].angle_min, -181.0);
      this->get_parameter_or<float>(prefix + ".angle_max", lasers_[i].angle_max, 181.0);

      int r_val = 255, g_val = 0, b_val = 0;
      this->get_parameter_or<int>(prefix + ".r", r_val, 255);
      this->get_parameter_or<int>(prefix + ".g", g_val, 0);
      this->get_parameter_or<int>(prefix + ".b", b_val, 0);
      lasers_[i].r = static_cast<uint8_t>(r_val);
      lasers_[i].g = static_cast<uint8_t>(g_val);
      lasers_[i].b = static_cast<uint8_t>(b_val);

      this->get_parameter_or<bool>(prefix + ".show", lasers_[i].show, true);
      this->get_parameter_or<bool>(prefix + ".flip", lasers_[i].flip, false);
      this->get_parameter_or<bool>(prefix + ".inverse", lasers_[i].inverse, false);
    }
  }

  std::string cloud_topic_, cloud_frame_id_;
  std::vector<LaserConfig> lasers_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scanMerger>());
  rclcpp::shutdown();
  return 0;
}
