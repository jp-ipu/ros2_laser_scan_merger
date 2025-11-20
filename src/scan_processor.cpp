#include "laser_scan_merger/scan_processor.hpp"

#include <cmath>

namespace laser_scan_merger {

std::vector<ColoredPoint> LaserScanProcessor::ProcessScan(
    const ScanData& scan, const math::Transform3D& transform,
    const ScanProcessingConfig& config) const {
  // Reset statistics
  last_stats_ = Statistics();

  std::vector<ColoredPoint> result;
  result.reserve(scan.ranges.size());  // Pre-allocate for efficiency

  // Convert filter angles from degrees to radians
  const float filter_min_rad = math::DegreesToRadians(config.angle_min_deg);
  const float filter_max_rad = math::DegreesToRadians(config.angle_max_deg);

  // Normalize angle range
  float angle_min = scan.angle_min;
  float angle_max = scan.angle_max;
  if (angle_min > angle_max) {
    std::swap(angle_min, angle_max);
  }

  const size_t num_points = scan.ranges.size();
  last_stats_.total_points = num_points;

  float current_angle = angle_min;

  for (size_t i = 0; i < num_points; ++i) {
    // Apply flip if configured
    const size_t index = config.flip ? (num_points - 1 - i) : i;
    const float range = scan.ranges[index];

    // Check for invalid range
    if (!std::isfinite(range) || range <= 0.0f) {
      last_stats_.invalid_points++;
      current_angle += scan.angle_increment;
      continue;
    }

    // Apply angle filtering
    if (!math::ShouldIncludePoint(current_angle, filter_min_rad, filter_max_rad,
                                   config.inverse)) {
      last_stats_.filtered_points++;
      current_angle += scan.angle_increment;
      continue;
    }

    // Convert polar to Cartesian (in laser frame)
    math::Point3D local_point = math::PolarToCartesian(range, current_angle);

    // Apply 3D transform (laser frame -> target frame)
    math::Point3D transformed_point = math::ApplyTransform(transform, local_point);

    // Create colored point
    ColoredPoint point;
    point.x = transformed_point.x;
    point.y = transformed_point.y;
    point.z = transformed_point.z;
    point.r = config.r;
    point.g = config.g;
    point.b = config.b;

    result.push_back(point);
    current_angle += scan.angle_increment;
  }

  last_stats_.output_points = result.size();
  return result;
}

}  // namespace laser_scan_merger
