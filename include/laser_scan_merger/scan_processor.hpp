#ifndef LASER_SCAN_MERGER_SCAN_PROCESSOR_HPP_
#define LASER_SCAN_MERGER_SCAN_PROCESSOR_HPP_

#include <cstdint>
#include <vector>

#include "laser_scan_merger/math_utils.hpp"

namespace laser_scan_merger {

// Configuration for processing a single laser scan
struct ScanProcessingConfig {
  float angle_min_deg;  // Minimum angle to include (degrees)
  float angle_max_deg;  // Maximum angle to include (degrees)
  bool flip;            // Flip scan data order
  bool inverse;         // Inverse angle filtering logic
  uint8_t r, g, b;      // RGB color for visualization
};

// Lightweight scan data structure (no ROS2 dependency)
struct ScanData {
  std::vector<float> ranges;
  float angle_min;        // radians
  float angle_max;        // radians
  float angle_increment;  // radians
};

// Result point (XYZRGB)
struct ColoredPoint {
  float x, y, z;
  uint8_t r, g, b;
};

// Core laser scan processing class with NO ROS2 dependencies
// All methods are pure or have explicit parameters - easily testable
class LaserScanProcessor {
 public:
  LaserScanProcessor() = default;
  ~LaserScanProcessor() = default;

  // Process a single laser scan and return colored points
  // Pure function (no side effects, no ROS2 dependencies)
  //
  // Input:
  //   - scan: Laser scan data (ranges, angles)
  //   - transform: 3D transform to apply (laser frame -> target frame)
  //   - config: Processing configuration (filtering, colors)
  //
  // Output:
  //   - Vector of colored 3D points in target frame
  //
  // This is the CORE ALGORITHM - 100% unit testable
  std::vector<ColoredPoint> ProcessScan(const ScanData& scan,
                                         const math::Transform3D& transform,
                                         const ScanProcessingConfig& config) const;

  // Get statistics from last processing (for debugging/monitoring)
  struct Statistics {
    size_t total_points{0};
    size_t filtered_points{0};
    size_t invalid_points{0};
    size_t output_points{0};
  };

  const Statistics& GetLastStatistics() const { return last_stats_; }

 private:
  mutable Statistics last_stats_;  // Updated during ProcessScan
};

}  // namespace laser_scan_merger

#endif  // LASER_SCAN_MERGER_SCAN_PROCESSOR_HPP_
