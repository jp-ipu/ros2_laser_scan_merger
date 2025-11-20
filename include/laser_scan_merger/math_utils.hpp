#ifndef LASER_SCAN_MERGER_MATH_UTILS_HPP_
#define LASER_SCAN_MERGER_MATH_UTILS_HPP_

#include <array>
#include <cmath>


namespace laser_scan_merger::math {

// 3D transform representation (rotation matrix + translation)
struct Transform3D {
  // Rotation matrix (row-major)
  float r00{}, r01{}, r02{};
  float r10{}, r11{}, r12{};
  float r20{}, r21{}, r22{};
  // Translation
  float tx{}, ty{}, tz{};

  bool valid{false};
};

// 3D point
struct Point3D {
  float x, y, z;
};

// Convert quaternion to 3D transform (rotation matrix + translation)
// Pure function - easily unit tested
// Input: quaternion (qx, qy, qz, qw) and translation (tx, ty, tz)
// Output: Transform3D with 3x3 rotation matrix and translation vector
Transform3D quaternion_to_transform(float qx, float qy, float qz, float qw,
                                   float tx, float ty, float tz);

// Apply 3D transform to a point
// Pure function - easily unit tested
// Input: transform and point in local frame
// Output: point in transformed frame
Point3D apply_transform(const Transform3D& transform, float x, float y, float z);

// Apply 3D transform to a point (overload for Point3D input)
Point3D apply_transform(const Transform3D& transform, const Point3D& point);

// Convert polar coordinates to Cartesian (in 2D, z=0)
// Pure function - easily unit tested
// Input: range (distance) and angle (radians)
// Output: Point3D with x, y, z=0
Point3D polar_to_cartesian(float range, float angle);

// Check if a point should be included based on angle filtering
// Pure function - easily unit tested
// Input: current angle, filter range (min, max), and inverse flag
// Output: true if point should be included
// Logic:
//   - If inverse=false: include points INSIDE [min, max]
//   - If inverse=true: include points OUTSIDE [min, max]
bool should_include_point(float angle_rad, float filter_min_rad,
                        float filter_max_rad, bool inverse);

// Convert degrees to radians
inline float degrees_to_radians(float degrees) {
  return static_cast<float>(M_PI) * degrees / 180.0f;
}

// Convert radians to degrees
inline float radians_to_degrees(float radians) {
  return radians * 180.0f / static_cast<float>(M_PI);
}

} // namespace laser_scan_merger::math


#endif  // LASER_SCAN_MERGER_MATH_UTILS_HPP_
