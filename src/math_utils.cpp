#include "laser_scan_merger/math_utils.hpp"

namespace laser_scan_merger {
namespace math {

Transform3D QuaternionToTransform(float qx, float qy, float qz, float qw,
                                   float tx, float ty, float tz) {
  Transform3D transform;

  // Convert quaternion to rotation matrix
  // Using the standard formula for quaternion to 3x3 rotation matrix
  transform.r00 = 1.0f - 2.0f * (qy * qy + qz * qz);
  transform.r01 = 2.0f * (qx * qy - qz * qw);
  transform.r02 = 2.0f * (qx * qz + qy * qw);

  transform.r10 = 2.0f * (qx * qy + qz * qw);
  transform.r11 = 1.0f - 2.0f * (qx * qx + qz * qz);
  transform.r12 = 2.0f * (qy * qz - qx * qw);

  transform.r20 = 2.0f * (qx * qz - qy * qw);
  transform.r21 = 2.0f * (qy * qz + qx * qw);
  transform.r22 = 1.0f - 2.0f * (qx * qx + qy * qy);

  // Set translation
  transform.tx = tx;
  transform.ty = ty;
  transform.tz = tz;

  transform.valid = true;
  return transform;
}

Point3D ApplyTransform(const Transform3D& transform, float x, float y, float z) {
  Point3D result;

  // Apply rotation matrix and translation
  // result = R * point + t
  result.x = transform.r00 * x + transform.r01 * y + transform.r02 * z + transform.tx;
  result.y = transform.r10 * x + transform.r11 * y + transform.r12 * z + transform.ty;
  result.z = transform.r20 * x + transform.r21 * y + transform.r22 * z + transform.tz;

  return result;
}

Point3D ApplyTransform(const Transform3D& transform, const Point3D& point) {
  return ApplyTransform(transform, point.x, point.y, point.z);
}

Point3D PolarToCartesian(float range, float angle) {
  Point3D result;
  result.x = range * std::cos(angle);
  result.y = range * std::sin(angle);
  result.z = 0.0f;  // Laser scans are 2D
  return result;
}

bool ShouldIncludePoint(float angle_rad, float filter_min_rad,
                        float filter_max_rad, bool inverse) {
  bool outside_range = (angle_rad < filter_min_rad) || (angle_rad > filter_max_rad);

  // Logic:
  //   inverse=false: include points INSIDE [min, max]  -> include if NOT outside
  //   inverse=true:  include points OUTSIDE [min, max] -> include if outside
  return inverse ? outside_range : !outside_range;
}

}  // namespace math
}  // namespace laser_scan_merger
