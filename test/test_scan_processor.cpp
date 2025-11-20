#include <gtest/gtest.h>
#include <cmath>
#include "laser_scan_merger/scan_processor.hpp"
#include "laser_scan_merger/math_utils.hpp"

using namespace laser_scan_merger;

// Test processing a simple scan with identity transform
TEST(ScanProcessorTest, process_scanIdentityTransform) {
  LaserScanProcessor processor;

  // Create scan data: 3 points at 0, 45, 90 degrees with range 1.0
  ScanData scan;
  scan.ranges = {1.0f, 1.0f, 1.0f};
  scan.angle_min = 0.0f;
  scan.angle_max = M_PI / 2.0f;
  scan.angle_increment = M_PI / 4.0f;  // 45 degrees

  // Identity transform (no rotation, no translation)
  math::Transform3D transform = math::quaternion_to_transform(
      0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);

  // Processing config: include all angles
  ScanProcessingConfig config;
  config.angle_min_deg = -180.0f;
  config.angle_max_deg = 180.0f;
  config.flip = false;
  config.inverse = false;
  config.r = 255;
  config.g = 0;
  config.b = 0;

  std::vector<ColoredPoint> points = processor.process_scan(scan, transform, config);

  // Should have 3 points
  ASSERT_EQ(points.size(), 3);

  // Check first point (0 degrees, range 1.0)
  EXPECT_NEAR(points[0].x, 1.0f, 1e-5);
  EXPECT_NEAR(points[0].y, 0.0f, 1e-5);
  EXPECT_FLOAT_EQ(points[0].z, 0.0f);
  EXPECT_EQ(points[0].r, 255);
  EXPECT_EQ(points[0].g, 0);
  EXPECT_EQ(points[0].b, 0);

  // Check second point (45 degrees, range 1.0)
  EXPECT_NEAR(points[1].x, std::cos(M_PI / 4.0f), 1e-5);
  EXPECT_NEAR(points[1].y, std::sin(M_PI / 4.0f), 1e-5);
  EXPECT_FLOAT_EQ(points[1].z, 0.0f);

  // Check third point (90 degrees, range 1.0)
  EXPECT_NEAR(points[2].x, 0.0f, 1e-5);
  EXPECT_NEAR(points[2].y, 1.0f, 1e-5);
  EXPECT_FLOAT_EQ(points[2].z, 0.0f);
}

// Test processing with translation transform
TEST(ScanProcessorTest, process_scanWithTranslation) {
  LaserScanProcessor processor;

  // Simple scan: 1 point at 0 degrees, range 1.0
  ScanData scan;
  scan.ranges = {1.0f};
  scan.angle_min = 0.0f;
  scan.angle_max = 0.0f;
  scan.angle_increment = 0.0f;

  // Transform with translation (10, 20, 30)
  math::Transform3D transform = math::quaternion_to_transform(
      0.0f, 0.0f, 0.0f, 1.0f, 10.0f, 20.0f, 30.0f);

  ScanProcessingConfig config;
  config.angle_min_deg = -180.0f;
  config.angle_max_deg = 180.0f;
  config.flip = false;
  config.inverse = false;
  config.r = 0;
  config.g = 255;
  config.b = 0;

  std::vector<ColoredPoint> points = processor.process_scan(scan, transform, config);

  ASSERT_EQ(points.size(), 1);

  // Point should be (1, 0, 0) + (10, 20, 30) = (11, 20, 30)
  EXPECT_FLOAT_EQ(points[0].x, 11.0f);
  EXPECT_FLOAT_EQ(points[0].y, 20.0f);
  EXPECT_FLOAT_EQ(points[0].z, 30.0f);
}

// Test angle filtering (normal mode)
TEST(ScanProcessorTest, AngleFilteringNormalMode) {
  LaserScanProcessor processor;

  // Create scan: 5 points from 0 to 180 degrees
  ScanData scan;
  scan.ranges = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
  scan.angle_min = 0.0f;
  scan.angle_max = M_PI;
  scan.angle_increment = M_PI / 4.0f;  // 45 degrees

  math::Transform3D transform = math::quaternion_to_transform(
      0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);

  // Filter to only include 0-90 degrees (first 3 points)
  ScanProcessingConfig config;
  config.angle_min_deg = 0.0f;
  config.angle_max_deg = 90.0f;
  config.flip = false;
  config.inverse = false;  // Normal mode: include inside range
  config.r = 0;
  config.g = 0;
  config.b = 255;

  std::vector<ColoredPoint> points = processor.process_scan(scan, transform, config);

  // Should have 3 points (0, 45, 90 degrees)
  EXPECT_EQ(points.size(), 3);
}

// Test angle filtering (inverse mode)
TEST(ScanProcessorTest, AngleFilteringInverseMode) {
  LaserScanProcessor processor;

  // Create scan: 5 points from 0 to 180 degrees
  ScanData scan;
  scan.ranges = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
  scan.angle_min = 0.0f;
  scan.angle_max = M_PI;
  scan.angle_increment = M_PI / 4.0f;

  math::Transform3D transform = math::quaternion_to_transform(
      0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);

  // Filter to exclude 0-90 degrees (inverse mode)
  ScanProcessingConfig config;
  config.angle_min_deg = 0.0f;
  config.angle_max_deg = 90.0f;
  config.flip = false;
  config.inverse = true;  // Inverse mode: exclude inside range
  config.r = 0;
  config.g = 0;
  config.b = 255;

  std::vector<ColoredPoint> points = processor.process_scan(scan, transform, config);

  // Should have 2 points (135, 180 degrees) - excluding 0, 45, 90
  EXPECT_EQ(points.size(), 2);
}

// Test flip mode
TEST(ScanProcessorTest, FlipMode) {
  LaserScanProcessor processor;

  // Create scan with different ranges
  ScanData scan;
  scan.ranges = {1.0f, 2.0f, 3.0f};
  scan.angle_min = 0.0f;
  scan.angle_max = M_PI / 2.0f;
  scan.angle_increment = M_PI / 4.0f;

  math::Transform3D transform = math::quaternion_to_transform(
      0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);

  ScanProcessingConfig config;
  config.angle_min_deg = -180.0f;
  config.angle_max_deg = 180.0f;
  config.flip = true;  // Reverse scan order
  config.inverse = false;
  config.r = 255;
  config.g = 0;
  config.b = 0;

  std::vector<ColoredPoint> points = processor.process_scan(scan, transform, config);

  ASSERT_EQ(points.size(), 3);

  // With flip, ranges should be reversed: 3.0, 2.0, 1.0
  // First point should use range 3.0
  float expected_x = 3.0f * std::cos(0.0f);
  float expected_y = 3.0f * std::sin(0.0f);
  EXPECT_NEAR(points[0].x, expected_x, 1e-5);
  EXPECT_NEAR(points[0].y, expected_y, 1e-5);
}

// Test invalid ranges are filtered
TEST(ScanProcessorTest, FilterInvalidRanges) {
  LaserScanProcessor processor;

  // Create scan with invalid ranges
  ScanData scan;
  scan.ranges = {1.0f, -1.0f, std::numeric_limits<float>::infinity(), 0.0f, 2.0f};
  scan.angle_min = 0.0f;
  scan.angle_max = M_PI;
  scan.angle_increment = M_PI / 4.0f;

  math::Transform3D transform = math::quaternion_to_transform(
      0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);

  ScanProcessingConfig config;
  config.angle_min_deg = -180.0f;
  config.angle_max_deg = 180.0f;
  config.flip = false;
  config.inverse = false;
  config.r = 255;
  config.g = 0;
  config.b = 0;

  std::vector<ColoredPoint> points = processor.process_scan(scan, transform, config);

  // Should only have 2 valid points (1.0 and 2.0)
  EXPECT_EQ(points.size(), 2);
}

// Test statistics
TEST(ScanProcessorTest, Statistics) {
  LaserScanProcessor processor;

  ScanData scan;
  scan.ranges = {1.0f, -1.0f, 2.0f, std::numeric_limits<float>::infinity(), 3.0f};
  scan.angle_min = 0.0f;
  scan.angle_max = M_PI;
  scan.angle_increment = M_PI / 4.0f;

  math::Transform3D transform = math::quaternion_to_transform(
      0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);

  ScanProcessingConfig config;
  config.angle_min_deg = -180.0f;
  config.angle_max_deg = 180.0f;
  config.flip = false;
  config.inverse = false;
  config.r = 255;
  config.g = 0;
  config.b = 0;

  std::vector<ColoredPoint> points = processor.process_scan(scan, transform, config);

  const auto& stats = processor.get_last_statistics();

  EXPECT_EQ(stats.total_points, 5);
  EXPECT_EQ(stats.invalid_points, 2);  // -1.0 and inf
  EXPECT_EQ(stats.output_points, 3);    // 1.0, 2.0, 3.0
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
