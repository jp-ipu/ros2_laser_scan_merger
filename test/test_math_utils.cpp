#include <gtest/gtest.h>
#include <cmath>
#include "laser_scan_merger/math_utils.hpp"

using namespace laser_scan_merger::math;

// Test quaternion to transform conversion (identity transform)
TEST(MathUtilsTest, QuaternionToTransformIdentity) {
  // Identity quaternion (no rotation)
  Transform3D transform = QuaternionToTransform(0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 2.0f, 3.0f);

  EXPECT_TRUE(transform.valid);

  // Check rotation matrix is identity
  EXPECT_NEAR(transform.r00, 1.0f, 1e-6);
  EXPECT_NEAR(transform.r01, 0.0f, 1e-6);
  EXPECT_NEAR(transform.r02, 0.0f, 1e-6);
  EXPECT_NEAR(transform.r10, 0.0f, 1e-6);
  EXPECT_NEAR(transform.r11, 1.0f, 1e-6);
  EXPECT_NEAR(transform.r12, 0.0f, 1e-6);
  EXPECT_NEAR(transform.r20, 0.0f, 1e-6);
  EXPECT_NEAR(transform.r21, 0.0f, 1e-6);
  EXPECT_NEAR(transform.r22, 1.0f, 1e-6);

  // Check translation
  EXPECT_FLOAT_EQ(transform.tx, 1.0f);
  EXPECT_FLOAT_EQ(transform.ty, 2.0f);
  EXPECT_FLOAT_EQ(transform.tz, 3.0f);
}

// Test 180 degree rotation around Z axis
TEST(MathUtilsTest, QuaternionToTransform180DegZ) {
  // 180 degree rotation around Z axis: qz = 1, qw = 0
  Transform3D transform = QuaternionToTransform(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  EXPECT_TRUE(transform.valid);

  // Should flip X and Y
  EXPECT_NEAR(transform.r00, -1.0f, 1e-6);
  EXPECT_NEAR(transform.r11, -1.0f, 1e-6);
  EXPECT_NEAR(transform.r22, 1.0f, 1e-6);
}

// Test applying identity transform
TEST(MathUtilsTest, ApplyTransformIdentity) {
  Transform3D identity = QuaternionToTransform(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);

  Point3D result = ApplyTransform(identity, 1.0f, 2.0f, 3.0f);

  EXPECT_FLOAT_EQ(result.x, 1.0f);
  EXPECT_FLOAT_EQ(result.y, 2.0f);
  EXPECT_FLOAT_EQ(result.z, 3.0f);
}

// Test applying translation
TEST(MathUtilsTest, ApplyTransformTranslation) {
  Transform3D translation = QuaternionToTransform(0.0f, 0.0f, 0.0f, 1.0f, 10.0f, 20.0f, 30.0f);

  Point3D result = ApplyTransform(translation, 1.0f, 2.0f, 3.0f);

  EXPECT_FLOAT_EQ(result.x, 11.0f);
  EXPECT_FLOAT_EQ(result.y, 22.0f);
  EXPECT_FLOAT_EQ(result.z, 33.0f);
}

// Test polar to Cartesian conversion
TEST(MathUtilsTest, PolarToCartesian) {
  // 0 degrees, range 5
  Point3D p1 = PolarToCartesian(5.0f, 0.0f);
  EXPECT_NEAR(p1.x, 5.0f, 1e-6);
  EXPECT_NEAR(p1.y, 0.0f, 1e-6);
  EXPECT_FLOAT_EQ(p1.z, 0.0f);

  // 90 degrees, range 3
  Point3D p2 = PolarToCartesian(3.0f, M_PI / 2.0f);
  EXPECT_NEAR(p2.x, 0.0f, 1e-6);
  EXPECT_NEAR(p2.y, 3.0f, 1e-6);
  EXPECT_FLOAT_EQ(p2.z, 0.0f);

  // 180 degrees, range 2
  Point3D p3 = PolarToCartesian(2.0f, M_PI);
  EXPECT_NEAR(p3.x, -2.0f, 1e-6);
  EXPECT_NEAR(p3.y, 0.0f, 1e-6);
  EXPECT_FLOAT_EQ(p3.z, 0.0f);
}

// Test angle filtering (normal mode - include inside range)
TEST(MathUtilsTest, ShouldIncludePointNormalMode) {
  const float min = 0.0f;
  const float max = M_PI / 2.0f;  // 90 degrees

  // Inside range - should include
  EXPECT_TRUE(ShouldIncludePoint(0.5f, min, max, false));
  EXPECT_TRUE(ShouldIncludePoint(M_PI / 4.0f, min, max, false));

  // Outside range - should not include
  EXPECT_FALSE(ShouldIncludePoint(-0.1f, min, max, false));
  EXPECT_FALSE(ShouldIncludePoint(M_PI, min, max, false));
}

// Test angle filtering (inverse mode - include outside range)
TEST(MathUtilsTest, ShouldIncludePointInverseMode) {
  const float min = 0.0f;
  const float max = M_PI / 2.0f;  // 90 degrees

  // Inside range - should not include (inverse mode)
  EXPECT_FALSE(ShouldIncludePoint(0.5f, min, max, true));
  EXPECT_FALSE(ShouldIncludePoint(M_PI / 4.0f, min, max, true));

  // Outside range - should include (inverse mode)
  EXPECT_TRUE(ShouldIncludePoint(-0.1f, min, max, true));
  EXPECT_TRUE(ShouldIncludePoint(M_PI, min, max, true));
}

// Test degree/radian conversions
TEST(MathUtilsTest, DegreeRadianConversions) {
  EXPECT_FLOAT_EQ(DegreesToRadians(0.0f), 0.0f);
  EXPECT_NEAR(DegreesToRadians(90.0f), M_PI / 2.0f, 1e-6);
  EXPECT_NEAR(DegreesToRadians(180.0f), M_PI, 1e-6);
  EXPECT_NEAR(DegreesToRadians(360.0f), 2.0f * M_PI, 1e-6);

  EXPECT_FLOAT_EQ(RadiansToDegrees(0.0f), 0.0f);
  EXPECT_NEAR(RadiansToDegrees(M_PI / 2.0f), 90.0f, 1e-4);
  EXPECT_NEAR(RadiansToDegrees(M_PI), 180.0f, 1e-4);
  EXPECT_NEAR(RadiansToDegrees(2.0f * M_PI), 360.0f, 1e-4);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
