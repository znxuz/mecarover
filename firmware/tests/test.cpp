#include <gtest/gtest.h>

#include <cstdio>
#include <random>

#include "../application/pose_types.hpp"

using namespace imsl;
using std::numbers::pi;

static constexpr double tolerance = 1e-10;

class PoseTest : public ::testing::Test {
 protected:
  std::mt19937 rng{std::random_device{}()};
  std::uniform_real_distribution<double> dist{-10000, 10000};

  Pose pose1{1.0, 2.0, Heading(pi / 4)};
  Pose pose2{3.0, 4.0, Heading(pi / 2)};
};

TEST_F(PoseTest, DefaultConstructor) {
  Pose p;
  EXPECT_EQ(p.x, 0);
  EXPECT_EQ(p.y, 0);
  EXPECT_EQ(static_cast<double>(p.theta), 0);
}

TEST_F(PoseTest, ParameterizedConstructor) {
  EXPECT_EQ(pose1.x, 1.0);
  EXPECT_EQ(pose1.y, 2.0);
  EXPECT_EQ(static_cast<double>(pose1.theta), pi / 4);
}

TEST_F(PoseTest, PoseFromVPoseConversion) {
  vPose vpose{1.5, 2.5, 3.5};
  Pose pose_from_vpose(vpose);

  EXPECT_EQ(pose_from_vpose.x, 1.5);
  EXPECT_EQ(pose_from_vpose.y, 2.5);
  EXPECT_EQ(static_cast<double>(pose_from_vpose.theta), Heading(3.5));
}

TEST_F(PoseTest, RandomizedAddition) {
  for (int i = 0; i < 10000; ++i) {
    double random_x = dist(rng);
    double random_y = dist(rng);
    double random_theta = dist(rng);

    Pose randomPose(random_x, random_y, Heading(random_theta));
    Pose result = pose1 + randomPose;

    EXPECT_EQ(result.x, pose1.x + random_x);
    EXPECT_EQ(result.y, pose1.y + random_y);
    EXPECT_NEAR(static_cast<double>(result.theta),
                Heading(static_cast<double>(pose1.theta) + random_theta),
                tolerance);
  }
}

TEST_F(PoseTest, ScalarMultiplicationOperator) {
  double factor = 2.5;

  Pose originalPose{2.0, 3.0, Heading(pi / 3)};
  Pose expectedPose{originalPose.x * factor, originalPose.y * factor,
                    originalPose.theta * factor};

  Pose scaledPose = originalPose * factor;

  EXPECT_DOUBLE_EQ(scaledPose.x, expectedPose.x);
  EXPECT_DOUBLE_EQ(scaledPose.y, expectedPose.y);
  EXPECT_DOUBLE_EQ(scaledPose.theta, expectedPose.theta);
}

TEST_F(PoseTest, DivisionOperator) {
  for (int i = 0; i < 100; ++i) {
    double x = dist(rng);
    double y = dist(rng);
    double theta = dist(rng);
    double divisor = dist(rng);

    if (divisor == 0.0) continue;

    Pose p(x, y, theta);
    Pose result = p / divisor;

    EXPECT_NEAR(result.x, x / divisor, tolerance);
    EXPECT_NEAR(result.y, y / divisor, tolerance);
    EXPECT_NEAR(result.theta, Heading(theta) / divisor, tolerance);
  }
}

TEST_F(PoseTest, pRF2pWFTransformation) {
  for (int i = 0; i < 100; ++i) {
    double x = dist(rng);
    double y = dist(rng);
    double theta = dist(rng);
    double th = dist(rng);

    Pose pose(x, y, theta);
    Pose transformed = pRF2pWF(pose, th);

    EXPECT_EQ(transformed.x, x * cos(th) - y * sin(th));
    EXPECT_EQ(transformed.y, x * sin(th) + y * cos(th));
    EXPECT_EQ(transformed.theta, Heading(theta));
  }
}

TEST_F(PoseTest, pWF2pRFTransformation) {
  for (int i = 0; i < 100; ++i) {
    double x = dist(rng);
    double y = dist(rng);
    double theta = dist(rng);
    double th = dist(rng);

    Pose pose(x, y, theta);
    Pose transformed = pWF2pRF(pose, th);

    EXPECT_EQ(transformed.x, x * cos(th) + y * sin(th));
    EXPECT_EQ(transformed.y, -x * sin(th) + y * cos(th));
    EXPECT_EQ(transformed.theta, Heading(theta));
  }
}

TEST_F(PoseTest, poseTFBack2Back) {
  for (int i = 0; i < 100; ++i) {
    double x = dist(rng);
    double y = dist(rng);
    double theta = dist(rng);

    auto pose = Pose{x, y, theta};
    double th = dist(rng);
    auto tf = pWF2pRF(pRF2pWF(pose, th), th);

    EXPECT_NEAR(tf.x, pose.x, tolerance);
    EXPECT_NEAR(tf.y, pose.y, tolerance);
    EXPECT_NEAR(tf.theta, pose.theta, tolerance);
  }
}

TEST_F(PoseTest, EqualityOperator) {
  Pose p1{1.0, 2.0, Heading(pi / 4)};
  Pose p2{1.0, 2.0, Heading(pi / 4)};
  EXPECT_TRUE(p1 == p2);
}

class vPoseTest : public ::testing::Test {
 protected:
  vPose vpose1{1.0, 2.0, 0.5};
  vPose vpose2{3.0, 4.0, 1.0};

  std::mt19937 rng{std::random_device{}()};
  std::uniform_real_distribution<double> dist{-10000, 10000};
};

TEST_F(vPoseTest, DefaultConstructor) {
  vPose vp;
  EXPECT_EQ(vp.vx, 0);
  EXPECT_EQ(vp.vy, 0);
  EXPECT_EQ(vp.omega, 0);
}

TEST_F(vPoseTest, ParameterizedConstructor) {
  EXPECT_EQ(vpose1.vx, 1.0);
  EXPECT_EQ(vpose1.vy, 2.0);
  EXPECT_EQ(vpose1.omega, 0.5);
}

TEST_F(vPoseTest, EqualityOperator) {
  vPose vp1{1.0, 2.0, 0.5};
  vPose vp2{1.0, 2.0, 0.5};
  EXPECT_TRUE(vp1 == vp2);
}

TEST_F(vPoseTest, AdditionOperator) {
  vPose result = vpose1 + vpose2;
  EXPECT_EQ(result.vx, vpose1.vx + vpose2.vx);
  EXPECT_EQ(result.vy, vpose1.vy + vpose2.vy);
  EXPECT_EQ(result.omega, vpose1.omega + vpose2.omega);
}

TEST_F(vPoseTest, SubtractionOperator) {
  vPose result = vpose1 - vpose2;
  EXPECT_EQ(result.vx, vpose1.vx - vpose2.vx);
  EXPECT_EQ(result.vy, vpose1.vy - vpose2.vy);
  EXPECT_EQ(result.omega, vpose1.omega - vpose2.omega);
}

TEST_F(vPoseTest, RandomizedAddition) {
  for (int i = 0; i < 10000; ++i) {
    double random_x = dist(rng);
    double random_y = dist(rng);
    double random_omega = dist(rng);

    vPose random_v(random_x, random_y, random_omega);
    vPose result = vpose1 + random_v;

    EXPECT_EQ(result.vx, vpose1.vx + random_x);
    EXPECT_EQ(result.vy, vpose1.vy + random_y);
    EXPECT_NEAR(static_cast<double>(result.omega),
                static_cast<double>(vpose1.omega) + random_omega, tolerance);
  }
}

TEST_F(vPoseTest, ScalarMultiplicationOperator) {
  for (int i = 0; i < 100; ++i) {
    double vx = dist(rng);
    double vy = dist(rng);
    double omega = dist(rng);
    double factor = dist(rng);

    vPose v{vx, vy, omega};
    vPose scaled = v * factor;

    EXPECT_EQ(scaled.vx, vx * factor);
    EXPECT_EQ(scaled.vy, vy * factor);
    EXPECT_EQ(scaled.omega, omega * factor);
  }
}

TEST_F(vPoseTest, VPoseFromPoseConversion) {
  Pose pose{4.0, 5.0, Heading(6.0)};
  vPose vpose_from_pose(pose);

  EXPECT_EQ(vpose_from_pose.vx, 4.0);
  EXPECT_EQ(vpose_from_pose.vy, 5.0);
  EXPECT_EQ(static_cast<double>(vpose_from_pose.omega), Heading(6.0));
}

TEST_F(vPoseTest, vRF2vWFTransformation) {
  for (int i = 0; i < 100; ++i) {
    double vx = dist(rng);
    double vy = dist(rng);
    double omega = dist(rng);
    double theta = dist(rng);

    vPose vRF(vx, vy, omega);
    vPose vWF = vRF2vWF(vRF, theta);

    EXPECT_EQ(vWF.vx, vx * cos(theta) - vy * sin(theta));
    EXPECT_EQ(vWF.vy, vx * sin(theta) + vy * cos(theta));
    EXPECT_EQ(vWF.omega, omega);
  }
}

TEST_F(vPoseTest, vWF2vRFTransformation) {
  for (int i = 0; i < 100; ++i) {
    double vx = dist(rng);
    double vy = dist(rng);
    double omega = dist(rng);
    double theta = dist(rng);

    vPose vWF(vx, vy, omega);
    vPose vRF = vWF2vRF(vWF, theta);

    EXPECT_EQ(vRF.vx, vx * cos(theta) + vy * sin(theta));
    EXPECT_EQ(vRF.vy, -vx * sin(theta) + vy * cos(theta));
    EXPECT_EQ(vRF.omega, omega);
  }
}

TEST_F(vPoseTest, vPoseTFBack2Back) {
  for (int i = 0; i < 100; ++i) {
    double x = dist(rng);
    double y = dist(rng);
    double theta = dist(rng);

    auto pose = vPose{x, y, theta};
    double th = dist(rng);
    auto tf = vWF2vRF(vRF2vWF(pose, th), th);

    EXPECT_NEAR(tf.vx, pose.vx, tolerance);
    EXPECT_NEAR(tf.vy, pose.vy, tolerance);
    EXPECT_NEAR(tf.omega, pose.omega, tolerance);
  }
}

TEST(PoseMathTests, DivisionAndConversionTest) {
  double k_v = 2.0;
  double dt = 0.1;

  Pose dpose(10.0, 5.0, Heading(pi / 4));
  vPose vpose_multiplied = vPose(dpose / dt) * k_v;

  // Check the results
  EXPECT_DOUBLE_EQ(vpose_multiplied.vx, (dpose.x / dt) * k_v);
  EXPECT_DOUBLE_EQ(vpose_multiplied.vy, (dpose.y / dt) * k_v);
  EXPECT_DOUBLE_EQ(vpose_multiplied.omega, Heading(dpose.theta / dt) * k_v);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
