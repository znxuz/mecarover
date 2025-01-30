#include <gtest/gtest.h>

#include <cstdio>
#include <random>

#include "../application/pose_types.hpp"

using namespace imsl;
using std::numbers::pi;

static constexpr double tolerance = 1e-10;

/* Transformation of small movements from robot frame into world frame */
template <typename FltType>
constexpr Eigen::Matrix<FltType, 4, 1>
pRF2pWF(const Eigen::Matrix<FltType, 4, 1> &dpose, FltType th) {
  return Eigen::Matrix<FltType, 4, 4>{{std::cos(th), -std::sin(th), 0.0, 0.0},
                                      {std::sin(th), std::cos(th), 0.0, 0.0},
                                      {0.0, 0.0, 1.0, 0.0},
                                      {0.0, 0.0, 0.0, 1.0}} *
         dpose;
}

template <typename FltType>
constexpr Eigen::Matrix<FltType, 4, 1>
pWF2pRF(const Eigen::Matrix<FltType, 4, 1> &dpose, FltType th) {
  return Eigen::Matrix<FltType, 4, 4>{{std::cos(th), std::sin(th), 0.0, 0.0},
                                      {-std::sin(th), std::cos(th), 0.0, 0.0},
                                      {0.0, 0.0, 1.0, 0.0},
                                      {0.0, 0.0, 0.0, 1.0}} *
         dpose;
}


class PoseTest : public ::testing::Test {
 protected:
  std::mt19937 rng{std::random_device{}()};
  std::uniform_real_distribution<double> dist{-10000, 10000};

  Pose<double> pose1{1.0, 2.0, Heading<double>(pi / 4)};
  Pose<double> pose2{3.0, 4.0, Heading<double>(pi / 2)};
};

TEST_F(PoseTest, DefaultConstructor) {
  Pose<double> p;
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
  vPose<double> vpose{1.5, 2.5, 3.5};
  Pose<double> pose_from_vpose(vpose);

  EXPECT_EQ(pose_from_vpose.x, 1.5);
  EXPECT_EQ(pose_from_vpose.y, 2.5);
  EXPECT_EQ(static_cast<double>(pose_from_vpose.theta), Heading<double>(3.5));
}

TEST_F(PoseTest, RandomizedAddition) {
  for (int i = 0; i < 10000; ++i) {
    double random_x = dist(rng);
    double random_y = dist(rng);
    double random_theta = dist(rng);

    Pose<double> randomPose(random_x, random_y, Heading<double>(random_theta));
    Pose<double> result = pose1 + randomPose;

    EXPECT_EQ(result.x, pose1.x + random_x);
    EXPECT_EQ(result.y, pose1.y + random_y);
    EXPECT_NEAR(
        static_cast<double>(result.theta),
        Heading<double>(static_cast<double>(pose1.theta) + random_theta),
        tolerance);
  }
}

TEST_F(PoseTest, ScalarMultiplicationOperator) {
  double factor = 2.5;

  Pose<double> originalPose{2.0, 3.0, Heading<double>(pi / 3)};
  Pose<double> expectedPose{originalPose.x * factor, originalPose.y * factor,
                            originalPose.theta * factor};

  Pose<double> scaledPose = originalPose * factor;

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

    Pose<double> p(x, y, theta);
    Pose<double> result = p / divisor;

    EXPECT_NEAR(result.x, x / divisor, tolerance);
    EXPECT_NEAR(result.y, y / divisor, tolerance);
    EXPECT_NEAR(result.theta, Heading<double>(theta) / divisor, tolerance);
  }
}

TEST_F(PoseTest, pRF2pWFTransformation) {
  for (int i = 0; i < 100; ++i) {
    double x = dist(rng);
    double y = dist(rng);
    double theta = dist(rng);
    double th = dist(rng);

    Pose<double> pose(x, y, theta);
    Pose<double> transformed = pRF2pWF(pose, th);

    EXPECT_EQ(transformed.x, x * cos(th) - y * sin(th));
    EXPECT_EQ(transformed.y, x * sin(th) + y * cos(th));
    EXPECT_EQ(transformed.theta, Heading<double>(theta));
  }
}

TEST_F(PoseTest, pWF2pRFTransformation) {
  for (int i = 0; i < 100; ++i) {
    double x = dist(rng);
    double y = dist(rng);
    double theta = dist(rng);
    double th = dist(rng);

    Pose<double> pose(x, y, theta);
    Pose<double> transformed = pWF2pRF(pose, th);

    EXPECT_EQ(transformed.x, x * cos(th) + y * sin(th));
    EXPECT_EQ(transformed.y, -x * sin(th) + y * cos(th));
    EXPECT_EQ(transformed.theta, Heading<double>(theta));
  }
}

TEST_F(PoseTest, poseTFBack2Back) {
  for (int i = 0; i < 100; ++i) {
    double x = dist(rng);
    double y = dist(rng);
    double theta = dist(rng);

    auto pose = Pose<double>{x, y, theta};
    double th = dist(rng);
    auto tf = pWF2pRF(pRF2pWF(pose, th), th);

    EXPECT_NEAR(tf.x, pose.x, tolerance);
    EXPECT_NEAR(tf.y, pose.y, tolerance);
    EXPECT_NEAR(tf.theta, pose.theta, tolerance);
  }
}

TEST_F(PoseTest, pRF2pWFTransformationComparison) {
  for (int i = 0; i < 100; ++i) {
    double x = dist(rng);
    double y = dist(rng);
    double theta = dist(rng);
    double th = dist(rng);

    // Use the friend function
    imsl::Pose<double> pose(x, y, theta);
    imsl::Pose<double> transformedFriend = pRF2pWF(pose, th);

    // Use the matrix function
    Eigen::Matrix<double, 4, 1> dpose;
    dpose << x, y, Heading<double>(theta), 1.0;
    Eigen::Matrix<double, 4, 1> transformedMatrix = pRF2pWF(dpose, th);

    EXPECT_NEAR(transformedFriend.x, transformedMatrix(0), tolerance);
    EXPECT_NEAR(transformedFriend.y, transformedMatrix(1), tolerance);
    EXPECT_NEAR(static_cast<double>(transformedFriend.theta),
                transformedMatrix(2), tolerance);
  }
}

TEST_F(PoseTest, pWF2pRFTransformationComparison) {
  for (int i = 0; i < 100; ++i) {
    double x = dist(rng);
    double y = dist(rng);
    double theta = dist(rng);
    double th = dist(rng);

    // Use the friend function
    imsl::Pose<double> pose(x, y, theta);
    imsl::Pose<double> transformedFriend = pWF2pRF(pose, th);

    // Use the matrix function
    Eigen::Matrix<double, 4, 1> dpose;
    dpose << x, y, Heading<double>(theta), 1.0;
    Eigen::Matrix<double, 4, 1> transformedMatrix = pWF2pRF(dpose, th);

    EXPECT_NEAR(transformedFriend.x, transformedMatrix(0), tolerance);
    EXPECT_NEAR(transformedFriend.y, transformedMatrix(1), tolerance);
    EXPECT_NEAR(static_cast<double>(transformedFriend.theta),
                transformedMatrix(2), tolerance);
  }
}

TEST_F(PoseTest, EqualityOperator) {
  Pose<double> p1{1.0, 2.0, Heading<double>(pi / 4)};
  Pose<double> p2{1.0, 2.0, Heading<double>(pi / 4)};
  EXPECT_TRUE(p1 == p2);
}

class vPoseTest : public ::testing::Test {
 protected:
  vPose<double> vpose1{1.0, 2.0, 0.5};
  vPose<double> vpose2{3.0, 4.0, 1.0};

  std::mt19937 rng{std::random_device{}()};
  std::uniform_real_distribution<double> dist{-10000, 10000};
};

TEST_F(vPoseTest, DefaultConstructor) {
  vPose<double> vp;
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
  vPose<double> vp1{1.0, 2.0, 0.5};
  vPose<double> vp2{1.0, 2.0, 0.5};
  EXPECT_TRUE(vp1 == vp2);
}

TEST_F(vPoseTest, AdditionOperator) {
  vPose<double> result = vpose1 + vpose2;
  EXPECT_EQ(result.vx, vpose1.vx + vpose2.vx);
  EXPECT_EQ(result.vy, vpose1.vy + vpose2.vy);
  EXPECT_EQ(result.omega, vpose1.omega + vpose2.omega);
}

TEST_F(vPoseTest, SubtractionOperator) {
  vPose<double> result = vpose1 - vpose2;
  EXPECT_EQ(result.vx, vpose1.vx - vpose2.vx);
  EXPECT_EQ(result.vy, vpose1.vy - vpose2.vy);
  EXPECT_EQ(result.omega, vpose1.omega - vpose2.omega);
}

TEST_F(vPoseTest, RandomizedAddition) {
  for (int i = 0; i < 10000; ++i) {
    double random_x = dist(rng);
    double random_y = dist(rng);
    double random_omega = dist(rng);

    vPose<double> random_v(random_x, random_y, random_omega);
    vPose<double> result = vpose1 + random_v;

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

    vPose<double> v{vx, vy, omega};
    vPose<double> scaled = v * factor;

    EXPECT_EQ(scaled.vx, vx * factor);
    EXPECT_EQ(scaled.vy, vy * factor);
    EXPECT_EQ(scaled.omega, omega * factor);
  }
}

TEST_F(vPoseTest, VPoseFromPoseConversion) {
  Pose<double> pose{4.0, 5.0, Heading<double>(6.0)};
  vPose<double> vpose_from_pose(pose);

  EXPECT_EQ(vpose_from_pose.vx, 4.0);
  EXPECT_EQ(vpose_from_pose.vy, 5.0);
  EXPECT_EQ(static_cast<double>(vpose_from_pose.omega), Heading<double>(6.0));
}

TEST_F(vPoseTest, vRF2vWFTransformation) {
  for (int i = 0; i < 100; ++i) {
    double vx = dist(rng);
    double vy = dist(rng);
    double omega = dist(rng);
    double theta = dist(rng);

    vPose<double> vRF(vx, vy, omega);
    vPose<double> vWF = vRF2vWF(vRF, theta);

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

    vPose<double> vWF(vx, vy, omega);
    vPose<double> vRF = vWF2vRF(vWF, theta);

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

    auto pose = vPose<double>{x, y, theta};
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

  Pose<double> dpose(10.0, 5.0, Heading<real_t>(pi / 4));
  vPose<double> vpose_multiplied = vPose<real_t>(dpose / dt) * k_v;

  // Check the results
  EXPECT_DOUBLE_EQ(vpose_multiplied.vx, (dpose.x / dt) * k_v);
  EXPECT_DOUBLE_EQ(vpose_multiplied.vy, (dpose.y / dt) * k_v);
  EXPECT_DOUBLE_EQ(vpose_multiplied.omega,
                   Heading<double>(dpose.theta / dt) * k_v);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
