#include <gtest/gtest.h>

#include <cstdio>
#include <random>

#include "../mecarover/mrcpptypes.hpp"

using namespace imsl;

static constexpr double tolerance = 1e-12;

class PoseTest : public ::testing::Test {
 protected:
  std::mt19937 rng{std::random_device{}()};
  std::uniform_real_distribution<double> dist{-100.0, 100.0};

  Pose<double> pose1{1.0, 2.0, Heading<double>(M_PI / 4)};
  Pose<double> pose2{3.0, 4.0, Heading<double>(M_PI / 2)};
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
  EXPECT_EQ(static_cast<double>(pose1.theta), M_PI / 4);
}

TEST_F(PoseTest, RandomizedAddition) {
  for (int i = 0; i < 100; ++i) {  // Test with 100 random cases
    double randomX = dist(rng);
    double randomY = dist(rng);
    double randomTheta = dist(rng);

    Pose<double> randomPose(randomX, randomY, Heading<double>(randomTheta));
    Pose<double> result = pose1 + randomPose;

    EXPECT_EQ(result.x, pose1.x + randomX);
    EXPECT_EQ(result.y, pose1.y + randomY);
    EXPECT_NEAR(static_cast<double>(result.theta),
                Heading<double>(static_cast<double>(pose1.theta) + randomTheta),
                tolerance);
  }
}
TEST_F(PoseTest, EqualityOperator) {
  Pose<double> p1{1.0, 2.0, Heading<double>(M_PI / 4)};
  Pose<double> p2{1.0, 2.0, Heading<double>(M_PI / 4)};
  EXPECT_TRUE(p1 == p2);
}

class vPoseTest : public ::testing::Test {
 protected:
  vPose<double> vpose1{1.0, 2.0, 0.5};
  vPose<double> vpose2{3.0, 4.0, 1.0};
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

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
