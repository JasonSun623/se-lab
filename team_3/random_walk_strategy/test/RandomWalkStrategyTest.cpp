#include "../src/RandomWalkStrategy.h"

#include <gtest/gtest.h>
#include <geometry_msgs/Pose2D.h>

typedef std::shared_ptr<RandomWalkStrategy> RandomWalkStrategyPtr;

TEST(RandomWalkStrategyTestSuite, noCircleTestCase) {

  geometry_msgs::Pose2D pose;
  pose.x = pose.y = -1.0;
  pose.theta = 0;

  const geometry_msgs::Pose2D::ConstPtr posePtr(
      new geometry_msgs::Pose2D(pose));
  RandomWalkStrategyPtr ptr(new RandomWalkStrategy());

  ptr->receiveCirclePosition(posePtr);

  ASSERT_FALSE(ptr->getCircleVisible());
}

TEST(RandomWalkStrategyTestSuite, controllerOutputTestCase) {

  geometry_msgs::Pose2D pose;
  pose.x = pose.y = 1.0;
  pose.theta = 120 * (M_PI / 180);

  const geometry_msgs::Pose2D::ConstPtr posePtr(
      new geometry_msgs::Pose2D(pose));
  RandomWalkStrategyPtr ptr(new RandomWalkStrategy());

  ptr->receiveCirclePosition(posePtr);

  geometry_msgs::Twist out = ptr->getControlOutput();

  ASSERT_GT(out.linear.x, 0);
  ASSERT_LT(out.angular.z, 0);

  pose.theta = 70 * (M_PI / 180);
  const geometry_msgs::Pose2D::ConstPtr posePtr2(
      new geometry_msgs::Pose2D(pose));

  ptr->receiveCirclePosition(posePtr2);

  out = ptr->getControlOutput();

  ASSERT_GT(out.linear.x, 0);
  ASSERT_GT(out.angular.z, 0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
