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

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
