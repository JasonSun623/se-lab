/** @file WallFollowingStrategyTest.cpp
  * Unit testing of functionality of wall following package
  * @author Mariia Gladkova
  * @author Leonhard Kuboschek
  */

/* Includes */
#include <geometry_msgs/Pose2D.h>

/* gtest include */
#include <gtest/gtest.h>

#include "../../include/WallFollowingStrategy.h"

typedef std::shared_ptr<WallFollowingStrategy> WallFollowingStrategyPtr;

/** @brief Tests if the node is successfully detecting a circle and not giving
 * false positives.
  */
TEST(WallFollowingStrategyTest, noCircleTestCase) {

  geometry_msgs::Pose2D pose;
  pose.x = pose.y = -1.0;
  pose.theta = 0;

  const geometry_msgs::Pose2D::ConstPtr posePtr(
      new geometry_msgs::Pose2D(pose));
  WallFollowingStrategyPtr ptr(new WallFollowingStrategy(0.6,0.2,-0.2,0.035,0.01));

  ptr->receiveCirclePosition(posePtr);

  ASSERT_FALSE(ptr->getCircleVisible());
}

/** @brief Tests if the node is successfully detecting a circle and not giving
 * false positives
 */
TEST(WallFollowingStrategyTest, CircleTestCase) {

  geometry_msgs::Pose2D pose;
  pose.x = 1.0;
  pose.y = 2.0;
  pose.theta = 50;

  const geometry_msgs::Pose2D::ConstPtr posePtr(
      new geometry_msgs::Pose2D(pose));
  WallFollowingStrategyPtr ptr(new WallFollowingStrategy(0.6,0.2,-0.2,0.035,0.01));

  ptr->receiveCirclePosition(posePtr);

  ASSERT_TRUE(ptr->getCircleVisible());
}


/** @brief Tests if the node is successfully detecting a crash situation
 */
TEST(WallFollowingStrategyTest, CrashTestCase) {

  geometry_msgs::Twist msg;
  msg.linear.x = 1.0;
  msg.angular.z = -3.0;

  const geometry_msgs::Twist::ConstPtr posePtr(
   new geometry_msgs::Twist(msg));
  WallFollowingStrategyPtr ptr(new WallFollowingStrategy(0.6,0.2,-0.2,0.035,0.01));

  ptr->getCrashRecovery(posePtr);

  ASSERT_TRUE(ptr->getCrashMode());
  ASSERT_EQ(msg.linear.x, 1);
  ASSERT_EQ(msg.angular.z, -3);
}


/** @brief Tests if the node is successfully detecting a crash situation
 */
TEST(WallFollowingStrategyTest, NoCrashTestCase) {

  geometry_msgs::Twist msg;
  msg.linear.x = 0.0;
  msg.angular.z = 0.0;

  const geometry_msgs::Twist::ConstPtr posePtr(
   new geometry_msgs::Twist(msg));
  WallFollowingStrategyPtr ptr(new WallFollowingStrategy(0.6,0.2,-0.2,0.035,0.01));

  ptr->getCrashRecovery(posePtr);

  ASSERT_FALSE(ptr->getCrashMode());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
