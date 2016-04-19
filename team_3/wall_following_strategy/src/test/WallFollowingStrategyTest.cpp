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

/** @brief Tests if the node is successfully detecting corners and not giving
 * false positives.
  */
TEST(WallFollowingStrategyTest, noCircleTestCase) {

  geometry_msgs::Pose2D pose;
  pose.x = pose.y = -1.0;
  pose.theta = 0;

  const geometry_msgs::Pose2D::ConstPtr posePtr(
      new geometry_msgs::Pose2D(pose));
  WallFollowingStrategyPtr ptr(new WallFollowingStrategy());

  ptr->receiveCirclePosition(posePtr);

  ASSERT_FALSE(ptr->getCircleVisible());
}

TEST(WallFollowingStrategyTest, imageSettingTestCase){
  WallFollowingStrategyPtr ptr(new WallFollowingStrategy());
  ASSERT_FALSE(ptr->getImage().data);
  std::string path = ros::package::getPath("wall_following_strategy");
  path += "/src/Image.jpg";
  cv::Mat image = cv::imread(path);
  ptr->setImage(image);
  ASSERT_TRUE(ptr->getImage().data);
}

TEST(WallFollowingStrategyTest, controlMovementTestCase) {

  geometry_msgs::Pose2D pose;
  pose.x = pose.y = 1.0;
  pose.theta = 120 * (M_PI / 180);
  std::string path = ros::package::getPath("wall_following_strategy");
  path += "/src/Image.jpg";
  cv::Mat image = cv::imread(path);

  const geometry_msgs::Pose2D::ConstPtr posePtr(
      new geometry_msgs::Pose2D(pose));
  WallFollowingStrategyPtr ptr(new WallFollowingStrategy());

  ptr->receiveCirclePosition(posePtr);
  // not the best practice, however an essential condition to run successfully
  // controlMovement method
  ptr->setImage(image);

  geometry_msgs::Twist out = ptr->controlMovement();

  ASSERT_GT(out.linear.x, 0);
  ASSERT_LT(out.angular.z, 0);

  pose.theta = 70 * (M_PI / 180);
  pose.x = 1.0;
  const geometry_msgs::Pose2D::ConstPtr posePtr2(
      new geometry_msgs::Pose2D(pose));

  ptr->receiveCirclePosition(posePtr2);

  out = ptr->controlMovement();

  ASSERT_GT(out.linear.x, 0);
  ASSERT_GT(out.angular.z, 0);
}

TEST(WallFollowingStrategyTest, getCrashRecoveryTestCase) {

  geometry_msgs::Twist msg;
  msg.linear.x = 1;
  msg.angular.z = 3;

  std::string path = ros::package::getPath("wall_following_strategy");
  path += "/src/Image.jpg";
  cv::Mat image = cv::imread(path);

  const geometry_msgs::Twist::ConstPtr cornerRecovPtr(
      new geometry_msgs::Twist(msg));
  WallFollowingStrategyPtr ptr(new WallFollowingStrategy());

  ptr->getCrashRecovery(cornerRecovPtr);
  ptr->setImage(image);

  geometry_msgs::Twist out = ptr->controlMovement();

  ASSERT_TRUE(ptr->getCrashMode());
  ASSERT_EQ(out.linear.x, 1);
  ASSERT_EQ(out.angular.z, 3);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
