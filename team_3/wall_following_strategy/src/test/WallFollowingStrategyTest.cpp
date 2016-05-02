/** @file WallFollowingStrategyTest.cpp
  * Unit testing of functionality of wall following package
  * @author Mariia Gladkova
  * @author Leonhard Kuboschek
  */

/* Includes */
#include <geometry_msgs/Pose2D.h>

/* ROS include */
#include <rosbag/view.h>
#include <rosbag/bag.h>
#include <ros/package.h>

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

/** @brief Tests if the robot moves correctly when a circle is detected
 */
TEST(WallFollowingStrategyTest, controlMovementTestCase) {

  geometry_msgs::Pose2D pose;
  geometry_msgs::Twist out;
  pose.x = pose.y = 1.0;
  pose.theta = 120;

  const geometry_msgs::Pose2D::ConstPtr posePtr(
      new geometry_msgs::Pose2D(pose));
  std::string path = ros::package::getPath("half_circle_detection");
  path += "/src/test/laserScan_HalfCircle.bag";

  rosbag::Bag bag;
  bag.open(path, rosbag::bagmode::Read);

  std::vector<std::string> topics = {"base_scan"};

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  //ensure semicircle is being detected
  for (rosbag::MessageInstance m : view) {

    sensor_msgs::LaserScan::ConstPtr laserScan =
        m.instantiate<sensor_msgs::LaserScan>();
    if (laserScan != NULL) {

      WallFollowingStrategyPtr ptr(new WallFollowingStrategy(0.6,0.2,-0.2,0.035,0.01));
      ptr->receiveLaserScan(laserScan);
      ptr->receiveCirclePosition(posePtr);

      std::string path = ros::package::getPath("wall_following_strategy");
      path += "/src/test/Image.jpg";
      cv::Mat image = cv::imread(path);
      ptr->setImage(image);
      
      ASSERT_TRUE(ptr->getImage().size().height);
      
      ptr->setCount(5);
      ASSERT_EQ(ptr->getCount(), 5);
      out = ptr->controlMovement();
      
      float variation = 90 - (pose.theta* (180 / M_PI));

      ASSERT_EQ(out.linear.x, 0.6f);
      // TO-DO: figure out the cause of the ratio
      ASSERT_NEAR(out.angular.z, variation*0.035/4, 10);
    }
  }
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
