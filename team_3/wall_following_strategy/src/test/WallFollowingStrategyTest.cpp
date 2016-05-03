/** @file WallFollowingStrategyTest.cpp
  * Unit testing of functionality of wall following package
  * @author Mariia Gladkova (mgladkova)
  * @author Leonhard Kuboschek (kuboschek)
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

typedef std::shared_ptr< WallFollowingStrategy > WallFollowingStrategyPtr;

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

<<<<<<< HEAD
/** @brief Tests if the node is successfully detecting a circle and not giving
 * false positives
 */
TEST(WallFollowingStrategyTest, CircleTestCase) {
=======
TEST(WallFollowingStrategyTest, imageSettingTestCase) {
  WallFollowingStrategyPtr ptr(new WallFollowingStrategy());
  ASSERT_FALSE(ptr->getImage().data);
  std::string path = ros::package::getPath("wall_following_strategy");
  path += "/src/Image.jpg";
  cv::Mat image = cv::imread(path);
  ptr->setImage(image);
  ASSERT_TRUE(ptr->getImage().data);
}

TEST(WallFollowingStrategyTest, controlMovementTestCase) {
>>>>>>> feature-crash-recovery

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

/** @brief Tests if the number of lines eliminated after applying HoughLinesP from OpenCV
 * is reduced such that it is close to the real number of walls
 */
TEST(WallFollowingStrategyTest, LineDetectionAccuracy){
  cv::Mat bw,s;
  WallFollowingStrategyPtr ptr(new WallFollowingStrategy(0.6,0.2,-0.2,0.035,0.01));
  std::string path1 = ros::package::getPath("wall_following_strategy");
  path1 += "/src/test/Image.jpg";
  std::vector<cv::Vec4i> lines;
  s = cv::imread(path1);
  cv::Canny(s, bw, 50, 200, 3);
  cv::HoughLinesP(bw, lines, 1, CV_PI / 180, 20, 10, 10);
  ptr->removeLines(lines);
  ASSERT_NEAR(ptr->getLines().size(), 4, 2);
  lines.clear();
  ptr->clearData();
  
  std::string path2 = ros::package::getPath("wall_following_strategy");
  path2 += "/src/test/Image3.jpg";
  s = cv::imread(path2);
  cv::Canny(s, bw, 50, 200, 3);
  cv::HoughLinesP(bw, lines, 1, CV_PI / 180, 20, 10, 10);
  ptr->removeLines(lines);
  // the image is very noisy thus we allow bigger error
  ASSERT_NEAR(ptr->getLines().size(), 7, 5);
  lines.clear();
  ptr->clearData();

  std::string path3 = ros::package::getPath("wall_following_strategy");
  path3 += "/src/test/Image4.jpg";
  s = cv::imread(path3);
  cv::Canny(s, bw, 50, 200, 3);
  cv::HoughLinesP(bw, lines, 1, CV_PI / 180, 20, 10, 10);
  ptr->removeLines(lines);
  ASSERT_NEAR(ptr->getLines().size(), 6, 2);
  lines.clear();
  ptr->clearData();
}

/** @brief Tests if the robot moves correctly when a circle is detected
 */
TEST(WallFollowingStrategyTest, controlMovementTestCase) {

  geometry_msgs::Pose2D pose,pose1;
  geometry_msgs::Twist out;

  std::string path = ros::package::getPath("half_circle_detection");
  path += "/src/test/laserScan_HalfCircle.bag";

  rosbag::Bag bag;
  bag.open(path, rosbag::bagmode::Read);

  std::vector<std::string> topics = {"base_scan"};

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  WallFollowingStrategyPtr ptr(new WallFollowingStrategy(0.6,0.2,-0.2,0.035,0.01));

  for (rosbag::MessageInstance m : view) {

    sensor_msgs::LaserScan::ConstPtr laserScan =
        m.instantiate<sensor_msgs::LaserScan>();
    if (laserScan != NULL) {
      ptr->receiveLaserScan(laserScan);
  
      std::string path = ros::package::getPath("wall_following_strategy");
      path += "/src/test/Image.jpg";
      cv::Mat image = cv::imread(path);
      ptr->setImage(image);
      ASSERT_TRUE(ptr->getImage().size().height);
      
      pose.x = pose.y = 1.0;
      pose.theta = 120;

      pose1.x = pose1.y = 1.0;
      pose1.theta = 30;

      const geometry_msgs::Pose2D::ConstPtr posePtr(
      new geometry_msgs::Pose2D(pose));
      const geometry_msgs::Pose2D::ConstPtr posePtr1(
      new geometry_msgs::Pose2D(pose1));
      ptr->receiveCirclePosition(posePtr);
      
      ptr->setCount(5);
      ASSERT_EQ(ptr->getCount(), 5);
      out = ptr->controlMovement();
      
      float variation = 90 - (pose.theta* (180 / M_PI));

      ASSERT_EQ(out.linear.x, 0.6f);
      // TO-DO: figure out the cause of the ratio
      ASSERT_NEAR(out.angular.z, variation*0.035/4, 10);

      ptr->receiveCirclePosition(posePtr1);

      out = ptr->controlMovement();
      
      float variation1 = 90 - (pose1.theta* (180 / M_PI));

      ASSERT_EQ(out.linear.x, 0.6f);
      // TO-DO: check whether the ratio depends on the difference in the frequency of publishing
      ASSERT_NEAR(out.angular.z, variation1*0.035/4, 10);
    }
  }
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
